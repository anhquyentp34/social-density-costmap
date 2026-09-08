#!/usr/bin/env python3
"""GTSAM Pose2 LIVO v2 — deeper sliding-window smoother.

Improvements vs v1:
  - VIO: relative BetweenFactorPose2 synced to SRF key pairs (no absolute Prior)
  - Noise scaled by /reliability/srf and /reliability/vio_soft
  - Prefer /odom_srf_directional when available
  - Skip marginalCovariance each tick (latency)
  - Softer window reset (only on max_keys; longer lag)
  - IMU yaw Between with σ_yaw from preint sample count

Publishes /odom_livo + /odom_livo/latency_ms.
Requires: pip gtsam>=4.2
"""
from __future__ import annotations

import math
from collections import deque
from typing import Deque, Dict, Optional, Tuple

import numpy as np
import rospy
import tf2_ros
from geometry_msgs.msg import Quaternion, TransformStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import Imu
from std_msgs.msg import Float32

from gtsam import (
    BetweenFactorPose2,
    ISAM2,
    ISAM2Params,
    NonlinearFactorGraph,
    Pose2,
    PreintegratedAhrsMeasurements,
    PreintegrationParams,
    PriorFactorPose2,
    Values,
    noiseModel,
)
from gtsam.symbol_shorthand import X


def yaw_from_quat(q) -> float:
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


def quat_from_yaw(yaw: float) -> Quaternion:
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def odom_to_pose2(msg: Odometry) -> Pose2:
    p = msg.pose.pose.position
    return Pose2(p.x, p.y, yaw_from_quat(msg.pose.pose.orientation))


def pose_sigmas(msg: Odometry, floor=(0.025, 0.025, 0.05), scale=1.0) -> np.ndarray:
    c = msg.pose.covariance
    sx = max(math.sqrt(max(c[0], 1e-8)) * scale, floor[0])
    sy = max(math.sqrt(max(c[7], 1e-8)) * scale, floor[1])
    syaw = max(math.sqrt(max(c[35], 1e-8)) * scale, floor[2])
    return np.array([sx, sy, syaw], dtype=float)


class GtsamLivoNodeV2:
    def __init__(self):
        self.lag_s = float(rospy.get_param("~lag_s", 5.0))
        self.max_keys = int(rospy.get_param("~max_keys", 60))
        self.imu_yaw_sigma0 = float(rospy.get_param("~imu_yaw_sigma", 0.05))
        self.imu_xy_sigma = float(rospy.get_param("~imu_xy_sigma", 80.0))
        self.vio_min_soft = float(rospy.get_param("~vio_min_soft", 0.18))
        self.vio_base_sigma = float(rospy.get_param("~vio_base_sigma", 0.06))
        self.srf_topic = rospy.get_param("~srf_topic", "/odom_srf_directional")
        self.srf_fallback = rospy.get_param("~srf_fallback", "/odom_srf")
        self.imu_topic = rospy.get_param("~imu_topic", "/zed_imu")
        self.vio_topic = rospy.get_param("~vio_topic", "/odom_zed_soft")
        self.soft_topic = rospy.get_param("~soft_topic", "/reliability/vio_soft")
        self.srf_rel_topic = rospy.get_param("~srf_rel_topic", "/reliability/srf")
        self.odom_frame = rospy.get_param("~odom_frame", "odom")
        self.base_frame = rospy.get_param("~base_frame", "base_footprint")
        self.compute_marginal = bool(rospy.get_param("~compute_marginal", False))
        self.publish_tf = bool(rospy.get_param("~publish_tf", False))

        self.soft_r = 0.5
        self.srf_r = 0.7
        # IMU: (t, wx, wy, wz)
        self.imu_buf: Deque[Tuple[float, float, float, float]] = deque(maxlen=4000)
        # VIO buffer: (t, Pose2)
        self.vio_buf: Deque[Tuple[float, Pose2]] = deque(maxlen=500)
        self._imu_params = PreintegrationParams.MakeSharedU(9.81)
        self._imu_bias = np.zeros(3)

        self.key_idx = -1
        self.key_times: Dict[int, float] = {}
        self.last_srf_pose: Optional[Pose2] = None
        self.last_srf_t: Optional[float] = None
        self.last_srf_msg: Optional[Odometry] = None
        self.initialized = False
        self.using_directional = False

        params = ISAM2Params()
        params.setRelinearizeThreshold(0.08)
        params.relinearizeSkip = 2
        self.isam = ISAM2(params)

        self.pub = rospy.Publisher("/odom_livo", Odometry, queue_size=30)
        self.pub_lat = rospy.Publisher("/odom_livo/latency_ms", Float32, queue_size=30)
        self.pub_diag = rospy.Publisher("/odom_livo/n_factors_hint", Float32, queue_size=5)
        self._tf_br = tf2_ros.TransformBroadcaster() if self.publish_tf else None

        rospy.Subscriber(self.soft_topic, Float32, self._soft_cb, queue_size=20)
        rospy.Subscriber(self.srf_rel_topic, Float32, self._srf_rel_cb, queue_size=20)
        rospy.Subscriber(self.imu_topic, Imu, self._imu_cb, queue_size=200)
        rospy.Subscriber(self.vio_topic, Odometry, self._vio_cb, queue_size=30)
        # Prefer directional; also subscribe fallback and use whichever arrives first per stamp
        rospy.Subscriber(self.srf_topic, Odometry, self._srf_cb, queue_size=20)
        rospy.Subscriber(self.srf_fallback, Odometry, self._srf_fallback_cb, queue_size=20)

        rospy.loginfo(
            "gtsam_livo_v2: srf=%s|%s imu=%s vio=%s soft>=%.2f lag=%.1fs keys=%d publish_tf=%s",
            self.srf_topic,
            self.srf_fallback,
            self.imu_topic,
            self.vio_topic,
            self.vio_min_soft,
            self.lag_s,
            self.max_keys,
            self.publish_tf,
        )

    def _soft_cb(self, msg: Float32):
        self.soft_r = float(msg.data)

    def _srf_rel_cb(self, msg: Float32):
        self.srf_r = float(msg.data)

    def _imu_cb(self, msg: Imu):
        t = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        w = msg.angular_velocity
        self.imu_buf.append((t, float(w.x), float(w.y), float(w.z)))

    def _vio_cb(self, msg: Odometry):
        t = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        self.vio_buf.append((t, odom_to_pose2(msg)))

    def _srf_fallback_cb(self, msg: Odometry):
        # Only use fallback if directional never seen / idle
        if self.using_directional:
            return
        self._srf_cb(msg, from_directional=False)

    def _integrate_imu_yaw(self, t0: float, t1: float) -> Tuple[Optional[float], int]:
        """AHRS preintegration (GTSAM) → yaw; fallback trapezoid on wz."""
        if t1 <= t0 or len(self.imu_buf) < 2:
            return None, 0
        samples = [s for s in self.imu_buf if t0 - 1e-3 <= s[0] <= t1 + 1e-3]
        if len(samples) < 2:
            return None, 0
        try:
            pim = PreintegratedAhrsMeasurements(self._imu_params, self._imu_bias)
            n = 0
            for i in range(1, len(samples)):
                dt = samples[i][0] - samples[i - 1][0]
                if dt <= 0 or dt > 0.25:
                    continue
                # mid-point omega
                wx = 0.5 * (samples[i - 1][1] + samples[i][1])
                wy = 0.5 * (samples[i - 1][2] + samples[i][2])
                wz = 0.5 * (samples[i - 1][3] + samples[i][3])
                pim.integrateMeasurement(np.array([wx, wy, wz], dtype=float), float(dt))
                n += 1
            if n < 2:
                return None, 0
            return float(pim.deltaRij().yaw()), n
        except Exception as e:
            rospy.logwarn_throttle(10.0, "AHRS preint fail, trapezoid: %s", e)
            yaw = 0.0
            n = 0
            for i in range(1, len(samples)):
                dt = samples[i][0] - samples[i - 1][0]
                if dt <= 0 or dt > 0.25:
                    continue
                yaw += 0.5 * (samples[i - 1][3] + samples[i][3]) * dt
                n += 1
            return yaw, n

    def _vio_pose_at(self, t: float, tol: float = 0.12) -> Optional[Pose2]:
        if not self.vio_buf:
            return None
        best = None
        best_dt = tol
        for ts, pose in self.vio_buf:
            dt = abs(ts - t)
            if dt < best_dt:
                best_dt = dt
                best = pose
        return best

    def _prune_if_needed(self, now_t: float):
        if self.key_idx < 0:
            return
        too_many = self.key_idx + 1 > self.max_keys
        oldest_t = min(self.key_times.values()) if self.key_times else now_t
        too_old = (now_t - oldest_t) > self.lag_s and len(self.key_times) > 8
        if not too_many and not too_old:
            return
        try:
            pose = self.isam.calculateEstimate().atPose2(X(self.key_idx))
        except Exception as e:
            rospy.logwarn_throttle(5.0, "gtsam_v2 prune skip: %s", e)
            return
        self.isam = ISAM2(ISAM2Params())
        graph = NonlinearFactorGraph()
        vals = Values()
        graph.add(
            PriorFactorPose2(
                X(0),
                pose,
                noiseModel.Diagonal.Sigmas(np.array([0.04, 0.04, 0.04])),
            )
        )
        vals.insert(X(0), pose)
        self.isam.update(graph, vals)
        self.key_times = {0: now_t}
        self.key_idx = 0
        # Keep last_srf_pose as raw SRF continuity for deltas — do NOT replace with estimate
        # (delta must stay in SRF measurement space). Only graph keys reset.
        rospy.loginfo_throttle(15.0, "gtsam_v2: window reseed key→0")

    def _srf_cb(self, msg: Odometry, from_directional: bool = True):
        if from_directional:
            self.using_directional = True
        t = msg.header.stamp.to_sec() or rospy.Time.now().to_sec()
        pose = odom_to_pose2(msg)
        # Inflate SRF noise when reliability low
        srf_scale = 1.0 / max(self.srf_r, 0.15)
        sig = pose_sigmas(msg, floor=(0.03, 0.03, 0.06), scale=srf_scale)

        graph = NonlinearFactorGraph()
        vals = Values()

        if not self.initialized:
            self.key_idx = 0
            graph.add(PriorFactorPose2(X(0), pose, noiseModel.Diagonal.Sigmas(sig)))
            vals.insert(X(0), pose)
            self.isam.update(graph, vals)
            self.key_times[0] = t
            self.last_srf_pose = pose
            self.last_srf_t = t
            self.last_srf_msg = msg
            self.initialized = True
            self._publish(pose, msg, t)
            return

        self._prune_if_needed(t)
        k_prev = self.key_idx
        k_new = k_prev + 1
        assert self.last_srf_pose is not None and self.last_srf_t is not None

        delta_srf = self.last_srf_pose.between(pose)
        graph.add(
            BetweenFactorPose2(
                X(k_prev), X(k_new), delta_srf, noiseModel.Diagonal.Sigmas(sig)
            )
        )

        # IMU yaw Between
        dyaw, n_imu = self._integrate_imu_yaw(self.last_srf_t, t)
        if dyaw is not None and n_imu >= 2:
            # More samples → slightly tighter yaw
            yaw_sig = self.imu_yaw_sigma0 * math.sqrt(8.0 / max(n_imu, 1))
            yaw_sig = max(0.02, min(0.2, yaw_sig))
            imu_sig = np.array([self.imu_xy_sigma, self.imu_xy_sigma, yaw_sig])
            graph.add(
                BetweenFactorPose2(
                    X(k_prev),
                    X(k_new),
                    Pose2(0.0, 0.0, dyaw),
                    noiseModel.Diagonal.Sigmas(imu_sig),
                )
            )

        # Relative soft-VIO Between on same key pair
        if self.soft_r >= self.vio_min_soft:
            v0 = self._vio_pose_at(self.last_srf_t)
            v1 = self._vio_pose_at(t)
            if v0 is not None and v1 is not None:
                delta_vio = v0.between(v1)
                # Reject absurd VIO jumps vs SRF
                d_srf = math.hypot(delta_srf.x(), delta_srf.y())
                d_vio = math.hypot(delta_vio.x(), delta_vio.y())
                if d_srf < 1e-4 or abs(d_vio - d_srf) < 0.35 + 0.5 * d_srf:
                    scale = 1.0 / max(self.soft_r, 0.08)
                    v_sig = np.array(
                        [
                            self.vio_base_sigma * scale,
                            self.vio_base_sigma * scale,
                            self.vio_base_sigma * 1.8 * scale,
                        ]
                    )
                    graph.add(
                        BetweenFactorPose2(
                            X(k_prev),
                            X(k_new),
                            delta_vio,
                            noiseModel.Diagonal.Sigmas(v_sig),
                        )
                    )

        pred = self.isam.calculateEstimate().atPose2(X(k_prev)).compose(delta_srf)
        vals.insert(X(k_new), pred)
        self.isam.update(graph, vals)

        self.key_idx = k_new
        self.key_times[k_new] = t
        self.last_srf_pose = pose
        self.last_srf_t = t
        self.last_srf_msg = msg

        try:
            est = self.isam.calculateEstimate().atPose2(X(k_new))
        except Exception as e:
            rospy.logwarn_throttle(2.0, "gtsam_v2 estimate fail: %s", e)
            est = pose
        self._publish(est, msg, t)

    def _publish(self, pose: Pose2, src: Odometry, stamp_sec: float):
        out = Odometry()
        out.header.stamp = (
            src.header.stamp if src.header.stamp.to_sec() > 0 else rospy.Time.now()
        )
        out.header.frame_id = self.odom_frame
        out.child_frame_id = self.base_frame
        out.pose.pose.position.x = pose.x()
        out.pose.pose.position.y = pose.y()
        out.pose.pose.orientation = quat_from_yaw(pose.theta())
        if self.compute_marginal and self.key_idx >= 0:
            try:
                marg = self.isam.marginalCovariance(X(self.key_idx))
                out.pose.covariance[0] = float(marg[0, 0])
                out.pose.covariance[7] = float(marg[1, 1])
                out.pose.covariance[35] = float(marg[2, 2])
            except Exception:
                out.pose.covariance[0] = 0.05
                out.pose.covariance[7] = 0.05
                out.pose.covariance[35] = 0.08
        else:
            # Proxies from reliability
            s = 0.04 / max(self.srf_r, 0.2)
            out.pose.covariance[0] = s
            out.pose.covariance[7] = s
            out.pose.covariance[35] = 0.06 / max(self.srf_r, 0.2)
        out.twist = src.twist
        self.pub.publish(out)
        if self._tf_br is not None:
            tf_msg = TransformStamped()
            tf_msg.header.stamp = out.header.stamp
            tf_msg.header.frame_id = self.odom_frame
            tf_msg.child_frame_id = self.base_frame
            tf_msg.transform.translation.x = pose.x()
            tf_msg.transform.translation.y = pose.y()
            tf_msg.transform.translation.z = 0.0
            tf_msg.transform.rotation = out.pose.pose.orientation
            self._tf_br.sendTransform(tf_msg)
        lat_ms = (rospy.Time.now().to_sec() - stamp_sec) * 1000.0
        self.pub_lat.publish(Float32(data=lat_ms))
        self.pub_diag.publish(Float32(data=float(self.key_idx + 1)))


def main():
    rospy.init_node("gtsam_livo")
    GtsamLivoNodeV2()
    rospy.spin()


if __name__ == "__main__":
    main()
