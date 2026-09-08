#!/usr/bin/env python3
"""P7/P8/P9 reliability + directional cov + soft-gated VIO.

P7: scalar gate → /odom_*_gated
P8: /odom_srf_directional (R theo hướng)
P9: /odom_zed_soft — VIO cov soft-weighted by tracking reliability ×
    disagreement vs latest SRF pose (exp decay). Near-reject when soft_r low.
"""
from __future__ import annotations

import math

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32, Float32MultiArray, Int8


class ReliabilityNode:
    def __init__(self):
        self.thresh = float(rospy.get_param("~gate_threshold", 0.35))
        self.inflate = float(rospy.get_param("~inflate_gain", 20.0))
        self.republish = bool(rospy.get_param("~republish_gated_odom", True))
        self.directional = bool(rospy.get_param("~enable_directional_cov", True))
        self.soft_vio = bool(rospy.get_param("~enable_soft_vio_gate", True))

        self.along_scale = float(rospy.get_param("~along_scale", 0.55))
        self.cross_scale = float(rospy.get_param("~cross_scale", 1.80))
        self.spin_yaw_scale = float(rospy.get_param("~spin_yaw_scale", 1.40))
        self.v_eps = float(rospy.get_param("~motion_eps", 0.03))
        self.w_eps = float(rospy.get_param("~spin_eps", 0.15))
        self.cond_ref = float(rospy.get_param("~cond_ref", 50.0))

        # P9 disagreement soft-gate
        self.disagree_sigma = float(rospy.get_param("~disagree_sigma_m", 0.12))
        self.soft_floor = float(rospy.get_param("~soft_r_floor", 0.05))
        self.soft_hard_reject = float(rospy.get_param("~soft_hard_reject", 0.12))
        self.soft_reject_gain = float(rospy.get_param("~soft_reject_gain", 200.0))

        self.srf_r = 1.0
        self.vio_r = 1.0
        self.cond = 1.0
        self.trace = 0.0
        self.mean_w = 1.0
        self.out_frac = 0.0
        self.last_srf_xy = None  # (x, y, yaw, t)
        self.last_soft_r = 1.0

        self.pub_srf_r = rospy.Publisher("/reliability/srf", Float32, queue_size=5)
        self.pub_vio_r = rospy.Publisher("/reliability/vio", Float32, queue_size=5)
        self.pub_vio_soft = rospy.Publisher("/reliability/vio_soft", Float32, queue_size=5)
        self.pub_diag = rospy.Publisher("/reliability/summary", Float32MultiArray, queue_size=5)
        self.pub_dir_meta = rospy.Publisher("/srf_cov_directional", Float32MultiArray, queue_size=5)

        rospy.Subscriber("/srf_reliability", Float32, self._on_srf_r, queue_size=10)
        rospy.Subscriber("/vio_reliability", Float32, self._on_vio_r, queue_size=10)
        rospy.Subscriber("/vio_tracking_state", Int8, self._on_vio_state, queue_size=10)
        rospy.Subscriber("/srf_diagnostics", Float32MultiArray, self._on_srf_diag, queue_size=10)

        need_srf = self.republish or self.directional or self.soft_vio
        if need_srf:
            rospy.Subscriber("/odom_srf", Odometry, self._on_srf_odom, queue_size=10)
        if self.republish or self.soft_vio:
            rospy.Subscriber("/odom_zed", Odometry, self._on_zed_odom, queue_size=10)

        if self.republish:
            self.pub_srf_g = rospy.Publisher("/odom_srf_gated", Odometry, queue_size=10)
            self.pub_zed_g = rospy.Publisher("/odom_zed_gated", Odometry, queue_size=10)
        if self.directional:
            self.pub_srf_d = rospy.Publisher("/odom_srf_directional", Odometry, queue_size=10)
        if self.soft_vio:
            self.pub_zed_soft = rospy.Publisher("/odom_zed_soft", Odometry, queue_size=10)

        rospy.loginfo(
            "reliability_node P7/P8/P9: gated=%s directional=%s soft_vio=%s sigma=%.2fm",
            self.republish, self.directional, self.soft_vio, self.disagree_sigma,
        )

    def _on_srf_r(self, msg: Float32):
        self.srf_r = max(0.0, min(1.0, float(msg.data)))
        self.pub_srf_r.publish(Float32(self.srf_r))
        self._summary()

    def _on_vio_r(self, msg: Float32):
        self.vio_r = max(0.0, min(1.0, float(msg.data)))
        self.pub_vio_r.publish(Float32(self.vio_r))
        self._summary()

    def _on_vio_state(self, msg: Int8):
        pass

    def _on_srf_diag(self, msg: Float32MultiArray):
        if len(msg.data) >= 3:
            self.trace = float(msg.data[1])
            self.cond = max(1.0, float(msg.data[2]))
        if len(msg.data) >= 10:
            self.mean_w = float(msg.data[8])
            self.out_frac = float(msg.data[9])

    def _summary(self):
        m = Float32MultiArray()
        m.data = [
            self.srf_r, self.vio_r, self.thresh, self.cond, self.trace,
            self.mean_w, self.out_frac, self.last_soft_r,
        ]
        self.pub_diag.publish(m)

    def _yaw(self, odom: Odometry) -> float:
        q = odom.pose.pose.orientation
        return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))

    def _copy_odom(self, odom: Odometry) -> Odometry:
        out = Odometry()
        out.header = odom.header
        out.child_frame_id = odom.child_frame_id
        out.pose = odom.pose
        out.twist = odom.twist
        out.pose.covariance = list(odom.pose.covariance)
        out.twist.covariance = list(odom.twist.covariance)
        return out

    def _gate(self, odom: Odometry, reliab: float) -> Odometry:
        out = self._copy_odom(odom)
        if reliab < self.thresh:
            gain = self.inflate * (1.0 + (self.thresh - reliab))
            for i in (0, 7, 35):
                out.pose.covariance[i] *= gain
        elif reliab < 0.7:
            gain = 1.0 + (0.7 - reliab) * 3.0
            for i in (0, 7, 35):
                out.pose.covariance[i] *= gain
        return out

    def _directional(self, odom: Odometry) -> Odometry:
        out = self._gate(odom, self.srf_r)
        vx = float(odom.twist.twist.linear.x)
        vy = float(odom.twist.twist.linear.y)
        wz = float(odom.twist.twist.angular.z)
        ax, ay, aw = abs(vx), abs(vy), abs(wz)

        rel_factor = 1.0 + max(0.0, 0.7 - self.srf_r) * 2.0
        cond_factor = 1.0 + max(0.0, self.cond - 1.0) / self.cond_ref
        # P9: IRLS outlier soft factor
        out_factor = 1.0 + 3.0 * self.out_frac

        mode = 0.0
        sx = sy = syaw = 1.0
        if aw > self.w_eps and aw > 2.0 * max(ax, ay):
            mode = 3.0
            sx = sy = self.cross_scale
            syaw = self.spin_yaw_scale
        elif ax < self.v_eps and ay < self.v_eps:
            mode = 0.0
            sx = sy = 1.15
        elif ay > ax:
            mode = 2.0
            sy = self.along_scale
            sx = self.cross_scale
            syaw = 1.1
        else:
            mode = 1.0
            sx = self.along_scale
            sy = self.cross_scale
            syaw = 1.1

        sx *= rel_factor * cond_factor * out_factor
        sy *= rel_factor * cond_factor * out_factor
        syaw *= rel_factor * out_factor

        out.pose.covariance[0] *= sx
        out.pose.covariance[7] *= sy
        out.pose.covariance[35] *= syaw
        out.twist.covariance[0] *= sx
        out.twist.covariance[7] *= sy
        out.twist.covariance[35] *= syaw

        meta = Float32MultiArray()
        meta.data = [
            out.pose.covariance[0], out.pose.covariance[7], out.pose.covariance[35],
            mode, sx, sy, syaw, self.srf_r, self.cond, vx, vy, wz,
            self.mean_w, self.out_frac,
        ]
        self.pub_dir_meta.publish(meta)
        return out

    def _soft_vio(self, zed: Odometry) -> Odometry:
        """P9: scale VIO R by tracking r × exp(-‖Δxy‖/σ)."""
        out = self._copy_odom(zed)
        soft = max(self.soft_floor, self.vio_r)
        dxy = 0.0
        if self.last_srf_xy is not None:
            sx, sy, _, _ = self.last_srf_xy
            zx = zed.pose.pose.position.x
            zy = zed.pose.pose.position.y
            dxy = math.hypot(zx - sx, zy - sy)
            soft *= math.exp(-dxy / max(1e-3, self.disagree_sigma))
        soft = max(self.soft_floor, min(1.0, soft))
        self.last_soft_r = soft
        self.pub_vio_soft.publish(Float32(soft))

        if soft < self.soft_hard_reject:
            gain = self.soft_reject_gain
        else:
            # R ∝ 1/soft^2 → tin ít khi soft thấp
            gain = 1.0 / (soft * soft)

        for i in (0, 7, 35):
            out.pose.covariance[i] *= gain
        return out

    def _on_srf_odom(self, msg: Odometry):
        self.last_srf_xy = (
            msg.pose.pose.position.x,
            msg.pose.pose.position.y,
            self._yaw(msg),
            msg.header.stamp.to_sec(),
        )
        if self.republish:
            self.pub_srf_g.publish(self._gate(msg, self.srf_r))
        if self.directional:
            self.pub_srf_d.publish(self._directional(msg))

    def _on_zed_odom(self, msg: Odometry):
        if self.republish:
            self.pub_zed_g.publish(self._gate(msg, self.vio_r))
        if self.soft_vio:
            self.pub_zed_soft.publish(self._soft_vio(msg))


def main():
    rospy.init_node("reliability_node")
    ReliabilityNode()
    rospy.spin()


if __name__ == "__main__":
    main()
