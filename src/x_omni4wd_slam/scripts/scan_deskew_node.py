#!/usr/bin/env python3
"""Intra-scan rotation deskew for 2D LaserScan using IMU gyro.

Deskews each beam to the scan start frame (header.stamp):
  t_i = stamp + i * time_increment
  p_ref = R(-Δψ(t_ref → t_i)) * p_i

Phase-1: rotation-only (no translation). Requires high-rate IMU (e.g. ZED
imu/data_raw ~200 Hz). Optional imu_time_offset corrects ZED↔host stamp skew.
"""
from __future__ import annotations

import collections
import math
import threading

import rospy
import tf2_ros
from sensor_msgs.msg import Imu, LaserScan
from geometry_msgs.msg import Vector3Stamped


class ScanDeskewNode:
    def __init__(self):
        self.scan_in = rospy.get_param("~scan_in", "/scan")
        self.scan_out = rospy.get_param("~scan_out", "/scan_deskewed")
        self.imu_topic = rospy.get_param("~imu_topic", "/zed2i/zed_node/imu/data_raw")
        self.lidar_frame = rospy.get_param("~lidar_frame", "lidar_link")
        self.imu_frame = rospy.get_param("~imu_frame", "")  # empty → use msg.header.frame_id
        self.imu_time_offset = float(rospy.get_param("~imu_time_offset", 0.0))
        self.buffer_sec = float(rospy.get_param("~imu_buffer_sec", 1.0))
        self.enabled = bool(rospy.get_param("~enabled", True))
        self.bypass_if_fail = bool(rospy.get_param("~bypass_if_fail", True))
        self.min_imu_for_scan = int(rospy.get_param("~min_imu_for_scan", 3))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))

        self._imu_lock = threading.Lock()
        # (t_corr, wz_lidar)
        self._imu = collections.deque()

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pub = rospy.Publisher(self.scan_out, LaserScan, queue_size=2)
        self._sub_imu = rospy.Subscriber(self.imu_topic, Imu, self._on_imu, queue_size=200)
        self._sub_scan = rospy.Subscriber(self.scan_in, LaserScan, self._on_scan, queue_size=5)

        self._n_ok = 0
        self._n_bypass = 0
        self._n_fail = 0
        rospy.loginfo(
            "scan_deskew: in=%s out=%s imu=%s offset=%.4fs enabled=%s",
            self.scan_in, self.scan_out, self.imu_topic, self.imu_time_offset, self.enabled,
        )

    def _on_imu(self, msg: Imu):
        t = msg.header.stamp.to_sec() + self.imu_time_offset
        frame = self.imu_frame or msg.header.frame_id
        wx, wy, wz = (
            msg.angular_velocity.x,
            msg.angular_velocity.y,
            msg.angular_velocity.z,
        )
        wz_l = self._wz_in_lidar(wx, wy, wz, frame, msg.header.stamp)
        if wz_l is None:
            # Fallback: assume nearly coplanar upright frames (URDF ≈ parallel Z)
            wz_l = wz
        with self._imu_lock:
            self._imu.append((t, wz_l))
            t_cut = t - self.buffer_sec
            while self._imu and self._imu[0][0] < t_cut:
                self._imu.popleft()

    def _wz_in_lidar(self, wx, wy, wz, imu_frame, stamp):
        if not imu_frame or imu_frame == self.lidar_frame:
            return wz
        try:
            # Rotate angular velocity vector imu → lidar (static/extrinsic)
            v = Vector3Stamped()
            v.header.stamp = stamp
            v.header.frame_id = imu_frame
            v.vector.x, v.vector.y, v.vector.z = wx, wy, wz
            out = self._tf_buffer.transform(v, self.lidar_frame, rospy.Duration(self.tf_timeout))
            return out.vector.z
        except Exception:
            return None

    def _imu_snapshot(self):
        with self._imu_lock:
            return list(self._imu)

    @staticmethod
    def _integrate_yaw(samples, t0, t1):
        """Integrate wz from t0 to t1 (seconds). samples sorted by time."""
        if t1 < t0:
            return -ScanDeskewNode._integrate_yaw(samples, t1, t0)
        if not samples:
            return None
        # Find segment covering [t0, t1]
        if t0 < samples[0][0] - 1e-3 or t1 > samples[-1][0] + 1e-3:
            return None
        yaw = 0.0
        # Walk intervals intersecting [t0, t1]
        for i in range(len(samples) - 1):
            ta, wa = samples[i]
            tb, wb = samples[i + 1]
            if tb <= t0 or ta >= t1:
                continue
            a = max(ta, t0)
            b = min(tb, t1)
            if b <= a:
                continue
            # Linear interpolate wz at mid of overlap
            if tb > ta:
                alpha = ((a + b) * 0.5 - ta) / (tb - ta)
                w = wa + alpha * (wb - wa)
            else:
                w = wa
            yaw += w * (b - a)
        return yaw

    def _deskew(self, scan: LaserScan, samples):
        n = len(scan.ranges)
        if n < 2:
            return None
        t_ref = scan.header.stamp.to_sec()
        dt = scan.time_increment if scan.time_increment > 0.0 else (
            scan.scan_time / float(n - 1) if scan.scan_time > 0 and n > 1 else 0.0
        )
        if dt <= 0.0:
            return None

        # Count IMU samples overlapping scan window
        t_end = t_ref + dt * (n - 1)
        n_imu = sum(1 for t, _ in samples if t_ref - 0.01 <= t <= t_end + 0.01)
        if n_imu < self.min_imu_for_scan:
            return None

        # Accumulate yaw from t_ref for each beam via prefix integration
        # Precompute yaw at each IMU sample relative to t_ref
        bins = [math.inf] * n
        angle_min = scan.angle_min
        angle_inc = scan.angle_increment
        rmin = scan.range_min
        rmax = scan.range_max if scan.range_max > 0 else 1e6

        # Cache yaw lookups on a grid of beam times (batch)
        yaw_cache = {}
        for i, r in enumerate(scan.ranges):
            if not (rmin < r < rmax) or math.isinf(r) or math.isnan(r):
                continue
            ti = t_ref + i * dt
            key = int(round(ti * 1e4))  # 0.1 ms
            if key not in yaw_cache:
                dyaw = self._integrate_yaw(samples, t_ref, ti)
                if dyaw is None:
                    return None
                yaw_cache[key] = dyaw
            dyaw = yaw_cache[key]
            th = angle_min + i * angle_inc
            # Measured in lidar frame at ti; rotate back to t_ref
            c, s = math.cos(th), math.sin(th)
            x, y = r * c, r * s
            cd, sd = math.cos(-dyaw), math.sin(-dyaw)
            xr = cd * x - sd * y
            yr = sd * x + cd * y
            th_new = math.atan2(yr, xr)
            r_new = math.hypot(xr, yr)
            j = int(round((th_new - angle_min) / angle_inc))
            if 0 <= j < n and rmin < r_new < rmax:
                if r_new < bins[j]:
                    bins[j] = r_new

        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        out.ranges = [float(v) if not math.isinf(v) else float("inf") for v in bins]
        if scan.intensities:
            # Drop intensities after rebin (not meaningful)
            out.intensities = []
        return out

    def _on_scan(self, scan: LaserScan):
        if not self.enabled:
            self._pub.publish(scan)
            self._n_bypass += 1
            return
        samples = self._imu_snapshot()
        deskewed = self._deskew(scan, samples) if samples else None
        if deskewed is None:
            self._n_fail += 1
            if self.bypass_if_fail:
                self._pub.publish(scan)
                self._n_bypass += 1
            if self._n_fail % 50 == 1:
                rospy.logwarn_throttle(
                    5.0,
                    "scan_deskew: fail/bypass (ok=%d bypass=%d fail=%d imu_buf=%d)",
                    self._n_ok, self._n_bypass, self._n_fail, len(samples),
                )
            return
        self._pub.publish(deskewed)
        self._n_ok += 1
        if self._n_ok % 100 == 1:
            rospy.loginfo_throttle(
                10.0,
                "scan_deskew: ok=%d bypass=%d fail=%d",
                self._n_ok, self._n_bypass, self._n_fail,
            )


def main():
    rospy.init_node("scan_deskew")
    ScanDeskewNode()
    rospy.spin()


if __name__ == "__main__":
    main()
