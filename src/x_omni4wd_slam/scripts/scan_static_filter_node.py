#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Lọc điểm động trên LaserScan trước khi cấp SRF / RF2O.

Hai nguồn (bật độc lập):
1) Map occupancy: giữ beam nếu endpoint gần ô occupied trên /map.
2) (Tuỳ chọn) ZED /bodies: xoá beam đi qua hình trụ người.

Output: /scan_static (range bị loại = +inf). Costmap vẫn dùng /scan gốc.
"""
from __future__ import annotations

import math
import threading

import numpy as np
import rospy
import tf2_ros
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan


class ScanStaticFilterNode:
    def __init__(self):
        self.scan_in = rospy.get_param("~scan_in", "/scan")
        self.scan_out = rospy.get_param("~scan_out", "/scan_static")
        self.map_topic = rospy.get_param("~map_topic", "/map")
        self.map_frame = rospy.get_param("~map_frame", "map")
        self.use_map_filter = bool(rospy.get_param("~use_map_filter", True))
        self.use_body_filter = bool(rospy.get_param("~use_body_filter", False))
        self.bodies_topic = rospy.get_param("~bodies_topic", "/bodies")
        self.body_radius = float(rospy.get_param("~body_radius", 0.35))
        self.body_max_age = float(rospy.get_param("~body_max_age", 0.5))
        self.static_hit_tol = float(rospy.get_param("~static_hit_tolerance", 0.25))
        self.occupied_threshold = int(rospy.get_param("~occupied_threshold", 50))
        self.passthrough_if_no_map = bool(rospy.get_param("~passthrough_if_no_map", True))
        self.tf_timeout = float(rospy.get_param("~tf_timeout", 0.05))

        self._lock = threading.Lock()
        self._map = None
        self._map_data = None
        self._bodies_xy = []
        self._bodies_stamp = None

        self._tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(30.0))
        self._tf_listener = tf2_ros.TransformListener(self._tf_buffer)

        self._pub = rospy.Publisher(self.scan_out, LaserScan, queue_size=2)
        rospy.Subscriber(self.map_topic, OccupancyGrid, self._on_map, queue_size=1)
        rospy.Subscriber(self.scan_in, LaserScan, self._on_scan, queue_size=5)

        if self.use_body_filter:
            try:
                from zed_sdk_bridge.msg import BodiesStamped

                rospy.Subscriber(
                    self.bodies_topic, BodiesStamped, self._on_bodies, queue_size=5
                )
                rospy.loginfo(
                    "scan_static_filter: body filter ON topic=%s r=%.2f",
                    self.bodies_topic,
                    self.body_radius,
                )
            except ImportError:
                rospy.logwarn(
                    "scan_static_filter: zed_sdk_bridge khong co — tat body filter"
                )
                self.use_body_filter = False

        self._n_in = 0
        self._n_dyn = 0
        self._n_pass = 0
        rospy.Timer(rospy.Duration(5.0), self._log_stats)

        rospy.loginfo(
            "scan_static_filter: %s -> %s | map=%s body=%s tol=%.2fm",
            self.scan_in,
            self.scan_out,
            self.use_map_filter,
            self.use_body_filter,
            self.static_hit_tol,
        )

    def _log_stats(self, _evt):
        if self._n_in == 0:
            return
        rospy.loginfo_throttle(
            5.0,
            "scan_static_filter: scans=%d passthrough=%d dyn_beams~%d/scan",
            self._n_in,
            self._n_pass,
            int(self._n_dyn / max(self._n_in, 1)),
        )

    def _on_map(self, msg):
        h = msg.info.height
        w = msg.info.width
        data = np.asarray(msg.data, dtype=np.int8).reshape((h, w))
        with self._lock:
            self._map = msg
            self._map_data = data

    def _on_bodies(self, msg):
        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame,
                msg.header.frame_id,
                msg.header.stamp,
                rospy.Duration(self.tf_timeout),
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    msg.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout),
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "body TF fail: %s", exc)
                return

        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)
        c, s = math.cos(yaw), math.sin(yaw)

        pts = []
        for body in msg.bodies:
            if len(body.position) < 2:
                continue
            bx, by = float(body.position[0]), float(body.position[1])
            if not (math.isfinite(bx) and math.isfinite(by)):
                continue
            mx = t.x + c * bx - s * by
            my = t.y + s * bx + c * by
            pts.append((mx, my))

        with self._lock:
            self._bodies_xy = pts
            self._bodies_stamp = rospy.Time.now()

    def _world_to_map_idx(self, info, x, y):
        mx = int((x - info.origin.position.x) / info.resolution)
        my = int((y - info.origin.position.y) / info.resolution)
        return mx, my

    def _near_occupied(self, info, data, x, y):
        res = info.resolution
        tol_cells = max(1, int(math.ceil(self.static_hit_tol / res)))
        cx, cy = self._world_to_map_idx(info, x, y)
        h, w = data.shape
        for dy in range(-tol_cells, tol_cells + 1):
            for dx in range(-tol_cells, tol_cells + 1):
                ix, iy = cx + dx, cy + dy
                if ix < 0 or iy < 0 or ix >= w or iy >= h:
                    continue
                if float(dx * dx + dy * dy) * res * res > self.static_hit_tol ** 2:
                    continue
                v = int(data[iy, ix])
                if v >= self.occupied_threshold:
                    return True
        return False

    def _hit_body(self, x, y, bodies):
        r2 = self.body_radius * self.body_radius
        for bx, by in bodies:
            dx = x - bx
            dy = y - by
            if dx * dx + dy * dy <= r2:
                return True
        return False

    def _on_scan(self, scan):
        self._n_in += 1
        out = LaserScan()
        out.header = scan.header
        out.angle_min = scan.angle_min
        out.angle_max = scan.angle_max
        out.angle_increment = scan.angle_increment
        out.time_increment = scan.time_increment
        out.scan_time = scan.scan_time
        out.range_min = scan.range_min
        out.range_max = scan.range_max
        ranges = list(scan.ranges)
        intensities = list(scan.intensities) if scan.intensities else []

        with self._lock:
            grid = self._map
            data = self._map_data
            bodies = list(self._bodies_xy)
            body_stamp = self._bodies_stamp

        bodies_ok = False
        if self.use_body_filter and body_stamp is not None:
            age = (rospy.Time.now() - body_stamp).to_sec()
            bodies_ok = age <= self.body_max_age and len(bodies) > 0

        map_ok = self.use_map_filter and grid is not None and data is not None

        if not map_ok and not bodies_ok:
            if self.passthrough_if_no_map:
                self._n_pass += 1
                self._pub.publish(scan)
            return

        try:
            tf = self._tf_buffer.lookup_transform(
                self.map_frame,
                scan.header.frame_id,
                scan.header.stamp,
                rospy.Duration(self.tf_timeout),
            )
        except Exception:
            try:
                tf = self._tf_buffer.lookup_transform(
                    self.map_frame,
                    scan.header.frame_id,
                    rospy.Time(0),
                    rospy.Duration(self.tf_timeout),
                )
            except Exception as exc:
                rospy.logwarn_throttle(2.0, "scan TF fail: %s", exc)
                if self.passthrough_if_no_map:
                    self._n_pass += 1
                    self._pub.publish(scan)
                return

        t = tf.transform.translation
        q = tf.transform.rotation
        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
        yaw = math.atan2(siny, cosy)

        info = grid.info if grid is not None else None
        n_dyn = 0
        for i, r in enumerate(ranges):
            if not math.isfinite(r) or r < scan.range_min or r > scan.range_max:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            lx = r * math.cos(ang)
            ly = r * math.sin(ang)
            c, s = math.cos(yaw), math.sin(yaw)
            mx = t.x + c * lx - s * ly
            my = t.y + s * lx + c * ly

            drop = False
            if bodies_ok and self._hit_body(mx, my, bodies):
                drop = True
            elif map_ok and not self._near_occupied(info, data, mx, my):
                drop = True

            if drop:
                ranges[i] = float("inf")
                n_dyn += 1

        self._n_dyn += n_dyn
        out.ranges = ranges
        if intensities:
            out.intensities = intensities
        self._pub.publish(out)


def main():
    rospy.init_node("scan_static_filter")
    ScanStaticFilterNode()
    rospy.spin()


if __name__ == "__main__":
    main()
