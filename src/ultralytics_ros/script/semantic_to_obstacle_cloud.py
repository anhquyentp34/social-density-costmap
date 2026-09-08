#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic labels + depth → PointCloud2 mark/clear cho costmap (Track A C1).

Giữ contract topic:
  /yolo_obstacle_cloud  marking
  /yolo_obstacle_clear  clearing
"""

from __future__ import print_function

import math
import struct
from copy import copy
from threading import Lock

import cv_bridge
import numpy as np
import rospy
import tf2_ros
import yaml
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header


def _pack_cloud(header, points):
    cloud = PointCloud2()
    cloud.header = header
    cloud.height = 1
    cloud.width = len(points)
    cloud.fields = [
        PointField(name="x", offset=0, datatype=PointField.FLOAT32, count=1),
        PointField(name="y", offset=4, datatype=PointField.FLOAT32, count=1),
        PointField(name="z", offset=8, datatype=PointField.FLOAT32, count=1),
    ]
    cloud.is_bigendian = False
    cloud.point_step = 12
    cloud.row_step = cloud.point_step * cloud.width
    cloud.is_dense = True
    buf = bytearray(cloud.row_step)
    for i, (x, y, z) in enumerate(points):
        struct.pack_into("<fff", buf, i * 12, float(x), float(y), float(z))
    cloud.data = bytes(buf)
    return cloud


class SemanticToObstacleCloud(object):
    def __init__(self):
        self.labels_topic = rospy.get_param("~labels_topic", "/semantic/labels")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera_info")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/yolo_obstacle_cloud")
        self.clear_topic = rospy.get_param("~clear_topic", "/yolo_obstacle_clear")
        self.classes_yaml = rospy.get_param("~classes_yaml", "")
        self.depth_stride = int(rospy.get_param("~depth_stride", 4))
        self.depth_band = float(rospy.get_param("~depth_band", 0.50))
        self.min_depth = float(rospy.get_param("~min_depth", 0.3))
        self.max_depth = float(rospy.get_param("~max_depth", 8.0))
        self.max_points = int(rospy.get_param("~max_points", 2500))
        self.min_depth_points = int(rospy.get_param("~min_depth_points", 20))
        self.z_clamp_min = float(rospy.get_param("~z_clamp_min", 0.02))
        self.z_clamp_max = float(rospy.get_param("~z_clamp_max", 1.8))
        self.hold_sec = float(rospy.get_param("~hold_sec", 1.5))
        self.stale_sec = float(rospy.get_param("~stale_sec", 2.5))
        self.republish_hz = float(rospy.get_param("~republish_hz", 10.0))
        self.frame_id = rospy.get_param("~default_frame_id", "camera_link")
        self.fixed_frame = rospy.get_param("~fixed_frame", "odom")

        self.obstacle_ids = set()
        self._load_policy()

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)
        self._bridge = cv_bridge.CvBridge()
        self._lock = Lock()
        self._fx = self._fy = self._cx = self._cy = None
        self._depth = None
        self._labels = None
        self._last_stamp = None
        self._last_header = Header(frame_id=self.frame_id)
        self._mark_world = []
        self._prev_mark_world = []
        self._pending_clear = []

        self.pub_mark = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self.pub_clear = rospy.Publisher(self.clear_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.labels_topic, Image, self.cb_labels, queue_size=1, buff_size=2**22)
        rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1, buff_size=2**22)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)

        period = 1.0 / max(1.0, self.republish_hz)
        rospy.Timer(rospy.Duration(period), self._timer_cb)

        rospy.loginfo(
            "semantic→costmap labels=%s depth=%s mark=%s obstacles=%s fixed=%s",
            self.labels_topic,
            self.depth_topic,
            self.cloud_topic,
            sorted(self.obstacle_ids),
            self.fixed_frame,
        )

    def _load_policy(self):
        path = self.classes_yaml
        if not path:
            # Fallback: furniture VOC
            self.obstacle_ids = {2, 5, 6, 7, 9, 11, 14, 16, 18, 20}
            return
        try:
            with open(path, "r") as f:
                data = yaml.safe_load(f) or {}
            for item in data.get("classes", []):
                role = str(item.get("role", "ignore")).strip().lower()
                if role == "obstacle":
                    self.obstacle_ids.add(int(item["id"]))
        except Exception as e:
            rospy.logerr("Không đọc được %s: %s — dùng fallback furniture", path, e)
            self.obstacle_ids = {2, 5, 6, 7, 9, 11, 14, 16, 18, 20}

    def cb_info(self, msg):
        self._fx = float(msg.K[0])
        self._fy = float(msg.K[4])
        self._cx = float(msg.K[2])
        self._cy = float(msg.K[5])
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def cb_depth(self, msg):
        try:
            depth = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "depth convert failed: %s", e)
            return
        if depth.ndim == 3:
            depth = depth[:, :, 0]
        with self._lock:
            self._depth = np.asarray(depth)

    def cb_labels(self, msg):
        try:
            labels = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "labels convert failed: %s", e)
            return
        if labels.ndim == 3:
            labels = labels[:, :, 0]
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        with self._lock:
            self._labels = np.asarray(labels)
            self._last_stamp = now
            self._last_header = msg.header
            if not self._last_header.frame_id:
                self._last_header.frame_id = self.frame_id
            depth = self._depth
            fx, fy, cx, cy = self._fx, self._fy, self._cx, self._cy

        if depth is None or fx is None:
            return
        pts = self._label_depth_points(labels, depth, fx, fy, cx, cy)
        if len(pts) < self.min_depth_points:
            return
        cam_frame = self._last_header.frame_id or self.frame_id
        world = self._transform_points(pts, cam_frame, self.fixed_frame, now)
        if world is None:
            return
        self._mark_world = world
        self._publish_active(now)

    def _pixel_to_camera(self, u, v, depth, fx, fy, cx, cy):
        x_opt = (u - cx) * depth / fx
        y_opt = (v - cy) * depth / fy
        return depth, -x_opt, -y_opt

    def _label_depth_points(self, labels, depth, fx, fy, cx, cy):
        h = min(labels.shape[0], depth.shape[0])
        w = min(labels.shape[1], depth.shape[1])
        stride = max(1, self.depth_stride)
        mask = np.isin(labels[:h:stride, :w:stride], list(self.obstacle_ids))
        if not np.any(mask):
            return []

        ys, xs = np.where(mask)
        depth_s = depth[:h:stride, :w:stride].astype(np.float32)
        dvals = depth_s[ys, xs]
        valid = np.isfinite(dvals) & (dvals > self.min_depth) & (dvals < self.max_depth)
        if np.count_nonzero(valid) < self.min_depth_points:
            return []

        # Không lọc median toàn khung (nhiều vật xa/gần); chỉ min/max depth + z clamp
        points = []
        for yi, xi, d in zip(ys[valid], xs[valid], dvals[valid]):
            u = xi * stride + 0.5
            v = yi * stride + 0.5
            x, y, z = self._pixel_to_camera(u, v, float(d), fx, fy, cx, cy)
            if z < self.z_clamp_min or z > self.z_clamp_max:
                continue
            points.append((x, y, z))
            if len(points) >= self.max_points:
                break
        return points

    @staticmethod
    def _apply_tf(trans, points):
        t = trans.transform.translation
        q = trans.transform.rotation
        qx, qy, qz, qw = q.x, q.y, q.z, q.w
        out = []
        for vx, vy, vz in points:
            tx = 2.0 * (qy * vz - qz * vy)
            ty = 2.0 * (qz * vx - qx * vz)
            tz = 2.0 * (qx * vy - qy * vx)
            out.append(
                (
                    vx + qw * tx + (qy * tz - qz * ty) + t.x,
                    vy + qw * ty + (qz * tx - qx * tz) + t.y,
                    vz + qw * tz + (qx * ty - qy * tx) + t.z,
                )
            )
        return out

    def _transform_points(self, points, from_frame, to_frame, stamp):
        if not points:
            return []
        if not from_frame or not to_frame or from_frame == to_frame:
            return list(points)
        lookup_stamp = stamp if stamp is not None and stamp.to_sec() > 0 else rospy.Time(0)
        try:
            trans = self.tf_buffer.lookup_transform(
                to_frame, from_frame, lookup_stamp, rospy.Duration(0.05)
            )
        except Exception:
            try:
                trans = self.tf_buffer.lookup_transform(
                    to_frame, from_frame, rospy.Time(0), rospy.Duration(0.05)
                )
            except Exception as e:
                rospy.logwarn_throttle(2.0, "TF %s → %s failed: %s", from_frame, to_frame, e)
                return None
        return self._apply_tf(trans, points)

    def _publish_active(self, now):
        if self._last_stamp is None:
            return
        age = (now - self._last_stamp).to_sec()
        if age > self.stale_sec:
            dropped = list(self._mark_world) + list(self._prev_mark_world)
            self._mark_world = []
            self._prev_mark_world = []
            mark_world = []
            clear_world = dropped + self._pending_clear
            self._pending_clear = []
        elif age > self.hold_sec and not self._mark_world:
            mark_world = list(self._prev_mark_world)
            clear_world = list(self._pending_clear)
            self._pending_clear = []
        else:
            mark_world = list(self._mark_world) if self._mark_world else list(self._prev_mark_world)
            clear_world = list(self._prev_mark_world) + list(self._pending_clear)
            self._pending_clear = []

        header = copy(self._last_header)
        header.stamp = now
        cam_frame = header.frame_id or self.frame_id
        header.frame_id = cam_frame

        mark_cam = self._transform_points(mark_world, self.fixed_frame, cam_frame, now)
        clear_cam = self._transform_points(clear_world, self.fixed_frame, cam_frame, now)
        if mark_cam is None or clear_cam is None:
            return

        self.pub_clear.publish(_pack_cloud(header, clear_cam))
        self.pub_mark.publish(_pack_cloud(header, mark_cam))
        self._prev_mark_world = list(mark_world)

        rospy.loginfo_throttle(
            2.0,
            "semantic+depth costmap: mark=%d clear=%d age=%.2fs %s→%s",
            len(mark_cam),
            len(clear_cam),
            age,
            self.fixed_frame,
            cam_frame,
        )

    def _timer_cb(self, _evt):
        self._publish_active(rospy.Time.now())


if __name__ == "__main__":
    rospy.init_node("semantic_to_obstacle_cloud")
    SemanticToObstacleCloud()
    rospy.spin()
