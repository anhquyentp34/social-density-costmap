#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
YOLO /objects + camera depth → PointCloud2 mark/clear cho costmap.

Không chỉ bbox/kích thước ước lượng: lấy điểm 3D từ depth trong ROI bbox
(lọc quanh median depth) → hình dạng chiếm chỗ sát vật hơn.

  /yolo_obstacle_cloud  marking
  /yolo_obstacle_clear  clearing (raytrace khi mất detect)
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
from sensor_msgs.msg import CameraInfo, Image, PointCloud2, PointField
from std_msgs.msg import Header
from zed_sdk_bridge.msg import ObjectsStamped


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


class YoloObjectsToObstacleCloud(object):
    def __init__(self):
        self.objects_topic = rospy.get_param("~objects_topic", "/objects")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera_info")
        self.cloud_topic = rospy.get_param("~cloud_topic", "/yolo_obstacle_cloud")
        self.clear_topic = rospy.get_param("~clear_topic", "/yolo_obstacle_clear")
        self.use_depth = bool(rospy.get_param("~use_depth", True))
        self.depth_stride = int(rospy.get_param("~depth_stride", 4))
        self.depth_band = float(rospy.get_param("~depth_band", 0.40))  # m quanh median
        self.min_depth = float(rospy.get_param("~min_depth", 0.3))
        self.max_depth = float(rospy.get_param("~max_depth", 8.0))
        self.max_points_per_obj = int(rospy.get_param("~max_points_per_obj", 800))
        self.min_depth_points = int(rospy.get_param("~min_depth_points", 12))
        self.grid_step = float(rospy.get_param("~grid_step", 0.05))
        self.min_half = float(rospy.get_param("~min_half_size", 0.12))
        self.max_half = float(rospy.get_param("~max_half_size", 0.75))
        self.z_clamp_min = float(rospy.get_param("~z_clamp_min", 0.02))
        self.z_clamp_max = float(rospy.get_param("~z_clamp_max", 1.8))
        self.min_confidence = float(rospy.get_param("~min_confidence", 25.0))
        self.exclude_labels = set(
            s.strip().lower()
            for s in rospy.get_param("~exclude_labels", ["person"])
            if str(s).strip()
        )
        self.hold_sec = float(rospy.get_param("~hold_sec", 0.35))
        self.stale_sec = float(rospy.get_param("~stale_sec", 0.7))
        self.republish_hz = float(rospy.get_param("~republish_hz", 10.0))
        self.frame_id = rospy.get_param("~default_frame_id", "camera_link")
        # Lưu track theo khung cố định (odom) để khi robot quay vật không xoay theo camera
        self.fixed_frame = rospy.get_param("~fixed_frame", "odom")
        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(10.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self._bridge = cv_bridge.CvBridge()
        self._lock = Lock()
        self._fx = self._fy = self._cx = self._cy = None
        self._depth = None
        self._depth_stamp = None

        self._tracks = {}
        self._last_objects_stamp = None
        self._last_header = Header(frame_id=self.frame_id)
        self._prev_mark_points = []
        self._pending_clear_points = []
        self._stats = {"depth": 0, "fallback": 0}

        self.pub_mark = rospy.Publisher(self.cloud_topic, PointCloud2, queue_size=1)
        self.pub_clear = rospy.Publisher(self.clear_topic, PointCloud2, queue_size=1)
        rospy.Subscriber(self.objects_topic, ObjectsStamped, self.cb_objects, queue_size=1)
        if self.use_depth:
            rospy.Subscriber(self.depth_topic, Image, self.cb_depth, queue_size=1)
            rospy.Subscriber(self.camera_info_topic, CameraInfo, self.cb_info, queue_size=1)
        period = 1.0 / max(1.0, self.republish_hz)
        rospy.Timer(rospy.Duration(period), self._timer_cb)

        rospy.loginfo(
            "yolo→costmap DEPTH=%s mark=%s clear=%s stride=%d band=%.2fm hold=%.2fs fixed=%s",
            self.use_depth,
            self.cloud_topic,
            self.clear_topic,
            self.depth_stride,
            self.depth_band,
            self.hold_sec,
            self.fixed_frame,
        )

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
            self._depth_stamp = msg.header.stamp

    def _pixel_to_camera(self, u, v, depth):
        # Optical → RIGHT_HANDED_Z_UP_X_FWD (giống yolo_objects_bridge)
        x_opt = (u - self._cx) * depth / self._fx
        y_opt = (v - self._cy) * depth / self._fy
        return depth, -x_opt, -y_opt

    def _bbox_depth_points(self, obj):
        """Lấy điểm 3D từ depth trong bbox 2D (lọc median±band)."""
        if (
            not self.use_depth
            or self._fx is None
            or self._depth is None
            or len(obj.bounding_box_2d_tl) < 2
            or len(obj.bounding_box_2d_br) < 2
        ):
            return []

        with self._lock:
            depth = self._depth
            if depth is None:
                return []
            h, w = depth.shape[:2]
            u0 = int(max(0, min(w - 1, math.floor(obj.bounding_box_2d_tl[0]))))
            v0 = int(max(0, min(h - 1, math.floor(obj.bounding_box_2d_tl[1]))))
            u1 = int(max(0, min(w - 1, math.ceil(obj.bounding_box_2d_br[0]))))
            v1 = int(max(0, min(h - 1, math.ceil(obj.bounding_box_2d_br[1]))))

        if u1 <= u0 or v1 <= v0:
            return []

        stride = max(1, self.depth_stride)
        # Co bbox 10% để bớt nền quanh vật
        du = u1 - u0
        dv = v1 - v0
        u0i = u0 + int(0.1 * du)
        u1i = u1 - int(0.1 * du)
        v0i = v0 + int(0.1 * dv)
        v1i = v1 - int(0.1 * dv)
        if u1i <= u0i or v1i <= v0i:
            u0i, u1i, v0i, v1i = u0, u1, v0, v1

        patch = depth[v0i : v1i + 1 : stride, u0i : u1i + 1 : stride].astype(np.float32)
        valid = patch[np.isfinite(patch) & (patch > self.min_depth) & (patch < self.max_depth)]
        if valid.size < self.min_depth_points:
            return []

        med = float(np.median(valid))
        band = self.depth_band
        points = []
        ys = range(v0i, v1i + 1, stride)
        xs = range(u0i, u1i + 1, stride)
        for v in ys:
            row = depth[v]
            for u in xs:
                d = float(row[u])
                if not np.isfinite(d) or d < self.min_depth or d > self.max_depth:
                    continue
                if abs(d - med) > band:
                    continue
                x, y, z = self._pixel_to_camera(u + 0.5, v + 0.5, d)
                if z < self.z_clamp_min or z > self.z_clamp_max:
                    continue
                points.append((x, y, z))
                if len(points) >= self.max_points_per_obj:
                    return points
        return points

    def _footprint_fallback(self, obj):
        """Fallback: hình chữ nhật từ position + dimensions (khi depth yếu)."""
        x0 = float(obj.position[0])
        y0 = float(obj.position[1])
        z0 = float(obj.position[2])
        z0 = max(self.z_clamp_min, min(self.z_clamp_max, z0))
        half_y = 0.5 * float(obj.dimensions[0]) if len(obj.dimensions) > 0 else self.min_half
        half_x = 0.5 * float(obj.dimensions[2]) if len(obj.dimensions) > 2 else self.min_half
        half_x = max(self.min_half, min(self.max_half, abs(half_x)))
        half_y = max(self.min_half, min(self.max_half, abs(half_y)))
        step = max(0.03, self.grid_step)
        points = []
        nx = max(1, int(math.ceil(2.0 * half_x / step)))
        ny = max(1, int(math.ceil(2.0 * half_y / step)))
        for ix in range(nx + 1):
            for iy in range(ny + 1):
                dx = -half_x + (2.0 * half_x * ix / nx if nx else 0.0)
                dy = -half_y + (2.0 * half_y * iy / ny if ny else 0.0)
                points.append((x0 + dx, y0 + dy, z0))
        points.append((x0, y0, z0))
        return points

    def _points_for_obj(self, obj):
        label = (obj.label or "").strip().lower()
        if label in self.exclude_labels:
            return []
        if float(obj.confidence) < self.min_confidence:
            return []
        pts = self._bbox_depth_points(obj)
        if len(pts) >= self.min_depth_points:
            self._stats["depth"] += 1
            return pts
        self._stats["fallback"] += 1
        return self._footprint_fallback(obj)

    def cb_objects(self, msg):
        now = msg.header.stamp if msg.header.stamp.to_sec() > 0 else rospy.Time.now()
        self._last_objects_stamp = now
        self._last_header = msg.header
        if not self._last_header.frame_id:
            self._last_header.frame_id = self.frame_id
        cam_frame = self._last_header.frame_id or self.frame_id

        for i, obj in enumerate(msg.objects):
            pts = self._points_for_obj(obj)
            if not pts:
                continue
            world_pts = self._transform_points(pts, cam_frame, self.fixed_frame, now)
            if world_pts is None:
                continue
            oid = int(obj.id) if obj.id is not None else -(i + 1)
            if oid == 0:
                oid = -(i + 1)
            self._tracks[oid] = {
                "points": world_pts,
                "last_seen": now,
                "label": obj.label or "",
            }
        self._publish_active(now)

    def _prune(self, now):
        dropped_pts = []
        if self._last_objects_stamp is None:
            for tr in self._tracks.values():
                dropped_pts.extend(tr["points"])
            self._tracks.clear()
            return dropped_pts
        if (now - self._last_objects_stamp).to_sec() > self.stale_sec:
            for tr in self._tracks.values():
                dropped_pts.extend(tr["points"])
            self._tracks.clear()
            return dropped_pts
        dead = []
        for oid, tr in self._tracks.items():
            if (now - tr["last_seen"]).to_sec() > self.hold_sec:
                dead.append(oid)
                dropped_pts.extend(tr["points"])
        for oid in dead:
            del self._tracks[oid]
        return dropped_pts

    def _publish_active(self, now):
        dropped = self._prune(now)
        mark_world = []
        for tr in self._tracks.values():
            mark_world.extend(tr["points"])

        clear_world = list(self._prev_mark_points)
        clear_world.extend(dropped)
        clear_world.extend(self._pending_clear_points)
        self._pending_clear_points = []

        header = copy(self._last_header)
        header.stamp = now
        cam_frame = header.frame_id or self.frame_id
        header.frame_id = cam_frame

        # Track lưu trong odom; publish lại camera_link theo TF hiện tại
        # để ObstacleLayer raytrace từ camera, vật đứng yên trên map khi robot quay.
        mark_cam = self._transform_points(mark_world, self.fixed_frame, cam_frame, now)
        clear_cam = self._transform_points(clear_world, self.fixed_frame, cam_frame, now)
        if mark_cam is None or clear_cam is None:
            return

        self.pub_clear.publish(_pack_cloud(header, clear_cam))
        self.pub_mark.publish(_pack_cloud(header, mark_cam))
        self._prev_mark_points = list(mark_world)

        rospy.loginfo_throttle(
            2.0,
            "YOLO+depth costmap: mark=%d clear=%d tracks=%d (depth_ok=%d fallback=%d) %s→%s",
            len(mark_cam),
            len(clear_cam),
            len(self._tracks),
            self._stats["depth"],
            self._stats["fallback"],
            self.fixed_frame,
            cam_frame,
        )
        self._stats = {"depth": 0, "fallback": 0}

    def _timer_cb(self, _evt):
        self._publish_active(rospy.Time.now())


if __name__ == "__main__":
    rospy.init_node("yolo_objects_to_obstacle_cloud")
    YoloObjectsToObstacleCloud()
    rospy.spin()
