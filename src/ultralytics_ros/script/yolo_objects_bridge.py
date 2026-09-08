#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Bridge Ultralytics YOLO (/yolo_result) + ZED depth → zed_sdk_bridge/ObjectsStamped (/objects).

Thay ZED SDK Object Detection: fusion / visualizer vẫn dùng /objects như cũ.
Hệ tọa độ: camera_link RIGHT_HANDED_Z_UP_X_FWD (x forward, y left, z up).
"""

import math

import cv_bridge
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import CameraInfo, Image
from ultralytics_ros.msg import YoloResult
from zed_sdk_bridge.msg import Object, ObjectsStamped

# COCO class names (80)
COCO_NAMES = [
    "person", "bicycle", "car", "motorcycle", "airplane", "bus", "train", "truck",
    "boat", "traffic light", "fire hydrant", "stop sign", "parking meter", "bench",
    "bird", "cat", "dog", "horse", "sheep", "cow", "elephant", "bear", "zebra",
    "giraffe", "backpack", "umbrella", "handbag", "tie", "suitcase", "frisbee",
    "skis", "snowboard", "sports ball", "kite", "baseball bat", "baseball glove",
    "skateboard", "surfboard", "tennis racket", "bottle", "wine glass", "cup",
    "fork", "knife", "spoon", "bowl", "banana", "apple", "sandwich", "orange",
    "broccoli", "carrot", "hot dog", "pizza", "donut", "cake", "chair", "couch",
    "potted plant", "bed", "dining table", "toilet", "tv", "laptop", "mouse",
    "remote", "keyboard", "cell phone", "microwave", "oven", "toaster", "sink",
    "refrigerator", "book", "clock", "vase", "scissors", "teddy bear", "hair drier",
    "toothbrush",
]


class YoloObjectsBridge:
    def __init__(self):
        self.bridge = cv_bridge.CvBridge()
        self.yolo_topic = rospy.get_param("~yolo_topic", "/yolo_result")
        self.depth_topic = rospy.get_param("~depth_topic", "/depth")
        self.camera_info_topic = rospy.get_param("~camera_info_topic", "/camera_info")
        self.objects_topic = rospy.get_param("~objects_topic", "/objects")
        self.filter_person = bool(rospy.get_param("~filter_person", True))
        self.min_confidence = float(rospy.get_param("~min_confidence", 0.25))  # 0-1
        self.min_depth = float(rospy.get_param("~min_depth", 0.3))
        self.max_depth = float(rospy.get_param("~max_depth", 15.0))
        self.depth_patch = int(rospy.get_param("~depth_patch", 5))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.15))

        self.fx = None
        self.fy = None
        self.cx = None
        self.cy = None
        self.frame_id = "camera_link"

        self.objects_pub = rospy.Publisher(self.objects_topic, ObjectsStamped, queue_size=1)
        rospy.Subscriber(self.camera_info_topic, CameraInfo, self.camera_info_cb, queue_size=1)

        yolo_sub = message_filters.Subscriber(self.yolo_topic, YoloResult)
        depth_sub = message_filters.Subscriber(self.depth_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [yolo_sub, depth_sub], queue_size=5, slop=self.sync_slop
        )
        self.sync.registerCallback(self.synced_cb)

        rospy.loginfo(
            "yolo_objects_bridge: yolo=%s depth=%s → %s (filter_person=%s)",
            self.yolo_topic,
            self.depth_topic,
            self.objects_topic,
            self.filter_person,
        )

    def camera_info_cb(self, msg):
        self.fx = float(msg.K[0])
        self.fy = float(msg.K[4])
        self.cx = float(msg.K[2])
        self.cy = float(msg.K[5])
        if msg.header.frame_id:
            self.frame_id = msg.header.frame_id

    def sample_depth(self, depth, u, v):
        h, w = depth.shape[:2]
        half = max(1, self.depth_patch // 2)
        u0 = max(0, int(u) - half)
        u1 = min(w, int(u) + half + 1)
        v0 = max(0, int(v) - half)
        v1 = min(h, int(v) + half + 1)
        patch = depth[v0:v1, u0:u1].astype(np.float32).reshape(-1)
        valid = patch[np.isfinite(patch) & (patch > self.min_depth) & (patch < self.max_depth)]
        if valid.size == 0:
            return None
        return float(np.median(valid))

    def pixel_to_camera(self, u, v, depth):
        """Optical (x right, y down, z fwd) → RIGHT_HANDED_Z_UP_X_FWD."""
        x_opt = (u - self.cx) * depth / self.fx
        y_opt = (v - self.cy) * depth / self.fy
        z_opt = depth
        # x forward, y left, z up
        return z_opt, -x_opt, -y_opt

    def synced_cb(self, yolo_msg, depth_msg):
        if self.fx is None:
            rospy.logwarn_throttle(5.0, "Waiting for /camera_info...")
            return

        try:
            depth = self.bridge.imgmsg_to_cv2(depth_msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "depth convert failed: %s", e)
            return

        if depth.ndim == 3:
            depth = depth[:, :, 0]

        out = ObjectsStamped()
        out.header = yolo_msg.header
        out.header.frame_id = self.frame_id

        dets = yolo_msg.detections.detections
        track_ids = list(yolo_msg.track_ids) if yolo_msg.track_ids else []

        for i, det in enumerate(dets):
            if not det.results:
                continue
            hyp = det.results[0]
            cls_id = int(hyp.id)
            conf = float(hyp.score)
            if conf < self.min_confidence:
                continue
            if self.filter_person and cls_id == 0:
                continue

            u = float(det.bbox.center.x)
            v = float(det.bbox.center.y)
            sx = float(det.bbox.size_x)
            sy = float(det.bbox.size_y)
            z = self.sample_depth(depth, u, v)
            if z is None:
                continue

            x, y, zz = self.pixel_to_camera(u, v, z)

            obj = Object()
            if i < len(track_ids) and track_ids[i] >= 0:
                obj.id = int(track_ids[i])
            else:
                obj.id = i + 1
            if 0 <= cls_id < len(COCO_NAMES):
                obj.label = COCO_NAMES[cls_id]
            else:
                obj.label = "class_%d" % cls_id
            obj.label_id = cls_id if cls_id < 128 else 127
            obj.confidence = conf * 100.0  # fusion dùng 0-100
            obj.position = [x, y, zz]
            obj.velocity = [0.0, 0.0, 0.0]
            # Ước lượng kích thước thô từ bbox + depth
            w_m = sx * z / self.fx if self.fx > 1e-6 else 0.3
            h_m = sy * z / self.fy if self.fy > 1e-6 else 0.3
            obj.dimensions = [max(0.1, w_m), max(0.1, h_m), max(0.1, min(w_m, h_m))]
            obj.tracking_state = 1  # OK — fusion bỏ qua state==0
            obj.action_state = 0
            obj.bounding_box_2d_tl = [u - sx * 0.5, v - sy * 0.5]
            obj.bounding_box_2d_br = [u + sx * 0.5, v + sy * 0.5]
            out.objects.append(obj)

        self.objects_pub.publish(out)
        rospy.loginfo_throttle(2.0, "YOLO→/objects: %d objs", len(out.objects))


if __name__ == "__main__":
    rospy.init_node("yolo_objects_bridge")
    YoloObjectsBridge()
    rospy.spin()
