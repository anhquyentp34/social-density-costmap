#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Ghép semantic lên ảnh ZED boxes (/image_skeleton) → một khung hình.

  /image_skeleton + /semantic/labels  →  /image_skeleton_semantic
"""

from __future__ import print_function

import threading

import cv2
import cv_bridge
import message_filters
import numpy as np
import rospy
from sensor_msgs.msg import Image

# Palette VOC (cùng semantic_seg_node)
_VOC_COLORS = np.array(
    [
        [0, 0, 0],
        [128, 0, 0],
        [0, 128, 0],
        [128, 128, 0],
        [0, 0, 128],
        [128, 0, 128],
        [0, 128, 128],
        [128, 128, 128],
        [64, 0, 0],
        [192, 0, 0],
        [64, 128, 0],
        [192, 128, 0],
        [64, 0, 128],
        [192, 0, 128],
        [64, 128, 128],
        [192, 128, 128],
        [0, 64, 0],
        [128, 64, 0],
        [0, 192, 0],
        [128, 192, 0],
        [0, 64, 128],
    ],
    dtype=np.uint8,
)


class SemanticZedOverlay(object):
    def __init__(self):
        self.base_topic = rospy.get_param("~base_image_topic", "/image_skeleton")
        self.labels_topic = rospy.get_param("~labels_topic", "/semantic/labels")
        self.out_topic = rospy.get_param("~output_topic", "/image_skeleton_semantic")
        self.alpha = float(rospy.get_param("~alpha", 0.45))
        self.slop = float(rospy.get_param("~sync_slop", 0.25))
        self.overlay_background = bool(rospy.get_param("~overlay_background", False))

        self._bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._last_labels = None

        self.pub = rospy.Publisher(self.out_topic, Image, queue_size=1)

        # Labels chậm hơn camera → giữ label mới nhất; sync mềm với approximate
        rospy.Subscriber(self.labels_topic, Image, self._cb_labels, queue_size=1, buff_size=2**22)

        base_sub = message_filters.Subscriber(self.base_topic, Image)
        self.sync = message_filters.ApproximateTimeSynchronizer(
            [base_sub], queue_size=5, slop=self.slop, allow_headerless=False
        )
        # ApproximateTimeSynchronizer với 1 sub không hợp lệ — dùng callback trực tiếp
        rospy.Subscriber(self.base_topic, Image, self._cb_base, queue_size=1, buff_size=2**24)

        rospy.loginfo(
            "semantic↔ZED overlay: %s + %s → %s (alpha=%.2f)",
            self.base_topic,
            self.labels_topic,
            self.out_topic,
            self.alpha,
        )

    def _cb_labels(self, msg):
        try:
            labels = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
        except Exception as e:
            rospy.logwarn_throttle(2.0, "labels convert failed: %s", e)
            return
        if labels.ndim == 3:
            labels = labels[:, :, 0]
        with self._lock:
            self._last_labels = np.asarray(labels)

    def _cb_base(self, msg):
        with self._lock:
            labels = self._last_labels
        try:
            base = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        except Exception:
            try:
                raw = self._bridge.imgmsg_to_cv2(msg, desired_encoding="passthrough")
                if raw.ndim == 2:
                    base = cv2.cvtColor(raw, cv2.COLOR_GRAY2BGR)
                elif raw.shape[2] == 4:
                    base = cv2.cvtColor(raw, cv2.COLOR_BGRA2BGR)
                else:
                    base = raw
            except Exception as e:
                rospy.logwarn_throttle(2.0, "base convert failed: %s", e)
                return

        out = base
        if labels is not None:
            h, w = base.shape[:2]
            if labels.shape[0] != h or labels.shape[1] != w:
                labels = cv2.resize(
                    labels.astype(np.int32), (w, h), interpolation=cv2.INTER_NEAREST
                )
            idx = np.clip(labels, 0, len(_VOC_COLORS) - 1)
            color = _VOC_COLORS[idx]
            if self.overlay_background:
                mask = np.ones((h, w), dtype=bool)
            else:
                mask = labels > 0  # bỏ background
            blended = base.copy()
            if np.any(mask):
                a = self.alpha
                blended[mask] = (
                    (1.0 - a) * base[mask].astype(np.float32)
                    + a * color[mask].astype(np.float32)
                ).astype(np.uint8)
            out = blended

        out_msg = Image()
        out_msg.header = msg.header
        out_msg.height = int(out.shape[0])
        out_msg.width = int(out.shape[1])
        out_msg.encoding = "bgr8"
        out_msg.is_bigendian = False
        out_msg.step = int(3 * out_msg.width)
        out_msg.data = np.ascontiguousarray(out).tobytes()
        self.pub.publish(out_msg)


if __name__ == "__main__":
    rospy.init_node("semantic_zed_overlay")
    SemanticZedOverlay()
    rospy.spin()
