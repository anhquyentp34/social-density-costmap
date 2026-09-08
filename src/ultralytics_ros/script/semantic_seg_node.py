#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Semantic segmentation (DeepLabV3-ResNet50, VOC 21 class) → label map + color viz.

  /image_raw  →  /semantic/labels (16UC1) + /semantic/color (bgr8)
"""

from __future__ import print_function

import threading

import cv2
import cv_bridge
import numpy as np
import rospy
import torch
from PIL import Image as PILImage
from sensor_msgs.msg import Image
from torchvision.models.segmentation import DeepLabV3_ResNet50_Weights, deeplabv3_resnet50


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


class SemanticSegNode(object):
    def __init__(self):
        self.input_topic = rospy.get_param("~input_topic", "/image_raw")
        self.labels_topic = rospy.get_param("~labels_topic", "/semantic/labels")
        self.color_topic = rospy.get_param("~color_topic", "/semantic/color")
        self.device_str = str(rospy.get_param("~device", "cuda:0"))
        self.process_hz = float(rospy.get_param("~process_hz", 3.0))
        self.publish_color = bool(rospy.get_param("~publish_color", True))

        if self.device_str.startswith("cuda") and not torch.cuda.is_available():
            rospy.logwarn("CUDA không khả dụng — fallback CPU")
            self.device = torch.device("cpu")
        else:
            try:
                self.device = torch.device(self.device_str)
            except Exception:
                self.device = torch.device("cpu")

        weights = DeepLabV3_ResNet50_Weights.COCO_WITH_VOC_LABELS_V1
        self.preprocess = weights.transforms()
        self.model = deeplabv3_resnet50(weights=weights)
        self.model.to(self.device)
        self.model.eval()

        self._bridge = cv_bridge.CvBridge()
        self._lock = threading.Lock()
        self._latest_msg = None
        self._busy = False

        self.pub_labels = rospy.Publisher(self.labels_topic, Image, queue_size=1)
        self.pub_color = (
            rospy.Publisher(self.color_topic, Image, queue_size=1)
            if self.publish_color
            else None
        )
        rospy.Subscriber(self.input_topic, Image, self._cb_image, queue_size=1, buff_size=2**24)

        period = 1.0 / max(0.5, self.process_hz)
        rospy.Timer(rospy.Duration(period), self._timer_cb)

        rospy.loginfo(
            "semantic_seg: DeepLabV3-ResNet50 VOC | %s → %s | device=%s hz=%.1f",
            self.input_topic,
            self.labels_topic,
            self.device,
            self.process_hz,
        )

    def _cb_image(self, msg):
        with self._lock:
            self._latest_msg = msg

    @torch.no_grad()
    def _infer(self, msg):
        bgr = self._bridge.imgmsg_to_cv2(msg, desired_encoding="bgr8")
        rgb = np.ascontiguousarray(bgr[:, :, ::-1])
        pil = PILImage.fromarray(rgb)
        tensor = self.preprocess(pil)
        if tensor.ndim == 3:
            tensor = tensor.unsqueeze(0)
        tensor = tensor.to(self.device)

        out = self.model(tensor)["out"]
        labels_small = out.argmax(1)[0].detach().cpu().numpy().astype(np.uint16)

        h, w = bgr.shape[:2]
        if labels_small.shape[0] != h or labels_small.shape[1] != w:
            labels = cv2.resize(
                labels_small.astype(np.int32), (w, h), interpolation=cv2.INTER_NEAREST
            ).astype(np.uint16)
        else:
            labels = labels_small
        labels = np.ascontiguousarray(labels)

        label_msg = Image()
        label_msg.header = msg.header
        label_msg.height = int(labels.shape[0])
        label_msg.width = int(labels.shape[1])
        label_msg.encoding = "16UC1"
        label_msg.is_bigendian = False
        label_msg.step = int(2 * label_msg.width)
        label_msg.data = labels.astype(np.uint16).tobytes()
        self.pub_labels.publish(label_msg)

        if self.pub_color is not None:
            idx = np.clip(labels, 0, len(_VOC_COLORS) - 1)
            color = np.ascontiguousarray(_VOC_COLORS[idx], dtype=np.uint8)
            color_msg = Image()
            color_msg.header = msg.header
            color_msg.height = int(color.shape[0])
            color_msg.width = int(color.shape[1])
            color_msg.encoding = "bgr8"
            color_msg.is_bigendian = False
            color_msg.step = int(3 * color_msg.width)
            color_msg.data = color.tobytes()
            self.pub_color.publish(color_msg)

    def _timer_cb(self, _evt):
        if self._busy:
            return
        with self._lock:
            msg = self._latest_msg
            self._latest_msg = None
        if msg is None:
            return
        self._busy = True
        try:
            self._infer(msg)
        except Exception as e:
            import traceback
            rospy.logwarn_throttle(
                2.0, "semantic infer failed: %r\n%s", e, traceback.format_exc(limit=5)
            )
        finally:
            self._busy = False


if __name__ == "__main__":
    rospy.init_node("semantic_seg_node")
    SemanticSegNode()
    rospy.spin()
