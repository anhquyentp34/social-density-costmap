#!/usr/bin/env python3
"""Track D — FER realtime từ ZED /image_raw + /bodies → /person_emotions."""

from __future__ import annotations

import os
import sys
from collections import deque, defaultdict
from typing import Deque, Dict, List, Optional, Tuple

import rospy
from cv_bridge import CvBridge, CvBridgeError
from message_filters import ApproximateTimeSynchronizer, Subscriber
from sensor_msgs.msg import Image
from zed_body_tracking.msg import BodiesStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from emotion_fer_backend import FerPlusBackend, crop_face_from_body  # noqa: E402

from social_density_fusion.msg import PersonEmotion, PersonEmotionsStamped  # noqa: E402


class EmotionSmoother:
    """Majority vote trên N frame gần nhất cho mỗi person_id."""

    def __init__(self, window: int = 5):
        self.window = max(1, window)
        self._history: Dict[int, Deque[Tuple[str, str, float]]] = defaultdict(
            lambda: deque(maxlen=self.window)
        )

    def update(self, person_id: int, social: str, raw: str, conf: float) -> Tuple[str, str, float]:
        self._history[person_id].append((social, raw, conf))
        hist = list(self._history[person_id])
        social_votes: Dict[str, float] = defaultdict(float)
        raw_votes: Dict[str, float] = defaultdict(float)
        for s, r, c in hist:
            social_votes[s] += c
            raw_votes[r] += c
        best_social = max(social_votes.items(), key=lambda kv: kv[1])[0]
        best_raw = max(raw_votes.items(), key=lambda kv: kv[1])[0]
        avg_conf = sum(c for s, r, c in hist if s == best_social) / max(
            1, sum(1 for s, r, c in hist if s == best_social)
        )
        return best_social, best_raw, float(avg_conf)


class EmotionDetectionNode:
    TRACKING_OK = 1
    TRACKING_SEARCHING = 2

    @staticmethod
    def person_name(body_id: int) -> str:
        return "person_{}".format(body_id)

    def __init__(self):
        rospy.init_node("emotion_detection_node", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/image_raw")
        self.bodies_topic = rospy.get_param("~bodies_topic", "/bodies")
        self.output_topic = rospy.get_param("~output_topic", "/person_emotions")
        self.model_path = rospy.get_param("~model_path", "")
        self.min_body_confidence = float(rospy.get_param("~min_body_confidence", 40.0))
        self.min_fer_confidence = float(rospy.get_param("~min_fer_confidence", 0.35))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.08))
        self.sync_queue = int(rospy.get_param("~sync_queue", 10))
        self.smooth_window = int(rospy.get_param("~smooth_window", 5))
        self.crop_scale = float(rospy.get_param("~crop_scale", 2.2))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))

        self.backend = FerPlusBackend(self.model_path or None)
        self.smoother = EmotionSmoother(self.smooth_window)
        self.bridge = CvBridge()

        self.pub = rospy.Publisher(self.output_topic, PersonEmotionsStamped, queue_size=5)

        img_sub = Subscriber(self.image_topic, Image)
        bodies_sub = Subscriber(self.bodies_topic, BodiesStamped)
        self.sync = ApproximateTimeSynchronizer(
            [img_sub, bodies_sub], self.sync_queue, self.sync_slop, allow_headerless=False
        )
        self.sync.registerCallback(self.sync_callback)

        rospy.loginfo(
            "emotion_detection_node: FER+ model=%s image=%s bodies=%s → %s",
            self.backend.model_path,
            self.image_topic,
            self.bodies_topic,
            self.output_topic,
        )

    def sync_callback(self, image_msg: Image, bodies_msg: BodiesStamped):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "CvBridge failed: %s", exc)
            return

        out = PersonEmotionsStamped()
        out.header = bodies_msg.header
        if not out.header.stamp:
            out.header.stamp = image_msg.header.stamp
        if not out.header.frame_id:
            out.header.frame_id = image_msg.header.frame_id

        detected = 0
        for body in bodies_msg.bodies:
            if body.tracking_state not in (self.TRACKING_OK, self.TRACKING_SEARCHING):
                continue
            if float(body.confidence) < self.min_body_confidence:
                continue

            crop = crop_face_from_body(
                image_bgr,
                body.keypoints_2d,
                int(body.body_format) if body.body_format else 34,
                bbox_tl=body.bounding_box_2d_tl,
                bbox_br=body.bounding_box_2d_br,
                crop_scale=self.crop_scale,
            )
            if crop is None:
                continue

            social, raw, conf = self.backend.predict(crop)
            if conf < self.min_fer_confidence:
                social, raw = "Neutral", "neutral"

            social, raw, conf = self.smoother.update(int(body.id), social, raw, conf)

            pe = PersonEmotion()
            pe.person_id = int(body.id)
            pe.name = self.person_name(body.id)
            pe.emotion = social
            pe.raw_emotion = raw
            pe.confidence = conf
            out.emotions.append(pe)
            detected += 1

            if self.publish_debug:
                rospy.loginfo_throttle(
                    2.0,
                    "[FER] %s → %s (%s conf=%.2f)",
                    pe.name,
                    pe.emotion,
                    pe.raw_emotion,
                    pe.confidence,
                )

        self.pub.publish(out)
        if detected == 0:
            rospy.logdebug_throttle(5.0, "emotion_detection_node: no face crop this frame")


def main():
    try:
        EmotionDetectionNode()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    except FileNotFoundError as exc:
        rospy.logfatal("%s", exc)
        raise SystemExit(1)


if __name__ == "__main__":
    main()
