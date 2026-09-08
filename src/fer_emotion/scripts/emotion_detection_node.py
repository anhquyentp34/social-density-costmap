#!/usr/bin/env python3
"""Track D — FER realtime từ ZED /image_raw + /bodies → /person_emotions.

Mỗi ZED body.id được gán nhãn cảm xúc ổn định: name=person_{id}, emotion=Happy|Angry|Neutral.
"""

from __future__ import annotations

import os
import sys
from collections import deque, defaultdict
from typing import Deque, Dict, Optional, Set, Tuple

import cv2
import rospy
from cv_bridge import CvBridge, CvBridgeError
from sensor_msgs.msg import Image
from social_msgs.msg import SocialPeople, SocialPerson
from zed_sdk_bridge.msg import BodiesStamped

sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from emotion_fer_backend import FerPlusBackend, crop_face_from_body  # noqa: E402

EMOTION_BGR = {
    "Angry": (0, 0, 255),      # đỏ
    "Happy": (0, 255, 0),      # xanh lá
    "Neutral": (0, 255, 0),    # xanh lá
    "Sad": (0, 255, 0),        # xanh lá
}


class EmotionSmoother:
    """Vote theo cửa sổ + yêu cầu margin để đổi nhãn (tránh nhảy liên tục)."""

    def __init__(self, window: int = 8, switch_margin: float = 0.15):
        self.window = max(1, window)
        self.switch_margin = float(switch_margin)
        self._history: Dict[int, Deque[Tuple[str, str, float]]] = defaultdict(
            lambda: deque(maxlen=self.window)
        )
        self._locked: Dict[int, str] = {}

    def update(self, person_id: int, social: str, raw: str, conf: float) -> Tuple[str, str, float]:
        self._history[person_id].append((social, raw, conf))
        hist = list(self._history[person_id])
        social_votes: Dict[str, float] = defaultdict(float)
        raw_votes: Dict[str, float] = defaultdict(float)
        for s, r, c in hist:
            social_votes[s] += c
            raw_votes[r] += c

        ranked = sorted(social_votes.items(), key=lambda kv: kv[1], reverse=True)
        best_social = ranked[0][0]
        best_score = ranked[0][1]
        second_score = ranked[1][1] if len(ranked) > 1 else 0.0

        locked = self._locked.get(person_id)
        if locked is None:
            self._locked[person_id] = best_social
        elif best_social != locked:
            if best_score - second_score >= self.switch_margin * len(hist):
                self._locked[person_id] = best_social
            else:
                best_social = locked

        best_raw = max(raw_votes.items(), key=lambda kv: kv[1])[0]
        avg_conf = sum(c for s, r, c in hist if s == best_social) / max(
            1, sum(1 for s, r, c in hist if s == best_social)
        )
        return best_social, best_raw, float(avg_conf)

    def forget(self, person_id: int) -> None:
        self._history.pop(person_id, None)
        self._locked.pop(person_id, None)


class EmotionDetectionNode:
    TRACKING_OK = 1
    TRACKING_SEARCHING = 2

    @staticmethod
    def person_name(body_id: int) -> str:
        return "person_{}".format(int(body_id))

    def __init__(self):
        rospy.init_node("emotion_detection_node", anonymous=False)

        self.image_topic = rospy.get_param("~image_topic", "/image_raw")
        self.bodies_topic = rospy.get_param("~bodies_topic", "/bodies")
        self.output_topic = rospy.get_param("~output_topic", "/person_emotions")
        self.debug_image_topic = rospy.get_param("~debug_image_topic", "/image_emotion")
        self.model_path = rospy.get_param("~model_path", "")
        self.min_body_confidence = float(rospy.get_param("~min_body_confidence", 40.0))
        self.min_fer_confidence = float(rospy.get_param("~min_fer_confidence", 0.35))
        self.sync_slop = float(rospy.get_param("~sync_slop", 0.35))
        self.sync_queue = int(rospy.get_param("~sync_queue", 10))
        self.smooth_window = int(rospy.get_param("~smooth_window", 8))
        self.group_margin = float(rospy.get_param("~group_margin", 0.12))
        self.focus_emotion = str(rospy.get_param("~focus_emotion", "Angry"))
        self.angry_min_score = float(rospy.get_param("~angry_min_score", 0.12))
        self.angry_margin = float(rospy.get_param("~angry_margin", 0.04))
        self.angry_neutral_slack = float(rospy.get_param("~angry_neutral_slack", 0.08))
        self.angry_raw_spike = float(rospy.get_param("~angry_raw_spike", 0.04))
        self.crop_scale = float(rospy.get_param("~crop_scale", 2.2))
        self.publish_debug = bool(rospy.get_param("~publish_debug", True))
        self.hold_last_sec = float(rospy.get_param("~hold_last_sec", 2.0))

        self.backend = FerPlusBackend(
            self.model_path or None,
            group_margin=self.group_margin,
            focus_emotion=self.focus_emotion,
            angry_min_score=self.angry_min_score,
            angry_margin=self.angry_margin,
            angry_neutral_slack=self.angry_neutral_slack,
            angry_raw_spike=self.angry_raw_spike,
        )
        self.smoother = EmotionSmoother(self.smooth_window)
        self.bridge = CvBridge()
        # body.id -> (social, raw, conf, stamp)
        self.last_labels: Dict[int, Tuple[str, str, float, rospy.Time]] = {}

        self.pub = rospy.Publisher(self.output_topic, SocialPeople, queue_size=5)
        self.debug_pub = rospy.Publisher(self.debug_image_topic, Image, queue_size=1)
        self.latest_image = None
        self._last_infer = rospy.Time(0)
        self.max_infer_hz = float(rospy.get_param("~max_infer_hz", 10.0))

        rospy.Subscriber(self.image_topic, Image, self.image_callback, queue_size=1, buff_size=2**24)
        rospy.Subscriber(self.bodies_topic, BodiesStamped, self.bodies_callback, queue_size=1)

        rospy.loginfo(
            "emotion_detection_node: FER+ model=%s focus=%s angry_min=%.2f "
            "image=%s bodies=%s → %s (label=person_{id}) debug=%s",
            self.backend.model_path,
            self.focus_emotion,
            self.angry_min_score,
            self.image_topic,
            self.bodies_topic,
            self.output_topic,
            self.debug_image_topic,
        )

    def _make_person(self, body_id: int, social: str, conf: float) -> SocialPerson:
        pe = SocialPerson()
        pe.name = self.person_name(body_id)
        pe.emotion = social
        pe.reliability = float(conf)
        return pe

    @staticmethod
    def _draw_label(image, body, social: str, conf: float) -> None:
        tlx, tly = int(body.bounding_box_2d_tl[0]), int(body.bounding_box_2d_tl[1])
        brx, bry = int(body.bounding_box_2d_br[0]), int(body.bounding_box_2d_br[1])
        if brx <= tlx or bry <= tly:
            return
        color = EMOTION_BGR.get(social, (0, 255, 0))
        cv2.rectangle(image, (tlx, tly), (brx, bry), color, 2)
        text = "ID:{} {} {:.2f}".format(int(body.id), social, conf)
        (tw, th), baseline = cv2.getTextSize(text, cv2.FONT_HERSHEY_SIMPLEX, 0.55, 2)
        org = (tlx, max(th + 4, tly - 8))
        cv2.rectangle(
            image,
            (org[0] - 2, org[1] - th - 2),
            (org[0] + tw + 2, org[1] + baseline + 2),
            (0, 0, 0),
            -1,
        )
        cv2.putText(image, text, org, cv2.FONT_HERSHEY_SIMPLEX, 0.55, color, 2, cv2.LINE_AA)

    def image_callback(self, image_msg: Image):
        self.latest_image = image_msg

    def bodies_callback(self, bodies_msg: BodiesStamped):
        image_msg = self.latest_image
        if image_msg is None:
            return
        now = rospy.Time.now()
        if self.max_infer_hz > 0 and self._last_infer.to_sec() > 0:
            min_dt = 1.0 / self.max_infer_hz
            if (now - self._last_infer).to_sec() < min_dt:
                return
        self._last_infer = now
        dt = abs((bodies_msg.header.stamp - image_msg.header.stamp).to_sec())
        if dt > self.sync_slop:
            rospy.logwarn_throttle(
                5.0,
                "emotion_detection_node: image/bodies dt=%.3fs > %.2fs — vẫn gán nhãn ID bằng frame mới nhất",
                dt,
                self.sync_slop,
            )
        self.sync_callback(image_msg, bodies_msg)

    def sync_callback(self, image_msg: Image, bodies_msg: BodiesStamped):
        try:
            image_bgr = self.bridge.imgmsg_to_cv2(image_msg, desired_encoding="bgr8")
        except CvBridgeError as exc:
            rospy.logwarn_throttle(5.0, "CvBridge failed: %s", exc)
            return

        now = rospy.Time.now()
        out = SocialPeople()
        out.header = bodies_msg.header
        if not out.header.stamp:
            out.header.stamp = image_msg.header.stamp
        if not out.header.frame_id:
            out.header.frame_id = image_msg.header.frame_id

        debug = image_bgr.copy() if self.publish_debug else None
        seen: Set[int] = set()
        detected = 0

        for body in bodies_msg.bodies:
            if body.tracking_state not in (self.TRACKING_OK, self.TRACKING_SEARCHING):
                continue
            if float(body.confidence) < self.min_body_confidence:
                continue

            body_id = int(body.id)
            seen.add(body_id)
            social: Optional[str] = None
            raw = ""
            conf = 0.0

            crop = crop_face_from_body(
                image_bgr,
                body.keypoints_2d,
                int(body.body_format) if body.body_format else 34,
                bbox_tl=body.bounding_box_2d_tl,
                bbox_br=body.bounding_box_2d_br,
                crop_scale=self.crop_scale,
            )
            if crop is not None:
                social, raw, conf = self.backend.predict(crop)
                if conf < self.min_fer_confidence:
                    social, raw = "Neutral", "neutral"
                social, raw, conf = self.smoother.update(body_id, social, raw, conf)
                self.last_labels[body_id] = (social, raw, conf, now)
                detected += 1
            else:
                held = self.last_labels.get(body_id)
                if held is not None and (now - held[3]).to_sec() <= self.hold_last_sec:
                    social, raw, conf, _ = held

            if social is None:
                continue

            out.people.append(self._make_person(body_id, social, conf))
            if debug is not None:
                self._draw_label(debug, body, social, conf)
            if self.publish_debug:
                rospy.loginfo_throttle(
                    2.0,
                    "[FER] ID=%d name=%s → %s (%s conf=%.2f)",
                    body_id,
                    self.person_name(body_id),
                    social,
                    raw,
                    conf,
                )

        stale = [pid for pid in self.last_labels if pid not in seen]
        for pid in stale:
            age = (now - self.last_labels[pid][3]).to_sec()
            if age > self.hold_last_sec:
                self.last_labels.pop(pid, None)
                self.smoother.forget(pid)

        self.pub.publish(out)
        if debug is not None and self.debug_pub.get_num_connections() > 0:
            dbg_msg = self.bridge.cv2_to_imgmsg(debug, encoding="bgr8")
            dbg_msg.header = image_msg.header
            self.debug_pub.publish(dbg_msg)
        if detected == 0:
            rospy.logdebug_throttle(5.0, "emotion_detection_node: no new FER crop this frame")


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
