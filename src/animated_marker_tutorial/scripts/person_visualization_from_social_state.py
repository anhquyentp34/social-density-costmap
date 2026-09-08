#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Đánh dấu người trên RViz giống mô phỏng, từ /social_state (robot thật Track D).

Skeleton / facing vẫn do ZED + fusion tính toán; node này chỉ vẽ mesh người
(AnimatedMarker) + nhãn, không vẽ skeleton hay mũi tên hướng mặt.
"""

from __future__ import annotations

import math
import re

import rospy
import tf.transformations as tft
from animated_marker_msgs.msg import AnimatedMarker, AnimatedMarkerArray
from geometry_msgs.msg import Quaternion
from social_msgs.msg import SocialState
from visualization_msgs.msg import Marker, MarkerArray


def _stable_person_id(name: str) -> int:
    digits = "".join(c for c in name if c.isdigit())
    if digits:
        return int(digits)
    return hash(name) & 0x7FFFFFFF


def _yaw_from_orientation(ori) -> float:
    q = [ori.x, ori.y, ori.z, ori.w]
    _, _, yaw = tft.euler_from_quaternion(q)
    return float(yaw)


class PersonVisualizationFromSocialState(object):
    def __init__(self):
        self._social_topic = rospy.get_param("~social_state_topic", "/social_state")
        self._output_topic = rospy.get_param("~output_topic", "/actor_visualization")
        self._frame_id = rospy.get_param("~frame_id", "map")
        self._publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self._stationary_speed_threshold = float(
            rospy.get_param("~stationary_speed_threshold", 0.08)
        )
        self._stationary_animation_speed = float(
            rospy.get_param("~stationary_animation_speed", 0.0)
        )
        default_moving_mesh = rospy.get_param(
            "~human_mesh_resource",
            "package://animated_marker_tutorial/meshes/animated_walking_man.mesh",
        )
        self._moving_human_mesh_resource = rospy.get_param(
            "~moving_human_mesh_resource", default_moving_mesh
        )
        self._standing_human_mesh_resource = rospy.get_param(
            "~standing_human_mesh_resource",
            "package://x_omni4wd_gazebo/models/person_standing/meshes/standing.dae",
        )
        # Chiều cao mục tiêu trên RViz (m). Scale = target / native_height.
        self._target_person_height = float(rospy.get_param("~target_person_height", 1.75))
        # standing.dae: AABB z ≈ 0..1.90 m (đo từ mesh)
        self._standing_mesh_native_height = float(
            rospy.get_param("~standing_mesh_native_height", 1.90)
        )
        # animated_walking_man.mesh: scale 0.38 trong sim ≈ người Gazebo ~1.75 m
        # → native ≈ 1.75/0.38 ≈ 4.605 m
        self._moving_mesh_native_height = float(
            rospy.get_param("~moving_mesh_native_height", 4.605)
        )
        # Cho phép override thủ công; nếu <0 thì tự tính từ target_person_height
        moving_override = float(rospy.get_param("~human_mesh_scale", -1.0))
        standing_override = float(rospy.get_param("~standing_human_mesh_scale", -1.0))
        self._moving_human_mesh_scale = (
            moving_override
            if moving_override > 0.0
            else self._target_person_height / max(self._moving_mesh_native_height, 1e-3)
        )
        self._standing_human_mesh_scale = (
            standing_override
            if standing_override > 0.0
            else self._target_person_height / max(self._standing_mesh_native_height, 1e-3)
        )
        self._moving_human_mesh_z_offset = float(rospy.get_param("~human_mesh_z_offset", 0.0))
        self._standing_human_mesh_z_offset = float(
            rospy.get_param("~standing_human_mesh_z_offset", 0.0)
        )
        self._show_name_text = bool(rospy.get_param("~show_name_text", True))
        self._show_emotion_in_text = bool(rospy.get_param("~show_emotion_in_text", True))
        self._color_by_emotion = bool(rospy.get_param("~color_by_emotion", True))
        self._use_unique_colors = bool(rospy.get_param("~use_unique_colors", False))
        self._mesh_use_embedded_materials = bool(
            rospy.get_param("~mesh_use_embedded_materials", False)
        )
        # ZED /social_state thường cho z ≈ cao thân (~0.8–1.2 m) → mesh lơ lửng.
        # Mặc định ghim chân xuống mặt sàn map (giống actor Gazebo).
        self._pin_to_ground = bool(rospy.get_param("~pin_to_ground", True))
        self._ground_z = float(rospy.get_param("~ground_z", 0.0))
        self._default_person_z = float(rospy.get_param("~default_person_z", 0.0))
        name_regex = rospy.get_param("~person_name_regex", r"^person_\d+$|^actor\d+$")
        self._name_re = re.compile(name_regex)
        self._color_palette = [
            (0.90, 0.25, 0.25),
            (0.25, 0.55, 0.95),
            (0.20, 0.75, 0.35),
            (0.95, 0.65, 0.20),
            (0.70, 0.40, 0.95),
            (0.15, 0.75, 0.75),
            (0.95, 0.40, 0.70),
            (0.70, 0.70, 0.20),
        ]
        # Angry = đỏ; còn lại = xanh lá (đồng bộ box FER)
        self._emotion_colors = {
            "Angry": (0.95, 0.15, 0.15),
            "Happy": (0.15, 0.80, 0.25),
            "Neutral": (0.15, 0.80, 0.25),
            "Sad": (0.15, 0.80, 0.25),
        }

        self._latest = None
        self._active_ids = set()
        self._pub = rospy.Publisher(self._output_topic, AnimatedMarkerArray, queue_size=1)
        self._text_pub = None
        if self._show_name_text:
            self._text_pub = rospy.Publisher(
                self._output_topic + "_text", MarkerArray, queue_size=1
            )

        rospy.Subscriber(self._social_topic, SocialState, self._cb, queue_size=1)
        period = 1.0 / max(self._publish_rate, 0.5)
        rospy.Timer(rospy.Duration(period), self._on_timer)
        rospy.loginfo(
            "person_visualization_from_social_state: %s -> %s "
            "(height=%.2fm stand_scale=%.3f move_scale=%.3f pin_to_ground=%s)",
            self._social_topic,
            self._output_topic,
            self._target_person_height,
            self._standing_human_mesh_scale,
            self._moving_human_mesh_scale,
            self._pin_to_ground,
        )

    def _cb(self, msg):
        self._latest = msg

    def _person_color(self, name, emotion, mid):
        if self._color_by_emotion:
            return self._emotion_colors.get(emotion, (0.15, 0.80, 0.25))
        if self._use_unique_colors:
            return self._color_palette[mid % len(self._color_palette)]
        return (0.85, 0.85, 0.88)

    def _on_timer(self, _evt):
        if self._latest is None:
            return
        msg = self._latest
        now = rospy.Time.now()
        frame = msg.header.frame_id if msg.header.frame_id else self._frame_id
        arr = AnimatedMarkerArray()
        text_arr = MarkerArray() if self._show_name_text else None
        seen = set()

        for person in msg.people.people:
            name = person.name or ""
            if not self._name_re.match(name):
                continue
            pos = person.position.position
            if not (math.isfinite(pos.x) and math.isfinite(pos.y)):
                continue

            speed = math.hypot(person.velocity.linear.x, person.velocity.linear.y)
            is_stationary = speed < self._stationary_speed_threshold
            mid = _stable_person_id(name)
            seen.add(mid)

            yaw = _yaw_from_orientation(person.position.orientation)
            # Mesh đi bộ: cùng quy ước actor_visualization_node (roll=90°, yaw+90°)
            # Mesh đứng: upright + 90° quanh Z
            if is_stationary:
                q_fix = tft.quaternion_from_euler(0.0, 0.0, yaw + 1.5708)
                mesh_resource = self._standing_human_mesh_resource
                mesh_scale = self._standing_human_mesh_scale
                z_off = self._standing_human_mesh_z_offset
                anim_speed = self._stationary_animation_speed
            else:
                theta_deg = yaw * 180.0 / math.pi
                q_fix = tft.quaternion_from_euler(
                    math.pi / 2.0, 0.0, (theta_deg + 90.0) / 180.0 * math.pi
                )
                mesh_resource = self._moving_human_mesh_resource
                mesh_scale = self._moving_human_mesh_scale
                z_off = self._moving_human_mesh_z_offset
                anim_speed = 0.7 * min(speed, 3.0)

            z = pos.z if abs(pos.z) > 1e-3 else self._default_person_z
            emotion = person.emotion or ""
            cr, cg, cb = self._person_color(name, emotion, mid)

            m = AnimatedMarker()
            m.header.frame_id = frame
            m.header.stamp = now
            m.ns = "actors"
            m.id = mid
            m.type = AnimatedMarker.MESH_RESOURCE
            m.action = AnimatedMarker.ADD
            m.pose.position.x = pos.x
            m.pose.position.y = pos.y
            # Chân trên sàn: không dùng z thân từ ZED (tránh lơ lửng)
            if self._pin_to_ground:
                m.pose.position.z = self._ground_z + z_off
                text_z = self._ground_z + z_off + self._target_person_height + 0.15
            else:
                m.pose.position.z = z + z_off
                text_z = z + z_off + self._target_person_height + 0.15
            m.pose.orientation = Quaternion(q_fix[0], q_fix[1], q_fix[2], q_fix[3])
            m.scale.x = m.scale.y = m.scale.z = mesh_scale
            m.color.r = cr
            m.color.g = cg
            m.color.b = cb
            m.color.a = 1.0
            m.lifetime = rospy.Duration(0.5)
            m.frame_locked = True
            m.mesh_use_embedded_materials = self._mesh_use_embedded_materials
            m.mesh_resource = mesh_resource
            m.animation_speed = anim_speed
            arr.markers.append(m)

            if text_arr is not None:
                tm = Marker()
                tm.header.frame_id = frame
                tm.header.stamp = now
                tm.ns = "actor_names"
                tm.id = 10000 + mid
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = pos.x
                tm.pose.position.y = pos.y
                tm.pose.position.z = text_z
                tm.pose.orientation.w = 1.0
                tm.scale.z = 0.35
                # Cùng màu cảm xúc với mesh (Angry=đỏ, còn lại=xanh)
                tm.color.r = cr
                tm.color.g = cg
                tm.color.b = cb
                tm.color.a = 1.0
                if self._show_emotion_in_text and emotion:
                    tm.text = "{} | {}".format(name, emotion)
                else:
                    tm.text = name
                tm.lifetime = rospy.Duration(0.5)
                text_arr.markers.append(tm)

        # Xóa marker người đã mất track
        for old_id in self._active_ids - seen:
            dm = AnimatedMarker()
            dm.header.frame_id = frame
            dm.header.stamp = now
            dm.ns = "actors"
            dm.id = old_id
            dm.action = AnimatedMarker.DELETE
            arr.markers.append(dm)
            if text_arr is not None:
                dt = Marker()
                dt.header.frame_id = frame
                dt.header.stamp = now
                dt.ns = "actor_names"
                dt.id = 10000 + old_id
                dt.action = Marker.DELETE
                text_arr.markers.append(dt)
        self._active_ids = seen

        self._pub.publish(arr)
        if text_arr is not None and self._text_pub is not None:
            self._text_pub.publish(text_arr)


if __name__ == "__main__":
    rospy.init_node("person_visualization_from_social_state", anonymous=False)
    PersonVisualizationFromSocialState()
    rospy.spin()
