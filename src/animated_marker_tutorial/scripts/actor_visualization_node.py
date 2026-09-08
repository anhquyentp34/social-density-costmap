#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish AnimatedMarkerArray (+ optional MarkerArray text) từ /gazebo/model_states (actor đứng + di chuyển)."""

import math
import os
import re

import rospy
import tf.transformations as tft
from animated_marker_msgs.msg import AnimatedMarker, AnimatedMarkerArray
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Quaternion
from visualization_msgs.msg import Marker, MarkerArray


def _package_uri_exists(uri):
    if not uri.startswith("package://"):
        return os.path.isfile(uri)
    rest = uri[len("package://") :]
    pkg, _, rel = rest.partition("/")
    try:
        from rospkg import RosPack

        return os.path.isfile(os.path.join(RosPack().get_path(pkg), rel))
    except Exception:
        return False


def _stable_actor_id(name):
    digits = "".join(c for c in name if c.isdigit())
    return int(digits) if digits else (hash(name) & 0x7FFFFFFF)


class ActorVisualizationNode(object):
    def __init__(self):
        self._model_states_topic = rospy.get_param("~model_states_topic", "/gazebo/model_states")
        self._output_topic = rospy.get_param("~output_topic", "/actor_visualization")
        self._frame_id = rospy.get_param("~frame_id", "map")
        self._publish_rate = float(rospy.get_param("~publish_rate", 15.0))
        self._stationary_speed_threshold = float(rospy.get_param("~stationary_speed_threshold", 0.05))
        # Khi v ≈ 0: 0 = đóng băng animation đi bộ (giống đứng yên); >0 = vẫn lướt nhẹ
        self._stationary_animation_speed = float(rospy.get_param("~stationary_animation_speed", 0.0))
        default_moving_mesh = rospy.get_param(
            "~human_mesh_resource",
            "package://animated_marker_tutorial/meshes/animated_walking_man.mesh",
        )
        self._moving_human_mesh_resource = rospy.get_param("~moving_human_mesh_resource", default_moving_mesh)
        self._standing_human_mesh_resource = rospy.get_param(
            "~standing_human_mesh_resource",
            "package://x_omni4wd_gazebo/models/person_standing/meshes/standing.dae",
        )
        # Mesh đi bộ (.mesh) cần scale nhỏ để khớp actor Gazebo.
        self._moving_human_mesh_scale = float(rospy.get_param("~human_mesh_scale", 0.38))
        self._moving_human_mesh_z_offset = float(rospy.get_param("~human_mesh_z_offset", 0.0))
        # Mesh đứng (.dae) dùng hệ trục/kích thước khác: để upright và tỉ lệ gần actor 30/31.
        self._standing_human_mesh_scale = float(rospy.get_param("~standing_human_mesh_scale", 0.9))
        self._standing_human_mesh_z_offset = float(rospy.get_param("~standing_human_mesh_z_offset", 0.0))
        self._show_name_text = bool(rospy.get_param("~show_name_text", True))
        self._use_actor_unique_colors = bool(rospy.get_param("~use_actor_unique_colors", True))
        self._mesh_use_embedded_materials = bool(rospy.get_param("~mesh_use_embedded_materials", False))
        # Mọi model actor1, actor2, … (bộ lọc tên)
        actor_regex = rospy.get_param("~actor_name_regex", r"^actor\d+$")
        self._actor_re = re.compile(actor_regex)
        # Nhóm “đứng yên” trong RViz (mesh tĩnh, animation_speed=0). Mặc định: ICR2026_env3 — actor1–29 tĩnh, từ 30 trở đi đi bộ.
        # Với ICR2026_env5_65actor: đặt ~static_actor_name_regex trong launch (actor1–62 tĩnh, 63–64 đi bộ).
        static_actor_regex = rospy.get_param("~static_actor_name_regex", r"^actor([1-9]|1[0-9]|2[0-9])$")
        self._static_actor_re = re.compile(static_actor_regex)
        self._color_palette = [
            (0.90, 0.25, 0.25),
            (0.25, 0.55, 0.95),
            (0.20, 0.75, 0.35),
            (0.95, 0.65, 0.20),
            (0.70, 0.40, 0.95),
            (0.15, 0.75, 0.75),
            (0.95, 0.40, 0.70),
            (0.70, 0.70, 0.20),
            (0.95, 0.55, 0.35),
            (0.45, 0.65, 0.95),
        ]

        if not _package_uri_exists(self._moving_human_mesh_resource):
            rospy.logwarn(
                "Không thấy mesh đi bộ %s — dùng standing.dae cho RViz MarkerArray.",
                self._moving_human_mesh_resource,
            )
            self._moving_human_mesh_resource = self._standing_human_mesh_resource
            self._moving_human_mesh_scale = self._standing_human_mesh_scale

        self._last_pos = {}
        self._latest = None
        self._pub = rospy.Publisher(self._output_topic, AnimatedMarkerArray, queue_size=1)
        # RViz mặc định không có plugin AnimatedMarkerArray → thêm MarkerArray thường.
        self._mesh_pub = rospy.Publisher(self._output_topic + "_mesh", MarkerArray, queue_size=1)
        self._text_pub = None
        if self._show_name_text:
            self._text_pub = rospy.Publisher(self._output_topic + "_text", MarkerArray, queue_size=1)

        rospy.Subscriber(self._model_states_topic, ModelStates, self._cb, queue_size=1)
        period = 1.0 / max(self._publish_rate, 0.5)
        rospy.Timer(rospy.Duration(period), self._on_timer)

        rospy.loginfo(
            "actor_visualization_node: %s -> %s (mesh=%s)",
            self._model_states_topic,
            self._output_topic,
            self._moving_human_mesh_resource,
        )

    def _cb(self, msg):
        self._latest = msg

    def _on_timer(self, _evt):
        if self._latest is None:
            return
        msg = self._latest
        now = rospy.Time.now()
        arr = AnimatedMarkerArray()
        mesh_arr = MarkerArray()
        text_arr = MarkerArray() if self._show_name_text else None

        for i, name in enumerate(msg.name):
            if "_collision" in name:
                continue
            if not self._actor_re.match(name):
                continue

            pose = msg.pose[i]
            pos = pose.position
            ori = pose.orientation
            q = [ori.x, ori.y, ori.z, ori.w]
            roll, pitch, yaw = tft.euler_from_quaternion(q)
            theta_deg = yaw * 180.0 / math.pi

            speed = 0.0
            if name in self._last_pos:
                lx, ly, lz, lt = self._last_pos[name]
                dt = (now - lt).to_sec()
                if dt > 1e-6:
                    speed = math.sqrt((pos.x - lx) ** 2 + (pos.y - ly) ** 2) / dt
            self._last_pos[name] = (pos.x, pos.y, pos.z, now)

            if speed < self._stationary_speed_threshold:
                anim_speed = self._stationary_animation_speed
            else:
                anim_speed = 0.7 * min(speed, 3.0)
            is_static_actor = bool(self._static_actor_re.match(name))

            mid = _stable_actor_id(name)
            m = AnimatedMarker()
            m.header.frame_id = self._frame_id
            m.header.stamp = now
            m.ns = "actors"
            m.id = mid
            m.type = AnimatedMarker.MESH_RESOURCE
            m.action = AnimatedMarker.ADD
            m.pose.position.x = pos.x
            m.pose.position.y = pos.y
            if is_static_actor:
                m.pose.position.z = pos.z + self._standing_human_mesh_z_offset
                # standing.dae vốn upright, không xoay roll/pitch để tránh "nằm".
                # Cộng thêm +90deg quanh trục Z để hướng mặt khớp trục Ox của actor Gazebo.
                q_fix = tft.quaternion_from_euler(0.0, 0.0, yaw + 1.5708)
                mesh_scale = self._standing_human_mesh_scale
            else:
                m.pose.position.z = pos.z + self._moving_human_mesh_z_offset
                q_fix = tft.quaternion_from_euler(math.pi / 2.0, 0.0, (theta_deg + 90.0) / 180.0 * math.pi)
                mesh_scale = self._moving_human_mesh_scale
            m.pose.orientation = Quaternion(q_fix[0], q_fix[1], q_fix[2], q_fix[3])
            m.scale.x = m.scale.y = m.scale.z = mesh_scale
            if self._use_actor_unique_colors:
                cr, cg, cb = self._color_palette[mid % len(self._color_palette)]
            else:
                cr, cg, cb = (0.85, 0.85, 0.88)
            m.color.r = cr
            m.color.g = cg
            m.color.b = cb
            m.color.a = 1.0
            m.lifetime = rospy.Duration(0)
            m.frame_locked = True
            m.mesh_use_embedded_materials = self._mesh_use_embedded_materials
            if is_static_actor:
                m.mesh_resource = self._standing_human_mesh_resource
                m.animation_speed = 0.0
            else:
                m.mesh_resource = self._moving_human_mesh_resource
                m.animation_speed = anim_speed
            arr.markers.append(m)

            sm = Marker()
            sm.header = m.header
            sm.ns = "actors"
            sm.id = mid
            sm.type = Marker.MESH_RESOURCE
            sm.action = Marker.ADD
            sm.pose = m.pose
            sm.scale = m.scale
            sm.color = m.color
            sm.lifetime = rospy.Duration(0)
            sm.frame_locked = True
            sm.mesh_resource = m.mesh_resource
            sm.mesh_use_embedded_materials = m.mesh_use_embedded_materials
            mesh_arr.markers.append(sm)

            if text_arr is not None:
                tm = Marker()
                tm.header.frame_id = self._frame_id
                tm.header.stamp = now
                tm.ns = "actor_names"
                tm.id = 10000 + mid
                tm.type = Marker.TEXT_VIEW_FACING
                tm.action = Marker.ADD
                tm.pose.position.x = pos.x
                tm.pose.position.y = pos.y
                text_z_offset = self._standing_human_mesh_z_offset if is_static_actor else self._moving_human_mesh_z_offset
                tm.pose.position.z = pos.z + text_z_offset + 1.15
                tm.pose.orientation.w = 1.0
                tm.scale.z = 0.35
                tm.color.r = 1.0
                tm.color.g = 1.0
                tm.color.b = 1.0
                tm.color.a = 1.0
                tm.text = name
                tm.lifetime = rospy.Duration(0)
                text_arr.markers.append(tm)

        self._pub.publish(arr)
        self._mesh_pub.publish(mesh_arr)
        if text_arr is not None and self._text_pub is not None:
            self._text_pub.publish(text_arr)


if __name__ == "__main__":
    rospy.init_node("actor_visualization_node", anonymous=False)
    ActorVisualizationNode()
    rospy.spin()
