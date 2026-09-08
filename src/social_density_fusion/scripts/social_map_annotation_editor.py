#!/usr/bin/env python3
"""Chỉnh actors / objects / emotion_zones trên RViz → lưu YAML / xuất kịch bản chuẩn.

Cách dùng:
  roslaunch social_density_fusion social_map_annotation_session.launch

RViz InteractiveMarkers: /social_map_annotation_controls/update
Pedestrian path lines: /social_map_annotation/pedestrian_paths (MarkerArray)

Thao tác:
  - Kéo actor / object / zone (Interact) → auto_save YAML; thả chuột → auto_export canonical
  - Pedestrian: marker cầu (waypoints) + đường LINE_STRIP; menu Add/Delete waypoint
  - Timer ~5s: nếu còn thay đổi → tự export canonical + sync exp5/6
  - Menu marker: Save YAML | Export canonical + MATLAB (thủ công)
  - rosservice call /social_map_annotation_editor/export_canonical std_srvs/Trigger
"""
from __future__ import print_function

import copy
import math
import os
import sys

import rospy
import yaml
from geometry_msgs.msg import Point, PointStamped, Pose, Quaternion
from interactive_markers.interactive_marker_server import InteractiveMarkerServer
from interactive_markers.menu_handler import MenuHandler
from std_msgs.msg import ColorRGBA
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import (
    InteractiveMarker,
    InteractiveMarkerControl,
    InteractiveMarkerFeedback,
    Marker,
    MarkerArray,
)

_SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
if _SCRIPT_DIR not in sys.path:
    sys.path.insert(0, _SCRIPT_DIR)
from export_annotation_to_canonical import export_scenario  # noqa: E402
from pathlib import Path  # noqa: E402


def _f(v, nd=2):
    return round(float(v), nd)


def _yaw_to_quat(yaw):
    return Quaternion(x=0.0, y=0.0, z=math.sin(yaw * 0.5), w=math.cos(yaw * 0.5))


def _quat_to_yaw(q):
    # z-up yaw from quaternion
    return math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))


class SocialMapAnnotationEditor(object):
    def __init__(self):
        rospy.init_node("social_map_annotation_editor")

        self.config_path = str(rospy.get_param("~config_path", ""))
        also = rospy.get_param("~also_update_paths", [])
        if isinstance(also, str):
            also = [also] if also else []
        self.also_update_paths = [str(p) for p in also if p]
        self.frame_id = str(rospy.get_param("~frame_id", "map"))
        self.fusion_reload_service = str(
            rospy.get_param(
                "~fusion_reload_service",
                "/social_density_fusion_node_pos/reload_config",
            )
        )
        self.zone_default_half = float(rospy.get_param("~zone_default_half_size", 1.0))
        self.auto_save = bool(rospy.get_param("~auto_save", True))
        self._last_auto_save = 0.0
        self._auto_save_min_interval = float(rospy.get_param("~auto_save_min_interval", 0.35))
        # Tự động Export canonical (+ sync exp5/6) — lưu kịch bản bền vững
        self.auto_export_canonical = bool(rospy.get_param("~auto_export_canonical", True))
        self._last_auto_export = 0.0
        self._auto_export_min_interval = float(
            rospy.get_param("~auto_export_min_interval", 2.0)
        )
        self._persist_timer_sec = float(rospy.get_param("~auto_persist_timer_sec", 5.0))
        self._dirty = False  # có thay đổi chưa ghi canonical

        cfg_dir = os.path.join(
            os.path.dirname(__file__), "..", "config", "D-density-costmap", "real"
        )
        cfg_dir = os.path.abspath(cfg_dir)
        self.canonical_path = str(
            rospy.get_param(
                "~canonical_path",
                os.path.join(cfg_dir, "social_scenario_canonical_trackD_real.yaml"),
            )
        )
        self.matlab_yaml_path = str(
            rospy.get_param(
                "~matlab_yaml_path",
                os.path.join(cfg_dir, "social_scenario_matlab_trackD_real.yaml"),
            )
        )
        self.actor_layout_path = str(
            rospy.get_param(
                "~actor_layout_path",
                os.path.join(cfg_dir, "social_actor_layout_from_sim.yaml"),
            )
        )
        self.exp5_path = str(
            rospy.get_param(
                "~exp5_path",
                os.path.join(cfg_dir, "social_config_real_exp5_social_layers_pos.yaml"),
            )
        )
        self.exp6_path = str(
            rospy.get_param(
                "~exp6_path",
                os.path.join(cfg_dir, "social_config_real_exp6_density_gate_pos.yaml"),
            )
        )
        self.sync_layout_on_export = bool(rospy.get_param("~sync_layout_on_export", True))
        self.sync_exp_on_export = bool(rospy.get_param("~sync_exp_on_export", True))
        self.matlab_pack_dir = str(rospy.get_param("~matlab_pack_dir", "")).strip()
        self.export_exp_variant = str(rospy.get_param("~export_exp_variant", "annotation"))
        self.reload_fusion_on_save = bool(rospy.get_param("~reload_fusion_on_save", False))
        self.gt_reload_service = str(
            rospy.get_param("~gt_reload_service", "/scenario_gt_social_state_publisher/reload")
        )
        # Số điểm tối thiểu trên quỹ đạo pedestrian (để kéo cong dễ hơn)
        self.pedestrian_path_min_points = max(
            2, int(rospy.get_param("~pedestrian_path_min_points", 5))
        )
        self.auto_densify_short_paths = bool(
            rospy.get_param("~auto_densify_short_paths", True)
        )

        if not self.config_path or not os.path.isfile(self.config_path):
            raise SystemExit("config_path khong hop le: %s" % self.config_path)

        self.cfg = self._load_yaml(self.config_path)
        self.objects = list(self.cfg.get("objects") or [])
        self.zones = list(self.cfg.get("emotion_zones") or [])
        self.actors = list(self.cfg.get("actors") or [])
        self.selected = {"kind": None, "index": -1}  # for clicked_point
        self._group_last_xy = None  # tâm cụm actor lần feedback trước (tịnh tiến cả cụm)

        for actor in self.actors:
            self._ensure_pedestrian_path(actor)
            if (
                self.auto_densify_short_paths
                and self._is_pedestrian(actor)
                and len(actor.get("waypoints") or []) < self.pedestrian_path_min_points
            ):
                self._densify_pedestrian_path(actor, self.pedestrian_path_min_points)

        self.server = InteractiveMarkerServer("social_map_annotation_controls")
        self.menu = MenuHandler()
        self.menu.insert("Save YAML + reload fusion", callback=self._menu_save)
        self.menu.insert("Export canonical + MATLAB", callback=self._menu_export_canonical)
        self.menu.insert("Select for Publish Point", callback=self._menu_select)
        self.menu.insert("Reset group handle to centroid", callback=self._menu_reset_group_handle)
        self.menu.insert("Add path waypoint (pedestrian)", callback=self._menu_add_waypoint)
        self.menu.insert(
            "Densify path (more points for curve)", callback=self._menu_densify_path
        )
        self.menu_wp = MenuHandler()
        self.menu_wp.insert("Add waypoint after this", callback=self._menu_add_waypoint_after)
        self.menu_wp.insert("Densify path (more points for curve)", callback=self._menu_densify_path)
        self.menu_wp.insert("Delete this waypoint", callback=self._menu_delete_waypoint)
        self.menu_wp.insert("Save YAML + reload fusion", callback=self._menu_save)
        self.menu_wp.insert("Export canonical + MATLAB", callback=self._menu_export_canonical)

        self.path_pub = rospy.Publisher(
            "/social_map_annotation/pedestrian_paths",
            MarkerArray,
            queue_size=1,
            latch=True,
        )

        rospy.Service("~save", Trigger, self.handle_save)
        rospy.Service("~export_canonical", Trigger, self.handle_export_canonical)
        rospy.Service("~reload_from_file", Trigger, self.handle_reload_file)
        rospy.Service("~recenter_actors_group", Trigger, self.handle_recenter_group)
        rospy.Subscriber("/clicked_point", PointStamped, self.clicked_point_cb, queue_size=1)

        self._rebuild_markers()
        self.server.applyChanges()

        if self.auto_save and self._persist_timer_sec > 0.0:
            self._persist_timer = rospy.Timer(
                rospy.Duration(self._persist_timer_sec), self._timer_auto_persist
            )
        else:
            self._persist_timer = None

        n_ped = sum(
            1
            for a in self.actors
            if str(a.get("role", "")).lower() == "pedestrian"
        )
        rospy.loginfo(
            "social_map_annotation_editor: %d actors (%d pedestrian), %d objects, %d zones | file=%s",
            len(self.actors),
            n_ped,
            len(self.objects),
            len(self.zones),
            self.config_path,
        )
        rospy.loginfo(
            "RViz: Interact → kéo actor/object/zone/waypoint | path lines=/social_map_annotation/pedestrian_paths | "
            "auto_save=%s | auto_export_canonical=%s (timer=%.1fs)",
            self.auto_save,
            self.auto_export_canonical,
            self._persist_timer_sec,
        )

    @staticmethod
    def _load_yaml(path):
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}
        if not isinstance(data, dict):
            raise ValueError("YAML root must be mapping")
        return data

    @staticmethod
    def _is_pedestrian(actor):
        return str(actor.get("role", "")).lower() == "pedestrian"

    def _ensure_pedestrian_path(self, actor):
        """Chuẩn hoá waypoints + walk_goal cho actor pedestrian.

        Schema:
          waypoints: [{x,y}, ...]  — tập điểm quỹ đạo (kéo thả trên RViz)
          walk_goal: {x,y}         — luôn = điểm cuối (tương thích cũ)
        """
        if not self._is_pedestrian(actor):
            return
        wps = actor.get("waypoints")
        if isinstance(wps, list) and wps:
            clean = []
            for p in wps:
                if isinstance(p, dict) and "x" in p and "y" in p:
                    clean.append({"x": _f(p["x"]), "y": _f(p["y"])})
            if clean:
                actor["waypoints"] = clean
                actor["walk_goal"] = {"x": clean[-1]["x"], "y": clean[-1]["y"]}
                return
        wg = actor.get("walk_goal")
        sx = float(actor.get("x", 0.0))
        sy = float(actor.get("y", 0.0))
        if isinstance(wg, dict) and "x" in wg and "y" in wg:
            gx, gy = _f(wg["x"]), _f(wg["y"])
        else:
            yaw = float(actor.get("yaw", 0.0))
            gx = _f(sx + 2.0 * math.cos(yaw))
            gy = _f(sy + 2.0 * math.sin(yaw))
        # Seed nhiều điểm thẳng start→goal để kéo lệch thành đường cong
        n = max(2, int(getattr(self, "pedestrian_path_min_points", 5)))
        actor["waypoints"] = self._sample_line_waypoints(sx, sy, gx, gy, n)
        actor["walk_goal"] = {"x": actor["waypoints"][-1]["x"], "y": actor["waypoints"][-1]["y"]}

    @staticmethod
    def _sample_line_waypoints(sx, sy, gx, gy, n_points):
        """n_points điểm đều trên đoạn (không gồm start)."""
        n_points = max(1, int(n_points))
        out = []
        for i in range(1, n_points + 1):
            t = float(i) / float(n_points)
            out.append({"x": _f(sx + t * (gx - sx)), "y": _f(sy + t * (gy - sy))})
        return out

    @staticmethod
    def _resample_polyline(points, n_points):
        """Lấy n_points điểm đều theo độ dài cung (points[0]=start, trả về không gồm start)."""
        if not points or n_points < 1:
            return []
        if len(points) == 1:
            return [{"x": _f(points[0][0]), "y": _f(points[0][1])}] * n_points

        seg_len = []
        total = 0.0
        for i in range(1, len(points)):
            dx = points[i][0] - points[i - 1][0]
            dy = points[i][1] - points[i - 1][1]
            L = math.hypot(dx, dy)
            seg_len.append(L)
            total += L
        if total < 1e-6:
            x, y = points[-1]
            return [{"x": _f(x), "y": _f(y)} for _ in range(n_points)]

        out = []
        for i in range(1, n_points + 1):
            target = total * (float(i) / float(n_points))
            acc = 0.0
            for si, L in enumerate(seg_len):
                if acc + L >= target - 1e-9 or si == len(seg_len) - 1:
                    t = 0.0 if L < 1e-9 else (target - acc) / L
                    t = max(0.0, min(1.0, t))
                    x0, y0 = points[si]
                    x1, y1 = points[si + 1]
                    out.append({"x": _f(x0 + t * (x1 - x0)), "y": _f(y0 + t * (y1 - y0))})
                    break
                acc += L
        # Bảo toàn điểm cuối
        out[-1] = {"x": _f(points[-1][0]), "y": _f(points[-1][1])}
        return out

    def _densify_pedestrian_path(self, actor, n_points=None):
        """Tăng số waypoint (resample đều) để dễ kéo quỹ đạo cong."""
        if not self._is_pedestrian(actor):
            return False
        self._ensure_pedestrian_path(actor)
        cur = actor.get("waypoints") or []
        target = int(n_points or self.pedestrian_path_min_points)
        # Mỗi lần densify: tối thiểu gấp đôi, hoặc chèn midpoint (+len)
        denser = max(target, len(cur) * 2, len(cur) + max(2, len(cur)))
        denser = min(denser, 24)  # trần tránh quá nhiều marker
        if denser <= len(cur):
            denser = min(24, len(cur) + 2)
        poly = [(float(actor.get("x", 0.0)), float(actor.get("y", 0.0)))]
        for wp in cur:
            poly.append((float(wp["x"]), float(wp["y"])))
        actor["waypoints"] = self._resample_polyline(poly, denser)
        self._sync_walk_goal_from_waypoints(actor)
        return True

    def _sync_walk_goal_from_waypoints(self, actor):
        if not self._is_pedestrian(actor):
            return
        wps = actor.get("waypoints") or []
        if not wps:
            self._ensure_pedestrian_path(actor)
            wps = actor.get("waypoints") or []
        if wps:
            actor["walk_goal"] = {"x": wps[-1]["x"], "y": wps[-1]["y"]}

    def _rebuild_markers(self):
        self.server.clear()
        for i, actor in enumerate(self.actors):
            self._ensure_pedestrian_path(actor)
            self._insert_actor_marker(i, actor)
            if self._is_pedestrian(actor):
                self._insert_pedestrian_path_markers(i, actor)
        if self.actors:
            self._insert_actors_group_marker()
        for i, obj in enumerate(self.objects):
            self._insert_object_marker(i, obj)
        for i, z in enumerate(self.zones):
            self._insert_zone_markers(i, z)
        self._publish_pedestrian_path_lines()

    def _actors_centroid(self):
        if not self.actors:
            return 0.0, 0.0
        sx = sum(float(a.get("x", 0.0)) for a in self.actors)
        sy = sum(float(a.get("y", 0.0)) for a in self.actors)
        n = float(len(self.actors))
        return sx / n, sy / n

    def _insert_actors_group_marker(self):
        """Marker tâm cụm — kéo để tịnh tiến toàn bộ actor (giữ khoảng cách tương đối)."""
        cx, cy = self._actors_centroid()
        self._group_last_xy = (cx, cy)

        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = "actors_group"
        im.description = "ALL_ACTORS (%d)\nInteract → kéo = tịnh tiến cả cụm" % len(self.actors)
        im.pose.position.x = cx
        im.pose.position.y = cy
        im.pose.position.z = 2.2
        im.pose.orientation.w = 1.0
        im.scale = 1.6

        ring = Marker()
        ring.type = Marker.CYLINDER
        ring.scale.x = ring.scale.y = 1.2
        ring.scale.z = 0.15
        ring.color.r, ring.color.g, ring.color.b, ring.color.a = 0.05, 0.85, 0.35, 0.85

        cross = Marker()
        cross.type = Marker.SPHERE
        cross.scale.x = cross.scale.y = cross.scale.z = 0.45
        cross.color.r, cross.color.g, cross.color.b, cross.color.a = 0.1, 1.0, 0.4, 1.0
        cross.pose.position.z = 0.25

        drag = self._make_draggable_control(ring, "drag_all_actors")
        drag.markers.append(cross)
        im.controls.append(drag)
        for c in self._make_axis_controls():
            im.controls.append(c)

        self.server.insert(im, self._feedback)
        self.menu.apply(self.server, "actors_group")

    def _apply_actors_translation(self, dx, dy, update_poses=True):
        if abs(dx) < 1e-9 and abs(dy) < 1e-9:
            return
        for i, actor in enumerate(self.actors):
            actor["x"] = _f(float(actor.get("x", 0.0)) + dx)
            actor["y"] = _f(float(actor.get("y", 0.0)) + dy)
            if self._is_pedestrian(actor):
                self._ensure_pedestrian_path(actor)
                for wp in actor["waypoints"]:
                    wp["x"] = _f(float(wp["x"]) + dx)
                    wp["y"] = _f(float(wp["y"]) + dy)
                self._sync_walk_goal_from_waypoints(actor)
            if update_poses:
                pose = Pose()
                pose.position.x = float(actor["x"])
                pose.position.y = float(actor["y"])
                pose.position.z = 0.9
                pose.orientation = _yaw_to_quat(float(actor.get("yaw", 0.0)))
                self.server.setPose("actor_%d" % i, pose)
                if self._is_pedestrian(actor):
                    for wi, wp in enumerate(actor["waypoints"]):
                        wpose = Pose()
                        wpose.orientation.w = 1.0
                        wpose.position.x = float(wp["x"])
                        wpose.position.y = float(wp["y"])
                        wpose.position.z = 0.35
                        self.server.setPose("pedwp_%d_%d" % (i, wi), wpose)
        if update_poses:
            self._publish_pedestrian_path_lines()

    def _make_axis_controls(self):
        """Hai trục MOVE_AXIS (X/Y) — dễ kéo hơn MOVE_PLANE thuần."""
        controls = []
        cx = InteractiveMarkerControl()
        cx.name = "move_x"
        cx.orientation = Quaternion(x=1.0, y=0.0, z=0.0, w=1.0)
        cx.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(cx)
        cy = InteractiveMarkerControl()
        cy.name = "move_y"
        cy.orientation = Quaternion(x=0.0, y=0.0, z=1.0, w=1.0)
        cy.interaction_mode = InteractiveMarkerControl.MOVE_AXIS
        controls.append(cy)
        return controls

    def _make_draggable_control(self, marker, name="move_plane"):
        """Visual + MOVE_PLANE cùng một control — bắt buộc để kéo được khối."""
        ctrl = InteractiveMarkerControl()
        ctrl.name = name
        ctrl.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        ctrl.interaction_mode = InteractiveMarkerControl.MOVE_PLANE
        ctrl.always_visible = True
        ctrl.markers.append(marker)
        return ctrl

    def _insert_actor_marker(self, index, actor):
        """Cột người (cam/tím) + mũi tên facing — seed từ sim, kéo trên map thật."""
        name = "actor_%d" % index
        yaw = float(actor.get("yaw", 0.0))
        emotion = str(actor.get("emotion", "Neutral"))
        group = str(actor.get("group") or "")
        is_ped = self._is_pedestrian(actor)
        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = name
        if is_ped:
            nwp = len(actor.get("waypoints") or [])
            im.description = "%s | PEDESTRIAN | %d wp\n(Interact → kéo start; menu → Add waypoint)" % (
                actor.get("name", name),
                nwp,
            )
        else:
            im.description = "%s | %s%s\n(Interact → kéo / xoay)" % (
                actor.get("name", name),
                emotion,
                (" | " + group) if group else "",
            )
        im.pose.position.x = float(actor.get("x", 0.0))
        im.pose.position.y = float(actor.get("y", 0.0))
        im.pose.position.z = 0.9
        im.pose.orientation = _yaw_to_quat(yaw)
        im.scale = 1.0

        # Angry = cam; Neutral = tím nhạt; Pedestrian = lục
        if is_ped:
            cr, cg, cb = 0.05, 0.75, 0.45
        elif emotion.lower() == "angry":
            cr, cg, cb = 1.0, 0.45, 0.05
        else:
            cr, cg, cb = 0.55, 0.25, 0.85

        body = Marker()
        body.type = Marker.CYLINDER
        body.scale.x = body.scale.y = 0.45
        body.scale.z = 1.6
        body.color.r, body.color.g, body.color.b, body.color.a = cr, cg, cb, 0.80

        arrow = Marker()
        arrow.type = Marker.ARROW
        arrow.scale.x = 0.55
        arrow.scale.y = 0.08
        arrow.scale.z = 0.08
        arrow.color.r, arrow.color.g, arrow.color.b, arrow.color.a = 1.0, 1.0, 1.0, 0.95
        arrow.pose.position.z = 0.15

        drag = self._make_draggable_control(body, "drag_actor")
        drag.markers.append(arrow)
        im.controls.append(drag)
        for c in self._make_axis_controls():
            im.controls.append(c)

        # Xoay yaw quanh Z
        rot = InteractiveMarkerControl()
        rot.name = "rotate_z"
        rot.orientation = Quaternion(x=0.0, y=1.0, z=0.0, w=1.0)
        rot.interaction_mode = InteractiveMarkerControl.ROTATE_AXIS
        im.controls.append(rot)

        self.server.insert(im, self._feedback)
        self.menu.apply(self.server, name)

    def _insert_pedestrian_path_markers(self, actor_idx, actor):
        """Mỗi waypoint = cầu kéo thả; điểm cuối = walk_goal."""
        self._ensure_pedestrian_path(actor)
        wps = actor.get("waypoints") or []
        n = len(wps)
        for wi, wp in enumerate(wps):
            name = "pedwp_%d_%d" % (actor_idx, wi)
            im = InteractiveMarker()
            im.header.frame_id = self.frame_id
            im.name = name
            label = "GOAL" if wi == n - 1 else "WP%d" % (wi + 1)
            im.description = "%s | %s\n(Interact → kéo điểm quỹ đạo)" % (
                actor.get("name", "actor_%d" % actor_idx),
                label,
            )
            im.pose.position.x = float(wp["x"])
            im.pose.position.y = float(wp["y"])
            im.pose.position.z = 0.35
            im.pose.orientation.w = 1.0
            im.scale = 0.7

            sph = Marker()
            sph.type = Marker.SPHERE
            sph.scale.x = sph.scale.y = sph.scale.z = 0.38
            if wi == n - 1:
                sph.color.r, sph.color.g, sph.color.b, sph.color.a = 1.0, 0.55, 0.05, 0.95
            else:
                sph.color.r, sph.color.g, sph.color.b, sph.color.a = 0.1, 0.9, 0.55, 0.9

            im.controls.append(self._make_draggable_control(sph, "drag_pedwp"))
            for c in self._make_axis_controls():
                im.controls.append(c)

            self.server.insert(im, self._feedback)
            self.menu_wp.apply(self.server, name)

    def _publish_pedestrian_path_lines(self):
        """LINE_STRIP start→waypoints cho từng pedestrian (MarkerArray latch)."""
        arr = MarkerArray()
        # Clear cũ
        clear = Marker()
        clear.header.frame_id = self.frame_id
        clear.header.stamp = rospy.Time.now()
        clear.ns = "pedestrian_paths"
        clear.id = 0
        clear.action = Marker.DELETEALL
        arr.markers.append(clear)

        mid = 1
        for i, actor in enumerate(self.actors):
            if not self._is_pedestrian(actor):
                continue
            self._ensure_pedestrian_path(actor)
            line = Marker()
            line.header.frame_id = self.frame_id
            line.header.stamp = rospy.Time.now()
            line.ns = "pedestrian_paths"
            line.id = mid
            mid += 1
            line.type = Marker.LINE_STRIP
            line.action = Marker.ADD
            line.scale.x = 0.08
            line.color = ColorRGBA(0.05, 0.85, 0.55, 0.95)
            line.pose.orientation.w = 1.0
            pts = [
                Point(x=float(actor.get("x", 0.0)), y=float(actor.get("y", 0.0)), z=0.25)
            ]
            for wp in actor.get("waypoints") or []:
                pts.append(Point(x=float(wp["x"]), y=float(wp["y"]), z=0.25))
            line.points = pts
            arr.markers.append(line)

            # Số thứ tự nhỏ trên mỗi waypoint
            for wi, wp in enumerate(actor.get("waypoints") or []):
                txt = Marker()
                txt.header.frame_id = self.frame_id
                txt.header.stamp = line.header.stamp
                txt.ns = "pedestrian_paths"
                txt.id = mid
                mid += 1
                txt.type = Marker.TEXT_VIEW_FACING
                txt.action = Marker.ADD
                txt.pose.position.x = float(wp["x"])
                txt.pose.position.y = float(wp["y"])
                txt.pose.position.z = 0.75
                txt.pose.orientation.w = 1.0
                txt.scale.z = 0.28
                txt.color = ColorRGBA(1.0, 1.0, 1.0, 0.95)
                nwp = len(actor.get("waypoints") or [])
                txt.text = "G" if wi == nwp - 1 else str(wi + 1)
                arr.markers.append(txt)

        self.path_pub.publish(arr)

    def _insert_object_marker(self, index, obj):
        name = "object_%d" % index
        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = name
        im.description = "%s | %s\n(Interact tool → kéo)" % (
            obj.get("name", name),
            obj.get("interaction_type", "seeing"),
        )
        im.pose.position.x = float(obj.get("x", 0.0))
        im.pose.position.y = float(obj.get("y", 0.0))
        im.pose.position.z = 0.9
        im.pose.orientation.w = 1.0
        im.scale = 1.0

        box = Marker()
        box.type = Marker.CUBE
        box.scale.x = box.scale.y = 0.55
        box.scale.z = 1.5
        box.color.r, box.color.g, box.color.b, box.color.a = 0.1, 0.45, 0.95, 0.85

        im.controls.append(self._make_draggable_control(box, "drag_object"))
        for c in self._make_axis_controls():
            im.controls.append(c)

        self.server.insert(im, self._feedback)
        self.menu.apply(self.server, name)

    def _insert_zone_markers(self, index, z):
        x0 = float(z["x_min"])
        y0 = float(z["y_min"])
        x1 = float(z["x_max"])
        y1 = float(z["y_max"])
        cx = 0.5 * (x0 + x1)
        cy = 0.5 * (y0 + y1)
        sx = max(0.2, x1 - x0)
        sy = max(0.2, y1 - y0)

        cname = "zone_%d_center" % index
        im = InteractiveMarker()
        im.header.frame_id = self.frame_id
        im.name = cname
        im.description = "%s | %s\n(Interact → kéo tâm)" % (
            z.get("name", cname),
            z.get("emotion", "Neutral"),
        )
        im.pose.position.x = cx
        im.pose.position.y = cy
        im.pose.position.z = 0.9
        im.pose.orientation.w = 1.0
        im.scale = 1.0

        box = Marker()
        box.type = Marker.CUBE
        # Khối kéo nhỏ ở tâm (AABB thật vẫn hiện ở Density_visualization)
        box.scale.x = min(1.2, max(0.6, 0.35 * sx))
        box.scale.y = min(1.2, max(0.6, 0.35 * sy))
        box.scale.z = 1.4
        box.color.r, box.color.g, box.color.b, box.color.a = 0.95, 0.15, 0.15, 0.75

        im.controls.append(self._make_draggable_control(box, "drag_zone"))
        for c in self._make_axis_controls():
            im.controls.append(c)
        self.server.insert(im, self._feedback)
        self.menu.apply(self.server, cname)

        rname = "zone_%d_corner" % index
        rim = InteractiveMarker()
        rim.header.frame_id = self.frame_id
        rim.name = rname
        rim.description = "%s\n(Interact → kéo góc size)" % z.get("name", rname)
        rim.pose.position.x = x1
        rim.pose.position.y = y1
        rim.pose.position.z = 0.5
        rim.pose.orientation.w = 1.0
        rim.scale = 0.8

        sph = Marker()
        sph.type = Marker.SPHERE
        sph.scale.x = sph.scale.y = sph.scale.z = 0.40
        sph.color.r, sph.color.g, sph.color.b, sph.color.a = 1.0, 0.85, 0.1, 1.0

        rim.controls.append(self._make_draggable_control(sph, "drag_corner"))
        for c in self._make_axis_controls():
            rim.controls.append(c)
        self.server.insert(rim, self._feedback)
        self.menu.apply(self.server, rname)

    def _feedback(self, feedback):
        if feedback.event_type not in (
            InteractiveMarkerFeedback.POSE_UPDATE,
            InteractiveMarkerFeedback.MOUSE_UP,
        ):
            return

        name = feedback.marker_name
        x = feedback.pose.position.x
        y = feedback.pose.position.y
        # Không erase/rebuild khi đang kéo — sẽ làm mất tương tác.
        # MOUSE_UP: lưu + reload fusion ngay; POSE: throttle để costmap theo marker.
        do_sync = feedback.event_type == InteractiveMarkerFeedback.MOUSE_UP
        do_live_save = True

        if name.startswith("object_"):
            idx = int(name.split("_")[1])
            if 0 <= idx < len(self.objects):
                self.objects[idx]["x"] = _f(x)
                self.objects[idx]["y"] = _f(y)
                self.objects[idx]["z"] = _f(self.objects[idx].get("z", 0.0))
                self.selected = {"kind": "object", "index": idx}
                rospy.loginfo_throttle(
                    1.0, "object[%d] %s -> (%.2f, %.2f)", idx, self.objects[idx].get("name"), x, y
                )

        elif name.startswith("actor_"):
            idx = int(name.split("_")[1])
            if 0 <= idx < len(self.actors):
                self.actors[idx]["x"] = _f(x)
                self.actors[idx]["y"] = _f(y)
                self.actors[idx]["yaw"] = _f(_quat_to_yaw(feedback.pose.orientation), 3)
                self.selected = {"kind": "actor", "index": idx}
                # Cập nhật tâm cụm khi chỉnh từng người
                if do_sync and self.actors:
                    cx, cy = self._actors_centroid()
                    self._group_last_xy = (cx, cy)
                    gpose = Pose()
                    gpose.orientation.w = 1.0
                    gpose.position.x, gpose.position.y, gpose.position.z = cx, cy, 2.2
                    self.server.setPose("actors_group", gpose)
                    self.server.applyChanges()
                if self._is_pedestrian(self.actors[idx]):
                    self._publish_pedestrian_path_lines()
                rospy.loginfo_throttle(
                    1.0,
                    "actor[%d] %s -> (%.2f, %.2f, yaw=%.2f)",
                    idx,
                    self.actors[idx].get("name"),
                    x,
                    y,
                    self.actors[idx]["yaw"],
                )

        elif name.startswith("pedwp_"):
            parts = name.split("_")
            # pedwp_{actor}_{wp}
            if len(parts) >= 3:
                ai = int(parts[1])
                wi = int(parts[2])
                if 0 <= ai < len(self.actors) and self._is_pedestrian(self.actors[ai]):
                    actor = self.actors[ai]
                    self._ensure_pedestrian_path(actor)
                    if 0 <= wi < len(actor["waypoints"]):
                        actor["waypoints"][wi]["x"] = _f(x)
                        actor["waypoints"][wi]["y"] = _f(y)
                        self._sync_walk_goal_from_waypoints(actor)
                        self.selected = {"kind": "pedwp", "index": ai, "wp": wi}
                        self._publish_pedestrian_path_lines()
                        rospy.loginfo_throttle(
                            1.0,
                            "pedwp actor[%d] wp[%d] %s -> (%.2f, %.2f)",
                            ai,
                            wi,
                            actor.get("name"),
                            x,
                            y,
                        )

        elif name == "actors_group":
            if self._group_last_xy is None:
                self._group_last_xy = (x, y)
            dx = x - self._group_last_xy[0]
            dy = y - self._group_last_xy[1]
            self._group_last_xy = (x, y)
            self._apply_actors_translation(dx, dy, update_poses=True)
            if do_sync:
                self.server.applyChanges()
                rospy.loginfo(
                    "ALL_ACTORS translated; centroid now (%.2f, %.2f)",
                    x,
                    y,
                )
            else:
                rospy.loginfo_throttle(1.0, "ALL_ACTORS drag d=(%.2f, %.2f)", dx, dy)

        elif name.startswith("zone_") and name.endswith("_center"):
            idx = int(name.split("_")[1])
            if 0 <= idx < len(self.zones):
                z = self.zones[idx]
                hx = 0.5 * max(0.2, float(z["x_max"]) - float(z["x_min"]))
                hy = 0.5 * max(0.2, float(z["y_max"]) - float(z["y_min"]))
                z["x_min"] = _f(x - hx)
                z["x_max"] = _f(x + hx)
                z["y_min"] = _f(y - hy)
                z["y_max"] = _f(y + hy)
                self.selected = {"kind": "zone", "index": idx}
                if do_sync:
                    self._sync_zone_pair_poses(idx)
                rospy.loginfo_throttle(
                    1.0, "zone[%d] %s center -> (%.2f, %.2f)", idx, z.get("name"), x, y
                )

        elif name.startswith("zone_") and name.endswith("_corner"):
            idx = int(name.split("_")[1])
            if 0 <= idx < len(self.zones):
                z = self.zones[idx]
                x_min = float(z["x_min"])
                y_min = float(z["y_min"])
                z["x_max"] = _f(max(x_min + 0.2, x))
                z["y_max"] = _f(max(y_min + 0.2, y))
                self.selected = {"kind": "zone", "index": idx}
                if do_sync:
                    # Đổi size → cần rebuild visual khối tâm
                    self._sync_zone_markers(idx)
                    self.server.applyChanges()
                rospy.loginfo_throttle(
                    1.0,
                    "zone[%d] %s size -> [%.2f,%.2f]x[%.2f,%.2f]",
                    idx,
                    z.get("name"),
                    z["x_min"],
                    z["x_max"],
                    z["y_min"],
                    z["y_max"],
                )

        if do_sync:
            # Thả chuột: lưu YAML + export canonical (kịch bản bền vững)
            self._maybe_auto_save(force=True)
            self._maybe_auto_export(force=True)
        elif do_live_save:
            self._dirty = True
            self._maybe_auto_save(force=False)

    def _maybe_auto_save(self, force=False):
        if not self.auto_save:
            return
        now = rospy.get_time()
        min_iv = 0.0 if force else max(0.25, self._auto_save_min_interval)
        if (now - self._last_auto_save) < min_iv:
            return
        self._last_auto_save = now
        ok, msg = self.save_all()
        self._dirty = True
        if ok:
            rospy.loginfo_throttle(2.0, "auto_save OK: %s", msg)
        else:
            rospy.logwarn("auto_save FAIL: %s", msg)

    def _maybe_auto_export(self, force=False):
        """Ghi canonical + matlab + sync layout/exp — lưu kịch bản đầy đủ."""
        if not self.auto_export_canonical:
            return
        now = rospy.get_time()
        min_iv = 0.0 if force else max(1.0, self._auto_export_min_interval)
        if (now - self._last_auto_export) < min_iv:
            return
        self._last_auto_export = now
        ok, msg = self.export_canonical()
        if ok:
            self._dirty = False
            rospy.loginfo_throttle(2.0, "auto_export canonical OK: %s", msg)
        else:
            rospy.logwarn("auto_export canonical FAIL: %s", msg)

    def _timer_auto_persist(self, _event):
        """Định kỳ: nếu còn thay đổi chưa export thì tự lưu kịch bản."""
        if not self._dirty:
            return
        if self.auto_export_canonical:
            self._maybe_auto_export(force=False)
        elif self.auto_save:
            self._maybe_auto_save(force=False)

    def _sync_zone_pair_poses(self, idx):
        """Chỉ cập nhật pose cặp center/corner — không erase (an toàn khi vừa thả chuột)."""
        z = self.zones[idx]
        x0, y0 = float(z["x_min"]), float(z["y_min"])
        x1, y1 = float(z["x_max"]), float(z["y_max"])
        cx, cy = 0.5 * (x0 + x1), 0.5 * (y0 + y1)

        pose_c = Pose()
        pose_c.orientation.w = 1.0
        pose_c.position.x, pose_c.position.y, pose_c.position.z = cx, cy, 0.9
        self.server.setPose("zone_%d_center" % idx, pose_c)

        pose_r = Pose()
        pose_r.orientation.w = 1.0
        pose_r.position.x, pose_r.position.y, pose_r.position.z = x1, y1, 0.5
        self.server.setPose("zone_%d_corner" % idx, pose_r)
        self.server.applyChanges()

    def _sync_zone_markers(self, idx):
        z = self.zones[idx]
        self.server.erase("zone_%d_center" % idx)
        self.server.erase("zone_%d_corner" % idx)
        self._insert_zone_markers(idx, z)

    def clicked_point_cb(self, msg):
        kind = self.selected.get("kind")
        idx = int(self.selected.get("index", -1))
        x, y = float(msg.point.x), float(msg.point.y)
        if kind == "object" and 0 <= idx < len(self.objects):
            self.objects[idx]["x"] = _f(x)
            self.objects[idx]["y"] = _f(y)
            self.server.erase("object_%d" % idx)
            self._insert_object_marker(idx, self.objects[idx])
            self.server.applyChanges()
            rospy.loginfo("clicked_point → object[%d] (%.2f, %.2f)", idx, x, y)
            self._dirty = True
            self._maybe_auto_save(force=True)
            self._maybe_auto_export(force=True)
        elif kind == "actor" and 0 <= idx < len(self.actors):
            self.actors[idx]["x"] = _f(x)
            self.actors[idx]["y"] = _f(y)
            self.server.erase("actor_%d" % idx)
            self._insert_actor_marker(idx, self.actors[idx])
            if self._is_pedestrian(self.actors[idx]):
                self._publish_pedestrian_path_lines()
            self.server.applyChanges()
            rospy.loginfo("clicked_point → actor[%d] (%.2f, %.2f)", idx, x, y)
            self._dirty = True
            self._maybe_auto_save(force=True)
            self._maybe_auto_export(force=True)
        elif kind == "pedwp" and 0 <= idx < len(self.actors):
            actor = self.actors[idx]
            wi = int(self.selected.get("wp", -1))
            self._ensure_pedestrian_path(actor)
            if 0 <= wi < len(actor["waypoints"]):
                actor["waypoints"][wi]["x"] = _f(x)
                actor["waypoints"][wi]["y"] = _f(y)
                self._sync_walk_goal_from_waypoints(actor)
                self._rebuild_markers()
                self.server.applyChanges()
                rospy.loginfo(
                    "clicked_point → pedwp actor[%d] wp[%d] (%.2f, %.2f)", idx, wi, x, y
                )
                self._dirty = True
                self._maybe_auto_save(force=True)
                self._maybe_auto_export(force=True)
        elif kind == "zone" and 0 <= idx < len(self.zones):
            z = self.zones[idx]
            hx = 0.5 * max(0.2, float(z["x_max"]) - float(z["x_min"]))
            hy = 0.5 * max(0.2, float(z["y_max"]) - float(z["y_min"]))
            z["x_min"], z["x_max"] = _f(x - hx), _f(x + hx)
            z["y_min"], z["y_max"] = _f(y - hy), _f(y + hy)
            self._sync_zone_markers(idx)
            self.server.applyChanges()
            rospy.loginfo("clicked_point → zone[%d] center (%.2f, %.2f)", idx, x, y)
            self._dirty = True
            self._maybe_auto_save(force=True)
            self._maybe_auto_export(force=True)
        else:
            rospy.logwarn(
                "clicked_point bo qua: chua chon marker (chuot phai → Select for Publish Point)"
            )

    def _menu_reset_group_handle(self, _feedback):
        if not self.actors:
            return
        cx, cy = self._actors_centroid()
        self._group_last_xy = (cx, cy)
        pose = Pose()
        pose.orientation.w = 1.0
        pose.position.x, pose.position.y, pose.position.z = cx, cy, 2.2
        self.server.setPose("actors_group", pose)
        self.server.applyChanges()
        rospy.loginfo("Group handle recentered at (%.2f, %.2f)", cx, cy)

    def handle_recenter_group(self, _req):
        self._menu_reset_group_handle(None)
        return TriggerResponse(success=True, message="actors_group recentered")

    def _menu_select(self, feedback):
        name = feedback.marker_name
        if name.startswith("object_"):
            self.selected = {"kind": "object", "index": int(name.split("_")[1])}
        elif name.startswith("actor_"):
            self.selected = {"kind": "actor", "index": int(name.split("_")[1])}
        elif name.startswith("pedwp_"):
            parts = name.split("_")
            self.selected = {
                "kind": "pedwp",
                "index": int(parts[1]),
                "wp": int(parts[2]),
            }
        elif name.startswith("zone_"):
            self.selected = {"kind": "zone", "index": int(name.split("_")[1])}
        rospy.loginfo("Selected %s", self.selected)

    def _parse_pedwp_name(self, name):
        parts = str(name).split("_")
        if len(parts) < 3 or parts[0] != "pedwp":
            return None, None
        return int(parts[1]), int(parts[2])

    def _add_waypoint_at(self, actor_idx, after_wp=-1):
        """Thêm điểm quỹ đạo sau after_wp (-1 = cuối)."""
        if not (0 <= actor_idx < len(self.actors)):
            return False
        actor = self.actors[actor_idx]
        if not self._is_pedestrian(actor):
            rospy.logwarn("actor[%d] khong phai pedestrian", actor_idx)
            return False
        self._ensure_pedestrian_path(actor)
        wps = actor["waypoints"]
        if after_wp < 0:
            after_wp = len(wps) - 1
        after_wp = max(0, min(after_wp, len(wps) - 1))
        # Điểm mới: giữa after và next, hoặc kéo dài theo hướng cuối
        p0 = wps[after_wp]
        if after_wp + 1 < len(wps):
            p1 = wps[after_wp + 1]
            nx = _f(0.5 * (float(p0["x"]) + float(p1["x"])))
            ny = _f(0.5 * (float(p0["y"]) + float(p1["y"])))
        else:
            if after_wp > 0:
                prev = wps[after_wp - 1]
                dx = float(p0["x"]) - float(prev["x"])
                dy = float(p0["y"]) - float(prev["y"])
            else:
                dx = float(p0["x"]) - float(actor.get("x", 0.0))
                dy = float(p0["y"]) - float(actor.get("y", 0.0))
            L = math.hypot(dx, dy)
            if L < 1e-3:
                dx, dy = 1.0, 0.0
                L = 1.0
            step = 1.5
            nx = _f(float(p0["x"]) + step * dx / L)
            ny = _f(float(p0["y"]) + step * dy / L)
        wps.insert(after_wp + 1, {"x": nx, "y": ny})
        self._sync_walk_goal_from_waypoints(actor)
        self._rebuild_markers()
        self.server.applyChanges()
        self._dirty = True
        self._maybe_auto_save(force=True)
        self._maybe_auto_export(force=True)
        rospy.loginfo(
            "Added waypoint after #%d for %s -> (%.2f, %.2f) | n=%d",
            after_wp + 1,
            actor.get("name"),
            nx,
            ny,
            len(wps),
        )
        return True

    def _menu_add_waypoint(self, feedback):
        name = feedback.marker_name
        if name.startswith("actor_"):
            idx = int(name.split("_")[1])
            self._add_waypoint_at(idx, after_wp=-1)
        elif name.startswith("pedwp_"):
            ai, wi = self._parse_pedwp_name(name)
            if ai is not None:
                self._add_waypoint_at(ai, after_wp=wi)

    def _menu_add_waypoint_after(self, feedback):
        ai, wi = self._parse_pedwp_name(feedback.marker_name)
        if ai is not None:
            self._add_waypoint_at(ai, after_wp=wi)

    def _menu_densify_path(self, feedback):
        name = feedback.marker_name
        ai = None
        if name.startswith("actor_"):
            ai = int(name.split("_")[1])
        elif name.startswith("pedwp_"):
            ai, _ = self._parse_pedwp_name(name)
        if ai is None or not (0 <= ai < len(self.actors)):
            return
        actor = self.actors[ai]
        if not self._densify_pedestrian_path(actor):
            return
        self._rebuild_markers()
        self.server.applyChanges()
        self._dirty = True
        self._maybe_auto_save(force=True)
        self._maybe_auto_export(force=True)
        rospy.loginfo(
            "Densified path %s → %d waypoints (kéo lệch các điểm giữa để tạo đường cong)",
            actor.get("name"),
            len(actor.get("waypoints") or []),
        )

    def _menu_delete_waypoint(self, feedback):
        ai, wi = self._parse_pedwp_name(feedback.marker_name)
        if ai is None or not (0 <= ai < len(self.actors)):
            return
        actor = self.actors[ai]
        if not self._is_pedestrian(actor):
            return
        self._ensure_pedestrian_path(actor)
        if len(actor["waypoints"]) <= 1:
            rospy.logwarn("Giữ ít nhất 1 waypoint (walk_goal) cho %s", actor.get("name"))
            return
        if not (0 <= wi < len(actor["waypoints"])):
            return
        del actor["waypoints"][wi]
        self._sync_walk_goal_from_waypoints(actor)
        self._rebuild_markers()
        self.server.applyChanges()
        self._dirty = True
        self._maybe_auto_save(force=True)
        self._maybe_auto_export(force=True)
        rospy.loginfo("Deleted waypoint #%d of %s", wi + 1, actor.get("name"))

    def _menu_save(self, _feedback):
        ok, msg = self.save_all()
        rospy.loginfo("%s: %s", "OK" if ok else "FAIL", msg)

    def _menu_export_canonical(self, _feedback):
        ok, msg = self.export_canonical()
        rospy.loginfo("export_canonical %s: %s", "OK" if ok else "FAIL", msg)

    def handle_save(self, _req):
        ok, msg = self.save_all()
        return TriggerResponse(success=ok, message=msg)

    def handle_export_canonical(self, _req):
        ok, msg = self.export_canonical()
        return TriggerResponse(success=ok, message=msg)

    def export_canonical(self):
        """Lưu layout đang chỉnh → canonical + matlab yaml (+ sync + pack)."""
        ok, msg = self.save_all()
        if not ok:
            return False, "save before export failed: %s" % msg

        source = {
            "actors": copy.deepcopy(self.actors),
            "objects": copy.deepcopy(self.objects),
            "emotion_zones": copy.deepcopy(self.zones),
            "groups": copy.deepcopy(self.cfg.get("groups") or []),
            "interactions": copy.deepcopy(self.cfg.get("interactions") or []),
            "density": copy.deepcopy(self.cfg.get("density") or {}),
        }
        # groups/interactions có thể chỉ nằm trên file layout / canonical
        if not source["groups"] or not source["interactions"]:
            for path in (self.actor_layout_path, self.canonical_path, self.config_path):
                if not path or not os.path.isfile(path):
                    continue
                extra = self._load_yaml(path)
                if not source["groups"] and extra.get("groups"):
                    source["groups"] = copy.deepcopy(extra["groups"])
                if not source["interactions"] and extra.get("interactions"):
                    source["interactions"] = copy.deepcopy(extra["interactions"])

        pack_dir = Path(self.matlab_pack_dir) if self.matlab_pack_dir else None
        try:
            _, written = export_scenario(
                source,
                canonical_path=Path(self.canonical_path),
                matlab_path=Path(self.matlab_yaml_path),
                base_canonical_path=Path(self.canonical_path),
                sync_layout=self.sync_layout_on_export,
                sync_exp_configs=self.sync_exp_on_export,
                actor_layout_path=Path(self.actor_layout_path),
                exp5_path=Path(self.exp5_path),
                exp6_path=Path(self.exp6_path),
                matlab_pack_dir=pack_dir,
                exp_variant=self.export_exp_variant,
            )
        except Exception as exc:
            rospy.logerr("export_canonical failed: %s", exc)
            return False, str(exc)

        msg = "exported: " + "; ".join(written)
        rospy.loginfo("%s", msg)
        return True, msg

    def handle_reload_file(self, _req):
        try:
            self.cfg = self._load_yaml(self.config_path)
            self.objects = list(self.cfg.get("objects") or [])
            self.zones = list(self.cfg.get("emotion_zones") or [])
            self.actors = list(self.cfg.get("actors") or [])
            for actor in self.actors:
                self._ensure_pedestrian_path(actor)
            self._rebuild_markers()
            self.server.applyChanges()
            return TriggerResponse(success=True, message="Reloaded markers from YAML")
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def save_all(self):
        paths = [self.config_path] + self.also_update_paths
        errors = []
        for path in paths:
            try:
                self._write_yaml(path)
            except Exception as exc:
                errors.append("%s: %s" % (path, exc))
        if errors:
            return False, "; ".join(errors)

        parts = ["Saved %s" % ", ".join(paths)]
        if self.reload_fusion_on_save:
            parts.append(self._try_reload_fusion())
        parts.append(self._try_reload_gt())
        return True, " | ".join(parts)

    def _try_reload_gt(self):
        if not self.gt_reload_service:
            return "skip gt reload"
        try:
            # GT đọc cùng file layout đang chỉnh
            rospy.set_param("/scenario_gt_social_state_publisher/scenario_path", self.config_path)
            rospy.wait_for_service(self.gt_reload_service, timeout=1.0)
            call = rospy.ServiceProxy(self.gt_reload_service, Trigger)
            resp = call()
            return "gt reload: %s" % resp.message
        except Exception as exc:
            return "gt reload skip (%s)" % exc

    def _write_yaml(self, path):
        for actor in self.actors:
            self._ensure_pedestrian_path(actor)
        try:
            data = self._load_yaml(path) if os.path.isfile(path) else dict(self.cfg)
        except Exception as exc:
            rospy.logwarn("YAML corrupt (%s), rewrite from editor state: %s", path, exc)
            data = dict(self.cfg)
        data["objects"] = copy.deepcopy(self.objects)
        data["emotion_zones"] = copy.deepcopy(self.zones)
        data["actors"] = copy.deepcopy(self.actors)

        header = (
            "# Auto-updated by social_map_annotation_editor (RViz InteractiveMarkers).\n"
            "# actors + objects + emotion_zones synced; other keys preserved.\n"
        )
        if os.path.isfile(path):
            with open(path, "r", encoding="utf-8") as f:
                first = f.readline()
            if first.startswith("# Track D"):
                with open(path, "r", encoding="utf-8") as f:
                    lines = f.readlines()
                keep = []
                for line in lines:
                    if line.startswith("#") or line.strip() == "":
                        keep.append(line)
                        if line.strip() == "" and keep:
                            break
                    else:
                        break
                if keep:
                    header = "".join(keep)
                    if not header.endswith("\n"):
                        header += "\n"

        body = yaml.safe_dump(
            data,
            default_flow_style=False,
            allow_unicode=True,
            sort_keys=False,
        )
        with open(path, "w", encoding="utf-8") as f:
            f.write(header)
            if not header.endswith("\n"):
                f.write("\n")
            f.write(body)
        rospy.loginfo("Wrote %s", path)

    def _try_reload_fusion(self):
        if not self.fusion_reload_service:
            return "skip fusion reload"
        try:
            rospy.wait_for_service(self.fusion_reload_service, timeout=3.0)
            call = rospy.ServiceProxy(self.fusion_reload_service, Trigger)
            resp = call()
            return "fusion reload: %s" % resp.message
        except Exception as exc:
            rospy.logwarn("fusion reload FAIL: %s", exc)
            return "fusion reload skip (%s)" % exc


def main():
    SocialMapAnnotationEditor()
    rospy.spin()


if __name__ == "__main__":
    main()
