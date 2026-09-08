#!/usr/bin/env python3

import math
import os
import yaml

import rospy
from social_density_fusion.msg import DensityInfo
from gazebo_msgs.msg import ModelStates
from geometry_msgs.msg import Point, PolygonStamped, PoseWithCovarianceStamped
from social_msgs.msg import (
    SocialGroup,
    SocialGroups,
    SocialInteraction,
    SocialInteractions,
    SocialPeople,
    SocialPerson,
    SocialState,
)
from std_srvs.srv import Trigger, TriggerResponse
from visualization_msgs.msg import Marker, MarkerArray


class KalmanFilter:
    def __init__(self, process_noise=0.1, measurement_noise=0.1):
        self.process_noise = process_noise
        self.measurement_noise = measurement_noise
        self.velocity = 0.0
        self.error = 1.0

    def update(self, measurement, dt):
        predicted_velocity = self.velocity
        predicted_error = self.error + self.process_noise * dt
        kalman_gain = predicted_error / (predicted_error + self.measurement_noise)
        self.velocity = predicted_velocity + kalman_gain * (measurement - predicted_velocity)
        self.error = (1 - kalman_gain) * predicted_error
        return self.velocity


class SocialDensityFusionNode:
    def __init__(self):
        rospy.init_node("social_density_fusion_node", anonymous=True)

        self.process_noise = rospy.get_param("~process_noise", 0.1)
        self.measurement_noise = rospy.get_param("~measurement_noise", 0.1)
        # High-speed navigation needs higher refresh to reduce social-state latency.
        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.fusion_rate = float(rospy.get_param("~fusion_rate", 20.0))
        self.fusion_on_lidar_callback = bool(rospy.get_param("~fusion_on_lidar_callback", True))
        self.max_data_age_sec = float(rospy.get_param("~max_data_age_sec", 0.25))
        # Allow small tolerance around lidar polygon boundary (meters).
        self.polygon_boundary_margin = float(rospy.get_param("~polygon_boundary_margin", 0.25))

        # Actor có vx,vy,vz ước lượng từ pose + Kalman; các actor khác gán vận tốc 0.
        if rospy.has_param("~kalman_velocity_actors"):
            self.kalman_velocity_actors = frozenset(rospy.get_param("~kalman_velocity_actors"))
        else:
            env_actors = os.environ.get("SOCIAL_DENSITY_KALMAN_VELOCITY_ACTORS", "").strip()
            if env_actors:
                self.kalman_velocity_actors = frozenset(
                    x.strip() for x in env_actors.split(",") if x.strip()
                )
            else:
                self.kalman_velocity_actors = frozenset(
                    ["actor30", "actor31", "actor32", "actor33"]
                )

        self.config_path = rospy.get_param("~config_path", "")
        self.auto_reload_config = rospy.get_param("~auto_reload_config", False)
        self.config_reload_interval_sec = float(rospy.get_param("~config_reload_interval_sec", 1.0))
        self.last_config_mtime = None
        self.lidar_polygon_topic = rospy.get_param("~lidar_polygon_topic", "/lidar_map_polygon")

        self.raw_model_states = None
        self.raw_model_states_recv_time = None
        self.lidar_polygon_xy = []
        self.lidar_polygon_recv_time = None
        self.lidar_polygon_bbox = None
        self.lidar_polygon_area = 0.0
        self.robot_pose = None

        self.actor_data = {}
        self.object_data = {}
        self.previous_actor_data = {}
        self.velocity_filters = {}
        self.currentActor_detected = []
        self.currentObject_detected = []
        self.current_density = 0.0
        self.current_people_count = 0

        self.groups = {}
        self.interactions = {}
        self.interaction_types = {}
        self.config_actor_emotions = {}
        self.config_density_publish_enabled = False

        self.state_pub = rospy.Publisher("/social_state", SocialState, queue_size=10)
        self.density_info_pub = rospy.Publisher("/density_info", DensityInfo, queue_size=10)
        self.marker_pub = rospy.Publisher("/density_visualization", MarkerArray, queue_size=10)

        # queue_size=1 keeps only the latest frame and prevents lag accumulation.
        rospy.Subscriber(
            "/gazebo/model_states",
            ModelStates,
            self.raw_model_states_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            self.lidar_polygon_topic,
            PolygonStamped,
            self.lidar_polygon_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        rospy.Subscriber(
            "/amcl_pose",
            PoseWithCovarianceStamped,
            self.amcl_callback,
            queue_size=1,
            tcp_nodelay=True,
        )

        self.load_scenario_config()
        if self.config_path:
            ok, msg = self.load_scenario_config_from_file()
            if ok:
                rospy.loginfo(msg)
            else:
                rospy.logwarn("Khong the nap config: %s", msg)

        self.reload_service = rospy.Service("~reload_config", Trigger, self.handle_reload_config)
        self.fusion_timer = rospy.Timer(rospy.Duration(1.0 / self.fusion_rate), self.update_fused_model_states)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_social_state)

        if self.auto_reload_config:
            self.config_watch_timer = rospy.Timer(
                rospy.Duration(max(0.2, self.config_reload_interval_sec)), self.check_config_changes
            )

    @staticmethod
    def canonical_actor_name(name):
        if name.startswith("actor") and name.endswith("_collision"):
            return name[: -len("_collision")]
        return name

    def raw_model_states_callback(self, msg):
        self.raw_model_states = msg
        self.raw_model_states_recv_time = rospy.Time.now()

    def lidar_polygon_callback(self, msg):
        if msg.header.frame_id != "map":
            rospy.logwarn_throttle(
                10.0,
                "lidar_polygon frame_id=%s (expected map); vẫn dùng điểm nhận được.",
                msg.header.frame_id,
            )
        self.lidar_polygon_xy = [(float(p.x), float(p.y)) for p in msg.polygon.points]
        self.lidar_polygon_recv_time = rospy.Time.now()
        if len(self.lidar_polygon_xy) >= 3:
            xs = [p[0] for p in self.lidar_polygon_xy]
            ys = [p[1] for p in self.lidar_polygon_xy]
            self.lidar_polygon_bbox = (min(xs), max(xs), min(ys), max(ys))
            self.lidar_polygon_area = self.calculate_polygon_area(self.lidar_polygon_xy)
        else:
            self.lidar_polygon_bbox = None
            self.lidar_polygon_area = 0.0

        # Process immediately on fresh lidar polygon to reduce timer-quantization latency.
        if self.fusion_on_lidar_callback:
            self.update_fused_model_states(None)

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def load_scenario_config(self, config_data=None):
        if isinstance(config_data, dict):
            people_cfg = config_data.get("people", [])
            groups_cfg = config_data.get("groups", {})
            interactions_cfg = config_data.get("interactions", {})
            actors_cfg = config_data.get("actors", {})
            density_cfg = config_data.get("density", {})
        else:
            people_cfg = rospy.get_param("~people", [])
            groups_cfg = rospy.get_param("~groups", {})
            interactions_cfg = rospy.get_param("~interactions", {})
            actors_cfg = rospy.get_param("~actors", {})
            density_cfg = rospy.get_param("~density", {})

        emotions = {}
        if isinstance(people_cfg, list) and people_cfg:
            for p in people_cfg:
                if isinstance(p, dict) and p.get("name"):
                    emotions[self.canonical_actor_name(str(p["name"]))] = str(p.get("emotion", "Neutral"))
        else:
            for actor, info in actors_cfg.items():
                cname = self.canonical_actor_name(str(actor))
                emotions[cname] = str(info.get("emotion", "Neutral")) if isinstance(info, dict) else str(info)
        self.config_actor_emotions = emotions

        groups = {}
        if isinstance(groups_cfg, list):
            for g in groups_cfg:
                if not isinstance(g, dict) or not g.get("group_name"):
                    continue
                members = []
                for m in g.get("members", []):
                    cm = self.canonical_actor_name(str(m))
                    if cm not in members:
                        members.append(cm)
                groups[str(g["group_name"])] = members
        else:
            for gname, members in groups_cfg.items():
                groups[str(gname)] = []
                for m in members:
                    cm = self.canonical_actor_name(str(m))
                    if cm not in groups[str(gname)]:
                        groups[str(gname)].append(cm)
        self.groups = groups

        interactions = {}
        interaction_types = {}
        if isinstance(interactions_cfg, list):
            for it in interactions_cfg:
                if not isinstance(it, dict) or not it.get("object_name"):
                    continue
                obj = str(it["object_name"])
                parts = []
                for p in it.get("participants", []):
                    cp = self.canonical_actor_name(str(p))
                    if cp not in parts:
                        parts.append(cp)
                interactions[obj] = parts
                interaction_types[obj] = str(it.get("interaction_type", "default"))
        else:
            for obj, info in interactions_cfg.items():
                participants = []
                itype = "default"
                if isinstance(info, dict):
                    participants = info.get("participants", [])
                    itype = str(info.get("interaction_type", "default"))
                elif isinstance(info, list):
                    participants = info
                parts = []
                for p in participants:
                    cp = self.canonical_actor_name(str(p))
                    if cp not in parts:
                        parts.append(cp)
                interactions[str(obj)] = parts
                interaction_types[str(obj)] = itype
        self.interactions = interactions
        self.interaction_types = interaction_types

        if isinstance(density_cfg, dict):
            self.config_density_publish_enabled = bool(density_cfg.get("button_state", False))
        elif isinstance(density_cfg, bool):
            self.config_density_publish_enabled = density_cfg

    def load_scenario_config_from_file(self):
        if not self.config_path:
            return False, "config_path rong"
        if not os.path.isfile(self.config_path):
            return False, f"Khong tim thay file {self.config_path}"
        try:
            with open(self.config_path, "r", encoding="utf-8") as f:
                data = yaml.safe_load(f) or {}
            if not isinstance(data, dict):
                return False, "Noi dung YAML phai la map"
            self.load_scenario_config(data)
            self.last_config_mtime = os.path.getmtime(self.config_path)
            return True, f"Da nap config {self.config_path}"
        except Exception as exc:
            return False, f"Loi doc YAML: {exc}"

    def handle_reload_config(self, _req):
        ok, msg = self.load_scenario_config_from_file()
        return TriggerResponse(success=ok, message=msg)

    def check_config_changes(self, _event):
        if not self.config_path or not os.path.isfile(self.config_path):
            return
        try:
            current_mtime = os.path.getmtime(self.config_path)
        except OSError:
            return
        if self.last_config_mtime is None:
            self.last_config_mtime = current_mtime
            return
        if current_mtime > self.last_config_mtime:
            ok, msg = self.load_scenario_config_from_file()
            if ok:
                rospy.loginfo("Auto reload config: %s", msg)
            else:
                rospy.logwarn("Auto reload that bai: %s", msg)

    @staticmethod
    def calculate_polygon_area(points):
        if len(points) < 3:
            return 0.0
        area = 0.0
        for i in range(len(points)):
            j = (i + 1) % len(points)
            area += points[i][0] * points[j][1]
            area -= points[j][0] * points[i][1]
        return abs(area) * 0.5

    @staticmethod
    def is_point_inside_polygon(x, y, polygon):
        if len(polygon) < 3:
            return False
        inside = False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / ((yj - yi) + 1e-12) + xi):
                inside = not inside
            j = i
        return inside

    @staticmethod
    def point_to_segment_distance(x, y, x1, y1, x2, y2):
        dx = x2 - x1
        dy = y2 - y1
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-12:
            return math.hypot(x - x1, y - y1)
        t = ((x - x1) * dx + (y - y1) * dy) / seg_len_sq
        t = max(0.0, min(1.0, t))
        proj_x = x1 + t * dx
        proj_y = y1 + t * dy
        return math.hypot(x - proj_x, y - proj_y)

    @classmethod
    def is_point_near_polygon_boundary(cls, x, y, polygon, margin):
        if margin <= 0.0 or len(polygon) < 2:
            return False
        j = len(polygon) - 1
        for i in range(len(polygon)):
            xi, yi = polygon[i]
            xj, yj = polygon[j]
            if cls.point_to_segment_distance(x, y, xi, yi, xj, yj) <= margin:
                return True
            j = i
        return False

    def publish_density_visualization(self, lidar_polygon, density, people_count):
        marker_array = MarkerArray()
        now = rospy.Time.now()

        lidar_marker = Marker()
        lidar_marker.header.frame_id = "map"
        lidar_marker.header.stamp = now
        lidar_marker.ns = "lidar_area"
        lidar_marker.id = 0
        lidar_marker.type = Marker.LINE_STRIP
        lidar_marker.action = Marker.ADD
        lidar_marker.scale.x = 0.1
        lidar_marker.color.g = 1.0
        lidar_marker.color.a = 1.0
        lidar_marker.pose.orientation.w = 1.0
        for x, y in lidar_polygon:
            p = Point()
            p.x = x
            p.y = y
            lidar_marker.points.append(p)
        if lidar_marker.points:
            lidar_marker.points.append(lidar_marker.points[0])
            marker_array.markers.append(lidar_marker)

        if self.config_density_publish_enabled:
            text_marker = Marker()
            text_marker.header.frame_id = "map"
            text_marker.header.stamp = now
            text_marker.ns = "density_text"
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.5
            # xanh dương đậm
            text_marker.color.r = 0.0
            text_marker.color.g = 0.12
            text_marker.color.b = 0.45
            text_marker.color.a = 1.0
            text_marker.pose.orientation.w = 1.0
            yaw = 2.0 * math.atan2(self.robot_pose.orientation.z, self.robot_pose.orientation.w)
            back_dx = -1.0 * math.cos(yaw)
            back_dy = -1.0 * math.sin(yaw)
            text_marker.pose.position.x = self.robot_pose.position.x + back_dx
            text_marker.pose.position.y = self.robot_pose.position.y + back_dy
            text_marker.pose.position.z = 1.0
            text_marker.text = f"{people_count} persons\n{density:.4f} people/m²"
            marker_array.markers.append(text_marker)
        else:
            clear_text = Marker()
            clear_text.header.frame_id = "map"
            clear_text.header.stamp = now
            clear_text.ns = "density_text"
            clear_text.id = 1
            clear_text.action = Marker.DELETE
            marker_array.markers.append(clear_text)

        self.marker_pub.publish(marker_array)

    def update_fused_model_states(self, _event):
        if self.raw_model_states is None or self.robot_pose is None:
            return
        now = rospy.Time.now()
        if self.raw_model_states_recv_time is None or self.lidar_polygon_recv_time is None:
            return
        model_age = (now - self.raw_model_states_recv_time).to_sec()
        poly_age = (now - self.lidar_polygon_recv_time).to_sec()
        if model_age > self.max_data_age_sec or poly_age > self.max_data_age_sec:
            rospy.logwarn_throttle(
                2.0,
                "Du lieu stale: model_age=%.3fs, polygon_age=%.3fs (nguong=%.3fs)",
                model_age,
                poly_age,
                self.max_data_age_sec,
            )
            return
        lidar_polygon = self.lidar_polygon_xy
        if len(lidar_polygon) < 3:
            return
        if self.lidar_polygon_bbox is None:
            return
        min_x, max_x, min_y, max_y = self.lidar_polygon_bbox
        margin = max(0.0, self.polygon_boundary_margin)
        expanded_min_x = min_x - margin
        expanded_max_x = max_x + margin
        expanded_min_y = min_y - margin
        expanded_max_y = max_y + margin

        actor_indices = {}
        object_indices = []
        people_points = []

        for i, raw_name in enumerate(self.raw_model_states.name):
            pose = self.raw_model_states.pose[i]
            x = pose.position.x
            y = pose.position.y
            canonical = self.canonical_actor_name(raw_name)
            # Actor đi bộ (kalman_velocity_actors): luôn đưa vào social_state/costmap,
            # không phụ thuộc đa giác LiDAR; mật độ chỉ tính actor trong vùng quét.
            skip_lidar_polygon = (
                canonical.startswith("actor") and canonical in self.kalman_velocity_actors
            )
            if not skip_lidar_polygon:
                if x < expanded_min_x or x > expanded_max_x or y < expanded_min_y or y > expanded_max_y:
                    continue
                if not self.is_point_inside_polygon(x, y, lidar_polygon):
                    if not self.is_point_near_polygon_boundary(x, y, lidar_polygon, margin):
                        continue
            if canonical.startswith("actor"):
                if margin > 0.0 and (x < min_x or x > max_x or y < min_y or y > max_y):
                    rospy.loginfo_throttle(
                        2.0,
                        "Actor sat bien lidar duoc giu lai boi polygon_boundary_margin=%.2fm",
                        margin,
                    )
            if canonical.startswith("actor"):
                # Always prefer actor*_collision over actor* when both exist,
                # so proxemic center aligns with lidar-detected obstacle center.
                if canonical in actor_indices:
                    prev_name = self.raw_model_states.name[actor_indices[canonical]]
                    prev_is_collision = prev_name.endswith("_collision")
                    curr_is_collision = raw_name.endswith("_collision")
                    if (not prev_is_collision) and curr_is_collision:
                        actor_indices[canonical] = i
                else:
                    actor_indices[canonical] = i
            else:
                object_indices.append((raw_name, i))

        for actor_name, idx in actor_indices.items():
            pose = self.raw_model_states.pose[idx]
            if actor_name not in self.kalman_velocity_actors:
                people_points.append((pose.position.x, pose.position.y))
            else:
                x = pose.position.x
                y = pose.position.y
                if self.is_point_inside_polygon(x, y, lidar_polygon) or self.is_point_near_polygon_boundary(
                    x, y, lidar_polygon, margin
                ):
                    people_points.append((x, y))

        available_area = self.lidar_polygon_area
        people_count = len(people_points)
        density = (float(people_count) / available_area) if available_area > 0 else 0.0
        if self.config_density_publish_enabled:
            density_msg = DensityInfo()
            density_msg.robot_x = self.robot_pose.position.x
            density_msg.robot_y = self.robot_pose.position.y
            density_msg.people_count = people_count
            density_msg.density = density
            self.density_info_pub.publish(density_msg)
            self.current_density = density
            self.current_people_count = people_count
        else:
            self.current_density = 0.0
            self.current_people_count = 0

        self.publish_density_visualization(lidar_polygon, density, people_count)
        self.update_dynamic_state(actor_indices, object_indices)

    def update_dynamic_state(self, actor_indices, object_indices):
        current_time = rospy.Time.now().to_sec()
        self.currentActor_detected = list(actor_indices.keys())
        self.currentObject_detected = [name for name, _ in object_indices]

        self.actor_data = {}
        for name, idx in actor_indices.items():
            pose = self.raw_model_states.pose[idx]
            if name in self.kalman_velocity_actors:
                if name not in self.velocity_filters:
                    self.velocity_filters[name] = {
                        "x": KalmanFilter(self.process_noise, self.measurement_noise),
                        "y": KalmanFilter(self.process_noise, self.measurement_noise),
                        "z": KalmanFilter(self.process_noise, self.measurement_noise),
                    }
                if name in self.previous_actor_data:
                    dt = current_time - self.previous_actor_data[name]["time"]
                    if dt > 1e-3:
                        inst_vx = (pose.position.x - self.previous_actor_data[name]["x"]) / dt
                        inst_vy = (pose.position.y - self.previous_actor_data[name]["y"]) / dt
                        inst_vz = (pose.position.z - self.previous_actor_data[name]["z"]) / dt
                        vx = self.velocity_filters[name]["x"].update(inst_vx, dt)
                        vy = self.velocity_filters[name]["y"].update(inst_vy, dt)
                        vz = self.velocity_filters[name]["z"].update(inst_vz, dt)
                    else:
                        vx = self.velocity_filters[name]["x"].velocity
                        vy = self.velocity_filters[name]["y"].velocity
                        vz = self.velocity_filters[name]["z"].velocity
                else:
                    vx = vy = vz = 0.0
                self.previous_actor_data[name] = {
                    "x": pose.position.x,
                    "y": pose.position.y,
                    "z": pose.position.z,
                    "time": current_time,
                }
            else:
                vx = vy = vz = 0.0

            self.actor_data[name] = {
                "x": round(pose.position.x, 2),
                "y": round(pose.position.y, 2),
                "z": round(pose.position.z, 2),
                "vx": round(vx, 2),
                "vy": round(vy, 2),
                "vz": round(vz, 2),
            }

        self.object_data = {}
        for name, idx in object_indices:
            pose = self.raw_model_states.pose[idx]
            twist = self.raw_model_states.twist[idx]
            self.object_data[name] = {
                "x": round(pose.position.x, 2),
                "y": round(pose.position.y, 2),
                "z": round(pose.position.z, 2),
                "vx": round(twist.linear.x, 2),
                "vy": round(twist.linear.y, 2),
                "vz": round(twist.linear.z, 2),
            }

    def publish_social_state(self, _event):
        state_msg = SocialState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = "map"
        state_msg.density = self.current_density

        people_msg = SocialPeople()
        people_msg.header = state_msg.header
        for actor_name, data in self.actor_data.items():
            person = SocialPerson()
            person.name = actor_name
            person.reliability = 1.0
            person.emotion = self.config_actor_emotions.get(actor_name, "Neutral")
            person.position.position.x = data["x"]
            person.position.position.y = data["y"]
            person.position.position.z = data["z"]
            person.velocity.linear.x = data["vx"]
            person.velocity.linear.y = data["vy"]
            person.velocity.linear.z = data["vz"]
            people_msg.people.append(person)
        state_msg.people = people_msg

        groups_msg = SocialGroups()
        groups_msg.header = state_msg.header
        for group_name, members in self.groups.items():
            member_msgs = []
            for actor_name in members:
                if actor_name not in self.actor_data:
                    continue
                data = self.actor_data[actor_name]
                m = SocialPerson()
                m.name = actor_name
                m.reliability = 1.0
                m.emotion = self.config_actor_emotions.get(actor_name, "Neutral")
                m.position.position.x = data["x"]
                m.position.position.y = data["y"]
                m.position.position.z = data["z"]
                m.velocity.linear.x = data["vx"]
                m.velocity.linear.y = data["vy"]
                m.velocity.linear.z = data["vz"]
                member_msgs.append(m)
            if not member_msgs:
                continue
            group_msg = SocialGroup()
            group_msg.group_name = group_name
            group_msg.members = member_msgs
            groups_msg.groups.append(group_msg)
        state_msg.groups = groups_msg

        interactions_msg = SocialInteractions()
        interactions_msg.header = state_msg.header
        for object_name, participants in self.interactions.items():
            if object_name not in self.object_data:
                continue
            participant_msgs = []
            for actor_name in participants:
                if actor_name not in self.actor_data:
                    continue
                data = self.actor_data[actor_name]
                part = SocialPerson()
                part.name = actor_name
                part.position.position.x = data["x"]
                part.position.position.y = data["y"]
                part.position.position.z = data["z"]
                part.velocity.linear.x = data["vx"]
                part.velocity.linear.y = data["vy"]
                part.velocity.linear.z = data["vz"]
                part.emotion = self.config_actor_emotions.get(actor_name, "Neutral")
                participant_msgs.append(part)
            if not participant_msgs:
                continue
            object_data = self.object_data[object_name]
            interaction = SocialInteraction()
            interaction.object_name = object_name
            interaction.object_position.position.x = object_data["x"]
            interaction.object_position.position.y = object_data["y"]
            interaction.object_position.position.z = object_data["z"]
            interaction.object_velocity.linear.x = object_data["vx"]
            interaction.object_velocity.linear.y = object_data["vy"]
            interaction.object_velocity.linear.z = object_data["vz"]
            interaction.reliability = 1.0
            interaction.participants = participant_msgs
            interaction.interaction_type = self.interaction_types.get(object_name, "default")
            interaction.timestamp = rospy.Time.now()
            interactions_msg.interactions.append(interaction)
        state_msg.interactions = interactions_msg

        self.state_pub.publish(state_msg)


if __name__ == "__main__":
    SocialDensityFusionNode()
    rospy.spin()
