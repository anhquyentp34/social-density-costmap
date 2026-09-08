#!/usr/bin/env python3
"""Social density fusion cho robot thật: nguồn người từ zed_sdk_bridge (/bodies).

Bổ sung:
- Ước lượng facing_orientation từ keypoints skeleton (tách khỏi motion_heading).
- Bù vận tốc camera-relative → map bằng robot odometry (không chỉ xoay vector).
- Phát hiện nhóm (F-formation) và tương tác người–object.
- Diagnostic /zed_velocity_compensation/debug và /pedestrian_measurements.
"""

import math
import os
import sys
import yaml

import rospy
import tf2_ros
from geometry_msgs.msg import Point, PolygonStamped, PoseWithCovarianceStamped, Quaternion
from social_density_fusion.msg import (
    DensityInfo,
    PedestrianMeasurement,
    PedestrianMeasurementArray,
    PersonVelocityDebug,
    PersonVelocityDebugArray,
    VelocityCompensationDebug,
    VelocityCompensationDebugArray,
)
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
from zed_sdk_bridge.msg import BodiesStamped, ObjectsStamped

# Same-directory helper (installed next to this script)
sys.path.insert(0, os.path.dirname(os.path.abspath(__file__)))
from velocity_compensation import VelocityCompensator  # noqa: E402


# BODY_18 (COCO): nose=0, neck=1, R_shoulder=2, L_shoulder=5
BODY18_NOSE, BODY18_NECK = 0, 1
BODY18_R_SHOULDER, BODY18_L_SHOULDER = 2, 5

# BODY_34 (ZED): L_shoulder=5, R_shoulder=12, head=26, nose=27, chest=2
BODY34_CHEST, BODY34_L_SHOULDER, BODY34_R_SHOULDER = 2, 5, 12
BODY34_HEAD, BODY34_NOSE = 26, 27


def rotate_point_by_quaternion(x, y, z, qx, qy, qz, qw):
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)

    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def yaw_to_quaternion(yaw):
    q = Quaternion()
    q.z = math.sin(yaw * 0.5)
    q.w = math.cos(yaw * 0.5)
    return q


def angle_diff(a, b):
    """Chênh lệch góc trong [-pi, pi]."""
    d = (a - b + math.pi) % (2.0 * math.pi) - math.pi
    return d


def is_valid_kp(keypoints_3d, idx):
    if idx * 3 + 2 >= len(keypoints_3d):
        return False
    x = float(keypoints_3d[idx * 3])
    y = float(keypoints_3d[idx * 3 + 1])
    z = float(keypoints_3d[idx * 3 + 2])
    if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
        return False
    if abs(x) < 1e-6 and abs(y) < 1e-6 and abs(z) < 1e-6:
        return False
    if abs(x) > 100.0 or abs(y) > 100.0 or abs(z) > 100.0:
        return False
    return True


def kp_xyz(keypoints_3d, idx):
    return (
        float(keypoints_3d[idx * 3]),
        float(keypoints_3d[idx * 3 + 1]),
        float(keypoints_3d[idx * 3 + 2]),
    )


def sanitize_velocity(vx, vy, vz):
    """ZED SDK đôi khi publish NaN velocity lúc khởi động — tránh crash costmap C++."""
    if all(math.isfinite(float(c)) for c in (vx, vy, vz)):
        return float(vx), float(vy), float(vz)
    return 0.0, 0.0, 0.0


class SocialDensityFusionZedNode:
    TRACKING_OK = 1
    TRACKING_SEARCHING = 2

    def __init__(self):
        rospy.init_node("social_density_fusion_node", anonymous=True)

        self.publish_rate = float(rospy.get_param("~publish_rate", 30.0))
        self.fusion_rate = float(rospy.get_param("~fusion_rate", 20.0))
        self.fusion_on_lidar_callback = bool(rospy.get_param("~fusion_on_lidar_callback", True))
        self.max_data_age_sec = float(rospy.get_param("~max_data_age_sec", 0.25))
        self.polygon_boundary_margin = float(rospy.get_param("~polygon_boundary_margin", 0.25))
        self.min_confidence = float(rospy.get_param("~min_confidence", 40.0))
        self.default_emotion = str(rospy.get_param("~default_emotion", "Neutral"))
        self.use_live_emotions = bool(rospy.get_param("~use_live_emotions", True))
        self.person_emotions_topic = str(rospy.get_param("~person_emotions_topic", "/person_emotions"))
        self.emotion_stale_timeout_sec = float(rospy.get_param("~emotion_stale_timeout_sec", 2.0))
        self.bodies_topic = rospy.get_param("~bodies_topic", "/bodies")
        self.objects_topic = rospy.get_param("~objects_topic", "/objects")
        self.use_live_objects = bool(rospy.get_param("~use_live_objects", True))
        self.od_min_confidence = float(rospy.get_param("~od_min_confidence", 40.0))
        self.source_frame = rospy.get_param("~source_frame", "camera_link")
        self.target_frame = rospy.get_param("~target_frame", "map")
        self.tf_lookup_timeout = float(rospy.get_param("~tf_lookup_timeout", 0.15))

        # Velocity compensation (ROS map + robot EKF are the primary reference)
        self.body_measurement_reference = str(
            rospy.get_param("~body_measurement_reference", "camera")
        ).lower()
        self.velocity_compensation_mode = str(
            rospy.get_param("~velocity_compensation_mode", "map_finite_difference")
        ).lower()
        self.navigation_frame = rospy.get_param("~navigation_frame", self.target_frame)
        self.camera_frame = rospy.get_param("~camera_frame", self.source_frame)
        self.robot_base_frame = rospy.get_param("~robot_base_frame", "base_footprint")
        self.robot_odom_topic = rospy.get_param("~robot_odom_topic", "/odom")
        self.enable_camera_translation_compensation = bool(
            rospy.get_param("~enable_camera_translation_compensation", True)
        )
        self.enable_camera_rotation_compensation = bool(
            rospy.get_param("~enable_camera_rotation_compensation", True)
        )
        self.rotational_compensation_sign = float(
            rospy.get_param("~rotational_compensation_sign", -1.0)
        )
        self.velocity_low_pass_alpha = float(rospy.get_param("~velocity_low_pass_alpha", 0.3))
        self.velocity_max_valid = float(rospy.get_param("~velocity_max_valid", 4.0))
        self.velocity_stationary_threshold = float(
            rospy.get_param("~velocity_stationary_threshold", 0.10)
        )
        self.velocity_jump_threshold = float(rospy.get_param("~velocity_jump_threshold", 1.5))
        self.velocity_heading_min_speed = float(
            rospy.get_param("~velocity_heading_min_speed", 0.15)
        )
        self.velocity_fd_min_dt = float(rospy.get_param("~velocity_fd_min_dt", 0.03))
        self.velocity_fd_max_dt = float(rospy.get_param("~velocity_fd_max_dt", 0.50))
        # Spin-tuned (live PASS 22/08: robot quay + người đứng yên → med/p90≈0)
        self.enable_fd_yaw_compensation = bool(
            rospy.get_param("~enable_fd_yaw_compensation", True)
        )
        self.fd_yaw_omega_gate = float(rospy.get_param("~fd_yaw_omega_gate", 0.05))
        self.fd_yaw_low_pass_alpha = float(rospy.get_param("~fd_yaw_low_pass_alpha", 0.12))
        self.spin_stationary_threshold = float(
            rospy.get_param("~spin_stationary_threshold", 0.35)
        )
        self.force_zero_while_spinning = bool(
            rospy.get_param("~force_zero_while_spinning", True)
        )
        self.tf_timeout = float(rospy.get_param("~tf_timeout", self.tf_lookup_timeout))
        self.odom_timeout = float(rospy.get_param("~odom_timeout", 0.20))
        self.publish_velocity_debug = bool(rospy.get_param("~publish_velocity_debug", True))
        self.publish_pedestrian_measurements = bool(
            rospy.get_param("~publish_pedestrian_measurements", True)
        )
        self.velocity_arrow_length_scale = float(
            rospy.get_param("~velocity_arrow_length_scale", 0.8)
        )

        self._validate_velocity_config()

        # Group / interaction detection
        self.enable_group_detection = bool(rospy.get_param("~enable_group_detection", True))
        self.enable_interaction_detection = bool(rospy.get_param("~enable_interaction_detection", True))
        self.group_max_member_distance = float(rospy.get_param("~group_max_member_distance", 2.5))
        self.group_facing_angle_deg = float(rospy.get_param("~group_facing_angle_deg", 60.0))
        self.group_min_facing_ratio = float(rospy.get_param("~group_min_facing_ratio", 0.5))
        self.interaction_max_distance = float(rospy.get_param("~interaction_max_distance", 3.0))
        self.interaction_facing_angle_deg = float(rospy.get_param("~interaction_facing_angle_deg", 45.0))
        self.velocity_facing_min_speed = float(
            rospy.get_param("~velocity_facing_min_speed", self.velocity_heading_min_speed)
        )
        self.facing_arrow_length = float(rospy.get_param("~facing_arrow_length", 0.6))
        # Facing chỉ dùng để tính group/interaction; mặc định không vẽ mũi tên trên RViz
        self.publish_facing_markers = bool(rospy.get_param("~publish_facing_markers", False))
        self.allow_velocity_facing_fallback = bool(
            rospy.get_param("~allow_velocity_facing_fallback", False)
        )

        self.config_path = rospy.get_param("~config_path", "")
        self.auto_reload_config = rospy.get_param("~auto_reload_config", False)
        self.config_reload_interval_sec = float(rospy.get_param("~config_reload_interval_sec", 1.0))
        self.last_config_mtime = None
        self.lidar_polygon_topic = rospy.get_param("~lidar_polygon_topic", "/lidar_map_polygon")

        self.raw_bodies = None
        self.raw_bodies_recv_time = None
        self.raw_objects = None
        self.raw_objects_recv_time = None
        self.live_objects_map = []  # [{name,x,y,z,interaction_type}] after TF
        self.lidar_polygon_xy = []
        self.lidar_polygon_recv_time = None
        self.lidar_polygon_bbox = None
        self.lidar_polygon_area = 0.0
        self.robot_pose = None

        self.person_data = {}
        self.detected_groups = []  # list of dicts
        self.detected_interactions = []  # list of dicts
        self.current_density = 0.0
        self.current_people_count = 0

        # Nhớ người ngoài FOV chỉ để tính mật độ (/density_info) — không inject social layers
        self.enable_density_track_memory = bool(
            rospy.get_param("~enable_density_track_memory", True)
        )
        self.density_memory_hold_sec = float(rospy.get_param("~density_memory_hold_sec", 60.0))
        self.density_memory_coast = bool(rospy.get_param("~density_memory_coast", True))
        self.density_memory_max_speed = float(rospy.get_param("~density_memory_max_speed", 1.5))
        self.density_track_memory = {}  # id -> {x,y,vx,vy,last_seen,name,from_fov}
        self.density_memory_count = 0

        self.config_groups = {}
        self.config_interactions = {}
        self.interaction_types = {}
        self.config_objects = []  # [{name, x, y, z, interaction_type}]
        self.config_actor_emotions = {}
        self.config_density_publish_enabled = False
        self.live_emotions = {}  # name -> emotion str
        self.live_emotions_recv_time = None

        self.tf_buffer = tf2_ros.Buffer(cache_time=rospy.Duration(5.0))
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer)

        self.velocity_compensator = VelocityCompensator(
            self.tf_buffer,
            navigation_frame=self.navigation_frame,
            camera_frame=self.camera_frame,
            robot_base_frame=self.robot_base_frame,
            odom_topic=self.robot_odom_topic,
            odom_timeout=self.odom_timeout,
            enable_translation=self.enable_camera_translation_compensation,
            enable_rotation=self.enable_camera_rotation_compensation,
            rotational_compensation_sign=self.rotational_compensation_sign,
            low_pass_alpha=self.velocity_low_pass_alpha,
            velocity_max_valid=self.velocity_max_valid,
            stationary_threshold=self.velocity_stationary_threshold,
            jump_threshold=self.velocity_jump_threshold,
            heading_min_speed=self.velocity_heading_min_speed,
            fd_min_dt=self.velocity_fd_min_dt,
            fd_max_dt=self.velocity_fd_max_dt,
            enable_fd_yaw_compensation=self.enable_fd_yaw_compensation,
            fd_yaw_omega_gate=self.fd_yaw_omega_gate,
            fd_yaw_low_pass_alpha=self.fd_yaw_low_pass_alpha,
            spin_stationary_threshold=self.spin_stationary_threshold,
            force_zero_while_spinning=self.force_zero_while_spinning,
        )

        social_state_topic = str(rospy.get_param("~social_state_topic", "/social_state"))
        self.state_pub = rospy.Publisher(social_state_topic, SocialState, queue_size=10)
        rospy.loginfo("social_density_fusion_node_zed: publish SocialState → %s", social_state_topic)
        self.density_info_pub = rospy.Publisher("/density_info", DensityInfo, queue_size=10)
        self.marker_pub = rospy.Publisher("/density_visualization", MarkerArray, queue_size=10)
        self.vel_debug_pub = rospy.Publisher(
            "/zed_velocity_compensation/debug",
            VelocityCompensationDebugArray,
            queue_size=5,
        )
        self.person_vel_debug_pub = rospy.Publisher(
            "/zed_person_velocity/debug",
            PersonVelocityDebugArray,
            queue_size=5,
        )
        self.vel_marker_pub = rospy.Publisher(
            "/zed_velocity_compensation/markers",
            MarkerArray,
            queue_size=5,
        )
        self.pedestrian_meas_pub = rospy.Publisher(
            "/pedestrian_measurements",
            PedestrianMeasurementArray,
            queue_size=5,
        )
        self.last_velocity_debug = []

        rospy.Subscriber(
            self.bodies_topic,
            BodiesStamped,
            self.bodies_callback,
            queue_size=1,
            tcp_nodelay=True,
        )
        if self.use_live_objects:
            rospy.Subscriber(
                self.objects_topic,
                ObjectsStamped,
                self.objects_callback,
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
        if self.use_live_emotions and self.person_emotions_topic:
            rospy.Subscriber(
                self.person_emotions_topic,
                SocialPeople,
                self.person_emotions_callback,
                queue_size=5,
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
        self.fusion_timer = rospy.Timer(rospy.Duration(1.0 / self.fusion_rate), self.update_fused_bodies)
        self.publish_timer = rospy.Timer(rospy.Duration(1.0 / self.publish_rate), self.publish_social_state)

        rospy.loginfo(
            "Velocity config: reference=%s mode=%s odom=%s nav=%s",
            self.body_measurement_reference,
            self.velocity_compensation_mode,
            self.robot_odom_topic,
            self.navigation_frame,
        )

        if self.auto_reload_config:
            self.config_watch_timer = rospy.Timer(
                rospy.Duration(max(0.2, self.config_reload_interval_sec)), self.check_config_changes
            )

        rospy.loginfo(
            "social_density_fusion_node_zed: bodies=%s, group=%s, interaction=%s, objects=%d, "
            "live_emotions=%s, density_memory=%s hold=%.1fs",
            self.bodies_topic,
            self.enable_group_detection,
            self.enable_interaction_detection,
            len(self.config_objects),
            self.use_live_emotions,
            self.enable_density_track_memory,
            self.density_memory_hold_sec,
        )

    def person_emotions_callback(self, msg: SocialPeople):
        live = {}
        for pe in msg.people:
            if not pe.name:
                continue
            live[str(pe.name)] = str(pe.emotion or self.default_emotion)
        self.live_emotions = live
        self.live_emotions_recv_time = rospy.Time.now()

    def resolve_person_emotion(self, name):
        """Ưu tiên: FER live → YAML config → default."""
        if self.use_live_emotions and self.live_emotions_recv_time is not None:
            age = (rospy.Time.now() - self.live_emotions_recv_time).to_sec()
            if age <= self.emotion_stale_timeout_sec and name in self.live_emotions:
                return self.live_emotions[name]
        if name in self.config_actor_emotions:
            return self.config_actor_emotions[name]
        return self.default_emotion

    def _validate_velocity_config(self):
        ref = self.body_measurement_reference
        mode = self.velocity_compensation_mode
        valid_refs = ("camera", "world")
        valid_modes = ("none", "zed_world", "robot_odometry", "map_finite_difference")
        if ref not in valid_refs:
            rospy.logwarn("body_measurement_reference=%s invalid; falling back to camera", ref)
            self.body_measurement_reference = "camera"
            ref = "camera"
        if mode not in valid_modes:
            rospy.logwarn(
                "velocity_compensation_mode=%s invalid; falling back to map_finite_difference",
                mode,
            )
            self.velocity_compensation_mode = "map_finite_difference"
            mode = "map_finite_difference"

        incompatible = (
            (ref == "camera" and mode == "zed_world")
            or (ref == "world" and mode in ("robot_odometry", "map_finite_difference"))
        )
        if incompatible:
            rospy.logerr(
                "Incompatible velocity config: body_measurement_reference=%s with "
                "velocity_compensation_mode=%s. Refusing unsafe mix; forcing "
                "reference=camera, mode=map_finite_difference.",
                ref,
                mode,
            )
            self.body_measurement_reference = "camera"
            self.velocity_compensation_mode = "map_finite_difference"
            mode = "map_finite_difference"

        if mode == "none":
            rospy.logwarn(
                "velocity_compensation_mode=none: raw/rotated velocity only (debug). "
                "NOT suitable for costmap proxemic."
            )
        if mode == "zed_world":
            rospy.logwarn(
                "velocity_compensation_mode=zed_world: ZED WORLD ≠ ROS map without "
                "verified extrinsic. Measurements marked compensation_valid=False."
            )
        if mode == "map_finite_difference":
            rospy.loginfo(
                "velocity_compensation_mode=map_finite_difference: "
                "world velocity = d(p_map)/dt (robot ego-motion cancelled)."
            )

    @staticmethod
    def person_name(body_id):
        return f"person_{body_id}"

    def bodies_callback(self, msg):
        self.raw_bodies = msg
        self.raw_bodies_recv_time = rospy.Time.now()

    def objects_callback(self, msg):
        self.raw_objects = msg
        self.raw_objects_recv_time = rospy.Time.now()

    def lidar_polygon_callback(self, msg):
        if msg.header.frame_id != self.target_frame:
            rospy.logwarn_throttle(
                10.0,
                "lidar_polygon frame_id=%s (expected %s); van dung diem nhan duoc.",
                msg.header.frame_id,
                self.target_frame,
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

        if self.fusion_on_lidar_callback:
            self.update_fused_bodies(None)

    def amcl_callback(self, msg):
        self.robot_pose = msg.pose.pose

    def load_scenario_config(self, config_data=None):
        if isinstance(config_data, dict):
            people_cfg = config_data.get("people", [])
            groups_cfg = config_data.get("groups", {})
            interactions_cfg = config_data.get("interactions", {})
            density_cfg = config_data.get("density", {})
            objects_cfg = config_data.get("objects", [])
            detection_cfg = config_data.get("detection", {})
        else:
            people_cfg = rospy.get_param("~people", [])
            groups_cfg = rospy.get_param("~groups", {})
            interactions_cfg = rospy.get_param("~interactions", {})
            density_cfg = rospy.get_param("~density", {})
            objects_cfg = rospy.get_param("~objects", [])
            detection_cfg = rospy.get_param("~detection", {})

        if objects_cfg is None:
            objects_cfg = []
        if people_cfg is None:
            people_cfg = []
        if groups_cfg is None:
            groups_cfg = []
        if interactions_cfg is None:
            interactions_cfg = []
        if detection_cfg is None:
            detection_cfg = {}
        if density_cfg is None:
            density_cfg = {}

        emotions = {}
        if isinstance(people_cfg, list):
            for p in people_cfg:
                if isinstance(p, dict) and p.get("name"):
                    emotions[str(p["name"])] = str(p.get("emotion", self.default_emotion))
        self.config_actor_emotions = emotions

        # Optional static groups from YAML (rarely used on real robot)
        groups = {}
        if isinstance(groups_cfg, list):
            for g in groups_cfg:
                if not isinstance(g, dict) or not g.get("group_name"):
                    continue
                members = [str(m) for m in g.get("members", [])]
                groups[str(g["group_name"])] = members
        elif isinstance(groups_cfg, dict):
            for gname, members in groups_cfg.items():
                groups[str(gname)] = [str(m) for m in members]
        self.config_groups = groups

        # Objects in map frame for interaction detection
        objects = []
        if isinstance(objects_cfg, list):
            for obj in objects_cfg:
                if not isinstance(obj, dict) or not obj.get("name"):
                    continue
                objects.append(
                    {
                        "name": str(obj["name"]),
                        "x": float(obj.get("x", 0.0)),
                        "y": float(obj.get("y", 0.0)),
                        "z": float(obj.get("z", 0.0)),
                        "interaction_type": str(obj.get("interaction_type", "seeing")),
                    }
                )
        self.config_objects = objects

        # Legacy interactions YAML (object_name + participants) — kept for compatibility
        interactions = {}
        interaction_types = {}
        if isinstance(interactions_cfg, list):
            for it in interactions_cfg:
                if not isinstance(it, dict) or not it.get("object_name"):
                    continue
                obj = str(it["object_name"])
                interactions[obj] = [str(p) for p in it.get("participants", [])]
                interaction_types[obj] = str(it.get("interaction_type", "default"))
        self.config_interactions = interactions
        self.interaction_types = interaction_types

        if isinstance(density_cfg, dict):
            self.config_density_publish_enabled = bool(density_cfg.get("button_state", False))
        elif isinstance(density_cfg, bool):
            self.config_density_publish_enabled = density_cfg

        if isinstance(detection_cfg, dict):
            if "enable_group_detection" in detection_cfg:
                self.enable_group_detection = bool(detection_cfg["enable_group_detection"])
            if "enable_interaction_detection" in detection_cfg:
                self.enable_interaction_detection = bool(detection_cfg["enable_interaction_detection"])
            if "group_max_member_distance" in detection_cfg:
                self.group_max_member_distance = float(detection_cfg["group_max_member_distance"])
            if "group_facing_angle_deg" in detection_cfg:
                self.group_facing_angle_deg = float(detection_cfg["group_facing_angle_deg"])
            if "interaction_max_distance" in detection_cfg:
                self.interaction_max_distance = float(detection_cfg["interaction_max_distance"])
            if "interaction_facing_angle_deg" in detection_cfg:
                self.interaction_facing_angle_deg = float(detection_cfg["interaction_facing_angle_deg"])

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

    def lookup_body_transform(self, stamp):
        src_frame = self.source_frame
        if self.raw_bodies is not None and self.raw_bodies.header.frame_id:
            src_frame = self.raw_bodies.header.frame_id

        try:
            return self.tf_buffer.lookup_transform(
                self.target_frame,
                src_frame,
                stamp,
                rospy.Duration(self.tf_lookup_timeout),
            )
        except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
            try:
                return self.tf_buffer.lookup_transform(
                    self.target_frame,
                    src_frame,
                    rospy.Time(0),
                    rospy.Duration(self.tf_lookup_timeout),
                )
            except (tf2_ros.LookupException, tf2_ros.ConnectivityException, tf2_ros.ExtrapolationException):
                rospy.logwarn_throttle(
                    2.0,
                    "Khong lookup duoc TF %s <- %s",
                    self.target_frame,
                    src_frame,
                )
                return None

    def transform_body_to_map(self, transform, body):
        """Position: full rigid transform. Velocity: rotate-only (legacy helper).

        Absolute map velocity must go through VelocityCompensator — rotate-only is
        insufficient when BodyData is CAMERA-relative.
        """
        q = transform.transform.rotation
        t = transform.transform.translation

        px, py, pz = float(body.position[0]), float(body.position[1]), float(body.position[2])
        if not all(math.isfinite(c) for c in (px, py, pz)):
            px, py, pz = 0.0, 0.0, 0.0
        vx, vy, vz = sanitize_velocity(
            body.velocity[0], body.velocity[1], body.velocity[2]
        )

        rx, ry, rz = rotate_point_by_quaternion(px, py, pz, q.x, q.y, q.z, q.w)
        rvx, rvy, rvz = rotate_point_by_quaternion(vx, vy, vz, q.x, q.y, q.z, q.w)

        return (
            t.x + rx,
            t.y + ry,
            t.z + rz,
            rvx,
            rvy,
            rvz,
            float(body.confidence),
            (px, py, pz),
            (vx, vy, vz),
        )

    def transform_keypoint_to_map(self, transform, keypoints_3d, idx):
        if not is_valid_kp(keypoints_3d, idx):
            return None
        x, y, z = kp_xyz(keypoints_3d, idx)
        q = transform.transform.rotation
        t = transform.transform.translation
        rx, ry, rz = rotate_point_by_quaternion(x, y, z, q.x, q.y, q.z, q.w)
        return (t.x + rx, t.y + ry, t.z + rz)

    def estimate_facing_orientation(self, transform, body, map_vx=0.0, map_vy=0.0):
        """Facing orientation (body/face) — NOT motion heading.

        Primary: shoulder–nose skeleton. Optional weak velocity fallback only if enabled.
        """
        fmt = int(body.body_format) if body.body_format else 34
        kps = body.keypoints_3d

        left_idx = right_idx = nose_idx = None
        if fmt == 18:
            left_idx, right_idx, nose_idx = BODY18_L_SHOULDER, BODY18_R_SHOULDER, BODY18_NOSE
        else:
            left_idx, right_idx = BODY34_L_SHOULDER, BODY34_R_SHOULDER
            nose_idx = BODY34_NOSE if is_valid_kp(kps, BODY34_NOSE) else BODY34_HEAD

        left = self.transform_keypoint_to_map(transform, kps, left_idx) if left_idx is not None else None
        right = self.transform_keypoint_to_map(transform, kps, right_idx) if right_idx is not None else None
        nose = self.transform_keypoint_to_map(transform, kps, nose_idx) if nose_idx is not None else None

        if left is not None and right is not None:
            sx = right[0] - left[0]
            sy = right[1] - left[1]
            fx, fy = sy, -sx
            mid_x = 0.5 * (left[0] + right[0])
            mid_y = 0.5 * (left[1] + right[1])
            if nose is not None:
                nx, ny = nose[0] - mid_x, nose[1] - mid_y
                if fx * nx + fy * ny < 0.0:
                    fx, fy = -fx, -fy
            norm = math.hypot(fx, fy)
            if norm > 1e-3:
                return math.atan2(fy, fx), True, "skeleton"

        if self.allow_velocity_facing_fallback:
            speed = math.hypot(map_vx, map_vy)
            if speed >= self.velocity_facing_min_speed:
                return math.atan2(map_vy, map_vx), False, "velocity_fallback_uncertain"

        return None, False, "none"

    # Backward-compatible alias
    def estimate_facing_yaw(self, transform, body, map_vx, map_vy):
        yaw, from_skel, _src = self.estimate_facing_orientation(transform, body, map_vx, map_vy)
        return yaw, from_skel

    def detect_groups(self, person_data):
        """Nhóm: khoảng cách gần + hướng về tâm chung (F-formation)."""
        names = [n for n, d in person_data.items() if d.get("yaw") is not None]
        if len(names) < 2:
            return []

        max_dist = self.group_max_member_distance
        facing_lim = math.radians(self.group_facing_angle_deg)

        # Union-Find theo khoảng cách
        parent = {n: n for n in names}

        def find(a):
            while parent[a] != a:
                parent[a] = parent[parent[a]]
                a = parent[a]
            return a

        def union(a, b):
            ra, rb = find(a), find(b)
            if ra != rb:
                parent[rb] = ra

        for i, ni in enumerate(names):
            di = person_data[ni]
            for nj in names[i + 1 :]:
                dj = person_data[nj]
                if math.hypot(di["x"] - dj["x"], di["y"] - dj["y"]) <= max_dist:
                    union(ni, nj)

        clusters = {}
        for n in names:
            root = find(n)
            clusters.setdefault(root, []).append(n)

        groups = []
        gid = 0
        for members in clusters.values():
            if len(members) < 2:
                continue
            cx = sum(person_data[m]["x"] for m in members) / len(members)
            cy = sum(person_data[m]["y"] for m in members) / len(members)

            facing_members = []
            for m in members:
                d = person_data[m]
                to_center = math.atan2(cy - d["y"], cx - d["x"])
                if abs(angle_diff(d["yaw"], to_center)) <= facing_lim:
                    facing_members.append(m)

            ratio = float(len(facing_members)) / float(len(members))
            if len(facing_members) < 2 or ratio < self.group_min_facing_ratio:
                continue

            radius = max(math.hypot(person_data[m]["x"] - cx, person_data[m]["y"] - cy) for m in members)
            gid += 1
            groups.append(
                {
                    "group_name": f"group_{gid}",
                    "members": list(facing_members) if len(facing_members) >= 2 else list(members),
                    "cx": cx,
                    "cy": cy,
                    "radius": max(radius, 0.3),
                    "reliability": ratio,
                }
            )
        return groups

    def detect_interactions(self, person_data):
        """Tương tác: người hướng về object (YAML tĩnh + /objects live) trong khoảng cách."""
        object_list = list(self.config_objects)
        object_list.extend(self.live_objects_map)
        if not object_list:
            return []

        facing_lim = math.radians(self.interaction_facing_angle_deg)
        max_dist = self.interaction_max_distance
        interactions = []

        for obj in object_list:
            participants = []
            for name, d in person_data.items():
                if d.get("yaw") is None:
                    continue
                dx = obj["x"] - d["x"]
                dy = obj["y"] - d["y"]
                dist = math.hypot(dx, dy)
                if dist > max_dist or dist < 1e-3:
                    continue
                to_obj = math.atan2(dy, dx)
                if abs(angle_diff(d["yaw"], to_obj)) <= facing_lim:
                    participants.append(name)

            if not participants:
                continue
            interactions.append(
                {
                    "object_name": obj["name"],
                    "x": obj["x"],
                    "y": obj["y"],
                    "z": obj["z"],
                    "participants": participants,
                    "interaction_type": obj.get("interaction_type", "seeing"),
                    "reliability": min(1.0, float(len(participants)) / 3.0),
                }
            )
        return interactions

    def update_live_objects_map(self, transform):
        """Transform /objects (camera frame) → map cho interaction detection."""
        live = []
        if not self.use_live_objects or self.raw_objects is None or transform is None:
            self.live_objects_map = live
            return

        now = rospy.Time.now()
        if self.raw_objects_recv_time is not None:
            age = (now - self.raw_objects_recv_time).to_sec()
            if age > self.max_data_age_sec:
                self.live_objects_map = live
                return

        q = transform.transform.rotation
        t = transform.transform.translation
        for obj in self.raw_objects.objects:
            if float(obj.confidence) < self.od_min_confidence:
                continue
            if int(obj.tracking_state) == 0:
                continue
            px, py, pz = float(obj.position[0]), float(obj.position[1]), float(obj.position[2])
            if not all(math.isfinite(c) for c in (px, py, pz)):
                continue
            rx, ry, rz = rotate_point_by_quaternion(px, py, pz, q.x, q.y, q.z, q.w)
            mx, my, mz = t.x + rx, t.y + ry, t.z + rz
            if not all(math.isfinite(c) for c in (mx, my, mz)):
                continue
            label = obj.label if obj.label else "Object"
            live.append(
                {
                    "name": f"{label}_{obj.id}",
                    "x": mx,
                    "y": my,
                    "z": mz,
                    "interaction_type": "seeing",
                    "live": True,
                }
            )
        self.live_objects_map = live

    def make_person_msg(self, name, data, header=None):
        person = SocialPerson()
        person.name = name
        person.reliability = min(1.0, max(0.0, data["confidence"] / 100.0))
        person.emotion = self.resolve_person_emotion(name)
        person.position.position.x = data["x"]
        person.position.position.y = data["y"]
        person.position.position.z = data["z"]
        facing = data.get("facing_orientation", data.get("yaw"))
        if facing is not None:
            person.position.orientation = yaw_to_quaternion(facing)
        else:
            person.position.orientation.w = 1.0
        person.velocity.linear.x = data["vx"]
        person.velocity.linear.y = data["vy"]
        person.velocity.linear.z = data["vz"]
        vx, vy, vz = sanitize_velocity(
            person.velocity.linear.x,
            person.velocity.linear.y,
            person.velocity.linear.z,
        )
        person.velocity.linear.x = vx
        person.velocity.linear.y = vy
        person.velocity.linear.z = vz
        return person

    def publish_velocity_markers(self, debug_samples, person_data, stamp):
        if self.vel_marker_pub.get_num_connections() == 0:
            return
        ma = MarkerArray()
        scale = max(0.1, self.velocity_arrow_length_scale)
        mid = 0
        for dbg in debug_samples:
            name = dbg.name
            data = person_data.get(name)
            if data is None:
                continue
            px, py = data["x"], data["y"]
            z = 1.35

            def add_arrow(ns, mid_, vx, vy, r, g, b, length_scale):
                m = Marker()
                m.header.frame_id = self.navigation_frame
                m.header.stamp = stamp
                m.ns = ns
                m.id = mid_
                m.type = Marker.ARROW
                m.action = Marker.ADD
                m.scale.x = 0.04
                m.scale.y = 0.07
                m.scale.z = 0.07
                m.color.r, m.color.g, m.color.b, m.color.a = r, g, b, 0.95
                m.pose.orientation.w = 1.0
                p0 = Point(x=px, y=py, z=z)
                speed = math.hypot(vx, vy)
                L = length_scale * min(2.0, max(0.15, speed))
                if speed < 1e-3:
                    p1 = Point(x=px, y=py, z=z)
                else:
                    p1 = Point(x=px + L * vx / speed, y=py + L * vy / speed, z=z)
                m.points = [p0, p1]
                return m

            # Raw camera velocity rotated to map (no ego compensation) — orange
            rv = dbg.rotated_velocity_map
            ma.markers.append(add_arrow("raw_rotated_velocity", mid, rv[0], rv[1], 1.0, 0.45, 0.0, scale))
            mid += 1
            # Final compensated — green
            fv = dbg.final_velocity_map
            ma.markers.append(add_arrow("final_velocity", mid, fv[0], fv[1], 0.1, 0.85, 0.2, scale))
            mid += 1
            # Facing — cyan (same as density viz, for side-by-side)
            if dbg.facing_orientation_valid:
                fx = math.cos(dbg.facing_orientation)
                fy = math.sin(dbg.facing_orientation)
                ma.markers.append(
                    add_arrow("facing_orientation", mid, fx, fy, 0.1, 0.7, 1.0, self.facing_arrow_length)
                )
                mid += 1
            text = Marker()
            text.header.frame_id = self.navigation_frame
            text.header.stamp = stamp
            text.ns = "velocity_text"
            text.id = mid
            mid += 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = px
            text.pose.position.y = py
            text.pose.position.z = z + 0.35
            text.pose.orientation.w = 1.0
            text.scale.z = 0.25
            text.color.r = text.color.g = text.color.b = 1.0
            text.color.a = 1.0
            text.text = "id=%d %s v=%.2f %s" % (
                dbg.person_id,
                self.resolve_person_emotion(name),
                dbg.speed,
                "OK" if dbg.compensation_valid else "INVALID",
            )
            ma.markers.append(text)

        self.vel_marker_pub.publish(ma)

    def publish_density_visualization(self, lidar_polygon, density, people_count):
        if self.marker_pub.get_num_connections() == 0:
            return
        marker_array = MarkerArray()
        now = rospy.Time.now()

        lidar_marker = Marker()
        lidar_marker.header.frame_id = self.target_frame
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

        if self.config_density_publish_enabled and self.robot_pose is not None:
            text_marker = Marker()
            text_marker.header.frame_id = self.target_frame
            text_marker.header.stamp = now
            text_marker.ns = "density_text"
            text_marker.id = 1
            text_marker.type = Marker.TEXT_VIEW_FACING
            text_marker.action = Marker.ADD
            text_marker.scale.z = 0.5
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
            text_marker.text = (
                f"{people_count} persons\n{density:.4f} people/m²\n"
                f"groups={len(self.detected_groups)} interactions={len(self.detected_interactions)}"
                f" ({self.density_memory_count} remembered)\n"
            )
            marker_array.markers.append(text_marker)
        else:
            clear_text = Marker()
            clear_text.header.frame_id = self.target_frame
            clear_text.header.stamp = now
            clear_text.ns = "density_text"
            clear_text.id = 1
            clear_text.action = Marker.DELETE
            marker_array.markers.append(clear_text)

        # Facing chỉ phục vụ tính toán group/interaction — không vẽ mặc định
        if self.publish_facing_markers:
            facing_id = 0
            for name, data in self.person_data.items():
                facing = data.get("facing_orientation", data.get("yaw"))
                if facing is None:
                    continue
                arrow = Marker()
                arrow.header.frame_id = self.target_frame
                arrow.header.stamp = now
                arrow.ns = "facing_orientation"
                arrow.id = facing_id
                facing_id += 1
                arrow.type = Marker.ARROW
                arrow.action = Marker.ADD
                arrow.scale.x = 0.05
                arrow.scale.y = 0.1
                arrow.scale.z = 0.1
                arrow.color.r = 0.1
                arrow.color.g = 0.6
                arrow.color.b = 1.0
                arrow.color.a = 0.9
                arrow.pose.orientation.w = 1.0
                p0 = Point()
                p0.x = data["x"]
                p0.y = data["y"]
                p0.z = 1.2
                p1 = Point()
                p1.x = data["x"] + self.facing_arrow_length * math.cos(facing)
                p1.y = data["y"] + self.facing_arrow_length * math.sin(facing)
                p1.z = 1.2
                arrow.points = [p0, p1]
                marker_array.markers.append(arrow)

        # Group centers
        for i, g in enumerate(self.detected_groups):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = now
            m.ns = "group_center"
            m.id = i
            m.type = Marker.SPHERE
            m.action = Marker.ADD
            m.pose.position.x = g["cx"]
            m.pose.position.y = g["cy"]
            m.pose.position.z = 0.3
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = m.scale.z = 0.35
            m.color.r = 1.0
            m.color.g = 0.5
            m.color.a = 0.8
            marker_array.markers.append(m)

        # Objects (YAML tĩnh + OD live)
        all_objs = list(self.config_objects) + list(self.live_objects_map)
        for i, obj in enumerate(all_objs):
            m = Marker()
            m.header.frame_id = self.target_frame
            m.header.stamp = now
            m.ns = "objects"
            m.id = i
            m.type = Marker.CUBE
            m.action = Marker.ADD
            m.pose.position.x = obj["x"]
            m.pose.position.y = obj["y"]
            m.pose.position.z = 0.5
            m.pose.orientation.w = 1.0
            m.scale.x = m.scale.y = 0.25
            m.scale.z = 1.0
            if obj.get("live"):
                m.color.r = 0.2
                m.color.g = 0.9
                m.color.b = 0.3
            else:
                m.color.b = 1.0
            m.color.a = 0.5
            marker_array.markers.append(m)

        self.marker_pub.publish(marker_array)

    def _point_in_density_region(self, x, y, lidar_polygon, margin, bbox_expanded):
        emin_x, emax_x, emin_y, emax_y = bbox_expanded
        if x < emin_x or x > emax_x or y < emin_y or y > emax_y:
            return False
        if self.is_point_inside_polygon(x, y, lidar_polygon):
            return True
        return self.is_point_near_polygon_boundary(x, y, lidar_polygon, margin)

    def _touch_density_memory(self, person_id, x, y, vx, vy, name, now_sec):
        """Cập nhật track khi ZED còn thấy người (trong FOV) — chỉ phục vụ mật độ."""
        if not self.enable_density_track_memory:
            return
        speed = math.hypot(vx, vy)
        if speed > self.density_memory_max_speed:
            scale = self.density_memory_max_speed / max(speed, 1e-6)
            vx, vy = vx * scale, vy * scale
        self.density_track_memory[int(person_id)] = {
            "x": float(x),
            "y": float(y),
            "vx": float(vx),
            "vy": float(vy),
            "name": name,
            "last_seen": float(now_sec),
            "last_update": float(now_sec),
            "from_fov": True,
        }

    def _refresh_density_memory(self, now_sec, visible_ids):
        """Coast + hết hạn track khi người đã ra khỏi FOV."""
        if not self.enable_density_track_memory:
            self.density_track_memory = {}
            return
        expired = []
        for pid, tr in self.density_track_memory.items():
            age = now_sec - tr["last_seen"]
            if age > self.density_memory_hold_sec:
                expired.append(pid)
                continue
            if pid in visible_ids:
                continue
            if self.density_memory_coast:
                dt = now_sec - tr.get("last_update", tr["last_seen"])
                if dt > 0.0:
                    tr["x"] += tr["vx"] * dt
                    tr["y"] += tr["vy"] * dt
                    tr["last_update"] = now_sec
            tr["from_fov"] = False
        for pid in expired:
            del self.density_track_memory[pid]

    def _merge_density_people_points(
        self, people_points, visible_ids, lidar_polygon, margin, bbox_expanded, now_sec
    ):
        """Gộp track nhớ (ngoài FOV) vào điểm tính density — không trùng ID."""
        self.density_memory_count = 0
        if not self.enable_density_track_memory:
            return people_points
        self._refresh_density_memory(now_sec, visible_ids)
        for pid, tr in self.density_track_memory.items():
            if pid in visible_ids:
                continue
            x, y = tr["x"], tr["y"]
            if self._point_in_density_region(x, y, lidar_polygon, margin, bbox_expanded):
                people_points.append((x, y))
                self.density_memory_count += 1
        return people_points

    def update_fused_bodies(self, _event):
        if self.raw_bodies is None or self.robot_pose is None:
            return

        now = rospy.Time.now()
        if self.raw_bodies_recv_time is None or self.lidar_polygon_recv_time is None:
            return

        bodies_age = (now - self.raw_bodies_recv_time).to_sec()
        poly_age = (now - self.lidar_polygon_recv_time).to_sec()
        if bodies_age > self.max_data_age_sec or poly_age > self.max_data_age_sec:
            rospy.logwarn_throttle(
                2.0,
                "Du lieu stale: bodies_age=%.3fs, polygon_age=%.3fs (nguong=%.3fs)",
                bodies_age,
                poly_age,
                self.max_data_age_sec,
            )
            return

        lidar_polygon = self.lidar_polygon_xy
        if len(lidar_polygon) < 3 or self.lidar_polygon_bbox is None:
            return

        transform = self.lookup_body_transform(self.raw_bodies.header.stamp)
        if transform is None:
            return

        self.update_live_objects_map(transform)

        min_x, max_x, min_y, max_y = self.lidar_polygon_bbox
        margin = max(0.0, self.polygon_boundary_margin)
        expanded_min_x = min_x - margin
        expanded_max_x = max_x + margin
        expanded_min_y = min_y - margin
        expanded_max_y = max_y + margin

        person_data = {}
        people_points = []
        debug_samples = []
        person_vel_samples = []
        visible_ids = set()
        now_sec = now.to_sec()
        bbox_expanded = (expanded_min_x, expanded_max_x, expanded_min_y, expanded_max_y)
        ped_meas = PedestrianMeasurementArray()
        ped_meas.header.stamp = self.raw_bodies.header.stamp
        ped_meas.header.frame_id = self.navigation_frame

        body_stamp = self.raw_bodies.header.stamp

        for body in self.raw_bodies.bodies:
            if body.tracking_state not in (self.TRACKING_OK, self.TRACKING_SEARCHING):
                continue
            if float(body.confidence) < self.min_confidence:
                continue

            x, y, z, rot_vx, rot_vy, rot_vz, confidence, p_cam, v_cam = self.transform_body_to_map(
                transform, body
            )
            if not all(math.isfinite(c) for c in (x, y, z)):
                rospy.logwarn_throttle(
                    2.0, "Skip body id=%s: non-finite position on /bodies", body.id
                )
                continue
            name = self.person_name(body.id)
            pid = int(body.id)
            visible_ids.add(pid)

            comp = self.velocity_compensator.compensate(
                person_id=int(body.id),
                stamp=body_stamp,
                p_cam=p_cam,
                v_cam=v_cam,
                mode=self.velocity_compensation_mode,
                measurement_reference=self.body_measurement_reference,
                p_map=(x, y, z),
            )

            if comp["compensation_valid"]:
                vx, vy, vz = comp["final_velocity_map"]
            else:
                # Never publish camera-relative / rotate-only into costmap —
                # that injects robot ego-motion as fake person velocity.
                vx, vy, vz = 0.0, 0.0, 0.0
                rospy.logwarn_throttle(
                    2.0,
                    "Velocity estimate invalid for %s; publishing zero "
                    "(safe for proxemic costmap).",
                    name,
                )
            vx, vy, vz = sanitize_velocity(vx, vy, vz)

            facing, from_skel, facing_src = self.estimate_facing_orientation(
                transform, body, vx, vy
            )
            motion_heading = comp.get("motion_heading")
            motion_valid = bool(comp.get("motion_heading_valid"))
            motion_src = comp.get("motion_heading_source", "none")

            person_data[name] = {
                "id": int(body.id),
                "x": round(x, 2),
                "y": round(y, 2),
                "z": round(z, 2),
                "vx": round(vx, 2),
                "vy": round(vy, 2),
                "vz": round(vz, 2),
                "confidence": confidence,
                "tracking_state": int(body.tracking_state),
                "action_state": int(body.action_state) if hasattr(body, "action_state") else 0,
                # yaw = facing for group/interaction backward compatibility
                "yaw": facing,
                "yaw_from_skeleton": from_skel,
                "facing_orientation": facing,
                "facing_orientation_valid": facing is not None,
                "facing_orientation_source": facing_src,
                "motion_heading": motion_heading,
                "motion_heading_valid": motion_valid,
                "motion_heading_source": motion_src,
                "velocity_compensation_valid": bool(comp["compensation_valid"]),
                "speed": float(comp.get("speed", math.hypot(vx, vy))),
                "body_stamp": float(comp.get("body_stamp", 0.0)),
                "odom_stamp": float(comp.get("odom_stamp", 0.0)),
                "body_odom_time_offset": float(comp.get("body_odom_time_offset", float("nan"))),
            }

            dbg = VelocityCompensationDebug()
            dbg.header.stamp = body_stamp
            dbg.header.frame_id = self.navigation_frame
            dbg.person_id = int(body.id)
            dbg.name = name
            dbg.tracking_state = int(body.tracking_state)
            dbg.confidence = float(confidence)
            dbg.measurement_reference = self.body_measurement_reference
            dbg.velocity_compensation_mode = self.velocity_compensation_mode
            dbg.compensation_valid = bool(comp["compensation_valid"])
            dbg.raw_velocity_camera = list(comp["raw_velocity_camera"])
            dbg.rotated_velocity_map = list(comp["rotated_velocity_map"])
            dbg.robot_linear_velocity_map = list(comp["robot_linear_velocity_map"])
            dbg.robot_angular_z = float(comp.get("robot_angular_z", 0.0))
            dbg.rotational_compensation_map = list(comp["rotational_compensation_map"])
            dbg.final_velocity_map = list(comp["final_velocity_map"]) if comp["compensation_valid"] else [vx, vy, vz]
            dbg.speed = float(person_data[name]["speed"])
            dbg.motion_heading = float(motion_heading) if motion_heading is not None else 0.0
            dbg.motion_heading_valid = motion_valid
            dbg.facing_orientation = float(facing) if facing is not None else 0.0
            dbg.facing_orientation_valid = facing is not None
            dbg.body_stamp = person_data[name]["body_stamp"]
            dbg.odom_stamp = person_data[name]["odom_stamp"]
            dbg.body_odom_time_offset = person_data[name]["body_odom_time_offset"]
            debug_samples.append(dbg)

            pvd = PersonVelocityDebug()
            pvd.header.stamp = body_stamp
            pvd.header.frame_id = self.navigation_frame
            pvd.person_id = int(body.id)
            pvd.name = name
            pvd.tracking_state = int(body.tracking_state)
            pvd.confidence = float(confidence)
            pvd.position_camera = list(p_cam)
            pvd.position_map = [x, y, z]
            pvd.raw_velocity_camera = list(comp["raw_velocity_camera"])
            pvd.rotated_velocity_map = list(comp["rotated_velocity_map"])
            pvd.robot_velocity_map = list(comp["robot_linear_velocity_map"])
            pvd.rotational_compensation_map = list(comp["rotational_compensation_map"])
            pvd.compensated_velocity_map = list(
                comp.get("compensated_velocity_map", comp["rotated_velocity_map"])
            )
            pvd.filtered_velocity_map = list(
                comp.get("filtered_velocity_map", comp["final_velocity_map"])
            )
            pub_v = comp.get("published_velocity_map", comp["final_velocity_map"])
            if not comp["compensation_valid"]:
                pub_v = (vx, vy, vz)
            pvd.published_velocity_map = list(pub_v)
            pvd.raw_speed = float(comp.get("raw_speed", 0.0))
            pvd.compensated_speed = float(comp.get("compensated_speed", 0.0))
            pvd.filtered_speed = float(comp.get("filtered_speed", 0.0))
            pvd.published_speed = float(comp.get("published_speed", person_data[name]["speed"]))
            pvd.stationary_threshold_applied = bool(comp.get("stationary_threshold_applied", False))
            pvd.jump_rejected = bool(comp.get("jump_rejected", False))
            pvd.low_pass_applied = bool(comp.get("low_pass_applied", False))
            pvd.velocity_valid = bool(comp.get("velocity_valid", comp["compensation_valid"]))
            pvd.velocity_compensation_mode = self.velocity_compensation_mode
            pvd.measurement_reference = self.body_measurement_reference
            pvd.motion_heading = float(motion_heading) if motion_heading is not None else 0.0
            pvd.motion_heading_valid = motion_valid
            pvd.facing_orientation = float(facing) if facing is not None else 0.0
            pvd.facing_orientation_valid = facing is not None
            pvd.body_timestamp = person_data[name]["body_stamp"]
            pvd.odom_timestamp = person_data[name]["odom_stamp"]
            pvd.time_offset = person_data[name]["body_odom_time_offset"]
            person_vel_samples.append(pvd)

            if self.publish_pedestrian_measurements:
                m = PedestrianMeasurement()
                m.header.stamp = body_stamp
                m.header.frame_id = self.navigation_frame
                m.id = int(body.id)
                m.name = name
                m.position_x = x
                m.position_y = y
                m.position_z = z
                m.velocity_x = vx
                m.velocity_y = vy
                m.velocity_z = vz
                m.speed = person_data[name]["speed"]
                m.confidence = float(confidence)
                m.tracking_state = int(body.tracking_state)
                m.action_state = person_data[name]["action_state"]
                m.facing_orientation = float(facing) if facing is not None else 0.0
                m.facing_valid = facing is not None
                m.facing_source = facing_src
                m.motion_heading = float(motion_heading) if motion_heading is not None else 0.0
                m.motion_heading_valid = motion_valid
                m.motion_heading_source = motion_src
                m.velocity_compensation_valid = bool(comp["compensation_valid"])
                m.measurement_reference = self.body_measurement_reference
                m.velocity_compensation_mode = self.velocity_compensation_mode
                m.body_stamp = person_data[name]["body_stamp"]
                m.odom_stamp = person_data[name]["odom_stamp"]
                m.body_odom_time_offset = person_data[name]["body_odom_time_offset"]
                ped_meas.measurements.append(m)

            if self._point_in_density_region(x, y, lidar_polygon, margin, bbox_expanded):
                people_points.append((x, y))

            self._touch_density_memory(pid, x, y, vx, vy, name, now_sec)

        people_points = self._merge_density_people_points(
            people_points, visible_ids, lidar_polygon, margin, bbox_expanded, now_sec
        )

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

        self.person_data = person_data
        self.last_velocity_debug = debug_samples
        self.detected_groups = self.detect_groups(person_data) if self.enable_group_detection else []
        self.detected_interactions = (
            self.detect_interactions(person_data) if self.enable_interaction_detection else []
        )
        self.publish_density_visualization(lidar_polygon, density, people_count)

        if self.publish_velocity_debug:
            dbg_arr = VelocityCompensationDebugArray()
            dbg_arr.header.stamp = body_stamp
            dbg_arr.header.frame_id = self.navigation_frame
            dbg_arr.samples = debug_samples
            self.vel_debug_pub.publish(dbg_arr)
            self.publish_velocity_markers(debug_samples, person_data, body_stamp)

            pvd_arr = PersonVelocityDebugArray()
            pvd_arr.header.stamp = body_stamp
            pvd_arr.header.frame_id = self.navigation_frame
            pvd_arr.samples = person_vel_samples
            self.person_vel_debug_pub.publish(pvd_arr)

        if self.publish_pedestrian_measurements:
            self.pedestrian_meas_pub.publish(ped_meas)

    def publish_social_state(self, _event):
        state_msg = SocialState()
        state_msg.header.stamp = rospy.Time.now()
        state_msg.header.frame_id = self.target_frame
        state_msg.density = self.current_density

        people_msg = SocialPeople()
        people_msg.header = state_msg.header
        for person_name, data in self.person_data.items():
            people_msg.people.append(self.make_person_msg(person_name, data))
        state_msg.people = people_msg

        groups_msg = SocialGroups()
        groups_msg.header = state_msg.header
        for g in self.detected_groups:
            group_msg = SocialGroup()
            group_msg.group_name = g["group_name"]
            group_msg.position.position.x = g["cx"]
            group_msg.position.position.y = g["cy"]
            group_msg.position.orientation.w = 1.0
            group_msg.radius = g["radius"]
            group_msg.reliability = g["reliability"]
            for member_name in g["members"]:
                if member_name not in self.person_data:
                    continue
                group_msg.members.append(self.make_person_msg(member_name, self.person_data[member_name]))
            if len(group_msg.members) >= 2:
                groups_msg.groups.append(group_msg)
        state_msg.groups = groups_msg

        interactions_msg = SocialInteractions()
        interactions_msg.header = state_msg.header
        for it in self.detected_interactions:
            interaction = SocialInteraction()
            interaction.object_name = it["object_name"]
            interaction.object_position.position.x = it["x"]
            interaction.object_position.position.y = it["y"]
            interaction.object_position.position.z = it["z"]
            interaction.object_position.orientation.w = 1.0
            interaction.reliability = it["reliability"]
            interaction.interaction_type = it["interaction_type"]
            interaction.timestamp = state_msg.header.stamp
            for pname in it["participants"]:
                if pname not in self.person_data:
                    continue
                interaction.participants.append(self.make_person_msg(pname, self.person_data[pname]))
            if interaction.participants:
                interactions_msg.interactions.append(interaction)
        state_msg.interactions = interactions_msg

        self.state_pub.publish(state_msg)


if __name__ == "__main__":
    SocialDensityFusionZedNode()
    rospy.spin()
