#!/usr/bin/env python3

import math

import rospy
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from dynamic_reconfigure.client import Client
from nav_msgs.msg import Odometry
from nav_msgs.msg import Path
from rosgraph_msgs.msg import Log
from std_msgs.msg import Bool


EPS = 1e-6


def _ros_param_bool(name, default):
    v = rospy.get_param(name, default)
    return _coerce_bool(v)


def _coerce_bool(v):
    if isinstance(v, bool):
        return v
    if isinstance(v, str):
        return v.strip().lower() in ("1", "true", "yes", "on")
    return bool(v)


class TargetState:
    def __init__(self, layer_name, target_name, server, weight_key, timeout):
        self.layer_name = layer_name
        self.target_name = target_name
        self.server = server
        self.weight_key = weight_key
        self.timeout = timeout
        self.client = None
        self.base_weight = None
        self.current_weight = None

    def ensure_connected(self):
        if self.client is not None:
            return True
        try:
            self.client = Client(self.server, timeout=self.timeout)
            cfg = self.client.get_configuration(timeout=self.timeout)
            if self.weight_key not in cfg:
                raise KeyError("missing key '{}' in {}".format(self.weight_key, self.server))
            self.base_weight = float(cfg[self.weight_key])
            self.current_weight = self.base_weight
            rospy.loginfo(
                "[%s:%s] Connected %s key=%s base=%.3f",
                self.layer_name,
                self.target_name,
                self.server,
                self.weight_key,
                self.base_weight,
            )
            return True
        except Exception as exc:
            rospy.logwarn_throttle(
                5.0,
                "[%s:%s] Waiting dynamic_reconfigure %s (%s)",
                self.layer_name,
                self.target_name,
                self.server,
                str(exc),
            )
            self.client = None
            return False

    def set_weight(self, value):
        if self.client is None:
            return False
        if self.current_weight is not None and math.isclose(self.current_weight, value, abs_tol=EPS):
            return False
        self.client.update_configuration({self.weight_key: value})
        self.current_weight = value
        return True


class LogicalLayer:
    def __init__(self, name, step, min_weight, max_weight, targets):
        self.name = name
        self.step = step
        self.min_weight = min_weight
        self.max_weight = max_weight
        self.targets = targets
        self.base_weight = None
        self.current_weight = None

    def _clamp(self, value):
        return max(self.min_weight, min(self.max_weight, value))

    def ensure_connected(self):
        all_ok = True
        base_candidates = []
        for target in self.targets:
            ok = target.ensure_connected()
            all_ok = all_ok and ok
            if ok and target.base_weight is not None:
                base_candidates.append(target.base_weight)
        if not all_ok or not base_candidates:
            return False
        if self.base_weight is None:
            self.base_weight = self._clamp(min(base_candidates))
            self.current_weight = self.base_weight
            rospy.loginfo(
                "[%s] Logical layer initialized base=%.3f range=[%.3f, %.3f] step=%.3f",
                self.name,
                self.base_weight,
                self.min_weight,
                self.max_weight,
                self.step,
            )
        return True

    def push_current_to_all_targets(self):
        changed = False
        target_weight = self._clamp(self.current_weight)
        for target in self.targets:
            changed = target.set_weight(target_weight) or changed
        self.current_weight = target_weight
        return changed

    def set_to_min(self):
        self.current_weight = self.min_weight
        return self.push_current_to_all_targets()

    def increase_once(self):
        if self.current_weight is None:
            return False
        self.current_weight = self._clamp(self.current_weight + self.step)
        return self.push_current_to_all_targets()

    def decrease_toward_base_once(self):
        if self.current_weight is None or self.base_weight is None:
            return False
        if self.current_weight <= self.base_weight + EPS:
            return False
        self.current_weight = self._clamp(max(self.base_weight, self.current_weight - self.step))
        return self.push_current_to_all_targets()

    def at_max(self):
        if self.current_weight is None:
            return False
        return self.current_weight >= self.max_weight - EPS

    def above_base(self):
        if self.current_weight is None or self.base_weight is None:
            return False
        return self.current_weight > self.base_weight + EPS


class PathDensityWeightController:
    def __init__(self):
        self.plan_topic = rospy.get_param("~plan_topic", "/move_base/GlobalPlanner/plan")
        self.control_rate_hz = rospy.get_param("~control_rate_hz", 2.0)
        self.force_min_base_weight = _ros_param_bool("~force_min_base_weight", True)
        self.no_path_trigger_count = int(rospy.get_param("~no_path_trigger_count", 1))
        self.no_path_grace_after_goal_sec = float(
            rospy.get_param("~no_path_grace_after_goal_sec", 1.5)
        )
        self.reset_to_min_on_start = bool(rospy.get_param("~reset_to_min_on_start", True))
        self.use_teb_footprint_infeasible = _ros_param_bool("~use_teb_footprint_infeasible", True)
        self.teb_footprint_infeasible_topic = rospy.get_param(
            "~teb_footprint_infeasible_topic",
            "/move_base/TebLocalPlannerROS/footprint_trajectory_infeasible",
        )
        self.teb_infeasible_trigger_count = int(rospy.get_param("~teb_infeasible_trigger_count", 2))
        self.move_base_status_topic = rospy.get_param("~move_base_status_topic", "/move_base/status")
        self.use_no_progress_relax = _ros_param_bool("~use_no_progress_relax", True)
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.no_progress_distance_eps = float(rospy.get_param("~no_progress_distance_eps", 0.08))
        self.no_progress_duration_sec = float(rospy.get_param("~no_progress_duration_sec", 3.0))
        self.no_progress_relax_min_interval_sec = float(
            rospy.get_param("~no_progress_relax_min_interval_sec", 1.5)
        )
        self.use_progress_restore = _ros_param_bool("~use_progress_restore", True)
        self.progress_restore_distance_m = float(rospy.get_param("~progress_restore_distance_m", 1.5))
        self.progress_restore_min_interval_sec = float(
            rospy.get_param("~progress_restore_min_interval_sec", 2.0)
        )
        self.use_active_goal_timeout_relax = _ros_param_bool("~use_active_goal_timeout_relax", True)
        self.active_goal_relax_delay_sec = float(
            rospy.get_param("~active_goal_relax_delay_sec", 12.0)
        )
        self.active_goal_relax_min_interval_sec = float(
            rospy.get_param("~active_goal_relax_min_interval_sec", 8.0)
        )
        self.use_teb_oscillation_relax = _ros_param_bool("~use_teb_oscillation_relax", True)
        self.teb_oscillation_log_topic = rospy.get_param("~teb_oscillation_log_topic", "/rosout_agg")
        self.teb_oscillation_trigger_count = int(rospy.get_param("~teb_oscillation_trigger_count", 1))
        self.teb_oscillation_relax_min_interval_sec = float(
            rospy.get_param("~teb_oscillation_relax_min_interval_sec", 2.0)
        )
        self.density_button_state_param = rospy.get_param(
            "~density_button_state_param", "/social_density_fusion_node/density/button_state"
        )
        self.density_info_topic = rospy.get_param("~density_info_topic", "/density_info")
        self.require_density_info_topic = _ros_param_bool("~require_density_info_topic", True)
        self.density_info_topic_timeout_sec = float(
            rospy.get_param("~density_info_topic_timeout_sec", 1.0)
        )
        self.degrade_when_density_unavailable = _ros_param_bool(
            "~degrade_when_density_unavailable", True
        )
        client_timeout = float(rospy.get_param("~client_timeout_sec", 5.0))

        self.no_path_count = 0
        self.teb_infeasible_count = 0
        self.latest_has_path = None
        self._move_base_has_active_goal = False
        self._no_progress_anchor_xy = None
        self._no_progress_anchor_wall_time = None
        self._last_no_progress_relax_wall_time = None
        self._latest_xy = None
        self._progress_restore_anchor_xy = None
        self._last_progress_restore_wall_time = None
        self._active_goal_start_wall_time = None
        self._last_active_goal_timeout_relax_wall_time = None
        self.teb_oscillation_count = 0
        self._last_teb_oscillation_relax_wall_time = None
        self._last_density_info_wall_time = None
        self._density_disabled_applied = False
        self.initialized_once = False
        # Tranh nhap nhay: khong cho giam ngay sau khi vua tang do No Path ngan.
        self.restore_hold_after_relax_sec = float(
            rospy.get_param("~restore_hold_after_relax_sec", 2.5)
        )
        self.restore_min_interval_sec = float(rospy.get_param("~restore_min_interval_sec", 2.0))
        self._last_relax_wall_time = None
        self._last_restore_wall_time = None
        # Lop vua duoc tang (relax) gan nhat: restore uu tien ha tu day ve phia emotion.
        self._last_relax_layer_name = None

        self.layers = [
            LogicalLayer(
                name="emotion",
                step=float(rospy.get_param("~emotion_step", 4.0)),
                min_weight=float(rospy.get_param("~emotion_min_weight", 0.0)),
                max_weight=float(rospy.get_param("~emotion_max_weight", 20.0)),
                targets=[
                    TargetState(
                        "emotion",
                        "global",
                        rospy.get_param("~emotion_server", "/move_base/global_costmap/emotion_layer"),
                        rospy.get_param("~emotion_weight_key", "emotion_density_weight"),
                        client_timeout,
                    ),
                    TargetState(
                        "emotion",
                        "local",
                        rospy.get_param("~emotion_server_local", "/move_base/local_costmap/emotion_layer"),
                        rospy.get_param("~emotion_weight_key_local", "emotion_density_weight"),
                        client_timeout,
                    ),
                ],
            ),
            LogicalLayer(
                name="human_object",
                step=float(rospy.get_param("~human_object_step", 4.0)),
                min_weight=float(rospy.get_param("~human_object_min_weight", 0.0)),
                max_weight=float(rospy.get_param("~human_object_max_weight", 20.0)),
                targets=[
                    TargetState(
                        "human_object",
                        "global",
                        rospy.get_param("~human_object_server", "/move_base/global_costmap/human_object_layer"),
                        rospy.get_param("~human_object_weight_key", "human_object_density_weight"),
                        client_timeout,
                    ),
                    TargetState(
                        "human_object",
                        "local",
                        rospy.get_param("~human_object_server_local", "/move_base/local_costmap/human_object_layer"),
                        rospy.get_param("~human_object_weight_key_local", "human_object_density_weight"),
                        client_timeout,
                    ),
                ],
            ),
            LogicalLayer(
                name="human_group",
                step=float(rospy.get_param("~human_group_step", 20.0)),
                min_weight=float(rospy.get_param("~human_group_min_weight", 0.0)),
                max_weight=float(rospy.get_param("~human_group_max_weight", 40.0)),
                targets=[
                    TargetState(
                        "human_group",
                        "global",
                        rospy.get_param("~human_group_server", "/move_base/global_costmap/human_group_layer"),
                        rospy.get_param("~human_group_weight_key", "human_group_density_weight"),
                        client_timeout,
                    ),
                    TargetState(
                        "human_group",
                        "local",
                        rospy.get_param("~human_group_server_local", "/move_base/local_costmap/human_group_layer"),
                        rospy.get_param("~human_group_weight_key_local", "human_group_density_weight"),
                        client_timeout,
                    ),
                ],
            ),
            LogicalLayer(
                name="proxemic",
                step=float(rospy.get_param("~proxemic_step", 15.0)),
                min_weight=float(rospy.get_param("~proxemic_min_weight", 0.0)),
                max_weight=float(rospy.get_param("~proxemic_max_weight", 45.0)),
                targets=[
                    TargetState(
                        "proxemic",
                        "global",
                        rospy.get_param("~proxemic_server", "/move_base/global_costmap/proxemic"),
                        rospy.get_param("~proxemic_weight_key", "proxemic_density_weight"),
                        client_timeout,
                    ),
                    TargetState(
                        "proxemic",
                        "local",
                        rospy.get_param("~proxemic_server_local", "/move_base/local_costmap/proxemic"),
                        rospy.get_param("~proxemic_weight_key_local", "proxemic_density_weight"),
                        client_timeout,
                    ),
                ],
            ),
        ]

        rospy.Subscriber(self.plan_topic, Path, self.path_callback, queue_size=20)
        if self.use_teb_footprint_infeasible:
            rospy.Subscriber(
                self.teb_footprint_infeasible_topic,
                Bool,
                self.teb_footprint_infeasible_callback,
                queue_size=50,
            )
        rospy.Subscriber(
            self.move_base_status_topic,
            GoalStatusArray,
            self.move_base_status_callback,
            queue_size=20,
        )
        if self.use_no_progress_relax or self.use_progress_restore:
            rospy.Subscriber(self.odom_topic, Odometry, self.odom_callback, queue_size=50)
        if self.use_teb_oscillation_relax:
            rospy.Subscriber(
                self.teb_oscillation_log_topic,
                Log,
                self.teb_oscillation_log_callback,
                queue_size=200,
            )
        if self.density_info_topic:
            rospy.Subscriber(
                self.density_info_topic,
                rospy.AnyMsg,
                self.density_info_callback,
                queue_size=50,
            )
        rospy.Timer(rospy.Duration(1.0 / max(self.control_rate_hz, 0.1)), self.control_loop)
        rospy.loginfo(
            "Path Density Weight Controller started. plan_topic=%s "
            "force_min_base_weight=%s "
            "restore_min_interval_sec=%.3f restore_hold_after_relax_sec=%.3f "
            "use_teb_footprint_infeasible=%s teb_topic=%s teb_infeasible_trigger_count=%d "
            "move_base_status_topic=%s no_path_grace_after_goal_sec=%.3f "
            "use_no_progress_relax=%s odom_topic=%s no_progress_duration_sec=%.3f "
            "use_progress_restore=%s progress_restore_distance_m=%.3f progress_restore_min_interval_sec=%.3f "
            "use_active_goal_timeout_relax=%s active_goal_relax_delay_sec=%.3f active_goal_relax_min_interval_sec=%.3f "
            "use_teb_oscillation_relax=%s teb_oscillation_trigger_count=%d teb_oscillation_relax_min_interval_sec=%.3f "
            "density_info_topic=%s require_density_info_topic=%s density_info_topic_timeout_sec=%.3f "
            "density_button_state_param=%s degrade_when_density_unavailable=%s",
            self.plan_topic,
            self.force_min_base_weight,
            self.restore_min_interval_sec,
            self.restore_hold_after_relax_sec,
            self.use_teb_footprint_infeasible,
            self.teb_footprint_infeasible_topic,
            self.teb_infeasible_trigger_count,
            self.move_base_status_topic,
            self.no_path_grace_after_goal_sec,
            self.use_no_progress_relax,
            self.odom_topic,
            self.no_progress_duration_sec,
            self.use_progress_restore,
            self.progress_restore_distance_m,
            self.progress_restore_min_interval_sec,
            self.use_active_goal_timeout_relax,
            self.active_goal_relax_delay_sec,
            self.active_goal_relax_min_interval_sec,
            self.use_teb_oscillation_relax,
            self.teb_oscillation_trigger_count,
            self.teb_oscillation_relax_min_interval_sec,
            self.density_info_topic,
            self.require_density_info_topic,
            self.density_info_topic_timeout_sec,
            self.density_button_state_param,
            self.degrade_when_density_unavailable,
        )

    def path_callback(self, msg):
        has_path = bool(msg.poses)
        if has_path:
            self.no_path_count = 0
        else:
            self.no_path_count += 1
        self.latest_has_path = has_path
        self.evaluate_transition()

    def teb_footprint_infeasible_callback(self, msg):
        if not msg.data:
            self.teb_infeasible_count = 0
        else:
            self.teb_infeasible_count += 1
        self.evaluate_transition()

    def density_info_callback(self, _msg):
        self._last_density_info_wall_time = rospy.Time.now().to_sec()
        self.evaluate_transition()

    def move_base_status_callback(self, msg):
        prev_active = self._move_base_has_active_goal
        self._move_base_has_active_goal = any(
            status.status in (GoalStatus.PENDING, GoalStatus.ACTIVE)
            for status in msg.status_list
        )
        now_wall = rospy.Time.now().to_sec()
        if self._move_base_has_active_goal and not prev_active:
            self._active_goal_start_wall_time = now_wall
        if not self._move_base_has_active_goal:
            self._no_progress_anchor_xy = None
            self._no_progress_anchor_wall_time = None
            self._progress_restore_anchor_xy = None
            self._active_goal_start_wall_time = None
            self._last_relax_layer_name = None

    def odom_callback(self, msg):
        if not (self.use_no_progress_relax or self.use_progress_restore):
            return
        now_wall = rospy.Time.now().to_sec()
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        # progress_restore: can odom ngay khi con goal, khong phu thuoc global plan co path hay khong.
        if self._move_base_has_active_goal and self.use_progress_restore:
            self._latest_xy = (x, y)
            if self._progress_restore_anchor_xy is None:
                self._progress_restore_anchor_xy = (x, y)

        if not self.use_no_progress_relax:
            return
        if not self._move_base_has_active_goal or not self.latest_has_path:
            self._no_progress_anchor_xy = None
            self._no_progress_anchor_wall_time = None
            return
        if self._no_progress_anchor_xy is None:
            self._no_progress_anchor_xy = (x, y)
            self._no_progress_anchor_wall_time = now_wall
            return
        dx = x - self._no_progress_anchor_xy[0]
        dy = y - self._no_progress_anchor_xy[1]
        if math.hypot(dx, dy) > self.no_progress_distance_eps:
            self._no_progress_anchor_xy = (x, y)
            self._no_progress_anchor_wall_time = now_wall

    def _has_any_layer_above_base(self):
        return any(layer.above_base() for layer in self.layers)

    def teb_oscillation_log_callback(self, msg):
        if not self.use_teb_oscillation_relax:
            return
        if msg.level < Log.WARN:
            return
        text = (msg.msg or "").lower()
        if "teblocalplannerros" not in text or "possible oscillation" not in text:
            return
        self.teb_oscillation_count += 1
        self.evaluate_transition()

    def _no_progress_relax_allows(self):
        if not self.use_no_progress_relax:
            return False
        if not self._move_base_has_active_goal or not self.latest_has_path:
            return False
        if self._no_progress_anchor_wall_time is None:
            return False
        now_wall = rospy.Time.now().to_sec()
        if now_wall - self._no_progress_anchor_wall_time < self.no_progress_duration_sec:
            return False
        if self._last_no_progress_relax_wall_time is not None:
            if (
                now_wall - self._last_no_progress_relax_wall_time
                < self.no_progress_relax_min_interval_sec
            ):
                return False
        return True

    def _active_goal_timeout_relax_allows(self):
        if not self.use_active_goal_timeout_relax:
            return False
        if not self._move_base_has_active_goal or not self.latest_has_path:
            return False
        if self._active_goal_start_wall_time is None:
            return False
        now_wall = rospy.Time.now().to_sec()
        if now_wall - self._active_goal_start_wall_time < self.active_goal_relax_delay_sec:
            return False
        if self._last_active_goal_timeout_relax_wall_time is not None:
            if (
                now_wall - self._last_active_goal_timeout_relax_wall_time
                < self.active_goal_relax_min_interval_sec
            ):
                return False
        return True

    def _teb_oscillation_relax_allows(self):
        if not self.use_teb_oscillation_relax:
            return False
        if self.teb_oscillation_count < self.teb_oscillation_trigger_count:
            return False
        now_wall = rospy.Time.now().to_sec()
        if self._last_teb_oscillation_relax_wall_time is not None:
            if (
                now_wall - self._last_teb_oscillation_relax_wall_time
                < self.teb_oscillation_relax_min_interval_sec
            ):
                return False
        return True

    def _progress_restore_allows(self):
        if not self.use_progress_restore:
            return False
        if not self._move_base_has_active_goal:
            return False
        if not self._has_any_layer_above_base():
            return False
        if self._latest_xy is None:
            return False
        if self._progress_restore_anchor_xy is None:
            self._progress_restore_anchor_xy = self._latest_xy
            return False
        dx = self._latest_xy[0] - self._progress_restore_anchor_xy[0]
        dy = self._latest_xy[1] - self._progress_restore_anchor_xy[1]
        if math.hypot(dx, dy) < self.progress_restore_distance_m:
            return False
        now_wall = rospy.Time.now().to_sec()
        if self._last_progress_restore_wall_time is not None:
            if now_wall - self._last_progress_restore_wall_time < self.progress_restore_min_interval_sec:
                return False
        return True

    def _no_path_relax_allows(self):
        if self.no_path_count < self.no_path_trigger_count:
            return False
        # Khong tang weight trong khoang "khong co goal" (vua cham dich / dang chuyen goal).
        if not self._move_base_has_active_goal:
            return False
        if self.no_path_grace_after_goal_sec <= 0:
            return True
        if self._active_goal_start_wall_time is None:
            return True
        now_wall = rospy.Time.now().to_sec()
        return (now_wall - self._active_goal_start_wall_time) >= self.no_path_grace_after_goal_sec

    def _density_info_available(self):
        if not self.degrade_when_density_unavailable:
            return True
        if self.density_info_topic:
            if self._last_density_info_wall_time is None:
                return not self.require_density_info_topic
            if self.density_info_topic_timeout_sec > 0:
                now_wall = rospy.Time.now().to_sec()
                if now_wall - self._last_density_info_wall_time > self.density_info_topic_timeout_sec:
                    return False
            return True
        if not rospy.has_param(self.density_button_state_param):
            return True
        try:
            return _coerce_bool(rospy.get_param(self.density_button_state_param))
        except Exception:
            return True

    def control_loop(self, _event):
        # Keep retrying initialization even if no plan messages arrive yet.
        self.evaluate_transition()

    def evaluate_transition(self):
        if not all(layer.ensure_connected() for layer in self.layers):
            return
        if not self.initialized_once:
            self.initialized_once = True
            if self.force_min_base_weight:
                for layer in self.layers:
                    layer.base_weight = layer.min_weight
            if self.reset_to_min_on_start:
                for layer in self.layers:
                    layer.base_weight = layer.min_weight
                    layer.set_to_min()
                rospy.logwarn("[INIT] Reset all density weights to layer min values.")
        if self.latest_has_path is None:
            return

        relax_reasons = []
        should_relax = False
        density_available = self._density_info_available()
        if not density_available:
            if not self._density_disabled_applied:
                for layer in self.layers:
                    target_weight = layer.base_weight if layer.base_weight is not None else layer.min_weight
                    layer.current_weight = layer._clamp(target_weight)
                    layer.push_current_to_all_targets()
                self._density_disabled_applied = True
                self._last_relax_layer_name = None
                rospy.logwarn(
                    "[DENSITY] no fresh data on %s (timeout=%.2fs) -> controller disabled, restore all weights to base.",
                    self.density_info_topic,
                    self.density_info_topic_timeout_sec,
                )
            self.no_path_count = 0
            self.teb_infeasible_count = 0
            self.teb_oscillation_count = 0
            return
        if self._density_disabled_applied:
            self._density_disabled_applied = False
            rospy.loginfo(
                "[DENSITY] fresh data detected on %s -> controller re-enabled.",
                self.density_info_topic,
            )
        if self._no_path_relax_allows():
            should_relax = True
            relax_reasons.append("no_path")
            self.no_path_count = 0
        if (
            self.use_teb_footprint_infeasible
            and self.teb_infeasible_count >= self.teb_infeasible_trigger_count
        ):
            should_relax = True
            relax_reasons.append("teb_footprint_infeasible")
            self.teb_infeasible_count = 0
        if self._no_progress_relax_allows():
            should_relax = True
            relax_reasons.append("no_progress")
            now_wall = rospy.Time.now().to_sec()
            self._last_no_progress_relax_wall_time = now_wall
            self._no_progress_anchor_wall_time = now_wall
        if self._active_goal_timeout_relax_allows():
            should_relax = True
            relax_reasons.append("active_goal_timeout")
            self._last_active_goal_timeout_relax_wall_time = rospy.Time.now().to_sec()
        if self._teb_oscillation_relax_allows():
            should_relax = True
            relax_reasons.append("teb_oscillation")
            self.teb_oscillation_count = 0
            self._last_teb_oscillation_relax_wall_time = rospy.Time.now().to_sec()
        if should_relax:
            self.relax_social_constraints_once("+".join(relax_reasons))
            return

        if self._progress_restore_allows() and self._restore_guard_allows_progress_restore():
            if self.restore_social_constraints_once("progress_restore"):
                now_wall = rospy.Time.now().to_sec()
                self._last_progress_restore_wall_time = now_wall
                self._progress_restore_anchor_xy = self._latest_xy

    def _restore_guard_allows_progress_restore(self):
        """progress_restore: khong doi path on dinh (odom da la tin hieu tien do)."""
        now_wall = rospy.Time.now().to_sec()
        if self._last_restore_wall_time is not None and self.restore_min_interval_sec > 0:
            if now_wall - self._last_restore_wall_time < self.restore_min_interval_sec:
                return False
        if self._last_relax_wall_time is not None and self.restore_hold_after_relax_sec > 0:
            if now_wall - self._last_relax_wall_time < self.restore_hold_after_relax_sec:
                return False
        return True

    def relax_social_constraints_once(self, reason_tag="no_path"):
        layer_by_name = {layer.name: layer for layer in self.layers}
        # Tang lan luot: moi lan relax chi nang mot layer (theo thu tu relax_order).
        # Trigger TEB van dung cung thu tu, khong nang gap doi hai layer trong mot lan goi.
        relax_order = [layer.name for layer in self.layers]

        label = {
            "no_path": "[NO PATH]",
            "teb_footprint_infeasible": "[TEB FOOTPRINT INFEASIBLE]",
            "no_progress": "[NO PROGRESS]",
            "active_goal_timeout": "[ACTIVE GOAL TIMEOUT]",
            "teb_oscillation": "[TEB OSCILLATION]",
            "no_path+teb_footprint_infeasible": "[NO PATH + TEB FOOTPRINT INFEASIBLE]",
            "no_path+no_progress": "[NO PATH + NO PROGRESS]",
            "no_path+active_goal_timeout": "[NO PATH + ACTIVE GOAL TIMEOUT]",
            "teb_footprint_infeasible+no_progress": "[TEB FOOTPRINT INFEASIBLE + NO PROGRESS]",
            "teb_footprint_infeasible+active_goal_timeout": "[TEB FOOTPRINT INFEASIBLE + ACTIVE GOAL TIMEOUT]",
            "no_progress+active_goal_timeout": "[NO PROGRESS + ACTIVE GOAL TIMEOUT]",
            "no_path+teb_footprint_infeasible+no_progress+active_goal_timeout": "[NO PATH + TEB FOOTPRINT INFEASIBLE + NO PROGRESS + ACTIVE GOAL TIMEOUT]",
            "no_path+teb_footprint_infeasible+no_progress": "[NO PATH + TEB FOOTPRINT INFEASIBLE + NO PROGRESS]",
        }.get(reason_tag, "[RELAX]")

        max_increases = 1
        increases_done = 0
        for layer_name in relax_order:
            layer = layer_by_name.get(layer_name)
            if layer is None:
                continue
            if not layer.at_max():
                changed = layer.increase_once()
                if changed:
                    self._last_relax_layer_name = layer.name
                    self._last_relax_wall_time = rospy.Time.now().to_sec()
                    rospy.logwarn(
                        "%s Increase %s -> %.3f (global+local together) reasons=%s",
                        label,
                        layer.name,
                        layer.current_weight,
                        reason_tag,
                    )
                    increases_done += 1
                    if increases_done >= max_increases:
                        return
        if increases_done > 0:
            return
        rospy.logwarn_throttle(
            5.0,
            "[%s] All layers already at configured max weights.",
            reason_tag,
        )

    def restore_social_constraints_once(self, reason_tag="progress_restore"):
        # Giam tu "moc" relax gan nhat tro ve phia emotion; moi lan goi chi giam **mot buoc step**
        # (doi xung voi relax: moi lan chi tang mot buoc).
        layer_by_name = {layer.name: layer for layer in self.layers}
        relax_order = [layer.name for layer in self.layers]
        if self._last_relax_layer_name in layer_by_name:
            start_idx = relax_order.index(self._last_relax_layer_name)
            restore_frontier = self._last_relax_layer_name
        else:
            start_idx = len(relax_order) - 1
            restore_frontier = relax_order[start_idx]
        for k in range(start_idx, -1, -1):
            name = relax_order[k]
            layer = layer_by_name.get(name)
            if layer is None or not layer.above_base():
                continue
            if not layer.decrease_toward_base_once():
                rospy.logwarn_throttle(
                    5.0,
                    "[RESTORE:%s] Could not decrease layer %s toward base (stuck at %.3f)",
                    reason_tag,
                    layer.name,
                    layer.current_weight,
                )
                return False
            now_wall = rospy.Time.now().to_sec()
            self._last_restore_wall_time = now_wall
            if not any(ly.above_base() for ly in self.layers):
                self._last_relax_layer_name = None
            rospy.loginfo(
                "[RESTORE:%s] Decrease %s -> %.3f (1 step, start=%s, global+local together)",
                reason_tag,
                layer.name,
                layer.current_weight,
                restore_frontier,
            )
            return True
        return False


def main():
    rospy.init_node("path_density_weight_controller", anonymous=False)
    try:
        _controller = PathDensityWeightController()
    except Exception as exc:
        rospy.logerr("Failed to initialize path_density_weight_controller: %s", str(exc))
        raise
    rospy.spin()


if __name__ == "__main__":
    main()
