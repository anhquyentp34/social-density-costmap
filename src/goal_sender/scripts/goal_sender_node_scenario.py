#!/usr/bin/env python3
"""
Goal sender với kịch bản waypoints (easy/medium/hard) và file YAML.
Dùng cho thí nghiệm so sánh Diffbot vs x_omni4wd; không sửa goal_sender_node.py (dùng ở dự án khác).

Tham số:
  ~scenario (str): easy | medium | hard → load config/waypoints_<scenario>.yaml
  ~waypoints_file (str): đường dẫn tuyệt đối tới file YAML (ưu tiên hơn scenario)
  ~num_cycles (int): số chu kỳ lặp waypoints
  ~advance_only_on_succeeded (bool, mặc định True): chỉ tăng waypoint khi SUCCEEDED
  ~verify_pose_on_succeed (bool, mặc định True): thêm kiểm tra AMCL trong goal_reach_xy_tol
  ~goal_reach_xy_tol (float, mặc định 0.35): ngưỡng khoảng cách (m) coi như đạt mục tiêu
  ~spin_on_arrival (bool, mặc định False): NGOÀI LỀ — quay tại chỗ khi tới goal (exp7);
    tắt mặc định nên không ảnh hưởng thí nghiệm 1–6
  ~spin_angular_vel / ~spin_revolutions: chỉ dùng khi spin_on_arrival=true

Publish (latch):
  /goal_sender/last_succeeded_waypoint (std_msgs/String): tên waypoint vừa SUCCEEDED
"""

import os
import rospy
import actionlib
import yaml
import math
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped, PoseWithCovarianceStamped
import tf
from actionlib_msgs.msg import GoalStatus
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA, String

# Waypoints mặc định (trung bình) khi không load được file
DEFAULT_WAYPOINTS = [
    {'name': 'GOAL_1', 'x': -2.0, 'y': 7.0, 'yaw': -1.57},
    {'name': 'GOAL_2', 'x': -4.5, 'y': -2.0, 'yaw': 0.0},
    {'name': 'GOAL_3', 'x': 10.0, 'y': -1.0, 'yaw': 3.14},
    {'name': 'GOAL_4', 'x': 3.0, 'y': 7.0, 'yaw': -1.57},
    {'name': 'GOAL_5', 'x': 3.0, 'y': 2.0, 'yaw': 1.57},
    {'name': 'GOAL_6', 'x': -10.0, 'y': -7.0, 'yaw': 1.57},
]


def load_waypoints_from_yaml(filepath):
    """Đọc waypoints từ file YAML. Trả về (list dict, num_cycles hoặc None)."""
    if not filepath or not os.path.isfile(filepath):
        return None, None
    try:
        with open(filepath, 'r') as f:
            data = yaml.safe_load(f)
        num_cycles = None
        if isinstance(data, dict):
            if 'num_cycles' in data:
                num_cycles = int(data['num_cycles'])
            wp_list = data.get('waypoints', data)
        else:
            wp_list = data
        if not wp_list:
            return None, num_cycles
        waypoints = [
            {
                'name': str(w.get('name', 'GOAL_{}'.format(i + 1))),
                'x': float(w['x']),
                'y': float(w['y']),
                'yaw': float(w['yaw']),
            }
            for i, w in enumerate(wp_list)
        ]
        return waypoints, num_cycles
    except Exception as e:
        rospy.logwarn("load_waypoints_from_yaml failed: %s", e)
        return None, None


def get_waypoints_config_path(scenario):
    """Trả về đường dẫn file config waypoints theo scenario (easy/medium/hard)."""
    try:
        import rospkg
        pkg_path = rospkg.RosPack().get_path('goal_sender')
        return os.path.join(pkg_path, 'config', 'waypoints_{}.yaml'.format(scenario.lower()))
    except Exception:
        return None


class GoalSenderScenario:
    def __init__(self):
        rospy.init_node('goal_sender_node_scenario')
        rospy.loginfo("Goal sender (scenario) node initialized")

        scenario = rospy.get_param('~scenario', 'medium')
        waypoints_file = rospy.get_param('~waypoints_file', '')
        self.num_cycles = int(rospy.get_param('~num_cycles', 1))
        yaml_num_cycles = None

        self.waypoints = None
        if waypoints_file:
            self.waypoints, yaml_num_cycles = load_waypoints_from_yaml(waypoints_file)
            if self.waypoints:
                rospy.loginfo("Loaded %d waypoints from file: %s", len(self.waypoints), waypoints_file)
        if self.waypoints is None:
            config_path = get_waypoints_config_path(scenario)
            if config_path:
                self.waypoints, yaml_num_cycles = load_waypoints_from_yaml(config_path)
            if self.waypoints:
                rospy.loginfo("Loaded %d waypoints for scenario '%s'", len(self.waypoints), scenario)
        if self.waypoints is None:
            self.waypoints = DEFAULT_WAYPOINTS
            rospy.loginfo("Using default (medium) waypoints, count=%d", len(self.waypoints))
        if yaml_num_cycles is not None:
            self.num_cycles = yaml_num_cycles
            rospy.loginfo("Using num_cycles=%d from waypoints YAML", self.num_cycles)

        self.current_cycle = 0
        self.current_goal_index = 0
        self.goal_active = False
        self.last_goal_id = None
        self.active_goal_timeout = float(rospy.get_param('~active_goal_timeout', 0.0))
        self.active_goal_start_time = None  # thời điểm goal chuyển sang ACTIVE
        # Chỉ bỏ goal ABORTED khi proxemic đã nới tới ngưỡng max.
        self.abort_only_when_proxemic_max = bool(
            rospy.get_param('~abort_only_when_proxemic_max', True)
        )
        self.proxemic_max_weight = float(rospy.get_param('~proxemic_max_weight', 50.0))
        self.proxemic_max_tolerance = float(rospy.get_param('~proxemic_max_tolerance', 1e-3))
        self.proxemic_weight_param_global = rospy.get_param(
            '~proxemic_weight_param_global',
            '/move_base/global_costmap/proxemic/proxemic_density_weight',
        )
        self.proxemic_weight_param_local = rospy.get_param(
            '~proxemic_weight_param_local',
            '/move_base/local_costmap/proxemic/proxemic_density_weight',
        )
        self.abort_retry_delay_sec = float(rospy.get_param('~abort_retry_delay_sec', 0.5))
        # Chỉ chuyển goal tiếp theo khi SUCCEEDED (+ tùy chọn kiểm tra pose).
        # Tránh bỏ qua waypoint khi ABORTED / PREEMPTED / REJECTED / RECALLED.
        self.advance_only_on_succeeded = bool(
            rospy.get_param('~advance_only_on_succeeded', True)
        )
        self.verify_pose_on_succeed = bool(
            rospy.get_param('~verify_pose_on_succeed', True)
        )
        self.goal_reach_xy_tol = float(rospy.get_param('~goal_reach_xy_tol', 0.35))
        self.goal_retry_delay_sec = float(
            rospy.get_param('~goal_retry_delay_sec', self.abort_retry_delay_sec)
        )
        self._robot_x = None
        self._robot_y = None
        self._advancing = False  # chặn status_callback trùng khi đang sleep/gửi goal

        # Báo hiệu tới nơi: quay tại chỗ N vòng (đánh dấu vị trí actor / khảo sát)
        self.spin_on_arrival = bool(rospy.get_param('~spin_on_arrival', False))
        self.spin_angular_vel = float(rospy.get_param('~spin_angular_vel', 0.7))
        self.spin_revolutions = float(rospy.get_param('~spin_revolutions', 1.0))
        self.cmd_vel_topic = str(rospy.get_param('~cmd_vel_topic', '/cmd_vel'))
        self._cmd_vel_pub = None
        if self.spin_on_arrival:
            from geometry_msgs.msg import Twist
            self._Twist = Twist
            self._cmd_vel_pub = rospy.Publisher(self.cmd_vel_topic, Twist, queue_size=1)
            rospy.loginfo(
                "spin_on_arrival=ON: %.2f vòng @ %.2f rad/s trên %s",
                self.spin_revolutions, self.spin_angular_vel, self.cmd_vel_topic,
            )

        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        self.marker_pub = rospy.Publisher('/visualization_markers_start_goal', MarkerArray, queue_size=10)
        # Latch: fusion/nodes khác biết waypoint vừa SUCCEEDED (vd. arm density gate exp6).
        self.last_succeeded_pub = rospy.Publisher(
            '/goal_sender/last_succeeded_waypoint', String, queue_size=1, latch=True
        )
        rospy.loginfo("Publisher for markers initialized")

        rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, self._amcl_pose_callback, queue_size=1)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        rospy.sleep(1.0)
        self.send_next_goal()

    def _amcl_pose_callback(self, msg):
        self._robot_x = msg.pose.pose.position.x
        self._robot_y = msg.pose.pose.position.y

    def _distance_to_current_waypoint(self):
        if self._robot_x is None or self.current_goal_index >= len(self.waypoints):
            return None
        wp = self.waypoints[self.current_goal_index]
        return math.hypot(self._robot_x - wp['x'], self._robot_y - wp['y'])

    def _pose_reached_current_goal(self):
        """True nếu AMCL nằm trong goal_reach_xy_tol của waypoint hiện tại."""
        if not self.verify_pose_on_succeed:
            return True
        dist = self._distance_to_current_waypoint()
        if dist is None:
            # Chưa có AMCL: tin status SUCCEEDED của move_base.
            return True
        return dist <= self.goal_reach_xy_tol

    def _spin_in_place(self, wp_name=""):
        """Quay tại chỗ spin_revolutions vòng để báo hiệu đã tới waypoint."""
        if not self.spin_on_arrival or self._cmd_vel_pub is None:
            return
        omega = abs(self.spin_angular_vel)
        if omega < 1e-3:
            return
        duration = abs(self.spin_revolutions) * 2.0 * math.pi / omega
        rospy.loginfo(
            "🔔 Tới %s — quay tại chỗ %.1f vòng (%.1fs) để đánh dấu.",
            wp_name or "goal", self.spin_revolutions, duration,
        )
        # Hủy move_base để không tranh /cmd_vel khi quay
        try:
            self.move_base_client.cancel_all_goals()
        except Exception:
            pass
        rate = rospy.Rate(20)
        tw = self._Twist()
        tw.angular.z = math.copysign(omega, self.spin_angular_vel if self.spin_angular_vel != 0 else 1.0)
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if elapsed >= duration:
                break
            self._cmd_vel_pub.publish(tw)
            rate.sleep()
        stop = self._Twist()
        for _ in range(5):
            self._cmd_vel_pub.publish(stop)
            rate.sleep()
        rospy.loginfo("🔔 Quay xong tại %s — tiếp tục goal kế.", wp_name or "goal")

    def _finish_current_and_send_next(self, reason):
        """Tăng index và gửi goal tiếp theo (chỉ gọi khi đã đạt mục tiêu)."""
        if self._advancing:
            return
        self._advancing = True
        try:
            wp = self.waypoints[self.current_goal_index] if self.current_goal_index < len(self.waypoints) else {}
            wp_name = str(wp.get('name', '') or '')
            rospy.loginfo(
                "Goal %d đạt mục tiêu (%s) -> gửi goal tiếp theo.",
                self.current_goal_index + 1, reason
            )
            if wp_name:
                self.last_succeeded_pub.publish(String(data=wp_name))
                rospy.loginfo("Published last_succeeded_waypoint: %s", wp_name)
            self.goal_active = False
            self.last_goal_id = None
            self.active_goal_start_time = None
            self._spin_in_place(wp_name)
            self.current_goal_index += 1
            rospy.sleep(self.goal_retry_delay_sec)
            self.send_next_goal()
        finally:
            self._advancing = False

    def _retry_current_goal(self, reason):
        """Gửi lại cùng waypoint — không tăng index."""
        if self._advancing:
            return
        self._advancing = True
        try:
            rospy.logwarn(
                "Goal %d chưa đạt (%s) -> gửi lại cùng goal.",
                self.current_goal_index + 1, reason
            )
            self.goal_active = False
            self.last_goal_id = None
            self.active_goal_start_time = None
            rospy.sleep(self.goal_retry_delay_sec)
            self.send_next_goal()
        finally:
            self._advancing = False

    def _read_proxemic_weights(self):
        values = []
        for key in (self.proxemic_weight_param_global, self.proxemic_weight_param_local):
            if rospy.has_param(key):
                try:
                    values.append(float(rospy.get_param(key)))
                except Exception:
                    pass
        return values

    def _proxemic_reached_max(self):
        values = self._read_proxemic_weights()
        if not values:
            # Nếu chưa đọc được param, không chặn luồng bình thường.
            return True
        threshold = self.proxemic_max_weight - self.proxemic_max_tolerance
        return min(values) >= threshold

    def send_next_goal(self):
        if self.current_goal_index < len(self.waypoints):
            wp = self.waypoints[self.current_goal_index]
            goal_pose = PoseStamped()
            goal_pose.header.frame_id = "map"
            goal_pose.header.stamp = rospy.Time.now()
            goal_pose.pose.position.x = wp['x']
            goal_pose.pose.position.y = wp['y']
            goal_pose.pose.position.z = 0.0
            quaternion = tf.transformations.quaternion_from_euler(0, 0, wp['yaw'])
            goal_pose.pose.orientation.x = quaternion[0]
            goal_pose.pose.orientation.y = quaternion[1]
            goal_pose.pose.orientation.z = quaternion[2]
            goal_pose.pose.orientation.w = quaternion[3]

            goal = MoveBaseGoal()
            goal.target_pose = goal_pose

            rospy.loginfo(
                "Sending goal %d/%d (Cycle %d/%d): %s at (%.2f, %.2f, %.2f)",
                self.current_goal_index + 1, len(self.waypoints),
                self.current_cycle + 1, self.num_cycles,
                wp['name'], wp['x'], wp['y'], wp['yaw']
            )
            self.move_base_client.send_goal(goal)
            # Không dùng gh nội bộ (dễ lỗi với một số bản actionlib/robot). Sẽ nhận goal_id từ /move_base/status (PENDING/ACTIVE).
            self.last_goal_id = None
            self.goal_active = True
            self.active_goal_start_time = None
            self.publish_current_goal_marker(wp)
        else:
            self.current_cycle += 1
            if self.current_cycle < self.num_cycles:
                rospy.loginfo("Completed cycle %d/%d, restarting waypoints.", self.current_cycle, self.num_cycles)
                self.current_goal_index = 0
                self.send_next_goal()
            else:
                rospy.loginfo("All cycles completed. Shutting down node.")
                rospy.signal_shutdown("All goals sent.")

    def status_callback(self, msg):
        """Theo dõi /move_base/status.

        Mặc định chỉ chuyển sang waypoint tiếp theo khi SUCCEEDED và (tùy chọn)
        pose AMCL nằm trong goal_reach_xy_tol. Các trạng thái kết thúc khác
        (PREEMPTED/ABORTED/REJECTED/RECALLED) → gửi lại cùng goal.
        """
        if not self.goal_active or self._advancing or not msg.status_list:
            return
        now = rospy.Time.now()
        for status in msg.status_list:
            if self.last_goal_id is None and status.status in [
                GoalStatus.PENDING, GoalStatus.ACTIVE
            ]:
                self.last_goal_id = status.goal_id.id
                if status.status == GoalStatus.ACTIVE:
                    self.active_goal_start_time = now
            if self.last_goal_id and status.goal_id.id == self.last_goal_id:
                if status.status == GoalStatus.ACTIVE:
                    if self.active_goal_start_time is None:
                        self.active_goal_start_time = now
                    if self.active_goal_timeout > 0 and self.active_goal_start_time:
                        elapsed = (now - self.active_goal_start_time).to_sec()
                        if elapsed >= self.active_goal_timeout:
                            # Timeout: chỉ coi là đạt nếu pose gần waypoint; không thì retry.
                            if self._pose_reached_current_goal():
                                self._finish_current_and_send_next(
                                    "active_goal_timeout + pose OK ({:.0f}s)".format(elapsed)
                                )
                            else:
                                dist = self._distance_to_current_waypoint()
                                self._retry_current_goal(
                                    "ACTIVE timeout {:.0f}s, dist={}".format(
                                        elapsed,
                                        "{:.2f}m".format(dist) if dist is not None else "n/a",
                                    )
                                )
                            return
                elif status.status == GoalStatus.SUCCEEDED:
                    if self._pose_reached_current_goal():
                        dist = self._distance_to_current_waypoint()
                        self._finish_current_and_send_next(
                            "SUCCEEDED, dist={}".format(
                                "{:.2f}m".format(dist) if dist is not None else "n/a"
                            )
                        )
                    else:
                        dist = self._distance_to_current_waypoint()
                        self._retry_current_goal(
                            "SUCCEEDED nhưng pose xa (dist={:.2f}m > {:.2f}m)".format(
                                dist if dist is not None else -1.0,
                                self.goal_reach_xy_tol,
                            )
                        )
                    return
                elif status.status in [
                    GoalStatus.PREEMPTED,
                    GoalStatus.ABORTED,
                    GoalStatus.REJECTED,
                    GoalStatus.RECALLED,
                ]:
                    if not self.advance_only_on_succeeded:
                        # Hành vi cũ: bỏ qua goal khi kết thúc không thành công.
                        self._finish_current_and_send_next(
                            "status={} (advance_only_on_succeeded=false)".format(status.status)
                        )
                        return
                    if status.status == GoalStatus.ABORTED and self.abort_only_when_proxemic_max:
                        if not self._proxemic_reached_max():
                            values = self._read_proxemic_weights()
                            self._retry_current_goal(
                                "ABORTED, proxemic chua max (global/local={} < {:.3f})".format(
                                    ",".join("{:.3f}".format(v) for v in values) if values else "n/a",
                                    self.proxemic_max_weight,
                                )
                            )
                            return
                    self._retry_current_goal(
                        "status={} (chua SUCCEEDED)".format(status.status)
                    )
                    return
                break

    def publish_current_goal_marker(self, wp):
        marker_array = MarkerArray()
        marker = Marker()
        marker.header.frame_id = "map"
        marker.header.stamp = rospy.Time.now()
        marker.ns = "current_goal"
        marker.id = 0
        marker.type = Marker.SPHERE
        marker.action = Marker.ADD
        marker.pose.position.x = wp['x']
        marker.pose.position.y = wp['y']
        marker.pose.position.z = 0.0
        marker.scale.x = 0.4
        marker.scale.y = 0.4
        marker.scale.z = 0.4
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
        text_marker = Marker()
        text_marker.header.frame_id = "map"
        text_marker.header.stamp = rospy.Time.now()
        text_marker.ns = "current_goal_text"
        text_marker.id = 1
        text_marker.type = Marker.TEXT_VIEW_FACING
        text_marker.action = Marker.ADD
        text_marker.pose.position.x = wp['x']
        text_marker.pose.position.y = wp['y']
        text_marker.pose.position.z = 0.6
        text_marker.scale.z = 0.4
        text_marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)
        text_marker.text = wp['name']
        text_marker.pose.orientation.w = 1.0
        marker_array.markers.append(text_marker)
        self.marker_pub.publish(marker_array)


if __name__ == '__main__':
    try:
        GoalSenderScenario()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
