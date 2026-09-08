#!/usr/bin/env python3
"""
Tự động kiểm tra trạng thái robot: phát hiện khi robot đang không chạy (stuck) hoặc thiếu dữ liệu.
- Không có /odom hoặc /cmd_vel: báo topic không hoạt động.
- move_base ACTIVE + có cmd_vel nhưng odom gần như không đổi: báo robot không chuyển động (stuck).
Publish kết quả lên /robot_state_check/status (String) và log.
"""
import rospy
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
from actionlib_msgs.msg import GoalStatusArray
from std_msgs.msg import String
import math

# Trạng thái
STATE_OK = "OK"
STATE_NO_ODOM = "NO_ODOM"
STATE_NO_CMD_VEL_TOPIC = "NO_CMD_VEL_TOPIC"
STATE_ROBOT_NOT_MOVING = "ROBOT_NOT_MOVING"
STATE_MOVE_BASE_INACTIVE = "MOVE_BASE_NO_ACTIVE_GOAL"


class RobotStateChecker:
    def __init__(self):
        rospy.init_node("robot_state_checker", anonymous=False)

        self.odom_received = False
        self.cmd_vel_received = False
        self.last_odom_pos = None
        self.last_odom_time = None
        self.last_cmd_vel_time = None
        self.last_cmd_vel_linear = 0.0
        self.last_cmd_vel_angular = 0.0
        self.move_base_status = None  # 0=PENDING, 1=ACTIVE, 2=PREEMPTED, 3=SUCCEEDED, 4=ABORTED, ...

        # Tham số
        self.check_interval = rospy.get_param("~check_interval", 2.0)
        self.odom_timeout = rospy.get_param("~odom_timeout", 1.0)
        self.cmd_vel_timeout = rospy.get_param("~cmd_vel_timeout", 2.0)
        self.min_distance_moved = rospy.get_param("~min_distance_moved", 0.05)
        self.cmd_vel_active_threshold = rospy.get_param("~cmd_vel_active_threshold", 0.05)
        self.stuck_check_duration = rospy.get_param("~stuck_check_duration", 5.0)

        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")

        self.stuck_odom_pos_at_start = None
        self.stuck_check_start_time = None

        self.state_pub = rospy.Publisher("/robot_state_check/status", String, queue_size=1, latch=True)
        self.state_pub.publish(String(data=STATE_OK))

        rospy.Subscriber(self.odom_topic, Odometry, self.odom_cb)
        rospy.Subscriber(self.cmd_vel_topic, Twist, self.cmd_vel_cb)
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_cb)

        rospy.Timer(rospy.Duration(self.check_interval), self.check_cb)
        rospy.loginfo(
            "robot_state_checker: interval=%.1fs, min_distance_moved=%.2fm, stuck_duration=%.1fs",
            self.check_interval, self.min_distance_moved, self.stuck_check_duration,
        )

    def odom_cb(self, msg):
        self.odom_received = True
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        self.last_odom_pos = (x, y)
        self.last_odom_time = rospy.Time.now()

    def cmd_vel_cb(self, msg):
        self.cmd_vel_received = True
        self.last_cmd_vel_time = rospy.Time.now()
        self.last_cmd_vel_linear = math.hypot(msg.linear.x, msg.linear.y)
        self.last_cmd_vel_angular = abs(msg.angular.z)

    def status_cb(self, msg):
        if msg.status_list:
            self.move_base_status = msg.status_list[0].status

    def check_cb(self, event):
        now = rospy.Time.now()
        state = STATE_OK
        reason = ""

        # 1) Kiểm tra topic không có dữ liệu
        if not self.odom_received:
            state = STATE_NO_ODOM
            reason = "Chua nhan /odom"
        elif self.last_odom_time and (now - self.last_odom_time).to_sec() > self.odom_timeout:
            state = STATE_NO_ODOM
            reason = "odom qua lau khong cap nhat (timeout %.1fs)" % self.odom_timeout

        if state != STATE_OK:
            self._publish_and_log(state, reason)
            return

        if not self.cmd_vel_received and (self.move_base_status == 1):
            # Có goal ACTIVE nhưng chưa từng nhận cmd_vel
            pass  # Có thể move_base chưa gửi, chưa kết luận stuck

        # 2) Kiểm tra robot stuck: move_base ACTIVE, có cmd_vel, nhưng odom gần như không đổi
        if self.move_base_status == 1 and self.last_odom_pos and self.last_odom_time:
            cmd_recent = (
                self.last_cmd_vel_time
                and (now - self.last_cmd_vel_time).to_sec() < self.cmd_vel_timeout
                and (self.last_cmd_vel_linear > self.cmd_vel_active_threshold or self.last_cmd_vel_angular > self.cmd_vel_active_threshold)
            )
            if cmd_recent:
                if self.stuck_odom_pos_at_start is None:
                    self.stuck_odom_pos_at_start = self.last_odom_pos
                    self.stuck_check_start_time = now
                else:
                    elapsed = (now - self.stuck_check_start_time).to_sec()
                    if elapsed >= self.stuck_check_duration:
                        dx = self.last_odom_pos[0] - self.stuck_odom_pos_at_start[0]
                        dy = self.last_odom_pos[1] - self.stuck_odom_pos_at_start[1]
                        dist = math.hypot(dx, dy)
                        if dist < self.min_distance_moved:
                            state = STATE_ROBOT_NOT_MOVING
                            reason = "move_base ACTIVE, cmd_vel co lenh nhung robot gan nhu khong di chuyen (%.2fm trong %.1fs)" % (
                                dist, elapsed,
                            )
                        self.stuck_odom_pos_at_start = None
                        self.stuck_check_start_time = None
            else:
                self.stuck_odom_pos_at_start = None
                self.stuck_check_start_time = None

        if state != STATE_OK:
            self._publish_and_log(state, reason)
            return

        self._publish_and_log(STATE_OK, "")

    def _publish_and_log(self, state, reason):
        self.state_pub.publish(String(data=state))
        if state != STATE_OK and reason:
            rospy.logwarn_throttle(5.0, "[robot_state_check] %s: %s", state, reason)


def main():
    try:
        RobotStateChecker()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass


if __name__ == "__main__":
    main()
