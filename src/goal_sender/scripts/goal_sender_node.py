#!/usr/bin/env python3

import rospy
import actionlib
from move_base_msgs.msg import MoveBaseAction, MoveBaseGoal
from geometry_msgs.msg import PoseStamped
import tf
from actionlib_msgs.msg import GoalStatusArray
from visualization_msgs.msg import Marker, MarkerArray
from std_msgs.msg import ColorRGBA

class GoalSender:
    def __init__(self):
        rospy.init_node('goal_sender_node')
        rospy.loginfo("Goal sender node initialized")
        
        # Khởi tạo action client cho move_base
        self.move_base_client = actionlib.SimpleActionClient('move_base', MoveBaseAction)
        rospy.loginfo("Waiting for move_base action server...")
        self.move_base_client.wait_for_server()
        rospy.loginfo("Connected to move_base action server")

        # Publisher cho marker
        self.marker_pub = rospy.Publisher('/visualization_markers_start_goal', MarkerArray, queue_size=10)
        rospy.loginfo("Publisher for markers initialized")
        
        # Cấu hình thí nghiệm
        self.num_cycles = 1  # Số lần lặp lại
        self.current_cycle = 0
        self.current_goal_index = 0
        self.goal_active = False
        self.last_goal_id = None

        # Định nghĩa các waypoints (đã bỏ phần màu)
        self.waypoints = [
            {'name': 'GOAL_1', 'x': -2.0, 'y': 7.0, 'yaw': -1.57},
            {'name': 'GOAL_2', 'x': -4.5, 'y': -2.0, 'yaw': 0.0},
            {'name': 'GOAL_3', 'x': 10.0, 'y': -1.0, 'yaw': 3.14},
            {'name': 'GOAL_4', 'x': 3.0, 'y': 7.0, 'yaw': -1.57},
            {'name': 'GOAL_5', 'x': 3.0, 'y': 2.0, 'yaw': 1.57},
            {'name': 'GOAL_6', 'x': -10.0, 'y': -7.0, 'yaw': 1.57},
        ]
        self.current_goal_index = 0
        self.goal_active = False

        # Theo dõi trạng thái move_base
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)

        # Gửi mục tiêu đầu tiên
        rospy.sleep(1.0)  # Đợi publisher sẵn sàng
        self.send_next_goal()

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

            rospy.loginfo(f"Sending goal {self.current_goal_index+1}/{len(self.waypoints)} (Cycle {self.current_cycle+1}/{self.num_cycles}): {wp['name']} at ({wp['x']}, {wp['y']}, {wp['yaw']})")
            self.move_base_client.send_goal(goal)
            # Lưu lại goal_id vừa gửi
            self.last_goal_id = self.move_base_client.gh.comm_state_machine.action_goal.goal_id.id
            self.goal_active = True
            self.publish_current_goal_marker(wp)
        else:
            self.current_cycle += 1
            if self.current_cycle < self.num_cycles:
                rospy.loginfo(f"Completed cycle {self.current_cycle}/{self.num_cycles}, restarting waypoints.")
                self.current_goal_index = 0
                self.send_next_goal()
            else:
                rospy.loginfo("All cycles completed. Shutting down node.")
                rospy.signal_shutdown("All goals sent.")

    def status_callback(self, msg):
        if not self.goal_active or not self.last_goal_id:
            return
        if not msg.status_list:
            return
        # Chỉ xử lý status của goal_id vừa gửi
        for status in msg.status_list:
            if status.goal_id.id == self.last_goal_id:
                if status.status in [2, 3, 4, 5, 8]:
                    rospy.loginfo(f"Goal {self.current_goal_index+1} finished with status: {status.status}")
                    self.current_goal_index += 1
                    self.goal_active = False
                    self.last_goal_id = None
                    rospy.sleep(0.5)  # Đợi một chút trước khi gửi tiếp
                    self.send_next_goal()
                break

    def publish_current_goal_marker(self, wp):
        marker_array = MarkerArray()
        # Marker hình cầu
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
        marker.color = ColorRGBA(1.0, 0.0, 0.0, 1.0)  # Đỏ
        marker.pose.orientation.w = 1.0
        marker_array.markers.append(marker)
        # Marker text
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
        GoalSender()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass 