#!/usr/bin/env python3

import rospy
import csv
import os
from datetime import datetime
from actionlib_msgs.msg import GoalStatusArray
from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from std_msgs.msg import String
import threading
import time

class MoveBaseStateMonitor:
    def __init__(self):
        rospy.init_node('move_base_state_monitor', anonymous=True)
        
        # Khởi tạo các biến
        self.current_state = "UNKNOWN"
        self.goal_status = None
        self.robot_pose = None
        self.robot_velocity = None
        self.start_time = rospy.Time.now()
        
        # Tạo thư mục logs nếu chưa tồn tại (hỗ trợ so sánh: MOVE_BASE_LOG_DIR theo robot_type/run)
        self.log_dir = os.environ.get("MOVE_BASE_LOG_DIR", os.path.expanduser("~/move_base_logs"))
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        # Tạo file log với timestamp
        timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"move_base_state_{timestamp}.csv")
        
        # Khởi tạo CSV file
        self.init_csv_file()
        
        # Subscribers
        rospy.Subscriber("/move_base/status", GoalStatusArray, self.status_callback)
        rospy.Subscriber("/move_base/current_goal", PoseStamped, self.goal_callback)
        rospy.Subscriber("/odom", Odometry, self.odom_callback)
        rospy.Subscriber("/cmd_vel", Twist, self.cmd_vel_callback)
        
        # Publisher để publish trạng thái hiện tại
        self.state_pub = rospy.Publisher('/move_base_monitor/current_state', String, queue_size=10)
        
        # Timer để log định kỳ (1 Hz đủ cho monitoring, giảm CPU)
        self.log_timer = rospy.Timer(rospy.Duration(1.0), self.log_state)
        
        rospy.loginfo("Move Base State Monitor đã khởi động")
        rospy.loginfo(f"Log file: {self.log_file}")
        
    def init_csv_file(self):
        """Khởi tạo file CSV với headers"""
        with open(self.log_file, 'w', newline='') as csvfile:
            writer = csv.writer(csvfile)
            writer.writerow([
                'Timestamp', 'Time_Elapsed(s)', 'State', 'Goal_Status', 
                'Robot_X', 'Robot_Y', 'Robot_Yaw', 'Linear_Vel', 'Angular_Vel',
                'Goal_X', 'Goal_Y', 'Goal_Yaw'
            ])
    
    def status_callback(self, msg):
        """Callback cho move_base status"""
        if msg.status_list:
            status = msg.status_list[0]
            self.goal_status = status.status
            
            # Chuyển đổi status code thành text
            status_text = self.get_status_text(status.status)
            if status_text != self.current_state:
                rospy.loginfo(f"Move Base State changed: {self.current_state} -> {status_text}")
                self.current_state = status_text
                
                # Publish trạng thái mới
                self.state_pub.publish(status_text)
    
    def goal_callback(self, msg):
        """Callback cho current goal"""
        self.current_goal = msg
    
    def odom_callback(self, msg):
        """Callback cho odometry"""
        self.robot_pose = msg.pose.pose
        self.robot_velocity = msg.twist.twist
    
    def cmd_vel_callback(self, msg):
        """Callback cho command velocity"""
        self.cmd_vel = msg
    
    def get_status_text(self, status_code):
        """Chuyển đổi status code thành text"""
        status_dict = {
            0: "PENDING",
            1: "ACTIVE", 
            2: "PREEMPTED",
            3: "SUCCEEDED",
            4: "ABORTED",
            5: "REJECTED",
            6: "PREEMPTING",
            7: "RECALLING",
            8: "RECALLED",
            9: "LOST"
        }
        return status_dict.get(status_code, "UNKNOWN")
    
    def log_state(self, event):
        """Log trạng thái hiện tại vào CSV file"""
        try:
            current_time = rospy.Time.now()
            elapsed_time = (current_time - self.start_time).to_sec()
            
            # Lấy thông tin robot pose
            robot_x = robot_y = robot_yaw = 0.0
            if self.robot_pose:
                robot_x = self.robot_pose.position.x
                robot_y = self.robot_pose.position.y
                # Tính yaw từ quaternion
                import math
                q = self.robot_pose.orientation
                robot_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # Lấy thông tin velocity
            linear_vel = angular_vel = 0.0
            if self.robot_velocity:
                linear_vel = self.robot_velocity.linear.x
                angular_vel = self.robot_velocity.angular.z
            
            # Lấy thông tin goal
            goal_x = goal_y = goal_yaw = 0.0
            if hasattr(self, 'current_goal') and self.current_goal:
                goal_x = self.current_goal.pose.position.x
                goal_y = self.current_goal.pose.position.y
                # Tính yaw từ quaternion
                q = self.current_goal.pose.orientation
                goal_yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y), 1.0 - 2.0 * (q.y * q.y + q.z * q.z))
            
            # Ghi vào CSV
            with open(self.log_file, 'a', newline='') as csvfile:
                writer = csv.writer(csvfile)
                writer.writerow([
                    datetime.now().strftime("%Y-%m-%d %H:%M:%S.%f")[:-3],
                    f"{elapsed_time:.3f}",
                    self.current_state,
                    self.goal_status if self.goal_status is not None else "N/A",
                    f"{robot_x:.3f}",
                    f"{robot_y:.3f}",
                    f"{robot_yaw:.3f}",
                    f"{linear_vel:.3f}",
                    f"{angular_vel:.3f}",
                    f"{goal_x:.3f}",
                    f"{goal_y:.3f}",
                    f"{goal_yaw:.3f}"
                ])
                
        except Exception as e:
            rospy.logerr(f"Lỗi khi log state: {e}")
    
    def print_summary(self):
        """In tổng kết khi node shutdown"""
        rospy.loginfo("=== MOVE BASE STATE MONITOR SUMMARY ===")
        rospy.loginfo(f"Log file: {self.log_file}")
        rospy.loginfo(f"Total monitoring time: {(rospy.Time.now() - self.start_time).to_sec():.2f} seconds")
        rospy.loginfo("=======================================")

def main():
    try:
        monitor = MoveBaseStateMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
    finally:
        if 'monitor' in locals():
            monitor.print_summary()

if __name__ == '__main__':
    main() 