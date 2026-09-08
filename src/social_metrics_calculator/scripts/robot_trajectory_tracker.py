#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Robot Trajectory Tracker Node
----------------------------
This node tracks and visualizes:
- Robot trajectory using /amcl_pose
- Linear and angular velocities using /odom

Author: Quyen Anh
Date: 2024
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from nav_msgs.msg import Odometry
from geometry_msgs.msg import PoseWithCovarianceStamped
import threading
import queue

# ==== Global Variables ====
# Data queues for thread-safe communication
pose_queue = queue.Queue()
odom_queue = queue.Queue()

# ==== Plot Setup ====
plt.ion()
fig = plt.figure(figsize=(15, 10))
fig.canvas.manager.set_window_title("Robot Trajectory and Motion")

# Create subplots
ax1 = plt.subplot2grid((2, 2), (0, 0))  # Trajectory plot
ax2 = plt.subplot2grid((2, 2), (0, 1))  # Linear velocity plot
ax3 = plt.subplot2grid((2, 2), (1, 0))  # Angular velocity plot
ax4 = plt.subplot2grid((2, 2), (1, 1))  # Statistics

plt.subplots_adjust(hspace=0.4, wspace=0.3)

# Setup trajectory plot
ax1.set_xlabel('X (m)')
ax1.set_ylabel('Y (m)')
ax1.set_title('Robot Trajectory')
ax1.grid(True, alpha=0.3)
ax1.set_aspect('equal')

line, = ax1.plot([], [], 'b-', label='Trajectory', linewidth=2)
robot_point, = ax1.plot([], [], 'bo', label='Robot', markersize=10)
ax1.legend(loc='upper right')

# Setup velocity plots
ax2.set_xlabel('Time (s)')
ax2.set_ylabel('Linear Velocity (m/s)')
ax2.set_title('Linear Velocity')
ax2.grid(True, alpha=0.3)
linear_vel_line, = ax2.plot([], [], 'g-', label='Linear Velocity', linewidth=2)
ax2.legend(loc='upper right')

ax3.set_xlabel('Time (s)')
ax3.set_ylabel('Angular Velocity (rad/s)')
ax3.set_title('Angular Velocity')
ax3.grid(True, alpha=0.3)
angular_vel_line, = ax3.plot([], [], 'r-', label='Angular Velocity', linewidth=2)
ax3.legend(loc='upper right')

# Setup statistics text
stats_text = ax4.text(0.05, 0.95, '', transform=ax4.transAxes, 
                     verticalalignment='top', fontsize=10,
                     bbox=dict(facecolor='white', edgecolor='black', alpha=0.8))
ax4.axis('off')

# Data storage
x_positions = []
y_positions = []
pose_times = []
linear_velocities = []
angular_velocities = []
vel_times = []

def update_plot(frame):
    """Update all plots"""
    try:
        # Process pose data
        while not pose_queue.empty():
            x, y, t = pose_queue.get()
            x_positions.append(x)
            y_positions.append(y)
            pose_times.append(t)

        # Process odom data
        while not odom_queue.empty():
            lin_vel, ang_vel, t = odom_queue.get()
            linear_velocities.append(lin_vel)
            angular_velocities.append(ang_vel)
            vel_times.append(t)

        # Update trajectory plot
        if len(x_positions) > 0:
            line.set_data(x_positions, y_positions)
            robot_point.set_data([x_positions[-1]], [y_positions[-1]])
            
            # Update plot limits
            ax1.set_xlim(min(x_positions) - 1, max(x_positions) + 1)
            ax1.set_ylim(min(y_positions) - 1, max(y_positions) + 1)

        # Update velocity plots
        if len(vel_times) > 0:
            linear_vel_line.set_data(vel_times, linear_velocities)
            angular_vel_line.set_data(vel_times, angular_velocities)
            
            ax2.set_xlim(min(vel_times), max(vel_times))
            ax3.set_xlim(min(vel_times), max(vel_times))
            
            ax2.relim()
            ax2.autoscale_view()
            ax3.relim()
            ax3.autoscale_view()

        # Update statistics
        if len(x_positions) > 0 and len(linear_velocities) > 0:
            avg_linear_vel = np.mean(linear_velocities)
            avg_angular_vel = np.mean(angular_velocities)
            current_linear_vel = linear_velocities[-1]
            current_angular_vel = angular_velocities[-1]
            
            stats_str = f"=== ROBOT STATISTICS ===\n"
            stats_str += f"- Current Position: ({x_positions[-1]:.2f}, {y_positions[-1]:.2f})\n"
            stats_str += f"- Current Linear Velocity: {current_linear_vel:.2f} m/s\n"
            stats_str += f"- Current Angular Velocity: {current_angular_vel:.2f} rad/s\n"
            stats_str += f"- Avg Linear Velocity: {avg_linear_vel:.2f} m/s\n"
            stats_str += f"- Avg Angular Velocity: {avg_angular_vel:.2f} rad/s\n"
            stats_text.set_text(stats_str)

    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")

def pose_callback(msg):
    """Callback for robot pose updates from /amcl_pose"""
    # Get current time
    current_time = msg.header.stamp.to_sec()
    
    # Put data in queue
    pose_queue.put((msg.pose.pose.position.x, 
                   msg.pose.pose.position.y, 
                   current_time))

def odom_callback(msg):
    """Callback for robot odometry updates from /odom"""
    # Get current time
    current_time = msg.header.stamp.to_sec()
    
    # Calculate linear velocity
    linear_vel = np.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2)
    
    # Put data in queue
    odom_queue.put((linear_vel, 
                   msg.twist.twist.angular.z, 
                   current_time))

def main():
    """Main function"""
    # Initialize ROS node
    rospy.init_node('robot_trajectory_tracker', anonymous=True)

    # Create subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, pose_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)

    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=10)  # Update every 10ms
    
    # Show plot
    plt.show(block=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 