#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SII Calculator Node
------------------
This node calculates the Social Individual Index (SII) based on:
- Robot position from /amcl_pose
- Social state information from /social_state
It publishes the SII values to /sii topic.

Author: Quyen Anh
Date: 2024
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from social_msgs.msg import SocialState, SocialPerson
from social_metrics_calculator.msg import SII
import time

# ==== Global Variables ====
robot_pose = None
agent_positions = {}
agent_velocities = {}
agents_in_groups = set()
start_time = None

# ==== Gaussian Model Parameters ====
sigma_p = 0.45  # Personal space standard deviation (dc/2 where dc = 0.9m)
MIN_DISTANCE = 0.1  # Minimum distance threshold in meters

def distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points"""
    return math.hypot(x1 - x2, y1 - y2)

def compute_sii(robot_x, robot_y):
    """Calculate SII for all agents"""
    sii_values = []
    agent_info = []

    # Log robot position
    rospy.logdebug(f"Robot position: ({robot_x:.2f}, {robot_y:.2f})")

    # Process all agents
    for agent_id, (ax, ay) in agent_positions.items():
        # Calculate distance between robot and agent
        h = distance(robot_x, robot_y, ax, ay)
        
        # Apply minimum distance threshold
        h = max(h, MIN_DISTANCE)
        
        # Calculate SII using Gaussian model
        sii = math.exp(-(h**2) / (2 * sigma_p**2))
        sii_values.append(sii)
        
        # Mark if agent is in a group
        is_in_group = agent_id in agents_in_groups
        agent_info.append((agent_id, 1, is_in_group))  # (agent_id, num_members, is_in_group)

        # Debug information
        rospy.logdebug(f"Agent {agent_id}:")
        rospy.logdebug(f"  Position: ({ax:.2f}, {ay:.2f})")
        rospy.logdebug(f"  Distance to robot: {h:.2f}")
        rospy.logdebug(f"  SII: {sii:.4f}")
        rospy.logdebug(f"  In group: {is_in_group}")

    # Return max SII and corresponding agent info
    if sii_values:
        max_idx = np.argmax(sii_values)
        return sii_values[max_idx], agent_info[max_idx]
    return 0.0, ("none", 0, False)

def robot_callback(msg):
    """Callback for robot pose updates"""
    global robot_pose
    robot_pose = msg.pose.pose
    rospy.logdebug(f"Received robot pose: ({robot_pose.position.x:.2f}, {robot_pose.position.y:.2f})")

def social_state_callback(msg):
    """Callback for social state updates"""
    global agent_positions, agent_velocities, agents_in_groups, start_time

    try:
        # Initialize start_time if not set
        if start_time is None:
            start_time = rospy.Time.now()

        # Update agent positions and velocities
        agent_positions.clear()
        agent_velocities.clear()
        agents_in_groups.clear()

        # Process individual agents
        for person in msg.people.people:
            # Validate position data
            if not (math.isfinite(person.position.position.x) and 
                   math.isfinite(person.position.position.y)):
                rospy.logwarn(f"Invalid position data for agent {person.name}")
                continue
                
            agent_positions[person.name] = (person.position.position.x, person.position.position.y)
            agent_velocities[person.name] = (person.velocity.linear.x, person.velocity.linear.y)
            rospy.logdebug(f"Updated agent {person.name} position: ({person.position.position.x:.2f}, {person.position.position.y:.2f})")

        # Process groups to identify agents in groups
        for group in msg.groups.groups:
            for member in group.members:
                agents_in_groups.add(member.name)

        # Calculate and publish SII if we have robot pose
        if robot_pose and not rospy.is_shutdown():
            # Validate robot position
            if not (math.isfinite(robot_pose.position.x) and 
                   math.isfinite(robot_pose.position.y)):
                rospy.logwarn("Invalid robot position data")
                return
                
            sii_value, (agent_id, num_members, is_in_group) = compute_sii(
                robot_pose.position.x, 
                robot_pose.position.y
            )

            # Create and publish SII message
            sii_msg = SII()
            sii_msg.header.stamp = msg.header.stamp
            sii_msg.sii = sii_value
            sii_msg.timestamp = sii_msg.header.stamp
            sii_msg.agent_id = agent_id
            sii_msg.num_members = num_members
            
            # Check if node is still running before publishing
            if not rospy.is_shutdown():
                sii_pub.publish(sii_msg)
                rospy.logdebug(f"Published SII: {sii_value:.4f} for agent {agent_id}")

    except rospy.ROSInterruptException:
        raise
    except Exception as e:
        rospy.logerr(f"Error in social_state_callback: {str(e)}")

def main():
    """Main function"""
    global sii_pub

    # Initialize ROS node
    rospy.init_node('sii_calculator', anonymous=True)

    # Set log level to debug for detailed information
    rospy.loginfo("SII Calculator node started")
    rospy.loginfo("Debug information will be printed for each agent")

    # Create publisher
    sii_pub = rospy.Publisher('/sii', SII, queue_size=10)

    # Create subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, robot_callback)
    rospy.Subscriber('/social_state', SocialState, social_state_callback)

    # Spin to keep the node alive
    rospy.spin()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 