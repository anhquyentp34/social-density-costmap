#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SII Calculator Node
------------------
This node calculates the Social Individual Index (SII) based on:
- Robot position from /amcl_pose
- Social state information from /social_state
It publishes the SII values to /sii topic.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from social_msgs.msg import SocialState
from social_navigation_monitor.msg import SII

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

    for agent_id, (ax, ay) in agent_positions.items():
        h = distance(robot_x, robot_y, ax, ay)
        h = max(h, MIN_DISTANCE)
        sii = math.exp(-(h**2) / (2 * sigma_p**2))
        sii_values.append(sii)

        is_in_group = agent_id in agents_in_groups
        agent_info.append((agent_id, 1, is_in_group))

    if sii_values:
        max_idx = np.argmax(sii_values)
        return sii_values[max_idx], agent_info[max_idx]
    return 0.0, ("none", 0, False)


def robot_callback(msg):
    """Callback for robot pose updates"""
    global robot_pose
    robot_pose = msg.pose.pose


def social_state_callback(msg):
    """Callback for social state updates"""
    global agent_positions, agent_velocities, agents_in_groups, start_time

    try:
        if start_time is None:
            start_time = rospy.Time.now()

        agent_positions.clear()
        agent_velocities.clear()
        agents_in_groups.clear()

        for person in msg.people.people:
            if not (
                math.isfinite(person.position.position.x)
                and math.isfinite(person.position.position.y)
            ):
                rospy.logwarn(f"Invalid position data for agent {person.name}")
                continue

            agent_positions[person.name] = (
                person.position.position.x,
                person.position.position.y,
            )
            agent_velocities[person.name] = (
                person.velocity.linear.x,
                person.velocity.linear.y,
            )

        for group in msg.groups.groups:
            for member in group.members:
                agents_in_groups.add(member.name)

        if robot_pose and not rospy.is_shutdown():
            if not (
                math.isfinite(robot_pose.position.x)
                and math.isfinite(robot_pose.position.y)
            ):
                rospy.logwarn("Invalid robot position data")
                return

            sii_value, (agent_id, num_members, _is_in_group) = compute_sii(
                robot_pose.position.x,
                robot_pose.position.y,
            )

            sii_msg = SII()
            sii_msg.header.stamp = msg.header.stamp
            sii_msg.sii = sii_value
            sii_msg.timestamp = sii_msg.header.stamp
            sii_msg.agent_id = agent_id
            sii_msg.num_members = num_members

            if not rospy.is_shutdown():
                sii_pub.publish(sii_msg)

    except rospy.ROSInterruptException:
        raise
    except Exception as e:
        rospy.logerr(f"Error in social_state_callback: {str(e)}")


def main():
    """Main function"""
    global sii_pub

    rospy.init_node("sii_calculator", anonymous=True)
    rospy.loginfo("SII Calculator node started")

    sii_pub = rospy.Publisher("/sii", SII, queue_size=10)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, robot_callback)
    rospy.Subscriber("/social_state", SocialState, social_state_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
