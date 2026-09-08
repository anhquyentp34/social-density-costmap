#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SGI Calculator Node
------------------
This node calculates the Social Grouping Index (SGI) based on:
- Robot position from /amcl_pose
- Social state information from /social_state
It publishes the SGI values to /sgi topic.
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from social_msgs.msg import SocialState
from social_navigation_monitor.msg import SGI

# ==== Global Variables ====
robot_pose = None
agent_positions = {}
agent_velocities = {}
agents_in_groups = set()
start_time = None

# ==== Gaussian Model Parameters ====
sigma_g = 0.5
sigma_o = 0.5


class Circle:
    def __init__(self):
        self.a = 0.0
        self.b = 0.0
        self.r = 0.0
        self.s = 0.0
        self.j = 0


class Data:
    def __init__(self):
        self.X = []
        self.Y = []
        self.meanX = 0.0
        self.meanY = 0.0
        self.n = 0

    def means(self):
        self.meanX = sum(self.X) / self.n
        self.meanY = sum(self.Y) / self.n


def circle_fit_by_taubin(data):
    if data.n == 2:
        circle = Circle()
        circle.a = (data.X[0] + data.X[1]) / 2.0
        circle.b = (data.Y[0] + data.Y[1]) / 2.0
        dx = data.X[1] - data.X[0]
        dy = data.Y[1] - data.Y[0]
        circle.r = math.sqrt(dx * dx + dy * dy) / 2.0
        circle.j = 0
        return circle

    FOUR = 4.0
    THREE = 3.0
    TWO = 2.0
    ITER_MAX = 99

    data.means()
    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.0

    for i in range(data.n):
        Xi = data.X[i] - data.meanX
        Yi = data.Y[i] - data.meanY
        Zi = Xi * Xi + Yi * Yi

        Mxy += Xi * Yi
        Mxx += Xi * Xi
        Myy += Yi * Yi
        Mxz += Xi * Zi
        Myz += Yi * Zi
        Mzz += Zi * Zi

    Mxx /= data.n
    Myy /= data.n
    Mxy /= data.n
    Mxz /= data.n
    Myz /= data.n
    Mzz /= data.n

    Mz = Mxx + Myy
    Cov_xy = Mxx * Myy - Mxy * Mxy
    Var_z = Mzz - Mz * Mz
    A3 = FOUR * Mz
    A2 = -THREE * Mz * Mz - Mzz
    A1 = Var_z * Mz + FOUR * Cov_xy * Mz - Mxz * Mxz - Myz * Myz
    A0 = Mxz * (Mxz * Myy - Myz * Mxy) + Myz * (Myz * Mxx - Mxz * Mxy) - Var_z * Cov_xy
    A22 = A2 + A2
    A33 = A3 + A3 + A3

    x = 0.0
    y = A0
    for i in range(ITER_MAX):
        Dy = A1 + x * (A22 + A33 * x)
        xnew = x - y / Dy
        if (xnew == x) or (not math.isfinite(xnew)):
            break
        ynew = A0 + xnew * (A1 + xnew * (A2 + xnew * A3))
        if abs(ynew) >= abs(y):
            break
        x = xnew
        y = ynew

    DET = x * x - x * Mz + Cov_xy
    Xcenter = (Mxz * (Myy - x) - Myz * Mxy) / DET / TWO
    Ycenter = (Myz * (Mxx - x) - Mxz * Mxy) / DET / TWO

    circle = Circle()
    circle.a = Xcenter + data.meanX
    circle.b = Ycenter + data.meanY
    circle.r = math.sqrt(Xcenter * Xcenter + Ycenter * Ycenter + Mz)
    circle.j = i

    return circle


def distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)


def compute_sgi(robot_x, robot_y, groups, interactions):
    sgi_values = []
    group_info = []

    for group in groups:
        if len(group.members) < 2:
            continue

        data = Data()
        for member in group.members:
            data.X.append(member.position.position.x)
            data.Y.append(member.position.position.y)
        data.n = len(data.X)

        circle = circle_fit_by_taubin(data)
        group_x = circle.a
        group_y = circle.b
        group_radius = circle.r

        h = distance(robot_x, robot_y, group_x, group_y)
        adjusted_radius = max(group_radius, 0.1)
        adjusted_sigma = sigma_g * (adjusted_radius / 1.0)
        sgi = math.exp(-(h**2) / (2 * adjusted_sigma**2))
        sgi_values.append(sgi)
        group_info.append((group.group_name, len(group.members)))

    for interaction in interactions:
        person_x = interaction.participants[0].position.position.x
        person_y = interaction.participants[0].position.position.y
        object_x = interaction.object_position.position.x
        object_y = interaction.object_position.position.y

        center_x = (person_x + object_x) / 2.0
        center_y = (person_y + object_y) / 2.0
        interaction_radius = distance(person_x, person_y, object_x, object_y) / 2.0

        h = distance(robot_x, robot_y, center_x, center_y)
        adjusted_radius = max(interaction_radius, 0.1)
        adjusted_sigma = sigma_o * (adjusted_radius / 1.0)

        sgi = math.exp(-(h**2) / (2 * adjusted_sigma**2))
        sgi_values.append(sgi)
        group_info.append((f"interaction_{interaction.object_name}", 1))

    if sgi_values:
        max_idx = np.argmax(sgi_values)
        return sgi_values[max_idx], group_info[max_idx]
    return 0.0, ("none", 0)


def robot_callback(msg):
    global robot_pose
    robot_pose = msg.pose.pose


def social_state_callback(msg):
    global agent_positions, agent_velocities, agents_in_groups, start_time

    try:
        if start_time is None:
            start_time = rospy.Time.now()

        agent_positions.clear()
        agent_velocities.clear()
        agents_in_groups.clear()

        for person in msg.people.people:
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
            sgi_value, (group_name, num_members) = compute_sgi(
                robot_pose.position.x,
                robot_pose.position.y,
                msg.groups.groups,
                msg.interactions.interactions,
            )

            sgi_msg = SGI()
            sgi_msg.header.stamp = msg.header.stamp
            sgi_msg.sgi = sgi_value
            sgi_msg.timestamp = sgi_msg.header.stamp
            sgi_msg.group_id = group_name
            sgi_msg.num_members = num_members

            if not rospy.is_shutdown():
                sgi_pub.publish(sgi_msg)

    except rospy.ROSInterruptException:
        raise
    except Exception as e:
        rospy.logerr(f"Error in social_state_callback: {str(e)}")


def main():
    global sgi_pub

    rospy.init_node("sgi_calculator", anonymous=True)
    rospy.loginfo("SGI Calculator node started")

    sgi_pub = rospy.Publisher("/sgi", SGI, queue_size=10)
    rospy.Subscriber("/amcl_pose", PoseWithCovarianceStamped, robot_callback)
    rospy.Subscriber("/social_state", SocialState, social_state_callback)
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
