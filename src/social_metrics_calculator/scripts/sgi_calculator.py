#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
SGI Calculator Node
------------------
This node calculates the Social Grouping Index (SGI) based on:
- Robot position from /amcl_pose
- Social state information from /social_state
It publishes the SGI values to /sgi topic.

Author: Quyen Anh
Date: 2024
"""

import rospy
import math
import numpy as np
from geometry_msgs.msg import PoseWithCovarianceStamped
from social_msgs.msg import SocialState, SocialPerson, SocialGroup
from social_metrics_calculator.msg import SGI
import time

# ==== Global Variables ====
robot_pose = None
agent_positions = {}
agent_velocities = {}
agents_in_groups = set()
start_time = None

# ==== Gaussian Model Parameters ====
sigma_g = 0.5   # Group space standard deviation (rg/2 where rg = 1.0m)
sigma_o = 0.5   # Object interaction standard deviation (ro/2 where ro = 1.0m)

# ==== Circle Fitting Structures and Functions ====
class Circle:
    def __init__(self):
        self.a = 0.0  # X-coordinate of center
        self.b = 0.0  # Y-coordinate of center
        self.r = 0.0  # Radius
        self.s = 0.0  # Root mean square error
        self.j = 0    # Number of iterations

class Data:
    def __init__(self):
        self.X = []  # X coordinates
        self.Y = []  # Y coordinates
        self.meanX = 0.0
        self.meanY = 0.0
        self.n = 0   # Number of points

    def means(self):
        """Calculate mean of X and Y coordinates"""
        self.meanX = sum(self.X) / self.n
        self.meanY = sum(self.Y) / self.n

def circle_fit_by_taubin(data):
    """Fit a circle to data points using Taubin's method"""
    # Special handling for 2 people
    if data.n == 2:
        circle = Circle()
        # Calculate midpoint (center)
        circle.a = (data.X[0] + data.X[1]) / 2.0
        circle.b = (data.Y[0] + data.Y[1]) / 2.0
        # Calculate radius as half the distance between points
        dx = data.X[1] - data.X[0]
        dy = data.Y[1] - data.Y[0]
        circle.r = math.sqrt(dx*dx + dy*dy) / 2.0
        circle.j = 0  # No iterations needed for 2 points
        return circle

    # Constants for circle fitting
    FOUR = 4.0
    THREE = 3.0
    TWO = 2.0
    ITER_MAX = 99

    # Calculate means
    data.means()

    # Computing moments
    Mxx = Myy = Mxy = Mxz = Myz = Mzz = 0.0
    
    for i in range(data.n):
        Xi = data.X[i] - data.meanX   # centered x-coordinates
        Yi = data.Y[i] - data.meanY   # centered y-coordinates
        Zi = Xi*Xi + Yi*Yi
        
        Mxy += Xi*Yi
        Mxx += Xi*Xi
        Myy += Yi*Yi
        Mxz += Xi*Zi
        Myz += Yi*Zi
        Mzz += Zi*Zi

    Mxx /= data.n
    Myy /= data.n
    Mxy /= data.n
    Mxz /= data.n
    Myz /= data.n
    Mzz /= data.n
    
    # Computing coefficients of the characteristic polynomial
    Mz = Mxx + Myy
    Cov_xy = Mxx*Myy - Mxy*Mxy
    Var_z = Mzz - Mz*Mz
    A3 = FOUR*Mz
    A2 = -THREE*Mz*Mz - Mzz
    A1 = Var_z*Mz + FOUR*Cov_xy*Mz - Mxz*Mxz - Myz*Myz
    A0 = Mxz*(Mxz*Myy - Myz*Mxy) + Myz*(Myz*Mxx - Mxz*Mxy) - Var_z*Cov_xy
    A22 = A2 + A2
    A33 = A3 + A3 + A3

    # Finding the root of the characteristic polynomial using Newton's method
    x = 0.0
    y = A0
    for iter in range(ITER_MAX):
        Dy = A1 + x*(A22 + A33*x)
        xnew = x - y/Dy
        if (xnew == x) or (not math.isfinite(xnew)):
            break
        ynew = A0 + xnew*(A1 + xnew*(A2 + xnew*A3))
        if abs(ynew) >= abs(y):
            break
        x = xnew
        y = ynew
    
    # Computing parameters of the fitting circle
    DET = x*x - x*Mz + Cov_xy
    Xcenter = (Mxz*(Myy - x) - Myz*Mxy)/DET/TWO
    Ycenter = (Myz*(Mxx - x) - Mxz*Mxy)/DET/TWO

    # Assembling the output
    circle = Circle()
    circle.a = Xcenter + data.meanX
    circle.b = Ycenter + data.meanY
    circle.r = math.sqrt(Xcenter*Xcenter + Ycenter*Ycenter + Mz)
    circle.j = iter
    
    return circle

def distance(x1, y1, x2, y2):
    """Calculate Euclidean distance between two points"""
    return math.hypot(x1 - x2, y1 - y2)

def compute_sgi(robot_x, robot_y, groups, interactions):
    """Calculate SGI for all groups and interactions"""
    sgi_values = []
    group_info = []

    # Process groups
    for group in groups:
        if len(group.members) < 2:
            continue

        # Prepare data for circle fitting
        data = Data()
        for member in group.members:
            data.X.append(member.position.position.x)
            data.Y.append(member.position.position.y)
        data.n = len(data.X)

        # Fit circle to group members
        circle = circle_fit_by_taubin(data)
        
        # Use circle center and radius for SGI calculation
        group_x = circle.a
        group_y = circle.b
        group_radius = circle.r

        # Calculate SGI for this group using circle center and radius
        h = distance(robot_x, robot_y, group_x, group_y)
        
        # Adjust sigma based on group radius only
        # Use minimum radius to avoid division by zero
        min_radius = 0.1  # Minimum radius in meters
        adjusted_radius = max(group_radius, min_radius)
        adjusted_sigma = sigma_g * (adjusted_radius / 1.0)
        
        # Calculate SGI with adjusted sigma
        sgi = math.exp(-(h**2) / (2 * adjusted_sigma**2))
        sgi_values.append(sgi)
        group_info.append((group.group_name, len(group.members)))

        # Debug information
        rospy.logdebug(f"Group {group.group_name}:")
        rospy.logdebug(f"  Circle Center: ({group_x:.2f}, {group_y:.2f})")
        rospy.logdebug(f"  Circle Radius: {group_radius:.2f} (adjusted: {adjusted_radius:.2f})")
        rospy.logdebug(f"  Distance to robot: {h:.2f}")
        rospy.logdebug(f"  Adjusted sigma: {adjusted_sigma:.2f}")
        rospy.logdebug(f"  SGI: {sgi:.4f}")

    # Process object interactions
    for interaction in interactions:
        # Get person and object positions
        person_x = interaction.participants[0].position.position.x
        person_y = interaction.participants[0].position.position.y
        object_x = interaction.object_position.position.x
        object_y = interaction.object_position.position.y

        # Calculate interaction center (midpoint between person and object)
        center_x = (person_x + object_x) / 2.0
        center_y = (person_y + object_y) / 2.0

        # Calculate interaction radius (half the distance between person and object)
        interaction_radius = distance(person_x, person_y, object_x, object_y) / 2.0

        # Calculate SGI for this interaction
        h = distance(robot_x, robot_y, center_x, center_y)
        
        # Adjust sigma based on interaction radius
        min_radius = 0.1  # Minimum radius in meters
        adjusted_radius = max(interaction_radius, min_radius)
        adjusted_sigma = sigma_o * (adjusted_radius / 1.0)
        
        # Calculate SGI with adjusted sigma
        sgi = math.exp(-(h**2) / (2 * adjusted_sigma**2))
        sgi_values.append(sgi)
        group_info.append((f"interaction_{interaction.object_name}", 1))

        # Debug information
        rospy.logdebug(f"Interaction with {interaction.object_name}:")
        rospy.logdebug(f"  Person: ({person_x:.2f}, {person_y:.2f})")
        rospy.logdebug(f"  Object: ({object_x:.2f}, {object_y:.2f})")
        rospy.logdebug(f"  Center: ({center_x:.2f}, {center_y:.2f})")
        rospy.logdebug(f"  Radius: {interaction_radius:.2f} (adjusted: {adjusted_radius:.2f})")
        rospy.logdebug(f"  Distance to robot: {h:.2f}")
        rospy.logdebug(f"  Adjusted sigma: {adjusted_sigma:.2f}")
        rospy.logdebug(f"  SGI: {sgi:.4f}")

    # Return max SGI and corresponding group info
    if sgi_values:
        max_idx = np.argmax(sgi_values)
        return sgi_values[max_idx], group_info[max_idx]
    return 0.0, ("none", 0)

def robot_callback(msg):
    """Callback for robot pose updates"""
    global robot_pose
    robot_pose = msg.pose.pose

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
            agent_positions[person.name] = (person.position.position.x, person.position.position.y)
            agent_velocities[person.name] = (person.velocity.linear.x, person.velocity.linear.y)

        # Process groups to identify agents in groups
        for group in msg.groups.groups:
            for member in group.members:
                agents_in_groups.add(member.name)

        # Calculate and publish SGI if we have robot pose
        if robot_pose and not rospy.is_shutdown():
            sgi_value, (group_name, num_members) = compute_sgi(
                robot_pose.position.x, 
                robot_pose.position.y,
                msg.groups.groups,
                msg.interactions.interactions
            )

            # Create and publish SGI message
            sgi_msg = SGI()
            sgi_msg.header.stamp = msg.header.stamp
            sgi_msg.sgi = sgi_value
            sgi_msg.timestamp = sgi_msg.header.stamp
            sgi_msg.group_id = group_name
            sgi_msg.num_members = num_members
            
            # Check if node is still running before publishing
            if not rospy.is_shutdown():
                sgi_pub.publish(sgi_msg)
                rospy.logdebug(f"Published SGI: {sgi_value:.4f} for group {group_name}")

    except rospy.ROSInterruptException:
        raise
    except Exception as e:
        rospy.logerr(f"Error in social_state_callback: {str(e)}")

def main():
    """Main function"""
    global sgi_pub

    # Initialize ROS node
    rospy.init_node('sgi_calculator', anonymous=True)

    # Set log level to debug for detailed information
    rospy.loginfo("SGI Calculator node started")
    rospy.loginfo("Debug information will be printed for each group")

    # Create publisher
    sgi_pub = rospy.Publisher('/sgi', SGI, queue_size=10)

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