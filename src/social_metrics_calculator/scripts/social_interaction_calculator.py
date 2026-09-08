#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Social Interaction Calculator Node
---------------------------------
This node calculates social interaction metrics (SII, SGI and RMI) based on social state information
from the social_cue_sim package. It processes:
- Individual agent positions and states
- Group formations and interactions
- Robot's position relative to social entities
- Robot's trajectory, distance traveled, and velocity

The node:
- Calculates and publishes SII (Social Individual Index) for individual interactions
- Calculates and publishes SGI (Social Grouping Index) for group interactions
- Calculates and publishes RMI (Relative Motion Index) for motion-based interactions
- Visualizes SII, SGI and RMI metrics in real-time plots
- Tracks and visualizes robot trajectory, distance, and velocity
- Records metrics to CSV file for later analysis

Author: Quyen Anh
Date: 2024
"""

import rospy
import math
import os
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.animation as animation
import signal
import sys
import time
from social_msgs.msg import SocialState, SocialPerson, SocialGroup
from social_metrics_calculator.msg import SocialIndices
from geometry_msgs.msg import PoseWithCovarianceStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32

# ==== Global Variables ====
current_sii = 0.0
current_sgi = 0.0
current_rmi = 0.0
robot_pose = None
robot_velocity = None
agent_positions = {}
agent_velocities = {}
agents_in_groups = set()
last_wp_visited = -1  # Initialize last waypoint visited

# Plot data storage
time_data = []
sii_data = []
sgi_data = []
rmi_data = []
start_time = None

# Trajectory tracking
waypoints = np.array([(-10.0, -6.0),(-5.0, 6.0), (3.0, 2.0), (-3.0, -1.0), (9.0, -6.0)])
x_positions = []
y_positions = []
velocities = []
times = []
total_distance = 0.0
last_position = None
waypoint_times = {i: [] for i in range(len(waypoints))}
loop_counter = 0

# Get package path and create data directory if it doesn't exist
package_path = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
data_dir = os.path.join(package_path, 'data')
os.makedirs(data_dir, exist_ok=True)
csv_path = os.path.join(data_dir, 'social_interaction_metrics.csv')

# ==== Gaussian Model Parameters ====
sigma_p = 0.45  # Personal space standard deviation (dc/2 where dc = 0.9m)
sigma_g = 0.5   # Group space standard deviation (rg/2 where rg = 1.0m)
sigma_o = 0.5   # Object interaction standard deviation (ro/2 where ro = 1.0m)

# ==== Plot Setup ====
plt.ion()  # Turn on interactive mode
fig = plt.figure(figsize=(15, 10))
fig.canvas.manager.set_window_title("Social Metrics and Trajectory")

# Create subplots
ax1 = plt.subplot2grid((2, 2), (0, 0))  # SII plot
ax2 = plt.subplot2grid((2, 2), (0, 1))  # SGI plot
ax3 = plt.subplot2grid((2, 2), (1, 0))  # RMI plot
ax4 = plt.subplot2grid((2, 2), (1, 1))  # Trajectory plot

plt.subplots_adjust(hspace=0.4, wspace=0.3)

# Setup trajectory plot
ax4.set_xlabel('X (m)')
ax4.set_ylabel('Y (m)')
ax4.set_title('Robot Trajectory')
ax4.grid(True)
ax4.set_aspect('equal')

# Plot waypoints
ax4.plot(waypoints[:, 0], waypoints[:, 1], 'ro', label='Waypoints')
for i, wp in enumerate(waypoints):
    ax4.text(wp[0], wp[1], f'WP{i}', fontsize=10, ha='right')

line, = ax4.plot([], [], 'b-', label='Trajectory')
robot_point, = ax4.plot([], [], 'bo', label='Robot', markersize=10)
stats_text = fig.text(0.78, 0.5, '', ha='left', va='center', fontsize=10,
                     bbox=dict(facecolor='white', edgecolor='black', alpha=0.9))

# Initialize plot data
sii_line, = ax1.plot([], [], 'b-', label='SII')
sgi_line, = ax2.plot([], [], 'r-', label='SGI')
rmi_line, = ax3.plot([], [], 'g-', label='RMI')

# Set up plot limits and labels
ax1.set_ylabel("SII Value")
ax1.set_title("Social Individual Index (SII)")
ax1.grid(True)
ax1.legend()
ax1.set_ylim(0, 1)

ax2.set_xlabel("Time (s)")
ax2.set_ylabel("SGI Value")
ax2.set_title("Social Grouping Index (SGI)")
ax2.grid(True)
ax2.legend()
ax2.set_ylim(0, 1)

ax3.set_xlabel("Time (s)")
ax3.set_ylabel("RMI Value")
ax3.set_title("Relative Motion Index (RMI)")
ax3.grid(True)
ax3.legend()

# Add threshold lines
ax1.axhline(y=0.14, color='cyan', linestyle='--', label="Threshold 0.14")
ax1.axhline(y=0.54, color='red', linestyle='--', label="Threshold 0.54")
ax2.axhline(y=0.14, color='cyan', linestyle='--', label="Threshold 0.14")
ax2.axhline(y=0.54, color='red', linestyle='--', label="Threshold 0.54")

# Flag to control the main loop
running = True
last_plot_update = 0
PLOT_UPDATE_INTERVAL = 0.1  # Update plot every 100ms

# Online metrics storage
online_metrics = {
    'sii': {'values': [], 'times': []},
    'sgi': {'values': [], 'times': []},
    'rmi': {'values': [], 'times': []}
}

def signal_handler(sig, frame):
    """Handle Ctrl+C signal"""
    global running
    print("\nShutting down gracefully...")
    running = False
    rospy.signal_shutdown("User requested shutdown")

def update_online_metrics():
    """Update online metrics with current values"""
    current_time = time.time()
    online_metrics['sii']['values'].append(current_sii)
    online_metrics['sii']['times'].append(current_time)
    online_metrics['sgi']['values'].append(current_sgi)
    online_metrics['sgi']['times'].append(current_time)
    online_metrics['rmi']['values'].append(current_rmi)
    online_metrics['rmi']['times'].append(current_time)

    # Keep only last 100 points for smooth display
    max_points = 100
    for metric in online_metrics.values():
        if len(metric['values']) > max_points:
            metric['values'] = metric['values'][-max_points:]
            metric['times'] = metric['times'][-max_points:]

def update_plot():
    """Update all plots"""
    global last_plot_update
    
    try:
        current_time = time.time()
        if current_time - last_plot_update < PLOT_UPDATE_INTERVAL:
            return
            
        last_plot_update = current_time
        
        # Update online metrics
        update_online_metrics()
        
        if len(time_data) > 0:
            # Update SII plot
            sii_line.set_data(time_data, sii_data)
            ax1.set_xlim(min(time_data), max(time_data))
            ax1.relim()
            ax1.autoscale_view()

            # Update SGI plot
            sgi_line.set_data(time_data, sgi_data)
            ax2.set_xlim(min(time_data), max(time_data))
            ax2.relim()
            ax2.autoscale_view()

            # Update RMI plot
            rmi_line.set_data(time_data, rmi_data)
            ax3.set_xlim(min(time_data), max(time_data))
            ax3.relim()
            ax3.autoscale_view()

            # Update trajectory plot
            if len(x_positions) > 0:
                # Clear previous trajectory
                line.set_data([], [])
                robot_point.set_data([], [])
                
                # Update with new data
                line.set_data(x_positions, y_positions)
                if len(x_positions) > 0:
                    robot_point.set_data([x_positions[-1]], [y_positions[-1]])

                # Update plot limits
                x_all = x_positions + waypoints[:, 0].tolist()
                y_all = y_positions + waypoints[:, 1].tolist()
                if x_all and y_all:  # Check if lists are not empty
                    ax4.set_xlim(min(x_all) - 1, max(x_all) + 1)
                    ax4.set_ylim(min(y_all) - 1, max(y_all) + 1)

                # Update stats text
                avg_velocity = np.mean(velocities) if velocities else 0
                avg_angular_velocity = np.mean(angular_velocities) if angular_velocities else 0
                current_linear_velocity = velocities[-1] if velocities else 0
                current_angular_velocity = angular_velocities[-1] if angular_velocities else 0
                
                stats_str = f"=== STATISTICS ===\n"
                stats_str += f"- Distance: {total_distance:.2f} m\n"
                stats_str += f"- Avg Linear Velocity: {avg_velocity:.2f} m/s\n"
                stats_str += f"- Current Linear Velocity: {current_linear_velocity:.2f} m/s\n"
                stats_str += f"- Avg Angular Velocity: {avg_angular_velocity:.2f} rad/s\n"
                stats_str += f"- Current Angular Velocity: {current_angular_velocity:.2f} rad/s\n"
                stats_str += f"- Current Indices:\n"
                stats_str += f"  SII: {current_sii:.3f}\n"
                stats_str += f"  SGI: {current_sgi:.3f}\n"
                stats_str += f"  RMI: {current_rmi:.3f}\n"
                stats_str += f"- Time to goals:\n"
                for wp_id, times in waypoint_times.items():
                    if times:
                        time_str = ", ".join(["%.1fs" % t for t in times])
                        stats_str += f"  WP{wp_id}: {time_str}\n"
                stats_text.set_text(stats_str)

            # Draw the plot
            fig.canvas.draw()
            fig.canvas.flush_events()
            
    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")

# ==== Euclidean Distance ====
def distance(x1, y1, x2, y2):
    return math.hypot(x1 - x2, y1 - y2)

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

# ==== Calculate SII: Individual Agents ====
def compute_sii(robot_x, robot_y):
    sii_values = []
    for agent_id, (ax, ay) in agent_positions.items():
        if agent_id in agents_in_groups:
            continue
        h = distance(robot_x, robot_y, ax, ay)
        sii = math.exp(-(h**2) / (2 * sigma_p**2))
        sii_values.append(sii)
    return max(sii_values) if sii_values else 0.0

# ==== Calculate SGI: Agent Groups and Object Interactions ====
def compute_sgi(robot_x, robot_y, groups, interactions):
    sgi_values = []

    # Process groups
    for group in groups:
        if len(group.members) < 2:
            continue

        # Prepare data for circle fitting
        data = Data()
        data.X = []
        data.Y = []
        
        # Collect member positions
        for member in group.members:
            if member.name in agent_positions:
                x, y = agent_positions[member.name]
                data.X.append(x)
                data.Y.append(y)
        
        data.n = len(data.X)
        if data.n < 2:
            continue

        # Fit circle to group members
        circle = circle_fit_by_taubin(data)

        # Calculate SGI for this group
        h = distance(robot_x, robot_y, circle.a, circle.b)
        sgi = math.exp(-(h**2) / (2 * sigma_g**2))
        sgi_values.append(sgi)

    # Process object interactions
    for interaction in interactions:
        # Calculate interaction center as midpoint between person and object
        center_x = (interaction.object_position.position.x + 
                   interaction.participants[0].position.position.x) / 2.0
        center_y = (interaction.object_position.position.y + 
                   interaction.participants[0].position.position.y) / 2.0

        # Calculate radius as half the distance between person and object
        dx = interaction.object_position.position.x - interaction.participants[0].position.position.x
        dy = interaction.object_position.position.y - interaction.participants[0].position.position.y
        radius = math.sqrt(dx*dx + dy*dy) / 2.0

        # Calculate SGI for this interaction
        h = distance(robot_x, robot_y, center_x, center_y)
        sgi = math.exp(-(h**2) / (2 * sigma_o**2))
        sgi_values.append(sgi)

    return max(sgi_values) if sgi_values else 0.0

# ==== Calculate RMI: Relative Motion ====
def compute_rmi(robot_x, robot_y, robot_vx, robot_vy):
    if not robot_pose or not agent_positions:
        return 0.0

    rmi_values = []
    robot_velocity = math.sqrt(robot_vx**2 + robot_vy**2)

    for agent_id, (ax, ay) in agent_positions.items():
        # Calculate relative position vector
        dx = ax - robot_x
        dy = ay - robot_y
        distance = math.sqrt(dx*dx + dy*dy)
        
        if distance < 0.001:  # Avoid division by zero
            continue

        # Calculate angles
        beta = math.atan2(dy, dx)  # Angle between robot heading and robot-to-person vector
        phi = math.atan2(-dy, -dx)  # Angle between person heading and person-to-robot vector

        # Get person velocity
        person_vx, person_vy = agent_velocities.get(agent_id, (0.0, 0.0))
        person_velocity = math.sqrt(person_vx**2 + person_vy**2)

        # Calculate RMI
        rmi = (robot_velocity * math.cos(beta) + person_velocity * math.cos(phi)) / distance
        rmi_values.append(rmi)

    return max(rmi_values) if rmi_values else 0.0

def robot_callback(msg):
    """Callback for robot pose updates"""
    global robot_pose
    robot_pose = msg.pose.pose

def odom_callback(msg):
    """Callback for robot odometry updates"""
    global robot_velocity, total_distance, last_position, start_time, last_wp_visited

    # Get current time
    current_time = rospy.Time.now()
    if start_time is None:
        start_time = current_time
    elapsed_time = (current_time - start_time).to_sec()

    # Get current position
    current_position = (msg.pose.pose.position.x, msg.pose.pose.position.y)
    
    # Calculate distance traveled
    if last_position is not None:
        dx = current_position[0] - last_position[0]
        dy = current_position[1] - last_position[1]
        segment_distance = math.sqrt(dx*dx + dy*dy)
        total_distance += segment_distance

    last_position = current_position

    # Store position and velocity data
    x_positions.append(current_position[0])
    y_positions.append(current_position[1])
    velocities.append(math.sqrt(msg.twist.twist.linear.x**2 + msg.twist.twist.linear.y**2))
    angular_velocities.append(msg.twist.twist.angular.z)  # Store angular velocity
    times.append(elapsed_time)

    # Update robot velocity
    robot_velocity = (msg.twist.twist.linear.x, msg.twist.twist.linear.y)

    # Check waypoint visits
    for i, wp in enumerate(waypoints):
        if distance(current_position[0], current_position[1], wp[0], wp[1]) < 0.5:
            if i != last_wp_visited:
                waypoint_times[i].append(elapsed_time)
                last_wp_visited = i
                if i == len(waypoints) - 1:
                    loop_counter += 1

    # Update plots
    update_plot()

def social_state_callback(msg):
    """Callback for social state updates"""
    global current_sii, current_sgi, current_rmi, agent_positions, agent_velocities, agents_in_groups

    # Skip processing if node is shutting down
    if not running:
        return

    # Update agent positions and velocities
    agent_positions.clear()
    agent_velocities.clear()
    agents_in_groups.clear()

    # Process individual agents
    for person in msg.people.people:
        agent_positions[person.name] = (person.position.position.x, person.position.position.y)
        agent_velocities[person.name] = (person.velocity.linear.x, person.velocity.linear.y)

    # Process groups
    for group in msg.groups.groups:
        for member in group.members:
            agents_in_groups.add(member.name)

    # Calculate indices
    if robot_pose:
        current_sii = compute_sii(robot_pose.position.x, robot_pose.position.y)
        current_sgi = compute_sgi(robot_pose.position.x, robot_pose.position.y, msg.groups.groups, msg.interactions.interactions)
        current_rmi = compute_rmi(robot_pose.position.x, robot_pose.position.y, 
                                robot_velocity[0] if robot_velocity else 0.0,
                                robot_velocity[1] if robot_velocity else 0.0)

        # Store data for plotting
        if start_time is not None:
            elapsed_time = (rospy.Time.now() - start_time).to_sec()
            
            # Ensure all arrays have the same length
            if len(time_data) == len(sii_data) == len(sgi_data) == len(rmi_data):
                time_data.append(elapsed_time)
                sii_data.append(current_sii)
                sgi_data.append(current_sgi)
                rmi_data.append(current_rmi)
            else:
                # If arrays have different lengths, reset them
                time_data.clear()
                sii_data.clear()
                sgi_data.clear()
                rmi_data.clear()
                time_data.append(elapsed_time)
                sii_data.append(current_sii)
                sgi_data.append(current_sgi)
                rmi_data.append(current_rmi)

            # Publish indices only if node is still running
            if running:
                indices_msg = SocialIndices()
                indices_msg.header.stamp = rospy.Time.now()
                indices_msg.sii = current_sii
                indices_msg.sgi = current_sgi
                indices_msg.rmi = current_rmi
                try:
                    indices_pub.publish(indices_msg)
                except rospy.ROSException:
                    pass  # Ignore publish errors during shutdown

def write_to_file():
    """Write metrics to CSV file"""
    try:
        with open(csv_path, 'w') as f:
            f.write("Time,SII,SGI,RMI\n")
            for t, sii, sgi, rmi in zip(time_data, sii_data, sgi_data, rmi_data):
                f.write(f"{t:.2f},{sii:.4f},{sgi:.4f},{rmi:.4f}\n")
        rospy.loginfo(f"Metrics saved to {csv_path}")
    except Exception as e:
        rospy.logerr(f"Error writing to file: {str(e)}")

def main():
    """Main function"""
    global indices_pub, running

    # Initialize ROS node
    rospy.init_node('social_interaction_calculator', anonymous=True)

    # Setup signal handler
    signal.signal(signal.SIGINT, signal_handler)

    # Create publishers
    indices_pub = rospy.Publisher('/social_indices', SocialIndices, queue_size=10)

    # Create subscribers
    rospy.Subscriber('/amcl_pose', PoseWithCovarianceStamped, robot_callback)
    rospy.Subscriber('/odom', Odometry, odom_callback)
    rospy.Subscriber('/social_state', SocialState, social_state_callback)

    # Show initial plot
    plt.show(block=False)

    # Main loop
    rate = rospy.Rate(10)  # 10 Hz
    while not rospy.is_shutdown() and running:
        try:
            update_plot()
            rate.sleep()
        except Exception as e:
            rospy.logerr(f"Error in main loop: {str(e)}")
            break

    # Save data before exiting
    write_to_file()
    plt.close('all')

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass

