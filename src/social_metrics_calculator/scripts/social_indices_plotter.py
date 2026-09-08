#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Social Indices Plotter Node
--------------------------
This node subscribes to /sii and /sgi topics and plots the values over time.
It shows both SII and SGI indices in the same window for easy comparison.

Author: Quyen Anh
Date: 2024
"""

import rospy
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
from social_metrics_calculator.msg import SII, SGI
import queue
import time

# ==== Global Variables ====
# Data queues for thread-safe communication
sii_queue = queue.Queue()
sgi_queue = queue.Queue()

# Data storage
sii_times = []
sii_values = []
sii_agents = []
sgi_times = []
sgi_values = []
sgi_groups = []

# ==== Plot Setup ====
plt.ion()  # Turn on interactive mode
fig, (ax1, ax2) = plt.subplots(2, 1, figsize=(12, 8))
fig.canvas.manager.set_window_title("Social Indices Visualization")

# Setup SII plot
ax1.set_xlabel("Time (s)")
ax1.set_ylabel("SII Value")
ax1.set_title("Social Individual Index (SII)")
ax1.grid(True, alpha=0.3)
ax1.set_ylim(0, 1)
sii_line, = ax1.plot([], [], 'b-', label='SII', linewidth=2)
ax1.axhline(y=0.14, color='cyan', linestyle='--', label="Tc 0.14")
ax1.axhline(y=0.54, color='red', linestyle='--', label="Tp 0.54")
ax1.legend(loc='upper right')

# Setup SGI plot
ax2.set_xlabel("Time (s)")
ax2.set_ylabel("SGI Value")
ax2.set_title("Social Group Index (SGI)")
ax2.grid(True, alpha=0.3)
ax2.set_ylim(0, 1)
sgi_line, = ax2.plot([], [], 'r-', label='SGI', linewidth=2)
ax2.axhline(y=0.14, color='cyan', linestyle='--', label="Tg 0.14")
# ax2.axhline(y=0.54, color='red', linestyle='--', label="Tp 0.54")
ax2.legend(loc='upper right')

def sii_callback(msg):
    """Callback for SII messages"""
    # Get current time
    current_time = msg.header.stamp.to_sec()
    
    # Put data in queue
    sii_queue.put((current_time, msg.sii, msg.agent_id))

def sgi_callback(msg):
    """Callback for SGI messages"""
    # Get current time
    current_time = msg.header.stamp.to_sec()
    
    # Put data in queue
    sgi_queue.put((current_time, msg.sgi, msg.group_id))

def update_plot(frame):
    """Update the plot with new data"""
    try:
        # Process SII data
        while not sii_queue.empty():
            t, v, a = sii_queue.get()
            sii_times.append(t)
            sii_values.append(v)
            sii_agents.append(a)

        # Process SGI data
        while not sgi_queue.empty():
            t, v, g = sgi_queue.get()
            sgi_times.append(t)
            sgi_values.append(v)
            sgi_groups.append(g)

        # Update SII plot
        if sii_times:
            sii_line.set_data(sii_times, sii_values)
            ax1.set_xlim(min(sii_times), max(sii_times))
            ax1.relim()
            ax1.autoscale_view()

        # Update SGI plot
        if sgi_times:
            sgi_line.set_data(sgi_times, sgi_values)
            ax2.set_xlim(min(sgi_times), max(sgi_times))
            ax2.relim()
            ax2.autoscale_view()

    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")

def main():
    """Main function"""
    # Initialize ROS node
    rospy.init_node('social_indices_plotter', anonymous=True)
    rospy.loginfo("Social Indices Plotter node started")

    # Create subscribers
    rospy.Subscriber('/sii', SII, sii_callback)
    rospy.Subscriber('/sgi', SGI, sgi_callback)

    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=10)  # Update every 10ms
    
    # Show plot
    plt.show(block=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 