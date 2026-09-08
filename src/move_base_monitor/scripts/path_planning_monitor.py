#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from nav_msgs.msg import Path
from datetime import datetime
import time
import queue
import os
import sys
import select

# ==== Global Variables ====
# Data queue for thread-safe communication
path_queue = queue.Queue()

# Data storage
timestamps = []
path_found = []  # True if path found, False if not
start_time = None

# Statistics
total_planning_attempts = 0
successful_plans = 0
failed_plans = 0
last_path_time = None
last_no_path_time = None

# ==== Plot Setup ====
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(figsize=(14, 8))
fig.canvas.manager.set_window_title("Path Planning Monitor")

# Setup path status plot
ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Path Status", fontsize=12)
ax.set_title("Path Planning Status Timeline", fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.set_ylim(-0.1, 1.1)
ax.set_yticks([0, 1])
ax.set_yticklabels(['No Path', 'Path Found'])

# Create line for plot
path_status_line, = ax.plot([], [], 'b-', linewidth=2, marker='o', markersize=4, label='Path Status')

# Add legend
ax.legend(loc='upper right')

def path_callback(msg):
    """Callback for move_base NavfnROS plan messages"""
    global start_time, total_planning_attempts, successful_plans, failed_plans
    global last_path_time, last_no_path_time
    
    # Initialize start_time on first message
    if start_time is None:
        start_time = time.time()
    
    current_time = time.time() - start_time
    total_planning_attempts += 1
    
    # Check if path is found
    if msg.poses:
        path_found_status = True
        successful_plans += 1
        last_path_time = current_time
        
        rospy.loginfo(f"[{current_time:.2f}s] Path FOUND - Poses: {len(msg.poses)}")
    else:
        path_found_status = False
        failed_plans += 1
        last_no_path_time = current_time
        
        rospy.logwarn(f"[{current_time:.2f}s] NO PATH FOUND")
    
    # Put data in queue
    path_queue.put((current_time, path_found_status))

def update_plot(frame):
    """Update the plot with new data"""
    global timestamps, path_found
    
    try:
        # Process path data
        while not path_queue.empty():
            t, found = path_queue.get()
            timestamps.append(t)
            path_found.append(found)
            
            # Không giới hạn số điểm dữ liệu - giữ toàn bộ lịch sử

        # Update path status plot
        if timestamps:
            current_time = timestamps[-1]
            # Convert boolean to int for plotting
            path_status_int = [1 if found else 0 for found in path_found]
            path_status_line.set_data(timestamps, path_status_int)
            
            # Auto-scale x-axis to show full experiment time
            if len(timestamps) > 1:
                time_range = max(timestamps) - min(timestamps)
                current_time = max(timestamps)
                start_time_plot = min(timestamps)
                
                # Tính toán padding thông minh dựa trên tổng thời gian
                if time_range < 10:  # Dưới 10 giây
                    padding = 2
                elif time_range < 60:  # Dưới 1 phút
                    padding = 5
                elif time_range < 300:  # Dưới 5 phút
                    padding = 10
                elif time_range < 1800:  # Dưới 30 phút
                    padding = 30
                else:  # Trên 30 phút
                    padding = 60
                
                # Đảm bảo hiển thị từ thời điểm bắt đầu đến thời điểm hiện tại + padding
                x_min = max(0, start_time_plot - padding)
                x_max = current_time + padding
                
                # Nếu có ít dữ liệu, đảm bảo hiển thị ít nhất 10 giây
                if x_max - x_min < 10:
                    x_max = x_min + 10
                
                ax.set_xlim(x_min, x_max)
                
                # Format x-axis labels
                if time_range < 60:
                    ax.set_xlabel("Time (s)", fontsize=12)
                elif time_range < 300:
                    ax.set_xlabel("Time (s) - 5s intervals", fontsize=12)
                elif time_range < 1800:
                    ax.set_xlabel("Time (s) - 30s intervals", fontsize=12)
                else:
                    ax.set_xlabel("Time (s) - 60s intervals", fontsize=12)
            
            # Update title with statistics and duration
            success_rate = (successful_plans / total_planning_attempts * 100) if total_planning_attempts > 0 else 0
            duration_str = f"Duration: {current_time:.1f}s" if timestamps else ""
            ax.set_title(f"Path Planning Status Timeline - Success Rate: {success_rate:.1f}% ({successful_plans}/{total_planning_attempts}) - {duration_str}", 
                         fontsize=14, fontweight='bold')
            
            # Add current status annotation
            if path_found:
                current_status = "Path Found" if path_found[-1] else "No Path"
                current_time = timestamps[-1]
                
                # Remove old annotation if exists
                for text in ax.texts:
                    text.remove()
                
                # Add new annotation
                ax.annotate(f'Current: {current_status}', 
                           xy=(current_time, 1 if path_found[-1] else 0),
                           xytext=(10, 10), textcoords='offset points',
                           fontsize=10, fontweight='bold',
                           bbox=dict(boxstyle="round,pad=0.3", 
                                   facecolor="green" if path_found[-1] else "red", 
                                   alpha=0.7),
                           arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0"))

    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")

def print_statistics():
    """Print current statistics"""
    if total_planning_attempts > 0:
        success_rate = (successful_plans / total_planning_attempts * 100)
        rospy.loginfo("=" * 50)
        rospy.loginfo("PATH PLANNING STATISTICS")
        rospy.loginfo("=" * 50)
        rospy.loginfo(f"Total planning attempts: {total_planning_attempts}")
        rospy.loginfo(f"Successful plans: {successful_plans}")
        rospy.loginfo(f"Failed plans: {failed_plans}")
        rospy.loginfo(f"Success rate: {success_rate:.2f}%")
        
        if last_path_time is not None:
            rospy.loginfo(f"Last successful plan: {last_path_time:.2f}s")
        if last_no_path_time is not None:
            rospy.loginfo(f"Last failed plan: {last_no_path_time:.2f}s")
        rospy.loginfo("=" * 50)


def ask_save_dir_with_timeout(default_dir, timeout_sec=5):
    """Ask save directory with timeout to avoid blocking roslaunch shutdown."""
    try:
        if not sys.stdin.isatty():
            return default_dir

        prompt = (
            f"\nNhap thu muc luu anh PNG (Enter de dung mac dinh: {default_dir}). "
            f"Tu dong chon mac dinh sau {timeout_sec}s: "
        )
        print(prompt, end="", flush=True)
        ready, _, _ = select.select([sys.stdin], [], [], timeout_sec)
        if ready:
            user_input = sys.stdin.readline().strip()
            if user_input:
                return os.path.expanduser(user_input)
        print("")
    except Exception:
        pass
    return default_dir


def save_plot_on_shutdown():
    """Write PNG on shutdown."""
    try:
        if not timestamps:
            rospy.loginfo("No path planning data collected, skip saving timeline image.")
            return

        default_dir = os.path.expanduser("~/move_base_logs")
        default_name = f"path_planning_timeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        save_dir = ask_save_dir_with_timeout(default_dir, timeout_sec=5)

        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, default_name)
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        rospy.loginfo(f"Saved path planning timeline image: {save_path}")
    except Exception as e:
        rospy.logwarn(f"Failed to save path planning timeline image: {str(e)}")

def main():
    """Main function"""
    global start_time
    
    # Initialize ROS node
    rospy.init_node('path_planning_monitor', anonymous=True)
    rospy.loginfo("Path Planning Monitor đã khởi động")
    rospy.loginfo("Monitoring /move_base/NavfnROS/plan topic")
    rospy.loginfo("Green = Path Found, Red = No Path")

    rospy.on_shutdown(save_plot_on_shutdown)
    
    # Create subscriber
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, path_callback)
    
    # Create animation
    ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    
    # Set up timer to print statistics every 30 seconds
    def print_stats_timer(event):
        print_statistics()
    
    timer = rospy.Timer(rospy.Duration(30.0), print_stats_timer)
    
    # Show plot
    plt.show(block=True)
    
    # Print final statistics when node is shutting down
    print_statistics()

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        print_statistics()
        pass 