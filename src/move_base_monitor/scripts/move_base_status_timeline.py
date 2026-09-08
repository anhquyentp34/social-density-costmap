#!/usr/bin/env python3

import rospy
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import numpy as np
from actionlib_msgs.msg import GoalStatusArray
from datetime import datetime
import time
import queue
import os
import sys
import select

# ==== Global Variables ====
# Data queue for thread-safe communication
status_queue = queue.Queue()

# Data storage
timestamps = []
status_codes = []
start_time = None

# Mapping status codes
status_names = {
    0: "PENDING",      # The goal has yet to be processed by the action server
    1: "ACTIVE",       # The goal is currently being processed by the action server
    2: "PREEMPTED",    # The goal received a cancel request after it started executing and has since completed its execution (Terminal State)
    3: "SUCCEEDED",    # The goal was achieved successfully by the action server (Terminal State)
    4: "ABORTED",      # The goal was aborted during execution by the action server due to some failure (Terminal State)
    5: "REJECTED",     # The goal was rejected by the action server without being processed, because the goal was unattainable or invalid (Terminal State)
    6: "PREEMPTING",   # The goal received a cancel request after it started executing and has not yet completed execution
    7: "RECALLING",    # The goal received a cancel request before it started executing, but the action server has not yet confirmed that the goal is canceled
    8: "RECALLED",     # The goal received a cancel request before it started executing and was successfully cancelled (Terminal State)
    9: "LOST"          # An action client can determine that a goal is LOST. This should not be sent over the wire by an action server
}

# Terminal states
terminal_states = [2, 3, 4, 5, 8]  # PREEMPTED, SUCCEEDED, ABORTED, REJECTED, RECALLED

# ==== Plot Setup ====
plt.ion()  # Turn on interactive mode
fig, ax = plt.subplots(figsize=(3.5, 2))
fig.canvas.manager.set_window_title("Move Base Status Timeline")

# Setup main status plot
ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Status", fontsize=12)
ax.set_title("Move Base Status Timeline", fontsize=14, fontweight='bold')
ax.grid(True, alpha=0.3)
ax.set_ylim(-1, 10)
ax.set_yticks(list(status_names.keys()))
ax.set_yticklabels(list(status_names.values()))

# Create line for status timeline
status_line, = ax.plot([], [], 'b-', linewidth=2, marker='o', markersize=4, label='Status')
ax.legend(loc='upper right')

# Thêm annotation cho trạng thái terminal
for state in terminal_states:
    ax.axhline(y=state, color='gray', linestyle=':', alpha=0.3)

def status_callback(msg):
    """Callback for move_base status messages"""
    global start_time
    
    if msg.status_list:
        status = msg.status_list[0]
        
        # Initialize start time on first message
        if start_time is None:
            start_time = time.time()
        
        current_time = time.time() - start_time
        
        # Put data in queue
        status_queue.put((current_time, status.status))

def update_plot(frame):
    """Update the plot with new data"""
    try:
        # Process status data
        while not status_queue.empty():
            t, s = status_queue.get()
            timestamps.append(t)
            status_codes.append(s)
            
            # Không giới hạn số điểm dữ liệu - giữ toàn bộ lịch sử

        # Update main status plot
        if timestamps:
            status_line.set_data(timestamps, status_codes)
            
            # Auto-scale x-axis to show full experiment time
            if len(timestamps) > 1:
                # Tính toán range thời gian
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
                
                # Format x-axis labels based on time range
                if time_range < 60:
                    ax.set_xlabel("Time (s)", fontsize=12)
                elif time_range < 300:
                    ax.set_xlabel("Time (s) - 5s intervals", fontsize=12)
                elif time_range < 1800:
                    ax.set_xlabel("Time (s) - 30s intervals", fontsize=12)
                else:
                    ax.set_xlabel("Time (s) - 60s intervals", fontsize=12)
                
                # Cập nhật title với thông tin thời gian
                duration_str = f"Duration: {current_time:.1f}s"
                ax.set_title(f"Move Base Status Timeline - {duration_str}", 
                           fontsize=14, fontweight='bold')
            
            # Highlight current status
            if status_codes:
                current_status = status_codes[-1]
                current_status_name = status_names.get(current_status, "UNKNOWN")
                
                # Remove old horizontal line if exists
                for line in ax.lines[1:]:  # Keep the main status line and terminal state lines
                    if line.get_linestyle() == '--':  # Only remove current status line
                        line.remove()
                
                # Add new horizontal line for current status
                ax.axhline(y=current_status, color='red', linestyle='--', alpha=0.7, 
                          linewidth=2, label=f'Current: {current_status_name}')
                
                # Update legend
                ax.legend(loc='upper right')
                
                # Add status change annotations
                if len(status_codes) > 1 and status_codes[-1] != status_codes[-2]:
                    # Add annotation for status change
                    ax.annotate(f'{current_status_name}', 
                               xy=(timestamps[-1], current_status),
                               xytext=(10, 10), textcoords='offset points',
                               fontsize=10, fontweight='bold',
                               bbox=dict(boxstyle="round,pad=0.3", facecolor="yellow", alpha=0.7),
                               arrowprops=dict(arrowstyle="->", connectionstyle="arc3,rad=0"))

    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")


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
            rospy.loginfo("No status data collected, skip saving timeline image.")
            return

        default_dir = os.path.expanduser("~/move_base_logs")
        default_name = f"move_base_status_timeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        save_dir = ask_save_dir_with_timeout(default_dir, timeout_sec=5)

        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, default_name)
        fig.savefig(save_path, dpi=150, bbox_inches='tight')
        rospy.loginfo(f"Saved move_base status timeline image: {save_path}")
    except Exception as e:
        rospy.logwarn(f"Failed to save move_base status timeline image: {str(e)}")

def main():
    """Main function"""
    global start_time
    
    # Initialize ROS node
    rospy.init_node('move_base_status_timeline', anonymous=True)
    rospy.loginfo("Move Base Status Timeline đã khởi động")
    rospy.loginfo("Status codes:")
    for code, name in status_names.items():
        rospy.loginfo(f"  {code}: {name}")

    rospy.on_shutdown(save_plot_on_shutdown)
    
    # Create subscriber
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)
    
    # Create animation with save_count to avoid warning
    ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)  # Update every 100ms
    
    # Show plot
    plt.show(block=True)

if __name__ == '__main__':
    try:
        main()
    except rospy.ROSInterruptException:
        pass 