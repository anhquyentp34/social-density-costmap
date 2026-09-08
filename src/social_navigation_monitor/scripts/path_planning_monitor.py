#!/usr/bin/env python3

import os
import queue
import select
import sys
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from nav_msgs.msg import Path

path_queue = queue.Queue()
timestamps = []
path_found = []
start_time = None

total_planning_attempts = 0
successful_plans = 0
failed_plans = 0
last_path_time = None
last_no_path_time = None

plt.ion()
fig, ax = plt.subplots(figsize=(14, 8))
fig.canvas.manager.set_window_title("Path Planning Monitor")
ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Path Status", fontsize=12)
ax.set_title("Path Planning Status Timeline", fontsize=14, fontweight="bold")
ax.grid(True, alpha=0.3)
ax.set_ylim(-0.1, 1.1)
ax.set_yticks([0, 1])
ax.set_yticklabels(["No Path", "Path Found"])
path_status_line, = ax.plot([], [], "b-", linewidth=2, marker="o", markersize=4, label="Path Status")
ax.legend(loc="upper right")


def get_message_time_sec(msg):
    if hasattr(msg, "header") and msg.header and msg.header.stamp:
        stamp_sec = msg.header.stamp.to_sec()
        if stamp_sec > 0:
            return stamp_sec
    return rospy.Time.now().to_sec()


def path_callback(msg):
    global start_time, total_planning_attempts, successful_plans, failed_plans
    global last_path_time, last_no_path_time
    msg_time_sec = get_message_time_sec(msg)
    if start_time is None:
        start_time = msg_time_sec
    current_time = max(0.0, msg_time_sec - start_time)
    total_planning_attempts += 1

    if msg.poses:
        path_found_status = True
        successful_plans += 1
        last_path_time = current_time
    else:
        path_found_status = False
        failed_plans += 1
        last_no_path_time = current_time
    path_queue.put((current_time, path_found_status))


def update_plot(_frame):
    try:
        while not path_queue.empty():
            t, found = path_queue.get()
            timestamps.append(t)
            path_found.append(found)

        if timestamps:
            current_time = timestamps[-1]
            path_status_int = [1 if found else 0 for found in path_found]
            path_status_line.set_data(timestamps, path_status_int)
            if len(timestamps) > 1:
                time_range = max(timestamps) - min(timestamps)
                current_time = max(timestamps)
                start_time_plot = min(timestamps)
                if time_range < 10:
                    padding = 2
                elif time_range < 60:
                    padding = 5
                elif time_range < 300:
                    padding = 10
                elif time_range < 1800:
                    padding = 30
                else:
                    padding = 60
                x_min = max(0, start_time_plot - padding)
                x_max = current_time + padding
                if x_max - x_min < 10:
                    x_max = x_min + 10
                ax.set_xlim(x_min, x_max)
                ax.set_xlabel("Time (s)", fontsize=12)

            success_rate = (successful_plans / total_planning_attempts * 100) if total_planning_attempts > 0 else 0
            ax.set_title(
                f"Path Planning Status Timeline - Success Rate: {success_rate:.1f}% ({successful_plans}/{total_planning_attempts}) - Duration: {current_time:.1f}s",
                fontsize=14,
                fontweight="bold",
            )
    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")


def print_statistics():
    if total_planning_attempts > 0:
        success_rate = successful_plans / total_planning_attempts * 100
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
    try:
        if not timestamps:
            rospy.loginfo("No path planning data collected, skip saving timeline image.")
            return
        default_dir = os.path.expanduser("~/move_base_logs")
        default_name = f"path_planning_timeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        save_dir = ask_save_dir_with_timeout(default_dir, timeout_sec=5)
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, default_name)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        rospy.loginfo(f"Saved path planning timeline image: {save_path}")
    except Exception as e:
        rospy.logwarn(f"Failed to save path planning timeline image: {str(e)}")


def main():
    rospy.init_node("path_planning_monitor", anonymous=True)
    rospy.loginfo("Path Planning Monitor da khoi dong")
    rospy.loginfo("Monitoring /move_base/NavfnROS/plan topic")
    rospy.on_shutdown(save_plot_on_shutdown)
    rospy.Subscriber("/move_base/NavfnROS/plan", Path, path_callback)
    _ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    rospy.Timer(rospy.Duration(30.0), lambda _event: print_statistics())
    plt.show(block=True)
    print_statistics()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        print_statistics()
        pass
