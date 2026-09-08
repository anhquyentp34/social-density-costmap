#!/usr/bin/env python3

import os
import queue
import select
import sys
from datetime import datetime

import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import rospy
from actionlib_msgs.msg import GoalStatusArray

status_queue = queue.Queue()
timestamps = []
status_codes = []
start_time = None

status_names = {
    0: "PENDING",
    1: "ACTIVE",
    2: "PREEMPTED",
    3: "SUCCEEDED",
    4: "ABORTED",
    5: "REJECTED",
    6: "PREEMPTING",
    7: "RECALLING",
    8: "RECALLED",
    9: "LOST",
}
terminal_states = [2, 3, 4, 5, 8]

plt.ion()
fig, ax = plt.subplots(figsize=(3.5, 2))
fig.canvas.manager.set_window_title("Move Base Status Timeline")
ax.set_xlabel("Time (s)", fontsize=12)
ax.set_ylabel("Status", fontsize=12)
ax.set_title("Move Base Status Timeline", fontsize=14, fontweight="bold")
ax.grid(True, alpha=0.3)
ax.set_ylim(-1, 10)
ax.set_yticks(list(status_names.keys()))
ax.set_yticklabels(list(status_names.values()))
status_line, = ax.plot([], [], "b-", linewidth=2, marker="o", markersize=4, label="Status")
ax.legend(loc="upper right")
for state in terminal_states:
    ax.axhline(y=state, color="gray", linestyle=":", alpha=0.3)


def get_message_time_sec(msg):
    if hasattr(msg, "header") and msg.header and msg.header.stamp:
        stamp_sec = msg.header.stamp.to_sec()
        if stamp_sec > 0:
            return stamp_sec
    return rospy.Time.now().to_sec()


def status_callback(msg):
    global start_time
    if msg.status_list:
        status = msg.status_list[0]
        msg_time_sec = get_message_time_sec(msg)
        if start_time is None:
            start_time = msg_time_sec
        current_time = max(0.0, msg_time_sec - start_time)
        status_queue.put((current_time, status.status))


def update_plot(_frame):
    try:
        while not status_queue.empty():
            t, s = status_queue.get()
            timestamps.append(t)
            status_codes.append(s)

        if timestamps:
            status_line.set_data(timestamps, status_codes)
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
                ax.set_title(
                    f"Move Base Status Timeline - Duration: {current_time:.1f}s",
                    fontsize=14,
                    fontweight="bold",
                )

            if status_codes:
                current_status = status_codes[-1]
                current_status_name = status_names.get(current_status, "UNKNOWN")
                for line in ax.lines[1:]:
                    if line.get_linestyle() == "--":
                        line.remove()
                ax.axhline(
                    y=current_status,
                    color="red",
                    linestyle="--",
                    alpha=0.7,
                    linewidth=2,
                    label=f"Current: {current_status_name}",
                )
                ax.legend(loc="upper right")
    except Exception as e:
        rospy.logwarn(f"Error updating plot: {str(e)}")


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
            rospy.loginfo("No status data collected, skip saving timeline image.")
            return
        default_dir = os.path.expanduser("~/move_base_logs")
        default_name = f"move_base_status_timeline_{datetime.now().strftime('%Y%m%d_%H%M%S')}.png"
        save_dir = ask_save_dir_with_timeout(default_dir, timeout_sec=5)
        os.makedirs(save_dir, exist_ok=True)
        save_path = os.path.join(save_dir, default_name)
        fig.savefig(save_path, dpi=150, bbox_inches="tight")
        rospy.loginfo(f"Saved move_base status timeline image: {save_path}")
    except Exception as e:
        rospy.logwarn(f"Failed to save move_base status timeline image: {str(e)}")


def main():
    rospy.init_node("move_base_status_timeline", anonymous=True)
    rospy.loginfo("Move Base Status Timeline da khoi dong")
    rospy.on_shutdown(save_plot_on_shutdown)
    rospy.Subscriber("/move_base/status", GoalStatusArray, status_callback)
    _ani = FuncAnimation(fig, update_plot, interval=100, cache_frame_data=False)
    plt.show(block=True)


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
