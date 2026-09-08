#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Gọi stop_motor + zero cmd_vel khi roslaunch/Ctrl+C tắt stack.

Chạy cùng bringup để LiDAR/Arduino không còn quay/chạy sau khi dừng.
Fallback serial DTR nếu ROS service không còn sống.
"""

from __future__ import print_function

import os
import subprocess
import sys

import rospy
from geometry_msgs.msg import Twist
from std_srvs.srv import Empty


def _call_stop_motor_ros():
    for name in ("/stop_motor", "/rplidarNode/stop_motor"):
        try:
            rospy.wait_for_service(name, timeout=0.5)
            rospy.ServiceProxy(name, Empty)()
            rospy.loginfo("hardware_shutdown_guard: called %s", name)
            return True
        except Exception:
            continue
    return False


def _call_stop_motor_serial():
    script = os.path.join(os.path.dirname(os.path.abspath(__file__)), "stop_lidar_motor.py")
    if not os.path.isfile(script):
        # devel/install layout
        try:
            import rospkg

            script = os.path.join(
                rospkg.RosPack().get_path("x_omni4wd_bringup"),
                "scripts",
                "stop_lidar_motor.py",
            )
        except Exception:
            return False
    if not os.path.isfile(script):
        return False
    try:
        r = subprocess.run(
            [sys.executable, script, "--serial-only"],
            timeout=3,
            check=False,
        )
        return r.returncode == 0
    except Exception as e:
        rospy.logwarn("hardware_shutdown_guard: serial fallback lỗi: %s", e)
        return False


def _zero_cmd(pub):
    msg = Twist()
    for _ in range(8):
        pub.publish(msg)
        rospy.sleep(0.05)


def main():
    rospy.init_node("hardware_shutdown_guard")
    cmd_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
    pub = rospy.Publisher(cmd_topic, Twist, queue_size=1, latch=True)

    def on_shutdown():
        rospy.logwarn("hardware_shutdown_guard: shutting down hardware...")
        if not _call_stop_motor_ros():
            _call_stop_motor_serial()
        try:
            _zero_cmd(pub)
        except Exception:
            pass

    rospy.on_shutdown(on_shutdown)
    rospy.loginfo(
        "hardware_shutdown_guard: armed (stop_motor + serial fallback + %s=0)",
        cmd_topic,
    )
    rospy.spin()


if __name__ == "__main__":
    try:
        main()
    except rospy.ROSInterruptException:
        pass
