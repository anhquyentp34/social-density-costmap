#!/usr/bin/env python3
"""Scale SRF odometry xy (pose + twist) trước EKF — hiệu chỉnh hành trình dài.

Đặt scale_xy từ GT T1: scale = gt_distance / odom_distance
  (nếu odom dài hơn thật → scale < 1)

GT T1 2026-08-07 (3 runs): mean(odom/gt) ≈ 1.0345 → scale_xy ≈ 0.9667
"""
from __future__ import annotations

import copy

import rospy
from nav_msgs.msg import Odometry


def main():
    rospy.init_node("odom_xy_scale")
    in_topic = rospy.get_param("~input", "/odom_srf")
    out_topic = rospy.get_param("~output", "/odom_srf_scaled")
    scale_xy = float(rospy.get_param("~scale_xy", 0.9667))
    scale_vx = float(rospy.get_param("~scale_vx", scale_xy))
    scale_vy = float(rospy.get_param("~scale_vy", scale_xy))

    pub = rospy.Publisher(out_topic, Odometry, queue_size=20)
    last_in = None
    last_out_x = 0.0
    last_out_y = 0.0
    init = False

    def cb(msg: Odometry):
        nonlocal last_in, last_out_x, last_out_y, init
        out = copy.deepcopy(msg)
        x = msg.pose.pose.position.x
        y = msg.pose.pose.position.y
        if not init or last_in is None:
            last_out_x, last_out_y = x, y
            init = True
        else:
            dx = x - last_in[0]
            dy = y - last_in[1]
            last_out_x += dx * scale_xy
            last_out_y += dy * scale_xy
        last_in = (x, y)
        out.pose.pose.position.x = last_out_x
        out.pose.pose.position.y = last_out_y
        out.twist.twist.linear.x = msg.twist.twist.linear.x * scale_vx
        out.twist.twist.linear.y = msg.twist.twist.linear.y * scale_vy
        pub.publish(out)

    rospy.Subscriber(in_topic, Odometry, cb, queue_size=20)
    rospy.loginfo(
        "odom_xy_scale: %s → %s  scale_xy=%.4f vx=%.4f vy=%.4f",
        in_topic,
        out_topic,
        scale_xy,
        scale_vx,
        scale_vy,
    )
    rospy.spin()


if __name__ == "__main__":
    main()
