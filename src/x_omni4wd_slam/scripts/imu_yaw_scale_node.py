#!/usr/bin/env python3
"""Scale IMU yaw-rate trước khi vào EKF (hiệu chỉnh scale quay từ GT thước).

GT T2 2026-08-06 (srf_zed_imu, 3 runs ~180°):
  mean(odom_yaw/gt_yaw) ≈ 1.012 → scale_wz = 1/1.012 ≈ 0.988
"""
from __future__ import annotations

import rospy
from sensor_msgs.msg import Imu


def main():
    rospy.init_node("imu_yaw_scale")
    in_topic = rospy.get_param("~input", "/zed_imu")
    out_topic = rospy.get_param("~output", "/zed_imu_scaled")
    # < 1.0 nếu odom over-estimate góc
    scale_wz = float(rospy.get_param("~scale_wz", 0.988))

    pub = rospy.Publisher(out_topic, Imu, queue_size=100)

    def cb(msg: Imu):
        out = Imu()
        out.header = msg.header
        out.orientation = msg.orientation
        out.orientation_covariance = msg.orientation_covariance
        out.angular_velocity = msg.angular_velocity
        out.angular_velocity.z = msg.angular_velocity.z * scale_wz
        out.angular_velocity_covariance = msg.angular_velocity_covariance
        out.linear_acceleration = msg.linear_acceleration
        out.linear_acceleration_covariance = msg.linear_acceleration_covariance
        pub.publish(out)

    rospy.Subscriber(in_topic, Imu, cb, queue_size=100)
    rospy.loginfo(
        "imu_yaw_scale: %s → %s  scale_wz=%.4f", in_topic, out_topic, scale_wz
    )
    rospy.spin()


if __name__ == "__main__":
    main()
