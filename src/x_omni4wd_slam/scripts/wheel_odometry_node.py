#!/usr/bin/env python3
"""
Wheel odometry for x_omni4wd (thay RF2O).

Đọc vận tốc góc 4 bánh từ sensor_msgs/JointState, giải động học thuận
(Jacobian chuẩn — khớp cmd_vel_to_omni_wheels.py / MSD_H20_OMNI_Bridge.ino),
tích phân pose 2D và publish nav_msgs/Odometry + TF odom -> base_footprint.

Yêu cầu: joint_states phải có trường velocity (Gazebo ros_control hoặc encoder thật).
"""

import math

import rospy
import tf
from geometry_msgs.msg import Quaternion
from nav_msgs.msg import Odometry
from sensor_msgs.msg import JointState

K_LIN = 0.7071  # 1/sqrt(2)

# Mặc định khớp URDF x_omni4wd_description + controllers.yaml (Gazebo ros_control)
DEFAULT_JOINT_FL = "front_left_wheel_link_joint"
DEFAULT_JOINT_RL = "rear_left_wheel_link_joint"
DEFAULT_JOINT_RR = "rear_right_wheel_link_joint"
DEFAULT_JOINT_FR = "front_right_wheel_link_joint"


class WheelOdometryNode:
    def __init__(self):
        rospy.init_node("wheel_odometry", anonymous=False)

        self.wheel_radius = float(rospy.get_param("~wheel_radius", 0.0635))
        self.robot_r = float(rospy.get_param("~R", 0.2505))
        self.gear_ratio = float(rospy.get_param("~gear_ratio", 1.0))
        self.odom_frame = rospy.get_param("~odom_frame_id", "odom")
        self.base_frame = rospy.get_param("~base_frame_id", "base_footprint")
        self.odom_topic = rospy.get_param("~odom_topic", "/odom")
        self.joint_states_topic = rospy.get_param("~joint_states_topic", "/joint_states")
        self.publish_tf = bool(rospy.get_param("~publish_tf", True))
        self.rate_hz = float(rospy.get_param("~rate", 50.0))

        self.sign_fl = float(rospy.get_param("~sign_fl", 1.0))
        self.sign_rl = float(rospy.get_param("~sign_rl", 1.0))
        self.sign_rr = float(rospy.get_param("~sign_rr", 1.0))
        self.sign_fr = float(rospy.get_param("~sign_fr", 1.0))

        self.joint_fl = rospy.get_param("~joint_fl", DEFAULT_JOINT_FL)
        self.joint_rl = rospy.get_param("~joint_rl", DEFAULT_JOINT_RL)
        self.joint_rr = rospy.get_param("~joint_rr", DEFAULT_JOINT_RR)
        self.joint_fr = rospy.get_param("~joint_fr", DEFAULT_JOINT_FR)

        self.pose_x = 0.0
        self.pose_y = 0.0
        self.pose_yaw = 0.0
        self.last_time = None
        self.wheel_omega = [0.0, 0.0, 0.0, 0.0]  # FL, RL, RR, FR (rad/s at wheel)
        self.have_wheel_vel = False
        self._last_fk_log = rospy.Time(0)

        self.odom_pub = rospy.Publisher(self.odom_topic, Odometry, queue_size=10)
        self.tf_broadcaster = tf.TransformBroadcaster() if self.publish_tf else None

        rospy.Subscriber(
            self.joint_states_topic,
            JointState,
            self._joint_states_cb,
            queue_size=10,
        )

        rospy.loginfo(
            "wheel_odometry: topic=%s <- %s, r=%.4f R=%.4f gear=%.2f TF=%s",
            self.odom_topic,
            self.joint_states_topic,
            self.wheel_radius,
            self.robot_r,
            self.gear_ratio,
            self.publish_tf,
        )
        rospy.loginfo(
            "  joints FL=%s RL=%s RR=%s FR=%s",
            self.joint_fl,
            self.joint_rl,
            self.joint_rr,
            self.joint_fr,
        )

    @staticmethod
    def _find_velocity(msg, joint_name):
        for idx, name in enumerate(msg.name):
            if name == joint_name and idx < len(msg.velocity):
                return msg.velocity[idx]
        return None

    def _joint_states_cb(self, msg):
        names = (self.joint_fl, self.joint_rl, self.joint_rr, self.joint_fr)
        signs = (self.sign_fl, self.sign_rl, self.sign_rr, self.sign_fr)
        omegas = []
        for joint_name, sign in zip(names, signs):
            vel = self._find_velocity(msg, joint_name)
            if vel is None:
                return
            omegas.append(sign * vel / self.gear_ratio)

        self.wheel_omega = omegas
        self.have_wheel_vel = True

    def forward_kinematics(self, w_fl, w_rl, w_rr, w_fr):
        """Jacobian chuẩn (nghịch của cmd_vel_to_omni_wheels)."""
        r = self.wheel_radius
        k = K_LIN
        r_robot = self.robot_r

        vx = r * (w_fl + w_rl - w_rr - w_fr) / (4.0 * k)
        vy = r * (-w_fl + w_rl + w_rr - w_fr) / (4.0 * k)
        wz = -r * (w_fl + w_rl + w_rr + w_fr) / (4.0 * r_robot)
        return vx, vy, wz

    def _integrate(self, vx, vy, wz, dt):
        c = math.cos(self.pose_yaw)
        s = math.sin(self.pose_yaw)
        self.pose_x += (vx * c - vy * s) * dt
        self.pose_y += (vx * s + vy * c) * dt
        self.pose_yaw += wz * dt
        self.pose_yaw = math.atan2(math.sin(self.pose_yaw), math.cos(self.pose_yaw))

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            now = rospy.Time.now()
            vx = vy = wz = 0.0
            if self.last_time is not None and self.have_wheel_vel:
                dt = (now - self.last_time).to_sec()
                if 0.0 < dt < 1.0:
                    w_fl, w_rl, w_rr, w_fr = self.wheel_omega
                    vx, vy, wz = self.forward_kinematics(w_fl, w_rl, w_rr, w_fr)
                    self._integrate(vx, vy, wz, dt)
                    w_max = max(abs(w) for w in self.wheel_omega)
                    if w_max > 0.05 and abs(vx) + abs(vy) + abs(wz) < 1e-4:
                        now_log = rospy.Time.now()
                        if (now_log - self._last_fk_log).to_sec() > 5.0:
                            rospy.logwarn(
                                "wheel_odometry: |omega_banh|=%.3f nhưng vx,vy,wz~0 — "
                                "kiểm tra sign_* hoặc WVEL/encoder",
                                w_max,
                            )
                            self._last_fk_log = now_log

            # Luôn publish odom/TF (pose ban đầu 0,0,0) để RViz RobotModel có map->odom->base_footprint
            self._publish_odom(now, vx, vy, wz)
            self.last_time = now
            rate.sleep()

    def _publish_odom(self, stamp, vx, vy, wz):
        odom = Odometry()
        odom.header.stamp = stamp
        odom.header.frame_id = self.odom_frame
        odom.child_frame_id = self.base_frame

        odom.pose.pose.position.x = self.pose_x
        odom.pose.pose.position.y = self.pose_y
        odom.pose.pose.position.z = 0.0

        q = tf.transformations.quaternion_from_euler(0.0, 0.0, self.pose_yaw)
        odom.pose.pose.orientation = Quaternion(*q)

        odom.twist.twist.linear.x = vx
        odom.twist.twist.linear.y = vy
        odom.twist.twist.angular.z = wz

        # Covariance — tune 2026-08-04 (benchmark 1.2m manual, run_004):
        #   wheel +5..11% tiến/lùi, +16..22% ngang, quay lệch → tăng twist R (EKF ít tin hơn)
        odom.pose.covariance[0] = 0.25
        odom.pose.covariance[7] = 0.25
        odom.pose.covariance[35] = 0.22
        odom.twist.covariance[0] = 0.32
        odom.twist.covariance[7] = 0.32
        odom.twist.covariance[35] = 0.28

        self.odom_pub.publish(odom)

        if self.tf_broadcaster is not None:
            self.tf_broadcaster.sendTransform(
                (self.pose_x, self.pose_y, 0.0),
                q,
                stamp,
                self.base_frame,
                self.odom_frame,
            )


if __name__ == "__main__":
    try:
        WheelOdometryNode().spin()
    except rospy.ROSInterruptException:
        pass
