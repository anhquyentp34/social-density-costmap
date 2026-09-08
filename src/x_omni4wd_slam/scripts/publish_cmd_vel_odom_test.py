#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Gửi tín hiệu cmd_vel có kiểm soát và so sánh các nguồn odom.

Ví dụ:
  rosrun x_omni4wd_slam publish_cmd_vel_odom_test.py _profile:=straight
  rosrun x_omni4wd_slam publish_cmd_vel_odom_test.py _profile:=rotation _wz:=0.3 _duration:=5.0
  rosrun x_omni4wd_slam publish_cmd_vel_odom_test.py _profile:=square

Profiles: static | straight | lateral | rotation | square
"""

from __future__ import annotations

import math
import threading

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry


def yaw_from_odom(msg: Odometry) -> float:
    q = msg.pose.pose.orientation
    # yaw from quaternion (z,w dominant in 2D)
    siny_cosp = 2.0 * (q.w * q.z + q.x * q.y)
    cosy_cosp = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny_cosp, cosy_cosp)


def wrap_pi(a: float) -> float:
    while a > math.pi:
        a -= 2.0 * math.pi
    while a < -math.pi:
        a += 2.0 * math.pi
    return a


class OdomProbe:
    def __init__(self, topic: str):
        self.topic = topic
        self.lock = threading.Lock()
        self.msg = None
        self.count = 0
        self.sub = rospy.Subscriber(topic, Odometry, self._cb, queue_size=20)

    def _cb(self, msg: Odometry):
        with self.lock:
            self.msg = msg
            self.count += 1

    def snapshot(self):
        with self.lock:
            if self.msg is None:
                return None
            p = self.msg.pose.pose.position
            return {
                "x": p.x,
                "y": p.y,
                "yaw": yaw_from_odom(self.msg),
                "count": self.count,
            }


class CmdVelOdomTest:
    def __init__(self):
        self.cmd_topic = rospy.get_param("~cmd_topic", "/cmd_vel")
        self.profile = str(rospy.get_param("~profile", "straight")).strip().lower()
        self.rate_hz = float(rospy.get_param("~rate", 30.0))
        self.vx = float(rospy.get_param("~vx", 0.15))
        self.vy = float(rospy.get_param("~vy", 0.15))
        self.wz = float(rospy.get_param("~wz", 0.30))
        self.duration = float(rospy.get_param("~duration", 4.0))
        self.side_duration = float(rospy.get_param("~side_duration", 3.0))
        self.pause_sec = float(rospy.get_param("~pause_sec", 0.8))
        self.settle_sec = float(rospy.get_param("~settle_sec", 1.0))

        topics = rospy.get_param(
            "~odom_topics",
            ["/odom_wheel", "/odom_rf2o", "/odom_zed", "/odometry/filtered"],
        )
        self.probes = {t: OdomProbe(t) for t in topics}
        self.pub = rospy.Publisher(self.cmd_topic, Twist, queue_size=10)

        rospy.loginfo(
            "cmd_vel odom test: topic=%s profile=%s vx=%.2f vy=%.2f wz=%.2f",
            self.cmd_topic,
            self.profile,
            self.vx,
            self.vy,
            self.wz,
        )
        rospy.logwarn("Robot sẽ chuyển động. Đảm bảo không gian trống quanh robot.")

    def _publish_twist(self, vx=0.0, vy=0.0, wz=0.0, duration=0.0):
        rate = rospy.Rate(self.rate_hz)
        t0 = rospy.Time.now()
        msg = Twist()
        msg.linear.x = vx
        msg.linear.y = vy
        msg.angular.z = wz
        while not rospy.is_shutdown():
            elapsed = (rospy.Time.now() - t0).to_sec()
            if duration > 0.0 and elapsed >= duration:
                break
            self.pub.publish(msg)
            rate.sleep()

    def _stop(self, duration=0.5):
        self._publish_twist(0.0, 0.0, 0.0, duration)

    def _wait_topics(self, timeout=15.0):
        t0 = rospy.Time.now()
        while not rospy.is_shutdown():
            ready = all(p.snapshot() is not None for p in self.probes.values())
            if ready:
                return True
            if (rospy.Time.now() - t0).to_sec() > timeout:
                missing = [t for t, p in self.probes.items() if p.snapshot() is None]
                rospy.logwarn("Timeout chờ odom: %s", ", ".join(missing))
                return False
            rospy.sleep(0.1)
        return False

    def _capture(self):
        return {t: p.snapshot() for t, p in self.probes.items()}

    def _delta(self, before, after):
        out = {}
        for t in self.probes:
            b, a = before.get(t), after.get(t)
            if b is None or a is None:
                out[t] = None
                continue
            out[t] = {
                "dx": a["x"] - b["x"],
                "dy": a["y"] - b["y"],
                "dyaw": wrap_pi(a["yaw"] - b["yaw"]),
                "dist": math.hypot(a["x"] - b["x"], a["y"] - b["y"]),
                "x0": b["x"],
                "y0": b["y"],
                "yaw0": b["yaw"],
                "x1": a["x"],
                "y1": a["y"],
                "yaw1": a["yaw"],
            }
        return out

    def _print_report(self, label, deltas, expected=None):
        rospy.loginfo("======== ODOM RESULT: %s ========", label)
        if expected:
            rospy.loginfo(
                "Expected (rough): dist≈%.2f m, dyaw≈%.2f rad",
                expected.get("dist", 0.0),
                expected.get("dyaw", 0.0),
            )
        for t, d in deltas.items():
            if d is None:
                rospy.logwarn("%-22s  NO DATA", t)
                continue
            rospy.loginfo(
                "%-22s  dx=%+7.3f  dy=%+7.3f  dist=%6.3f  dyaw=%+7.3f rad (%+6.1f deg)",
                t,
                d["dx"],
                d["dy"],
                d["dist"],
                d["dyaw"],
                math.degrees(d["dyaw"]),
            )
        # So sánh với filtered nếu có
        ref = deltas.get("/odometry/filtered")
        if ref:
            rospy.loginfo("--- vs /odometry/filtered ---")
            for t, d in deltas.items():
                if t == "/odometry/filtered" or d is None:
                    continue
                rospy.loginfo(
                    "%-22s  Δdist=%+6.3f m  Δyaw=%+6.3f rad",
                    t,
                    d["dist"] - ref["dist"],
                    wrap_pi(d["dyaw"] - ref["dyaw"]),
                )

    def run_profile(self):
        if not self._wait_topics():
            rospy.logerr("Không đủ topic odom — abort")
            return 1

        rospy.sleep(self.settle_sec)
        before = self._capture()

        if self.profile == "static":
            self._stop(self.duration)
            expected = {"dist": 0.0, "dyaw": 0.0}
        elif self.profile == "straight":
            self._publish_twist(self.vx, 0.0, 0.0, self.duration)
            expected = {"dist": abs(self.vx) * self.duration, "dyaw": 0.0}
        elif self.profile == "lateral":
            self._publish_twist(0.0, self.vy, 0.0, self.duration)
            expected = {"dist": abs(self.vy) * self.duration, "dyaw": 0.0}
        elif self.profile == "rotation":
            self._publish_twist(0.0, 0.0, self.wz, self.duration)
            expected = {"dist": 0.0, "dyaw": self.wz * self.duration}
        elif self.profile == "square":
            # 4 cạnh tiến + 4 góc quay ~90°
            side_t = self.side_duration
            turn_t = (math.pi / 2.0) / max(abs(self.wz), 1e-3)
            for i in range(4):
                rospy.loginfo("Square side %d/4", i + 1)
                self._publish_twist(self.vx, 0.0, 0.0, side_t)
                self._stop(self.pause_sec)
                self._publish_twist(0.0, 0.0, self.wz, turn_t)
                self._stop(self.pause_sec)
            expected = {
                "dist": 0.0,  # lý tưởng đóng quỹ đạo
                "dyaw": 2.0 * math.pi * math.copysign(1.0, self.wz),
            }
        else:
            rospy.logerr("Unknown profile: %s", self.profile)
            return 2

        self._stop(0.6)
        rospy.sleep(self.settle_sec)
        after = self._capture()
        deltas = self._delta(before, after)
        self._print_report(self.profile, deltas, expected)
        return 0


def main():
    rospy.init_node("publish_cmd_vel_odom_test")
    node = CmdVelOdomTest()
    code = node.run_profile()
    # gửi thêm stop
    node._stop(0.3)
    return code


if __name__ == "__main__":
    try:
        raise SystemExit(main())
    except rospy.ROSInterruptException:
        pass
