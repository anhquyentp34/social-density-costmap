#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Theo dõi topic cốt lõi khi kiểm tra toàn bộ hệ thống navigation + ZED."""

from __future__ import print_function

import rospy
from diagnostic_msgs.msg import DiagnosticArray, DiagnosticStatus, KeyValue


class SystemHealthMonitor(object):
    BASE_TOPICS = (
        ("/scan", 5.0, 2.0),
        ("/odom", 10.0, 5.0),
        ("/bodies", 5.0, 1.0),
        ("/social_state", 5.0, 1.0),
        ("/amcl_pose", 2.0, 0.5),
        ("/move_base/status", 1.0, 0.2),
    )
    YOLO_TOPICS = (
        ("/yolo_result", 3.0, 0.5),
        ("/objects", 3.0, 0.5),
    )

    def __init__(self):
        rospy.init_node("system_health_monitor", anonymous=False)
        monitor_yolo = rospy.get_param("~monitor_yolo", False)
        self._topics = list(self.BASE_TOPICS)
        if monitor_yolo:
            self._topics.extend(self.YOLO_TOPICS)
        self._last_msg = {}
        self._subs = []
        for topic, _, _ in self._topics:
            self._subs.append(
                rospy.Subscriber(
                    topic,
                    rospy.AnyMsg,
                    self._make_cb(topic),
                    queue_size=1,
                )
            )
        self._pub = rospy.Publisher("/system_health", DiagnosticArray, queue_size=1)
        self._timer = rospy.Timer(rospy.Duration(2.0), self._publish)
        rospy.loginfo(
            "system_health_monitor: theo dõi %d topic (yolo=%s) → /system_health",
            len(self._topics),
            monitor_yolo,
        )

    def _make_cb(self, topic):
        def _cb(_msg):
            self._last_msg[topic] = rospy.Time.now()

        return _cb

    def _publish(self, _event):
        now = rospy.Time.now()
        arr = DiagnosticArray()
        arr.header.stamp = now
        ok_count = 0
        for topic, expect_hz, min_hz in self._topics:
            st = DiagnosticStatus()
            st.name = topic
            last = self._last_msg.get(topic)
            if last is None:
                st.level = DiagnosticStatus.ERROR
                st.message = "Chưa nhận message"
            else:
                age = (now - last).to_sec()
                est_hz = 1.0 / age if age > 1e-6 else expect_hz
                st.hardware_id = "zed_nav_stack"
                st.values = [
                    KeyValue("age_sec", "{:.3f}".format(age)),
                    KeyValue("est_hz", "{:.2f}".format(min(est_hz, expect_hz))),
                ]
                if age > (2.0 / max(min_hz, 0.1)):
                    st.level = DiagnosticStatus.ERROR
                    st.message = "Stale {:.2f}s (cần >= {:.1f} Hz)".format(age, min_hz)
                elif age > (1.5 / max(expect_hz, 0.1)):
                    st.level = DiagnosticStatus.WARN
                    st.message = "Chậm {:.2f}s".format(age)
                    ok_count += 1
                else:
                    st.level = DiagnosticStatus.OK
                    st.message = "OK"
                    ok_count += 1
            arr.status.append(st)

        self._pub.publish(arr)
        if ok_count < len(self._topics):
            rospy.logwarn_throttle(
                5.0,
                "system_health: %d/%d topic OK",
                ok_count,
                len(self._topics),
            )


if __name__ == "__main__":
    try:
        SystemHealthMonitor()
        rospy.spin()
    except rospy.ROSInterruptException:
        pass
