#!/usr/bin/env python3
"""P10: publish /odom_livo from /odometry/filtered + latency diagnostic.

Also optional relay to /odom (if ~relay_to_odom).
Reports /odom_livo/latency_ms (Float32) = now - header.stamp.
"""
from __future__ import annotations

import rospy
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32


class OdomLivoRelay:
    def __init__(self):
        self.src = rospy.get_param("~source", "/odometry/filtered")
        self.relay_odom = bool(rospy.get_param("~relay_to_odom", False))
        self.pub = rospy.Publisher("/odom_livo", Odometry, queue_size=20)
        self.pub_lat = rospy.Publisher("/odom_livo/latency_ms", Float32, queue_size=20)
        if self.relay_odom:
            self.pub_odom = rospy.Publisher("/odom", Odometry, queue_size=20)
        rospy.Subscriber(self.src, Odometry, self._cb, queue_size=20)
        rospy.loginfo("odom_livo_relay: %s → /odom_livo (relay_odom=%s)", self.src, self.relay_odom)

    def _cb(self, msg: Odometry):
        now = rospy.Time.now()
        stamp = msg.header.stamp
        if stamp.to_sec() > 0:
            lat_ms = (now - stamp).to_sec() * 1000.0
            self.pub_lat.publish(Float32(data=lat_ms))
        self.pub.publish(msg)
        if self.relay_odom:
            self.pub_odom.publish(msg)


def main():
    rospy.init_node("odom_livo_relay")
    OdomLivoRelay()
    rospy.spin()


if __name__ == "__main__":
    main()
