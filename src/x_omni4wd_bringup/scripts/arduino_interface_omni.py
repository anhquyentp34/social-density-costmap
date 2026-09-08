#!/usr/bin/env python3
"""
Serial bridge for real x_omni4wd robot.

Subscribe cmd_vel (Twist: vx, vy, wz) and send firmware command:
  VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>
to MSD_H20_OMNI_Bridge.ino through Arduino USB serial.
"""

import threading
import time

import rospy
from geometry_msgs.msg import Twist

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("pyserial is required: sudo apt install python3-serial") from exc


class ArduinoInterfaceOmni:
    # Scaled integer protocol to reduce UART latency.
    # vx, vy: mm/s  -> scale=1000
    # w:       mrad/s -> scale=1000
    LIN_SCALE = 1000.0
    ANG_SCALE = 1000.0

    def __init__(self):
        rospy.init_node("arduino_interface_omni", anonymous=True)

        self.serial_port = rospy.get_param("~serial_port", "/dev/robot_arduino")
        self.baud_rate = int(rospy.get_param("~baud_rate", 19200))
        self.timeout = float(rospy.get_param("~timeout", 0.2))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.max_linear = float(rospy.get_param("~max_linear", 0.7))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.8))
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self._lock = threading.Lock()
        self._ser = None

        self._connect_serial()
        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=10)

    def _connect_serial(self):
        self._ser = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(0.2)
        rospy.loginfo("arduino_interface_omni connected: %s @ %d", self.serial_port, self.baud_rate)

    def _cmd_vel_cb(self, msg: Twist):
        with self._lock:
            self.last_cmd = msg
            self.last_cmd_time = rospy.Time.now()

    @staticmethod
    def _clamp(v, mn, mx):
        return mn if v < mn else mx if v > mx else v

    def _safe_cmd(self):
        now = rospy.Time.now()
        with self._lock:
            timed_out = (now - self.last_cmd_time).to_sec() > self.cmd_timeout
            msg = Twist() if timed_out else self.last_cmd

        vx = self._clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = self._clamp(msg.linear.y, -self.max_linear, self.max_linear)
        wz = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)
        return vx, vy, wz

    def run(self):
        rate = rospy.Rate(self.send_rate_hz)
        while not rospy.is_shutdown():
            vx, vy, wz = self._safe_cmd()
            vx_i = int(round(vx * self.LIN_SCALE))
            vy_i = int(round(vy * self.LIN_SCALE))
            wz_i = int(round(wz * self.ANG_SCALE))
            line = f"VXYW {vx_i} {vy_i} {wz_i}\n"
            try:
                self._ser.write(line.encode("ascii"))
            except serial.SerialException as err:
                rospy.logerr_throttle(2.0, "Serial write failed: %s", err)
            rate.sleep()


if __name__ == "__main__":
    try:
        node = ArduinoInterfaceOmni()
        node.run()
    except rospy.ROSInterruptException:
        pass
