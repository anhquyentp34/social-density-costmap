#!/usr/bin/env python3
"""
Serial bridge robot thật: cmd_vel -> VXYW; firmware WheelOdom -> WVEL -> joint_states.

Firmware: MSD_H20_OMNI_Bridge_WheelOdom.ino
  IN:  VXYW <vx_mm_s> <vy_mm_s> <w_mrad_s>
  OUT: WVEL <w_fl> <w_rl> <w_rr> <w_fr>  (mrad/s * 1000, rad/s tại trục bánh)
"""

import threading
import time

import rospy
from geometry_msgs.msg import Twist
from sensor_msgs.msg import JointState

try:
    import serial
except ImportError as exc:  # pragma: no cover
    raise RuntimeError("pyserial is required: sudo apt install python3-serial") from exc

JOINT_FL = "front_left_wheel_link_joint"
JOINT_RL = "rear_left_wheel_link_joint"
JOINT_RR = "rear_right_wheel_link_joint"
JOINT_FR = "front_right_wheel_link_joint"
JOINT_NAMES = (JOINT_FL, JOINT_RL, JOINT_RR, JOINT_FR)
VEL_SCALE = 1.0 / 1000.0  # mrad/s * 1000 -> rad/s
K_LIN = 0.7071
WHEEL_R_DEFAULT = 0.0635
ROBOT_R_DEFAULT = 0.2505
WVEL_STALE_SEC = 0.15  # dùng cmd_vel FK nếu không nhận WVEL mới


class ArduinoInterfaceOmniWheelOdom:
    LIN_SCALE = 1000.0
    ANG_SCALE = 1000.0

    def __init__(self):
        rospy.init_node("arduino_interface_omni_wheel_odom", anonymous=False)

        self.serial_port = rospy.get_param("~serial_port", "/dev/robot_arduino")
        self.baud_rate = int(rospy.get_param("~baud_rate", 19200))
        self.timeout = float(rospy.get_param("~timeout", 0.2))
        self.cmd_vel_topic = rospy.get_param("~cmd_vel_topic", "/cmd_vel")
        self.max_linear = float(rospy.get_param("~max_linear", 0.7))
        self.max_angular = float(rospy.get_param("~max_angular", 1.2))
        self.cmd_timeout = float(rospy.get_param("~cmd_timeout", 0.8))
        self.send_rate_hz = float(rospy.get_param("~send_rate_hz", 20.0))
        self.wheel_joint_states_topic = rospy.get_param(
            "~wheel_joint_states_topic", "/wheel_encoder_joint_states"
        )
        self.wheel_r = float(rospy.get_param("~wheel_radius", WHEEL_R_DEFAULT))
        self.robot_r = float(rospy.get_param("~R", ROBOT_R_DEFAULT))
        self.wvel_stale_sec = float(rospy.get_param("~wvel_stale_sec", WVEL_STALE_SEC))
        self.use_cmd_vel_fallback = bool(rospy.get_param("~use_cmd_vel_fallback", True))

        self.last_cmd = Twist()
        self.last_cmd_time = rospy.Time(0)
        self._cmd_lock = threading.Lock()
        self._ser = None
        self._stop = threading.Event()

        self._wheel_vel = [0.0, 0.0, 0.0, 0.0]
        self._wheel_pos = [0.0, 0.0, 0.0, 0.0]
        self._last_wvel_time = rospy.Time(0)
        self._last_sent_cmd = None
        self._last_vxyw_send = rospy.Time(0)
        self._vxyw_keepalive_sec = float(rospy.get_param("~vxyw_keepalive_sec", 0.5))
        self._vel_lock = threading.Lock()

        self._connect_serial()
        self._js_pub = rospy.Publisher(
            self.wheel_joint_states_topic, JointState, queue_size=10
        )
        self._read_thread = threading.Thread(target=self._serial_read_loop, daemon=True)
        self._read_thread.start()

        rospy.Subscriber(self.cmd_vel_topic, Twist, self._cmd_vel_cb, queue_size=10)

        rospy.loginfo(
            "arduino_interface_omni_wheel_odom: %s @ %d, WVEL -> %s",
            self.serial_port,
            self.baud_rate,
            self.wheel_joint_states_topic,
        )

    def _connect_serial(self):
        self._ser = serial.Serial(
            port=self.serial_port,
            baudrate=self.baud_rate,
            timeout=self.timeout,
            write_timeout=self.timeout,
        )
        time.sleep(0.5)
        self._ser.reset_input_buffer()

    def _cmd_vel_cb(self, msg: Twist):
        with self._cmd_lock:
            self.last_cmd = msg
            self.last_cmd_time = rospy.Time.now()

    @staticmethod
    def _clamp(v, mn, mx):
        return mn if v < mn else mx if v > mx else v

    def _safe_cmd(self):
        now = rospy.Time.now()
        with self._cmd_lock:
            timed_out = (now - self.last_cmd_time).to_sec() > self.cmd_timeout
            msg = Twist() if timed_out else self.last_cmd

        vx = self._clamp(msg.linear.x, -self.max_linear, self.max_linear)
        vy = self._clamp(msg.linear.y, -self.max_linear, self.max_linear)
        wz = self._clamp(msg.angular.z, -self.max_angular, self.max_angular)
        return vx, vy, wz

    def _parse_wvel_line(self, line: str) -> bool:
        line = line.strip()
        if not line.startswith("WVEL"):
            return False
        parts = line.split()
        if len(parts) != 5:
            rospy.logwarn_throttle(2.0, "Bad WVEL line: %s", line)
            return False
        try:
            vals = [int(parts[i]) * VEL_SCALE for i in range(1, 5)]
        except ValueError:
            return False
        with self._vel_lock:
            self._wheel_vel = vals
            self._last_wvel_time = rospy.Time.now()
        return True

    def _fk_wheel_rad_s(self, vx: float, vy: float, wz: float):
        """ω_bánh từ cmd_vel — khớp firmware / wheel_odometry_node."""
        k = K_LIN
        r = self.wheel_r
        R = self.robot_r
        w_fl = (k * vx - k * vy - R * wz) / r
        w_rl = (k * vx + k * vy - R * wz) / r
        w_rr = (-k * vx + k * vy - R * wz) / r
        w_fr = (-k * vx - k * vy - R * wz) / r
        return [w_fl, w_rl, w_rr, w_fr]

    def _serial_read_loop(self):
        buf = ""
        while not self._stop.is_set() and not rospy.is_shutdown():
            try:
                chunk = self._ser.read(self._ser.in_waiting or 1)
                if not chunk:
                    continue
                buf += chunk.decode("ascii", errors="ignore")
            except serial.SerialException as err:
                rospy.logerr_throttle(2.0, "Serial read failed: %s", err)
                time.sleep(0.2)
                continue

            while "\n" in buf:
                line, buf = buf.split("\n", 1)
                line = line.strip("\r")
                if line:
                    self._parse_wvel_line(line)

    def _effective_wheel_velocities(self, vx: float, vy: float, wz: float):
        with self._vel_lock:
            velocities = list(self._wheel_vel)
            wvel_age = (rospy.Time.now() - self._last_wvel_time).to_sec()

        if (
            self.use_cmd_vel_fallback
            and wvel_age > self.wvel_stale_sec
            and (abs(vx) + abs(vy) + abs(wz) > 1e-4)
        ):
            velocities = self._fk_wheel_rad_s(vx, vy, wz)
        return velocities

    def _publish_joint_states(self, dt: float, velocities):
        with self._vel_lock:
            for i in range(4):
                self._wheel_pos[i] += velocities[i] * dt

        js = JointState()
        js.header.stamp = rospy.Time.now()
        js.name = list(JOINT_NAMES)
        js.position = list(self._wheel_pos)
        js.velocity = velocities
        js.effort = []
        self._js_pub.publish(js)

    def run(self):
        rate = rospy.Rate(self.send_rate_hz)
        last_js = rospy.Time.now()

        while not rospy.is_shutdown():
            vx, vy, wz = self._safe_cmd()
            cmd_tuple = (
                int(round(vx * self.LIN_SCALE)),
                int(round(vy * self.LIN_SCALE)),
                int(round(wz * self.ANG_SCALE)),
            )
            now = rospy.Time.now()
            need_send = (
                cmd_tuple != self._last_sent_cmd
                or (now - self._last_vxyw_send).to_sec() >= self._vxyw_keepalive_sec
            )
            if need_send:
                line = f"VXYW {cmd_tuple[0]} {cmd_tuple[1]} {cmd_tuple[2]}\n"
                try:
                    self._ser.write(line.encode("ascii"))
                    self._last_sent_cmd = cmd_tuple
                    self._last_vxyw_send = now
                except serial.SerialException as err:
                    rospy.logerr_throttle(2.0, "Serial write failed: %s", err)

            dt_js = (now - last_js).to_sec()
            if dt_js >= 1.0 / 50.0:
                velocities = self._effective_wheel_velocities(vx, vy, wz)
                self._publish_joint_states(dt_js, velocities)
                last_js = now

            rate.sleep()

        self._stop.set()

    def shutdown(self):
        self._stop.set()


if __name__ == "__main__":
    node = None
    try:
        node = ArduinoInterfaceOmniWheelOdom()
        node.run()
    except rospy.ROSInterruptException:
        pass
    finally:
        if node is not None:
            node.shutdown()
