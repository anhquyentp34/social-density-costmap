#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Velocity helpers: ZED camera-relative body velocity → ROS map (world) frame.

Primary mode: ``map_finite_difference`` (spin-tuned — live PASS 22/08 khi robot quay,
người đứng yên: med/p90 ≈ 0 với force_zero_while_spinning + ω×r strip).

Dùng chung Track D exp1–3 và POS exp4–6.
"""
from __future__ import print_function

import math
import threading
from collections import deque

import rospy
from nav_msgs.msg import Odometry


def rotate_vec(x, y, z, qx, qy, qz, qw):
    tx = 2.0 * (qy * z - qz * y)
    ty = 2.0 * (qz * x - qx * z)
    tz = 2.0 * (qx * y - qy * x)
    rx = x + qw * tx + (qy * tz - qz * ty)
    ry = y + qw * ty + (qz * tx - qx * tz)
    rz = z + qw * tz + (qx * ty - qy * tx)
    return rx, ry, rz


def is_finite_vec(v):
    return all(math.isfinite(float(c)) for c in v)


class _OdomBuffer(object):
    """Ring buffer for time-aligned odometry lookup at body stamp."""

    def __init__(self, maxlen=200):
        self._buf = deque(maxlen=maxlen)
        self._lock = threading.Lock()

    def push(self, msg):
        if msg.header.stamp and msg.header.stamp.to_sec() > 0.0:
            with self._lock:
                self._buf.append(msg)

    def lookup(self, stamp):
        with self._lock:
            if not self._buf:
                return None, None
            if stamp is None or stamp.to_sec() <= 0.0:
                return self._buf[-1], 0.0
            best = None
            best_dt = float("inf")
            for msg in self._buf:
                dt = abs((msg.header.stamp - stamp).to_sec())
                if dt < best_dt:
                    best_dt = dt
                    best = msg
            return best, best_dt


class VelocityCompensator(object):
    """Estimate absolute person velocity in the navigation (map) frame.

    Modes
    -----
    map_finite_difference (default / recommended for costmap):
        v_p^map ≈ d/dt (p_p^map) with low-pass + jump gate.
        Independent of ZED camera-relative velocity / ego-motion coupling.

    robot_odometry (legacy):
        v_p^map = v_c^map + R_map^cam * v_{p/c}^cam + sign * (omega × r)

    none / zed_world:
        Debug-only rotate / passthrough paths.
    """

    def __init__(
        self,
        tf_buffer,
        navigation_frame="map",
        camera_frame="camera_link",
        robot_base_frame="base_footprint",
        odom_topic="/odom",
        odom_timeout=0.20,
        enable_translation=True,
        enable_rotation=True,
        rotational_compensation_sign=-1.0,
        low_pass_alpha=0.3,
        velocity_max_valid=4.0,
        stationary_threshold=0.10,
        jump_threshold=1.5,
        heading_min_speed=0.15,
        fd_min_dt=0.03,
        fd_max_dt=0.50,
        enable_fd_yaw_compensation=True,
        fd_yaw_omega_gate=0.05,
        fd_yaw_low_pass_alpha=0.12,
        spin_stationary_threshold=0.35,
        force_zero_while_spinning=True,
    ):
        self.tf_buffer = tf_buffer
        self.navigation_frame = navigation_frame
        self.camera_frame = camera_frame
        self.robot_base_frame = robot_base_frame
        self.odom_timeout = float(odom_timeout)
        self.enable_translation = bool(enable_translation)
        self.enable_rotation = bool(enable_rotation)
        self.rotational_compensation_sign = float(rotational_compensation_sign)
        self.low_pass_alpha = float(low_pass_alpha)
        self.velocity_max_valid = float(velocity_max_valid)
        self.stationary_threshold = float(stationary_threshold)
        self.jump_threshold = float(jump_threshold)
        self.heading_min_speed = float(heading_min_speed)
        self.fd_min_dt = float(fd_min_dt)
        self.fd_max_dt = float(fd_max_dt)
        # When TF falls back to Time(0) while bodies are slightly stale, map-FD
        # sees a fake tangential velocity ≈ ω × r during robot yaw. Subtract it.
        self.enable_fd_yaw_compensation = bool(enable_fd_yaw_compensation)
        self.fd_yaw_omega_gate = float(fd_yaw_omega_gate)
        self.fd_yaw_low_pass_alpha = float(fd_yaw_low_pass_alpha)
        # While yawing, raise the zero-velocity gate (live tune: residual ≤~0.3).
        self.spin_stationary_threshold = float(spin_stationary_threshold)
        # Live spin test: map-FD residuals persist despite ω×r strip; for costmap
        # proxemic, prefer zero person velocity while robot is yawing.
        self.force_zero_while_spinning = bool(force_zero_while_spinning)
        self._last_wz = 0.0

        self._odom_buffer = _OdomBuffer()
        self._filtered = {}  # id -> (vx, vy, vz)
        self._pos_hist = {}  # id -> (stamp_sec, x, y, z)

        self._odom_sub = rospy.Subscriber(
            odom_topic, Odometry, self._odom_cb, queue_size=50, tcp_nodelay=True
        )
        rospy.loginfo(
            "VelocityCompensator: odom=%s nav=%s cam=%s base=%s "
            "trans=%s rot=%s rot_sign=%.1f timeout=%.2fs fd_dt=[%.2f,%.2f] "
            "fd_yaw=%s omega_gate=%.2f spin_zero=%s spin_gate=%.2f",
            odom_topic,
            navigation_frame,
            camera_frame,
            robot_base_frame,
            enable_translation,
            enable_rotation,
            self.rotational_compensation_sign,
            odom_timeout,
            self.fd_min_dt,
            self.fd_max_dt,
            self.enable_fd_yaw_compensation,
            self.fd_yaw_omega_gate,
            self.force_zero_while_spinning,
            self.spin_stationary_threshold,
        )

    def _odom_cb(self, msg):
        self._odom_buffer.push(msg)

    def _find_odom_near(self, stamp):
        return self._odom_buffer.lookup(stamp)

    def _lookup(self, target, source, stamp, allow_latest=True):
        """Return (transform, used_latest)."""
        try:
            return (
                self.tf_buffer.lookup_transform(
                    target, source, stamp, rospy.Duration(0.05)
                ),
                False,
            )
        except Exception:
            if not allow_latest:
                return None, False
            try:
                return (
                    self.tf_buffer.lookup_transform(
                        target, source, rospy.Time(0), rospy.Duration(0.05)
                    ),
                    True,
                )
            except Exception as ex:
                rospy.logwarn_throttle(
                    2.0, "VelocityCompensator: TF %s<-%s failed: %s", target, source, ex
                )
                return None, False

    def _robot_yaw_rate(self, stamp):
        """Return (wz, odom_msg, dt). Always prefer a usable wz for spin gating.

        Do NOT zero wz when body↔odom dt exceeds odom_timeout — that disabled
        yaw stripping / spin stationary gate exactly when TF skew is worst.
        """
        odom_msg, dt = self._find_odom_near(stamp)
        if odom_msg is None:
            odom_msg, dt = self._odom_buffer.lookup(None)
        if odom_msg is None:
            return 0.0, None, float("nan")
        wz = float(odom_msg.twist.twist.angular.z)
        return wz, odom_msg, float(dt) if dt is not None else 0.0

    def _empty_out(self, stamp, p_cam, v_cam):
        px, py, pz = [float(c) for c in p_cam]
        vx, vy, vz = [float(c) for c in v_cam]
        return {
            "raw_velocity_camera": (vx, vy, vz),
            "rotated_velocity_map": (0.0, 0.0, 0.0),
            "robot_linear_velocity_map": (0.0, 0.0, 0.0),
            "robot_angular_z": 0.0,
            "rotational_compensation_map": (0.0, 0.0, 0.0),
            "compensated_velocity_map": (0.0, 0.0, 0.0),
            "filtered_velocity_map": (0.0, 0.0, 0.0),
            "published_velocity_map": (0.0, 0.0, 0.0),
            "final_velocity_map": (0.0, 0.0, 0.0),
            "raw_speed": 0.0,
            "compensated_speed": 0.0,
            "filtered_speed": 0.0,
            "published_speed": 0.0,
            "speed": 0.0,
            "compensation_valid": False,
            "velocity_valid": False,
            "jump_rejected": False,
            "low_pass_applied": False,
            "stationary_threshold_applied": False,
            "body_stamp": stamp.to_sec() if stamp else 0.0,
            "odom_stamp": 0.0,
            "body_odom_time_offset": float("nan"),
            "motion_heading": None,
            "motion_heading_valid": False,
            "motion_heading_source": "none",
            "position_map": None,
            "position_camera": (px, py, pz),
            "velocity_source": "none",
        }

    def _apply_filters(self, person_id, out, fx, fy, fz):
        """Low-pass / jump / stationary gates → published velocity."""
        out["compensated_velocity_map"] = (fx, fy, fz)
        out["compensated_speed"] = math.sqrt(fx * fx + fy * fy + fz * fz)

        prev = self._filtered.get(person_id)
        a = max(0.0, min(1.0, self.low_pass_alpha))
        jump_rejected = False
        low_pass_applied = prev is not None and a < 1.0 - 1e-9

        if prev is not None:
            jump = math.sqrt((fx - prev[0]) ** 2 + (fy - prev[1]) ** 2)
            if jump > self.jump_threshold:
                jump_rejected = True
                out["jump_rejected"] = True
                prev_speed = math.sqrt(prev[0] ** 2 + prev[1] ** 2 + prev[2] ** 2)
                # Không khóa spike: nếu prev đã lớn bất thường thì reset track.
                if prev_speed > self.jump_threshold:
                    rospy.logwarn_throttle(
                        2.0,
                        "VelocityCompensator: jump %.2f m/s id=%s; drop spiked track (prev=%.2f)",
                        jump,
                        person_id,
                        prev_speed,
                    )
                    fx, fy, fz = 0.0, 0.0, 0.0
                    self._filtered.pop(person_id, None)
                    self._pos_hist.pop(person_id, None)
                else:
                    rospy.logwarn_throttle(
                        2.0,
                        "VelocityCompensator: jump %.2f m/s id=%s; hold previous",
                        jump,
                        person_id,
                    )
                    fx, fy, fz = prev
            elif low_pass_applied:
                fx = a * fx + (1.0 - a) * prev[0]
                fy = a * fy + (1.0 - a) * prev[1]
                fz = a * fz + (1.0 - a) * prev[2]
                out["low_pass_applied"] = True

        out["filtered_velocity_map"] = (fx, fy, fz)
        out["filtered_speed"] = math.sqrt(fx * fx + fy * fy + fz * fz)

        if out["filtered_speed"] > self.velocity_max_valid or not is_finite_vec((fx, fy, fz)):
            rospy.logwarn_throttle(
                2.0,
                "VelocityCompensator: invalid speed %.2f id=%s",
                out["filtered_speed"],
                person_id,
            )
            out["compensation_valid"] = False
            out["velocity_valid"] = False
            return out

        published = (fx, fy, fz)
        # Proxemic / costmap are planar — gate on xy only (ignore noisy vz).
        published_speed_xy = math.sqrt(fx * fx + fy * fy)
        published_speed = published_speed_xy
        gate = self.stationary_threshold
        if abs(self._last_wz) >= self.fd_yaw_omega_gate:
            gate = max(gate, self.spin_stationary_threshold)
        if published_speed_xy < gate:
            published = (0.0, 0.0, 0.0)
            published_speed = 0.0
            out["stationary_threshold_applied"] = True

        self._filtered[person_id] = (fx, fy, fz)
        out["published_velocity_map"] = published
        out["published_speed"] = published_speed
        out["final_velocity_map"] = published
        out["speed"] = published_speed
        out["compensation_valid"] = True
        out["velocity_valid"] = not jump_rejected
        self._update_heading(out)
        return out

    def compensate(
        self,
        person_id,
        stamp,
        p_cam,
        v_cam,
        mode="map_finite_difference",
        measurement_reference="camera",
        p_map=None,
    ):
        px, py, pz = [float(c) for c in p_cam]
        vx, vy, vz = [float(c) for c in v_cam]
        out = self._empty_out(stamp, (px, py, pz), (vx, vy, vz))

        if not is_finite_vec((px, py, pz)):
            rospy.logwarn_throttle(
                2.0, "VelocityCompensator: non-finite position id=%s", person_id
            )
            return out

        if not is_finite_vec((vx, vy, vz)):
            rospy.logwarn_throttle(
                2.0,
                "VelocityCompensator: non-finite SDK velocity id=%s; using stationary",
                person_id,
            )
            vx, vy, vz = 0.0, 0.0, 0.0
            out["raw_velocity_camera"] = (vx, vy, vz)

        # Prefer caller-provided map position (same TF as /social_state) to avoid
        # a second TF lookup that may diverge to Time(0) during yaw.
        tf_cam, tf_used_latest = self._lookup(
            self.navigation_frame, self.camera_frame, stamp
        )
        r_map = (0.0, 0.0, 0.0)
        t = None
        rvx = rvy = rvz = 0.0
        tf_skew = 0.0
        if tf_cam is not None:
            q = tf_cam.transform.rotation
            t = tf_cam.transform.translation
            rvx, rvy, rvz = rotate_vec(vx, vy, vz, q.x, q.y, q.z, q.w)
            r_map = rotate_vec(px, py, pz, q.x, q.y, q.z, q.w)
            if (
                stamp is not None
                and stamp.to_sec() > 0.0
                and tf_cam.header.stamp
                and tf_cam.header.stamp.to_sec() > 0.0
            ):
                tf_skew = abs((tf_cam.header.stamp - stamp).to_sec())
            if p_map is None:
                p_map = (t.x + r_map[0], t.y + r_map[1], t.z + r_map[2])
            else:
                # Keep r relative to camera for yaw compensation.
                r_map = (
                    float(p_map[0]) - t.x,
                    float(p_map[1]) - t.y,
                    float(p_map[2]) - t.z,
                )
        elif p_map is not None and is_finite_vec(p_map):
            p_map = (float(p_map[0]), float(p_map[1]), float(p_map[2]))
            tf_used_latest = True  # unknown TF quality for caller-only path
        else:
            return out

        out["rotated_velocity_map"] = (rvx, rvy, rvz)
        out["raw_speed"] = math.sqrt(vx * vx + vy * vy + vz * vz)
        out["position_map"] = tuple(p_map)

        if mode == "none":
            final = (rvx, rvy, rvz)
            out["compensated_velocity_map"] = final
            out["filtered_velocity_map"] = final
            out["published_velocity_map"] = final
            out["final_velocity_map"] = final
            out["compensated_speed"] = math.sqrt(final[0] ** 2 + final[1] ** 2 + final[2] ** 2)
            out["filtered_speed"] = out["compensated_speed"]
            out["published_speed"] = out["compensated_speed"]
            out["speed"] = out["published_speed"]
            out["compensation_valid"] = True
            out["velocity_valid"] = True
            out["body_odom_time_offset"] = 0.0
            out["velocity_source"] = "rotated_only"
            self._update_heading(out)
            return out

        if mode == "zed_world":
            final = (rvx, rvy, rvz)
            out["compensated_velocity_map"] = final
            out["filtered_velocity_map"] = final
            out["published_velocity_map"] = final
            out["final_velocity_map"] = final
            out["compensated_speed"] = math.sqrt(final[0] ** 2 + final[1] ** 2 + final[2] ** 2)
            out["filtered_speed"] = out["compensated_speed"]
            out["published_speed"] = out["compensated_speed"]
            out["speed"] = out["published_speed"]
            out["compensation_valid"] = False
            out["velocity_valid"] = False
            out["velocity_source"] = "zed_world_unverified"
            rospy.logwarn_throttle(
                5.0,
                "VelocityCompensator: zed_world mode has no verified ZED-world→map "
                "extrinsic; compensation_valid=False",
            )
            self._update_heading(out)
            return out

        if mode == "map_finite_difference":
            return self._compensate_map_fd(
                person_id,
                stamp,
                p_map,
                r_map,
                out,
                tf_used_latest=tf_used_latest,
                tf_skew=tf_skew,
            )

        if mode != "robot_odometry":
            rospy.logwarn_throttle(5.0, "Unknown velocity_compensation_mode=%s", mode)
            return out

        if measurement_reference != "camera":
            rospy.logwarn_throttle(
                5.0,
                "robot_odometry expects body_measurement_reference=camera (got %s)",
                measurement_reference,
            )
            return out

        if t is None:
            return out
        return self._compensate_robot_odometry(person_id, stamp, r_map, t, rvx, rvy, rvz, out)

    def _compensate_map_fd(
        self,
        person_id,
        stamp,
        p_map,
        r_map,
        out,
        tf_used_latest=False,
        tf_skew=0.0,
    ):
        """World velocity from d(p_map)/dt, with yaw ego-motion residual removal."""
        out["velocity_source"] = "map_finite_difference"
        stamp_sec = stamp.to_sec() if stamp and stamp.to_sec() > 0.0 else rospy.Time.now().to_sec()
        prev = self._pos_hist.get(person_id)
        self._pos_hist[person_id] = (stamp_sec, p_map[0], p_map[1], p_map[2])

        wz, odom_msg, odom_dt = self._robot_yaw_rate(stamp)
        self._last_wz = wz
        out["robot_angular_z"] = wz
        if odom_msg is not None:
            out["odom_stamp"] = odom_msg.header.stamp.to_sec() if odom_msg.header.stamp else 0.0
            out["body_odom_time_offset"] = odom_dt
        else:
            out["body_odom_time_offset"] = 0.0

        spinning = abs(wz) >= self.fd_yaw_omega_gate
        # Stronger smoothing while yawing — FD is noisier under TF/body skew.
        saved_alpha = self.low_pass_alpha
        if spinning:
            self.low_pass_alpha = min(self.low_pass_alpha, self.fd_yaw_low_pass_alpha)

        try:
            if prev is None:
                out["compensation_valid"] = True
                out["velocity_valid"] = True
                out["compensated_velocity_map"] = (0.0, 0.0, 0.0)
                out["filtered_velocity_map"] = (0.0, 0.0, 0.0)
                out["published_velocity_map"] = (0.0, 0.0, 0.0)
                out["final_velocity_map"] = (0.0, 0.0, 0.0)
                out["speed"] = 0.0
                self._update_heading(out)
                return out

            dt = stamp_sec - prev[0]
            if dt < self.fd_min_dt:
                prev_f = self._filtered.get(person_id, (0.0, 0.0, 0.0))
                out["compensation_valid"] = True
                out["velocity_valid"] = True
                return self._apply_filters(person_id, out, prev_f[0], prev_f[1], prev_f[2])

            if dt > self.fd_max_dt:
                rospy.logwarn_throttle(
                    2.0,
                    "VelocityCompensator: FD dt=%.3fs > %.3fs id=%s; reset track",
                    dt,
                    self.fd_max_dt,
                    person_id,
                )
                self._filtered.pop(person_id, None)
                out["compensation_valid"] = True
                out["velocity_valid"] = True
                out["compensated_velocity_map"] = (0.0, 0.0, 0.0)
                out["filtered_velocity_map"] = (0.0, 0.0, 0.0)
                out["published_velocity_map"] = (0.0, 0.0, 0.0)
                out["final_velocity_map"] = (0.0, 0.0, 0.0)
                out["speed"] = 0.0
                self._update_heading(out)
                return out

            fx = (p_map[0] - prev[1]) / dt
            fy = (p_map[1] - prev[2]) / dt
            fz = (p_map[2] - prev[3]) / dt

            # During yaw, strip tangential ω×r; if motion was mostly tangential,
            # zero the whole residual (radial noise from TF skew).
            if self.enable_fd_yaw_compensation and spinning:
                speed_before = math.sqrt(fx * fx + fy * fy)
                sx = -wz * float(r_map[1])
                sy = wz * float(r_map[0])
                ss = math.sqrt(sx * sx + sy * sy)
                if ss > 1e-6 and speed_before > 1e-6:
                    ux, uy = sx / ss, sy / ss
                    proj = fx * ux + fy * uy
                    out["rotational_compensation_map"] = (proj * ux, proj * uy, 0.0)
                    fx -= proj * ux
                    fy -= proj * uy
                    out["velocity_source"] = "map_finite_difference+yaw"
                    if abs(proj) / speed_before >= 0.45:
                        fx, fy, fz = 0.0, 0.0, 0.0

            # Costmap-safe: while robot yaws, do not publish person velocity.
            # Live tune (spin + stationary person): residual spikes ~0.7 m/s remain
            # after ω×r strip due to TF/body skew — worse than missing walk-while-spin.
            if spinning and self.force_zero_while_spinning:
                fx, fy, fz = 0.0, 0.0, 0.0
                out["velocity_source"] = "map_finite_difference+spin_zero"
                self._filtered[person_id] = (0.0, 0.0, 0.0)

            return self._apply_filters(person_id, out, fx, fy, fz)
        finally:
            self.low_pass_alpha = saved_alpha

    def _compensate_robot_odometry(self, person_id, stamp, r_map, t_cam, rvx, rvy, rvz, out):
        out["velocity_source"] = "robot_odometry"
        odom_msg, dt = self._find_odom_near(stamp)
        if odom_msg is None:
            rospy.logwarn_throttle(2.0, "VelocityCompensator: no odometry yet")
            return out

        out["odom_stamp"] = odom_msg.header.stamp.to_sec() if odom_msg.header.stamp else 0.0
        out["body_odom_time_offset"] = float(dt) if dt is not None else float("nan")
        if dt is not None and dt > self.odom_timeout:
            rospy.logwarn_throttle(
                2.0,
                "VelocityCompensator: odom too old (dt=%.3fs > %.3fs); skip compensation",
                dt,
                self.odom_timeout,
            )
            return out

        child = odom_msg.child_frame_id if odom_msg.child_frame_id else self.robot_base_frame
        tf_mc, _ = self._lookup(self.navigation_frame, child, stamp)
        if tf_mc is None:
            return out

        qo = tf_mc.transform.rotation
        twist = odom_msg.twist.twist
        vbx, vby, vbz = rotate_vec(
            twist.linear.x, twist.linear.y, twist.linear.z, qo.x, qo.y, qo.z, qo.w
        )
        wz = float(twist.angular.z)
        out["robot_angular_z"] = wz

        v_cam_map = (vbx, vby, vbz)
        if self.enable_translation:
            tf_base, _ = self._lookup(self.navigation_frame, self.robot_base_frame, stamp)
            if tf_base is not None:
                rcx = t_cam.x - tf_base.transform.translation.x
                rcy = t_cam.y - tf_base.transform.translation.y
                v_cam_map = (vbx - wz * rcy, vby + wz * rcx, vbz)

        out["robot_linear_velocity_map"] = v_cam_map

        rot_comp = (0.0, 0.0, 0.0)
        if self.enable_rotation:
            rot_comp = (-wz * r_map[1], wz * r_map[0], 0.0)
        out["rotational_compensation_map"] = rot_comp

        rot_sign = self.rotational_compensation_sign if self.enable_rotation else 0.0
        vcx = v_cam_map[0] if self.enable_translation else 0.0
        vcy = v_cam_map[1] if self.enable_translation else 0.0
        vcz = v_cam_map[2] if self.enable_translation else 0.0
        fx = vcx + rvx + rot_sign * rot_comp[0]
        fy = vcy + rvy + rot_sign * rot_comp[1]
        fz = vcz + rvz + rot_sign * rot_comp[2]
        return self._apply_filters(person_id, out, fx, fy, fz)

    def _update_heading(self, out):
        fx, fy, _ = out["final_velocity_map"]
        speed_xy = math.sqrt(fx * fx + fy * fy)
        if speed_xy >= self.heading_min_speed and out["compensation_valid"]:
            out["motion_heading"] = math.atan2(fy, fx)
            out["motion_heading_valid"] = True
            out["motion_heading_source"] = "compensated_velocity"
        else:
            out["motion_heading"] = None
            out["motion_heading_valid"] = False
            out["motion_heading_source"] = "none"
