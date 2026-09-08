#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Kiểm tự động bù ego-motion cho vận tốc người (ZED).

Giả định người đứng yên trong khung hình.
Phases:
  V1 — robot đứng yên → final_speed ≈ 0
  V2 — robot tịnh tiến nhẹ → final_speed ≈ 0
  V3 — robot quay tại chỗ nhẹ → final_speed ≈ 0

Usage (ROS đã chạy, fusion + /odom + /bodies):
  rosrun social_density_fusion auto_verify_velocity_compensation.py
  python3 auto_verify_velocity_compensation.py --out /tmp/vel_verify
"""

from __future__ import print_function

import argparse
import json
import math
import os
import statistics
import sys
import time

import rospy
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from social_density_fusion.msg import VelocityCompensationDebugArray


def _hypot(x, y):
    return math.sqrt(x * x + y * y)


def _stats(xs):
    if not xs:
        return {
            "n": 0,
            "mean": float("nan"),
            "median": float("nan"),
            "p90": float("nan"),
            "max": float("nan"),
            "pct_lt_0_10": float("nan"),
            "pct_lt_0_15": float("nan"),
        }
    xs = sorted(float(v) for v in xs)
    n = len(xs)
    p90 = xs[min(n - 1, int(0.9 * (n - 1)))]
    return {
        "n": n,
        "mean": statistics.mean(xs),
        "median": statistics.median(xs),
        "p90": p90,
        "max": xs[-1],
        "pct_lt_0_10": 100.0 * sum(v < 0.10 for v in xs) / n,
        "pct_lt_0_15": 100.0 * sum(v < 0.15 for v in xs) / n,
    }


class Collector(object):
    def __init__(self):
        self.rows = []
        self._odom = None
        rospy.Subscriber("/odom", Odometry, self._odom_cb, queue_size=20)
        rospy.Subscriber(
            "/zed_velocity_compensation/debug",
            VelocityCompensationDebugArray,
            self._dbg_cb,
            queue_size=20,
        )

    def _odom_cb(self, msg):
        self._odom = msg

    def _dbg_cb(self, msg):
        if self._odom is None:
            return
        tw = self._odom.twist.twist
        rs = _hypot(tw.linear.x, tw.linear.y)
        wz = float(tw.angular.z)
        t = rospy.Time.now().to_sec()
        for s in msg.samples:
            self.rows.append(
                {
                    "t": t,
                    "id": int(s.person_id),
                    "final": float(s.speed),
                    "raw": _hypot(s.raw_velocity_camera[0], s.raw_velocity_camera[1]),
                    "robot": rs,
                    "wz": wz,
                    "valid": bool(s.compensation_valid),
                    "offset": float(s.body_odom_time_offset),
                }
            )

    def clear(self):
        self.rows = []

    def wait_person(self, timeout):
        t0 = time.time()
        while not rospy.is_shutdown() and (time.time() - t0) < timeout:
            if any(True for _ in self.rows):
                return True
            rospy.sleep(0.2)
        return False


def publish_cmd(pub, vx=0.0, vy=0.0, wz=0.0, duration=3.0, rate_hz=15.0):
    msg = Twist()
    msg.linear.x = float(vx)
    msg.linear.y = float(vy)
    msg.angular.z = float(wz)
    rate = rospy.Rate(rate_hz)
    t_end = rospy.Time.now() + rospy.Duration(duration)
    while not rospy.is_shutdown() and rospy.Time.now() < t_end:
        pub.publish(msg)
        rate.sleep()
    zero = Twist()
    for _ in range(8):
        pub.publish(zero)
        rate.sleep()


def evaluate_phase(name, rows, robot_moving_expected, thresh_mean=0.15, thresh_p90=0.25):
    finals = [r["final"] for r in rows]
    st = _stats(finals)
    robot_mean = statistics.mean([r["robot"] for r in rows]) if rows else float("nan")
    wz_mean = statistics.mean([abs(r["wz"]) for r in rows]) if rows else float("nan")
    raw_mean = statistics.mean([r["raw"] for r in rows]) if rows else float("nan")

    if not rows:
        return {
            "phase": name,
            "pass": False,
            "reason": "no_person_samples",
            "stats": st,
            "robot_speed_mean": robot_mean,
            "wz_abs_mean": wz_mean,
            "raw_speed_mean": raw_mean,
        }

    ok = st["mean"] <= thresh_mean and st["p90"] <= thresh_p90
    reason = "ok" if ok else "person_speed_too_high_while_assumed_stationary"
    if robot_moving_expected and robot_mean < 0.03 and wz_mean < 0.08:
        # Motion command may not have reached wheels — soft fail note
        reason = "robot_motion_weak_" + reason

    return {
        "phase": name,
        "pass": bool(ok),
        "reason": reason,
        "stats": st,
        "robot_speed_mean": robot_mean,
        "wz_abs_mean": wz_mean,
        "raw_speed_mean": raw_mean,
        "mode_hint": "expect final≈0 if person đứng yên",
    }


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="/tmp/velocity_compensation_verify")
    ap.add_argument("--wait-person", type=float, default=45.0)
    ap.add_argument("--v1-sec", type=float, default=5.0)
    ap.add_argument("--v2-sec", type=float, default=4.0)
    ap.add_argument("--v3-sec", type=float, default=4.0)
    ap.add_argument("--vx", type=float, default=0.10)
    ap.add_argument("--wz", type=float, default=0.25)
    ap.add_argument("--skip-motion", action="store_true")
    ap.add_argument("--mean-thresh", type=float, default=0.15)
    ap.add_argument("--p90-thresh", type=float, default=0.25)
    args, _ = ap.parse_known_args()

    out_dir = args.out
    if not os.path.isabs(out_dir):
        out_dir = os.path.join(os.path.expanduser("~"), "omni4wd_ws", out_dir)
    os.makedirs(out_dir, exist_ok=True)

    rospy.init_node("auto_verify_velocity_compensation", anonymous=True)
    mode = rospy.get_param(
        "/social_density_fusion_node/velocity_compensation_mode", "unknown"
    )
    rospy.loginfo("velocity_compensation_mode=%s", mode)

    pub = rospy.Publisher("/cmd_vel", Twist, queue_size=1)
    rospy.sleep(0.5)
    col = Collector()

    print("=== Chờ người trong khung hình ZED (tối đa %.0fs) ===" % args.wait_person)
    print("→ Đứng YÊN trước camera.")
    # seed
    rospy.sleep(1.0)
    if not col.wait_person(args.wait_person):
        report = {
            "overall_pass": False,
            "mode": mode,
            "error": "timeout_waiting_for_person",
            "phases": [],
        }
        path = os.path.join(out_dir, "report.json")
        with open(path, "w") as f:
            json.dump(report, f, indent=2)
        print("FAIL: không thấy người trong /zed_velocity_compensation/debug")
        print("report:", path)
        sys.exit(2)

    results = []

    def phase_collect(label, moving_expected, cmd_kwargs, duration):
        print("=== %s ===" % label)
        # Đảm bảo còn người trước khi đo
        col.clear()
        if not col.wait_person(min(20.0, args.wait_person)):
            return evaluate_phase(
                label,
                [],
                robot_moving_expected=moving_expected,
                thresh_mean=args.mean_thresh,
                thresh_p90=args.p90_thresh,
            )
        col.clear()
        if cmd_kwargs:
            publish_cmd(pub, duration=duration, **cmd_kwargs)
        else:
            publish_cmd(pub, 0, 0, 0, duration=0.3)
            rospy.sleep(duration)
        # giữ thêm mẫu ngay sau lệnh
        rospy.sleep(0.5)
        r = evaluate_phase(
            label,
            list(col.rows),
            robot_moving_expected=moving_expected,
            thresh_mean=args.mean_thresh,
            thresh_p90=args.p90_thresh,
        )
        print(
            "%s final mean=%s p90=%s robot=%.3f pass=%s"
            % (
                label,
                ("%.3f" % r["stats"]["mean"]) if r["stats"]["n"] else "nan",
                ("%.3f" % r["stats"]["p90"]) if r["stats"]["n"] else "nan",
                r.get("robot_speed_mean") or float("nan"),
                r["pass"],
            )
        )
        return r

    r1 = phase_collect("V1_robot_still", False, None, args.v1_sec)
    results.append(r1)

    if not args.skip_motion:
        r2 = phase_collect(
            "V2_robot_translate", True, {"vx": args.vx}, args.v2_sec
        )
        results.append(r2)
        r3 = phase_collect(
            "V3_robot_spin", True, {"wz": args.wz}, args.v3_sec
        )
        results.append(r3)

    publish_cmd(pub, 0, 0, 0, duration=0.4)

    overall = all(r["pass"] for r in results) and len(results) > 0
    report = {
        "overall_pass": overall,
        "mode": mode,
        "thresholds": {"mean": args.mean_thresh, "p90": args.p90_thresh},
        "phases": results,
        "assumption": "person completely stationary in camera FOV",
    }
    path = os.path.join(out_dir, "report.json")
    with open(path, "w") as f:
        json.dump(report, f, indent=2)

    txt = os.path.join(out_dir, "summary.txt")
    with open(txt, "w") as f:
        f.write("mode=%s overall_pass=%s\n" % (mode, overall))
        for r in results:
            f.write(
                "%s pass=%s mean=%.3f p90=%.3f max=%.3f pct<0.10=%.1f robot=%.3f wz=%.3f reason=%s\n"
                % (
                    r["phase"],
                    r["pass"],
                    r["stats"]["mean"],
                    r["stats"]["p90"],
                    r["stats"]["max"],
                    r["stats"]["pct_lt_0_10"],
                    r["robot_speed_mean"],
                    r["wz_abs_mean"],
                    r["reason"],
                )
            )

    print("=== KẾT QUẢ ===")
    print("mode=%s overall_pass=%s" % (mode, overall))
    print("report:", path)
    print("summary:", txt)
    sys.exit(0 if overall else 1)


if __name__ == "__main__":
    main()
