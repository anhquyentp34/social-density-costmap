#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phân tích rosbag bù vận tốc ZED → CSV + đồ thị.

Usage:
  rosrun social_density_fusion analyze_zed_velocity_bag.py _bag:=/path/to.bag
  # hoặc:
  python3 analyze_zed_velocity_bag.py /path/to.bag --out /tmp/out_dir
"""

from __future__ import print_function

import argparse
import csv
import math
import os
import sys


def main():
    parser = argparse.ArgumentParser(description="Analyze ZED velocity compensation bag")
    parser.add_argument("bag", nargs="?", default=None, help="Path to rosbag")
    parser.add_argument("--out", default=None, help="Output directory")
    parser.add_argument("--no-plot", action="store_true")
    args, _unknown = parser.parse_known_args()

    bag_path = args.bag
    if bag_path is None:
        # rospy private param fallback
        try:
            import rospy

            rospy.init_node("analyze_zed_velocity_bag", anonymous=True)
            bag_path = rospy.get_param("~bag", "")
            if not args.out:
                args.out = rospy.get_param("~out", "")
        except Exception:
            pass

    if not bag_path or not os.path.isfile(bag_path):
        print("ERROR: bag not found. Pass path or _bag:=...", file=sys.stderr)
        sys.exit(1)

    out_dir = args.out or (bag_path + "_analysis")
    os.makedirs(out_dir, exist_ok=True)

    import rosbag
    from social_density_fusion.msg import VelocityCompensationDebugArray
    from nav_msgs.msg import Odometry

    rows = []
    odom_series = []
    spikes = []
    invalid_count = 0
    total = 0

    topics = [
        "/zed_velocity_compensation/debug",
        "/odom",
        "/odometry/filtered",
    ]

    with rosbag.Bag(bag_path) as bag:
        for topic, msg, t in bag.read_messages(topics=topics):
            if topic in ("/odom", "/odometry/filtered"):
                odom_series.append(
                    {
                        "time": t.to_sec(),
                        "vx": msg.twist.twist.linear.x,
                        "vy": msg.twist.twist.linear.y,
                        "wz": msg.twist.twist.angular.z,
                    }
                )
                continue

            if not isinstance(msg, VelocityCompensationDebugArray) and not hasattr(msg, "samples"):
                continue
            for s in msg.samples:
                total += 1
                raw = list(s.raw_velocity_camera)
                rot = list(s.rotated_velocity_map)
                robot = list(s.robot_linear_velocity_map)
                rcomp = list(s.rotational_compensation_map)
                final = list(s.final_velocity_map)
                row = {
                    "time": t.to_sec(),
                    "person_id": int(s.person_id),
                    "raw_vx": raw[0],
                    "raw_vy": raw[1],
                    "rotated_vx": rot[0],
                    "rotated_vy": rot[1],
                    "robot_vx": robot[0],
                    "robot_vy": robot[1],
                    "robot_wz": float(s.robot_angular_z),
                    "rotation_comp_vx": rcomp[0],
                    "rotation_comp_vy": rcomp[1],
                    "final_vx": final[0],
                    "final_vy": final[1],
                    "final_speed": float(s.speed),
                    "confidence": float(s.confidence),
                    "tracking_state": int(s.tracking_state),
                    "compensation_valid": int(bool(s.compensation_valid)),
                    "body_odom_time_offset": float(s.body_odom_time_offset),
                    "motion_heading": float(s.motion_heading) if s.motion_heading_valid else float("nan"),
                    "facing_orientation": float(s.facing_orientation)
                    if s.facing_orientation_valid
                    else float("nan"),
                }
                rows.append(row)
                if not s.compensation_valid:
                    invalid_count += 1
                if s.speed > 2.5:
                    spikes.append(row)

    csv_path = os.path.join(out_dir, "velocity_compensation.csv")
    fieldnames = list(rows[0].keys()) if rows else [
        "time",
        "person_id",
        "raw_vx",
        "raw_vy",
        "rotated_vx",
        "rotated_vy",
        "robot_vx",
        "robot_vy",
        "robot_wz",
        "rotation_comp_vx",
        "rotation_comp_vy",
        "final_vx",
        "final_vy",
        "final_speed",
        "confidence",
        "tracking_state",
    ]
    with open(csv_path, "w") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames)
        w.writeheader()
        for r in rows:
            w.writerow(r)

    summary_path = os.path.join(out_dir, "summary.txt")
    speeds = [r["final_speed"] for r in rows]
    mean_speed = sum(speeds) / len(speeds) if speeds else float("nan")
    max_speed = max(speeds) if speeds else float("nan")
    offsets = [
        r["body_odom_time_offset"]
        for r in rows
        if math.isfinite(r["body_odom_time_offset"])
    ]
    mean_off = sum(offsets) / len(offsets) if offsets else float("nan")

    with open(summary_path, "w") as f:
        f.write("bag: %s\n" % bag_path)
        f.write("samples: %d\n" % total)
        f.write("invalid_compensation: %d\n" % invalid_count)
        f.write("mean_final_speed: %.4f\n" % mean_speed)
        f.write("max_final_speed: %.4f\n" % max_speed)
        f.write("spikes_speed_gt_2.5: %d\n" % len(spikes))
        f.write("mean_body_odom_offset_s: %.4f\n" % mean_off)
        f.write("odom_samples: %d\n" % len(odom_series))
        f.write(
            "NOTE: Do not claim compensation works without V2 (robot translate) "
            "and V3 (robot rotate) experiments.\n"
        )

    print("Wrote", csv_path)
    print("Wrote", summary_path)
    print(
        "samples=%d invalid=%d mean_speed=%.3f max_speed=%.3f spikes=%d"
        % (total, invalid_count, mean_speed, max_speed, len(spikes))
    )

    if args.no_plot or not rows:
        return

    try:
        import matplotlib

        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except Exception as ex:
        print("Plot skipped:", ex)
        return

    t0 = rows[0]["time"]
    ts = [r["time"] - t0 for r in rows]

    def save(fig, name):
        path = os.path.join(out_dir, name)
        fig.tight_layout()
        fig.savefig(path, dpi=120)
        plt.close(fig)
        print("Wrote", path)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["raw_vx"] for r in rows], label="raw_vx (cam)", alpha=0.7)
    ax.plot(ts, [r["final_vx"] for r in rows], label="final_vx (map)", alpha=0.9)
    ax.plot(ts, [r["raw_vy"] for r in rows], label="raw_vy (cam)", alpha=0.5)
    ax.plot(ts, [r["final_vy"] for r in rows], label="final_vy (map)", alpha=0.7)
    ax.set_xlabel("t [s]")
    ax.set_ylabel("v [m/s]")
    ax.legend()
    ax.set_title("Raw vs final velocity")
    save(fig, "01_raw_vs_final.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["robot_vx"] for r in rows], label="robot_vx_map")
    ax.plot(ts, [r["final_vx"] for r in rows], label="person_final_vx")
    ax.plot(ts, [r["robot_wz"] for r in rows], label="robot_wz", alpha=0.6)
    ax.legend()
    ax.set_title("Robot vs person velocity")
    save(fig, "02_robot_vs_person.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["final_speed"] for r in rows], label="final_speed")
    ax.axhline(0.1, color="gray", ls="--", label="stationary_thr~0.1")
    ax.legend()
    ax.set_title("Final speed over time")
    save(fig, "03_speed.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["motion_heading"] for r in rows], label="motion_heading")
    ax.plot(ts, [r["facing_orientation"] for r in rows], label="facing_orientation")
    ax.legend()
    ax.set_title("Motion heading vs facing")
    save(fig, "04_heading_vs_facing.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["body_odom_time_offset"] for r in rows], label="body-odom dt")
    ax.legend()
    ax.set_title("Timestamp offset")
    save(fig, "05_timestamp_offset.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["confidence"] for r in rows], label="confidence")
    ax.plot(ts, [r["tracking_state"] for r in rows], label="tracking_state")
    ax.legend()
    ax.set_title("Confidence / tracking state")
    save(fig, "06_confidence_tracking.png")


if __name__ == "__main__":
    main()
