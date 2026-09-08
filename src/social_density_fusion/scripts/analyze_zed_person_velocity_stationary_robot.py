#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Phân tích rosbag vận tốc người — robot đứng yên.

Usage:
  python3 analyze_zed_person_velocity_stationary_robot.py bag.bag --person-id 0
  python3 analyze_zed_person_velocity_stationary_robot.py bag.bag --test-type S1
  python3 analyze_zed_person_velocity_stationary_robot.py bag.bag --manual-ref 1.2

Supports F0/F1/F2 offline replay from PersonVelocityDebug stages.
"""

from __future__ import print_function

import argparse
import csv
import json
import math
import os
import sys
from collections import defaultdict

import numpy as np


def wrap_angle(a):
    return (a + math.pi) % (2.0 * math.pi) - math.pi


def circular_mae_deg(errors_rad):
    if not errors_rad:
        return float("nan")
    return math.degrees(sum(abs(wrap_angle(e)) for e in errors_rad) / len(errors_rad))


def percentile(vals, p):
    if not vals:
        return float("nan")
    return float(np.percentile(np.asarray(vals, dtype=float), p))


def stats_speed(speeds):
    if not speeds:
        return {}
    arr = np.asarray(speeds, dtype=float)
    return {
        "n": int(len(arr)),
        "mean": float(np.mean(arr)),
        "median": float(np.median(arr)),
        "std": float(np.std(arr)),
        "rmse_zero": float(np.sqrt(np.mean(arr ** 2))),
        "p90": percentile(speeds, 90),
        "p95": percentile(speeds, 95),
        "max": float(np.max(arr)),
        "pct_lt_0.05": float(100.0 * np.mean(arr < 0.05)),
        "pct_lt_0.10": float(100.0 * np.mean(arr < 0.10)),
    }


def central_diff(times, xs, ys, window=3):
    """Position-derived velocity with central difference (window samples each side)."""
    n = len(times)
    vx = [float("nan")] * n
    vy = [float("nan")] * n
    w = max(1, int(window))
    for i in range(w, n - w):
        dt = times[i + w] - times[i - w]
        if dt <= 1e-6:
            continue
        vx[i] = (xs[i + w] - xs[i - w]) / dt
        vy[i] = (ys[i + w] - ys[i - w]) / dt
    return vx, vy


def detect_id_switches(ids):
    switches = 0
    prev = None
    for pid in ids:
        if prev is not None and pid != prev:
            switches += 1
        prev = pid
    return switches


def detect_timestamp_gaps(times, threshold=0.15):
    gaps = []
    for i in range(1, len(times)):
        dt = times[i] - times[i - 1]
        if dt > threshold:
            gaps.append((times[i - 1], times[i], dt))
    return gaps


def mae_rmse_bias(pred, ref, mask_min=None):
    pairs = []
    for p, r in zip(pred, ref):
        if not (math.isfinite(p) and math.isfinite(r)):
            continue
        if mask_min is not None and abs(r) < mask_min:
            continue
        pairs.append((p, r))
    if not pairs:
        return {"mae": float("nan"), "rmse": float("nan"), "bias": float("nan"), "n": 0}
    err = [p - r for p, r in pairs]
    abs_err = [abs(e) for e in err]
    return {
        "mae": float(np.mean(abs_err)),
        "rmse": float(np.sqrt(np.mean([e ** 2 for e in err]))),
        "bias": float(np.mean(err)),
        "n": len(pairs),
    }


def estimate_delay(times, speed, threshold=0.10):
    """Rough delay: first time speed crosses threshold after t0."""
    if not times:
        return float("nan")
    for t, s in zip(times, speed):
        if math.isfinite(s) and s >= threshold:
            return t - times[0]
    return float("nan")


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("bag", help="Path to rosbag")
    parser.add_argument("--out", default=None)
    parser.add_argument("--person-id", type=int, default=None)
    parser.add_argument("--test-type", default="unknown", help="S0|S1|S2|...|M1|...")
    parser.add_argument("--manual-ref", type=float, default=float("nan"),
                        help="Manual distance/time reference speed [m/s]")
    parser.add_argument("--diff-window", type=int, default=3)
    parser.add_argument("--no-plot", action="store_true")
    args = parser.parse_args()

    if not os.path.isfile(args.bag):
        print("ERROR: bag not found:", args.bag, file=sys.stderr)
        sys.exit(1)

    out_dir = args.out or (args.bag.replace(".bag", "") + "_analysis")
    os.makedirs(out_dir, exist_ok=True)

    import rosbag
    from nav_msgs.msg import Odometry
    from geometry_msgs.msg import Twist

    try:
        from social_density_fusion.msg import PersonVelocityDebugArray, VelocityCompensationDebugArray
        has_pvd = True
        has_vcd = True
    except ImportError:
        has_pvd = False
        has_vcd = False

    rows = []
    odom_rows = []
    cmd_rows = []
    bodies_empty = 0
    bodies_nonempty = 0
    bodies_ids = set()

    topics = ["/odom", "/cmd_vel", "/bodies", "/pedestrian_measurements"]
    if has_pvd:
        topics.append("/zed_person_velocity/debug")
    if has_vcd:
        topics.append("/zed_velocity_compensation/debug")

    with rosbag.Bag(args.bag) as bag:
        info = bag.get_type_and_topic_info()
        available = set(info.topics.keys())
        for t in topics:
            if t not in available:
                print("WARN: topic missing:", t)

        for topic, msg, t in bag.read_messages(topics=topics):
            ts = t.to_sec()
            if topic == "/odom":
                tw = msg.twist.twist
                vr = math.hypot(tw.linear.x, tw.linear.y)
                odom_rows.append({
                    "time": ts,
                    "odom_vx": tw.linear.x,
                    "odom_vy": tw.linear.y,
                    "odom_vr": vr,
                    "odom_wz": tw.angular.z,
                })
            elif topic == "/cmd_vel":
                cmd_rows.append({
                    "time": ts,
                    "cmd_vx": msg.linear.x,
                    "cmd_vy": msg.linear.y,
                    "cmd_wz": msg.angular.z,
                })
            elif topic == "/bodies":
                if not msg.bodies:
                    bodies_empty += 1
                else:
                    bodies_nonempty += 1
                    for b in msg.bodies:
                        bodies_ids.add(int(b.id))
            elif topic == "/pedestrian_measurements":
                for m in msg.measurements:
                    if args.person_id is not None and int(m.id) != args.person_id:
                        continue
                    rows.append({
                        "time": ts,
                        "person_id": int(m.id),
                        "tracking_state": int(m.tracking_state),
                        "confidence": float(m.confidence),
                        "position_x": m.position_x,
                        "position_y": m.position_y,
                        "position_z": m.position_z,
                        "raw_vx": float("nan"),
                        "raw_vy": float("nan"),
                        "raw_speed": float("nan"),
                        "rotated_vx": float("nan"),
                        "rotated_vy": float("nan"),
                        "compensated_vx": m.velocity_x,
                        "compensated_vy": m.velocity_y,
                        "compensated_speed": float(m.speed),
                        "filtered_vx": m.velocity_x,
                        "filtered_vy": m.velocity_y,
                        "filtered_speed": float(m.speed),
                        "published_vx": m.velocity_x,
                        "published_vy": m.velocity_y,
                        "published_speed": float(m.speed),
                        "robot_vx": float("nan"),
                        "robot_vy": float("nan"),
                        "robot_wz": float("nan"),
                        "rotation_comp_vx": float("nan"),
                        "rotation_comp_vy": float("nan"),
                        "velocity_valid": int(bool(m.velocity_compensation_valid)),
                        "jump_rejected": 0,
                        "stationary_threshold_applied": 0,
                        "low_pass_applied": 0,
                        "body_odom_time_offset": float(m.body_odom_time_offset),
                        "motion_heading": float(m.motion_heading) if m.motion_heading_valid else float("nan"),
                        "facing_orientation": float(m.facing_orientation) if m.facing_valid else float("nan"),
                        "manual_reference_speed": args.manual_ref,
                        "_source": "pedestrian_measurements",
                    })
            elif topic == "/zed_person_velocity/debug" and has_pvd:
                for s in msg.samples:
                    if args.person_id is not None and int(s.person_id) != args.person_id:
                        continue
                    raw = list(s.raw_velocity_camera)
                    rot = list(s.rotated_velocity_map)
                    comp = list(s.compensated_velocity_map)
                    filt = list(s.filtered_velocity_map)
                    pub = list(s.published_velocity_map)
                    pos = list(s.position_map)
                    rows.append({
                        "time": ts,
                        "person_id": int(s.person_id),
                        "tracking_state": int(s.tracking_state),
                        "confidence": float(s.confidence),
                        "position_x": pos[0],
                        "position_y": pos[1],
                        "position_z": pos[2] if len(pos) > 2 else 0.0,
                        "raw_vx": raw[0],
                        "raw_vy": raw[1],
                        "raw_speed": float(s.raw_speed),
                        "rotated_vx": rot[0],
                        "rotated_vy": rot[1],
                        "compensated_vx": comp[0],
                        "compensated_vy": comp[1],
                        "compensated_speed": float(s.compensated_speed),
                        "filtered_vx": filt[0],
                        "filtered_vy": filt[1],
                        "filtered_speed": float(s.filtered_speed),
                        "published_vx": pub[0],
                        "published_vy": pub[1],
                        "published_speed": float(s.published_speed),
                        "robot_vx": list(s.robot_velocity_map)[0],
                        "robot_vy": list(s.robot_velocity_map)[1],
                        "robot_wz": float("nan"),
                        "rotation_comp_vx": list(s.rotational_compensation_map)[0],
                        "rotation_comp_vy": list(s.rotational_compensation_map)[1],
                        "velocity_valid": int(bool(s.velocity_valid)),
                        "jump_rejected": int(bool(s.jump_rejected)),
                        "stationary_threshold_applied": int(bool(s.stationary_threshold_applied)),
                        "low_pass_applied": int(bool(s.low_pass_applied)),
                        "body_odom_time_offset": float(s.time_offset),
                        "motion_heading": float(s.motion_heading) if s.motion_heading_valid else float("nan"),
                        "facing_orientation": float(s.facing_orientation) if s.facing_orientation_valid else float("nan"),
                        "manual_reference_speed": args.manual_ref,
                        "_source": "person_velocity_debug",
                    })
            elif topic == "/zed_velocity_compensation/debug" and has_vcd:
                for s in msg.samples:
                    if args.person_id is not None and int(s.person_id) != args.person_id:
                        continue
                    raw = list(s.raw_velocity_camera)
                    rot = list(s.rotated_velocity_map)
                    final = list(s.final_velocity_map)
                    rows.append({
                        "time": ts,
                        "person_id": int(s.person_id),
                        "tracking_state": int(s.tracking_state),
                        "confidence": float(s.confidence),
                        "position_x": float("nan"),
                        "position_y": float("nan"),
                        "position_z": float("nan"),
                        "raw_vx": raw[0],
                        "raw_vy": raw[1],
                        "raw_speed": math.hypot(raw[0], raw[1]),
                        "rotated_vx": rot[0],
                        "rotated_vy": rot[1],
                        "compensated_vx": final[0],
                        "compensated_vy": final[1],
                        "compensated_speed": float(s.speed),
                        "filtered_vx": final[0],
                        "filtered_vy": final[1],
                        "filtered_speed": float(s.speed),
                        "published_vx": final[0],
                        "published_vy": final[1],
                        "published_speed": float(s.speed),
                        "robot_vx": list(s.robot_linear_velocity_map)[0],
                        "robot_vy": list(s.robot_linear_velocity_map)[1],
                        "robot_wz": float(s.robot_angular_z),
                        "rotation_comp_vx": list(s.rotational_compensation_map)[0],
                        "rotation_comp_vy": list(s.rotational_compensation_map)[1],
                        "velocity_valid": int(bool(s.compensation_valid)),
                        "jump_rejected": 0,
                        "stationary_threshold_applied": 0,
                        "low_pass_applied": 0,
                        "body_odom_time_offset": float(s.body_odom_time_offset),
                        "motion_heading": float(s.motion_heading) if s.motion_heading_valid else float("nan"),
                        "facing_orientation": float(s.facing_orientation) if s.facing_orientation_valid else float("nan"),
                        "manual_reference_speed": args.manual_ref,
                        "_source": "velocity_compensation_debug",
                    })

    # Deduplicate: prefer person_velocity_debug > velocity_compensation_debug > pedestrian_measurements
    if rows:
        priority = {"person_velocity_debug": 0, "velocity_compensation_debug": 1, "pedestrian_measurements": 2}
        best = {}
        for r in rows:
            key = (r["time"], r["person_id"])
            src = r.get("_source", "velocity_compensation_debug")
            r["_prio"] = priority.get(src, 9)
            if key not in best or r["_prio"] < best[key]["_prio"]:
                best[key] = r
        rows = sorted(best.values(), key=lambda r: (r["person_id"], r["time"]))
        for r in rows:
            r.pop("_source", None)
            r.pop("_prio", None)

    # Position-derived velocity
    if rows:
        rows.sort(key=lambda r: (r["person_id"], r["time"]))
        by_id = defaultdict(list)
        for r in rows:
            by_id[r["person_id"]].append(r)
        merged = []
        for pid, rs in by_id.items():
            times = [r["time"] for r in rs]
            xs = [r["position_x"] for r in rs]
            ys = [r["position_y"] for r in rs]
            pdx, pdy = central_diff(times, xs, ys, window=args.diff_window)
            for i, r in enumerate(rs):
                r["position_derived_vx"] = pdx[i]
                r["position_derived_vy"] = pdy[i]
                spd = math.hypot(pdx[i], pdy[i]) if math.isfinite(pdx[i]) else float("nan")
                r["position_derived_speed"] = spd
                merged.append(r)
        rows = sorted(merged, key=lambda r: r["time"])

    # CSV
    fieldnames = [
        "time", "person_id", "tracking_state", "confidence",
        "position_x", "position_y",
        "raw_vx", "raw_vy", "raw_speed",
        "compensated_vx", "compensated_vy", "compensated_speed",
        "filtered_vx", "filtered_vy", "filtered_speed",
        "published_vx", "published_vy", "published_speed",
        "position_derived_vx", "position_derived_vy", "position_derived_speed",
        "manual_reference_speed",
        "velocity_valid", "jump_rejected", "stationary_threshold_applied",
        "body_odom_time_offset",
    ]
    csv_path = os.path.join(out_dir, "person_velocity.csv")
    with open(csv_path, "w") as f:
        w = csv.DictWriter(f, fieldnames=fieldnames, extrasaction="ignore")
        w.writeheader()
        for r in rows:
            w.writerow(r)

    # Robot stationary check
    odom_vr = [r["odom_vr"] for r in odom_rows]
    odom_wz = [abs(r["odom_wz"]) for r in odom_rows]
    robot_stats = {
        "odom_samples": len(odom_rows),
        "odom_vr_mean": float(np.mean(odom_vr)) if odom_vr else float("nan"),
        "odom_vr_max": float(np.max(odom_vr)) if odom_vr else float("nan"),
        "odom_wz_mean": float(np.mean(odom_wz)) if odom_wz else float("nan"),
        "odom_wz_max": float(np.max(odom_wz)) if odom_wz else float("nan"),
        "cmd_samples": len(cmd_rows),
    }

    # Metrics per signal stage (F0/F1/F2)
    f0 = stats_speed([math.hypot(r["raw_vx"], r["raw_vy"]) for r in rows])  # camera frame approx
    f1 = stats_speed([r["compensated_speed"] for r in rows])
    f2 = stats_speed([r["published_speed"] for r in rows])
    pos_derived = stats_speed([r["position_derived_speed"] for r in rows if math.isfinite(r.get("position_derived_speed", float("nan")))])

    ok_rows = [r for r in rows if r["tracking_state"] == 1]
    valid_ratio = (sum(r["velocity_valid"] for r in ok_rows) / len(ok_rows) * 100.0) if ok_rows else float("nan")

    summary = {
        "bag": args.bag,
        "test_type": args.test_type,
        "person_samples": len(rows),
        "bodies_empty_msgs": bodies_empty,
        "bodies_nonempty_msgs": bodies_nonempty,
        "bodies_unique_ids": sorted(bodies_ids),
        "robot": robot_stats,
        "F0_raw_camera_speed": f0,
        "F1_compensated_no_stationary": f1,
        "F2_published_final": f2,
        "position_derived_speed": pos_derived,
        "valid_ratio_ok_tracking_pct": valid_ratio,
        "jump_reject_count": sum(r["jump_rejected"] for r in rows),
        "stationary_zero_count": sum(r["stationary_threshold_applied"] for r in rows),
        "id_switches": detect_id_switches([r["person_id"] for r in rows]),
        "timestamp_gaps": len(detect_timestamp_gaps([r["time"] for r in rows])),
    }

    # Moving tests: compare to references
    if math.isfinite(args.manual_ref) and rows:
        pub_speeds = [r["published_speed"] for r in rows]
        ref = [args.manual_ref] * len(pub_speeds)
        summary["vs_manual_ref_published"] = mae_rmse_bias(pub_speeds, ref)
        summary["vs_manual_ref_compensated"] = mae_rmse_bias(
            [r["compensated_speed"] for r in rows], ref
        )

    if rows:
        pos_spd = [r["position_derived_speed"] for r in rows]
        pub_spd = [r["published_speed"] for r in rows]
        summary["published_vs_pos_derived"] = mae_rmse_bias(pub_spd, pos_spd, mask_min=0.05)

    summary_path = os.path.join(out_dir, "summary.json")
    with open(summary_path, "w") as f:
        json.dump(summary, f, indent=2)

    txt_path = os.path.join(out_dir, "summary.txt")
    with open(txt_path, "w") as f:
        f.write("=== ZED Person Velocity Analysis (robot stationary) ===\n")
        f.write("bag: %s\n" % args.bag)
        f.write("test: %s\n" % args.test_type)
        f.write("person_samples: %d\n" % len(rows))
        f.write("\n--- Robot stationary check ---\n")
        f.write("odom |v| mean=%.6f max=%.6f m/s\n" % (robot_stats["odom_vr_mean"], robot_stats["odom_vr_max"]))
        f.write("odom |wz| mean=%.6f max=%.6f rad/s\n" % (robot_stats["odom_wz_mean"], robot_stats["odom_wz_max"]))
        f.write("\n--- /bodies ---\n")
        f.write("empty=%d  nonempty=%d\n" % (bodies_empty, bodies_nonempty))
        if f2:
            f.write("\n--- F2 published speed (stationary ref=0) ---\n")
            for k, v in f2.items():
                f.write("  %s: %s\n" % (k, v))
        if f1:
            f.write("\n--- F1 compensated speed ---\n")
            for k, v in f1.items():
                f.write("  %s: %s\n" % (k, v))
        f.write("\nvalid_ratio (tracking OK): %.1f%%\n" % (valid_ratio if valid_ratio == valid_ratio else -1))
        f.write("jump_reject: %d  stationary_zero: %d  id_switches: %d  ts_gaps: %d\n" % (
            summary["jump_reject_count"], summary["stationary_zero_count"],
            summary["id_switches"], summary["timestamp_gaps"]))
        f.write("\nEngineering criteria (NOT ZED official specs):\n")
        f.write("  stationary: median<0.05, p95<0.10, valid>90%%\n")
        f.write("  moving: MAE<0.15, RMSE<0.20, dir median<15deg\n")
        if not rows:
            f.write("\nWARNING: no person velocity samples — run S1+ with person in view.\n")

    print("Wrote", csv_path)
    print("Wrote", summary_path)
    print("Wrote", txt_path)
    if rows:
        print("F2 published: median=%.4f p95=%.4f max=%.4f m/s" % (
            f2.get("median", float("nan")), f2.get("p95", float("nan")), f2.get("max", float("nan"))))
    else:
        print("No person samples in bag.")

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
        p = os.path.join(out_dir, name)
        fig.tight_layout()
        fig.savefig(p, dpi=120)
        plt.close(fig)
        print("Wrote", p)

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["position_x"] for r in rows], label="x")
    ax.plot(ts, [r["position_y"] for r in rows], label="y")
    ax.legend()
    ax.set_title("Position (map)")
    save(fig, "01_position.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["raw_speed"] for r in rows], label="F0 raw", alpha=0.6)
    ax.plot(ts, [r["compensated_speed"] for r in rows], label="F1 compensated", alpha=0.8)
    ax.plot(ts, [r["published_speed"] for r in rows], label="F2 published", alpha=0.9)
    if math.isfinite(args.manual_ref):
        ax.axhline(args.manual_ref, color="r", ls="--", label="manual ref")
    ax.plot(ts, [r.get("position_derived_speed", float("nan")) for r in rows],
            label="pos-derived", alpha=0.5)
    ax.legend()
    ax.set_title("Speed stages")
    save(fig, "02_speed_stages.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["published_vx"] for r in rows], label="pub_vx")
    ax.plot(ts, [r["published_vy"] for r in rows], label="pub_vy")
    ax.legend()
    save(fig, "03_published_velocity.png")

    fig, ax = plt.subplots(figsize=(10, 4))
    ax.plot(ts, [r["confidence"] for r in rows], label="confidence")
    ax.plot(ts, [r["tracking_state"] for r in rows], label="tracking_state")
    ax.legend()
    save(fig, "04_confidence_tracking.png")

    fig, ax = plt.subplots(figsize=(6, 4))
    spd = [r["published_speed"] for r in rows]
    ax.hist(spd, bins=30, edgecolor="k", alpha=0.7)
    ax.axvline(0.05, color="r", ls="--", label="0.05")
    ax.axvline(0.10, color="orange", ls="--", label="0.10")
    ax.set_title("Published speed histogram")
    ax.legend()
    save(fig, "05_speed_histogram.png")

    if odom_rows:
        fig, ax = plt.subplots(figsize=(10, 3))
        ot = [r["time"] - odom_rows[0]["time"] for r in odom_rows]
        ax.plot(ot, [r["odom_vr"] for r in odom_rows], label="|v_robot|")
        ax.plot(ot, [abs(r["odom_wz"]) for r in odom_rows], label="|wz|")
        ax.legend()
        ax.set_title("Robot odom (should be ~0)")
        save(fig, "06_robot_odom.png")


if __name__ == "__main__":
    main()
