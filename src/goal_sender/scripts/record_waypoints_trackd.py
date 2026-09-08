#!/usr/bin/env python3
"""
Ghi waypoints Track D robot thật (exp4/5/6 dùng chung waypoints_trackD_real.yaml).

Cách dùng:
  1. Chạy stack navigation (AMCL + map) — có thể teleop robot tới từng điểm.
  2. rosrun goal_sender record_waypoints_trackd.py
     hoặc: ./tracks/D-density-costmap/run_record_waypoints_trackd.sh

Lệnh trong terminal (Enter sau mỗi lệnh):
  start          — ghi điểm START (xanh)
  goal [tên]     — ghi GOAL_n hoặc tên tuỳ chọn (cam)
  add [tên]      — alias của goal
  list           — in danh sách
  undo           — xóa điểm cuối
  save           — ghi YAML (mặc định: config/D-density-costmap/waypoints_trackD_real.yaml)
  quit           — save rồi thoát

Tham số ROS (~):
  output_file    — đường dẫn YAML đích
  amcl_topic     — mặc định /amcl_pose
  marker_topic   — mặc định /waypoints_trackd_preview
"""

from __future__ import print_function

import math
import os
import sys
import threading
from datetime import datetime

import rospy
import rospkg
import tf.transformations
import yaml
from geometry_msgs.msg import PoseWithCovarianceStamped
from std_msgs.msg import ColorRGBA
from visualization_msgs.msg import Marker, MarkerArray

DEFAULT_NAMES = ["START", "GOAL_1", "GOAL_2", "GOAL_3", "GOAL_4"]
START_COLOR = [0.0, 1.0, 0.0]
GOAL_COLOR = [1.0, 0.5, 0.0]


def default_output_path():
    try:
        pkg = rospkg.RosPack().get_path("goal_sender")
        return os.path.join(
            pkg, "config", "D-density-costmap", "waypoints_trackD_real.yaml"
        )
    except Exception:
        return ""


def yaw_from_pose(pose):
    q = pose.orientation
    _, _, yaw = tf.transformations.euler_from_quaternion(
        [q.x, q.y, q.z, q.w]
    )
    return yaw


def load_yaml(path):
    if not path or not os.path.isfile(path):
        return {"num_cycles": 1, "movement_mode": "cycle", "waypoints": []}
    with open(path, "r") as f:
        data = yaml.safe_load(f) or {}
    if "waypoints" not in data:
        data["waypoints"] = []
    data.setdefault("num_cycles", 1)
    data.setdefault("movement_mode", "cycle")
    return data


def save_yaml(path, data):
    os.makedirs(os.path.dirname(path), exist_ok=True)
    header = (
        "# Track D — Robot thật — Waypoints cho goal_sender\n"
        "# Lộ trình: START → GOAL_1 → GOAL_2 → GOAL_3 → GOAL_4 (không GOAL_5)\n"
        "# Ghi bởi record_waypoints_trackd.py — {ts}\n\n".format(
            ts=datetime.now().strftime("%d-%m-%Y %H:%M")
        )
    )
    body = yaml.dump(
        {
            "num_cycles": int(data.get("num_cycles", 1)),
            "movement_mode": str(data.get("movement_mode", "cycle")),
            "waypoints": data.get("waypoints", []),
        },
        default_flow_style=False,
        allow_unicode=True,
        sort_keys=False,
    )
    with open(path, "w") as f:
        f.write(header + body)


class WaypointRecorder(object):
    def __init__(self):
        self.output_file = rospy.get_param("~output_file", default_output_path())
        amcl_topic = rospy.get_param("~amcl_topic", "/amcl_pose")
        marker_topic = rospy.get_param("~marker_topic", "/waypoints_trackd_preview")

        self.data = load_yaml(self.output_file)
        self.waypoints = self.data["waypoints"]
        self._pose_lock = threading.Lock()
        self._last_pose = None

        self.marker_pub = rospy.Publisher(marker_topic, MarkerArray, queue_size=1, latch=True)
        rospy.Subscriber(amcl_topic, PoseWithCovarianceStamped, self._on_amcl, queue_size=1)

        self._publish_markers()
        rospy.loginfo("Waypoint recorder — output: %s", self.output_file)
        rospy.loginfo("Đang chờ %s ... (teleop robot tới vị trí rồi gõ lệnh)", amcl_topic)
        self._print_help()

    def _on_amcl(self, msg):
        with self._pose_lock:
            self._last_pose = msg.pose.pose

    def _current_pose(self):
        with self._pose_lock:
            if self._last_pose is None:
                return None
            p = self._last_pose.position
            yaw = yaw_from_pose(self._last_pose)
            return p.x, p.y, yaw

    def _next_default_name(self):
        used = {w.get("name") for w in self.waypoints}
        for name in DEFAULT_NAMES:
            if name not in used:
                return name
        n = len(self.waypoints) + 1
        return "GOAL_{}".format(n)

    def add_waypoint(self, name=None, as_start=False):
        pose = self._current_pose()
        if pose is None:
            print("❌ Chưa có /amcl_pose — bật AMCL và localization trước.")
            return
        x, y, yaw = pose
        if as_start:
            name = "START"
            color = START_COLOR
        else:
            name = name or self._next_default_name()
            color = GOAL_COLOR if name != "START" else START_COLOR

        entry = {
            "name": str(name),
            "x": round(x, 4),
            "y": round(y, 4),
            "yaw": round(yaw, 8),
            "color": color,
        }
        # Thay START/GOAL trùng tên
        self.waypoints = [w for w in self.waypoints if w.get("name") != name]
        self.waypoints.append(entry)
        print(
            "✅ {} @ ({:.4f}, {:.4f}, yaw={:.4f})".format(name, x, y, yaw)
        )
        self._publish_markers()

    def list_waypoints(self):
        if not self.waypoints:
            print("(trống)")
            return
        for i, w in enumerate(self.waypoints):
            print(
                "  [{:d}] {:10s}  x={:.4f}  y={:.4f}  yaw={:.4f}".format(
                    i, w.get("name", "?"), w["x"], w["y"], w["yaw"]
                )
            )

    def undo(self):
        if not self.waypoints:
            print("(không có gì để xóa)")
            return
        removed = self.waypoints.pop()
        print("↩ Đã xóa:", removed.get("name"))
        self._publish_markers()

    def save(self):
        self.data["waypoints"] = self.waypoints
        save_yaml(self.output_file, self.data)
        print("💾 Đã lưu {} điểm → {}".format(len(self.waypoints), self.output_file))

    def _publish_markers(self):
        arr = MarkerArray()
        delete_all = Marker()
        delete_all.action = Marker.DELETEALL
        arr.markers.append(delete_all)

        for i, w in enumerate(self.waypoints):
            color = w.get("color", GOAL_COLOR)
            sphere = Marker()
            sphere.header.frame_id = "map"
            sphere.header.stamp = rospy.Time.now()
            sphere.ns = "waypoints_trackd"
            sphere.id = i * 2
            sphere.type = Marker.SPHERE
            sphere.action = Marker.ADD
            sphere.pose.position.x = w["x"]
            sphere.pose.position.y = w["y"]
            sphere.pose.position.z = 0.15
            sphere.pose.orientation.w = 1.0
            sphere.scale.x = sphere.scale.y = sphere.scale.z = 0.35
            sphere.color = ColorRGBA(color[0], color[1], color[2], 0.9)
            arr.markers.append(sphere)

            text = Marker()
            text.header.frame_id = "map"
            text.header.stamp = rospy.Time.now()
            text.ns = "waypoints_trackd_labels"
            text.id = i * 2 + 1
            text.type = Marker.TEXT_VIEW_FACING
            text.action = Marker.ADD
            text.pose.position.x = w["x"]
            text.pose.position.y = w["y"]
            text.pose.position.z = 0.65
            text.pose.orientation.w = 1.0
            text.scale.z = 0.35
            text.color = ColorRGBA(1.0, 1.0, 1.0, 1.0)
            text.text = w.get("name", "WP")
            arr.markers.append(text)

        self.marker_pub.publish(arr)

    def _print_help(self):
        print(
            "\n--- Waypoint recorder (exp4/5/6 → waypoints_trackD_real.yaml) ---\n"
            "  start              ghi START\n"
            "  goal [GOAL_1]      ghi goal (tên mặc định nếu bỏ trống)\n"
            "  list | undo | save | quit\n"
            "  RViz: MarkerArray topic /waypoints_trackd_preview\n"
        )

    def handle_line(self, line):
        parts = line.strip().split()
        if not parts:
            return True
        cmd = parts[0].lower()
        arg = parts[1] if len(parts) > 1 else None

        if cmd in ("help", "h", "?"):
            self._print_help()
        elif cmd == "start":
            self.add_waypoint(as_start=True)
        elif cmd in ("goal", "add", "g", "a"):
            self.add_waypoint(name=arg)
        elif cmd == "list":
            self.list_waypoints()
        elif cmd == "undo":
            self.undo()
        elif cmd == "save":
            self.save()
        elif cmd in ("quit", "exit", "q"):
            self.save()
            rospy.signal_shutdown("done")
            return False
        else:
            print("Không hiểu lệnh:", cmd, "— gõ help")
        return True


def stdin_loop(recorder):
    while not rospy.is_shutdown():
        try:
            line = sys.stdin.readline()
            if not line:
                break
            if not recorder.handle_line(line):
                break
        except EOFError:
            break


if __name__ == "__main__":
    rospy.init_node("record_waypoints_trackd")
    rec = WaypointRecorder()
    t = threading.Thread(target=stdin_loop, args=(rec,))
    t.daemon = True
    t.start()
    rospy.spin()
