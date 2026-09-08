#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Log timeline ρ + *_density_weight (+ hệ số suy giảm social) cho MATLAB.

Publish:
  /social_costmap/layer_weights       std_msgs/Float32MultiArray
      data = [w_proxemic, w_group, w_interaction, w_emotion, rho]
  /social_costmap/layer_weights_json  std_msgs/String (JSON)

CSV (nếu ~csv_path khác rỗng):
  timestamp, rho, people_count,
  w_proxemic, w_group, w_interaction, w_emotion,
  atten_proxemic, atten_group, atten_interaction, atten_emotion

atten_* = exp(-w * rho)  — gần 1: lớp social đầy đủ; gần 0: social “biến mất” trên costmap
(cùng dạng với scale hình học group/object/emotion và hệ số 1/exp(w*ρ) của proxemic).
"""
from __future__ import print_function

import csv
import json
import math
import os

import rospy
from std_msgs.msg import Float32MultiArray, MultiArrayDimension, String

try:
    from social_density_fusion.msg import DensityInfo
except ImportError:
    DensityInfo = None

# Ưu tiên param global costmap (controller đồng bộ global+local)
PREFERRED_PARAMS = {
    "w_proxemic": (
        "/move_base/global_costmap/proxemic/proxemic_density_weight",
        "/move_base/local_costmap/proxemic/proxemic_density_weight",
    ),
    "w_group": (
        "/move_base/global_costmap/human_group_layer/human_group_density_weight",
        "/move_base/local_costmap/human_group_layer/human_group_density_weight",
    ),
    "w_interaction": (
        "/move_base/global_costmap/human_object_layer/human_object_density_weight",
        "/move_base/local_costmap/human_object_layer/human_object_density_weight",
    ),
    "w_emotion": (
        "/move_base/global_costmap/emotion_layer/emotion_density_weight",
        "/move_base/local_costmap/emotion_layer/emotion_density_weight",
    ),
}

SUFFIX_FALLBACK = {
    "w_proxemic": ("proxemic_density_weight",),
    "w_group": ("human_group_density_weight", "group_density_weight"),
    "w_interaction": (
        "human_object_density_weight",
        "interaction_density_weight",
    ),
    "w_emotion": ("emotion_density_weight",),
}


def _atten(w, rho):
    if w is None or rho is None:
        return float("nan")
    if math.isnan(w) or math.isnan(rho):
        return float("nan")
    return math.exp(-float(w) * float(rho))


class LayerWeightsLogger(object):
    def __init__(self):
        self.rate_hz = float(rospy.get_param("~rate", 10.0))
        self.rho = float("nan")
        self.people_count = float("nan")
        self.csv_path = rospy.get_param("~csv_path", "").strip()
        self._csv_file = None
        self._csv_writer = None
        self._warned_missing = set()

        self.pub = rospy.Publisher(
            "/social_costmap/layer_weights", Float32MultiArray, queue_size=10
        )
        self.pub_json = rospy.Publisher(
            "/social_costmap/layer_weights_json", String, queue_size=10
        )

        rho_topic = rospy.get_param("~rho_topic", "/density_info")
        if rho_topic and DensityInfo is not None:
            rospy.Subscriber(rho_topic, DensityInfo, self._on_density_info, queue_size=10)
        elif rho_topic and DensityInfo is None:
            rospy.logwarn(
                "layer_weights_logger: DensityInfo not importable — skip %s", rho_topic
            )

        if self.csv_path:
            parent = os.path.dirname(self.csv_path)
            if parent:
                os.makedirs(parent, exist_ok=True)
            self._csv_file = open(self.csv_path, "w", newline="")
            self._csv_writer = csv.writer(self._csv_file)
            self._csv_writer.writerow(
                [
                    "timestamp",
                    "rho",
                    "people_count",
                    "w_proxemic",
                    "w_group",
                    "w_interaction",
                    "w_emotion",
                    "atten_proxemic",
                    "atten_group",
                    "atten_interaction",
                    "atten_emotion",
                ]
            )
            self._csv_file.flush()
            rospy.loginfo("layer_weights_logger: CSV → %s", self.csv_path)

    def _on_density_info(self, msg):
        try:
            self.rho = float(msg.density)
            self.people_count = float(msg.people_count)
        except (TypeError, ValueError, AttributeError):
            pass

    def _lookup(self, key):
        for name in PREFERRED_PARAMS.get(key, ()):
            if rospy.has_param(name):
                try:
                    return float(rospy.get_param(name)), name
                except (TypeError, ValueError, KeyError):
                    continue
        suffixes = SUFFIX_FALLBACK.get(key, ())
        try:
            names = rospy.get_param_names()
        except Exception:
            names = []
        for suf in suffixes:
            for name in names:
                if name.endswith(suf) or name.endswith("/" + suf):
                    try:
                        return float(rospy.get_param(name)), name
                    except (TypeError, ValueError, KeyError):
                        continue
        return float("nan"), ""

    def spin(self):
        rate = rospy.Rate(self.rate_hz)
        while not rospy.is_shutdown():
            resolved = {}
            values = []
            wmap = {}
            for key in ("w_proxemic", "w_group", "w_interaction", "w_emotion"):
                val, src = self._lookup(key)
                wmap[key] = val
                resolved[key] = {
                    "value": None if math.isnan(val) else val,
                    "param": src,
                }
                values.append(val)
                if src == "" and key not in self._warned_missing:
                    rospy.logwarn("layer_weights_logger: missing param for %s", key)
                    self._warned_missing.add(key)
            values.append(self.rho)

            arr = Float32MultiArray()
            dim = MultiArrayDimension()
            dim.label = "w_proxemic,w_group,w_interaction,w_emotion,rho"
            dim.size = 5
            dim.stride = 5
            arr.layout.dim = [dim]
            arr.data = values
            self.pub.publish(arr)

            stamp = rospy.Time.now().to_sec()
            atten = {
                "atten_proxemic": _atten(wmap["w_proxemic"], self.rho),
                "atten_group": _atten(wmap["w_group"], self.rho),
                "atten_interaction": _atten(wmap["w_interaction"], self.rho),
                "atten_emotion": _atten(wmap["w_emotion"], self.rho),
            }
            payload = {
                "stamp": stamp,
                "rho": None if math.isnan(self.rho) else self.rho,
                "people_count": None
                if math.isnan(self.people_count)
                else self.people_count,
                "weights": resolved,
                "attenuation": {
                    k: (None if math.isnan(v) else v) for k, v in atten.items()
                },
            }
            self.pub_json.publish(String(data=json.dumps(payload, sort_keys=True)))

            if self._csv_writer is not None:
                self._csv_writer.writerow(
                    [
                        "%.6f" % stamp,
                        "" if math.isnan(self.rho) else "%.8f" % self.rho,
                        ""
                        if math.isnan(self.people_count)
                        else "%d" % int(self.people_count),
                        "" if math.isnan(wmap["w_proxemic"]) else "%.6f" % wmap["w_proxemic"],
                        "" if math.isnan(wmap["w_group"]) else "%.6f" % wmap["w_group"],
                        ""
                        if math.isnan(wmap["w_interaction"])
                        else "%.6f" % wmap["w_interaction"],
                        "" if math.isnan(wmap["w_emotion"]) else "%.6f" % wmap["w_emotion"],
                        ""
                        if math.isnan(atten["atten_proxemic"])
                        else "%.8f" % atten["atten_proxemic"],
                        ""
                        if math.isnan(atten["atten_group"])
                        else "%.8f" % atten["atten_group"],
                        ""
                        if math.isnan(atten["atten_interaction"])
                        else "%.8f" % atten["atten_interaction"],
                        ""
                        if math.isnan(atten["atten_emotion"])
                        else "%.8f" % atten["atten_emotion"],
                    ]
                )
                self._csv_file.flush()
            rate.sleep()

    def close(self):
        if self._csv_file is not None:
            self._csv_file.close()
            self._csv_file = None


def main():
    rospy.init_node("layer_weights_logger")
    node = LayerWeightsLogger()
    try:
        node.spin()
    finally:
        node.close()


if __name__ == "__main__":
    main()
