#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""Publish /social_state_gt từ kịch bản chuẩn (canonical / actor layout).

Dùng cho SII/SGI đo lường + (annotation) relay sang /social_state cho costmap.
Có service ~reload để nạp lại YAML sau khi kéo RViz.
"""
from __future__ import annotations

import math
import os

import rospy
import yaml
from geometry_msgs.msg import Pose, Twist
from social_msgs.msg import (
    SocialGroup,
    SocialGroups,
    SocialInteraction,
    SocialInteractions,
    SocialPeople,
    SocialPerson,
    SocialState,
)
from std_srvs.srv import Trigger, TriggerResponse


def _yaw_to_quat(yaw: float):
    return (0.0, 0.0, math.sin(yaw * 0.5), math.cos(yaw * 0.5))


def _make_person(name: str, x: float, y: float, yaw: float, emotion: str) -> SocialPerson:
    p = SocialPerson()
    p.name = str(name)
    p.reliability = 1.0
    p.emotion = str(emotion or "Neutral")
    p.position = Pose()
    p.position.position.x = float(x)
    p.position.position.y = float(y)
    p.position.position.z = 0.0
    qx, qy, qz, qw = _yaw_to_quat(float(yaw))
    p.position.orientation.x = qx
    p.position.orientation.y = qy
    p.position.orientation.z = qz
    p.position.orientation.w = qw
    p.velocity = Twist()
    return p


class ScenarioGtSocialStatePublisher:
    def __init__(self):
        path = rospy.get_param("~scenario_path", "")
        if not path:
            path = os.path.join(
                os.path.dirname(__file__),
                "..",
                "config",
                "D-density-costmap",
                "real",
                "social_scenario_canonical_trackD_real.yaml",
            )
        self.scenario_path = os.path.abspath(path)
        rate_hz = float(rospy.get_param("~rate", 10.0))
        self.frame_id = str(rospy.get_param("~frame_id", "map"))
        self.exclude_pedestrians = bool(rospy.get_param("~exclude_pedestrians", True))
        self.output_topic = str(rospy.get_param("~output_topic", "/social_state_gt"))

        self.people_by_name = {}
        self.groups_cfg = []
        self.interactions_cfg = []
        self._load_scenario(self.scenario_path)

        self.pub = rospy.Publisher(self.output_topic, SocialState, queue_size=2, latch=True)
        rospy.Service("~reload", Trigger, self.handle_reload)
        rospy.loginfo(
            "scenario_gt_social_state: %s | people=%d groups=%d interactions=%d → %s",
            self.scenario_path,
            len(self.people_by_name),
            len(self.groups_cfg),
            len(self.interactions_cfg),
            self.output_topic,
        )
        self.timer = rospy.Timer(rospy.Duration(1.0 / max(rate_hz, 0.5)), self._tick)
        self._tick(None)

    def _load_scenario(self, path: str) -> None:
        with open(path, "r", encoding="utf-8") as f:
            data = yaml.safe_load(f) or {}

        self.people_by_name = {}
        for actor in data.get("actors") or []:
            if not isinstance(actor, dict) or "name" not in actor:
                continue
            role = str(actor.get("role", "")).lower()
            if self.exclude_pedestrians and role == "pedestrian":
                continue
            name = str(actor["name"])
            self.people_by_name[name] = _make_person(
                name,
                float(actor.get("x", 0.0)),
                float(actor.get("y", 0.0)),
                float(actor.get("yaw", 0.0)),
                str(actor.get("emotion", "Neutral")),
            )

        for p in data.get("people") or []:
            if not isinstance(p, dict) or "name" not in p:
                continue
            name = str(p["name"])
            if name in self.people_by_name and p.get("emotion"):
                self.people_by_name[name].emotion = str(p["emotion"])

        self.groups_cfg = []
        for g in data.get("groups") or []:
            if not isinstance(g, dict):
                continue
            members = [str(m) for m in (g.get("members") or [])]
            members = [m for m in members if m in self.people_by_name]
            if len(members) < 2:
                continue
            cx = sum(self.people_by_name[m].position.position.x for m in members) / len(members)
            cy = sum(self.people_by_name[m].position.position.y for m in members) / len(members)
            if g.get("center_xy") and len(g["center_xy"]) >= 2:
                cx, cy = float(g["center_xy"][0]), float(g["center_xy"][1])
            radius = max(
                0.3,
                max(
                    math.hypot(
                        self.people_by_name[m].position.position.x - cx,
                        self.people_by_name[m].position.position.y - cy,
                    )
                    for m in members
                ),
            )
            self.groups_cfg.append(
                {
                    "name": str(g.get("group_name", "group")),
                    "members": members,
                    "cx": cx,
                    "cy": cy,
                    "radius": radius,
                }
            )

        objs = {
            str(o["name"]): o
            for o in (data.get("objects") or [])
            if isinstance(o, dict) and "name" in o
        }
        self.interactions_cfg = []
        for it in data.get("interactions") or []:
            if not isinstance(it, dict):
                continue
            oname = str(it.get("object_name", ""))
            parts = [str(p) for p in (it.get("participants") or []) if str(p) in self.people_by_name]
            if not oname or not parts:
                continue
            obj = objs.get(oname, {})
            self.interactions_cfg.append(
                {
                    "object_name": oname,
                    "x": float(obj.get("x", 0.0)),
                    "y": float(obj.get("y", 0.0)),
                    "z": float(obj.get("z", 0.0)),
                    "participants": parts,
                    "interaction_type": str(
                        it.get("interaction_type", obj.get("interaction_type", "seeing"))
                    ),
                }
            )
        self.scenario_path = os.path.abspath(path)

    def handle_reload(self, _req):
        try:
            new_path = str(rospy.get_param("~scenario_path", self.scenario_path))
            self._load_scenario(new_path)
            self._tick(None)
            msg = "reloaded %s people=%d" % (self.scenario_path, len(self.people_by_name))
            rospy.loginfo("scenario_gt_social_state: %s", msg)
            return TriggerResponse(success=True, message=msg)
        except Exception as exc:
            return TriggerResponse(success=False, message=str(exc))

    def _tick(self, _evt):
        msg = SocialState()
        msg.header.stamp = rospy.Time.now()
        msg.header.frame_id = self.frame_id
        msg.density = 0.0

        people = SocialPeople()
        people.header = msg.header
        for p in self.people_by_name.values():
            people.people.append(p)
        msg.people = people

        groups = SocialGroups()
        groups.header = msg.header
        for g in self.groups_cfg:
            gm = SocialGroup()
            gm.group_name = g["name"]
            gm.position.position.x = g["cx"]
            gm.position.position.y = g["cy"]
            gm.position.orientation.w = 1.0
            gm.radius = g["radius"]
            gm.reliability = 1.0
            for m in g["members"]:
                gm.members.append(self.people_by_name[m])
            groups.groups.append(gm)
        msg.groups = groups

        interactions = SocialInteractions()
        interactions.header = msg.header
        for it in self.interactions_cfg:
            im = SocialInteraction()
            im.object_name = it["object_name"]
            im.object_position.position.x = it["x"]
            im.object_position.position.y = it["y"]
            im.object_position.position.z = it["z"]
            im.object_position.orientation.w = 1.0
            im.reliability = 1.0
            im.interaction_type = it["interaction_type"]
            im.timestamp = msg.header.stamp
            for pname in it["participants"]:
                im.participants.append(self.people_by_name[pname])
            interactions.interactions.append(im)
        msg.interactions = interactions

        self.pub.publish(msg)


def main():
    rospy.init_node("scenario_gt_social_state_publisher")
    ScenarioGtSocialStatePublisher()
    rospy.spin()


if __name__ == "__main__":
    main()
