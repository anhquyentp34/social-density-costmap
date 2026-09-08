#!/usr/bin/env python3
"""
Entry VCCA / actor đi bộ: Kalman vận tốc cho actor63, actor64 (Proxemic + emotion trên costmap).

Dùng cùng logic với social_density_fusion_node.py; chỉ đặt biến môi trường
SOCIAL_DENSITY_KALMAN_VELOCITY_ACTORS trước khi chạy file gốc (có thể ghi đè
bằng tham số ROS ~kalman_velocity_actors nếu đã set trên param server).
"""
import os
import runpy
from pathlib import Path

os.environ["SOCIAL_DENSITY_KALMAN_VELOCITY_ACTORS"] = "actor63,actor64"
runpy.run_path(str(Path(__file__).resolve().parent / "social_density_fusion_node.py"), run_name="__main__")
