#!/usr/bin/env python3
"""
Entry VCCA / actor di bo: Kalman van toc cho actor14, actor15 (Proxemic + emotion tren costmap).

Dung cung logic voi social_density_fusion_node.py; chi dat bien moi truong
SOCIAL_DENSITY_KALMAN_VELOCITY_ACTORS truoc khi chay file goc (co the ghi de
bang tham so ROS ~kalman_velocity_actors neu da set tren param server).
"""
import os
import runpy
from pathlib import Path

os.environ["SOCIAL_DENSITY_KALMAN_VELOCITY_ACTORS"] = "actor14,actor15"
runpy.run_path(str(Path(__file__).resolve().parent / "social_density_fusion_node.py"), run_name="__main__")
