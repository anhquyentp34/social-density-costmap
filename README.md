# Social Density Costmap

Full **ROS Noetic** workspace slice to reproduce real-robot social-density costmap experiments (proxemic / social layers / density gate, live and scenario-latch modes).

After build, run the same entry scripts as on the lab machine:

```bash
./tracks/D-density-costmap/run_real_exp1_proxemic_only.sh
./tracks/D-density-costmap/run_real_exp2_social_layers.sh
./tracks/D-density-costmap/run_real_exp3_density_gate.sh
./tracks/D-density-costmap/run_real_exp4_proxemic_only_pos.sh
./tracks/D-density-costmap/run_real_exp5_social_layers_pos.sh
./tracks/D-density-costmap/run_real_exp6_density_gate_pos.sh
```

## What is included

| Area | Packages / paths |
|------|------------------|
| Robot bringup | `x_omni4wd_bringup`, `x_omni4wd_description`, `rplidar_ros` |
| Odom / filters | `x_omni4wd_slam`, `srf_laser_odometry` |
| Navigation | `x_omni4wd_navigation`, `teb_local_planner` |
| Perception | `zed_sdk_bridge`, `fer_emotion` (exp2–3), `ultralytics_ros` (exp1–3) |
| Social stack | `social_msgs`, `social_density_fusion`, `social_navigation_layers`, `social_navigation_monitor` |
| Viz / goals | `animated_marker_*`, `goal_sender`, standing mesh |
| Scripts | `tracks/D-density-costmap/`, `tracks/lib/`, `scripts/` |

## Requirements (other machine)

### System
- Ubuntu **20.04** + **ROS Noetic**
- NVIDIA GPU + drivers (ZED / YOLO / FER)
- **ZED SDK 4.x** installed under `/usr/local/zed` (not shipped in this repo)
- Display for RViz (`DISPLAY`)

### Apt (typical)
```bash
sudo apt update
sudo apt install -y \
  ros-noetic-desktop-full \
  ros-noetic-navigation \
  ros-noetic-robot-localization \
  ros-noetic-laser-filters \
  ros-noetic-mrpt2 \
  ros-noetic-people-msgs \
  ros-noetic-costmap-converter \
  ros-noetic-libg2o \
  python3-serial ffmpeg
```

### Python (exp1–3 YOLO / FER)
```bash
pip3 install --user ultralytics onnx opencv-python-headless numpy
# FER models: see fer_emotion/scripts/download_fer_model.sh if missing
```

## Build

```bash
git clone https://github.com/anhquyentp34/social-density-costmap.git
cd social-density-costmap
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_BUILD_TYPE=Release
source devel/setup.bash
```

Optional: `export WORKSPACE_PATH=$PWD` (scripts auto-detect workspace if `src/` + `tracks/` exist).

## Run experiments

Connect robot hardware (Arduino, RPLidar, ZED) as on the lab setup, then:

```bash
cd /path/to/social-density-costmap
source devel/setup.bash
./tracks/D-density-costmap/run_real_exp1_proxemic_only.sh
# …
./tracks/D-density-costmap/run_real_exp6_density_gate_pos.sh
```

Optional duration (minutes): `./tracks/D-density-costmap/run_real_exp3_density_gate.sh 15`

Outputs: `bags/`, `videos/`, `csv/`, `runs/` (gitignored).

| Exp | Mode | Notes |
|-----|------|--------|
| 1 | Live | Proxemic only + YOLO |
| 2 | Live | Social layers + FER + YOLO |
| 3 | Live | + density gate |
| 4 | Latch | Proxemic only (no FER) |
| 5 | Latch | Social layers, gate OFF |
| 6 | Latch | Social layers + density gate |

## Density gate (exp 3 & 6)

Controller: `social_navigation_layers/.../path_density_weight_controller.py`  
Relaxes `*_density_weight` when stuck; restores after travel ≥ 2 m (see paper / docs).

## Hardware notes

- Default ports: `/dev/robot_arduino`, `/dev/robot_lidar` (udev rules may be needed)
- Default odom: `srf_zed_imu`
- Without ZED SDK / camera, bringup will fail — this release targets the **real robot** stack

## Citation

```bibtex
@inproceedings{YourKey2026,
  title={...},
  author={...},
  booktitle={...},
  year={2026}
}
```

## License

MIT for our packages. Third-party packages keep their original licenses (TEB, RPLidar, Ultralytics, …). ZED SDK is subject to Stereolabs terms and is **not** redistributed here.
EOF
