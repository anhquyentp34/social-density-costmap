#!/usr/bin/env bash
# Track D — Density-based layered social costmap (ZED body tracking + YOLO objects).
# RQ-D: Điều phối layered social costmap theo mật độ khi TEB kẹt.
#
# Ví dụ:
#   ./run_track_d.sh
#   ./run_track_d.sh odom_mode:=srf_zed_imu open_rviz:=true
#   ./run_track_d.sh social_scenario_config:=/path/to/custom.yaml
#   ./run_track_d.sh use_yolo:=false use_live_objects:=false   # social không có YOLO objects
#
# Mặc định: odom_mode=srf_zed_imu, social ON, YOLO ON (cho /objects vào social fusion)
exec "$(dirname "$0")/../../run_navigation_with_selectable_odom.sh" \
  odom_mode:=srf_zed_imu \
  use_zed:=true \
  use_yolo:=true \
  use_yolo_obstacle_layer:=false \
  use_social_density_fusion:=true \
  use_path_density_weight_controller:=true \
  zed_body_enable_visualizer:=true \
  "$@"
