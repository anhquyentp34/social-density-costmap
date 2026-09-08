#!/usr/bin/env bash
# Track D — chỉ chạy SII/SGI monitor (horizontal), không launch robot/nav.
# Dùng chung cho live và POS; cần ROS master + /amcl_pose nếu muốn chỉ số theo robot.
set -euo pipefail
cd "$(dirname "$0")/../.."
# shellcheck disable=SC1091
source devel/setup.bash 2>/dev/null || source /home/quyenanh/omni4wd_ws/devel/setup.bash
CANONICAL="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_scenario_canonical_trackD_real.yaml"
exec roslaunch social_navigation_monitor track_d_sii_sgi_monitor.launch \
  use_scenario_gt:=true \
  scenario_gt_path:="$CANONICAL" \
  open_dashboard:=true \
  dashboard_layout:=horizontal \
  dashboard_image_width:=1840 \
  "$@"
