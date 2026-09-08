#!/usr/bin/env bash
# Ghi điểm đích cho exp4 / exp5 / exp6 (cùng waypoints_trackD_real.yaml).
#
# Chuẩn bị:
#   1. Stack robot + map + AMCL đang chạy (/amcl_pose có dữ liệu).
#   2. Teleop hoặc lái robot tới từng vị trí, gõ lệnh trong terminal này.
#
# Cách dùng:
#   ./tracks/D-density-costmap/run_record_waypoints_trackd.sh
#   ./tracks/D-density-costmap/run_record_waypoints_trackd.sh /path/to/custom.yaml
#
# Lệnh trong recorder: start | goal [GOAL_1] | list | undo | save | quit
# RViz: thêm MarkerArray → /waypoints_trackd_preview

set -euo pipefail
cd "$(dirname "$0")/../.."

source /opt/ros/noetic/setup.bash
# shellcheck disable=SC1091
[ -f devel/setup.bash ] && source devel/setup.bash

OUT="${1:-$(rospack find goal_sender)/config/D-density-costmap/waypoints_trackD_real.yaml}"

echo "=============================================="
echo " Track D — ghi waypoints (exp4/5/6)"
echo " File đích: $OUT"
echo " Cần topic: /amcl_pose"
echo "=============================================="

if ! rostopic list 2>/dev/null | grep -qx '/amcl_pose'; then
  echo "⚠️  Chưa thấy /amcl_pose — hãy chạy stack navigation trước."
  echo "   Ví dụ (không gửi goal tự động):"
  echo "   ./tracks/D-density-costmap/run_real_exp4_proxemic_only_pos.sh 0 use_goal_sender:=false"
  read -r -p "Vẫn tiếp tục? [y/N] " ans
  [[ "${ans,,}" == "y" ]] || exit 1
fi

exec rosrun goal_sender record_waypoints_trackd.py "_output_file:=$OUT"
