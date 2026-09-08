#!/usr/bin/env bash
# Chạy tự động mapping Cartographer cho x_omni4wd (RPLIDAR + RF2O).
# Cách dùng:
#   chmod +x run_x_omni4wd_cartographer_mapping.sh
#   ./run_x_omni4wd_cartographer_mapping.sh
#   ./run_x_omni4wd_cartographer_mapping.sh open_rviz:=false
#
# Thêm tham số roslaunch tùy ý (truyền xuống x_omni4wd_cartographer_mapping.launch).

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# scripts/ -> x_omni4wd_slam -> src -> workspace
WS_ROOT="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

NOETIC="/opt/ros/noetic/setup.bash"
ISOLATED="${WS_ROOT}/install_isolated/setup.bash"
DEVEL="${WS_ROOT}/devel/setup.bash"

if [[ ! -f "${NOETIC}" ]]; then
  echo "Lỗi: Không tìm thấy ${NOETIC}" >&2
  exit 1
fi

if [[ ! -f "${ISOLATED}" ]]; then
  echo "Cảnh báo: Chưa có ${ISOLATED} — Cartographer thường cần catkin_make_isolated." >&2
  echo "         Tiếp tục dùng chỉ Noetic + devel (có thể thiếu cartographer_ros)." >&2
fi

# shellcheck source=/dev/null
source "${NOETIC}"

if [[ -f "${ISOLATED}" ]]; then
  # shellcheck source=/dev/null
  source "${ISOLATED}"
fi

if [[ ! -f "${DEVEL}" ]]; then
  echo "Lỗi: Không tìm thấy ${DEVEL} — hãy catkin_make workspace trước." >&2
  exit 1
fi
# shellcheck source=/dev/null
source "${DEVEL}"

exec roslaunch x_omni4wd_slam x_omni4wd_cartographer_mapping.launch "$@"
