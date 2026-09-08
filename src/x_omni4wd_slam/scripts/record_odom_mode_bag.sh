#!/usr/bin/env bash
# Ghi rosbag theo odom_mode / test_name / run_index.
# Topic lấy từ hệ thống thực tế của VCCA2026 (không hard-code wrapper ZED).
#
# Cách dùng:
#   ./record_odom_mode_bag.sh <odom_mode> <test_name> [run_index]
# Ví dụ:
#   ./record_odom_mode_bag.sh wheel_rf2o_zed square 01
#   ./record_odom_mode_bag.sh baseline body_static 02
#
# Test gợi ý: static | straight | lateral | rotation | square | body_static | body_moving

set -euo pipefail

ODOM_MODE="${1:?usage: $0 <odom_mode> <test_name> [run_index]}"
TEST_NAME="${2:?usage: $0 <odom_mode> <test_name> [run_index]}"
RUN_INDEX="${3:-01}"
DATE_STR="$(date +%Y-%m-%d)"
WS="${WORKSPACE_PATH:-/home/quyenanhpt/VCCA2026}"
OUT_DIR="${BAG_OUT_DIR:-$WS/odom_mode_bags}"
mkdir -p "$OUT_DIR"

BAG_NAME="${DATE_STR}_${ODOM_MODE}_${TEST_NAME}_run${RUN_INDEX}"
BAG_PATH="${OUT_DIR}/${BAG_NAME}"

# Topic tối thiểu — chỉ ghi nếu đang có trên master (tránh rosbag fail)
CANDIDATES=(
  /odom_wheel
  /odom_rf2o
  /odom_zed
  /odometry/filtered
  /odom
  /zed_imu
  /bodies
  /social_state
  /tf
  /tf_static
  /cmd_vel
  /scan
  /amcl_pose
  /wheel_encoder_joint_states
)

TOPICS=()
for t in "${CANDIDATES[@]}"; do
  if rostopic list 2>/dev/null | grep -qx "$t"; then
    TOPICS+=("$t")
  fi
done

if [ "${#TOPICS[@]}" -eq 0 ]; then
  echo "ERROR: không thấy topic nào. Hãy chạy navigation trước (roscore + launch)." >&2
  exit 1
fi

echo "Recording → ${BAG_PATH}.bag"
echo "odom_mode=${ODOM_MODE} test=${TEST_NAME} run=${RUN_INDEX}"
echo "topics: ${TOPICS[*]}"
echo "Ctrl-C để dừng."

# shellcheck disable=SC2086
exec rosbag record -O "${BAG_PATH}" "${TOPICS[@]}"
