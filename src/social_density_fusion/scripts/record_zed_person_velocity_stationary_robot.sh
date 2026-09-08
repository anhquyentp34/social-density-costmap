#!/usr/bin/env bash
# Ghi rosbag đánh giá vận tốc người khi robot đứng yên.
# Usage:
#   ./record_zed_person_velocity_stationary_robot.sh S0 none 30
#   ./record_zed_person_velocity_stationary_robot.sh S1_2m stationary 30
#   ./record_zed_person_velocity_stationary_robot.sh M1_lateral_normal lateral 45

set -euo pipefail

TEST_NAME="${1:-S0_none}"
MOTION_TYPE="${2:-none}"
DURATION="${3:-30}"
OUT_DIR="${4:-$(pwd)/zed_person_velocity_bags}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${OUT_DIR}/${TEST_NAME}_${MOTION_TYPE}_${STAMP}"

mkdir -p "${OUT_DIR}"

echo "=== ZED person velocity bag (robot stationary) ==="
echo "Test: ${TEST_NAME}  motion: ${MOTION_TYPE}  duration: ${DURATION}s"
echo "Output: ${BAG}.bag"
echo ""
echo "Checklist robot:"
echo "  rostopic echo -n 5 /odom   → v_r ≈ 0, |omega| ≈ 0"
echo "  rostopic echo -n 5 /cmd_vel  → không có lệnh di chuyển"
echo ""
echo "Recording ${DURATION}s ..."

timeout "$((DURATION + 5))" rosbag record -O "${BAG}" \
  /bodies \
  /pedestrian_measurements \
  /social_state \
  /zed_person_velocity/debug \
  /zed_velocity_compensation/debug \
  /odom \
  /cmd_vel \
  /tf \
  /tf_static \
  --duration="${DURATION}" \
  2>/dev/null || true

if [[ -f "${BAG}.bag" ]]; then
  echo "Saved: ${BAG}.bag"
  rosbag info "${BAG}.bag" 2>/dev/null | head -25
else
  echo "ERROR: bag not created"
  exit 1
fi
