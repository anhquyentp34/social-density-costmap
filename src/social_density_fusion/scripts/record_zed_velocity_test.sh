#!/usr/bin/env bash
# Ghi rosbag kiểm thử bù vận tốc ZED (V1–V5).
# Usage:
#   ./record_zed_velocity_test.sh V2
#   ./record_zed_velocity_test.sh V3 /tmp/mybag

set -euo pipefail

CASE="${1:-V1}"
OUT_DIR="${2:-$(pwd)/zed_velocity_bags}"
mkdir -p "${OUT_DIR}"
STAMP="$(date +%Y%m%d_%H%M%S)"
BAG="${OUT_DIR}/zed_vel_${CASE}_${STAMP}"

echo "=== ZED velocity compensation bag: ${CASE} ==="
echo "Output: ${BAG}.bag"
echo ""
echo "Checklist:"
case "${CASE}" in
  V1) echo "  Robot ĐỨNG, người ĐỨNG  → kỳ vọng final speed ≈ 0" ;;
  V2) echo "  Robot chạy thẳng 0.1–0.2 m/s, người ĐỨNG → final ≈ 0; raw ≈ -v_robot" ;;
  V3) echo "  Robot quay tại chỗ (omega≠0), người ĐỨNG → final ≈ 0 (cần omega×r)" ;;
  V4) echo "  Robot ĐỨNG, người đi thẳng → hướng/speed ổn; motion≠facing có thể" ;;
  V5) echo "  Robot + người cùng chuyển động → không spike khi robot tăng tốc/quay" ;;
  *) echo "  Unknown case ${CASE}" ;;
esac
echo ""
echo "Ctrl+C để dừng ghi."

rosbag record -O "${BAG}" \
  /bodies \
  /social_state \
  /odom \
  /odometry/filtered \
  /tf \
  /tf_static \
  /cmd_vel \
  /zed_velocity_compensation/debug \
  /pedestrian_measurements
