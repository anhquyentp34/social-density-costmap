#!/usr/bin/env bash
# Test FER trên webcam (IdeaPad) — không cần ROS master
set -euo pipefail
WS="${HOME}/omni4wd_ws"
SCRIPT="${WS}/src/fer_emotion/scripts/test_fer_webcam.py"
if [[ ! -f "${SCRIPT}" ]]; then
  echo "[FAIL] Không tìm thấy ${SCRIPT}"
  exit 1
fi
exec python3 "${SCRIPT}" "$@"
