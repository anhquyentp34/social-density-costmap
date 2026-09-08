#!/usr/bin/env bash
# Track D — Simulation Gazebo — pipeline đầy đủ (mirror ICR2026)
#
# Cách dùng:
#   ./tracks/D-density-costmap/run_sim_track_d.sh [EXP_NAME] [DURATION_MIN] [LAUNCH_SELECT]
#
#   LAUNCH_SELECT: social_v5 (mặc định)
#     | exp1 | proxemic_only | tn1
#     | exp2 | social_layers | tn2
#     | exp3 | density_gate  | tn3
#     | base | costmapEx | *.launch
#
# Output: bags/csv/videos/runs/D-density-costmap/<EXP>/

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/experiment_pipeline.sh"

PIPE_TRACK="D-density-costmap"
PIPE_MODE="sim"
PIPE_READY_TOPIC="/gazebo/model_states"
PIPE_READY_TIMEOUT_SEC=120
PIPE_WALL_SEGMENTS="$PIPE_WS/environment/D-density-costmap/wall_segments.txt"

_ts=$(date +"%Hh%M_%d-%m-%Y")
PIPE_EXPERIMENT_NAME="${1:-D_sim_${_ts}}"
PIPE_DURATION_MINUTES="${2:-0}"
LAUNCH_SELECT="${3:-}"

LAUNCH_PKG="x_omni4wd_simulation"
LAUNCH_FILE="D-density-costmap/sim_social_v5.launch"

select_launch_file() {
  case "$LAUNCH_SELECT" in
    ""|social_v5) LAUNCH_FILE="D-density-costmap/sim_social_v5.launch" ;;
    exp1|proxemic_only|tn1|d1) LAUNCH_FILE="D-density-costmap/sim_D_exp1_proxemic_only.launch" ;;
    exp2|social_layers|tn2|d2) LAUNCH_FILE="D-density-costmap/sim_D_exp2_social_layers.launch" ;;
    exp3|density_gate|tn3|d3) LAUNCH_FILE="D-density-costmap/sim_D_exp3_density_gate.launch" ;;
    base) LAUNCH_FILE="D-density-costmap/sim_base.launch" ;;
    costmapEx) LAUNCH_FILE="D-density-costmap/sim_costmapEx.launch" ;;
    *.launch) LAUNCH_FILE="$LAUNCH_SELECT" ;;
    *)
      echo "❌ launch không hợp lệ: ${LAUNCH_SELECT}"
      echo "   Hợp lệ: social_v5 | exp1|proxemic_only | exp2|social_layers | exp3|density_gate | base | costmapEx"
      exit 1
      ;;
  esac
  local launch_path="$PIPE_WS/src/$LAUNCH_PKG/launch/$LAUNCH_FILE"
  if [ ! -f "$launch_path" ]; then
    echo "❌ Không tìm thấy: $launch_path"
    exit 1
  fi
  LAUNCH_ABS="$launch_path"
}

select_launch_file

PIPE_TOPICS=(
  "/amcl_pose"
  "/gazebo/model_states"
  "/sii"
  "/sgi"
  "/move_base/status"
  "/move_base/GlobalPlanner/plan"
  "/social_state"
  "/density_info"
  "/cmd_vel"
)

pipeline_start_launch() {
  pipeline_log "🚀 roslaunch $LAUNCH_ABS"
  if command -v setsid >/dev/null 2>&1; then
    setsid roslaunch "$LAUNCH_ABS" >>"$PIPE_LOG_FILE" 2>&1 &
  else
    roslaunch "$LAUNCH_ABS" >>"$PIPE_LOG_FILE" 2>&1 &
  fi
  LAUNCH_PID=$!
  pipeline_log "   roslaunch PID: $LAUNCH_PID"
}

pipeline_run
