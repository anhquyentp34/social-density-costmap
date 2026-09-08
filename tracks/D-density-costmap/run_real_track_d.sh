#!/usr/bin/env bash
# Track D — Robot THẬT — pipeline đầy đủ (bag + video + CSV + data_config.m)
# Exp.1/2/3: launch riêng real_D_exp*.launch + RViz track_d_real.rviz (không dùng file sim)
#
# Cách dùng:
#   ./tracks/D-density-costmap/run_real_track_d.sh <EXP_NAME> <EXP_VARIANT> [DURATION_MIN] [roslaunch args...]
#
#   EXP_VARIANT: real_tn1 | real_tn2 | real_tn3

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/experiment_pipeline.sh"

PIPE_TRACK="D-density-costmap"
PIPE_MODE="real"
PIPE_READY_TOPIC="/move_base/status"
PIPE_READY_TIMEOUT_SEC="180"
PIPE_WALL_SEGMENTS="$PIPE_WS/environment/D-density-costmap/wall_segments.txt"

_ts=$(date +"%Hh%M_%d-%m-%Y")
PIPE_EXPERIMENT_NAME="${1:-D_real_${_ts}}"
EXP_VARIANT="${2:-real_tn1}"
PIPE_DURATION_MINUTES="${3:-0}"
[[ $# -ge 3 ]] && shift 3 || shift $#
EXTRA_LAUNCH_ARGS=("$@")

pipeline_refresh_paths

NAV_LAUNCH_DIR="$(rospack find x_omni4wd_navigation)/launch/D-density-costmap"
MAP_FILE="$(rospack find x_omni4wd_navigation)/maps/D-density-costmap/trackD_real_map_exp456_v3.yaml"
case "$EXP_VARIANT" in
  real_tn1)
    LAUNCH_FILE="$NAV_LAUNCH_DIR/real_D_exp1_proxemic_only.launch"
    ;;
  real_tn2)
    LAUNCH_FILE="$NAV_LAUNCH_DIR/real_D_exp2_social_layers.launch"
    ;;
  real_tn3)
    LAUNCH_FILE="$NAV_LAUNCH_DIR/real_D_exp3_density_gate.launch"
    ;;
  *)
    echo "❌ EXP_VARIANT phải là: real_tn1 | real_tn2 | real_tn3" >&2
    exit 1
    ;;
esac

PIPE_LAUNCH_KILL_PATTERN="x_omni4wd_navigation_real_track_d.launch|real_D_exp"

PIPE_TOPICS=(
  "/amcl_pose"
  "/odom"
  "/odometry/filtered"
  "/odom_srf"
  "/zed_imu"
  "/scan"
  "/move_base/status"
  "/move_base/GlobalPlanner/plan"
  "/move_base/TebLocalPlannerROS/footprint_trajectory_infeasible"
  "/social_state"
  "/social_state_gt"
  "/density_info"
  "/social_costmap/layer_weights"
  "/social_costmap/layer_weights_json"
  "/person_emotions"
  "/bodies"
  "/objects"
  "/image_emotion"
  "/sii"
  "/sgi"
  "/cmd_vel"
)

# Canonical GT — SII/SGI đo + MATLAB (không phụ thuộc cảm biến)
PIPE_SCENARIO_CANONICAL="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_scenario_canonical_trackD_real.yaml"
PIPE_SCENARIO_MATLAB="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_scenario_matlab_trackD_real.yaml"

pipeline_start_launch() {
  pipeline_log "🚀 Launch Track D real ($EXP_VARIANT) — $LAUNCH_FILE"
  pipeline_log "   Map: $MAP_FILE"
  pipeline_log "   RViz: track_d_real.rviz | SII/SGI ← scenario GT"
  pipeline_roslaunch_bg "$LAUNCH_FILE" \
    map_file:="$MAP_FILE" \
    hardware_respawn:=false \
    use_scenario_gt_for_sii_sgi:=true \
    scenario_gt_path:="$PIPE_SCENARIO_CANONICAL" \
    use_layer_weights_logger:=true \
    layer_weights_csv_path:="$PIPE_CSV_DIR/layer_weights_timeline.csv" \
    "${EXTRA_LAUNCH_ARGS[@]}"
}

# Sau extract: đóng gói social_scenario.yaml (GT) cho MATLAB
eval "$(echo '_pipeline_post_process_orig()'; declare -f pipeline_post_process | tail -n +2)"
pipeline_post_process() {
  _pipeline_post_process_orig || return 1
  local pack="$PIPE_WS/scripts/export_track_d_matlab_pack.py"
  if [ -f "$pack" ]; then
    pipeline_log "📦 MATLAB pack (scenario GT → social_scenario.yaml)..."
    python3 "$pack" "$PIPE_CSV_DIR" \
      --exp-name "$PIPE_EXPERIMENT_NAME" \
      --exp-variant "$EXP_VARIANT" \
      --canonical "$PIPE_SCENARIO_CANONICAL" \
      --matlab-yaml "$PIPE_SCENARIO_MATLAB" \
      --wall "${PIPE_WALL_SEGMENTS:-}" \
      >>"$PIPE_LOG_FILE" 2>&1 || pipeline_log "⚠️ MATLAB pack lỗi (xem log)"
  else
    pipeline_log "⚠️ Không thấy $pack"
  fi
  return 0
}

pipeline_run
