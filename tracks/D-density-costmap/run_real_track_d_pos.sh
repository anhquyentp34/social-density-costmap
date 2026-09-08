#!/usr/bin/env bash
# Track D — Robot THẬT — nhánh POS exp4/exp5/exp6 (*_pos, không FER)
# Map: trackD_real_map_exp456_v3.yaml (cùng exp1–3 + annotation)
#
# Cách dùng:
#   ./tracks/D-density-costmap/run_real_track_d_pos.sh <EXP_NAME> <EXP_VARIANT> [DURATION_MIN] [roslaunch args...]
#   EXP_VARIANT: real_tn4 | real_tn5 | real_tn6

set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
# shellcheck disable=SC1091
source "$SCRIPT_DIR/../lib/experiment_pipeline.sh"

PIPE_TRACK="D-density-costmap"
PIPE_MODE="real"
PIPE_READY_TOPIC="/move_base/status"
PIPE_READY_TIMEOUT_SEC=120

_ts=$(date +"%Hh%M_%d-%m-%Y")
PIPE_EXPERIMENT_NAME="${1:-D_real_pos_${_ts}}"
EXP_VARIANT="${2:-real_tn5}"
PIPE_DURATION_MINUTES="${3:-0}"
[[ $# -ge 3 ]] && shift 3 || shift $#
EXTRA_LAUNCH_ARGS=("$@")

# Refresh paths sau khi biết PIPE_TRACK / PIPE_EXPERIMENT_NAME
PIPE_BAG_DIR="$PIPE_WS/bags/$PIPE_TRACK"
PIPE_VIDEO_DIR="$PIPE_WS/videos/$PIPE_TRACK"
PIPE_RUN_DIR="$PIPE_WS/runs/$PIPE_TRACK"
PIPE_CSV_DIR="$PIPE_WS/csv/$PIPE_TRACK/$PIPE_EXPERIMENT_NAME"
PIPE_ENV_DIR="$PIPE_WS/environment/$PIPE_TRACK"
PIPE_BAG_FILE="$PIPE_BAG_DIR/${PIPE_EXPERIMENT_NAME}.bag"
PIPE_VIDEO_FILE="$PIPE_VIDEO_DIR/${PIPE_EXPERIMENT_NAME}_screen.mp4"
PIPE_LOG_FILE="$PIPE_RUN_DIR/${PIPE_EXPERIMENT_NAME}.log"
PIPE_WALL_SEGMENTS="$PIPE_ENV_DIR/wall_segments.txt"

case "$EXP_VARIANT" in
  real_tn4) SOCIAL_CONFIG="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_config_real_exp4_proxemic_only_pos.yaml" ;;
  real_tn5) SOCIAL_CONFIG="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_config_real_exp5_social_layers_pos.yaml" ;;
  real_tn6) SOCIAL_CONFIG="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_config_real_exp6_density_gate_pos.yaml" ;;
  *)
    echo "❌ EXP_VARIANT phải là: real_tn4 | real_tn5 | real_tn6" >&2
    exit 1
    ;;
esac

MAP_FILE="$(rospack find x_omni4wd_navigation)/maps/D-density-costmap/trackD_real_map_exp456_v3.yaml"
LAUNCH_FILE="$(rospack find x_omni4wd_navigation)/launch/D-density-costmap/x_omni4wd_navigation_real_track_d_pos.launch"
GOAL_SENDER_WAYPOINTS="$(rospack find goal_sender)/config/D-density-costmap/waypoints_trackD_real.yaml"
GOAL_SENDER_NUM_CYCLES=1
PIPE_LAUNCH_KILL_PATTERN="x_omni4wd_navigation_real_track_d_pos.launch"

PIPE_TOPICS=(
  "/amcl_pose"
  "/odom"
  "/odometry/filtered"
  "/odom_srf"
  "/zed_imu"
  "/scan"
  "/scan_static"
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
  "/sii"
  "/sgi"
  "/cmd_vel"
)

PIPE_SCENARIO_CANONICAL="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_scenario_canonical_trackD_real.yaml"
PIPE_SCENARIO_MATLAB="$(rospack find social_density_fusion)/config/D-density-costmap/real/social_scenario_matlab_trackD_real.yaml"

pipeline_start_launch() {
  pipeline_log "🚀 Launch Track D POS ($EXP_VARIANT) — không FER | SII/SGI ← scenario GT"
  pipeline_log "   Social config: $SOCIAL_CONFIG"
  pipeline_log "   Goal sender : $GOAL_SENDER_WAYPOINTS (cycles=$GOAL_SENDER_NUM_CYCLES)"
  pipeline_roslaunch_bg "$LAUNCH_FILE" \
    exp_variant:="$EXP_VARIANT" \
    social_config:="$SOCIAL_CONFIG" \
    map_file:="$MAP_FILE" \
    use_goal_sender:=true \
    goal_sender_waypoints_file:="$GOAL_SENDER_WAYPOINTS" \
    goal_sender_num_cycles:="$GOAL_SENDER_NUM_CYCLES" \
    hardware_respawn:=false \
    use_scenario_gt_for_sii_sgi:=true \
    scenario_gt_path:="$PIPE_SCENARIO_CANONICAL" \
    use_layer_weights_logger:=true \
    layer_weights_csv_path:="$PIPE_CSV_DIR/layer_weights_timeline.csv" \
    "${EXTRA_LAUNCH_ARGS[@]}"
}

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
  fi
  return 0
}

pipeline_run
