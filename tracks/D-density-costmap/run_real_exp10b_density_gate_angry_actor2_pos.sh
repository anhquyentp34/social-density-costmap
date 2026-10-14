#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.10B (POS, độc lập): như exp10 + actor2 Angry
# Config: real_tn10b + social_config_real_exp10b_* — không đụng exp10.
# Density ON từ đầu (giống exp10).
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp10b_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi

WP="$(rospack find goal_sender)/config/D-density-costmap/waypoints_trackD_real_exp8910.yaml"

./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn10b "$DURATION" \
  goal_sender_waypoints_file:="$WP" \
  initial_pose_x:=12.96 \
  initial_pose_y:=1.32 \
  initial_pose_a:=1.5948 \
  "$@"
