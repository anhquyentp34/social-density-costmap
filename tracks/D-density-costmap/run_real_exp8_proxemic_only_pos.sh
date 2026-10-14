#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.8 (POS, độc lập): proxemic only + latch — fork exp4
# Config riêng: real_tn8 + social_config_real_exp8_* (không đụng exp4).
# Waypoints: START→GOAL_1..3 (waypoints_trackD_real_exp8910.yaml).
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp8_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi

WP="$(rospack find goal_sender)/config/D-density-costmap/waypoints_trackD_real_exp8910.yaml"

./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn8 "$DURATION" \
  goal_sender_waypoints_file:="$WP" \
  initial_pose_x:=12.96 \
  initial_pose_y:=1.32 \
  initial_pose_a:=1.5948 \
  "$@"
