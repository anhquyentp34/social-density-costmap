#!/usr/bin/env bash
# NGOÀI LỀ — KHÔNG phải thí nghiệm paper Track D exp1–6.
# Exp.7: đánh dấu vị trí actor (tái dùng stack exp4/tn4, waypoints + spin riêng).
# Không ghi đè waypoints_trackD_real.yaml / costmap mặc định exp1–6.
#
#   ./tracks/D-density-costmap/run_real_exp7_actor_mark_pos.sh [DURATION_MIN]
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp7_mark_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi

WP="$(rospack find goal_sender)/config/D-density-costmap/waypoints_trackD_real_exp7_actors.yaml"
./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn4 "$DURATION" \
  goal_sender_waypoints_file:="$WP" \
  goal_sender_spin_on_arrival:=true \
  goal_sender_spin_angular_vel:=0.7 \
  goal_sender_spin_revolutions:=1.0 \
  goal_sender_goal_reach_xy_tol:=0.50 \
  goal_sender_extra_delay:=5.0 \
  "$@"
