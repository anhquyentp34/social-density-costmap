#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.4 (POS): proxemic only + latch kịch bản canonical
# Camera detect người → inject actors lên costmap, giữ mãi. Không FER.
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp4_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn4 "$DURATION" "$@"
