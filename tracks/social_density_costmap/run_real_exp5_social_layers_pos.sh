#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.5 (độc lập): social layers theo vị trí, không density gate, không FER
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp5_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn5 "$DURATION" "$@"
