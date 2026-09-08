#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.6 (độc lập): social layers + density gate, không FER
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp6_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d_pos.sh "$EXP_NAME" real_tn6 "$DURATION" "$@"
