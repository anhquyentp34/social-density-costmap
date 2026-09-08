#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.2: 4 lớp social, không density gate (+ FER)
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp2_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d.sh "$EXP_NAME" real_tn2 "$DURATION" "$@"
