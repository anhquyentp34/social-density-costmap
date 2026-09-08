#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.3: 4 lớp social + density gate FULL (+ FER)
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp3_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d.sh "$EXP_NAME" real_tn3 "$DURATION" "$@"
