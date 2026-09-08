#!/usr/bin/env bash
# Track D — Robot THẬT — Exp.1: Proxemic only (baseline, không social layers bổ sung)
cd "$(dirname "$0")/../.."
EXP_NAME="D_real_exp1_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_real_track_d.sh "$EXP_NAME" real_tn1 "$DURATION" "$@"
