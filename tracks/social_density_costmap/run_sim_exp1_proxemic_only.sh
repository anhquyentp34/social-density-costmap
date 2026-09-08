#!/usr/bin/env bash
# Track D — Simulation Exp.1: Proxemic only (không social layers, không density gate)
cd "$(dirname "$0")/../.."
EXP_NAME="D_sim_exp1_proxemic_only_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_sim_track_d.sh "$EXP_NAME" "$DURATION" exp1 "$@"
