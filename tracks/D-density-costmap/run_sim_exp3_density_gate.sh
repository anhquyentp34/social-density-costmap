#!/usr/bin/env bash
# Track D — Simulation Exp.3: Social layers + density gate FULL
cd "$(dirname "$0")/../.."
EXP_NAME="D_sim_exp3_density_gate_$(date +%Hh%M_%d-%m-%Y)"
DURATION=0
if [[ "${1:-}" =~ ^[0-9]+$ ]]; then DURATION=$1; shift; fi
./tracks/D-density-costmap/run_sim_track_d.sh "$EXP_NAME" "$DURATION" exp3 "$@"
