#!/usr/bin/env bash
# Drive the native baked Isaac ROS 2 graph and record /ugv_sim/odom.
# Usage: ./trial_sim.sh [sequence.yaml] [out.csv] [calib.yaml]
# SIM_TRIAL_MODE=direct selects the explicit ROS-free physics diagnostic.
set -euo pipefail
HERE="$(cd "$(dirname "$0")" && pwd)"
REPO="$(cd "$HERE/../.." && pwd)"
SEQ="${1:-$HERE/../parallel/sequences/square.yaml}"
OUT="${2:-$HERE/sim_run.csv}"
CALIB="${3:-}"
# Generated ROS setup scripts reference optional unset variables.
set +u
source "$REPO/ros2env.sh"
set -u
ARGS=(--sequence "$SEQ" --out "$OUT" --mode "${SIM_TRIAL_MODE:-ros}")
[ -n "$CALIB" ] && ARGS+=(--calib "$CALIB")
echo "[sim] driving $(basename "$SEQ") in Isaac (${SIM_TRIAL_MODE:-ros}) -> $OUT"
python3 "$REPO/sim2real_calib/optimize/sim_eval.py" "${ARGS[@]}"
echo "[sim] done -> $OUT ($(wc -l < "$OUT") rows)"
