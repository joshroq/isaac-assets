#!/usr/bin/env bash
# Staged PHYSICS fit on recorded real trials (no rover needed) - every optimisable physics
# key back in play, including the ones frozen for carpet (drive_damping, chassis damping,
# corner/middle wheel friction). Concrete 2026-11-11.
#
# Why staged: one Isaac evaluation of the whole 12-test battery takes ~5 min, and 16 keys at
# once would need hundreds of them. Each stage fits only the keys its tests can identify
# (the grouping of optimize/calibrate.STAGES) on just those tests, then hands its result to
# the next stage (--init). The last stage refines ALL keys jointly on the FULL battery in a
# small trust region and reports before/after on everything.
#
#   bash loop/staged_physics.sh loop/runs/concrete_20261111_140923/iter_01/real [surface]
#   START=C INIT=surfaces/concrete_physics_B_<stamp>.yaml bash loop/staged_physics.sh ...  # resume
# Long (~5 h): run it detached - setsid nohup bash loop/staged_physics.sh ... > log 2>&1 &
#
# combine_mode average: wheel AND floor friction both act (under 'min' whichever is larger
# is dead). Note: per contact only the combined (wheel+floor)/2 is physically identifiable,
# so the individual wheel/floor values trade off - read them together.
set -euo pipefail
REAL="${1:?usage: staged_physics.sh <real dir: .../iter_NN/real> [surface]}"
SURF="${2:-concrete}"
cd "$(dirname "$0")/.."                       # sim2real_calib/
OUT=surfaces
STAMP=$(date +%Y%m%d_%H%M%S)
COMMON=(--surface "$SURF" --real-from "$REAL" --iterations 1 --combine-mode average
        --no-show --no-hold --no-videos --repeats 3)
START="${START:-A}"
run() {   # name keys tests evals trust [init]
  local name=$1 keys=$2 tests=$3 evals=$4 trust=$5 init=${6:-}
  if [[ "$name" < "$START" ]]; then echo "=== stage $name: skipped (START=$START)"; return; fi
  local out="$OUT/${SURF}_physics_${name}_${STAMP}.yaml"
  echo "=== stage $name: keys [$keys] on [$tests], $evals evals -> $out"
  python3 -u loop/closed_loop.py "${COMMON[@]}" --keys "$keys" --tests "$tests" \
      --max-evals "$evals" --trust "$trust" --out "$out" ${init:+--init "$init"}
  LAST="$out"
}
LAST="${INIT:-}"
# A. drivetrain ramps + drive gain: accel/decel onsets, stops, spin-up
run A max_acceleration,max_deceleration,max_angular_acceleration,drive_damping \
    T1_straight,T5_stop,T2_spin 30 0.5
# B. speed clamps: top linear speed, top yaw rate
run B max_linear_speed,max_angular_speed T3b_top_speed,T6_spin_sweep 15 0.5 "$LAST"
# C. traction / turning: the skid-steer contact (the regime concrete changes most)
run C wheel_distance,corner_mu_dynamic,corner_mu_static,middle_mu_dynamic,middle_mu_static,floor_mu_dynamic,floor_mu_static,drive_max_force \
    T1_straight,T3_speed_sweep,T2_spin,T6_spin_sweep,R2_spin_rates 45 0.5 "$LAST"
# D. chassis damping: rolling resistance / yaw drag on coasts and spin stops
run D base_linear_damping,base_angular_damping \
    T1_straight,T3_speed_sweep,T5_stop,T2_spin,T6_spin_sweep 15 0.5 "$LAST"
# E. joint refinement of ALL keys on the FULL battery (small trust region)
run E wheel_distance,max_linear_speed,max_angular_speed,max_acceleration,max_deceleration,max_angular_acceleration,corner_mu_dynamic,corner_mu_static,middle_mu_dynamic,middle_mu_static,floor_mu_dynamic,floor_mu_static,drive_damping,drive_max_force,base_linear_damping,base_angular_damping \
    T1_straight,T2_spin,T3_speed_sweep,T3b_top_speed,T4_arc,T5_stop,T6_spin_sweep,R1_straight_fb,R2_spin_rates,R3_arcs_slow,R4_arcs_fast,R5_nav_pattern 24 0.15 "$LAST"
cp "$LAST" "$OUT/${SURF}_physics.yaml"
echo "=== done: $OUT/${SURF}_physics.yaml (stages: $OUT/${SURF}_physics_[A-E]_${STAMP}.yaml)"
