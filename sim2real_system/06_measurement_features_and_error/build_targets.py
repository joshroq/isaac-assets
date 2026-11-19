#!/usr/bin/env python3
"""Build a staged-optimiser targets.yaml from REAL camera trajectories.

Each --test maps a stage test NAME (the sequence stem, as referenced in calibrate.py STAGES:
T1_straight, T2_spin, T3b_top_speed, T4_arc, T5_stop, T6_spin_sweep) to (real_csv,
sequence_file). Record >= 5 repeats per test and aggregate them (features.aggregate) - a
single trial gets only the 2 % sigma floor. We extract the canonical motion features
from the real rover's camera track and emit {feature: {mean, sigma}} per test, plus the
measured top_speed (max |v|, max |omega|) for the speed-clamp read-off stage.

--exclude <test>:<feature> drops a feature that is known-unreliable for a given maneuver
(e.g. T2_spin:tau_omega if the real breakaway is erratic). Timing features (tau_v, coast_*)
are measured from motion onset, so command latency no longer contaminates them; it is
reported separately as t_lat.

  python3 build_targets.py --out targets/live.yaml --combine-mode min \
    --test T1_straight:../demo/runs/sweep/T1_straight/real.csv:../parallel/sequences/T1_straight.yaml \
    --test T2_spin:../demo/runs/sweep/T2_spin/real.csv:../parallel/sequences/T2_spin.yaml \
    --test T4_arc:../demo/runs/sweep/T4_arc/real.csv:../parallel/sequences/T4_arc.yaml \
    --test T6_spin_sweep:../demo/runs/sweep/T6_spin_sweep/real.csv:../parallel/sequences/T6_spin_sweep.yaml \
    --test T3b_top_speed:../demo/runs/sweep/T3b_top_speed/real.csv:../parallel/sequences/T3b_top_speed.yaml
"""
from __future__ import annotations

import argparse
import sys
from pathlib import Path

import numpy as np
import yaml

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import features as F  # noqa: E402


# Seconds of zero command at the start of a sequence (before the first motion).
def settle_time(steps):
    """Leading zero-command duration before the first commanded motion."""
    idle_s = 0.0
    for step in steps:
        if abs(step.get("linear", 0)) < 1e-9 and abs(step.get("angular", 0)) < 1e-9:
            idle_s += step["duration"]
        else:
            break
    return idle_s


# Time of the first 5-sample run of motion (> 3 cm/s or > 0.12 rad/s) in a track.
def motion_onset(t, x, y, yaw):
    """First sustained motion in the real track (5 consecutive moving samples)."""
    speed, yaw_rate, _ = F._speed_yawrate(t, x, y, yaw)
    is_moving = (np.abs(speed) > 0.03) | (np.abs(yaw_rate) > 0.12)
    for index in range(len(is_moving) - 5):
        if is_moving[index:index + 5].all():
            return float(t[index])
    return float(t[0])


# Time shift that lines the real track's motion onset up with the sequence's first
# commanded motion (see docstring).
def aligned_offset(t, x, y, yaw, steps):
    """t_offset mapping the real motion onset to the sequence's first commanded motion.

    The real rover is driven over SSH, whose connect + rclpy startup delays the command
    stream (and the motion) by ~4 s relative to the demo clock. The camera captures this
    faithfully, so every feature window would land in the wrong maneuver. Re-timing the
    track to its own motion onset removes that pure delay and makes the real features
    directly comparable to the sequence-aligned sim features.
    """
    return motion_onset(t, x, y, yaw) - settle_time(steps)


# Features of a real CSV after onset alignment; returns (features, offset s).
def extract_aligned(csv, steps):
    t, x, y, yaw = F.load_csv(csv)
    offset_s = aligned_offset(t, x, y, yaw, steps)
    return F.extract(t, x, y, yaw, steps, t_offset=offset_s), offset_s


# 95th-percentile |speed| and |yaw rate| of a track (robust 'top speed').
def top_speed(csv):
    t, x, y, yaw = F.load_csv(csv)
    speed, yaw_rate, _ = F._speed_yawrate(t, x, y, yaw)
    # robust peak: 95th percentile of |v|,|w| (ignores single-sample camera spikes)
    return float(np.percentile(np.abs(speed), 95)), float(np.percentile(np.abs(yaw_rate), 95))


# Command line: one targets.yaml from NAME:real_csv:sequence triples (see docstring).
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--out", required=True)
    parser.add_argument("--test", action="append", default=[],
                    help="NAME:real_csv:sequence.yaml")
    parser.add_argument("--exclude", action="append", default=[],
                    help="NAME:feature to drop")
    parser.add_argument("--combine-mode", default="min")
    parser.add_argument("--top-speed-from", default="T3b_top_speed",
                    help="test NAME whose track sets the measured top LINEAR speed")
    parser.add_argument("--top-rate-from", default="T6_spin_sweep",
                    help="test NAME whose track sets the measured top YAW rate")
    args = parser.parse_args()

    excludes = {}
    for exclude_spec in args.exclude:
        name, feature_name = exclude_spec.split(":")
        excludes.setdefault(name, set()).add(feature_name)

    targets_doc = {"surface": "live", "combine_mode": args.combine_mode, "tests": {}}
    for test_spec in args.test:
        name, csv, sequence = test_spec.split(":")
        csv_path = (HERE / csv).resolve() if not Path(csv).is_absolute() else Path(csv)
        sequence_path = (HERE / sequence).resolve() if not Path(sequence).is_absolute() else Path(sequence)
        if not csv_path.exists():
            sys.exit(f"missing real csv for {name}: {csv_path}")
        steps = F.load_sequence(str(sequence_path))
        trial_features, offset_s = extract_aligned(str(csv_path), steps)
        aggregated = F.aggregate([trial_features])
        for excluded_feature in excludes.get(name, ()):
            aggregated.pop(excluded_feature, None)
        targets_doc["tests"][name] = {"sequence": steps,
                              "features": {feature: {"mean": stats["mean"], "sigma": stats["sigma"]}
                                           for feature, stats in aggregated.items()},
                              "real_csv": str(csv_path), "onset_offset_s": round(offset_s, 3)}
        print(f"[{name}] features from {csv_path.name} (motion-onset offset {offset_s:+.2f}s):")
        for feature in sorted(aggregated):
            tag = " (EXCLUDED)" if feature in excludes.get(name, ()) else ""
            print(f"    {feature:12s} {aggregated[feature]['mean']:+.4f}  (sigma {aggregated[feature]['sigma']:.4f}){tag}")
        if name in (args.top_speed_from, args.top_rate_from):
            top_linear, top_angular = top_speed(str(csv_path))
            top_speeds = targets_doc.setdefault("top_speed", {})
            if name == args.top_speed_from:
                top_speeds["linear"] = top_linear
                print(f"    top_speed    linear={top_linear:.4f} m/s")
            if name == args.top_rate_from:
                top_speeds["angular"] = top_angular
                print(f"    top_speed    angular={top_angular:.4f} rad/s")

    out_path = (HERE / args.out).resolve() if not Path(args.out).is_absolute() else Path(args.out)
    out_path.parent.mkdir(parents=True, exist_ok=True)
    out_path.write_text(yaml.safe_dump(targets_doc, sort_keys=False))
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
