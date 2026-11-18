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


def settle_time(steps):
    """Leading zero-command duration before the first commanded motion."""
    t = 0.0
    for s in steps:
        if abs(s.get("linear", 0)) < 1e-9 and abs(s.get("angular", 0)) < 1e-9:
            t += s["duration"]
        else:
            break
    return t


def motion_onset(t, x, y, yaw):
    """First sustained motion in the real track (5 consecutive moving samples)."""
    v, w, _ = F._speed_yawrate(t, x, y, yaw)
    moving = (np.abs(v) > 0.03) | (np.abs(w) > 0.12)
    for i in range(len(moving) - 5):
        if moving[i:i + 5].all():
            return float(t[i])
    return float(t[0])


def aligned_offset(t, x, y, yaw, steps):
    """t_offset mapping the real motion onset to the sequence's first commanded motion.

    The real rover is driven over SSH, whose connect + rclpy startup delays the command
    stream (and the motion) by ~4 s relative to the demo clock. The camera captures this
    faithfully, so every feature window would land in the wrong maneuver. Re-timing the
    track to its own motion onset removes that pure delay and makes the real features
    directly comparable to the sequence-aligned sim features.
    """
    return motion_onset(t, x, y, yaw) - settle_time(steps)


def extract_aligned(csv, steps):
    t, x, y, yaw = F.load_csv(csv)
    off = aligned_offset(t, x, y, yaw, steps)
    return F.extract(t, x, y, yaw, steps, t_offset=off), off


def top_speed(csv):
    t, x, y, yaw = F.load_csv(csv)
    v, w, _ = F._speed_yawrate(t, x, y, yaw)
    # robust peak: 95th percentile of |v|,|w| (ignores single-sample camera spikes)
    return float(np.percentile(np.abs(v), 95)), float(np.percentile(np.abs(w), 95))


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--out", required=True)
    ap.add_argument("--test", action="append", default=[],
                    help="NAME:real_csv:sequence.yaml")
    ap.add_argument("--exclude", action="append", default=[],
                    help="NAME:feature to drop")
    ap.add_argument("--combine-mode", default="min")
    ap.add_argument("--top-speed-from", default="T3b_top_speed",
                    help="test NAME whose track sets the measured top LINEAR speed")
    ap.add_argument("--top-rate-from", default="T6_spin_sweep",
                    help="test NAME whose track sets the measured top YAW rate")
    args = ap.parse_args()

    excludes = {}
    for e in args.exclude:
        name, feat = e.split(":")
        excludes.setdefault(name, set()).add(feat)

    doc = {"surface": "live", "combine_mode": args.combine_mode, "tests": {}}
    for spec in args.test:
        name, csv, seq = spec.split(":")
        csv_p = (HERE / csv).resolve() if not Path(csv).is_absolute() else Path(csv)
        seq_p = (HERE / seq).resolve() if not Path(seq).is_absolute() else Path(seq)
        if not csv_p.exists():
            sys.exit(f"missing real csv for {name}: {csv_p}")
        steps = F.load_sequence(str(seq_p))
        feats, off = extract_aligned(str(csv_p), steps)
        agg = F.aggregate([feats])
        for f in excludes.get(name, ()):
            agg.pop(f, None)
        doc["tests"][name] = {"sequence": steps,
                              "features": {k: {"mean": v["mean"], "sigma": v["sigma"]}
                                           for k, v in agg.items()},
                              "real_csv": str(csv_p), "onset_offset_s": round(off, 3)}
        print(f"[{name}] features from {csv_p.name} (motion-onset offset {off:+.2f}s):")
        for k in sorted(agg):
            tag = " (EXCLUDED)" if k in excludes.get(name, ()) else ""
            print(f"    {k:12s} {agg[k]['mean']:+.4f}  (sigma {agg[k]['sigma']:.4f}){tag}")
        if name in (args.top_speed_from, args.top_rate_from):
            vmax, wmax = top_speed(str(csv_p))
            top = doc.setdefault("top_speed", {})
            if name == args.top_speed_from:
                top["linear"] = vmax
                print(f"    top_speed    linear={vmax:.4f} m/s")
            if name == args.top_rate_from:
                top["angular"] = wmax
                print(f"    top_speed    angular={wmax:.4f} rad/s")

    out = (HERE / args.out).resolve() if not Path(args.out).is_absolute() else Path(args.out)
    out.parent.mkdir(parents=True, exist_ok=True)
    out.write_text(yaml.safe_dump(doc, sort_keys=False))
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
