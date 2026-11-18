#!/usr/bin/env python3
"""Extract per-maneuver motion features from a pose-vs-time track.

Works identically on (a) the REAL rover's camera/ArUco track, (b) the SIM /odom track, and
(c) the analytic forward model's output — so sim and real features are computed the SAME way
and are directly comparable. A track is (t, x, y, yaw); a `sequence` (the cmd_sequence YAML
steps) tells us which time windows are straight / spin / coast / arc so each feature comes
from the right maneuver.

Canonical features (SI units; None if the sequence lacks that maneuver):
  v_ss        steady forward speed on the longest straight (ties -> highest command) [m/s]
  v_ss_s<i>   steady speed on the i-th straight step; v_ss_max = max over steps      [m/s]
  dist        path length over that straight                                       [m]
  t_lat       command-to-motion latency: first command edge -> first motion          [s]
  tau_v       time from MOTION ONSET to 63% of v_ss (latency removed)              [s]
  coast_dist  distance from DECELERATION ONSET after cmd->0 until ~stopped         [m]
  coast_time  time from deceleration onset to ~stopped (latency removed)           [s]
  omega_ss    median |yaw rate| over the tail of the longest spin                  [rad/s]
  omega_mean  mean yaw rate over the longest spin, dyaw / duration (robust to the
              sim's stick-slip judder, which makes omega_ss overstate rotation)    [rad/s]
  omega_s<i>  mean yaw rate over the tail of the i-th spin step; omega_max = max   [rad/s]
  dyaw        total heading change over that spin                                  [rad]
  tau_omega   yaw accel time-constant                                              [s]
  turn_walk   centroid translation during the spin (skid walk)                     [m]
  spin_coast_angle / spin_coast_time  heading change / time after a spin -> 0      [rad]/[s]
  arc_radius  v/omega on the first arc segment                                     [m]
  v_k<i>      SIGNED body-frame forward speed over the tail of sequence step i (every
              non-idle step; i = index in the sequence, so backward steps are < 0)  [m/s]
  w_k<i>      SIGNED yaw rate (net heading change / time) over the tail of step i   [rad/s]
              -> together the command->response map over the Nav2/teleop envelope

CLI:
  python3 features.py track.csv --sequence seq.yaml                 # print features
  python3 features.py --aggregate a.csv b.csv ... --sequence seq.yaml  # mean+sigma to yaml
"""
from __future__ import annotations

import argparse
import csv
import math
import sys

import numpy as np

STOP_SPEED = 0.02  # m/s, "stopped" threshold for coast
STOP_RATE = 0.10   # rad/s, "stopped turning" threshold for spin coast
MOVE_SPEED = 0.02  # m/s, motion-onset threshold (latency)
MOVE_RATE = 0.10   # rad/s
SS_TAIL = 0.4      # fraction of a segment's tail used for steady-state means
# Absolute sigma floors for SIGNED per-step features, which can legitimately be ~0 (yaw
# rate of a straight step): a relative floor would make those terms explode.
ABS_SIGMA_FLOOR = {"v_k": 0.005, "w_k": 0.02}
MIN_REPEATS = 5    # repeats per test before sigma counts as measured


def load_csv(path):
    t, x, y, yaw = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); x.append(float(row["x"]))
            y.append(float(row["y"])); yaw.append(float(row["yaw"]))
    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def load_sequence(path):
    with open(path) as f:
        text = f.read()
    try:
        import yaml
        data = yaml.safe_load(text)
    except Exception:
        import json
        data = json.loads(text)
    steps = data["steps"]
    for s in steps:
        s.setdefault("linear", 0.0); s.setdefault("angular", 0.0); s.setdefault("duration", 0.0)
    return steps


def _seg_type(step):
    lin, ang = abs(step["linear"]) > 1e-9, abs(step["angular"]) > 1e-9
    if lin and ang:
        return "arc"
    if lin:
        return "straight"
    if ang:
        return "spin"
    return "idle"


def _segments(steps, t0=0.0):
    """Yield (type, t_start, t_end, step) over the sequence timeline (aligned to track t=0)."""
    t = t0
    out = []
    for s in steps:
        out.append((_seg_type(s), t, t + s["duration"], s))
        t += s["duration"]
    return out


def _window(t, lo, hi):
    return (t >= lo) & (t < hi)


def _speed_yawrate(t, x, y, yaw):
    yaw_u = np.unwrap(yaw)
    dt = np.gradient(t)
    dt[dt == 0] = 1e-6
    v = np.hypot(np.gradient(x), np.gradient(y)) / dt
    w = np.gradient(yaw_u) / dt
    return v, w, yaw_u


def extract(t, x, y, yaw, steps, t_offset=0.0):
    """Return the canonical feature dict for one track + sequence."""
    if len(t) < 4:
        return {}
    v, w, yaw_u = _speed_yawrate(t, x, y, yaw)
    segs = _segments(steps, t_offset)
    feats: dict = {}

    def arclen(m):
        return float(np.sum(np.hypot(np.diff(x[m]), np.diff(y[m])))) if m.sum() > 1 else 0.0

    def tail(lo, hi):
        m = _window(t, hi - SS_TAIL * (hi - lo), hi)
        return m if m.sum() else _window(t, lo, hi)

    # --- latency: first commanded motion edge -> first sample that moves ---
    first = next((s for s in segs if s[0] != "idle"), None)
    if first is not None:
        moving = (v > MOVE_SPEED) | (np.abs(w) > MOVE_RATE)
        idx = np.nonzero((t >= first[1]) & moving)[0]
        if len(idx):
            feats["t_lat"] = float(t[idx[0]] - first[1])

    # --- straight: every step (per-step speeds) + the longest (ties -> highest command) ---
    straights = [(s[1], s[2], s[3]) for s in segs if s[0] == "straight"]
    for i, (lo, hi, _) in enumerate(straights):
        m = tail(lo, hi)
        if m.sum():
            feats[f"v_ss_s{i}"] = float(np.median(v[m]))
    if straights:
        step_v = [feats[f"v_ss_s{i}"] for i in range(len(straights)) if f"v_ss_s{i}" in feats]
        if step_v:
            feats["v_ss_max"] = float(max(step_v))
        lo, hi, _ = max(straights, key=lambda s: (round(s[1] - s[0], 6), abs(s[2]["linear"])))
        m = _window(t, lo, hi)
        if m.sum() > 2:
            vss = float(np.median(v[tail(lo, hi)]))
            feats["v_ss"] = vss
            feats["dist"] = arclen(m)
            # tau_v from MOTION ONSET (removes command latency, which the old definition
            # folded in): onset = first sample above MOVE_SPEED in the step. Only for a step
            # that starts from rest (a speed-sweep step starts already moving).
            prev = [s for s in segs if abs(s[2] - lo) < 1e-9]
            from_rest = not prev or prev[0][0] == "idle"
            tt, vv = t[m], v[m]
            on = np.argmax(vv > MOVE_SPEED) if np.any(vv > MOVE_SPEED) else 0
            reach = np.argmax(vv >= 0.632 * vss) if np.any(vv >= 0.632 * vss) else len(vv) - 1
            if from_rest:
                feats["tau_v"] = float(max(tt[reach] - tt[on], 1e-3))

    # --- coast: first idle segment that follows a straight/arc, from DECELERATION ONSET ---
    for i, s in enumerate(segs):
        if s[0] == "idle" and i > 0 and segs[i - 1][0] in ("straight", "arc"):
            lo, hi = s[1], s[2]
            m = _window(t, lo, hi)
            pre = _window(t, segs[i - 1][1], lo)
            if m.sum() > 2 and pre.sum():
                tt, vv = t[m], v[m]
                v0 = float(np.median(v[pre][-max(1, pre.sum() // 5):]))
                dec = np.argmax(vv < 0.9 * v0) if np.any(vv < 0.9 * v0) else 0
                stop = dec + (np.argmax(vv[dec:] <= STOP_SPEED) if np.any(vv[dec:] <= STOP_SPEED)
                              else len(vv) - 1 - dec)
                feats["coast_time"] = float(max(tt[stop] - tt[dec], 0.0))
                feats["coast_dist"] = arclen(_window(t, tt[dec], tt[stop] + 1e-9))
            break

    # --- spin: every step (mean rate over the tail) + the longest ---
    spins = [(s[1], s[2]) for s in segs if s[0] == "spin"]
    for i, (lo, hi) in enumerate(spins):
        m = tail(lo, hi)
        if m.sum() > 1:
            tt = t[m]
            feats[f"omega_s{i}"] = float(abs(yaw_u[m][-1] - yaw_u[m][0]) / max(tt[-1] - tt[0], 1e-6))
    if spins:
        rates = [feats[f"omega_s{i}"] for i in range(len(spins)) if f"omega_s{i}" in feats]
        if rates:
            feats["omega_max"] = float(max(rates))
        lo, hi = max(spins, key=lambda ab: ab[1] - ab[0])
        m = _window(t, lo, hi)
        if m.sum() > 2:
            wss = float(np.median(np.abs(w[tail(lo, hi)])))
            feats["omega_ss"] = wss
            feats["dyaw"] = float(abs(yaw_u[m][-1] - yaw_u[m][0]))
            feats["omega_mean"] = feats["dyaw"] / max(hi - lo, 1e-6)
            tt, ww = t[m], np.abs(w[m])
            thresh = 0.632 * wss
            idx = np.argmax(ww >= thresh) if np.any(ww >= thresh) else len(ww) - 1
            feats["tau_omega"] = float(max(tt[idx] - tt[0], 1e-3))
            # turn-walk: how far the body centre translated during the spin
            feats["turn_walk"] = float(np.hypot(x[m][-1] - x[m][0], y[m][-1] - y[m][0]))

    # --- spin coast: first idle segment after a spin ---
    for i, s in enumerate(segs):
        if s[0] == "idle" and i > 0 and segs[i - 1][0] == "spin":
            m = _window(t, s[1], s[2])
            if m.sum() > 2:
                tt, ww = t[m], np.abs(w[m])
                stop = np.argmax(ww <= STOP_RATE) if np.any(ww <= STOP_RATE) else len(ww) - 1
                feats["spin_coast_time"] = float(tt[stop] - tt[0])
                feats["spin_coast_angle"] = float(abs(yaw_u[m][stop] - yaw_u[m][0]))
            break

    # --- every non-idle step: signed response (command -> achieved twist) ---
    fwd = np.cos(yaw_u) * np.gradient(x) + np.sin(yaw_u) * np.gradient(y)
    dt = np.gradient(t)
    dt[dt == 0] = 1e-6
    v_signed = fwd / dt
    for i, (kind, lo, hi, _) in enumerate(segs):
        if kind == "idle":
            continue
        m = tail(lo, hi)
        if m.sum() > 1:
            tt = t[m]
            feats[f"v_k{i}"] = float(np.median(v_signed[m]))
            feats[f"w_k{i}"] = float((yaw_u[m][-1] - yaw_u[m][0]) / max(tt[-1] - tt[0], 1e-6))

    # --- arc: first arc segment ---
    arcs = [(s[1], s[2]) for s in segs if s[0] == "arc"]
    if arcs:
        lo, hi = arcs[0]
        m = _window(t, lo, hi)
        if m.sum() > 2:
            vv, ww = np.median(v[m]), np.median(np.abs(w[m]))
            feats["arc_radius"] = float(vv / ww) if ww > 1e-6 else float("inf")
    return feats


def aggregate(feature_dicts):
    """{feature: {mean, sigma}} across repeated trials; sigma floored so the loss is stable."""
    keys = set().union(*[set(d) for d in feature_dicts]) if feature_dicts else set()
    out = {}
    few = []
    for k in keys:
        vals = np.array([d[k] for d in feature_dicts if k in d and np.isfinite(d[k])])
        if len(vals) == 0:
            continue
        mean = float(np.mean(vals))
        sigma = float(np.std(vals, ddof=1)) if len(vals) > 1 else 0.0
        floor = max(0.02 * abs(mean), ABS_SIGMA_FLOOR.get(k.split("_k")[0] + "_k"
                                                          if "_k" in k else k, 1e-3))
        if len(vals) < MIN_REPEATS:
            few.append(len(vals))
        out[k] = {"mean": mean, "sigma": max(sigma, floor), "n": int(len(vals))}
    if few:                                   # one line, not one per feature
        print(f"[features] note: {len(few)} feature(s) from only {min(few)}-{max(few)} trial(s) "
              f"(< {MIN_REPEATS}): sigma = measured spread, floored at max(2 %, absolute "
              f"floor), so it is a rough estimate", file=sys.stderr)
    return out


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="*", help="track CSV(s)")
    ap.add_argument("--sequence", required=True)
    ap.add_argument("--aggregate", nargs="+", help="CSVs to aggregate to mean+sigma")
    ap.add_argument("--offset", type=float, default=0.0, help="track t for sequence t=0")
    args = ap.parse_args()
    steps = load_sequence(args.sequence)
    if args.aggregate:
        feats = [extract(*load_csv(c), steps, args.offset) for c in args.aggregate]
        agg = aggregate(feats)
        try:
            import yaml
            print(yaml.safe_dump(agg, sort_keys=True))
        except Exception:
            print(agg)
        return 0
    for c in args.csv:
        f = extract(*load_csv(c), steps, args.offset)
        print(c)
        for k in sorted(f):
            print(f"  {k:12s} {f[k]:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
