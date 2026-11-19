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


# Track CSV with columns t, x, y, yaw -> four numpy arrays.
def load_csv(path):
    t, x, y, yaw = [], [], [], []
    with open(path) as csv_file:
        for row in csv.DictReader(csv_file):
            t.append(float(row["t"])); x.append(float(row["x"]))
            y.append(float(row["y"])); yaw.append(float(row["yaw"]))
    return np.array(t), np.array(x), np.array(y), np.array(yaw)


# Command sequence yaml ('steps': linear, angular, duration) -> list of steps.
def load_sequence(path):
    with open(path) as yaml_file:
        text = yaml_file.read()
    try:
        import yaml
        data = yaml.safe_load(text)
    except Exception:
        import json
        data = json.loads(text)
    steps = data["steps"]
    for step in steps:
        step.setdefault("linear", 0.0); step.setdefault("angular", 0.0); step.setdefault("duration", 0.0)
    return steps


# Kind of one command step: 'straight', 'spin', 'arc' or 'idle'.
def _seg_type(step):
    has_linear, has_angular = abs(step["linear"]) > 1e-9, abs(step["angular"]) > 1e-9
    if has_linear and has_angular:
        return "arc"
    if has_linear:
        return "straight"
    if has_angular:
        return "spin"
    return "idle"


# Timeline of the sequence: (kind, start time, end time, step) per step.
def _segments(steps, t0=0.0):
    """Yield (type, t_start, t_end, step) over the sequence timeline (aligned to track t=0)."""
    elapsed = t0
    segments = []
    for step in steps:
        segments.append((_seg_type(step), elapsed, elapsed + step["duration"], step))
        elapsed += step["duration"]
    return segments


# Boolean mask of samples with lo <= t < hi.
def _window(t, lo, hi):
    return (t >= lo) & (t < hi)


# Speed (m/s) and yaw rate (rad/s) per sample by finite differences.
def _speed_yawrate(t, x, y, yaw):
    yaw_unwrapped = np.unwrap(yaw)
    dt_s = np.gradient(t)
    dt_s[dt_s == 0] = 1e-6
    speed = np.hypot(np.gradient(x), np.gradient(y)) / dt_s
    yaw_rate = np.gradient(yaw_unwrapped) / dt_s
    return speed, yaw_rate, yaw_unwrapped


# Every canonical feature (see the module docstring) of one track, measured inside
# the time windows of the sequence's straight / spin / arc / idle steps. Steady-state
# values use the last SS_TAIL fraction of a step (tail()).
def extract(t, x, y, yaw, steps, t_offset=0.0):
    """Return the canonical feature dict for one track + sequence."""
    if len(t) < 4:
        return {}
    speed, yaw_rate, yaw_u = _speed_yawrate(t, x, y, yaw)
    segments = _segments(steps, t_offset)
    feats: dict = {}

    # Path length of the masked samples.
    def arclen(m):
        return float(np.sum(np.hypot(np.diff(x[m]), np.diff(y[m])))) if m.sum() > 1 else 0.0

    # Mask of the last SS_TAIL fraction of [lo, hi) (the whole window if that is empty).
    def tail(lo, hi):
        m = _window(t, hi - SS_TAIL * (hi - lo), hi)
        return m if m.sum() else _window(t, lo, hi)

    # --- latency: first commanded motion edge -> first sample that moves ---
    first_moving_seg = next((s for s in segments if s[0] != "idle"), None)
    if first_moving_seg is not None:
        is_moving = (speed > MOVE_SPEED) | (np.abs(yaw_rate) > MOVE_RATE)
        sample_index = np.nonzero((t >= first_moving_seg[1]) & is_moving)[0]
        if len(sample_index):
            feats["t_lat"] = float(t[sample_index[0]] - first_moving_seg[1])

    # --- straight: every step (per-step speeds) + the longest (ties -> highest command) ---
    straights = [(s[1], s[2], s[3]) for s in segments if s[0] == "straight"]
    for index, (lo, hi, _) in enumerate(straights):
        m = tail(lo, hi)
        if m.sum():
            feats[f"v_ss_s{index}"] = float(np.median(speed[m]))
    if straights:
        straight_speeds = [feats[f"v_ss_s{index}"] for index in range(len(straights)) if f"v_ss_s{index}" in feats]
        if straight_speeds:
            feats["v_ss_max"] = float(max(straight_speeds))
        lo, hi, _ = max(straights, key=lambda s: (round(s[1] - s[0], 6), abs(s[2]["linear"])))
        m = _window(t, lo, hi)
        if m.sum() > 2:
            steady_speed = float(np.median(speed[tail(lo, hi)]))
            feats["v_ss"] = steady_speed
            feats["dist"] = arclen(m)
            # tau_v from MOTION ONSET (removes command latency, which the old definition
            # folded in): onset = first sample above MOVE_SPEED in the step. Only for a step
            # that starts from rest (a speed-sweep step starts already moving).
            previous_segs = [s for s in segments if abs(s[2] - lo) < 1e-9]
            from_rest = not previous_segs or previous_segs[0][0] == "idle"
            seg_times, seg_speeds = t[m], speed[m]
            onset_index = np.argmax(seg_speeds > MOVE_SPEED) if np.any(seg_speeds > MOVE_SPEED) else 0
            reach_index = np.argmax(seg_speeds >= 0.632 * steady_speed) if np.any(seg_speeds >= 0.632 * steady_speed) else len(seg_speeds) - 1
            if from_rest:
                feats["tau_v"] = float(max(seg_times[reach_index] - seg_times[onset_index], 1e-3))

    # --- coast: first idle segment that follows a straight/arc, from DECELERATION ONSET ---
    for index, s in enumerate(segments):
        if s[0] == "idle" and index > 0 and segments[index - 1][0] in ("straight", "arc"):
            lo, hi = s[1], s[2]
            m = _window(t, lo, hi)
            before_coast = _window(t, segments[index - 1][1], lo)
            if m.sum() > 2 and before_coast.sum():
                seg_times, seg_speeds = t[m], speed[m]
                speed_before = float(np.median(speed[before_coast][-max(1, before_coast.sum() // 5):]))
                decel_index = np.argmax(seg_speeds < 0.9 * speed_before) if np.any(seg_speeds < 0.9 * speed_before) else 0
                stop_index = decel_index + (np.argmax(seg_speeds[decel_index:] <= STOP_SPEED) if np.any(seg_speeds[decel_index:] <= STOP_SPEED)
                              else len(seg_speeds) - 1 - decel_index)
                feats["coast_time"] = float(max(seg_times[stop_index] - seg_times[decel_index], 0.0))
                feats["coast_dist"] = arclen(_window(t, seg_times[decel_index], seg_times[stop_index] + 1e-9))
            break

    # --- spin: every step (mean rate over the tail) + the longest ---
    spins = [(s[1], s[2]) for s in segments if s[0] == "spin"]
    for index, (lo, hi) in enumerate(spins):
        m = tail(lo, hi)
        if m.sum() > 1:
            seg_times = t[m]
            feats[f"omega_s{index}"] = float(abs(yaw_u[m][-1] - yaw_u[m][0]) / max(seg_times[-1] - seg_times[0], 1e-6))
    if spins:
        spin_rates = [feats[f"omega_s{index}"] for index in range(len(spins)) if f"omega_s{index}" in feats]
        if spin_rates:
            feats["omega_max"] = float(max(spin_rates))
        lo, hi = max(spins, key=lambda ab: ab[1] - ab[0])
        m = _window(t, lo, hi)
        if m.sum() > 2:
            steady_yaw_rate = float(np.median(np.abs(yaw_rate[tail(lo, hi)])))
            feats["omega_ss"] = steady_yaw_rate
            feats["dyaw"] = float(abs(yaw_u[m][-1] - yaw_u[m][0]))
            feats["omega_mean"] = feats["dyaw"] / max(hi - lo, 1e-6)
            seg_times, seg_yaw_rates = t[m], np.abs(yaw_rate[m])
            rise_threshold = 0.632 * steady_yaw_rate
            sample_index = np.argmax(seg_yaw_rates >= rise_threshold) if np.any(seg_yaw_rates >= rise_threshold) else len(seg_yaw_rates) - 1
            feats["tau_omega"] = float(max(seg_times[sample_index] - seg_times[0], 1e-3))
            # turn-walk: how far the body centre translated during the spin
            feats["turn_walk"] = float(np.hypot(x[m][-1] - x[m][0], y[m][-1] - y[m][0]))

    # --- spin coast: first idle segment after a spin ---
    for index, s in enumerate(segments):
        if s[0] == "idle" and index > 0 and segments[index - 1][0] == "spin":
            m = _window(t, s[1], s[2])
            if m.sum() > 2:
                seg_times, seg_yaw_rates = t[m], np.abs(yaw_rate[m])
                stop_index = np.argmax(seg_yaw_rates <= STOP_RATE) if np.any(seg_yaw_rates <= STOP_RATE) else len(seg_yaw_rates) - 1
                feats["spin_coast_time"] = float(seg_times[stop_index] - seg_times[0])
                feats["spin_coast_angle"] = float(abs(yaw_u[m][stop_index] - yaw_u[m][0]))
            break

    # --- every non-idle step: signed response (command -> achieved twist) ---
    forward_step = np.cos(yaw_u) * np.gradient(x) + np.sin(yaw_u) * np.gradient(y)
    dt_s = np.gradient(t)
    dt_s[dt_s == 0] = 1e-6
    v_signed = forward_step / dt_s
    for index, (kind, lo, hi, _) in enumerate(segments):
        if kind == "idle":
            continue
        m = tail(lo, hi)
        if m.sum() > 1:
            seg_times = t[m]
            feats[f"v_k{index}"] = float(np.median(v_signed[m]))
            feats[f"w_k{index}"] = float((yaw_u[m][-1] - yaw_u[m][0]) / max(seg_times[-1] - seg_times[0], 1e-6))

    # --- arc: first arc segment ---
    arcs = [(s[1], s[2]) for s in segments if s[0] == "arc"]
    if arcs:
        lo, hi = arcs[0]
        m = _window(t, lo, hi)
        if m.sum() > 2:
            seg_speeds, seg_yaw_rates = np.median(speed[m]), np.median(np.abs(yaw_rate[m]))
            feats["arc_radius"] = float(seg_speeds / seg_yaw_rates) if seg_yaw_rates > 1e-6 else float("inf")
    return feats


# Across repeats: mean, sample sigma (floored at 2 % of the mean / an absolute floor,
# so the loss can't blow up on a lucky low spread) and count, per feature.
def aggregate(feature_dicts):
    """{feature: {mean, sigma}} across repeated trials; sigma floored so the loss is stable."""
    feature_names = set().union(*[set(trial_features) for trial_features in feature_dicts]) if feature_dicts else set()
    stats = {}
    small_counts = []
    for feature in feature_names:
        values = np.array([trial_features[feature] for trial_features in feature_dicts if feature in trial_features and np.isfinite(trial_features[feature])])
        if len(values) == 0:
            continue
        mean = float(np.mean(values))
        sigma = float(np.std(values, ddof=1)) if len(values) > 1 else 0.0
        sigma_floor = max(0.02 * abs(mean), ABS_SIGMA_FLOOR.get(feature.split("_k")[0] + "_k"
                                                          if "_k" in feature else feature, 1e-3))
        if len(values) < MIN_REPEATS:
            small_counts.append(len(values))
        stats[feature] = {"mean": mean, "sigma": max(sigma, sigma_floor), "n": int(len(values))}
    if small_counts:                                   # one line, not one per feature
        print(f"[features] note: {len(small_counts)} feature(s) from only {min(small_counts)}-{max(small_counts)} trial(s) "
              f"(< {MIN_REPEATS}): sigma = measured spread, floored at max(2 %, absolute "
              f"floor), so it is a rough estimate", file=sys.stderr)
    return stats


# Command line: print one track's features, or aggregate several (mean + sigma).
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("csv", nargs="*", help="track CSV(s)")
    parser.add_argument("--sequence", required=True)
    parser.add_argument("--aggregate", nargs="+", help="CSVs to aggregate to mean+sigma")
    parser.add_argument("--offset", type=float, default=0.0, help="track t for sequence t=0")
    args = parser.parse_args()
    steps = load_sequence(args.sequence)
    if args.aggregate:
        trial_features = [extract(*load_csv(csv_path), steps, args.offset) for csv_path in args.aggregate]
        aggregated = aggregate(trial_features)
        try:
            import yaml
            print(yaml.safe_dump(aggregated, sort_keys=True))
        except Exception:
            print(aggregated)
        return 0
    for csv_path in args.csv:
        features = extract(*load_csv(csv_path), steps, args.offset)
        print(csv_path)
        for feature in sorted(features):
            print(f"  {feature:12s} {features[feature]:.4f}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
