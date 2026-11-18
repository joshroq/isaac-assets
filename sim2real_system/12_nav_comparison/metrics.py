#!/usr/bin/env python3
"""navcmp metrics: per-leg navigation performance and real-vs-sim agreement.

Units and meaning (all positions in the COURSE frame, from the SLAM map pose, so the real
and sim rovers are measured the same way - by their own lidar SLAM):

per leg (one goal of one trial)
  success          1 if Nav2 reported SUCCEEDED
  time_s           goal sent -> result (ROS clock: sim time in Isaac, wall time on the rover)
  path_len_m       length of the driven path
  efficiency       straight-line distance / path_len (1 = straight)
  mean_speed       path_len / time_s  [m/s]
  rotate_frac      share of samples turning in place (|v| < 0.02 m/s, |w| > 0.15 rad/s)
  stop_frac        share of samples standing still (|v| < 0.02 m/s, |w| < 0.05 rad/s)
  min_clear_m      closest lidar return during the leg (rover-to-obstacle clearance proxy)
  final_err_cm     distance from goal when the leg ended
  final_yaw_err_deg
  recoveries       Nav2 recovery behaviours triggered (spin / backup / clear costmap)
  track_v, track_w achieved/commanded speed and turn rate (odometry / cmd_vel, median of
                   samples with a real command): THE drive response the optimizer fitted
  odom_drift_cm    |SLAM displacement - odometry displacement| over the leg (real: wheel
                   slip + odometry error; sim: odometry is Isaac ground truth -> SLAM error)

agreement with the real rover (computed per leg, pooled over legs)
  z                (sim trial value - real mean) / real sd, the real rover's own scatter in
                   the same units -> dimensionless. |z| ~ 1 = a typical real repeat.
                   sd is floored (SD_FLOOR) so 3 near-identical real runs cannot make every
                   difference look huge.
  rms z            sqrt(mean z^2) over the SCORE_METRICS x legs x trials of a condition
  real floor       the same rms z for each real trial against the OTHER real trials
                   (leave-one-out) - what a perfect simulator would score
  path gap         discrete Frechet distance between two paths of the same leg [cm]:
                   median(real_i vs sim_j) / median(real_i vs real_k) = path gap ratio
                   (1 = sim paths differ from real ones no more than real ones differ
                   from each other)
"""
from __future__ import annotations

import csv
import json
import math
import random
from pathlib import Path

SCORE_METRICS = ["time_s", "path_len_m", "mean_speed", "rotate_frac", "stop_frac",
                 "min_clear_m", "final_err_cm", "recoveries"]
DIAG_METRICS = ["efficiency", "final_yaw_err_deg", "track_v", "track_w", "odom_drift_cm"]
SD_FLOOR = {"time_s": 1.0, "path_len_m": 0.03, "mean_speed": 0.01, "rotate_frac": 0.03,
            "stop_frac": 0.03, "min_clear_m": 0.02, "final_err_cm": 2.0, "recoveries": 0.5,
            "efficiency": 0.02, "final_yaw_err_deg": 3.0, "track_v": 0.05, "track_w": 0.05,
            "odom_drift_cm": 2.0}
LABELS = {"success": "success", "time_s": "time [s]", "path_len_m": "path length [m]",
          "efficiency": "path efficiency", "mean_speed": "mean speed [m/s]",
          "rotate_frac": "turning in place [share]", "stop_frac": "standing still [share]",
          "min_clear_m": "min clearance [m]", "final_err_cm": "final position error [cm]",
          "final_yaw_err_deg": "final heading error [deg]", "recoveries": "recoveries",
          "track_v": "achieved/commanded speed", "track_w": "achieved/commanded turn rate",
          "odom_drift_cm": "odometry vs SLAM drift [cm]"}
RESAMPLE_N = 60


def _f(v):
    try:
        x = float(v)
    except (TypeError, ValueError):
        return math.nan
    return x


def _median(xs):
    xs = sorted(x for x in xs if math.isfinite(x))
    if not xs:
        return math.nan
    n = len(xs)
    return xs[n // 2] if n % 2 else 0.5 * (xs[n // 2 - 1] + xs[n // 2])


def _mean(xs):
    xs = [x for x in xs if math.isfinite(x)]
    return sum(xs) / len(xs) if xs else math.nan


def _sd(xs):
    xs = [x for x in xs if math.isfinite(x)]
    if len(xs) < 2:
        return math.nan
    m = sum(xs) / len(xs)
    return math.sqrt(sum((x - m) ** 2 for x in xs) / (len(xs) - 1))


# ── loading ──────────────────────────────────────────────────────────────────
def load_condition(cdir):
    """{'legs': [row dicts], 'traces': {trial: [row dicts]}, 'meta', 'info', 'map'}"""
    cdir = Path(cdir)
    data = cdir / "data"
    out = {"name": cdir.name, "legs": [], "traces": {}, "meta": {}, "info": {}, "map": None,
           "start_scan": None}
    if (cdir / "condition.json").exists():
        out["info"] = json.loads((cdir / "condition.json").read_text())
    if (data / "meta.json").exists():
        out["meta"] = json.loads((data / "meta.json").read_text())
    if (data / "map.json").exists():
        out["map"] = json.loads((data / "map.json").read_text())
    out["start_scan"] = (json.loads((data / "start_scan.json").read_text())
                         if (data / "start_scan.json").exists() else None)
    if (data / "legs.csv").exists():
        with (data / "legs.csv").open() as f:
            out["legs"] = list(csv.DictReader(f))
    for t in sorted(data.glob("trial_*")) if data.exists() else []:
        tr = t / "trace.csv"
        if tr.exists():
            with tr.open() as f:
                out["traces"][int(t.name.split("_")[1])] = list(csv.DictReader(f))
    return out


# ── per-leg metrics ──────────────────────────────────────────────────────────
def leg_path(trace_rows, leg):
    return [(_f(r["t"]), _f(r["x"]), _f(r["y"])) for r in trace_rows if r["leg"] == leg]


def leg_metrics(leg_row, trace_rows):
    leg = leg_row["leg"]
    rows = [r for r in trace_rows if r["leg"] == leg]
    pts = [(_f(r["x"]), _f(r["y"])) for r in rows]
    pts = [p for p in pts if all(map(math.isfinite, p))]
    length = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
    sx, sy = _f(leg_row["start_x"]), _f(leg_row["start_y"])
    gx, gy = _f(leg_row["goal_x"]), _f(leg_row["goal_y"])
    straight = math.hypot(gx - sx, gy - sy)
    t = _f(leg_row["duration_s"])
    v = [_f(r["odom_v"]) for r in rows]
    w = [_f(r["odom_w"]) for r in rows]
    cv = [_f(r["cmd_v"]) for r in rows]
    cw = [_f(r["cmd_w"]) for r in rows]
    n = max(len(rows), 1)
    rot = sum(1 for a, b in zip(v, w) if abs(a) < 0.02 and abs(b) > 0.15) / n
    stop = sum(1 for a, b in zip(v, w) if abs(a) < 0.02 and abs(b) < 0.05) / n
    tv = _median([a / c for a, c in zip(v, cv) if abs(c) > 0.05])
    tw = _median([a / c for a, c in zip(w, cw) if abs(c) > 0.15])
    clear = [_f(r["scan_min"]) for r in rows]
    clear = [c for c in clear if math.isfinite(c)]
    drift = math.nan
    if len(rows) >= 2:
        a, b = rows[0], rows[-1]
        dm = (_f(b["x"]) - _f(a["x"]), _f(b["y"]) - _f(a["y"]))
        do = (_f(b["odom_x"]) - _f(a["odom_x"]), _f(b["odom_y"]) - _f(a["odom_y"]))
        drift = 100.0 * math.hypot(dm[0] - do[0], dm[1] - do[1])
    return {
        "trial": int(leg_row["trial"]), "leg": leg, "leg_index": int(leg_row["leg_index"]),
        "result": leg_row["result"], "success": 1.0 if leg_row["result"] == "SUCCEEDED" else 0.0,
        "time_s": t, "path_len_m": length, "straight_m": straight,
        "efficiency": straight / length if length > 1e-3 else math.nan,
        "mean_speed": length / t if t > 1e-3 else math.nan,
        "rotate_frac": rot, "stop_frac": stop,
        "min_clear_m": min(clear) if clear else math.nan,
        "final_err_cm": 100.0 * _f(leg_row["final_err_m"]),
        "final_yaw_err_deg": _f(leg_row["final_yaw_err_deg"]),
        "recoveries": _f(leg_row["recoveries"]), "track_v": tv, "track_w": tw,
        "odom_drift_cm": drift,
    }


def condition_metrics(cond):
    """Per-leg metrics. STOPPED legs (operator / link / battery stop) are not the rover's
    doing and are dropped; FENCE legs (left the arena) count as failures."""
    out = []
    for row in cond["legs"]:
        if row["result"] == "STOPPED":
            continue
        trace = cond["traces"].get(int(row["trial"]), [])
        out.append(leg_metrics(row, trace))
    return out


# ── path agreement ───────────────────────────────────────────────────────────
def resample(path, n=RESAMPLE_N):
    pts = [(x, y) for _, x, y in path if math.isfinite(x) and math.isfinite(y)]
    if len(pts) < 2:
        return pts
    d = [0.0]
    for a, b in zip(pts, pts[1:]):
        d.append(d[-1] + math.hypot(b[0] - a[0], b[1] - a[1]))
    total = d[-1]
    if total < 1e-6:
        return [pts[0]] * n
    out, j = [], 0
    for i in range(n):
        s = total * i / (n - 1)
        while j < len(d) - 2 and d[j + 1] < s:
            j += 1
        seg = d[j + 1] - d[j]
        u = (s - d[j]) / seg if seg > 1e-9 else 0.0
        out.append((pts[j][0] + u * (pts[j + 1][0] - pts[j][0]),
                    pts[j][1] + u * (pts[j + 1][1] - pts[j][1])))
    return out


def frechet(p, q):
    """Discrete Frechet distance [same units as the points]."""
    if not p or not q:
        return math.nan
    n, m = len(p), len(q)
    prev = [0.0] * m
    for i in range(n):
        cur = [0.0] * m
        for j in range(m):
            d = math.hypot(p[i][0] - q[j][0], p[i][1] - q[j][1])
            if i == 0 and j == 0:
                cur[j] = d
            elif i == 0:
                cur[j] = max(cur[j - 1], d)
            elif j == 0:
                cur[j] = max(prev[j], d)
            else:
                cur[j] = max(min(prev[j], prev[j - 1], cur[j - 1]), d)
        prev = cur
    return prev[-1]


def mean_deviation(p, q):
    """Symmetric mean closest-point distance between two resampled paths."""
    if not p or not q:
        return math.nan

    def one(a, b):
        return sum(min(math.hypot(x - u, y - v) for u, v in b) for x, y in a) / len(a)
    return 0.5 * (one(p, q) + one(q, p))


def paths_by_leg(cond):
    """{leg: {trial: resampled path}} for SUCCEEDED legs only (failed legs end anywhere)."""
    out = {}
    for row in cond["legs"]:
        if row["result"] != "SUCCEEDED":
            continue
        tr = cond["traces"].get(int(row["trial"]), [])
        p = resample(leg_path(tr, row["leg"]))
        if len(p) >= 2:
            out.setdefault(row["leg"], {})[int(row["trial"])] = p
    return out


def path_gaps(real_paths, other_paths, same=False):
    """Frechet + mean deviation [cm] for every (real trial, other trial) pair, per leg."""
    rows = []
    for leg, rp in real_paths.items():
        op = other_paths.get(leg, {})
        for ti, p in rp.items():
            for tj, q in op.items():
                if same and tj <= ti:
                    continue
                rows.append({"leg": leg, "real_trial": ti, "other_trial": tj,
                             "frechet_cm": 100 * frechet(p, q),
                             "mean_dev_cm": 100 * mean_deviation(p, q)})
    return rows


# ── distribution agreement ───────────────────────────────────────────────────
def real_stats(real_rows, metrics):
    """{(leg, metric): (mean, floored sd, n)} from the real rover's legs."""
    st = {}
    legs = sorted({r["leg"] for r in real_rows})
    for leg in legs:
        for m in metrics:
            xs = [r[m] for r in real_rows if r["leg"] == leg and math.isfinite(r[m])]
            sd = _sd(xs)
            sd = max(sd if math.isfinite(sd) else 0.0, SD_FLOOR[m])
            st[(leg, m)] = (_mean(xs), sd, len(xs))
    return st


def z_rows(rows, stats, metrics, exclude_trial=None):
    out = []
    for r in rows:
        for m in metrics:
            key = (r["leg"], m)
            if key not in stats or not math.isfinite(r[m]):
                continue
            mu, sd, n = stats[key]
            if n == 0 or not math.isfinite(mu):
                continue
            out.append({"trial": r["trial"], "leg": r["leg"], "metric": m,
                        "z": (r[m] - mu) / sd})
    return out


def rms(zs):
    zs = [z for z in zs if math.isfinite(z)]
    return math.sqrt(sum(z * z for z in zs) / len(zs)) if zs else math.nan


def real_floor(real_rows, metrics):
    """Leave-one-out rms z of each real trial against the other real trials."""
    trials = sorted({r["trial"] for r in real_rows})
    if len(trials) < 3:
        return math.nan, []
    per = []
    for t in trials:
        rest = [r for r in real_rows if r["trial"] != t]
        mine = [r for r in real_rows if r["trial"] == t]
        zs = [z["z"] for z in z_rows(mine, real_stats(rest, metrics), metrics)]
        per.append(rms(zs))
    return rms_of_trials(per), per


def rms_of_trials(per_trial_rms):
    xs = [x for x in per_trial_rms if math.isfinite(x)]
    return math.sqrt(sum(x * x for x in xs) / len(xs)) if xs else math.nan


def bootstrap_rms(zrows, n=2000, seed=1):
    """95 % CI of rms z, resampling whole TRIALS (legs of one trial are not independent)."""
    by = {}
    for z in zrows:
        by.setdefault(z["trial"], []).append(z["z"])
    trials = list(by)
    if len(trials) < 2:
        return math.nan, math.nan
    rng = random.Random(seed)
    vals = []
    for _ in range(n):
        pick = [rng.choice(trials) for _ in trials]
        vals.append(rms([z for t in pick for z in by[t]]))
    vals.sort()
    return vals[int(0.025 * n)], vals[int(0.975 * n) - 1]


def compare(conds, real="real"):
    """Everything the report needs. conds: {name: load_condition(...)}"""
    rows = {name: condition_metrics(c) for name, c in conds.items()}
    paths = {name: paths_by_leg(c) for name, c in conds.items()}
    res = {"rows": rows, "conditions": {}, "real": real}
    if real not in rows or not rows[real]:
        res["error"] = f"no '{real}' data - nothing to compare against"
        return res
    stats = real_stats(rows[real], SCORE_METRICS)
    floor, floor_per = real_floor(rows[real], SCORE_METRICS)
    rr = path_gaps(paths[real], paths[real], same=True)
    rr_med = _median([g["frechet_cm"] for g in rr])
    res["real_floor_rms_z"] = floor
    res["real_floor_per_trial"] = floor_per
    res["real_vs_real_frechet_cm"] = rr_med
    res["real_stats"] = {f"{k[0]}|{k[1]}": v for k, v in stats.items()}
    for name, r in rows.items():
        trials = sorted({x["trial"] for x in r})
        n_legs = len(r)
        entry = {"trials": len(trials), "legs": n_legs,
                 "success_rate": _mean([x["success"] for x in r]) if r else math.nan,
                 "trial_time_s": _mean([sum(x["time_s"] for x in r if x["trial"] == t) for t in trials]),
                 "recoveries_per_trial": _mean([sum(x["recoveries"] for x in r if x["trial"] == t)
                                                for t in trials])}
        if name != real:
            zr = z_rows(r, stats, SCORE_METRICS)
            entry["rms_z"] = rms([z["z"] for z in zr])
            entry["rms_z_ci"] = bootstrap_rms(zr)
            entry["per_metric_rms_z"] = {m: rms([z["z"] for z in zr if z["metric"] == m])
                                         for m in SCORE_METRICS}
            entry["per_metric_mean_z"] = {m: _mean([z["z"] for z in zr if z["metric"] == m])
                                          for m in SCORE_METRICS}
            gaps = path_gaps(paths[real], paths[name])
            entry["frechet_cm"] = _median([g["frechet_cm"] for g in gaps])
            entry["mean_dev_cm"] = _median([g["mean_dev_cm"] for g in gaps])
            entry["path_gap_ratio"] = (entry["frechet_cm"] / rr_med
                                       if math.isfinite(rr_med) and rr_med > 0 else math.nan)
            entry["path_gaps"] = gaps
        res["conditions"][name] = entry
    # leg x metric table: mean, sd per condition
    table = {}
    for name, r in rows.items():
        for leg in sorted({x["leg"] for x in r}, key=lambda L: min(x["leg_index"] for x in r if x["leg"] == L)):
            for m in ["success"] + SCORE_METRICS + DIAG_METRICS:
                xs = [x[m] for x in r if x["leg"] == leg]
                table.setdefault(leg, {}).setdefault(m, {})[name] = (_mean(xs), _sd(xs), len(xs))
    res["table"] = table
    res["paths"] = paths
    return res


def scan_summary(scan):
    """Beam count, valid share and range limits of one LaserScan dict (start_scan.json)."""
    if not scan:
        return None
    r = scan["ranges"]
    lo, hi = scan["range_min"], scan["range_max"]
    valid = [x for x in r if x is not None and lo <= x <= hi]
    return {"beams": len(r), "valid_share": len(valid) / len(r) if r else math.nan,
            "range_min": lo, "range_max": hi,
            "nearest_m": min(valid) if valid else math.nan,
            "resolution_deg": math.degrees(scan["angle_increment"])}


def scan_points(scan):
    """Valid returns as (x, y) in the lidar frame."""
    lo, hi = scan["range_min"], scan["range_max"]
    a0, da = scan["angle_min"], scan["angle_increment"]
    return [(x * math.cos(a0 + i * da), x * math.sin(a0 + i * da))
            for i, x in enumerate(scan["ranges"]) if x is not None and lo <= x <= hi]
