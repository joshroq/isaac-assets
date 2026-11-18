#!/usr/bin/env python3
"""Evaluate a whole test battery in ONE Isaac launch (wraps optimize/sim_eval.run_trial).

A headless Isaac launch costs ~50 s before the first command; a test sequence is
~10 s. The loop evaluates several tests per parameter set, so it concatenates every
sequence of the battery (with a short idle gap between them), runs that once, and
splits the recorded track back into one CSV per test, re-timed to t=0 at the test's
own start. Features and path comparisons are start-relative, so the carried-over
position between tests does not matter.

    from sim_batch import run_battery
    csvs = run_battery(theta, {"T1_straight": steps1, "T2_spin": steps2}, workdir)
"""
from __future__ import annotations

import csv
import sys
from pathlib import Path

SC = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(SC / "optimize"))

GAP_S = 1.5          # extra zero-command time between tests (sequences already end idle)


def concat(plan):
    """plan: {name: steps}. Returns (steps, [(name, t_start, t_end)])."""
    steps, spans, t = [], [], 0.0
    for name, seq in plan.items():
        dur = sum(float(s["duration"]) for s in seq)
        spans.append((name, t, t + dur))
        steps.extend(dict(s) for s in seq)
        steps.append({"linear": 0.0, "angular": 0.0, "duration": GAP_S})
        t += dur + GAP_S
    return steps, spans


def split(csv_path, spans, outdir):
    rows = []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            rows.append((float(r["t"]), float(r["x"]), float(r["y"]), float(r["yaw"])))
    out = {}
    for name, t0, t1 in spans:
        path = Path(outdir) / f"{name}.csv"
        with open(path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(("t", "x", "y", "yaw"))
            for t, x, y, yaw in rows:
                if t0 - 1e-6 <= t <= t1 + 1e-6:
                    w.writerow((f"{t - t0:.4f}", x, y, yaw))
        out[name] = str(path)
    return out


def run_battery(theta, plan, workdir, combine_mode="min", timeout=None):
    """Run every sequence in `plan` under `theta` in one Isaac launch -> {name: csv}."""
    import sim_eval
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    steps, spans = concat(plan)
    total = sum(s["duration"] for s in steps)
    timeout = timeout or 240.0 + 3.0 * total      # startup + (possibly slow) sim time
    full = sim_eval.run_trial(theta, steps, surface="battery", combine_mode=combine_mode,
                              workdir=str(workdir), startup_timeout=timeout)
    return split(full, spans, workdir)
