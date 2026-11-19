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


# Join every test's command steps into one sequence with a GAP_S idle step after each;
# returns (all steps, [(test name, start time, end time)]).
def concat(plan):
    """plan: {name: steps}. Returns (steps, [(name, t_start, t_end)])."""
    steps, test_spans, elapsed_s = [], [], 0.0
    for name, test_steps in plan.items():
        test_duration = sum(float(step["duration"]) for step in test_steps)
        test_spans.append((name, elapsed_s, elapsed_s + test_duration))
        steps.extend(dict(step) for step in test_steps)
        steps.append({"linear": 0.0, "angular": 0.0, "duration": GAP_S})
        elapsed_s += test_duration + GAP_S
    return steps, test_spans


# Cut the battery's (t, x, y, yaw) track CSV back into one CSV per test, each re-timed
# so the test starts at t = 0. Returns {test name: csv path}.
def split(csv_path, spans, outdir):
    track_rows = []
    with open(csv_path) as csv_file:
        for row in csv.DictReader(csv_file):
            track_rows.append((float(row["t"]), float(row["x"]), float(row["y"]), float(row["yaw"])))
    test_csvs = {}
    for name, test_start, test_end in spans:
        test_csv = Path(outdir) / f"{name}.csv"
        with open(test_csv, "w", newline="") as csv_file:
            writer = csv.writer(csv_file)
            writer.writerow(("t", "x", "y", "yaw"))
            for time_s, x, y, yaw in track_rows:
                if test_start - 1e-6 <= time_s <= test_end + 1e-6:
                    writer.writerow((f"{time_s - test_start:.4f}", x, y, yaw))
        test_csvs[name] = str(test_csv)
    return test_csvs


# One Isaac launch for the whole battery: concat -> sim_eval.run_trial -> split.
# The timeout allows 240 s of start-up plus 3x the sequence time.
def run_battery(theta, plan, workdir, combine_mode="min", timeout=None):
    """Run every sequence in `plan` under `theta` in one Isaac launch -> {name: csv}."""
    import sim_eval
    workdir = Path(workdir)
    workdir.mkdir(parents=True, exist_ok=True)
    steps, test_spans = concat(plan)
    total_duration = sum(step["duration"] for step in steps)
    timeout = timeout or 240.0 + 3.0 * total_duration      # startup + (possibly slow) sim time
    battery_csv = sim_eval.run_trial(theta, steps, surface="battery", combine_mode=combine_mode,
                              workdir=str(workdir), startup_timeout=timeout)
    return split(battery_csv, test_spans, workdir)
