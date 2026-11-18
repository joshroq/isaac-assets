#!/usr/bin/env python3
"""Aggregate trial results into a per-condition summary table for the paper.

Reads one or more results.csv (from run_nav_trial.py) and reports, per
(condition, nav_system, goal): #trials, success rate, and mean±std of duration,
path length, path efficiency, and final position error — the nav-system /
known-map-vs-cold-start comparison table (sim only). Prints it and writes summary.csv.

    python3 summarize.py experiments/data/*/results.csv --out summary.csv
"""
import argparse
import csv
import glob
import math
import statistics as st
from collections import defaultdict

METRICS = ["duration_s", "path_length_m", "path_efficiency", "final_pos_error_m",
           "n_recoveries", "n_restarts"]


def mstd(vals):
    vals = [v for v in vals if v is not None]
    if not vals:
        return (float("nan"), float("nan"))
    return (st.mean(vals), st.pstdev(vals) if len(vals) > 1 else 0.0)


def main():
    ap = argparse.ArgumentParser(description=__doc__)
    ap.add_argument("results", nargs="+", help="results.csv file(s) or globs")
    ap.add_argument("--out", default="summary.csv")
    args = ap.parse_args()

    files = []
    for pat in args.results:
        files.extend(glob.glob(pat) or [pat])
    groups = defaultdict(list)
    for fp in files:
        try:
            for r in csv.DictReader(open(fp)):
                if r.get("result") == "INIT_FAILED":
                    continue
                groups[(r["condition"], r["nav_system"], r["goal"])].append(r)
        except FileNotFoundError:
            print(f"skip missing {fp}")

    header = ["condition", "nav_system", "goal", "n", "success_rate"]
    for m in METRICS:
        header += [f"{m}_mean", f"{m}_std"]
    rows = []
    for key, rs in sorted(groups.items()):
        n = len(rs)
        succ = sum(r["result"] == "SUCCEEDED" for r in rs) / n if n else 0.0
        # metrics computed over SUCCEEDED trials (fair timing/path comparison)
        ok = [r for r in rs if r["result"] == "SUCCEEDED"] or rs
        row = list(key) + [n, round(succ, 3)]
        for m in METRICS:
            mean, std = mstd([float(r[m]) for r in ok if r.get(m) not in (None, "")])
            row += [round(mean, 3) if not math.isnan(mean) else "",
                    round(std, 3) if not math.isnan(std) else ""]
        rows.append(row)

    with open(args.out, "w", newline="") as f:
        w = csv.writer(f); w.writerow(header); w.writerows(rows)

    # pretty print the key columns
    print(f"\n{'condition':11} {'nav':8} {'goal':13} {'n':>3} {'succ':>5} "
          f"{'dur_s':>8} {'len_m':>8} {'eff':>6} {'err_m':>6} {'recov':>6}")
    for r in rows:
        d = dict(zip(header, r))
        print(f"{d['condition']:11} {d['nav_system']:8} {d['goal']:13} {d['n']:>3} "
              f"{d['success_rate']:>5} {str(d['duration_s_mean']):>8} "
              f"{str(d['path_length_m_mean']):>8} {str(d['path_efficiency_mean']):>6} "
              f"{str(d['final_pos_error_m_mean']):>6} {str(d['n_recoveries_mean']):>6}")
    print(f"\nwrote {args.out}")


if __name__ == "__main__":
    main()
