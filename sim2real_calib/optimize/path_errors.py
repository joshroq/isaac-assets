#!/usr/bin/env python3
"""Trajectory-path errors in PHYSICAL units (cm, deg), sim vs real, per test.

The optimiser's score (rms z, dist_score.py) does not compare paths point by point. It
compares per-step speed and turn rate, divided by the real rover's scatter, so it has no
units. This tool answers the direct question "how far apart are the paths?":

  1. Every track (t, x, y, yaw) is put in its own start frame: the start pose (mean of the
     first 5 samples) moves to the origin and its heading is rotated to +x
     (trials/compare_paths.py does the same with the first sample).
  2. Real and sim tracks are resampled on a common time grid. Real CSVs are already
     re-timed to the rover's motion onset (closed_loop.real_trial), and sim time starts
     at the first command.
  3. For each (sim trial, real trial) pair:
       endpoint error   |p_real(T) - p_sim(T)|                          [cm]
       mean error       mean over t of |p_real(t) - p_sim(t)|           [cm]
       heading error    |wrap(yaw_real(T) - yaw_sim(T))|                [deg]
     T = the end of the overlapping time.
  4. Report the median over all pairs. Real-vs-real pairs (every two real trials of the
     same test) give the floor in the same units: how far the real rover's own repeats
     land from each other.

    python3 optimize/path_errors.py <real dir> --sim NAME=DIR [--sim NAME=DIR ...] [--out x.json]

<real dir> holds <test>/rep*.csv; each sim DIR holds <test>.csv (+ <test>__repK.csv).
"""
import argparse
import glob
import itertools
import json
import math
import sys
from pathlib import Path

import numpy as np

TESTS = ["R1_straight_fb", "R2_spin_rates", "R3_arcs_slow", "R4_arcs_fast", "R5_nav_pattern",
         "T1_straight", "T2_spin", "T4_arc"]


def load(path):
    a = np.genfromtxt(path, delimiter=",", names=True)
    return a["t"], a["x"], a["y"], np.unwrap(a["yaw"])


def start_frame(t, x, y, yaw, n=5):
    """Origin = mean start position, +x = mean start heading."""
    n = min(n, len(t))
    x0, y0 = x[:n].mean(), y[:n].mean()
    a0 = math.atan2(np.sin(yaw[:n]).mean(), np.cos(yaw[:n]).mean())
    c, s = math.cos(-a0), math.sin(-a0)
    return t, c * (x - x0) - s * (y - y0), s * (x - x0) + c * (y - y0), yaw - a0


def pair_errors(a, b):
    """Two start-framed tracks -> (endpoint cm, mean cm, heading deg) over their overlap."""
    ta, xa, ya, ha = a
    tb, xb, yb, hb = b
    t1 = min(ta[-1], tb[-1])
    tg = np.linspace(0.0, t1, max(50, int(t1 * 10)))
    dx = np.interp(tg, ta, xa) - np.interp(tg, tb, xb)
    dy = np.interp(tg, ta, ya) - np.interp(tg, tb, yb)
    d = np.hypot(dx, dy)
    dh = np.interp(t1, ta, ha) - np.interp(t1, tb, hb)
    dh = abs(math.degrees(math.atan2(math.sin(dh), math.cos(dh))))
    return 100 * d[-1], 100 * d.mean(), dh


def summarise(pairs):
    e = np.array(pairs)
    return {"endpoint_cm": float(np.median(e[:, 0])), "mean_cm": float(np.median(e[:, 1])),
            "heading_deg": float(np.median(e[:, 2])), "pairs": len(e)}


def tracks_real(d, test):
    return [start_frame(*load(c)) for c in sorted(glob.glob(f"{d}/{test}/rep[0-9].csv"))]


def tracks_sim(d, test):
    cs = sorted(glob.glob(f"{d}/{test}.csv") + glob.glob(f"{d}/{test}__rep*.csv"))
    return [start_frame(*load(c)) for c in cs]


def main():
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real")
    ap.add_argument("--sim", action="append", default=[], help="NAME=DIR")
    ap.add_argument("--out")
    a = ap.parse_args()
    sims = dict(s.split("=", 1) for s in a.sim)
    res = {}
    for test in TESTS:
        real = tracks_real(a.real, test)
        if len(real) < 2:
            continue
        row = {"real vs real": summarise([pair_errors(p, q) for p, q in itertools.combinations(real, 2)])}
        for name, d in sims.items():
            st = tracks_sim(d, test)
            if st:
                row[name] = summarise([pair_errors(r, s) for r in real for s in st])
        res[test] = row
    names = ["real vs real", *sims]
    for metric, unit in (("endpoint_cm", "cm"), ("mean_cm", "cm"), ("heading_deg", "deg")):
        print(f"\n{metric.replace('_cm', '').replace('_deg', '')} error [{unit}], median over pairs")
        print(f"{'test':16s}" + "".join(f"{n[:22]:>24s}" for n in names))
        for test, row in res.items():
            print(f"{test:16s}" + "".join(f"{row[n][metric]:24.1f}" if n in row else f"{'-':>24s}"
                                          for n in names))
    if a.out:
        Path(a.out).write_text(json.dumps(res, indent=2))
        print(f"\nwrote {a.out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
