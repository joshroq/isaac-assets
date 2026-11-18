#!/usr/bin/env python3
"""Compare a real vs sim trajectory (CSV: t,x,y,yaw) and report the gap.

Each path is expressed relative to its own start pose (translate to origin, rotate
so initial heading = +X), so shape/drift are compared regardless of where each run
started or which way it faced. Then both are resampled onto a common time base for
pointwise error.

    python3 compare_paths.py real_run.csv sim_run.csv --out compare.png

Prints: endpoint error, mean/max pointwise error, final-heading error, path lengths.
Saves an overlay plot (+ error-over-time) to --out.
"""
import argparse
import csv
import math
import sys

import numpy as np


def load(path):
    t, x, y, yaw = [], [], [], []
    with open(path) as f:
        for row in csv.DictReader(f):
            t.append(float(row["t"])); x.append(float(row["x"]))
            y.append(float(row["y"])); yaw.append(float(row["yaw"]))
    if len(t) < 2:
        raise SystemExit(f"{path}: need >=2 samples, got {len(t)}")
    return (np.array(t), np.array(x), np.array(y), np.array(yaw))


def to_start_frame(t, x, y, yaw):
    """Translate so first sample is origin, rotate so initial yaw -> +X."""
    x0, y0, a0 = x[0], y[0], yaw[0]
    c, s = math.cos(-a0), math.sin(-a0)
    rx = c * (x - x0) - s * (y - y0)
    ry = s * (x - x0) + c * (y - y0)
    ryaw = yaw - a0
    return rx, ry, ryaw


def path_len(x, y):
    return float(np.sum(np.hypot(np.diff(x), np.diff(y))))


def resample(t, v, tgrid):
    return np.interp(tgrid, t, v)


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("real_csv")
    ap.add_argument("sim_csv")
    ap.add_argument("--out", default="compare.png")
    ap.add_argument("--json", default=None, help="also write metrics as JSON here")
    ap.add_argument("--label", default=None, help="optional tag stored in the JSON")
    args = ap.parse_args()

    tr, xr, yr, ar = load(args.real_csv)
    ts, xs, ys, as_ = load(args.sim_csv)
    xr, yr, ar = to_start_frame(tr, xr, yr, ar)
    xs, ys, as_ = to_start_frame(ts, xs, ys, as_)

    # common time base over the overlapping duration
    tmax = min(tr[-1], ts[-1])
    tg = np.linspace(0, tmax, max(50, int(tmax * 10)))
    rxg, ryg = resample(tr, xr, tg), resample(tr, yr, tg)
    sxg, syg = resample(ts, xs, tg), resample(ts, ys, tg)
    err = np.hypot(rxg - sxg, ryg - syg)

    endpoint_err = float(np.hypot(xr[-1] - xs[-1], yr[-1] - ys[-1]))
    heading_err = math.degrees(abs(((ar[-1] - as_[-1]) + math.pi) % (2*math.pi) - math.pi))

    print(f"samples: real={len(tr)} sim={len(ts)}  overlap={tmax:.1f}s")
    print(f"path length:   real={path_len(xr,yr):.3f} m   sim={path_len(xs,ys):.3f} m")
    print(f"endpoint error:        {endpoint_err*100:.1f} cm")
    print(f"mean pointwise error:  {err.mean()*100:.1f} cm")
    print(f"max  pointwise error:  {err.max()*100:.1f} cm")
    print(f"final heading error:   {heading_err:.1f} deg")

    if args.json:
        import json
        metrics = {
            "label": args.label,
            "real_csv": args.real_csv, "sim_csv": args.sim_csv,
            "n_real": len(tr), "n_sim": len(ts), "overlap_s": round(float(tmax), 3),
            "path_len_real_m": round(path_len(xr, yr), 4),
            "path_len_sim_m": round(path_len(xs, ys), 4),
            "endpoint_err_m": round(endpoint_err, 4),
            "mean_err_m": round(float(err.mean()), 4),
            "max_err_m": round(float(err.max()), 4),
            "heading_err_deg": round(heading_err, 2),
        }
        with open(args.json, "w") as jf:
            json.dump(metrics, jf, indent=2)
        print(f"wrote {args.json}")

    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        fig, (axp, axe) = plt.subplots(1, 2, figsize=(12, 5))
        axp.plot(xr, yr, "-", color="#d62728", label="real (camera)")
        axp.plot(xs, ys, "-", color="#2ca02c", label="sim (/odom)")
        axp.plot(0, 0, "ko", ms=5); axp.set_aspect("equal", "box")
        axp.set_title("paths (start-aligned)"); axp.set_xlabel("x [m]"); axp.set_ylabel("y [m]")
        axp.legend(); axp.grid(True, alpha=0.3)
        axe.plot(tg, err * 100, color="#1f77b4")
        axe.set_title("pointwise error"); axe.set_xlabel("t [s]"); axe.set_ylabel("error [cm]")
        axe.grid(True, alpha=0.3)
        fig.tight_layout(); fig.savefig(args.out, dpi=120)
        print(f"wrote {args.out}")
    except Exception as e:  # matplotlib missing / headless issue - metrics still printed
        print(f"(plot skipped: {e})", file=sys.stderr)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
