#!/usr/bin/env python3
"""Plot the recorded navigation trajectories over the map — a figure for the paper.

Reads one or more experiment output dirs (each with results.csv, paths/*.csv), draws every
trajectory as a line over the occupancy map, marks the goals and start, and colours the
paths by a chosen field so you can compare (lidar vs rtab vs …, or known_map vs cold_start).
Sim-only. Prefers the shared ground-truth map (experiments/maps/<env>/map.npz via --map);
falls back to a run's own map.npz. Saves PNG + PDF.

    # compare nav systems on one goal:
    python3 plot_paths.py experiments/data/*_G3_east --group-by nav_system \
        --map experiments/maps/warehouse/map.npz --title "Nav systems, G3"
    # compare the two conditions for one system:
    python3 plot_paths.py experiments/data/known_map_rtab_* experiments/data/cold_start_rtab_* \
        --group-by condition --title "RTAB: known-map vs cold-start"

Needs matplotlib + numpy (pip install matplotlib numpy).
"""
import argparse
import csv
import glob
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt

# colourblind-friendly (Okabe-Ito)
PALETTE = ["#0072B2", "#D55E00", "#009E73", "#CC79A7", "#E69F00", "#56B4E9",
           "#F0E442", "#000000"]


def load_dir(d):
    res = os.path.join(d, "results.csv")
    rows = list(csv.DictReader(open(res))) if os.path.exists(res) else []
    mp = os.path.join(d, "map.npz")
    grid = np.load(mp) if os.path.exists(mp) else None
    return d, rows, grid


def read_path(csv_path):
    xs, ys = [], []
    with open(csv_path) as f:
        for r in csv.DictReader(f):
            xs.append(float(r["x"])); ys.append(float(r["y"]))
    return xs, ys


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("dirs", nargs="+", help="experiment output dir(s) or globs")
    ap.add_argument("--group-by", default="nav_system",
                    choices=["nav_system", "condition", "goal", "trial", "result"],
                    help="colour trajectories by this field")
    ap.add_argument("--map", default=None,
                    help="shared ground-truth map.npz for the background (from make_occupancy_map.py)")
    ap.add_argument("--out", default="experiment_paths.png")
    ap.add_argument("--title", default="Navigation trajectories")
    ap.add_argument("--no-map", action="store_true", help="skip the occupancy background")
    args = ap.parse_args()

    dirs = []
    for pat in args.dirs:
        dirs.extend(sorted(glob.glob(pat)) or ([pat] if os.path.isdir(pat) else []))
    if not dirs:
        raise SystemExit("no experiment dirs matched")

    fig, ax = plt.subplots(figsize=(9, 9))
    bg = None
    goals, groups = {}, {}

    # prefer the shared ground-truth map for the background if given
    if args.map and not args.no_map and os.path.exists(args.map):
        grid = np.load(args.map)
        g = grid["grid"].astype(float)
        img = np.where(g < 0, 0.85, 1.0 - g / 100.0)
        res = float(grid["resolution"]); ox = float(grid["origin_x"]); oy = float(grid["origin_y"])
        ax.imshow(img, cmap="gray", origin="lower",
                  extent=[ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res],
                  vmin=0, vmax=1, zorder=0)
        bg = True

    for d in dirs:
        _, rows, grid = load_dir(d)
        if grid is not None and bg is None and not args.no_map:
            g = grid["grid"].astype(float)
            img = np.where(g < 0, 0.85, 1.0 - g / 100.0)  # unknown grey, free white, occ black
            res = float(grid["resolution"]); ox = float(grid["origin_x"]); oy = float(grid["origin_y"])
            extent = [ox, ox + img.shape[1] * res, oy, oy + img.shape[0] * res]
            ax.imshow(img, cmap="gray", origin="lower", extent=extent, vmin=0, vmax=1, zorder=0)
            bg = True
        for r in rows:
            key = r.get(args.group_by, "?")
            groups.setdefault(key, [])
            pf_field = r.get("path_file") or ""   # empty for INIT_FAILED rows
            pf = pf_field if os.path.isabs(pf_field) else os.path.join(d, pf_field)
            if pf_field and os.path.isfile(pf):
                groups[key].append(read_path(pf))
            goals[r["goal"]] = (float(r["goal_x"]), float(r["goal_y"]))

    # trajectories, coloured by group
    for i, (key, paths) in enumerate(sorted(groups.items())):
        c = PALETTE[i % len(PALETTE)]
        for j, (xs, ys) in enumerate(paths):
            ax.plot(xs, ys, "-", color=c, alpha=0.75, linewidth=1.6,
                    label=str(key) if j == 0 else None, zorder=3)
        # start markers
        for xs, ys in paths:
            if xs:
                ax.plot(xs[0], ys[0], "o", color=c, ms=5, zorder=4)

    # goals
    for name, (gx, gy) in goals.items():
        ax.plot(gx, gy, "*", color="crimson", ms=18, markeredgecolor="k", zorder=5)
        ax.annotate(name, (gx, gy), textcoords="offset points", xytext=(8, 6),
                    fontsize=9, weight="bold", color="crimson")

    ax.set_aspect("equal", "box")
    ax.set_xlabel("x (m)"); ax.set_ylabel("y (m)")
    ax.set_title(args.title)
    ax.grid(True, alpha=0.25)
    ax.legend(title=args.group_by.replace("_", " "), loc="best", framealpha=0.9)
    fig.tight_layout()
    fig.savefig(args.out, dpi=200)
    pdf = os.path.splitext(args.out)[0] + ".pdf"
    fig.savefig(pdf)
    print(f"wrote {args.out} and {pdf}")


if __name__ == "__main__":
    main()
