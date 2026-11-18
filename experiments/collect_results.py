#!/usr/bin/env python3
"""Gather every cell's results + route figure into one easy-to-access place.

Scans experiments/data/<cell>/ for results.csv + routes.png and produces, under
experiments/data/:
  * master_results.csv   every trial from every cell, one file
  * routes_montage.png   a contact sheet of all cells' route figures
  * INDEX.md             a table: cell, condition, nav, #trials, success%, link to routes.png

    python3 collect_results.py            # scans experiments/data/
"""
import csv
import glob
import os

import numpy as np
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import matplotlib.image as mpimg

HERE = os.path.dirname(os.path.abspath(__file__))
DATA = os.path.join(HERE, "data")


def main():
    cells = sorted(d for d in glob.glob(os.path.join(DATA, "*"))
                   if os.path.isdir(d) and os.path.exists(os.path.join(d, "results.csv")))
    if not cells:
        print("no cells with results.csv under experiments/data/ yet"); return

    # master CSV
    rows, header = [], None
    for d in cells:
        with open(os.path.join(d, "results.csv")) as f:
            r = csv.DictReader(f)
            header = header or r.fieldnames
            rows.extend(list(r))
    with open(os.path.join(DATA, "master_results.csv"), "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=header); w.writeheader(); w.writerows(rows)

    # per-cell summary + INDEX.md
    lines = ["# Experiment results index\n",
             "| cell | condition | nav | trials | success | routes |",
             "|---|---|---|---:|---:|---|"]
    figs = []
    for d in cells:
        name = os.path.basename(d)
        rs = [r for r in rows if f"{r.get('condition')}_{r.get('nav_system')}_{r.get('goal')}" == name] \
             or [r for r in rows if r.get("path_file", "").startswith(("paths/",)) and os.path.basename(d) in d]
        cr = list(csv.DictReader(open(os.path.join(d, "results.csv"))))
        n = len([x for x in cr if x.get("trial") not in ("", "0")])
        succ = sum(x.get("result") == "SUCCEEDED" for x in cr)
        cond = cr[0].get("condition", "?") if cr else "?"
        nav = cr[0].get("nav_system", "?") if cr else "?"
        png = os.path.join(d, "routes.png")
        link = os.path.relpath(png, DATA) if os.path.exists(png) else "—"
        lines.append(f"| {name} | {cond} | {nav} | {n} | {succ}/{n} | {link} |")
        if os.path.exists(png):
            figs.append((name, png))
    with open(os.path.join(DATA, "INDEX.md"), "w") as f:
        f.write("\n".join(lines) + "\n")

    # montage of route figures
    if figs:
        cols = min(3, len(figs)); rowsn = (len(figs) + cols - 1) // cols
        fig, axes = plt.subplots(rowsn, cols, figsize=(6 * cols, 6 * rowsn))
        axes = np.array(axes).reshape(-1)
        for ax in axes:
            ax.axis("off")
        for ax, (name, png) in zip(axes, figs):
            ax.imshow(mpimg.imread(png)); ax.set_title(name, fontsize=10); ax.axis("off")
        fig.tight_layout()
        fig.savefig(os.path.join(DATA, "routes_montage.png"), dpi=120)
        print(f"routes_montage.png ({len(figs)} cells)")
    print(f"wrote master_results.csv ({len(rows)} trials), INDEX.md over {len(cells)} cells")


if __name__ == "__main__":
    main()
