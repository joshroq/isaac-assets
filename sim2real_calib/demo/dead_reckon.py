#!/usr/bin/env python3
"""Dead-reckon a /cmd_vel sequence into a nominal (t,x,y,yaw) path.

This is the *commanded* path (perfect, no slip/dynamics) — a stand-in "reference line" for the
demo while real camera path-tracking isn't wired yet. It is NOT the real rover's path; it is
what you asked the rover to do. Swap it for the camera-tracked CSV (record_traj --source camera)
as soon as ArUco tracking is reliable — every downstream tool takes the same t,x,y,yaw CSV.

    python3 dead_reckon.py ../parallel/sequences/square.yaml --out reference.csv
"""
from __future__ import annotations

import argparse
import csv
import math
import os


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


def integrate(steps, dt=0.02, x0=0.0, y0=0.0, yaw0=0.0):
    x, y, yaw, t = x0, y0, yaw0, 0.0
    rows = [(0.0, x, y, yaw)]
    for s in steps:
        n = max(1, int(round(s["duration"] / dt)))
        for _ in range(n):
            yaw += s["angular"] * dt
            x += s["linear"] * math.cos(yaw) * dt
            y += s["linear"] * math.sin(yaw) * dt
            t += dt
            rows.append((t, x, y, yaw))
    return rows


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("sequence")
    ap.add_argument("--out", required=True)
    ap.add_argument("--dt", type=float, default=0.02)
    ap.add_argument("--start", type=float, nargs=3, default=[0.0, 0.0, 0.0],
                    metavar=("X", "Y", "YAW"), help="start pose in the grid frame")
    args = ap.parse_args()
    rows = integrate(load_sequence(args.sequence), args.dt, *args.start)
    with open(args.out, "w") as f:
        w = csv.writer(f); w.writerow(["t", "x", "y", "yaw"])
        for t, x, y, yaw in rows:
            w.writerow([f"{t:.3f}", f"{x:.5f}", f"{y:.5f}", f"{yaw:.5f}"])
    print(f"[dead_reckon] {len(rows)} samples -> {args.out}  "
          f"(end {rows[-1][1]:.3f},{rows[-1][2]:.3f} m, {math.degrees(rows[-1][3]):.1f}°)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
