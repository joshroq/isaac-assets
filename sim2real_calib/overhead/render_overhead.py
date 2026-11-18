#!/usr/bin/env python3
"""Render a rover run as an overhead view that matches the real webcam's viewpoint.

Given a pose track (t,x,y,yaw CSV — e.g. the SIM's `/odom` from trials/record_traj.py) and a
`camera_setup.yaml` (the matched camera pose), draws, frame by frame, from that camera's
viewpoint: the 1×1 m metric grid, the rover footprint + heading, and the path line traced so
far. Writes an mp4 and/or shows a live window. This is the "temporary video of the run" — it is
geometrically the same projection the webcam has, so the sim view lines up with the real view.

It does NOT need Isaac (it draws synthetically from the track). For the photoreal Isaac render
from the same camera, see run_overhead.py.

    python3 render_overhead.py sim_run.csv --out sim_overhead.mp4
    python3 render_overhead.py sim_run.csv --live
    python3 render_overhead.py real_run.csv sim_run.csv --out compare_overhead.mp4   # overlay both
"""
from __future__ import annotations

import argparse
import csv
import math
import os

import cv2
import numpy as np

import camera_model as cm

HERE = os.path.dirname(os.path.abspath(__file__))
SETUP = os.path.join(HERE, "camera_setup.yaml")
ROVER_L, ROVER_W = 0.20, 0.17            # chassis footprint (m): ~corner wheelbase × track


def load_setup(path):
    import yaml
    return yaml.safe_load(open(path))


def load_track(path):
    t, x, y, yaw = [], [], [], []
    with open(path) as f:
        for r in csv.DictReader(f):
            t.append(float(r["t"])); x.append(float(r["x"]))
            y.append(float(r["y"])); yaw.append(float(r["yaw"]))
    return np.array(t), np.array(x), np.array(y), np.array(yaw)


def _poly(img, cam, pts_world, color, th=2, close=False):
    px = cam.project(pts_world)
    ok = np.isfinite(px).all(axis=1)
    pts = px[ok].astype(np.int32)
    if len(pts) >= 2:
        cv2.polylines(img, [pts], close, color, th, cv2.LINE_AA)
    return px


def draw_grid(img, cam, step=0.1, major_m=0.5):
    gw, gh = cam.grid_dims
    for length, other, vertical in ((gw, gh, True), (gh, gw, False)):
        n = int(round(length / step))
        for i in range(n + 1):
            a = min(i * step, length)
            major = i == n or abs(a / major_m - round(a / major_m)) < 1e-6
            col = (0, 220, 255) if major else (60, 160, 90)
            th = 2 if major else 1
            seg = [[a, 0], [a, other]] if vertical else [[0, a], [other, a]]
            _poly(img, cam, cam.grid_origin + np.array(seg), col, th)
    # origin axes
    o = cam.grid_origin
    _poly(img, cam, [o + [0, 0], o + [0.3, 0]], (0, 0, 255), 3)
    _poly(img, cam, [o + [0, 0], o + [0, 0.3]], (0, 255, 0), 3)


def draw_rover(img, cam, x, y, yaw, color):
    c, s = math.cos(yaw), math.sin(yaw)
    # body rectangle corners in world
    hl, hw = ROVER_L / 2, ROVER_W / 2
    local = np.array([[hl, hw], [hl, -hw], [-hl, -hw], [-hl, hw]])
    world = np.array([[x + c * lx - s * ly, y + s * lx + c * ly] for lx, ly in local])
    _poly(img, cam, world, color, 2, close=True)
    # heading arrow
    tip = cam.project([[x + c * (hl + 0.08), y + s * (hl + 0.08)]])[0]
    ctr = cam.project([[x, y]])[0]
    if np.isfinite(tip).all() and np.isfinite(ctr).all():
        cv2.arrowedLine(img, tuple(ctr.astype(int)), tuple(tip.astype(int)),
                        color, 2, tipLength=0.3)


def render(tracks, setup, out=None, live=False, fps=30.0, labels=None, bg=25):
    cam = cm.OverheadCamera(setup)
    W, H = cam.res
    colors = [(0, 220, 0), (0, 0, 255), (255, 180, 0)]
    labels = labels or [f"track{i}" for i in range(len(tracks))]
    tmax = max(tr[0][-1] for tr in tracks)
    writer = None
    if out:
        writer = cv2.VideoWriter(out, cv2.VideoWriter_fourcc(*"mp4v"), fps, (W, H))
    n = int(tmax * fps) + 1
    for k in range(n):
        tf = k / fps
        img = np.full((H, W, 3), bg, np.uint8)
        draw_grid(img, cam)
        for i, (t, x, y, yaw) in enumerate(tracks):
            m = t <= tf
            if m.sum() < 1:
                continue
            col = colors[i % len(colors)]
            if m.sum() >= 2:                                  # path so far
                _poly(img, cam, np.column_stack([x[m], y[m]]), col, 2)
            draw_rover(img, cam, x[m][-1], y[m][-1], yaw[m][-1], col)
            cv2.putText(img, f"{labels[i]}  ({x[m][-1]:.2f},{y[m][-1]:.2f})m",
                        (12, 26 + 24 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 2, cv2.LINE_AA)
        cv2.putText(img, f"t={tf:4.1f}s  overhead view (matched to webcam)",
                    (12, H - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.55, (230, 230, 230), 1, cv2.LINE_AA)
        if writer:
            writer.write(img)
        if live:
            cv2.imshow("overhead (sim)  q=quit", img)
            if (cv2.waitKey(int(1000 / fps)) & 0xFF) in (ord("q"), 27):
                break
    if writer:
        writer.release()
        print(f"[overhead] wrote {out}  ({n} frames @ {fps:g} fps, {W}x{H})")
    if live:
        cv2.destroyAllWindows()


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("csv", nargs="+", help="pose track CSV(s): t,x,y,yaw")
    ap.add_argument("--setup", default=SETUP)
    ap.add_argument("--out", default=None, help="write mp4 here")
    ap.add_argument("--live", action="store_true")
    ap.add_argument("--fps", type=float, default=30.0)
    ap.add_argument("--labels", nargs="+", default=None)
    args = ap.parse_args()
    if not os.path.exists(args.setup):
        raise SystemExit(f"{args.setup} missing — run derive_camera_pose.py first.")
    tracks = [load_track(c) for c in args.csv]
    labels = args.labels or [os.path.splitext(os.path.basename(c))[0] for c in args.csv]
    render(tracks, load_setup(args.setup), out=args.out, live=args.live,
           fps=args.fps, labels=labels)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
