#!/usr/bin/env python3
"""Sticker calibration check: run the overhead feed and report where every sticker is.

View-only - never commands the rover. Each frame, every configured colour is detected
on its own (no triangle needed) and projected to grid metres, so you can see exactly
what the tracker sees:

  - each sticker's (x, y) in metres, live on the image and in a side panel
  - per-sticker detection rate and jitter (std, mm) while things sit still
  - when all three form the rover triangle: chassis centre, yaw, fit error, each
    edge's measured length vs sticker_config.json, and the camera height the
    triangle's apparent size implies (consistency check of the height correction)
  - optional ground truth: --expect NAME=x,y and/or --expect-center x,y[,yaw]
    turn the run into a pass/fail accuracy check

Plane: stickers on the rover are 0.159 m up, so by default positions are projected on
the sticker plane (the tracker's height-corrected mapping). For loose stickers lying on
the floor (e.g. placed on known tape crossings to check the grid) use --plane floor.

    python3 calib/verify_stickers.py                                  # live window, 1 Hz printout
    python3 calib/verify_stickers.py --duration 15 --report /tmp/stickers.json --csv /tmp/s.csv
    python3 calib/verify_stickers.py --plane floor --expect green=0.5,0.5 --expect pink=1,0.5
    python3 calib/verify_stickers.py --expect-center 0.5,0.5,90       # rover parked on a mark
    python3 calib/verify_stickers.py --image frame.png --save /tmp/stickers.png
    python3 calib/verify_stickers.py --video loop/runs/<run>/.../rep1.mp4

Keys: q/Esc quit | r reset statistics | s snapshot.  Exit code: 0 PASS, 1 FAIL, 2+ error.
Only one program can read UDP 5000: close live_tracking.py / receive.py / loop runs first.
"""
from __future__ import annotations

import argparse
import csv
import json
import math
import os
import sys
import time
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent
for sub in ("", "capture", "viz", "loop"):
    sys.path.insert(0, str(SC / sub))

from sticker_tracker import DEFAULT_CONFIG, StickerTracker, transform_points  # noqa: E402
from metric_overlay import load_calib, draw_metric_grid, draw_axes_and_labels, _txt  # noqa: E402

GRID_CALIB = HERE / "grid_calib.json"
SNAP_DIR = SC / "capture" / "snapshots"
STICKER_BGR = {"green": (60, 220, 60), "orange_red": (40, 90, 255), "pink": (180, 105, 255),
               "yellow": (0, 230, 255)}  # yellow: pre-2026-11-18 rear sticker (old run configs)
PANEL_W = 360
EDGES = ((0, 1), (0, 2), (1, 2))


# ---------------------------------------------------------------- measurement
def hsv_at(frame, px, r=6):
    """Median HSV of the 30 most saturated pixels around px (what a missed dot looked like)."""
    x, y = int(round(px[0])), int(round(px[1]))
    patch = frame[max(0, y - r):y + r + 1, max(0, x - r):x + r + 1]
    if patch.size == 0:
        return None
    hsv = cv2.cvtColor(patch, cv2.COLOR_BGR2HSV).reshape(-1, 3)
    return np.median(hsv[np.argsort(hsv[:, 1])[-30:]], axis=0)


def measure(frame, tracker, H_floor, plane="sticker", last_px=None, gate_px=40.0):
    """One frame -> dict(stickers={name: {...} | None}, triangle={...} | None, status).

    With the full triangle the matched dots are used. Without it a colour's blob is only
    accepted near that sticker's last known pixel (last_px), so a same-coloured object
    elsewhere is never reported as the sticker; missing dots get their HSV recorded."""
    last_px = {} if last_px is None else last_px
    try:
        tri = tracker.detect(frame, H_floor)            # also fills tracker.last_candidates
        H_plane = tracker.plane_homography(H_floor) if plane == "sticker" else H_floor
    except ValueError as exc:
        return dict(stickers={s["name"]: None for s in tracker.stickers}, triangle=None,
                    status=str(exc), candidates={})
    cands = tracker.last_candidates
    stickers, missed_hsv = {}, {}
    for i, st in enumerate(tracker.stickers):
        name, items = st["name"], cands.get(st["name"], [])
        prev = last_px.get(name)
        if tri is not None:
            px = np.asarray(tri["sticker_pixels"][i], float)
        elif items and prev is not None:
            near = min(items, key=lambda it: np.linalg.norm(it["px"] - prev))
            px = np.asarray(near["px"], float) if np.linalg.norm(near["px"] - prev) <= gate_px else None
        elif items:
            px = np.asarray(items[0]["px"], float)       # nothing to gate on: roundest blob
        else:
            px = None
        if px is None:
            stickers[name] = None
            if prev is not None:
                missed_hsv[name] = hsv_at(frame, prev)
            continue
        item = min(items, key=lambda it: np.linalg.norm(it["px"] - px)) if items else {}
        xy = transform_points(H_plane, [px])[0]
        floor = transform_points(H_floor, [px])[0]
        stickers[name] = dict(px=px, xy=xy, floor_xy=floor, n_candidates=len(items),
                              area=float(item.get("area", float("nan"))),
                              circularity=float(item.get("circularity", float("nan"))),
                              in_triangle=tri is not None)
    triangle = None
    if tri is not None:
        names = [s["name"] for s in tracker.stickers]
        pts = np.array([stickers[n]["xy"] for n in names])
        flo = np.array([stickers[n]["floor_xy"] for n in names])
        body = tracker.body
        edge_err = [float(np.linalg.norm(pts[i] - pts[j]) - np.linalg.norm(body[i] - body[j]))
                    for i, j in EDGES]
        # Size check of the height correction: the floor-projected triangle is enlarged by
        # Cz / (Cz - h); invert it to get the camera height the observed size implies.
        ratio = np.mean([np.linalg.norm(body[i] - body[j]) / np.linalg.norm(flo[i] - flo[j])
                         for i, j in EDGES])
        h = float(tracker.config.get("sticker_height_m", 0.0))
        implied = h / (1.0 - ratio) if h > 0 and ratio < 0.999 else None
        triangle = dict(center=np.asarray(tri["center_m"], float), yaw_deg=float(tri["heading_deg"]),
                        fit_m=float(tri["fit_error_m"]), edge_err_m=edge_err,
                        implied_camera_z=implied)
    for name, st in stickers.items():
        if st is not None:
            last_px[name] = st["px"]
    return dict(stickers=stickers, triangle=triangle, status=tracker.status, candidates=cands,
                missed_hsv=missed_hsv)


def _circ_mean_std_deg(a):
    r = np.radians(np.asarray(a, float))
    s, c = np.sin(r).mean(), np.cos(r).mean()
    R = min(1.0, math.hypot(s, c))
    return math.degrees(math.atan2(s, c)), math.degrees(math.sqrt(max(0.0, -2 * math.log(max(R, 1e-12)))))


class Stats:
    def __init__(self, names):
        self.names = names
        self.reset()

    def reset(self):
        self.n = 0
        self.t0 = None
        self.xy = {n: [] for n in self.names}
        self.amb = {n: 0 for n in self.names}
        self.missed_hsv = {n: [] for n in self.names}
        self.tri = []

    def add(self, t, m):
        self.t0 = t if self.t0 is None else self.t0
        self.t_last = t
        self.n += 1
        for n in self.names:
            s = m["stickers"].get(n)
            if s is not None:
                self.xy[n].append(s["xy"])
                if s["n_candidates"] > 1 and not s["in_triangle"]:
                    self.amb[n] += 1
        for n, hsv in m.get("missed_hsv", {}).items():
            if hsv is not None:
                self.missed_hsv[n].append(hsv)
        if m["triangle"] is not None:
            self.tri.append(m["triangle"])

    def sticker(self, n):
        a = np.array(self.xy[n])
        if not len(a):
            return dict(detect=0.0)
        mean = a.mean(axis=0)
        out = dict(detect=len(a) / max(1, self.n), mean=mean,
                   std_mm=float(np.sqrt(np.mean(np.sum((a - mean) ** 2, axis=1)))) * 1000,
                   ambiguous=self.amb[n] / max(1, self.n))
        if self.missed_hsv[n]:
            out["missed_frames_hsv_median"] = np.median(self.missed_hsv[n], axis=0)
        return out

    def triangle(self):
        if not self.tri:
            return dict(detect=0.0)
        c = np.array([t["center"] for t in self.tri])
        e = np.array([t["edge_err_m"] for t in self.tri]) * 1000
        z = [t["implied_camera_z"] for t in self.tri if t["implied_camera_z"]]
        yaw_mean, yaw_std = _circ_mean_std_deg([t["yaw_deg"] for t in self.tri])
        return dict(detect=len(self.tri) / max(1, self.n), center=c.mean(axis=0),
                    center_std_mm=float(np.sqrt(np.mean(np.sum((c - c.mean(0)) ** 2, axis=1)))) * 1000,
                    yaw=yaw_mean, yaw_std=yaw_std, edge_err_mm=e.mean(axis=0),
                    fit_mm=float(np.mean([t["fit_m"] for t in self.tri])) * 1000,
                    implied_camera_z=float(np.median(z)) if z else None)


# ---------------------------------------------------------------- verdict
def checks(stats, args, tracker):
    """-> list of (label, value text, passed)."""
    out = []
    for n in stats.names:
        s = stats.sticker(n)
        out.append((f"{n} detected", f"{s['detect'] * 100:.0f}%", s["detect"] >= args.min_detect))
        if s["detect"] > 0 and not args.moving:
            out.append((f"{n} jitter", f"{s['std_mm']:.1f} mm", s["std_mm"] <= args.max_jitter_mm))
    tri = stats.triangle()
    if args.plane == "sticker":
        out.append(("triangle found", f"{tri['detect'] * 100:.0f}%", tri["detect"] >= args.min_detect))
        if tri["detect"] > 0:
            names = [s["name"] for s in tracker.stickers]
            for k, (i, j) in enumerate(EDGES):
                err = tri["edge_err_mm"][k]
                out.append((f"edge {names[i][:6]}-{names[j][:6]}", f"{err:+.1f} mm",
                            abs(err) <= args.max_edge_mm))
    for name, (x, y) in args.expect.items():
        s = stats.sticker(name)
        if s["detect"] == 0:
            out.append((f"{name} at ({x:g},{y:g})", "not seen", False))
            continue
        d = float(np.linalg.norm(s["mean"] - [x, y])) * 1000
        out.append((f"{name} at ({x:g},{y:g})", f"{d:.1f} mm off", d <= args.expect_tol_mm))
    if args.expect_center is not None:
        ec = args.expect_center
        if tri["detect"] == 0:
            out.append(("centre", "not seen", False))
        else:
            d = float(np.linalg.norm(tri["center"] - ec[:2])) * 1000
            out.append((f"centre at ({ec[0]:g},{ec[1]:g})", f"{d:.1f} mm off", d <= args.expect_tol_mm))
            if len(ec) > 2:
                dy = abs((tri["yaw"] - ec[2] + 180) % 360 - 180)
                out.append((f"yaw {ec[2]:g} deg", f"{dy:.1f} deg off", dy <= args.expect_yaw_deg))
    return out


def report_dict(stats, args, tracker, grid_path):
    res = checks(stats, args, tracker)
    tri = stats.triangle()
    jl = lambda v: None if v is None else (np.asarray(v).tolist() if hasattr(v, "__len__") else v)
    return dict(
        created=datetime.now().isoformat(timespec="seconds"),
        grid_calib=str(grid_path), sticker_config=str(tracker.config_path), plane=args.plane,
        frames=stats.n, seconds=round(getattr(stats, "t_last", 0) - (stats.t0 or 0), 2),
        camera_position_m=tracker.config.get("camera_position_m"),
        hsv_ranges={st["name"]: st["hsv_ranges"] for st in tracker.stickers},
        stickers={n: {k: jl(v) for k, v in stats.sticker(n).items()} for n in stats.names},
        triangle={k: jl(v) for k, v in tri.items()},
        checks=[dict(check=a, value=b, passed=bool(c)) for a, b, c in res],
        passed=all(c for _, _, c in res) and stats.n > 0)


def print_summary(rep):
    print("\n=== sticker verification ===")
    print(f"frames {rep['frames']} over {rep['seconds']} s, plane: {rep['plane']}")
    for n, s in rep["stickers"].items():
        if s.get("mean") is None:
            print(f"  {n:<11} never detected")
            continue
        print(f"  {n:<11} ({s['mean'][0]:+.4f}, {s['mean'][1]:+.4f}) m   jitter {s['std_mm']:.1f} mm"
              f"   detected {s['detect'] * 100:.0f}%")
        if s.get("missed_frames_hsv_median") is not None:
            hsv = ", ".join(f"{v:.0f}" for v in s["missed_frames_hsv_median"])
            print(f"  {'':<11} when missed, its spot looked like HSV ({hsv}); compare "
                  "sticker_config.json hsv_ranges")
    t = rep["triangle"]
    if t.get("center") is not None:
        z = t.get("implied_camera_z")
        print(f"  chassis centre ({t['center'][0]:+.4f}, {t['center'][1]:+.4f}) m  yaw {t['yaw']:+.1f} deg"
              f"  (jitter {t['center_std_mm']:.1f} mm / {t['yaw_std']:.2f} deg)  fit {t['fit_mm']:.1f} mm")
        cz = (rep.get("camera_position_m") or [None] * 3)[2]
        if z is not None and cz:
            print(f"  camera height implied by triangle size {z:.2f} m (sticker_config: {cz:.2f} m)")
    for c in rep["checks"]:
        print(f"  [{'PASS' if c['passed'] else 'FAIL'}] {c['check']:<28} {c['value']}")
    print("RESULT:", "PASS" if rep["passed"] else "FAIL")


# ---------------------------------------------------------------- drawing
def draw(frame, m, H_m2img, grid_m, show_grid=True):
    img = frame.copy()
    if show_grid:
        draw_metric_grid(img, H_m2img, grid_m)
        draw_axes_and_labels(img, H_m2img, grid_m)
    for name, items in m.get("candidates", {}).items():
        col = STICKER_BGR.get(name, (255, 255, 255))
        for it in items:
            r = max(5, int(math.sqrt(it["area"] / math.pi)) + 4)
            cv2.circle(img, tuple(np.int32(it["px"])), r, col, 1, cv2.LINE_AA)
    tri = m["triangle"]
    pts = [s["px"] for s in m["stickers"].values() if s is not None]
    if tri is not None and len(pts) == 3:
        cv2.polylines(img, [np.int32(pts)], True, (255, 255, 255), 1, cv2.LINE_AA)
    mid = np.mean(pts, axis=0) if pts else None
    for name, s in m["stickers"].items():
        if s is None:
            continue
        col = STICKER_BGR.get(name, (255, 255, 255))
        p = tuple(np.int32(np.round(s["px"])))
        cv2.circle(img, p, 11, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.circle(img, p, 11, col, 2, cv2.LINE_AA)
        cv2.drawMarker(img, p, (255, 255, 255), cv2.MARKER_CROSS, 6, 1)
        # push each label outward from the triangle so neighbouring labels don't collide
        label = f"{name} ({s['xy'][0]:+.3f}, {s['xy'][1]:+.3f})"
        d = s["px"] - mid if len(pts) > 1 else np.array([1.0, -0.5])
        d = d / (np.linalg.norm(d) + 1e-9)
        tw = cv2.getTextSize(label, cv2.FONT_HERSHEY_SIMPLEX, 0.5, 1)[0][0]
        org = np.array(p) + d * 22 + [0 if d[0] >= 0 else -tw, 5 + (12 if d[1] > 0.3 else 0)]
        _txt(img, label, tuple(np.int32(org)), col, 0.5)
    return img


def panel(h, m, stats, args, tracker, fps):
    pnl = np.full((h, PANEL_W, 3), 28, np.uint8)
    y = 30

    def line(s, col=(230, 230, 230), sc=0.5, dy=22, th=1):
        nonlocal y
        cv2.putText(pnl, s, (12, y), cv2.FONT_HERSHEY_SIMPLEX, sc, col, th, cv2.LINE_AA)
        y += dy

    line("STICKER VERIFICATION", (255, 255, 255), 0.62, 26, 2)
    line(f"plane: {args.plane}   {fps:4.1f} fps   {stats.n} frames", (170, 170, 170), 0.45)
    line(tracker.status[:52], (170, 170, 170), 0.4, 26)
    for st in tracker.stickers:
        n = st["name"]
        col = STICKER_BGR.get(n, (255, 255, 255))
        s, agg = m["stickers"].get(n), stats.sticker(n)
        line(f"{n}", col, 0.55, 22, 2)
        if s is None:
            line("   not detected", (90, 90, 255), 0.5)
        else:
            line(f"   x {s['xy'][0]:+.4f}  y {s['xy'][1]:+.4f} m", (255, 255, 255), 0.5)
        if agg["detect"] > 0:
            line(f"   seen {agg['detect'] * 100:3.0f}%  jitter {agg['std_mm']:4.1f} mm  "
                 f"blobs {len(m.get('candidates', {}).get(n, []))}", (180, 180, 180), 0.42, 24)
        else:
            y += 2
    tri = m["triangle"]
    line("triangle / chassis centre", (255, 255, 255), 0.55, 22, 2)
    if tri is None:
        line("   --", (128, 128, 128), 0.5)
    else:
        c = tri["center"]
        line(f"   ({c[0]:+.4f}, {c[1]:+.4f}) m  {tri['yaw_deg']:+.1f} deg", (255, 255, 255), 0.5)
        e = tri["edge_err_m"]
        line(f"   edges {e[0]*1000:+.1f} {e[1]*1000:+.1f} {e[2]*1000:+.1f} mm  fit "
             f"{tri['fit_m']*1000:.1f}", (180, 180, 180), 0.42)
        if tri["implied_camera_z"]:
            cz = (tracker.config.get("camera_position_m") or [0, 0, 0])[2]
            line(f"   size -> camera z {tri['implied_camera_z']:.2f} m (cfg {cz:.2f})",
                 (180, 180, 180), 0.42)
    y += 6
    if stats.n:
        for label, val, ok in checks(stats, args, tracker):
            if y > h - 30:
                break
            line(f"{'OK  ' if ok else 'FAIL'} {label[:24]:<24} {val}",
                 (0, 220, 0) if ok else (60, 60, 255), 0.4, 18)
    cv2.putText(pnl, "q quit   r reset stats   s snapshot", (10, h - 12),
                cv2.FONT_HERSHEY_SIMPLEX, 0.42, (150, 150, 150), 1, cv2.LINE_AA)
    return pnl


def fmt_line(t, m):
    parts = [f"t={t:6.1f}s"]
    for n, s in m["stickers"].items():
        parts.append(f"{n} " + ("--" if s is None else f"({s['xy'][0]:+.4f},{s['xy'][1]:+.4f})"))
    tri = m["triangle"]
    parts.append("| centre " + ("--" if tri is None else
                                f"({tri['center'][0]:+.4f},{tri['center'][1]:+.4f}) "
                                f"{tri['yaw_deg']:+.1f}deg fit {tri['fit_m']*1000:.1f}mm"))
    return "  ".join(parts)


# ---------------------------------------------------------------- main
def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=str(GRID_CALIB), help="grid calibration JSON")
    ap.add_argument("--stickers", default=str(DEFAULT_CONFIG), help="sticker config JSON")
    ap.add_argument("--plane", choices=("sticker", "floor"), default="sticker",
                    help="project on the rover's sticker plane (default) or the floor")
    ap.add_argument("--image", help="analyse one saved frame")
    ap.add_argument("--video", help="replay a recorded RAW overhead clip")
    ap.add_argument("--no-pi", action="store_true", help="don't SSH to the Pi to start its stream")
    ap.add_argument("--duration", type=float, default=0.0, help="stop after N s (0 = until q)")
    ap.add_argument("--warmup", type=float, default=2.0,
                    help="ignore the first N s of the stream (H.264 frames before a keyframe "
                         "are corrupt)")
    ap.add_argument("--print-every", type=float, default=1.0, help="stdout line period (s, 0=off)")
    ap.add_argument("--csv", help="per-frame sticker positions")
    ap.add_argument("--report", help="write the summary + checks as JSON")
    ap.add_argument("--save", help="write the last annotated view (PNG)")
    ap.add_argument("--expect", action="append", default=[], metavar="NAME=x,y",
                    help="known position of one sticker (repeatable)")
    ap.add_argument("--expect-center", help="known chassis centre x,y[,yaw_deg]")
    ap.add_argument("--expect-tol-mm", type=float, default=15.0)
    ap.add_argument("--expect-yaw-deg", type=float, default=3.0)
    ap.add_argument("--min-detect", type=float, default=0.9, help="required detection fraction")
    ap.add_argument("--max-jitter-mm", type=float, default=3.0)
    ap.add_argument("--max-edge-mm", type=float, default=5.0)
    ap.add_argument("--moving", action="store_true",
                    help="stickers move during the run: skip the stationary jitter checks")
    args = ap.parse_args(argv)
    expect = {}
    for e in args.expect:
        try:
            name, xy = e.split("=")
            x, y = (float(v) for v in xy.split(","))
        except ValueError:
            ap.error(f"--expect wants NAME=x,y, got {e!r}")
        expect[name.strip()] = (x, y)
    args.expect = expect
    if args.expect_center:
        args.expect_center = np.array([float(v) for v in args.expect_center.split(",")])
        if len(args.expect_center) not in (2, 3):
            ap.error("--expect-center wants x,y or x,y,yaw_deg")
    return args


def main(argv=None):
    args = parse_args(argv)
    try:
        H_img2m, H_m2img, grid_m = load_calib(args.calib)
        tracker = StickerTracker(args.stickers)
    except (OSError, ValueError, KeyError) as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    names = [s["name"] for s in tracker.stickers]
    unknown = set(args.expect) - set(names)
    if unknown:
        print(f"ERROR: --expect names must be among {names}, got {sorted(unknown)}", file=sys.stderr)
        return 2
    stats = Stats(names)

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"ERROR: cannot read {args.image}", file=sys.stderr)
            return 2
        m = measure(frame, tracker, H_img2m, args.plane)
        stats.add(0.0, m)
        print(fmt_line(0.0, m))
        args.moving = True                       # one frame: jitter is meaningless
        if args.save:
            img = draw(frame, m, H_m2img, grid_m)
            cv2.imwrite(args.save, np.hstack([img, panel(img.shape[0], m, stats, args, tracker, 0)]))
            print(f"wrote {args.save}")
        return finish(stats, args, tracker)

    from live_tracking import VideoReplay, ensure_pi_stream, stop_pi_stream
    from stream import LatestFrame
    started_pi = False
    if args.video:
        source = VideoReplay(args.video)
    else:
        if not args.no_pi:
            started_pi = ensure_pi_stream()
        source = LatestFrame()
    try:
        if not source.opened() or source.wait_first(20) is None:
            print("ERROR: no overhead frames (Pi streaming? another program on UDP 5000?)",
                  file=sys.stderr)
            return 3
        return run(source, args, tracker, stats, H_img2m, H_m2img, grid_m)
    finally:
        source.release()
        if started_pi:
            stop_pi_stream()


def run(source, args, tracker, stats, H_img2m, H_m2img, grid_m):
    show = bool(os.environ.get("DISPLAY"))
    win = "sticker verification  (q quit, r reset, s snapshot)"
    if show:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280 + PANEL_W, 720)
    writer = fh = None
    if args.csv:
        fh = open(args.csv, "w", newline="")
        writer = csv.writer(fh)
        head = ["t", "frame"]
        for n in stats.names:
            head += [f"{n}_u", f"{n}_v", f"{n}_x", f"{n}_y", f"{n}_blobs"]
        writer.writerow(head + ["center_x", "center_y", "yaw_deg", "fit_mm", "plane"])
    t_start, last_n, last_print = time.time(), -1, -1e9
    last_px = {}
    warm_until = t_start + args.warmup
    fps, fps_t, fps_n = 0.0, time.time(), 0
    out = None
    try:
        while not args.duration or time.time() - t_start < args.duration:
            frame, stamp, n = source.read()
            if frame is None or n == last_n:
                if show and (cv2.waitKey(5) & 0xFF) in (ord("q"), 27):
                    break
                time.sleep(0.003)
                continue
            last_n = n
            if stamp < warm_until:
                continue                     # first frames after (re)connecting can be corrupt
            t = stamp - t_start
            m = measure(frame, tracker, H_img2m, args.plane, last_px)
            stats.add(t, m)
            if writer:
                row = [f"{t:.3f}", n]
                for name in stats.names:
                    s = m["stickers"][name]
                    row += ([""] * 4 if s is None else
                            [f"{s['px'][0]:.2f}", f"{s['px'][1]:.2f}", f"{s['xy'][0]:.5f}",
                             f"{s['xy'][1]:.5f}"]) + [len(m["candidates"].get(name, []))]
                tri = m["triangle"]
                row += ([""] * 4 if tri is None else
                        [f"{tri['center'][0]:.5f}", f"{tri['center'][1]:.5f}",
                         f"{tri['yaw_deg']:.2f}", f"{tri['fit_m'] * 1000:.2f}"]) + [args.plane]
                writer.writerow(row)
            now = time.time()
            if now - fps_t >= 0.5:
                fps, fps_t, fps_n = (stats.n - fps_n) / (now - fps_t), now, stats.n
            if args.print_every and now - last_print >= args.print_every:
                last_print = now
                print(fmt_line(t, m), flush=True)
            if show or args.save:
                img = draw(frame, m, H_m2img, grid_m)
                out = np.hstack([img, panel(img.shape[0], m, stats, args, tracker, fps)])
            if not show:
                continue
            cv2.imshow(win, out)
            k = cv2.waitKey(1) & 0xFF
            if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord("r"):
                stats.reset()
                print("[verify] statistics reset")
            elif k == ord("s"):
                SNAP_DIR.mkdir(parents=True, exist_ok=True)
                fn = SNAP_DIR / datetime.now().strftime("stickers_%Y%m%d_%H%M%S.png")
                cv2.imwrite(str(fn), out)
                print(f"[verify] snapshot -> {fn}")
    except KeyboardInterrupt:
        pass
    finally:
        if fh:
            fh.close()
            print(f"wrote {args.csv}")
        if args.save and out is not None:
            cv2.imwrite(args.save, out)
            print(f"wrote {args.save}")
        if show:
            cv2.destroyAllWindows()
    return finish(stats, args, tracker)


def finish(stats, args, tracker):
    rep = report_dict(stats, args, tracker, args.calib)
    print_summary(rep)
    if args.report:
        Path(args.report).write_text(json.dumps(rep, indent=2) + "\n")
        print(f"wrote {args.report}")
    return 0 if rep["passed"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
