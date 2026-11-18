#!/usr/bin/env python3
"""Live overhead view of the grid + rover: sticker tracking, position and heading.

View only - this never commands the rover. It opens the Pi's overhead C920e
stream (UDP 5000), runs the existing colour-sticker tracker (calib/sticker_tracker.py
via loop/camera_track.py, including its 2-of-3 sticker occlusion fallback), and
draws on the live feed:

  - the 0.1 m metric grid warped onto the tape grid, origin + X/Y axes
  - every candidate colour blob (thin rings) and the matched sticker triangle
  - the chassis-centre position and a heading arrow, plus a fading trail
  - a side panel: x, y (m), yaw (deg), heading compass, per-colour blob counts,
    fit error, fps, feed age and tracked-frame rate

    python3 sim2real_calib/live_tracking.py            # starts the Pi stream if it is off
    python3 sim2real_calib/live_tracking.py --no-pi    # don't touch the Pi
    python3 sim2real_calib/live_tracking.py --video loop/runs/<run>/iter_01/real/T2_spin/rep1.mp4

Keys: q/Esc quit | c clear trail | b toggle blob rings | g toggle grid | s snapshot

Only one program can consume UDP 5000 at a time: close receive.py, metric_overlay.py,
run_viz.sh or a running loop/demo before starting this.
"""
from __future__ import annotations

import argparse
import math
import os
import subprocess
import sys
import threading
import time
from collections import deque
from datetime import datetime
from pathlib import Path

import cv2
import numpy as np

SC = Path(__file__).resolve().parent
sys.path.insert(0, str(SC / "loop"))
from camera_track import CameraTrack, CameraError, GRID_CALIB  # noqa: E402
from metric_overlay import draw_metric_grid, draw_axes_and_labels, m2img, _txt  # noqa: E402
from sticker_tracker import DEFAULT_CONFIG  # noqa: E402

PI = "joshu@pi-cam.local"
PI_SERVICE = "sim2real-camera.service"
SSHOPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "ConnectTimeout=5"]
SNAP_DIR = SC / "capture" / "snapshots"

STALE_S = 0.5          # feed older than this -> pose not shown as live
TRAIL_LEN = 600        # trail points kept (~20 s at 30 fps)
HEADING_ARROW_M = 0.15
PANEL_W = 300

# BGR display colour per configured sticker name
STICKER_BGR = {"green": (60, 220, 60), "orange_red": (40, 90, 255), "pink": (180, 105, 255),
               "yellow": (0, 230, 255)}  # yellow: pre-2026-11-18 rear sticker (old run configs)


# ---------------------------------------------------------------- Pi stream
def _ssh(cmd, timeout=10):
    try:
        return subprocess.run(["ssh", *SSHOPTS, PI, cmd], capture_output=True, text=True,
                              timeout=timeout)
    except (subprocess.TimeoutExpired, FileNotFoundError):
        return None


def ensure_pi_stream():
    """Start the Pi camera service if it is off. -> True if we started it."""
    r = _ssh(f"systemctl is-active {PI_SERVICE}")
    if r is None or r.returncode == 255:
        print(f"[live] WARN: can't reach {PI} over SSH; assuming the stream is already "
              "running (start it on the Pi if no frames arrive)", file=sys.stderr)
        return False
    if r.stdout.strip() == "active":
        print("[live] Pi camera stream already running")
        return False
    print("[live] starting Pi camera stream ...")
    r = _ssh(f"sudo -n systemctl start {PI_SERVICE}")
    if r is None or r.returncode != 0:
        print("[live] WARN: couldn't start the Pi stream; start it manually", file=sys.stderr)
        return False
    return True


def stop_pi_stream():
    print("[live] stopping the Pi camera stream we started (protects its power supply)")
    _ssh(f"sudo -n systemctl stop {PI_SERVICE}")


# ---------------------------------------------------------------- offline source
class VideoReplay:
    """A recorded raw overhead clip served like LatestFrame, paced at its fps and looped."""

    def __init__(self, path):
        self.cap = cv2.VideoCapture(str(path))
        self.fps = self.cap.get(cv2.CAP_PROP_FPS) or 30.0
        self._lock = threading.Lock()
        self._frame, self._stamp, self._n = None, 0.0, 0
        self._running = self.cap.isOpened()
        self._t = None
        if self._running:
            self._t = threading.Thread(target=self._loop, daemon=True)
            self._t.start()

    def opened(self):
        return self.cap.isOpened()

    def _loop(self):
        period = 1.0 / self.fps
        while self._running:
            ok, frame = self.cap.read()
            if not ok:
                self.cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
                continue
            with self._lock:
                self._frame, self._stamp, self._n = frame, time.time(), self._n + 1
            time.sleep(period)

    def read(self):
        with self._lock:
            return self._frame, self._stamp, self._n

    def wait_first(self, timeout=20.0):
        t_end = time.time() + timeout
        while time.time() < t_end:
            if self.read()[0] is not None:
                return self.read()[0]
            time.sleep(0.05)
        return None

    def release(self):
        self._running = False
        if self._t is not None:
            self._t.join(timeout=3.0)
            if self._t.is_alive():
                return
        self.cap.release()


# ---------------------------------------------------------------- drawing
def draw_blobs(img, candidates):
    for name, items in candidates.items():
        col = STICKER_BGR.get(name, (255, 255, 255))
        for it in items:
            r = max(5, int(math.sqrt(it["area"] / math.pi)) + 4)
            cv2.circle(img, tuple(np.int32(it["px"])), r, col, 1, cv2.LINE_AA)


def draw_trail(img, H_m2img, trail):
    if len(trail) < 2:
        return
    px = np.int32(m2img(H_m2img, list(trail)))
    n = len(px)
    for i in range(1, n):          # older segments fade out
        a = i / n
        cv2.line(img, tuple(px[i - 1]), tuple(px[i]),
                 (int(255 * a), int(255 * a), int(80 * a)), 2, cv2.LINE_AA)


def draw_rover(img, H_m2img, r, live):
    partial = r.get("partial", False)
    col = (0, 200, 255) if partial else (0, 255, 0)
    if not live:
        col = (128, 128, 128)
    pts = np.int32(r["sticker_pixels"])
    if len(pts) >= 2:
        cv2.polylines(img, [pts], len(pts) == 3, (255, 255, 255), 1, cv2.LINE_AA)
    for p, name in zip(pts, r["sticker_names"]):
        cv2.circle(img, tuple(p), 9, STICKER_BGR.get(name, (255, 255, 255)), 2, cv2.LINE_AA)
    c = np.asarray(r["center_m"], float)
    yaw = math.radians(r["heading_deg"])
    tip = c + HEADING_ARROW_M * np.array([math.cos(yaw), math.sin(yaw)])
    c_px, tip_px = np.int32(m2img(H_m2img, [c, tip]))
    cv2.arrowedLine(img, tuple(c_px), tuple(tip_px), (0, 0, 0), 5, cv2.LINE_AA, tipLength=0.3)
    cv2.arrowedLine(img, tuple(c_px), tuple(tip_px), col, 2, cv2.LINE_AA, tipLength=0.3)
    cv2.circle(img, tuple(c_px), 5, (0, 0, 255), -1, cv2.LINE_AA)
    _txt(img, f"({c[0]:.3f}, {c[1]:.3f}) m  {r['heading_deg']:.1f} deg",
         tuple(c_px + [14, 24]), col, 0.55)


def draw_compass(panel, center, radius, yaw_deg, col):
    cx, cy = center
    cv2.circle(panel, center, radius, (90, 90, 90), 1, cv2.LINE_AA)
    # grid frame as seen from overhead: +X right, +Y up (matches the camera view)
    for ang, lab in ((0, "+X"), (90, "+Y"), (180, "-X"), (270, "-Y")):
        a = math.radians(ang)
        q = (int(cx + (radius + 14) * math.cos(a)) - 10, int(cy - (radius + 14) * math.sin(a)) + 5)
        cv2.putText(panel, lab, q, cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
    if yaw_deg is not None:
        a = math.radians(yaw_deg)
        tip = (int(cx + radius * 0.85 * math.cos(a)), int(cy - radius * 0.85 * math.sin(a)))
        cv2.arrowedLine(panel, center, tip, col, 3, cv2.LINE_AA, tipLength=0.25)
    cv2.circle(panel, center, 3, (200, 200, 200), -1)


def build_panel(h, r, live, tracker, stats):
    panel = np.full((h, PANEL_W, 3), 28, np.uint8)
    y = 34

    def line(s, col=(230, 230, 230), scale=0.55, dy=26, th=1):
        nonlocal y
        cv2.putText(panel, s, (14, y), cv2.FONT_HERSHEY_SIMPLEX, scale, col, th, cv2.LINE_AA)
        y += dy

    if not stats["feed_ok"]:
        state, scol = "FEED STALE", (60, 60, 255)
    elif r is None:
        state, scol = "LOST", (60, 60, 255)
    elif r.get("partial"):
        state, scol = "TRACKING 2/3", (0, 200, 255)
    else:
        state, scol = "TRACKING 3/3", (0, 255, 0)
    line("ROVER TRACKING", (255, 255, 255), 0.65, 34, 2)
    line(state, scol, 0.8, 30, 2)
    line(tracker.status[:46], (170, 170, 170), 0.42, 30)

    if r is not None:
        x, yy = r["center_m"]
        vcol = (255, 255, 255) if live else (128, 128, 128)
        line(f"x    {x:+.3f} m", vcol, 0.75, 32, 2)
        line(f"y    {yy:+.3f} m", vcol, 0.75, 32, 2)
        line(f"yaw  {r['heading_deg']:+7.1f} deg", vcol, 0.75, 32, 2)
        err = "edge err" if r.get("partial") else "fit err"
        line(f"{err}  {r['fit_error_m'] * 1000:.1f} mm", (170, 170, 170), 0.5, 26)
    else:
        for lab in ("x", "y", "yaw"):
            line(f"{lab:<4} --", (128, 128, 128), 0.75, 32, 2)
        line("", dy=26)

    radius = 62
    y += 16
    draw_compass(panel, (PANEL_W // 2, y + radius), radius,
                 r["heading_deg"] if r is not None else None, scol)
    y += 2 * radius + 40

    line("blobs per colour:", (200, 200, 200), 0.5, 22)
    for st in tracker.stickers:
        n = len(tracker.last_candidates.get(st["name"], []))
        line(f"  {st['name']:<11} {n}", STICKER_BGR.get(st["name"], (255, 255, 255)), 0.5, 22)
    y += 6
    line(f"{stats['fps']:4.1f} fps   feed age {stats['age_ms']:4.0f} ms", (200, 200, 200), 0.48, 22)
    line(f"tracked {stats['tracked_pct']:5.1f}% of last {stats['window']} frames",
         (200, 200, 200), 0.48, 22)

    y = h - 16
    cv2.putText(panel, "q quit  c trail  b blobs  g grid  s snap", (10, y),
                cv2.FONT_HERSHEY_SIMPLEX, 0.4, (150, 150, 150), 1, cv2.LINE_AA)
    return panel


def render(frame, r, live, cam, trail, show_grid, show_blobs, stats):
    img = frame.copy()
    if show_grid:
        draw_metric_grid(img, cam.H_m2img, cam.grid_m)
        draw_axes_and_labels(img, cam.H_m2img, cam.grid_m)
    if show_blobs:
        draw_blobs(img, cam.tracker.last_candidates)
    draw_trail(img, cam.H_m2img, trail)
    if r is not None:
        draw_rover(img, cam.H_m2img, r, live)
    return np.hstack([img, build_panel(img.shape[0], r, live, cam.tracker, stats)])


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--no-pi", action="store_true",
                    help="don't SSH to the Pi to start/stop its camera stream")
    ap.add_argument("--video", default=None,
                    help="replay a recorded RAW overhead clip instead of the live stream")
    ap.add_argument("--calib", default=str(GRID_CALIB), help="grid calibration JSON")
    ap.add_argument("--stickers", default=str(DEFAULT_CONFIG), help="sticker config JSON")
    ap.add_argument("--duration", type=float, default=0.0,
                    help="quit after N seconds (0 = until q)")
    ap.add_argument("--save-last", default=None,
                    help="write the last rendered view to this PNG on exit")
    args = ap.parse_args()

    show = bool(os.environ.get("DISPLAY"))
    if not show and not args.save_last:
        print("[live] ERROR: no DISPLAY - run this on the desktop session", file=sys.stderr)
        return 2

    started_pi = False
    source = None
    if args.video:
        source = VideoReplay(args.video)
    elif not args.no_pi:
        started_pi = ensure_pi_stream()

    try:
        print("[live] waiting for the overhead feed ...")
        try:
            cam = CameraTrack(grid_calib=args.calib, stickers=args.stickers,
                              allow_approximate=True, source=source)
        except CameraError as exc:
            print(f"[live] ERROR: {exc}\n       close receive.py / metric_overlay.py / "
                  "run_viz.sh / loop runs first", file=sys.stderr)
            return 3
        except (ValueError, OSError) as exc:
            print(f"[live] ERROR: {exc}", file=sys.stderr)
            return 2
        return run(cam, args, show)
    finally:
        if started_pi:
            stop_pi_stream()


def run(cam, args, show):
    win = "rover live tracking  (q quit)"
    if show:
        cv2.namedWindow(win, cv2.WINDOW_NORMAL)
        cv2.resizeWindow(win, 1280 + PANEL_W, 720)
    trail = deque(maxlen=TRAIL_LEN)
    hits = deque(maxlen=150)               # tracked yes/no for recent frames
    show_grid = show_blobs = True
    last_n, fps, fps_t, fps_n = -1, 0.0, time.time(), cam.n_frames
    last_r = None
    out = None
    t_start = time.monotonic()
    try:
        while not args.duration or time.monotonic() - t_start < args.duration:
            frame, r = cam.latest()
            if frame is None:
                time.sleep(0.01)
                continue
            _, stamp, _ = cam.cam.read()
            now = time.time()
            age = now - stamp
            feed_ok = age <= STALE_S
            n = cam.n_frames
            if n != last_n:                 # a newly processed frame
                last_n = n
                hits.append(r is not None)
                if r is not None:
                    trail.append(tuple(float(v) for v in r["center_m"]))
                    last_r = r
            if now - fps_t >= 0.5:
                fps, fps_t, fps_n = (n - fps_n) / (now - fps_t), now, n
            # keep the last pose on screen (greyed) while lost, so you can see where it was
            shown, live = (r, feed_ok) if r is not None else (None, False)
            stats = dict(feed_ok=feed_ok, fps=fps, age_ms=age * 1000.0, window=len(hits),
                         tracked_pct=100.0 * sum(hits) / max(1, len(hits)))
            out = render(frame, shown, live, cam, trail, show_grid, show_blobs, stats)
            if r is None and last_r is not None:
                draw_rover(out, cam.H_m2img, last_r, live=False)
            if not show:
                time.sleep(0.03)
                continue
            cv2.imshow(win, out)
            k = cv2.waitKey(15) & 0xFF
            if k in (ord("q"), 27) or cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) < 1:
                break
            if k == ord("c"):
                trail.clear()
            elif k == ord("b"):
                show_blobs = not show_blobs
            elif k == ord("g"):
                show_grid = not show_grid
            elif k == ord("s"):
                SNAP_DIR.mkdir(parents=True, exist_ok=True)
                fn = SNAP_DIR / datetime.now().strftime("live_%Y%m%d_%H%M%S.png")
                cv2.imwrite(str(fn), out)
                print(f"[live] snapshot -> {fn}")
    except KeyboardInterrupt:
        pass
    finally:
        if args.save_last and out is not None:
            cv2.imwrite(args.save_last, out)
            print(f"[live] wrote {args.save_last}")
        cam.close()
        if show:
            cv2.destroyAllWindows()
        print(f"[live] frames {cam.n_frames}, tracked {cam.n_detect} "
              f"({cam.n_partial} with 2/3 stickers)")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
