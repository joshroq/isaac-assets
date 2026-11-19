"""One persistent window for a whole sim-vs-real session (demo battery or closed loop).

The window is created once and owned by a GUI thread, so it stays open and responsive
while the main thread is blocked (rover returning to start, Isaac launches, optimiser
evaluations, comparison plots, video encoding). Callers only push state:

    dash = Dashboard("sim2real demo", grid_dims=(1, 1), show=True, snapshot="live.png")
    dash.camera_source(lambda: cam.overlay(trails=...))   # polled ~15 Hz: live camera
    dash.status("T2: returning to start")                  # banner
    dash.log("[real] ...")                                 # ticker (last lines)
    dash.current("T2 live", [Track("real", pts, REAL), Track("sim", pts2, SIM)], ["err 3 cm"])
    dash.result("T2", "T2_spin", tracks, ["endpoint 2.1 cm", ...])   # gallery, stays up
    dash.params(param_sheet.build(theta, keys, ...))       # fitted-parameter panel + sheet
    dash.hold("done - press q")                            # block until q / window closed
    Dashboard(..., record="session_video.mp4")             # whole session as one video
    dash.close()

Layout: banner | live camera (left) + current test top-down (right) | gallery of every
finished test (top-down real vs sim, metrics) + log ticker. Paths are drawn in GRID
metres (+Y up), so all tests share one frame. Without a display the same composite is
still written to `snapshot` once a second (headless runs stay observable).

Keys: p toggles the full parameter sheet (every sim parameter: being fitted, held,
measured, locked geometry) over the camera panel. q / Esc or closing the window during
a run interrupts the main thread (SIGINT), which the scripts already handle by stopping
both rovers.
"""
from __future__ import annotations

import math
import os
import signal
import threading
import time
from collections import OrderedDict, deque
from dataclasses import dataclass, field

import cv2
import numpy as np

REAL = (60, 220, 60)
SIM = (255, 200, 0)
BEFORE = (60, 60, 255)
AFTER = (255, 200, 0)
CANDIDATE = (170, 170, 170)

W, H = 1600, 900
BANNER = 46
TOP_H = 576                       # camera 1024 x 576 (16:9) + 576 x 576 top-down
CAM_W = W - TOP_H
LOG_W = 520


# One path to draw: label, points in grid metres, BGR colour, line thickness.
@dataclass
class Track:
    label: str
    pts: object                   # (N, 2|3|4) grid metres: x, y[, yaw] or t, x, y, yaw
    color: tuple
    thickness: int = 2
    xy: np.ndarray = field(init=False, repr=False)

    # Keep just the (x, y) columns: rows may be (x, y), (x, y, yaw) or (t, x, y, yaw).
    def __post_init__(self):
        points = np.asarray(self.pts, float)
        if points.ndim != 2 or len(points) == 0:
            self.xy = np.zeros((0, 2))
        elif points.shape[1] == 4:                      # t, x, y, yaw (trajectory CSV rows)
            self.xy = points[:, 1:3]
        else:
            self.xy = points[:, :2]


_ASCII = {"±": "+/-", "°": " deg", "→": "->", "—": "-", "–": "-", "×": "x", "≈": "~",
          "σ": "sigma", "…": "..."}


# Replace characters OpenCV's fonts cannot draw (see docstring).
def _ascii(s):
    """OpenCV's Hershey fonts are ASCII-only (anything else renders as '??')."""
    s = "".join(_ASCII.get(c, c) for c in str(s))
    return s.encode("ascii", "replace").decode()


# Text with a black outline so it stays readable on any background.
def _txt(img, s, org, color=(255, 255, 255), scale=0.5, th=1):
    s = _ascii(s)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, th, cv2.LINE_AA)


# Top-down plot image: the grid, optional workspace fence, each track (open circle =
# start, filled = end), title, legend and footer lines. The view auto-fits the grid
# plus all tracks with a 12 cm margin; nested P() maps metres -> image pixels.
def topdown(size, tracks, grid_dims=(1.0, 1.0), title=None, lines=(), workspace=None):
    """Top-down plot of tracks over the grid, auto-fitted. size = (w, h) pixels."""
    width_px, height_px = size
    img = np.full((height_px, width_px, 3), 24, np.uint8)
    grid_w_m, grid_h_m = grid_dims
    extent_points = [np.array([[0, 0], [grid_w_m, grid_h_m]], float)] + [track.xy for track in tracks if len(track.xy)]
    all_points = np.vstack(extent_points)
    view_min, view_max = all_points.min(axis=0) - 0.12, all_points.max(axis=0) + 0.12
    top_margin = 24 if title else 6
    bottom_margin = 8 + 17 * len(lines)
    scale = min((width_px - 12) / (view_max[0] - view_min[0]), (height_px - top_margin - bottom_margin) / (view_max[1] - view_min[1]))
    offset_x = 6 + ((width_px - 12) - scale * (view_max[0] - view_min[0])) / 2
    offset_y = top_margin + ((height_px - top_margin - bottom_margin) - scale * (view_max[1] - view_min[1])) / 2

    def P(xy):
        xy = np.atleast_2d(np.asarray(xy, float))
        return np.column_stack([offset_x + (xy[:, 0] - view_min[0]) * scale,
                                offset_y + (view_max[1] - xy[:, 1]) * scale]).astype(np.int32)

    if workspace is not None:
        (fence_x0, fence_x1), (fence_y0, fence_y1) = workspace
        cv2.rectangle(img, tuple(P([fence_x0, fence_y1])[0]), tuple(P([fence_x1, fence_y0])[0]), (60, 60, 90), 1)
    grid_step = 0.5 if max(grid_w_m, grid_h_m) > 1.01 else 0.25
    for x in np.arange(0, grid_w_m + 1e-6, grid_step):
        cv2.line(img, tuple(P([x, 0])[0]), tuple(P([x, grid_h_m])[0]), (55, 55, 55), 1)
    for y in np.arange(0, grid_h_m + 1e-6, grid_step):
        cv2.line(img, tuple(P([0, y])[0]), tuple(P([grid_w_m, y])[0]), (55, 55, 55), 1)
    cv2.rectangle(img, tuple(P([0, grid_h_m])[0]), tuple(P([grid_w_m, 0])[0]), (0, 170, 210), 1)
    for track in tracks:
        if len(track.xy) >= 2:
            cv2.polylines(img, [P(track.xy)], False, track.color, track.thickness, cv2.LINE_AA)
        if len(track.xy):
            cv2.circle(img, tuple(P(track.xy[0])[0]), 4, track.color, 1, cv2.LINE_AA)
            cv2.circle(img, tuple(P(track.xy[-1])[0]), 4, track.color, -1, cv2.LINE_AA)
    if title:
        _txt(img, title, (6, 17), (255, 255, 255), 0.5)
    y = 17 + (20 if title else 0)
    for track in tracks:
        if track.label:
            _txt(img, track.label, (8, y), track.color, 0.42)
            y += 16
    for line_index, line in enumerate(lines):
        _txt(img, line, (6, height_px - bottom_margin + 14 + 17 * line_index), (225, 225, 225), 0.42)
    return img


# Scale an image to fit inside w x h, centred on a dark canvas.
def _fit(img, w, h):
    """Letterbox img into a w x h canvas."""
    canvas = np.full((h, w, 3), 18, np.uint8)
    if img is None:
        _txt(canvas, "no camera image", (w // 2 - 80, h // 2), (150, 150, 150), 0.6)
        return canvas
    scale = min(w / img.shape[1], h / img.shape[0])
    resized = cv2.resize(img, (max(1, int(img.shape[1] * scale)), max(1, int(img.shape[0] * scale))),
                   interpolation=cv2.INTER_AREA)
    top, left = (h - resized.shape[0]) // 2, (w - resized.shape[1]) // 2
    canvas[top:top + resized.shape[0], left:left + resized.shape[1]] = resized
    return canvas


# The single session window, drawn by its own GUI thread. Other threads only set
# state (status, log, camera source, current plot, gallery tiles, parameter sheet);
# compose() turns that state into one 1600 x 900 frame.
class Dashboard:
    # Set up state and start the GUI thread. snapshot = live.jpg written every second;
    # record = mp4 of the window at record_fps, kept in real time.
    def __init__(self, title, grid_dims=(1.0, 1.0), show=True, snapshot=None, workspace=None,
                 fps=15.0, record=None, record_fps=8.0, record_size=(1280, 720)):
        self.title = title
        self.grid_dims = tuple(grid_dims)
        self.workspace = workspace
        self.show = bool(show and os.environ.get("DISPLAY"))
        self.snapshot = str(snapshot) if snapshot else None
        self.period = 1.0 / fps
        # session recording: the composed window -> one mp4 for the whole session
        self.record = str(record) if record else None
        self.record_period = 1.0 / record_fps
        self.record_fps, self.record_size = record_fps, tuple(record_size)
        self._last_frame = None
        self._rec_stop = threading.Event()
        self._rec_done = threading.Event()
        self._lock = threading.Lock()
        self._status, self._status_t = "starting", time.time()
        self._log = deque(maxlen=9)
        self._camera_fn = None
        self._current = None
        self._gallery = OrderedDict()
        self._params = None
        self._show_sheet = False
        self._holding = False
        self._released = threading.Event()
        self._running = True
        self.quit_requested = False
        self._thread = threading.Thread(target=self._loop, name="dashboard", daemon=True)
        self._thread.start()

    # ---------------- state (any thread) ----------------
    # Banner text (the clock beside it restarts at every new status).
    def status(self, text):
        with self._lock:
            self._status, self._status_t = str(text), time.time()

    # Add one line to the log ticker.
    def log(self, line):
        with self._lock:
            self._log.append(str(line))

    def camera_source(self, fn):
        """fn() -> BGR image (annotated camera view) or None; polled from the GUI thread."""
        with self._lock:
            self._camera_fn = fn

    # Big top-down panel: the current run (title, tracks, text lines).
    def current(self, title, tracks, lines=()):
        with self._lock:
            self._current = (title, list(tracks), list(lines))

    def result(self, key, title, tracks, lines=()):
        """Add / replace a finished test in the gallery (stays until replaced)."""
        with self._lock:
            self._gallery[key] = (title, list(tracks), list(lines))
            self._gallery.move_to_end(key)

    def params(self, sheet):
        """Parameter sheet from viz/param_sheet.build(): the fitted keys are always shown
        above the log; 'p' opens the full sheet."""
        with self._lock:
            self._params = sheet

    def stop_recording(self, timeout=5.0):
        """Finalise session_video.mp4 now (before any risky teardown); the window stays up."""
        if not self.record:
            return
        self._rec_stop.set()
        self._rec_done.wait(timeout)

    def save_frame(self, path):
        """Write the current composed window to an image (e.g. the final summary view)."""
        frame = self._last_frame if self._last_frame is not None else self.compose()
        cv2.imwrite(str(path), frame)

    def hold(self, text="finished - press q (or close the window) to exit"):
        """Block until the user dismisses the window (returns at once when not shown)."""
        self.status(text)
        if not self.show:
            return
        with self._lock:
            self._holding = True
        try:
            while not self._released.wait(0.2):
                if not self._thread.is_alive():
                    break
        except KeyboardInterrupt:
            pass

    def close(self):
        self._running = False
        self._thread.join(timeout=2.0)

    # ---------------- rendering (GUI thread) ----------------
    # Draw one frame: banner (title, status, elapsed time), camera panel (or the
    # parameter sheet), current top-down, the newest gallery tiles, the fitted-parameter
    # panel and the log.
    def compose(self):
        with self._lock:
            status, status_time = self._status, self._status_t
            log = list(self._log)
            camera_fn = self._camera_fn
            current_view = self._current
            gallery = list(self._gallery.items())
            sheet, show_sheet = self._params, self._show_sheet
        img = np.full((H, W, 3), 32, np.uint8)
        # banner
        cv2.rectangle(img, (0, 0), (W, BANNER), (48, 48, 48), -1)
        _txt(img, self.title, (12, 30), (0, 220, 255), 0.75, 2)
        title_w = cv2.getTextSize(_ascii(self.title), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0][0]
        _txt(img, status, (32 + title_w, 30), (255, 255, 255), 0.7, 2)
        elapsed_s = time.time() - status_time
        _txt(img, f"{int(elapsed_s // 60):02d}:{int(elapsed_s % 60):02d}", (W - 70, 30), (170, 170, 170), 0.6)
        # live camera
        camera_img = None
        if camera_fn is not None:
            try:
                camera_img = camera_fn()
            except Exception as exc:               # never let a drawing error kill the GUI
                camera_img = None
                log = log + [f"[view] camera panel: {exc}"]
        img[BANNER:BANNER + TOP_H, :CAM_W] = _fit(camera_img, CAM_W, TOP_H)
        if sheet is not None and show_sheet:
            img[BANNER:BANNER + TOP_H, :CAM_W] = param_table(sheet, CAM_W, TOP_H)
        # current test
        if current_view is not None:
            title, tracks, lines = current_view
            img[BANNER:BANNER + TOP_H, CAM_W:] = topdown((TOP_H, TOP_H), tracks, self.grid_dims,
                                                         title, lines, self.workspace)
        else:
            img[BANNER:BANNER + TOP_H, CAM_W:] = topdown((TOP_H, TOP_H), [], self.grid_dims,
                                                         "waiting for the first run", (),
                                                         self.workspace)
        cv2.line(img, (CAM_W, BANNER), (CAM_W, BANNER + TOP_H), (90, 90, 90), 1)
        # gallery of finished tests
        gallery_top = BANNER + TOP_H + 4
        gallery_h = H - gallery_top - 4
        gallery_w = W - LOG_W - 8
        if gallery:
            tile_count = len(gallery)
            tile_w = int(min(gallery_h, max(120, (gallery_w - 4) / tile_count - 6)))
            tiles_per_row = max(1, int((gallery_w - 4) // (tile_w + 6)))
            shown_tiles = gallery[-tiles_per_row:]
            for tile_index, (key, (title, tracks, lines)) in enumerate(shown_tiles):
                tile = topdown((tile_w, gallery_h), tracks, self.grid_dims, title, lines[:4])
                tile_x = 4 + tile_index * (tile_w + 6)
                img[gallery_top:gallery_top + gallery_h, tile_x:tile_x + tile_w] = tile
            if tile_count > tiles_per_row:
                _txt(img, f"+{tile_count - tiles_per_row} earlier", (8, H - 8), (170, 170, 170), 0.42)
        else:
            _txt(img, "finished tests appear here (real vs sim, metrics)",
                 (12, gallery_top + 26), (150, 150, 150), 0.5)
        # fitted parameters (top of the right column) + log ticker below
        log_x = W - LOG_W
        cv2.rectangle(img, (log_x, gallery_top), (W - 4, H - 4), (20, 20, 20), -1)
        log_y = gallery_top
        if sheet is not None:
            log_y = fitted_panel(img, sheet, log_x, gallery_top, LOG_W - 4, H - 4 - gallery_top)
        for tile_index, line in enumerate(log[-max(0, (H - 14 - log_y) // 19):] if H - 14 - log_y >= 19 else []):
            _txt(img, line[:70], (log_x + 8, log_y + 18 + 19 * tile_index), (205, 205, 205), 0.4)
        return img

    # GUI thread: compose ~fps frames/s, write live.jpg and the video, show the window
    # and handle keys (p = parameter sheet, q / Esc / closing = stop the run via SIGINT).
    def _loop(self):
        window_name = self.title
        created = False
        last_snap = 0.0
        video_writer = None
        record_start, frames_written = None, 0  # frames are due by the CLOCK: the video keeps real time
        seen_visible = False     # GTK reports a new window invisible until the WM maps it
        while self._running:
            frame_start = time.time()
            try:
                frame = self.compose()
            except Exception as exc:
                frame = np.zeros((H, W, 3), np.uint8)
                _txt(frame, f"dashboard error: {exc}", (20, 60), (0, 0, 255), 0.7)
            self._last_frame = frame
            if self.snapshot and frame_start - last_snap >= 1.0:
                cv2.imwrite(self.snapshot, frame)
                last_snap = frame_start
            if self._rec_stop.is_set() and not self._rec_done.is_set():
                if video_writer is not None:
                    video_writer.release()
                    video_writer = None
                self._rec_done.set()
            if self.record and not self._rec_stop.is_set():
                if video_writer is None:
                    video_writer = cv2.VideoWriter(self.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                             self.record_fps, self.record_size)
                    record_start = frame_start
                # Writing one frame per loop pass only when a period had elapsed aliased
                # (an 88 ms loop gave 5.7 fps into an 8 fps file) and lost whole stretches
                # when the loop was slow: real sessions came out 2-70 % of their length.
                # Repeat the current frame for every period that has passed instead.
                frames_due = int((frame_start - record_start) * self.record_fps) + 1 - frames_written
                if frames_due > 0:
                    small_frame = cv2.resize(frame, self.record_size, interpolation=cv2.INTER_AREA)
                    for _ in range(min(frames_due, int(600 * self.record_fps))):
                        video_writer.write(small_frame)
                    frames_written += frames_due
            if self.show:
                if not created:
                    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(window_name, W, H)
                    created = True
                cv2.imshow(window_name, frame)
                pressed_key = cv2.waitKey(max(1, int(self.period * 1000))) & 0xFF
                visible = cv2.getWindowProperty(window_name, cv2.WND_PROP_VISIBLE) >= 1
                seen_visible = seen_visible or visible
                closed = seen_visible and not visible
                if pressed_key == ord("p"):
                    with self._lock:
                        self._show_sheet = not self._show_sheet
                if pressed_key in (ord("q"), 27) or closed:
                    with self._lock:
                        holding = self._holding
                    if holding:
                        self._released.set()
                        break
                    if not self.quit_requested:
                        self.quit_requested = True
                        self.log("[view] quit requested - stopping the rovers")
                        os.kill(os.getpid(), signal.SIGINT)
                    if closed:                  # re-open so the shutdown stays visible
                        created = seen_visible = False
            else:
                needs_frames = self.snapshot or self.record
                time.sleep(max(0.0, (self.period if needs_frames else 0.5) - (time.time() - frame_start)))
        if video_writer is not None:
            video_writer.release()
        self._rec_done.set()
        if created:
            try:
                cv2.destroyWindow(window_name)
                cv2.waitKey(1)
            except cv2.error:
                pass
        self._released.set()


# ───────────────────────── parameter panels ─────────────────────────
# Position of v between lo and hi as 0..1 (logarithmic if scale == 'log').
def _frac(v, lo, hi, scale):
    if v is None or lo is None or hi is None or v != v or hi <= lo:
        return None
    if scale == "log" and lo > 0 and v > 0:
        return (math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo))
    return (v - lo) / (hi - lo)


# Range bar for one fitted parameter (see docstring).
def _bar(img, x, y, w, r):
    """Range bar: full bounds, shaded trust region, start tick, candidate (grey) and
    best (blue) dots, current value (white diamond)."""
    low, high, scale_kind = r.get("lo"), r.get("hi"), r.get("scale", "lin")
    cv2.line(img, (x, y), (x + w, y), (90, 90, 90), 1, cv2.LINE_AA)
    if low is None:
        return

    def X(v):
        f = _frac(v, low, high, scale_kind)
        return None if f is None else int(x + max(0.0, min(1.0, f)) * w)

    trust_range = r.get("trust")
    if trust_range:
        trust_left_px, trust_right_px = X(trust_range[0]), X(trust_range[1])
        if trust_left_px is not None and trust_right_px is not None:
            cv2.rectangle(img, (trust_left_px, y - 4), (trust_right_px, y + 4), (70, 70, 40), -1)
    for v, colour in ((r.get("start"), (150, 150, 150)),):
        marker_x = X(v)
        if marker_x is not None:
            cv2.line(img, (marker_x, y - 7), (marker_x, y + 7), colour, 1)
    marker_x = X(r.get("cand"))
    if marker_x is not None:
        cv2.circle(img, (marker_x, y), 4, CANDIDATE, -1, cv2.LINE_AA)
    marker_x = X(r.get("best"))
    if marker_x is not None:
        cv2.circle(img, (marker_x, y), 5, AFTER, -1, cv2.LINE_AA)
    marker_x = X(r.get("value"))
    if marker_x is not None:
        cv2.drawMarker(img, (marker_x, y), (255, 255, 255), cv2.MARKER_DIAMOND, 9, 1, cv2.LINE_AA)
    _txt(img, _fmt(low), (x, y + 17), (130, 130, 130), 0.35)
    text_w = cv2.getTextSize(_fmt(high), cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0][0]
    _txt(img, _fmt(high), (x + w - text_w, y + 17), (130, 130, 130), 0.35)


# Short number format shared with param_sheet.
def _fmt(v):
    from param_sheet import fmt
    return fmt(v)


# 'SIM PARAMETERS BEING FITTED' block: per key its value, start, best and range bar.
def fitted_panel(img, sheet, x, y, w, h_max):
    """'Parameters being fitted' block; returns the y where the log can start."""
    fitted_rows = sheet.get("changing") or []
    header = sheet.get("header") or ""
    title = "SIM PARAMETERS BEING FITTED" if fitted_rows else "SIM PARAMETERS: fixed this run"
    _txt(img, title, (x + 8, y + 18), (0, 220, 255), 0.45, 1)
    _txt(img, "[p] all parameters", (x + w - 150, y + 18), (150, 150, 150), 0.4)
    cursor_y = y + 24
    if header:
        _txt(img, header[:70], (x + 8, cursor_y + 14), (180, 180, 180), 0.4)
        cursor_y += 18
    row_h = 46
    rows_that_fit = max(1, int((h_max * 0.62 - (cursor_y - y)) // row_h))
    for row in fitted_rows[:rows_that_fit]:
        unit = f" {row['unit']}" if row.get("unit") else ""
        line = f"{row['key']}  {_fmt(row['value'])}{unit}"
        if row.get("start") is not None:
            line += f"   start {_fmt(row['start'])}"
        if row.get("best") is not None:
            line += f"  best {_fmt(row['best'])}"
        _txt(img, line[:66], (x + 8, cursor_y + 14), (255, 255, 255), 0.42)
        _bar(img, x + 12, cursor_y + 26, w - 40, row)
        cursor_y += row_h
    if len(fitted_rows) > rows_that_fit:
        _txt(img, f"+{len(fitted_rows) - rows_that_fit} more fitted keys - press p", (x + 8, cursor_y + 12),
             (170, 170, 170), 0.4)
        cursor_y += 18
    cv2.line(img, (x + 4, cursor_y + 4), (x + w - 4, cursor_y + 4), (70, 70, 70), 1)
    return cursor_y + 6


# Full parameter sheet (see docstring).
def param_table(sheet, w, h):
    """Full parameter sheet (toggled with 'p'), drawn over the camera panel."""
    img = np.full((h, w, 3), 22, np.uint8)
    _txt(img, "SIM PARAMETERS   (p closes)", (12, 24), (0, 220, 255), 0.6, 2)
    if sheet.get("header"):
        _txt(img, sheet["header"][:90], (330, 24), (180, 180, 180), 0.45)
    y = 46
    column_x = (14, 250, 360, 470, 575, 680)

    def heading(t):
        nonlocal y
        y += 6
        _txt(img, t, (column_x[0], y + 12), (0, 200, 255), 0.47, 1)
        y += 20

    fitted_rows = sheet.get("changing") or []
    if fitted_rows:
        heading("Being fitted this session")
        for column, column_label in enumerate(("parameter", "current", "start", "best", "candidate",
                                 "bounds / history")):
            _txt(img, column_label, (column_x[column], y + 10), (130, 130, 130), 0.38)
        y += 16
        for row in fitted_rows:
            unit = f" {row['unit']}" if row.get("unit") else ""
            cells = [row["key"], _fmt(row["value"]) + unit, _fmt(row.get("start")), _fmt(row.get("best")),
                     _fmt(row.get("cand"))]
            for column, cell in enumerate(cells):
                _txt(img, cell, (column_x[column], y + 12), (255, 255, 255) if column else (210, 230, 255), 0.42)
            history = row.get("history") or []
            history_tail = " > ".join(_fmt(v) for v in history[-4:])
            _txt(img, f"[{_fmt(row.get('lo'))}, {_fmt(row.get('hi'))}]  {history_tail}"[:58],
                 (column_x[5], y + 12), (190, 190, 190), 0.4)
            y += 18
            if row.get("derived"):
                _txt(img, "   " + row["derived"], (column_x[0], y + 12), (170, 210, 170), 0.4)
                y += 18
    for title, group_rows in sheet.get("groups", []):
        if y > h - 40:
            break
        heading(title)
        first_column_count = (len(group_rows) + 1) // 2 if len(group_rows) > 6 else len(group_rows)
        for row_index, row in enumerate(group_rows):
            cell_x = 14 if row_index < first_column_count else w // 2 + 4
            row_y = y + 18 * (row_index if row_index < first_column_count else row_index - first_column_count)
            if row_y > h - 20:
                continue
            unit = f" {row['unit']}" if row.get("unit") else ""
            _txt(img, row["key"], (cell_x, row_y + 12), (225, 225, 225), 0.4)
            _txt(img, f"{_fmt(row['value'])}{unit}", (cell_x + 200, row_y + 12), (255, 255, 255), 0.4)
            if row.get("lo") is not None:
                _txt(img, f"[{_fmt(row['lo'])}, {_fmt(row['hi'])}]", (cell_x + 310, row_y + 12),
                     (150, 150, 150), 0.38)
        y += 18 * first_column_count
        for row in group_rows:
            if row.get("derived") and y < h - 20:
                _txt(img, "   " + row["key"] + ": " + row["derived"], (14, y + 12), (170, 210, 170), 0.4)
                y += 18
    return img


# Move a start-relative track so it starts at `start` (x, y, yaw): rotate/translate.
def place_at(rows, start):
    """Re-express a start-relative trajectory (t, x, y, yaw rows) at start=(x, y, yaw)."""
    rows_arr = np.asarray(rows, float)
    if not len(rows_arr):
        return rows_arr
    sim_x0, sim_y0, sim_yaw0 = rows_arr[0, 1], rows_arr[0, 2], rows_arr[0, 3]
    cos_r, sin_r = math.cos(-sim_yaw0), math.sin(-sim_yaw0)
    rel_x = cos_r * (rows_arr[:, 1] - sim_x0) - sin_r * (rows_arr[:, 2] - sim_y0)
    rel_y = sin_r * (rows_arr[:, 1] - sim_x0) + cos_r * (rows_arr[:, 2] - sim_y0)
    start_x, start_y, start_yaw = start
    cos_start, sin_start = math.cos(start_yaw), math.sin(start_yaw)
    placed = rows_arr.copy()
    placed[:, 1] = start_x + cos_start * rel_x - sin_start * rel_y
    placed[:, 2] = start_y + sin_start * rel_x + cos_start * rel_y
    placed[:, 3] = (rows_arr[:, 3] - sim_yaw0 + start_yaw + math.pi) % (2 * math.pi) - math.pi
    return placed


# Gallery text lines from a compare_paths.py metrics dict.
def metric_lines(m):
    """compare_paths.py metrics.json -> short gallery lines."""
    if not m:
        return ["no comparison (too few samples)"]
    return [f"endpoint {m['endpoint_err_m'] * 100:.1f} cm   heading {m['heading_err_deg']:.1f} deg",
            f"mean {m['mean_err_m'] * 100:.1f} cm   max {m['max_err_m'] * 100:.1f} cm",
            f"length real {m['path_len_real_m']:.2f} / sim {m['path_len_sim_m']:.2f} m"]
