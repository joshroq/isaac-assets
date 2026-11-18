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


@dataclass
class Track:
    label: str
    pts: object                   # (N, 2|3|4) grid metres: x, y[, yaw] or t, x, y, yaw
    color: tuple
    thickness: int = 2
    xy: np.ndarray = field(init=False, repr=False)

    def __post_init__(self):
        a = np.asarray(self.pts, float)
        if a.ndim != 2 or len(a) == 0:
            self.xy = np.zeros((0, 2))
        elif a.shape[1] == 4:                      # t, x, y, yaw (trajectory CSV rows)
            self.xy = a[:, 1:3]
        else:
            self.xy = a[:, :2]


_ASCII = {"±": "+/-", "°": " deg", "→": "->", "—": "-", "–": "-", "×": "x", "≈": "~",
          "σ": "sigma", "…": "..."}


def _ascii(s):
    """OpenCV's Hershey fonts are ASCII-only (anything else renders as '??')."""
    s = "".join(_ASCII.get(c, c) for c in str(s))
    return s.encode("ascii", "replace").decode()


def _txt(img, s, org, color=(255, 255, 255), scale=0.5, th=1):
    s = _ascii(s)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), th + 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, th, cv2.LINE_AA)


def topdown(size, tracks, grid_dims=(1.0, 1.0), title=None, lines=(), workspace=None):
    """Top-down plot of tracks over the grid, auto-fitted. size = (w, h) pixels."""
    w, h = size
    img = np.full((h, w, 3), 24, np.uint8)
    gw, gh = grid_dims
    pts = [np.array([[0, 0], [gw, gh]], float)] + [t.xy for t in tracks if len(t.xy)]
    allp = np.vstack(pts)
    lo, hi = allp.min(axis=0) - 0.12, allp.max(axis=0) + 0.12
    top = 24 if title else 6
    bottom = 8 + 17 * len(lines)
    s = min((w - 12) / (hi[0] - lo[0]), (h - top - bottom) / (hi[1] - lo[1]))
    ox = 6 + ((w - 12) - s * (hi[0] - lo[0])) / 2
    oy = top + ((h - top - bottom) - s * (hi[1] - lo[1])) / 2

    def P(xy):
        xy = np.atleast_2d(np.asarray(xy, float))
        return np.column_stack([ox + (xy[:, 0] - lo[0]) * s,
                                oy + (hi[1] - xy[:, 1]) * s]).astype(np.int32)

    if workspace is not None:
        (x0, x1), (y0, y1) = workspace
        cv2.rectangle(img, tuple(P([x0, y1])[0]), tuple(P([x1, y0])[0]), (60, 60, 90), 1)
    step = 0.5 if max(gw, gh) > 1.01 else 0.25
    for x in np.arange(0, gw + 1e-6, step):
        cv2.line(img, tuple(P([x, 0])[0]), tuple(P([x, gh])[0]), (55, 55, 55), 1)
    for y in np.arange(0, gh + 1e-6, step):
        cv2.line(img, tuple(P([0, y])[0]), tuple(P([gw, y])[0]), (55, 55, 55), 1)
    cv2.rectangle(img, tuple(P([0, gh])[0]), tuple(P([gw, 0])[0]), (0, 170, 210), 1)
    for t in tracks:
        if len(t.xy) >= 2:
            cv2.polylines(img, [P(t.xy)], False, t.color, t.thickness, cv2.LINE_AA)
        if len(t.xy):
            cv2.circle(img, tuple(P(t.xy[0])[0]), 4, t.color, 1, cv2.LINE_AA)
            cv2.circle(img, tuple(P(t.xy[-1])[0]), 4, t.color, -1, cv2.LINE_AA)
    if title:
        _txt(img, title, (6, 17), (255, 255, 255), 0.5)
    y = 17 + (20 if title else 0)
    for t in tracks:
        if t.label:
            _txt(img, t.label, (8, y), t.color, 0.42)
            y += 16
    for i, line in enumerate(lines):
        _txt(img, line, (6, h - bottom + 14 + 17 * i), (225, 225, 225), 0.42)
    return img


def _fit(img, w, h):
    """Letterbox img into a w x h canvas."""
    out = np.full((h, w, 3), 18, np.uint8)
    if img is None:
        _txt(out, "no camera image", (w // 2 - 80, h // 2), (150, 150, 150), 0.6)
        return out
    s = min(w / img.shape[1], h / img.shape[0])
    r = cv2.resize(img, (max(1, int(img.shape[1] * s)), max(1, int(img.shape[0] * s))),
                   interpolation=cv2.INTER_AREA)
    y0, x0 = (h - r.shape[0]) // 2, (w - r.shape[1]) // 2
    out[y0:y0 + r.shape[0], x0:x0 + r.shape[1]] = r
    return out


class Dashboard:
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
    def status(self, text):
        with self._lock:
            self._status, self._status_t = str(text), time.time()

    def log(self, line):
        with self._lock:
            self._log.append(str(line))

    def camera_source(self, fn):
        """fn() -> BGR image (annotated camera view) or None; polled from the GUI thread."""
        with self._lock:
            self._camera_fn = fn

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
    def compose(self):
        with self._lock:
            status, st_t = self._status, self._status_t
            log = list(self._log)
            fn = self._camera_fn
            cur = self._current
            gallery = list(self._gallery.items())
            sheet, show_sheet = self._params, self._show_sheet
        img = np.full((H, W, 3), 32, np.uint8)
        # banner
        cv2.rectangle(img, (0, 0), (W, BANNER), (48, 48, 48), -1)
        _txt(img, self.title, (12, 30), (0, 220, 255), 0.75, 2)
        tw = cv2.getTextSize(_ascii(self.title), cv2.FONT_HERSHEY_SIMPLEX, 0.75, 2)[0][0]
        _txt(img, status, (32 + tw, 30), (255, 255, 255), 0.7, 2)
        el = time.time() - st_t
        _txt(img, f"{int(el // 60):02d}:{int(el % 60):02d}", (W - 70, 30), (170, 170, 170), 0.6)
        # live camera
        cam = None
        if fn is not None:
            try:
                cam = fn()
            except Exception as exc:               # never let a drawing error kill the GUI
                cam = None
                log = log + [f"[view] camera panel: {exc}"]
        img[BANNER:BANNER + TOP_H, :CAM_W] = _fit(cam, CAM_W, TOP_H)
        if sheet is not None and show_sheet:
            img[BANNER:BANNER + TOP_H, :CAM_W] = param_table(sheet, CAM_W, TOP_H)
        # current test
        if cur is not None:
            title, tracks, lines = cur
            img[BANNER:BANNER + TOP_H, CAM_W:] = topdown((TOP_H, TOP_H), tracks, self.grid_dims,
                                                         title, lines, self.workspace)
        else:
            img[BANNER:BANNER + TOP_H, CAM_W:] = topdown((TOP_H, TOP_H), [], self.grid_dims,
                                                         "waiting for the first run", (),
                                                         self.workspace)
        cv2.line(img, (CAM_W, BANNER), (CAM_W, BANNER + TOP_H), (90, 90, 90), 1)
        # gallery of finished tests
        gy = BANNER + TOP_H + 4
        gh = H - gy - 4
        gw_total = W - LOG_W - 8
        if gallery:
            n = len(gallery)
            tw_ = int(min(gh, max(120, (gw_total - 4) / n - 6)))
            per_row = max(1, int((gw_total - 4) // (tw_ + 6)))
            shown = gallery[-per_row:]
            for i, (key, (title, tracks, lines)) in enumerate(shown):
                tile = topdown((tw_, gh), tracks, self.grid_dims, title, lines[:4])
                x = 4 + i * (tw_ + 6)
                img[gy:gy + gh, x:x + tw_] = tile
            if n > per_row:
                _txt(img, f"+{n - per_row} earlier", (8, H - 8), (170, 170, 170), 0.42)
        else:
            _txt(img, "finished tests appear here (real vs sim, metrics)",
                 (12, gy + 26), (150, 150, 150), 0.5)
        # fitted parameters (top of the right column) + log ticker below
        lx = W - LOG_W
        cv2.rectangle(img, (lx, gy), (W - 4, H - 4), (20, 20, 20), -1)
        ly = gy
        if sheet is not None:
            ly = fitted_panel(img, sheet, lx, gy, LOG_W - 4, H - 4 - gy)
        for i, line in enumerate(log[-max(0, (H - 14 - ly) // 19):] if H - 14 - ly >= 19 else []):
            _txt(img, line[:70], (lx + 8, ly + 18 + 19 * i), (205, 205, 205), 0.4)
        return img

    def _loop(self):
        win = self.title
        created = False
        last_snap = 0.0
        writer = None
        rec_t0, rec_n = None, 0  # frames are due by the CLOCK: the video keeps real time
        seen_visible = False     # GTK reports a new window invisible until the WM maps it
        while self._running:
            t0 = time.time()
            try:
                frame = self.compose()
            except Exception as exc:
                frame = np.zeros((H, W, 3), np.uint8)
                _txt(frame, f"dashboard error: {exc}", (20, 60), (0, 0, 255), 0.7)
            self._last_frame = frame
            if self.snapshot and t0 - last_snap >= 1.0:
                cv2.imwrite(self.snapshot, frame)
                last_snap = t0
            if self._rec_stop.is_set() and not self._rec_done.is_set():
                if writer is not None:
                    writer.release()
                    writer = None
                self._rec_done.set()
            if self.record and not self._rec_stop.is_set():
                if writer is None:
                    writer = cv2.VideoWriter(self.record, cv2.VideoWriter_fourcc(*"mp4v"),
                                             self.record_fps, self.record_size)
                    rec_t0 = t0
                # Writing one frame per loop pass only when a period had elapsed aliased
                # (an 88 ms loop gave 5.7 fps into an 8 fps file) and lost whole stretches
                # when the loop was slow: real sessions came out 2-70 % of their length.
                # Repeat the current frame for every period that has passed instead.
                due = int((t0 - rec_t0) * self.record_fps) + 1 - rec_n
                if due > 0:
                    small = cv2.resize(frame, self.record_size, interpolation=cv2.INTER_AREA)
                    for _ in range(min(due, int(600 * self.record_fps))):
                        writer.write(small)
                    rec_n += due
            if self.show:
                if not created:
                    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
                    cv2.resizeWindow(win, W, H)
                    created = True
                cv2.imshow(win, frame)
                k = cv2.waitKey(max(1, int(self.period * 1000))) & 0xFF
                visible = cv2.getWindowProperty(win, cv2.WND_PROP_VISIBLE) >= 1
                seen_visible = seen_visible or visible
                closed = seen_visible and not visible
                if k == ord("p"):
                    with self._lock:
                        self._show_sheet = not self._show_sheet
                if k in (ord("q"), 27) or closed:
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
                busy = self.snapshot or self.record
                time.sleep(max(0.0, (self.period if busy else 0.5) - (time.time() - t0)))
        if writer is not None:
            writer.release()
        self._rec_done.set()
        if created:
            try:
                cv2.destroyWindow(win)
                cv2.waitKey(1)
            except cv2.error:
                pass
        self._released.set()


# ───────────────────────── parameter panels ─────────────────────────
def _frac(v, lo, hi, scale):
    if v is None or lo is None or hi is None or v != v or hi <= lo:
        return None
    if scale == "log" and lo > 0 and v > 0:
        return (math.log(v) - math.log(lo)) / (math.log(hi) - math.log(lo))
    return (v - lo) / (hi - lo)


def _bar(img, x, y, w, r):
    """Range bar: full bounds, shaded trust region, start tick, candidate (grey) and
    best (blue) dots, current value (white diamond)."""
    lo, hi, sc = r.get("lo"), r.get("hi"), r.get("scale", "lin")
    cv2.line(img, (x, y), (x + w, y), (90, 90, 90), 1, cv2.LINE_AA)
    if lo is None:
        return

    def X(v):
        f = _frac(v, lo, hi, sc)
        return None if f is None else int(x + max(0.0, min(1.0, f)) * w)

    tr = r.get("trust")
    if tr:
        a, b = X(tr[0]), X(tr[1])
        if a is not None and b is not None:
            cv2.rectangle(img, (a, y - 4), (b, y + 4), (70, 70, 40), -1)
    for v, col in ((r.get("start"), (150, 150, 150)),):
        px = X(v)
        if px is not None:
            cv2.line(img, (px, y - 7), (px, y + 7), col, 1)
    px = X(r.get("cand"))
    if px is not None:
        cv2.circle(img, (px, y), 4, CANDIDATE, -1, cv2.LINE_AA)
    px = X(r.get("best"))
    if px is not None:
        cv2.circle(img, (px, y), 5, AFTER, -1, cv2.LINE_AA)
    px = X(r.get("value"))
    if px is not None:
        cv2.drawMarker(img, (px, y), (255, 255, 255), cv2.MARKER_DIAMOND, 9, 1, cv2.LINE_AA)
    _txt(img, _fmt(lo), (x, y + 17), (130, 130, 130), 0.35)
    tw = cv2.getTextSize(_fmt(hi), cv2.FONT_HERSHEY_SIMPLEX, 0.35, 1)[0][0]
    _txt(img, _fmt(hi), (x + w - tw, y + 17), (130, 130, 130), 0.35)


def _fmt(v):
    from param_sheet import fmt
    return fmt(v)


def fitted_panel(img, sheet, x, y, w, h_max):
    """'Parameters being fitted' block; returns the y where the log can start."""
    rows = sheet.get("changing") or []
    head = sheet.get("header") or ""
    title = "SIM PARAMETERS BEING FITTED" if rows else "SIM PARAMETERS: fixed this run"
    _txt(img, title, (x + 8, y + 18), (0, 220, 255), 0.45, 1)
    _txt(img, "[p] all parameters", (x + w - 150, y + 18), (150, 150, 150), 0.4)
    yy = y + 24
    if head:
        _txt(img, head[:70], (x + 8, yy + 14), (180, 180, 180), 0.4)
        yy += 18
    per = 46
    room = max(1, int((h_max * 0.62 - (yy - y)) // per))
    for r in rows[:room]:
        unit = f" {r['unit']}" if r.get("unit") else ""
        line = f"{r['key']}  {_fmt(r['value'])}{unit}"
        if r.get("start") is not None:
            line += f"   start {_fmt(r['start'])}"
        if r.get("best") is not None:
            line += f"  best {_fmt(r['best'])}"
        _txt(img, line[:66], (x + 8, yy + 14), (255, 255, 255), 0.42)
        _bar(img, x + 12, yy + 26, w - 40, r)
        yy += per
    if len(rows) > room:
        _txt(img, f"+{len(rows) - room} more fitted keys - press p", (x + 8, yy + 12),
             (170, 170, 170), 0.4)
        yy += 18
    cv2.line(img, (x + 4, yy + 4), (x + w - 4, yy + 4), (70, 70, 70), 1)
    return yy + 6


def param_table(sheet, w, h):
    """Full parameter sheet (toggled with 'p'), drawn over the camera panel."""
    img = np.full((h, w, 3), 22, np.uint8)
    _txt(img, "SIM PARAMETERS   (p closes)", (12, 24), (0, 220, 255), 0.6, 2)
    if sheet.get("header"):
        _txt(img, sheet["header"][:90], (330, 24), (180, 180, 180), 0.45)
    y = 46
    cols = (14, 250, 360, 470, 575, 680)

    def heading(t):
        nonlocal y
        y += 6
        _txt(img, t, (cols[0], y + 12), (0, 200, 255), 0.47, 1)
        y += 20

    rows = sheet.get("changing") or []
    if rows:
        heading("Being fitted this session")
        for i, lab in enumerate(("parameter", "current", "start", "best", "candidate",
                                 "bounds / history")):
            _txt(img, lab, (cols[i], y + 10), (130, 130, 130), 0.38)
        y += 16
        for r in rows:
            u = f" {r['unit']}" if r.get("unit") else ""
            cells = [r["key"], _fmt(r["value"]) + u, _fmt(r.get("start")), _fmt(r.get("best")),
                     _fmt(r.get("cand"))]
            for i, c in enumerate(cells):
                _txt(img, c, (cols[i], y + 12), (255, 255, 255) if i else (210, 230, 255), 0.42)
            hist = r.get("history") or []
            tail = " > ".join(_fmt(v) for v in hist[-4:])
            _txt(img, f"[{_fmt(r.get('lo'))}, {_fmt(r.get('hi'))}]  {tail}"[:58],
                 (cols[5], y + 12), (190, 190, 190), 0.4)
            y += 18
            if r.get("derived"):
                _txt(img, "   " + r["derived"], (cols[0], y + 12), (170, 210, 170), 0.4)
                y += 18
    for title, grp in sheet.get("groups", []):
        if y > h - 40:
            break
        heading(title)
        half = (len(grp) + 1) // 2 if len(grp) > 6 else len(grp)
        for j, r in enumerate(grp):
            cx = 14 if j < half else w // 2 + 4
            ry = y + 18 * (j if j < half else j - half)
            if ry > h - 20:
                continue
            u = f" {r['unit']}" if r.get("unit") else ""
            _txt(img, r["key"], (cx, ry + 12), (225, 225, 225), 0.4)
            _txt(img, f"{_fmt(r['value'])}{u}", (cx + 200, ry + 12), (255, 255, 255), 0.4)
            if r.get("lo") is not None:
                _txt(img, f"[{_fmt(r['lo'])}, {_fmt(r['hi'])}]", (cx + 310, ry + 12),
                     (150, 150, 150), 0.38)
        y += 18 * half
        for r in grp:
            if r.get("derived") and y < h - 20:
                _txt(img, "   " + r["key"] + ": " + r["derived"], (14, y + 12), (170, 210, 170), 0.4)
                y += 18
    return img


def place_at(rows, start):
    """Re-express a start-relative trajectory (t, x, y, yaw rows) at start=(x, y, yaw)."""
    s = np.asarray(rows, float)
    if not len(s):
        return s
    x0, y0, a0 = s[0, 1], s[0, 2], s[0, 3]
    c, sn = math.cos(-a0), math.sin(-a0)
    rx = c * (s[:, 1] - x0) - sn * (s[:, 2] - y0)
    ry = sn * (s[:, 1] - x0) + c * (s[:, 2] - y0)
    X0, Y0, A0 = start
    C0, S0 = math.cos(A0), math.sin(A0)
    out = s.copy()
    out[:, 1] = X0 + C0 * rx - S0 * ry
    out[:, 2] = Y0 + S0 * rx + C0 * ry
    out[:, 3] = (s[:, 3] - a0 + A0 + math.pi) % (2 * math.pi) - math.pi
    return out


def metric_lines(m):
    """compare_paths.py metrics.json -> short gallery lines."""
    if not m:
        return ["no comparison (too few samples)"]
    return [f"endpoint {m['endpoint_err_m'] * 100:.1f} cm   heading {m['heading_err_deg']:.1f} deg",
            f"mean {m['mean_err_m'] * 100:.1f} cm   max {m['max_err_m'] * 100:.1f} cm",
            f"length real {m['path_len_real_m']:.2f} / sim {m['path_len_sim_m']:.2f} m"]
