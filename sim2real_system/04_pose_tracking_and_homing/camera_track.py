#!/usr/bin/env python3
"""Overhead-camera rover pose in grid metres, tracked in a background thread.

Wraps the existing pieces (capture/stream.LatestFrame, calib/sticker_tracker,
viz/metric_overlay) so the loop's homing controller and trial recorder share one
UDP-5000 consumer:

    cam = CameraTrack()             # opens the stream, waits for a frame
    p = cam.pose(max_age=0.4)       # (t, x, y, yaw_rad) or None if stale/lost
    cam.start_recording("trial.mp4"); ...; rows, frame_ts = cam.stop_recording()
    img = cam.overlay()             # latest frame with grid + tracking drawn

Pose t is the desktop time the frame was decoded (time.time()).

Occlusion fallback: near the edges of the view the rover's own hardware can hide one
sticker from the (off-centre) camera, which makes the 3-sticker fit fail. Any TWO
colour-labelled stickers still fix the planar pose, so when the full fit fails a
2-sticker fit is accepted - but only if it is consistent with a 3-sticker pose seen
within the last couple of seconds (continuity gate), so a stray blob can't teleport it.
"""
from __future__ import annotations

import math
import sys
import threading
import time
from pathlib import Path

SC = Path(__file__).resolve().parents[1]
for sub in ("calib", "capture", "viz"):
    sys.path.insert(0, str(SC / sub))

from metric_overlay import (load_calib, draw_metric_grid, draw_axes_and_labels,  # noqa: E402
                            detect_rover, _txt)
from sticker_tracker import (StickerTracker, DEFAULT_CONFIG, draw_tracking,  # noqa: E402
                             fit_pose, transform_points)
from stream import LatestFrame, STREAM_URL  # noqa: E402

GRID_CALIB = SC / "calib" / "grid_calib.json"


class CameraError(RuntimeError):
    pass


class CameraTrack:
    def __init__(self, url=STREAM_URL, grid_calib=GRID_CALIB, stickers=DEFAULT_CONFIG,
                 allow_approximate=False, first_frame_timeout=20.0, source=None):
        self.H_img2m, self.H_m2img, self.grid_m = load_calib(str(grid_calib))
        self.tracker = StickerTracker(str(stickers))
        if not allow_approximate:
            self.tracker.require_measurement_ready()
        # source: anything with LatestFrame's opened/read/wait_first/release API
        # (e.g. a recorded clip replayed at camera speed); default = the live stream
        self.cam = source if source is not None else LatestFrame(url)
        if not self.cam.opened() or self.cam.wait_first(first_frame_timeout) is None:
            self.cam.release()
            raise CameraError("no overhead camera stream on UDP 5000 (Pi streaming? "
                              "another receiver holding the port?)")
        self._lock = threading.Lock()
        self._pose = None           # (t, x, y, yaw)
        self._result = None
        self._frame = None
        self._rec = None
        self._video = None          # (path, writer|None, [frame stamps])
        self.n_frames = self.n_detect = self.n_partial = 0
        self.latency_s = 0.0        # camera lag (set from loop_config fusion.cam_latency_s)
        self._last_full = None      # (t, center, yaw) of the last accepted pose
        self._running = True
        self._thread = threading.Thread(target=self._loop, daemon=True)
        self._thread.start()

    def _loop(self):
        last_n = -1
        while self._running:
            frame, stamp, n = self.cam.read()
            if frame is None or n == last_n:
                time.sleep(0.003)
                continue
            last_n = n
            try:
                r = detect_rover(frame, self.H_img2m, self.tracker)
                if r is not None:
                    self._last_full = (stamp, r["center_m"], math.radians(r["heading_deg"]))
                else:
                    r = self._partial(stamp)
            except ValueError as exc:
                self.tracker.status = str(exc)
                r = None
            with self._lock:
                self.n_frames += 1
                self._frame, self._result = frame, r
                if self._video is not None:
                    path, writer, stamps = self._video
                    if writer is None:
                        import cv2
                        writer = cv2.VideoWriter(str(path), cv2.VideoWriter_fourcc(*"mp4v"),
                                                 30.0, (frame.shape[1], frame.shape[0]))
                        self._video = (path, writer, stamps)
                    writer.write(frame)
                    stamps.append(stamp)
                if r is not None:
                    self.n_detect += 1
                    yaw = math.radians(r["heading_deg"])
                    self._pose = (stamp, float(r["center_m"][0]), float(r["center_m"][1]), yaw)
                    if self._rec is not None:
                        self._rec.append(self._pose)

    def _partial(self, stamp, max_gap_s=2.5, max_jump_m=0.12, max_turn_deg=35.0):
        """2-sticker pose from the candidates of the failed full detection."""
        import numpy as np
        tr = self.tracker
        gated = self._last_full is not None and stamp - self._last_full[0] <= max_gap_s
        H = tr.plane_homography(self.H_img2m)
        names = [st["name"] for st in tr.stickers]
        tol = tr.config.get("max_edge_error_m", .015)
        best = None
        for i, j in ((0, 1), (0, 2), (1, 2)):
            ca, cb = tr.last_candidates.get(names[i], []), tr.last_candidates.get(names[j], [])
            if not gated and (len(ca) != 1 or len(cb) != 1):
                continue        # no recent pose to gate against: only unambiguous pairs
            for a in ca:
                for b in cb:
                    m = transform_points(H, [a["px"], b["px"]])
                    d_err = abs(np.linalg.norm(m[0] - m[1])
                                - np.linalg.norm(tr.body[i] - tr.body[j]))
                    if d_err > tol:
                        continue
                    rot, center, _ = fit_pose(tr.body[[i, j]], m)
                    yaw = math.atan2(rot[1, 0], rot[0, 0])
                    if gated:
                        _, c_ref, yaw_ref = self._last_full
                        jump = float(np.linalg.norm(center - c_ref))
                        turn = abs(math.degrees((yaw - yaw_ref + math.pi) % (2 * math.pi)
                                                - math.pi))
                        if jump > max_jump_m or turn > max_turn_deg:
                            continue
                    if best is None or d_err < best[0]:
                        best = (d_err, rot, center, np.array([a["px"], b["px"]]), (i, j))
        if best is None:
            return None
        d_err, rot, center, pixels, pair = best
        inv = np.linalg.inv(H)
        center_px, front_px = transform_points(inv, [center, center + rot @ [.06, 0]])
        self.n_partial += 1
        tr.status = f"2/3 stickers ({names[pair[0]]}+{names[pair[1]]}); edge {d_err*1000:.1f} mm"
        # keep the gate anchored to the latest good pose so slow motion stays tracked
        self._last_full = (stamp, center, math.atan2(rot[1, 0], rot[0, 0]))
        return dict(center_m=center, center_px=center_px, top_px=front_px,
                    heading_deg=math.degrees(math.atan2(rot[1, 0], rot[0, 0])),
                    dist_m=float(np.linalg.norm(center)), sticker_pixels=pixels,
                    sticker_names=[names[pair[0]], names[pair[1]]], fit_error_m=d_err,
                    reference_point=tr.reference_point, measurement_ready=False, partial=True)

    # ---------------- api ----------------
    def set_grid_homography(self, H_m2img):
        """Swap in a corrected grid<->pixel homography (camera_guard found it moved)."""
        import numpy as np
        H_m2img = np.asarray(H_m2img, float)
        H_img2m = np.linalg.inv(H_m2img)
        with self._lock:
            self.H_m2img, self.H_img2m = H_m2img, H_img2m / H_img2m[2, 2]

    def pose(self, max_age=0.4):
        with self._lock:
            p = self._pose
        if p is None or time.time() - p[0] > max_age:
            return None
        return p

    def wait_pose(self, timeout=5.0, max_age=0.4):
        t_end = time.time() + timeout
        while time.time() < t_end:
            p = self.pose(max_age)
            if p is not None:
                return p
            time.sleep(0.03)
        return None

    def settled_pose(self, duration=0.6, max_age=0.4):
        """Median pose over `duration` s (rover should be stationary). Frames arrive
        latency_s late (measured 0.71 s on this rig), so frames decoded right after a stop
        still show the rover moving: wait them out, then only use frames captured later."""
        import numpy as np
        t_from = time.time() + self.latency_s       # decode time of the first valid frame
        while time.time() < t_from:
            time.sleep(0.02)
        rows, t_end = [], time.time() + duration
        while time.time() < t_end:
            p = self.pose(max_age)
            if p is not None and (not rows or p[0] != rows[-1][0]):
                rows.append(p)
            time.sleep(0.02)
        if not rows:
            return None
        a = np.array(rows)
        yaw = math.atan2(np.median(np.sin(a[:, 3])), np.median(np.cos(a[:, 3])))
        return (float(a[-1, 0]), float(np.median(a[:, 1])), float(np.median(a[:, 2])), yaw)

    def start_recording(self, video_path=None):
        """Record every detected pose (and optionally the raw frames to an mp4)."""
        with self._lock:
            self._rec = []
            self._video = (video_path, None, []) if video_path else None

    def stop_recording(self):
        """-> (pose rows [(t, x, y, yaw)], frame stamps of the video or [])."""
        with self._lock:
            rows, self._rec = self._rec or [], None
            video, self._video = self._video, None
        stamps = []
        if video is not None:
            _, writer, stamps = video
            if writer is not None:
                writer.release()
        return rows, stamps

    def latest(self):
        """-> (frame, tracking result or None) of the newest processed frame."""
        with self._lock:
            return self._frame, self._result

    @property
    def status(self):
        return self.tracker.status

    def overlay(self, trails=(), text=None):
        """Latest frame with metric grid, tracking and optional (pts, BGR) trails."""
        import cv2
        import numpy as np
        with self._lock:
            frame, r = self._frame, self._result
        if frame is None:
            return None
        img = frame.copy()
        draw_metric_grid(img, self.H_m2img, self.grid_m)
        draw_axes_and_labels(img, self.H_m2img, self.grid_m)
        draw_tracking(img, r, self.tracker)
        for pts, color in trails:
            if len(pts) >= 2:
                px = cv2.perspectiveTransform(np.asarray(pts, float).reshape(-1, 1, 2),
                                              self.H_m2img).reshape(-1, 2)
                cv2.polylines(img, [px.astype(np.int32)], False, color, 2, cv2.LINE_AA)
        if text:
            for i, s in enumerate(text if isinstance(text, (list, tuple)) else [text]):
                _txt(img, s, (10, img.shape[0] - 14 - 22 * i), (255, 255, 255), 0.6)
        return img

    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cam.release()
