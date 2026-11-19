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


# The overhead stream is not available.
class CameraError(RuntimeError):
    pass


# Background tracker: one thread reads the overhead stream, finds the rover in every
# new frame and keeps the newest pose (t, x_m, y_m, yaw_rad) in grid metres. Homing,
# the trial recorder and the dashboard all read from this one object.
class CameraTrack:
    # Load the grid calibration + sticker tracker, open the stream (or a replay `source`)
    # and start the thread. Without allow_approximate the sticker config must be
    # measurement-ready (measured geometry + verified projection).
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

    # Thread body: for each NEW frame run the full 3-sticker detection, else the 2-sticker
    # fallback (_partial); store frame/result/pose, append the pose while recording, and
    # write the frame to the video while a video recording is open.
    def _loop(self):
        last_frame_number = -1
        while self._running:
            frame, stamp, frame_number = self.cam.read()
            if frame is None or frame_number == last_frame_number:
                time.sleep(0.003)
                continue
            last_frame_number = frame_number
            try:
                result = detect_rover(frame, self.H_img2m, self.tracker)
                if result is not None:
                    self._last_full = (stamp, result["center_m"], math.radians(result["heading_deg"]))
                else:
                    result = self._partial(stamp)
            except ValueError as exc:
                self.tracker.status = str(exc)
                result = None
            with self._lock:
                self.n_frames += 1
                self._frame, self._result = frame, result
                if self._video is not None:
                    video_path, video_writer, frame_stamps = self._video
                    if video_writer is None:
                        import cv2
                        video_writer = cv2.VideoWriter(str(video_path), cv2.VideoWriter_fourcc(*"mp4v"),
                                                 30.0, (frame.shape[1], frame.shape[0]))
                        self._video = (video_path, video_writer, frame_stamps)
                    video_writer.write(frame)
                    frame_stamps.append(stamp)
                if result is not None:
                    self.n_detect += 1
                    yaw_rad = math.radians(result["heading_deg"])
                    self._pose = (stamp, float(result["center_m"][0]), float(result["center_m"][1]), yaw_rad)
                    if self._rec is not None:
                        self._rec.append(self._pose)

    # Occlusion fallback when only two stickers are visible: for each pair of colours,
    # check the metric distance matches the configured edge, fit the 2-point rigid pose,
    # and (if a full pose was seen in the last max_gap_s) reject jumps > max_jump_m or turns
    # > max_turn_deg. Partial poses are never measurement_ready.
    def _partial(self, stamp, max_gap_s=2.5, max_jump_m=0.12, max_turn_deg=35.0):
        """2-sticker pose from the candidates of the failed full detection."""
        import numpy as np
        tracker = self.tracker
        has_recent_full_pose = self._last_full is not None and stamp - self._last_full[0] <= max_gap_s
        image_to_plane = tracker.plane_homography(self.H_img2m)
        sticker_names = [sticker["name"] for sticker in tracker.stickers]
        edge_tolerance_m = tracker.config.get("max_edge_error_m", .015)
        best_pair = None
        for first, second in ((0, 1), (0, 2), (1, 2)):
            first_candidates, second_candidates = tracker.last_candidates.get(sticker_names[first], []), tracker.last_candidates.get(sticker_names[second], [])
            if not has_recent_full_pose and (len(first_candidates) != 1 or len(second_candidates) != 1):
                continue        # no recent pose to gate against: only unambiguous pairs
            for first_blob in first_candidates:
                for second_blob in second_candidates:
                    pair_xy_m = transform_points(image_to_plane, [first_blob["px"], second_blob["px"]])
                    edge_error_m = abs(np.linalg.norm(pair_xy_m[0] - pair_xy_m[1])
                                - np.linalg.norm(tracker.body[first] - tracker.body[second]))
                    if edge_error_m > edge_tolerance_m:
                        continue
                    rotation, center_m, _ = fit_pose(tracker.body[[first, second]], pair_xy_m)
                    yaw_rad = math.atan2(rotation[1, 0], rotation[0, 0])
                    if has_recent_full_pose:
                        _, last_center_m, last_yaw_rad = self._last_full
                        jump_m = float(np.linalg.norm(center_m - last_center_m))
                        turn_deg = abs(math.degrees((yaw_rad - last_yaw_rad + math.pi) % (2 * math.pi)
                                                - math.pi))
                        if jump_m > max_jump_m or turn_deg > max_turn_deg:
                            continue
                    if best_pair is None or edge_error_m < best_pair[0]:
                        best_pair = (edge_error_m, rotation, center_m, np.array([first_blob["px"], second_blob["px"]]), (first, second))
        if best_pair is None:
            return None
        edge_error_m, rotation, center_m, pair_pixels, pair_indices = best_pair
        plane_to_image = np.linalg.inv(image_to_plane)
        center_px, front_px = transform_points(plane_to_image, [center_m, center_m + rotation @ [.06, 0]])
        self.n_partial += 1
        tracker.status = f"2/3 stickers ({sticker_names[pair_indices[0]]}+{sticker_names[pair_indices[1]]}); edge {edge_error_m*1000:.1f} mm"
        # keep the gate anchored to the latest good pose so slow motion stays tracked
        self._last_full = (stamp, center_m, math.atan2(rotation[1, 0], rotation[0, 0]))
        return dict(center_m=center_m, center_px=center_px, top_px=front_px,
                    heading_deg=math.degrees(math.atan2(rotation[1, 0], rotation[0, 0])),
                    dist_m=float(np.linalg.norm(center_m)), sticker_pixels=pair_pixels,
                    sticker_names=[sticker_names[pair_indices[0]], sticker_names[pair_indices[1]]], fit_error_m=edge_error_m,
                    reference_point=tracker.reference_point, measurement_ready=False, partial=True)

    # ---------------- api ----------------
    # Swap in a corrected grid<->pixel mapping (camera_guard found the camera moved).
    def set_grid_homography(self, H_m2img):
        """Swap in a corrected grid<->pixel homography (camera_guard found it moved)."""
        import numpy as np
        H_m2img = np.asarray(H_m2img, float)
        H_img2m = np.linalg.inv(H_m2img)
        with self._lock:
            self.H_m2img, self.H_img2m = H_m2img, H_img2m / H_img2m[2, 2]

    # Newest pose (t, x, y, yaw) or None if older than max_age seconds.
    def pose(self, max_age=0.4):
        with self._lock:
            latest_pose = self._pose
        if latest_pose is None or time.time() - latest_pose[0] > max_age:
            return None
        return latest_pose

    # Poll pose() until one is available or timeout.
    def wait_pose(self, timeout=5.0, max_age=0.4):
        deadline = time.time() + timeout
        while time.time() < deadline:
            latest_pose = self.pose(max_age)
            if latest_pose is not None:
                return latest_pose
            time.sleep(0.03)
        return None

    # Median pose of a stationary rover (see docstring for the latency wait).
    def settled_pose(self, duration=0.6, max_age=0.4):
        """Median pose over `duration` s (rover should be stationary). Frames arrive
        latency_s late (measured 0.71 s on this rig), so frames decoded right after a stop
        still show the rover moving: wait them out, then only use frames captured later."""
        import numpy as np
        first_valid_time = time.time() + self.latency_s       # decode time of the first valid frame
        while time.time() < first_valid_time:
            time.sleep(0.02)
        poses, deadline = [], time.time() + duration
        while time.time() < deadline:
            latest_pose = self.pose(max_age)
            if latest_pose is not None and (not poses or latest_pose[0] != poses[-1][0]):
                poses.append(latest_pose)
            time.sleep(0.02)
        if not poses:
            return None
        pose_array = np.array(poses)
        median_yaw = math.atan2(np.median(np.sin(pose_array[:, 3])), np.median(np.cos(pose_array[:, 3])))
        return (float(pose_array[-1, 0]), float(np.median(pose_array[:, 1])), float(np.median(pose_array[:, 2])), median_yaw)

    # Start collecting every pose (and optionally raw frames into an mp4).
    def start_recording(self, video_path=None):
        """Record every detected pose (and optionally the raw frames to an mp4)."""
        with self._lock:
            self._rec = []
            self._video = (video_path, None, []) if video_path else None

    # Stop recording; return (poses, video frame stamps) and close the video.
    def stop_recording(self):
        """-> (pose rows [(t, x, y, yaw)], frame stamps of the video or [])."""
        with self._lock:
            poses, self._rec = self._rec or [], None
            video_state, self._video = self._video, None
        frame_stamps = []
        if video_state is not None:
            _, video_writer, frame_stamps = video_state
            if video_writer is not None:
                video_writer.release()
        return poses, frame_stamps

    # Newest frame and its tracking result.
    def latest(self):
        """-> (frame, tracking result or None) of the newest processed frame."""
        with self._lock:
            return self._frame, self._result

    # Tracker's last status text (e.g. '3/3 stickers; fit 1.4 mm', 'missing: pink').
    @property
    def status(self):
        return self.tracker.status

    # Newest frame with grid, axes, tracking, path trails (metres) and text lines drawn.
    def overlay(self, trails=(), text=None):
        """Latest frame with metric grid, tracking and optional (pts, BGR) trails."""
        import cv2
        import numpy as np
        with self._lock:
            frame, result = self._frame, self._result
        if frame is None:
            return None
        image = frame.copy()
        draw_metric_grid(image, self.H_m2img, self.grid_m)
        draw_axes_and_labels(image, self.H_m2img, self.grid_m)
        draw_tracking(image, result, self.tracker)
        for trail_points_m, colour in trails:
            if len(trail_points_m) >= 2:
                trail_px = cv2.perspectiveTransform(np.asarray(trail_points_m, float).reshape(-1, 1, 2),
                                              self.H_m2img).reshape(-1, 2)
                cv2.polylines(image, [trail_px.astype(np.int32)], False, colour, 2, cv2.LINE_AA)
        if text:
            for line_index, text_line in enumerate(text if isinstance(text, (list, tuple)) else [text]):
                _txt(image, text_line, (10, image.shape[0] - 14 - 22 * line_index), (255, 255, 255), 0.6)
        return image

    # Stop the thread and release the stream.
    def close(self):
        self._running = False
        self._thread.join(timeout=1.0)
        self.cam.release()
