#!/usr/bin/env python3
"""FusedTracker: the pose the homing controller steers on.

Same interface as CameraTrack (pose / wait_pose / settled_pose / status / n_partial),
but pose() is the camera+odometry fusion from fusion.py: current (no camera lag),
smooth, and it rides through sticker occlusions on odometry. settled_pose() stays a
pure camera median (absolute) because the rover is stationary then and that is the
number the start-pose check should be judged on.
"""
from __future__ import annotations

import threading
import time

from fusion import PoseFuser


class FusedTracker:
    def __init__(self, cam, rover, cam_latency_s=0.2, gain=0.3, max_occlusion_s=4.0):
        self.cam, self.rover = cam, rover
        self.fuser = PoseFuser(rover.integrator, cam_latency_s=cam_latency_s, gain=gain)
        self.max_occlusion = max_occlusion_s
        self._running = True
        self._last_stamp = None
        self._t = threading.Thread(target=self._feed, daemon=True)
        self._t.start()

    def _feed(self):
        while self._running:
            p = self.cam.pose(max_age=0.5)
            if p is not None and p[0] != self._last_stamp:
                self._last_stamp = p[0]
                self.fuser.camera(*p)
            time.sleep(0.01)

    # ---- CameraTrack-compatible api ----
    def pose(self, max_age=0.35):
        p = self.fuser.pose(max_fix_age=self.max_occlusion)
        if p is None or time.time() - p[0] > max_age:
            return self.cam.pose(max_age)            # not anchored yet: raw camera
        return p

    def wait_pose(self, timeout=5.0, max_age=0.35):
        t_end = time.time() + timeout
        while time.time() < t_end:
            p = self.pose(max_age)
            if p is not None:
                return p
            time.sleep(0.03)
        return None

    def settled_pose(self, duration=0.6, max_age=0.4):
        return self.cam.settled_pose(duration, max_age)

    @property
    def status(self):
        age = time.time() - self.fuser.last_fix_t if self.fuser.last_fix_t else float("inf")
        return f"{self.cam.status}; fused (last camera fix {age:.1f}s ago)"

    @property
    def n_partial(self):
        return self.cam.n_partial

    @property
    def fused(self):
        return self.fuser.C is not None

    def close(self):
        self._running = False
        self._t.join(timeout=1.0)
