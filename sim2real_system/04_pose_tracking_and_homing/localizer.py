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


# Camera + odometry pose for homing (drop-in replacement for CameraTrack).
class FusedTracker:
    # cam = CameraTrack, rover = RoverLink (whose odometry integrator is fused);
    # max_occlusion_s = how long the fused pose may run on odometry alone.
    def __init__(self, cam, rover, cam_latency_s=0.2, gain=0.3, max_occlusion_s=4.0):
        self.cam, self.rover = cam, rover
        self.fuser = PoseFuser(rover.integrator, cam_latency_s=cam_latency_s, gain=gain)
        self.max_occlusion = max_occlusion_s
        self._running = True
        self._last_stamp = None
        self._t = threading.Thread(target=self._feed, daemon=True)
        self._t.start()

    # Thread: pass every new camera pose to the fuser.
    def _feed(self):
        while self._running:
            camera_pose = self.cam.pose(max_age=0.5)
            if camera_pose is not None and camera_pose[0] != self._last_stamp:
                self._last_stamp = camera_pose[0]
                self.fuser.camera(*camera_pose)
            time.sleep(0.01)

    # ---- CameraTrack-compatible api ----
    # Fused pose, or the raw camera pose until the fuser has its first anchor.
    def pose(self, max_age=0.35):
        fused_pose = self.fuser.pose(max_fix_age=self.max_occlusion)
        if fused_pose is None or time.time() - fused_pose[0] > max_age:
            return self.cam.pose(max_age)            # not anchored yet: raw camera
        return fused_pose

    # Poll pose() until available or timeout.
    def wait_pose(self, timeout=5.0, max_age=0.35):
        deadline = time.time() + timeout
        while time.time() < deadline:
            current_pose = self.pose(max_age)
            if current_pose is not None:
                return current_pose
            time.sleep(0.03)
        return None

    # Camera-only median pose (absolute; used to judge start poses).
    def settled_pose(self, duration=0.6, max_age=0.4):
        return self.cam.settled_pose(duration, max_age)

    # Camera status plus the age of the last camera fix.
    @property
    def status(self):
        fix_age_s = time.time() - self.fuser.last_fix_t if self.fuser.last_fix_t else float("inf")
        return f"{self.cam.status}; fused (last camera fix {fix_age_s:.1f}s ago)"

    # Number of 2-sticker (partial) camera poses so far.
    @property
    def n_partial(self):
        return self.cam.n_partial

    # True once the fuser has been anchored by a camera fix.
    @property
    def fused(self):
        return self.fuser.C is not None

    # Stop the feeding thread.
    def close(self):
        self._running = False
        self._t.join(timeout=1.0)
