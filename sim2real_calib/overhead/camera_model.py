#!/usr/bin/env python3
"""Shared overhead-camera model: pose + intrinsics ↔ pixel projection.

One source of truth for "where the camera is and how it sees the grid", consumed by:
  - `render_overhead.py` (draw the SIM run top-down exactly as the real webcam sees it),
  - `run_overhead.py` (author a matching camera in Isaac so the sim VIEW matches the webcam),
  - `derive_camera_pose.py` (estimate the pose from the existing grid homography).

A camera setup (see camera_setup.yaml) is intuitive, in the grid frame (metres, Z up):
    resolution:        [W, H]           pixels
    horizontal_fov_deg: 70.4            C920e nominal (replace with the measured/҂calibrated value)
    position_m:        [x, y, z]        camera position above the grid (z = height)
    look_at_m:         [x, y, z]        ground point the camera is aimed at (z≈0)
    up_hint:           [x, y, z]        grid direction that should point "up" in the image
    grid_size_m:       1.0             (or grid_width_m / grid_height_m for a W x H grid)
    grid_origin_m:     [x, y]           grid origin in the sim/world frame (rover start)

The OpenCV convention (z forward, x right, y down) is used for pixel projection; a USD/Isaac
look-at (−Z forward, +Y up) is provided for authoring the sim camera.
"""
from __future__ import annotations

import math

import numpy as np


def _n(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


def intrinsics_from_fov(resolution, hfov_deg):
    """Pinhole K from image size + horizontal FOV (square pixels, centre principal point)."""
    w, h = resolution
    fx = (w / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    fy = fx
    return np.array([[fx, 0, w / 2.0], [0, fy, h / 2.0], [0, 0, 1.0]])


class OverheadCamera:
    def __init__(self, setup: dict):
        self.res = tuple(int(x) for x in setup["resolution"])
        self.hfov_deg = float(setup.get("horizontal_fov_deg", 70.4))
        self.K = intrinsics_from_fov(self.res, self.hfov_deg)
        self.C = np.asarray(setup["position_m"], float)
        self.T = np.asarray(setup["look_at_m"], float)
        self.up = _n(setup.get("up_hint", [0, 1, 0]))
        if "grid_width_m" in setup:
            self.grid_dims = (float(setup["grid_width_m"]), float(setup["grid_height_m"]))
        else:
            gs = setup.get("grid_size_m", 1.0)
            self.grid_dims = (float(gs[0]), float(gs[1])) if isinstance(gs, list) else (float(gs),) * 2
        self.grid_size = max(self.grid_dims)
        self.grid_origin = np.asarray(setup.get("grid_origin_m", [0, 0]), float)
        # OpenCV world->camera rotation (rows = image x,y,z axes in world)
        z = _n(self.T - self.C)                 # optical axis (forward)
        x = _n(np.cross(self.up, z))            # image +x (right)
        y = np.cross(z, x)                      # image +y (down)
        self.R = np.vstack([x, y, z])           # world -> camera
        self.t = -self.R @ self.C

    # ---- geometry helpers -------------------------------------------------------
    def height(self):
        return float(self.C[2])

    def tilt_deg(self):
        """Angle of the optical axis from straight-down (0 = looking straight down)."""
        z = _n(self.T - self.C)
        return math.degrees(math.acos(max(-1.0, min(1.0, np.dot(z, [0, 0, -1])))))

    # ---- projection -------------------------------------------------------------
    def project(self, pts_world):
        """World points (N,3 or N,2 with z=0) -> pixels (N,2); NaN where behind camera."""
        P = np.asarray(pts_world, float)
        if P.ndim == 1:
            P = P[None, :]
        if P.shape[1] == 2:
            P = np.hstack([P, np.zeros((len(P), 1))])
        Pc = (self.R @ (P - self.C).T).T        # camera frame
        z = Pc[:, 2:3]
        uv = (self.K @ Pc.T).T
        px = uv[:, :2] / np.where(np.abs(uv[:, 2:3]) < 1e-9, 1e-9, uv[:, 2:3])
        px[(z[:, 0] <= 1e-6)] = np.nan          # behind the camera
        return px

    def ground_to_px(self, xy):
        """Grid-plane points (metres, in grid frame) -> pixels, accounting for grid_origin."""
        xy = np.asarray(xy, float).reshape(-1, 2)
        world = np.hstack([xy + self.grid_origin, np.zeros((len(xy), 1))])
        return self.project(world)

    # ---- USD / Isaac authoring --------------------------------------------------
    def usd_pose(self):
        """(translation, quaternion wxyz) for an Isaac camera (looks -Z, up +Y) matching this."""
        z_back = _n(self.C - self.T)            # USD cam looks along -Z, so +Z points back
        x_r = _n(np.cross(self.up, z_back))
        y_u = np.cross(z_back, x_r)
        Rwc = np.vstack([x_r, y_u, z_back]).T   # columns = cam axes in world
        return self.C.tolist(), _mat_to_quat(Rwc)


def _mat_to_quat(R):
    """3x3 rotation -> (w,x,y,z)."""
    t = np.trace(R)
    if t > 0:
        s = math.sqrt(t + 1.0) * 2
        w = 0.25 * s
        x = (R[2, 1] - R[1, 2]) / s
        y = (R[0, 2] - R[2, 0]) / s
        z = (R[1, 0] - R[0, 1]) / s
    else:
        i = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if i == 0:
            s = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / s; x = 0.25 * s
            y = (R[0, 1] + R[1, 0]) / s; z = (R[0, 2] + R[2, 0]) / s
        elif i == 1:
            s = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / s; x = (R[0, 1] + R[1, 0]) / s
            y = 0.25 * s; z = (R[1, 2] + R[2, 1]) / s
        else:
            s = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w = (R[1, 0] - R[0, 1]) / s; x = (R[0, 2] + R[2, 0]) / s
            y = (R[1, 2] + R[2, 1]) / s; z = 0.25 * s
    return [w, x, y, z]


def planar_pose_from_homography(H_m2img, K):
    """Recover camera pose (R world->cam, C position in grid frame) from a plane homography.

    H maps grid-plane (X,Y,1) -> image (u,v,1) up to scale: H = K [r1 r2 t]. Standard planar
    decomposition. Absolute scale (height) is only as accurate as K (hence calibrate intrinsics
    or enter a measured height). Returns (R, C).
    """
    H = np.asarray(H_m2img, float)
    A = np.linalg.inv(K) @ H
    lam = 1.0 / np.linalg.norm(A[:, 0])
    r1 = A[:, 0] * lam
    r2 = A[:, 1] * lam
    t = A[:, 2] * lam
    if t[2] < 0:                                # camera must be in front of the plane
        r1, r2, t = -r1, -r2, -t
    r3 = np.cross(r1, r2)
    R = np.vstack([r1, r2, r3]).T
    U, _, Vt = np.linalg.svd(R)                 # nearest orthonormal
    R = U @ Vt
    C = -R.T @ t                                # camera position in grid frame
    return R, C
