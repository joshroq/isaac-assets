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


# Unit vector in the direction of v (tiny epsilon avoids dividing by zero).
def _n(v):
    v = np.asarray(v, float)
    return v / (np.linalg.norm(v) + 1e-12)


# Pinhole camera matrix K from the image size and horizontal field of view:
# focal length f = (width / 2) / tan(hfov / 2), principal point at the image centre.
def intrinsics_from_fov(resolution, hfov_deg):
    """Pinhole K from image size + horizontal FOV (square pixels, centre principal point)."""
    width_px, height_px = resolution
    focal_x_px = (width_px / 2.0) / math.tan(math.radians(hfov_deg) / 2.0)
    focal_y_px = focal_x_px
    return np.array([[focal_x_px, 0, width_px / 2.0], [0, focal_y_px, height_px / 2.0], [0, 0, 1.0]])


# Ideal pinhole model of the overhead camera from a camera_setup.yaml dict.
# Attributes: K intrinsics, C camera position, T look-at point, R/t world -> camera.
class OverheadCamera:
    # Build K and the look-at rotation: optical axis = towards the look-at point,
    # image right = up_hint x optical axis, image down = optical axis x right.
    def __init__(self, setup: dict):
        self.res = tuple(int(value) for value in setup["resolution"])
        self.hfov_deg = float(setup.get("horizontal_fov_deg", 70.4))
        self.K = intrinsics_from_fov(self.res, self.hfov_deg)
        self.C = np.asarray(setup["position_m"], float)
        self.T = np.asarray(setup["look_at_m"], float)
        self.up = _n(setup.get("up_hint", [0, 1, 0]))
        if "grid_width_m" in setup:
            self.grid_dims = (float(setup["grid_width_m"]), float(setup["grid_height_m"]))
        else:
            grid_size_setting = setup.get("grid_size_m", 1.0)
            self.grid_dims = (float(grid_size_setting[0]), float(grid_size_setting[1])) if isinstance(grid_size_setting, list) else (float(grid_size_setting),) * 2
        self.grid_size = max(self.grid_dims)
        self.grid_origin = np.asarray(setup.get("grid_origin_m", [0, 0]), float)
        # OpenCV world->camera rotation (rows = image x,y,z axes in world)
        optical_axis = _n(self.T - self.C)                 # optical axis (forward)
        image_right_axis = _n(np.cross(self.up, optical_axis))            # image +x (right)
        image_down_axis = np.cross(optical_axis, image_right_axis)                      # image +y (down)
        self.R = np.vstack([image_right_axis, image_down_axis, optical_axis])           # world -> camera
        self.t = -self.R @ self.C

    # ---- geometry helpers -------------------------------------------------------
    # Camera height above the grid plane (m).
    def height(self):
        return float(self.C[2])

    # Optical-axis angle from straight down (degrees).
    def tilt_deg(self):
        """Angle of the optical axis from straight-down (0 = looking straight down)."""
        optical_axis = _n(self.T - self.C)
        return math.degrees(math.acos(max(-1.0, min(1.0, np.dot(optical_axis, [0, 0, -1])))))

    # ---- projection -------------------------------------------------------------
    # World points -> pixels: rotate/translate into the camera frame, multiply by K,
    # divide by depth. Points behind the camera become NaN.
    def project(self, pts_world):
        """World points (N,3 or N,2 with z=0) -> pixels (N,2); NaN where behind camera."""
        points = np.asarray(pts_world, float)
        if points.ndim == 1:
            points = points[None, :]
        if points.shape[1] == 2:
            points = np.hstack([points, np.zeros((len(points), 1))])
        points_cam = (self.R @ (points - self.C).T).T        # camera frame
        depth = points_cam[:, 2:3]
        homogeneous_px = (self.K @ points_cam.T).T
        pixels = homogeneous_px[:, :2] / np.where(np.abs(homogeneous_px[:, 2:3]) < 1e-9, 1e-9, homogeneous_px[:, 2:3])
        pixels[(depth[:, 0] <= 1e-6)] = np.nan          # behind the camera
        return pixels

    # Grid-frame floor points (z = 0) -> pixels, shifting by grid_origin first.
    def ground_to_px(self, xy):
        """Grid-plane points (metres, in grid frame) -> pixels, accounting for grid_origin."""
        xy = np.asarray(xy, float).reshape(-1, 2)
        world_points = np.hstack([xy + self.grid_origin, np.zeros((len(xy), 1))])
        return self.project(world_points)

    # ---- USD / Isaac authoring --------------------------------------------------
    # Same camera as an Isaac/USD pose (USD cameras look along -Z with +Y up).
    def usd_pose(self):
        """(translation, quaternion wxyz) for an Isaac camera (looks -Z, up +Y) matching this."""
        z_back = _n(self.C - self.T)            # USD cam looks along -Z, so +Z points back
        right_axis = _n(np.cross(self.up, z_back))
        up_axis = np.cross(z_back, right_axis)
        cam_to_world = np.vstack([right_axis, up_axis, z_back]).T   # columns = cam axes in world
        return self.C.tolist(), _mat_to_quat(cam_to_world)


# Rotation matrix -> quaternion [w, x, y, z] (numerically stable branch on the
# largest diagonal element).
def _mat_to_quat(R):
    """3x3 rotation -> (w,x,y,z)."""
    trace = np.trace(R)
    if trace > 0:
        scale = math.sqrt(trace + 1.0) * 2
        w = 0.25 * scale
        x = (R[2, 1] - R[1, 2]) / scale
        y = (R[0, 2] - R[2, 0]) / scale
        z = (R[1, 0] - R[0, 1]) / scale
    else:
        largest_diagonal = int(np.argmax([R[0, 0], R[1, 1], R[2, 2]]))
        if largest_diagonal == 0:
            scale = math.sqrt(1.0 + R[0, 0] - R[1, 1] - R[2, 2]) * 2
            w = (R[2, 1] - R[1, 2]) / scale; x = 0.25 * scale
            y = (R[0, 1] + R[1, 0]) / scale; z = (R[0, 2] + R[2, 0]) / scale
        elif largest_diagonal == 1:
            scale = math.sqrt(1.0 - R[0, 0] + R[1, 1] - R[2, 2]) * 2
            w = (R[0, 2] - R[2, 0]) / scale; x = (R[0, 1] + R[1, 0]) / scale
            y = 0.25 * scale; z = (R[1, 2] + R[2, 1]) / scale
        else:
            scale = math.sqrt(1.0 - R[0, 0] - R[1, 1] + R[2, 2]) * 2
            w = (R[1, 0] - R[0, 1]) / scale; x = (R[0, 2] + R[2, 0]) / scale
            y = (R[1, 2] + R[2, 1]) / scale; z = 0.25 * scale
    return [w, x, y, z]


# Camera pose from a floor homography (Zhang's planar decomposition):
# K^-1 H = scale * [r1 r2 t]; r3 = r1 x r2; the rotation is snapped to the nearest true
# rotation with an SVD; camera centre C = -R^T t (in grid metres).
def planar_pose_from_homography(H_m2img, K):
    """Recover camera pose (R world->cam, C position in grid frame) from a plane homography.

    H maps grid-plane (X,Y,1) -> image (u,v,1) up to scale: H = K [r1 r2 t]. Standard planar
    decomposition. Absolute scale (height) is only as accurate as K (hence calibrate intrinsics
    or enter a measured height). Returns (R, C).
    """
    H = np.asarray(H_m2img, float)
    normalized_H = np.linalg.inv(K) @ H
    scale = 1.0 / np.linalg.norm(normalized_H[:, 0])
    r1 = normalized_H[:, 0] * scale
    r2 = normalized_H[:, 1] * scale
    t = normalized_H[:, 2] * scale
    if t[2] < 0:                                # camera must be in front of the plane
        r1, r2, t = -r1, -r2, -t
    r3 = np.cross(r1, r2)
    R = np.vstack([r1, r2, r3]).T
    U, _, Vt = np.linalg.svd(R)                 # nearest orthonormal
    R = U @ Vt
    C = -R.T @ t                                # camera position in grid frame
    return R, C
