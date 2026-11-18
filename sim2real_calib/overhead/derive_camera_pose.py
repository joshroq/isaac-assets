#!/usr/bin/env python3
"""Estimate the real overhead-camera pose from the existing grid calibration.

Reads `calib/grid_calib.json` (the metric↔pixel homography from clicking the grid corners) and
a camera intrinsics source (calibrated `camera_intrinsics.yaml` if present, else a nominal
C920e FOV), decomposes the plane homography into a camera pose, and writes `camera_setup.yaml`
that both the synthetic renderer and the Isaac camera consume.

A round-trip check re-projects the grid corners through the recovered pose and compares to the
clicked pixels — this is self-consistent by construction (so it validates the MATH), but the
ABSOLUTE height/tilt are only as accurate as the intrinsics. Calibrate intrinsics, or just drop
measured `position_m`/`look_at_m` into camera_setup.yaml, to make it exact.

    python3 derive_camera_pose.py                       # nominal C920e FOV
    python3 derive_camera_pose.py --hfov 70.4           # set the horizontal FOV
    python3 derive_camera_pose.py --height 1.85         # scale the solution to a MEASURED height
"""
from __future__ import annotations

import argparse
import json
import os

import sys

import numpy as np

import camera_model as cm

sys.path.insert(0, os.path.join(os.path.dirname(os.path.abspath(__file__)), "..", "calib"))
from grid_target import grid_dims  # noqa: E402

HERE = os.path.dirname(os.path.abspath(__file__))
GRID = os.path.join(HERE, "..", "calib", "grid_calib.json")
INTR = os.path.join(HERE, "..", "calib", "camera_intrinsics.yaml")
OUT = os.path.join(HERE, "camera_setup.yaml")
C920E_HFOV = 70.4  # Logitech C920e nominal horizontal FOV at 16:9


def load_K(resolution, hfov_deg):
    if os.path.exists(INTR):
        try:
            import yaml
            d = yaml.safe_load(open(INTR))
            K = np.array(d["camera_matrix"], float).reshape(3, 3)
            print(f"[derive] using calibrated intrinsics {INTR}")
            return K, "calibrated"
        except Exception as e:
            print(f"[derive] couldn't read {INTR} ({e}); falling back to nominal FOV")
    return cm.intrinsics_from_fov(resolution, hfov_deg), f"nominal {hfov_deg:.1f}° FOV"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--grid", default=GRID)
    ap.add_argument("--hfov", type=float, default=C920E_HFOV)
    ap.add_argument("--height", type=float, default=None,
                    help="MEASURED camera height (m): rescales the solution to be metric-exact")
    ap.add_argument("--grid-origin", type=float, nargs=2, default=[0.0, 0.0],
                    help="grid origin in the sim/world frame (rover start); default 0 0")
    ap.add_argument("--out", default=OUT)
    args = ap.parse_args()

    gc = json.load(open(args.grid))
    res = gc.get("image_size", [1280, 720])
    gw, gh = grid_dims(gc)
    H_m2img = np.array(gc["H_m2img"], float)
    K, ksrc = load_K(res, args.hfov)

    R, C = cm.planar_pose_from_homography(H_m2img, K)
    fwd = R[2, :].copy()                        # optical axis in world
    if fwd[2] > 0:                              # must point downward toward the plane
        fwd = -fwd
    if args.height is not None and C[2] > 1e-6:  # rescale so height matches a tape measure
        C = C * (args.height / C[2])
    s = -C[2] / fwd[2] if abs(fwd[2]) > 1e-6 else 1.0
    look_at = C + fwd * s                       # ground point the axis hits (z≈0)
    up_hint = R[1, :]                           # reproduces R via camera_model's look-at

    setup = {
        "resolution": [int(res[0]), int(res[1])],
        "horizontal_fov_deg": float(args.hfov),
        "position_m": [float(v) for v in C],
        "look_at_m": [float(v) for v in look_at],
        "up_hint": [float(v) for v in up_hint],
        "grid_size_m": gw if gw == gh else [gw, gh],
        "grid_width_m": gw,
        "grid_height_m": gh,
        "grid_origin_m": [float(args.grid_origin[0]), float(args.grid_origin[1])],
        "_intrinsics_source": ksrc,
        "_note": "position_m/look_at_m are ESTIMATES from the grid homography + "
                 f"{ksrc}; replace with measured values (or --height) to make the view exact.",
    }

    # round-trip: project the metric grid corners and compare to grid_calib's mapping
    cam = cm.OverheadCamera(setup)
    corners_m = np.array([[0, 0], [gw, 0], [gw, gh], [0, gh]], float)
    import cv2
    truth = cv2.perspectiveTransform(corners_m.reshape(-1, 1, 2), H_m2img).reshape(-1, 2)
    mine = cam.ground_to_px(corners_m)
    resid = float(np.nanmax(np.linalg.norm(truth - mine, axis=1)))

    try:
        import yaml
        open(args.out, "w").write(yaml.safe_dump(setup, sort_keys=False))
    except Exception:
        open(args.out, "w").write(json.dumps(setup, indent=2))

    print(f"[derive] intrinsics: {ksrc}")
    print(f"[derive] camera height above grid : {cam.height():.3f} m"
          + ("  (scaled to --height)" if args.height else "  (estimate — verify with a tape)"))
    print(f"[derive] look-down tilt from vertical: {cam.tilt_deg():.1f}°  (0 = straight down)")
    print(f"[derive] aimed at grid point        : ({look_at[0]:.3f}, {look_at[1]:.3f}) m")
    print(f"[derive] grid-corner reprojection residual: {resid:.2f} px "
          + ("OK (math consistent; <1% of frame)" if resid < 10.0 else "— check inputs"))
    print(f"[derive] wrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
