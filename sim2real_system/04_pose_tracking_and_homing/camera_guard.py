#!/usr/bin/env python3
"""Detect (and compensate) a moved overhead camera.

Every camera-measured start pose depends on calib/grid_calib.json, which is only valid
while the camera stays exactly where it was when the grid was clicked. It has been
bumped before (memory: overlay drifted half a cell). This guard keeps a REFERENCE image
of the floor taken right after calibration and registers each check image against it
(ORB features on the floor/grid region, rover masked out, RANSAC homography):

    shift_px = max displacement of the four grid corners between reference and now
      < warn_px      camera unchanged
      >= warn_px     camera moved -> the corrected homography H_ref->now ∘ H_m2img is
                     returned so tracking stays metric; >= fail_px refuse (re-calibrate)

    python3 loop/camera_guard.py --save-reference     # after calibrate_grid.py
    python3 loop/camera_guard.py --check              # report shift
    python3 loop/camera_guard.py --check --write      # also rewrite grid_calib.json
                                                      # (old file backed up)
"""
from __future__ import annotations

import json
import shutil
import sys
import time
from pathlib import Path

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent
REF_DIR = HERE / "stations"
REF_IMG = REF_DIR / "camera_reference.png"
REF_META = REF_DIR / "camera_reference.json"
GRID = SC / "calib" / "grid_calib.json"
sys.path.insert(0, str(SC / "calib"))
from grid_target import as_dims, grid_dims  # noqa: E402


# Mask (255 = use) of the floor around the grid, padded by pad_m, with a disc of
# radius rover_r around the rover cut out: features on the moving rover would make the
# camera look like it moved.
def floor_mask(shape, H_m2img, grid_m=1.0, pad_m=0.35, rover_xy=None, rover_r=0.25):
    """Floor region around the grid (where the static tape/carpet texture is), with the
    rover's disc removed so its motion can't drag the registration."""
    image_h, image_w = shape[:2]
    grid_w_m, grid_h_m = as_dims(grid_m)
    mask = np.zeros((image_h, image_w), np.uint8)
    floor_polygon_m = np.array([[-pad_m, -pad_m], [grid_w_m + pad_m, -pad_m],
                     [grid_w_m + pad_m, grid_h_m + pad_m], [-pad_m, grid_h_m + pad_m]], float)
    floor_polygon_px = cv2.perspectiveTransform(floor_polygon_m.reshape(-1, 1, 2), H_m2img).reshape(-1, 2)
    cv2.fillConvexPoly(mask, floor_polygon_px.astype(np.int32), 255)
    if rover_xy is not None:
        angles = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        rover_disc_m = np.column_stack([rover_xy[0] + rover_r * np.cos(angles),
                                rover_xy[1] + rover_r * np.sin(angles)])
        rover_disc_px = cv2.perspectiveTransform(rover_disc_m.reshape(-1, 1, 2), H_m2img).reshape(-1, 2)
        cv2.fillConvexPoly(mask, rover_disc_px.astype(np.int32), 0)
    return mask


# Feature registration of the reference image onto the current one: ORB keypoints in
# the masks, Lowe ratio test (0.75), RANSAC homography (2 px threshold). Needs >= 40
# matches and >= 30 inliers. Returns (3x3 reference->current pixel H, stats) or (None, why).
def register(ref, cur, mask_ref, mask_cur, n_features=4000):
    """Homography mapping reference pixels -> current pixels (None if unreliable)."""
    orb_detector = cv2.ORB_create(n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
    ref_gray = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
    cur_gray = cv2.cvtColor(cur, cv2.COLOR_BGR2GRAY)
    ref_keypoints, ref_descriptors = orb_detector.detectAndCompute(ref_gray, mask_ref)
    cur_keypoints, cur_descriptors = orb_detector.detectAndCompute(cur_gray, mask_cur)
    if ref_descriptors is None or cur_descriptors is None or len(ref_keypoints) < 50 or len(cur_keypoints) < 50:
        return None, {"why": "too few features"}
    matcher = cv2.BFMatcher(cv2.NORM_HAMMING)
    good_matches = []
    for match_pair in matcher.knnMatch(ref_descriptors, cur_descriptors, k=2):
        if len(match_pair) == 2 and match_pair[0].distance < 0.75 * match_pair[1].distance:
            good_matches.append(match_pair[0])
    if len(good_matches) < 40:
        return None, {"why": f"only {len(good_matches)} feature matches"}
    ref_points = np.float32([ref_keypoints[match.queryIdx].pt for match in good_matches])
    cur_points = np.float32([cur_keypoints[match.trainIdx].pt for match in good_matches])
    ref_to_cur, inlier_mask = cv2.findHomography(ref_points, cur_points, cv2.RANSAC, 2.0, maxIters=5000, confidence=0.999)
    if ref_to_cur is None:
        return None, {"why": "no homography"}
    inlier_count = int(inlier_mask.sum())
    if inlier_count < 30:
        return None, {"why": f"only {inlier_count} RANSAC inliers"}
    return ref_to_cur, {"matches": len(good_matches), "inliers": inlier_count}


# Compares the live camera image with the reference saved right after the grid
# calibration, to tell whether the camera has moved since.
class CameraGuard:
    # Load the reference image + metadata. warn_px = shift that counts as 'moved';
    # fail_px = shift too large to correct (the grid must be re-calibrated).
    def __init__(self, warn_px=2.0, fail_px=25.0):
        self.warn_px, self.fail_px = warn_px, fail_px
        if not REF_IMG.exists():
            raise FileNotFoundError(f"no camera reference ({REF_IMG}); run "
                                    "camera_guard.py --save-reference")
        self.ref = cv2.imread(str(REF_IMG))
        meta = json.loads(REF_META.read_text())
        self.H_m2img_ref = np.array(meta["H_m2img"], float)
        self.grid_m = grid_dims(meta)
        self.ref_rover = meta.get("rover_xy")
        self.meta = meta

    # The reference only makes sense with the grid calibration it was saved with. Returns
    # a reason string if the grid was re-calibrated (or resized) after the reference was
    # saved; None if it still matches.
    def stale_reason(self, grid=None):
        """Why the reference no longer belongs to grid_calib.json (None = it does).

        A corrected homography is H_ref->now composed with the REFERENCE's grid mapping, so
        a reference saved before the grid was re-calibrated would silently bring back the
        old calibration (or the old grid size)."""
        if grid is None:
            if not GRID.exists():
                return None
            grid = json.loads(GRID.read_text())
        if grid_dims(grid) != self.grid_m:
            return (f"reference was saved for a {self.grid_m[0]:g}x{self.grid_m[1]:g} m grid, "
                    f"grid_calib.json is {grid_dims(grid)[0]:g}x{grid_dims(grid)[1]:g} m")
        if "grid_calibrated_at" in self.meta:
            if self.meta["grid_calibrated_at"] != grid.get("calibrated_at"):
                return "grid_calib.json was re-calibrated after the reference was saved"
            return None
        # legacy reference: compare the two grid->pixel mappings directly
        grid_w_m, grid_h_m = self.grid_m
        corners_m = np.array([[0, 0], [grid_w_m, 0], [grid_w_m, grid_h_m], [0, grid_h_m]], float).reshape(-1, 1, 2)
        ref_corners_px = cv2.perspectiveTransform(corners_m, self.H_m2img_ref).reshape(-1, 2)
        grid_corners_px = cv2.perspectiveTransform(corners_m, np.array(grid["H_m2img"], float)).reshape(-1, 2)
        max_difference_px = float(np.max(np.linalg.norm(ref_corners_px - grid_corners_px, axis=1)))
        if max_difference_px > 3.0:
            return f"grid_calib.json differs from the reference's grid by {max_difference_px:.0f} px"
        return None

    # Register `frame` to the reference and report how far (pixels) the four grid corners
    # moved - the camera-motion size that check() compares against warn_px / fail_px.
    def measure_shift(self, frame, rover_xy=None):
        """Register frame against the reference -> dict(shift_px, H (ref px -> now px), ...)."""
        ref_mask = floor_mask(self.ref.shape, self.H_m2img_ref, self.grid_m, rover_xy=self.ref_rover)
        cur_mask = floor_mask(frame.shape, self.H_m2img_ref, self.grid_m, rover_xy=rover_xy,
                           rover_r=0.30)
        ref_to_now, match_info = register(self.ref, frame, ref_mask, cur_mask)
        if ref_to_now is None:
            return {"shift_px": None, "why": match_info["why"]}
        grid_w_m, grid_h_m = self.grid_m
        corners = np.array([[0, 0], [grid_w_m, 0], [grid_w_m, grid_h_m], [0, grid_h_m]], float).reshape(-1, 1, 2)
        ref_corners_px = cv2.perspectiveTransform(corners, self.H_m2img_ref).reshape(-1, 2)
        now_corners_px = cv2.perspectiveTransform(ref_corners_px.reshape(-1, 1, 2), ref_to_now).reshape(-1, 2)
        shift_px = float(np.max(np.linalg.norm(now_corners_px - ref_corners_px, axis=1)))
        return {"shift_px": round(shift_px, 2), "H": ref_to_now, **match_info}

    # Full check: stale reference -> not ok; else measure the shift and return the
    # corrected grid->pixel homography (reference->now motion composed with the reference
    # mapping), ok = shift below fail_px, moved = shift at least warn_px.
    def check(self, frame, rover_xy=None):
        """-> dict(ok, moved, shift_px, H_m2img (corrected, grid->current pixels), ...)."""
        stale_why = self.stale_reason()
        if stale_why:
            return {"ok": False, "moved": None,
                    "why": f"camera reference is stale ({stale_why}); run "
                           "loop/camera_guard.py --save-reference"}
        measurement = self.measure_shift(frame, rover_xy)
        if measurement["shift_px"] is None:
            return {"ok": False, "moved": None, "why": measurement["why"]}
        ref_to_now, shift_px = measurement.pop("H"), measurement["shift_px"]
        corrected_m2img = ref_to_now @ self.H_m2img_ref
        corrected_m2img /= corrected_m2img[2, 2]
        result = {"ok": shift_px < self.fail_px, "moved": shift_px >= self.warn_px,
               "H_m2img": corrected_m2img, **measurement}
        if shift_px >= self.fail_px:
            result["why"] = (f"camera moved {shift_px:.0f} px - too far to trust a correction; "
                          "re-run calib/calibrate_grid.py then --save-reference")
        return result


# One clean frame from the overhead stream (waits 1 s for a keyframe).
def grab_frame(timeout=20.0):
    sys.path.insert(0, str(SC / "capture"))
    from stream import LatestFrame, STREAM_URL
    camera_stream = LatestFrame(STREAM_URL)
    try:
        if camera_stream.wait_first(timeout) is None:
            raise RuntimeError("no camera frame (stream down / UDP 5000 busy)")
        time.sleep(1.0)              # let the decoder reach a clean keyframe
        return camera_stream.read()[0].copy()
    finally:
        camera_stream.release()


# Store the current frame + the current grid mapping as the new reference.
def save_reference(frame, rover_xy=None):
    REF_DIR.mkdir(exist_ok=True)
    grid = json.loads(GRID.read_text())
    cv2.imwrite(str(REF_IMG), frame)
    grid_w_m, grid_h_m = grid_dims(grid)
    REF_META.write_text(json.dumps({"H_m2img": grid["H_m2img"],
                                    "grid_size_m": grid.get("grid_size_m", 1.0),
                                    "grid_width_m": grid_w_m, "grid_height_m": grid_h_m,
                                    "grid_calibrated_at": grid.get("calibrated_at"),
                                    "rover_xy": rover_xy, "saved": time.ctime()}, indent=2))


# Overwrite grid_calib.json with a corrected homography (old file backed up first).
def write_grid(H_m2img):
    grid = json.loads(GRID.read_text())
    backup = GRID.with_name(f"grid_calib.before_guard_{time.strftime('%Y%m%d_%H%M%S')}.json")
    shutil.copy(GRID, backup)
    H_m2img = np.asarray(H_m2img, float)
    H_img2m = np.linalg.inv(H_m2img)
    H_img2m /= H_img2m[2, 2]
    grid_w_m, grid_h_m = grid_dims(grid)
    corners = np.array([[0, 0], [grid_w_m, 0], [grid_w_m, grid_h_m], [0, grid_h_m]], float)
    grid["H_m2img"] = H_m2img.tolist()
    grid["H_img2m"] = H_img2m.tolist()
    grid["corners_px"] = cv2.perspectiveTransform(
        corners.reshape(-1, 1, 2), H_m2img).reshape(-1, 2).round(1).tolist()
    grid["_updated_by"] = f"loop/camera_guard.py {time.ctime()} (backup {backup.name})"
    GRID.write_text(json.dumps(grid, indent=2))
    return backup


# Warp the reference by known small camera motions; the guard must measure each
# shift to within 1 px.
def self_test():
    """Warp the reference by a known small camera motion; the guard must measure it."""
    guard = CameraGuard()
    image_h, image_w = guard.ref.shape[:2]
    for shift_x_px, shift_y_px, rotation_deg in [(0, 0, 0.0), (6, -4, 0.4), (-12, 8, -0.8)]:
        affine = cv2.getRotationMatrix2D((image_w / 2, image_h / 2), rotation_deg, 1.0)
        affine[0, 2] += shift_x_px
        affine[1, 2] += shift_y_px
        moved_image = cv2.warpAffine(guard.ref, affine, (image_w, image_h), borderMode=cv2.BORDER_REFLECT)
        result = guard.measure_shift(moved_image)
        result["moved"] = result["shift_px"] is not None and result["shift_px"] >= guard.warn_px
        true_motion = np.vstack([affine, [0, 0, 1]])
        grid_w_m, grid_h_m = guard.grid_m
        ref_corners_px = cv2.perspectiveTransform(np.array([[0, 0], [grid_w_m, 0], [grid_w_m, grid_h_m], [0, grid_h_m]], float)
                                     .reshape(-1, 1, 2), guard.H_m2img_ref).reshape(-1, 2)
        true_shift_px = np.max(np.linalg.norm(cv2.perspectiveTransform(ref_corners_px.reshape(-1, 1, 2), true_motion)
                                      .reshape(-1, 2) - ref_corners_px, axis=1))
        print(f"  applied shift {true_shift_px:5.1f} px -> measured {result.get('shift_px')} px  "
              f"moved={result.get('moved')} inliers={result.get('inliers')}")
        if result.get("shift_px") is None or abs(result["shift_px"] - true_shift_px) > 1.0:
            return False
    return True


# Command line: --save-reference | --check [--write] | --self-test.
def main():
    import argparse
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--save-reference", action="store_true")
    parser.add_argument("--check", action="store_true")
    parser.add_argument("--write", action="store_true", help="with --check: rewrite grid_calib.json")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()
    if args.self_test:
        passed = self_test()
        print("SELF-TEST", "PASSED" if passed else "FAILED")
        return 0 if passed else 1
    if args.save_reference:
        frame = grab_frame()
        save_reference(frame)
        print(f"saved {REF_IMG} with the current grid_calib.json homography")
        return 0
    if args.check:
        result = CameraGuard().check(grab_frame())
        print({key: value for key, value in result.items() if key != "H_m2img"})
        if args.write and result.get("moved") and result.get("ok"):
            print(f"grid_calib.json updated (backup {write_grid(result['H_m2img']).name})")
        return 0 if result.get("ok") else 1
    parser.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
