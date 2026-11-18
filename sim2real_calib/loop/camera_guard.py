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


def floor_mask(shape, H_m2img, grid_m=1.0, pad_m=0.35, rover_xy=None, rover_r=0.25):
    """Floor region around the grid (where the static tape/carpet texture is), with the
    rover's disc removed so its motion can't drag the registration."""
    h, w = shape[:2]
    gw, gh = as_dims(grid_m)
    m = np.zeros((h, w), np.uint8)
    poly = np.array([[-pad_m, -pad_m], [gw + pad_m, -pad_m],
                     [gw + pad_m, gh + pad_m], [-pad_m, gh + pad_m]], float)
    px = cv2.perspectiveTransform(poly.reshape(-1, 1, 2), H_m2img).reshape(-1, 2)
    cv2.fillConvexPoly(m, px.astype(np.int32), 255)
    if rover_xy is not None:
        ang = np.linspace(0, 2 * np.pi, 24, endpoint=False)
        disc = np.column_stack([rover_xy[0] + rover_r * np.cos(ang),
                                rover_xy[1] + rover_r * np.sin(ang)])
        dp = cv2.perspectiveTransform(disc.reshape(-1, 1, 2), H_m2img).reshape(-1, 2)
        cv2.fillConvexPoly(m, dp.astype(np.int32), 0)
    return m


def register(ref, cur, mask_ref, mask_cur, n_features=4000):
    """Homography mapping reference pixels -> current pixels (None if unreliable)."""
    orb = cv2.ORB_create(n_features, scaleFactor=1.2, nlevels=8, fastThreshold=10)
    g1 = cv2.cvtColor(ref, cv2.COLOR_BGR2GRAY)
    g2 = cv2.cvtColor(cur, cv2.COLOR_BGR2GRAY)
    k1, d1 = orb.detectAndCompute(g1, mask_ref)
    k2, d2 = orb.detectAndCompute(g2, mask_cur)
    if d1 is None or d2 is None or len(k1) < 50 or len(k2) < 50:
        return None, {"why": "too few features"}
    bf = cv2.BFMatcher(cv2.NORM_HAMMING)
    good = []
    for pair in bf.knnMatch(d1, d2, k=2):
        if len(pair) == 2 and pair[0].distance < 0.75 * pair[1].distance:
            good.append(pair[0])
    if len(good) < 40:
        return None, {"why": f"only {len(good)} feature matches"}
    p1 = np.float32([k1[m.queryIdx].pt for m in good])
    p2 = np.float32([k2[m.trainIdx].pt for m in good])
    H, inl = cv2.findHomography(p1, p2, cv2.RANSAC, 2.0, maxIters=5000, confidence=0.999)
    if H is None:
        return None, {"why": "no homography"}
    n_in = int(inl.sum())
    if n_in < 30:
        return None, {"why": f"only {n_in} RANSAC inliers"}
    return H, {"matches": len(good), "inliers": n_in}


class CameraGuard:
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
        w, h = self.grid_m
        c = np.array([[0, 0], [w, 0], [w, h], [0, h]], float).reshape(-1, 1, 2)
        a = cv2.perspectiveTransform(c, self.H_m2img_ref).reshape(-1, 2)
        b = cv2.perspectiveTransform(c, np.array(grid["H_m2img"], float)).reshape(-1, 2)
        d = float(np.max(np.linalg.norm(a - b, axis=1)))
        if d > 3.0:
            return f"grid_calib.json differs from the reference's grid by {d:.0f} px"
        return None

    def measure_shift(self, frame, rover_xy=None):
        """Register frame against the reference -> dict(shift_px, H (ref px -> now px), ...)."""
        m_ref = floor_mask(self.ref.shape, self.H_m2img_ref, self.grid_m, rover_xy=self.ref_rover)
        m_cur = floor_mask(frame.shape, self.H_m2img_ref, self.grid_m, rover_xy=rover_xy,
                           rover_r=0.30)
        H, info = register(self.ref, frame, m_ref, m_cur)
        if H is None:
            return {"shift_px": None, "why": info["why"]}
        w, h = self.grid_m
        corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], float).reshape(-1, 1, 2)
        c_ref = cv2.perspectiveTransform(corners, self.H_m2img_ref).reshape(-1, 2)
        c_now = cv2.perspectiveTransform(c_ref.reshape(-1, 1, 2), H).reshape(-1, 2)
        shift = float(np.max(np.linalg.norm(c_now - c_ref, axis=1)))
        return {"shift_px": round(shift, 2), "H": H, **info}

    def check(self, frame, rover_xy=None):
        """-> dict(ok, moved, shift_px, H_m2img (corrected, grid->current pixels), ...)."""
        stale = self.stale_reason()
        if stale:
            return {"ok": False, "moved": None,
                    "why": f"camera reference is stale ({stale}); run "
                           "loop/camera_guard.py --save-reference"}
        m = self.measure_shift(frame, rover_xy)
        if m["shift_px"] is None:
            return {"ok": False, "moved": None, "why": m["why"]}
        H, shift = m.pop("H"), m["shift_px"]
        H_new = H @ self.H_m2img_ref
        H_new /= H_new[2, 2]
        res = {"ok": shift < self.fail_px, "moved": shift >= self.warn_px,
               "H_m2img": H_new, **m}
        if shift >= self.fail_px:
            res["why"] = (f"camera moved {shift:.0f} px - too far to trust a correction; "
                          "re-run calib/calibrate_grid.py then --save-reference")
        return res


def grab_frame(timeout=20.0):
    sys.path.insert(0, str(SC / "capture"))
    from stream import LatestFrame, STREAM_URL
    cam = LatestFrame(STREAM_URL)
    try:
        if cam.wait_first(timeout) is None:
            raise RuntimeError("no camera frame (stream down / UDP 5000 busy)")
        time.sleep(1.0)              # let the decoder reach a clean keyframe
        return cam.read()[0].copy()
    finally:
        cam.release()


def save_reference(frame, rover_xy=None):
    REF_DIR.mkdir(exist_ok=True)
    grid = json.loads(GRID.read_text())
    cv2.imwrite(str(REF_IMG), frame)
    w, h = grid_dims(grid)
    REF_META.write_text(json.dumps({"H_m2img": grid["H_m2img"],
                                    "grid_size_m": grid.get("grid_size_m", 1.0),
                                    "grid_width_m": w, "grid_height_m": h,
                                    "grid_calibrated_at": grid.get("calibrated_at"),
                                    "rover_xy": rover_xy, "saved": time.ctime()}, indent=2))


def write_grid(H_m2img):
    grid = json.loads(GRID.read_text())
    backup = GRID.with_name(f"grid_calib.before_guard_{time.strftime('%Y%m%d_%H%M%S')}.json")
    shutil.copy(GRID, backup)
    H_m2img = np.asarray(H_m2img, float)
    H_img2m = np.linalg.inv(H_m2img)
    H_img2m /= H_img2m[2, 2]
    w, h = grid_dims(grid)
    corners = np.array([[0, 0], [w, 0], [w, h], [0, h]], float)
    grid["H_m2img"] = H_m2img.tolist()
    grid["H_img2m"] = H_img2m.tolist()
    grid["corners_px"] = cv2.perspectiveTransform(
        corners.reshape(-1, 1, 2), H_m2img).reshape(-1, 2).round(1).tolist()
    grid["_updated_by"] = f"loop/camera_guard.py {time.ctime()} (backup {backup.name})"
    GRID.write_text(json.dumps(grid, indent=2))
    return backup


def self_test():
    """Warp the reference by a known small camera motion; the guard must measure it."""
    guard = CameraGuard()
    h, w = guard.ref.shape[:2]
    for dx, dy, rot in [(0, 0, 0.0), (6, -4, 0.4), (-12, 8, -0.8)]:
        M = cv2.getRotationMatrix2D((w / 2, h / 2), rot, 1.0)
        M[0, 2] += dx
        M[1, 2] += dy
        moved = cv2.warpAffine(guard.ref, M, (w, h), borderMode=cv2.BORDER_REFLECT)
        res = guard.measure_shift(moved)
        res["moved"] = res["shift_px"] is not None and res["shift_px"] >= guard.warn_px
        Ht = np.vstack([M, [0, 0, 1]])
        gw, gh = guard.grid_m
        c = cv2.perspectiveTransform(np.array([[0, 0], [gw, 0], [gw, gh], [0, gh]], float)
                                     .reshape(-1, 1, 2), guard.H_m2img_ref).reshape(-1, 2)
        truth = np.max(np.linalg.norm(cv2.perspectiveTransform(c.reshape(-1, 1, 2), Ht)
                                      .reshape(-1, 2) - c, axis=1))
        print(f"  applied shift {truth:5.1f} px -> measured {res.get('shift_px')} px  "
              f"moved={res.get('moved')} inliers={res.get('inliers')}")
        if res.get("shift_px") is None or abs(res["shift_px"] - truth) > 1.0:
            return False
    return True


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--save-reference", action="store_true")
    ap.add_argument("--check", action="store_true")
    ap.add_argument("--write", action="store_true", help="with --check: rewrite grid_calib.json")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()
    if args.self_test:
        ok = self_test()
        print("SELF-TEST", "PASSED" if ok else "FAILED")
        return 0 if ok else 1
    if args.save_reference:
        f = grab_frame()
        save_reference(f)
        print(f"saved {REF_IMG} with the current grid_calib.json homography")
        return 0
    if args.check:
        res = CameraGuard().check(grab_frame())
        print({k: v for k, v in res.items() if k != "H_m2img"})
        if args.write and res.get("moved") and res.get("ok"):
            print(f"grid_calib.json updated (backup {write_grid(res['H_m2img']).name})")
        return 0 if res.get("ok") else 1
    ap.print_help()
    return 2


if __name__ == "__main__":
    raise SystemExit(main())
