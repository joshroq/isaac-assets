#!/usr/bin/env python3
"""Grid-as-target calibration for a tape grid of ANY size: pixels <-> floor metres.

The floor grid is a W x H metre rectangle with tape lines every S metres. Click
four anchors (by default the outer corners); the tool then finds EVERY tape
intersection automatically, fits the homography to all of them by least squares
and shows the per-point residuals before saving. Clicks only need to be roughly
on the tape crossing (a couple of cm is fine) - the result does not depend on them.

Metric frame: origin at the BL outer corner as seen in the camera image,
+X along the bottom edge (BL -> BR), +Y along the left edge (BL -> TL), metres.
W is the X extent, H the Y extent, so the 1 x 4 m strip is either --size 1x4
(long side going up the image) or --size 4x1 (long side across the image).

Interactive (one consumer of UDP 5000 at a time - close live_tracking/receive first):
    python3 calibrate_grid.py --source stream --size 1   --spacing 0.25   # the 1x1 m grid
    python3 calibrate_grid.py --source stream --size 2x2 --spacing 0.5
    python3 calibrate_grid.py --source stream --size 1x4 --spacing 0.5
    # click BL, BR, TR, TL (U undo, Q quit). Review: ENTER/S save, U re-click, Q quit.

Outer corners out of view? Click any 4+ known crossings instead:
    python3 calibrate_grid.py --source stream --size 1x4 --spacing 0.5 \\
        --anchors "0,0.5 1,0.5 1,3.5 0,3.5"

Check the saved calibration against the live grid (no clicks; exit 1 if it drifted):
    python3 calibrate_grid.py --verify --source stream [--save /tmp/grid_verify.png]
    # files from before this version store no spacing: add e.g. --spacing 0.25

Scripted:  --image f.png --points-px "x,y x,y x,y x,y" --yes
Sticker-plane target (README): --out plane.json --no-refine  (targets are not tape)

Output grid_calib.json: H_img2m / H_m2img, grid_width_m / grid_height_m /
grid_spacing_m, every detected point with its residual, rms/max residual and an
estimated camera position. Saving the default file also backs up the old one,
re-saves the camera-guard reference and re-expresses the sticker tracker's
camera position in the new grid frame (see --keep-sticker-camera).
"""
import argparse
import json
import os
import shutil
import sys
import time

import cv2
import numpy as np

import grid_target as gt

HERE = os.path.dirname(os.path.abspath(__file__))
SC = os.path.dirname(HERE)
for sub in ("capture", "overhead", "loop"):
    sys.path.insert(0, os.path.join(SC, sub))

DEFAULT_OUT = os.path.join(HERE, "grid_calib.json")
STICKERS = os.path.join(HERE, "sticker_config.json")
C920E_HFOV = 70.4


def grab_frame():
    from stream import LatestFrame, STREAM_URL
    src = LatestFrame(STREAM_URL)
    try:
        f = src.wait_first(20) if src.opened() else None
        if f is not None:
            time.sleep(1.0)               # let the decoder reach a clean keyframe
            f = src.read()[0].copy()
    finally:
        src.release()
    if f is None:
        print("ERROR: no stream. Start the Pi camera service and stop other UDP 5000 "
              "consumers (live_tracking.py, receive.py, metric_overlay.py).", file=sys.stderr)
    return f


def parse_pairs(text):
    pts = [[float(v) for v in p.split(",")] for p in text.split()]
    if not pts or any(len(p) != 2 for p in pts):
        raise ValueError(f"expected 'x,y x,y ...', got {text!r}")
    return np.array(pts, float)


# ---------------------------------------------------------------- interactive UI
def _schematic(img, dims, anchors_m, n_done, origin=(None, 10), box=150):
    """Small map of the grid (to scale) with the clicked/next anchors, top-right."""
    w, h = dims
    s = box / max(w, h)
    bw, bh = int(w * s), int(h * s)
    x0 = img.shape[1] - bw - 40 if origin[0] is None else origin[0]
    y0 = origin[1] + 24
    cv2.rectangle(img, (x0 - 30, y0 - 24), (x0 + bw + 30, y0 + bh + 26), (30, 30, 30), -1)

    def P(m):
        return int(x0 + m[0] * s), int(y0 + bh - m[1] * s)

    cv2.rectangle(img, P((0, h)), P((w, 0)), (255, 160, 0), 2)
    cv2.arrowedLine(img, P((0, 0)), P((min(w, h) * 0.4, 0)), (0, 0, 255), 2, tipLength=0.3)
    cv2.arrowedLine(img, P((0, 0)), P((0, min(w, h) * 0.4)), (0, 255, 0), 2, tipLength=0.3)
    cv2.putText(img, f"{w:g} m (X)", (x0 + bw // 2 - 28, y0 + bh + 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(img, f"{h:g} m (Y)", (x0 - 28, y0 - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (220, 220, 220), 1, cv2.LINE_AA)
    for i, a in enumerate(anchors_m):
        col = (0, 255, 0) if i < n_done else ((0, 255, 255) if i == n_done else (120, 120, 120))
        cv2.circle(img, P(a), 6 if i == n_done else 4, col, -1, cv2.LINE_AA)


def _magnifier(img, frame, at, zoom=4, half=24):
    if at is None:
        return
    x, y = int(at[0]), int(at[1])
    H, W = frame.shape[:2]
    x0, y0 = max(0, x - half), max(0, y - half)
    x1, y1 = min(W, x + half), min(H, y + half)
    crop = frame[y0:y1, x0:x1]
    if crop.size == 0:
        return
    big = cv2.resize(crop, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_NEAREST)
    cx, cy = (x - x0) * zoom, (y - y0) * zoom
    cv2.line(big, (cx, 0), (cx, big.shape[0]), (0, 0, 255), 1)
    cv2.line(big, (0, cy), (big.shape[1], cy), (0, 0, 255), 1)
    bh, bw = big.shape[:2]
    ox, oy = 10, H - bh - 10
    img[oy:oy + bh, ox:ox + bw] = big
    cv2.rectangle(img, (ox, oy), (ox + bw, oy + bh), (255, 255, 255), 1)


def click_anchors(frame, dims, anchors_m, names, start=()):
    pts = list(start)
    state = {"hover": None}
    win = "grid calibration: click anchors  (U undo, Q quit)"

    def on_mouse(event, x, y, flags, _):
        state["hover"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(pts) < len(anchors_m):
            pts.append((float(x), float(y)))

    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, frame.shape[1], frame.shape[0])
    cv2.setMouseCallback(win, on_mouse)
    try:
        while True:
            disp = frame.copy()
            for i, (x, y) in enumerate(pts):
                cv2.drawMarker(disp, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(disp, names[i], (int(x) + 8, int(y) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            if len(pts) >= 2:
                cv2.polylines(disp, [np.int32(pts)], len(pts) == len(anchors_m), (0, 200, 255), 1)
            if len(pts) == len(anchors_m):
                return np.array(pts, float)
            msg = f"click {names[len(pts)]}  - centre of the tape crossing"
            cv2.putText(disp, msg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(disp, msg, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            _schematic(disp, dims, anchors_m, len(pts))
            _magnifier(disp, frame, state["hover"])
            cv2.imshow(win, disp)
            k = cv2.waitKey(20) & 0xFF
            if k in (ord("q"), 27):
                return None
            if k == ord("u") and pts:
                pts.pop()
    finally:
        cv2.destroyWindow(win)


def review(img):
    win = "grid calibration: review  (ENTER/S save, U re-click, Q quit)"
    cv2.namedWindow(win, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(win, img.shape[1], img.shape[0])
    try:
        while True:
            cv2.imshow(win, img)
            k = cv2.waitKey(30) & 0xFF
            if k in (13, 10, ord("s")):
                return "save"
            if k == ord("u"):
                return "redo"
            if k in (ord("q"), 27):
                return "quit"
    finally:
        cv2.destroyWindow(win)


def _text_block(img, lines, org=(10, 28)):
    x, y = org
    for s, col in lines:
        cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, s, (x, y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, col, 1, cv2.LINE_AA)
        y += 24


# ---------------------------------------------------------------- core
def calibrate(frame, dims, spacing, anchors_m, clicks_px, refine=True, border=gt.DEFAULT_BORDER,
              tape_width=gt.TAPE_WIDTH_M):
    """Solve image->metric from the clicks (and, if refine, every tape crossing)."""
    if len(clicks_px) != len(anchors_m) or len(anchors_m) < 4:
        raise ValueError("need one click per anchor and at least 4 anchors")
    H0, _ = cv2.findHomography(np.asarray(clicks_px, float), np.asarray(anchors_m, float), 0)
    if H0 is None:
        raise ValueError("anchors are degenerate (collinear?)")
    out = dict(method="clicked anchors", found=[], tape=None)
    if refine:
        tg, found = gt.detect(frame, H0, dims, spacing, border=border, tape_width=tape_width)
        ok = [f for f in found if f["px"] is not None]
        out.update(found=found, tape=tg)
        if len(ok) >= 4:
            H, used, res = gt.solve([f["px"] for f in ok], [f["m"] for f in ok])
            out.update(H_img2m=H, used=used, res=res, method="tape-intersection fit",
                       points_px=np.array([f["px"] for f in ok]),
                       points_m=np.array([f["m"] for f in ok]))
            return out
        print(f"WARN: only {len(ok)} tape crossings detected - falling back to the raw clicks",
              file=sys.stderr)
    H, used, res = gt.solve(clicks_px, anchors_m)
    out.update(H_img2m=H, used=used, res=res, points_px=np.asarray(clicks_px, float),
               points_m=np.asarray(anchors_m, float))
    return out


def camera_position_estimate(H_m2img, image_size):
    """Camera centre in grid metres from the plane homography (nominal/calibrated K)."""
    import camera_model as cm
    intr = os.path.join(HERE, "camera_intrinsics.yaml")
    K, src = None, f"nominal C920e {C920E_HFOV:g} deg HFOV"
    if os.path.exists(intr):
        try:
            fs = cv2.FileStorage(intr, cv2.FILE_STORAGE_READ)
            K = fs.getNode("camera_matrix").mat()
            src = "calib/camera_intrinsics.yaml"
        except cv2.error:
            K = None
    if K is None or K.shape != (3, 3):
        K = cm.intrinsics_from_fov(image_size, C920E_HFOV)
    _, C = cm.planar_pose_from_homography(H_m2img, K)
    return [float(v) for v in C], src


def build_json(cal, frame, dims, spacing, anchors_m, clicks_px, border=gt.DEFAULT_BORDER):
    H = cal["H_img2m"]
    Hinv = gt.normalized(np.linalg.inv(H))
    stats = gt.residual_stats(cal["res"], cal["used"])
    corners, _ = gt.outer_corners(dims)
    image_size = [int(frame.shape[1]), int(frame.shape[0])]
    cam, cam_src = camera_position_estimate(Hinv, image_size)
    found = cal["found"]
    missed = [dict(m=f["m"].tolist(), why=f["why"]) for f in found if f["px"] is None]
    tape = cal["tape"]
    return {
        "grid_size_m": dims[0] if dims[0] == dims[1] else [dims[0], dims[1]],
        "grid_width_m": dims[0],
        "grid_height_m": dims[1],
        "grid_spacing_m": [spacing[0], spacing[1]],
        "image_size": image_size,
        "corners_px": gt.transform(Hinv, corners).round(2).tolist(),  # BL BR TR TL (fitted)
        "H_img2m": H.tolist(),
        "H_m2img": Hinv.tolist(),
        "method": cal["method"],
        "n_points": stats["n"],
        "n_intersections": len(found) if found else len(anchors_m),
        "rms_residual_m": stats["rms_m"],
        "max_residual_m": stats["max_m"],
        "corner_residual_m": stats["rms_m"],
        "points": [dict(m=[float(a) for a in m], px=[round(float(b), 2) for b in p],
                        residual_mm=round(float(r) * 1000, 2), used=bool(u))
                   for m, p, r, u in zip(cal["points_m"], cal["points_px"], cal["res"], cal["used"])],
        "missed": missed,
        "anchors_m": np.asarray(anchors_m, float).tolist(),
        "anchor_clicks_px": np.asarray(clicks_px, float).round(1).tolist(),
        # tape width used for the border inset (known 1 inch unless --tape-width-mm 0) and the
        # width measured from the image (diagnostic only; it varies with the light)
        "tape_width_m": round(tape.tape_width_used_m, 5) if tape else None,
        "tape_width_measured_m": round(tape.tape_width_measured_m, 4) if tape else None,
        # border tape inside the grid, outer edge on 0/W: its centreline is half a tape in
        "border_tape": border,
        "border_inset_m": round(tape.tape_width_used_m / 2, 5) if (tape and border == "inside") else 0.0,
        "camera_position_est_m": cam,
        "camera_position_est_source": cam_src,
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


def summary_lines(data, cal):
    rms, mx = data["rms_residual_m"] * 1000, data["max_residual_m"] * 1000
    col = (0, 255, 0) if rms <= 8 and mx <= 20 else (0, 200, 255)
    w, h = data["grid_width_m"], data["grid_height_m"]
    sx, sy = data["grid_spacing_m"]
    lines = [(f"{w:g} x {h:g} m grid, tape every {sx:g} x {sy:g} m   [{cal['method']}]", (255, 255, 255)),
             (f"{data['n_points']}/{data['n_intersections']} points used   residual rms "
              f"{rms:.1f} mm  max {mx:.1f} mm", col)]
    dropped = sum(1 for p in data["points"] if not p["used"])
    if dropped or data["missed"]:
        lines.append((f"{len(data['missed'])} crossings not found (red X), {dropped} rejected "
                      "as outliers (orange)", (0, 200, 255)))
    lines.append(("magenta arrows = residual x10 (consistent offsets along one tape = "
                  "that strip is laid off-line)", (220, 220, 220)))
    return lines


def review_image(frame, cal, data, dims, spacing):
    img = frame.copy()
    found = cal["found"] or [dict(m=m, px=p, why=None)
                             for m, p in zip(cal["points_m"], cal["points_px"])]
    gt.draw_detections(img, found, cal["H_img2m"], dims, spacing, cal["res"], cal["used"])
    _text_block(img, summary_lines(data, cal))
    return img


# ---------------------------------------------------------------- side effects of a new grid
def _load(path):
    try:
        with open(path) as f:
            return json.load(f)
    except (OSError, ValueError):
        return None


def camera_shift_px(frame):
    """Pixels the camera moved since the guard reference image (None = unknown)."""
    try:
        import camera_guard
        guard = camera_guard.CameraGuard()
    except Exception:
        return None
    res = guard.measure_shift(frame)
    return res.get("shift_px")


def transfer_point(C, old_grid, new_grid):
    """Re-express a camera centre from the old grid frame in the new one (same camera)."""
    old_m2img = np.array(old_grid["H_m2img"], float)
    new_img2m = np.array(new_grid["H_img2m"], float)
    p = np.array([[C[0], C[1]], [C[0] + 0.05, C[1]], [C[0], C[1] + 0.05]])
    q = gt.transform(new_img2m, gt.transform(old_m2img, p))
    scale = 0.5 * (np.linalg.norm(q[1] - q[0]) + np.linalg.norm(q[2] - q[0])) / 0.05
    return [float(q[0][0]), float(q[0][1]), float(C[2] * scale)]


def same_camera(old_grid, new_grid, shift_px):
    """Did the previous calibration see the floor from this same camera pose? -> (bool, why)."""
    if old_grid is None:
        return False, "no previous grid calibration"
    if gt.grid_dims(old_grid) == gt.grid_dims(new_grid):
        # Same grid: a moved camera shifts the whole tape pattern, a re-click only by click error.
        d = gt.area_disagreement(np.array(new_grid["H_img2m"]), np.array(old_grid["H_m2img"]),
                                 gt.grid_dims(new_grid))
        if d < 0.05:
            return True, f"previous calibration of this grid still fits within {d * 1000:.0f} mm"
        return False, f"previous calibration is {d * 1000:.0f} mm off (camera or tape moved)"
    if shift_px is not None and shift_px < 5.0:
        return True, f"camera-guard reference: camera unmoved ({shift_px:.1f} px)"
    return False, ("new grid and no camera-guard reference to prove the camera is unmoved"
                   if shift_px is None else f"camera moved {shift_px:.0f} px since the guard reference")


def update_sticker_camera(new_grid, old_grid, shift_px, keep):
    cfg = _load(STICKERS)
    if not cfg or cfg.get("camera_position_m") is None:
        return
    old_C = cfg["camera_position_m"]
    source = cfg.get("camera_position_source", "unrecorded")
    same, why = same_camera(old_grid, new_grid, shift_px)
    if same:
        new_C, new_src = transfer_point(old_C, old_grid, new_grid), source
        how = f"{why} -> same camera position re-expressed in the new grid frame"
    else:
        new_C = new_grid["camera_position_est_m"]
        new_src = "grid_homography_estimate"
        how = (f"{why} -> homography estimate ({new_grid['camera_position_est_source']}); "
               "it is approximate - tape-measure the camera and run calibrate_stickers.py "
               "--camera-position-m for precision")
        if source == "measured":
            print("WARN: sticker_config.json had a MEASURED camera position; it belongs to the "
                  "old setup. Re-measure and run calibrate_stickers.py --camera-position-m.",
                  file=sys.stderr)
    fmt = lambda c: "(" + ", ".join(f"{v:.3f}" for v in c) + ")"
    print(f"sticker camera position {fmt(old_C)} -> {fmt(new_C)} m\n  {how}")
    if keep:
        print("  --keep-sticker-camera: sticker_config.json NOT changed")
        return
    backup = STICKERS.replace(".json", f".before_regrid_{time.strftime('%Y%m%d_%H%M%S')}.json")
    shutil.copy(STICKERS, backup)
    cfg["camera_position_m"] = new_C
    cfg["camera_position_source"] = new_src
    tmp = STICKERS + ".tmp"
    with open(tmp, "w") as f:
        json.dump(cfg, f, indent=2)
        f.write("\n")
    os.replace(tmp, STICKERS)
    print(f"  updated {os.path.relpath(STICKERS, SC)} (backup {os.path.basename(backup)})")


def rover_xy(frame, H_img2m):
    try:
        from sticker_tracker import StickerTracker
        r = StickerTracker(STICKERS).detect(frame, H_img2m)
        return None if r is None else [float(v) for v in r["center_m"]]
    except Exception:
        return None


def save(data, frame, out, keep_sticker_camera):
    is_default = os.path.abspath(out) == os.path.abspath(DEFAULT_OUT)
    old = _load(out) if os.path.exists(out) else None
    shift = camera_shift_px(frame) if is_default else None
    if old is not None:
        backup = out.replace(".json", f".before_regrid_{time.strftime('%Y%m%d_%H%M%S')}.json")
        shutil.copy(out, backup)
        print(f"backed up the previous calibration -> {os.path.basename(backup)}")
    tmp = out + ".tmp"
    with open(tmp, "w") as f:
        json.dump(data, f, indent=2)
        f.write("\n")
    os.replace(tmp, out)
    print(f"wrote {out}")
    if not is_default:
        return
    update_sticker_camera(data, old, shift, keep_sticker_camera)
    try:
        import camera_guard
        camera_guard.save_reference(frame, rover_xy(frame, np.array(data["H_img2m"])))
        print("saved a new camera-guard reference (loop/stations/camera_reference.*)")
    except Exception as exc:
        print(f"WARN: camera-guard reference not saved ({exc}); run "
              "loop/camera_guard.py --save-reference", file=sys.stderr)
    if old is not None and gt.grid_dims(old) != gt.grid_dims(data):
        print("NOTE: the grid frame changed - stations taught in loop/stations/ use the old "
              "frame; re-teach them (loop/stations.py teach ...) before closed-loop runs.")


# ---------------------------------------------------------------- verify
def verify(frame, calib_path, save_png=None, max_drift_mm=10.0, show=False, spacing=None,
           border=gt.DEFAULT_BORDER, tape_width=gt.TAPE_WIDTH_M):
    calib = _load(calib_path)
    if calib is None:
        print(f"ERROR: cannot read {calib_path}", file=sys.stderr)
        return 2
    dims = gt.grid_dims(calib)
    spacing = gt.parse_spacing(spacing, dims) if spacing else gt.grid_spacing(calib)
    if calib.get("image_size") != [frame.shape[1], frame.shape[0]]:
        print("ERROR: camera resolution differs from the calibration", file=sys.stderr)
        return 2
    H_saved = np.array(calib["H_img2m"], float)
    tg, found = gt.detect(frame, H_saved, dims, spacing, rounds=1, border=border, tape_width=tape_width)
    if calib.get("border_tape", "centre") != border:
        print(f"NOTE: the saved calibration used border tape '{calib.get('border_tape', 'centre')}'"
              f" but the grid is checked as '{border}' (tape {tg.tape_width_used_m*1000:.1f} mm) -"
              " re-run calibrate_grid.py if this fails")
    ok = [f for f in found if f["px"] is not None]
    print(f"grid {dims[0]:g} x {dims[1]:g} m, spacing {spacing[0]:g} x {spacing[1]:g} m: "
          f"{len(ok)}/{len(found)} tape crossings found")
    if len(ok) < 4:
        print("FAIL: too few crossings found - camera moved a lot, grid occluded, or wrong "
              "--size/--spacing. Re-run calibrate_grid.py.")
        return 1
    px, m = np.array([f["px"] for f in ok]), np.array([f["m"] for f in ok])
    saved_res = np.linalg.norm(gt.transform(H_saved, px) - m, axis=1)
    H_fresh, used, fresh_res = gt.solve(px, m)
    drift = gt.area_disagreement(H_saved, np.linalg.inv(H_fresh), dims) * 1000
    s_saved, s_fresh = gt.residual_stats(saved_res), gt.residual_stats(fresh_res, used)
    print(f"  saved calibration vs crossings : rms {s_saved['rms_m']*1000:5.1f} mm  "
          f"max {s_saved['max_m']*1000:5.1f} mm")
    print(f"  best fit to these crossings    : rms {s_fresh['rms_m']*1000:5.1f} mm  "
          f"max {s_fresh['max_m']*1000:5.1f} mm   (tape layout / detection floor)")
    print(f"  saved vs best fit, worst point over the grid area: {drift:5.1f} mm")
    passed = drift <= max_drift_mm
    print(("PASS" if passed else "FAIL") + f": calibration error {drift:.1f} mm "
          f"({'<=' if passed else '>'} {max_drift_mm:g} mm)"
          + ("" if passed else " - re-run calibrate_grid.py"))
    if save_png or show:
        img = frame.copy()
        gt.draw_detections(img, found, H_saved, dims, spacing, saved_res)
        _text_block(img, [(f"VERIFY {'PASS' if passed else 'FAIL'}: saved calibration off by up to "
                           f"{drift:.1f} mm over the grid", (0, 255, 0) if passed else (0, 0, 255)),
                          (f"{len(ok)}/{len(found)} crossings; saved rms {s_saved['rms_m']*1000:.1f} mm, "
                           f"best possible {s_fresh['rms_m']*1000:.1f} mm", (255, 255, 255)),
                          ("magenta = residual of the SAVED calibration x10", (220, 220, 220))])
        if save_png:
            cv2.imwrite(save_png, img)
            print(f"wrote {save_png}")
        if show:
            cv2.imshow("grid verify (any key)", img)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    return 0 if passed else 1


# ---------------------------------------------------------------- main
def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default=None, help="'stream' to grab the live overhead feed")
    ap.add_argument("--image", default=None, help="calibrate from a saved frame")
    ap.add_argument("--size", default=None,
                    help="grid W x H in metres, e.g. 1, 2x2, 1x4, 4x1 (default: the saved grid)")
    ap.add_argument("--spacing", default=None,
                    help="tape line spacing in metres, e.g. 0.25 or 0.5,0.25 (default: saved "
                         "spacing if the size matches, else outer edges only)")
    ap.add_argument("--anchors", default=None,
                    help="metric points to click, 'x,y x,y ...' (>=4; default: the 4 outer corners)")
    ap.add_argument("--points-px", "--corners", dest="points_px", default=None,
                    help="non-interactive: pixel clicks for the anchors, 'x,y x,y ...'")
    ap.add_argument("--no-refine", action="store_true",
                    help="use only the clicks (no tape detection) - for non-tape targets")
    ap.add_argument("--yes", action="store_true", help="save without the review window")
    ap.add_argument("--tape-width-mm", type=float, default=gt.TAPE_WIDTH_M * 1000,
                    help="tape width for the border inset (default 25.4 = 1 inch, the grid's tape); "
                         "0 = measure it from the image")
    ap.add_argument("--border", choices=gt.BORDERS, default=gt.DEFAULT_BORDER,
                    help="inside (default): border tape lies inside the grid with its OUTER edge "
                         "on 0/W m (clicked outer corners = exact metres); inner lines are "
                         "measured to the tape centre. centre: every line to its centre.")
    ap.add_argument("--out", default=DEFAULT_OUT)
    ap.add_argument("--keep-sticker-camera", action="store_true",
                    help="do not touch sticker_config.json camera_position_m")
    ap.add_argument("--verify", action="store_true",
                    help="check the saved --out calibration against the live/--image grid")
    ap.add_argument("--max-drift-mm", type=float, default=10.0, help="--verify pass threshold")
    ap.add_argument("--save", default=None, help="write the review/verify image here")
    args = ap.parse_args()

    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"ERROR: cannot read {args.image}", file=sys.stderr)
            return 2
    elif args.source == "stream":
        frame = grab_frame()
        if frame is None:
            return 3
    else:
        print("ERROR: give --image PATH or --source stream", file=sys.stderr)
        return 2

    if args.verify:
        try:
            return verify(frame, args.out, args.save, args.max_drift_mm,
                          show=bool(os.environ.get("DISPLAY")) and not args.save,
                          spacing=args.spacing, border=args.border,
                          tape_width=args.tape_width_mm / 1000 or None)
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            return 2

    saved = _load(args.out) or {}
    try:
        dims = gt.parse_size(args.size) if args.size else (gt.grid_dims(saved) if saved else (1.0, 1.0))
        if args.spacing is not None:
            spacing = gt.parse_spacing(args.spacing, dims)
        elif saved.get("grid_spacing_m") and gt.grid_dims(saved) == dims:
            spacing = gt.parse_spacing(",".join(map(str, saved["grid_spacing_m"])), dims)
        else:
            spacing = dims
        if args.anchors:
            anchors_m = parse_pairs(args.anchors)
            names = [f"({x:g},{y:g})" for x, y in anchors_m]
        else:
            anchors_m, names = gt.outer_corners(dims)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2
    print(f"grid {dims[0]:g} x {dims[1]:g} m (X x Y), tape every {spacing[0]:g} x {spacing[1]:g} m, "
          f"{len(gt.intersections(dims, spacing))} crossings")

    interactive = not args.points_px
    if interactive and not os.environ.get("DISPLAY"):
        print("ERROR: no DISPLAY for clicking; pass --points-px", file=sys.stderr)
        return 2
    clicks = parse_pairs(args.points_px) if args.points_px else None
    while True:
        if clicks is None:
            clicks = click_anchors(frame, dims, anchors_m, names)
            if clicks is None:
                print("cancelled")
                return 1
        try:
            cal = calibrate(frame, dims, spacing, anchors_m, clicks, refine=not args.no_refine,
                            border=args.border, tape_width=args.tape_width_mm / 1000 or None)
            data = build_json(cal, frame, dims, spacing, anchors_m, clicks, border=args.border)
        except ValueError as exc:
            print(f"ERROR: {exc}", file=sys.stderr)
            if not interactive:
                return 2
            clicks = None
            continue
        for s, _ in summary_lines(data, cal):
            print("  " + s)
        img = review_image(frame, cal, data, dims, spacing)
        if args.save:
            cv2.imwrite(args.save, img)
            print(f"wrote {args.save}")
        if args.yes or not interactive:
            break
        choice = review(img)
        if choice == "save":
            break
        if choice == "quit":
            print("cancelled")
            return 1
        clicks = None
    if data["rms_residual_m"] > 0.015:
        print("WARN: residual rms > 15 mm - check --size/--spacing and the anchor order",
              file=sys.stderr)
    save(data, frame, args.out, args.keep_sticker_camera)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
