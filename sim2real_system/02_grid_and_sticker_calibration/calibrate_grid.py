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


# Capture one image from the Raspberry Pi camera stream.
# Returns a BGR NumPy image (height x width x 3), or None.
# Theory: digital image sampling; no homography is calculated here.
def grab_frame():
    from stream import LatestFrame, STREAM_URL
    camera_stream = LatestFrame(STREAM_URL)
    try:
        frame = camera_stream.wait_first(20) if camera_stream.opened() else None
        if frame is not None:
            time.sleep(1.0)               # let the decoder reach a clean keyframe
            frame = camera_stream.read()[0].copy()
    finally:
        camera_stream.release()
    if frame is None:
        print("ERROR: no stream. Start the Pi camera service and stop other UDP 5000 "
              "consumers (live_tracking.py, receive.py, metric_overlay.py).", file=sys.stderr)
    return frame


# Convert CLI pairs such as "0,0 1,0" into an N x 2 float array.
# Columns are X/Y metres for anchors or u/v pixels for clicks.
def parse_pairs(text):
    coordinate_pairs = [[float(coordinate) for coordinate in pair_text.split(",")] for pair_text in text.split()]
    if not coordinate_pairs or any(len(pair_text) != 2 for pair_text in coordinate_pairs):
        raise ValueError(f"expected 'x,y x,y ...', got {text!r}")
    return np.array(coordinate_pairs, float)


# ---------------------------------------------------------------- interactive UI
# ---------------------------------------------------------------- interactive UI
# Draw a miniature metric grid. The scale pixels_per_metre = box/max(W,H)
# is ONLY a UI drawing scale, not the camera-to-floor homography.
# Y is negated for screen drawing because pixel rows increase downward.
def _schematic(img, dims, anchors_m, n_done, origin=(None, 10), box=150):
    """Small map of the grid (to scale) with the clicked/next anchors, top-right."""
    grid_width_m, grid_height_m = dims
    pixels_per_metre = box / max(grid_width_m, grid_height_m)
    box_width_px, box_height_px = int(grid_width_m * pixels_per_metre), int(grid_height_m * pixels_per_metre)
    box_left_px = img.shape[1] - box_width_px - 40 if origin[0] is None else origin[0]
    box_top_px = origin[1] + 24
    cv2.rectangle(img, (box_left_px - 30, box_top_px - 24), (box_left_px + box_width_px + 30, box_top_px + box_height_px + 26), (30, 30, 30), -1)

    def P(m):
        return int(box_left_px + m[0] * pixels_per_metre), int(box_top_px + box_height_px - m[1] * pixels_per_metre)

    cv2.rectangle(img, P((0, grid_height_m)), P((grid_width_m, 0)), (255, 160, 0), 2)
    cv2.arrowedLine(img, P((0, 0)), P((min(grid_width_m, grid_height_m) * 0.4, 0)), (0, 0, 255), 2, tipLength=0.3)
    cv2.arrowedLine(img, P((0, 0)), P((0, min(grid_width_m, grid_height_m) * 0.4)), (0, 255, 0), 2, tipLength=0.3)
    cv2.putText(img, f"{grid_width_m:g} m (X)", (box_left_px + box_width_px // 2 - 28, box_top_px + box_height_px + 18), cv2.FONT_HERSHEY_SIMPLEX,
                0.4, (220, 220, 220), 1, cv2.LINE_AA)
    cv2.putText(img, f"{grid_height_m:g} m (Y)", (box_left_px - 28, box_top_px - 8), cv2.FONT_HERSHEY_SIMPLEX, 0.4,
                (220, 220, 220), 1, cv2.LINE_AA)
    for anchor_index, anchor_m in enumerate(anchors_m):
        marker_color = (0, 255, 0) if anchor_index < n_done else ((0, 255, 255) if anchor_index == n_done else (120, 120, 120))
        cv2.circle(img, P(anchor_m), 6 if anchor_index == n_done else 4, marker_color, -1, cv2.LINE_AA)


# Crop an image window and resize it by an integer zoom factor.
# Purely visual: nearest-neighbor interpolation duplicates original pixels.
def _magnifier(img, frame, at, zoom=4, half=24):
    if at is None:
        return
    cursor_x, cursor_y = int(at[0]), int(at[1])
    frame_height_px, frame_width_px = frame.shape[:2]
    crop_left, crop_top = max(0, cursor_x - half), max(0, cursor_y - half)
    crop_right, crop_bottom = min(frame_width_px, cursor_x + half), min(frame_height_px, cursor_y + half)
    crop = frame[crop_top:crop_bottom, crop_left:crop_right]
    if crop.size == 0:
        return
    zoomed_crop = cv2.resize(crop, None, fx=zoom, fy=zoom, interpolation=cv2.INTER_NEAREST)
    crosshair_x, crosshair_y = (cursor_x - crop_left) * zoom, (cursor_y - crop_top) * zoom
    cv2.line(zoomed_crop, (crosshair_x, 0), (crosshair_x, zoomed_crop.shape[0]), (0, 0, 255), 1)
    cv2.line(zoomed_crop, (0, crosshair_y), (zoomed_crop.shape[1], crosshair_y), (0, 0, 255), 1)
    zoom_height, zoom_width = zoomed_crop.shape[:2]
    overlay_x, overlay_y = 10, frame_height_px - zoom_height - 10
    img[overlay_y:overlay_y + zoom_height, overlay_x:overlay_x + zoom_width] = zoomed_crop
    cv2.rectangle(img, (overlay_x, overlay_y), (overlay_x + zoom_width, overlay_y + zoom_height), (255, 255, 255), 1)


# Register corresponding image pixels for known grid anchors.
# The user clicks in EXACTLY the order of anchor_coordinates_m.
# These pairs initialize the planar homography in calibrate().
def click_anchors(frame, dims, anchors_m, names, start=()):
    clicked_pixels = list(start)
    mouse_state = {"hover": None}
    window_name = "grid calibration: click anchors  (U undo, Q quit)"

    def on_mouse(event, x, y, flags, _):
        mouse_state["hover"] = (x, y)
        if event == cv2.EVENT_LBUTTONDOWN and len(clicked_pixels) < len(anchors_m):
            clicked_pixels.append((float(x), float(y)))

    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, frame.shape[1], frame.shape[0])
    cv2.setMouseCallback(window_name, on_mouse)
    try:
        while True:
            display_frame = frame.copy()
            for point_index, (x, y) in enumerate(clicked_pixels):
                cv2.drawMarker(display_frame, (int(x), int(y)), (0, 255, 0), cv2.MARKER_CROSS, 16, 2)
                cv2.putText(display_frame, names[point_index], (int(x) + 8, int(y) - 8),
                            cv2.FONT_HERSHEY_SIMPLEX, 0.55, (0, 255, 0), 2, cv2.LINE_AA)
            if len(clicked_pixels) >= 2:
                cv2.polylines(display_frame, [np.int32(clicked_pixels)], len(clicked_pixels) == len(anchors_m), (0, 200, 255), 1)
            if len(clicked_pixels) == len(anchors_m):
                return np.array(clicked_pixels, float)
            instruction = f"click {names[len(clicked_pixels)]}  - centre of the tape crossing"
            cv2.putText(display_frame, instruction, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(display_frame, instruction, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 255, 255), 2, cv2.LINE_AA)
            _schematic(display_frame, dims, anchors_m, len(clicked_pixels))
            _magnifier(display_frame, frame, mouse_state["hover"])
            cv2.imshow(window_name, display_frame)
            pressed_key = cv2.waitKey(20) & 0xFF
            if pressed_key in (ord("q"), 27):
                return None
            if pressed_key == ord("u") and clicked_pixels:
                clicked_pixels.pop()
    finally:
        cv2.destroyWindow(window_name)


# Return save / redo / quit from the OpenCV review window.
def review(img):
    window_name = "grid calibration: review  (ENTER/S save, U re-click, Q quit)"
    cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
    cv2.resizeWindow(window_name, img.shape[1], img.shape[0])
    try:
        while True:
            cv2.imshow(window_name, img)
            pressed_key = cv2.waitKey(30) & 0xFF
            if pressed_key in (13, 10, ord("s")):
                return "save"
            if pressed_key == ord("u"):
                return "redo"
            if pressed_key in (ord("q"), 27):
                return "quit"
    finally:
        cv2.destroyWindow(window_name)


# Draw outlined readable text; drawing twice does not change data.
def _text_block(img, lines, org=(10, 28)):
    text_x, text_y = org
    for message, color in lines:
        cv2.putText(img, message, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, message, (text_x, text_y), cv2.FONT_HERSHEY_SIMPLEX, 0.6, color, 1, cv2.LINE_AA)
        text_y += 24


# ---------------------------------------------------------------- core
# ---------------------------------------------------------------- core
# MAIN ESTIMATION STEP.
# (1) Initial 3x3 planar homography from clicked pixel-to-metre pairs.
# (2) grid_target.detect() searches for grid crossings around their predicted
# locations (implementation lives in grid_target.py).
# (3) grid_target.solve() refits from detected correspondences and may
# reject outliers. Its exact solver, objective and rejection logic must
# be inspected in grid_target.py rather than assumed here.
# (4) If too few intersections are found, refit from manual clicks.
# H maps [u,v,1] to proportional [X,Y,1], followed by division by w.
def calibrate(frame, dims, spacing, anchors_m, clicks_px, refine=True, border=gt.DEFAULT_BORDER,
              tape_width=gt.TAPE_WIDTH_M):
    """Solve image->metric from the clicks (and, if refine, every tape crossing)."""
    if len(clicks_px) != len(anchors_m) or len(anchors_m) < 4:
        raise ValueError("need one click per anchor and at least 4 anchors")
    initial_img_to_m, _ = cv2.findHomography(np.asarray(clicks_px, float), np.asarray(anchors_m, float), 0)
    if initial_img_to_m is None:
        raise ValueError("anchors are degenerate (collinear?)")
    result = dict(method="clicked anchors", found=[], tape=None)
    if refine:
        tape_result, detected_intersections = gt.detect(frame, initial_img_to_m, dims, spacing, border=border, tape_width=tape_width)
        valid_intersections = [intersection for intersection in detected_intersections if intersection["px"] is not None]
        result.update(found=detected_intersections, tape=tape_result)
        if len(valid_intersections) >= 4:
            image_to_m_homography, inlier_mask, residuals_m = gt.solve([intersection["px"] for intersection in valid_intersections], [intersection["m"] for intersection in valid_intersections])
            result.update(H_img2m=image_to_m_homography, used=inlier_mask, res=residuals_m, method="tape-intersection fit",
                       points_px=np.array([intersection["px"] for intersection in valid_intersections]),
                       points_m=np.array([intersection["m"] for intersection in valid_intersections]))
            return result
        print(f"WARN: only {len(valid_intersections)} tape crossings detected - falling back to the raw clicks",
              file=sys.stderr)
    image_to_m_homography, inlier_mask, residuals_m = gt.solve(clicks_px, anchors_m)
    result.update(H_img2m=image_to_m_homography, used=inlier_mask, res=residuals_m, points_px=np.asarray(clicks_px, float),
               points_m=np.asarray(anchors_m, float))
    return result


# Estimate the 3D camera centre using grid->image H and intrinsic K.
# A calibrated K is preferred; a nominal field-of-view K is a fallback.
# Pose decomposition is delegated to camera_model.planar_pose_from_homography().
# Not a tape-measured or independently verified 3D camera position.
def camera_position_estimate(H_m2img, image_size):
    """Camera centre in grid metres from the plane homography (nominal/calibrated K)."""
    import camera_model as cm
    intrinsics_path = os.path.join(HERE, "camera_intrinsics.yaml")
    intrinsic_matrix, intrinsics_source = None, f"nominal C920e {C920E_HFOV:g} deg HFOV"
    if os.path.exists(intrinsics_path):
        try:
            file_storage = cv2.FileStorage(intrinsics_path, cv2.FILE_STORAGE_READ)
            intrinsic_matrix = file_storage.getNode("camera_matrix").mat()
            intrinsics_source = "calib/camera_intrinsics.yaml"
        except cv2.error:
            intrinsic_matrix = None
    if intrinsic_matrix is None or intrinsic_matrix.shape != (3, 3):
        intrinsic_matrix = cm.intrinsics_from_fov(image_size, C920E_HFOV)
    _, camera_center_m = cm.planar_pose_from_homography(H_m2img, intrinsic_matrix)
    return [float(v) for v in camera_center_m], intrinsics_source


# Serialize calibration, inverses, detected-point quality, and metadata.
# The homography inverse maps planar floor metres back to image pixels.
# WARNING: corner_residual_m is currently set equal to global RMS residual;
# it is NOT independently calculated from four corner errors.
def build_json(cal, frame, dims, spacing, anchors_m, clicks_px, border=gt.DEFAULT_BORDER):
    image_to_m_homography = cal["H_img2m"]
    m_to_image_homography = gt.normalized(np.linalg.inv(image_to_m_homography))
    residual_statistics = gt.residual_stats(cal["res"], cal["used"])
    grid_corners_m, _ = gt.outer_corners(dims)
    image_size = [int(frame.shape[1]), int(frame.shape[0])]
    camera_center_m, camera_center_source = camera_position_estimate(m_to_image_homography, image_size)
    intersections = cal["found"]
    missed_intersections = [dict(m=intersection["m"].tolist(), why=intersection["why"]) for intersection in intersections if intersection["px"] is None]
    tape_detection = cal["tape"]
    return {
        "grid_size_m": dims[0] if dims[0] == dims[1] else [dims[0], dims[1]],
        "grid_width_m": dims[0],
        "grid_height_m": dims[1],
        "grid_spacing_m": [spacing[0], spacing[1]],
        "image_size": image_size,
        "corners_px": gt.transform(m_to_image_homography, grid_corners_m).round(2).tolist(),  # BL BR TR TL (fitted)
        "H_img2m": image_to_m_homography.tolist(),
        "H_m2img": m_to_image_homography.tolist(),
        "method": cal["method"],
        "n_points": residual_statistics["n"],
        "n_intersections": len(intersections) if intersections else len(anchors_m),
        "rms_residual_m": residual_statistics["rms_m"],
        "max_residual_m": residual_statistics["max_m"],
        "corner_residual_m": residual_statistics["rms_m"],
        "points": [dict(m=[float(coordinate) for coordinate in metric_point], px=[round(float(pixel_coordinate), 2) for pixel_coordinate in pixel_point],
                        residual_mm=round(float(residual_m) * 1000, 2), used=bool(is_used))
                   for metric_point, pixel_point, residual_m, is_used in zip(cal["points_m"], cal["points_px"], cal["res"], cal["used"])],
        "missed": missed_intersections,
        "anchors_m": np.asarray(anchors_m, float).tolist(),
        "anchor_clicks_px": np.asarray(clicks_px, float).round(1).tolist(),
        # tape width used for the border inset (known 1 inch unless --tape-width-mm 0) and the
        # width measured from the image (diagnostic only; it varies with the light)
        "tape_width_m": round(tape_detection.tape_width_used_m, 5) if tape_detection else None,
        "tape_width_measured_m": round(tape_detection.tape_width_measured_m, 4) if tape_detection else None,
        # border tape inside the grid, outer edge on 0/W: its centreline is half a tape in
        "border_tape": border,
        "border_inset_m": round(tape_detection.tape_width_used_m / 2, 5) if (tape_detection and border == "inside") else 0.0,
        "camera_position_est_m": camera_center_m,
        "camera_position_est_source": camera_center_source,
        "calibrated_at": time.strftime("%Y-%m-%d %H:%M:%S"),
    }


# Construct human-readable quality diagnostics. Green thresholds
# (8 mm RMS and 20 mm max) are UI heuristics, not proven accuracy bounds.
def summary_lines(data, cal):
    rms_mm, maximum_mm = data["rms_residual_m"] * 1000, data["max_residual_m"] * 1000
    quality_color = (0, 255, 0) if rms_mm <= 8 and maximum_mm <= 20 else (0, 200, 255)
    grid_width_m, grid_height_m = data["grid_width_m"], data["grid_height_m"]
    spacing_x_m, spacing_y_m = data["grid_spacing_m"]
    lines = [(f"{grid_width_m:g} x {grid_height_m:g} m grid, tape every {spacing_x_m:g} x {spacing_y_m:g} m   [{cal['method']}]", (255, 255, 255)),
             (f"{data['n_points']}/{data['n_intersections']} points used   residual rms "
              f"{rms_mm:.1f} mm  max {maximum_mm:.1f} mm", quality_color)]
    dropped = sum(1 for point in data["points"] if not point["used"])
    if dropped or data["missed"]:
        lines.append((f"{len(data['missed'])} crossings not found (red X), {dropped} rejected "
                      "as outliers (orange)", (0, 200, 255)))
    lines.append(("magenta arrows = residual x10 (consistent offsets along one tape = "
                  "that strip is laid off-line)", (220, 220, 220)))
    return lines


# Draw detected points, outliers, residual arrows and summary.
# Rendering is delegated partly to grid_target.draw_detections().
def review_image(frame, cal, data, dims, spacing):
    review_frame = frame.copy()
    detections = cal["found"] or [dict(m=metric_point, px=pixel_point, why=None)
                             for metric_point, pixel_point in zip(cal["points_m"], cal["points_px"])]
    gt.draw_detections(review_frame, detections, cal["H_img2m"], dims, spacing, cal["res"], cal["used"])
    _text_block(review_frame, summary_lines(data, cal))
    return review_frame


# ---------------------------------------------------------------- side effects of a new grid
# ---------------------------------------------------------------- side effects of a new grid
# Load JSON or return None on parse and filesystem errors.
def _load(path):
    try:
        with open(path) as json_file:
            return json.load(json_file)
    except (OSError, ValueError):
        return None


# Ask camera_guard to measure pixel shift from a saved reference.
# This is not grid homography estimation; algorithm is in camera_guard.py.
def camera_shift_px(frame):
    """Pixels the camera moved since the guard reference image (None = unknown)."""
    try:
        import camera_guard
        camera_guard_instance = camera_guard.CameraGuard()
    except Exception:
        return None
    shift_result = camera_guard_instance.measure_shift(frame)
    return shift_result.get("shift_px")


# Re-express the old camera centre XY through old grid -> pixels ->
# new grid. Approximate Z rescaling from TWO 5-cm local basis probes.
# The resulting Z scale is heuristic; homography does not directly give
# a globally valid 3D coordinate transform for an off-plane camera centre.
def transfer_point(C, old_grid, new_grid):
    """Re-express a camera centre from the old grid frame in the new one (same camera)."""
    old_m_to_image = np.array(old_grid["H_m2img"], float)
    new_image_to_m = np.array(new_grid["H_img2m"], float)
    sample_points_old_m = np.array([[C[0], C[1]], [C[0] + 0.05, C[1]], [C[0], C[1] + 0.05]])
    sample_points_new_m = gt.transform(new_image_to_m, gt.transform(old_m_to_image, sample_points_old_m))
    local_scale_ratio = 0.5 * (np.linalg.norm(sample_points_new_m[1] - sample_points_new_m[0]) + np.linalg.norm(sample_points_new_m[2] - sample_points_new_m[0])) / 0.05
    return [float(sample_points_new_m[0][0]), float(sample_points_new_m[0][1]), float(C[2] * local_scale_ratio)]


# Decision heuristic: compare grid calibrations over an area for equal
# dimensions, or rely on camera_guard shift (<5 pixels) for new dimensions.
# A similar image/grid does not rigorously prove unchanged camera pose.
def same_camera(old_grid, new_grid, shift_px):
    """Did the previous calibration see the floor from this same camera pose? -> (bool, why)."""
    if old_grid is None:
        return False, "no previous grid calibration"
    if gt.grid_dims(old_grid) == gt.grid_dims(new_grid):
        # Same grid: a moved camera shifts the whole tape pattern, a re-click only by click error.
        calibration_difference_m = gt.area_disagreement(np.array(new_grid["H_img2m"]), np.array(old_grid["H_m2img"]),
                                 gt.grid_dims(new_grid))
        if calibration_difference_m < 0.05:
            return True, f"previous calibration of this grid still fits within {calibration_difference_m * 1000:.0f} mm"
        return False, f"previous calibration is {calibration_difference_m * 1000:.0f} mm off (camera or tape moved)"
    if shift_px is not None and shift_px < 5.0:
        return True, f"camera-guard reference: camera unmoved ({shift_px:.1f} px)"
    return False, ("new grid and no camera-guard reference to prove the camera is unmoved"
                   if shift_px is None else f"camera moved {shift_px:.0f} px since the guard reference")


# Carry sticker tracker camera coordinates forward when safe, else
# replace with nominal homography estimate. Backup is made before editing.
def update_sticker_camera(new_grid, old_grid, shift_px, keep):
    sticker_config = _load(STICKERS)
    if not sticker_config or sticker_config.get("camera_position_m") is None:
        return
    old_camera_center_m = sticker_config["camera_position_m"]
    old_position_source = sticker_config.get("camera_position_source", "unrecorded")
    camera_is_same, camera_comparison_reason = same_camera(old_grid, new_grid, shift_px)
    if camera_is_same:
        new_camera_center_m, new_position_source = transfer_point(old_camera_center_m, old_grid, new_grid), old_position_source
        change_explanation = f"{camera_comparison_reason} -> same camera position re-expressed in the new grid frame"
    else:
        new_camera_center_m = new_grid["camera_position_est_m"]
        new_position_source = "grid_homography_estimate"
        change_explanation = (f"{camera_comparison_reason} -> homography estimate ({new_grid['camera_position_est_source']}); "
               "it is approximate - tape-measure the camera and run calibrate_stickers.py "
               "--camera-position-m for precision")
        if old_position_source == "measured":
            print("WARN: sticker_config.json had a MEASURED camera position; it belongs to the "
                  "old setup. Re-measure and run calibrate_stickers.py --camera-position-m.",
                  file=sys.stderr)
    format_position = lambda c: "(" + ", ".join(f"{value:.3f}" for value in c) + ")"
    print(f"sticker camera position {format_position(old_camera_center_m)} -> {format_position(new_camera_center_m)} m\n  {change_explanation}")
    if keep:
        print("  --keep-sticker-camera: sticker_config.json NOT changed")
        return
    backup = STICKERS.replace(".json", f".before_regrid_{time.strftime('%Y%m%d_%H%M%S')}.json")
    shutil.copy(STICKERS, backup)
    sticker_config["camera_position_m"] = new_camera_center_m
    sticker_config["camera_position_source"] = new_position_source
    temporary_path = STICKERS + ".tmp"
    with open(temporary_path, "w") as f:
        json.dump(sticker_config, f, indent=2)
        f.write("\n")
    os.replace(temporary_path, STICKERS)
    print(f"  updated {os.path.relpath(STICKERS, SC)} (backup {os.path.basename(backup)})")


# Obtain rover planar centre from sticker_tracker.StickerTracker.
# Sticker detection and pose math are located in sticker_tracker.py.
def rover_xy(frame, H_img2m):
    try:
        from sticker_tracker import StickerTracker
        detection = StickerTracker(STICKERS).detect(frame, H_img2m)
        return None if detection is None else [float(v) for v in detection["center_m"]]
    except Exception:
        return None


# Back up old JSON, atomically replace calibration, and (for default
# output only) update sticker camera / camera guard and station warnings.
def save(data, frame, out, keep_sticker_camera):
    is_default = os.path.abspath(out) == os.path.abspath(DEFAULT_OUT)
    previous_calibration = _load(out) if os.path.exists(out) else None
    camera_shift = camera_shift_px(frame) if is_default else None
    if previous_calibration is not None:
        backup = out.replace(".json", f".before_regrid_{time.strftime('%Y%m%d_%H%M%S')}.json")
        shutil.copy(out, backup)
        print(f"backed up the previous calibration -> {os.path.basename(backup)}")
    temporary_path = out + ".tmp"
    with open(temporary_path, "w") as json_file:
        json.dump(data, json_file, indent=2)
        json_file.write("\n")
    os.replace(temporary_path, out)
    print(f"wrote {out}")
    if not is_default:
        return
    update_sticker_camera(data, previous_calibration, camera_shift, keep_sticker_camera)
    try:
        import camera_guard
        camera_guard.save_reference(frame, rover_xy(frame, np.array(data["H_img2m"])))
        print("saved a new camera-guard reference (loop/stations/camera_reference.*)")
    except Exception as error:
        print(f"WARN: camera-guard reference not saved ({error}); run "
              "loop/camera_guard.py --save-reference", file=sys.stderr)
    if previous_calibration is not None and gt.grid_dims(previous_calibration) != gt.grid_dims(data):
        print("NOTE: the grid frame changed - stations taught in loop/stations/ use the old "
              "frame; re-teach them (loop/stations.py teach ...) before closed-loop runs.")


# ---------------------------------------------------------------- verify
# ---------------------------------------------------------------- verify
# Compare saved homography with current detected grid crossings.
# Pointwise saved residuals: || project(H_saved, pixel_i) - metre_i ||.
# Fresh homography is refitted to current crossings. Area disagreement
# compares the saved and fresh mappings over the grid, not just at clicks.
# Exact grid sampling and statistics live in grid_target.py.
def verify(frame, calib_path, save_png=None, max_drift_mm=10.0, show=False, spacing=None,
           border=gt.DEFAULT_BORDER, tape_width=gt.TAPE_WIDTH_M):
    saved_calibration = _load(calib_path)
    if saved_calibration is None:
        print(f"ERROR: cannot read {calib_path}", file=sys.stderr)
        return 2
    grid_dimensions_m = gt.grid_dims(saved_calibration)
    spacing = gt.parse_spacing(spacing, grid_dimensions_m) if spacing else gt.grid_spacing(saved_calibration)
    if saved_calibration.get("image_size") != [frame.shape[1], frame.shape[0]]:
        print("ERROR: camera resolution differs from the calibration", file=sys.stderr)
        return 2
    saved_image_to_m = np.array(saved_calibration["H_img2m"], float)
    tape_detection, intersections = gt.detect(frame, saved_image_to_m, grid_dimensions_m, spacing, rounds=1, border=border, tape_width=tape_width)
    if saved_calibration.get("border_tape", "centre") != border:
        print(f"NOTE: the saved calibration used border tape '{saved_calibration.get('border_tape', 'centre')}'"
              f" but the grid is checked as '{border}' (tape {tape_detection.tape_width_used_m*1000:.1f} mm) -"
              " re-run calibrate_grid.py if this fails")
    detected_intersections = [intersection for intersection in intersections if intersection["px"] is not None]
    print(f"grid {grid_dimensions_m[0]:g} x {grid_dimensions_m[1]:g} m, spacing {spacing[0]:g} x {spacing[1]:g} m: "
          f"{len(detected_intersections)}/{len(intersections)} tape crossings found")
    if len(detected_intersections) < 4:
        print("FAIL: too few crossings found - camera moved a lot, grid occluded, or wrong "
              "--size/--spacing. Re-run calibrate_grid.py.")
        return 1
    pixels, metres = np.array([intersection["px"] for intersection in detected_intersections]), np.array([intersection["m"] for intersection in detected_intersections])
    saved_residuals_m = np.linalg.norm(gt.transform(saved_image_to_m, pixels) - metres, axis=1)
    fresh_image_to_m, inlier_mask, fresh_residuals_m = gt.solve(pixels, metres)
    maximum_drift_mm = gt.area_disagreement(saved_image_to_m, np.linalg.inv(fresh_image_to_m), grid_dimensions_m) * 1000
    saved_statistics, fresh_statistics = gt.residual_stats(saved_residuals_m), gt.residual_stats(fresh_residuals_m, inlier_mask)
    print(f"  saved calibration vs crossings : rms {saved_statistics['rms_m']*1000:5.1f} mm  "
          f"max {saved_statistics['max_m']*1000:5.1f} mm")
    print(f"  best fit to these crossings    : rms {fresh_statistics['rms_m']*1000:5.1f} mm  "
          f"max {fresh_statistics['max_m']*1000:5.1f} mm   (tape layout / detection floor)")
    print(f"  saved vs best fit, worst point over the grid area: {maximum_drift_mm:5.1f} mm")
    passed = maximum_drift_mm <= max_drift_mm
    print(("PASS" if passed else "FAIL") + f": calibration error {maximum_drift_mm:.1f} mm "
          f"({'<=' if passed else '>'} {max_drift_mm:g} mm)"
          + ("" if passed else " - re-run calibrate_grid.py"))
    if save_png or show:
        verification_frame = frame.copy()
        gt.draw_detections(verification_frame, intersections, saved_image_to_m, grid_dimensions_m, spacing, saved_residuals_m)
        _text_block(verification_frame, [(f"VERIFY {'PASS' if passed else 'FAIL'}: saved calibration off by up to "
                           f"{maximum_drift_mm:.1f} mm over the grid", (0, 255, 0) if passed else (0, 0, 255)),
                          (f"{len(detected_intersections)}/{len(intersections)} crossings; saved rms {saved_statistics['rms_m']*1000:.1f} mm, "
                           f"best possible {fresh_statistics['rms_m']*1000:.1f} mm", (255, 255, 255)),
                          ("magenta = residual of the SAVED calibration x10", (220, 220, 220))])
        if save_png:
            cv2.imwrite(save_png, verification_frame)
            print(f"wrote {save_png}")
        if show:
            cv2.imshow("grid verify (any key)", verification_frame)
            cv2.waitKey(0)
            cv2.destroyAllWindows()
    return 0 if passed else 1


# ---------------------------------------------------------------- main
# ---------------------------------------------------------------- main
# CLI orchestration: choose camera frame, parse geometry, click anchors,
# run calibration, review it, save, or run saved-calibration verification.
# Returns a process exit code, not calibration data.
def main():
    argument_parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    argument_parser.add_argument("--source", default=None, help="'stream' to grab the live overhead feed")
    argument_parser.add_argument("--image", default=None, help="calibrate from a saved frame")
    argument_parser.add_argument("--size", default=None,
                    help="grid W x H in metres, e.g. 1, 2x2, 1x4, 4x1 (default: the saved grid)")
    argument_parser.add_argument("--spacing", default=None,
                    help="tape line spacing in metres, e.g. 0.25 or 0.5,0.25 (default: saved "
                         "spacing if the size matches, else outer edges only)")
    argument_parser.add_argument("--anchors", default=None,
                    help="metric points to click, 'x,y x,y ...' (>=4; default: the 4 outer corners)")
    argument_parser.add_argument("--points-px", "--corners", dest="points_px", default=None,
                    help="non-interactive: pixel clicks for the anchors, 'x,y x,y ...'")
    argument_parser.add_argument("--no-refine", action="store_true",
                    help="use only the clicks (no tape detection) - for non-tape targets")
    argument_parser.add_argument("--yes", action="store_true", help="save without the review window")
    argument_parser.add_argument("--tape-width-mm", type=float, default=gt.TAPE_WIDTH_M * 1000,
                    help="tape width for the border inset (default 25.4 = 1 inch, the grid's tape); "
                         "0 = measure it from the image")
    argument_parser.add_argument("--border", choices=gt.BORDERS, default=gt.DEFAULT_BORDER,
                    help="inside (default): border tape lies inside the grid with its OUTER edge "
                         "on 0/W m (clicked outer corners = exact metres); inner lines are "
                         "measured to the tape centre. centre: every line to its centre.")
    argument_parser.add_argument("--out", default=DEFAULT_OUT)
    argument_parser.add_argument("--keep-sticker-camera", action="store_true",
                    help="do not touch sticker_config.json camera_position_m")
    argument_parser.add_argument("--verify", action="store_true",
                    help="check the saved --out calibration against the live/--image grid")
    argument_parser.add_argument("--max-drift-mm", type=float, default=10.0, help="--verify pass threshold")
    argument_parser.add_argument("--save", default=None, help="write the review/verify image here")
    arguments = argument_parser.parse_args()

    if arguments.image:
        frame = cv2.imread(arguments.image)
        if frame is None:
            print(f"ERROR: cannot read {arguments.image}", file=sys.stderr)
            return 2
    elif arguments.source == "stream":
        frame = grab_frame()
        if frame is None:
            return 3
    else:
        print("ERROR: give --image PATH or --source stream", file=sys.stderr)
        return 2

    if arguments.verify:
        try:
            return verify(frame, arguments.out, arguments.save, arguments.max_drift_mm,
                          show=bool(os.environ.get("DISPLAY")) and not arguments.save,
                          spacing=arguments.spacing, border=arguments.border,
                          tape_width=arguments.tape_width_mm / 1000 or None)
        except ValueError as error:
            print(f"ERROR: {error}", file=sys.stderr)
            return 2

    previous_calibration = _load(arguments.out) or {}
    try:
        grid_dimensions_m = gt.parse_size(arguments.size) if arguments.size else (gt.grid_dims(previous_calibration) if previous_calibration else (1.0, 1.0))
        if arguments.spacing is not None:
            grid_spacing_m = gt.parse_spacing(arguments.spacing, grid_dimensions_m)
        elif previous_calibration.get("grid_spacing_m") and gt.grid_dims(previous_calibration) == grid_dimensions_m:
            grid_spacing_m = gt.parse_spacing(",".join(map(str, previous_calibration["grid_spacing_m"])), grid_dimensions_m)
        else:
            grid_spacing_m = grid_dimensions_m
        if arguments.anchors:
            anchor_coordinates_m = parse_pairs(arguments.anchors)
            anchor_labels = [f"({x:g},{y:g})" for x, y in anchor_coordinates_m]
        else:
            anchor_coordinates_m, anchor_labels = gt.outer_corners(grid_dimensions_m)
    except ValueError as error:
        print(f"ERROR: {error}", file=sys.stderr)
        return 2
    print(f"grid {grid_dimensions_m[0]:g} x {grid_dimensions_m[1]:g} m (X x Y), tape every {grid_spacing_m[0]:g} x {grid_spacing_m[1]:g} m, "
          f"{len(gt.intersections(grid_dimensions_m, grid_spacing_m))} crossings")

    interactive = not arguments.points_px
    if interactive and not os.environ.get("DISPLAY"):
        print("ERROR: no DISPLAY for clicking; pass --points-px", file=sys.stderr)
        return 2
    clicked_pixels = parse_pairs(arguments.points_px) if arguments.points_px else None
    while True:
        if clicked_pixels is None:
            clicked_pixels = click_anchors(frame, grid_dimensions_m, anchor_coordinates_m, anchor_labels)
            if clicked_pixels is None:
                print("cancelled")
                return 1
        try:
            calibration_result = calibrate(frame, grid_dimensions_m, grid_spacing_m, anchor_coordinates_m, clicked_pixels, refine=not arguments.no_refine,
                            border=arguments.border, tape_width=arguments.tape_width_mm / 1000 or None)
            calibration_data = build_json(calibration_result, frame, grid_dimensions_m, grid_spacing_m, anchor_coordinates_m, clicked_pixels, border=arguments.border)
        except ValueError as error:
            print(f"ERROR: {error}", file=sys.stderr)
            if not interactive:
                return 2
            clicked_pixels = None
            continue
        for summary_text, _ in summary_lines(calibration_data, calibration_result):
            print("  " + summary_text)
        review_frame = review_image(frame, calibration_result, calibration_data, grid_dimensions_m, grid_spacing_m)
        if arguments.save:
            cv2.imwrite(arguments.save, review_frame)
            print(f"wrote {arguments.save}")
        if arguments.yes or not interactive:
            break
        review_choice = review(review_frame)
        if review_choice == "save":
            break
        if review_choice == "quit":
            print("cancelled")
            return 1
        clicked_pixels = None
    if calibration_data["rms_residual_m"] > 0.015:
        print("WARN: residual rms > 15 mm - check --size/--spacing and the anchor order",
              file=sys.stderr)
    save(calibration_data, frame, arguments.out, arguments.keep_sticker_camera)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
