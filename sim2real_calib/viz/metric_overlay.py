#!/usr/bin/env python3
"""Live metric visualization of the overhead feed on the desktop.

Loads the grid calibration (grid_calib.json) and overlays real-world distances on
the camera image:
  - a metric grid every 0.1 m, warped onto the real tape grid
  - origin axes + corner labels in metres
  - hover anywhere to read that point's (x, y) in metres
  - click two points to measure the real distance between them
  - if the rover's green/orange-red/pink sticker triangle is visible: position (m),
    heading, and distance from the grid origin

Live (stop receive.py first - one consumer of UDP 5000):
    python3 metric_overlay.py                 # keys: q quit, c clear measurement

Static test (render onto a saved frame and save the annotated image):
    python3 metric_overlay.py --image /tmp/pi_grid_frame.png --save /tmp/overlay.png
"""
import argparse
import json
import math
import os
import sys
import time

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "capture"))
sys.path.insert(0, os.path.join(HERE, "..", "calib"))
from sticker_tracker import StickerTracker, DEFAULT_CONFIG, draw_tracking
from grid_target import as_dims, grid_dims
CALIB_DEFAULT = os.path.join(HERE, "..", "calib", "grid_calib.json")


# grid_calib.json -> (image->metre homography, metre->image homography, grid size).
def load_calib(path):
    """-> (H_img2m, H_m2img, (grid_width_m, grid_height_m))."""
    with open(path) as calib_file:
        calib = json.load(calib_file)
    return (np.array(calib["H_img2m"], dtype=np.float64),
            np.array(calib["H_m2img"], dtype=np.float64),
            grid_dims(calib))


# Floor metres -> image pixels (N x 2).
def m2img(H_m2img, pts_m):
    points = np.array(pts_m, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, H_m2img).reshape(-1, 2)


# Image pixels -> floor metres (N x 2).
def img2m(H_img2m, pts_px):
    points = np.array(pts_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points, H_img2m).reshape(-1, 2)


# Draw lines every `step` metres along both axes, bold every `major_m` and on the edges.
def draw_metric_grid(img, H_m2img, grid_m, step=0.1, major_m=0.5):
    """0.1 m lines over the W x H grid; every 0.5 m and the outer edges drawn bold."""
    grid_w_m, grid_h_m = as_dims(grid_m)
    for length, other, vertical in ((grid_w_m, grid_h_m, True), (grid_h_m, grid_w_m, False)):
        line_count = int(round(length / step))
        for line_index in range(line_count + 1):
            position_m = min(line_index * step, length)
            major = line_index == line_count or abs(position_m / major_m - round(position_m / major_m)) < 1e-6
            colour = (0, 220, 255) if major else (60, 160, 90)
            thickness = 2 if major else 1
            # vertical metric line x=g, y:0..h ; horizontal line y=g, x:0..w
            line_ends_m = [(position_m, 0), (position_m, other)] if vertical else [(0, position_m), (other, position_m)]
            start_px, end_px = m2img(H_m2img, line_ends_m)
            cv2.line(img, tuple(np.int32(start_px)), tuple(np.int32(end_px)), colour, thickness, cv2.LINE_AA)


# X (red) and Y (green) axis arrows from the origin and the corner coordinates.
def draw_axes_and_labels(img, H_m2img, grid_m):
    origin_px, x_axis_end_px, y_axis_end_px = m2img(H_m2img, [(0, 0), (0.3, 0), (0, 0.3)])
    cv2.arrowedLine(img, tuple(np.int32(origin_px)), tuple(np.int32(x_axis_end_px)), (0, 0, 255), 3,
                    tipLength=0.2)
    cv2.arrowedLine(img, tuple(np.int32(origin_px)), tuple(np.int32(y_axis_end_px)), (0, 255, 0), 3,
                    tipLength=0.2)
    cv2.putText(img, "X", tuple(np.int32(x_axis_end_px) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Y", tuple(np.int32(y_axis_end_px) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2, cv2.LINE_AA)
    grid_w_m, grid_h_m = as_dims(grid_m)
    for (corner_x_m, corner_y_m), label in [((0, 0), "(0,0)"), ((grid_w_m, 0), f"({grid_w_m:g},0)"),
                          ((grid_w_m, grid_h_m), f"({grid_w_m:g},{grid_h_m:g})"), ((0, grid_h_m), f"(0,{grid_h_m:g})")]:
        label_px = m2img(H_m2img, [(corner_x_m, corner_y_m)])[0]
        _txt(img, label, tuple(np.int32(label_px)))


# Outlined text (readable on any background).
def _txt(img, s, org, color=(255, 255, 255), scale=0.55):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


# Rover pose from the sticker tracker (kept as a function for older callers).
def detect_rover(frame, H_img2m, tracker):
    """Return the pose of the configured sticker triangle."""
    return tracker.detect(frame, H_img2m)


# One annotated frame: metric grid, axes, tracked rover, the two-click distance
# measurement and the hovered point's metric coordinates.
def render(frame, calib, tracker, measure, hover):
    H_img2m, H_m2img, grid_m = calib
    img = frame.copy()
    draw_metric_grid(img, H_m2img, grid_m)
    draw_axes_and_labels(img, H_m2img, grid_m)

    if tracker is not None:
        result = detect_rover(frame, H_img2m, tracker)
        draw_tracking(img, result, tracker)

    # click-to-measure
    for point_px in measure:
        cv2.circle(img, tuple(np.int32(point_px)), 5, (255, 0, 255), -1)
    if len(measure) == 2:
        first_m, second_m = img2m(H_img2m, measure)
        distance_m = float(np.linalg.norm(second_m - first_m))
        cv2.line(img, tuple(np.int32(measure[0])), tuple(np.int32(measure[1])),
                 (255, 0, 255), 2, cv2.LINE_AA)
        midpoint_px = ((measure[0][0] + measure[1][0]) / 2, (measure[0][1] + measure[1][1]) / 2)
        _txt(img, f"{distance_m*100:.1f} cm", (int(midpoint_px[0]), int(midpoint_px[1])), (255, 0, 255), 0.7)

    if hover is not None:
        hover_m = img2m(H_img2m, [hover])[0]
        _txt(img, f"({hover_m[0]:.3f}, {hover_m[1]:.3f}) m", (int(hover[0]) + 12, int(hover[1])),
             (0, 255, 255), 0.55)
    return img


# --image: annotate one frame and save it; otherwise live window on the stream
# (hover = read metres, two clicks = measure, c = clear, q = quit).
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--calib", default=CALIB_DEFAULT)
    parser.add_argument("--image", default=None, help="static: annotate this frame")
    parser.add_argument("--save", default=None, help="static: write annotated image here")
    parser.add_argument("--stickers", default=str(DEFAULT_CONFIG), help="sticker tracker configuration JSON")
    parser.add_argument("--no-tracking", "--no-aruco", dest="no_tracking", action="store_true",
                    help="skip rover tracking")
    args = parser.parse_args()

    if not os.path.exists(args.calib):
        print(f"ERROR: {args.calib} missing. Run calibrate_grid.py first.",
              file=sys.stderr); return 2
    calib = load_calib(args.calib)

    try:
        tracker = None if args.no_tracking else StickerTracker(args.stickers)
    except ValueError as exc:
        print(f"ERROR: {exc}", file=sys.stderr)
        return 2

    # ---- static mode: render once and save (for validation / reports) ----
    if args.image:
        frame = cv2.imread(args.image)
        if frame is None:
            print(f"ERROR: cannot read {args.image}", file=sys.stderr); return 2
        annotated = render(frame, calib, tracker, [], None)
        out_path = args.save or "/tmp/metric_overlay.png"
        cv2.imwrite(out_path, annotated)
        print(f"wrote {out_path}")
        return 0

    # ---- live mode ----
    from stream import LatestFrame, STREAM_URL
    camera_stream = LatestFrame(STREAM_URL)
    if not camera_stream.opened() or camera_stream.wait_first(20) is None:
        print("ERROR: no stream. Start the Pi camera service and stop receive.py.",
              file=sys.stderr); return 3

    state = {"measure": [], "hover": None}
    window_name = "metric overlay  (q quit, c clear measure)"

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_MOUSEMOVE:
            state["hover"] = (float(x), float(y))
        elif event == cv2.EVENT_LBUTTONDOWN:
            if len(state["measure"]) >= 2:
                state["measure"] = []
            state["measure"].append((float(x), float(y)))

    cv2.namedWindow(window_name)
    cv2.setMouseCallback(window_name, on_mouse)
    last_fps_time, last_frame_number, fps = time.time(), 0, 0.0
    while True:
        frame, _, frame_number = camera_stream.read()
        if frame is None:
            continue
        annotated = render(frame, calib, tracker, state["measure"], state["hover"])
        now = time.time()
        if now - last_fps_time >= 0.5:
            fps = (frame_number - last_frame_number) / (now - last_fps_time); last_fps_time, last_frame_number = now, frame_number
        _txt(annotated, f"{fps:4.1f} fps  |  hover=read m, click x2=measure", (10, 24),
             (255, 255, 255), 0.6)
        cv2.imshow(window_name, annotated)
        pressed_key = cv2.waitKey(1) & 0xFF
        if pressed_key in (ord("q"), 27):
            break
        if pressed_key == ord("c"):
            state["measure"] = []
    camera_stream.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
