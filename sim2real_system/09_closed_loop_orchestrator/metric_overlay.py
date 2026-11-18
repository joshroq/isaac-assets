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


def load_calib(path):
    """-> (H_img2m, H_m2img, (grid_width_m, grid_height_m))."""
    with open(path) as f:
        d = json.load(f)
    return (np.array(d["H_img2m"], dtype=np.float64),
            np.array(d["H_m2img"], dtype=np.float64),
            grid_dims(d))


def m2img(H_m2img, pts_m):
    pts = np.array(pts_m, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_m2img).reshape(-1, 2)


def img2m(H_img2m, pts_px):
    pts = np.array(pts_px, dtype=np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(pts, H_img2m).reshape(-1, 2)


def draw_metric_grid(img, H_m2img, grid_m, step=0.1, major_m=0.5):
    """0.1 m lines over the W x H grid; every 0.5 m and the outer edges drawn bold."""
    w, h = as_dims(grid_m)
    for length, other, vertical in ((w, h, True), (h, w, False)):
        n = int(round(length / step))
        for i in range(n + 1):
            g = min(i * step, length)
            major = i == n or abs(g / major_m - round(g / major_m)) < 1e-6
            col = (0, 220, 255) if major else (60, 160, 90)
            th = 2 if major else 1
            # vertical metric line x=g, y:0..h ; horizontal line y=g, x:0..w
            ends = [(g, 0), (g, other)] if vertical else [(0, g), (other, g)]
            a, b = m2img(H_m2img, ends)
            cv2.line(img, tuple(np.int32(a)), tuple(np.int32(b)), col, th, cv2.LINE_AA)


def draw_axes_and_labels(img, H_m2img, grid_m):
    o, xend, yend = m2img(H_m2img, [(0, 0), (0.3, 0), (0, 0.3)])
    cv2.arrowedLine(img, tuple(np.int32(o)), tuple(np.int32(xend)), (0, 0, 255), 3,
                    tipLength=0.2)
    cv2.arrowedLine(img, tuple(np.int32(o)), tuple(np.int32(yend)), (0, 255, 0), 3,
                    tipLength=0.2)
    cv2.putText(img, "X", tuple(np.int32(xend) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(img, "Y", tuple(np.int32(yend) + 6), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                (0, 255, 0), 2, cv2.LINE_AA)
    w, h = as_dims(grid_m)
    for (mx, my), lab in [((0, 0), "(0,0)"), ((w, 0), f"({w:g},0)"),
                          ((w, h), f"({w:g},{h:g})"), ((0, h), f"(0,{h:g})")]:
        p = m2img(H_m2img, [(mx, my)])[0]
        _txt(img, lab, tuple(np.int32(p)))


def _txt(img, s, org, color=(255, 255, 255), scale=0.55):
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, (0, 0, 0), 3, cv2.LINE_AA)
    cv2.putText(img, s, org, cv2.FONT_HERSHEY_SIMPLEX, scale, color, 1, cv2.LINE_AA)


def detect_rover(frame, H_img2m, tracker):
    """Return the pose of the configured sticker triangle."""
    return tracker.detect(frame, H_img2m)


def render(frame, calib, tracker, measure, hover):
    H_img2m, H_m2img, grid_m = calib
    img = frame.copy()
    draw_metric_grid(img, H_m2img, grid_m)
    draw_axes_and_labels(img, H_m2img, grid_m)

    if tracker is not None:
        r = detect_rover(frame, H_img2m, tracker)
        draw_tracking(img, r, tracker)

    # click-to-measure
    for p in measure:
        cv2.circle(img, tuple(np.int32(p)), 5, (255, 0, 255), -1)
    if len(measure) == 2:
        a_m, b_m = img2m(H_img2m, measure)
        d = float(np.linalg.norm(b_m - a_m))
        cv2.line(img, tuple(np.int32(measure[0])), tuple(np.int32(measure[1])),
                 (255, 0, 255), 2, cv2.LINE_AA)
        mid = ((measure[0][0] + measure[1][0]) / 2, (measure[0][1] + measure[1][1]) / 2)
        _txt(img, f"{d*100:.1f} cm", (int(mid[0]), int(mid[1])), (255, 0, 255), 0.7)

    if hover is not None:
        hm = img2m(H_img2m, [hover])[0]
        _txt(img, f"({hm[0]:.3f}, {hm[1]:.3f}) m", (int(hover[0]) + 12, int(hover[1])),
             (0, 255, 255), 0.55)
    return img


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--calib", default=CALIB_DEFAULT)
    ap.add_argument("--image", default=None, help="static: annotate this frame")
    ap.add_argument("--save", default=None, help="static: write annotated image here")
    ap.add_argument("--stickers", default=str(DEFAULT_CONFIG), help="sticker tracker configuration JSON")
    ap.add_argument("--no-tracking", "--no-aruco", dest="no_tracking", action="store_true",
                    help="skip rover tracking")
    args = ap.parse_args()

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
        out = render(frame, calib, tracker, [], None)
        dest = args.save or "/tmp/metric_overlay.png"
        cv2.imwrite(dest, out)
        print(f"wrote {dest}")
        return 0

    # ---- live mode ----
    from stream import LatestFrame, STREAM_URL
    src = LatestFrame(STREAM_URL)
    if not src.opened() or src.wait_first(20) is None:
        print("ERROR: no stream. Start the Pi camera service and stop receive.py.",
              file=sys.stderr); return 3

    state = {"measure": [], "hover": None}
    win = "metric overlay  (q quit, c clear measure)"

    def on_mouse(event, x, y, flags, _):
        if event == cv2.EVENT_MOUSEMOVE:
            state["hover"] = (float(x), float(y))
        elif event == cv2.EVENT_LBUTTONDOWN:
            if len(state["measure"]) >= 2:
                state["measure"] = []
            state["measure"].append((float(x), float(y)))

    cv2.namedWindow(win)
    cv2.setMouseCallback(win, on_mouse)
    last_t, n0, fps = time.time(), 0, 0.0
    while True:
        frame, _, n = src.read()
        if frame is None:
            continue
        out = render(frame, calib, tracker, state["measure"], state["hover"])
        now = time.time()
        if now - last_t >= 0.5:
            fps = (n - n0) / (now - last_t); last_t, n0 = now, n
        _txt(out, f"{fps:4.1f} fps  |  hover=read m, click x2=measure", (10, 24),
             (255, 255, 255), 0.6)
        cv2.imshow(win, out)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            break
        if k == ord("c"):
            state["measure"] = []
    src.release()
    cv2.destroyAllWindows()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
