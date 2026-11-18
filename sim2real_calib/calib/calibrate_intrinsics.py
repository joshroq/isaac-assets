#!/usr/bin/env python3
"""Solve the C920e's intrinsics (camera matrix + distortion) from a ChArUco board.

Two ways to feed it views of the printed board:

  Live (stop receive.py first - one consumer of UDP 5000 at a time):
      python3 calibrate_intrinsics.py --source stream
      # move the board around: tilts, corners, near/far. SPACE grabs a view when
      # it's well detected (green). Aim for 15-25 varied views, then press C.

  From saved images (no port conflict; e.g. PNGs you saved with receive.py's 's'):
      python3 calibrate_intrinsics.py --source /path/to/board_images/

Output: camera_intrinsics.yaml (camera_matrix, dist_coeffs, image_size, rms).
That file feeds undistortion for every later step (homography, rover tracking).
"""
import argparse
import glob
import json
import os
import sys

import cv2
import numpy as np

HERE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, os.path.join(HERE, "..", "capture"))
from stream import LatestFrame, STREAM_URL  # noqa: E402


def load_board(spec_path):
    with open(spec_path) as f:
        spec = json.load(f)
    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, spec["dictionary"]))
    board = cv2.aruco.CharucoBoard_create(
        spec["squares_x"], spec["squares_y"],
        spec["square_len_m"], spec["marker_len_m"], dictionary)
    return board, dictionary, spec


def detect(gray, board, dictionary, params):
    corners, ids, _ = cv2.aruco.detectMarkers(gray, dictionary, parameters=params)
    if ids is None or len(ids) == 0:
        return None, None, 0
    n, ch_corners, ch_ids = cv2.aruco.interpolateCornersCharuco(
        corners, ids, gray, board)
    if ch_corners is None:
        return corners, None, 0
    return corners, (ch_corners, ch_ids), int(n)


def solve(all_corners, all_ids, board, image_size):
    flags = cv2.CALIB_RATIONAL_MODEL  # C920e has enough distortion to warrant it
    rms, K, dist, _, _ = cv2.aruco.calibrateCameraCharuco(
        all_corners, all_ids, board, image_size, None, None, flags=flags)
    return rms, K, dist


def save_yaml(path, K, dist, image_size, rms, spec):
    fs = cv2.FileStorage(path, cv2.FILE_STORAGE_WRITE)
    fs.write("image_width", int(image_size[0]))
    fs.write("image_height", int(image_size[1]))
    fs.write("camera_matrix", K)
    fs.write("dist_coeffs", dist)
    fs.write("rms_reproj_error_px", float(rms))
    fs.write("board_square_len_m", float(spec["square_len_m"]))
    fs.release()


def run_stream(board, dictionary, params, min_corners):
    src = LatestFrame(STREAM_URL)
    if not src.opened() or src.wait_first(20) is None:
        print("ERROR: no stream. Stop receive.py and confirm the sender is on.",
              file=sys.stderr)
        return None, None
    print("SPACE=grab view  C=calibrate  Q=quit")
    all_corners, all_ids = [], []
    image_size = None
    win = "charuco calib (SPACE grab / C calibrate / Q quit)"
    while True:
        frame, _, _ = src.read()
        if frame is None:
            continue
        gray = cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        _, ch, n = detect(gray, board, dictionary, params)
        disp = frame.copy()
        good = ch is not None and n >= min_corners
        if ch is not None:
            cv2.aruco.drawDetectedCornersCharuco(disp, ch[0], ch[1])
        color = (0, 255, 0) if good else (0, 0, 255)
        cv2.putText(disp, f"corners:{n}  captured:{len(all_corners)}",
                    (10, 28), cv2.FONT_HERSHEY_SIMPLEX, 0.7, color, 2, cv2.LINE_AA)
        cv2.imshow(win, disp)
        k = cv2.waitKey(1) & 0xFF
        if k in (ord("q"), 27):
            src.release(); cv2.destroyAllWindows(); return None, None
        if k == ord(" ") and good:
            all_corners.append(ch[0]); all_ids.append(ch[1])
            print(f"  captured view {len(all_corners)} ({n} corners)")
        if k == ord("c"):
            break
    src.release(); cv2.destroyAllWindows()
    return (all_corners, all_ids), image_size


def run_images(img_dir, board, dictionary, params, min_corners):
    paths = sorted(sum([glob.glob(os.path.join(img_dir, e))
                        for e in ("*.png", "*.jpg", "*.jpeg")], []))
    if not paths:
        print(f"ERROR: no images in {img_dir}", file=sys.stderr)
        return None, None
    all_corners, all_ids = [], []
    image_size = None
    for p in paths:
        img = cv2.imread(p)
        if img is None:
            continue
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        image_size = gray.shape[::-1]
        _, ch, n = detect(gray, board, dictionary, params)
        if ch is not None and n >= min_corners:
            all_corners.append(ch[0]); all_ids.append(ch[1])
            print(f"  {os.path.basename(p)}: {n} corners  (used)")
        else:
            print(f"  {os.path.basename(p)}: {n} corners  (skipped)")
    return (all_corners, all_ids), image_size


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--source", default="stream",
                    help="'stream' for live, or a directory of board images")
    ap.add_argument("--spec", default=os.path.join(HERE, "charuco_spec.json"),
                    help="board spec from make_charuco.py")
    ap.add_argument("--out", default=os.path.join(HERE, "camera_intrinsics.yaml"))
    ap.add_argument("--min-corners", type=int, default=6,
                    help="min ChArUco corners to accept a view")
    args = ap.parse_args()

    if not os.path.exists(args.spec):
        print(f"ERROR: {args.spec} missing. Run make_charuco.py first.",
              file=sys.stderr)
        return 2
    board, dictionary, spec = load_board(args.spec)
    params = cv2.aruco.DetectorParameters_create()

    if args.source == "stream":
        data, image_size = run_stream(board, dictionary, params, args.min_corners)
    else:
        data, image_size = run_images(args.source, board, dictionary, params,
                                      args.min_corners)
    if data is None:
        return 1
    all_corners, all_ids = data
    if len(all_corners) < 8:
        print(f"ERROR: only {len(all_corners)} good views; need >=8 (15-25 ideal).",
              file=sys.stderr)
        return 1

    print(f"calibrating from {len(all_corners)} views ...")
    rms, K, dist = solve(all_corners, all_ids, board, image_size)
    save_yaml(args.out, K, dist, image_size, rms, spec)
    print(f"\nRMS reprojection error: {rms:.3f} px  (aim < 0.5; < 1.0 is usable)")
    print(f"camera matrix:\n{K}")
    print(f"dist coeffs: {dist.ravel()}")
    print(f"\nwrote {args.out}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
