#!/usr/bin/env python3
"""Generate a ChArUco calibration board PNG (+ spec) to print for intrinsics.

ChArUco = checkerboard fused with ArUco markers. It gives sub-pixel corner
accuracy *and* works even when partly out of frame, so it's the robust choice
for calibrating the C920e's lens distortion.

Workflow:
    1. python3 make_charuco.py            # writes charuco_board.png + charuco_spec.json
    2. Print charuco_board.png at 100% / "actual size" (NO "fit to page").
    3. Tape it FLAT to a rigid board (foam board / clipboard). Wrinkles ruin it.
    4. Measure one white square edge with a ruler. If it isn't the nominal size,
       put the real measurement into charuco_spec.json ("square_len_m"). The
       calibrator reads that file, so accurate scale matters.

Defaults fit on US Letter / A4 with margin.
"""
import argparse
import json
import os

import cv2
import numpy as np

DICT_NAME = "DICT_4X4_50"


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--squares-x", type=int, default=5, help="squares across")
    ap.add_argument("--squares-y", type=int, default=7, help="squares down")
    ap.add_argument("--square-mm", type=float, default=35.0,
                    help="nominal checker square size in mm")
    ap.add_argument("--marker-ratio", type=float, default=0.75,
                    help="marker size as fraction of square (default 0.75)")
    ap.add_argument("--dpi", type=int, default=300, help="print DPI")
    ap.add_argument("--out-dir", default=None, help="output dir (default: alongside this script)")
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)

    square_m = args.square_mm / 1000.0
    marker_m = square_m * args.marker_ratio

    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, DICT_NAME))
    board = cv2.aruco.CharucoBoard_create(
        args.squares_x, args.squares_y, square_m, marker_m, dictionary)

    px_per_m = args.dpi / 0.0254
    w_px = int(round(args.squares_x * square_m * px_per_m))
    h_px = int(round(args.squares_y * square_m * px_per_m))
    margin_px = int(round(0.01 * px_per_m))  # ~1 cm quiet border
    img = board.draw((w_px, h_px), marginSize=margin_px, borderBits=1)

    png = os.path.join(out_dir, "charuco_board.png")
    cv2.imwrite(png, img)

    spec = {
        "type": "charuco",
        "dictionary": DICT_NAME,
        "squares_x": args.squares_x,
        "squares_y": args.squares_y,
        "square_len_m": round(square_m, 6),
        "marker_len_m": round(marker_m, 6),
        "note": "MEASURE a printed square and correct square_len_m if it differs.",
    }
    spec_path = os.path.join(out_dir, "charuco_spec.json")
    with open(spec_path, "w") as f:
        json.dump(spec, f, indent=2)

    print(f"wrote {png}  ({w_px}x{h_px}px @ {args.dpi}dpi)")
    print(f"wrote {spec_path}")
    print(f"nominal: {args.squares_x}x{args.squares_y} squares, "
          f"{args.square_mm:.1f}mm each, marker {marker_m*1000:.1f}mm")
    print(f"printed board is about "
          f"{args.squares_x*args.square_mm/10:.1f} x "
          f"{args.squares_y*args.square_mm/10:.1f} cm")
    print("PRINT AT 100% (actual size), then measure a square and fix the spec.")


if __name__ == "__main__":
    main()
