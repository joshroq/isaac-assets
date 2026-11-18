#!/usr/bin/env python3
"""Generate an ArUco marker PNG to print and mount flat on top of the rover.

Tracking this marker from the overhead camera gives precise position AND heading
every frame - far more robust than segmenting the black rover on carpet.

Defaults: DICT_4X4_50 (fewest bits -> most robust at small sizes from the overhead
camera) at id 23 (above the ChArUco board's ids, so they never collide even if both
are in frame). White quiet-zone border is included - detection fails without it.

    python3 make_marker.py                 # id 23, 60 mm, DICT_4X4_50
    python3 make_marker.py --mm 50         # smaller if the deck is tight

Print at 100% (actual size), measure the black square edge, and put the real value
into the tracking config later (marker size sets the metric scale for pose).
"""
import argparse
import json
import os

import cv2
import numpy as np


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dict", default="DICT_4X4_50", help="ArUco dictionary")
    ap.add_argument("--id", type=int, default=23, help="marker id")
    ap.add_argument("--mm", type=float, default=60.0, help="black-square size in mm")
    ap.add_argument("--dpi", type=int, default=300)
    ap.add_argument("--out-dir", default=None)
    args = ap.parse_args()

    out_dir = args.out_dir or os.path.dirname(os.path.abspath(__file__))
    os.makedirs(out_dir, exist_ok=True)

    dictionary = cv2.aruco.Dictionary_get(getattr(cv2.aruco, args.dict))
    px_per_mm = args.dpi / 25.4
    side = int(round(args.mm * px_per_mm))
    marker = cv2.aruco.drawMarker(dictionary, args.id, side)

    # Add a white quiet zone (~1 marker-module wide, min 1 cm) - required for detect.
    quiet = max(int(round(10 * px_per_mm)), side // 7)
    canvas = np.full((side + 2 * quiet, side + 2 * quiet), 255, dtype=np.uint8)
    canvas[quiet:quiet + side, quiet:quiet + side] = marker

    png = os.path.join(out_dir, f"marker_{args.dict}_id{args.id}.png")
    cv2.imwrite(png, canvas)

    spec = {"dictionary": args.dict, "id": args.id,
            "marker_len_m": round(args.mm / 1000.0, 6)}
    with open(os.path.join(out_dir, "marker_spec.json"), "w") as f:
        json.dump(spec, f, indent=2)

    print(f"wrote {png}  (black square {args.mm:.0f} mm @ {args.dpi} dpi)")
    print("Print at 100% (actual size), mount FLAT and centered on the rover top,")
    print("aligned so the marker's 'up' matches the rover's forward direction.")


if __name__ == "__main__":
    main()
