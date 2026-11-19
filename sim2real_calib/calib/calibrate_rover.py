#!/usr/bin/env python3
"""Check the sticker tracker, or open its setup tool.

  python3 calibrate_rover.py --mode check
  python3 calibrate_rover.py --mode setup [calibrate_stickers.py options]

The old rear-tag circle-fit/offset calibration is no longer used. Body offsets
are measured from the middle-wheel-hub midpoint and saved per sticker.
"""
import argparse
import math
from pathlib import Path
import sys
import time

from sticker_tracker import DEFAULT_CONFIG, StickerTracker

HERE = Path(__file__).resolve().parent


# --mode setup: hand over to calibrate_stickers.py (same command-line options).
# --mode check: print one line per NEW camera frame - the tracked chassis pose
# ('chassis center: x=.. y=.. yaw=..') or 'No pose: <why>'. surface_demo.py counts these
# lines to measure the tracking rate. Exit code 0 if any pose was found, 4 if none.
def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--mode', choices=('check', 'setup'), default='check')
    parser.add_argument('--stickers', default=str(DEFAULT_CONFIG))
    parser.add_argument('--duration', type=float, default=0., help='check duration; 0 until Ctrl-C')
    args, setup_args = parser.parse_known_args()
    if args.mode == 'setup':
        import calibrate_stickers
        sys.argv = [str(HERE / 'calibrate_stickers.py'), '--out', args.stickers, *setup_args]
        return calibrate_stickers.main()
    if setup_args:
        parser.error('Unrecognized check options: ' + ' '.join(setup_args))
    sys.path.insert(0, str(HERE.parent / 'capture'))
    sys.path.insert(0, str(HERE.parent / 'viz'))
    from stream import LatestFrame
    from metric_overlay import load_calib
    try:
        tracker = StickerTracker(args.stickers)
    except ValueError as exc:
        print('ERROR:', exc, file=sys.stderr)
        return 2
    image_to_grid_homography, _, _ = load_calib(HERE.parent / 'calib' / 'grid_calib.json')
    camera_stream = LatestFrame()
    deadline = time.monotonic() + args.duration if args.duration else math.inf
    last_frame_number = -1
    poses_found = 0
    try:
        while time.monotonic() < deadline:
            frame, _, frame_number = camera_stream.read()
            if frame is None or frame_number == last_frame_number:
                time.sleep(.01)
                continue
            last_frame_number = frame_number
            result = tracker.detect(frame, image_to_grid_homography)
            if result is None:
                print('No pose:', tracker.status, flush=True)
            else:
                poses_found += 1
                chassis_x_m, chassis_y_m = result['center_m']
                label = 'chassis center' if tracker.measurement_ready else 'PREVIEW ' + tracker.reference_point
                print(f'{label}: x={chassis_x_m:.4f} y={chassis_y_m:.4f} yaw={result["heading_deg"]:.2f} deg; '
                      f'{tracker.status}', flush=True)
    except KeyboardInterrupt:
        pass
    finally:
        camera_stream.release()
    return 0 if poses_found else 4


if __name__ == '__main__':
    raise SystemExit(main())
