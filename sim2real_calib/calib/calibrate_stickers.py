#!/usr/bin/env python3
"""Set up green/front-left, orange-red/front-right, pink/rear sticker tracking.

Click the three centers in that order in an overhead frame (U undo, Enter save).
Colors are sampled from the real camera. Without measured body offsets this
creates a labelled sticker-centroid preview; measured recording stays disabled.

Example measured setup (replace all measurements with your own):
  python3 calibrate_stickers.py --positions-m '0.07,0.05 0.07,-0.05 -0.07,0' \
      --height-m 0.12 --camera-position-m '0.58,-0.28,1.48'
Use --plane-calib instead for a homography calibrated at sticker height.
"""
from __future__ import annotations

import argparse
import json
import math
from pathlib import Path
import sys
import time

import cv2
import numpy as np

from sticker_tracker import DEFAULT_CONFIG, StickerTracker, transform_points

HERE = Path(__file__).resolve().parent
CALIB = HERE.parent / 'calib' / 'grid_calib.json'
NAMES = ['green', 'orange_red', 'pink']


def parse_pairs(text, count=3):
    points = np.asarray([[float(v) for v in pair.split(',')] for pair in text.split()], float)
    if points.shape != (count, 2) or not np.isfinite(points).all():
        raise ValueError(f'Expected {count} finite x,y pairs separated by spaces')
    return points


def sample_color(frames, point, radius=3):
    """HSV ranges for the sticker at point; pass several frames of a still rover so
    H.264 noise on a ~10 px dot does not set the thresholds."""
    frames = frames if isinstance(frames, (list, tuple)) else [frames]
    x, y = (int(round(v)) for v in point)
    h, w = frames[0].shape[:2]
    if not radius <= x < w - radius or not radius <= y < h - radius:
        raise ValueError('Click inside a sticker, away from the image boundary')
    values = np.concatenate([cv2.cvtColor(f[y-radius:y+radius+1, x-radius:x+radius+1],
                                          cv2.COLOR_BGR2HSV).reshape(-1, 3) for f in frames])
    values = values[(values[:, 1] >= 15) & (values[:, 2] >= 60)]
    if len(values) < 8:
        raise ValueError('Not enough colored pixels; click inside the colored sticker')
    # Circular mean is essential for orange-red straddling hue 179/0.
    angles = values[:, 0] * (2 * math.pi / 180)
    hue = int(round(math.atan2(np.sin(angles).mean(), np.cos(angles).mean()) * 180 / (2 * math.pi))) % 180
    delta = np.abs((values[:, 0].astype(float) - hue + 90) % 180 - 90)
    margin = max(8, min(18, int(math.ceil(np.percentile(delta, 90))) + 4))
    # Harsh light washes stickers out (orange-red reaches S~30 at V 255); a floor of 30
    # fragments such a dot below min_circularity, so go lower but demand brightness:
    # a washed-out sticker is only separable from warm-grey mat/concrete when it is bright,
    # and bright concrete sits at S~15-21, so keep a washed-out floor close to the sticker.
    s20 = np.percentile(values[:, 1], 20)
    saturation = max(15, int(s20 * (.7 if s20 < 50 else .6)))
    brightness = max(60, int(np.percentile(values[:, 2], 20) * (.8 if saturation < 30 else .55)))
    intervals = []
    lo, hi = hue - margin, hue + margin
    spans = [(max(0, lo), min(179, hi))]
    if lo < 0:
        spans.append((180 + lo, 179))
    if hi > 179:
        spans.append((0, hi - 180))
    for a, b in spans:
        intervals.append(dict(lower=[a, saturation, brightness], upper=[b, 255, 255]))
    return intervals


def click_points(frame):
    points = []
    title = 'Sticker setup: green, orange-red, pink (U undo, Enter finish, Q cancel)'
    cv2.namedWindow(title)
    def click(event, x, y, flags, _):
        if event == cv2.EVENT_LBUTTONDOWN and len(points) < 3:
            points.append((float(x), float(y)))
    cv2.setMouseCallback(title, click)
    try:
        while True:
            display = frame.copy()
            for point, name in zip(points, NAMES):
                x, y = (int(v) for v in point)
                cv2.circle(display, (x, y), 8, (255, 255, 255), 1)
                cv2.putText(display, name, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            label = 'Click ' + NAMES[len(points)] if len(points) < 3 else 'Enter to save'
            cv2.putText(display, label, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            cv2.imshow(title, display)
            key = cv2.waitKey(20) & 0xff
            if key in (ord('q'), 27):
                return None
            if key == ord('u') and points:
                points.pop()
            if key in (10, 13) and len(points) == 3:
                return np.asarray(points, float)
    finally:
        cv2.destroyWindow(title)


def build_config(frame, points, grid, positions=None, height=0., camera=None, plane=None,
                 sample_frames=None):
    config = dict(schema_version=1, image_size=[frame.shape[1], frame.shape[0]],
                  sticker_diameter_m=.01905, geometry_measured=positions is not None,
                  projection_verified=plane is not None or (height > 0 and camera is not None),
                  reference_point='chassis_center' if positions is not None else 'sticker_centroid',
                  sticker_height_m=height, camera_position_m=camera,
                  plane_H_img2m=plane, min_area_px=8, max_area_px=2500,
                  min_circularity=.45, max_candidates_per_color=16,
                  max_edge_error_m=.015, max_fit_error_m=.006)
    if not math.isfinite(height) or height < 0:
        raise ValueError('Sticker height must be finite and nonnegative')
    # Use a temporary geometry only to obtain the corrected plane mapping.
    config['stickers'] = [dict(name=n, body_xy_m=p, hsv_ranges=sample_color(sample_frames or frame, pt))
                          for n, p, pt in zip(NAMES, [[.07,.05],[.07,-.05],[-.07,0]], points)]
    provisional = StickerTracker(config)
    H = provisional.plane_homography(grid['H_img2m'])
    metric = transform_points(H, points)
    if positions is None:
        front = (metric[0] + metric[1]) / 2 - metric[2]
        if np.linalg.norm(front) < .01:
            raise ValueError('Front and rear stickers are too close')
        forward = front / np.linalg.norm(front)
        left = np.array([-forward[1], forward[0]])
        positions = (metric - metric.mean(axis=0)) @ np.column_stack([forward, left])
    for sticker, position in zip(config['stickers'], positions):
        sticker['body_xy_m'] = np.asarray(position).tolist()
    config['setup_points_px'] = np.asarray(points).tolist()
    config['note'] = ('Measured body coordinates: +x forward, +y left, origin midway between middle-wheel hubs.'
                      if config['geometry_measured'] else
                      'PREVIEW: triangle learned from image; origin is sticker centroid, not chassis center.')
    tracker = StickerTracker(config)
    if tracker.detect(frame, grid['H_img2m']) is None:
        raise ValueError('Sampled colors/geometry did not detect this triangle: ' + tracker.status)
    return config


def main():
    parser = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument('--image', help='Overhead-camera snapshot; otherwise capture live UDP feed')
    parser.add_argument('--points-px', help="Scripted centers: 'green_x,y orange_x,y pink_x,y'")
    parser.add_argument('--positions-m', help="Measured body offsets in the same order: 'forward,left ...'")
    parser.add_argument('--height-m', type=float, default=None, help='sticker height; defaults to saved measurement')
    parser.add_argument('--camera-position-m', help='Measured camera position in grid coordinates: x,y,z')
    parser.add_argument('--plane-calib', help='Calibration JSON made at sticker height, in the grid XY frame')
    parser.add_argument('--calib', default=str(CALIB))
    parser.add_argument('--out', default=str(DEFAULT_CONFIG))
    args = parser.parse_args()
    try:
        with open(args.calib) as f:
            grid = json.load(f)
        saved = {}
        if Path(args.out).exists():
            with open(args.out) as f:
                saved = json.load(f)
        sample_frames = []
        if args.image:
            frame = cv2.imread(args.image)
        else:
            sys.path.insert(0, str(HERE.parent / 'capture'))
            from stream import LatestFrame
            source = LatestFrame()
            try:
                frame = source.wait_first(20) if source.opened() else None
                # ~1 s of distinct frames for colour sampling; keep the rover still
                last, deadline = None, time.time() + 2.0
                while frame is not None and len(sample_frames) < 15 and time.time() < deadline:
                    f, _, n = source.read()
                    if n != last:
                        sample_frames.append(f)
                        last = n
                    time.sleep(.02)
            finally:
                source.release()
        if frame is None:
            raise ValueError('No overhead frame. Start the camera stream and stop other UDP 5000 consumers.')
        if grid.get('image_size') != [frame.shape[1], frame.shape[0]]:
            raise ValueError('Grid calibration resolution does not match the overhead frame')
        points = parse_pairs(args.points_px) if args.points_px else click_points(frame)
        if points is None:
            return 1
        positions = parse_pairs(args.positions_m) if args.positions_m else None
        if positions is None and saved.get('geometry_measured'):
            positions = np.asarray([s['body_xy_m'] for s in saved['stickers']], float)
        height = args.height_m if args.height_m is not None else saved.get('sticker_height_m', 0.)
        camera = [float(v) for v in args.camera_position_m.split(',')] if args.camera_position_m else saved.get('camera_position_m')
        plane = None
        if args.plane_calib:
            with open(args.plane_calib) as f:
                plane_data = json.load(f)
            if plane_data.get('image_size') != grid['image_size']:
                raise ValueError('Sticker-plane calibration resolution does not match the grid')
            plane = plane_data['H_img2m']
        if not args.plane_calib:
            plane = saved.get('plane_H_img2m')
        config = build_config(frame, points, grid, positions, height, camera, plane, sample_frames)
        if not args.camera_position_m and not args.plane_calib:
            config['projection_verified'] = saved.get('projection_verified', False)
            if saved.get('camera_position_source'):
                config['camera_position_source'] = saved['camera_position_source']
        if args.camera_position_m:
            # calibrate_grid.py keeps a measured position (re-expressed) when the grid is redone
            config['camera_position_source'] = 'measured'
        path = Path(args.out)
        path.parent.mkdir(parents=True, exist_ok=True)
        temporary = path.with_suffix(path.suffix + '.tmp')
        temporary.write_text(json.dumps(config, indent=2) + '\n')
        temporary.replace(path)
        print('Saved', path)
        print(config['note'])
        if not StickerTracker(config).measurement_ready:
            print('Preview enabled. Metric trial recording requires measured body offsets and sticker-plane calibration.')
        return 0
    except (ValueError, OSError, KeyError) as exc:
        print('ERROR:', exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
