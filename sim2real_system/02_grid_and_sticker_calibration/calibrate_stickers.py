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


# Parse 'x,y x,y x,y' into a count x 2 float array (pixels or metres).
def parse_pairs(text, count=3):
    points = np.asarray([[float(coordinate) for coordinate in pair_text.split(',')] for pair_text in text.split()], float)
    if points.shape != (count, 2) or not np.isfinite(points).all():
        raise ValueError(f'Expected {count} finite x,y pairs separated by spaces')
    return points


# Learn one sticker's HSV colour range from the pixels around its clicked centre.
# OpenCV HSV: hue 0-179 (a circle - red wraps from 179 back to 0), saturation and
# value (brightness) 0-255. Steps: pool a (2*radius+1)^2 patch over all frames, drop grey
# or dark pixels, take the circular mean hue, widen it by the 90th-percentile spread
# (8-18 hue steps), and set saturation/brightness floors from the 20th percentiles.
# Returns a list of {lower, upper} ranges - two when the hue range wraps past 0/179.
def sample_color(frames, point, radius=3):
    """HSV ranges for the sticker at point; pass several frames of a still rover so
    H.264 noise on a ~10 px dot does not set the thresholds."""
    frames = frames if isinstance(frames, (list, tuple)) else [frames]
    center_x, center_y = (int(round(coordinate)) for coordinate in point)
    image_h, image_w = frames[0].shape[:2]
    if not radius <= center_x < image_w - radius or not radius <= center_y < image_h - radius:
        raise ValueError('Click inside a sticker, away from the image boundary')
    hsv_pixels = np.concatenate([cv2.cvtColor(frame[center_y-radius:center_y+radius+1, center_x-radius:center_x+radius+1],
                                          cv2.COLOR_BGR2HSV).reshape(-1, 3) for frame in frames])
    hsv_pixels = hsv_pixels[(hsv_pixels[:, 1] >= 15) & (hsv_pixels[:, 2] >= 60)]
    if len(hsv_pixels) < 8:
        raise ValueError('Not enough colored pixels; click inside the colored sticker')
    # Circular mean is essential for orange-red straddling hue 179/0.
    hue_angles_rad = hsv_pixels[:, 0] * (2 * math.pi / 180)
    center_hue = int(round(math.atan2(np.sin(hue_angles_rad).mean(), np.cos(hue_angles_rad).mean()) * 180 / (2 * math.pi))) % 180
    hue_distance = np.abs((hsv_pixels[:, 0].astype(float) - center_hue + 90) % 180 - 90)
    hue_margin = max(8, min(18, int(math.ceil(np.percentile(hue_distance, 90))) + 4))
    # Harsh light washes stickers out (orange-red reaches S~30 at V 255); a floor of 30
    # fragments such a dot below min_circularity, so go lower but demand brightness:
    # a washed-out sticker is only separable from warm-grey mat/concrete when it is bright,
    # and bright concrete sits at S~15-21, so keep a washed-out floor close to the sticker.
    saturation_p20 = np.percentile(hsv_pixels[:, 1], 20)
    min_saturation = max(15, int(saturation_p20 * (.7 if saturation_p20 < 50 else .6)))
    min_brightness = max(60, int(np.percentile(hsv_pixels[:, 2], 20) * (.8 if min_saturation < 30 else .55)))
    hsv_ranges = []
    hue_low, hue_high = center_hue - hue_margin, center_hue + hue_margin
    hue_spans = [(max(0, hue_low), min(179, hue_high))]
    if hue_low < 0:
        hue_spans.append((180 + hue_low, 179))
    if hue_high > 179:
        hue_spans.append((0, hue_high - 180))
    for span_low, span_high in hue_spans:
        hsv_ranges.append(dict(lower=[span_low, min_saturation, min_brightness], upper=[span_high, 255, 255]))
    return hsv_ranges


# OpenCV window: the user clicks the green, orange-red, pink sticker centres in that
# order (U undo, Enter accept, Q/Esc cancel). Returns a 3 x 2 pixel array or None.
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
            display_frame = frame.copy()
            for point, name in zip(points, NAMES):
                x, y = (int(v) for v in point)
                cv2.circle(display_frame, (x, y), 8, (255, 255, 255), 1)
                cv2.putText(display_frame, name, (x + 10, y), cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1)
            instruction = 'Click ' + NAMES[len(points)] if len(points) < 3 else 'Enter to save'
            cv2.putText(display_frame, instruction, (10, 28), cv2.FONT_HERSHEY_SIMPLEX, .7, (0, 255, 255), 2)
            cv2.imshow(title, display_frame)
            pressed_key = cv2.waitKey(20) & 0xff
            if pressed_key in (ord('q'), 27):
                return None
            if pressed_key == ord('u') and points:
                points.pop()
            if pressed_key in (10, 13) and len(points) == 3:
                return np.asarray(points, float)
    finally:
        cv2.destroyWindow(title)


# Build the full sticker_config.json dict: tracker thresholds, the sampled colour
# ranges, and each sticker's body position. Measured positions are used when given;
# otherwise the triangle seen in the image becomes a PREVIEW geometry. The result is
# checked by actually detecting the triangle in `frame` before it is returned.
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
    config['stickers'] = [dict(name=sticker_name, body_xy_m=provisional_xy_m, hsv_ranges=sample_color(sample_frames or frame, click_px))
                          for sticker_name, provisional_xy_m, click_px in zip(NAMES, [[.07,.05],[.07,-.05],[-.07,0]], points)]
    provisional_tracker = StickerTracker(config)
    sticker_plane_H = provisional_tracker.plane_homography(grid['H_img2m'])
    stickers_xy_m = transform_points(sticker_plane_H, points)
    if positions is None:
        front_vector = (stickers_xy_m[0] + stickers_xy_m[1]) / 2 - stickers_xy_m[2]
        if np.linalg.norm(front_vector) < .01:
            raise ValueError('Front and rear stickers are too close')
        forward_unit = front_vector / np.linalg.norm(front_vector)
        left_unit = np.array([-forward_unit[1], forward_unit[0]])
        positions = (stickers_xy_m - stickers_xy_m.mean(axis=0)) @ np.column_stack([forward_unit, left_unit])
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


# Load the grid calibration and any saved sticker config, grab a live frame (plus ~1 s
# of frames for colour sampling), get the 3 click points (clicked or --points-px), keep
# the saved measured geometry / height / camera position unless overridden, build and
# verify the config, then write it atomically (temp file + rename).
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
            camera_stream = LatestFrame()
            try:
                frame = camera_stream.wait_first(20) if camera_stream.opened() else None
                # ~1 s of distinct frames for colour sampling; keep the rover still
                last_counter, deadline = None, time.time() + 2.0
                while frame is not None and len(sample_frames) < 15 and time.time() < deadline:
                    f, _, frame_counter = camera_stream.read()
                    if frame_counter != last_counter:
                        sample_frames.append(f)
                        last_counter = frame_counter
                    time.sleep(.02)
            finally:
                camera_stream.release()
        if frame is None:
            raise ValueError('No overhead frame. Start the camera stream and stop other UDP 5000 consumers.')
        if grid.get('image_size') != [frame.shape[1], frame.shape[0]]:
            raise ValueError('Grid calibration resolution does not match the overhead frame')
        points = parse_pairs(args.points_px) if args.points_px else click_points(frame)
        if points is None:
            return 1
        positions = parse_pairs(args.positions_m) if args.positions_m else None
        if positions is None and saved.get('geometry_measured'):
            positions = np.asarray([sticker['body_xy_m'] for sticker in saved['stickers']], float)
        height = args.height_m if args.height_m is not None else saved.get('sticker_height_m', 0.)
        camera = [float(coordinate) for coordinate in args.camera_position_m.split(',')] if args.camera_position_m else saved.get('camera_position_m')
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
        out_path = Path(args.out)
        out_path.parent.mkdir(parents=True, exist_ok=True)
        temporary_path = out_path.with_suffix(out_path.suffix + '.tmp')
        temporary_path.write_text(json.dumps(config, indent=2) + '\n')
        temporary_path.replace(out_path)
        print('Saved', out_path)
        print(config['note'])
        if not StickerTracker(config).measurement_ready:
            print('Preview enabled. Metric trial recording requires measured body offsets and sticker-plane calibration.')
        return 0
    except (ValueError, OSError, KeyError) as exc:
        print('ERROR:', exc, file=sys.stderr)
        return 2


if __name__ == '__main__':
    raise SystemExit(main())
