"""Color-labelled triangle tracking in the overhead camera's metric frame.

Three round stickers identify a rigid 2-D body. The fit returns its configured
origin, not the triangle centroid unless that is explicitly the chosen reference.
No ArUco dependency or legacy marker-to-center correction is used.
"""
from __future__ import annotations

import itertools
import json
import math
from pathlib import Path

import cv2
import numpy as np

DEFAULT_CONFIG = Path(__file__).with_name('sticker_config.json')
SETUP_COMMAND = 'python3 sim2real_calib/calib/calibrate_stickers.py'


def transform_points(H, points):
    p = np.asarray(points, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, np.asarray(H, np.float64)).reshape(-1, 2)


def fit_pose(body, observed):
    """Least-squares proper rotation + translation, with no scale/reflection fit."""
    a, b = np.asarray(body, float), np.asarray(observed, float)
    ac, bc = a.mean(axis=0), b.mean(axis=0)
    u, _, vt = np.linalg.svd((a - ac).T @ (b - bc))
    correction = np.diag([1., np.linalg.det(vt.T @ u.T)])
    rotation = vt.T @ correction @ u.T
    center = bc - rotation @ ac
    residual = float(np.sqrt(np.mean(np.sum((a @ rotation.T + center - b) ** 2, axis=1))))
    return rotation, center, residual


def color_mask(hsv, ranges):
    """OR ranges; separate intervals support red's OpenCV hue wrap at 179/0."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for interval in ranges:
        mask |= cv2.inRange(hsv, np.array(interval['lower'], np.uint8),
                           np.array(interval['upper'], np.uint8))
    # A small close joins compression/glare holes without eroding small dots.
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


class StickerTracker:
    def __init__(self, config=DEFAULT_CONFIG):
        if isinstance(config, (str, Path)):
            self.config_path = str(config)
            try:
                with open(config) as f:
                    self.config = json.load(f)
            except (OSError, ValueError) as exc:
                raise ValueError(f'Cannot load sticker config {config}: {exc}. Run {SETUP_COMMAND}') from exc
        else:
            self.config_path = None
            self.config = config
        c = self.config
        if c.get('schema_version') != 1:
            raise ValueError('Unsupported sticker configuration schema')
        self.stickers = c.get('stickers', [])
        if len(self.stickers) != 3 or len({s['name'] for s in self.stickers}) != 3:
            raise ValueError('Configure exactly three uniquely named stickers')
        self.body = np.asarray([s['body_xy_m'] for s in self.stickers], float)
        if self.body.shape != (3, 2) or not np.isfinite(self.body).all():
            raise ValueError(f'Sticker geometry is missing. Run {SETUP_COMMAND}')
        if abs(np.cross(self.body[1] - self.body[0], self.body[2] - self.body[0])) < 1e-5:
            raise ValueError('Sticker centers must form a non-collinear triangle')
        if c.get('reference_point') not in ('chassis_center', 'sticker_centroid'):
            raise ValueError('Set reference_point to chassis_center or sticker_centroid')
        for sticker in self.stickers:
            if not sticker.get('hsv_ranges'):
                raise ValueError(f"No HSV ranges for {sticker['name']}")
            for interval in sticker['hsv_ranges']:
                lo, hi = np.asarray(interval['lower']), np.asarray(interval['upper'])
                if (lo.shape != (3,) or hi.shape != (3,) or np.any(lo < 0)
                        or np.any(hi > [179, 255, 255]) or np.any(lo > hi)):
                    raise ValueError(f"Invalid HSV range for {sticker['name']}")
        self.reference_point = c['reference_point']
        self.measurement_ready = bool(c.get('geometry_measured') and c.get('projection_verified')
                                      and self.reference_point == 'chassis_center')
        self.status = 'waiting for three stickers'
        self.last_candidates = {}
        self.last_result = None

    def require_measurement_ready(self):
        if not self.measurement_ready:
            missing = []
            if not self.config.get('geometry_measured'):
                missing.append('measured sticker offsets')
            if self.reference_point != 'chassis_center':
                missing.append('a chassis-center reference')
            if not self.config.get('projection_verified'):
                missing.append('verified sticker-plane projection')
            raise ValueError('Sticker tracker is configured for preview: precision recording still needs '
                             + ', '.join(missing) + f'. Run {SETUP_COMMAND}; '
                             'or explicitly use --allow-approximate for a diagnostic CSV.')

    def plane_homography(self, H_ground):
        """Map pixels to the sticker plane; retain the same grid XY coordinates."""
        c = self.config
        if c.get('plane_H_img2m') is not None:
            H = np.asarray(c['plane_H_img2m'], float)
        else:
            H = np.asarray(H_ground, float)
            height = float(c.get('sticker_height_m', 0.0))
            if height:
                camera = np.asarray(c.get('camera_position_m'), float)
                if camera.shape != (3,) or not np.isfinite(camera).all() or not 0 <= height < camera[2]:
                    raise ValueError('Height correction needs camera_position_m=[x,y,z], z above stickers')
                scale = 1.0 - height / camera[2]
                A = np.array([[scale, 0, (1 - scale) * camera[0]],
                              [0, scale, (1 - scale) * camera[1]], [0, 0, 1]])
                H = A @ H
        if H.shape != (3, 3) or not np.isfinite(H).all() or abs(np.linalg.det(H)) < 1e-15:
            raise ValueError('Invalid pixel-to-sticker-plane homography')
        return H

    def candidates(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        c = self.config
        found = {}
        for sticker in self.stickers:
            mask = color_mask(hsv, sticker['hsv_ranges'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            items = []
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                if not c.get('min_area_px', 8) <= area <= c.get('max_area_px', 2500) or perimeter <= 0:
                    continue
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                if circularity < c.get('min_circularity', .45):
                    continue
                moments = cv2.moments(contour)
                px = np.array([moments['m10'] / moments['m00'], moments['m01'] / moments['m00']])
                items.append(dict(px=px, area=area, circularity=circularity))
            # Do not blindly choose the largest blob (wires/tape can share a hue).
            items.sort(key=lambda item: item['circularity'], reverse=True)
            found[sticker['name']] = items[:c.get('max_candidates_per_color', 8)]
        self.last_candidates = found
        return found

    def detect(self, frame, H_ground):
        c = self.config
        expected_size = c.get('image_size')
        if expected_size and list(frame.shape[1::-1]) != expected_size:
            raise ValueError(f'Camera resolution changed: expected {expected_size}, '
                             f'got {list(frame.shape[1::-1])}; recalibrate stickers and grid')
        H = self.plane_homography(H_ground)
        pools = list(self.candidates(frame).values())
        missing = [s['name'] for s, pool in zip(self.stickers, pools) if not pool]
        if missing:
            self.status = 'missing: ' + ', '.join(missing)
            self.last_result = None
            return None
        best = None
        for combination in itertools.product(*pools):
            pixels = np.array([item['px'] for item in combination])
            if min(np.linalg.norm(pixels[i] - pixels[j]) for i, j in ((0, 1), (0, 2), (1, 2))) < 3:
                continue  # overlapping color thresholds must not reuse the same dot
            metric = transform_points(H, pixels)
            if not np.isfinite(metric).all():
                continue
            if any(abs(np.linalg.norm(metric[i] - metric[j]) - np.linalg.norm(self.body[i] - self.body[j]))
                   > c.get('max_edge_error_m', .015) for i, j in ((0, 1), (0, 2), (1, 2))):
                continue
            rotation, center, residual = fit_pose(self.body, metric)
            if residual > c.get('max_fit_error_m', .006):
                continue
            if best is None or residual < best[0]:
                best = (residual, rotation, center, pixels)
        if best is None:
            self.status = 'colors found, triangle geometry rejected'
            self.last_result = None
            return None
        residual, rotation, center, pixels = best
        heading = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        inv = np.linalg.inv(H)
        center_px, front_px = transform_points(inv, [center, center + rotation @ [.06, 0]])
        result = dict(center_m=center, center_px=center_px, top_px=front_px,
                      heading_deg=heading, dist_m=float(np.linalg.norm(center)),
                      sticker_pixels=pixels, sticker_names=[s['name'] for s in self.stickers],
                      fit_error_m=residual, reference_point=self.reference_point,
                      measurement_ready=self.measurement_ready)
        self.status = f'3/3 stickers; fit {residual * 1000:.1f} mm'
        self.last_result = result
        return result


def draw_tracking(image, result, tracker):
    if result is None:
        cv2.putText(image, 'Tracking lost: ' + tracker.status, (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 150, 255), 2, cv2.LINE_AA)
        return
    pts = np.int32(result['sticker_pixels'])
    cv2.polylines(image, [pts], True, (255, 255, 255), 1, cv2.LINE_AA)
    for point, name in zip(pts, result['sticker_names']):
        cv2.circle(image, tuple(point), 9, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, name, tuple(point + [10, -8]), cv2.FONT_HERSHEY_SIMPLEX,
                    .4, (255, 255, 255), 1, cv2.LINE_AA)
    center, front = np.int32(result['center_px']), np.int32(result['top_px'])
    cv2.circle(image, tuple(center), 4, (0, 0, 255), -1)
    cv2.arrowedLine(image, tuple(center), tuple(front), (0, 0, 255), 2, tipLength=.3)
    x, y = result['center_m']
    if result['measurement_ready']:
        label = 'chassis center'
    elif tracker.reference_point == 'chassis_center' and tracker.config.get('geometry_measured'):
        label = 'chassis center (camera estimate)'
    else:
        label = 'PREVIEW ' + result['reference_point']
    cv2.putText(image, f'{label}: ({x:.3f}, {y:.3f}) m  yaw={result["heading_deg"]:.1f} deg',
                (10, 52), cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 255, 255), 2, cv2.LINE_AA)


if __name__ == '__main__':
    import argparse
    import sys
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--check-config', action='store_true', required=True)
    parser.add_argument('--stickers', default=str(DEFAULT_CONFIG))
    args = parser.parse_args()
    try:
        tracker = StickerTracker(args.stickers)
        tracker.require_measurement_ready()
        print('Sticker tracker ready for chassis-center recording')
    except ValueError as exc:
        print('ERROR:', exc, file=sys.stderr)
        raise SystemExit(2)
