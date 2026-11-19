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


# Apply a 3x3 homography H to N (x, y) points -> N x 2 array (pixels <-> metres).
def transform_points(H, points):
    points_array = np.asarray(points, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points_array, np.asarray(H, np.float64)).reshape(-1, 2)


# 2-D rigid fit (Kabsch/Procrustes): find the rotation R and translation t that best
# map the sticker body coordinates onto the observed metric positions:
#   minimise sum |R @ body_i + t - observed_i|^2,  det(R) = +1 (no mirror, no scale).
# SVD of the cross-covariance gives R; the `correction` matrix flips a reflection back
# to a proper rotation. Returns (R, t = body origin in the floor frame, RMS residual m).
def fit_pose(body, observed):
    """Least-squares proper rotation + translation, with no scale/reflection fit."""
    body_xy, observed_xy = np.asarray(body, float), np.asarray(observed, float)
    body_centroid, observed_centroid = body_xy.mean(axis=0), observed_xy.mean(axis=0)
    svd_u, _, svd_vt = np.linalg.svd((body_xy - body_centroid).T @ (observed_xy - observed_centroid))
    correction = np.diag([1., np.linalg.det(svd_vt.T @ svd_u.T)])
    rotation = svd_vt.T @ correction @ svd_u.T
    center = observed_centroid - rotation @ body_centroid
    residual = float(np.sqrt(np.mean(np.sum((body_xy @ rotation.T + center - observed_xy) ** 2, axis=1))))
    return rotation, center, residual


# Binary mask of pixels inside ANY of the sticker's HSV ranges, with a 3x3 closing to
# fill small compression/glare holes inside a dot.
def color_mask(hsv, ranges):
    """OR ranges; separate intervals support red's OpenCV hue wrap at 179/0."""
    mask = np.zeros(hsv.shape[:2], np.uint8)
    for interval in ranges:
        mask |= cv2.inRange(hsv, np.array(interval['lower'], np.uint8),
                           np.array(interval['upper'], np.uint8))
    # A small close joins compression/glare holes without eroding small dots.
    return cv2.morphologyEx(mask, cv2.MORPH_CLOSE, np.ones((3, 3), np.uint8))


# Finds the rover's 3-sticker triangle in an overhead frame and returns its pose.
# config (sticker_config.json): per sticker a name, body_xy_m (position on the rover,
# +x forward, +y left, origin = chassis centre) and hsv_ranges; plus blob-size,
# roundness and geometry-error thresholds.
class StickerTracker:
    # Load and validate the config (path or dict): exactly 3 uniquely named stickers,
    # a non-collinear triangle, sane HSV ranges. measurement_ready = geometry measured AND
    # projection verified AND origin is the chassis centre (needed for recorded trials).
    def __init__(self, config=DEFAULT_CONFIG):
        if isinstance(config, (str, Path)):
            self.config_path = str(config)
            try:
                with open(config) as config_file:
                    self.config = json.load(config_file)
            except (OSError, ValueError) as exc:
                raise ValueError(f'Cannot load sticker config {config}: {exc}. Run {SETUP_COMMAND}') from exc
        else:
            self.config_path = None
            self.config = config
        loaded_config = self.config
        if loaded_config.get('schema_version') != 1:
            raise ValueError('Unsupported sticker configuration schema')
        self.stickers = loaded_config.get('stickers', [])
        if len(self.stickers) != 3 or len({sticker['name'] for sticker in self.stickers}) != 3:
            raise ValueError('Configure exactly three uniquely named stickers')
        self.body = np.asarray([sticker['body_xy_m'] for sticker in self.stickers], float)
        if self.body.shape != (3, 2) or not np.isfinite(self.body).all():
            raise ValueError(f'Sticker geometry is missing. Run {SETUP_COMMAND}')
        if abs(np.cross(self.body[1] - self.body[0], self.body[2] - self.body[0])) < 1e-5:
            raise ValueError('Sticker centers must form a non-collinear triangle')
        if loaded_config.get('reference_point') not in ('chassis_center', 'sticker_centroid'):
            raise ValueError('Set reference_point to chassis_center or sticker_centroid')
        for sticker in self.stickers:
            if not sticker.get('hsv_ranges'):
                raise ValueError(f"No HSV ranges for {sticker['name']}")
            for interval in sticker['hsv_ranges']:
                hsv_lower, hsv_upper = np.asarray(interval['lower']), np.asarray(interval['upper'])
                if (hsv_lower.shape != (3,) or hsv_upper.shape != (3,) or np.any(hsv_lower < 0)
                        or np.any(hsv_upper > [179, 255, 255]) or np.any(hsv_lower > hsv_upper)):
                    raise ValueError(f"Invalid HSV range for {sticker['name']}")
        self.reference_point = loaded_config['reference_point']
        self.measurement_ready = bool(loaded_config.get('geometry_measured') and loaded_config.get('projection_verified')
                                      and self.reference_point == 'chassis_center')
        self.status = 'waiting for three stickers'
        self.last_candidates = {}
        self.last_result = None

    # Raise with a list of what is missing if the config is only good for previews.
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

    # Pixel -> metre mapping for the STICKER plane, not the floor. Stickers sit
    # sticker_height_m above the floor, so the floor homography is scaled towards the
    # point below the camera: s = 1 - h / camera_z (similar triangles). A homography
    # calibrated directly at sticker height (plane_H_img2m) is used instead if present.
    def plane_homography(self, H_ground):
        """Map pixels to the sticker plane; retain the same grid XY coordinates."""
        config = self.config
        if config.get('plane_H_img2m') is not None:
            image_to_plane = np.asarray(config['plane_H_img2m'], float)
        else:
            image_to_plane = np.asarray(H_ground, float)
            height = float(config.get('sticker_height_m', 0.0))
            if height:
                camera = np.asarray(config.get('camera_position_m'), float)
                if camera.shape != (3,) or not np.isfinite(camera).all() or not 0 <= height < camera[2]:
                    raise ValueError('Height correction needs camera_position_m=[x,y,z], z above stickers')
                scale = 1.0 - height / camera[2]
                height_correction = np.array([[scale, 0, (1 - scale) * camera[0]],
                              [0, scale, (1 - scale) * camera[1]], [0, 0, 1]])
                image_to_plane = height_correction @ image_to_plane
        if image_to_plane.shape != (3, 3) or not np.isfinite(image_to_plane).all() or abs(np.linalg.det(image_to_plane)) < 1e-15:
            raise ValueError('Invalid pixel-to-sticker-plane homography')
        return image_to_plane

    # Per sticker colour: every blob of the right size (min/max_area_px) and roundness
    # (4*pi*area/perimeter^2 >= min_circularity), roundest first, at most
    # max_candidates_per_color. Each candidate: {'px': centroid, 'area', 'circularity'}.
    def candidates(self, frame):
        hsv = cv2.cvtColor(frame, cv2.COLOR_BGR2HSV)
        config = self.config
        found = {}
        for sticker in self.stickers:
            mask = color_mask(hsv, sticker['hsv_ranges'])
            contours, _ = cv2.findContours(mask, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
            blobs = []
            for contour in contours:
                area = cv2.contourArea(contour)
                perimeter = cv2.arcLength(contour, True)
                if not config.get('min_area_px', 8) <= area <= config.get('max_area_px', 2500) or perimeter <= 0:
                    continue
                circularity = 4 * math.pi * area / (perimeter * perimeter)
                if circularity < config.get('min_circularity', .45):
                    continue
                moments = cv2.moments(contour)
                centroid_px = np.array([moments['m10'] / moments['m00'], moments['m01'] / moments['m00']])
                blobs.append(dict(px=centroid_px, area=area, circularity=circularity))
            # Do not blindly choose the largest blob (wires/tape can share a hue).
            blobs.sort(key=lambda item: item['circularity'], reverse=True)
            found[sticker['name']] = blobs[:config.get('max_candidates_per_color', 8)]
        self.last_candidates = found
        return found

    # Pose of the rover in one frame, or None (self.status says why).
    # Tries every combination of one candidate per colour: rejects triples whose metric edge
    # lengths differ from the configured triangle by > max_edge_error_m, fits the rigid pose
    # (fit_pose) and keeps the lowest residual under max_fit_error_m. Heading comes from the
    # rotation matrix; centre/front pixels are projected back for drawing.
    def detect(self, frame, H_ground):
        config = self.config
        expected_size = config.get('image_size')
        if expected_size and list(frame.shape[1::-1]) != expected_size:
            raise ValueError(f'Camera resolution changed: expected {expected_size}, '
                             f'got {list(frame.shape[1::-1])}; recalibrate stickers and grid')
        image_to_plane = self.plane_homography(H_ground)
        candidate_pools = list(self.candidates(frame).values())
        missing = [sticker['name'] for sticker, candidate_pool in zip(self.stickers, candidate_pools) if not candidate_pool]
        if missing:
            self.status = 'missing: ' + ', '.join(missing)
            self.last_result = None
            return None
        best_fit = None
        for candidate_triple in itertools.product(*candidate_pools):
            pixels = np.array([candidate['px'] for candidate in candidate_triple])
            if min(np.linalg.norm(pixels[i] - pixels[j]) for i, j in ((0, 1), (0, 2), (1, 2))) < 3:
                continue  # overlapping color thresholds must not reuse the same dot
            stickers_xy_m = transform_points(image_to_plane, pixels)
            if not np.isfinite(stickers_xy_m).all():
                continue
            if any(abs(np.linalg.norm(stickers_xy_m[i] - stickers_xy_m[j]) - np.linalg.norm(self.body[i] - self.body[j]))
                   > config.get('max_edge_error_m', .015) for i, j in ((0, 1), (0, 2), (1, 2))):
                continue
            rotation, center, residual = fit_pose(self.body, stickers_xy_m)
            if residual > config.get('max_fit_error_m', .006):
                continue
            if best_fit is None or residual < best_fit[0]:
                best_fit = (residual, rotation, center, pixels)
        if best_fit is None:
            self.status = 'colors found, triangle geometry rejected'
            self.last_result = None
            return None
        residual, rotation, center, pixels = best_fit
        heading = math.degrees(math.atan2(rotation[1, 0], rotation[0, 0]))
        plane_to_image = np.linalg.inv(image_to_plane)
        center_px, front_px = transform_points(plane_to_image, [center, center + rotation @ [.06, 0]])
        result = dict(center_m=center, center_px=center_px, top_px=front_px,
                      heading_deg=heading, dist_m=float(np.linalg.norm(center)),
                      sticker_pixels=pixels, sticker_names=[sticker['name'] for sticker in self.stickers],
                      fit_error_m=residual, reference_point=self.reference_point,
                      measurement_ready=self.measurement_ready)
        self.status = f'3/3 stickers; fit {residual * 1000:.1f} mm'
        self.last_result = result
        return result


# Overlay the tracked triangle, the reference point, a heading arrow and a text line
# (or 'Tracking lost: <reason>') on `image` in place.
def draw_tracking(image, result, tracker):
    if result is None:
        cv2.putText(image, 'Tracking lost: ' + tracker.status, (10, 52),
                    cv2.FONT_HERSHEY_SIMPLEX, .55, (0, 150, 255), 2, cv2.LINE_AA)
        return
    sticker_px = np.int32(result['sticker_pixels'])
    cv2.polylines(image, [sticker_px], True, (255, 255, 255), 1, cv2.LINE_AA)
    for point, name in zip(sticker_px, result['sticker_names']):
        cv2.circle(image, tuple(point), 9, (255, 255, 255), 1, cv2.LINE_AA)
        cv2.putText(image, name, tuple(point + [10, -8]), cv2.FONT_HERSHEY_SIMPLEX,
                    .4, (255, 255, 255), 1, cv2.LINE_AA)
    center_px, front_px = np.int32(result['center_px']), np.int32(result['top_px'])
    cv2.circle(image, tuple(center_px), 4, (0, 0, 255), -1)
    cv2.arrowedLine(image, tuple(center_px), tuple(front_px), (0, 0, 255), 2, tipLength=.3)
    center_x_m, center_y_m = result['center_m']
    if result['measurement_ready']:
        label = 'chassis center'
    elif tracker.reference_point == 'chassis_center' and tracker.config.get('geometry_measured'):
        label = 'chassis center (camera estimate)'
    else:
        label = 'PREVIEW ' + result['reference_point']
    cv2.putText(image, f'{label}: ({center_x_m:.3f}, {center_y_m:.3f}) m  yaw={result["heading_deg"]:.1f} deg',
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
