"""Tracker regressions using rendered dots with known metric poses."""
import copy
import json
import math
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

from sticker_tracker import StickerTracker, transform_points
from calibrate_stickers import build_config


class StickerTrackerTest(unittest.TestCase):
    def setUp(self):
        self.H = np.array([[.0015, 0, -.6], [0, -.0015, .6], [0, 0, 1]])
        self.positions = np.array([[.075, .055], [.065, -.055], [-.08, .01]])
        ranges = [([70, 100, 80], [95, 255, 255]),
                  ([0, 80, 80], [10, 255, 255]), ([145, 35, 80], [170, 255, 255])]
        self.config = dict(schema_version=1, image_size=[800, 700], geometry_measured=True,
                           projection_verified=True, reference_point='chassis_center',
                           max_fit_error_m=.003, max_edge_error_m=.008,
                           stickers=[dict(name=n, body_xy_m=p.tolist(),
                                          hsv_ranges=[dict(lower=lo, upper=hi)])
                                     for n, p, (lo, hi) in zip(
                                         ['green', 'orange_red', 'pink'], self.positions, ranges)])
        self.config['stickers'][1]['hsv_ranges'].append(dict(lower=[174,80,80], upper=[179,255,255]))
        self.colors = [self.bgr(84,230,220), self.bgr(2,180,245), self.bgr(158,70,240)]

    @staticmethod
    def bgr(h, s, v):
        return tuple(int(x) for x in cv2.cvtColor(np.uint8([[[h,s,v]]]), cv2.COLOR_HSV2BGR)[0,0])

    def frame(self, heading=0., center=(0., 0.), H=None):
        rotation = np.array([[math.cos(heading), -math.sin(heading)],
                             [math.sin(heading), math.cos(heading)]])
        observed = self.positions @ rotation.T + center
        pixels = transform_points(np.linalg.inv(self.H if H is None else H), observed)
        image = np.full((700,800,3), 35, np.uint8)
        for pixel, color in zip(pixels, self.colors):
            cv2.circle(image, tuple(np.int32(np.rint(pixel))), 6, color, -1)
        return image, pixels

    def test_center_and_heading_across_full_rotation(self):
        tracker = StickerTracker(self.config)
        for heading in np.linspace(-math.pi, math.pi, 13):
            image, _ = self.frame(heading, (.04,-.03))
            result = tracker.detect(image, self.H)
            self.assertIsNotNone(result, tracker.status)
            self.assertLess(np.linalg.norm(result['center_m'] - [.04,-.03]), .0015)
            error = (math.radians(result['heading_deg']) - heading + math.pi) % (2*math.pi) - math.pi
            self.assertLess(abs(error), .02)
            self.assertTrue(result['measurement_ready'])

    def test_same_color_distractors_do_not_replace_stickers(self):
        image, _ = self.frame(.4)
        for color, point in zip(self.colors, [(90,100),(650,580),(700,80)]):
            cv2.circle(image, point, 14, color, -1)
        result = StickerTracker(self.config).detect(image, self.H)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result['center_m']), .002)

    def test_missing_sticker_loses_pose_without_reusing_previous(self):
        image, pixels = self.frame()
        tracker = StickerTracker(self.config)
        self.assertIsNotNone(tracker.detect(image,self.H))
        cv2.circle(image, tuple(np.int32(np.rint(pixels[2]))), 12, (35,35,35), -1)
        self.assertIsNone(tracker.detect(image,self.H))
        self.assertIsNone(tracker.last_result)
        self.assertIn('pink', tracker.status)

    def test_wrong_triangle_and_reflection_are_rejected(self):
        image, pixels = self.frame()
        cv2.circle(image, tuple(np.int32(np.rint(pixels[2]))), 12, (35,35,35), -1)
        cv2.circle(image, (int(pixels[2,0]+60),int(pixels[2,1])),6,self.colors[2],-1)
        self.assertIsNone(StickerTracker(self.config).detect(image,self.H))
        image, _ = self.frame()
        self.assertIsNone(StickerTracker(self.config).detect(cv2.flip(image,1),self.H))

    def test_hue_wrap_and_low_saturation_pink(self):
        for hue in [0,178]:
            self.colors[1] = self.bgr(hue,180,245)
            image, _ = self.frame()
            self.assertIsNotNone(StickerTracker(self.config).detect(image,self.H))

    def test_height_corrected_pose(self):
        config = copy.deepcopy(self.config)
        config.update(sticker_height_m=.15875, camera_position_m=[.3,.2,1.55])
        tracker = StickerTracker(config)
        H = tracker.plane_homography(self.H)
        image, _ = self.frame(.6, (.07,.04), H)
        result = tracker.detect(image,self.H)
        self.assertIsNotNone(result)
        self.assertLess(np.linalg.norm(result['center_m'] - [.07,.04]),.0015)

    def test_preview_is_not_silently_recorded_as_chassis_center(self):
        image, pixels = self.frame(.3)
        grid = dict(H_img2m=self.H.tolist(),image_size=[800,700])
        config = build_config(image,pixels,grid)
        tracker = StickerTracker(config)
        self.assertEqual(tracker.reference_point,'sticker_centroid')
        self.assertIsNotNone(tracker.detect(image,self.H))
        with self.assertRaisesRegex(ValueError,'preview'):
            tracker.require_measurement_ready()
        with tempfile.TemporaryDirectory() as folder:
            path=Path(folder)/'config.json'; path.write_text(json.dumps(config))
            sys.path.insert(0,str(Path(__file__).resolve().parents[1]/'trials'))
            from record_traj import record_camera
            out=Path(folder)/'real.csv'
            self.assertEqual(record_camera(str(out),.1,
                str(Path(__file__).resolve().parents[1]/'calib'/'grid_calib.json'),str(path)),2)
            self.assertFalse(out.exists())

    def test_measured_setup_and_plane_calibration(self):
        image,pixels=self.frame(.3)
        grid=dict(H_img2m=self.H.tolist(),image_size=[800,700])
        config=build_config(image,pixels,grid,positions=self.positions,plane=self.H.tolist())
        tracker=StickerTracker(config)
        tracker.require_measurement_ready()
        result=tracker.detect(image,self.H)
        self.assertLess(np.linalg.norm(result['center_m']),.002)

    def test_resolution_change_and_collinear_geometry_fail(self):
        image,_=self.frame()
        with self.assertRaisesRegex(ValueError,'resolution'):
            StickerTracker(self.config).detect(cv2.resize(image,(400,350)),self.H)
        config=copy.deepcopy(self.config)
        for s,p in zip(config['stickers'],[[0,0],[.1,0],[.2,0]]):s['body_xy_m']=p
        with self.assertRaisesRegex(ValueError,'non-collinear'):
            StickerTracker(config)


if __name__=='__main__':
    unittest.main()
