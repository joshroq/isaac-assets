"""Grid calibration regressions on rendered tape grids with a known camera homography.

    python3 -m unittest discover -s sim2real_calib/calib -p test_grid_target.py -v
"""
import json
from pathlib import Path
import sys
import tempfile
import unittest

import cv2
import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parent / "overhead"))
sys.path.insert(0, str(HERE.parent / "viz"))

import grid_target as gt  # noqa: E402
import calibrate_grid as cg  # noqa: E402
import camera_model as cm  # noqa: E402


def camera_H(dims, tilt_m=(0.15, -0.25), res=(1280, 720), margin_px=50):
    """Grid metres -> pixels for an oblique camera over the grid, raised just enough
    that the whole grid (plus 0.1 m of tape overhang) is in view."""
    w, h = dims
    c = np.array([w / 2, h / 2])
    src = np.array([[0, 0], [w, 0], [w, h], [0, h]], float)
    ext = src + np.array([[-.1, -.1], [.1, -.1], [.1, .1], [-.1, .1]])
    for height in np.arange(1.0, 12.0, 0.05):
        cam = cm.OverheadCamera({"resolution": list(res), "horizontal_fov_deg": 70.4,
                                 "position_m": [c[0] + tilt_m[0], c[1] + tilt_m[1], height],
                                 "look_at_m": [c[0], c[1], 0.0], "up_hint": [0, 1, 0]})
        px = cam.ground_to_px(ext)
        if (px.min(axis=0) >= margin_px).all() and (px.max(axis=0) <= np.array(res) - margin_px).all():
            return cv2.findHomography(src, cam.ground_to_px(src))[0]
    raise AssertionError("grid never fits in view")


def render(dims, spacing, H_m2img, tape_m=0.025, ppm=600, res=(1280, 720), occlude=None, seed=0,
           border="inside"):
    """Beige textured floor, blue tape on each grid line, optional black box.
    border="inside": like the real grid, the border tapes lie inside W x H with their outer
    edge on 0/W; inner tapes are centred on their line. "centre": every tape centred."""
    rng = np.random.default_rng(seed)
    w, h = dims
    pad = 0.5
    W, Hh = int((w + 2 * pad) * ppm), int((h + 2 * pad) * ppm)
    canvas = np.empty((Hh, W, 3), np.uint8)
    canvas[:] = (150, 190, 205)
    canvas = np.clip(canvas + rng.normal(0, 8, canvas.shape), 0, 255).astype(np.uint8)
    A = np.array([[ppm, 0, pad * ppm], [0, -ppm, (h + pad) * ppm], [0, 0, 1]])
    half = tape_m / 2
    blue = (215, 120, 30)
    for x in gt.centre_lines(w, spacing[0], tape_m, border):
        a, b = gt.transform(A, [(x - half, -0.1), (x + half, h + 0.1)])
        cv2.rectangle(canvas, tuple(np.int32(a)), tuple(np.int32(b)), blue, -1)
    for y in gt.centre_lines(h, spacing[1], tape_m, border):
        a, b = gt.transform(A, [(-0.1, y - half), (w + 0.1, y + half)])
        cv2.rectangle(canvas, tuple(np.int32(a)), tuple(np.int32(b)), blue, -1)
    if occlude is not None:
        (x0, y0), (x1, y1) = occlude
        a, b = gt.transform(A, [(x0, y0), (x1, y1)])
        cv2.rectangle(canvas, tuple(np.int32(a)), tuple(np.int32(b)), (25, 25, 25), -1)
    img = cv2.warpPerspective(canvas, H_m2img @ np.linalg.inv(A), res, flags=cv2.INTER_AREA,
                              borderValue=(120, 120, 120))
    return np.clip(img + rng.normal(0, 3, img.shape), 0, 255).astype(np.uint8)


class GridTargetTest(unittest.TestCase):
    def solve_case(self, dims, spacing, click_noise_px=8.0, occlude=None, anchors=None,
                   layout="inside", border=gt.DEFAULT_BORDER, tape_m=gt.TAPE_WIDTH_M,
                   tape_width=gt.TAPE_WIDTH_M):
        H_true = camera_H(dims)
        frame = render(dims, spacing, H_true, occlude=occlude, border=layout, tape_m=tape_m)
        anchors_m = gt.outer_corners(dims)[0] if anchors is None else np.asarray(anchors, float)
        clicks = gt.transform(H_true, anchors_m)
        clicks += np.random.default_rng(1).normal(0, click_noise_px, clicks.shape)
        cal = cg.calibrate(frame, dims, spacing, anchors_m, clicks, border=border,
                           tape_width=tape_width)
        err = gt.area_disagreement(cal["H_img2m"], H_true, dims)
        return cal, err, frame, H_true

    def test_border_tape_inside_the_grid_gives_exact_metres(self):
        # the real grid: 1 inch (25.4 mm) tape, border tape inside the 1 m square
        cal, err, *_ = self.solve_case((1.0, 1.0), (0.25, 0.25), layout="inside",
                                       border="inside", tape_m=0.0254)
        self.assertLess(err, 0.0015)
        self.assertEqual(cal["tape"].tape_width_used_m, gt.TAPE_WIDTH_M)
        self.assertLess(float(np.sqrt(np.mean(cal["res"][cal["used"]] ** 2))), 0.002)

    def test_other_tape_width_can_be_measured(self):
        cal, err, *_ = self.solve_case((1.0, 1.0), (0.25, 0.25), layout="inside",
                                       border="inside", tape_m=0.035, tape_width=None)
        self.assertAlmostEqual(cal["tape"].tape_width_used_m, 0.035, delta=0.004)
        self.assertLess(err, 0.0025)

    def test_old_centre_assumption_on_inside_border_is_off_by_half_a_tape(self):
        # what calibrations did before 2026-11-04: the border tape centre taken as 0 / 1 m
        cal, err, *_ = self.solve_case((1.0, 1.0), (0.25, 0.25), layout="inside",
                                       border="centre", tape_m=0.0254)
        self.assertGreater(err, 0.008)                     # ~ half the 25.4 mm tape
        self.assertGreater(float(np.max(cal["res"][cal["used"]])), 0.006)

    def test_centred_border_layout_still_supported(self):
        cal, err, *_ = self.solve_case((1.0, 1.0), (0.25, 0.25), layout="centre",
                                       border="centre")
        self.assertLess(err, 0.002)

    def test_parse_size_and_spacing(self):
        self.assertEqual(gt.parse_size("2x2"), (2.0, 2.0))
        self.assertEqual(gt.parse_size("1x4"), (1.0, 4.0))
        self.assertEqual(gt.parse_size("1.5"), (1.5, 1.5))
        self.assertEqual(gt.parse_spacing("0.5", (1.0, 4.0)), (0.5, 0.5))
        self.assertEqual(gt.parse_spacing(None, (1.0, 4.0)), (1.0, 4.0))
        with self.assertRaises(ValueError):
            gt.parse_spacing("0.3", (1.0, 1.0))        # does not divide 1 m
        with self.assertRaises(ValueError):
            gt.parse_size("0x2")

    def test_legacy_square_files_still_load(self):
        self.assertEqual(gt.grid_dims({"grid_size_m": 1.0}), (1.0, 1.0))
        self.assertEqual(gt.grid_dims({"grid_size_m": [1, 4]}), (1.0, 4.0))
        self.assertEqual(gt.grid_dims({"grid_size_m": 1.0, "grid_width_m": 2,
                                       "grid_height_m": 2}), (2.0, 2.0))
        self.assertEqual(gt.grid_spacing({"grid_size_m": 1.0}), (1.0, 1.0))

    def test_1x1_quarter_grid_is_click_independent(self):
        cal, err, *_ = self.solve_case((1.0, 1.0), (0.25, 0.25), click_noise_px=12)
        self.assertEqual(cal["method"], "tape-intersection fit")
        self.assertEqual(int(cal["used"].sum()), 25)
        self.assertLess(err, 0.002)

    def test_2x2_half_metre_grid(self):
        cal, err, *_ = self.solve_case((2.0, 2.0), (0.5, 0.5))
        self.assertEqual(int(cal["used"].sum()), 25)
        self.assertLess(err, 0.003)

    def test_1x4_strip_long_side_up_the_image(self):
        cal, err, *_ = self.solve_case((1.0, 4.0), (0.5, 0.5))
        self.assertGreaterEqual(int(cal["used"].sum()), 25)
        self.assertLess(err, 0.004)

    def test_4x1_strip_long_side_across_the_image(self):
        cal, err, *_ = self.solve_case((4.0, 1.0), (0.5, 0.5))
        self.assertGreaterEqual(int(cal["used"].sum()), 25)
        self.assertLess(err, 0.003)

    def test_occluded_crossing_is_skipped_not_misplaced(self):
        cal, err, *_ = self.solve_case((2.0, 2.0), (0.5, 0.5),
                                       occlude=((0.85, 0.85), (1.15, 1.15)))
        missed = [f["m"].tolist() for f in cal["found"] if f["px"] is None]
        self.assertIn([1.0, 1.0], missed)
        self.assertLess(err, 0.003)

    def test_interior_anchors_when_corners_are_out_of_view(self):
        cal, err, *_ = self.solve_case((1.0, 4.0), (0.5, 0.5),
                                       anchors=[(0, 0.5), (1, 0.5), (1, 3.5), (0, 3.5)])
        self.assertLess(err, 0.004)

    def test_outer_corners_only(self):
        cal, err, *_ = self.solve_case((2.0, 2.0), (2.0, 2.0))
        self.assertEqual(int(cal["used"].sum()), 4)
        self.assertLess(err, 0.004)

    def test_gross_outlier_rejected(self):
        dims, spacing = (2.0, 2.0), (0.5, 0.5)
        H = camera_H(dims)
        m = gt.intersections(dims, spacing)
        px = gt.transform(H, m)
        px[7] += (25, -18)                                  # one wildly wrong point
        Hs, used, res = gt.solve(px, m)
        self.assertFalse(used[7])
        self.assertLess(gt.area_disagreement(Hs, H, dims), 1e-6)

    def test_saved_json_and_verify(self):
        dims, spacing = (2.0, 2.0), (0.5, 0.5)
        cal, err, frame, H_true = self.solve_case(dims, spacing)
        data = cg.build_json(cal, frame, dims, spacing, gt.outer_corners(dims)[0],
                             gt.transform(H_true, gt.outer_corners(dims)[0]))
        self.assertEqual((data["grid_width_m"], data["grid_height_m"]), dims)
        self.assertEqual(data["grid_size_m"], 2.0)
        self.assertEqual(gt.grid_dims(data), dims)
        self.assertLess(data["rms_residual_m"], 0.002)
        with tempfile.TemporaryDirectory() as tmp:
            path = Path(tmp) / "grid.json"
            path.write_text(json.dumps(data))
            self.assertEqual(cg.verify(frame, str(path)), 0)
            # a calibration shifted by 2 cm must fail verification
            bad = dict(data)
            shift = np.array([[1, 0, 0.02], [0, 1, 0], [0, 0, 1]])
            bad["H_img2m"] = (shift @ np.array(data["H_img2m"])).tolist()
            path.write_text(json.dumps(bad))
            self.assertEqual(cg.verify(frame, str(path)), 1)
        rect = cg.build_json(cal, frame, (2.0, 2.0), spacing, gt.outer_corners(dims)[0],
                             gt.transform(H_true, gt.outer_corners(dims)[0]))
        self.assertEqual(rect["n_intersections"], 25)

    def test_rectangular_overlay_draws(self):
        from metric_overlay import draw_metric_grid, draw_axes_and_labels
        dims = (1.0, 4.0)
        H = camera_H(dims)
        img = np.zeros((720, 1280, 3), np.uint8)
        draw_metric_grid(img, H, dims)
        draw_axes_and_labels(img, H, dims)
        top_right = gt.transform(H, [(1.0, 4.0)])[0]
        x, y = np.int32(top_right)
        self.assertGreater(img[max(0, y - 3):y + 4, max(0, x - 3):x + 4].sum(), 0)
        draw_metric_grid(img, H, 1.0)                       # legacy scalar still accepted


if __name__ == "__main__":
    unittest.main()
