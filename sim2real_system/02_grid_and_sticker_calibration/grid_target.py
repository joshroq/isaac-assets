"""Rectangular tape-grid target of any size: geometry, intersection detection, solve.

The floor grid is a W x H metre rectangle of tape lines every (sx, sy) metres.
Metric frame (unchanged from the original 1x1 m calibration): origin at the
bottom-left outer corner as seen by the overhead camera, +X towards the
bottom-right corner, +Y towards the top-left corner, metres.

Calibration only needs four clicked anchors for a first homography. Every tape
intersection is then located automatically in a top-down rectified view: the
tape colour is sampled at the predicted intersections, the floor colour in the
cell centres, and each intersection is the crossing of the two tape centrelines
fitted on the scanlines around it (works for +, T and L junctions; the crossing
patch itself is excluded). The final homography is a least-squares fit over all
detected intersections, so its residuals measure both click-free accuracy and
how straight the tape was laid.
"""
from __future__ import annotations

import math
import re

import cv2
import numpy as np

# Lab channel weights for tape/floor colour distance: lightness down-weighted so
# shadows and the black rover body are not mistaken for coloured tape.
LAB_WEIGHTS = np.array([0.25, 1.0, 1.0], np.float32)


# ---------------------------------------------------------------- geometry
# Grid size text -> (width, height) in metres; one number means a square grid.
def parse_size(text):
    """'2x2' | '1x4' | '1.5' -> (width_x_m, height_y_m)."""
    size_parts = [part for part in re.split(r"[x,X*×]", str(text).strip()) if part]
    sizes_m = [float(part) for part in size_parts]
    if len(sizes_m) == 1:
        sizes_m = sizes_m * 2
    if len(sizes_m) != 2 or not all(math.isfinite(value) and value > 0 for value in sizes_m):
        raise ValueError(f"grid size must look like 2x2 or 1x4 (metres), got {text!r}")
    return float(sizes_m[0]), float(sizes_m[1])


# Tape spacing text -> (x spacing, y spacing) in metres, checked to divide the grid.
def parse_spacing(text, dims):
    """'0.5' | '0.5,0.25' | None (outer edges only) -> (sx, sy)."""
    if text is None:
        return float(dims[0]), float(dims[1])
    spacings_m = [float(value) for value in re.split(r"[x,X]", str(text).strip()) if value]
    if len(spacings_m) == 1:
        spacings_m = spacings_m * 2
    if len(spacings_m) != 2 or not all(math.isfinite(value) and value > 0 for value in spacings_m):
        raise ValueError(f"spacing must look like 0.5 or 0.5,0.25 (metres), got {text!r}")
    axis_lines(dims[0], spacings_m[0])
    axis_lines(dims[1], spacings_m[1])
    return float(spacings_m[0]), float(spacings_m[1])


# Positions 0, step, 2*step ... length of the tape lines along one axis (error if the
# step doesn't divide the length).
def axis_lines(length, step):
    segment_count = int(round(length / step))
    if segment_count < 1 or abs(segment_count * step - length) > 1e-6 * max(1.0, length):
        raise ValueError(f"tape spacing {step:g} m does not divide the {length:g} m side")
    return np.linspace(0.0, length, segment_count + 1)


# Grid (width, height) from a saved calibration dict.
def grid_dims(calib):
    """(width_x_m, height_y_m) from a grid_calib dict, old square files included."""
    if "grid_width_m" in calib and "grid_height_m" in calib:
        return float(calib["grid_width_m"]), float(calib["grid_height_m"])
    size = calib.get("grid_size_m", 1.0)
    if isinstance(size, (list, tuple)):
        return float(size[0]), float(size[1])
    return float(size), float(size)


# Normalise a grid size (scalar or pair) to a (width, height) tuple.
def as_dims(grid_m):
    """Accept the legacy scalar grid size or a (w, h) pair."""
    if isinstance(grid_m, (list, tuple, np.ndarray)):
        return float(grid_m[0]), float(grid_m[1])
    return float(grid_m), float(grid_m)


# Saved tape spacing, or (width, height) = only the outer edges.
def grid_spacing(calib):
    width_m, height_m = grid_dims(calib)
    spacing_m = calib.get("grid_spacing_m")
    return (float(spacing_m[0]), float(spacing_m[1])) if spacing_m else (width_m, height_m)


# How the BORDER tape relates to the grid size. The physical grid (2026-11-04): the border
# tapes lie INSIDE the W x H rectangle with their OUTER edge on 0 / W (so the clicked outer
# corners are exactly (0,0)..(W,H)), while the inner lines (0.25, 0.5, 0.75 m) are measured
# to the tape CENTRE. "centre" = every line, border included, measured to its centre.
BORDERS = ("inside", "centre")
DEFAULT_BORDER = "inside"
# The grid tape is exactly 1 inch wide. Measuring it from the image read 21-30 mm depending
# on the light (22.6 mm at 20:24 on 2026-11-04), and half of that error goes straight into
# the border inset, so the known width is used; None = measure it (other tape).
TAPE_WIDTH_M = 0.0254


# Where each tape's CENTRELINE lies along one axis. With border='inside', the two
# border tapes sit inside the square, so their centres move in by half a tape width.
def centre_lines(length, step, tape_w=0.0, border=DEFAULT_BORDER):
    """Metric position of each tape CENTRELINE along one axis."""
    line_positions_m = axis_lines(length, step).astype(float)
    if border == "inside" and len(line_positions_m) >= 2 and tape_w > 0:
        line_positions_m[0] += tape_w / 2.0
        line_positions_m[-1] -= tape_w / 2.0
    elif border not in BORDERS:
        raise ValueError(f"border must be one of {BORDERS}")
    return line_positions_m


# Every expected tape-centreline crossing in metres, row by row from the origin.
def intersections(dims, spacing, tape_w=0.0, border="centre"):
    """All tape-centreline crossings, row by row from the origin -> (N, 2) metres.
    border="inside" + the tape width moves the border lines in by half a tape."""
    x_lines_m = centre_lines(dims[0], spacing[0], tape_w, border)
    y_lines_m = centre_lines(dims[1], spacing[1], tape_w, border)
    return np.array([(x_m, y_m) for y_m in y_lines_m for x_m in x_lines_m], float)


# The four outer corners (BL, BR, TR, TL) in metres plus click labels.
def outer_corners(dims):
    """BL, BR, TR, TL with labels - the default click anchors."""
    width_m, height_m = dims
    corners_m = np.array([[0, 0], [width_m, 0], [width_m, height_m], [0, height_m]], float)
    labels = ["BL origin (0,0)", f"BR ({width_m:g},0)", f"TR ({width_m:g},{height_m:g})", f"TL (0,{height_m:g})"]
    return corners_m, labels


# Apply a 3x3 homography to N (x, y) points -> N x 2 array.
def transform(H, pts):
    points_array = np.asarray(pts, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(points_array, np.asarray(H, np.float64)).reshape(-1, 2)


# Scale a homography so its bottom-right element is 1 (homographies are defined only
# up to scale; this makes saved matrices comparable).
def normalized(H):
    H = np.asarray(H, float)
    return H / H[2, 2]


# ---------------------------------------------------------------- detection
# Finds tape crossings in a 'rectified' top-down image: the camera frame is warped by
# the predicted image->metre homography into a metric map (ppm pixels per metre, pad_m
# border), where tape lines run straight along rows/columns.
class TapeGrid:
    """Locate tape intersections near their predicted positions (H_img2m prediction)."""

    # Choose the rectified image scale (<= ppm_max px/m, <= max_px wide) and build A, the
    # metres -> rectified-pixel matrix (+Y up, so rows are flipped).
    def __init__(self, frame, H_img2m, dims, spacing, pad_m=0.25, max_px=2400, ppm_max=400.0):
        self.frame, self.dims, self.spacing = frame, tuple(dims), tuple(spacing)
        width_m, height_m = self.dims
        self.pad = pad_m
        self.ppm = min(ppm_max, max_px / (max(width_m, height_m) + 2 * pad_m))
        # metric -> rectified pixels, +Y up (rows grow downwards)
        self.A = np.array([[self.ppm, 0, pad_m * self.ppm],
                           [0, -self.ppm, (height_m + pad_m) * self.ppm], [0, 0, 1]])
        self.size = (int(round((width_m + 2 * pad_m) * self.ppm)), int(round((height_m + 2 * pad_m) * self.ppm)))
        self.set_prediction(H_img2m)

    # Warp the frame with the current prediction, learn the floor colour (cell centres)
    # and the tape colour (along the predicted lines) in Lab space, then score every pixel
    # by how far it lies from floor towards tape (0 = floor, 1 = tape). Pixels off that
    # colour axis (rover, shadows) score 0. mask = fraction > 0.35.
    def set_prediction(self, H_img2m):
        self.H_img2m = normalized(H_img2m)
        self.H_rect = self.A @ self.H_img2m                     # image px -> rectified px
        self.H_rect_inv = np.linalg.inv(self.H_rect)
        rectified = cv2.warpPerspective(self.frame, self.H_rect, self.size, flags=cv2.INTER_LINEAR)
        valid_mask = cv2.warpPerspective(np.full(self.frame.shape[:2], 255, np.uint8), self.H_rect,
                                    self.size, flags=cv2.INTER_NEAREST)
        valid_mask = cv2.erode(valid_mask, np.ones((5, 5), np.uint8))
        self.valid = valid_mask
        lab_image = cv2.cvtColor(rectified, cv2.COLOR_BGR2LAB).astype(np.float32)
        cell_centres_m = [((cell_x0 + cell_x1) / 2, (cell_y0 + cell_y1) / 2)
                 for cell_x0, cell_x1 in zip(*_pairs(axis_lines(self.dims[0], self.spacing[0])))
                 for cell_y0, cell_y1 in zip(*_pairs(axis_lines(self.dims[1], self.spacing[1])))]
        floor_lab = self._median(lab_image, valid_mask, cell_centres_m, max(2, int(0.08 * self.ppm)))
        tape_lab = None if floor_lab is None else self._tape_colour(lab_image, valid_mask, floor_lab)
        if tape_lab is None or floor_lab is None:
            raise ValueError("grid is not inside the camera view - check the clicked anchors")
        self.tape_lab, self.floor_lab = tape_lab, floor_lab
        colour_axis = (tape_lab - floor_lab) * LAB_WEIGHTS
        self.separation = float(np.linalg.norm(colour_axis))
        if self.separation < 10.0:
            raise ValueError("tape and floor colours are too similar to detect the grid")
        # Tape fraction per pixel: position along the floor->tape colour axis (0 floor, 1 tape).
        # Soft, so thin tape blended with the floor still has a symmetric profile whose
        # weighted centre is the tape centreline; colours far off that axis (rover, shadows)
        # are not tape at all.
        colour_offset = (lab_image - floor_lab) * LAB_WEIGHTS
        tape_fraction = (colour_offset @ colour_axis) / self.separation ** 2
        off_axis_distance = np.linalg.norm(colour_offset - tape_fraction[..., None] * colour_axis, axis=2)
        tape_fraction[(off_axis_distance > 0.6 * self.separation) | (valid_mask == 0)] = 0.0
        self.frac = np.clip(tape_fraction, 0.0, 1.0).astype(np.float32)
        self.mask = (self.frac > 0.35).astype(np.uint8)
        self.tape_width_m = self._tape_width()

    # Median Lab colour of (2r+1)^2 patches around the given metric points (inside view).
    def _median(self, lab, valid, pts_m, r):
        patches = []
        for col, row in np.int32(np.round(self.m2r(pts_m))):
            if r <= col < lab.shape[1] - r and r <= row < lab.shape[0] - r and valid[row, col]:
                patches.append(lab[row - r:row + r + 1, col - r:col + r + 1].reshape(-1, 3))
        return np.median(np.vstack(patches), axis=0) if patches else None

    # Tape colour estimate: draw the predicted tape lines 4 cm wide, take the 20 % of
    # pixels under them that differ most from the floor, return their median colour.
    def _tape_colour(self, lab, valid, floor):
        """Colour of the most tape-like pixels along the predicted tape lines.

        Robust to thin/blurred tape (a crossing patch would be half floor) and to a rough
        first prediction: the lines are drawn 4 cm wide, then the 20 % of pixels that
        differ most from the floor are taken as tape."""
        line_band_mask = np.zeros(valid.shape, np.uint8)
        line_thickness_px = max(2, int(round(0.04 * self.ppm)))
        width_m, height_m = self.dims
        for x_m in axis_lines(width_m, self.spacing[0]):
            line_start, line_end = np.int32(np.round(self.m2r([(x_m, 0), (x_m, height_m)])))
            cv2.line(line_band_mask, tuple(line_start), tuple(line_end), 1, line_thickness_px)
        for y_m in axis_lines(height_m, self.spacing[1]):
            line_start, line_end = np.int32(np.round(self.m2r([(0, y_m), (width_m, y_m)])))
            cv2.line(line_band_mask, tuple(line_start), tuple(line_end), 1, line_thickness_px)
        colour_offset = (lab[(line_band_mask > 0) & (valid > 0)] - floor) * LAB_WEIGHTS
        if len(colour_offset) < 50:
            return None
        offset_magnitude = np.linalg.norm(colour_offset, axis=1)
        most_tape_like = colour_offset[offset_magnitude >= np.percentile(offset_magnitude, 80)]
        return floor + np.median(most_tape_like, axis=0) / LAB_WEIGHTS

    # Metres -> rectified-image pixels.
    def m2r(self, pts_m):
        return transform(self.A, pts_m)

    # Rectified-image pixels -> metres.
    def r2m(self, pts_r):
        return transform(np.linalg.inv(self.A), pts_r)

    # Measured tape width in metres: across the middle of every grid segment, sum the soft
    # tape-fraction profile (= width in px for a blurred edge), median over all segments.
    def _tape_width(self):
        """Median tape width (m) across the middle of each grid segment."""
        widths_px = []
        x_lines_m = axis_lines(self.dims[0], self.spacing[0])
        y_lines_m = axis_lines(self.dims[1], self.spacing[1])
        half_window_px = int(0.08 * self.ppm)

        def width(mask_line, frac_line):
            # area under the soft tape-fraction profile around the run: unbiased for
            # blurred edges (counting thresholded pixels read ~1 px = 2.5 mm wide, and the
            # border-tape inset is half of this width)
            tape_run = _run(mask_line)
            if not tape_run:
                return None
            start_index, end_index = max(0, tape_run[0] - 3), min(len(frac_line), tape_run[1] + 4)
            return float(np.sum(frac_line[start_index:end_index]))

        for x_m in x_lines_m:
            for seg_y0, seg_y1 in zip(*_pairs(y_lines_m)):
                col, row = np.int32(np.round(self.m2r([(x_m, (seg_y0 + seg_y1) / 2)])[0]))
                if 0 <= row < self.mask.shape[0] and half_window_px <= col < self.mask.shape[1] - half_window_px:
                    width_px = width(self.mask[row, col - half_window_px:col + half_window_px + 1], self.frac[row, col - half_window_px:col + half_window_px + 1])
                    if width_px:
                        widths_px.append(width_px)
        for y_m in y_lines_m:
            for seg_x0, seg_x1 in zip(*_pairs(x_lines_m)):
                col, row = np.int32(np.round(self.m2r([((seg_x0 + seg_x1) / 2, y_m)])[0]))
                if 0 <= col < self.mask.shape[1] and half_window_px <= row < self.mask.shape[0] - half_window_px:
                    width_px = width(self.mask[row - half_window_px:row + half_window_px + 1, col], self.frac[row - half_window_px:row + half_window_px + 1, col])
                    if width_px:
                        widths_px.append(width_px)
        return float(np.median(widths_px) / self.ppm) if len(widths_px) >= 2 else 0.05

    # Fit one tape's centreline near a crossing. Scan perpendicular lines at offsets
    # |d| >= excl from the crossing (the crossing patch itself is ambiguous), find the tape
    # run on each, take its fraction-weighted centre, then least-squares fit
    # across = a + b * (t - along) with one round of outlier rejection.
    # Returns ((a, b), rms in px, number of scanlines) or None.
    def _centreline(self, along, across, R, band, excl, vertical):
        """Fit the tape centreline across = a + b*(t - along) on scanlines |t-along| >= excl."""
        mask = self.mask
        samples = []
        for offset in range(-R, R + 1):
            if abs(offset) < excl:
                continue
            scan_index = int(round(along)) + offset
            band_lo, band_hi = int(round(across - band)), int(round(across + band)) + 1
            if vertical:
                if not (0 <= scan_index < mask.shape[0] and band_lo >= 0 and band_hi <= mask.shape[1]):
                    continue
                mask_line, valid_line, fraction_line = mask[scan_index, band_lo:band_hi], self.valid[scan_index, band_lo:band_hi], self.frac[scan_index, band_lo:band_hi]
            else:
                if not (0 <= scan_index < mask.shape[1] and band_lo >= 0 and band_hi <= mask.shape[0]):
                    continue
                mask_line, valid_line, fraction_line = mask[band_lo:band_hi, scan_index], self.valid[band_lo:band_hi, scan_index], self.frac[band_lo:band_hi, scan_index]
            tape_run = _run(mask_line)
            if tape_run is None or tape_run[0] <= 1 or tape_run[1] >= len(mask_line) - 2:
                continue                      # touches the band edge: not an isolated tape strip
            if not (valid_line[tape_run[0] - 2] and valid_line[tape_run[1] + 2]):
                continue                      # cut by the image border: centre would be biased
            indices = np.arange(tape_run[0] - 1, tape_run[1] + 2)
            weights = fraction_line[indices]
            samples.append((scan_index - along, band_lo + float((indices * weights).sum() / weights.sum()), tape_run[1] - tape_run[0] + 1))
        if len(samples) < 6:
            return None
        sample_array = np.array(samples, float)
        design_matrix = np.column_stack([np.ones(len(sample_array)), sample_array[:, 0]])
        line_coef = np.linalg.lstsq(design_matrix, sample_array[:, 1], rcond=None)[0]
        residuals = sample_array[:, 1] - design_matrix @ line_coef
        inliers = np.abs(residuals) <= max(1.5, 2.5 * float(np.std(residuals)))
        if inliers.sum() < 6:
            return None
        line_coef = np.linalg.lstsq(design_matrix[inliers], sample_array[inliers, 1], rcond=None)[0]
        rms_px = float(np.sqrt(np.mean((sample_array[inliers, 1] - design_matrix[inliers] @ line_coef) ** 2)))
        return line_coef, rms_px, int(inliers.sum())

    # Locate one crossing precisely: intersect the vertical and horizontal tape
    # centrelines (2x2 linear solve), re-centre and repeat up to 4 times. Rejects the
    # point if it moved > max_shift_m from the prediction or the fit is ragged (> 6 mm).
    # Returns (image pixel, info) or (None, reason).
    def refine(self, pm, max_shift_m=0.06):
        """-> (pixel (2,), info dict) or (None, reason)."""
        px_per_m = self.ppm
        col, row = self.m2r([pm])[0]
        col_start, row_start = col, row
        tape_width = self.tape_width_m
        exclude_px = int(round(max(0.035, 0.75 * tape_width + 0.012) * px_per_m))
        reach_px = int(round(min(0.42 * min(self.spacing), 0.12) * px_per_m))
        if reach_px - exclude_px < int(0.025 * px_per_m):
            reach_px = exclude_px + int(0.025 * px_per_m)
        band_px = int(round(max(0.07, 1.3 * tape_width) * px_per_m))
        fit_info = {}
        for _ in range(4):
            vertical_fit = self._centreline(row, col, reach_px, band_px, exclude_px, vertical=True)    # x = a + b (y - v)
            horizontal_fit = self._centreline(col, row, reach_px, band_px, exclude_px, vertical=False)   # y = a + b (x - u)
            if vertical_fit is None or horizontal_fit is None:
                return None, "tape not found (occluded / outside view / wrong colour)"
            (v_offset, v_slope), v_rms, v_count = vertical_fit
            (h_offset, h_slope), h_rms, h_count = horizontal_fit
            system = np.array([[1.0, -v_slope], [-h_slope, 1.0]])
            new_col, new_row = np.linalg.solve(system, [v_offset - v_slope * row, h_offset - h_slope * col])
            step_px = math.hypot(new_col - col, new_row - row)
            col, row = new_col, new_row
            band_px = int(round(max(0.04, 1.0 * tape_width) * px_per_m))
            fit_info = dict(fit_rms_m=max(v_rms, h_rms) / px_per_m, scanlines=(v_count, h_count))
            if step_px < 0.05:
                break
        shift_m = math.hypot(col - col_start, row - row_start) / px_per_m
        if shift_m > max_shift_m:
            return None, f"snapped {shift_m * 100:.1f} cm from prediction (> {max_shift_m * 100:.0f} cm)"
        if fit_info["fit_rms_m"] > 0.006:
            return None, f"ragged tape edge (fit {fit_info['fit_rms_m'] * 1000:.1f} mm)"
        fit_info["shift_m"] = shift_m
        return transform(self.H_rect_inv, [(col, row)])[0], fit_info


# Consecutive pairs (v[i], v[i+1]) as two arrays - the start/end of each grid segment.
def _pairs(vals):
    return vals[:-1], vals[1:]


# (first, last) index of the longest run of non-zero values (>= 3 long), or None.
def _run(line):
    """(first, last) index of the longest run of nonzero values, or None."""
    if line is None or len(line) == 0:
        return None
    nonzero_index = np.flatnonzero(line)
    if len(nonzero_index) < 3:
        return None
    runs = np.split(nonzero_index, np.flatnonzero(np.diff(nonzero_index) > 1) + 1)
    longest_run = max(runs, key=len)
    return (int(longest_run[0]), int(longest_run[-1])) if len(longest_run) >= 3 else None


# Detect every crossing; after each round re-fit a homography from the found
# points and re-predict, so a rough 4-click start converges. Returns the TapeGrid and
# a list of {m: metres, px: pixel or None, info, why} per crossing.
def detect(frame, H_img2m, dims, spacing, max_shift_m=0.06, rounds=2, border=DEFAULT_BORDER,
           tape_width=TAPE_WIDTH_M):
    """Detect every intersection; re-predict from the improving fit each round.
    Each detection's "m" is where that tape-centreline crossing really is (border tape
    inset by half its measured width for border="inside").

    -> (TapeGrid, list of dict(m, px|None, info|None, why|None)).
    """
    tape_grid = TapeGrid(frame, H_img2m, dims, spacing)
    tape_grid.border = border
    detections = []
    for round_index in range(rounds):
        tape_grid.tape_width_measured_m = tape_grid.tape_width_m
        tape_width_m = tape_width if tape_width else tape_grid.tape_width_m
        crossings_m = intersections(dims, spacing, tape_width_m, border)
        tape_grid.tape_width_used_m = tape_width_m
        detections = []
        for crossing_m in crossings_m:
            crossing_px, fit_info = tape_grid.refine(crossing_m, max_shift_m)
            detections.append(dict(m=crossing_m, px=crossing_px, info=fit_info if crossing_px is not None else None,
                              why=None if crossing_px is not None else fit_info))
        detected = [detection for detection in detections if detection["px"] is not None]
        if round_index + 1 < rounds and len(detected) >= 4:
            refit_H, _ = cv2.findHomography(np.array([detection["px"] for detection in detected]),
                                      np.array([detection["m"] for detection in detected]), 0)
            if refit_H is None:
                break
            tape_grid.set_prediction(refit_H)
    return tape_grid, detections


# ---------------------------------------------------------------- solve + stats
# Least-squares homography image -> metres from all points, dropping the worst point
# while it is > max(reject_m, 4 x median residual) (keeps at least 6).
# Returns (H_img2m, used mask, residual in metres per point).
def solve(points_px, points_m, reject_m=0.015):
    """Least-squares homography image->metric, dropping gross outliers one at a time.

    -> (H_img2m, used mask, residual metres per point).
    """
    px_array, m_array = np.asarray(points_px, float), np.asarray(points_m, float)
    if len(px_array) < 4:
        raise ValueError("need at least 4 points for a homography")
    used = np.ones(len(px_array), bool)
    while True:
        img_to_m, _ = cv2.findHomography(px_array[used], m_array[used], 0)
        if img_to_m is None:
            raise ValueError("degenerate points (collinear?) - homography failed")
        residuals_m = np.linalg.norm(transform(img_to_m, px_array) - m_array, axis=1)
        if used.sum() <= 6:
            break
        worst_index = int(np.argmax(np.where(used, residuals_m, -1)))
        median_residual = float(np.median(residuals_m[used]))
        if residuals_m[worst_index] > max(reject_m, 4 * median_residual):
            used[worst_index] = False
            continue
        break
    return normalized(img_to_m), used, residuals_m


# Count, RMS and max of the residuals (only the used points if a mask is given).
def residual_stats(res_m, used=None):
    residuals = np.asarray(res_m, float)
    if used is not None:
        residuals = residuals[np.asarray(used, bool)]
    if not len(residuals):
        return dict(n=0, rms_m=float("nan"), max_m=float("nan"))
    return dict(n=int(len(residuals)), rms_m=float(np.sqrt(np.mean(residuals ** 2))), max_m=float(residuals.max()))


# Largest distance (m) between two calibrations over the grid: map a 5 cm lattice
# through B to pixels and back through A.
def area_disagreement(H_a_img2m, H_b_m2img, dims, step=0.05):
    """Max metric distance between two calibrations over the grid area.

    Maps grid points through calibration B to pixels and back through A."""
    width_m, height_m = dims
    grid_points_m = np.array([(x_m, y_m) for x_m in np.linspace(0, width_m, int(round(width_m / step)) + 1)
                  for y_m in np.linspace(0, height_m, int(round(height_m / step)) + 1)])
    round_trip_m = transform(H_a_img2m, transform(H_b_m2img, grid_points_m))
    return float(np.max(np.linalg.norm(round_trip_m - grid_points_m, axis=1)))


# ---------------------------------------------------------------- drawing
# Overlay predicted tape lines, found crossings (green; orange = rejected in the
# fit), misses (red X) and residual arrows magnified arrow_gain times.
def draw_detections(img, found, H_img2m, dims, spacing, res_m=None, used=None, arrow_gain=10.0):
    """Tape-line prediction from H, detected crossings (green), misses (red X),
    residual arrows (x arrow_gain) for points used in the fit."""
    H_m2img = np.linalg.inv(H_img2m)
    x_lines_m, y_lines_m = axis_lines(dims[0], spacing[0]), axis_lines(dims[1], spacing[1])
    for x_m in x_lines_m:
        line_start_px, line_end_px = transform(H_m2img, [(x_m, 0), (x_m, dims[1])])
        cv2.line(img, tuple(np.int32(line_start_px)), tuple(np.int32(line_end_px)), (0, 200, 255), 1, cv2.LINE_AA)
    for y_m in y_lines_m:
        line_start_px, line_end_px = transform(H_m2img, [(0, y_m), (dims[0], y_m)])
        cv2.line(img, tuple(np.int32(line_start_px)), tuple(np.int32(line_end_px)), (0, 200, 255), 1, cv2.LINE_AA)
    used_index = 0
    for detection in found:
        if detection["px"] is None:
            point_px = transform(H_m2img, [detection["m"]])[0]
            cv2.drawMarker(img, tuple(np.int32(point_px)), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
            continue
        point_px = np.asarray(detection["px"])
        was_used = used is None or used[used_index]
        colour = (0, 255, 0) if was_used else (0, 140, 255)
        cv2.circle(img, tuple(np.int32(np.round(point_px))), 4, colour, -1, cv2.LINE_AA)
        if res_m is not None:
            ideal_px = transform(H_m2img, [detection["m"]])[0]
            arrow_tip_px = ideal_px + (point_px - ideal_px) * arrow_gain
            cv2.arrowedLine(img, tuple(np.int32(ideal_px)), tuple(np.int32(arrow_tip_px)), (255, 0, 255), 1,
                            cv2.LINE_AA, tipLength=0.25)
        used_index += 1
    return img
