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
def parse_size(text):
    """'2x2' | '1x4' | '1.5' -> (width_x_m, height_y_m)."""
    parts = [p for p in re.split(r"[x,X*×]", str(text).strip()) if p]
    vals = [float(p) for p in parts]
    if len(vals) == 1:
        vals = vals * 2
    if len(vals) != 2 or not all(math.isfinite(v) and v > 0 for v in vals):
        raise ValueError(f"grid size must look like 2x2 or 1x4 (metres), got {text!r}")
    return float(vals[0]), float(vals[1])


def parse_spacing(text, dims):
    """'0.5' | '0.5,0.25' | None (outer edges only) -> (sx, sy)."""
    if text is None:
        return float(dims[0]), float(dims[1])
    vals = [float(v) for v in re.split(r"[x,X]", str(text).strip()) if v]
    if len(vals) == 1:
        vals = vals * 2
    if len(vals) != 2 or not all(math.isfinite(v) and v > 0 for v in vals):
        raise ValueError(f"spacing must look like 0.5 or 0.5,0.25 (metres), got {text!r}")
    axis_lines(dims[0], vals[0])
    axis_lines(dims[1], vals[1])
    return float(vals[0]), float(vals[1])


def axis_lines(length, step):
    n = int(round(length / step))
    if n < 1 or abs(n * step - length) > 1e-6 * max(1.0, length):
        raise ValueError(f"tape spacing {step:g} m does not divide the {length:g} m side")
    return np.linspace(0.0, length, n + 1)


def grid_dims(calib):
    """(width_x_m, height_y_m) from a grid_calib dict, old square files included."""
    if "grid_width_m" in calib and "grid_height_m" in calib:
        return float(calib["grid_width_m"]), float(calib["grid_height_m"])
    size = calib.get("grid_size_m", 1.0)
    if isinstance(size, (list, tuple)):
        return float(size[0]), float(size[1])
    return float(size), float(size)


def as_dims(grid_m):
    """Accept the legacy scalar grid size or a (w, h) pair."""
    if isinstance(grid_m, (list, tuple, np.ndarray)):
        return float(grid_m[0]), float(grid_m[1])
    return float(grid_m), float(grid_m)


def grid_spacing(calib):
    w, h = grid_dims(calib)
    s = calib.get("grid_spacing_m")
    return (float(s[0]), float(s[1])) if s else (w, h)


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


def centre_lines(length, step, tape_w=0.0, border=DEFAULT_BORDER):
    """Metric position of each tape CENTRELINE along one axis."""
    xs = axis_lines(length, step).astype(float)
    if border == "inside" and len(xs) >= 2 and tape_w > 0:
        xs[0] += tape_w / 2.0
        xs[-1] -= tape_w / 2.0
    elif border not in BORDERS:
        raise ValueError(f"border must be one of {BORDERS}")
    return xs


def intersections(dims, spacing, tape_w=0.0, border="centre"):
    """All tape-centreline crossings, row by row from the origin -> (N, 2) metres.
    border="inside" + the tape width moves the border lines in by half a tape."""
    xs = centre_lines(dims[0], spacing[0], tape_w, border)
    ys = centre_lines(dims[1], spacing[1], tape_w, border)
    return np.array([(x, y) for y in ys for x in xs], float)


def outer_corners(dims):
    """BL, BR, TR, TL with labels - the default click anchors."""
    w, h = dims
    pts = np.array([[0, 0], [w, 0], [w, h], [0, h]], float)
    names = ["BL origin (0,0)", f"BR ({w:g},0)", f"TR ({w:g},{h:g})", f"TL (0,{h:g})"]
    return pts, names


def transform(H, pts):
    p = np.asarray(pts, np.float64).reshape(-1, 1, 2)
    return cv2.perspectiveTransform(p, np.asarray(H, np.float64)).reshape(-1, 2)


def normalized(H):
    H = np.asarray(H, float)
    return H / H[2, 2]


# ---------------------------------------------------------------- detection
class TapeGrid:
    """Locate tape intersections near their predicted positions (H_img2m prediction)."""

    def __init__(self, frame, H_img2m, dims, spacing, pad_m=0.25, max_px=2400, ppm_max=400.0):
        self.frame, self.dims, self.spacing = frame, tuple(dims), tuple(spacing)
        w, h = self.dims
        self.pad = pad_m
        self.ppm = min(ppm_max, max_px / (max(w, h) + 2 * pad_m))
        # metric -> rectified pixels, +Y up (rows grow downwards)
        self.A = np.array([[self.ppm, 0, pad_m * self.ppm],
                           [0, -self.ppm, (h + pad_m) * self.ppm], [0, 0, 1]])
        self.size = (int(round((w + 2 * pad_m) * self.ppm)), int(round((h + 2 * pad_m) * self.ppm)))
        self.set_prediction(H_img2m)

    def set_prediction(self, H_img2m):
        self.H_img2m = normalized(H_img2m)
        self.H_rect = self.A @ self.H_img2m                     # image px -> rectified px
        self.H_rect_inv = np.linalg.inv(self.H_rect)
        warp = cv2.warpPerspective(self.frame, self.H_rect, self.size, flags=cv2.INTER_LINEAR)
        valid = cv2.warpPerspective(np.full(self.frame.shape[:2], 255, np.uint8), self.H_rect,
                                    self.size, flags=cv2.INTER_NEAREST)
        valid = cv2.erode(valid, np.ones((5, 5), np.uint8))
        self.valid = valid
        lab = cv2.cvtColor(warp, cv2.COLOR_BGR2LAB).astype(np.float32)
        cells = [((x0 + x1) / 2, (y0 + y1) / 2)
                 for x0, x1 in zip(*_pairs(axis_lines(self.dims[0], self.spacing[0])))
                 for y0, y1 in zip(*_pairs(axis_lines(self.dims[1], self.spacing[1])))]
        floor = self._median(lab, valid, cells, max(2, int(0.08 * self.ppm)))
        tape = None if floor is None else self._tape_colour(lab, valid, floor)
        if tape is None or floor is None:
            raise ValueError("grid is not inside the camera view - check the clicked anchors")
        self.tape_lab, self.floor_lab = tape, floor
        axis = (tape - floor) * LAB_WEIGHTS
        self.separation = float(np.linalg.norm(axis))
        if self.separation < 10.0:
            raise ValueError("tape and floor colours are too similar to detect the grid")
        # Tape fraction per pixel: position along the floor->tape colour axis (0 floor, 1 tape).
        # Soft, so thin tape blended with the floor still has a symmetric profile whose
        # weighted centre is the tape centreline; colours far off that axis (rover, shadows)
        # are not tape at all.
        d = (lab - floor) * LAB_WEIGHTS
        frac = (d @ axis) / self.separation ** 2
        off = np.linalg.norm(d - frac[..., None] * axis, axis=2)
        frac[(off > 0.6 * self.separation) | (valid == 0)] = 0.0
        self.frac = np.clip(frac, 0.0, 1.0).astype(np.float32)
        self.mask = (self.frac > 0.35).astype(np.uint8)
        self.tape_width_m = self._tape_width()

    def _median(self, lab, valid, pts_m, r):
        samples = []
        for u, v in np.int32(np.round(self.m2r(pts_m))):
            if r <= u < lab.shape[1] - r and r <= v < lab.shape[0] - r and valid[v, u]:
                samples.append(lab[v - r:v + r + 1, u - r:u + r + 1].reshape(-1, 3))
        return np.median(np.vstack(samples), axis=0) if samples else None

    def _tape_colour(self, lab, valid, floor):
        """Colour of the most tape-like pixels along the predicted tape lines.

        Robust to thin/blurred tape (a crossing patch would be half floor) and to a rough
        first prediction: the lines are drawn 4 cm wide, then the 20 % of pixels that
        differ most from the floor are taken as tape."""
        band = np.zeros(valid.shape, np.uint8)
        th = max(2, int(round(0.04 * self.ppm)))
        w, h = self.dims
        for x in axis_lines(w, self.spacing[0]):
            a, b = np.int32(np.round(self.m2r([(x, 0), (x, h)])))
            cv2.line(band, tuple(a), tuple(b), 1, th)
        for y in axis_lines(h, self.spacing[1]):
            a, b = np.int32(np.round(self.m2r([(0, y), (w, y)])))
            cv2.line(band, tuple(a), tuple(b), 1, th)
        d = (lab[(band > 0) & (valid > 0)] - floor) * LAB_WEIGHTS
        if len(d) < 50:
            return None
        mag = np.linalg.norm(d, axis=1)
        top = d[mag >= np.percentile(mag, 80)]
        return floor + np.median(top, axis=0) / LAB_WEIGHTS

    def m2r(self, pts_m):
        return transform(self.A, pts_m)

    def r2m(self, pts_r):
        return transform(np.linalg.inv(self.A), pts_r)

    def _tape_width(self):
        """Median tape width (m) across the middle of each grid segment."""
        widths = []
        xs = axis_lines(self.dims[0], self.spacing[0])
        ys = axis_lines(self.dims[1], self.spacing[1])
        band = int(0.08 * self.ppm)

        def width(mask_line, frac_line):
            # area under the soft tape-fraction profile around the run: unbiased for
            # blurred edges (counting thresholded pixels read ~1 px = 2.5 mm wide, and the
            # border-tape inset is half of this width)
            run = _run(mask_line)
            if not run:
                return None
            a, b = max(0, run[0] - 3), min(len(frac_line), run[1] + 4)
            return float(np.sum(frac_line[a:b]))

        for x in xs:
            for y0, y1 in zip(*_pairs(ys)):
                u, v = np.int32(np.round(self.m2r([(x, (y0 + y1) / 2)])[0]))
                if 0 <= v < self.mask.shape[0] and band <= u < self.mask.shape[1] - band:
                    wpx = width(self.mask[v, u - band:u + band + 1], self.frac[v, u - band:u + band + 1])
                    if wpx:
                        widths.append(wpx)
        for y in ys:
            for x0, x1 in zip(*_pairs(xs)):
                u, v = np.int32(np.round(self.m2r([((x0 + x1) / 2, y)])[0]))
                if 0 <= u < self.mask.shape[1] and band <= v < self.mask.shape[0] - band:
                    wpx = width(self.mask[v - band:v + band + 1, u], self.frac[v - band:v + band + 1, u])
                    if wpx:
                        widths.append(wpx)
        return float(np.median(widths) / self.ppm) if len(widths) >= 2 else 0.05

    def _centreline(self, along, across, R, band, excl, vertical):
        """Fit the tape centreline across = a + b*(t - along) on scanlines |t-along| >= excl."""
        mask = self.mask
        pts = []
        for d in range(-R, R + 1):
            if abs(d) < excl:
                continue
            t = int(round(along)) + d
            lo, hi = int(round(across - band)), int(round(across + band)) + 1
            if vertical:
                if not (0 <= t < mask.shape[0] and lo >= 0 and hi <= mask.shape[1]):
                    continue
                line, seen, frac = mask[t, lo:hi], self.valid[t, lo:hi], self.frac[t, lo:hi]
            else:
                if not (0 <= t < mask.shape[1] and lo >= 0 and hi <= mask.shape[0]):
                    continue
                line, seen, frac = mask[lo:hi, t], self.valid[lo:hi, t], self.frac[lo:hi, t]
            run = _run(line)
            if run is None or run[0] <= 1 or run[1] >= len(line) - 2:
                continue                      # touches the band edge: not an isolated tape strip
            if not (seen[run[0] - 2] and seen[run[1] + 2]):
                continue                      # cut by the image border: centre would be biased
            k = np.arange(run[0] - 1, run[1] + 2)
            wgt = frac[k]
            pts.append((t - along, lo + float((k * wgt).sum() / wgt.sum()), run[1] - run[0] + 1))
        if len(pts) < 6:
            return None
        p = np.array(pts, float)
        A = np.column_stack([np.ones(len(p)), p[:, 0]])
        coef = np.linalg.lstsq(A, p[:, 1], rcond=None)[0]
        res = p[:, 1] - A @ coef
        keep = np.abs(res) <= max(1.5, 2.5 * float(np.std(res)))
        if keep.sum() < 6:
            return None
        coef = np.linalg.lstsq(A[keep], p[keep, 1], rcond=None)[0]
        rms = float(np.sqrt(np.mean((p[keep, 1] - A[keep] @ coef) ** 2)))
        return coef, rms, int(keep.sum())

    def refine(self, pm, max_shift_m=0.06):
        """-> (pixel (2,), info dict) or (None, reason)."""
        ppm = self.ppm
        u, v = self.m2r([pm])[0]
        u0, v0 = u, v
        tw = self.tape_width_m
        excl = int(round(max(0.035, 0.75 * tw + 0.012) * ppm))
        R = int(round(min(0.42 * min(self.spacing), 0.12) * ppm))
        if R - excl < int(0.025 * ppm):
            R = excl + int(0.025 * ppm)
        band = int(round(max(0.07, 1.3 * tw) * ppm))
        info = {}
        for _ in range(4):
            vf = self._centreline(v, u, R, band, excl, vertical=True)    # x = a + b (y - v)
            hf = self._centreline(u, v, R, band, excl, vertical=False)   # y = a + b (x - u)
            if vf is None or hf is None:
                return None, "tape not found (occluded / outside view / wrong colour)"
            (a1, b1), r1, n1 = vf
            (a2, b2), r2, n2 = hf
            M = np.array([[1.0, -b1], [-b2, 1.0]])
            nu, nv = np.linalg.solve(M, [a1 - b1 * v, a2 - b2 * u])
            step = math.hypot(nu - u, nv - v)
            u, v = nu, nv
            band = int(round(max(0.04, 1.0 * tw) * ppm))
            info = dict(fit_rms_m=max(r1, r2) / ppm, scanlines=(n1, n2))
            if step < 0.05:
                break
        shift = math.hypot(u - u0, v - v0) / ppm
        if shift > max_shift_m:
            return None, f"snapped {shift * 100:.1f} cm from prediction (> {max_shift_m * 100:.0f} cm)"
        if info["fit_rms_m"] > 0.006:
            return None, f"ragged tape edge (fit {info['fit_rms_m'] * 1000:.1f} mm)"
        info["shift_m"] = shift
        return transform(self.H_rect_inv, [(u, v)])[0], info


def _pairs(vals):
    return vals[:-1], vals[1:]


def _run(line):
    """(first, last) index of the longest run of nonzero values, or None."""
    if line is None or len(line) == 0:
        return None
    idx = np.flatnonzero(line)
    if len(idx) < 3:
        return None
    runs = np.split(idx, np.flatnonzero(np.diff(idx) > 1) + 1)
    run = max(runs, key=len)
    return (int(run[0]), int(run[-1])) if len(run) >= 3 else None


def detect(frame, H_img2m, dims, spacing, max_shift_m=0.06, rounds=2, border=DEFAULT_BORDER,
           tape_width=TAPE_WIDTH_M):
    """Detect every intersection; re-predict from the improving fit each round.
    Each detection's "m" is where that tape-centreline crossing really is (border tape
    inset by half its measured width for border="inside").

    -> (TapeGrid, list of dict(m, px|None, info|None, why|None)).
    """
    tg = TapeGrid(frame, H_img2m, dims, spacing)
    tg.border = border
    found = []
    for rnd in range(rounds):
        tg.tape_width_measured_m = tg.tape_width_m
        tw = tape_width if tape_width else tg.tape_width_m
        pts = intersections(dims, spacing, tw, border)
        tg.tape_width_used_m = tw
        found = []
        for pm in pts:
            px, info = tg.refine(pm, max_shift_m)
            found.append(dict(m=pm, px=px, info=info if px is not None else None,
                              why=None if px is not None else info))
        ok = [f for f in found if f["px"] is not None]
        if rnd + 1 < rounds and len(ok) >= 4:
            H, _ = cv2.findHomography(np.array([f["px"] for f in ok]),
                                      np.array([f["m"] for f in ok]), 0)
            if H is None:
                break
            tg.set_prediction(H)
    return tg, found


# ---------------------------------------------------------------- solve + stats
def solve(points_px, points_m, reject_m=0.015):
    """Least-squares homography image->metric, dropping gross outliers one at a time.

    -> (H_img2m, used mask, residual metres per point).
    """
    px, m = np.asarray(points_px, float), np.asarray(points_m, float)
    if len(px) < 4:
        raise ValueError("need at least 4 points for a homography")
    used = np.ones(len(px), bool)
    while True:
        H, _ = cv2.findHomography(px[used], m[used], 0)
        if H is None:
            raise ValueError("degenerate points (collinear?) - homography failed")
        res = np.linalg.norm(transform(H, px) - m, axis=1)
        if used.sum() <= 6:
            break
        worst = int(np.argmax(np.where(used, res, -1)))
        typical = float(np.median(res[used]))
        if res[worst] > max(reject_m, 4 * typical):
            used[worst] = False
            continue
        break
    return normalized(H), used, res


def residual_stats(res_m, used=None):
    r = np.asarray(res_m, float)
    if used is not None:
        r = r[np.asarray(used, bool)]
    if not len(r):
        return dict(n=0, rms_m=float("nan"), max_m=float("nan"))
    return dict(n=int(len(r)), rms_m=float(np.sqrt(np.mean(r ** 2))), max_m=float(r.max()))


def area_disagreement(H_a_img2m, H_b_m2img, dims, step=0.05):
    """Max metric distance between two calibrations over the grid area.

    Maps grid points through calibration B to pixels and back through A."""
    w, h = dims
    g = np.array([(x, y) for x in np.linspace(0, w, int(round(w / step)) + 1)
                  for y in np.linspace(0, h, int(round(h / step)) + 1)])
    back = transform(H_a_img2m, transform(H_b_m2img, g))
    return float(np.max(np.linalg.norm(back - g, axis=1)))


# ---------------------------------------------------------------- drawing
def draw_detections(img, found, H_img2m, dims, spacing, res_m=None, used=None, arrow_gain=10.0):
    """Tape-line prediction from H, detected crossings (green), misses (red X),
    residual arrows (x arrow_gain) for points used in the fit."""
    H_m2img = np.linalg.inv(H_img2m)
    xs, ys = axis_lines(dims[0], spacing[0]), axis_lines(dims[1], spacing[1])
    for x in xs:
        a, b = transform(H_m2img, [(x, 0), (x, dims[1])])
        cv2.line(img, tuple(np.int32(a)), tuple(np.int32(b)), (0, 200, 255), 1, cv2.LINE_AA)
    for y in ys:
        a, b = transform(H_m2img, [(0, y), (dims[0], y)])
        cv2.line(img, tuple(np.int32(a)), tuple(np.int32(b)), (0, 200, 255), 1, cv2.LINE_AA)
    k = 0
    for f in found:
        if f["px"] is None:
            p = transform(H_m2img, [f["m"]])[0]
            cv2.drawMarker(img, tuple(np.int32(p)), (0, 0, 255), cv2.MARKER_TILTED_CROSS, 14, 2)
            continue
        p = np.asarray(f["px"])
        ok = used is None or used[k]
        col = (0, 255, 0) if ok else (0, 140, 255)
        cv2.circle(img, tuple(np.int32(np.round(p))), 4, col, -1, cv2.LINE_AA)
        if res_m is not None:
            ideal = transform(H_m2img, [f["m"]])[0]
            tip = ideal + (p - ideal) * arrow_gain
            cv2.arrowedLine(img, tuple(np.int32(ideal)), tuple(np.int32(tip)), (255, 0, 255), 1,
                            cv2.LINE_AA, tipLength=0.25)
        k += 1
    return img
