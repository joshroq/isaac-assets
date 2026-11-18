#!/usr/bin/env python3
"""2-D lidar scan matching for station fingerprints (camera-independent "same spot" check).

A *station* stores a reference LD19 scan taken at the taught start pose. Back at the
station, the current scan is aligned to the reference; the result is the rover's pose
error IN THE STATION'S OWN BODY FRAME (dx forward, dy left, dyaw CCW), i.e. exactly
what the final approach must undo. The room is static at the scale that matters (walls,
bed, dresser); moved clutter / people are rejected as outliers.

Method: point-to-line ICP (normals from the reference neighbourhood), trimmed (worst
residuals dropped), with a coarse rotation search seed so a few degrees of start error
cannot trap it in a local minimum. Quality is reported so callers can refuse a bad fit.

    from icp2d import scan_to_points, median_scan, match
    ref = scan_to_points(ranges, angle_min, angle_inc)
    res = match(ref_pts, cur_pts)   # {'dx','dy','dyaw','rms','inlier_frac','ok',...}
"""
from __future__ import annotations

import math
import warnings

import numpy as np

# RANGE_MIN also removes the rover's own hardware: the real LD19 sees parts of the rover
# at 3-9 cm (fixed in the body frame, they would pin the match at "no motion")
RANGE_MIN, RANGE_MAX = 0.20, 8.0
GRID_BINS = 720                      # common 0.5 deg grid all scans are resampled onto


def resample(ranges, angle_min, angle_inc, n_bins=GRID_BINS):
    """Put a scan on the fixed angular grid (angle_min 0, inc 2*pi/n_bins).
    The LD19 delivers 500-503 beams per turn with a varying increment, so beam INDEX is
    not a fixed angle; per-bin median is only meaningful after this."""
    r = np.asarray(ranges, float)
    ang = (angle_min + angle_inc * np.arange(len(r))) % (2 * math.pi)
    idx = np.round(ang / (2 * math.pi / n_bins)).astype(int) % n_bins
    out = np.full(n_bins, np.nan)
    ok = np.isfinite(r) & (r > 0)
    for i, v in zip(idx[ok], r[ok]):        # nearest beam per bin (keep the closer)
        if not np.isfinite(out[i]) or v < out[i]:
            out[i] = v
    return out, 0.0, 2 * math.pi / n_bins


def median_scan(scans):
    """Median per beam over several stationary scans (kills speckle + moving people).
    scans: list of 1-D range arrays of equal length (0/inf/nan = no return)."""
    a = np.array([np.where(np.isfinite(s) & (s > 0), s, np.nan) for s in scans], float)
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)    # bins with no return -> NaN
        med = np.nanmedian(a, axis=0)
        valid = np.sum(np.isfinite(a), axis=0) >= max(1, len(scans) // 2)
    med[~valid] = np.nan
    return med


def scan_to_points(ranges, angle_min, angle_inc, offset=(0.0, 0.0, 0.0), flip=False):
    """Ranges -> Nx2 points in the BODY frame (lidar mount offset x, y, yaw applied;
    flip=True for a scan that turns clockwise, i.e. mirrored)."""
    r = np.asarray(ranges, float)
    ang = angle_min + angle_inc * np.arange(len(r))
    ang = (-ang if flip else ang) + offset[2]
    ok = np.isfinite(r) & (r > RANGE_MIN) & (r < RANGE_MAX)
    return np.column_stack([offset[0] + r[ok] * np.cos(ang[ok]),
                            offset[1] + r[ok] * np.sin(ang[ok])])


def _rot(a):
    c, s = math.cos(a), math.sin(a)
    return np.array([[c, -s], [s, c]])


def _normals(ref, tree, k=6):
    _, idx = tree.query(ref, k=min(k, len(ref)))
    n = np.zeros_like(ref)
    planar = np.zeros(len(ref), bool)
    for i, nb in enumerate(idx):
        p = ref[nb] - ref[nb].mean(axis=0)
        w, v = np.linalg.eigh(p.T @ p)
        n[i] = v[:, 0]                       # smallest eigenvector = surface normal
        planar[i] = w[1] > 8 * max(w[0], 1e-12)
    return n, planar


def match(ref, cur, init=(0.0, 0.0, 0.0), max_iter=40, trim=0.8, max_corr=0.25,
          yaw_search_deg=10.0):
    """Pose of `cur` in `ref`'s frame: ref_point ~= R(dyaw) @ cur_point + (dx, dy).
    Since both scans are in the body frame, (dx, dy, dyaw) is the rover's displacement
    from the reference pose expressed in the reference body frame."""
    from scipy.spatial import cKDTree
    ref = np.asarray(ref, float)
    cur = np.asarray(cur, float)
    if len(ref) < 30 or len(cur) < 30:
        return {"ok": False, "why": f"too few points (ref {len(ref)}, cur {len(cur)})"}
    tree = cKDTree(ref)
    normals, planar = _normals(ref, tree)

    def residuals(x, y, a, gate):
        p = cur @ _rot(a).T + [x, y]
        d, j = tree.query(p, distance_upper_bound=gate)
        m = np.isfinite(d)
        return p, d, j, m

    # coarse yaw seed (cheap): pick the rotation with the most close correspondences
    x, y, a = init
    best = None
    for da in np.radians(np.arange(-yaw_search_deg, yaw_search_deg + 0.1, 1.0)):
        _, d, _, m = residuals(x, y, a + da, 0.08)
        score = m.sum() - (np.mean(d[m]) if m.any() else 0)
        if best is None or score > best[0]:
            best = (score, a + da)
    a = best[1]
    gate = max_corr
    for it in range(max_iter):
        p, d, j, m = residuals(x, y, a, gate)
        if m.sum() < 20:
            return {"ok": False, "why": "lost correspondences"}
        idx = np.nonzero(m)[0]
        jj = j[m]
        n = normals[jj]
        r = np.einsum("ij,ij->i", p[idx] - ref[jj], n)       # point-to-line residual
        keep = np.abs(r) <= np.quantile(np.abs(r), trim)        # trimmed
        keep &= planar[jj] | (np.abs(r) < 0.02)
        idx, jj, n, r = idx[keep], jj[keep], n[keep], r[keep]
        if len(idx) < 20:
            return {"ok": False, "why": "too few inliers"}
        q = cur[idx]
        # linearised: r + n . (dt + dA * J q') = 0, J = d/da R(a) q
        Rq = q @ _rot(a).T
        Jq = np.column_stack([-Rq[:, 1], Rq[:, 0]])
        A = np.column_stack([n[:, 0], n[:, 1], np.einsum("ij,ij->i", n, Jq)])
        delta, *_ = np.linalg.lstsq(A, -r, rcond=None)
        x, y, a = x + delta[0], y + delta[1], a + delta[2]
        gate = max(0.05, gate * 0.7)
        if np.hypot(delta[0], delta[1]) < 1e-4 and abs(delta[2]) < 1e-4:
            break
    p, d, j, m = residuals(x, y, a, 0.05)
    rms = float(np.sqrt(np.mean(d[m] ** 2))) if m.any() else float("inf")
    frac = float(m.mean())
    # conditioning: a corridor-like scene leaves one direction unconstrained
    cond = float(np.linalg.cond(A.T @ A)) if len(A) else float("inf")
    ok = frac >= 0.45 and rms <= 0.03 and cond < 1e6
    return {"ok": ok, "dx": float(x), "dy": float(y), "dyaw": float(a),
            "rms": rms, "inlier_frac": frac, "iterations": it + 1, "cond": cond,
            "why": "" if ok else f"weak fit (inliers {frac:.0%}, rms {rms*1000:.1f} mm)"}


def self_test(seed=0):
    """Synthetic room: recover a known offset with clutter, noise and a moved object."""
    rng = np.random.default_rng(seed)
    # room 3.0 x 2.4 m, rover at (1.1, 0.9), plus a 'dresser' box and a 'bed' edge
    segs = [((0, 0), (3.0, 0)), ((3.0, 0), (3.0, 2.4)), ((3.0, 2.4), (0, 2.4)),
            ((0, 2.4), (0, 0)), ((2.2, 1.6), (2.8, 1.6)), ((2.2, 1.6), (2.2, 2.4)),
            ((0, 0.6), (0.7, 0.6)), ((0.7, 0.6), (0.7, 0))]

    def scan_at(px, py, pa, moved=False):
        angs = np.radians(np.arange(0, 360, 0.8))
        r = np.full(len(angs), np.inf)
        extra = [((1.6, 1.4), (1.9, 1.4))] if moved else []
        for i, t in enumerate(angs):
            d = np.array([math.cos(t + pa), math.sin(t + pa)])
            for (ax, ay), (bx, by) in segs + extra:
                e = np.array([bx - ax, by - ay]); w = np.array([ax - px, ay - py])
                den = d[0] * (-e[1]) - d[1] * (-e[0])
                if abs(den) < 1e-9:
                    continue
                tt = (w[0] * (-e[1]) - w[1] * (-e[0])) / den
                u = (d[0] * w[1] - d[1] * w[0]) / den
                if tt > 0 and 0 <= u <= 1:
                    r[i] = min(r[i], tt)
        r += rng.normal(0, 0.008, len(r))
        r[rng.random(len(r)) < 0.05] = np.inf              # dropouts
        return r, angs[0], angs[1] - angs[0]

    ref_r, a0, inc = scan_at(1.1, 0.9, 0.0)
    worst = 0.0
    for true in [(0.03, -0.02, math.radians(4)), (-0.05, 0.04, math.radians(-7)),
                 (0.0, 0.0, 0.0), (0.08, 0.05, math.radians(9))]:
        # rover displaced by `true` in the reference body frame
        px, py = 1.1 + true[0], 0.9 + true[1]
        cur_r, _, _ = scan_at(px, py, true[2], moved=True)
        res = match(scan_to_points(ref_r, a0, inc), scan_to_points(cur_r, a0, inc))
        err = (abs(res["dx"] - true[0]), abs(res["dy"] - true[1]),
               abs(math.degrees(res["dyaw"] - true[2])))
        worst = max(worst, err[0], err[1])
        print(f"  true ({true[0]*100:+.1f},{true[1]*100:+.1f} cm, {math.degrees(true[2]):+.1f} deg)"
              f" -> ({res['dx']*100:+.1f},{res['dy']*100:+.1f} cm, {math.degrees(res['dyaw']):+.2f} deg)"
              f"  err {err[0]*1000:.1f}/{err[1]*1000:.1f} mm {err[2]:.2f} deg  ok={res['ok']}"
              f" rms {res['rms']*1000:.1f} mm inliers {res['inlier_frac']:.0%}")
    return worst


if __name__ == "__main__":
    w = self_test()
    print("SELF-TEST", "PASSED" if w < 0.005 else "FAILED", f"(worst {w*1000:.1f} mm)")
