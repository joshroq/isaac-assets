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


# Bin a scan onto GRID_BINS fixed angles (nearest return per bin) -> (ranges, 0, step).
def resample(ranges, angle_min, angle_inc, n_bins=GRID_BINS):
    """Put a scan on the fixed angular grid (angle_min 0, inc 2*pi/n_bins).
    The LD19 delivers 500-503 beams per turn with a varying increment, so beam INDEX is
    not a fixed angle; per-bin median is only meaningful after this."""
    ranges_arr = np.asarray(ranges, float)
    angles = (angle_min + angle_inc * np.arange(len(ranges_arr))) % (2 * math.pi)
    bin_index = np.round(angles / (2 * math.pi / n_bins)).astype(int) % n_bins
    binned = np.full(n_bins, np.nan)
    valid = np.isfinite(ranges_arr) & (ranges_arr > 0)
    for bin_number, range_m in zip(bin_index[valid], ranges_arr[valid]):        # nearest beam per bin (keep the closer)
        if not np.isfinite(binned[bin_number]) or range_m < binned[bin_number]:
            binned[bin_number] = range_m
    return binned, 0.0, 2 * math.pi / n_bins


# Per-bin median of several scans; bins with returns in under half the scans -> NaN.
def median_scan(scans):
    """Median per beam over several stationary scans (kills speckle + moving people).
    scans: list of 1-D range arrays of equal length (0/inf/nan = no return)."""
    stacked = np.array([np.where(np.isfinite(scan) & (scan > 0), scan, np.nan) for scan in scans], float)
    with np.errstate(all="ignore"), warnings.catch_warnings():
        warnings.simplefilter("ignore", RuntimeWarning)    # bins with no return -> NaN
        median_ranges = np.nanmedian(stacked, axis=0)
        enough_returns = np.sum(np.isfinite(stacked), axis=0) >= max(1, len(scans) // 2)
    median_ranges[~enough_returns] = np.nan
    return median_ranges


# Polar scan -> body-frame (x, y) points: ranges between RANGE_MIN and RANGE_MAX,
# mirrored if flip, rotated + shifted by the lidar mounting offset (x, y, yaw).
def scan_to_points(ranges, angle_min, angle_inc, offset=(0.0, 0.0, 0.0), flip=False):
    """Ranges -> Nx2 points in the BODY frame (lidar mount offset x, y, yaw applied;
    flip=True for a scan that turns clockwise, i.e. mirrored)."""
    ranges_arr = np.asarray(ranges, float)
    angles = angle_min + angle_inc * np.arange(len(ranges_arr))
    angles = (-angles if flip else angles) + offset[2]
    valid = np.isfinite(ranges_arr) & (ranges_arr > RANGE_MIN) & (ranges_arr < RANGE_MAX)
    return np.column_stack([offset[0] + ranges_arr[valid] * np.cos(angles[valid]),
                            offset[1] + ranges_arr[valid] * np.sin(angles[valid])])


# 2x2 rotation matrix for angle a (radians).
def _rot(a):
    cos_a, sin_a = math.cos(a), math.sin(a)
    return np.array([[cos_a, -sin_a], [sin_a, cos_a]])


# Surface normal at every reference point (PCA of its k nearest neighbours: smallest
# eigenvector) and whether that neighbourhood is line-like enough to trust it.
def _normals(ref, tree, k=6):
    _, neighbour_index = tree.query(ref, k=min(k, len(ref)))
    point_normals = np.zeros_like(ref)
    planar = np.zeros(len(ref), bool)
    for point_index, neighbours in enumerate(neighbour_index):
        centred = ref[neighbours] - ref[neighbours].mean(axis=0)
        eigenvalues, eigenvectors = np.linalg.eigh(centred.T @ centred)
        point_normals[point_index] = eigenvectors[:, 0]                       # smallest eigenvector = surface normal
        planar[point_index] = eigenvalues[1] > 8 * max(eigenvalues[0], 1e-12)
    return point_normals, planar


# Point-to-line ICP of `cur` onto `ref` (see the module docstring):
# 1) coarse yaw seed: try +-yaw_search_deg in 1 deg steps, keep the most correspondences;
# 2) iterate: nearest neighbours within a shrinking gate, residual along the reference
#    normal, drop the worst (1-trim), solve the linearised least squares for (dx, dy, dyaw);
# 3) quality: RMS, inlier fraction and conditioning (a corridor is unconstrained along it).
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
    ref_tree = cKDTree(ref)
    normals, planar = _normals(ref, ref_tree)

    def residuals(x, y, a, gate):
        cur_in_ref = cur @ _rot(a).T + [x, y]
        nn_distance, nn_index = ref_tree.query(cur_in_ref, distance_upper_bound=gate)
        has_match = np.isfinite(nn_distance)
        return cur_in_ref, nn_distance, nn_index, has_match

    # coarse yaw seed (cheap): pick the rotation with the most close correspondences
    x, y, a = init
    best_seed = None
    for yaw_offset in np.radians(np.arange(-yaw_search_deg, yaw_search_deg + 0.1, 1.0)):
        _, nn_distance, _, has_match = residuals(x, y, a + yaw_offset, 0.08)
        seed_score = has_match.sum() - (np.mean(nn_distance[has_match]) if has_match.any() else 0)
        if best_seed is None or seed_score > best_seed[0]:
            best_seed = (seed_score, a + yaw_offset)
    a = best_seed[1]
    gate = max_corr
    for iteration in range(max_iter):
        cur_in_ref, nn_distance, nn_index, has_match = residuals(x, y, a, gate)
        if has_match.sum() < 20:
            return {"ok": False, "why": "lost correspondences"}
        matched_index = np.nonzero(has_match)[0]
        ref_index = nn_index[has_match]
        line_normals = normals[ref_index]
        line_residuals = np.einsum("ij,ij->i", cur_in_ref[matched_index] - ref[ref_index], line_normals)       # point-to-line residual
        inliers = np.abs(line_residuals) <= np.quantile(np.abs(line_residuals), trim)        # trimmed
        inliers &= planar[ref_index] | (np.abs(line_residuals) < 0.02)
        matched_index, ref_index, line_normals, line_residuals = matched_index[inliers], ref_index[inliers], line_normals[inliers], line_residuals[inliers]
        if len(matched_index) < 20:
            return {"ok": False, "why": "too few inliers"}
        cur_points = cur[matched_index]
        # linearised: r + n . (dt + dA * J q') = 0, J = d/da R(a) q
        rotated = cur_points @ _rot(a).T
        rotation_jacobian = np.column_stack([-rotated[:, 1], rotated[:, 0]])
        jacobian = np.column_stack([line_normals[:, 0], line_normals[:, 1], np.einsum("ij,ij->i", line_normals, rotation_jacobian)])
        step, *_ = np.linalg.lstsq(jacobian, -line_residuals, rcond=None)
        x, y, a = x + step[0], y + step[1], a + step[2]
        gate = max(0.05, gate * 0.7)
        if np.hypot(step[0], step[1]) < 1e-4 and abs(step[2]) < 1e-4:
            break
    cur_in_ref, nn_distance, nn_index, has_match = residuals(x, y, a, 0.05)
    rms = float(np.sqrt(np.mean(nn_distance[has_match] ** 2))) if has_match.any() else float("inf")
    inlier_fraction = float(has_match.mean())
    # conditioning: a corridor-like scene leaves one direction unconstrained
    condition_number = float(np.linalg.cond(jacobian.T @ jacobian)) if len(jacobian) else float("inf")
    ok = inlier_fraction >= 0.45 and rms <= 0.03 and condition_number < 1e6
    return {"ok": ok, "dx": float(x), "dy": float(y), "dyaw": float(a),
            "rms": rms, "inlier_frac": inlier_fraction, "iterations": iteration + 1, "cond": condition_number,
            "why": "" if ok else f"weak fit (inliers {inlier_fraction:.0%}, rms {rms*1000:.1f} mm)"}


# Synthetic room with clutter, noise, dropouts and a moved object: known offsets must
# be recovered within 5 mm.
def self_test(seed=0):
    """Synthetic room: recover a known offset with clutter, noise and a moved object."""
    rng = np.random.default_rng(seed)
    # room 3.0 x 2.4 m, rover at (1.1, 0.9), plus a 'dresser' box and a 'bed' edge
    walls = [((0, 0), (3.0, 0)), ((3.0, 0), (3.0, 2.4)), ((3.0, 2.4), (0, 2.4)),
            ((0, 2.4), (0, 0)), ((2.2, 1.6), (2.8, 1.6)), ((2.2, 1.6), (2.2, 2.4)),
            ((0, 0.6), (0.7, 0.6)), ((0.7, 0.6), (0.7, 0))]

    # Ray-cast a synthetic scan from pose (px, py, pa) against the room's wall segments.
    def scan_at(px, py, pa, moved=False):
        angles = np.radians(np.arange(0, 360, 0.8))
        ranges_arr = np.full(len(angles), np.inf)
        moved_object = [((1.6, 1.4), (1.9, 1.4))] if moved else []
        for beam_index, beam_angle in enumerate(angles):
            ray_dir = np.array([math.cos(beam_angle + pa), math.sin(beam_angle + pa)])
            for (ax, ay), (bx, by) in walls + moved_object:
                segment_vec = np.array([bx - ax, by - ay]); to_segment = np.array([ax - px, ay - py])
                denominator = ray_dir[0] * (-segment_vec[1]) - ray_dir[1] * (-segment_vec[0])
                if abs(denominator) < 1e-9:
                    continue
                hit_range = (to_segment[0] * (-segment_vec[1]) - to_segment[1] * (-segment_vec[0])) / denominator
                segment_param = (ray_dir[0] * to_segment[1] - ray_dir[1] * to_segment[0]) / denominator
                if hit_range > 0 and 0 <= segment_param <= 1:
                    ranges_arr[beam_index] = min(ranges_arr[beam_index], hit_range)
        ranges_arr += rng.normal(0, 0.008, len(ranges_arr))
        ranges_arr[rng.random(len(ranges_arr)) < 0.05] = np.inf              # dropouts
        return ranges_arr, angles[0], angles[1] - angles[0]

    ref_ranges, angle_min, angle_increment = scan_at(1.1, 0.9, 0.0)
    worst_error = 0.0
    for true_offset in [(0.03, -0.02, math.radians(4)), (-0.05, 0.04, math.radians(-7)),
                 (0.0, 0.0, 0.0), (0.08, 0.05, math.radians(9))]:
        # rover displaced by `true` in the reference body frame
        px, py = 1.1 + true_offset[0], 0.9 + true_offset[1]
        cur_ranges, _, _ = scan_at(px, py, true_offset[2], moved=True)
        result = match(scan_to_points(ref_ranges, angle_min, angle_increment), scan_to_points(cur_ranges, angle_min, angle_increment))
        error = (abs(result["dx"] - true_offset[0]), abs(result["dy"] - true_offset[1]),
               abs(math.degrees(result["dyaw"] - true_offset[2])))
        worst_error = max(worst_error, error[0], error[1])
        print(f"  true ({true_offset[0]*100:+.1f},{true_offset[1]*100:+.1f} cm, {math.degrees(true_offset[2]):+.1f} deg)"
              f" -> ({result['dx']*100:+.1f},{result['dy']*100:+.1f} cm, {math.degrees(result['dyaw']):+.2f} deg)"
              f"  err {error[0]*1000:.1f}/{error[1]*1000:.1f} mm {error[2]:.2f} deg  ok={result['ok']}"
              f" rms {result['rms']*1000:.1f} mm inliers {result['inlier_frac']:.0%}")
    return worst_error


if __name__ == "__main__":
    w = self_test()
    print("SELF-TEST", "PASSED" if w < 0.005 else "FAILED", f"(worst {w*1000:.1f} mm)")
