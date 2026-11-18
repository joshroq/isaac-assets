#!/usr/bin/env python3
"""Re-express real trials recorded with a centre-convention grid calibration in the
corrected (border tape inside) metric frame - no rover or camera needed.

Before 2026-11-04 the grid fit put the BORDER tape centrelines on 0 / 1 m, but the border
tape lies inside the square with its OUTER edge on 0 / 1 m (2.2 % distance error); later
calibrations measured the 1-inch tape from the image (22.6-27.5 mm). This re-solves with
the true 25.4 mm. This re-solves the calibration a session used from its own saved
tape detections (border targets inset by half the tape width), maps every pose
old metric -> pixel -> new metric, and writes <real_dir>_gridfix/ (originals untouched).

    python3 calib/border_fix_data.py <session grid_calib.json> <real dir> [<real dir> ...]
"""
import json
import math
import shutil
import sys
from pathlib import Path

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parent))
import grid_target as gt  # noqa: E402


def resolve(calib, tape_w=gt.TAPE_WIDTH_M):
    """(H_old_img2m, H_new_img2m, inset_m) for a saved calibration: its tape crossings
    re-solved with the border-tape centrelines at tape_w/2 inside the grid. Works whatever
    border inset the calibration used (centre convention = 0, or a mis-measured width)."""
    e = tape_w / 2
    w, h = gt.grid_dims(calib)
    px = np.array([p["px"] for p in calib["points"]], float)
    m0 = np.array([p["m"] for p in calib["points"]], float)
    m = m0.copy()
    for k, L in ((0, w), (1, h)):
        m[m0[:, k] < 0.05, k] = e                    # border lines (0 or an old inset)
        m[m0[:, k] > L - 0.05, k] = L - e
    H_new, _, _ = gt.solve(px, m)
    return np.array(calib["H_img2m"], float), H_new, e


def mapper(H_old, H_new):
    M = H_new @ np.linalg.inv(H_old)                    # old metric -> new metric

    def f(x, y, yaw):
        p = gt.transform(M, np.column_stack([x, y]))
        q = gt.transform(M, np.column_stack([x + 0.01 * np.cos(yaw), y + 0.01 * np.sin(yaw)]))
        return p[:, 0], p[:, 1], np.arctan2(q[:, 1] - p[:, 1], q[:, 0] - p[:, 0])
    return M, f


def fix_dir(real_dir, f, suffix="_gridfix"):
    src = Path(real_dir)
    dst = src.with_name(src.name + suffix)
    n = 0
    for p in sorted(src.rglob("*")):
        out = dst / p.relative_to(src)
        if p.is_dir():
            out.mkdir(parents=True, exist_ok=True)
            continue
        out.parent.mkdir(parents=True, exist_ok=True)
        if p.suffix == ".csv" and (p.stem.startswith("rep") and (p.stem[3:].isdigit()
                                                                 or p.stem.endswith("_raw"))):
            d = np.genfromtxt(p, delimiter=",", names=True)
            x, y, yaw = f(d["x"], d["y"], d["yaw"])
            arr = np.column_stack([d["t"], x, y, yaw])
            np.savetxt(out, arr, delimiter=",", header="t,x,y,yaw", comments="", fmt="%.6f")
            n += 1
        elif p.suffix != ".mp4":                            # videos stay where they are
            shutil.copy2(p, out)
    (dst / "GRIDFIX.json").write_text(json.dumps({"source": str(src), "note": __doc__.split("\n\n")[1]}, indent=2))
    return dst, n


def main():
    import argparse
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("calib", help="the grid_calib.json the trials were recorded with")
    ap.add_argument("real", nargs="*", help="real dirs to re-express (-> <dir>_gridfix/)")
    ap.add_argument("--tape-width-m", type=float, default=gt.TAPE_WIDTH_M)
    ap.add_argument("--suffix", default="_gridfix")
    a = ap.parse_args()
    calib = json.loads(Path(a.calib).read_text())
    H_old, H_new, e = resolve(calib, a.tape_width_m)
    M, f = mapper(H_old, H_new)
    g = np.array([(x, y) for x in np.linspace(0.2, 0.8, 7) for y in np.linspace(0.1, 0.9, 9)])
    pn = gt.transform(M, g)
    scale = np.linalg.norm(pn[9:] - pn[:-9], axis=1) / np.linalg.norm(g[9:] - g[:-9], axis=1)
    print(f"calibration {calib.get('calibrated_at')} (border {calib.get('border_tape', 'centre')}, inset "
          f"{calib.get('border_inset_m', 0) * 1000:.2f} mm) -> inset {e*1000:.2f} mm: distance scale "
          f"new/old {scale.mean():.4f}, shift up to {np.max(np.linalg.norm(pn - g, axis=1))*1000:.1f} mm")
    for rd in a.real:
        dst, n = fix_dir(rd, f, a.suffix)
        print(f"  {rd} -> {dst} ({n} trial CSVs)")


if __name__ == "__main__":
    main()
