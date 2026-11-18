#!/usr/bin/env python3
"""Ground-truth 2D occupancy map of an Isaac environment USD, for known-map nav.

Builds the AUTHORITATIVE shared Nav2 planning map for the thesis "known-map"
experiment (Experiment A) directly from the simulated geometry -- no driving,
fully deterministic, reproducible. It composes the SAME rover + environment stage
that run_baked_rover.py loads, so the map is expressed in the map frame ANCHORED
AT THE ROVER'S START POSE (rover start = map (0,0), rover forward = +x). That is
the frame slam_toolbox/AMCL anchor at start, so the coordinates in goals.yaml
line up with this map exactly the way they line up with a driven SLAM map.

WHAT IT DOES
    * opens jetson_rover_v2.usd and references the environment under
      /World/NavEnvironment (identical composition to run_baked_rover.run())
    * reads the rover's base_footprint start transform -> map-frame origin/yaw
    * rasterises every environment mesh/primitive whose height overlaps the
      robot's collision band [--z-min, --z-max] into an occupancy grid
    * writes Nav2-standard  <out>/map.pgm  +  <out>/map.yaml   (free=254,
      occupied=0, unknown=205), plus <out>/map.npz for plot_paths.py backgrounds

ASSET RESOLUTION -- IMPORTANT
    The Isaac warehouse SHELL (walls, racks, pillars, pallets, floor tiles) is not
    inline geometry: each prim references an Omniverse asset on the Isaac S3 server
    (https://omniverse-content-production.s3.../Simple_Warehouse/Props/*.usd). Only
    the full Isaac Kit app registers the Omniverse asset resolver that can fetch
    those, so by DEFAULT this tool boots a HEADLESS SimulationApp -- the props then
    download/cache exactly as the running sim sees them, and the map is COMPLETE
    (walls + your inline obstacles). Pass --no-sim-app for the fast bare-pxr path,
    which resolves ONLY locally-authored geometry (your Cube/Sphere/Cone/Cylinder
    obstacles + ground plane) and SILENTLY OMITS the S3 warehouse shell -- fine for
    an all-inline environment, wrong for the standard warehouse.

RUN (needs Isaac's python; a launcher alias 'isaacpy' also works)
    ~/Applications/isaacsim/python.sh experiments/make_occupancy_map.py
    ~/Applications/isaacsim/python.sh experiments/make_occupancy_map.py \
        --env warehouse_basic_obstacles.usd --out experiments/maps/warehouse \
        --resolution 0.05 --z-min 0.05 --z-max 0.6

Then point any nav stack at it, e.g. the LiDAR stack in known-map mode:
    ros2 launch isaac_nav_lidar.launch.py use_slam:=False \
        map:=<repo>/experiments/maps/warehouse/map.yaml
"""
from __future__ import annotations

import argparse
import math
import os
import sys
from pathlib import Path

# --- locate the repo + the baked rover USD (same rules as run_baked_rover) -----
HERE = Path(__file__).resolve().parent
REPO = HERE.parent
ROVER_USD_NAMES = ("jetson_rover_v2.usd", "ugv_rover_v2.usd", "jetson_rover_new.usd")
ROVER_PRIM_PATH = "/World/ugv_rover"
BASE_FOOTPRINT_PRIM_PATH = "/World/ugv_rover/base_footprint"
ENVIRONMENT_PRIM_PATH = "/World/NavEnvironment"

# pxr modules are bound lazily by the chosen backend: with --no-sim-app we bootstrap
# the standalone omni.usd.libs onto the path; with SimulationApp we import pxr AFTER
# Kit boots (so it is the same USD runtime the sim uses, with the omni resolver).
Usd = UsdGeom = Gf = None


def _bootstrap_pxr_standalone() -> None:
    """Make `pxr` importable under Isaac's plain python.sh for the --no-sim-app path
    (python.sh does not put omni.usd.libs on sys.path / its .so on the loader path).
    Re-execs once with PYTHONPATH + LD_LIBRARY_PATH set (LD_LIBRARY_PATH must be set
    before process start for the dynamic loader). No-op if pxr already imports."""
    try:
        import pxr  # noqa: F401
        return
    except ImportError:
        pass
    if os.environ.get("_MAKE_OCC_MAP_REEXEC") == "1":
        return
    import glob
    candidates = []
    for base in (os.environ.get("ISAACSIM_PATH"),
                 os.path.expanduser("~/Applications/isaacsim"),
                 os.path.expanduser("~/.local/share/ov/pkg/isaac-sim")):
        if base:
            candidates += glob.glob(os.path.join(base, "extscache", "omni.usd.libs-*"))
    for lib in sorted(candidates, reverse=True):
        if not os.path.isdir(os.path.join(lib, "pxr")):
            continue
        env = dict(os.environ)
        env["_MAKE_OCC_MAP_REEXEC"] = "1"
        env["PYTHONPATH"] = lib + os.pathsep + env.get("PYTHONPATH", "")
        env["LD_LIBRARY_PATH"] = (os.path.join(lib, "bin") + os.pathsep
                                  + env.get("LD_LIBRARY_PATH", ""))
        os.execve(sys.executable, [sys.executable] + sys.argv, env)


def _bind_pxr() -> None:
    global Usd, UsdGeom, Gf
    from pxr import Usd as _U, UsdGeom as _UG, Gf as _G
    Usd, UsdGeom, Gf = _U, _UG, _G


def _find_rover_usd() -> Path:
    for name in ROVER_USD_NAMES:
        cand = REPO / name
        if cand.is_file():
            return cand
    sys.exit(f"No rover USD ({ROVER_USD_NAMES}) found in {REPO}")


def _yaw_from_matrix(m) -> float:
    """Z-up yaw of a frame from its local->world matrix (Gf row-vector convention:
    the frame's world x-axis is row 0)."""
    return math.atan2(m[0][1], m[0][0])


# ── stage backends ────────────────────────────────────────────────────────────
def open_stage_simapp(rover_path: Path, env_path: Path, settle: int):
    """Boot a headless Isaac Kit app so the Omniverse asset resolver is active, then
    compose rover + env exactly like run_baked_rover. Returns (stage, close_fn)."""
    from isaacsim import SimulationApp
    app = SimulationApp({"headless": True, "renderer": "RayTracedLighting",
                         "multi_gpu": False, "active_gpu": 0})
    import omni.usd
    from isaacsim.core.utils.stage import add_reference_to_stage
    _bind_pxr()
    if not omni.usd.get_context().open_stage(str(rover_path)):
        app.close(); sys.exit(f"Could not open rover USD: {rover_path}")
    for _ in range(10):
        app.update()
    stage = omni.usd.get_context().get_stage()
    if stage is None or not stage.GetPrimAtPath(ROVER_PRIM_PATH).IsValid():
        app.close(); sys.exit(f"Rover prim {ROVER_PRIM_PATH} missing from {rover_path}")
    add_reference_to_stage(usd_path=str(env_path), prim_path=ENVIRONMENT_PRIM_PATH)
    # let the S3/Nucleus props download + compose (first run pulls them, then cached)
    for _ in range(max(settle, 10)):
        app.update()
    return stage, app.close


def open_stage_pxr(rover_path: Path, env_path: Path, _settle: int):
    """Bare-pxr composition (fast, no GPU) -- resolves only local geometry."""
    _bootstrap_pxr_standalone()
    try:
        _bind_pxr()
    except ImportError:
        sys.exit("pxr not importable. Run under Isaac's python (python.sh), or drop "
                 "--no-sim-app so a SimulationApp provides it.")
    stage = Usd.Stage.Open(str(rover_path))
    if not stage:
        sys.exit(f"Could not open rover USD: {rover_path}")
    stage.DefinePrim(ENVIRONMENT_PRIM_PATH).GetReferences().AddReference(str(env_path))
    return stage, (lambda: None)


# ── occupancy build ─────────────────────────────────────────────────────────────
def build_map(stage, args) -> int:
    import numpy as np

    mpu = UsdGeom.GetStageMetersPerUnit(stage) or 1.0
    if UsdGeom.GetStageUpAxis(stage) != UsdGeom.Tokens.z:
        print(f"[warn] stage up-axis is not Z; assuming Z-up (Isaac default).")

    xcache = UsdGeom.XformCache(Usd.TimeCode.Default())
    env_prim = stage.GetPrimAtPath(ENVIRONMENT_PRIM_PATH)

    # map frame = rover base_footprint start pose
    base = stage.GetPrimAtPath(BASE_FOOTPRINT_PRIM_PATH)
    if not base or not base.IsValid():
        sys.exit(f"Rover base prim missing: {BASE_FOOTPRINT_PRIM_PATH}")
    base_w = xcache.GetLocalToWorldTransform(base)
    bt = base_w.ExtractTranslation()
    b_x, b_y = bt[0] * mpu, bt[1] * mpu
    b_yaw = _yaw_from_matrix(base_w)
    cos_y, sin_y = math.cos(-b_yaw), math.sin(-b_yaw)

    def to_map(wx: float, wy: float):
        dx, dy = wx - b_x, wy - b_y
        return cos_y * dx - sin_y * dy, sin_y * dx + cos_y * dy

    print(f"[info] rover start (world m): ({b_x:.3f}, {b_y:.3f}) "
          f"yaw {math.degrees(b_yaw):.1f} deg  ->  map (0,0)")

    predicate = Usd.TraverseInstanceProxies(Usd.PrimDefaultPredicate)
    z0, z1 = args.z_min, args.z_max
    tris = []           # (x0,y0,x1,y1,x2,y2) map frame, in the height band
    boxes = []          # (xmin,ymin,xmax,ymax) map frame, in the height band
    ext = [math.inf, math.inf, -math.inf, -math.inf]  # xmin,ymin,xmax,ymax

    def note(mx, my):
        ext[0] = min(ext[0], mx); ext[1] = min(ext[1], my)
        ext[2] = max(ext[2], mx); ext[3] = max(ext[3], my)

    n_mesh = n_prim = 0
    bbcache = UsdGeom.BBoxCache(Usd.TimeCode.Default(), [UsdGeom.Tokens.default_])
    for prim in Usd.PrimRange(env_prim, predicate):
        if not UsdGeom.Imageable(prim):
            continue
        # skip pure decals/lights/cameras cheaply
        if prim.IsA(UsdGeom.Mesh):
            mesh = UsdGeom.Mesh(prim)
            pts = mesh.GetPointsAttr().Get()
            counts = mesh.GetFaceVertexCountsAttr().Get()
            idxs = mesh.GetFaceVertexIndicesAttr().Get()
            if not pts or not counts or not idxs:
                continue
            m = xcache.GetLocalToWorldTransform(prim)
            wpts = []
            for p in pts:
                wp = m.Transform(Gf.Vec3d(p[0], p[1], p[2]))
                mx, my = to_map(wp[0] * mpu, wp[1] * mpu)
                wpts.append((mx, my, wp[2] * mpu)); note(mx, my)
            n_mesh += 1
            k = 0
            for c in counts:
                if c >= 3:
                    v0 = wpts[idxs[k]]
                    for j in range(1, c - 1):
                        v1 = wpts[idxs[k + j]]; v2 = wpts[idxs[k + j + 1]]
                        if max(v0[2], v1[2], v2[2]) >= z0 and min(v0[2], v1[2], v2[2]) <= z1:
                            tris.append((v0[0], v0[1], v1[0], v1[1], v2[0], v2[1]))
                k += c
        elif prim.IsA(UsdGeom.Gprim):
            rng = bbcache.ComputeWorldBound(prim).ComputeAlignedRange()
            if rng.IsEmpty():
                continue
            lo, hi = rng.GetMin(), rng.GetMax()
            corners = []
            for cx in (lo[0], hi[0]):
                for cy in (lo[1], hi[1]):
                    mx, my = to_map(cx * mpu, cy * mpu)
                    corners.append((mx, my)); note(mx, my)
            if hi[2] * mpu >= z0 and lo[2] * mpu <= z1:
                xs = [c[0] for c in corners]; ys = [c[1] for c in corners]
                boxes.append((min(xs), min(ys), max(xs), max(ys))); n_prim += 1

    if ext[0] is math.inf:
        sys.exit("No environment geometry found under /World/NavEnvironment "
                 "(if this is the S3 warehouse, drop --no-sim-app so props resolve).")

    res = args.resolution
    ox, oy = ext[0] - args.margin, ext[1] - args.margin
    W = int(math.ceil((ext[2] + args.margin - ox) / res))
    H = int(math.ceil((ext[3] + args.margin - oy) / res))
    occ = np.zeros((H, W), dtype=bool)
    print(f"[info] extent x[{ext[0]:.2f},{ext[2]:.2f}] y[{ext[1]:.2f},{ext[3]:.2f}]  "
          f"grid {W}x{H} @ {res} m  ({n_mesh} meshes, {n_prim} prims, "
          f"{len(tris)} band-tris, {len(boxes)} band-boxes)")

    def cell(mx, my):
        return int((mx - ox) / res), int((my - oy) / res)

    for (xmn, ymn, xmx, ymx) in boxes:
        i0, j0 = cell(xmn, ymn); i1, j1 = cell(xmx, ymx)
        i0, i1 = max(0, min(i0, i1)), min(W - 1, max(i0, i1))
        j0, j1 = max(0, min(j0, j1)), min(H - 1, max(j0, j1))
        if i1 >= i0 and j1 >= j0:
            occ[j0:j1 + 1, i0:i1 + 1] = True

    for (ax, ay, bx, by, cx, cy) in tris:
        i0, j0 = cell(min(ax, bx, cx), min(ay, by, cy))
        i1, j1 = cell(max(ax, bx, cx), max(ay, by, cy))
        i0, i1 = max(0, i0), min(W - 1, i1)
        j0, j1 = max(0, j0), min(H - 1, j1)
        if i1 < i0 or j1 < j0:
            continue
        area = (bx - ax) * (cy - ay) - (cx - ax) * (by - ay)
        if abs(area) < 1e-12:
            occ[j0:j1 + 1, i0:i1 + 1] = True
            continue
        inv = 1.0 / area
        for jj in range(j0, j1 + 1):
            py = oy + (jj + 0.5) * res
            for ii in range(i0, i1 + 1):
                px = ox + (ii + 0.5) * res
                w0 = ((bx - px) * (cy - py) - (cx - px) * (by - py)) * inv
                w1 = ((cx - px) * (ay - py) - (ax - px) * (cy - py)) * inv
                if w0 >= -1e-9 and w1 >= -1e-9 and (1.0 - w0 - w1) >= -1e-9:
                    occ[jj, ii] = True

    # fill everything not reachable (as free) from the rover start cell: solidifies
    # hollow obstacle interiors AND marks outside-the-walls occupied, so the planning
    # map is the true navigable free space. Robust to wall gaps (exterior stays free
    # only if actually reachable from start).
    if not args.no_fill:
        si, sj = cell(0.0, 0.0)
        if 0 <= si < W and 0 <= sj < H and not occ[sj, si]:
            free = ~occ
            try:
                from scipy.ndimage import label
                lbl, _ = label(free)
                reachable = lbl == lbl[sj, si]
            except ImportError:
                from collections import deque
                reachable = np.zeros_like(free)
                dq = deque([(sj, si)]); reachable[sj, si] = True
                while dq:
                    j, i = dq.popleft()
                    for dj, di in ((1, 0), (-1, 0), (0, 1), (0, -1)):
                        nj, ni = j + dj, i + di
                        if 0 <= nj < H and 0 <= ni < W and free[nj, ni] and not reachable[nj, ni]:
                            reachable[nj, ni] = True; dq.append((nj, ni))
            occ = occ | ~reachable
        else:
            print("[warn] start cell (0,0) is occupied/out of grid; skipping fill.")

    if args.inflate > 0:
        try:
            from scipy.ndimage import binary_dilation
            occ = binary_dilation(occ, iterations=args.inflate)
        except ImportError:
            k = args.inflate
            grown = np.zeros_like(occ)
            for dj in range(-k, k + 1):
                for di in range(-k, k + 1):
                    grown |= np.roll(np.roll(occ, dj, 0), di, 1)
            occ = grown

    out = Path(args.out); out.mkdir(parents=True, exist_ok=True)
    img = np.full((H, W), 254, dtype=np.uint8)   # free=254, occupied=0 (ROS convention)
    img[occ] = 0
    with open(out / "map.pgm", "wb") as f:       # ROS origin is lower-left -> flipud
        f.write(b"P5\n%d %d\n255\n" % (W, H))
        f.write(np.flipud(img).tobytes())
    with open(out / "map.yaml", "w") as f:
        f.write(
            "# Ground-truth occupancy map generated from the Isaac USD by\n"
            "# experiments/make_occupancy_map.py. Map frame anchored at the rover\n"
            "# start pose (rover start = (0,0)), so goals.yaml lines up unchanged.\n"
            "image: map.pgm\nmode: trinary\n"
            f"resolution: {res}\norigin: [{ox:.4f}, {oy:.4f}, 0.0]\n"
            "negate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.25\n")
    grid = np.zeros((H, W), dtype=np.int8); grid[occ] = 100
    np.savez(out / "map.npz", grid=grid, resolution=res, origin_x=ox, origin_y=oy)

    if not args.no_preview:
        try:
            import matplotlib; matplotlib.use("Agg")
            import matplotlib.pyplot as plt
            import yaml
            fig, ax = plt.subplots(figsize=(9, 9))
            ax.imshow(img, cmap="gray", origin="lower",
                      extent=[ox, ox + W * res, oy, oy + H * res])
            ax.plot(0, 0, "*", ms=16, color="lime")
            ax.annotate("START (0,0)", (0, 0), color="green", fontsize=8)
            goals_file = HERE / "goals.yaml"
            if goals_file.is_file():
                for k, g in (yaml.safe_load(open(goals_file)).get("goals") or {}).items():
                    gx, gy = g["x"], g["y"]
                    ci, cj = cell(gx, gy)
                    st = "OOB"
                    if 0 <= ci < W and 0 <= cj < H:
                        st = "FREE" if not occ[cj, ci] else "OCC(!)"
                    ax.plot(gx, gy, "o", ms=8, color="red")
                    ax.annotate(f"G{k} {g.get('name','')}\n{st}", (gx, gy),
                                color="red", fontsize=7)
            ax.set_aspect("equal"); ax.set_xlabel("x [m] (map frame)")
            ax.set_ylabel("y [m]"); ax.set_title(f"{Path(args.env).name} ground-truth map")
            fig.savefig(out / "map_preview.png", dpi=110, bbox_inches="tight")
            plt.close(fig)
            print(f"[ok] wrote {out/'map_preview.png'}")
        except Exception as e:  # matplotlib/yaml optional; never fail the map on preview
            print(f"[warn] preview skipped: {e}")

    n_occ = int(occ.sum())
    print(f"[ok] wrote {out/'map.pgm'}, map.yaml, map.npz")
    print(f"[ok] {n_occ} occupied cells ({100.0*n_occ/(W*H):.1f}% of grid), "
          f"origin=({ox:.3f},{oy:.3f})")
    if n_mesh <= 12 and args.no_sim_app:
        print("[warn] very few meshes resolved -- if this is the S3 warehouse, re-run "
              "WITHOUT --no-sim-app so the walls/racks download.")
    return 0


def main() -> int:
    ap = argparse.ArgumentParser(
        description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--env", default="warehouse_basic_obstacles.usd",
                    help="environment USD (name under repo root, or a path)")
    ap.add_argument("--rover", default=None, help="rover USD (default: auto-detect)")
    ap.add_argument("--out", default=str(HERE / "maps" / "warehouse"),
                    help="output directory for map.pgm/map.yaml/map.npz")
    ap.add_argument("--resolution", type=float, default=0.05, help="metres/cell")
    ap.add_argument("--z-min", type=float, default=0.05,
                    help="bottom of the robot collision band (m, map frame)")
    ap.add_argument("--z-max", type=float, default=0.60,
                    help="top of the robot collision band (m, map frame)")
    ap.add_argument("--margin", type=float, default=1.0,
                    help="metres of free padding around the geometry extent")
    ap.add_argument("--inflate", type=int, default=0,
                    help="dilate occupied cells by N cells (0 = raw geometry)")
    ap.add_argument("--no-fill", action="store_true",
                    help="keep raw surface rasterization (do NOT fill hollow obstacle "
                         "interiors / outside-the-walls from the start cell)")
    ap.add_argument("--no-preview", action="store_true",
                    help="skip writing map_preview.png (needs matplotlib)")
    ap.add_argument("--settle", type=int, default=30,
                    help="SimulationApp update() steps to let S3 props download/compose")
    ap.add_argument("--no-sim-app", action="store_true",
                    help="fast bare-pxr path; resolves ONLY local geometry (no S3 shell)")
    args = ap.parse_args()

    env_path = Path(args.env)
    if not env_path.is_file():
        env_path = REPO / args.env
    if not env_path.is_file():
        sys.exit(f"Environment USD not found: {args.env}")
    rover_path = Path(args.rover) if args.rover else _find_rover_usd()

    backend = open_stage_pxr if args.no_sim_app else open_stage_simapp
    stage, close = backend(rover_path, env_path, args.settle)
    try:
        return build_map(stage, args)
    finally:
        close()


if __name__ == "__main__":
    raise SystemExit(main())
