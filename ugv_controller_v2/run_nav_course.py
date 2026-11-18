"""Isaac launcher — baked-graph rover on a ground plane inside a navcmp COURSE.

    isaacpy ugv_controller_v2/run_nav_course.py --course sim2real_calib/navcmp/courses/grid2x2_box.yaml
    isaacpy ugv_controller_v2/run_nav_course.py --course <yaml> --headless --no-camera-topics
    isaacpy ugv_controller_v2/run_nav_course.py --course <yaml> --calib <surface yaml>

The sim twin of the arena built on the real grid: every wall/box of the course YAML is
authored as a static collider (rendered, so the RTX lidar and the depth camera see it),
placed so the rover's spawn pose IS the course start pose. Course frame -> world:
    world = T_spawn ∘ T_start⁻¹ ∘ course
The rover drives its wheels through PhysX wheel-floor contact; --calib applies a calibrated
physics surface as in run_ground_plane.py. --world-map adds walls from a saved
slam_toolbox map (navcmp map.json) so Isaac also sees the room the real lidar mapped.

LIDAR RANGE (--lidar-profile, default ld19): the baked lidar references NVIDIA's generic
Example_Rotary_2D profile, which publishes range_min 1.0 m / range_max 200 m - BLIND to
everything within 1 m (measured 2026-11-11: 39 % of beams -1.0 in a 2.5 m arena; SLAM
then lost the rover). The real LD19 ranges ~0.02-12 m. The ld19 profile overrides the
OmniLidar near/far range in memory (shared USD untouched) to 0.10-12 m (0.10 = the
slam_toolbox min_laser_range both rovers use). 'native' keeps the USD as authored.
"""

import argparse
import json
import math
import sys
from pathlib import Path

HERE = Path(__file__).resolve().parent
COMMON_DIR = HERE / "common"
NAVCMP_DIR = HERE.parent / "sim2real_calib" / "navcmp"
for _p in (COMMON_DIR, NAVCMP_DIR):
    if str(_p) not in sys.path:
        sys.path.insert(0, str(_p))

import course as C  # noqa: E402
from run_baked_rover import ROVER_ARTICULATION_PRIM_PATH, run  # noqa: E402

COURSE_PRIM = "/World/NavCourse"
COLORS = {"wall": (0.62, 0.55, 0.45), "box": (0.80, 0.45, 0.20)}


def _parse_res(value):
    if value.strip().lower() in ("native", "default", "full"):
        return None
    w, h = value.lower().split("x")
    return (int(w), int(h))


def _parse_args():
    p = argparse.ArgumentParser(description=__doc__,
                                formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--course", required=True, help="navcmp course YAML")
    p.add_argument("--world-map", default=None,
                   help="navcmp map.json (real slam map + its registration meta.json beside it): "
                        "add its occupied cells as walls")
    p.add_argument("--headless", action="store_true")
    p.add_argument("--no-camera-topics", dest="camera_topics", action="store_false", default=True,
                   help="lidar-only: skip the RGB/depth render products")
    p.add_argument("--camera-res", type=_parse_res, default=(640, 480),
                   help="WxH for the camera render products (default 640x480: full res "
                        "GPU-starves depth to ~0.1 Hz) or 'native'")
    p.add_argument("--lidar-profile", choices=["ld19", "native"], default="ld19",
                   help="ld19: override the lidar range to 0.10-12 m (see module doc)")
    p.add_argument("--calib", default=None, help="sim_calibration.yaml-shaped physics surface")
    p.add_argument("--combine-mode", choices=["min", "average", "multiply"], default=None)
    return p.parse_args()


def _map_boxes(map_json):
    """Occupied cells of a real-rover map.json, as boxes in the course frame."""
    mp = Path(map_json)
    m = json.loads(mp.read_text())
    meta = json.loads((mp.parent / "meta.json").read_text())
    T = meta["registration"]["T_course_map"]
    return C.boxes_from_occupancy(m["data"], m["width"], m["height"], m["resolution"],
                                  tuple(m["origin"]), to_course=tuple(T))


LD19_RANGE_M = (0.10, 12.0)


def _apply_lidar_profile(stage, near, far):
    """Set every OmniLidar under the rover to [near, far] metres (in memory)."""
    from pxr import Sdf, Vt
    n = 0
    for prim in stage.Traverse():
        if prim.GetTypeName() != "OmniLidar" or not str(prim.GetPath()).startswith("/World/ugv_rover"):
            continue
        before = {}
        for name, value in (("omni:sensor:Core:nearRangeM", near), ("omni:sensor:Core:farRangeM", far)):
            a = prim.GetAttribute(name)
            before[name] = a.Get() if a and a.IsValid() else None
            if not (a and a.IsValid()):
                a = prim.CreateAttribute(name, Sdf.ValueTypeNames.Float)
            a.Set(float(value))
        for name, value in (("omni:sensor:Core:rangesMinM", near), ("omni:sensor:Core:rangesMaxM", far)):
            a = prim.GetAttribute(name)
            cur = a.Get() if a and a.IsValid() else None
            before[name] = list(cur) if cur is not None else None
            if cur is not None and len(cur):
                a.Set(Vt.FloatArray([float(value)] * len(cur)))
        print(f"[run_nav_course] lidar {prim.GetPath()}: range -> {near}-{far} m (was {before})", flush=True)
        n += 1
    if not n:
        print("[run_nav_course] WARNING: no OmniLidar prim found - lidar range NOT changed", flush=True)


def _make_setup(course, extra_boxes, lidar_profile="ld19"):
    def setup(simulation_app, stage):
        from pxr import Gf, Usd, UsdGeom, UsdPhysics

        if lidar_profile == "ld19":
            _apply_lidar_profile(stage, *LD19_RANGE_M)

        base = stage.GetPrimAtPath(ROVER_ARTICULATION_PRIM_PATH)
        m = UsdGeom.Xformable(base).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
        t = m.ExtractTranslation()
        fwd = m.TransformDir(Gf.Vec3d(1, 0, 0))
        spawn = (t[0], t[1], math.atan2(fwd[1], fwd[0]))
        T_world_course = C.compose(spawn, C.inverse(course.start))
        UsdGeom.Xform.Define(stage, COURSE_PRIM)
        boxes = list(course.boxes) + list(extra_boxes)
        for i, b in enumerate(boxes):
            wx, wy, wyaw = C.compose(T_world_course, (b.cx, b.cy, b.yaw))
            path = f"{COURSE_PRIM}/{b.kind}_{i:03d}_{''.join(ch if ch.isalnum() else '_' for ch in b.name)}"
            cube = UsdGeom.Cube.Define(stage, path)
            cube.CreateSizeAttr(1.0)
            xf = UsdGeom.Xformable(cube)
            xf.ClearXformOpOrder()
            xf.AddTranslateOp().Set(Gf.Vec3d(wx, wy, b.sz / 2.0))
            xf.AddRotateZOp().Set(math.degrees(wyaw))
            xf.AddScaleOp().Set(Gf.Vec3f(b.sx, b.sy, b.sz))
            cube.CreateDisplayColorAttr([Gf.Vec3f(*COLORS.get(b.kind, COLORS["box"]))])
            UsdPhysics.CollisionAPI.Apply(cube.GetPrim())
        print(f"[run_nav_course] course '{course.name}': {len(boxes)} obstacles "
              f"({len(extra_boxes)} from the map); rover spawn {tuple(round(v, 3) for v in spawn)} "
              f"= course start {tuple(round(v, 3) for v in course.start)}", flush=True)
    return setup


if __name__ == "__main__":
    args = _parse_args()
    course = C.load(args.course)
    extra = _map_boxes(args.world_map) if args.world_map else []
    raise SystemExit(run(
        environment_usd=None,
        headless=args.headless,
        camera_topics=args.camera_topics,
        camera_resolution=args.camera_res,
        pre_play_setup=_make_setup(course, extra, args.lidar_profile),
        calibration=args.calib,
        friction_combine_mode=args.combine_mode,
    ))
