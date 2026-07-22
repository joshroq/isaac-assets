#!/usr/bin/env python3
"""Author the CAMERA PUBLISH RATE into jetson_rover_v2.usd (attribute-only, standalone pxr).

WHY
    The baked `/ROS2_Graphs/ROS_Camera` graph leaves `inputs:frameSkipCount` UNAUTHORED on
    every camera publisher, which means "skip nothing" — so the RGB, depth and camera_info
    helpers try to publish on EVERY rendered frame. `common/run_baked_rover.py` renders at
    RENDERING_DT = 1/60 (physics 1/240, render every 4th step), so the cameras are asking
    for 60 fps.

    The real rover's camera is 30 fps: `ugv_ws/src/ugv_main/ugv_vision/config/params.yaml`
    -> `framerate: 30.0` (at 640x480). That is the only frame-rate figure in ugv_ws — the
    OAK-D-Lite config (`oak_d_lite.yaml`) sets none and takes the depthai default. So the
    sim was publishing at twice the hardware rate.

HOW
    publish_rate = render_rate / (frameSkipCount + 1)
    => frameSkipCount = round(render_rate / target_fps) - 1
    At the baked 60 Hz render rate, 30 fps => frameSkipCount = 1 (publish every 2nd frame).

MEASURED ON THIS MACHINE (RTX 5060 Ti, headless RayTracedLighting, 2026-07-14) — READ THIS
BEFORE "CAPPING" ANYTHING. The cameras never actually reach 60 fps: image delivery is
bounded by RTX render/annotator throughput, not by the graph. Sim-time rates:

    frameSkipCount   RGB      depth    camera_info
    0 (every render) 17.1 fps 11.4 fps 34.3 fps     <-- current setting
    1 ("cap at 30")   5.7 fps  5.7 fps 17.1 fps

    So the 30 fps hardware limit is NOT the binding constraint here — the GPU is. Applying
    the cap does not hold the camera at 30; it throttles an already-starved 17 fps down to
    5.7, because the skip discards renders that DID carry a fresh image. Rendering at 30 Hz
    instead (RENDERING_DT = 1/30 + skip 0) was also tried and is worse still (5.3 fps): the
    limit is wall-clock image throughput, so fewer renders per sim-second just lowers the
    sim-time rate.

    => frameSkipCount is therefore left at 0: the closest we can get to 30 fps without
    exceeding it. Re-apply the cap with `--fps 30` only if the render pipeline ever gets
    fast enough to exceed 30 fps (cheaper renderer, lower render-product resolution, or a
    faster GPU) — at that point the cap becomes meaningful. Verify with the sim-time stamp
    deltas, NOT `ros2 topic hz` (which reports wall time and the sim runs at ~0.35x).

    RESOLUTION IS THE ACTUAL LEVER (2026-07): the binding constraint above is render
    throughput, and throughput scales with PIXELS, not frameSkipCount. The heavy
    nav_cuvslam_nvblox stack renders four render products at once, so at the unauthored
    1280x720 default the depth stream collapses to ~0.1 Hz. `common/run_baked_rover.py`
    now authors the render products to 640x480 in-memory before play (its
    DEFAULT_CAMERA_RESOLUTION / --camera-res flag) — the real lever this note predicted.
    That is separate from and complementary to frameSkipCount here.

USAGE (needs pxr; no Isaac app required — see the shell wrapper printed on ImportError)
    python3 set_camera_fps.py                 # 30 fps (the hardware reference)
    python3 set_camera_fps.py --fps 15
    python3 set_camera_fps.py --dry-run       # report only, author nothing
"""

import argparse
import datetime
import shutil
import sys

try:
    from pxr import Usd
except ImportError:
    sys.exit(
        "pxr not importable. Run with Isaac's USD libs, e.g.:\n"
        "  PXR=$HOME/Applications/isaacsim/extscache/omni.usd.libs-1.0.3+6312fa25.lx64.r.cp312\n"
        "  PYTHONPATH=$PXR LD_LIBRARY_PATH=$PXR/bin \\\n"
        "    $HOME/Applications/isaacsim/kit/python/bin/python3 set_camera_fps.py"
    )

DEFAULT_USD = "/home/joshroq/projects/isaac-assets/jetson_rover_v2.usd"
CAMERA_GRAPH = "/ROS2_Graphs/ROS_Camera"

# run_baked_rover.py: PHYSICS_DT = 1/240, RENDERING_DT = 1/60 -> the app renders at 60 Hz,
# and a camera helper can only publish on a rendered frame.
RENDER_HZ = 60.0

# The reference rate: ugv_ws/src/ugv_main/ugv_vision/config/params.yaml -> framerate: 30.0
HARDWARE_FPS = 30.0

# Every publisher fed by the camera render products. Keeping them on ONE skip count keeps
# their timestamps aligned.
PUBLISHER_NODES = (
    "RGBPublish",          # ROS2CameraHelper      -> /ugv_sim/oak/rgb/image_rect
    "DepthPublish",        # ROS2CameraHelper      -> /ugv_sim/oak/stereo/image_raw
    "DepthPclPublish",     # ROS2CameraHelper      -> /ugv_sim/oak/stereo/depth_pcl (disabled)
    "CameraInfoPublish",   # ROS2CameraInfoHelper  -> /ugv_sim/oak/rgb/camera_info
)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--usd", default=DEFAULT_USD)
    ap.add_argument("--fps", type=float, default=HARDWARE_FPS,
                    help=f"target camera publish rate (default {HARDWARE_FPS}, the ugv_ws "
                         f"hardware reference)")
    ap.add_argument("--render-hz", type=float, default=RENDER_HZ,
                    help=f"app render rate from run_baked_rover.py (default {RENDER_HZ})")
    ap.add_argument("--dry-run", action="store_true")
    args = ap.parse_args()

    if args.fps <= 0:
        sys.exit("--fps must be > 0")
    if args.fps > args.render_hz:
        sys.exit(f"--fps {args.fps} exceeds the render rate ({args.render_hz} Hz): a camera "
                 f"cannot publish faster than the app renders. Lower --fps, or raise "
                 f"RENDERING_DT in common/run_baked_rover.py.")

    skip = int(round(args.render_hz / args.fps)) - 1
    if skip < 0:
        skip = 0
    effective = args.render_hz / (skip + 1)
    print(f"target {args.fps} fps @ {args.render_hz} Hz render "
          f"-> frameSkipCount = {skip}  (effective {effective:g} fps)")
    if abs(effective - args.fps) > 1e-6:
        print(f"  NOTE: {args.fps} does not divide the render rate; nearest achievable is "
              f"{effective:g} fps.")

    stage = Usd.Stage.Open(args.usd)
    if stage is None:
        sys.exit(f"could not open {args.usd}")

    changes = []
    for name in PUBLISHER_NODES:
        prim = stage.GetPrimAtPath(f"{CAMERA_GRAPH}/{name}")
        if not prim or not prim.IsValid():
            print(f"  WARN  {name}: prim not found, skipping")
            continue
        attr = prim.GetAttribute("inputs:frameSkipCount")
        if not attr:
            print(f"  WARN  {name}: no inputs:frameSkipCount, skipping")
            continue
        before = attr.Get()
        if before == skip:
            print(f"  ok    {name:20s} frameSkipCount already {skip}")
            continue
        changes.append((name, before, attr))

    if not changes:
        print("Nothing to change.")
        return 0

    if args.dry_run:
        for name, before, _ in changes:
            print(f"  DRY   {name:20s} {before!r} -> {skip}")
        return 0

    backup = f"{args.usd}.bak-{datetime.datetime.now():%Y%m%d-%H%M%S}-camfps"
    shutil.copy2(args.usd, backup)
    print(f"backup: {backup}")

    for name, before, attr in changes:
        attr.Set(skip)
        print(f"  SET   {name:20s} {before!r} -> {attr.Get()}")

    stage.GetRootLayer().Save()
    print(f"saved {args.usd}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
