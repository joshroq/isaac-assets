"""
Core Isaac Sim runner for the BAKED-GRAPH rover (jetson_rover_new.usd).

Unlike the original `ugv_controller/common/ugv_sim.py`, this runner builds NO
OmniGraphs — everything (drive + sensors) is baked into the USD. The baked
skid-steer Drive_Graph drives the rover from /cmd_vel by default.

NOTE (2026-07): the baked Drive_Graph/SkidSteer_Controller was originally a
corrupted GUI-built graph (no OnPlaybackTick tick source, dangling node refs,
doubled/mis-wired connections, wrong articulation target) that stalled Kit's run
loop after play. It has been rebuilt in place (see tools/repair_drive_graph.py)
to the proven v1 topology: OnPlaybackTick -> ROS2SubscribeTwist(/cmd_vel) ->
DifferentialController -> two IsaacArticulationControllers driving the four
corner wheels on /World/ugv_rover/base_footprint. The skid-steer kinematics
(the DifferentialController node) live entirely in the USD, so the rover is
self-contained: it takes a raw /cmd_vel Twist and drives itself. Python does NOT
compute or apply any wheel velocities.

This runner therefore has to:

    1. enable the Isaac ROS 2 Bridge + sensor extensions (so the baked ROS 2
       graph nodes actually execute),
    2. open the rover USD as the stage (it is self-contained: rover + graphs +
       physics scene + light),
    3. optionally compose an environment (a warehouse) into it,
    4. configure the optional accessory controls,
    5. play the sim and step it.

The rover is driven EXCLUSIVELY by whatever publishes geometry_msgs/Twist on
/cmd_vel — either the ROS 2 keyboard teleop (see ../keyboard/) or Nav2's
controller output (see ../nav_systems/).

WHY THE ROVER USD IS OPENED, NOT REFERENCED
    jetson_rover_new.usd has TWO top-level roots: /World (the rover + its
    Drive_Graph) and /ROS2_Graphs (all the sensor/clock/odom/cmd_vel graphs),
    plus its own /physicsScene and /Environment light. USD references only pull
    in the *default prim's* subtree (/World), which would silently drop
    /ROS2_Graphs and kill every ROS 2 topic. So we OPEN the rover USD as the base
    stage and reference the environment INTO it, rather than the other way round.

LAUNCH
    Do not run this module directly. Use a scenario launcher:
        isaacpy ugv_controller_v2/run_small_warehouse.py
        isaacpy ugv_controller_v2/run_ground_plane.py
    (isaacpy scrubs system ROS so Isaac's bundled ROS 2 bridge loads cleanly.)
"""

from __future__ import annotations

import os
import sys
from pathlib import Path
from typing import Callable, Iterable


# ── Paths ────────────────────────────────────────────────────────────────────
# This module lives in <repo>/ugv_controller_v2/common/. The shared USD assets
# (jetson_rover_new.usd, *_warehouse.usd) live at the repo root — resolve it by
# walking up until we find the baked rover USD, so launchers never hard-code
# parent counts.
SCRIPT_DIR = Path(__file__).resolve().parent
# The baked rover USD. `jetson_rover_v2.usd` is the current name; the others are
# accepted as fallbacks so the runner keeps working across any rename.
ROVER_USD_NAMES = ("jetson_rover_v2.usd", "ugv_rover_v2.usd", "jetson_rover_new.usd")


def _find_rover_usd(start: Path) -> tuple[Path, Path]:
    """(asset_root, rover_usd_path) for the first baked rover USD found walking up."""
    for candidate in (start, *start.parents):
        for name in ROVER_USD_NAMES:
            if (candidate / name).exists():
                return candidate, candidate / name
    # Fall back to the preferred name under the old layout assumption.
    return start.parent, start.parent / ROVER_USD_NAMES[0]


ASSET_ROOT, ROVER_USD = _find_rover_usd(SCRIPT_DIR)


def asset_path(name: str) -> Path:
    """Absolute path to a shared asset (USD) at the repo root, e.g. a warehouse."""
    return ASSET_ROOT / name


# The rover USD's own roots (see module docstring). We reference the environment
# under /World so it shares the rover's stage, physics scene, and graphs.
ROVER_PRIM_PATH = "/World/ugv_rover"
# The articulation root (carries PhysicsArticulationRootAPI). Used by the accessory
# controller to command the pan/tilt arm DOFs (the wheels stay with the baked graph).
ROVER_ARTICULATION_PRIM_PATH = "/World/ugv_rover/base_footprint"
ENVIRONMENT_PRIM_PATH = "/World/NavEnvironment"

# On-board camera prims used by the windowed helper viewports. Use the baked RGB
# camera for the front UI pane; the pseudo-depth camera still feeds the ROS depth
# render product from the USD graph.
FRONT_CAMERA_PRIM_PATH = "/World/ugv_rover/front_camera_link/Realsense/RSD455/Camera_OmniVision_OV9782_Color"
ARM_CAMERA_PRIM_PATH = "/World/ugv_rover/pt_camera_link/Rover_Camera"

# ── Simulation timing ────────────────────────────────────────────────────────
# Match the original runner: step physics at 1/240 (4 substeps per 1/60 render
# frame) for stable six-wheel scrub contact, matching the real rover's Gazebo
# contact fidelity without rendering at 240 Hz.
PHYSICS_DT = 1.0 / 240.0
RENDERING_DT = 1.0 / 60.0
RENDER_EVERY_N_PHYSICS_STEPS = max(1, round(RENDERING_DT / PHYSICS_DT))

# ── Camera render-product resolution ─────────────────────────────────────────
# The baked /ROS2_Graphs/ROS_Camera render products leave inputs:width/height
# UNAUTHORED, so Isaac falls back to its 1280x720 default. This stack renders up
# to FOUR camera render products at once (color + pseudo-depth for nvblox, plus
# the left/right OV9782 stereo pair for cuVSLAM) on top of the RTX lidar, while
# cuVSLAM + nvblox also run on the same GPU. At 1280x720 x4 the RTX depth/color
# annotators are wall-clock bound and delivery collapses (measured ~0.1-2 Hz),
# which starves nvblox (empty costmap) and makes cuVSLAM's map->odom jittery.
#
# The real rover's OAK-D-Lite is 640x480 (ugv_ws/.../ugv_vision/config/params.yaml
# and camera_info.yaml; the ORB-SLAM3 reference isaac_rgbd.yaml is cx=320/cy=240
# at 640x480). So 640x480 is BOTH the intended hardware resolution and ~4x fewer
# pixels per product. We author it into the render products in-memory before play
# (nothing is written back to the shared USD). Every SLAM consumer reads the live
# camera_info, so intrinsics stay self-consistent at any resolution.
DEFAULT_CAMERA_RESOLUTION = (640, 480)

# ── ROS 2 configuration ──────────────────────────────────────────────────────
# The baked ROS 2 graphs read ROS_DOMAIN_ID / RMW from the environment. These
# must match the terminal running Nav2 / the keyboard teleop so DDS discovers
# both sides. /cmd_vel is the ONLY topic Isaac subscribes to; everything else it
# publishes.
ROS_DOMAIN_ID = 0
ROS_DISTRO = "humble"
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"

# Extensions the baked graphs depend on. Enabling them is what makes the
# OmniGraph ROS 2 / lidar / articulation nodes run once the sim plays.
REQUIRED_EXTENSIONS = (
    "isaacsim.ros2.bridge",              # ROS2Publish*/Subscribe* nodes + /clock
    "isaacsim.robot.wheeled_robots",     # DifferentialController in the Drive_Graph
    "isaacsim.core.nodes",               # IsaacArticulationController, ComputeOdometry, TF
    "isaacsim.sensors.rtx",              # RTX lidar runtime (ROS2RtxLidarHelper)
    "isaacsim.sensors.rtx.nodes",        # RTX lidar debug-draw writer
    "isaacsim.sensors.physics",          # IMU sensor runtime backend (ROS_IMU)
    "isaacsim.sensors.physics.nodes",    # IsaacReadIMU OGN node in the ROS_IMU graph
    "omni.graph.nodes",                  # generic math/array/constant nodes
)


def _configure_ros_environment() -> None:
    os.environ["ROS_DISTRO"] = ROS_DISTRO
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    os.environ["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    os.environ.pop("ROS_DISCOVERY_SERVER", None)


def _resolve_usd(path: Path, label: str) -> Path:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"{label} USD not found: {path}")
    if path.stat().st_size == 0:
        raise ValueError(f"{label} USD is empty (0 bytes): {path}")
    return path


def run(
    environment_usd: str | os.PathLike | None = None,
    headless: bool = False,
    camera_topics: bool = True,
    pre_play_setup: Callable | None = None,
    accessories_enabled: bool = True,
    viewports_enabled: bool = True,
    render_every_physics_step: bool = False,
    disabled_graph_paths: Iterable[str] = (),
    camera_resolution: tuple[int, int] | None = DEFAULT_CAMERA_RESOLUTION,
) -> int:
    """
    Launch the baked-graph rover simulation.

    The rover drives itself entirely from the baked USD: the skid-steer
    Drive_Graph subscribes to /cmd_vel and converts each Twist into wheel
    velocities (see the module docstring). Python builds nothing and drives
    nothing — it only opens the stage, composes the environment, and steps.

    Parameters
    ----------
    environment_usd : path or None
        A warehouse (or other) environment USD to compose INTO the rover stage
        under `/World/NavEnvironment`. When None, a default ground plane is added
        so the rover has a floor to drive on.
    headless : bool
        When True, run with no Isaac window. The RTX lidar and cameras still
        render offscreen, so all ROS 2 topics publish and Nav2 / keyboard teleop
        can drive the rover; there is simply no viewport. Use this when a windowed
        launch is unreliable (e.g. a remote-desktop server hijacks the display).
    camera_topics : bool
        When True, enable the baked RGB/depth/camera_info publishers. Enabled by
        default so RGB navigation can attach without relaunching Isaac. Pass
        False for lidar-only performance/debug sessions if the extra camera
        render products cause driver or windowed Kit load.
    pre_play_setup : callable or None
        Optional scenario hook called after the environment is composed and before
        World.reset(). It is intended for stage-only setup such as Replicator
        obstacle scattering; it must not build drive or navigation graphs.
    accessories_enabled : bool
        When False, skip the windowed Isaac-side camera-arm/light controller.
        This is intended for diagnostics only; driving and ROS 2 publishing are
        unaffected.
    viewports_enabled : bool
        When False, skip the extra on-screen arm/front camera viewports. This
        does not disable the baked ROS camera/depth topics.
    render_every_physics_step : bool
        Diagnostic mode: render on every physics step instead of every fourth
        step. Slower, but useful when isolating PhysX/RTX/SyntheticData timing.
    disabled_graph_paths : iterable[str]
        Diagnostic only: baked OmniGraph prim paths to deactivate before play.
        Defaults empty so the normal v2 graph setup is unchanged.
    camera_resolution : (width, height) or None
        Resolution to author on every baked camera render product before play,
        in-memory (the shared USD is never modified). Defaults to 640x480 — the
        real OAK-D-Lite resolution and ~4x fewer pixels than Isaac's 1280x720
        fallback, which is what makes the RGB/depth/stereo streams publish at a
        usable rate when cuVSLAM + nvblox share the GPU. Pass None to leave the
        render products unauthored (Isaac's 1280x720 default).
    """
    rover_usd = _resolve_usd(ROVER_USD, "Baked rover")
    environment_usd = (
        _resolve_usd(Path(environment_usd), "Environment")
        if environment_usd is not None
        else None
    )

    # ROS environment must be set before any Isaac Sim / ROS module is imported.
    _configure_ros_environment()

    from isaacsim import SimulationApp

    simulation_app = SimulationApp(
        {
            "headless": headless,
            "renderer": "RayTracedLighting",
            "multi_gpu": False,
            "active_gpu": 0,
        }
    )
    accessory = None
    try:
        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))

        # Enable the extensions the baked graphs need, then let them settle.
        from isaacsim.core.utils.extensions import enable_extension

        for extension_name in REQUIRED_EXTENSIONS:
            enable_extension(extension_name)
        for _ in range(2):
            simulation_app.update()

        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage

        # ── Open the SELF-CONTAINED rover USD as the base stage ─────────────────
        # This brings in the rover, the Drive_Graph, every /ROS2_Graphs node, the
        # physics scene, and the default light in one shot.
        if not omni.usd.get_context().open_stage(str(rover_usd)):
            raise RuntimeError(f"Could not open baked rover USD: {rover_usd}")
        for _ in range(5):
            simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not open the rover USD stage.")

        if not stage.GetPrimAtPath(ROVER_PRIM_PATH).IsValid():
            raise RuntimeError(
                f"Rover prim {ROVER_PRIM_PATH} missing from {rover_usd}. "
                "Was the baked USD moved or re-authored under a different root?"
            )

        # ── Compose the environment INTO the rover stage (or add a ground) ──────
        if environment_usd is not None:
            add_reference_to_stage(
                usd_path=str(environment_usd), prim_path=ENVIRONMENT_PRIM_PATH
            )
            for _ in range(5):
                simulation_app.update()

        if pre_play_setup is not None:
            pre_play_setup(simulation_app=simulation_app, stage=stage)

        _disable_graph_prims(stage, disabled_graph_paths)

        print("[run_baked_rover] Creating World.", flush=True)
        world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)
        if environment_usd is None:
            print("[run_baked_rover] Adding default ground plane.", flush=True)
            world.scene.add_default_ground_plane()

        _configure_camera_topics(stage, enabled=camera_topics,
                                 resolution=camera_resolution)

        # ── Accessory controls (camera arm + lights) — windowed only ───────────
        # Driving stays 100% ROS 2 /cmd_vel (baked graph). This only adds the
        # non-drive accessories: arrow keys pan/tilt the arm, Y resets it, H/B/N
        # toggle the lights (which start OFF). Its camera-joint drive mode must be
        # set BEFORE play, so create it here (pre-reset).
        accessory = None
        if not headless and accessories_enabled:
            try:
                from accessory_controller import AccessoryController

                accessory = AccessoryController(ROVER_ARTICULATION_PRIM_PATH)
                accessory.configure_camera_drives()
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: accessory controls unavailable:")
                traceback.print_exc()
                accessory = None

        # ── Start the sim. The baked graphs begin publishing/subscribing here ──
        print("[run_baked_rover] Resetting world.", flush=True)
        world.reset()
        print("[run_baked_rover] World reset complete.", flush=True)
        print("[run_baked_rover] Starting manual simulation stepping.", flush=True)
        try:
            from isaacsim.core.rendering_manager import RenderingManager

            RenderingManager._ensure_fabric_simulation_time()
        except Exception:
            import traceback
            print("[run_baked_rover] Warning: could not seed Fabric simulation time:")
            traceback.print_exc()

        # Bind the accessory articulation now that the sim is playing.
        if accessory is not None:
            try:
                accessory.initialize()
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: accessory init failed:")
                traceback.print_exc()
                accessory = None

        # ── On-launch camera perspectives (windowed only) ──────────────────────
        # Arm camera (top) + front RGB camera (bottom), docked in the left third.
        if not headless and viewports_enabled:
            try:
                from viewport_setup import setup_nav_viewports

                setup_nav_viewports(
                    robot_prim_path=ROVER_PRIM_PATH,
                    front_camera_prim=FRONT_CAMERA_PRIM_PATH,
                    arm_camera_prim=ARM_CAMERA_PRIM_PATH,
                )
                # The viewport docking is deferred a few app-update events; pump a
                # handful now so it docks into the left-third layout before the
                # manual step loop (whose render cadence pumps updates only every
                # Nth step) begins.
                for _ in range(8):
                    simulation_app.update()
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: could not open camera viewports:")
                traceback.print_exc()

        print("[run_baked_rover] Setup complete.", flush=True)
        print(f"[run_baked_rover] Rover USD:       {rover_usd}", flush=True)
        print(f"[run_baked_rover] Environment USD: {environment_usd or 'none (default ground plane)'}", flush=True)
        print(f"[run_baked_rover] ROS domain id:   {ROS_DOMAIN_ID}", flush=True)
        print(f"[run_baked_rover] Window:          {'headless' if headless else 'visible'}", flush=True)
        print(f"[run_baked_rover] Camera topics:   {'enabled' if camera_topics else 'disabled'}", flush=True)
        disabled_graph_paths = tuple(disabled_graph_paths)
        if disabled_graph_paths:
            print("[run_baked_rover] Disabled graph prims:", flush=True)
            for graph_path in disabled_graph_paths:
                print(f"[run_baked_rover]   {graph_path}", flush=True)
        print("[run_baked_rover] Drive: ROS 2 /cmd_vel (Twist) -> baked skid-steer Drive_Graph.", flush=True)
        print("[run_baked_rover]   keyboard drive : ../keyboard/keyboard_drive.sh", flush=True)
        print("[run_baked_rover]   Nav2           : ../nav_systems/<system>/  (2D Nav Goal in RViz)", flush=True)
        if accessory is not None:
            print("[run_baked_rover] Accessories (Isaac window): arrows pan/tilt arm, Y reset, "
                  "H/B/N lights.", flush=True)

        step_count = 0
        render_interval = 1 if render_every_physics_step else RENDER_EVERY_N_PHYSICS_STEPS
        print(
            "[run_baked_rover] Entering simulation loop "
            f"(render every {render_interval} physics steps).",
            flush=True,
        )
        while simulation_app.is_running():
            step_count += 1
            world.step(render=(step_count % render_interval == 0))
            if accessory is not None:
                accessory.update(PHYSICS_DT)

    finally:
        if accessory is not None:
            accessory.shutdown()
        simulation_app.close()

    return 0


def _disable_graph_prims(stage, graph_paths: Iterable[str]) -> None:
    """Deactivate selected baked graph prims for diagnostics."""
    for graph_path in tuple(graph_paths):
        prim = stage.GetPrimAtPath(graph_path)
        if not prim.IsValid():
            print(f"[run_baked_rover] Warning: graph prim not found: {graph_path}", flush=True)
            continue
        prim.SetActive(False)


def _configure_camera_topics(stage, enabled: bool,
                             resolution: tuple[int, int] | None = None) -> None:
    """Enable/disable baked camera render products and ROS image publishers, and
    (optionally) author their render resolution before play."""
    # DepthPclPublish is intentionally NOT listed: it is authored disabled in the
    # USD (the RGB-D cloud is built ROS-side by rtabmap point_cloud_xyzrgb), and
    # listing it here would force-enable it whenever camera topics are on.
    camera_nodes = (
        "/ROS2_Graphs/ROS_Camera/RenderProduct",
        "/ROS2_Graphs/ROS_Camera/RenderProduct_01",
        "/ROS2_Graphs/ROS_Camera/RGBPublish",
        "/ROS2_Graphs/ROS_Camera/DepthPublish",
        "/ROS2_Graphs/ROS_Camera/CameraInfoPublish",
        # Stereo pair for cuVSLAM (nav_cuvslam_nvblox); left/right OV9782 eyes.
        "/ROS2_Graphs/ROS_Camera/RenderProduct_Left",
        "/ROS2_Graphs/ROS_Camera/RenderProduct_Right",
        "/ROS2_Graphs/ROS_Camera/LeftRGBPublish",
        "/ROS2_Graphs/ROS_Camera/LeftCameraInfoPublish",
        "/ROS2_Graphs/ROS_Camera/RightRGBPublish",
        "/ROS2_Graphs/ROS_Camera/RightCameraInfoPublish",
    )
    for node_path in camera_nodes:
        prim = stage.GetPrimAtPath(node_path)
        if not prim.IsValid():
            continue
        attr = prim.GetAttribute("inputs:enabled")
        if attr.IsValid():
            attr.Set(enabled)

    # Author render-product resolution on the four RenderProduct nodes. All four
    # share one resolution so nvblox's shared color/depth camera_info stays valid
    # and the cuVSLAM left/right pair matches. In-memory only (never saved).
    if enabled and resolution is not None:
        width, height = int(resolution[0]), int(resolution[1])
        render_products = (
            "/ROS2_Graphs/ROS_Camera/RenderProduct",       # pseudo-depth (nvblox)
            "/ROS2_Graphs/ROS_Camera/RenderProduct_01",    # color (nvblox + RGB nav)
            "/ROS2_Graphs/ROS_Camera/RenderProduct_Left",  # cuVSLAM left eye
            "/ROS2_Graphs/ROS_Camera/RenderProduct_Right", # cuVSLAM right eye
        )
        applied = 0
        for node_path in render_products:
            prim = stage.GetPrimAtPath(node_path)
            if not prim.IsValid():
                continue
            w_attr = prim.GetAttribute("inputs:width")
            h_attr = prim.GetAttribute("inputs:height")
            if w_attr.IsValid() and h_attr.IsValid():
                w_attr.Set(width)
                h_attr.Set(height)
                applied += 1
        print(f"[run_baked_rover] Camera render resolution: {width}x{height} "
              f"({applied} render products).", flush=True)
