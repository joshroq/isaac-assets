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
import time
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
# By default we leave the baked /ROS2_Graphs/ROS_Camera render products UNAUTHORED,
# i.e. Isaac's native 1280x720 (DEFAULT_CAMERA_RESOLUTION = None). Authoring a
# non-native, non-matching aspect ratio also forced fx==fy on the CameraInfo
# ("renderer assumes square pixels") and tripped a DLSS minimum-resolution warning,
# so native is the clean default. It remains overridable: pass camera_resolution
# (WxH) / run_small_warehouse.py --camera-res WxH to shrink the render products if a
# small-VRAM GPU runs out of memory (the reduction is applied in-memory before play;
# the shared USD is never modified, and every SLAM consumer reads the live
# camera_info so intrinsics stay self-consistent at whatever resolution is used).
DEFAULT_CAMERA_RESOLUTION = None

# ── ROS 2 configuration ──────────────────────────────────────────────────────
# The baked ROS 2 graphs read ROS_DOMAIN_ID / RMW from the environment. These
# must match the terminal running Nav2 / the keyboard teleop so DDS discovers
# both sides. /cmd_vel is the ONLY topic Isaac subscribes to; everything else it
# publishes.
ROS_DOMAIN_ID = int(os.environ.get("ROS_DOMAIN_ID", "0"))
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
STAGE_RECORDER_EXTENSION = "omni.kit.stagerecorder.core"


def _configure_ros_environment() -> None:
    os.environ["ROS_DISTRO"] = ROS_DISTRO
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    os.environ["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    os.environ.setdefault("ROS_LOCALHOST_ONLY", "0")
    os.environ.pop("ROS_DISCOVERY_SERVER", None)


def _env_truthy(name: str, default: bool = False) -> bool:
    value = os.environ.get(name)
    if value is None:
        return default
    return value.strip().lower() in ("1", "true", "yes", "on")


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
    calibration: "str | os.PathLike | dict | None" = None,
    friction_combine_mode: str | None = None,
    overhead_camera: "str | os.PathLike | dict | None" = None,
    draw_overhead_path: bool = False,
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
        in-memory (the shared USD is never modified). Pass None to leave the
        render products unauthored (Isaac's 1280x720 default).
    calibration : path, dict, or None
        A sim->real calibration to apply IN-MEMORY before play (the shared USD is
        never modified) — the sim2real optimiser's per-trial theta. Accepts a
        sim_calibration.yaml path or an already-loaded nested dict. Floor-friction
        keys also create + bind a per-surface floor PhysicsMaterial on the ground.
        GEOMETRY is NOT part of this: it is locked in the USD and calib_params refuses to
        author it (wheel_distance, a controller turning gain, is the only exception).
    friction_combine_mode : 'min' | 'average' | 'multiply' | None
        If set, author this frictionCombineMode on the wheel + floor materials.
        'min' makes the floor material ~= the effective contact friction (a clean
        per-surface knob). None leaves the authored mode untouched.
    overhead_camera : path, dict, or None
        A sim2real_calib/overhead camera_setup (yaml path or dict) matched to the real
        overhead webcam. When set, an `/World/OverheadCam` is authored at that pose + FOV
        and (windowed) made the active viewport, so the sim VIEW matches the webcam view.
    draw_overhead_path : bool
        When True (with overhead_camera), trace the rover's ground path as a live debug line
        in the viewport as it drives — the sim analogue of the drawn path in the demo.
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
    recorder = None
    overhead = None
    try:
        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))

        # Enable the extensions the baked graphs need, then let them settle.
        from isaacsim.core.utils.extensions import enable_extension

        for extension_name in REQUIRED_EXTENSIONS:
            enable_extension(extension_name)
        if os.environ.get("ISAAC_STAGE_RECORD_DIR"):
            enable_extension(STAGE_RECORDER_EXTENSION)
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

        # Apply the sim->real calibration IN-MEMORY (never saves the shared USD). Done after
        # the ground plane exists so the floor material can bind, and before world.reset() so
        # every physics attr takes effect at play. Geometry is frozen in the USD, not here.
        if calibration is not None or friction_combine_mode is not None:
            _apply_calibration(stage, calibration, friction_combine_mode)

        # Overhead camera matched to the real webcam (authored pre-reset so the prim exists).
        overhead = None
        if overhead_camera is not None:
            try:
                overhead = _OverheadView(stage, overhead_camera, draw_overhead_path,
                                         ROVER_ARTICULATION_PRIM_PATH)
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: overhead camera setup failed:")
                traceback.print_exc()
                overhead = None

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

        recorder = _StageRecorderGate(
            stage=stage,
            enabled=bool(os.environ.get("ISAAC_STAGE_RECORD_DIR")),
            record_dir=os.environ.get("ISAAC_STAGE_RECORD_DIR", ""),
            take_name=os.environ.get("ISAAC_STAGE_RECORD_NAME", "stage_replay"),
            start_file=os.environ.get("ISAAC_STAGE_RECORD_START_FILE", ""),
            stop_file=os.environ.get("ISAAC_STAGE_RECORD_STOP_FILE", ""),
            done_file=os.environ.get("ISAAC_STAGE_RECORD_DONE_FILE", ""),
            fps=float(os.environ.get("ISAAC_STAGE_RECORD_FPS", "30")),
            exit_on_done=_env_truthy("ISAAC_STAGE_RECORD_EXIT_ON_DONE", True),
            environment_usd=str(environment_usd) if environment_usd is not None else "",
        )

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

        if overhead is not None:
            overhead.activate(headless)

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
        print("[run_baked_rover] Drive: ROS 2 /cmd_vel (Twist) -> baked skid-steer Drive_Graph "
              "-> wheel joint velocity drives (PhysX wheel-floor contact).", flush=True)
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
        if _env_truthy("SIM_ROS"):
            # Optional direct-wheel diagnostic; normal runs use the baked bridge.
            _run_sim_ros(world, stage, simulation_app)
            return 0
        if os.environ.get("SIM_TRIAL_SEQ"):
            # ROS-free sim trial: drive the wheels directly (replicating the baked
            # DifferentialController kinematics with the calibrated params) and record
            # the stage/physics pose to CSV for explicit ROS-free diagnostics.
            _run_sim_trial(world, stage)
            return 0
        if os.environ.get("SIM_DIAG"):
            # Diagnostic: is the timeline actually playing (OnPlaybackTick graphs tick)?
            # and does the rover physics move? Steps, reports, then exits.
            import omni.timeline
            tl = omni.timeline.get_timeline_interface()
            for _ in range(90):
                world.step(render=True)
            print(f"[SIM_DIAG] timeline.is_playing={tl.is_playing()}", flush=True)
            try:
                from pxr import UsdGeom, Usd
                prim = stage.GetPrimAtPath(ROVER_ARTICULATION_PRIM_PATH)
                m = UsdGeom.Xformable(prim).ComputeLocalToWorldTransform(Usd.TimeCode.Default())
                print(f"[SIM_DIAG] base translation={tuple(m.ExtractTranslation())}", flush=True)
            except Exception as e:
                print(f"[SIM_DIAG] pose err: {e}", flush=True)
            return 0
        pace_start = time.monotonic()
        t_pace0 = world.current_time
        pace_realtime = _env_truthy("SIM_REALTIME")
        # A render step (world.step(render=True) -> app.update) advances a whole rendering_dt
        # (4 physics substeps), not one PHYSICS_DT: use the real elapsed sim time.
        t_prev = world.current_time
        dt_last = PHYSICS_DT
        while simulation_app.is_running():
            step_count += 1
            render_now = step_count % render_interval == 0
            world.step(render=render_now)
            t_now = world.current_time
            dt_last = t_now - t_prev if t_now > t_prev else PHYSICS_DT
            t_prev = t_now
            if accessory is not None:
                accessory.update(dt_last)
            if overhead is not None:
                overhead.update()
            recorder.update()
            if recorder.should_exit:
                break
            if pace_realtime and step_count % render_interval == 0:
                delay = (t_now - t_pace0) - (time.monotonic() - pace_start)
                if delay > 0:
                    time.sleep(delay)

    finally:
        if recorder is not None:
            try:
                recorder.shutdown()
            except Exception:
                pass
        if overhead is not None:
            try:
                overhead.shutdown()
            except Exception:
                pass
        if accessory is not None:
            accessory.shutdown()
        simulation_app.close()

    return 0


_DIFF_PATH = "/World/ugv_rover/Drive_Graph/SkidSteer_Controller/DiffController"
_DRIVE_GRAPH = "/World/ugv_rover/Drive_Graph"
_ROS2_GRAPHS = "/ROS2_Graphs"
_CORNER_WHEEL_JOINTS = [
    "left_up_wheel_link_joint", "left_down_wheel_link_joint",
    "right_up_wheel_link_joint", "right_down_wheel_link_joint",
]


def _run_sim_trial(world, stage) -> None:
    """Drive a command sequence by directly commanding the corner wheel joints, and
    record the rover's true physics pose (t,x,y,yaw) to CSV. No ROS involved.

    Env: SIM_TRIAL_SEQ (yaml/json sequence), SIM_TRIAL_OUT (csv, default /tmp/sim_run.csv).
    Replicates Isaac's DifferentialController: per-side wheel rad/s =
      v_side = (2*lin +/- ang*W) / (2*R), clamped by the controller's max speeds — using
    the calibrated wheelRadius/wheelDistance/max* read off the authored DiffController, so
    all kinematic params still matter; the physics drives (damping/maxForce/friction/drag
    from --calib) shape the dynamic response exactly as in the baked graph.
    """
    import csv as _csv
    import json as _json
    import math as _math
    import numpy as _np
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction

    seq_path = os.environ["SIM_TRIAL_SEQ"]
    out_path = os.environ.get("SIM_TRIAL_OUT", "/tmp/sim_run.csv")
    try:
        import yaml
        data = yaml.safe_load(open(seq_path))
    except Exception:
        data = _json.load(open(seq_path))
    steps = data["steps"]

    diff = stage.GetPrimAtPath(_DIFF_PATH)

    def _ga(attr, default):
        try:
            v = diff.GetAttribute(attr).Get()
            return float(v) if v is not None else default
        except Exception:
            return default
    R = _ga("inputs:wheelRadius", 0.04)
    W = _ga("inputs:wheelDistance", 0.5)
    vmax = _ga("inputs:maxLinearSpeed", 0.5)
    wmax = _ga("inputs:maxAngularSpeed", 1.9)

    # Silence the baked graphs so only we command the wheels.
    for gp in (_DRIVE_GRAPH, _ROS2_GRAPHS):
        p = stage.GetPrimAtPath(gp)
        if p and p.IsValid():
            p.SetActive(False)

    robot = SingleArticulation(ROVER_ARTICULATION_PRIM_PATH)
    robot.initialize()
    dof = list(robot.dof_names)
    idx, sign = [], []
    for name in _CORNER_WHEEL_JOINTS:
        if name in dof:
            idx.append(dof.index(name))
            sign.append(-1.0 if name.startswith("left") else 1.0)  # left vs right side
    idx = _np.array(idx, dtype=_np.int32)
    sign = _np.array(sign)
    inv = float(os.environ.get("SIM_TRIAL_WHEEL_SIGN", "1"))  # flip if forward goes backward

    def pose():
        pos, quat = robot.get_world_pose()          # quat = (w, x, y, z)
        w, x, y, z = (float(quat[0]), float(quat[1]), float(quat[2]), float(quat[3]))
        yaw = _math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))
        return float(pos[0]), float(pos[1]), yaw

    dt = PHYSICS_DT
    f = open(out_path, "w")
    f.write("t,x,y,yaw\n")
    print(f"[SIM_TRIAL] R={R} W={W} vmax={vmax} wmax={wmax} dof_idx={idx.tolist()}", flush=True)
    t = 0.0
    t0 = None
    try:
        for s in steps:
            lin = max(-vmax, min(vmax, float(s.get("linear", 0.0))))
            ang = max(-wmax, min(wmax, float(s.get("angular", 0.0))))
            v_left = (2.0 * lin - ang * W) / (2.0 * R)    # rad/s
            v_right = (2.0 * lin + ang * W) / (2.0 * R)
            vels = inv * _np.array([v_left if sg < 0 else v_right for sg in sign])
            n = max(1, int(round(float(s.get("duration", 0.0)) / dt)))
            _k = 0
            for _ in range(n):
                robot.apply_action(ArticulationAction(joint_velocities=vels, joint_indices=idx))
                world.step(render=False)
                x, y, yaw = pose()
                if t0 is None:
                    t0 = t
                f.write(f"{t - t0:.4f},{x:.5f},{y:.5f},{yaw:.5f}\n")
                t += dt
                _k += 1
                if os.environ.get("SIM_TRIAL_DIAG") and _k % 120 == 0:
                    try:
                        av = robot.get_joint_velocities()
                        print(f"[SIM_TRIAL_DIAG] t={t - t0:.1f} cmd={[round(float(v),2) for v in vels]} "
                              f"act_wheel={[round(float(av[i]), 2) for i in idx]} "
                              f"yaw={_math.degrees(yaw):.1f}", flush=True)
                    except Exception as _e:
                        print(f"[SIM_TRIAL_DIAG] {_e}", flush=True)
        # stop
        robot.apply_action(ArticulationAction(joint_velocities=_np.zeros(len(idx)),
                                              joint_indices=idx))
        for _ in range(int(0.5 / dt)):
            world.step(render=False)
    finally:
        f.close()
    print(f"[SIM_TRIAL] wrote {out_path} ({t:.1f}s driven)", flush=True)


def _run_sim_ros(world, stage, simulation_app) -> None:
    """Optional direct-wheel ROS2 diagnostic using in-process rclpy. Subscribes
    SIM_ROS_CMD_TOPIC (default /cmd_vel) -> drives the 4 corner wheels with the
    calibrated DifferentialController kinematics -> publishes SIM_ROS_ODOM_TOPIC
    (default /odom) from the physics pose at ~60 Hz, paced to ~1x real time. External
    ROS2 nodes drive it and read its odom exactly like the native bridge would."""
    import time as _time
    import numpy as _np
    import rclpy
    from geometry_msgs.msg import Twist
    from nav_msgs.msg import Odometry
    from isaacsim.core.prims import SingleArticulation
    from isaacsim.core.utils.types import ArticulationAction

    diff = stage.GetPrimAtPath(_DIFF_PATH)

    def _ga(attr, default):
        try:
            v = diff.GetAttribute(attr).Get()
            return float(v) if v is not None else default
        except Exception:
            return default
    R = _ga("inputs:wheelRadius", 0.04)
    W = _ga("inputs:wheelDistance", 0.5)
    vmax = _ga("inputs:maxLinearSpeed", 0.5)
    wmax = _ga("inputs:maxAngularSpeed", 1.9)

    for gp in (_DRIVE_GRAPH, _ROS2_GRAPHS):
        p = stage.GetPrimAtPath(gp)
        if p and p.IsValid():
            p.SetActive(False)

    robot = SingleArticulation(ROVER_ARTICULATION_PRIM_PATH)
    robot.initialize()
    dof = list(robot.dof_names)
    idx, sign = [], []
    for name in _CORNER_WHEEL_JOINTS:
        if name in dof:
            idx.append(dof.index(name))
            sign.append(-1.0 if name.startswith("left") else 1.0)
    idx = _np.array(idx, dtype=_np.int32)
    sign = _np.array(sign)

    if not rclpy.ok():
        rclpy.init()
    node = rclpy.create_node("isaac_sim_ros")
    cmd = {"lin": 0.0, "ang": 0.0, "t": _time.time()}

    def _on_cmd(m):
        cmd["lin"] = float(m.linear.x)
        cmd["ang"] = float(m.angular.z)
        cmd["t"] = _time.time()
    cmd_topic = os.environ.get("SIM_ROS_CMD_TOPIC", "/cmd_vel")
    odom_topic = os.environ.get("SIM_ROS_ODOM_TOPIC", "/odom")
    node.create_subscription(Twist, cmd_topic, _on_cmd, 10)
    odom_pub = node.create_publisher(Odometry, odom_topic, 10)
    deadman = float(os.environ.get("SIM_ROS_DEADMAN", "0"))  # 0 = off

    print(f"[SIM_ROS] real-time ROS2: sub {cmd_topic} -> drive, pub {odom_topic}; "
          f"R={R} W={W} vmax={vmax} wmax={wmax}", flush=True)

    dt = PHYSICS_DT
    t0 = _time.time()
    sim_t = 0.0
    k = 0
    try:
        while simulation_app.is_running():
            rclpy.spin_once(node, timeout_sec=0.0)
            lin, ang = cmd["lin"], cmd["ang"]
            if deadman > 0 and (_time.time() - cmd["t"]) > deadman:
                lin, ang = 0.0, 0.0
            lin = max(-vmax, min(vmax, lin))
            ang = max(-wmax, min(wmax, ang))
            v_left = (2.0 * lin - ang * W) / (2.0 * R)
            v_right = (2.0 * lin + ang * W) / (2.0 * R)
            vels = _np.array([v_left if sg < 0 else v_right for sg in sign])
            robot.apply_action(ArticulationAction(joint_velocities=vels, joint_indices=idx))
            world.step(render=False)
            sim_t += dt
            k += 1
            if k % 4 == 0:  # ~60 Hz odom
                pos, quat = robot.get_world_pose()
                od = Odometry()
                od.header.stamp = node.get_clock().now().to_msg()
                od.header.frame_id = "odom"
                od.child_frame_id = "base_footprint"
                od.pose.pose.position.x = float(pos[0])
                od.pose.pose.position.y = float(pos[1])
                od.pose.pose.position.z = float(pos[2])
                od.pose.pose.orientation.w = float(quat[0])
                od.pose.pose.orientation.x = float(quat[1])
                od.pose.pose.orientation.y = float(quat[2])
                od.pose.pose.orientation.z = float(quat[3])
                odom_pub.publish(od)
            lag = sim_t - (_time.time() - t0)  # pace to ~1x real time
            if lag > 0:
                _time.sleep(lag)
    except KeyboardInterrupt:
        pass
    finally:
        try:
            node.destroy_node()
        except Exception:
            pass


def _apply_calibration(stage, calibration, combine_mode) -> None:
    """Author a sim->real calibration onto the live stage in-memory (no save).

    Delegates to the shared single-source-of-truth ugv_controller_v2/calibration/calib_params.py
    so the baker (apply_sim_calibration.py) and this in-memory path stay identical.
    """
    calib_dir = SCRIPT_DIR.parent / "calibration"
    if str(calib_dir) not in sys.path:
        sys.path.insert(0, str(calib_dir))
    try:
        import calib_params as C
    except Exception:
        import traceback
        print("[run_baked_rover] Warning: calib_params import failed; calibration skipped.")
        traceback.print_exc()
        return

    flat = {}
    if isinstance(calibration, dict):
        flat = C.flatten(calibration)
    elif calibration is not None:
        path = Path(calibration).expanduser()
        if not path.exists():
            print(f"[run_baked_rover] Warning: calibration file not found: {path}")
        else:
            try:
                import yaml
                flat = C.flatten(yaml.safe_load(path.read_text()) or {})
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: could not parse calibration yaml.")
                traceback.print_exc()

    try:
        changes = C.apply_to_stage(stage, flat, author_floor_material=True,
                                   combine_mode=combine_mode)
    except ValueError:
        # an inadmissible calibration (floor friction outside its literature band, unpaired
        # ramp limits) must fail the run, not silently fall back to the uncalibrated rover
        raise
    except Exception:
        import traceback
        print("[run_baked_rover] Warning: applying calibration failed.")
        traceback.print_exc()
        return
    sets = [c for c in changes if c[0] in ("set", "create", "bind-floor", "connect")]
    warns = [c for c in changes if str(c[0]).startswith("WARN")]
    print(f"[run_baked_rover] Calibration applied in-memory: {len(sets)} attrs"
          + (f", {len(warns)} warnings" if warns else "")
          + (f", combineMode={combine_mode}" if combine_mode else ""), flush=True)
    for w in warns:
        print(f"[run_baked_rover]   {w[0]}: {w[1]} {w[2]}", flush=True)


OVERHEAD_CAM_PATH = "/World/OverheadCam"


class _OverheadView:
    """Author an overhead camera matched to the real webcam and (optionally) trace the path.

    Uses the shared sim2real_calib/overhead/camera_model to turn a camera_setup (height, look_at,
    FOV) into a USD camera pose + focal length, so the Isaac render matches the webcam view. The
    live path is drawn with the debug-draw interface by sampling the articulation base each step.
    """

    def __init__(self, stage, setup_src, draw_path, base_prim_path):
        import math as _math

        from pxr import Gf, UsdGeom

        overhead_dir = SCRIPT_DIR.parents[1] / "sim2real_calib" / "overhead"
        if str(overhead_dir) not in sys.path:
            sys.path.insert(0, str(overhead_dir))
        import camera_model as cammod

        if isinstance(setup_src, dict):
            setup = setup_src
        else:
            import yaml
            setup = yaml.safe_load(Path(setup_src).expanduser().read_text())
        self._setup = setup
        self._cam = cammod.OverheadCamera(setup)
        self._draw_path = bool(draw_path)
        self._base_prim = stage.GetPrimAtPath(base_prim_path)
        self._pts = []
        self._physx = None
        self._draw = None

        translation, quat = self._cam.usd_pose()                 # (w,x,y,z)
        cam = UsdGeom.Camera.Define(stage, OVERHEAD_CAM_PATH)
        xform = UsdGeom.Xformable(cam)
        xform.ClearXformOpOrder()
        xform.AddTranslateOp().Set(Gf.Vec3d(*[float(v) for v in translation]))
        xform.AddOrientOp().Set(Gf.Quatf(float(quat[0]),
                                         Gf.Vec3f(float(quat[1]), float(quat[2]), float(quat[3]))))
        aperture = 36.0                                          # mm (full-frame convention)
        focal = aperture / (2.0 * _math.tan(_math.radians(self._cam.hfov_deg) / 2.0))
        w, h = self._cam.res
        cam.CreateFocalLengthAttr(float(focal))
        cam.CreateHorizontalApertureAttr(float(aperture))
        cam.CreateVerticalApertureAttr(float(aperture * h / w))
        cam.CreateClippingRangeAttr(Gf.Vec2f(0.01, 1000.0))
        print(f"[run_baked_rover] Overhead camera {OVERHEAD_CAM_PATH}: "
              f"height {self._cam.height():.2f} m, tilt {self._cam.tilt_deg():.1f}°, "
              f"hFOV {self._cam.hfov_deg:.1f}° (focal {focal:.1f}mm).", flush=True)

    def activate(self, headless):
        if headless:
            return
        try:
            from omni.kit.viewport.utility import get_active_viewport
            vp = get_active_viewport()
            if vp is not None:
                vp.camera_path = OVERHEAD_CAM_PATH
                print(f"[run_baked_rover] Active viewport -> {OVERHEAD_CAM_PATH}.", flush=True)
        except Exception:
            import traceback
            print("[run_baked_rover] Warning: could not set overhead viewport:")
            traceback.print_exc()
        if self._draw_path:
            try:
                from isaacsim.util.debug_draw import _debug_draw
                self._draw = _debug_draw.acquire_debug_draw_interface()
            except Exception:
                self._draw = None

    def update(self):
        if not self._draw_path or self._base_prim is None or not self._base_prim.IsValid():
            return
        from pxr import Usd, UsdGeom
        try:
            if self._physx is None:
                import omni.physx
                self._physx = omni.physx.get_physx_interface()
            self._physx.update_transformations(False, True, False)
            m = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(self._base_prim)
            p = m.ExtractTranslation()
            pt = (float(p[0]), float(p[1]), 0.01)
            if not self._pts or (abs(pt[0] - self._pts[-1][0]) + abs(pt[1] - self._pts[-1][1])) > 0.002:
                self._pts.append(pt)
            if self._draw is not None and len(self._pts) >= 2:
                self._draw.clear_lines()
                starts, ends = self._pts[:-1], self._pts[1:]
                cols = [(0.0, 1.0, 0.0, 1.0)] * len(starts)
                self._draw.draw_lines(starts, ends, cols, [3.0] * len(starts))
        except Exception:
            pass

    def shutdown(self):
        try:
            if self._draw is not None:
                self._draw.clear_lines()
        except Exception:
            pass


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


class _StageRecorderGate:
    """Bracket Isaac Stage Recorder capture with files created by run_nav_trial."""

    def __init__(
        self,
        stage,
        enabled: bool,
        record_dir: str,
        take_name: str,
        start_file: str,
        stop_file: str,
        done_file: str,
        fps: float,
        exit_on_done: bool,
        environment_usd: str,
    ) -> None:
        self.enabled = enabled
        self.stage = stage
        self.record_dir = Path(record_dir).expanduser().resolve() if record_dir else None
        self.take_name = take_name
        self.start_file = Path(start_file).expanduser().resolve() if start_file else None
        self.stop_file = Path(stop_file).expanduser().resolve() if stop_file else None
        self.done_file = Path(done_file).expanduser().resolve() if done_file else None
        self.fps = fps
        self.exit_on_done = exit_on_done
        self.environment_usd = environment_usd
        self.started = False
        self.stopped = False
        self.should_exit = False
        self._last_sample_wall = 0.0
        self._samples = {}
        self._sample_index = 0
        self._xform_paths = []
        self._physx = None
        self._rover_xform = None
        self._stage_recorder_started = False

        if not self.enabled:
            return
        self.record_dir.mkdir(parents=True, exist_ok=True)
        self._xform_paths = [
            str(p.GetPath())
            for p in self.stage.Traverse()
            if str(p.GetPath()).startswith(ROVER_PRIM_PATH)
        ]
        print(
            f"[run_baked_rover] Stage Recorder armed: {self.record_dir / (self.take_name + '.usd')}",
            flush=True,
        )

    def update(self) -> None:
        if not self.enabled or self.stopped:
            return
        if not self.started:
            if self.start_file and self.start_file.exists():
                self._start()
            return
        self._sample_custom_take()
        if self.stop_file and self.stop_file.exists():
            self._stop()

    def shutdown(self) -> None:
        if self.enabled and self.started and not self.stopped:
            self._stop()

    def _start(self) -> None:
        import carb

        settings = carb.settings.get_settings()
        settings.set_bool("/physics/updateToUsd", True)
        settings.set_bool("/persistent/omnigraph/updateToUsd", True)
        self.started = True
        self._last_sample_wall = 0.0
        self._sample_index = 0
        # We author the captured trajectory on the rover ROOT, but must SAMPLE the
        # articulation root base_footprint — that is the prim physics actually moves.
        # /World/ugv_rover is a static parent (sampling it gives all zeros).
        self._samples = {ROVER_PRIM_PATH: []}
        self._base_prim = self.stage.GetPrimAtPath(ROVER_ARTICULATION_PRIM_PATH)
        # The built-in Isaac Stage Recorder writes an EMPTY take here: it records USD notices
        # but PhysX transforms live in Fabric, so it captures nothing (581-byte file) even
        # though it returns RECORDING_SUCCESS. Our own sampled take below is the real capture,
        # so the built-in recorder is OFF by default. Set ISAAC_STAGE_RECORD_USE_BUILTIN=1 to
        # re-enable it (e.g. to retest on a future Fabric-aware Stage Recorder).
        if _env_truthy("ISAAC_STAGE_RECORD_USE_BUILTIN", False):
            try:
                import omni.kit.commands

                status, ret = omni.kit.commands.execute(
                    "StartRecording",
                    target_paths=[(ROVER_ARTICULATION_PRIM_PATH, True)],
                    live_mode=True,
                    use_frame_range=False,
                    start_frame=0,
                    end_frame=0,
                    use_preroll=False,
                    preroll_frame=0,
                    record_to="FILE",
                    take_name=self.take_name,
                    record_folder=str(self.record_dir),
                    increment_name=False,
                    apply_root_anim=True,
                    fps=self.fps,
                    user_tokens=[("xformOp:transform", "GfMatrix4d")],
                )
                self._stage_recorder_started = bool(status and ret and ret.get("error") is None)
                if not self._stage_recorder_started:
                    print(f"[run_baked_rover] Stage Recorder did not start: {ret}", flush=True)
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: built-in Stage Recorder start failed.")
                traceback.print_exc()
        print("[run_baked_rover] Replay capture started (sampled take"
              f"{' + built-in Stage Recorder' if self._stage_recorder_started else ''}).",
              flush=True)

    def _stop(self) -> None:
        rc = None
        if self._stage_recorder_started:
            try:
                import omni.kit.commands
                _, rc = omni.kit.commands.execute("StopRecording")
            except Exception:
                import traceback
                print("[run_baked_rover] Warning: Stage Recorder stop failed.")
                traceback.print_exc()
        sampled = self._write_sampled_take()
        if self.done_file:
            self.done_file.parent.mkdir(parents=True, exist_ok=True)
            self.done_file.write_text(
                f"stage_recorder_return={rc}\nsampled_usd={sampled}\n",
                encoding="utf-8",
            )
        print(f"[run_baked_rover] Stage Recorder capture stopped: {rc}; sampled={sampled}", flush=True)
        self.stopped = True
        self.should_exit = self.exit_on_done

    def _sample_custom_take(self) -> None:
        now = time.monotonic()
        if now - self._last_sample_wall < (1.0 / max(self.fps, 1.0)):
            return
        self._last_sample_wall = now
        try:
            if self._physx is None:
                import omni.physx
                self._physx = omni.physx.get_physx_interface()
            # Flush PhysX transforms into USD so XformCache below reads the LIVE driven pose
            # (physics writes to Fabric by default; base_footprint has no keyframes so its
            # composed transform then reflects exactly what physics produced this frame).
            self._physx.update_transformations(False, True, False)
        except Exception:
            pass
        from pxr import Usd, UsdGeom

        tc = self._sample_index
        try:
            if self._base_prim is not None and self._base_prim.IsValid():
                matrix = UsdGeom.XformCache(Usd.TimeCode.Default()).GetLocalToWorldTransform(
                    self._base_prim
                )
                self._samples[ROVER_PRIM_PATH].append((tc, matrix))
        except Exception:
            pass
        self._sample_index += 1

    def _write_sampled_take(self) -> str:
        if not self.record_dir:
            return ""
        out = self.record_dir / f"{self.take_name}_sampled.usda"
        try:
            from pxr import Sdf, Usd, UsdGeom, UsdPhysics

            stage = Usd.Stage.CreateNew(str(out))
            stage.SetDefaultPrim(stage.DefinePrim("/World", "Xform"))
            UsdGeom.SetStageUpAxis(stage, UsdGeom.Tokens.z)
            UsdGeom.SetStageMetersPerUnit(stage, 1.0)
            stage.SetTimeCodesPerSecond(self.fps)
            stage.SetStartTimeCode(0)
            stage.SetEndTimeCode(max(0, self._sample_index - 1))
            stage.GetRootLayer().subLayerPaths.append(str(ROVER_USD))
            if self.environment_usd:
                env = stage.DefinePrim(ENVIRONMENT_PRIM_PATH)
                env.GetReferences().AddReference(self.environment_usd)

            # Neutralise physics so the keyframes are the ONLY thing that moves the rover.
            # (Deactivating just the PhysicsScene is NOT enough — the articulation keeps
            # being simulated and fights the teleported root, driving PhysX non-finite = the
            # in-place spin. Keep the scene active, zero gravity, and disable every rover
            # rigid body + the articulation.
            scene = stage.OverridePrim("/physicsScene")
            scene.CreateAttribute("physics:gravityMagnitude", Sdf.ValueTypeNames.Float).Set(0.0)
            stage.OverridePrim("/ROS2_Graphs").SetActive(False)
            stage.OverridePrim("/World/ugv_rover/Drive_Graph").SetActive(False)
            stage.OverridePrim("/World/ugv_rover/joints").SetActive(False)
            for prim in stage.Traverse():
                if str(prim.GetPath()).startswith(ROVER_PRIM_PATH) and prim.HasAPI(
                    UsdPhysics.RigidBodyAPI
                ):
                    UsdPhysics.RigidBodyAPI(prim).CreateRigidBodyEnabledAttr(False)
            base = stage.OverridePrim(ROVER_ARTICULATION_PRIM_PATH)
            base.CreateAttribute(
                "physxArticulation:articulationEnabled", Sdf.ValueTypeNames.Bool
            ).Set(False)

            for path, samples in self._samples.items():
                if not samples:
                    continue
                prim = stage.OverridePrim(path)
                xform = UsdGeom.Xformable(prim)
                op = xform.MakeMatrixXform()
                for tc, matrix in samples:
                    op.Set(matrix, Usd.TimeCode(tc))

            stage.GetRootLayer().Save()
            return str(out)
        except Exception:
            import traceback
            print("[run_baked_rover] Warning: sampled replay USD write failed.")
            traceback.print_exc()
            return ""
