"""
UGV Jetson Rover simulation runner for Isaac Sim.

This is the single core runner for the rover demo. It launches Isaac Sim,
places the rover in an environment, and brings up BOTH control paths at once:

  * ROS 2 control  — an rclpy /cmd_vel subscriber drives the rover through
                     RoverDriveController; the Isaac ROS 2 Bridge publishes
                     /clock and sensor topics.
  * Keyboard control — KeyboardOverrideController drives the same rover.

The keyboard is IN CONTROL by default and can take over from ROS 2 at any time.
Press '\' to hand the rover to ROS 2 /cmd_vel, and '\' again to take it back.
To avoid two writers fighting over the wheels, the ROS 2 Bridge wheel-output
nodes stay disabled and a small rclpy subscriber routes /cmd_vel through the
same RoverDriveController used by the keyboard path. Keyboard handoff simply
enables/disables that subscriber, so there is always exactly one wheel writer.

You do not run this file directly. Use one of the no-flag launchers:

    python run_ground_plane.py        # rover on a plain ground plane
    python run_small_warehouse.py     # rover in the small warehouse
    python run_full_warehouse.py      # rover in the large (full) warehouse
    python run_small_warehouse_nav.py # small warehouse, Nav2-ready (see below)

All launch a visible window so the keyboard works.

Nav2 mode (run(..., nav=True)): additionally publishes the data Nav2 needs that
the real rover's hardware would supply — /odom + the odom->base_footprint TF
(replacing the encoder odometry) and a /scan LaserScan from an RTX lidar
(replacing the LD lidar) — and starts with ROS 2 owning the wheels so Nav2 can
drive immediately. Press '\' to grab manual keyboard control, '\' again to
return control to ROS 2. The odom/TF/scan publishers run regardless of which
side owns the wheels.
"""

from __future__ import annotations

import os
import sys
from pathlib import Path


# ── Paths ────────────────────────────────────────────────────────────────────
# This module lives in <workspace>/common/. The shared USD assets
# (jetson_rover.usd, *_warehouse.usd) live at the repo root, which is an ancestor
# of this file but at an unknown depth (run scripts live deeper still, under
# common/nav_systems/<system>/). So resolve the asset root by walking up until we
# find the rover USD, rather than assuming a fixed number of parents.
SCRIPT_DIR = Path(__file__).resolve().parent
ROVER_USD_NAME = "jetson_rover.usd"


def _find_asset_root(start: Path) -> Path:
    """Nearest ancestor of `start` containing the rover USD (the repo root)."""
    for candidate in (start, *start.parents):
        if (candidate / ROVER_USD_NAME).exists():
            return candidate
    # Fall back to the old layout assumption (<root>/common -> <root>).
    return start.parent


ASSET_ROOT = _find_asset_root(SCRIPT_DIR)
ROVER_USD = ASSET_ROOT / ROVER_USD_NAME


def asset_path(name: str) -> Path:
    """Absolute path to a shared asset (USD) at the repo root, e.g. a warehouse.

    Used by the per-system run scripts so they need not hard-code parent counts.
    """
    return ASSET_ROOT / name

# Where the rover USD is referenced, and the articulation root that appears
# after it loads.
REFERENCE_PRIM_PATH = "/World"
ROVER_PRIM_PATH = "/World/ugv_rover"

# ── Simulation timing ──────────────────────────────────────────────────────────
# The real rover's Gazebo model (ugv_ws/.../worlds/ugv_world.world) runs ODE at a
# 1 kHz physics step (max_step_size=0.001). Isaac's old 1/60 step was far too
# coarse for stable rolling/scrub contact on the six wheels, which is part of why
# in-place turning behaved badly. We step physics at 1/240 (4 substeps per 1/60
# render frame) to approach Gazebo's contact fidelity without rendering at 240 Hz.
# Per-articulation solver iteration counts are set manually in jetson_rover.usd
# (see tools/USD_CHANGES.md).
PHYSICS_DT = 1.0 / 240.0
RENDERING_DT = 1.0 / 60.0

# ── ROS 2 Bridge configuration ─────────────────────────────────────────────────
# Edit these constants if your ROS setup differs. ROS_DISCOVERY_SERVER stays
# empty for normal multicast discovery; set it to e.g. "172.20.1.2:11811" when
# Windows Isaac Sim and WSL cannot discover each other over multicast.
ROS_DOMAIN_ID = 0
ROS_DISTRO = "humble"
RMW_IMPLEMENTATION = "rmw_fastrtps_cpp"
ROS_DISCOVERY_SERVER = ""
CMD_VEL_TOPIC = "/cmd_vel"
CLOCK_TOPIC = "/clock"
DRIVE_GRAPH_PATH = "/World/ActionGraph_ROS2_Drive"


def _configure_ros_environment() -> None:
    os.environ["ROS_DISTRO"] = ROS_DISTRO
    os.environ["ROS_DOMAIN_ID"] = str(ROS_DOMAIN_ID)
    os.environ["RMW_IMPLEMENTATION"] = RMW_IMPLEMENTATION
    os.environ["ROS_LOCALHOST_ONLY"] = "0"
    if ROS_DISCOVERY_SERVER:
        os.environ["ROS_DISCOVERY_SERVER"] = ROS_DISCOVERY_SERVER
    else:
        os.environ.pop("ROS_DISCOVERY_SERVER", None)


def _resolve_usd(path: Path, label: str) -> Path:
    path = path.expanduser().resolve()
    if not path.exists():
        raise FileNotFoundError(f"{label} USD not found: {path}")
    if path.stat().st_size == 0:
        raise ValueError(
            f"{label} USD is empty (0 bytes): {path}\n"
            "Populate or re-export this USD before launching."
        )
    return path


def run(
    environment_usd: str | os.PathLike | None = None,
    nav: bool = False,
    headless: bool = False,
    lidar_debug_lines: bool = False,
    extra_sensor_setup=None,
) -> int:
    """
    Launch the rover simulation.

    This runner is nav-system AGNOSTIC. The universal Nav2 plumbing (/cmd_vel in,
    /clock, /odom + odom->base_footprint TF, /scan) is the same for every
    navigation system; anything a specific system needs on top (e.g. the depth
    camera for the RTAB-Map system) is injected via `extra_sensor_setup`.

    Parameters
    ----------
    environment_usd : path or None
        Environment USD opened as the stage before the rover is composed in.
        When None, a fresh stage with a default ground plane is used instead.
    nav : bool
        When True, enable Nav2 mode: publish /odom + odom->base_footprint TF and
        a /scan LaserScan from an RTX lidar, and start with ROS 2 owning the
        wheels (keyboard 'P' grabs/returns manual control). When False (default),
        behaves exactly like the manual-drive runners (keyboard owns the wheels,
        no odom/lidar overhead).
    headless : bool
        When True, run with no Isaac window. The RTX lidar still renders offscreen
        so /scan/odom/tf/clock all publish, and ROS 2 drives the wheels — but
        there is no in-window keyboard teleop. Use this when a windowed launch is
        unreliable (e.g. a NoMachine/remote-desktop server is running and hijacks
        the display, causing Vulkan swapchain failures). Pairs with nav=True.
    lidar_debug_lines : bool
        When True, ask the RTX lidar ROS helper to draw the lidar debug view in
        the Isaac viewport. This is useful for checking the inherited OmniLidar
        scan orientation and range hits.
    extra_sensor_setup : callable or None
        Optional nav-system sensor hook, called as
        `extra_sensor_setup(robot_prim_path, graph_path)` in the pre-play sensor
        block (nav=True only). This is how a navigation system adds its own
        sensors to the shared drive graph without coupling this runner to it — the
        camera systems (nav_depth, nav_rgb) pass
        common/depth_camera_graph.setup_depth_camera; nav_lidar passes nothing.
    """
    rover_usd = _resolve_usd(ROVER_USD, "Rover")
    environment_usd = (
        _resolve_usd(Path(environment_usd), "Environment")
        if environment_usd is not None
        else None
    )

    # ROS environment must be set before any Isaac Sim / ROS module is imported.
    _configure_ros_environment()

    # SimulationApp must be created before importing other Isaac Sim modules.
    from isaacsim import SimulationApp

    simulation_app = SimulationApp({"headless": headless})
    keyboard_ctrl = None
    cmd_vel_driver = None

    try:
        if str(SCRIPT_DIR) not in sys.path:
            sys.path.insert(0, str(SCRIPT_DIR))

        from rover_drive_controller import RoverDriveController
        from keyboard_override_controller import KeyboardOverrideController
        from ros2_cmd_vel_driver import Ros2CmdVelDriver
        from ros2_bridge_drive_graph import (
            enable_ros2_bridge_extensions,
            ensure_ros2_drive_graph,
            set_wheel_output_enabled,
            add_odometry_and_tf,
            add_lidar_scan,
            find_lidar_prim,
            find_depth_camera_prim,
        )

        # Bring up the ROS 2 Bridge extensions and let them settle.
        enable_ros2_bridge_extensions()
        for _ in range(2):
            simulation_app.update()

        import omni.usd
        from isaacsim.core.api import World
        from isaacsim.core.utils.stage import add_reference_to_stage
        from pxr import UsdGeom

        # ── Stage: open the environment, or start fresh for a ground plane ──────
        if environment_usd is None:
            omni.usd.get_context().new_stage()
        else:
            if not omni.usd.get_context().open_stage(str(environment_usd)):
                raise RuntimeError(
                    f"Could not open environment USD: {environment_usd}"
                )
            for _ in range(5):
                simulation_app.update()

        stage = omni.usd.get_context().get_stage()
        if stage is None:
            raise RuntimeError("Isaac Sim did not create or open a USD stage.")

        if not stage.GetPrimAtPath("/World").IsValid():
            UsdGeom.Xform.Define(stage, "/World")

        # Compose the rover into the stage.
        add_reference_to_stage(usd_path=str(rover_usd), prim_path=REFERENCE_PRIM_PATH)
        for _ in range(5):
            simulation_app.update()

        world = World(physics_dt=PHYSICS_DT, rendering_dt=RENDERING_DT)
        if environment_usd is None:
            world.scene.add_default_ground_plane()

        # In Nav2 mode, locate the lidar already baked into the rover USD (the
        # user positions it there). We use it as-is; we never author one.
        lidar_prim_path = None
        if nav:
            lidar_prim_path = find_lidar_prim(robot_prim_path=ROVER_PRIM_PATH)

        drive_ctrl = RoverDriveController(prim_path=ROVER_PRIM_PATH)

        # ── All USD physics-schema edits happen BEFORE the sim plays ────────────
        # Wheel drive mode, the ROS 2 graph, and camera drive mode all edit
        # physics schemas. Doing any of this after play forces a physics resync
        # that invalidates the live articulation tensor view and crashes the next
        # apply_action. So configure everything first, then play, then bind views.

        # ROS 2 Bridge graph (/clock out plus sensor publishers). It still
        # contains the generated /cmd_vel DifferentialController nodes for graph
        # compatibility, but their wheel outputs are kept disabled below; actual
        # /cmd_vel actuation goes through Ros2CmdVelDriver -> RoverDriveController
        # to match ugv_ws/ugv_driver.py and the keyboard path.
        try:
            ensure_ros2_drive_graph(
                robot_prim_path=ROVER_PRIM_PATH,
                graph_path=DRIVE_GRAPH_PATH,
                ros_domain_id=ROS_DOMAIN_ID,
                cmd_vel_topic=CMD_VEL_TOPIC,
                clock_topic=CLOCK_TOPIC,
            )
        except Exception as exc:
            import traceback
            print(f"[ugv_sim] ERROR creating ROS 2 Bridge graph:\n{exc}")
            traceback.print_exc()
            return 1

        # Nav2 publishers (odometry + TF + LaserScan) added to the same graph.
        # These always publish; they are not affected by the wheel-output toggle.
        if nav:
            import traceback
            try:
                add_odometry_and_tf(
                    robot_prim_path=ROVER_PRIM_PATH,
                    graph_path=DRIVE_GRAPH_PATH,
                )
            except Exception:
                print("[ugv_sim] ERROR adding odometry + TF publishers:")
                traceback.print_exc()
                print("[ugv_sim] Continuing; Nav2 will lack /odom and odom->base_footprint TF.")
            if lidar_prim_path is not None:
                try:
                    add_lidar_scan(
                        lidar_prim_path=lidar_prim_path,
                        graph_path=DRIVE_GRAPH_PATH,
                        show_debug_view=lidar_debug_lines,
                    )
                except Exception:
                    print("[ugv_sim] ERROR adding lidar /scan publisher:")
                    traceback.print_exc()
                    print("[ugv_sim] Continuing; Nav2 will lack /scan.")
            # Nav-system-specific sensors (e.g. the depth camera for the RTAB-Map
            # system). Injected by the run script so this runner stays agnostic.
            if extra_sensor_setup is not None:
                try:
                    extra_sensor_setup(ROVER_PRIM_PATH, DRIVE_GRAPH_PATH)
                except Exception:
                    print("[ugv_sim] ERROR in nav-system extra_sensor_setup hook:")
                    traceback.print_exc()
                    print("[ugv_sim] Continuing without the system's extra sensors.")

        drive_ctrl.configure_drive_mode()

        # Keyboard control needs an Isaac window, so it is skipped when headless.
        # In Nav2 mode ROS 2 owns the wheels at startup so Nav2 can drive
        # immediately; otherwise the keyboard is in control by default.
        # on_override_change flips the Python /cmd_vel subscriber so the keyboard
        # and ROS 2 never write the wheels at the same time. The generated graph's
        # wheel-output nodes remain disabled because they use Isaac's stock
        # DifferentialController instead of the ugv_ws-compatible controller.
        def on_override_change(keyboard_active: bool) -> None:
            set_wheel_output_enabled(False, graph_path=DRIVE_GRAPH_PATH)
            if cmd_vel_driver is not None:
                cmd_vel_driver.set_enabled(not keyboard_active)

        if not headless:
            keyboard_ctrl = KeyboardOverrideController(
                drive_controller=drive_ctrl,
                on_override_change=on_override_change,
                initial_override=not nav,
            )
            keyboard_ctrl.configure_camera_drives()

        # ── Start the sim, THEN bind articulation views (no more schema edits) ──
        world.reset()
        world.play()

        try:
            drive_ctrl.initialize()
        except RuntimeError as exc:
            print(f"[ugv_sim] ERROR during drive controller init:\n{exc}")
            print(
                "[ugv_sim] Check that the rover prim path exists in the stage.\n"
                f"  Reference prim path: {REFERENCE_PRIM_PATH}\n"
                f"  Robot prim path:     {ROVER_PRIM_PATH}\n"
                f"  Rover USD:           {rover_usd}"
            )
            return 1

        # The ROS 2 /cmd_vel driver needs rclpy. Under Isaac's Python 3.12 this
        # must be Isaac's BUNDLED rclpy. If a *system* ROS 2 (built for Python
        # 3.10) is sourced in the launching shell, it shadows the bundled one and
        # `import rclpy` fails with "No module named 'rclpy._rclpy_pybind11'".
        # Previously that exception propagated straight to shutdown ("Simulation
        # App Shutting Down" with no "Setup complete."). Catch it, print the fix,
        # and — when windowed — keep the sim usable in keyboard-only mode.
        try:
            cmd_vel_driver = Ros2CmdVelDriver(
                drive_controller=drive_ctrl,
                topic_name=CMD_VEL_TOPIC,
            )
        except Exception as exc:
            cmd_vel_driver = None
            print(
                "[ugv_sim] ERROR: could not start the ROS 2 /cmd_vel driver "
                f"(rclpy unavailable): {exc}\n"
                "[ugv_sim] Cause: a SYSTEM ROS 2 (Python 3.10) is sourced in this\n"
                "[ugv_sim] shell and shadows Isaac's bundled Python 3.12 rclpy.\n"
                "[ugv_sim] Fix: launch from a shell WITHOUT /opt/ros sourced — use the\n"
                "[ugv_sim] repo wrapper 'isaacpy_clean.sh' (or unset PYTHONPATH and\n"
                "[ugv_sim] AMENT_PREFIX_PATH). Nav2 runs in its own ROS-sourced\n"
                "[ugv_sim] terminal; the two still meet over DDS."
            )

        if keyboard_ctrl is not None:
            # Re-assert arbitration now that the graph is live post-play.
            on_override_change(keyboard_ctrl.override_active)
            world.add_physics_callback(
                "keyboard_step",
                lambda step_size: keyboard_ctrl.update(step_size),
            )
            if cmd_vel_driver is not None:
                world.add_physics_callback(
                    "ros2_cmd_vel_step",
                    lambda step_size: cmd_vel_driver.update(),
                )
            else:
                print("[ugv_sim] Continuing in KEYBOARD-ONLY mode (no ROS 2 /cmd_vel).")
        else:
            # Headless: no keyboard, so ROS 2 must own the wheels.
            set_wheel_output_enabled(False, graph_path=DRIVE_GRAPH_PATH)
            if cmd_vel_driver is None:
                print(
                    "[ugv_sim] FATAL: headless Nav2 mode has no keyboard fallback and\n"
                    "[ugv_sim] cannot drive the rover without rclpy. Aborting — re-launch\n"
                    "[ugv_sim] via isaacpy_clean.sh."
                )
                return 1
            cmd_vel_driver.set_enabled(True)
            world.add_physics_callback(
                "ros2_cmd_vel_step",
                lambda step_size: cmd_vel_driver.update(),
            )

        # ── On-launch camera perspectives (windowed only) ───────────────────────
        # Open the arm-top camera and front 3D depth camera viewports next to the
        # default free-fly Perspective (three perspectives total). Headless has no
        # viewports. Failures here are cosmetic and must never stop the sim.
        if not headless:
            try:
                from viewport_setup import setup_nav_viewports

                # The front depth camera prim exists in the rover USD regardless
                # of nav system, so the viewport works for every system (and even
                # with nav off). Locate it directly for the viewport.
                viewport_depth_prim = find_depth_camera_prim(
                    robot_prim_path=ROVER_PRIM_PATH
                )
                setup_nav_viewports(
                    robot_prim_path=ROVER_PRIM_PATH,
                    depth_camera_prim_path=viewport_depth_prim,
                )
            except Exception:
                import traceback
                print("[ugv_sim] Warning: could not open on-launch camera viewports:")
                traceback.print_exc()

        print("[ugv_sim] Setup complete.")
        print(f"[ugv_sim] Rover USD:       {rover_usd}")
        print(f"[ugv_sim] Environment USD: {environment_usd or 'none (default ground plane)'}")
        print(f"[ugv_sim] Robot prim:      {ROVER_PRIM_PATH}")
        print(f"[ugv_sim] ROS domain id:   {ROS_DOMAIN_ID}")
        print(f"[ugv_sim] cmd_vel topic:   {CMD_VEL_TOPIC}")
        print(f"[ugv_sim] clock topic:     {CLOCK_TOPIC}")
        print(f"[ugv_sim] Nav2 mode:       {'ON (/odom, /tf odom->base_footprint, /scan)' if nav else 'off'}")
        if nav:
            print(
                "[ugv_sim] Extra sensors:   "
                + (
                    "ON (nav-system extra_sensor_setup hook ran)"
                    if extra_sensor_setup is not None
                    else "none (lidar-only system)"
                )
            )
        print(f"[ugv_sim] Lidar debug:     {'ON' if lidar_debug_lines else 'off'}")
        print(f"[ugv_sim] Window:          {'headless (no window/keyboard)' if headless else 'visible'}")
        if headless:
            print("[ugv_sim] Control: ROS 2 / Nav2 drives the wheels (no keyboard in headless).")
        elif nav:
            print("[ugv_sim] Control: ROS 2 / Nav2 IN CONTROL by default. Press '\\' to")
            print("[ugv_sim]          grab manual keyboard control, '\\' again to return to ROS 2.")
        else:
            print("[ugv_sim] Control: keyboard IN CONTROL by default. Press '\\' to")
            print("[ugv_sim]          hand control to ROS 2 /cmd_vel, '\\' again to reclaim it.")

        while simulation_app.is_running():
            world.step(render=True)

    finally:
        if keyboard_ctrl is not None:
            keyboard_ctrl.shutdown()
        if cmd_vel_driver is not None:
            cmd_vel_driver.shutdown()
        simulation_app.close()

    return 0
