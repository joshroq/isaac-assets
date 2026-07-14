# keyboard_override_controller.py
#
# Keyboard input layer for the UGV Jetson Rover in Isaac Sim.
#
# Key layout is chosen to avoid Isaac Sim's viewport hotkeys (W/A/S/D/Q/E fly
# camera, W/E/R transform gizmos, F frame-selected), so driving the rover never
# fights the viewport.
#
# Movement (right-hand cluster, no WASD overlap):
#   u  i  o
#   j  k  l
#   m  ,  .
#
#   - / =  : decrease / increase both speeds 10%
#   [ / ]  : decrease / increase linear speed 10%
#   ; / '  : decrease / increase angular speed 10%
#   \        : toggle control  keyboard <-> ROS 2 /cmd_vel
#   SPACE / k : force stop (zero velocity)
#   arrows   : pan/tilt the camera arm
#   ESC / Ctrl+C : quit
#
# When override is ON  → keyboard commands reach the rover.
# When override is OFF → drive() calls are silently blocked so ROS 2 /cmd_vel
#                        can take over without conflict.

import carb
import carb.input
import omni.appwindow
import omni.usd
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics


# ── Key → (linear_multiplier, angular_multiplier) ───────────────────────────
# Exactly matches moveBindings from keyboard_ctrl.py
MOVE_BINDINGS = {
    carb.input.KeyboardInput.I:      ( 1,  0),   # forward
    carb.input.KeyboardInput.O:      ( 1, -2),   # forward + turn right
    carb.input.KeyboardInput.J:      ( 0,  1),   # turn left in place
    carb.input.KeyboardInput.L:      ( 0, -1),   # turn right in place
    carb.input.KeyboardInput.U:      ( 1,  2),   # forward + turn left
    carb.input.KeyboardInput.COMMA:  (-1,  0),   # backward
    carb.input.KeyboardInput.PERIOD: (-1,  2),   # backward + turn left
    carb.input.KeyboardInput.M:      (-1, -2),   # backward + turn right
    carb.input.KeyboardInput.K:      ( 0,  0),   # force stop
}

# ── Key → (speed_scale, turn_scale) ─────────────────────────────────────────
# Moved off Q/W/E (Isaac Sim fly-camera + transform gizmos) to a punctuation
# block that the 3D viewport leaves unbound.
SPEED_BINDINGS = {
    carb.input.KeyboardInput.EQUAL:         (1.1, 1.1),   # =  both speeds up
    carb.input.KeyboardInput.MINUS:         (0.9, 0.9),   # -  both speeds down
    carb.input.KeyboardInput.RIGHT_BRACKET: (1.1, 1.0),   # ]  linear up
    carb.input.KeyboardInput.LEFT_BRACKET:  (0.9, 1.0),   # [  linear down
    carb.input.KeyboardInput.APOSTROPHE:    (1.0, 1.1),   # '  angular up
    carb.input.KeyboardInput.SEMICOLON:     (1.0, 0.9),   # ;  angular down
}

LIGHT_GROUPS = {
    "headlights": ("R_Headlight", "L_Headlight"),
    "top": ("TopLight",),
    "all": ("R_Headlight", "L_Headlight", "TopLight"),
}

LIGHT_PARENT_HINTS = {
    "R_Headlight": "/front_camera_link/",
    "L_Headlight": "/front_camera_link/",
    "TopLight": "/pt_link2/",
}

CAMERA_JOINTS = {
    "pan": "pt_base_link_to_pt_link1",
    "tilt": "pt_link1_to_pt_link2",
}

CAMERA_LIMITS = {
    "pan": (-1.57, 1.57),
    "tilt": (-0.9, 0.9),
}

CAMERA_STEP_RAD = 0.05
CAMERA_RATE_RAD_PER_SEC = 0.9
DEFAULT_PHYSICS_DT = 1.0 / 60.0


def _key(name: str):
    return getattr(carb.input.KeyboardInput, name, None)


def _build_camera_bindings() -> dict:
    # Arrow keys only. The old T/G/R/F letter aliases collided with Isaac Sim's
    # transform gizmos (R = scale) and frame-selected (F), so they were removed.
    bindings = {}
    for key_name, value in (
        ("LEFT", ("pan", -1.0)),
        ("RIGHT", ("pan", 1.0)),
        ("UP", ("tilt", 1.0)),
        ("DOWN", ("tilt", -1.0)),
    ):
        key = _key(key_name)
        if key is not None:
            bindings[key] = value
    return bindings


CAMERA_BINDINGS = _build_camera_bindings()
CONTROL_TOGGLE_KEY = _key("BACKSLASH")
CONTROL_TOGGLE_LABEL = "\\"
CAMERA_RESET_KEY = _key("Y")
ACCESSORY_KEYS = {
    carb.input.KeyboardInput.H,
    carb.input.KeyboardInput.B,
    carb.input.KeyboardInput.N,
}
if CAMERA_RESET_KEY is not None:
    ACCESSORY_KEYS.add(CAMERA_RESET_KEY)


def _light_intensity_attr(prim):
    intensity_attr = prim.GetAttribute("inputs:intensity")
    if not intensity_attr.IsValid():
        intensity_attr = prim.GetAttribute("intensity")
    return intensity_attr


def _is_rover_light_prim(prim) -> bool:
    prim_name = prim.GetName()
    parent_hint = LIGHT_PARENT_HINTS.get(prim_name)
    if parent_hint is None:
        return False

    return parent_hint in str(prim.GetPath())


def _event_type(name: str):
    return getattr(carb.input.KeyboardEventType, name, None)


class KeyboardOverrideController:
    """
    Reads Isaac Sim keyboard events and drives the rover via a
    RoverDriveController instance.

    Override model:
        self.override_active = True   → keyboard controls rover
        self.override_active = False  → keyboard is silenced;
                                        nav AI can command rover freely

    The update() method must be called every simulation step from
    ugv_sim_runner.py's world step callback.
    """

    def __init__(
        self,
        drive_controller,
        initial_speed: float  = 0.6,
        initial_turn:  float  = 1.0,
        linear_limit:  float  = 1.3,
        angular_limit: float  = 1.5,
        on_override_change=None,
        initial_override: bool = True,
    ):
        """
        Parameters
        ----------
        drive_controller : RoverDriveController
            Already-initialized low-level actuator controller.
        initial_speed : float
            Starting linear speed multiplier (matches keyboard_ctrl.py default).
        initial_turn : float
            Starting angular speed multiplier (matches keyboard_ctrl.py default).
        linear_limit : float
            Maximum linear speed (matches rover ROS parameter linear_speed_limit).
        angular_limit : float
            Maximum angular speed (matches rover ROS parameter angular_speed_limit).
        on_override_change : callable | None
            Optional callback invoked as on_override_change(active: bool) whenever
            keyboard override turns on or off. The runner uses this to enable the
            ROS 2 Bridge wheel output when the keyboard hands control back to ROS 2,
            and to disable it again when the keyboard takes over. This keeps a
            single effective wheel writer at all times.
        initial_override : bool
            Whether the keyboard owns the wheels at startup. True (default) =
            keyboard in control, like the manual-drive runners. False = ROS 2
            owns the wheels from the start (used by the Nav2 runner); press
            backslash to grab manual keyboard control.
        """
        self._drive   = drive_controller
        self._speed   = initial_speed
        self._turn    = initial_turn
        self._lin_lim = linear_limit
        self._ang_lim = angular_limit
        self._on_override_change = on_override_change

        # Current commanded multipliers (x=linear direction, th=angular direction)
        self._x  = 0
        self._th = 0

        # Auto-stop counter — mirrors keyboard_ctrl.py "count > 4" logic
        self._no_key_count = 0

        # Override state — True means keyboard is active
        self.override_active = initial_override

        # Accessory state. Lights are forced off during startup as soon as the
        # stage is available; camera joints bind after the articulation starts.
        self._lights_initialized = False
        self._light_prims = {}
        self._light_on_intensities = {}
        self._headlights_on = False
        self._top_light_on = False

        self._camera_initialized = False
        self._camera_drives_configured = False
        self._camera_targets = {}
        self._camera_home_targets = {}
        self._camera_joint_indices = {}
        self._camera_active_keys = {}
        self._camera_axis_inputs = {"pan": 0.0, "tilt": 0.0}

        # Subscribe to Isaac Sim keyboard events
        self._input    = carb.input.acquire_input_interface()
        app_window     = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._sub_id   = self._input.subscribe_to_keyboard_events(
            self._keyboard,
            self._on_keyboard_event,
        )

        self._print_banner()
        self._initialize_lights()

        # Sync external listeners (e.g. the ROS 2 Bridge) with the starting
        # override state so the rover has exactly one wheel writer from frame 0.
        self._notify_override_change()

    # ── Keyboard event callback (fires on every key press/release) ───────────

    def _on_keyboard_event(
        self, event: carb.input.KeyboardEvent, *args, **kwargs
    ) -> bool:
        """
        Called by carb.input on every keyboard event.
        Drive controls act on key press to match keyboard_ctrl.py behaviour.
        Camera controls track both press and release for fluid held motion.
        Returns False for accessory keys so Isaac Sim default hotkeys do not run.
        """
        key = event.input
        key_press = _event_type("KEY_PRESS")
        key_release = _event_type("KEY_RELEASE")

        if event.type == key_release:
            if key in self._camera_active_keys:
                self._camera_active_keys.pop(key, None)
                self._recalculate_camera_axis_inputs()
                return False
            if key in ACCESSORY_KEYS:
                return False
            return True

        if event.type != key_press:
            return True

        # ── Control toggle (\ → keyboard <-> ROS 2). Kept away from Isaac
        # Sim's W/A/S/D fly-camera, transform, frame, and parent hotkeys.
        if CONTROL_TOGGLE_KEY is not None and key == CONTROL_TOGGLE_KEY:
            self.override_active = not self.override_active
            state = "ACTIVE  (keyboard controls rover)" \
                if self.override_active else \
                "INACTIVE (ROS 2 /cmd_vel controls rover)"
            print(f"[KeyboardOverride] Override: {state}")
            if not self.override_active:
                self._drive.stop()
            self._notify_override_change()
            return True

        # ── Force stop (space key — same as real rover) ──────────────────────
        if key == carb.input.KeyboardInput.SPACE:
            self._x  = 0
            self._th = 0
            if self.override_active:
                self._drive.stop()
            return True

        # ── Lights ───────────────────────────────────────────────────────────
        if key == carb.input.KeyboardInput.H:
            self._set_light_group("headlights", not self._headlights_on)
            return False

        if key == carb.input.KeyboardInput.B:
            self._set_light_group("top", not self._top_light_on)
            return False

        if key == carb.input.KeyboardInput.N:
            turn_on = not (self._headlights_on and self._top_light_on)
            self._set_light_group("all", turn_on)
            return False

        # ── Camera pan/tilt joints ───────────────────────────────────────────
        if key in CAMERA_BINDINGS:
            axis, direction = CAMERA_BINDINGS[key]
            self._camera_active_keys[key] = (axis, direction)
            self._recalculate_camera_axis_inputs()
            self._move_camera(axis, direction)
            return True

        if CAMERA_RESET_KEY is not None and key == CAMERA_RESET_KEY:
            self._reset_camera()
            return False

        # ── Movement keys ────────────────────────────────────────────────────
        if key in MOVE_BINDINGS:
            self._x, self._th = MOVE_BINDINGS[key]
            self._no_key_count = 0
            return True

        # ── Speed scaling keys ───────────────────────────────────────────────
        if key in SPEED_BINDINGS:
            sp_scale, tn_scale = SPEED_BINDINGS[key]
            self._speed = min(self._speed * sp_scale, self._lin_lim)
            self._turn  = min(self._turn  * tn_scale, self._ang_lim)
            self._no_key_count = 0
            print(
                f"[KeyboardOverride] Speed → linear={self._speed:.3f} m/s  "
                f"angular={self._turn:.3f} rad/s"
            )
            return True

        # ── Unknown key → increment no-key counter ───────────────────────────
        self._no_key_count += 1

        return True

    # ── Per-step update (call from simulation loop) ──────────────────────────

    def update(self, step_size: float = DEFAULT_PHYSICS_DT) -> None:
        """
        Called every simulation physics step.
        Applies current commanded velocity to the drive controller,
        but only if override is active.
        """
        if not self._lights_initialized:
            self._initialize_lights()
        if not self._camera_initialized:
            self._initialize_camera()
        self._update_camera_motion(step_size)

        if not self.override_active:
            return

        # Mirror keyboard_ctrl.py: if no valid key for >4 ticks, stop smoothly
        if self._no_key_count > 4:
            self._x  = 0
            self._th = 0

        linear  = self._speed * self._x
        angular = self._turn  * self._th

        self._drive.drive(linear, angular)

    def _notify_override_change(self) -> None:
        """Tell external listeners whether the keyboard currently owns the wheels."""
        if self._on_override_change is None:
            return
        try:
            self._on_override_change(self.override_active)
        except Exception as exc:  # never let a listener crash the sim loop
            print(f"[KeyboardOverride] Warning in override-change callback: {exc}")

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        """Unsubscribe from keyboard events and stop the rover."""
        try:
            self._drive.stop()
        except Exception as exc:
            print(f"[KeyboardOverride] Warning during shutdown stop: {exc}")

        try:
            self._set_light_group("all", False)
        except Exception as exc:
            print(f"[KeyboardOverride] Warning during shutdown lights-off: {exc}")

        if self._sub_id is not None:
            try:
                self._input.unsubscribe_to_keyboard_events(
                    self._keyboard, self._sub_id
                )
            except Exception as exc:
                print(f"[KeyboardOverride] Warning during keyboard unsubscribe: {exc}")
            self._sub_id = None

    # ── Helpers ──────────────────────────────────────────────────────────────

    def _rover_prim_path(self) -> str:
        return getattr(self._drive, "prim_path", "/World/ugv_rover")

    def _iter_rover_prims(self, stage):
        root_path = self._rover_prim_path()
        root_prim = stage.GetPrimAtPath(root_path)
        if root_prim.IsValid():
            root_prefix = root_path.rstrip("/") + "/"
            for prim in stage.Traverse():
                prim_path = str(prim.GetPath())
                if prim_path == root_path or prim_path.startswith(root_prefix):
                    yield prim
            return

        print(
            "[KeyboardOverride] Warning: rover root prim not found for "
            f"accessory lookup: {root_path}"
        )

    def _initialize_lights(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        root_path = self._rover_prim_path()
        if not stage.GetPrimAtPath(root_path).IsValid():
            print(
                "[KeyboardOverride] Warning: rover root prim not found for "
                f"light lookup: {root_path}"
            )
            return

        self._light_prims.clear()
        self._light_on_intensities.clear()

        wanted = set(LIGHT_GROUPS["all"])
        for prim in self._iter_rover_prims(stage):
            if not _is_rover_light_prim(prim):
                continue

            intensity_attr = _light_intensity_attr(prim)
            if not intensity_attr.IsValid():
                continue

            intensity = intensity_attr.Get()
            if intensity is None or intensity <= 0.0:
                intensity = 500.0

            name = prim.GetName()
            self._light_prims[name] = prim
            self._light_on_intensities[name] = float(intensity)
            intensity_attr.Set(0.0)

        self._headlights_on = False
        self._top_light_on = False
        self._lights_initialized = True

        missing = sorted(wanted - set(self._light_prims))
        if missing:
            print(f"[KeyboardOverride] Warning: light prims not found: {missing}")
        else:
            print("[KeyboardOverride] Lights initialized OFF.")

    def _set_light_group(self, group: str, enabled: bool) -> None:
        if not self._lights_initialized:
            self._initialize_lights()

        for light_name in LIGHT_GROUPS[group]:
            prim = self._light_prims.get(light_name)
            if prim is None:
                continue

            intensity_attr = _light_intensity_attr(prim)
            if not intensity_attr.IsValid():
                continue

            intensity_attr.Set(
                self._light_on_intensities.get(light_name, 500.0)
                if enabled else 0.0
            )

        if group in ("headlights", "all"):
            self._headlights_on = enabled
        if group in ("top", "all"):
            self._top_light_on = enabled

        print(
            "[KeyboardOverride] Lights → "
            f"headlights={'ON' if self._headlights_on else 'OFF'}  "
            f"top={'ON' if self._top_light_on else 'OFF'}"
        )

    def configure_camera_drives(self) -> None:
        """
        Apply position drive mode to the camera pan/tilt joints.

        This edits USD physics schemas, so it MUST run before the simulation
        starts playing. Applying DriveAPI while the sim is running forces a
        physics resync that invalidates live articulation tensor views (and
        would crash the wheel apply_action on the next step).
        """
        self._set_camera_position_drives()
        self._camera_drives_configured = True

    def _initialize_camera(self) -> bool:
        if not getattr(self._drive, "initialized", False):
            return False

        robot = self._drive.articulation
        joint_names = list(self._drive.joint_names)
        missing = [
            joint for joint in CAMERA_JOINTS.values()
            if joint not in joint_names
        ]
        if missing:
            print(f"[KeyboardOverride] Warning: camera joints not found: {missing}")
            self._camera_initialized = True
            return False

        # Drive mode is normally configured before play via
        # configure_camera_drives(). Only fall back to editing schemas here if
        # that never happened — and never while the sim is already running.
        if not self._camera_drives_configured:
            self._set_camera_position_drives()
            self._camera_drives_configured = True
        joint_positions = self._get_current_joint_positions(robot, joint_names)

        for axis, joint_name in CAMERA_JOINTS.items():
            joint_index = joint_names.index(joint_name)
            self._camera_joint_indices[axis] = joint_index
            self._camera_targets[axis] = float(joint_positions[joint_index])

        self._camera_home_targets = dict(self._camera_targets)
        self._camera_initialized = True
        print(
            "[KeyboardOverride] Camera arm initialized. "
            "Use arrow keys to pan/tilt."
        )
        return True

    def _get_current_joint_positions(self, robot, joint_names: list) -> np.ndarray:
        try:
            positions = robot.get_joint_positions()
            if positions is not None:
                return np.asarray(positions, dtype=float)
        except Exception:
            pass

        return np.zeros(len(joint_names), dtype=float)

    def _set_camera_position_drives(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return

        wanted = set(CAMERA_JOINTS.values())
        for prim in self._iter_rover_prims(stage):
            if prim.GetName() not in wanted:
                continue

            drive = UsdPhysics.DriveAPI.Apply(prim, "angular")
            drive.CreateStiffnessAttr(150.0)
            drive.CreateDampingAttr(25.0)
            drive.CreateMaxForceAttr(100.0)
            print(f"[KeyboardOverride] Position drive set on: {prim.GetPath()}")

    def _recalculate_camera_axis_inputs(self) -> None:
        self._camera_axis_inputs = {"pan": 0.0, "tilt": 0.0}
        for axis, direction in self._camera_active_keys.values():
            self._camera_axis_inputs[axis] += direction

        for axis in self._camera_axis_inputs:
            self._camera_axis_inputs[axis] = float(
                np.clip(self._camera_axis_inputs[axis], -1.0, 1.0)
            )

    def _update_camera_motion(self, step_size: float) -> None:
        if not self._camera_initialized:
            return

        if not any(self._camera_axis_inputs.values()):
            return

        dt = step_size if step_size and step_size > 0.0 else DEFAULT_PHYSICS_DT
        changed = False
        for axis, direction in self._camera_axis_inputs.items():
            if direction == 0.0 or axis not in self._camera_targets:
                continue

            low, high = CAMERA_LIMITS[axis]
            next_target = float(
                np.clip(
                    self._camera_targets[axis]
                    + CAMERA_RATE_RAD_PER_SEC * dt * direction,
                    low,
                    high,
                )
            )
            if next_target != self._camera_targets[axis]:
                self._camera_targets[axis] = next_target
                changed = True

        if changed:
            self._apply_camera_targets()

    def _reset_camera(self) -> None:
        if not self._camera_initialized and not self._initialize_camera():
            return

        if not self._camera_home_targets:
            return

        self._camera_active_keys.clear()
        self._recalculate_camera_axis_inputs()
        self._camera_targets = dict(self._camera_home_targets)
        self._apply_camera_targets()
        print(
            "[KeyboardOverride] Camera reset → "
            f"pan={self._camera_targets['pan']:.2f} rad  "
            f"tilt={self._camera_targets['tilt']:.2f} rad"
        )

    def _move_camera(self, axis: str, direction: float) -> None:
        if not self._camera_initialized and not self._initialize_camera():
            return

        if axis not in self._camera_targets:
            return

        low, high = CAMERA_LIMITS[axis]
        self._camera_targets[axis] = float(
            np.clip(
                self._camera_targets[axis] + CAMERA_STEP_RAD * direction,
                low,
                high,
            )
        )

        self._apply_camera_targets()

        print(
            "[KeyboardOverride] Camera → "
            f"pan={self._camera_targets['pan']:.2f} rad  "
            f"tilt={self._camera_targets['tilt']:.2f} rad"
        )

    def _apply_camera_targets(self) -> None:
        if not self._camera_initialized:
            return

        indices = np.array(
            [self._camera_joint_indices[name] for name in ("pan", "tilt")],
            dtype=np.int32,
        )
        positions = np.array(
            [self._camera_targets[name] for name in ("pan", "tilt")],
            dtype=np.float32,
        )

        self._drive.articulation.apply_action(
            ArticulationAction(
                joint_positions=positions,
                joint_indices=indices,
            )
        )

    def _print_banner(self) -> None:
        if self.override_active:
            control_message = (
                f"  Keyboard is IN CONTROL. Press '{CONTROL_TOGGLE_LABEL}' to hand the\n"
                f"  rover to ROS 2 /cmd_vel; press '{CONTROL_TOGGLE_LABEL}' again to take it back."
            )
        else:
            control_message = (
                f"  ROS 2 /cmd_vel is IN CONTROL. Press '{CONTROL_TOGGLE_LABEL}' to grab\n"
                f"  manual keyboard control; press '{CONTROL_TOGGLE_LABEL}' again to return to ROS 2."
            )

        print(f"""
[KeyboardOverride] UGV Jetson Rover — Isaac Sim Keyboard Control
=================================================================
  u  i  o       i = forward      , = backward
  j  k  l       j = turn left    l = turn right
  m  ,  .       u/o/m/. = diagonal

  - / =         decrease / increase both speeds  10%
  [ / ]         decrease / increase linear speed 10%
  ; / '         decrease / increase angular speed 10%

  H             toggle headlights
  B             toggle top light
  N             toggle all lights

  ← / →         pan camera arm
  ↑ / ↓         tilt camera arm
  Y             reset camera pan/tilt

  SPACE / k     force stop
  \\             toggle control: keyboard  <->  ROS 2 /cmd_vel
=================================================================
  Keys avoid Isaac Sim's W/A/S/D/Q/E camera and F frame hotkeys.
{control_message}
  Starting speed:  linear={self._speed:.2f} m/s   angular={self._turn:.2f} rad/s
  Speed limits:    linear={self._lin_lim:.2f} m/s   angular={self._ang_lim:.2f} rad/s
=================================================================
""")
