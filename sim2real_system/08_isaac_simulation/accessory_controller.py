"""
Isaac-side ACCESSORY keyboard controls for the baked-graph rover (windowed only).

The rover DRIVES exclusively from ROS 2 /cmd_vel (the baked skid-steer graph).
This module adds the non-drive accessory controls, adapted from the original
`ugv_controller/common/keyboard_override_controller.py` with the wheel-drive parts
removed so it never competes with the baked drive graph:

  ← / →   pan the camera arm      (joint pt_base_link_to_pt_link1)
  ↑ / ↓   tilt the camera arm     (joint pt_link1_to_pt_link2)
  Y       reset the arm to its home pan/tilt
  H       toggle headlights       (R_Headlight + L_Headlight, off at startup)
  B       toggle the top light    (TopLight, off at startup)
  N       toggle all lights

It binds its OWN SingleArticulation on the rover to move only the pan/tilt DOFs
(the wheels are left entirely to the baked graph). Lights are pure USD intensity
edits. Keyboard events come from Isaac's carb.input, so this only works windowed.
"""

from __future__ import annotations

import carb
import carb.input
import omni.appwindow
import omni.usd
import numpy as np
from isaacsim.core.utils.types import ArticulationAction
from pxr import UsdPhysics


# ── Lights ───────────────────────────────────────────────────────────────────
# Prim names + the parent-path hint that identifies each as a rover light.
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
DEFAULT_LIGHT_INTENSITY = 60000.0

# ── Camera arm ────────────────────────────────────────────────────────────────
CAMERA_JOINTS = {
    "pan": "pt_base_link_to_pt_link1",
    "tilt": "pt_link1_to_pt_link2",
}
CAMERA_LIMITS = {"pan": (-1.57, 1.57), "tilt": (-0.9, 0.9)}
CAMERA_STEP_RAD = 0.05
CAMERA_RATE_RAD_PER_SEC = 0.9
DEFAULT_PHYSICS_DT = 1.0 / 60.0


def _key(name: str):
    return getattr(carb.input.KeyboardInput, name, None)


def _event_type(name: str):
    return getattr(carb.input.KeyboardEventType, name, None)


def _light_intensity_attr(prim):
    attr = prim.GetAttribute("inputs:intensity")
    if not attr.IsValid():
        attr = prim.GetAttribute("intensity")
    return attr


def _build_camera_bindings() -> dict:
    bindings = {}
    for key_name, value in (("LEFT", ("pan", -1.0)), ("RIGHT", ("pan", 1.0)),
                            ("UP", ("tilt", 1.0)), ("DOWN", ("tilt", -1.0))):
        key = _key(key_name)
        if key is not None:
            bindings[key] = value
    return bindings


CAMERA_BINDINGS = _build_camera_bindings()
CAMERA_RESET_KEY = _key("Y")
ACCESSORY_KEYS = {carb.input.KeyboardInput.H, carb.input.KeyboardInput.B,
                  carb.input.KeyboardInput.N}
if CAMERA_RESET_KEY is not None:
    ACCESSORY_KEYS.add(CAMERA_RESET_KEY)


class AccessoryController:
    """Keyboard control of the rover's camera arm + lights (no wheel drive)."""

    def __init__(self, articulation_prim_path: str = "/World/ugv_rover/base_footprint"):
        self._prim_path = articulation_prim_path
        self._robot = None
        self._joint_names: list = []

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

        self._input = carb.input.acquire_input_interface()
        app_window = omni.appwindow.get_default_app_window()
        self._keyboard = app_window.get_keyboard()
        self._sub_id = self._input.subscribe_to_keyboard_events(
            self._keyboard, self._on_keyboard_event
        )

        self._print_banner()
        self._initialize_lights()  # force OFF as soon as the stage is available

    # ── Setup (before / after play) ──────────────────────────────────────────

    def configure_camera_drives(self) -> None:
        """Apply position-drive mode to the pan/tilt joints. MUST run before play
        (editing DriveAPI while running forces a physics resync)."""
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
            print(f"[accessory] Position drive set on: {prim.GetPath()}")
        self._camera_drives_configured = True

    def initialize(self) -> None:
        """Bind the articulation after play so the pan/tilt DOFs can be commanded."""
        from isaacsim.core.prims import SingleArticulation

        self._robot = SingleArticulation(self._prim_path)
        self._robot.initialize()
        self._joint_names = list(self._robot.dof_names)
        self._initialize_camera()

    # ── Per-step ──────────────────────────────────────────────────────────────

    def update(self, step_size: float = DEFAULT_PHYSICS_DT) -> None:
        if not self._lights_initialized:
            self._initialize_lights()
        if not self._camera_initialized:
            self._initialize_camera()
        self._update_camera_motion(step_size)

    # ── Keyboard ──────────────────────────────────────────────────────────────

    def _on_keyboard_event(self, event, *args, **kwargs) -> bool:
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

        if key in CAMERA_BINDINGS:
            axis, direction = CAMERA_BINDINGS[key]
            self._camera_active_keys[key] = (axis, direction)
            self._recalculate_camera_axis_inputs()
            self._move_camera(axis, direction)
            return False

        if CAMERA_RESET_KEY is not None and key == CAMERA_RESET_KEY:
            self._reset_camera()
            return False

        return True

    # ── Cleanup ──────────────────────────────────────────────────────────────

    def shutdown(self) -> None:
        try:
            self._set_light_group("all", False)
        except Exception as exc:
            print(f"[accessory] Warning during lights-off: {exc}")
        if self._sub_id is not None:
            try:
                self._input.unsubscribe_to_keyboard_events(self._keyboard, self._sub_id)
            except Exception as exc:
                print(f"[accessory] Warning during keyboard unsubscribe: {exc}")
            self._sub_id = None

    # ── Lights ────────────────────────────────────────────────────────────────

    def _iter_rover_prims(self, stage):
        root_prim = stage.GetPrimAtPath(self._prim_path)
        # Lights/joints live under the rover root, not the articulation root; walk
        # up to the rover prim (/World/ugv_rover) so we cover both.
        rover_root = "/World/ugv_rover"
        base = rover_root if stage.GetPrimAtPath(rover_root).IsValid() else self._prim_path
        prefix = base.rstrip("/") + "/"
        for prim in stage.Traverse():
            path = str(prim.GetPath())
            if path == base or path.startswith(prefix):
                yield prim

    def _is_rover_light_prim(self, prim) -> bool:
        hint = LIGHT_PARENT_HINTS.get(prim.GetName())
        return hint is not None and hint in str(prim.GetPath())

    def _initialize_lights(self) -> None:
        stage = omni.usd.get_context().get_stage()
        if stage is None:
            return
        self._light_prims.clear()
        self._light_on_intensities.clear()
        wanted = set(LIGHT_GROUPS["all"])
        for prim in self._iter_rover_prims(stage):
            if not self._is_rover_light_prim(prim):
                continue
            attr = _light_intensity_attr(prim)
            if not attr.IsValid():
                continue
            intensity = attr.Get()
            if intensity is None or intensity <= 0.0:
                intensity = DEFAULT_LIGHT_INTENSITY
            self._light_prims[prim.GetName()] = prim
            self._light_on_intensities[prim.GetName()] = float(intensity)
            attr.Set(0.0)  # start OFF
        self._headlights_on = False
        self._top_light_on = False
        self._lights_initialized = True
        missing = sorted(wanted - set(self._light_prims))
        if missing:
            print(f"[accessory] Warning: light prims not found: {missing}")
        else:
            print("[accessory] Lights initialized OFF.")

    def _set_light_group(self, group: str, enabled: bool) -> None:
        if not self._lights_initialized:
            self._initialize_lights()
        for name in LIGHT_GROUPS[group]:
            prim = self._light_prims.get(name)
            if prim is None:
                continue
            attr = _light_intensity_attr(prim)
            if attr.IsValid():
                attr.Set(self._light_on_intensities.get(name, DEFAULT_LIGHT_INTENSITY)
                         if enabled else 0.0)
        if group in ("headlights", "all"):
            self._headlights_on = enabled
        if group in ("top", "all"):
            self._top_light_on = enabled
        print(f"[accessory] Lights -> headlights={'ON' if self._headlights_on else 'OFF'}  "
              f"top={'ON' if self._top_light_on else 'OFF'}")

    # ── Camera arm ────────────────────────────────────────────────────────────

    def _initialize_camera(self) -> bool:
        if self._robot is None:
            return False
        missing = [j for j in CAMERA_JOINTS.values() if j not in self._joint_names]
        if missing:
            print(f"[accessory] Warning: camera joints not found: {missing}")
            self._camera_initialized = True
            return False
        if not self._camera_drives_configured:
            self.configure_camera_drives()
        try:
            positions = np.asarray(self._robot.get_joint_positions(), dtype=float)
        except Exception:
            positions = np.zeros(len(self._joint_names), dtype=float)
        for axis, joint_name in CAMERA_JOINTS.items():
            idx = self._joint_names.index(joint_name)
            self._camera_joint_indices[axis] = idx
            self._camera_targets[axis] = float(positions[idx])
        self._camera_home_targets = dict(self._camera_targets)
        self._camera_initialized = True
        print("[accessory] Camera arm initialized. Arrow keys pan/tilt; Y resets.")
        return True

    def _recalculate_camera_axis_inputs(self) -> None:
        self._camera_axis_inputs = {"pan": 0.0, "tilt": 0.0}
        for axis, direction in self._camera_active_keys.values():
            self._camera_axis_inputs[axis] += direction
        for axis in self._camera_axis_inputs:
            self._camera_axis_inputs[axis] = float(np.clip(self._camera_axis_inputs[axis], -1.0, 1.0))

    def _update_camera_motion(self, step_size: float) -> None:
        if not self._camera_initialized or not any(self._camera_axis_inputs.values()):
            return
        dt = step_size if step_size and step_size > 0.0 else DEFAULT_PHYSICS_DT
        changed = False
        for axis, direction in self._camera_axis_inputs.items():
            if direction == 0.0 or axis not in self._camera_targets:
                continue
            low, high = CAMERA_LIMITS[axis]
            nxt = float(np.clip(self._camera_targets[axis] + CAMERA_RATE_RAD_PER_SEC * dt * direction,
                                low, high))
            if nxt != self._camera_targets[axis]:
                self._camera_targets[axis] = nxt
                changed = True
        if changed:
            self._apply_camera_targets()

    def _move_camera(self, axis: str, direction: float) -> None:
        if not self._camera_initialized and not self._initialize_camera():
            return
        if axis not in self._camera_targets:
            return
        low, high = CAMERA_LIMITS[axis]
        self._camera_targets[axis] = float(np.clip(
            self._camera_targets[axis] + CAMERA_STEP_RAD * direction, low, high))
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
        print(f"[accessory] Camera reset -> pan={self._camera_targets['pan']:.2f} "
              f"tilt={self._camera_targets['tilt']:.2f} rad")

    def _apply_camera_targets(self) -> None:
        if not self._camera_initialized or self._robot is None:
            return
        indices = np.array([self._camera_joint_indices[a] for a in ("pan", "tilt")], dtype=np.int32)
        positions = np.array([self._camera_targets[a] for a in ("pan", "tilt")], dtype=np.float32)
        self._robot.apply_action(ArticulationAction(joint_positions=positions, joint_indices=indices))

    def _print_banner(self) -> None:
        print("""
[accessory] Rover accessory controls (Isaac window focused):
=================================================================
  <- / ->     pan camera arm        H   toggle headlights
  up / down   tilt camera arm       B   toggle top light
  Y           reset camera arm      N   toggle all lights
=================================================================
  Driving is ROS 2 /cmd_vel (keyboard: ../keyboard/keyboard_drive.sh).
  Lights start OFF.
=================================================================
""")
