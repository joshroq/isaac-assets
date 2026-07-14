# rover_drive_controller.py
#
# Low-level differential drive controller for the UGV Jetson Rover in Isaac Sim.
# Converts linear.x / angular.z Twist-style commands into wheel joint velocities
# and applies them to the articulation.
#
# Physical parameters come from the rover's ROS driver source (ugv_driver.py),
# Gazebo SDF, and published rover specs:
#   wheel_separation = 0.175 m
#   wheel_radius     = 0.040 m  (80 mm tire diameter)
#   linear speed limit = 1.3 m/s
#   angular speed default = 0.5 rad/s, limit = 1.0 rad/s
#
# This file has NO keyboard logic and NO ROS logic.
# It is imported by higher-level controllers.

from isaacsim.core.prims import SingleArticulation
from isaacsim.core.utils.types import ArticulationAction
import numpy as np


# ── Physical constants (from ugv_driver.py + URDF) ──────────────────────────

WHEEL_SEPARATION = 0.175   # metres, left-to-right wheel centre distance (ugv_ws)
# Wheel rolling radius used by the differential-drive kinematics.
WHEEL_RADIUS     = 0.04    # metres (real spec: 80 mm tyre)

MAX_LINEAR_VEL   = 1.3     # m/s  (product max speed)
MAX_ANGULAR_VEL  = 1.5     # rad/s (in-place-turn clamp; raised from 1.0 for snappier
                           # turning during nav testing — product turns at 0 m radius)

# Fallback per-wheel Isaac drive-force cap for the four motorized corner wheels.
# The real asset authors a very high maxForce on these velocity drives; preserve
# that authored USD value and only use this if the attribute is missing.
WHEEL_DRIVE_MAX_FORCE = 1.0


# The rover has SIX wheels on the ground (like the real robot). We drive the four
# CORNER wheels — up (front) + down (rear) on each side — and leave the centre
# pair as free-rolling idlers, matching the real skid-steer: one speed per side
# is all the differential-drive kinematics needs, the centre wheels sit near the
# turn pivot and mostly just roll. This mirrors the Gazebo skid_steer_drive model
# (num_wheel_pairs=2, corner joints only). The centre wheels stay in ground
# contact — do NOT disable their collision; the real rover rides on all six.
LEFT_WHEEL_JOINTS  = [
    "left_up_wheel_link_joint",
    "left_down_wheel_link_joint",
]
RIGHT_WHEEL_JOINTS = [
    "right_up_wheel_link_joint",
    "right_down_wheel_link_joint",
]


class RoverDriveController:
    """
    Wraps an Isaac Sim Articulation and exposes a single drive() method
    that accepts the same linear_x / angular_z values as the real rover's
    cmd_vel_callback in ugv_driver.py.

    Usage:
        controller = RoverDriveController("/World/ugv_rover")
        controller.initialize()          # call once after sim starts
        controller.drive(0.2, 0.0)       # forward at 0.2 m/s
        controller.drive(0.0, 0.5)       # spin left at 0.5 rad/s
        controller.stop()                # full stop
    """

    def __init__(
        self,
        prim_path: str = "/World/ugv_rover",
        wheel_separation: float = WHEEL_SEPARATION,
        wheel_radius: float = WHEEL_RADIUS,
        max_linear_vel: float = MAX_LINEAR_VEL,
        max_angular_vel: float = MAX_ANGULAR_VEL,
    ):
        self._prim_path       = prim_path
        self._wheel_sep       = wheel_separation
        self._wheel_rad       = wheel_radius
        self._max_lin         = max_linear_vel
        self._max_ang         = max_angular_vel

        self._robot: SingleArticulation = None
        self._joint_names: list   = []
        self._initialized         = False

    @property
    def articulation(self) -> SingleArticulation:
        """Initialized Isaac Sim articulation, or None before initialize()."""
        return self._robot

    @property
    def prim_path(self) -> str:
        """Root prim path used for the rover articulation."""
        return self._prim_path

    @property
    def joint_names(self) -> list:
        """Joint names discovered on the rover articulation."""
        return self._joint_names

    @property
    def initialized(self) -> bool:
        """True once the articulation has been initialized."""
        return self._initialized

    # ── Setup ────────────────────────────────────────────────────────────────

    def configure_drive_mode(self) -> None:
        """
        Set every wheel joint to velocity drive mode.

        This edits USD physics schemas, so it MUST run before the simulation
        starts playing. Applying DriveAPI while the sim is running forces a
        physics resync that invalidates live articulation tensor views.
        """
        self._set_velocity_drive_mode()

    def initialize(self) -> None:
        """
        Bind to the articulation and validate wheel joints.

        Must be called once after the simulation has started playing (i.e. after
        world.reset()/world.play()) so the tensor view is the final, live one.
        Do NOT apply any USD physics-schema edits after this point, or the view
        below will be invalidated. Drive mode is set earlier via
        configure_drive_mode().
        """
        self._robot = SingleArticulation(self._prim_path)
        self._robot.initialize()
        self._joint_names = list(self._robot.dof_names)

        # Validate that every expected joint actually exists
        missing = [
            j for j in LEFT_WHEEL_JOINTS + RIGHT_WHEEL_JOINTS
            if j not in self._joint_names
        ]
        if missing:
            raise RuntimeError(
                f"[RoverDriveController] Missing joints in USD articulation: "
                f"{missing}\n"
                f"Available joints: {self._joint_names}"
            )

        self._initialized = True
        print(
            f"[RoverDriveController] Initialized on '{self._prim_path}'\n"
            f"  Joints found: {self._joint_names}\n"
            f"  Wheel separation: {self._wheel_sep} m\n"
            f"  Wheel radius:     {self._wheel_rad} m"
        )

    def _set_velocity_drive_mode(self) -> None:
        from pxr import UsdPhysics
        from omni import usd as omni_usd

        stage = omni_usd.get_context().get_stage()

        for joint_name in LEFT_WHEEL_JOINTS + RIGHT_WHEEL_JOINTS:
            joint_prim = None

            # Search the entire stage for a prim whose name matches
            for prim in stage.Traverse():
                if prim.GetName() == joint_name:
                    joint_prim = prim
                    break 

            if joint_prim is None:
                print(
                    f"[RoverDriveController] Warning: joint '{joint_name}' "
                    f"not found in stage. Skipping drive mode setup."
                )
                continue

            drive = UsdPhysics.DriveAPI.Apply(joint_prim, "angular")
            drive.CreateTypeAttr("velocity")
            drive.CreateDampingAttr(1e4)
            drive.CreateStiffnessAttr(0.0)

            # The asset (jetson_rover.usd) is the single source of truth for the
            # drive force. Only set a default if the asset has not authored one,
            # so we never silently clobber an intentionally tuned per-wheel force.
            max_force_attr = drive.GetMaxForceAttr()
            if not max_force_attr or not max_force_attr.HasAuthoredValue():
                max_force_attr = drive.CreateMaxForceAttr()
                max_force_attr.Set(WHEEL_DRIVE_MAX_FORCE)
                source = f"default {WHEEL_DRIVE_MAX_FORCE}"
            else:
                source = f"asset {max_force_attr.Get()}"

            print(
                f"[RoverDriveController] Velocity drive set on: "
                f"{joint_prim.GetPath()} (maxForce={source})"
            )

    # ── Main control interface ────────────────────────────────────────────────

    def drive(self, linear_x: float, angular_z: float) -> None:
        """
        Mirror of the real rover's cmd_vel_callback logic in ugv_driver.py.

        Applies the same angular velocity minimum threshold the real driver
        uses when linear velocity is zero, then converts to wheel velocities.
        """
        if not self._initialized:
            print("[RoverDriveController] Warning: drive() called before initialize()")
            return

        # ── Mirror ugv_driver.py angular minimum threshold ───────────────────
        # Real driver: if linear==0 and 0 < angular < 0.2, clamp to 0.2
        if linear_x == 0.0:
            if 0.0 < angular_z < 0.2:
                angular_z = 0.2
            elif -0.2 < angular_z < 0.0:
                angular_z = -0.2

        # ── Clamp to physical limits ─────────────────────────────────────────
        linear_x  = float(np.clip(linear_x,  -self._max_lin, self._max_lin))
        angular_z = float(np.clip(angular_z, -self._max_ang, self._max_ang))

        # ── Differential drive kinematics ────────────────────────────────────
        # v_left  = (linear - angular * separation/2) / radius
        # v_right = (linear + angular * separation/2) / radius
        left_vel  = (linear_x - angular_z * self._wheel_sep / 2.0) / self._wheel_rad
        right_vel = (linear_x + angular_z * self._wheel_sep / 2.0) / self._wheel_rad

        self._apply_wheel_velocities(left_vel, right_vel)

    def stop(self) -> None:
        """Immediately zero all wheel velocities."""
        if self._initialized:
            self._apply_wheel_velocities(0.0, 0.0)

    # ── Internal ─────────────────────────────────────────────────────────────

    def _apply_wheel_velocities(
        self, left_vel: float, right_vel: float
    ) -> None:
        velocities = np.zeros(len(self._joint_names))

        for joint in LEFT_WHEEL_JOINTS:
            if joint in self._joint_names:
                velocities[self._joint_names.index(joint)] = left_vel

        for joint in RIGHT_WHEEL_JOINTS:
            if joint in self._joint_names:
                velocities[self._joint_names.index(joint)] = right_vel

        self._robot.apply_action(
            ArticulationAction(joint_velocities=velocities)
        )
