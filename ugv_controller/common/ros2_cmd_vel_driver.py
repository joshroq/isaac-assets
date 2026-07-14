"""
ROS 2 /cmd_vel subscriber that drives the Isaac rover through RoverDriveController.

The real rover's ugv_driver.py subscribes to geometry_msgs/Twist and forwards the
linear.x / angular.z pair directly to the chassis controller, with only a small
in-place angular deadband boost. This module keeps the Isaac Nav2 path on that
same contract instead of using Isaac's stock DifferentialController node, so
keyboard control and ROS 2 control share the same wheel conversion code.
"""

from __future__ import annotations

import time


def _ensure_isaac_rclpy() -> None:
    """Make ``import rclpy`` resolve to Isaac Sim's bundled (Python 3.12) rclpy.

    When a *system* ROS 2 (built for Python 3.10) is sourced in the launch shell,
    its rclpy shadows Isaac's on ``sys.path`` and ``import rclpy`` fails with
    "No module named 'rclpy._rclpy_pybind11'" (wrong Python ABI). Isaac ships a
    matching py3.12 rclpy inside the ``isaacsim.ros2.core`` extension; prefer it.

    No-op when rclpy already imports (e.g. launched via ``isaacpy_clean.sh``), and
    best-effort otherwise — on any failure the caller's ``import rclpy`` raises and
    the runner falls back to keyboard-only, exactly as before.
    """
    try:
        import rclpy  # noqa: F401
        return
    except Exception:
        pass

    import importlib
    import os
    import sys

    def _strip_ros_paths(value: str) -> str:
        parts = []
        for part in value.split(os.pathsep):
            if not part or "/opt/ros/" in part or "/ros2_ws/" in part:
                continue
            parts.append(part)
        return os.pathsep.join(parts)

    try:
        import isaacsim.ros2.core as _core
        # <ext>/isaacsim/ros2/core/__init__.py  ->  <ext>
        ext_root = os.path.dirname(
            os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(_core.__file__))))
        )
    except Exception:
        ext_root = os.path.expanduser("~/Applications/isaacsim/exts/isaacsim.ros2.core")
        if not os.path.isdir(ext_root):
            return  # ROS 2 bridge extension unavailable; let the caller's import raise

    distro = os.environ.get("ROS_DISTRO", "humble")
    bundled = os.path.join(ext_root, distro, "rclpy")
    bundled_lib = os.path.join(ext_root, distro, "lib")
    if not os.path.isdir(bundled):
        return

    # Drop system-ROS entries so the bundled py3.12 packages win, then prepend
    # the bundled Python and C library locations.
    for key in ("AMENT_PREFIX_PATH", "AMENT_CURRENT_PREFIX", "CMAKE_PREFIX_PATH", "COLCON_PREFIX_PATH"):
        os.environ.pop(key, None)
    os.environ["PYTHONPATH"] = bundled
    if os.path.isdir(bundled_lib):
        existing_ld = _strip_ros_paths(os.environ.get("LD_LIBRARY_PATH", ""))
        os.environ["LD_LIBRARY_PATH"] = (
            bundled_lib if not existing_ld else bundled_lib + os.pathsep + existing_ld
        )

    sys.path[:] = [p for p in sys.path if "/opt/ros/" not in p and "/ros2_ws/" not in p]
    if bundled not in sys.path:
        sys.path.insert(0, bundled)

    # Purge any half-loaded system rclpy so the bundled one imports cleanly.
    for _name in [m for m in sys.modules if m == "rclpy" or m.startswith("rclpy.")]:
        del sys.modules[_name]
    importlib.invalidate_caches()
    print("[Ros2CmdVelDriver] Using Isaac's bundled rclpy at", bundled)


class Ros2CmdVelDriver:
    """
    Non-blocking rclpy subscriber for /cmd_vel.

    Call update() from the Isaac physics loop. It spins rclpy once, applies the
    latest command through RoverDriveController.drive() while enabled, and stops
    the rover if commands go stale.
    """

    def __init__(
        self,
        drive_controller,
        topic_name: str = "/cmd_vel",
        timeout_sec: float = 0.5,
        node_name: str = "isaac_rover_cmd_vel_driver",
    ):
        self._drive = drive_controller
        self._topic_name = topic_name
        self._timeout_sec = timeout_sec
        self._enabled = False
        self._latest_linear = 0.0
        self._latest_angular = 0.0
        self._last_msg_time = 0.0
        self._stopped_for_timeout = True
        self._shutdown = False

        _ensure_isaac_rclpy()
        import rclpy
        from geometry_msgs.msg import Twist

        self._rclpy = rclpy
        if not rclpy.ok():
            rclpy.init(args=None)
            self._owns_rclpy = True
        else:
            self._owns_rclpy = False

        self._node = rclpy.create_node(node_name)
        self._sub = self._node.create_subscription(
            Twist,
            topic_name,
            self._on_twist,
            10,
        )

        print(
            "[Ros2CmdVelDriver] Subscribed to "
            f"{topic_name}; wheel output uses RoverDriveController"
        )

    @property
    def enabled(self) -> bool:
        return self._enabled

    def set_enabled(self, enabled: bool) -> None:
        enabled = bool(enabled)
        if self._enabled == enabled:
            return

        self._enabled = enabled
        if not enabled:
            self._drive.stop()
            self._stopped_for_timeout = True
        else:
            self._last_msg_time = 0.0
            self._latest_linear = 0.0
            self._latest_angular = 0.0
            self._stopped_for_timeout = True

        print(
            "[Ros2CmdVelDriver] "
            + ("ENABLED (ROS 2 /cmd_vel drives wheels)"
               if enabled else "DISABLED (keyboard owns wheels)")
        )

    def update(self) -> None:
        if self._shutdown:
            return

        self._rclpy.spin_once(self._node, timeout_sec=0.0)

        if not self._enabled:
            return

        if self._last_msg_time <= 0.0:
            return

        if time.monotonic() - self._last_msg_time > self._timeout_sec:
            if not self._stopped_for_timeout:
                self._drive.stop()
                self._stopped_for_timeout = True
            return

        self._drive.drive(self._latest_linear, self._latest_angular)
        self._stopped_for_timeout = False

    def shutdown(self) -> None:
        if self._shutdown:
            return
        self._shutdown = True

        try:
            self._drive.stop()
        except Exception as exc:
            print(f"[Ros2CmdVelDriver] Warning during stop: {exc}")

        try:
            self._node.destroy_subscription(self._sub)
            self._node.destroy_node()
        except Exception as exc:
            print(f"[Ros2CmdVelDriver] Warning during node cleanup: {exc}")

        if self._owns_rclpy:
            try:
                self._rclpy.shutdown()
            except Exception as exc:
                print(f"[Ros2CmdVelDriver] Warning during rclpy shutdown: {exc}")

    def _on_twist(self, msg) -> None:
        self._latest_linear = float(msg.linear.x)
        self._latest_angular = float(msg.angular.z)
        self._last_msg_time = time.monotonic()
