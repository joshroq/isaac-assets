#!/usr/bin/env python3
"""External mission driver for the v2 visual-nav systems (DPVO / ORB-SLAM3).

WHAT IT DOES (one command, no RViz clicking)
    1. PAUSE Nav2 so nothing fights us on /cmd_vel while we initialise the SLAM.
    2. PRIME the localizer: drive the rover on /cmd_vel (a safe reactive wander
       that watches /scan) until the visual front-end starts tracking — i.e.
       until a FRESH map->odom transform appears — and until it has travelled
       far enough for DPVO's monocular scale to lock. This is the "drive around
       first" step both DPVO and ORB-SLAM3 need; here it is automatic.
    3. STARTUP Nav2 (with retries) so the lifecycle-bringup race that leaves the
       stack inactive — the reason an RViz goal does nothing — cannot strand us.
       We wait until bt_navigator is actually ACTIVE.
    4. Pick a goal ACROSS THE MAP (farthest arena corner from the rover, or an
       explicit --goal / a free cell sampled from /map) and send it through the
       NavigateToPose action.
    5. Report SUCCEEDED / FAILED with time, path-length and recovery count, then
       optionally loop to the next goal for continuous testing on randomized maps.

WHY THIS ORDER
    The DPVO / ORB-SLAM3 bridges only broadcast map->odom WHILE they are
    tracking, so "map->odom is fresh" is the single, system-independent signal
    that the localizer is alive. Priming BEFORE Nav2 is active means the
    velocity_smoother / collision_monitor are not also publishing /cmd_vel, so
    our priming commands are the only ones the rover sees. Nav2 is only brought
    up once map->odom (and therefore /map from visual_map_builder) exists, so
    its costmaps come up on a valid transform tree.

PREREQUISITES (three terminals)
    A) Isaac + baked rover, e.g.:
         isaacpy ugv_controller_v2/run_small_warehouse.py
       or the randomized-obstacle warehouse:
         isaacpy ugv_controller_v2/nav_experiment/run_small_warehouse_random_nav.py
    B) A visual nav system:
         navlaunch -v2 -dpvo        # or: navlaunch -v2 -orbslam
    C) This driver (system ROS 2 python — NOT the dpvo conda env):
         source /opt/ros/humble/setup.bash
         source ~/projects/isaac-assets/ugv_ws/install/setup.bash
         source ~/.local/bin/ros2ddsenv
         python3 ugv_controller_v2/nav_experiment/nav_mission.py            # auto goals, one run
         python3 .../nav_mission.py --loop 5                                # 5 goals in a row
         python3 .../nav_mission.py --goal 4.0 -3.0                         # explicit goal
         python3 .../nav_mission.py --no-prime                             # SLAM already tracking
"""

from __future__ import annotations

import argparse
import math
import time

import rclpy
from rclpy.duration import Duration
from rclpy.parameter import Parameter
from rclpy.qos import (
    QoSDurabilityPolicy,
    QoSHistoryPolicy,
    QoSProfile,
    QoSReliabilityPolicy,
)
from rclpy.time import Time

from geometry_msgs.msg import PoseStamped, Twist
from lifecycle_msgs.msg import State as LifecycleState
from lifecycle_msgs.srv import GetState
from nav2_msgs.srv import ManageLifecycleNodes
from nav2_simple_commander.robot_navigator import BasicNavigator, TaskResult
from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan
from tf2_ros import Buffer, TransformListener


def _yaw_from_quat(x, y, z, w) -> float:
    return math.atan2(2.0 * (w * z + x * y), 1.0 - 2.0 * (y * y + z * z))


def _quat_from_yaw(yaw: float):
    return math.sin(0.5 * yaw), math.cos(0.5 * yaw)


class NavMission(BasicNavigator):
    def __init__(self, args: argparse.Namespace):
        super().__init__("nav_mission")
        self.args = args
        # Everything is stamped against Isaac's /clock.
        self.set_parameters([Parameter("use_sim_time", Parameter.Type.BOOL, True)])

        self._tf_buffer = Buffer()
        self._tf_listener = TransformListener(self._tf_buffer, self, spin_thread=False)

        self._cmd_pub = self.create_publisher(Twist, args.cmd_vel_topic, 10)

        sensor_qos = QoSProfile(
            depth=5,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.BEST_EFFORT,
        )
        self._scan = None
        self.create_subscription(LaserScan, args.scan_topic, self._on_scan, sensor_qos)

        map_qos = QoSProfile(
            depth=1,
            history=QoSHistoryPolicy.KEEP_LAST,
            reliability=QoSReliabilityPolicy.RELIABLE,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
        )
        self._map = None
        self.create_subscription(OccupancyGrid, args.map_topic, self._on_map, map_qos)

        mgr = args.lifecycle_manager
        self._manage_cli = self.create_client(
            ManageLifecycleNodes, f"/{mgr}/manage_nodes"
        )
        self._state_cli = self.create_client(GetState, f"/{args.navigator}/get_state")

    # ── callbacks / spinning ────────────────────────────────────────────────
    def _on_scan(self, msg: LaserScan):
        self._scan = msg

    def _on_map(self, msg: OccupancyGrid):
        self._map = msg

    def _spin(self, seconds: float) -> None:
        end = time.monotonic() + max(0.0, seconds)
        while rclpy.ok() and time.monotonic() < end:
            rclpy.spin_once(self, timeout_sec=0.05)

    # ── TF helpers ──────────────────────────────────────────────────────────
    def _lookup(self, target: str, source: str):
        try:
            return self._tf_buffer.lookup_transform(target, source, Time())
        except Exception:
            return None

    def _slam_age(self):
        """Seconds since the last map->odom, or None if it was never seen.

        A fresh map->odom is the localizer's 'I am tracking' heartbeat, shared by
        both the DPVO and ORB-SLAM3 bridges."""
        tf = self._lookup(self.args.map_frame, self.args.odom_frame)
        if tf is None:
            return None
        now = self.get_clock().now()
        if now.nanoseconds == 0:
            return None  # /clock not received yet; don't report false freshness
        stamp = Time.from_msg(tf.header.stamp)
        return max(0.0, (now - stamp).nanoseconds * 1e-9)

    def _slam_tracking(self) -> bool:
        age = self._slam_age()
        return age is not None and age <= self.args.slam_max_age

    def _robot_pose(self, frame: str):
        """(x, y, yaw) of base in `frame`, or None."""
        tf = self._lookup(frame, self.args.base_frame)
        if tf is None:
            return None
        t = tf.transform.translation
        r = tf.transform.rotation
        return t.x, t.y, _yaw_from_quat(r.x, r.y, r.z, r.w)

    # ── lifecycle ───────────────────────────────────────────────────────────
    def _manage(self, command: int, label: str) -> bool:
        if not self._manage_cli.wait_for_service(timeout_sec=5.0):
            self.warn(f"lifecycle manager service not up; cannot {label}")
            return False
        req = ManageLifecycleNodes.Request()
        req.command = command
        fut = self._manage_cli.call_async(req)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=30.0)
        ok = bool(fut.result() and fut.result().success)
        self.info(f"lifecycle {label}: {'ok' if ok else 'FAILED'}")
        return ok

    def _navigator_state(self):
        """Current lifecycle state id of the navigator, or None if unreachable."""
        if not self._state_cli.service_is_ready():
            self._state_cli.wait_for_service(timeout_sec=2.0)
        if not self._state_cli.service_is_ready():
            return None
        fut = self._state_cli.call_async(GetState.Request())
        rclpy.spin_until_future_complete(self, fut, timeout_sec=3.0)
        res = fut.result()
        return res.current_state.id if res else None

    def _navigator_active(self, timeout: float) -> bool:
        if not self._state_cli.wait_for_service(timeout_sec=timeout):
            return False
        deadline = time.monotonic() + timeout
        while rclpy.ok() and time.monotonic() < deadline:
            if self._navigator_state() == LifecycleState.PRIMARY_STATE_ACTIVE:
                return True
            self._spin(0.5)
        return False

    def pause_nav(self) -> None:
        # Only PAUSE if Nav2 actually autostarted; otherwise pushing a PAUSE onto
        # an unconfigured/aborted stack just logs errors. When active, PAUSE stops
        # the smoother/collision_monitor publishing /cmd_vel during priming.
        if self._navigator_state() == LifecycleState.PRIMARY_STATE_ACTIVE:
            self._manage(ManageLifecycleNodes.Request.PAUSE, "pause")

    def startup_nav(self) -> bool:
        """Send STARTUP and confirm bt_navigator is ACTIVE, retrying to beat the
        lifecycle bringup race seen under Isaac + GPU-SLAM CPU load."""
        for attempt in range(1, self.args.startup_retries + 1):
            self.info(f"Nav2 STARTUP attempt {attempt}/{self.args.startup_retries}…")
            self._manage(ManageLifecycleNodes.Request.STARTUP, "startup")
            if self._navigator_active(self.args.startup_timeout):
                self.info(f"Nav2 active ({self.args.navigator} reached ACTIVE).")
                return True
            self.warn(f"{self.args.navigator} not active yet; retrying bringup.")
        return False

    # ── priming ─────────────────────────────────────────────────────────────
    def _front_clearance(self) -> float:
        """Min /scan range in a ±30° front sector, or +inf if no scan yet."""
        scan = self._scan
        if scan is None or not scan.ranges:
            return math.inf
        best = math.inf
        n = len(scan.ranges)
        for i, r in enumerate(scan.ranges):
            if not math.isfinite(r) or r <= scan.range_min:
                continue
            ang = scan.angle_min + i * scan.angle_increment
            if -math.pi / 6.0 <= ang <= math.pi / 6.0:
                best = min(best, r)
        return best

    def _drive(self, lin: float, ang: float) -> None:
        msg = Twist()
        msg.linear.x = lin
        msg.angular.z = ang
        self._cmd_pub.publish(msg)

    def stop(self, pulses: int = 5) -> None:
        for _ in range(pulses):
            self._drive(0.0, 0.0)
            self._spin(0.05)

    def prime(self) -> bool:
        """Reactively wander (watching /scan) until the localizer tracks and the
        rover has driven far enough for monocular scale. Returns True on success."""
        self.info("Priming visual localizer: driving to initialise tracking…")
        start = None
        # Wait briefly for odom so we can measure travelled distance.
        for _ in range(40):
            start = self._robot_pose(self.args.odom_frame)
            if start is not None:
                break
            self._spin(0.1)
        if start is None:
            self.warn("No odom TF while priming; is Isaac publishing /tf?")
        travelled = 0.0
        deadline = time.monotonic() + self.args.prime_timeout
        wiggle = 0.0
        while rclpy.ok() and time.monotonic() < deadline:
            clear = self._front_clearance()
            if clear < self.args.prime_stop_range:
                # Obstacle ahead — rotate in place to find open, textured space.
                self._drive(0.0, self.args.prime_turn)
            else:
                # Mostly-forward with a slow yaw weave: forward gives the monocular
                # parallax DPVO needs; the weave spreads features for ORB-SLAM3.
                wiggle += 0.15
                self._drive(self.args.prime_speed, 0.25 * math.sin(wiggle))
            self._spin(0.1)

            pose = self._robot_pose(self.args.odom_frame)
            if pose is not None and start is not None:
                travelled = math.hypot(pose[0] - start[0], pose[1] - start[1])
            if self._slam_tracking() and travelled >= self.args.prime_min_travel:
                self.stop()
                self.info(
                    f"Localizer tracking (map->odom fresh) after {travelled:.2f} m."
                )
                return True
        self.stop()
        age = self._slam_age()
        self.warn(
            "Priming timed out: "
            + (
                f"map->odom last seen {age:.1f}s ago"
                if age is not None
                else "map->odom never appeared"
            )
            + f", travelled {travelled:.2f} m. The scene may lack texture or the "
            "camera stream is too slow/jittery for the front-end to initialise."
        )
        return False

    # ── goal selection ──────────────────────────────────────────────────────
    def _make_goal(self, x: float, y: float, ref) -> PoseStamped:
        goal = PoseStamped()
        goal.header.frame_id = self.args.map_frame
        goal.header.stamp = self.get_clock().now().to_msg()
        goal.pose.position.x = float(x)
        goal.pose.position.y = float(y)
        yaw = math.atan2(y - ref[1], x - ref[0]) if ref is not None else 0.0
        qz, qw = _quat_from_yaw(yaw)
        goal.pose.orientation.z = qz
        goal.pose.orientation.w = qw
        return goal

    def _arena_corners(self):
        xmin, ymin, xmax, ymax = self.args.arena
        return [(xmin, ymin), (xmin, ymax), (xmax, ymin), (xmax, ymax)]

    def _farthest_free_map_cell(self, ref):
        """Farthest FREE (value 0) /map cell from `ref`, or None if the map is
        too sparse to trust."""
        grid = self._map
        if grid is None:
            return None
        info = grid.info
        res, w, h = info.resolution, info.width, info.height
        ox, oy = info.origin.position.x, info.origin.position.y
        best, best_d = None, -1.0
        data = grid.data
        # Coarse stride keeps this cheap on an 800x800 grid.
        step = max(1, int(0.25 / res))
        for gy in range(0, h, step):
            row = gy * w
            for gx in range(0, w, step):
                if data[row + gx] != 0:  # 0 == known free; skip unknown/occupied
                    continue
                wx = ox + (gx + 0.5) * res
                wy = oy + (gy + 0.5) * res
                d = math.hypot(wx - ref[0], wy - ref[1])
                if d > best_d:
                    best, best_d = (wx, wy), d
        # Require a meaningful traverse, else the map is basically empty.
        return best if best_d >= self.args.min_goal_distance else None

    def choose_goal(self) -> PoseStamped | None:
        ref = self._robot_pose(self.args.map_frame)
        if ref is None:
            self.warn("No map->base TF; cannot place a map-frame goal yet.")
            return None
        if self.args.goal is not None:
            gx, gy = self.args.goal
            return self._make_goal(gx, gy, ref)
        if self.args.auto_map:
            cell = self._farthest_free_map_cell(ref)
            if cell is not None:
                return self._make_goal(cell[0], cell[1], ref)
            self.info("Map too sparse for cell sampling; using arena corners.")
        # Default: farthest arena corner from the rover — a goal 'across the map'.
        corner = max(self._arena_corners(), key=lambda c: math.hypot(c[0] - ref[0], c[1] - ref[1]))
        return self._make_goal(corner[0], corner[1], ref)

    # ── run one goal ────────────────────────────────────────────────────────
    def run_goal(self, goal: PoseStamped) -> dict:
        ref = self._robot_pose(self.args.map_frame) or (0.0, 0.0, 0.0)
        straight = math.hypot(goal.pose.position.x - ref[0], goal.pose.position.y - ref[1])
        self.info(
            f"Goal: ({goal.pose.position.x:.2f}, {goal.pose.position.y:.2f}) "
            f"[{straight:.2f} m away]"
        )
        t0 = time.monotonic()
        self.goToPose(goal)
        recoveries, remaining = 0, straight
        while not self.isTaskComplete():
            fb = self.getFeedback()
            if fb is not None:
                remaining = fb.distance_remaining
                recoveries = fb.number_of_recoveries
                if not self._slam_tracking():
                    self.warn("map->odom went stale mid-run — localizer lost tracking.")
            if time.monotonic() - t0 > self.args.nav_timeout:
                self.warn("Navigation timeout — cancelling this goal.")
                self.cancelTask()
                break
            self._spin(0.2)
        result = self.getResult()
        return {
            "result": result,
            "seconds": time.monotonic() - t0,
            "straight_m": straight,
            "remaining_m": remaining,
            "recoveries": recoveries,
        }

    def run(self) -> int:
        # Let discovery + /clock settle so timestamps are sim-time from the start.
        self._spin(1.0)

        if not self.args.no_prime:
            self.pause_nav()
            self.prime()  # warn-and-continue on failure; Nav2 may still recover

        if not self.startup_nav():
            self.error("Could not bring Nav2 to ACTIVE. Aborting mission.")
            return 1

        # Give costmaps a moment to ingest /map + the first scans.
        self._spin(self.args.settle)

        if self.args.loop <= 0:
            self.info("Nav2 is ACTIVE and will stay up — send goals from RViz "
                      "(2D Goal Pose) or rerun with --loop N to auto-drive.")
            return 0

        runs, failures = 0, 0
        for i in range(self.args.loop):
            if not self._slam_tracking() and not self.args.no_prime:
                self.warn(f"[{i + 1}/{self.args.loop}] Localizer not tracking; re-priming.")
                self.pause_nav()
                self.prime()
                self.startup_nav()
            goal = self.choose_goal()
            if goal is None:
                self.warn(f"[{i + 1}/{self.args.loop}] No goal available; skipping.")
                self._spin(1.0)
                continue
            m = self.run_goal(goal)
            runs += 1
            ok = m["result"] == TaskResult.SUCCEEDED
            failures += 0 if ok else 1
            self.info(
                f"[{i + 1}/{self.args.loop}] {m['result'].name}  "
                f"t={m['seconds']:.1f}s straight={m['straight_m']:.2f}m "
                f"remaining={m['remaining_m']:.2f}m recoveries={m['recoveries']}"
            )
        self.info(f"Mission done: {runs - failures}/{runs} goals SUCCEEDED.")
        return 0 if failures == 0 else 2


def _parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    p.add_argument("--goal", type=float, nargs=2, metavar=("X", "Y"), default=None,
                   help="Explicit map-frame goal. Omit to auto-pick across the map.")
    p.add_argument("--auto-map", action="store_true",
                   help="Auto goal: sample the farthest FREE cell from /map instead "
                        "of using arena corners (falls back to corners if map is sparse).")
    p.add_argument("--arena", type=float, nargs=4, metavar=("XMIN", "YMIN", "XMAX", "YMAX"),
                   default=(-4.5, -4.5, 4.5, 4.5),
                   help="Bounds used for auto corner goals (match the obstacle field).")
    p.add_argument("--loop", type=int, default=1, help="Number of goals to run in sequence.")
    p.add_argument("--min-goal-distance", type=float, default=3.0,
                   help="Reject auto goals closer than this (m).")

    p.add_argument("--no-prime", action="store_true",
                   help="Skip the drive-to-initialise step (localizer already tracking).")
    p.add_argument("--prime-speed", type=float, default=0.16, help="Priming forward speed (m/s).")
    p.add_argument("--prime-turn", type=float, default=0.5, help="Priming in-place turn (rad/s).")
    p.add_argument("--prime-stop-range", type=float, default=0.8,
                   help="Front /scan clearance (m) below which priming turns instead of advancing.")
    p.add_argument("--prime-min-travel", type=float, default=0.8,
                   help="Metres to drive before trusting tracking (DPVO scale lock).")
    p.add_argument("--prime-timeout", type=float, default=60.0, help="Max priming seconds.")

    p.add_argument("--slam-max-age", type=float, default=1.0,
                   help="map->odom older than this (s) counts as 'not tracking'.")
    p.add_argument("--nav-timeout", type=float, default=180.0,
                   help="Per-goal navigation timeout (s) before cancelling.")
    p.add_argument("--settle", type=float, default=3.0,
                   help="Seconds to wait after Nav2 activates before the first goal.")
    p.add_argument("--startup-retries", type=int, default=5,
                   help="Nav2 STARTUP attempts to beat the lifecycle bringup race.")
    p.add_argument("--startup-timeout", type=float, default=25.0,
                   help="Seconds to wait for the navigator to reach ACTIVE per attempt.")

    p.add_argument("--lifecycle-manager", default="lifecycle_manager_navigation")
    p.add_argument("--navigator", default="bt_navigator")
    p.add_argument("--cmd-vel-topic", default="/cmd_vel")
    p.add_argument("--scan-topic", default="/ugv_sim/scan")
    p.add_argument("--map-topic", default="/map")
    p.add_argument("--map-frame", default="map")
    p.add_argument("--odom-frame", default="odom")
    p.add_argument("--base-frame", default="base_footprint")
    return p.parse_args()


def main() -> int:
    args = _parse_args()
    rclpy.init()
    mission = NavMission(args)
    try:
        return mission.run()
    except KeyboardInterrupt:
        mission.stop()
        return 130
    finally:
        mission.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    raise SystemExit(main())
