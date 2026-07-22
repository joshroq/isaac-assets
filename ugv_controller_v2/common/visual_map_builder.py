#!/usr/bin/env python3
"""
Sparse-cloud -> 2D OccupancyGrid (/map) projector — SHARED by the VISUAL nav systems.

Lives in common/ because BOTH visual-SLAM systems use it: nav_orbslam3 (ORB-SLAM3 RGB-D)
and nav_dpvo (DPVO monocular visual odometry). Those systems localize with a camera and
are excellent at producing the map->odom transform, but — unlike slam_toolbox (nav_lidar)
or RTAB-Map (nav_rgb) — they do NOT emit a Nav2-ready occupancy grid. Their native map
output is a *sparse 3D point cloud* (ORB map points / DPVO patch points).

Nav2's global costmap here uses a `static_layer` that subscribes to /map (see
ugv_nav/params/*.yaml), so without a /map the planner has no global costmap and goals
never plan. This node closes that gap identically for both systems: it takes the SLAM
map-point cloud (already expressed in the `map` frame by the bridge) and projects it onto
the ground plane into a nav_msgs/OccupancyGrid, ray-tracing free space from the camera
origin (looked up via TF) to each point — the same obstacle/free/unknown semantics
RTAB-Map's Grid/RayTracing produces.

OPTIONAL 2D-LIDAR FUSION (`scan_topic`, off by default). A sparse visual cloud is a thin
seed for the initial /map: ORB-SLAM3 / DPVO must see and triangulate enough features
before the grid has any coverage, so early goals fail to plan. When `scan_topic` is set,
this node ALSO folds the baked rover's 2D lidar /scan into the very same hit/miss grid —
each finite return is an obstacle, each ray from the laser to its endpoint (or out to the
trust range for no-returns) is carved free. That makes the initial mapping dense and
metric from the first scan. Here the lidar shapes MAP GEOMETRY only — whether it also
feeds LOCALIZATION is a separate choice each system makes elsewhere (nav_orbslam3 fuses it
into map->odom via rf2o+EKF; nav_dpvo does not). nav_orbslam3 opts this builder in;
nav_dpvo leaves it off (default '') and behaves exactly as before. The lidar sits in the
map frame via map->odom -> ... -> laser_frame (from Isaac), looked up per scan.

Keeping ONE projector shared (rather than baking a grid builder into each bridge — one of
which is C++) mirrors the v1 tree, which keeps it in common/ for exactly this reason.
Frame defaults are the v2 baked-USD ones (see ugv_controller_v2/USD_CONTRACT.md).

    in:   <cloud_topic>   sensor_msgs/PointCloud2   SLAM map points, frame_id=map
    in:   <scan_topic>    sensor_msgs/LaserScan     2D lidar (optional, frame=laser_frame)
    out:  /map            nav_msgs/OccupancyGrid    (latched, transient_local)

Run standalone (each visual launch does this, remapping cloud):
    python3 visual_map_builder.py --ros-args \
        -r cloud:=/orbslam3/map_points -p use_sim_time:=true   # or /dpvo/map_points
    # fuse lidar too (nav_orbslam3):
    python3 visual_map_builder.py --ros-args -r cloud:=/orbslam3/map_points \
        -p scan_topic:=/ugv_sim/scan -p laser_frame:=base_lidar_link -p use_sim_time:=true
"""

import math

import numpy as np
import rclpy
from rclpy.executors import ExternalShutdownException
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import LaserScan, PointCloud2
from sensor_msgs_py import point_cloud2

import tf2_ros


class VisualMapBuilder(Node):
    def __init__(self):
        super().__init__('visual_map_builder')

        # ── Parameters ──────────────────────────────────────────────────────────
        self.declare_parameter('cloud_topic', 'cloud')
        self.declare_parameter('map_topic', '/map')
        self.declare_parameter('map_frame', 'map')
        # The camera optical frame the map points are seen from (ray-trace origin).
        # v2 baked-USD RGB optical frame; it is published on /tf by Isaac.
        self.declare_parameter('sensor_frame', 'Camera_OmniVision_OV9782_Color')
        self.declare_parameter('resolution', 0.05)          # m / cell (matches slam_nav)
        self.declare_parameter('size_cells', 800)           # 800*0.05 = 40 m square
        # Obstacle height band (m above the map ground plane). Points below z_min are
        # floor (traced as free), above z_max are ceiling/noise (ignored as obstacles).
        self.declare_parameter('obstacle_z_min', 0.08)
        self.declare_parameter('obstacle_z_max', 1.5)
        self.declare_parameter('max_ray_range', 8.0)        # trust range; VO points get noisy far out
        self.declare_parameter('occupied_thresh', 0.60)     # hit/(hit+miss) -> occupied
        self.declare_parameter('free_thresh', 0.30)         # below -> free
        self.declare_parameter('publish_period', 1.0)       # s
        # ── Optional 2D-lidar fusion (off unless scan_topic is set) ───────────────
        # When set, /scan is folded into the same grid for a dense metric initial map
        # (see module docstring). laser_frame is the 2D lidar's TF frame; scan_max_range
        # is how far a lidar return is trusted (>= the visual max_ray_range, since the
        # lidar stays metric far past where VO points get noisy).
        self.declare_parameter('scan_topic', '')
        self.declare_parameter('laser_frame', 'base_lidar_link')
        self.declare_parameter('scan_max_range', 12.0)

        gp = self.get_parameter
        self.cloud_topic = gp('cloud_topic').value
        self.map_frame = gp('map_frame').value
        self.sensor_frame = gp('sensor_frame').value
        self.res = float(gp('resolution').value)
        self.n = int(gp('size_cells').value)
        self.z_min = float(gp('obstacle_z_min').value)
        self.z_max = float(gp('obstacle_z_max').value)
        self.max_ray = float(gp('max_ray_range').value)
        self.occ_thresh = float(gp('occupied_thresh').value)
        self.free_thresh = float(gp('free_thresh').value)
        self.scan_topic = gp('scan_topic').value
        self.laser_frame = gp('laser_frame').value
        self.scan_max_range = float(gp('scan_max_range').value)

        # Grid centred on the map origin: cell (i,j) covers world
        #   x = origin_x + j*res,  y = origin_y + i*res.
        self.origin_x = -0.5 * self.n * self.res
        self.origin_y = -0.5 * self.n * self.res

        # Per-cell hit / miss counters -> occupancy probability. int32 to avoid overflow.
        self.hits = np.zeros((self.n, self.n), dtype=np.int32)
        self.miss = np.zeros((self.n, self.n), dtype=np.int32)

        # ── TF (for the ray-trace origin = camera position in map) ───────────────
        self.tf_buffer = tf2_ros.Buffer()
        self.tf_listener = tf2_ros.TransformListener(self.tf_buffer, self)

        # ── I/O ──────────────────────────────────────────────────────────────────
        # Best-effort in (matches the bridge's sensor-style publisher); latched out
        # (Nav2's static layer needs a transient-local /map to receive the last grid).
        cloud_qos = QoSProfile(depth=5)
        cloud_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, cloud_qos)

        # Optional lidar input, same best-effort sensor QoS as Isaac's LaserScanPublish.
        self.scan_sub = None
        if self.scan_topic:
            scan_qos = QoSProfile(depth=5)
            scan_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
            self.scan_sub = self.create_subscription(
                LaserScan, self.scan_topic, self._on_scan, scan_qos)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, gp('map_topic').value, map_qos)

        self.create_timer(float(gp('publish_period').value), self._publish_map)
        scan_info = (f", + lidar {self.scan_topic} (frame {self.laser_frame}, "
                     f"<= {self.scan_max_range} m)" if self.scan_topic else "")
        self.get_logger().info(
            f"visual_map_builder: {self.cloud_topic} (frame {self.map_frame}) -> "
            f"{gp('map_topic').value}  [{self.n}x{self.n} @ {self.res} m, "
            f"obstacle band {self.z_min}-{self.z_max} m, sensor {self.sensor_frame}"
            f"{scan_info}]")

    # ── Cloud / scan -> hit/miss accumulation ─────────────────────────────────────
    def _sensor_origin_cell(self, frame):
        """Position of `frame` in the map frame -> (i, j) cell, or None if TF unready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        return self._world_to_cell(t.x, t.y)

    def _map_pose(self, frame):
        """(tx, ty, yaw) of `frame` in the map frame, or None if TF not ready. The 2D
        pose is all a planar lidar needs to place its scan in the map frame."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        q = tf.transform.rotation
        yaw = math.atan2(2.0 * (q.w * q.z + q.x * q.y),
                         1.0 - 2.0 * (q.y * q.y + q.z * q.z))
        return (t.x, t.y, yaw)

    def _world_to_cell(self, x, y):
        j = int((x - self.origin_x) / self.res)
        i = int((y - self.origin_y) / self.res)
        if 0 <= i < self.n and 0 <= j < self.n:
            return (i, j)
        return None

    def _accumulate_ray(self, oi, oj, ci, cj, obstacle):
        """Carve free space from the sensor cell (oi,oj) to the endpoint (ci,cj), then
        mark the endpoint an obstacle (hit) or free (miss). Shared by cloud and scan."""
        for (fi, fj) in self._bresenham(oi, oj, ci, cj)[:-1]:
            self.miss[fi, fj] += 1
        if obstacle:
            self.hits[ci, cj] += 1
        else:
            self.miss[ci, cj] += 1

    def _on_cloud(self, msg: PointCloud2):
        origin = self._sensor_origin_cell(self.sensor_frame)
        if origin is None:
            return  # no camera pose in map yet -> can't ray-trace this frame
        oi, oj = origin

        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        for x, y, z in pts:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            cell = self._world_to_cell(float(x), float(y))
            if cell is None:
                continue
            ci, cj = cell
            # Reject points beyond the trusted range from the camera (VO tails).
            if math.hypot((ci - oi) * self.res, (cj - oj) * self.res) > self.max_ray:
                continue

            # z-band: below z_min is floor, above z_max is ceiling/noise -> traversable.
            obstacle = self.z_min <= float(z) <= self.z_max
            self._accumulate_ray(oi, oj, ci, cj, obstacle)

    def _on_scan(self, msg: LaserScan):
        """Fold a 2D lidar scan into the grid (opt-in; see module docstring). The laser's
        map pose comes from map->odom (visual SLAM) -> ... -> laser_frame (Isaac), so the
        scan lands in the same map frame as the cloud. Every finite in-range return is an
        obstacle; no-return beams carve free space out to the trust range."""
        pose = self._map_pose(self.laser_frame)
        if pose is None:
            return  # no laser pose in map yet (visual SLAM hasn't seeded map->odom)
        tx, ty, yaw = pose
        origin = self._world_to_cell(tx, ty)
        if origin is None:
            return
        oi, oj = origin

        cos_y, sin_y = math.cos(yaw), math.sin(yaw)
        r_max = min(self.scan_max_range, float(msg.range_max))
        r_min = float(msg.range_min)
        angle = float(msg.angle_min)
        for r in msg.ranges:
            a = angle
            angle += float(msg.angle_increment)
            finite = math.isfinite(r) and r_min <= r <= r_max
            # A real return is an obstacle at its endpoint; a no-return (inf / out of
            # range) still tells us the beam's path up to r_max is clear -> carve free.
            reach = r if finite else r_max
            # Endpoint in the laser frame, rotated by yaw and shifted to the map frame.
            lx = reach * math.cos(a)
            ly = reach * math.sin(a)
            cell = self._world_to_cell(tx + cos_y * lx - sin_y * ly,
                                       ty + sin_y * lx + cos_y * ly)
            if cell is None:
                continue
            ci, cj = cell
            self._accumulate_ray(oi, oj, ci, cj, obstacle=finite)

    @staticmethod
    def _bresenham(i0, j0, i1, j1):
        """Integer line from (i0,j0) to (i1,j1) inclusive (grid ray tracing)."""
        cells = []
        di = abs(i1 - i0)
        dj = abs(j1 - j0)
        si = 1 if i0 < i1 else -1
        sj = 1 if j0 < j1 else -1
        err = di - dj
        i, j = i0, j0
        while True:
            cells.append((i, j))
            if i == i1 and j == j1:
                break
            e2 = 2 * err
            if e2 > -dj:
                err -= dj
                i += si
            if e2 < di:
                err += di
                j += sj
        return cells

    # ── Publish ───────────────────────────────────────────────────────────────────
    def _publish_map(self):
        total = self.hits + self.miss
        grid = np.full((self.n, self.n), -1, dtype=np.int8)  # unknown
        seen = total > 0
        prob = np.zeros_like(self.hits, dtype=np.float32)
        np.divide(self.hits, np.maximum(total, 1), out=prob)
        grid[seen & (prob >= self.occ_thresh)] = 100
        grid[seen & (prob < self.free_thresh)] = 0
        # cells seen but ambiguous stay -1 (unknown) — Nav2 treats them as no-info.

        msg = OccupancyGrid()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = self.map_frame
        msg.info.resolution = self.res
        msg.info.width = self.n
        msg.info.height = self.n
        msg.info.origin.position.x = self.origin_x
        msg.info.origin.position.y = self.origin_y
        msg.info.origin.orientation.w = 1.0
        msg.data = grid.reshape(-1).tolist()
        self.pub.publish(msg)


def main():
    rclpy.init()
    node = VisualMapBuilder()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        # ExternalShutdownException is what rclpy raises when launch SIGINT/SIGTERMs us
        # on Ctrl-C; without it every shutdown of this system prints a stack trace.
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
