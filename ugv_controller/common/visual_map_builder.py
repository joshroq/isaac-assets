#!/usr/bin/env python3
"""
Sparse-cloud -> 2D OccupancyGrid (/map) projector — SHARED by the VISUAL nav systems.

Lives in common/ because BOTH visual-SLAM systems use it: nav_dpvo (DPVO monocular
visual odometry) and nav_orbslam3 (ORB-SLAM3 RGB-D). Those systems localize with a
camera and are excellent at producing the map->odom transform, but — unlike
slam_toolbox (nav_lidar) or RTAB-Map (nav_rgb) — they do NOT emit a Nav2-ready
occupancy grid. Their native output is a *sparse 3D point cloud* of map points.

Nav2's global costmap here uses a `static_layer` that subscribes to /map (see
ugv_nav/params/rpp.yaml), so without a /map the planner has no global costmap
and goals never plan. This node closes that gap identically for both systems: it
takes the SLAM map-point cloud (already expressed in the `map` frame by the bridge)
and projects it onto the ground plane into a nav_msgs/OccupancyGrid, ray-tracing
free space from the camera origin (looked up via TF) to each point — the same
obstacle/free/unknown semantics RTAB-Map's Grid/RayTracing produces.

Keeping this ONE projector shared (rather than baking a grid builder into each
bridge — one of which is C++) mirrors how common/depth_camera_graph.py is shared by
the two camera systems: only genuinely shared logic lives in common/.

    in:   <cloud_topic>   sensor_msgs/PointCloud2   SLAM map points, frame_id=map
    out:  /map            nav_msgs/OccupancyGrid    (latched, transient_local)

Run standalone (each visual launch does this, remapping cloud_topic):
    ros2 run ... OR: python3 visual_map_builder.py --ros-args \
        -r cloud:=/dpvo/map_points -p use_sim_time:=true
"""

import math

import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSDurabilityPolicy, QoSProfile, QoSReliabilityPolicy

from nav_msgs.msg import OccupancyGrid
from sensor_msgs.msg import PointCloud2
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
        self.declare_parameter('sensor_frame', 'oak_rgb_camera_optical_frame')
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
        # Best-effort in (matches the bridges' sensor-style publishers); latched out
        # (Nav2's static layer needs a transient-local /map to receive the last grid).
        cloud_qos = QoSProfile(depth=5)
        cloud_qos.reliability = QoSReliabilityPolicy.BEST_EFFORT
        self.sub = self.create_subscription(
            PointCloud2, self.cloud_topic, self._on_cloud, cloud_qos)

        map_qos = QoSProfile(depth=1)
        map_qos.durability = QoSDurabilityPolicy.TRANSIENT_LOCAL
        map_qos.reliability = QoSReliabilityPolicy.RELIABLE
        self.pub = self.create_publisher(OccupancyGrid, gp('map_topic').value, map_qos)

        self.create_timer(float(gp('publish_period').value), self._publish_map)
        self.get_logger().info(
            f"visual_map_builder: {self.cloud_topic} (frame {self.map_frame}) -> "
            f"{gp('map_topic').value}  [{self.n}x{self.n} @ {self.res} m, "
            f"obstacle band {self.z_min}-{self.z_max} m]")

    # ── Cloud -> hit/miss accumulation ────────────────────────────────────────────
    def _sensor_origin_cell(self):
        """Camera position in the map frame -> (i, j) cell, or None if TF not ready."""
        try:
            tf = self.tf_buffer.lookup_transform(
                self.map_frame, self.sensor_frame, rclpy.time.Time())
        except (tf2_ros.LookupException, tf2_ros.ExtrapolationException,
                tf2_ros.ConnectivityException):
            return None
        t = tf.transform.translation
        return self._world_to_cell(t.x, t.y)

    def _world_to_cell(self, x, y):
        j = int((x - self.origin_x) / self.res)
        i = int((y - self.origin_y) / self.res)
        if 0 <= i < self.n and 0 <= j < self.n:
            return (i, j)
        return None

    def _on_cloud(self, msg: PointCloud2):
        origin = self._sensor_origin_cell()
        if origin is None:
            return  # no camera pose in map yet -> can't ray-trace this frame
        oi, oj = origin

        pts = point_cloud2.read_points(
            msg, field_names=('x', 'y', 'z'), skip_nans=True)
        for x, y, z in pts:
            if not (math.isfinite(x) and math.isfinite(y) and math.isfinite(z)):
                continue
            # Reject points beyond the trusted range from the camera (VO tails).
            dx, dy = float(x), float(y)  # (origin subtraction done in cell space below)
            cell = self._world_to_cell(dx, dy)
            if cell is None:
                continue
            ci, cj = cell
            if math.hypot((ci - oi) * self.res, (cj - oj) * self.res) > self.max_ray:
                continue

            obstacle = self.z_min <= float(z) <= self.z_max
            # Free-space carving: everything between the camera and the point is free.
            for (fi, fj) in self._bresenham(oi, oj, ci, cj)[:-1]:
                self.miss[fi, fj] += 1
            if obstacle:
                self.hits[ci, cj] += 1
            else:
                self.miss[ci, cj] += 1  # floor / low return -> traversable

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
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
