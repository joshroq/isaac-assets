# nav_dpvo_archived — ARCHIVED / INACTIVE

**Not a Part 1 stack.** DPVO (Deep Patch Visual Odometry — learned, monocular visual
odometry) is **paused** for the Part 1 SLAM/Nav2 fidelity baseline. It is preserved
here, unchanged, for the later policy-training/generalization / visual-odometry
comparison phase. Nothing is deleted — the DPVO source
(`~/projects/slam_source/DPVO-main`), the `dpvo` conda env, and this bridge all stay.

Summary of this archived stack (for the record):

- **Localization / map:** DPVO monocular visual odometry → `map->odom`
  (sparse patch cloud → `common/visual_map_builder.py` → `/map`)
- **Obstacle/costmap source:** LiDAR `/scan` (upstream Nav2 obstacle layer)
- **RGB:** yes (monocular) · **Depth:** no · **LiDAR:** yes (costmap only) · **IMU:** no
- **Role:** learned monocular VO comparison (scale-ambiguous, no loop closure)

To run it anyway (comparison only):

```bash
ugv_controller_v2/nav_systems/nav_dpvo_archived/setup.sh    # env check (no build)
ros2 launch ugv_controller_v2/nav_systems/nav_dpvo_archived/isaac_nav_dpvo.launch.py
```
