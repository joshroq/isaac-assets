# nav_rgb — RTAB-Map RGB-D + LiDAR + Nav2

This is the active `nav_rgb` RTAB-Map stack. RTAB-Map fuses the **2D LiDAR
`/scan`** together with RGB-D (`subscribe_scan: True`, `/scan` remapped into the
RTAB-Map group), matching the archived LiDAR-capable implementation restored into
this directory.

Summary:

- **Localization / map:** RTAB-Map fusing RGB-D **+ LiDAR `/scan`**
- **Obstacle/costmap source:** LiDAR `/scan` (upstream Nav2 obstacle layer) + RGB-D
- **RGB:** yes
- **Depth:** yes
- **LiDAR:** yes
- **IMU:** no

Run:

```bash
ros2 launch ugv_controller_v2/nav_systems/nav_rgb/isaac_nav_rgb.launch.py
```
