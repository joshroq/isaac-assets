# Isaac Assets

Isaac Sim rover assets, ROS 2 Humble workspaces, and navigation launch helpers
for the Waveshare Jetson UGV rover.

## Main Pieces

- `jetson_rover_v2.usd` - baked Isaac rover with ROS 2 graph topics.
- `small_warehouse.usd` and `full_warehouse.usd` - Isaac environments.
- `ugv_controller_v2/` - Isaac-side launchers and navigation-system launch files.
- `ugv_ws/src/` - Waveshare ROS 2 packages used by the real rover and Nav2.
- `jetson_rover_setup/` - files to install on the physical Jetson rover.

## Sim + Real Command Contract

The intended shared-control setup is:

- Laptop/workstation runs ROS 2 commands, RViz, Nav2, or teleop.
- Isaac host runs `ugv_controller_v2/run_small_warehouse.py`.
- Jetson rover runs the physical `ugv_bringup` stack.
- All machines use the same `ROS_DOMAIN_ID` and RMW implementation.
- Both Isaac and the real rover subscribe to global `/cmd_vel`, so one command
  stream can move both at the same time.
- Isaac sensor topics are namespaced under `/ugv_sim/*`; real rover topics stay
  on the upstream Waveshare names.

Important limitation: the current Isaac USD publishes global `/tf` and
`/tf_static`. Running the real rover and sim on the same ROS graph can therefore
produce duplicate frame names. Use the setup for mirrored command tests, and run
navigation/localization against either the sim data or the real data deliberately.
Full simultaneous TF isolation needs a future fully namespaced sim TF tree.

## Quick Start

On the Jetson:

```bash
cd ~/isaac-assets/jetson_rover_setup
./install_jetson.sh
sudo systemctl enable --now ugv-ros-env-check.service
sudo systemctl enable --now ugv-bringup.service
```

On the Isaac/laptop side, source the matching environment from
`jetson_rover_setup/env/ros2_rover.env`, start Isaac, then run one of the
navigation systems from `ugv_controller_v2/nav_systems/`.

