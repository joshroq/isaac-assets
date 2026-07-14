# Jetson Rover Setup

This folder is meant to be copied with the repo onto the physical Waveshare
Jetson UGV rover. It installs the rover-side ROS 2 environment needed for a
laptop or workstation to publish ROS 2 commands that reach both:

- the Isaac simulated rover, and
- the physical Jetson rover.

## Network Model

Use the same values on all participating machines:

- `ROS_DOMAIN_ID`
- `RMW_IMPLEMENTATION`
- `ROS_LOCALHOST_ONLY=0`

Default values are in `env/ros2_rover.env`. Edit that file once, then install.

The mirrored-control path is intentionally simple:

```text
laptop/Nav2/teleop publishes /cmd_vel
        |
        +--> Isaac rover subscribes /cmd_vel
        |
        +--> physical Jetson rover subscribes /cmd_vel
```

Isaac publishes sim sensor data under `/ugv_sim/*`. The physical rover publishes
upstream real topics such as `/scan`, `/odom`, `/imu/*`, `/tf`, and `/tf_static`.

## Install On The Jetson

From the repo root on the Jetson:

```bash
cd jetson_rover_setup
./install_jetson.sh
```

The script installs apt dependencies, builds `../ugv_ws`, installs helper scripts
to `/usr/local/bin`, and installs systemd unit files.

Enable the rover bringup service:

```bash
sudo systemctl enable --now ugv-bringup.service
```

Check status:

```bash
ugv-check
systemctl status ugv-bringup.service
ros2 topic list
```

## Laptop / Workstation Setup

Use the same env values as the Jetson:

```bash
source /opt/ros/humble/setup.bash
set -a
source /path/to/isaac-assets/jetson_rover_setup/env/ros2_rover.env
set +a
```

Then start Isaac:

```bash
isaacpy /path/to/isaac-assets/ugv_controller_v2/run_small_warehouse.py
```

In another terminal, run one of the v2 navigation stacks:

```bash
source /opt/ros/humble/setup.bash
source /path/to/isaac-assets/ugv_ws/install/setup.bash
set -a
source /path/to/isaac-assets/jetson_rover_setup/env/ros2_rover.env
set +a

ros2 launch /path/to/isaac-assets/ugv_controller_v2/nav_systems/nav_lidar/isaac_nav_lidar.launch.py
ros2 launch /path/to/isaac-assets/ugv_controller_v2/nav_systems/nav_rgb/isaac_nav_rgb.launch.py
ros2 launch /path/to/isaac-assets/ugv_controller_v2/nav_systems/nav_depth/isaac_nav_depth.launch.py
ros2 launch /path/to/isaac-assets/ugv_controller_v2/nav_systems/nav_orbslam3/isaac_nav_orbslam3.launch.py
```

## Safety Notes

Publishing `/cmd_vel` moves the physical rover if `ugv-bringup.service` is
running and the Jetson is on the same ROS domain. Keep the rover on blocks while
testing mirrored command flow.

The current sim and real rover both publish global TF frames. That is acceptable
for command mirroring, but it is not a clean multi-robot localization graph.
When validating navigation, choose whether the navigation stack is consuming sim
topics or real rover topics and watch `ros2 topic info`/RViz carefully.

