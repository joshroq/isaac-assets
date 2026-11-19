# Isaac Assets

Isaac Sim rover assets, ROS 2 Humble workspaces, and navigation launch helpers
for the Waveshare Jetson UGV rover.

## Main Pieces

- `jetson_rover_v2.usd` - baked Isaac rover with ROS 2 graph topics.
- `small_warehouse.usd` and `full_warehouse.usd` - Isaac environments.
- `ugv_controller_v2/` - Isaac-side launchers and navigation-system launch files.
- `ugv_ws/src/` - Waveshare ROS 2 packages used by the real rover and Nav2.
- `jetson_rover_setup/` - files to install on the physical Jetson rover.
- [`sim2real_system/`](sim2real_system/README.md) - complete browsable snapshot of the
  camera, rover link, calibration, physics optimiser, simulation, and navigation
  comparison, arranged in execution order.
- [`sim2real_calib/`](sim2real_calib/README.md) - original runtime source layout for
  calibration and sim-to-real experiments.
- [`experiments/`](experiments/README.md) - navigation experiments and replay tools.

## Sim-to-real source and results

See the [per-file change guide](sim2real_calib/CODE_CHANGES.md) for the readability
updates, supporting tools, and saved settings. The
[demo-day guide](sim2real_calib/DEMO_DAY.md) walks through the current surface workflow.

`sim2real_system/` contains independent copies of the source, configuration, rover
USD, saved stations, tests, and a held-out verification report. It is a source
snapshot for browsing and sharing; run the system from the original repository
layout in `sim2real_calib/` and `ugv_controller_v2/`, following the
[runbook](sim2real_calib/docs/RUNBOOK.md).

After editing the original sources, refresh the snapshot with:

```bash
python3 sim2real_system/make_links.py
```

Generated trial directories, recordings, caches, build outputs, and large
RTAB-Map databases are excluded from Git. The local
[`sim2real_calib/RESULTS.md`](sim2real_calib/RESULTS.md) catalog references local run
data; those run links require the original results directories. The copied
[held-out report](sim2real_system/10_verification_and_report/report.md) is included
in Git.

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
