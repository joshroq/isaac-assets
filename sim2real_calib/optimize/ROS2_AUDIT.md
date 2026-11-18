# Isaac / ROS 2 verification — 2026-10-14

The native `ugv_controller_v2` baked ROS graphs work with system ROS 2 Humble.
The earlier failure was reproducible with mismatched discovery settings and the
wrong odometry topic; replacing the bridge with a direct-wheel driver is unnecessary.

The system shell inherited `ROS_LOCALHOST_ONLY=1` from `~/.bashrc`, while
`run_baked_rover._configure_ros_environment()` set `ROS_LOCALHOST_ONLY=0` after
process startup. `/proc/<pid>/environ` showed the launch environment, obscuring that
runtime change. Against the same running native Isaac process, the system probe
received zero odometry messages with localhost-only enabled and 261 with it disabled.
The baked odometry topic is `/ugv_sim/odom`, not `/odom`.

| Live check | Result |
|---|---|
| Ground-plane baked graph, system ROS commands | 0.605 m forward displacement, 0.828 rad final heading change; odom, clock, TF and joint states received |
| `sim_eval.run_trial`, domain 0 | 310 samples over 9.0125 sim seconds; 0.580 m displacement; measured steady forward speed 0.19929 m/s for a 0.2 m/s command |
| Second `sim_eval.run_trial`, domain 43 | 241 samples over 7.0 sim seconds; 0.386 m displacement; launch and teardown succeeded |
| `trial_sim.sh` command entry point | Native ROS trial completed and wrote 70 samples to the requested output CSV |
| Warehouse sensor interface, cameras at 640×360 | RGB, depth, stereo left/right, camera info, IMU, odometry and LiDAR all delivered; scan contained 3,600 finite ranges |
| ROS lifecycle regressions, no GPU | 3 passed: early process failure/log reporting, timeout killing the process group, simulation-time sequence with custom odometry topic |

`isaacpy_clean.sh` and `ros2env.sh` now agree on ordinary DDS discovery with
`ROS_LOCALHOST_ONLY=0`. The runner honors `ROS_DOMAIN_ID` and supports the explicit
wrapper override `ISAAC_ROS_LOCALHOST_ONLY`. `sim_eval` runs the baked drive and
odometry graphs by default, starts its sequence only after advancing odometry and
command-subscriber discovery, and records simulation timestamps. Inputs and
`isaac.log` remain in the trial directory. Timeouts and failures stop only that
trial's process group, without deleting shared memory or stopping the ROS daemon.
The caller's networking variables are restored afterward. Use a separate
`ROS_DOMAIN_ID` when running calibration alongside another simulation.

`trial_sim.sh` now uses the same backend. `mode="direct"` in the Python API or
`SIM_TRIAL_MODE=direct` in the shell retains the explicit physics diagnostic.
The optional `SIM_ROS` direct-wheel diagnostic remains available but is not used
by normal trials. Shared USD assets were not changed by this repair.

Reproduce from the repository root:

```bash
source ros2env.sh
python3 sim2real_calib/optimize/sim_eval.py
sim2real_calib/trials/trial_sim.sh <sequence.yaml> <out.csv> [calib.yaml]
python3 -m unittest discover -s sim2real_calib/optimize -p test_sim_eval.py -v
```

Full live logs and CSVs for this verification are temporarily retained at
`/tmp/isaac_ros_audit/`. This checks ROS command, pose and sensor delivery, not a
complete SLAM/Nav2 mission or full-resolution camera throughput. The analytic
calibrator self-test also ran; it still reports weak identifiability for static
floor friction, which is separate from ROS connectivity.

Isaac uses its bundled Python 3.12 ROS libraries while the external node uses
system Humble/Python 3.10; this matches NVIDIA's
[internal ROS library setup](https://docs.isaacsim.omniverse.nvidia.com/6.0.0/installation/install_ros_other_platforms.html).
