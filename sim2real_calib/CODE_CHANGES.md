# Sim-to-real code changes — 2026-11-19

The source updates use descriptive local names and add comments explaining each
function's role, coordinate frames, units, data flow, and numerical steps. Public
function and class names are retained so existing callers can keep using them.

This update also includes camera exposure control, surface profiles, an odometry
scale measurement tool, and saved rig settings. Those additions and settings
change runtime behavior; the update includes more than readability edits.

Paths below are relative to `sim2real_calib/`, except where noted. Run the tools
from the original source layout. The numbered folders in `sim2real_system/` are
independent copies for browsing and sharing.

## Camera and calibration

| File | What is easier to follow or newly available |
| --- | --- |
| [capture/stream.py](capture/stream.py) | Latest-frame buffering, frame counters, timeouts, and reader-thread shutdown. |
| [calib/calibrate_grid.py](calib/calibrate_grid.py) | Clicked anchors, image-pixel versus floor-metre coordinates, homography fitting, camera-position estimation, and verification overlays. |
| [calib/grid_target.py](calib/grid_target.py) | Tape geometry, crossing detection, border offsets, correspondence fitting, and residual rejection. |
| [calib/calibrate_stickers.py](calib/calibrate_stickers.py) | Sticker click order, colour sampling, measured sticker offsets, and configuration output. |
| [calib/sticker_tracker.py](calib/sticker_tracker.py) | Colour masks, candidate selection, rigid pose fitting, and sticker-plane correction. |
| [calib/calibrate_rover.py](calib/calibrate_rover.py) | Marker grouping, observed distances, and inferred rover dimensions. |
| [overhead/camera_model.py](overhead/camera_model.py) | Camera intrinsics, planar pose decomposition, projection, and height corrections. |
| [calib/camera_exposure.py](calib/camera_exposure.py) | New CLI to inspect, tune, save, and reapply Pi camera exposure and gain using sticker colour quality. |
| [pi_sender/stream_camera.sh](pi_sender/stream_camera.sh) | Fixed exposure and gain before streaming; `CAM_EXPOSURE` and `CAM_GAIN` override defaults. |

## Rover link, pose tracking, and homing

| File | What is easier to follow or newly available |
| --- | --- |
| [loop/rover_link.py](loop/rover_link.py) | Agent lifecycle, command messages, received telemetry, recording, and reconnect handling. |
| [loop/camera_track.py](loop/camera_track.py) | Frame processing, stamped poses, recording buffers, and camera health reporting. |
| [loop/fusion.py](loop/fusion.py) | SE(2) pose composition, gyro/odometry integration, interpolation, and latency-compensated camera corrections. |
| [loop/localizer.py](loop/localizer.py) | Fused versus raw camera poses and the tracker's recording interface. |
| [loop/homing.py](loop/homing.py) | Workspace checks, heading and position errors, command limits, retries, and stall diagnosis. |
| [loop/micromove.py](loop/micromove.py) | Small movement pulses, observed pose changes, and final pose correction. |
| [loop/icp2d.py](loop/icp2d.py) | Scan transforms, nearest-neighbour correspondences, surface normals, and point-to-line fitting. |
| [loop/stations.py](loop/stations.py) | Teaching and returning to test stations, lidar refinement, and station verification. |
| [loop/camera_guard.py](loop/camera_guard.py) | Reference capture, image matching, camera-movement checks, and small homography corrections. |
| [loop/selfcheck.py](loop/selfcheck.py) | Preflight checks, camera latency measurement, lidar convention checks, and suggested settings. |
| [loop/measure_odom_scale.py](loop/measure_odom_scale.py) | New forward/backward measurement of camera distance divided by wheel-odometry distance; `--write` saves `fusion.odom_scale`. This command moves the rover. |

## Features, physics, and optimization

| File | What is easier to follow |
| --- | --- |
| [optimize/features.py](optimize/features.py) | Time windows, motion onset, steady speeds, turn rates, coasting features, signed step features, and repeat statistics. |
| [optimize/build_targets.py](optimize/build_targets.py) | Trial discovery, feature aggregation, selected features, and target output. |
| [optimize/response_map.py](optimize/response_map.py) | Pairing real/sim command responses and reporting signed speed and turn-rate gaps. |
| [optimize/forward_model.py](optimize/forward_model.py) | Reduced-model actuator ramps, speed limits, friction effects, and trajectory integration. |
| [optimize/calibrate.py](optimize/calibrate.py) | Search stages, bounded parameter transforms, objective evaluation, simplex construction, and saved surface parameters. |
| [ugv_controller_v2/calibration/calib_params.py](../ugv_controller_v2/calibration/calib_params.py) | Parameter validation, surface bounds, geometry locks, material settings, and authoring physics values onto the USD stage. |
| [optimize/sim_eval.py](optimize/sim_eval.py) | Isaac launch and cleanup, simulation-time command playback, odometry recording, and failure handling. |
| [loop/sim_batch.py](loop/sim_batch.py) | Running multiple test sequences in one Isaac launch and collecting their trajectories. |

## Orchestration and visualization

| File | What is easier to follow or newly available |
| --- | --- |
| [loop/closed_loop.py](loop/closed_loop.py) | Trial recording and time alignment, resume handling, target construction, scoring, candidate evaluation, trust-region search, and result plots. |
| [loop/verify_params.py](loop/verify_params.py) | Baseline/calibrated evaluation against held-out real trials, feature scores, and report generation. |
| [viz/dashboard.py](viz/dashboard.py) | Window layout, image tiles, status updates, video recording, and cleanup. |
| [viz/metric_overlay.py](viz/metric_overlay.py) | Drawing the metric grid, workspace fence, rover pose, trajectory, and tracking status. |
| [viz/param_sheet.py](viz/param_sheet.py) | Displaying current, candidate, and best parameters with search bounds and history. |
| [viz/session_index.py](viz/session_index.py) | Session metadata, configuration copies, media links, per-test results, and the results catalog. |
| [rig_profile.py](rig_profile.py) | New CLI to save, create, inspect, and activate surface-specific rig profiles while keeping previous settings. |
| [surface_demo.py](surface_demo.py) | New guided surface workflow with setup, preflight, calibration, collection, optimization, verification, and resume state. |
| [DEMO_DAY.md](DEMO_DAY.md) | Operator commands and checkpoints for the surface workflow. |

## Saved settings and source snapshot

| Files | Included changes |
| --- | --- |
| [calib/grid_calib.json](calib/grid_calib.json), [calib/sticker_config.json](calib/sticker_config.json) | Updated grid homography and sticker calibration. These are rig-specific measurements. |
| [calib/camera_settings.json](calib/camera_settings.json) | Saved manual exposure/gain and sticker click points. |
| [calib/sticker_config.bright_exp100_20261118.json](calib/sticker_config.bright_exp100_20261118.json) | Saved alternative sticker calibration for the documented bright-light setup. |
| [loop/loop_config.yaml](loop/loop_config.yaml) | Updated workspace and carpet-preset homing/odometry settings. Remeasure settings when changing surfaces. |
| [loop/stations/](loop/stations/) | Updated camera-guard reference image and metadata. |
| [rig_profiles/](rig_profiles/) | Saved `padded` and `light_carpet` rig profiles. The light-carpet profile lists calibration steps still to complete in `profile.json`. |
| [surfaces/padded_loop.yaml](surfaces/padded_loop.yaml), [RESULTS.md](RESULTS.md) | Saved partial padded-surface fit and its catalog entry. Raw run directories remain local. |
| [sim2real_system/make_links.py](../sim2real_system/make_links.py) | Snapshot manifest now includes exposure control, surface workflow, rig-profile management, and odometry-scale measurement. Copied source and calibration references are refreshed from the runtime layout. |

Local regrid backups, automatic profile backups, profile history, and demo progress
files are excluded from Git. Saved named profiles and their calibration assets
are included.

## Validation

The following checks passed for this update: 25 calibration tests, 16 optimizer
and simulator-lifecycle tests, and 4 scoring/stall-diagnosis tests.

```bash
python3 -m unittest discover -s sim2real_calib/calib -p 'test_*.py' -v
python3 -m unittest discover -s sim2real_calib/optimize -p 'test_*.py' -v
python3 -m pytest -q sim2real_calib/loop/tests/test_scoring_globs.py sim2real_calib/loop/tests/test_stall_diagnosis.py
```

These checks do not validate the current camera placement, floor measurements,
physical rover motion, or a full Isaac physics run.
