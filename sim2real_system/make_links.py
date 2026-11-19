#!/usr/bin/env python3
"""Build a browser-uploadable snapshot of the rover + optimiser system by stage.

Entries are independent copies, including directory contents. No symbolic links
are created, so the folder can be uploaded to OneDrive through a browser.
The historical script name is retained for compatibility. Run it from the original
repository to refresh the snapshot; existing snapshot files will be overwritten.

The list was made by tracing the imports of every entry point
(closed_loop, staged_physics.sh, verify_params, selfcheck, calibrate_grid, verify_stickers,
run_isaac_battery, path_errors, run_ground_plane), plus the scripts they launch by name and
the config/data files they read. The sim rover always drives its wheels through PhysX
wheel-floor contact; there is no other drive mode.

    python3 sim2real_system/make_links.py      # refresh the copies
"""
import shutil
from pathlib import Path

HERE = Path(__file__).resolve().parent
ROOT = HERE.parent
SC = "sim2real_calib"
UC = "ugv_controller_v2"

MANIFEST = {
    "00_environment": [
        "ros2env.sh", "isaacpy_clean.sh", f"{SC}/docs/RUNBOOK.md"],
    "01_overhead_camera": [
        f"{SC}/pi_sender/stream_camera.sh", f"{SC}/pi_sender/install_on_pi.sh",
        f"{SC}/pi_sender/sim2real-camera.service", f"{SC}/pi_sender/README.md",
        f"{SC}/capture/stream.py", f"{SC}/capture/receive.py"],
    "02_grid_and_sticker_calibration": [
        f"{SC}/calib/grid_target.py", f"{SC}/calib/calibrate_grid.py", f"{SC}/calib/grid_calib.json",
        f"{SC}/calib/sticker_tracker.py", f"{SC}/calib/calibrate_stickers.py",
        f"{SC}/calib/sticker_config.json", f"{SC}/calib/verify_stickers.py",
        f"{SC}/overhead/camera_model.py", f"{SC}/live_tracking.py",
        f"{SC}/calib/camera_exposure.py", f"{SC}/calib/camera_settings.json"],
    "03_real_rover_link": [
        f"{SC}/loop/rover_link.py", f"{SC}/loop/rover_agent.py",
        "rover_nav/start_bringup.sh", "rover_nav/stop_all.sh", "rover_nav/local_ros_env.sh",
        "rover_nav/_check_single_stack.sh"],
    "04_pose_tracking_and_homing": [
        f"{SC}/loop/camera_track.py", f"{SC}/loop/fusion.py", f"{SC}/loop/homing.py",
        f"{SC}/loop/micromove.py", f"{SC}/loop/localizer.py", f"{SC}/loop/icp2d.py",
        f"{SC}/loop/stations.py", f"{SC}/loop/camera_guard.py", f"{SC}/loop/stations",
        f"{SC}/loop/loop_config.yaml"],
    "05_test_battery": [
        f"{SC}/parallel/sequences/{t}.yaml" for t in (
            "T1_straight", "T2_spin", "T3_speed_sweep", "T3b_top_speed", "T4_arc", "T5_stop",
            "T6_spin_sweep", "R1_straight_fb", "R2_spin_rates", "R3_arcs_slow", "R4_arcs_fast",
            "R5_nav_pattern")],
    "06_measurement_features_and_error": [
        f"{SC}/optimize/features.py", f"{SC}/optimize/build_targets.py",
        f"{SC}/trials/compare_paths.py", f"{SC}/optimize/path_errors.py",
        f"{SC}/optimize/response_map.py"],
    "07_physics_model_and_optimizer": [
        f"{UC}/calibration/calib_params.py", f"{UC}/calibration/apply_sim_calibration.py",
        f"{SC}/optimize/calibrate.py", f"{SC}/optimize/forward_model.py",
        f"{SC}/loop/staged_physics.sh", f"{SC}/PARAMETER_MAP.md", f"{SC}/docs/MODELING.md",
        f"{SC}/docs/figures.tex", f"{SC}/docs/figures.pdf",
        f"{SC}/surfaces/concrete_physics.yaml", f"{SC}/surfaces/carpet_loop.yaml"],
    "08_isaac_simulation": [
        f"{UC}/common/run_baked_rover.py",
        f"{UC}/common/accessory_controller.py", f"{UC}/common/viewport_setup.py",
        f"{UC}/run_ground_plane.py", f"{UC}/run_small_warehouse.py", f"{UC}/run_obstacle_warehouse.py",
        f"{SC}/optimize/sim_eval.py", f"{SC}/loop/sim_batch.py", f"{SC}/optimize/run_isaac_battery.py",
        "jetson_rover_v2.usd"],
    "09_closed_loop_orchestrator": [
        f"{SC}/loop/closed_loop.py", f"{SC}/loop/selfcheck.py", f"{SC}/viz/dashboard.py",
        f"{SC}/viz/metric_overlay.py", f"{SC}/viz/param_sheet.py", f"{SC}/viz/session_index.py",
        f"{SC}/surface_demo.py", f"{SC}/rig_profile.py", f"{SC}/DEMO_DAY.md",
        f"{SC}/loop/measure_odom_scale.py"],
    "10_verification_and_report": [
        f"{SC}/loop/verify_params.py", f"{SC}/loop/runs/verify_20261112_082754/report.md"],
    "11_tests": [
        f"{SC}/loop/tests/fake_rover.py", f"{SC}/loop/tests/test_reopen.py",
        f"{SC}/loop/tests/test_stall_diagnosis.py", f"{SC}/loop/tests/test_stations_sim.py",
        f"{SC}/loop/tests/test_lidar_convention.py", f"{SC}/loop/tests/test_scoring_globs.py",
        f"{SC}/loop/test_agent_local.py", f"{SC}/calib/test_grid_target.py",
        f"{SC}/calib/test_sticker_tracker.py", f"{SC}/optimize/test_sim_eval.py",
        f"{SC}/optimize/test_calibrate.py", f"{SC}/optimize/test_response_map.py"],
    # navigation-level check of the optimizer's results: real vs sim rover under Nav2
    # (lidar / depth, no visual SLAM). Start with README.md.
    "12_nav_comparison": [
        f"{SC}/navcmp/README.md", f"{SC}/navcmp/session.py", f"{SC}/navcmp/mission_runner.py",
        f"{SC}/navcmp/course.py", f"{SC}/navcmp/nav_params.py", f"{SC}/navcmp/conditions.yaml",
        f"{SC}/navcmp/stack/navcmp_stack.launch.py", f"{SC}/navcmp/metrics.py",
        f"{SC}/navcmp/report.py", f"{SC}/navcmp/courses/grid1x1_square.yaml",
        f"{SC}/navcmp/courses/grid2x2_box.yaml", f"{SC}/navcmp/courses/grid1x4_slalom.yaml",
        f"{UC}/run_nav_course.py", f"{SC}/navcmp/tests/test_navcmp.py",
        f"{SC}/navcmp/tests/test_ssh_transport.py", f"{SC}/navcmp/tests/synth.py"],
}


def copy_entry(src, dst):
    """Dereference source links and replace destination links without editing targets."""
    src = src.resolve(strict=True)
    if dst.is_symlink():
        dst.unlink()
    if src.is_dir():
        dst.mkdir(parents=True, exist_ok=True)
        for child in src.iterdir():
            copy_entry(child, dst / child.name)
        shutil.copystat(src, dst)
    else:
        dst.parent.mkdir(parents=True, exist_ok=True)
        shutil.copy2(src, dst)


def main():
    # Check the manifest before replacing any existing entries. The source
    # repository is required only to refresh, never to browse/upload the snapshot.
    missing = [f for files in MANIFEST.values() for f in files if not (ROOT / f).exists()]
    if missing:
        print("Source files missing; refresh this snapshot from the original isaac-assets repository:",
              *missing, sep="\n  ")
        return 1
    for folder, files in MANIFEST.items():
        for f in files:
            copy_entry(ROOT / f, HERE / folder / Path(f).name)
    n = sum(len(v) for v in MANIFEST.values())
    print(f"Copied {n} entries (including directory contents) into {HERE}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
