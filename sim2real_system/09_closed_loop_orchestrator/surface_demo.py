#!/usr/bin/env python3
"""One-file runner for a sim2real demonstration on a new floor surface.

Runs every stage automatically, one after another - rig setup, calibration, real-rover
data collection, Isaac simulation, optimisation, results - using the project's own tools.
The only input it needs is the two calibration click windows (4 grid corners, 3 stickers).
It stops at the first failed step and prints the command that continues from there.

    cd ~/projects/isaac-assets/sim2real_calib
    python3 surface_demo.py --surface light_carpet               # the full automated run
    python3 surface_demo.py --surface light_carpet --from rig    # continue from a step
    python3 surface_demo.py --surface light_carpet --skip grid,camera   # already calibrated
    python3 surface_demo.py --surface light_carpet live          # just these step(s)
    python3 surface_demo.py --surface light_carpet --list        # steps + what's done

Full run = setup preflight grid camera rig workspace live results save.
Extra steps (run by name): collect (full 12-test recording), optimise (fit recorded
trials, no rover), verify (held-out check).

Steps:
  setup      create/activate this surface's rig profile (homing preset, fresh stations)
  preflight  Pi camera stream + exposure, overhead frames, rover link + battery, GPU free
  grid       calibrate the floor grid (click 4 corners) or just verify it
  camera     tune camera exposure for the stickers, calibrate sticker colours, check tracking
  rig        MOVES: camera latency + lidar check, wheel-odometry scale for this floor
  workspace  draw the fence + every start pose on the camera image - check they're on the floor
  live       MOVES: the full closed loop with the rover in it - real trials -> Isaac ->
             optimiser -> verify, one dashboard window (quick test set, ~25-35 min)
  collect    MOVES: real-only recording (quick or full battery; resumes after a battery stop)
  optimise   no rover: Isaac + optimiser on the recorded trials -> surfaces/<surface>_loop.yaml
  verify     held-out check of the fitted parameters (recorded iteration 2, or fresh trials)
  results    print/open the session pages, plots and the results catalog
  save       save this surface's rig profile (so it can be restored later)

Re-runs itself inside the ROS environment (ros2env.sh) if that isn't sourced.
"""
from __future__ import annotations

import argparse
import json
import os
import subprocess
import sys
import time
from pathlib import Path

SC = Path(__file__).resolve().parent
REPO = SC.parent
PY = sys.executable
PROFILES = SC / "rig_profiles"
RUNS = SC / "loop" / "runs"
TESTS = {
    # straight + both spin tests: identifies the turning gain (wheel_distance) and the
    # torque cap (drive_max_force) the loop fits by default - the live-demo set
    "quick": "T1_straight,T2_spin,T6_spin_sweep",
    "full": ("T1_straight,T2_spin,T3_speed_sweep,T3b_top_speed,T4_arc,T5_stop,T6_spin_sweep,"
             "R1_straight_fb,R2_spin_rates,R3_arcs_slow,R4_arcs_fast,R5_nav_pattern"),
}
PRESET_FOR = {"light_carpet": "carpet", "carpet": "carpet", "heavy_carpet": "carpet",
              "concrete": "concrete", "padded": "foam", "foam": "foam", "eva_foam": "foam"}
STEPS = ["setup", "preflight", "grid", "camera", "rig", "workspace", "live", "collect",
         "optimise", "verify", "results", "save"]
PIPELINE = ["setup", "preflight", "grid", "camera", "rig", "workspace", "live", "results",
            "save"]
MOVES = {"rig", "live", "collect"}
RUNTIME = {"rig": "~2 min", "live": "~25-35 min", "collect": "quick ~8 min / full ~1 h + charges",
           "optimise": "~15-25 min", "verify": "~10-20 min"}


# ----------------------------------------------------------------------------- helpers
# Print one status line; kind = ok / warn / fail / step picks a coloured prefix.
def say(msg="", kind=None):
    prefix = {"ok": "\033[32m[ ok ]\033[0m ", "warn": "\033[33m[WARN]\033[0m ",
           "fail": "\033[31m[FAIL]\033[0m ", "step": "\n\033[1;36m==> \033[0m"}.get(kind, "")
    print(prefix + msg, flush=True)


# Run a project tool as a child process in sim2real_calib/, sharing this terminal (its
# output and OpenCV windows appear live). check=True turns a non-zero exit into StepFailed.
def run(args, check=False):
    """Run a project tool (inherits the terminal so its output/windows show live)."""
    say("$ " + " ".join(str(a) for a in args))
    result = subprocess.run([str(a) for a in args], cwd=SC)
    if check and result.returncode != 0:
        raise StepFailed(f"{Path(str(args[1])).name} exited with code {result.returncode}")
    return result.returncode


# Raised by a step to stop the run with a message (main prints the --from command).
class StepFailed(Exception):
    pass


# rig_profiles/<surface>/demo_state.json - which steps finished, when, and how.
def state_path(surface):
    return PROFILES / surface / "demo_state.json"


# The demo state dict ({} before the first step ran).
def load_state(surface):
    p = state_path(surface)
    return json.loads(p.read_text()) if p.exists() else {}


# Record a step's outcome (ok/failed, time, info) in the demo state file.
def mark(surface, step, ok, info=""):
    state_file = state_path(surface)
    state_file.parent.mkdir(parents=True, exist_ok=True)
    state = load_state(surface)
    state[step] = dict(ok=ok, at=time.strftime("%Y-%m-%d %H:%M:%S"), info=info)
    state_file.write_text(json.dumps(state, indent=2) + "\n")


# This surface's session folders in loop/runs/, newest first, with the command line
# that created each (read from its manifest.json). kind filters recordings ('real-only')
# or live closed loops ('loop' = neither --real-only nor --real-from).
def sessions(surface, kind=None):
    """Session dirs of this surface, newest first. kind: 'real-only' | 'loop' | None."""
    found = []
    for session_dir in sorted(RUNS.glob(f"{surface}_2*"), reverse=True):
        manifest_path = session_dir / "manifest.json"
        command = json.loads(manifest_path.read_text()).get("command", "") if manifest_path.exists() else ""
        if kind == "real-only" and "--real-only" not in command:
            continue
        if kind == "loop" and ("--real-only" in command or "--real-from" in command):
            continue
        found.append((session_dir, command))
    return found


# {test name: number of finished repeats} in a session's iter_NN/real/ folder.
def count_trials(real_dir):
    real_dir = Path(real_dir)
    return {test_dir.name: len(list(test_dir.glob("rep[0-9].json")))
            for test_dir in sorted(real_dir.iterdir()) if test_dir.is_dir()} if real_dir.exists() else {}


# Run `rover_link.py --check` (also proves the rover is reachable) and read the pack
# voltage from its 'route=... battery=11.76V ...' line. Below hard_min -> False (stop);
# below minimum -> warn but continue (the loop's own 10.4 V stop + resume handle it).
def battery_ok(minimum=10.8, hard_min=10.5):
    """Rover link check. False below hard_min (the loop would stop at 10.4 V almost at once);
    a warning between hard_min and minimum (the run may hit the battery stop part-way)."""
    link_check = subprocess.run([PY, "loop/rover_link.py", "--check", "--keep-bringup"], cwd=SC,
                         capture_output=True, text=True)
    status_lines = [output_line for output_line in link_check.stdout.splitlines() if output_line.startswith("route=")]
    if link_check.returncode != 0 or not status_lines:
        say("rover link check failed:\n" + (link_check.stdout + link_check.stderr)[-800:], "fail")
        return False
    battery_v = float(status_lines[0].split("battery=")[1].split("V")[0])
    say(f"rover reachable, battery {battery_v:.2f} V", "ok" if battery_v >= minimum else "warn")
    if battery_v < hard_min:
        say(f"battery below {hard_min} V - charge the rover first", "fail")
        return False
    if battery_v < minimum:
        say(f"battery below {minimum} V: the run may stop at 10.4 V part-way (re-run the "
            "same command after charging - it resumes)", "warn")
    return True


# ----------------------------------------------------------------------------- steps
# Make this surface's rig profile the active one: already active -> nothing; saved
# before -> activate it; new -> rig_profile.py new with the preset for the surface name.
def step_setup(a):
    active_surface = (PROFILES / "ACTIVE").read_text().strip() if (PROFILES / "ACTIVE").exists() else None
    if active_surface == a.surface:
        say(f"profile '{a.surface}' is already active", "ok")
        return
    if (PROFILES / a.surface / "loop_config.yaml").exists():
        say(f"activating the saved profile '{a.surface}' (current rig '{active_surface}' is auto-saved)")
        run([PY, "rig_profile.py", "activate", a.surface], check=True)
        return
    preset = a.preset or PRESET_FOR.get(a.surface)
    if not preset:
        raise StepFailed(f"no homing preset known for '{a.surface}': pass --preset carpet|concrete|foam")
    say(f"new surface '{a.surface}' with the {preset} preset; the current rig ('{active_surface}') is "
        "saved first")
    run([PY, "rig_profile.py", "new", a.surface, "--preset", preset], check=True)


# Everything a run needs, checked without moving the rover: Pi reachable + camera
# service running (started if not) + saved exposure re-applied, frames on UDP 5000,
# rover link + battery, Isaac launcher present, GPU not already busy.
def step_preflight(a):
    sys.path.insert(0, str(SC / "calib"))
    import camera_exposure as CE
    failed_checks = 0
    try:
        service_state = CE.pi("systemctl is-active sim2real-camera.service || true").strip()
        if service_state != "active":
            say("Pi camera stream is off - starting it")
            CE.pi("sudo -n systemctl start sim2real-camera.service")
            time.sleep(6)
        say(f"Pi reachable ({CE._host}), camera stream service running", "ok")
        saved_settings = CE.apply_saved(log=lambda m: None)
        say(f"camera exposure: {CE.describe(saved_settings) if saved_settings else 'no saved setting (auto)'}", "ok")
    except RuntimeError as exc:
        failed_checks += 1
        say(f"Pi: {exc}  - power the Pi and check its network; the stream must reach this "
            "desktop on UDP 5000", "fail")
    sys.path.insert(0, str(SC / "capture"))
    from stream import LatestFrame
    camera_stream = LatestFrame()
    try:
        frame = camera_stream.wait_first(15) if camera_stream.opened() else None
    finally:
        camera_stream.release()
    if frame is None:
        failed_checks += 1
        say("no overhead frames on UDP 5000 (stream down, or another viewer is open - close "
            "live_tracking.py / receive.py)", "fail")
    else:
        say(f"overhead frames arriving ({frame.shape[1]}x{frame.shape[0]})", "ok")
    if not battery_ok():
        failed_checks += 1
    if not (REPO / "isaacpy_clean.sh").exists():
        failed_checks += 1
        say("isaacpy_clean.sh missing - Isaac can't be launched", "fail")
    gpu_query = subprocess.run(["nvidia-smi", "--query-gpu=memory.used,memory.total",
                        "--format=csv,noheader,nounits"], capture_output=True, text=True)
    if gpu_query.returncode == 0:
        used_mib, total_mib = (int(x) for x in gpu_query.stdout.split(",")[:2])
        stray_isaac_pids = subprocess.run(["pgrep", "-f", "run_ground_plane|isaac-sim|kit/kit"],
                               capture_output=True, text=True).stdout.split()
        level = "ok" if used_mib < 3000 and not stray_isaac_pids else "warn"
        say(f"GPU {used_mib}/{total_mib} MiB used" + (f"; Isaac-like processes running: {stray_isaac_pids}" if stray_isaac_pids
                                               else ""), level)
    if failed_checks:
        raise StepFailed(f"{failed_checks} preflight check(s) failed")


# --grid calibrate: interactive 4-corner grid calibration; always followed by a
# no-click verify of the saved grid against the live tape crossings (<= 10 mm).
def step_grid(a):
    say("The tape grid must be EXACTLY 1 m (outer edge to outer edge), 1-inch tape, lines "
        "every 0.25 m, border tape INSIDE the square.")
    if a.grid == "calibrate":
        say("Click the 4 outer corners: bottom-left, bottom-right, top-right, top-left "
            "(U undo). Review window: Enter/S save, U re-click, Q quit.")
        run([PY, "calib/calibrate_grid.py", "--source", "stream", "--size", a.grid_size,
             "--spacing", a.grid_spacing], check=True)
    verify_image = SC / "rig_profiles" / a.surface / "grid_verify.png"
    verify_image.parent.mkdir(parents=True, exist_ok=True)
    exit_code = run([PY, "calib/calibrate_grid.py", "--verify", "--source", "stream", "--save", verify_image])
    if exit_code != 0:
        raise StepFailed(f"grid verify failed - see {verify_image}")
    say(f"grid verified ({verify_image.relative_to(SC)})", "ok")


# Tune the camera exposure for the stickers (user clicks the 3 stickers), calibrate
# the sticker colours at the same click points, measure the tracking rate for 10 s and
# re-save the camera-guard reference image (it depends on the exposure).
def step_camera(a):
    say("Park the rover near the grid centre, front facing UP the image, all three stickers "
        "(green front-left, orange-red front-right, pink rear) visible. Lighting as it will "
        "stay for the whole demo.")
    say("Click the sticker centres in order: green, orange-red, pink. Enter to finish. "
        "The camera then sweeps ~28 exposure/gain settings (~1 min).")
    run([PY, "calib/camera_exposure.py", "--tune"], check=True)
    click_points_px = json.loads((SC / "calib" / "camera_settings.json").read_text())["points_px"]
    points_text = " ".join(f"{px_x:.0f},{px_y:.0f}" for px_x, px_y in click_points_px)
    run([PY, "calib/calibrate_stickers.py", "--points-px", points_text], check=True)
    tracking_fraction = tracking_rate(10)
    say(f"tracking check: {tracking_fraction:.0%} of frames saw all 3 stickers", "ok" if tracking_fraction > 0.95 else "warn")
    if tracking_fraction < 0.8:
        raise StepFailed("sticker tracking below 80 % - re-run `camera` (check lighting)")
    run([PY, "loop/camera_guard.py", "--save-reference"])   # exposure changes its reference


# Fraction of frames in `seconds` where all 3 stickers were tracked, from the
# line-per-frame output of calibrate_rover.py --mode check.
def tracking_rate(seconds):
    result = subprocess.run([PY, "calib/calibrate_rover.py", "--mode", "check", "--duration",
                        str(seconds)], cwd=SC, capture_output=True, text=True)
    pose_lines = [output_line for output_line in result.stdout.splitlines() if output_line.startswith(("chassis", "No pose"))]
    return sum("3/3" in output_line for output_line in pose_lines) / max(len(pose_lines), 1)


# Rover-moving rig measurements: selfcheck --motion (camera latency, lidar convention)
# then measure_odom_scale --write (wheel-odometry slip on this floor).
def step_rig(a):
    if not battery_ok():
        raise StepFailed("battery")
    say("Rover will rock in place, drive ~9 cm and turn ~25 deg (latency + lidar check), then "
        "drive 4 x 30 cm straight legs (odometry scale).")
    rc = run([PY, "loop/selfcheck.py", "--motion", "--keep-bringup"])
    if rc != 0:
        say("selfcheck reported a FAIL (see above) - fix it before collecting data", "warn")
    run([PY, "loop/measure_odom_scale.py", "--write", "--keep-bringup"], check=True)


# Draw the workspace fence, every test's start pose (arrow + ~0.26 x 0.22 m rover
# footprint) and homing pre-goal (run_in_m behind the start) onto a live frame using
# the grid homography H_m2img (metres -> pixels). Fails if any start or pre-goal is
# outside the fence; the image itself is for checking the fence lies on the floor.
def step_workspace(a):
    import cv2
    import math
    import numpy as np
    import yaml
    sys.path.insert(0, str(SC / "capture"))
    from stream import LatestFrame
    config = yaml.safe_load((SC / "loop" / "loop_config.yaml").read_text())
    grid_calibration = json.loads((SC / "calib" / "grid_calib.json").read_text())
    grid_to_image = np.asarray(grid_calibration["H_m2img"], float)
    camera_stream = LatestFrame()
    try:
        frame = camera_stream.wait_first(15)
    finally:
        camera_stream.release()
    if frame is None:
        raise StepFailed("no overhead frame")
    to_pixels = lambda pts: cv2.perspectiveTransform(np.float64(pts).reshape(-1, 1, 2), grid_to_image).reshape(-1, 2)
    fence_x, fence_y = config["workspace"]["x"], config["workspace"]["y"]
    fence_px = to_pixels([[fence_x[0], fence_y[0]], [fence_x[1], fence_y[0]], [fence_x[1], fence_y[1]], [fence_x[0], fence_y[1]]])
    cv2.polylines(frame, [np.int32(fence_px)], True, (0, 0, 255), 2, cv2.LINE_AA)
    cv2.putText(frame, "workspace fence (rover centre must stay inside)", tuple(np.int32(fence_px[3]) + [5, 18]),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (0, 0, 255), 1, cv2.LINE_AA)
    run_in_m = config["homing"].get("run_in_m", 0.15)
    bad_poses = []
    for name, pose in config["start_poses"].items():
        if name == "default":
            continue
        yaw_rad = math.radians(pose["yaw"])
        for label, (check_x, check_y) in (("start", (pose["x"], pose["y"])),
                                ("pre-goal", (pose["x"] - run_in_m * math.cos(yaw_rad),
                                              pose["y"] - run_in_m * math.sin(yaw_rad)))):
            if not (fence_x[0] <= check_x <= fence_x[1] and fence_y[0] <= check_y <= fence_y[1]):
                bad_poses.append(f"{name} {label}")
        centre_px, arrow_tip_px = to_pixels([[pose["x"], pose["y"]], [pose["x"] + .1 * math.cos(yaw_rad), pose["y"] + .1 * math.sin(yaw_rad)]])
        pre_goal_px = to_pixels([[pose["x"] - run_in_m * math.cos(yaw_rad), pose["y"] - run_in_m * math.sin(yaw_rad)]])[0]
        # rover footprint (~0.26 x 0.22 m) at the start pose
        rotation = np.array([[math.cos(yaw_rad), -math.sin(yaw_rad)], [math.sin(yaw_rad), math.cos(yaw_rad)]])
        footprint_px = to_pixels((np.array([[.13, .11], [.13, -.11], [-.13, -.11], [-.13, .11]]) @ rotation.T) + [pose["x"], pose["y"]])
        cv2.polylines(frame, [np.int32(footprint_px)], True, (0, 200, 255), 1, cv2.LINE_AA)
        cv2.arrowedLine(frame, tuple(np.int32(centre_px)), tuple(np.int32(arrow_tip_px)), (0, 255, 255), 2, tipLength=.3)
        cv2.circle(frame, tuple(np.int32(pre_goal_px)), 3, (255, 0, 255), -1)
        cv2.putText(frame, name.split("_")[0], tuple(np.int32(centre_px) + [6, -6]), cv2.FONT_HERSHEY_SIMPLEX,
                    .4, (0, 255, 255), 1, cv2.LINE_AA)
    cv2.putText(frame, "yellow: start poses + rover footprint   magenta: homing pre-goals   "
                "red: fence  - all must sit on the test floor", (10, 20),
                cv2.FONT_HERSHEY_SIMPLEX, .5, (255, 255, 255), 1, cv2.LINE_AA)
    image_path = PROFILES / a.surface / "workspace.png"
    image_path.parent.mkdir(parents=True, exist_ok=True)
    cv2.imwrite(str(image_path), frame)
    say(f"wrote {image_path.relative_to(SC)}")
    if os.environ.get("DISPLAY"):
        cv2.imshow("workspace check", frame)
        cv2.waitKey(5000)                      # shown for 5 s, then the run continues
        cv2.destroyAllWindows()
    image_h, image_w = frame.shape[:2]
    if bad_poses:
        raise StepFailed("start pose / pre-goal outside the fence: " + ", ".join(bad_poses)
                         + " - edit loop/loop_config.yaml (workspace / start_poses)")
    if not ((fence_px >= 0) & (fence_px < [image_w, image_h])).all():
        say("the fence reaches outside the camera view - the rover can be lost near its "
            "edges; shrink `workspace` in loop/loop_config.yaml if trials abort there", "warn")
    say("Check the saved image: every footprint and pre-goal must sit on the test floor. "
        "If not, edit loop/loop_config.yaml (workspace / start_poses) and re-run "
        "`workspace`.")


# closed_loop.py command line for this surface (base driver kept running between steps).
def loop_cmd(a, *extra):
    return [PY, "loop/closed_loop.py", "--surface", a.surface, "--keep-bringup", *extra]


# The main demo: the full closed loop with the rover in it (real trials -> Isaac ->
# optimiser -> fitted re-run). An unfinished earlier session of the same tests resumes.
def step_live(a):
    if not battery_ok(11.0):
        raise StepFailed("battery")
    tests = TESTS[a.tests]
    previous_sessions = [session_dir for session_dir, command in sessions(a.surface, "loop") if f"--tests {tests}" in command]
    resume_args = []
    if previous_sessions and not (previous_sessions[0] / "summary.md").exists():
        say(f"resuming the unfinished live session {previous_sessions[0].name} (finished trials kept)")
        resume_args = ["--resume", str(previous_sessions[0].relative_to(SC))]
    say(f"Live closed loop on '{a.surface}': {tests}, {a.repeats} real repeats each, "
        f"{a.iterations} iteration(s), up to {a.max_evals} Isaac evaluations. The dashboard "
        "window shows every phase.")
    exit_code = run(loop_cmd(a, "--tests", tests, "--repeats", a.repeats, "--iterations", a.iterations,
                      "--max-evals", a.max_evals, "--no-hold", *resume_args))
    if exit_code != 0:
        raise StepFailed("closed loop stopped early (battery stop? charge, then re-run with "
                         "--from live: it resumes)")


# Real-only recording (no Isaac). Resumes the newest matching recording if it has fewer
# trials than tests x repeats x iterations. --tests full records 2 iterations: training
# + held-out.
def step_collect(a):
    tests = TESTS[a.tests]
    previous_sessions = [session_dir for session_dir, command in sessions(a.surface, "real-only") if f"--tests {tests}" in command]
    resume_args = []
    iterations = "2" if a.tests == "full" else str(a.iterations)
    if previous_sessions:
        expected_trials = len(tests.split(",")) * int(a.repeats) * int(iterations)
        recorded_trials = sum(sum(count_trials(previous_sessions[0] / f"iter_{iteration:02d}" / "real").values())
                   for iteration in range(1, int(iterations) + 1))
        if recorded_trials < expected_trials:
            say(f"resuming {previous_sessions[0].name}: {recorded_trials}/{expected_trials} trials recorded (finished trials kept)")
            resume_args = ["--resume", str(previous_sessions[0].relative_to(SC))]
    if not battery_ok():
        raise StepFailed("battery")
    say(f"Recording {tests} x {a.repeats} repeats, {iterations} iteration(s) (iteration 2 = "
        "held-out set). The loop stops itself at 10.4 V: charge, then run `collect` again.")
    exit_code = run(loop_cmd(a, "--real-only", "--tests", tests, "--repeats", a.repeats,
                      "--iterations", iterations, "--no-hold", *resume_args))
    if exit_code != 0:
        raise StepFailed("recording stopped early (battery? charge and re-run `collect`: it "
                         "resumes)")


# Newest session of this surface that has at least one recorded real trial.
def latest_real(a):
    for session_dir, command in sessions(a.surface, "real-only") + sessions(a.surface, "loop"):
        real_dir = session_dir / "iter_01" / "real"
        if any(count_trials(real_dir).values()):
            return session_dir
    raise StepFailed(f"no recorded real trials for '{a.surface}' - run `collect` (or `live`)")


# No rover: Isaac + optimiser on the newest recording's iteration-1 trials (tests with
# >= 2 repeats only, so each feature has a spread) -> surfaces/<surface>_loop.yaml.
def step_optimise(a):
    session_dir = latest_real(a)
    trials_per_test = count_trials(session_dir / "iter_01" / "real")
    usable_tests = [test_name for test_name, repeat_count in trials_per_test.items() if repeat_count >= 2]
    if not usable_tests:
        raise StepFailed(f"{session_dir.name}: no test has >= 2 repeats yet")
    say(f"Optimising on {session_dir.name}/iter_01/real: {', '.join(usable_tests)}")
    exit_code = run(loop_cmd(a, "--real-from", session_dir.relative_to(SC) / "iter_01" / "real", "--tests",
                      ",".join(usable_tests), "--iterations", "1", "--max-evals", a.max_evals))
    if exit_code != 0:
        raise StepFailed("optimiser failed (see the session.log)")
    say(f"fitted parameters -> surfaces/{a.surface}_loop.yaml", "ok")


# Held-out check of surfaces/<surface>_loop.yaml: against the recorded iteration 2 if
# it exists, otherwise verify_params drives fresh real trials.
def step_verify(a):
    fitted_yaml = SC / "surfaces" / f"{a.surface}_loop.yaml"
    if not fitted_yaml.exists():
        raise StepFailed(f"no {fitted_yaml.name} yet - run `optimise` or `live` first")
    session_dir = latest_real(a)
    held_out_dir = session_dir / "iter_02" / "real"
    held_out_tests = ",".join(test_name for test_name, repeat_count in count_trials(held_out_dir).items() if repeat_count >= 2) if held_out_dir.exists() else ""
    verify_command = [PY, "loop/verify_params.py", "--surface", a.surface, "--calibrated",
            fitted_yaml.relative_to(SC), "--keep-bringup"]
    if held_out_tests:
        say(f"held-out check on the recorded {held_out_dir.relative_to(SC)} ({held_out_tests})")
        verify_command += ["--real-from", held_out_dir.relative_to(SC), "--tests", held_out_tests]
    else:
        say("no recorded held-out trials: verify drives FRESH real trials (rover moves)")
        if not battery_ok():
            raise StepFailed("battery")
        verify_command += ["--tests", TESTS[a.tests]]
    if run(verify_command) != 0:
        raise StepFailed("verify_params failed")


# Print the newest sessions (kind + rms headline + index.md path) and the fitted
# parameters; open the newest final_view.png / progress.png when a display exists.
def step_results(a):
    recent_sessions = sessions(a.surface)
    if not recent_sessions:
        say(f"no sessions for '{a.surface}' yet", "warn")
    for session_dir, command in recent_sessions[:6]:
        session_kind = ("recording" if "--real-only" in command else "optimise (recorded)" if "--real-from" in command
                else "live closed loop")
        headline = ""
        if (session_dir / "summary.md").exists():
            headline = next((summary_line for summary_line in (session_dir / "summary.md").read_text().splitlines() if "rms" in summary_line), "")
        say(f"{session_dir.name:28s} {session_kind:20s} {headline[:90]}")
        say(f"    {(session_dir / 'index.md').relative_to(SC)}")
    fitted_yaml = SC / "surfaces" / f"{a.surface}_loop.yaml"
    say(f"fitted parameters: {fitted_yaml.relative_to(SC) if fitted_yaml.exists() else '(none yet)'}")
    say(f"all sessions: {(SC / 'RESULTS.md').relative_to(SC)}")
    if recent_sessions and os.environ.get("DISPLAY"):
        for plot_name in ("final_view.png", "progress.png"):
            plot_path = recent_sessions[0][0] / plot_name
            if plot_path.exists():
                subprocess.Popen(["xdg-open", str(plot_path)], stdout=subprocess.DEVNULL,
                                 stderr=subprocess.DEVNULL)


# Save the active rig as this surface's profile.
def step_save(a):
    run([PY, "rig_profile.py", "save", a.surface, "--note", "saved by surface_demo.py"], check=True)


# ----------------------------------------------------------------------------- main
# Re-exec inside ros2env.sh if needed, parse options, then run the steps in order -
# PIPELINE (or --from / --skip / named steps). Each step function receives the parsed
# options (`a` in the step functions). Stops at the first failure and prints the
# --from command that continues from that step.
def main():
    if "ROS_DISTRO" not in os.environ and os.environ.get("SURFACE_DEMO_REEXEC") != "1":
        child_env = dict(os.environ, SURFACE_DEMO_REEXEC="1")
        shell_command = f"source {REPO / 'ros2env.sh'} >/dev/null 2>&1; exec {PY} " + " ".join(
            f"'{part}'" for part in [__file__, *sys.argv[1:]])
        os.execvpe("bash", ["bash", "-c", shell_command], child_env)
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("steps", nargs="*", choices=STEPS + [[]], metavar="STEP",
                    help="run only these steps (default: the full automated run)")
    parser.add_argument("--from", dest="start", choices=PIPELINE,
                    help="continue the full run from this step")
    parser.add_argument("--skip", default="", help="comma list of steps to leave out")
    parser.add_argument("--grid", choices=["calibrate", "verify"], default="calibrate",
                    help="calibrate (click 4 corners; new floor) or only verify the saved grid")
    parser.add_argument("--surface", required=True, help="e.g. light_carpet")
    parser.add_argument("--preset", choices=["carpet", "concrete", "foam"],
                    help="homing preset for a new surface (default from the name)")
    parser.add_argument("--tests", choices=sorted(TESTS), default="quick",
                    help="quick = T1,T2,T6 (live demo); full = all 12 tests")
    parser.add_argument("--repeats", default="3")
    parser.add_argument("--iterations", default="1")
    parser.add_argument("--max-evals", default="8", help="Isaac evaluations per optimisation")
    parser.add_argument("--grid-size", default="1")
    parser.add_argument("--grid-spacing", default="0.25")
    parser.add_argument("--list", action="store_true", help="list steps and what's done")
    options = parser.parse_args()
    step_functions = {step: globals()[f"step_{step}"] for step in STEPS}
    state = load_state(options.surface)
    if options.list:
        for step in STEPS:
            step_state = state.get(step)
            status_text = ("done " + step_state["at"] if step_state and step_state["ok"] else
                    "FAILED " + step_state["at"] if step_state else "")
            say(f"  {step:10s} {'(moves rover) ' if step in MOVES else '              '}{status_text}")
        return 0
    steps_to_run = options.steps or PIPELINE[PIPELINE.index(options.start) if options.start else 0:]
    skipped_steps = {step.strip() for step in options.skip.split(",") if step.strip()}
    steps_to_run = [step for step in steps_to_run if step not in skipped_steps]
    say(f"surface '{options.surface}': running " + " -> ".join(steps_to_run))
    run_start = time.time()
    for step in steps_to_run:
        runtime_hint = f" [{RUNTIME[step]}]" if step in RUNTIME else ""
        say(f"{step}{runtime_hint}{'  ** rover moves **' if step in MOVES else ''}", "step")
        step_start = time.time()
        try:
            step_functions[step](options)
            mark(options.surface, step, True, f"{time.time() - step_start:.0f} s")
            say(f"{step} done in {(time.time() - step_start) / 60:.1f} min", "ok")
        except StepFailed as exc:
            mark(options.surface, step, False, str(exc))
            say(f"{step}: {exc}", "fail")
            if step in PIPELINE and not options.steps:
                say(f"fix it, then continue with:  python3 surface_demo.py --surface "
                    f"{options.surface} --from {step}")
            return 1
        except KeyboardInterrupt:
            mark(options.surface, step, False, "interrupted")
            say(f"{step} interrupted (the tools stop the rover on Ctrl-C). Continue with:  "
                f"python3 surface_demo.py --surface {options.surface} --from {step}", "warn")
            return 130
    say(f"all done in {(time.time() - run_start) / 60:.1f} min", "ok")
    return 0


if __name__ == "__main__":
    sys.exit(main())
