#!/usr/bin/env python3
"""CLOSED sim2real loop, fully automated:

    repeat for N iterations (or until the sim matches the real rover within noise):
      1. REAL   for each test x repeat: the rover RETURNS to the test's start pose
                (homing.py, camera feedback), runs the sequence (timed on the Jetson via
                rover_agent.py) while the overhead camera records its true path
      2. TARGETS real per-test motion features (mean +/- sigma over the repeats)
      3. SIM    the same battery in Isaac with the CURRENT parameters  ("before")
      4. COMPARE real vs sim: feature loss, path errors, path maps on the real camera view
      5. UPDATE  optimise the sim parameters against this iteration's real data (batched
                Isaac evaluations: every test of the battery in ONE launch per candidate)
      6. VERIFY  the best parameters' sim run ("after"); they seed the next iteration,
                whose fresh real repeats check them on unseen data.

Everything lands in loop/runs/<surface>_<stamp>/ (per-iteration CSVs, videos, plots,
optimizer evals, theta) plus summary.md / progress.png, and the final calibration is
written to surfaces/<surface>_loop.yaml (a new file - existing surfaces/*.yaml are not
touched). Existing demo/optimizer code is reused, not modified.

    source ros2env.sh
    python3 loop/closed_loop.py --surface carpet                       # defaults: T1,T2,T4 x3
    python3 loop/closed_loop.py --surface carpet --iterations 4 --repeats 3 --max-evals 12
    python3 loop/closed_loop.py --surface carpet --keys wheel_distance --tests T2
    # no rover / pipeline check: reuse the real CSVs of an earlier iteration
    python3 loop/closed_loop.py --surface carpet --real-from loop/runs/<session>/iter_01/real
    python3 loop/closed_loop.py --surface carpet --real-only          # just drive + record

Safety: the rover is only driven by sequences inside the configured workspace (abort +
stop on leaving it or on losing the camera), the agent zeroes on any link loss, and the
loop parks and stops when the battery falls below loop_config.yaml battery_min_v.
"""
from __future__ import annotations

import argparse
import csv
import datetime
import glob
import json
import math
import shutil
import subprocess
import sys
import time
import traceback
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
SC = HERE.parent
REPO = SC.parent
sys.path.insert(0, str(HERE))
sys.path.insert(0, str(SC / "optimize"))
sys.path.insert(0, str(SC / "viz"))
sys.path.insert(0, str(REPO / "ugv_controller_v2" / "calibration"))

import calib_params as C            # noqa: E402
import features as F                # noqa: E402
import calibrate as CAL             # noqa: E402
import build_targets as BT          # noqa: E402
from homing import (load_config, start_pose, go_to_pose, in_workspace,  # noqa: E402
                    HomingError, CONFIG)
from sim_batch import run_battery   # noqa: E402
import stations as ST               # noqa: E402
from dashboard import Dashboard, Track, CANDIDATE, place_at  # noqa: E402
import param_sheet as PS            # noqa: E402
from session_index import Session   # noqa: E402

SEQ_DIR = SC / "parallel" / "sequences"
PENALTY = 1e6                       # infeasible / failed candidate (finite for Nelder-Mead)
COLORS = {"real": (60, 220, 60), "before": (60, 60, 255), "after": (255, 200, 0)}


# ───────────────────────────── utilities ─────────────────────────────
# Session logger: each call prints '<HH:MM:SS> message', appends it to session.log
# and, once the dashboard exists, shows it in the window's ticker (sink).
class Log:
    def __init__(self, path):
        self.f = open(path, "a")
        self.sink = None                # dashboard ticker, once the window exists

    def __call__(self, *parts):
        text = " ".join(str(part) for part in parts)
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"{stamp} {text}", flush=True)
        self.f.write(f"{stamp} {text}\n")
        self.f.flush()
        if self.sink is not None:
            self.sink(f"{stamp} {text}")


# Test name ('T1', 'T1_straight' or a yaml path) -> (stem, sequence yaml path).
def resolve_test(name):
    """'T1' / 'T1_straight' / path -> (stem, path)."""
    as_path = Path(name)
    if as_path.suffix in (".yaml", ".yml") and as_path.exists():
        return as_path.stem, as_path
    exact = SEQ_DIR / f"{name}.yaml"
    if exact.exists():
        return name, exact
    matches = sorted(glob.glob(str(SEQ_DIR / f"{name}_*.yaml")))
    if not matches:
        sys.exit(f"no sequence for test '{name}' in {SEQ_DIR}")
    return Path(matches[0]).stem, Path(matches[0])


# Write rows (default columns t, x, y, yaw) with 5 decimals.
def write_csv(path, rows, header=("t", "x", "y", "yaw")):
    with open(path, "w", newline="") as csv_file:
        writer = csv.writer(csv_file)
        writer.writerow(header)
        for row in rows:
            writer.writerow([f"{value:.5f}" for value in row])


# Track CSV -> N x 4 array (t, x, y, yaw).
def load_rows(path):
    t, x, y, yaw = F.load_csv(str(path))
    return np.column_stack([t, x, y, yaw])


# Wrap angles to [-pi, pi) (numpy, elementwise).
def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


# The fitted keys' values from a full parameter set, as plain floats.
def theta_view(theta, keys):
    return {key: float(theta[key]) for key in keys}


# ───────────────────────────── real side ─────────────────────────────
# Dashboard recording path; never overwrites (adds _partN when resuming).
def session_video_path(session):
    """session_video.mp4, or _part2/_part3... when resuming (never overwrite a recording)."""
    video_path = Path(session) / "session_video.mp4"
    part_number = 2
    while video_path.exists():
        video_path = Path(session) / f"session_video_part{part_number}.mp4"
        part_number += 1
    return video_path


# Session window wrapper (see docstring).
class LiveView:
    """The session's one persistent window (viz/dashboard.py) + a 1 Hz live.jpg.

    Created once and kept open for every phase: live camera (homing, driving), the
    current real trial top-down, and per-test gallery tiles of real vs sim (before /
    optimiser candidates / after) that update as each run finishes. real_trial() only
    sets .trail / .text and calls update(pose)."""

    # Create the dashboard (recording to session_video*.mp4, live.jpg every second)
    # and feed it the camera overlay with the current trial's trail.
    def __init__(self, cam, session, show, grid_dims=(1.0, 1.0), workspace=None, title=None):
        self.cam = cam
        self.video = session_video_path(session)
        self.trail = []
        self._text = ""
        self.last_draw = 0.0
        self.dash = Dashboard(title or "sim2real closed loop", grid_dims=grid_dims, show=show,
                              snapshot=session / "live.jpg", workspace=workspace,
                              record=self.video)
        if cam is not None and hasattr(cam, "overlay"):
            self.dash.camera_source(
                lambda: cam.overlay(trails=[(self.trail[:], COLORS["real"])],
                                    text=self._text or None))

    @property
    def text(self):
        return self._text

    @text.setter
    def text(self, value):
        self._text = value
        if value:
            self.dash.status(value)

    def status(self, text):
        self.dash.status(text)

    # Add a pose to the live trail and redraw the 'current' panel (max 10 Hz).
    def update(self, pose=None):
        now = time.time()
        if pose is not None:
            self.trail.append(tuple(pose[1:3]))
            if len(self.trail) > 3000:
                self.trail = self.trail[-3000:]
        if now - self.last_draw < 0.1:
            return
        self.last_draw = now
        self.dash.current(self._text or "real rover", [Track("real (live)", self.trail[:],
                                                             COLORS["real"])])

    # Gallery tile for one test (see docstring).
    def test_tile(self, test, real_reps, sims, lines, title=None):
        """Gallery tile for one test: every real repeat + sim tracks placed at rep 1's start.
        sims = [(label, rows | csv path, colour)]."""
        try:
            real_tracks = [load_rows(r["csv"]) for r in real_reps]
        except Exception:
            return
        if not real_tracks or not len(real_tracks[0]):
            return
        start_pose = tuple(real_tracks[0][0, 1:4])
        tracks = [Track("real", real_tracks[0], COLORS["real"])]
        tracks += [Track("", r, (40, 140, 40), 1) for r in real_tracks[1:]]
        for label, rows, color in sims:
            if rows is None:
                continue
            rows = load_rows(rows) if isinstance(rows, (str, Path)) else np.asarray(rows)
            if len(rows):
                tracks.append(Track(label, place_at(rows, start_pose), color))
        self.dash.result(test, title or test, tracks, lines)
        self.dash.current(title or test, tracks, lines)      # latest result, full size

    # Close the window (optionally wait for the user first).
    def close(self, hold=False):
        if hold:
            self.dash.hold("loop finished - press q or close the window to exit")
        self.dash.close()


# Back up an output from an EARLIER session before overwriting it (see docstring).
def protect(out, session, log):
    """Back up a calibration file written by an EARLIER session before this one replaces
    it (surfaces/<surface>_loop.yaml is shared; a short demo run must not silently erase
    a validated calibration). Same-session rewrites (each iteration) are not backed up."""
    out = Path(out)
    marker_file = session / ".wrote_outputs"
    written_by_this_session = set(marker_file.read_text().split()) if marker_file.exists() else set()
    if out.exists() and str(out) not in written_by_this_session:
        backup_path = out.with_name(f"{out.stem}.before_{session.name}{out.suffix}")
        shutil.copy(out, backup_path)
        log(f"[loop] {out.name} existed (earlier session) -> backed up to {backup_path.name}")
    written_by_this_session.add(str(out))
    marker_file.write_text("\n".join(sorted(written_by_this_session)))


# Workspace fence as ((x_min, x_max), (y_min, y_max)) for drawing, or None.
def _ws(cfg):
    workspace = cfg.get("workspace")
    return (workspace["x"], workspace["y"]) if workspace else None


# Fitted-parameter panel of the window (see docstring).
class ParamPanel:
    """Keeps the window's parameter panel current: the fitted keys (start, value in use,
    best + latest candidate of this iteration, trust region, per-iteration history) and
    every other sim parameter grouped as held / measured / locked geometry."""

    def __init__(self, view, theta, keys, surface, history=()):
        self.view, self.keys, self.surface = view, keys, surface
        first_theta = history[0]["theta_before"] if history else theta
        self.start = {key: first_theta[key] for key in keys}
        self.hist = {key: [first_theta[key]] + [past["theta_after"][key] for past in history] for key in keys}
        self.theta = dict(theta)
        self.best = self.cand = self.trust = None
        self.header = ""

    # Send the panel contents to the window.
    def push(self, header=None):
        if header is not None:
            self.header = header
        if self.view is None:
            return
        self.view.dash.params(PS.build(self.theta, self.keys, self.surface, start=self.start,
                                       best=self.best, cand=self.cand, trust=self.trust,
                                       history=self.hist, header=self.header))

    # New iteration: reset best/candidate, compute this iteration's search box.
    def iteration(self, it, n_it, trust):
        self.best = self.cand = None
        self.trust = trust_region(self.keys, self.theta, self.surface, trust)
        self.push(f"iteration {it}/{n_it}: search box = {trust:.0%} of each range around "
                  "the current values")

    # After each Isaac evaluation: show its candidate and the best so far.
    def evaluated(self, rec, best, it):
        self.cand = {key: rec["theta"][key] for key in self.keys}
        self.best = {key: best["theta"][key] for key in self.keys}
        self.push(f"iteration {it}: eval {rec['n']} ({rec['tag']}) rms z {rec['rms_z']:.2f}, "
                  f"best {best['rms_z']:.2f}")

    # Iteration done: the new values are in use; extend the history.
    def finished(self, theta_new, it, rms_b, rms_a):
        self.theta = dict(theta_new)
        for key in self.keys:
            self.hist[key].append(theta_new[key])
        self.cand = None
        self.push(f"iteration {it} done: rms z {rms_b:.2f} -> {rms_a:.2f}; values now in use")


# Window updates during the Isaac phase (see docstring).
class SimTiles:
    """Feeds every Isaac evaluation into the window: status while Isaac runs, then each
    test's tile shows real vs sim BEFORE, the latest candidate and the best so far."""

    def __init__(self, view, real, keys, it, args, params=None):
        self.view, self.real, self.keys, self.it = view, real, keys, it
        self.params = params
        self.max_evals = args.max_evals
        # more real trials still to come after this Isaac phase: say so on screen, or the
        # idle rover looks finished and gets powered off (it did, 2026-11-05)
        self.rover_needed = it < args.iterations and not getattr(args, "sim_only", False)
        self.before = None
        self.best = None

    # Status line while an Isaac launch runs (and whether the rover is still needed).
    def start(self, n, tag):
        phase_labels = {"before": "current parameters (BEFORE)", "after": "best parameters (AFTER)"}
        self.view.status(f"iteration {self.it}: Isaac eval {n} - "
                         f"{phase_labels.get(tag, f'optimiser candidate (budget {self.max_evals})')}"
                         + (" | ROVER STILL NEEDED - keep it ON" if self.rover_needed else ""))

    # After an evaluation: redraw every test's tile with BEFORE / candidate / best.
    def done(self, rec):
        if not rec.get("csvs"):
            return
        if rec["tag"] == "before":
            self.before = rec
        if self.best is None or rec["loss"] < self.best["loss"]:
            self.best = rec
        if self.params is not None:
            self.params.evaluated(rec, self.best, self.it)
        theta_text = "  ".join(f"{key[:10]}={rec['theta'][key]:.3g}" for key in self.keys)
        for test, reps in self.real.items():
            if not reps:
                continue
            sims = []
            if self.before and test in self.before["csvs"]:
                sims.append(("sim BEFORE", self.before["csvs"][test], COLORS["before"]))
            if rec is not self.before and test in rec["csvs"]:
                sims.append((f"eval {rec['n']}", rec["csvs"][test], CANDIDATE))
            if self.best is not self.before and test in self.best["csvs"]:
                sims.append((f"best (eval {self.best['n']})", self.best["csvs"][test],
                             COLORS["after"]))
            per_test = rec.get("per_test") or per_test_rms(rec.get("parts"))
            own_rms = f"{per_test[test]:.2f}" if test in per_test else "-"
            self.view.test_tile(test, reps, sims,
                                [f"eval {rec['n']}: this test rms z {own_rms}  (all tests "
                                 f"{rec['rms_z']:.2f}, best {self.best['rms_z']:.2f})", theta_text],
                                f"{test}  iter {self.it}")


# Per-test rms z from {'test.feature': z} dicts (see docstring).
def per_test_rms(*parts):
    """{"test.feature": z} dicts (mean and/or spread terms) -> {test: rms z over its terms}.
    The pooled rms z mixes all tests; a test tile must show its own (2026-11-05: every tile
    showed the pooled 3.89, reading as if each test scored the same)."""
    squares_by_test = {}
    for z_dict in parts:
        for feature_key, z in (z_dict or {}).items():
            if z is not None and math.isfinite(z):
                squares_by_test.setdefault(feature_key.split(".", 1)[0], []).append(z * z)
    return {test: math.sqrt(sum(squares) / len(squares)) for test, squares in squares_by_test.items()}


# Camera-guard check at session start (see docstring); corrects the homography
# in memory for a small move, raises for a large one.
def camera_check(cam, log):
    """Camera moved since the reference? small -> correct the homography in memory;
    large -> refuse (start poses would be wrong)."""
    import camera_guard
    if not hasattr(cam, "cam"):                    # no image stream (test fake camera)
        return
    try:
        guard = camera_guard.CameraGuard()
    except FileNotFoundError:
        log("[camera] no reference image - run loop/camera_guard.py --save-reference "
            "(camera-move detection disabled)")
        return
    pose = cam.pose(1.0)
    frame = cam.cam.read()[0]
    guard_result = guard.check(frame, rover_xy=list(pose[1:3]) if pose else None)
    if not guard_result.get("ok"):
        raise RuntimeError(f"camera check failed: {guard_result.get('why')}")
    if guard_result["moved"]:
        cam.set_grid_homography(guard_result["H_m2img"])
        log(f"[camera] moved {guard_result['shift_px']} px since the reference -> corrected homography "
            f"applied for this session")
    else:
        log(f"[camera] unmoved ({guard_result['shift_px']} px)")


MAX_CAMERA_GAP_S = 0.5      # the feed runs at ~30 Hz (max gap 0.18 s on 2026-11-03)


# Reject a trial whose camera track has a gap: any gap > max_gap inside the sequence,
# or > max_gap + 1 s at its start/end. Returns the reason, or None if usable.
def recording_problem(t, seq_s, max_gap=MAX_CAMERA_GAP_S):
    """Camera sample times (s from sequence start) -> reason the trial is unusable, or None."""
    t = np.asarray(t, float)
    in_sequence = t[(t >= 0.0) & (t <= seq_s)]
    if len(in_sequence) < 10:
        return f"only {len(in_sequence)} samples during the {seq_s:.1f} s sequence"
    edges = np.concatenate([[0.0], in_sequence, [seq_s]])
    largest_gap = float(np.max(np.diff(edges)))
    if largest_gap > max_gap + 1.0:    # leading/trailing edge: allow the ~1 s camera/link latency
        return f"{largest_gap:.2f} s without a camera fix"
    sample_gaps = np.diff(in_sequence)
    if len(sample_gaps) and sample_gaps.max() > max_gap:
        gap_index = int(np.argmax(sample_gaps))
        return f"{sample_gaps[gap_index]:.2f} s camera gap at t={in_sequence[gap_index]:.1f} s"
    return None


# One real trial: home to the test's station, record camera (+ video) and wheel
# odometry while the rover runs the sequence (abort if the camera loses the rover or it
# leaves the workspace), then write rep<N>_raw.csv (t from the rover's sequence start),
# rep<N>.csv (re-timed to motion onset), odometry, frame times and a meta json.
def real_trial(rover, cam, tracker, cfg, test, steps, outdir, rep, log, view):
    """Return to the test's station (camera coarse + lidar fine + gate), run the
    sequence, record. -> dict(csv, raw, video, start=gate record, ...)."""
    view.trail, view.text = [], f"{test} rep {rep}: returning to start station"
    station_record = ST.goto_station(rover, cam, tracker, test, cfg, log)
    view.update(cam.pose(0.5))
    guard_events_before = len(rover.guard_events)
    video_path = outdir / f"rep{rep}.mp4"
    view.trail, view.text = [], f"{test} rep {rep}: driving"
    cam.start_recording(video_path=video_path)
    rover.odom_log = []
    preroll_start = time.time()
    while time.time() - preroll_start < 0.6:              # stationary pre-roll
        view.update(cam.pose(0.5))
        time.sleep(0.02)
    lost_state = {"since": None}

    # Called every tick of the running sequence; returns a reason string to abort.
    def guard(elapsed):
        camera_pose = cam.pose(0.5)
        view.update(camera_pose)
        if camera_pose is None:
            lost_state["since"] = lost_state["since"] or time.time()
            return "camera lost the rover" if time.time() - lost_state["since"] > 1.5 else None
        lost_state["since"] = None
        if not in_workspace(cfg, camera_pose[1], camera_pose[2]):
            return f"left the workspace at ({camera_pose[1]:.2f},{camera_pose[2]:.2f})"
        return None

    seq_start = None
    try:
        seq_start = rover.run_sequence(steps, on_tick=guard)
        coast_start = time.time()
        while time.time() - coast_start < 1.0:          # capture the coast-down
            view.update(cam.pose(0.5))
            time.sleep(0.02)
    finally:
        camera_rows, frame_times = cam.stop_recording()
        wheel_odom = rover.odom_log or []
        rover.odom_log = None
    camera_rows = np.array(camera_rows, float)
    if len(camera_rows) < 10:
        raise HomingError(f"only {len(camera_rows)} camera samples recorded")
    raw_track = camera_rows.copy()
    raw_track[:, 0] -= seq_start
    problem = recording_problem(raw_track[:, 0], sum(float(step["duration"]) for step in steps))
    if problem:                                    # a feed hiccup would corrupt step features
        raise HomingError(f"camera recording unusable: {problem}")
    write_csv(outdir / f"rep{rep}_raw.csv", raw_track)
    # Re-time to the rover's own motion onset (removes link + camera latency), the same
    # alignment build_targets.py applies, so real features line up with the sequence -
    # guarded against the onset detector firing late on slow stick-slip spins.
    offset_s, alignment_method, onset_estimate = alignment_offset(raw_track, steps, cfg)
    aligned = raw_track.copy()
    aligned[:, 0] -= offset_s
    aligned = aligned[aligned[:, 0] >= 0.0]
    write_csv(outdir / f"rep{rep}.csv", aligned)
    if wheel_odom:
        odom_track = np.array([(t - seq_start, x, y, yaw) for t, x, y, yaw, _, _ in wheel_odom])
        write_csv(outdir / f"rep{rep}_wheel_odom.csv", odom_track)
    write_csv(outdir / f"rep{rep}_frames.csv", [(t - seq_start - offset_s,) for t in frame_times], ("t",))
    meta = {"test": test, "rep": rep, "start": station_record,
            "onset_offset_s": round(float(offset_s), 3), "alignment": alignment_method,
            "onset_estimate_s": round(float(onset_estimate), 3), "n_camera": int(len(aligned)),
            "n_partial_total": int(cam.n_partial), "battery_v": rover.voltage,
            "obstacle_stops": rover.guard_events[guard_events_before:]}
    (outdir / f"rep{rep}.json").write_text(json.dumps(meta, indent=2))
    displacement_m = math.hypot(aligned[-1, 1] - aligned[0, 1], aligned[-1, 2] - aligned[0, 2])
    turned_deg = math.degrees(abs(np.unwrap(aligned[:, 3])[-1] - aligned[0, 3]))
    log(f"[real] {test} rep {rep}: {len(aligned)} samples, moved {displacement_m*100:.1f} cm, "
        f"turned {turned_deg:.0f} deg, onset offset {offset_s:+.2f}s, battery {rover.voltage:.2f} V")
    return {"csv": str(outdir / f"rep{rep}.csv"), "video": str(video_path), "offset": offset_s,
            "frames": str(outdir / f"rep{rep}_frames.csv"), "start": station_record}


ALIGN_TOL_S = 0.35
# Expected delay from sequence start to camera-visible motion (see docstring).
def nominal_latency(cfg):
    """Rig delay from the rover's sequence start to motion seen by the camera: camera lag
    (fusion.cam_latency_s, measured 0.71 s) + link/motor lag (timing.link_latency_s)."""
    return (float(cfg.get("fusion", {}).get("cam_latency_s", 0.71))
            + float(cfg.get("timing", {}).get("link_latency_s", 0.15)))


# How much to shift a trial's time axis (see docstring).
def alignment_offset(raw, steps, cfg):
    """-> (offset_s, 'onset'|'nominal', onset_estimate_s).

    The motion-onset estimate is used when it agrees with the rig's known latency (straights
    and arcs: 0.81-1.04 s vs nominal 0.86). Spins on carpet start slowly (stick-slip under
    the 0.12 rad/s detector threshold), so the detector fired 1-2.3 s late on R2/T2 and every
    per-step window landed in the next, opposite step (2026-11-03: wrong-sign yaw rates).
    Then the nominal latency is used instead."""
    onset_estimate = BT.aligned_offset(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3], steps)
    nominal = nominal_latency(cfg)
    if abs(onset_estimate - nominal) <= ALIGN_TOL_S:
        return onset_estimate, "onset", onset_estimate
    return nominal, "nominal", onset_estimate


# Re-time already recorded trials from their raw CSVs (--realign).
def realign_trials(real_dir, plan, cfg, log=print):
    """Rewrite <real_dir>/<test>/rep*.csv from rep*_raw.csv with alignment_offset()."""
    for test, steps in plan.items():
        for raw_path in sorted(Path(real_dir, test).glob("rep*_raw.csv")):
            rep_name = raw_path.name[:-len("_raw.csv")]
            raw_track = load_rows(raw_path)
            offset_s, alignment_method, onset_estimate = alignment_offset(raw_track, steps, cfg)
            aligned = raw_track.copy()
            aligned[:, 0] -= offset_s
            write_csv(raw_path.with_name(f"{rep_name}.csv"), aligned[aligned[:, 0] >= 0.0])
            meta_path = raw_path.with_name(f"{rep_name}.json")
            if meta_path.exists():
                meta = json.loads(meta_path.read_text())
                meta.update(onset_offset_s=round(float(offset_s), 3), alignment=alignment_method,
                            onset_estimate_s=round(float(onset_estimate), 3))
                meta_path.write_text(json.dumps(meta, indent=2))
            log(f"[align] {test} {rep_name}: offset {offset_s:.2f}s ({alignment_method}; onset estimate {onset_estimate:.2f}s)")


# (test, rep) pairs in the order they are driven (see docstring).
def trial_order(plan, repeats, order="round-robin"):
    """(test, rep) pairs in recording order. round-robin (rep 1 of every test, then rep 2, ...)
    means a run cut short (battery, WiFi) still has trials of EVERY test; test-by-test recorded
    all repeats of R1 first, so the last tests had none (2026-11-04: T4 got none)."""
    if order == "test":
        return [(test, rep) for test in plan for rep in range(1, repeats + 1)]
    return [(test, rep) for rep in range(1, repeats + 1) for test in plan]


# Record `repeats` trials of every test (resume keeps finished ones). Each trial
# gets up to `retries` extra attempts; link drops, rover reboots and lost base drivers
# are waited out / restarted. The battery floor stops the loop.
# Returns {test: [trial dicts]}.
def real_phase(rover, cam, tracker, cfg, plan, repeats, itdir, log, view, retries=2,
               order="round-robin"):
    trials = {test_name: [] for test_name in plan}
    battery_min_v = float(cfg.get("battery_min_v", 0))
    for test, rep in trial_order(plan, repeats, order):
        steps = plan[test]
        test_dir = itdir / "real" / test
        test_dir.mkdir(parents=True, exist_ok=True)
        done_csv, done_meta = test_dir / f"rep{rep}.csv", test_dir / f"rep{rep}.json"
        if done_csv.exists() and done_meta.exists():      # --resume: keep finished trials
            meta = json.loads(done_meta.read_text())
            trials[test].append({"csv": str(done_csv), "video": str(test_dir / f"rep{rep}.mp4"),
                              "frames": str(test_dir / f"rep{rep}_frames.csv"),
                              "start": meta.get("start")})
            log(f"[real] {test} rep {rep}: reusing the trial already recorded")
            continue
        for attempt in range(retries + 1):
            if battery_min_v and rover.voltage == rover.voltage and rover.voltage < battery_min_v:
                raise RuntimeError(f"battery {rover.voltage:.2f} V < {battery_min_v} V - "
                                   f"charge the rover (it browns out on a low pack)")
            try:
                trials[test].append(real_trial(rover, cam, tracker, cfg, test, steps, test_dir,
                                            rep, log, view))
                break
            except Exception as exc:          # homing / abort / link: retry the rep
                if getattr(rover, "alive", False) and not getattr(rover, "link_up", True):
                    # a network dropout (weak Wi-Fi / campus relay): the link reconnects by
                    # itself and the rover already stopped any velocity command
                    log(f"[real] {exc}; link down - waiting for it to reconnect")
                    view.status("rover link down - reconnecting")
                    if not rover.wait_link(rover.reconnect_timeout):
                        raise RuntimeError("rover link did not come back - check its power / "
                                           "WiFi / Tailscale, then --resume this session")
                    log("[real] link back - retrying the trial")
                elif not rover.alive and hasattr(rover, "reopen"):
                    # the agent's ssh itself is gone (rover rebooted / lost power / off the
                    # network): restarting only the base driver needs the agent and failed
                    # (2026-11-05). Wait for the rover, then rebuild the whole link.
                    log(f"[real] {exc}; rover agent gone - waiting up to 10 min for the rover "
                        f"to come back, then reconnecting")
                    view.status("rover unreachable - check its power / WiFi")
                    rover.reopen(600.0)
                    log(f"[rover] reconnected: battery {rover.voltage:.2f} V")
                elif "base driver stopped" in str(exc) or (hasattr(rover, "base_alive")
                                                            and not rover.base_alive):
                    log(f"[real] {exc}; restarting the rover base driver")
                    view.status("rover base driver lost - restarting it")
                    rover.restart_bringup()
                elif not rover.alive or "motors" in str(exc):
                    raise                     # link gone / base dead: stop the loop
                if "link to the rover stalled" in str(exc):
                    log(f"[real] {exc}; waiting up to 90 s for the link to recover")
                    view.status("rover link stalled - waiting for WiFi")
                    wait_deadline = time.time() + 90.0
                    while time.time() < wait_deadline and rover.alive and rover.rx_age > 0.5:
                        time.sleep(0.5)
                    if not rover.alive or rover.rx_age > 0.5:
                        raise RuntimeError("rover link did not recover in 90 s - fix the "
                                           "WiFi, then --resume this session")
                    log(f"[real] link back (last reply {rover.rx_age:.2f}s ago) - retrying")
                rover.stop()
                log(f"[real] {test} rep {rep} attempt {attempt+1} failed: {exc}")
                if attempt == retries:
                    log(f"[real] {test} rep {rep} skipped")
    return trials


# Recorded trials of an earlier session: {test: [{csv, video, frames}]}.
def load_real_from(path, plan):
    """Reuse real CSVs: <path>/<test>/rep*.csv (an earlier iteration's real/ dir)."""
    trials = {}
    for test in plan:
        rep_csvs = sorted(path for path in glob.glob(str(Path(path) / test / "rep*.csv"))
                      if Path(path).stem[3:].isdigit())
        trials[test] = [{"csv": csv_path, "video": csv_path[:-4] + ".mp4", "frames": csv_path[:-4] + "_frames.csv"}
                     for csv_path in rep_csvs]
    return trials


# ───────────────────────────── targets + loss ─────────────────────────────
# Does a feature name match one of the test's scoring patterns?
def is_scored(feature, patterns):
    """loop_config scoring entries are feature names or glob patterns (e.g. "v_k*")."""
    import fnmatch
    return any(fnmatch.fnmatchcase(feature, pattern) for pattern in patterns)


# Real targets per test: features extracted from every repeat, aggregated to mean /
# sigma / n; only features matching the test's scoring patterns are scored.
def build_targets(real, plan, scoring, log):
    targets = {}
    for test, reps in real.items():
        if not reps:
            continue
        rep_features = [F.extract(*F.load_csv(rep["csv"]), plan[test]) for rep in reps]
        aggregated = F.aggregate(rep_features)
        scored_patterns = scoring.get(test) or []
        targets[test] = {"steps": plan[test],
                       "features": {feature: {"mean": stats["mean"], "sigma": stats["sigma"], "n": stats["n"]}
                                    for feature, stats in aggregated.items() if is_scored(feature, scored_patterns)},
                       "all_features": {feature: stats["mean"] for feature, stats in aggregated.items()}}
        scored_text = ", ".join(f"{feature}={stats['mean']:.3f}±{stats['sigma']:.3f}"
                           for feature, stats in targets[test]["features"].items()) or "(display only)"
        log(f"[targets] {test} (n={len(reps)}): {scored_text}")
    return targets


# Loss = sum over scored features of z^2, z = (sim - real mean) / sigma, where sigma
# combines the real repeat spread with the sim's own run-to-run noise. Missing features
# cost MISSING_PENALTY. Returns (loss, number of features, {test.feature: z}).
def score(sim_feats, targets):
    """sigma-weighted squared feature error (same definition as calibrate.Objective)."""
    loss, feature_count, z_scores = 0.0, 0, {}
    for test, target in targets.items():
        sim_values = sim_feats.get(test, {})
        for feature, stats in target["features"].items():
            feature_count += 1
            if feature in sim_values and np.isfinite(sim_values[feature]):
                sigma = math.hypot(max(stats["sigma"], 1e-6), CAL._sim_noise(feature) * abs(stats["mean"]))
                z = (sim_values[feature] - stats["mean"]) / sigma
                loss += z * z
                z_scores[f"{test}.{feature}"] = round(float(z), 2)
            else:
                loss += CAL.MISSING_PENALTY
                z_scores[f"{test}.{feature}"] = None
    return loss, feature_count, z_scores


# Optimiser objective (see docstring): parameter vector -> loss, one Isaac launch for
# all scored tests per candidate; results cached by rounded vector, every evaluation
# logged to opt/evals.csv and pushed to the window.
class BatchObjective:
    """theta-vector -> loss, one Isaac launch per candidate (all scored tests at once)."""

    def __init__(self, base, keys, targets, plan, surface, combine, workroot, log):
        self.base, self.keys, self.targets = dict(base), keys, targets
        self.plan = {test: plan[test] for test in targets if targets[test]["features"]}
        self.surface, self.combine, self.workroot, self.log = surface, combine, workroot, log
        self.cache, self.n = {}, 0
        self.best = None
        self.on_start = None            # f(eval_no, tag): an Isaac launch is starting
        self.on_eval = None             # f(record): a candidate finished (for the window)
        self.evals_csv = workroot / "evals.csv"
        with open(self.evals_csv, "w") as csv_file:
            csv_file.write(",".join(["eval", *keys, "loss", "rms_z", "seconds"]) + "\n")

    # Optimiser vector -> full parameter set (fitted keys over the base values).
    def theta(self, x):
        theta_full = dict(self.base)
        theta_full.update(CAL._to_theta(self.keys, x))
        return theta_full

    # Run the battery in Isaac (one retry) and extract every test's features.
    # Returns (csvs, features, seconds) or (None, None, seconds).
    def evaluate(self, theta, plan, tag):
        self.n += 1
        if self.on_start is not None:
            self.on_start(self.n, tag)
        eval_dir = self.workroot / f"eval_{self.n:03d}_{tag}"
        start_time = time.time()
        for attempt in range(2):
            try:
                sim_csvs = run_battery(theta, plan, eval_dir, self.combine)
                break
            except Exception as exc:
                self.log(f"[sim] eval {self.n} launch failed ({exc}); "
                         f"{'retrying' if attempt == 0 else 'giving up'}")
                if attempt:
                    return None, None, time.time() - start_time
        sim_features = {test: F.extract(*F.load_csv(sim_csvs[test]), plan[test]) for test in plan}
        return sim_csvs, sim_features, time.time() - start_time

    # Loss of one candidate vector (PENALTY if invalid or the launch failed).
    def __call__(self, x, tag="opt"):
        x = np.atleast_1d(np.asarray(x, float))
        cache_key = tuple(np.round(x, 6))
        if cache_key in self.cache:
            return self.cache[cache_key]["loss"]
        theta = self.theta(x)
        try:
            C.validate(theta, self.surface)
        except ValueError:
            return PENALTY
        sim_csvs, sim_features, seconds = self.evaluate(theta, self.plan, tag)
        if sim_features is None:
            return PENALTY
        loss, feature_count, z_scores = score(sim_features, self.targets)
        rms_z = math.sqrt(loss / max(feature_count, 1))
        record = {"x": x, "theta": theta, "loss": loss, "rms_z": rms_z, "csvs": sim_csvs,
               "feats": sim_features, "parts": z_scores}
        self.cache[cache_key] = record
        if self.best is None or loss < self.best["loss"]:
            self.best = record
        self.log(f"[opt] eval {self.n:2d} {tag:8s} "
                 + " ".join(f"{key}={theta[key]:.4g}" for key in self.keys)
                 + f"  loss={loss:8.2f} rms_z={rms_z:5.2f}  ({seconds:.0f}s)"
                 + ("  *best*" if self.best is record else ""))
        with open(self.evals_csv, "a") as csv_file:
            csv_file.write(",".join([str(self.n), *(f"{theta[key]:.6g}" for key in self.keys),
                              f"{loss:.4f}", f"{rms_z:.4f}", f"{seconds:.1f}"]) + "\n")
        if self.on_eval is not None:
            try:
                self.on_eval(dict(record, n=self.n, tag=tag))
            except Exception as exc:
                self.log(f"[viz] window update failed: {exc}")
        return loss


# Search box around the current values (see docstring).
def trust_box(keys, theta, surface, trust):
    """Search box in optimiser coordinates: (x0, a, b). Always centred on the current
    theta, so it stays reachable; radius = trust x each key's (lin/log) range."""
    bounds = np.array(CAL._bounds(keys, surface), float)
    lower, upper = bounds[:, 0], bounds[:, 1]
    x0 = CAL._x0(keys, theta, surface)
    radius = trust * (upper - lower)
    return x0, np.maximum(lower, x0 - radius), np.minimum(upper, x0 + radius)


# The search box in parameter units, for the window.
def trust_region(keys, theta, surface, trust):
    """Same box in parameter units -> {key: (lo, hi)} (for the window)."""
    _, box_low, box_high = trust_box(keys, theta, surface, trust)
    return {key: (CAL._to_theta([key], [low])[key], CAL._to_theta([key], [high])[key])
            for key, low, high in zip(keys, box_low, box_high)}


# Search the trust region: one key -> bounded Brent line search; several keys ->
# Nelder-Mead from a simplex inside the box, with candidates reflected back into it.
# Results are collected by the objective itself (obj.best).
def optimise(obj, keys, theta, surface, max_evals, trust, log, warm=None):
    """Bounded search inside a trust region around the current theta."""
    from scipy.optimize import minimize, minimize_scalar
    x0, box_low, box_high = trust_box(keys, theta, surface, trust)
    if warm is not None:                           # analytic hint: start the search there
        x0 = np.clip(CAL._x0(keys, warm, surface), box_low, box_high)
    log(f"[opt] trust region: " + ", ".join(
        f"{key} in [{CAL._to_theta([key], [low])[key]:.4g}, {CAL._to_theta([key], [high])[key]:.4g}]"
        for key, low, high in zip(keys, box_low, box_high)))
    if len(keys) == 1:
        minimize_scalar(lambda v: obj([v]), bounds=(box_low[0], box_high[0]), method="bounded",
                        options={"maxiter": max_evals, "xatol": 0.01 * (box_high[0] - box_low[0])})
        return
    box_span = box_high - box_low

    def reflected(x):            # keep candidates inside the trust region (no clipping
        return box_low + box_span - np.abs((np.asarray(x) - box_low) % (2 * box_span) - box_span)   # collapse)

    simplex = [np.clip(x0, box_low, box_high)]
    for axis in range(len(keys)):
        v = simplex[0].copy()
        v[axis] = box_high[axis] - 0.1 * box_span[axis] if box_high[axis] - v[axis] >= v[axis] - box_low[axis] else box_low[axis] + 0.1 * box_span[axis]
        simplex.append(v)
    minimize(lambda x: obj(reflected(x)), simplex[0], method="Nelder-Mead",
             options={"maxfev": max_evals, "initial_simplex": np.array(simplex),
                      "xatol": 1e-3, "fatol": 0.05})


# Optional iteration-1 starting point from the reduced analytic model (--warm).
def analytic_warm(theta, keys, targets, combine, surface, log):
    """Iteration-1 hint from the reduced analytic model (only keys it represents)."""
    terms = [(test, entry["steps"], entry["features"], 1.0) for test, entry in targets.items() if entry["features"]]
    try:
        sensitive_keys = CAL.analytic_sensitive_keys(theta, keys, terms, combine, surface)
        if not sensitive_keys:
            return None
        objective = CAL.Objective(theta, sensitive_keys, terms, "analytic", combine, surface)
        best_params, model_loss = CAL.optimize_stage(objective, sensitive_keys, max_evals=40, seed=0, surface=surface)
        log(f"[opt] analytic warm start ({objective.n_eval} model evals): "
            + ", ".join(f"{key}={entry:.4g}" for key, entry in best_params.items()) + f"  (model loss {model_loss:.2f})")
        warm_theta = dict(theta)
        warm_theta.update(best_params)
        return warm_theta
    except Exception as exc:
        log(f"[opt] analytic warm start unavailable: {exc}")
        return None


# ───────────────────────────── visual outputs ─────────────────────────────
# Move a start-relative sim track onto the real start pose (rotate + translate).
def to_real_frame(sim_rows, real_start):
    """Place a sim track (start-relative) at the real start pose in grid metres."""
    sim = np.asarray(sim_rows, float)
    sim_x0, sim_y0, sim_yaw0 = sim[0, 1], sim[0, 2], sim[0, 3]
    cos_r, sin_r = math.cos(-sim_yaw0), math.sin(-sim_yaw0)
    rel_x = cos_r * (sim[:, 1] - sim_x0) - sin_r * (sim[:, 2] - sim_y0)
    rel_y = sin_r * (sim[:, 1] - sim_x0) + cos_r * (sim[:, 2] - sim_y0)
    real_x0, real_y0, real_yaw0 = real_start
    cos_real, sin_real = math.cos(real_yaw0), math.sin(real_yaw0)
    out = sim.copy()
    out[:, 1] = real_x0 + cos_real * rel_x - sin_real * rel_y
    out[:, 2] = real_y0 + sin_real * rel_x + cos_real * rel_y
    out[:, 3] = wrap(sim[:, 3] - sim_yaw0 + real_yaw0)
    return out


# Draw real vs sim-before vs sim-after on the real overhead camera image
# (paths_map.png) and, over the real trial's video, frame by frame (paths_map.mp4).
def path_maps(test, reps, sim_before, sim_after, outdir, H_m2img, log, theta_txt="",
              labels=None):
    """paths_map.png (+ .mp4 over the real video) of real vs sim-before vs sim-after,
    all drawn on the REAL overhead camera view via the grid homography."""
    import cv2
    first_rep = reps[0]
    real_rows = load_rows(first_rep["csv"])
    real_start = tuple(real_rows[0, 1:4])
    tracks = {"real": real_rows}
    if sim_before:
        tracks["before"] = to_real_frame(load_rows(sim_before), real_start)
    if sim_after:
        tracks["after"] = to_real_frame(load_rows(sim_after), real_start)

    # Track rows (metres) -> integer image pixels via the grid homography.
    def px(rows):
        return cv2.perspectiveTransform(rows[:, 1:3].reshape(-1, 1, 2).astype(float),
                                        H_m2img).reshape(-1, 2).astype(np.int32)

    # Legend + caption text on an image (optionally with the video time).
    def legend(img, tf=None):
        legend_names = {"real": "real (camera)", "before": "sim BEFORE update",
                 "after": "sim AFTER update"}
        legend_names.update(labels or {})
        for line_index, (track_name, _) in enumerate(tracks.items()):
            cv2.putText(img, legend_names[track_name], (12, 30 + 26 * line_index), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, legend_names[track_name], (12, 30 + 26 * line_index), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        COLORS[track_name], 2, cv2.LINE_AA)
        caption = f"{test}" + (f"  t={tf:4.1f}s" if tf is not None else "") + (f"   {theta_txt}" if theta_txt else "")
        cv2.putText(img, caption, (12, img.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, caption, (12, img.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

    video = cv2.VideoCapture(first_rep["video"]) if Path(first_rep["video"]).exists() else None
    first_frame = None
    if video is not None:
        read_ok, first_frame = video.read()
    if first_frame is None:
        first_frame = np.full((720, 1280, 3), 40, np.uint8)
    img = first_frame.copy()
    for other_rep in reps[1:]:
        other_rows = load_rows(other_rep["csv"])
        cv2.polylines(img, [px(other_rows)], False, (40, 140, 40), 1, cv2.LINE_AA)
    for track_name, rows in tracks.items():
        cv2.polylines(img, [px(rows)], False, COLORS[track_name], 2, cv2.LINE_AA)
        cv2.circle(img, tuple(px(rows)[-1]), 5, COLORS[track_name], -1)
    legend(img)
    cv2.imwrite(str(outdir / "paths_map.png"), img)

    # progressive overlay on the recorded real video (time-aligned to the sequence)
    if video is not None and Path(first_rep["frames"]).exists():
        frame_times = [float(r["t"]) for r in csv.DictReader(open(first_rep["frames"]))]
        video.set(cv2.CAP_PROP_POS_FRAMES, 0)
        video_w, video_h = first_frame.shape[1], first_frame.shape[0]
        video_writer = cv2.VideoWriter(str(outdir / "paths_map.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                             30.0, (video_w, video_h))
        for frame_time in frame_times:
            read_ok, video_frame = video.read()
            if not read_ok:
                break
            for track_name, rows in tracks.items():
                visible = rows[:, 0] <= max(frame_time, 0.0)
                if visible.sum() >= 2:
                    cv2.polylines(video_frame, [px(rows[visible])], False, COLORS[track_name], 2, cv2.LINE_AA)
            legend(video_frame, frame_time)
            video_writer.write(video_frame)
        video_writer.release()
    if video is not None:
        video.release()
    log(f"[viz] {test}: {outdir/'paths_map.png'}")


# Command -> response maps (real vs sim before/after) and their gaps; best effort.
def response_png(test, steps, real_reps, sim_before, sim_after, outdir):
    """Command -> response map: every real repeat vs sim BEFORE and AFTER (best effort)."""
    try:
        import response_map as RM
        real_summary = RM.summarise(RM.step_responses(steps, [r["csv"] for r in real_reps]))
        gaps = {}
        for tag, sim_csv in (("before", sim_before), ("after", sim_after)):
            if sim_csv:
                merged = RM.merge(real_summary, RM.summarise(RM.step_responses(steps, [sim_csv])))
                RM.plot(merged, str(outdir / f"response_map_{tag}.png"), f"{test}: command -> response ({tag})")
                gaps[tag] = RM.gap(merged)
        (outdir / "response_map.json").write_text(json.dumps(gaps, indent=1))
        return gaps
    except Exception:
        return {}


# Files listed per test in the session index.
def iteration_files(tdir, real_dir):
    return {"paths on the real camera, real vs sim before/after (still)": tdir / "paths_map.png",
            "comparison BEFORE": tdir / "compare_before.png",
            "comparison AFTER": tdir / "compare_after.png",
            "command -> response BEFORE": tdir / "response_map_before.png",
            "command -> response AFTER": tdir / "response_map_after.png",
            "paths on the real camera (video)": tdir / "paths_map.mp4",
            "real camera rep 1 (raw video)": real_dir / "rep1.mp4",
            "real path rep 1 csv": real_dir / "rep1.csv",
            "metrics BEFORE": tdir / "metrics_before.json",
            "metrics AFTER": tdir / "metrics_after.json",
            "response-map gaps": tdir / "response_map.json"}


# Run trials/compare_paths.py for one real vs sim track; returns its metrics json.
def compare_png(real_csv, sim_csv, out_png, out_json, label):
    subprocess.run([sys.executable, str(SC / "trials" / "compare_paths.py"), str(real_csv),
                    str(sim_csv), "--out", str(out_png), "--json", str(out_json),
                    "--label", label], check=False, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    try:
        return json.loads(Path(out_json).read_text())
    except Exception:
        return None


# progress.png: rms z before/after per iteration + each fitted key's trajectory.
def progress_plot(history, keys, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    iterations = [past["iteration"] for past in history]
    fig, axes = plt.subplots(1, 1 + len(keys), figsize=(5 * (1 + len(keys)), 4))
    axes = np.atleast_1d(axes)
    axes[0].plot(iterations, [past["rms_before"] for past in history], "o-", color="#d62728",
                 label="sim BEFORE update (fresh real data)")
    axes[0].plot(iterations, [past["rms_after"] for past in history], "s-", color="#1f77b4",
                 label="sim AFTER update")
    axes[0].axhline(1.0, color="k", ls=":", lw=1, label="camera noise floor (1 sigma)")
    axes[0].set_xlabel("iteration"); axes[0].set_ylabel("rms feature error [sigma]")
    axes[0].set_title("sim vs real mismatch"); axes[0].legend(fontsize=8)
    axes[0].grid(alpha=.3)
    for axis, key in zip(axes[1:], keys):
        axis.plot([0] + iterations, [history[0]["theta_before"][key]] + [past["theta_after"][key] for past in history],
                "o-", color="#2ca02c")
        axis.set_title(key); axis.set_xlabel("iteration"); axis.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


# summary.md: per-iteration rms z and values, per-test path errors, start-pose
# repeatability, the final parameters.
def write_summary(session, args, keys, history, final_theta, surface_out):
    lines = [f"# Closed sim2real loop — {args.surface}", "",
         f"session `{session.name}`  ·  tests {', '.join(args._tests)}  ·  "
         f"{args.repeats} real repeats/test  ·  keys {', '.join(keys)}", "",
         "| iter | rms z before | rms z after | " + " | ".join(keys) + " | real trials |",
         "|---|---|---|" + "---|" * len(keys) + "---|"]
    for past in history:
        lines.append(f"| {past['iteration']} | {past['rms_before']:.2f} | {past['rms_after']:.2f} | "
                 + " | ".join(f"{past['theta_after'][key]:.4g}" for key in keys)
                 + f" | {past['n_real']} |")
    lines += ["", "rms z = root-mean-square feature error in units of the real rover's "
          "repeat-to-repeat sigma (≈1 means the sim is inside the real rover's own scatter). "
          "'before' scores the parameters carried INTO the iteration against that "
          "iteration's fresh real repeats (an out-of-sample check of the previous update).",
          "", "## Per-test path error (start-aligned, real rep 1)", "",
          "| iter | test | endpoint before | endpoint after | heading before | heading after |",
          "|---|---|---|---|---|---|"]
    for past in history:
        for test, path_err in past["paths"].items():
            before, after = path_err.get("before") or {}, path_err.get("after") or {}
            lines.append(f"| {past['iteration']} | {test} | {before.get('endpoint_err_m', float('nan'))*100:.1f} cm"
                     f" | {after.get('endpoint_err_m', float('nan'))*100:.1f} cm"
                     f" | {before.get('heading_err_deg', float('nan')):.1f}° | "
                     f"{after.get('heading_err_deg', float('nan')):.1f}° |")
    lines += ["", "## Start-pose repeatability (all real trials)", "",
          "| test | trials | via lidar | lidar err mean / max | camera spread mean / max | "
          "camera yaw sd |", "|---|---|---|---|---|---|"]
    for test in args._tests:
        start_records = [record for past in history for record in past.get("starts", {}).get(test, []) if record]
        if not start_records:
            continue
        camera_poses = np.array([record["camera_pose"] for record in start_records])
        spread = np.hypot(camera_poses[:, 0] - camera_poses[:, 0].mean(), camera_poses[:, 1] - camera_poses[:, 1].mean())
        lidar_errors = [record["lidar_err_m"] for record in start_records if "lidar_err_m" in record]
        lines.append(f"| {test} | {len(start_records)} | {sum(record.get('method') == 'lidar' for record in start_records)} | "
                 + (f"{np.mean(lidar_errors)*100:.1f} / {np.max(lidar_errors)*100:.1f} cm" if lidar_errors else "n/a")
                 + f" | {spread.mean()*100:.1f} / {spread.max()*100:.1f} cm | "
                 f"{math.degrees(np.std(np.unwrap(camera_poses[:, 2]))):.1f}° |")
    lines += ["", f"Final parameters → `{surface_out}`", "", "```yaml",
          *(f"{key}: {final_theta[key]:.6g}" for key in keys), "```", "",
          "Per iteration: `iter_NN/<test>/paths_map.{png,mp4}` (real vs sim before/after on "
          "the real camera view), `compare_{before,after}.png`, `real/`, `opt/evals.csv`."]
    (session / "summary.md").write_text("\n".join(lines) + "\n")


# ───────────────────────────── main loop ─────────────────────────────
# Drive the rover to loop_config 'park' at the end of the real phase.
def park_rover(rover, tracker, cfg, log, view):
    view.status("parking the rover")
    park_pose = cfg.get("park")
    if park_pose:
        try:
            go_to_pose(rover, tracker, (park_pose["x"], park_pose["y"], math.radians(park_pose["yaw"])), cfg, log=log)
        except Exception as exc:
            log(f"[real] park failed: {exc}")


# The whole loop (see module docstring). Per iteration: real trials (or --real-from),
# targets, Isaac BEFORE run, optimisation (unless already converged), AFTER result,
# compare plots / path maps / response maps, iteration.json + theta.yaml, the surface
# yaml, summary and session index. --real-only stops after recording.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--surface", required=True,
                    help="floor material (concrete|carpet|foam or alias): sets the floor-mu "
                         "band and names the outputs")
    parser.add_argument("--tests", default="T1,T2,T4", help="comma list of test stems / prefixes")
    parser.add_argument("--repeats", type=int, default=3, help="real repeats per test per iteration")
    parser.add_argument("--iterations", type=int, default=3)
    parser.add_argument("--keys", default=None,
                    help="comma list of sim parameters to update (default: loop_config.yaml keys)")
    parser.add_argument("--max-evals", type=int, default=12, help="Isaac evaluations per iteration")
    parser.add_argument("--trust", type=float, default=0.5,
                    help="iteration-1 search radius as a fraction of each key's range; "
                         "shrinks x0.6 per iteration")
    parser.add_argument("--converge", type=float, default=1.2,
                    help="stop when 'before' rms z (fresh real data) is at or below this")
    parser.add_argument("--init", default=None,
                    help="start parameters: a surfaces/*.yaml (its 'calibrated') or a nested "
                         "sim_calibration yaml; default = schema start + surface floor band")
    parser.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    parser.add_argument("--config", default=str(CONFIG))
    parser.add_argument("--real-from", default=None,
                    help="reuse real CSVs (<dir>/<test>/rep*.csv) instead of driving the rover")
    parser.add_argument("--real-only", action="store_true", help="drive + record, skip the sim")
    parser.add_argument("--warm", action="store_true",
                    help="iteration 1: start the search at the reduced analytic model's optimum "
                         "(off by default - that model omits contact/torque effects)")
    parser.add_argument("--no-videos", action="store_true")
    parser.add_argument("--show", dest="show", action="store_true", default=True,
                    help="persistent session window (default when $DISPLAY is set)")
    parser.add_argument("--no-show", dest="show", action="store_false",
                    help="no window (session live.jpg is still written)")
    parser.add_argument("--no-hold", action="store_true",
                    help="exit at the end instead of keeping the window up")
    parser.add_argument("--allow-approximate", action="store_true")
    parser.add_argument("--keep-bringup", action="store_true",
                    help="leave a rover base driver we started running at the end")
    parser.add_argument("--test-fake-rover", action="store_true", help=argparse.SUPPRESS)
    parser.add_argument("--resume", default=None, metavar="SESSION_DIR",
                    help="continue an interrupted session (e.g. after a rover brown-out): "
                         "recorded real trials and completed iterations are kept")
    parser.add_argument("--trial-order", choices=("round-robin", "test"), default="round-robin",
                    help="round-robin (default): rep 1 of every test, then rep 2, ... so a run cut "
                         "short still covers every test; test: all repeats of one test first")
    parser.add_argument("--realign", metavar="REAL_DIR", default=None,
                    help="re-time recorded trials (<dir>/<test>/rep*_raw.csv -> rep*.csv) "
                         "with the guarded alignment, then exit")
    parser.add_argument("--out", default=None, help="final surface yaml "
                    "(default surfaces/<surface>_loop.yaml)")
    args = parser.parse_args()

    config = load_config(args.config)
    keys = [key.strip() for key in (args.keys or ",".join(config["optimise"]["keys"])).split(",")
            if key.strip()]
    for key in keys:
        if key not in C.OPTIMISABLE:
            sys.exit(f"'{key}' is not optimisable (geometry/masses are locked). "
                     f"Choose from {C.OPTIMISABLE}")
    tests = [resolve_test(test_name.strip()) for test_name in args.tests.split(",") if test_name.strip()]
    plan = {test_stem: F.load_sequence(str(sequence_path)) for test_stem, sequence_path in tests}
    args._tests = list(plan)
    scoring = {test_name: config["scoring"].get(test_name, []) for test_name in plan}

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = "TESTFAKE_" if args.test_fake_rover else ""     # never mistaken for real data
    session = Path(args.resume) if args.resume else HERE / "runs" / f"{prefix}{args.surface}_{stamp}"
    session.mkdir(parents=True, exist_ok=True)
    log = Log(session / "session.log")
    shutil.copy(args.config, session / "loop_config.yaml")
    (session / "args.json").write_text(json.dumps({key: value for key, value in vars(args).items()},
                                                  indent=2, default=str))

    measured = CAL.load_measured(str(SC / "optimize" / "measured.yaml"))
    seed_params = None
    if args.init:
        import yaml
        init_doc = yaml.safe_load(Path(args.init).read_text()) or {}
        seed_params = init_doc.get("calibrated") or C.flatten(init_doc.get("sim_calibration_yaml", init_doc))
    theta = CAL.start_theta(measured, args.surface, seed_params)
    log(f"[loop] session {session}")
    log(f"[loop] tests {list(plan)}  repeats {args.repeats}  iterations {args.iterations}  "
        f"keys {keys}")
    log(f"[loop] scored features: " + "; ".join(f"{test_name}: {scoring[test_name] or 'display only'}"
                                                 for test_name in plan))
    log(f"[loop] start theta: " + ", ".join(f"{key}={theta[key]:.4g}" for key in keys))

    rover = cam = view = tracker = None
    history = []
    first_it = 1
    if args.resume:                      # completed iterations carry their theta forward
        import yaml
        for resume_iter_dir in sorted(session.glob("iter_*")):
            record_path, theta_path = resume_iter_dir / "iteration.json", resume_iter_dir / "theta.yaml"
            if record_path.exists() and theta_path.exists():
                history.append(json.loads(record_path.read_text()))
                theta.update({key: value for key, value in C.flatten(yaml.safe_load(theta_path.read_text())).items()
                              if key in theta})
                first_it = history[-1]["iteration"] + 1
        log(f"[loop] RESUMING {session.name}: {len(history)} completed iteration(s); "
            f"continuing at iteration {first_it} with "
            + ", ".join(f"{key}={theta[key]:.4g}" for key in keys))
    if args.realign:
        realign_trials(args.realign, plan, config, log)
        return 0
    index = Session(session, "test" if args.test_fake_rover else "optimizer",
                    f"Closed sim2real loop - {args.surface} ({session.name})",
                    meta={"surface": args.surface, "tests": ", ".join(plan),
                          "repeats per test": args.repeats, "iterations": args.iterations,
                          "drive": "wheel contact (PhysX wheel drives)",
                          "fitted parameters": ", ".join(keys),
                          "real data": args.real_from or "real rover (this session)",
                          "max Isaac evals / iteration": args.max_evals})
    index.snapshot_config({"loop config": args.config,
                           "grid calibration": SC / "calib" / "grid_calib.json",
                           "sticker config": SC / "calib" / "sticker_config.json",
                           "start parameters": args.init,
                           **{f"sequence {test_name}": sequence_path for test_name, sequence_path in tests}})
    session_video_file = session_video_path(session)         # what LiveView will record to
    index.add_video("whole session (dashboard recording)" if session_video_file.name == "session_video.mp4"
                    else f"resumed session ({session_video_file.stem.split('_')[-1]})", session_video_file)
    index.add_video("progress plot (rms z + parameters per iteration)", session / "progress.png")
    index.write()
    H_m2img = None
    exit_code = None
    try:
        # ---- hardware ----
        from camera_track import CameraTrack
        if args.test_fake_rover:
            # TEST ONLY: fake_rover.py + fake camera stand in for the hardware
            import os
            import subprocess as sp
            sys.path.insert(0, str(HERE / "tests"))
            saved_env = {key: os.environ.get(key) for key in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
            os.environ.update(ROS_DOMAIN_ID="90", ROS_LOCALHOST_ONLY="1")
            import test_stations_sim as TS
            fake_rover_proc = sp.Popen([sys.executable, str(HERE / "tests" / "fake_rover.py"),
                             "0.5", "0.5", "1.5708"], start_new_session=True)
            import atexit
            atexit.register(lambda: os.killpg(fake_rover_proc.pid, 15))
            time.sleep(2.0)
            cam = TS.FakeCamera()
            from rover_link import RoverLink
            rover = RoverLink(host="local", log=log, fusion_cfg=config.get("fusion", {}))
            rover.open()
            ST.set_guard(rover, config)
            from localizer import FusedTracker
            tracker = FusedTracker(cam, rover, cam_latency_s=0.15)
            # Isaac must NOT share the fake rover's domain (its /cmd_vel would drive it)
            for key, value in saved_env.items():
                if value is None:
                    os.environ.pop(key, None)
                else:
                    os.environ[key] = value
            from metric_overlay import load_calib
            _, H_m2img, grid_dims_m = load_calib(str(SC / "calib" / "grid_calib.json"))
            view = LiveView(cam, session, args.show, grid_dims_m, _ws(config))
            log.sink = view.dash.log
            ST.ST_DIR = session / "stations"          # isolated stations for the test
            config["stations"]["lidar_offset"] = [0.0, 0.0, 0.0]   # fake rover conventions
            config["stations"]["lidar_flip"] = False
            for test_name in plan:
                ST.teach(rover, cam, tracker, test_name, config, log)
        elif args.real_from is None:
            from rover_link import RoverLink
            cam = CameraTrack(allow_approximate=args.allow_approximate)
            cam.latency_s = config.get("fusion", {}).get("cam_latency_s", 0.0)
            camera_check(cam, log)
            H_m2img = cam.H_m2img
            view = LiveView(cam, session, args.show, cam.grid_m, _ws(config))
            log.sink = view.dash.log
            view.status("connecting to the rover")
            rover = RoverLink(log=log, logfile=str(session / "rover_agent.log"),
                              fusion_cfg=config.get("fusion", {}))
            rover.open()
            ST.set_guard(rover, config)
            fusion_cfg = config.get("fusion", {})
            if fusion_cfg.get("enabled", True):
                from localizer import FusedTracker
                tracker = FusedTracker(cam, rover, cam_latency_s=fusion_cfg.get("cam_latency_s", 0.2),
                                       gain=fusion_cfg.get("gain", 0.3))
            else:
                tracker = cam
            taught_stations = ST.list_stations()
            log(f"[loop] stations taught: {taught_stations or 'none'}"
                + ("" if all(test_name in taught_stations for test_name in plan) else
                   f"  (untaught -> camera-only config pose: {[test_name for test_name in plan if test_name not in taught_stations]};"
                   f" teach with loop/stations.py teach <test>)"))
        else:
            from metric_overlay import load_calib
            _, H_m2img, grid_dims_m = load_calib(str(SC / "calib" / "grid_calib.json"))
            view = LiveView(None, session, args.show, grid_dims_m, _ws(config))
            log.sink = view.dash.log

        params = ParamPanel(view, theta, keys, args.surface, history)
        params.push(f"start values ({'resumed' if history else args.init or 'schema start'}"
                    f", surface {args.surface})")
        for iteration in range(first_it, args.iterations + 1):
            iter_dir = session / f"iter_{iteration:02d}"
            iter_dir.mkdir(exist_ok=True)
            log("=" * 78)
            log(f"[loop] ITERATION {iteration}/{args.iterations}")
            view.status(f"iteration {iteration}/{args.iterations}: real trials")

            # 1. real
            if args.real_from:
                real_trials = load_real_from(args.real_from, plan)
                log(f"[real] reusing {sum(len(value) for value in real_trials.values())} real runs from "
                    f"{args.real_from}")
            else:
                if iteration > 1:
                    camera_check(cam, log)         # bumped mid-session?
                real_trials = real_phase(rover, cam, tracker, config, plan, args.repeats, iter_dir, log,
                                  view, order=args.trial_order)
                if iteration == args.iterations and rover is not None:
                    # nothing after this needs the rover: park, stop the base driver and say so
                    park_rover(rover, tracker, config, log, view)
                    if tracker is not cam:
                        tracker.close()
                    tracker = cam
                    rover.close(stop_bringup=not args.keep_bringup)
                    rover = None
                    log("[real] " + "#" * 60)
                    log("[real] REAL ROVER DONE - all real trials recorded; it is safe to "
                        "power the rover off (sim phase continues)")
                    log("[real] " + "#" * 60)
                    view.status("REAL ROVER DONE - safe to power it off; sim phase running")
            if not any(real_trials.values()):
                raise RuntimeError("no successful real runs this iteration")
            for test in plan:                      # tiles show the fresh real data at once
                if real_trials.get(test):
                    view.test_tile(test, real_trials[test], [],
                                   [f"iter {iteration}: {len(real_trials[test])} real repeats"
                                    + ("" if args.real_only else ", sim pending")],
                                   f"{test}  iter {iteration}")
            if args.real_only:
                build_targets(real_trials, plan, scoring, log)
                continue

            # 2. targets
            targets = build_targets(real_trials, plan, scoring, log)
            (iter_dir / "targets.json").write_text(json.dumps(
                {test_name: {"features": value["features"], "all_features": value["all_features"]}
                 for test_name, value in targets.items()}, indent=2))
            if not any(value["features"] for value in targets.values()):
                raise RuntimeError("no scored features - check loop_config.yaml scoring")

            # 3. sim BEFORE (full battery incl. display-only tests), scored on fresh data
            opt_dir = iter_dir / "opt"
            opt_dir.mkdir(exist_ok=True)
            objective = BatchObjective(theta, keys, targets, plan, args.surface, args.combine_mode,
                                 opt_dir, log)
            params.iteration(iteration, args.iterations, max(args.trust * 0.6 ** (iteration - 1), 0.08))
            sim_tiles = SimTiles(view, real_trials, keys, iteration, args, params)
            objective.on_start, objective.on_eval = sim_tiles.start, sim_tiles.done
            x_current = CAL._x0(keys, theta, args.surface)
            log(f"[sim] BEFORE: current parameters on the full battery")
            csvs_before, features_before, _ = objective.evaluate(theta, {test_name: plan[test_name] for test_name in real_trials if real_trials[test_name]},
                                             "before")
            if features_before is None:
                raise RuntimeError("baseline Isaac run failed - see the eval dir isaac.log")
            loss_before, n_features, z_before = score(features_before, targets)
            rms_before = math.sqrt(loss_before / max(n_features, 1))
            sim_tiles.done({"n": objective.n, "tag": "before", "theta": dict(theta), "loss": loss_before,
                       "rms_z": rms_before, "csvs": csvs_before,
                       "per_test": per_test_rms(z_before)})
            objective.best = {"x": x_current, "theta": dict(theta), "loss": loss_before, "rms_z": rms_before,
                        "csvs": csvs_before, "feats": features_before, "parts": z_before}
            objective.cache[tuple(np.round(x_current, 6))] = objective.best
            log(f"[sim] BEFORE loss={loss_before:.2f} rms_z={rms_before:.2f}  per-feature z: {z_before}")

            # 4./5. update
            converged = rms_before <= args.converge
            if converged:
                log(f"[loop] converged: rms_z {rms_before:.2f} <= {args.converge} on fresh real "
                    f"data - no update needed")
            else:
                warm_start = None if (not args.warm or iteration > 1) else analytic_warm(
                    theta, keys, targets, args.combine_mode, args.surface, log)
                trust_fraction = max(args.trust * 0.6 ** (iteration - 1), 0.08)
                optimise(objective, keys, theta, args.surface, args.max_evals, trust_fraction, log, warm_start)
            best_record = objective.best
            theta_new = dict(best_record["theta"])

            # 6. verify AFTER on the full battery (display-only tests need a run too)
            if best_record["x"] is x_current or np.allclose(best_record["x"], x_current):
                csvs_after, features_after, rms_after, loss_after = csvs_before, features_before, rms_before, loss_before
                log("[loop] no candidate beat the current parameters this iteration")
            elif set(best_record["csvs"]) >= {test_name for test_name in plan if real_trials.get(test_name)}:
                csvs_after, features_after, rms_after, loss_after = best_record["csvs"], best_record["feats"], best_record["rms_z"], best_record["loss"]
            else:
                log("[sim] AFTER: best parameters on the full battery")
                csvs_after, features_after, _ = objective.evaluate(theta_new, {test_name: plan[test_name] for test_name in real_trials if real_trials[test_name]},
                                                 "after")
                loss_after, n_features_after, _ = score(features_after, targets) if features_after else (loss_before, n_features, None)
                rms_after = math.sqrt(loss_after / max(n_features, 1))
            log(f"[loop] iteration {iteration}: rms_z {rms_before:.2f} -> {rms_after:.2f}   "
                + ", ".join(f"{key}: {theta[key]:.4g} -> {theta_new[key]:.4g}" for key in keys))

            # visual + per-test outputs
            view.status(f"iteration {iteration}: comparing + drawing path maps")
            path_errors = {}
            for test in plan:
                if not real_trials.get(test):
                    continue
                test_dir = iter_dir / test
                test_dir.mkdir(exist_ok=True)
                sim_before_csv, sim_after_csv = (csvs_before or {}).get(test), (csvs_after or {}).get(test)
                real_csv = real_trials[test][0]["csv"]
                path_errors[test] = {
                    "before": compare_png(real_csv, sim_before_csv, test_dir / "compare_before.png",
                                          test_dir / "metrics_before.json", f"{test} before") if sim_before_csv else None,
                    "after": compare_png(real_csv, sim_after_csv, test_dir / "compare_after.png",
                                         test_dir / "metrics_after.json", f"{test} after") if sim_after_csv else None}
                path_before, path_after = path_errors[test]["before"] or {}, path_errors[test]["after"] or {}
                view.test_tile(test, real_trials[test],
                               [("sim BEFORE", sim_before_csv, COLORS["before"]),
                                ("sim AFTER", sim_after_csv, COLORS["after"])],
                               [f"rms z {rms_before:.2f} -> {rms_after:.2f}",
                                f"endpoint {path_before.get('endpoint_err_m', float('nan'))*100:.1f} -> "
                                f"{path_after.get('endpoint_err_m', float('nan'))*100:.1f} cm",
                                "  ".join(f"{key[:10]}={theta_new[key]:.3g}" for key in keys)],
                               f"{test}  iter {iteration} (done)")
                try:
                    theta_text = "  ".join(f"{key}={theta_new[key]:.3g}" for key in keys)
                    if not args.no_videos:
                        path_maps(test, real_trials[test], sim_before_csv, sim_after_csv, test_dir, H_m2img, log, theta_text)
                except Exception as exc:
                    log(f"[viz] {test}: path map failed: {exc}")
                response_gaps = response_png(test, plan[test], real_trials[test], sim_before_csv, sim_after_csv, test_dir)
                real_dir = Path(real_trials[test][0]["csv"]).parent
                index.add_test(f"iteration {iteration} - {test}", metrics={
                    "real repeats": len(real_trials[test]),
                    "endpoint before [m]": path_before.get("endpoint_err_m"),
                    "endpoint after [m]": path_after.get("endpoint_err_m"),
                    "heading before [deg]": path_before.get("heading_err_deg"),
                    "heading after [deg]": path_after.get("heading_err_deg"),
                    "response gap before (rms z)": (response_gaps.get("before") or {}).get("rms_z"),
                    "response gap after (rms z)": (response_gaps.get("after") or {}).get("rms_z")},
                    files=iteration_files(test_dir, real_dir))
            iteration_record = {"iteration": iteration, "rms_before": rms_before, "rms_after": rms_after,
                   "loss_before": loss_before, "loss_after": loss_after,
                   "theta_before": theta_view(theta, keys), "theta_after": theta_view(theta_new, keys),
                   "n_real": sum(len(value) for value in real_trials.values()), "paths": path_errors,
                   "z_before": z_before, "z_after": best_record.get("parts"),
                   "evals": objective.n, "converged": converged,
                   "starts": {test_name: [real_run.get("start") for real_run in real_trials[test_name] if real_run.get("start")]
                              for test_name in real_trials}}
            (iter_dir / "iteration.json").write_text(json.dumps(iteration_record, indent=2, default=float))
            import yaml
            (iter_dir / "theta.yaml").write_text(yaml.safe_dump(
                C.nest({key: value for key, value in theta_new.items() if key not in C.LOCKED}), sort_keys=False))
            history.append(iteration_record)
            theta = theta_new
            params.finished(theta_new, iteration, rms_before, rms_after)
            progress_plot(history, keys, session / "progress.png")
            # fake-rover tests must never replace a real surface/model file
            surfaces_dir = session if args.test_fake_rover else SC / "surfaces"
            surface_out = Path(args.out) if args.out else surfaces_dir / f"{args.surface}_loop.yaml"
            protect(surface_out, session, log)
            CAL.write_surface(str(surface_out), args.surface, theta,
                              {f"iteration_{past['iteration']}": {**past["theta_after"],
                                                                "rms_z": past["rms_after"]}
                               for past in history}, args.combine_mode)
            write_summary(session, args, keys, history, theta, surface_out)
            log(f"[loop] wrote {surface_out} and {session/'summary.md'}")
            index.set_summary([f"iteration {past['iteration']}: rms z {past['rms_before']:.2f} -> "
                               f"{past['rms_after']:.2f}; " + ", ".join(
                                   f"{key}={past['theta_after'][key]:.4g}" for key in keys) for past in history])
            index.snapshot_config({"result (written by this session)": surface_out})
            index.add_test("summary", files={"summary.md": session / "summary.md",
                                             "session log": session / "session.log",
                                             f"iteration {iteration} theta.yaml": iter_dir / "theta.yaml",
                                             "optimiser evals csv": iter_dir / "opt" / "evals.csv"})
            index.write()
            view.status(f"iteration {iteration} done: rms z {rms_before:.2f} -> {rms_after:.2f}")
            if converged:
                break

        # park the rover at the end
        if rover is not None:
            park_rover(rover, tracker, config, log, view)
    except KeyboardInterrupt:
        log("[loop] interrupted - stopping the rover")
        exit_code = 130
    except Exception as exc:
        log(f"[loop] FAILED: {exc}")
        log(traceback.format_exc())
        if view is not None:
            view.status(f"FAILED: {exc}"[:90])
        exit_code = 1
    finally:
        # Results first (final view, finished index, finalised video), THEN the hardware
        # teardown: a crash while closing the camera/rover must not cost the recordings.
        if view is not None:
            try:
                view.dash.save_frame(session / "final_view.png")
                index.add_video("final dashboard view", session / "final_view.png")
                view.dash.stop_recording()
                index.write(finished=exit_code is None)
                log(f"[loop] results: {session / 'index.md'}  (catalog: {SC / 'RESULTS.md'})")
            except Exception as exc:
                log(f"[loop] index not written: {exc}")
        if rover is not None:
            try:
                rover.close(stop_bringup=not args.keep_bringup)
            except Exception as exc:
                log(f"[rover] close: {exc}")
        if tracker is not None and tracker is not cam:
            tracker.close()
        if cam is not None:
            cam.close()
        if view is not None and exit_code is None:
            log(f"[loop] done. summary: {session/'summary.md'}")
        if view is not None:
            log.sink = None
            view.close(hold=not args.no_hold and exit_code != 130)
    return exit_code or 0


if __name__ == "__main__":
    raise SystemExit(main())
