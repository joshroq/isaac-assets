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
class Log:
    def __init__(self, path):
        self.f = open(path, "a")
        self.sink = None                # dashboard ticker, once the window exists

    def __call__(self, *parts):
        line = " ".join(str(p) for p in parts)
        stamp = datetime.datetime.now().strftime("%H:%M:%S")
        print(f"{stamp} {line}", flush=True)
        self.f.write(f"{stamp} {line}\n")
        self.f.flush()
        if self.sink is not None:
            self.sink(f"{stamp} {line}")


def resolve_test(name):
    """'T1' / 'T1_straight' / path -> (stem, path)."""
    p = Path(name)
    if p.suffix in (".yaml", ".yml") and p.exists():
        return p.stem, p
    exact = SEQ_DIR / f"{name}.yaml"
    if exact.exists():
        return name, exact
    hits = sorted(glob.glob(str(SEQ_DIR / f"{name}_*.yaml")))
    if not hits:
        sys.exit(f"no sequence for test '{name}' in {SEQ_DIR}")
    return Path(hits[0]).stem, Path(hits[0])


def write_csv(path, rows, header=("t", "x", "y", "yaw")):
    with open(path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(header)
        for r in rows:
            w.writerow([f"{v:.5f}" for v in r])


def load_rows(path):
    t, x, y, yaw = F.load_csv(str(path))
    return np.column_stack([t, x, y, yaw])


def wrap(a):
    return (a + np.pi) % (2 * np.pi) - np.pi


def theta_view(theta, keys):
    return {k: float(theta[k]) for k in keys}


# ───────────────────────────── real side ─────────────────────────────
def session_video_path(session):
    """session_video.mp4, or _part2/_part3... when resuming (never overwrite a recording)."""
    p = Path(session) / "session_video.mp4"
    k = 2
    while p.exists():
        p = Path(session) / f"session_video_part{k}.mp4"
        k += 1
    return p


class LiveView:
    """The session's one persistent window (viz/dashboard.py) + a 1 Hz live.jpg.

    Created once and kept open for every phase: live camera (homing, driving), the
    current real trial top-down, and per-test gallery tiles of real vs sim (before /
    optimiser candidates / after) that update as each run finishes. real_trial() only
    sets .trail / .text and calls update(pose)."""

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

    def test_tile(self, test, real_reps, sims, lines, title=None):
        """Gallery tile for one test: every real repeat + sim tracks placed at rep 1's start.
        sims = [(label, rows | csv path, colour)]."""
        try:
            reals = [load_rows(r["csv"]) for r in real_reps]
        except Exception:
            return
        if not reals or not len(reals[0]):
            return
        start = tuple(reals[0][0, 1:4])
        tracks = [Track("real", reals[0], COLORS["real"])]
        tracks += [Track("", r, (40, 140, 40), 1) for r in reals[1:]]
        for label, rows, color in sims:
            if rows is None:
                continue
            rows = load_rows(rows) if isinstance(rows, (str, Path)) else np.asarray(rows)
            if len(rows):
                tracks.append(Track(label, place_at(rows, start), color))
        self.dash.result(test, title or test, tracks, lines)
        self.dash.current(title or test, tracks, lines)      # latest result, full size

    def close(self, hold=False):
        if hold:
            self.dash.hold("loop finished - press q or close the window to exit")
        self.dash.close()


def protect(out, session, log):
    """Back up a calibration file written by an EARLIER session before this one replaces
    it (surfaces/<surface>_loop.yaml is shared; a short demo run must not silently erase
    a validated calibration). Same-session rewrites (each iteration) are not backed up."""
    out = Path(out)
    marker = session / ".wrote_outputs"
    mine = set(marker.read_text().split()) if marker.exists() else set()
    if out.exists() and str(out) not in mine:
        backup = out.with_name(f"{out.stem}.before_{session.name}{out.suffix}")
        shutil.copy(out, backup)
        log(f"[loop] {out.name} existed (earlier session) -> backed up to {backup.name}")
    mine.add(str(out))
    marker.write_text("\n".join(sorted(mine)))


def _ws(cfg):
    ws = cfg.get("workspace")
    return (ws["x"], ws["y"]) if ws else None


class ParamPanel:
    """Keeps the window's parameter panel current: the fitted keys (start, value in use,
    best + latest candidate of this iteration, trust region, per-iteration history) and
    every other sim parameter grouped as held / measured / locked geometry."""

    def __init__(self, view, theta, keys, surface, history=()):
        self.view, self.keys, self.surface = view, keys, surface
        first = history[0]["theta_before"] if history else theta
        self.start = {k: first[k] for k in keys}
        self.hist = {k: [first[k]] + [h["theta_after"][k] for h in history] for k in keys}
        self.theta = dict(theta)
        self.best = self.cand = self.trust = None
        self.header = ""

    def push(self, header=None):
        if header is not None:
            self.header = header
        if self.view is None:
            return
        self.view.dash.params(PS.build(self.theta, self.keys, self.surface, start=self.start,
                                       best=self.best, cand=self.cand, trust=self.trust,
                                       history=self.hist, header=self.header))

    def iteration(self, it, n_it, trust):
        self.best = self.cand = None
        self.trust = trust_region(self.keys, self.theta, self.surface, trust)
        self.push(f"iteration {it}/{n_it}: search box = {trust:.0%} of each range around "
                  "the current values")

    def evaluated(self, rec, best, it):
        self.cand = {k: rec["theta"][k] for k in self.keys}
        self.best = {k: best["theta"][k] for k in self.keys}
        self.push(f"iteration {it}: eval {rec['n']} ({rec['tag']}) rms z {rec['rms_z']:.2f}, "
                  f"best {best['rms_z']:.2f}")

    def finished(self, theta_new, it, rms_b, rms_a):
        self.theta = dict(theta_new)
        for k in self.keys:
            self.hist[k].append(theta_new[k])
        self.cand = None
        self.push(f"iteration {it} done: rms z {rms_b:.2f} -> {rms_a:.2f}; values now in use")


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

    def start(self, n, tag):
        what = {"before": "current parameters (BEFORE)", "after": "best parameters (AFTER)"}
        self.view.status(f"iteration {self.it}: Isaac eval {n} - "
                         f"{what.get(tag, f'optimiser candidate (budget {self.max_evals})')}"
                         + (" | ROVER STILL NEEDED - keep it ON" if self.rover_needed else ""))

    def done(self, rec):
        if not rec.get("csvs"):
            return
        if rec["tag"] == "before":
            self.before = rec
        if self.best is None or rec["loss"] < self.best["loss"]:
            self.best = rec
        if self.params is not None:
            self.params.evaluated(rec, self.best, self.it)
        th = "  ".join(f"{k[:10]}={rec['theta'][k]:.3g}" for k in self.keys)
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
            pt = rec.get("per_test") or per_test_rms(rec.get("parts"))
            own = f"{pt[test]:.2f}" if test in pt else "-"
            self.view.test_tile(test, reps, sims,
                                [f"eval {rec['n']}: this test rms z {own}  (all tests "
                                 f"{rec['rms_z']:.2f}, best {self.best['rms_z']:.2f})", th],
                                f"{test}  iter {self.it}")


def per_test_rms(*parts):
    """{"test.feature": z} dicts (mean and/or spread terms) -> {test: rms z over its terms}.
    The pooled rms z mixes all tests; a test tile must show its own (2026-11-05: every tile
    showed the pooled 3.89, reading as if each test scored the same)."""
    acc = {}
    for d in parts:
        for k, z in (d or {}).items():
            if z is not None and math.isfinite(z):
                acc.setdefault(k.split(".", 1)[0], []).append(z * z)
    return {t: math.sqrt(sum(v) / len(v)) for t, v in acc.items()}


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
    p = cam.pose(1.0)
    frame = cam.cam.read()[0]
    g = guard.check(frame, rover_xy=list(p[1:3]) if p else None)
    if not g.get("ok"):
        raise RuntimeError(f"camera check failed: {g.get('why')}")
    if g["moved"]:
        cam.set_grid_homography(g["H_m2img"])
        log(f"[camera] moved {g['shift_px']} px since the reference -> corrected homography "
            f"applied for this session")
    else:
        log(f"[camera] unmoved ({g['shift_px']} px)")


MAX_CAMERA_GAP_S = 0.5      # the feed runs at ~30 Hz (max gap 0.18 s on 2026-11-03)


def recording_problem(t, seq_s, max_gap=MAX_CAMERA_GAP_S):
    """Camera sample times (s from sequence start) -> reason the trial is unusable, or None."""
    t = np.asarray(t, float)
    inside = t[(t >= 0.0) & (t <= seq_s)]
    if len(inside) < 10:
        return f"only {len(inside)} samples during the {seq_s:.1f} s sequence"
    edges = np.concatenate([[0.0], inside, [seq_s]])
    gap = float(np.max(np.diff(edges)))
    if gap > max_gap + 1.0:    # leading/trailing edge: allow the ~1 s camera/link latency
        return f"{gap:.2f} s without a camera fix"
    gaps = np.diff(inside)
    if len(gaps) and gaps.max() > max_gap:
        i = int(np.argmax(gaps))
        return f"{gaps[i]:.2f} s camera gap at t={inside[i]:.1f} s"
    return None


def real_trial(rover, cam, tracker, cfg, test, steps, outdir, rep, log, view):
    """Return to the test's station (camera coarse + lidar fine + gate), run the
    sequence, record. -> dict(csv, raw, video, start=gate record, ...)."""
    view.trail, view.text = [], f"{test} rep {rep}: returning to start station"
    start = ST.goto_station(rover, cam, tracker, test, cfg, log)
    view.update(cam.pose(0.5))
    n_guard = len(rover.guard_events)
    video = outdir / f"rep{rep}.mp4"
    view.trail, view.text = [], f"{test} rep {rep}: driving"
    cam.start_recording(video_path=video)
    rover.odom_log = []
    t_pre = time.time()
    while time.time() - t_pre < 0.6:              # stationary pre-roll
        view.update(cam.pose(0.5))
        time.sleep(0.02)
    lost = {"since": None}

    def guard(elapsed):
        p = cam.pose(0.5)
        view.update(p)
        if p is None:
            lost["since"] = lost["since"] or time.time()
            return "camera lost the rover" if time.time() - lost["since"] > 1.5 else None
        lost["since"] = None
        if not in_workspace(cfg, p[1], p[2]):
            return f"left the workspace at ({p[1]:.2f},{p[2]:.2f})"
        return None

    t_seq = None
    try:
        t_seq = rover.run_sequence(steps, on_tick=guard)
        t_post = time.time()
        while time.time() - t_post < 1.0:          # capture the coast-down
            view.update(cam.pose(0.5))
            time.sleep(0.02)
    finally:
        rows, frame_ts = cam.stop_recording()
        odom = rover.odom_log or []
        rover.odom_log = None
    rows = np.array(rows, float)
    if len(rows) < 10:
        raise HomingError(f"only {len(rows)} camera samples recorded")
    raw = rows.copy()
    raw[:, 0] -= t_seq
    bad = recording_problem(raw[:, 0], sum(float(s["duration"]) for s in steps))
    if bad:                                    # a feed hiccup would corrupt step features
        raise HomingError(f"camera recording unusable: {bad}")
    write_csv(outdir / f"rep{rep}_raw.csv", raw)
    # Re-time to the rover's own motion onset (removes link + camera latency), the same
    # alignment build_targets.py applies, so real features line up with the sequence -
    # guarded against the onset detector firing late on slow stick-slip spins.
    off, how, est = alignment_offset(raw, steps, cfg)
    al = raw.copy()
    al[:, 0] -= off
    al = al[al[:, 0] >= 0.0]
    write_csv(outdir / f"rep{rep}.csv", al)
    if odom:
        od = np.array([(t - t_seq, x, y, yaw) for t, x, y, yaw, _, _ in odom])
        write_csv(outdir / f"rep{rep}_wheel_odom.csv", od)
    write_csv(outdir / f"rep{rep}_frames.csv", [(t - t_seq - off,) for t in frame_ts], ("t",))
    meta = {"test": test, "rep": rep, "start": start,
            "onset_offset_s": round(float(off), 3), "alignment": how,
            "onset_estimate_s": round(float(est), 3), "n_camera": int(len(al)),
            "n_partial_total": int(cam.n_partial), "battery_v": rover.voltage,
            "obstacle_stops": rover.guard_events[n_guard:]}
    (outdir / f"rep{rep}.json").write_text(json.dumps(meta, indent=2))
    disp = math.hypot(al[-1, 1] - al[0, 1], al[-1, 2] - al[0, 2])
    dyaw = math.degrees(abs(np.unwrap(al[:, 3])[-1] - al[0, 3]))
    log(f"[real] {test} rep {rep}: {len(al)} samples, moved {disp*100:.1f} cm, "
        f"turned {dyaw:.0f} deg, onset offset {off:+.2f}s, battery {rover.voltage:.2f} V")
    return {"csv": str(outdir / f"rep{rep}.csv"), "video": str(video), "offset": off,
            "frames": str(outdir / f"rep{rep}_frames.csv"), "start": start}


ALIGN_TOL_S = 0.35
def nominal_latency(cfg):
    """Rig delay from the rover's sequence start to motion seen by the camera: camera lag
    (fusion.cam_latency_s, measured 0.71 s) + link/motor lag (timing.link_latency_s)."""
    return (float(cfg.get("fusion", {}).get("cam_latency_s", 0.71))
            + float(cfg.get("timing", {}).get("link_latency_s", 0.15)))


def alignment_offset(raw, steps, cfg):
    """-> (offset_s, 'onset'|'nominal', onset_estimate_s).

    The motion-onset estimate is used when it agrees with the rig's known latency (straights
    and arcs: 0.81-1.04 s vs nominal 0.86). Spins on carpet start slowly (stick-slip under
    the 0.12 rad/s detector threshold), so the detector fired 1-2.3 s late on R2/T2 and every
    per-step window landed in the next, opposite step (2026-11-03: wrong-sign yaw rates).
    Then the nominal latency is used instead."""
    est = BT.aligned_offset(raw[:, 0], raw[:, 1], raw[:, 2], raw[:, 3], steps)
    nom = nominal_latency(cfg)
    if abs(est - nom) <= ALIGN_TOL_S:
        return est, "onset", est
    return nom, "nominal", est


def realign_trials(real_dir, plan, cfg, log=print):
    """Rewrite <real_dir>/<test>/rep*.csv from rep*_raw.csv with alignment_offset()."""
    for test, steps in plan.items():
        for raw_p in sorted(Path(real_dir, test).glob("rep*_raw.csv")):
            rep = raw_p.name[:-len("_raw.csv")]
            raw = load_rows(raw_p)
            off, how, est = alignment_offset(raw, steps, cfg)
            al = raw.copy()
            al[:, 0] -= off
            write_csv(raw_p.with_name(f"{rep}.csv"), al[al[:, 0] >= 0.0])
            mp = raw_p.with_name(f"{rep}.json")
            if mp.exists():
                meta = json.loads(mp.read_text())
                meta.update(onset_offset_s=round(float(off), 3), alignment=how,
                            onset_estimate_s=round(float(est), 3))
                mp.write_text(json.dumps(meta, indent=2))
            log(f"[align] {test} {rep}: offset {off:.2f}s ({how}; onset estimate {est:.2f}s)")


def trial_order(plan, repeats, order="round-robin"):
    """(test, rep) pairs in recording order. round-robin (rep 1 of every test, then rep 2, ...)
    means a run cut short (battery, WiFi) still has trials of EVERY test; test-by-test recorded
    all repeats of R1 first, so the last tests had none (2026-11-04: T4 got none)."""
    if order == "test":
        return [(t, r) for t in plan for r in range(1, repeats + 1)]
    return [(t, r) for r in range(1, repeats + 1) for t in plan]


def real_phase(rover, cam, tracker, cfg, plan, repeats, itdir, log, view, retries=2,
               order="round-robin"):
    out = {t: [] for t in plan}
    bmin = float(cfg.get("battery_min_v", 0))
    for test, rep in trial_order(plan, repeats, order):
        steps = plan[test]
        tdir = itdir / "real" / test
        tdir.mkdir(parents=True, exist_ok=True)
        done_csv, done_meta = tdir / f"rep{rep}.csv", tdir / f"rep{rep}.json"
        if done_csv.exists() and done_meta.exists():      # --resume: keep finished trials
            meta = json.loads(done_meta.read_text())
            out[test].append({"csv": str(done_csv), "video": str(tdir / f"rep{rep}.mp4"),
                              "frames": str(tdir / f"rep{rep}_frames.csv"),
                              "start": meta.get("start")})
            log(f"[real] {test} rep {rep}: reusing the trial already recorded")
            continue
        for attempt in range(retries + 1):
            if bmin and rover.voltage == rover.voltage and rover.voltage < bmin:
                raise RuntimeError(f"battery {rover.voltage:.2f} V < {bmin} V - "
                                   f"charge the rover (it browns out on a low pack)")
            try:
                out[test].append(real_trial(rover, cam, tracker, cfg, test, steps, tdir,
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
                    t_wait = time.time() + 90.0
                    while time.time() < t_wait and rover.alive and rover.rx_age > 0.5:
                        time.sleep(0.5)
                    if not rover.alive or rover.rx_age > 0.5:
                        raise RuntimeError("rover link did not recover in 90 s - fix the "
                                           "WiFi, then --resume this session")
                    log(f"[real] link back (last reply {rover.rx_age:.2f}s ago) - retrying")
                rover.stop()
                log(f"[real] {test} rep {rep} attempt {attempt+1} failed: {exc}")
                if attempt == retries:
                    log(f"[real] {test} rep {rep} skipped")
    return out


def load_real_from(path, plan):
    """Reuse real CSVs: <path>/<test>/rep*.csv (an earlier iteration's real/ dir)."""
    out = {}
    for test in plan:
        reps = sorted(p for p in glob.glob(str(Path(path) / test / "rep*.csv"))
                      if Path(p).stem[3:].isdigit())
        out[test] = [{"csv": r, "video": r[:-4] + ".mp4", "frames": r[:-4] + "_frames.csv"}
                     for r in reps]
    return out


# ───────────────────────────── targets + loss ─────────────────────────────
def is_scored(feature, patterns):
    """loop_config scoring entries are feature names or glob patterns (e.g. "v_k*")."""
    import fnmatch
    return any(fnmatch.fnmatchcase(feature, p) for p in patterns)


def build_targets(real, plan, scoring, log):
    tests = {}
    for test, reps in real.items():
        if not reps:
            continue
        feats = [F.extract(*F.load_csv(r["csv"]), plan[test]) for r in reps]
        agg = F.aggregate(feats)
        keep = scoring.get(test) or []
        tests[test] = {"steps": plan[test],
                       "features": {k: {"mean": v["mean"], "sigma": v["sigma"], "n": v["n"]}
                                    for k, v in agg.items() if is_scored(k, keep)},
                       "all_features": {k: v["mean"] for k, v in agg.items()}}
        scored = ", ".join(f"{k}={v['mean']:.3f}±{v['sigma']:.3f}"
                           for k, v in tests[test]["features"].items()) or "(display only)"
        log(f"[targets] {test} (n={len(reps)}): {scored}")
    return tests


def score(sim_feats, targets):
    """sigma-weighted squared feature error (same definition as calibrate.Objective)."""
    loss, n, parts = 0.0, 0, {}
    for test, tgt in targets.items():
        got = sim_feats.get(test, {})
        for f, t in tgt["features"].items():
            n += 1
            if f in got and np.isfinite(got[f]):
                sig = math.hypot(max(t["sigma"], 1e-6), CAL._sim_noise(f) * abs(t["mean"]))
                z = (got[f] - t["mean"]) / sig
                loss += z * z
                parts[f"{test}.{f}"] = round(float(z), 2)
            else:
                loss += CAL.MISSING_PENALTY
                parts[f"{test}.{f}"] = None
    return loss, n, parts


class BatchObjective:
    """theta-vector -> loss, one Isaac launch per candidate (all scored tests at once)."""

    def __init__(self, base, keys, targets, plan, surface, combine, workroot, log):
        self.base, self.keys, self.targets = dict(base), keys, targets
        self.plan = {t: plan[t] for t in targets if targets[t]["features"]}
        self.surface, self.combine, self.workroot, self.log = surface, combine, workroot, log
        self.cache, self.n = {}, 0
        self.best = None
        self.on_start = None            # f(eval_no, tag): an Isaac launch is starting
        self.on_eval = None             # f(record): a candidate finished (for the window)
        self.evals_csv = workroot / "evals.csv"
        with open(self.evals_csv, "w") as f:
            f.write(",".join(["eval", *keys, "loss", "rms_z", "seconds"]) + "\n")

    def theta(self, x):
        t = dict(self.base)
        t.update(CAL._to_theta(self.keys, x))
        return t

    def evaluate(self, theta, plan, tag):
        self.n += 1
        if self.on_start is not None:
            self.on_start(self.n, tag)
        wd = self.workroot / f"eval_{self.n:03d}_{tag}"
        t0 = time.time()
        for attempt in range(2):
            try:
                csvs = run_battery(theta, plan, wd, self.combine)
                break
            except Exception as exc:
                self.log(f"[sim] eval {self.n} launch failed ({exc}); "
                         f"{'retrying' if attempt == 0 else 'giving up'}")
                if attempt:
                    return None, None, time.time() - t0
        feats = {t: F.extract(*F.load_csv(csvs[t]), plan[t]) for t in plan}
        return csvs, feats, time.time() - t0

    def __call__(self, x, tag="opt"):
        x = np.atleast_1d(np.asarray(x, float))
        key = tuple(np.round(x, 6))
        if key in self.cache:
            return self.cache[key]["loss"]
        theta = self.theta(x)
        try:
            C.validate(theta, self.surface)
        except ValueError:
            return PENALTY
        csvs, feats, secs = self.evaluate(theta, self.plan, tag)
        if feats is None:
            return PENALTY
        loss, n, parts = score(feats, self.targets)
        rms = math.sqrt(loss / max(n, 1))
        rec = {"x": x, "theta": theta, "loss": loss, "rms_z": rms, "csvs": csvs,
               "feats": feats, "parts": parts}
        self.cache[key] = rec
        if self.best is None or loss < self.best["loss"]:
            self.best = rec
        self.log(f"[opt] eval {self.n:2d} {tag:8s} "
                 + " ".join(f"{k}={theta[k]:.4g}" for k in self.keys)
                 + f"  loss={loss:8.2f} rms_z={rms:5.2f}  ({secs:.0f}s)"
                 + ("  *best*" if self.best is rec else ""))
        with open(self.evals_csv, "a") as f:
            f.write(",".join([str(self.n), *(f"{theta[k]:.6g}" for k in self.keys),
                              f"{loss:.4f}", f"{rms:.4f}", f"{secs:.1f}"]) + "\n")
        if self.on_eval is not None:
            try:
                self.on_eval(dict(rec, n=self.n, tag=tag))
            except Exception as exc:
                self.log(f"[viz] window update failed: {exc}")
        return loss


def trust_box(keys, theta, surface, trust):
    """Search box in optimiser coordinates: (x0, a, b). Always centred on the current
    theta, so it stays reachable; radius = trust x each key's (lin/log) range."""
    bounds = np.array(CAL._bounds(keys, surface), float)
    lo, hi = bounds[:, 0], bounds[:, 1]
    x0 = CAL._x0(keys, theta, surface)
    radius = trust * (hi - lo)
    return x0, np.maximum(lo, x0 - radius), np.minimum(hi, x0 + radius)


def trust_region(keys, theta, surface, trust):
    """Same box in parameter units -> {key: (lo, hi)} (for the window)."""
    _, a, b = trust_box(keys, theta, surface, trust)
    return {k: (CAL._to_theta([k], [ai])[k], CAL._to_theta([k], [bi])[k])
            for k, ai, bi in zip(keys, a, b)}


def optimise(obj, keys, theta, surface, max_evals, trust, log, warm=None):
    """Bounded search inside a trust region around the current theta."""
    from scipy.optimize import minimize, minimize_scalar
    x0, a, b = trust_box(keys, theta, surface, trust)
    if warm is not None:                           # analytic hint: start the search there
        x0 = np.clip(CAL._x0(keys, warm, surface), a, b)
    log(f"[opt] trust region: " + ", ".join(
        f"{k} in [{CAL._to_theta([k], [ai])[k]:.4g}, {CAL._to_theta([k], [bi])[k]:.4g}]"
        for k, ai, bi in zip(keys, a, b)))
    if len(keys) == 1:
        minimize_scalar(lambda v: obj([v]), bounds=(a[0], b[0]), method="bounded",
                        options={"maxiter": max_evals, "xatol": 0.01 * (b[0] - a[0])})
        return
    span = b - a

    def reflected(x):            # keep candidates inside the trust region (no clipping
        return a + span - np.abs((np.asarray(x) - a) % (2 * span) - span)   # collapse)

    simplex = [np.clip(x0, a, b)]
    for i in range(len(keys)):
        v = simplex[0].copy()
        v[i] = b[i] - 0.1 * span[i] if b[i] - v[i] >= v[i] - a[i] else a[i] + 0.1 * span[i]
        simplex.append(v)
    minimize(lambda x: obj(reflected(x)), simplex[0], method="Nelder-Mead",
             options={"maxfev": max_evals, "initial_simplex": np.array(simplex),
                      "xatol": 1e-3, "fatol": 0.05})


def analytic_warm(theta, keys, targets, combine, surface, log):
    """Iteration-1 hint from the reduced analytic model (only keys it represents)."""
    terms = [(t, v["steps"], v["features"], 1.0) for t, v in targets.items() if v["features"]]
    try:
        wk = CAL.analytic_sensitive_keys(theta, keys, terms, combine, surface)
        if not wk:
            return None
        obj = CAL.Objective(theta, wk, terms, "analytic", combine, surface)
        best, f = CAL.optimize_stage(obj, wk, max_evals=40, seed=0, surface=surface)
        log(f"[opt] analytic warm start ({obj.n_eval} model evals): "
            + ", ".join(f"{k}={v:.4g}" for k, v in best.items()) + f"  (model loss {f:.2f})")
        w = dict(theta)
        w.update(best)
        return w
    except Exception as exc:
        log(f"[opt] analytic warm start unavailable: {exc}")
        return None


# ───────────────────────────── visual outputs ─────────────────────────────
def to_real_frame(sim_rows, real_start):
    """Place a sim track (start-relative) at the real start pose in grid metres."""
    s = np.asarray(sim_rows, float)
    x0, y0, a0 = s[0, 1], s[0, 2], s[0, 3]
    c, sn = math.cos(-a0), math.sin(-a0)
    rx = c * (s[:, 1] - x0) - sn * (s[:, 2] - y0)
    ry = sn * (s[:, 1] - x0) + c * (s[:, 2] - y0)
    X0, Y0, A0 = real_start
    C0, S0 = math.cos(A0), math.sin(A0)
    out = s.copy()
    out[:, 1] = X0 + C0 * rx - S0 * ry
    out[:, 2] = Y0 + S0 * rx + C0 * ry
    out[:, 3] = wrap(s[:, 3] - a0 + A0)
    return out


def path_maps(test, reps, sim_before, sim_after, outdir, H_m2img, log, theta_txt="",
              labels=None):
    """paths_map.png (+ .mp4 over the real video) of real vs sim-before vs sim-after,
    all drawn on the REAL overhead camera view via the grid homography."""
    import cv2
    rep = reps[0]
    real = load_rows(rep["csv"])
    start = tuple(real[0, 1:4])
    tracks = {"real": real}
    if sim_before:
        tracks["before"] = to_real_frame(load_rows(sim_before), start)
    if sim_after:
        tracks["after"] = to_real_frame(load_rows(sim_after), start)

    def px(rows):
        return cv2.perspectiveTransform(rows[:, 1:3].reshape(-1, 1, 2).astype(float),
                                        H_m2img).reshape(-1, 2).astype(np.int32)

    def legend(img, tf=None):
        names = {"real": "real (camera)", "before": "sim BEFORE update",
                 "after": "sim AFTER update"}
        names.update(labels or {})
        for i, (k, _) in enumerate(tracks.items()):
            cv2.putText(img, names[k], (12, 30 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        (0, 0, 0), 4, cv2.LINE_AA)
            cv2.putText(img, names[k], (12, 30 + 26 * i), cv2.FONT_HERSHEY_SIMPLEX, 0.7,
                        COLORS[k], 2, cv2.LINE_AA)
        txt = f"{test}" + (f"  t={tf:4.1f}s" if tf is not None else "") + (f"   {theta_txt}" if theta_txt else "")
        cv2.putText(img, txt, (12, img.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (0, 0, 0), 4, cv2.LINE_AA)
        cv2.putText(img, txt, (12, img.shape[0] - 16), cv2.FONT_HERSHEY_SIMPLEX, 0.6,
                    (255, 255, 255), 1, cv2.LINE_AA)

    cap = cv2.VideoCapture(rep["video"]) if Path(rep["video"]).exists() else None
    first = None
    if cap is not None:
        ok, first = cap.read()
    if first is None:
        first = np.full((720, 1280, 3), 40, np.uint8)
    img = first.copy()
    for other in reps[1:]:
        o = load_rows(other["csv"])
        cv2.polylines(img, [px(o)], False, (40, 140, 40), 1, cv2.LINE_AA)
    for k, rows in tracks.items():
        cv2.polylines(img, [px(rows)], False, COLORS[k], 2, cv2.LINE_AA)
        cv2.circle(img, tuple(px(rows)[-1]), 5, COLORS[k], -1)
    legend(img)
    cv2.imwrite(str(outdir / "paths_map.png"), img)

    # progressive overlay on the recorded real video (time-aligned to the sequence)
    if cap is not None and Path(rep["frames"]).exists():
        ft = [float(r["t"]) for r in csv.DictReader(open(rep["frames"]))]
        cap.set(cv2.CAP_PROP_POS_FRAMES, 0)
        W, Hh = first.shape[1], first.shape[0]
        vw = cv2.VideoWriter(str(outdir / "paths_map.mp4"), cv2.VideoWriter_fourcc(*"mp4v"),
                             30.0, (W, Hh))
        for t in ft:
            ok, fr = cap.read()
            if not ok:
                break
            for k, rows in tracks.items():
                m = rows[:, 0] <= max(t, 0.0)
                if m.sum() >= 2:
                    cv2.polylines(fr, [px(rows[m])], False, COLORS[k], 2, cv2.LINE_AA)
            legend(fr, t)
            vw.write(fr)
        vw.release()
    if cap is not None:
        cap.release()
    log(f"[viz] {test}: {outdir/'paths_map.png'}")


def response_png(test, steps, real_reps, sim_before, sim_after, outdir):
    """Command -> response map: every real repeat vs sim BEFORE and AFTER (best effort)."""
    try:
        import response_map as RM
        real = RM.summarise(RM.step_responses(steps, [r["csv"] for r in real_reps]))
        out = {}
        for tag, sim in (("before", sim_before), ("after", sim_after)):
            if sim:
                rows = RM.merge(real, RM.summarise(RM.step_responses(steps, [sim])))
                RM.plot(rows, str(outdir / f"response_map_{tag}.png"), f"{test}: command -> response ({tag})")
                out[tag] = RM.gap(rows)
        (outdir / "response_map.json").write_text(json.dumps(out, indent=1))
        return out
    except Exception:
        return {}


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


def compare_png(real_csv, sim_csv, out_png, out_json, label):
    subprocess.run([sys.executable, str(SC / "trials" / "compare_paths.py"), str(real_csv),
                    str(sim_csv), "--out", str(out_png), "--json", str(out_json),
                    "--label", label], check=False, stdout=subprocess.DEVNULL,
                   stderr=subprocess.DEVNULL)
    try:
        return json.loads(Path(out_json).read_text())
    except Exception:
        return None


def progress_plot(history, keys, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    its = [h["iteration"] for h in history]
    fig, axes = plt.subplots(1, 1 + len(keys), figsize=(5 * (1 + len(keys)), 4))
    axes = np.atleast_1d(axes)
    axes[0].plot(its, [h["rms_before"] for h in history], "o-", color="#d62728",
                 label="sim BEFORE update (fresh real data)")
    axes[0].plot(its, [h["rms_after"] for h in history], "s-", color="#1f77b4",
                 label="sim AFTER update")
    axes[0].axhline(1.0, color="k", ls=":", lw=1, label="camera noise floor (1 sigma)")
    axes[0].set_xlabel("iteration"); axes[0].set_ylabel("rms feature error [sigma]")
    axes[0].set_title("sim vs real mismatch"); axes[0].legend(fontsize=8)
    axes[0].grid(alpha=.3)
    for ax, k in zip(axes[1:], keys):
        ax.plot([0] + its, [history[0]["theta_before"][k]] + [h["theta_after"][k] for h in history],
                "o-", color="#2ca02c")
        ax.set_title(k); ax.set_xlabel("iteration"); ax.grid(alpha=.3)
    fig.tight_layout()
    fig.savefig(path, dpi=110)
    plt.close(fig)


def write_summary(session, args, keys, history, final_theta, surface_out):
    L = [f"# Closed sim2real loop — {args.surface}", "",
         f"session `{session.name}`  ·  tests {', '.join(args._tests)}  ·  "
         f"{args.repeats} real repeats/test  ·  keys {', '.join(keys)}", "",
         "| iter | rms z before | rms z after | " + " | ".join(keys) + " | real trials |",
         "|---|---|---|" + "---|" * len(keys) + "---|"]
    for h in history:
        L.append(f"| {h['iteration']} | {h['rms_before']:.2f} | {h['rms_after']:.2f} | "
                 + " | ".join(f"{h['theta_after'][k]:.4g}" for k in keys)
                 + f" | {h['n_real']} |")
    L += ["", "rms z = root-mean-square feature error in units of the real rover's "
          "repeat-to-repeat sigma (≈1 means the sim is inside the real rover's own scatter). "
          "'before' scores the parameters carried INTO the iteration against that "
          "iteration's fresh real repeats (an out-of-sample check of the previous update).",
          "", "## Per-test path error (start-aligned, real rep 1)", "",
          "| iter | test | endpoint before | endpoint after | heading before | heading after |",
          "|---|---|---|---|---|---|"]
    for h in history:
        for t, m in h["paths"].items():
            b, a = m.get("before") or {}, m.get("after") or {}
            L.append(f"| {h['iteration']} | {t} | {b.get('endpoint_err_m', float('nan'))*100:.1f} cm"
                     f" | {a.get('endpoint_err_m', float('nan'))*100:.1f} cm"
                     f" | {b.get('heading_err_deg', float('nan')):.1f}° | "
                     f"{a.get('heading_err_deg', float('nan')):.1f}° |")
    L += ["", "## Start-pose repeatability (all real trials)", "",
          "| test | trials | via lidar | lidar err mean / max | camera spread mean / max | "
          "camera yaw sd |", "|---|---|---|---|---|---|"]
    for t in args._tests:
        recs = [r for h in history for r in h.get("starts", {}).get(t, []) if r]
        if not recs:
            continue
        cp = np.array([r["camera_pose"] for r in recs])
        spread = np.hypot(cp[:, 0] - cp[:, 0].mean(), cp[:, 1] - cp[:, 1].mean())
        lid = [r["lidar_err_m"] for r in recs if "lidar_err_m" in r]
        L.append(f"| {t} | {len(recs)} | {sum(r.get('method') == 'lidar' for r in recs)} | "
                 + (f"{np.mean(lid)*100:.1f} / {np.max(lid)*100:.1f} cm" if lid else "n/a")
                 + f" | {spread.mean()*100:.1f} / {spread.max()*100:.1f} cm | "
                 f"{math.degrees(np.std(np.unwrap(cp[:, 2]))):.1f}° |")
    L += ["", f"Final parameters → `{surface_out}`", "", "```yaml",
          *(f"{k}: {final_theta[k]:.6g}" for k in keys), "```", "",
          "Per iteration: `iter_NN/<test>/paths_map.{png,mp4}` (real vs sim before/after on "
          "the real camera view), `compare_{before,after}.png`, `real/`, `opt/evals.csv`."]
    (session / "summary.md").write_text("\n".join(L) + "\n")


# ───────────────────────────── main loop ─────────────────────────────
def park_rover(rover, tracker, cfg, log, view):
    view.status("parking the rover")
    p = cfg.get("park")
    if p:
        try:
            go_to_pose(rover, tracker, (p["x"], p["y"], math.radians(p["yaw"])), cfg, log=log)
        except Exception as exc:
            log(f"[real] park failed: {exc}")


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--surface", required=True,
                    help="floor material (concrete|carpet|foam or alias): sets the floor-mu "
                         "band and names the outputs")
    ap.add_argument("--tests", default="T1,T2,T4", help="comma list of test stems / prefixes")
    ap.add_argument("--repeats", type=int, default=3, help="real repeats per test per iteration")
    ap.add_argument("--iterations", type=int, default=3)
    ap.add_argument("--keys", default=None,
                    help="comma list of sim parameters to update (default: loop_config.yaml keys)")
    ap.add_argument("--max-evals", type=int, default=12, help="Isaac evaluations per iteration")
    ap.add_argument("--trust", type=float, default=0.5,
                    help="iteration-1 search radius as a fraction of each key's range; "
                         "shrinks x0.6 per iteration")
    ap.add_argument("--converge", type=float, default=1.2,
                    help="stop when 'before' rms z (fresh real data) is at or below this")
    ap.add_argument("--init", default=None,
                    help="start parameters: a surfaces/*.yaml (its 'calibrated') or a nested "
                         "sim_calibration yaml; default = schema start + surface floor band")
    ap.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    ap.add_argument("--config", default=str(CONFIG))
    ap.add_argument("--real-from", default=None,
                    help="reuse real CSVs (<dir>/<test>/rep*.csv) instead of driving the rover")
    ap.add_argument("--real-only", action="store_true", help="drive + record, skip the sim")
    ap.add_argument("--warm", action="store_true",
                    help="iteration 1: start the search at the reduced analytic model's optimum "
                         "(off by default - that model omits contact/torque effects)")
    ap.add_argument("--no-videos", action="store_true")
    ap.add_argument("--show", dest="show", action="store_true", default=True,
                    help="persistent session window (default when $DISPLAY is set)")
    ap.add_argument("--no-show", dest="show", action="store_false",
                    help="no window (session live.jpg is still written)")
    ap.add_argument("--no-hold", action="store_true",
                    help="exit at the end instead of keeping the window up")
    ap.add_argument("--allow-approximate", action="store_true")
    ap.add_argument("--keep-bringup", action="store_true",
                    help="leave a rover base driver we started running at the end")
    ap.add_argument("--test-fake-rover", action="store_true", help=argparse.SUPPRESS)
    ap.add_argument("--resume", default=None, metavar="SESSION_DIR",
                    help="continue an interrupted session (e.g. after a rover brown-out): "
                         "recorded real trials and completed iterations are kept")
    ap.add_argument("--trial-order", choices=("round-robin", "test"), default="round-robin",
                    help="round-robin (default): rep 1 of every test, then rep 2, ... so a run cut "
                         "short still covers every test; test: all repeats of one test first")
    ap.add_argument("--realign", metavar="REAL_DIR", default=None,
                    help="re-time recorded trials (<dir>/<test>/rep*_raw.csv -> rep*.csv) "
                         "with the guarded alignment, then exit")
    ap.add_argument("--out", default=None, help="final surface yaml "
                    "(default surfaces/<surface>_loop.yaml)")
    args = ap.parse_args()

    cfg = load_config(args.config)
    keys = [k.strip() for k in (args.keys or ",".join(cfg["optimise"]["keys"])).split(",")
            if k.strip()]
    for k in keys:
        if k not in C.OPTIMISABLE:
            sys.exit(f"'{k}' is not optimisable (geometry/masses are locked). "
                     f"Choose from {C.OPTIMISABLE}")
    tests = [resolve_test(t.strip()) for t in args.tests.split(",") if t.strip()]
    plan = {stem: F.load_sequence(str(path)) for stem, path in tests}
    args._tests = list(plan)
    scoring = {t: cfg["scoring"].get(t, []) for t in plan}

    stamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
    prefix = "TESTFAKE_" if args.test_fake_rover else ""     # never mistaken for real data
    session = Path(args.resume) if args.resume else HERE / "runs" / f"{prefix}{args.surface}_{stamp}"
    session.mkdir(parents=True, exist_ok=True)
    log = Log(session / "session.log")
    shutil.copy(args.config, session / "loop_config.yaml")
    (session / "args.json").write_text(json.dumps({k: v for k, v in vars(args).items()},
                                                  indent=2, default=str))

    measured = CAL.load_measured(str(SC / "optimize" / "measured.yaml"))
    seed = None
    if args.init:
        import yaml
        doc = yaml.safe_load(Path(args.init).read_text()) or {}
        seed = doc.get("calibrated") or C.flatten(doc.get("sim_calibration_yaml", doc))
    theta = CAL.start_theta(measured, args.surface, seed)
    log(f"[loop] session {session}")
    log(f"[loop] tests {list(plan)}  repeats {args.repeats}  iterations {args.iterations}  "
        f"keys {keys}")
    log(f"[loop] scored features: " + "; ".join(f"{t}: {scoring[t] or 'display only'}"
                                                 for t in plan))
    log(f"[loop] start theta: " + ", ".join(f"{k}={theta[k]:.4g}" for k in keys))

    rover = cam = view = tracker = None
    history = []
    first_it = 1
    if args.resume:                      # completed iterations carry their theta forward
        import yaml
        for itd in sorted(session.glob("iter_*")):
            rec_p, th_p = itd / "iteration.json", itd / "theta.yaml"
            if rec_p.exists() and th_p.exists():
                history.append(json.loads(rec_p.read_text()))
                theta.update({k: v for k, v in C.flatten(yaml.safe_load(th_p.read_text())).items()
                              if k in theta})
                first_it = history[-1]["iteration"] + 1
        log(f"[loop] RESUMING {session.name}: {len(history)} completed iteration(s); "
            f"continuing at iteration {first_it} with "
            + ", ".join(f"{k}={theta[k]:.4g}" for k in keys))
    if args.realign:
        realign_trials(args.realign, plan, cfg, log)
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
                           **{f"sequence {t}": p for t, p in tests}})
    _vid = session_video_path(session)         # what LiveView will record to
    index.add_video("whole session (dashboard recording)" if _vid.name == "session_video.mp4"
                    else f"resumed session ({_vid.stem.split('_')[-1]})", _vid)
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
            saved_env = {k: os.environ.get(k) for k in ("ROS_DOMAIN_ID", "ROS_LOCALHOST_ONLY")}
            os.environ.update(ROS_DOMAIN_ID="90", ROS_LOCALHOST_ONLY="1")
            import test_stations_sim as TS
            fake = sp.Popen([sys.executable, str(HERE / "tests" / "fake_rover.py"),
                             "0.5", "0.5", "1.5708"], start_new_session=True)
            import atexit
            atexit.register(lambda: os.killpg(fake.pid, 15))
            time.sleep(2.0)
            cam = TS.FakeCamera()
            from rover_link import RoverLink
            rover = RoverLink(host="local", log=log, fusion_cfg=cfg.get("fusion", {}))
            rover.open()
            ST.set_guard(rover, cfg)
            from localizer import FusedTracker
            tracker = FusedTracker(cam, rover, cam_latency_s=0.15)
            # Isaac must NOT share the fake rover's domain (its /cmd_vel would drive it)
            for k, v in saved_env.items():
                if v is None:
                    os.environ.pop(k, None)
                else:
                    os.environ[k] = v
            from metric_overlay import load_calib
            _, H_m2img, dims = load_calib(str(SC / "calib" / "grid_calib.json"))
            view = LiveView(cam, session, args.show, dims, _ws(cfg))
            log.sink = view.dash.log
            ST.ST_DIR = session / "stations"          # isolated stations for the test
            cfg["stations"]["lidar_offset"] = [0.0, 0.0, 0.0]   # fake rover conventions
            cfg["stations"]["lidar_flip"] = False
            for t in plan:
                ST.teach(rover, cam, tracker, t, cfg, log)
        elif args.real_from is None:
            from rover_link import RoverLink
            cam = CameraTrack(allow_approximate=args.allow_approximate)
            cam.latency_s = cfg.get("fusion", {}).get("cam_latency_s", 0.0)
            camera_check(cam, log)
            H_m2img = cam.H_m2img
            view = LiveView(cam, session, args.show, cam.grid_m, _ws(cfg))
            log.sink = view.dash.log
            view.status("connecting to the rover")
            rover = RoverLink(log=log, logfile=str(session / "rover_agent.log"),
                              fusion_cfg=cfg.get("fusion", {}))
            rover.open()
            ST.set_guard(rover, cfg)
            fz = cfg.get("fusion", {})
            if fz.get("enabled", True):
                from localizer import FusedTracker
                tracker = FusedTracker(cam, rover, cam_latency_s=fz.get("cam_latency_s", 0.2),
                                       gain=fz.get("gain", 0.3))
            else:
                tracker = cam
            taught = ST.list_stations()
            log(f"[loop] stations taught: {taught or 'none'}"
                + ("" if all(t in taught for t in plan) else
                   f"  (untaught -> camera-only config pose: {[t for t in plan if t not in taught]};"
                   f" teach with loop/stations.py teach <test>)"))
        else:
            from metric_overlay import load_calib
            _, H_m2img, dims = load_calib(str(SC / "calib" / "grid_calib.json"))
            view = LiveView(None, session, args.show, dims, _ws(cfg))
            log.sink = view.dash.log

        params = ParamPanel(view, theta, keys, args.surface, history)
        params.push(f"start values ({'resumed' if history else args.init or 'schema start'}"
                    f", surface {args.surface})")
        for it in range(first_it, args.iterations + 1):
            itdir = session / f"iter_{it:02d}"
            itdir.mkdir(exist_ok=True)
            log("=" * 78)
            log(f"[loop] ITERATION {it}/{args.iterations}")
            view.status(f"iteration {it}/{args.iterations}: real trials")

            # 1. real
            if args.real_from:
                real = load_real_from(args.real_from, plan)
                log(f"[real] reusing {sum(len(v) for v in real.values())} real runs from "
                    f"{args.real_from}")
            else:
                if it > 1:
                    camera_check(cam, log)         # bumped mid-session?
                real = real_phase(rover, cam, tracker, cfg, plan, args.repeats, itdir, log,
                                  view, order=args.trial_order)
                if it == args.iterations and rover is not None:
                    # nothing after this needs the rover: park, stop the base driver and say so
                    park_rover(rover, tracker, cfg, log, view)
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
            if not any(real.values()):
                raise RuntimeError("no successful real runs this iteration")
            for test in plan:                      # tiles show the fresh real data at once
                if real.get(test):
                    view.test_tile(test, real[test], [],
                                   [f"iter {it}: {len(real[test])} real repeats"
                                    + ("" if args.real_only else ", sim pending")],
                                   f"{test}  iter {it}")
            if args.real_only:
                build_targets(real, plan, scoring, log)
                continue

            # 2. targets
            targets = build_targets(real, plan, scoring, log)
            (itdir / "targets.json").write_text(json.dumps(
                {t: {"features": v["features"], "all_features": v["all_features"]}
                 for t, v in targets.items()}, indent=2))
            if not any(v["features"] for v in targets.values()):
                raise RuntimeError("no scored features - check loop_config.yaml scoring")

            # 3. sim BEFORE (full battery incl. display-only tests), scored on fresh data
            optdir = itdir / "opt"
            optdir.mkdir(exist_ok=True)
            obj = BatchObjective(theta, keys, targets, plan, args.surface, args.combine_mode,
                                 optdir, log)
            params.iteration(it, args.iterations, max(args.trust * 0.6 ** (it - 1), 0.08))
            hook = SimTiles(view, real, keys, it, args, params)
            obj.on_start, obj.on_eval = hook.start, hook.done
            x_cur = CAL._x0(keys, theta, args.surface)
            log(f"[sim] BEFORE: current parameters on the full battery")
            csv_b, feats_b, _ = obj.evaluate(theta, {t: plan[t] for t in real if real[t]},
                                             "before")
            if feats_b is None:
                raise RuntimeError("baseline Isaac run failed - see the eval dir isaac.log")
            loss_b, n_b, parts_b = score(feats_b, targets)
            rms_b = math.sqrt(loss_b / max(n_b, 1))
            hook.done({"n": obj.n, "tag": "before", "theta": dict(theta), "loss": loss_b,
                       "rms_z": rms_b, "csvs": csv_b,
                       "per_test": per_test_rms(parts_b)})
            obj.best = {"x": x_cur, "theta": dict(theta), "loss": loss_b, "rms_z": rms_b,
                        "csvs": csv_b, "feats": feats_b, "parts": parts_b}
            obj.cache[tuple(np.round(x_cur, 6))] = obj.best
            log(f"[sim] BEFORE loss={loss_b:.2f} rms_z={rms_b:.2f}  per-feature z: {parts_b}")

            # 4./5. update
            converged = rms_b <= args.converge
            if converged:
                log(f"[loop] converged: rms_z {rms_b:.2f} <= {args.converge} on fresh real "
                    f"data - no update needed")
            else:
                warm = None if (not args.warm or it > 1) else analytic_warm(
                    theta, keys, targets, args.combine_mode, args.surface, log)
                trust = max(args.trust * 0.6 ** (it - 1), 0.08)
                optimise(obj, keys, theta, args.surface, args.max_evals, trust, log, warm)
            best = obj.best
            theta_new = dict(best["theta"])

            # 6. verify AFTER on the full battery (display-only tests need a run too)
            if best["x"] is x_cur or np.allclose(best["x"], x_cur):
                csv_a, feats_a, rms_a, loss_a = csv_b, feats_b, rms_b, loss_b
                log("[loop] no candidate beat the current parameters this iteration")
            elif set(best["csvs"]) >= {t for t in plan if real.get(t)}:
                csv_a, feats_a, rms_a, loss_a = best["csvs"], best["feats"], best["rms_z"], best["loss"]
            else:
                log("[sim] AFTER: best parameters on the full battery")
                csv_a, feats_a, _ = obj.evaluate(theta_new, {t: plan[t] for t in real if real[t]},
                                                 "after")
                loss_a, n_a, _ = score(feats_a, targets) if feats_a else (loss_b, n_b, None)
                rms_a = math.sqrt(loss_a / max(n_b, 1))
            log(f"[loop] iteration {it}: rms_z {rms_b:.2f} -> {rms_a:.2f}   "
                + ", ".join(f"{k}: {theta[k]:.4g} -> {theta_new[k]:.4g}" for k in keys))

            # visual + per-test outputs
            view.status(f"iteration {it}: comparing + drawing path maps")
            paths = {}
            for test in plan:
                if not real.get(test):
                    continue
                tdir = itdir / test
                tdir.mkdir(exist_ok=True)
                sb, sa = (csv_b or {}).get(test), (csv_a or {}).get(test)
                rc = real[test][0]["csv"]
                paths[test] = {
                    "before": compare_png(rc, sb, tdir / "compare_before.png",
                                          tdir / "metrics_before.json", f"{test} before") if sb else None,
                    "after": compare_png(rc, sa, tdir / "compare_after.png",
                                         tdir / "metrics_after.json", f"{test} after") if sa else None}
                pb, pa = paths[test]["before"] or {}, paths[test]["after"] or {}
                view.test_tile(test, real[test],
                               [("sim BEFORE", sb, COLORS["before"]),
                                ("sim AFTER", sa, COLORS["after"])],
                               [f"rms z {rms_b:.2f} -> {rms_a:.2f}",
                                f"endpoint {pb.get('endpoint_err_m', float('nan'))*100:.1f} -> "
                                f"{pa.get('endpoint_err_m', float('nan'))*100:.1f} cm",
                                "  ".join(f"{k[:10]}={theta_new[k]:.3g}" for k in keys)],
                               f"{test}  iter {it} (done)")
                try:
                    ttxt = "  ".join(f"{k}={theta_new[k]:.3g}" for k in keys)
                    if not args.no_videos:
                        path_maps(test, real[test], sb, sa, tdir, H_m2img, log, ttxt)
                except Exception as exc:
                    log(f"[viz] {test}: path map failed: {exc}")
                gaps = response_png(test, plan[test], real[test], sb, sa, tdir)
                real_dir = Path(real[test][0]["csv"]).parent
                index.add_test(f"iteration {it} - {test}", metrics={
                    "real repeats": len(real[test]),
                    "endpoint before [m]": pb.get("endpoint_err_m"),
                    "endpoint after [m]": pa.get("endpoint_err_m"),
                    "heading before [deg]": pb.get("heading_err_deg"),
                    "heading after [deg]": pa.get("heading_err_deg"),
                    "response gap before (rms z)": (gaps.get("before") or {}).get("rms_z"),
                    "response gap after (rms z)": (gaps.get("after") or {}).get("rms_z")},
                    files=iteration_files(tdir, real_dir))
            rec = {"iteration": it, "rms_before": rms_b, "rms_after": rms_a,
                   "loss_before": loss_b, "loss_after": loss_a,
                   "theta_before": theta_view(theta, keys), "theta_after": theta_view(theta_new, keys),
                   "n_real": sum(len(v) for v in real.values()), "paths": paths,
                   "z_before": parts_b, "z_after": best.get("parts"),
                   "evals": obj.n, "converged": converged,
                   "starts": {t: [r.get("start") for r in real[t] if r.get("start")]
                              for t in real}}
            (itdir / "iteration.json").write_text(json.dumps(rec, indent=2, default=float))
            import yaml
            (itdir / "theta.yaml").write_text(yaml.safe_dump(
                C.nest({k: v for k, v in theta_new.items() if k not in C.LOCKED}), sort_keys=False))
            history.append(rec)
            theta = theta_new
            params.finished(theta_new, it, rms_b, rms_a)
            progress_plot(history, keys, session / "progress.png")
            # fake-rover tests must never replace a real surface/model file
            surf_dir = session if args.test_fake_rover else SC / "surfaces"
            out = Path(args.out) if args.out else surf_dir / f"{args.surface}_loop.yaml"
            protect(out, session, log)
            CAL.write_surface(str(out), args.surface, theta,
                              {f"iteration_{h['iteration']}": {**h["theta_after"],
                                                                "rms_z": h["rms_after"]}
                               for h in history}, args.combine_mode)
            write_summary(session, args, keys, history, theta, out)
            log(f"[loop] wrote {out} and {session/'summary.md'}")
            index.set_summary([f"iteration {h['iteration']}: rms z {h['rms_before']:.2f} -> "
                               f"{h['rms_after']:.2f}; " + ", ".join(
                                   f"{k}={h['theta_after'][k]:.4g}" for k in keys) for h in history])
            index.snapshot_config({"result (written by this session)": out})
            index.add_test("summary", files={"summary.md": session / "summary.md",
                                             "session log": session / "session.log",
                                             f"iteration {it} theta.yaml": itdir / "theta.yaml",
                                             "optimiser evals csv": itdir / "opt" / "evals.csv"})
            index.write()
            view.status(f"iteration {it} done: rms z {rms_b:.2f} -> {rms_a:.2f}")
            if converged:
                break

        # park the rover at the end
        if rover is not None:
            park_rover(rover, tracker, cfg, log, view)
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
