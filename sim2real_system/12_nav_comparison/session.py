#!/usr/bin/env python3
"""navcmp session: the real rover and Isaac rovers drive the SAME course with the SAME
geometric nav stack (lidar, or lidar + depth cloud; no visual SLAM), then compare.

Its only job is the comparison: it does not tune anything. It tests the OPTIMIZER'S
RESULTS at the navigation level - does the sim rover, driving its wheels through PhysX
with the fitted physics parameters, behave under Nav2 like the real rover does, and better
than the un-calibrated sim?

    cd ~/projects/isaac-assets && source ros2env.sh && cd sim2real_calib
    # no rover needed - full pipeline with an Isaac stand-in for the real rover:
    python3 navcmp/session.py --course navcmp/courses/grid2x2_box.yaml --stack lidar \\
        --trials 3 --conditions real,sim_physics,sim_default --fake-real
    # with the real rover (arena built, rover on the start mark, nothing else running on it):
    python3 navcmp/session.py --course navcmp/courses/grid2x2_box.yaml --stack lidar \\
        --trials 5 --conditions real,sim_physics,sim_default
    python3 navcmp/session.py --resume navcmp/runs/<session>        # finish an interrupted one
    python3 navcmp/report.py navcmp/runs/<session>                  # (re)build the report

Every condition runs through ONE code path: stage the bundle (stack launch + shared Nav2
params + runner + course) onto its host -> [Isaac for sim] -> start the stack -> run
mission_runner.py -> fetch its data -> tear down. The real rover's host is SSH (the
stack and runner execute ON the Jetson, local ROS only); sim hosts are this desktop.
"""
from __future__ import annotations

import argparse
import csv
import hashlib
import json
import math
import os
import shlex
import shutil
import signal
import subprocess
import sys
import threading
import time
from pathlib import Path

import yaml

HERE = Path(__file__).resolve().parent
CALIB = HERE.parent
REPO = CALIB.parent
sys.path.insert(0, str(HERE))
import course as C  # noqa: E402
import nav_params  # noqa: E402

ISAACPY = os.environ.get("ISAACPY", str(REPO / "isaacpy_clean.sh"))
RUN_COURSE = REPO / "ugv_controller_v2" / "run_nav_course.py"
ROVER = os.environ.get("ROVER_HOST", "rover-lan")
SSHOPTS = ["-o", "BatchMode=yes", "-o", "StrictHostKeyChecking=accept-new",
           "-o", "ConnectTimeout=10", "-o", "ServerAliveInterval=5", "-o", "ServerAliveCountMax=3"]
REAL_ENV = "source ~/rover_nav/local_ros_env.sh >/dev/null 2>&1"
LOCAL_ENV = ("source /opt/ros/humble/setup.bash && "
             f"source {shlex.quote(str(REPO / 'ugv_ws' / 'install' / 'setup.bash'))} && "
             "export ROS_DOMAIN_ID=${ROS_DOMAIN_ID:-0} RMW_IMPLEMENTATION=rmw_fastrtps_cpp "
             "ROS_LOCALHOST_ONLY=0 && unset ROS_DISCOVERY_SERVER")
REAL_PACKAGES = ("nav2_bringup", "slam_toolbox", "teb_local_planner", "ugv_bringup",
                 "depth_image_proc")
BATTERY_MIN_V = 10.4        # pack browns the motors out at ~10.17 V (2026-11-03)


class SessionError(RuntimeError):
    pass


# ── hosts ────────────────────────────────────────────────────────────────────
class LocalHost:
    """This desktop. Runs the sim conditions (and the fake real rover)."""
    name = "local"
    env = LOCAL_ENV

    def __init__(self, root: Path):
        self.root = Path(root)

    def path(self, rel):
        return str(self.root / rel)

    def run(self, cmd, timeout=60):
        return subprocess.run(["bash", "-c", cmd], capture_output=True, text=True, timeout=timeout)

    def popen(self, cmd, log, stdin=subprocess.DEVNULL, tty=False):
        return subprocess.Popen(["bash", "-c", cmd], stdin=stdin, stdout=log,
                                stderr=subprocess.STDOUT if log is not subprocess.PIPE else subprocess.PIPE,
                                text=log is subprocess.PIPE, bufsize=1 if log is subprocess.PIPE else -1,
                                start_new_session=True)

    def put_dir(self, src, rel):
        dst = self.root / rel
        shutil.rmtree(dst, ignore_errors=True)
        shutil.copytree(src, dst)

    def get_dir(self, rel, dst):
        src = self.root / rel
        if not src.exists():
            return False
        shutil.copytree(src, dst, dirs_exist_ok=True)
        return True

    def stop(self, proc, grace=10):
        _stop_group(proc, grace)


class SSHHost:
    """The real rover. The stack and runner execute ON the Jetson; only files and the
    runner's JSON events cross the network."""
    env = REAL_ENV

    def __init__(self, host, root="navcmp_run"):
        self.name = host
        self.root = root      # relative: ssh commands and scp both start in the rover's $HOME

    def path(self, rel):
        return f"{self.root}/{rel}"

    def run(self, cmd, timeout=60):
        return subprocess.run(["ssh", *SSHOPTS, self.name, cmd], capture_output=True, text=True,
                              timeout=timeout)

    def popen(self, cmd, log, stdin=subprocess.PIPE, tty=False):
        # -tt: a launch held open by its ssh (a detached launch dies to session cleanup on
        # this rover); ^C over the tty stops it cleanly. The runner is NOT on a tty: its
        # stdin is the link watchdog (EOF = stop the rover).
        args = ["ssh", *(["-tt"] if tty else []), *SSHOPTS, self.name, cmd]
        return subprocess.Popen(args, stdin=stdin, stdout=log,
                                stderr=subprocess.STDOUT if log is not subprocess.PIPE else subprocess.PIPE,
                                text=log is subprocess.PIPE, bufsize=1 if log is subprocess.PIPE else -1,
                                start_new_session=True)

    def put_dir(self, src, rel):
        dst = self.path(rel)
        r = self.run(f"rm -rf {shlex.quote(dst)} && mkdir -p {shlex.quote(os.path.dirname(dst) or '.')}")
        if r.returncode:
            raise SessionError(f"cannot prepare {dst} on {self.name}: {r.stderr.strip()}")
        subprocess.run(["scp", *SSHOPTS, "-q", "-r", str(src), f"{self.name}:{dst}"],
                       check=True, timeout=120)

    def get_dir(self, rel, dst):
        Path(dst).mkdir(parents=True, exist_ok=True)
        r = subprocess.run(["scp", *SSHOPTS, "-q", "-r", f"{self.name}:{self.path(rel)}/.", str(dst)],
                           capture_output=True, text=True, timeout=300)
        return r.returncode == 0

    def stop(self, proc, grace=10):
        if proc is None or proc.poll() is not None:
            return
        try:                                   # ^C through the held-open tty
            proc.stdin.write(b"\x03")
            proc.stdin.flush()
        except Exception:
            pass
        try:
            proc.wait(timeout=grace)
        except subprocess.TimeoutExpired:
            _stop_group(proc, 3)


def _stop_group(proc, grace):
    if proc is None or proc.poll() is not None:
        return
    for sig, wait in ((signal.SIGINT, grace), (signal.SIGTERM, 5), (signal.SIGKILL, 5)):
        try:
            os.killpg(proc.pid, sig)
        except ProcessLookupError:
            return
        try:
            proc.wait(timeout=wait)
            return
        except subprocess.TimeoutExpired:
            continue


def clean_local_ros():
    """Leftover DDS state from killed processes silently breaks the next bringup (stale
    /dev/shm/fastrtps_* -> TF never reaches Nav2). `fastdds` has no shebang: run via sh."""
    subprocess.run(["bash", "-c", "source /opt/ros/humble/setup.bash && ros2 daemon stop"],
                   capture_output=True, timeout=30)
    exe = shutil.which("fastdds") or "/opt/ros/humble/bin/fastdds"
    try:
        subprocess.run(["sh", exe, "shm", "clean"], capture_output=True, timeout=15)
    except (OSError, subprocess.TimeoutExpired):
        pass


def kill_local_leftovers():
    for pat in ("navcmp_stack.launch.py", "run_nav_course.py", "navcmp/mission_runner.py",
                "_stage/.*/mission_runner.py"):
        subprocess.run(["pkill", "-INT", "-f", pat], capture_output=True)
    time.sleep(2)
    for pat in ("navcmp_stack.launch.py", "run_nav_course.py", "mission_runner.py --course"):
        subprocess.run(["pkill", "-KILL", "-f", pat], capture_output=True)


# ── session ──────────────────────────────────────────────────────────────────
class Session:
    def __init__(self, args):
        self.args = args
        self.fake = bool(args.fake_real)
        if args.resume:
            self.dir = Path(args.resume).resolve()
            cfg = json.loads((self.dir / "session.json").read_text())
            self.cfg = cfg
        else:
            course = C.load(args.course)
            stamp = time.strftime("%Y%m%d_%H%M%S")
            name = f"{'TESTFAKE_' if self.fake else ''}{course.name}_{args.stack}_{stamp}"
            self.dir = (Path(args.out_root) / name).resolve()
            self.dir.mkdir(parents=True)
            all_conds = yaml.safe_load((HERE / "conditions.yaml").read_text())
            if args.conditions_file:
                all_conds.update(yaml.safe_load(Path(args.conditions_file).read_text()) or {})
            names = [c.strip() for c in args.conditions.split(",") if c.strip()]
            conds = {}
            for n in names:
                key = "fake_real" if (n == "real" and self.fake) else n
                if key not in all_conds:
                    raise SessionError(f"unknown condition {n!r} (see navcmp/conditions.yaml)")
                conds[n] = dict(all_conds[key], fake=(key == "fake_real"))
            overrides = yaml.safe_load(Path(args.nav_overrides).read_text()) if args.nav_overrides else {}
            self.cfg = {"course": str(Path(args.course).resolve()), "course_name": course.name,
                        "stack": args.stack, "trials": args.trials, "conditions": conds,
                        "order": names, "fake_real": self.fake, "nav_overrides": overrides,
                        "created": time.strftime("%Y-%m-%d %H:%M:%S"),
                        "git": _git_rev(), "rover_host": ROVER,
                        "runner_args": {"on_fail": args.on_fail, "leg_pause": args.leg_pause,
                                        "sample_hz": 10.0},
                        "world_from_real_map": bool(args.world_from_real_map),
                        "lidar_profile": args.lidar_profile}
            self._build_bundle()
            (self.dir / "session.json").write_text(json.dumps(self.cfg, indent=1))
        self.logf = open(self.dir / "session.log", "a")
        self.course = C.load(self.dir / "bundle" / "course.yaml")

    def log(self, msg):
        line = f"{time.strftime('%H:%M:%S')} {msg}"
        print(line, flush=True)
        self.logf.write(line + "\n")
        self.logf.flush()

    def _build_bundle(self):
        b = self.dir / "bundle"
        b.mkdir()
        hashes = nav_params.write_stack(b, self.cfg["stack"], self.cfg["nav_overrides"])
        for f in ("mission_runner.py", "course.py"):
            shutil.copyfile(HERE / f, b / f)
        shutil.copyfile(self.cfg["course"], b / "course.yaml")
        for f in ("mission_runner.py", "course.py", "course.yaml"):
            hashes[f] = hashlib.sha256((b / f).read_bytes()).hexdigest()
        self.cfg["bundle_sha256"] = hashes

    # ── one condition ────────────────────────────────────────────────────────
    def condition_done(self, name):
        return self._trial_stats(self.dir / name)[0]

    def _trial_stats(self, cdir):
        """(complete trials, highest trial number used) from the merged legs.csv."""
        legs = cdir / "data" / "legs.csv"
        if not legs.exists():
            return 0, 0
        with legs.open() as f:
            rows = list(csv.DictReader(f))
        per = {}
        for r in rows:
            per.setdefault(int(r["trial"]), [])
            if r["result"] != "STOPPED":         # an interrupted leg is not a driven leg
                per[int(r["trial"])].append(r)
        n_legs = len(self.course.legs())
        stop_trial = self.cfg["runner_args"]["on_fail"] == "stop-trial"
        complete = sum(1 for rs in per.values()
                       if len(rs) >= n_legs or (rs and rs[-1]["result"] == "FENCE")
                       or (stop_trial and rs and rs[-1]["result"] != "SUCCEEDED"))
        return complete, max(per, default=0)

    def merge_parts(self, cdir):
        """Every runner invocation writes its own parts/partN (a resume never appends to
        old files); data/ is rebuilt from all parts."""
        parts = sorted((cdir / "parts").glob("part*"), key=lambda p: int(p.name[4:]))
        data = cdir / "data"
        shutil.rmtree(data, ignore_errors=True)
        data.mkdir(parents=True)
        header, rows, metas = None, [], []
        for part in parts:
            lf = part / "legs.csv"
            if lf.exists():
                with lf.open() as f:
                    r = csv.reader(f)
                    h = next(r, None)
                    header = header or h
                    rows += list(r)
            for t in part.glob("trial_*"):
                shutil.copytree(t, data / t.name, dirs_exist_ok=True)
            if (part / "meta.json").exists():
                metas.append(json.loads((part / "meta.json").read_text()))
            for f in ("map.json", "start_scan.json"):
                if (part / f).exists():
                    shutil.copyfile(part / f, data / f)
        if header:
            with (data / "legs.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(header)
                w.writerows(rows)
        if metas:
            meta = dict(metas[-1])
            meta["parts"] = [{k: m.get(k) for k in ("started", "finished", "status", "trials", "registration")}
                             for m in metas]
            (data / "meta.json").write_text(json.dumps(meta, indent=1))

    def run_condition(self, name):
        spec = self.cfg["conditions"][name]
        cdir = self.dir / name
        cdir.mkdir(exist_ok=True)
        done, last_trial = self._trial_stats(cdir)
        want = int(self.cfg["trials"])
        if done >= want:
            self.log(f"[{name}] already has {done}/{want} trials - skipping")
            return True
        platform = spec["platform"]
        host = SSHHost(ROVER) if platform == "real" else LocalHost(self.dir / "_stage" / name)
        info = {"name": name, "platform": platform, "fake": spec.get("fake", False),
                "spec": spec, "host": host.name, "started": time.strftime("%Y-%m-%d %H:%M:%S"),
                "trials_done": done, "status": "running"}
        self.log(f"=== condition {name} ({spec.get('note', platform)}) on {host.name}: "
                 f"trials {done + 1}..{want}")
        isaac = stack = oak = None
        logs = []
        try:
            if platform == "real":
                self.preflight_real(host)
            else:
                kill_local_leftovers()
                clean_local_ros()
            host.put_dir(self.dir / "bundle", "bundle")
            if platform == "sim":
                isaac = self.start_isaac(name, spec, cdir, info)
            elif not self.args.yes:
                self.prompt_real_start()
            if platform == "real" and self.cfg["stack"] == "depth":
                oak_log = open(cdir / "oak.log", "w")
                logs.append(oak_log)
                oak = host.popen("bash ~/rover_nav/start_oak.sh", oak_log, tty=True)
                self.log(f"[{name}] OAK driver starting (start_oak.sh)")
            stack_log = open(cdir / "stack.log", "w")
            logs.append(stack_log)
            bundle = host.path("bundle")
            single = "source ~/rover_nav/_check_single_stack.sh && " if platform == "real" else ""
            stack_cmd = (f"{host.env} && {single}exec ros2 launch {bundle}/stack/navcmp_stack.launch.py "
                         f"platform:={platform} stack:={self.cfg['stack']} bundle_dir:={bundle}")
            stack = host.popen(stack_cmd, stack_log, tty=(platform == "real"))
            self.log(f"[{name}] nav stack launching ({self.cfg['stack']}) -> {cdir.name}/stack.log")
            return self.run_runner(name, host, spec, cdir, want - done, last_trial + 1, platform,
                                   info, stack, isaac)
        except KeyboardInterrupt:
            info["status"] = "interrupted"
            raise
        except Exception as exc:
            info["status"] = f"error: {exc}"
            self.log(f"[{name}] ERROR: {exc}")
            return False
        finally:
            # order matters on the real rover: the runner has already zeroed /cmd_vel; only
            # now stop the stack (which stops the base driver), then the OAK driver.
            for p in (stack, oak):
                if p is not None:
                    host.stop(p)
            if isaac is not None:
                _stop_group(isaac, 10)
                self.log(f"[{name}] Isaac stopped")
            for f in logs:
                f.close()
            if platform == "real":
                host.run("pkill -INT -f navcmp_stack.launch.py; true", timeout=20)
            else:
                kill_local_leftovers()
                clean_local_ros()
            info["finished"] = time.strftime("%Y-%m-%d %H:%M:%S")
            info["trials_done"] = self._trial_stats(cdir)[0]
            if info["status"] == "running":
                info["status"] = "done" if info["trials_done"] >= want else "incomplete"
            (cdir / "condition.json").write_text(json.dumps(info, indent=1))
            self.log(f"[{name}] {info['status']}: {info['trials_done']}/{want} trials")
            if platform == "real" and not info.get("fake"):
                remaining = [n for n in self.cfg["order"]
                             if self.cfg["conditions"][n]["platform"] == "real" and n != name
                             and self.condition_done(n) < want]
                if not remaining:
                    self.log("REAL ROVER DONE - safe to power it off (sim conditions run on the desktop)")

    def run_runner(self, name, host, spec, cdir, n_trials, first_trial, platform, info, stack, isaac):
        bundle = host.path("bundle")
        part = 1 + len(list((cdir / "parts").glob("part*"))) if (cdir / "parts").exists() else 1
        out_rel = f"out/{name}_part{part}"
        if platform == "real":
            host.run(f"rm -rf {shlex.quote(host.path(out_rel))}; mkdir -p {shlex.quote(host.path(out_rel))}")
        py = "python3" if platform == "real" else "/usr/bin/python3"
        args = [f"{bundle}/mission_runner.py", "--course", f"{bundle}/course.yaml",
                "--out", host.path(out_rel), "--platform", platform, "--label", name,
                "--trials", str(n_trials), "--first-trial", str(first_trial),
                "--on-fail", self.cfg["runner_args"]["on_fail"],
                "--leg-pause", str(self.cfg["runner_args"]["leg_pause"]),
                "--ready-timeout", str(self.args.ready_timeout), "--watch-stdin"]
        if platform == "real" and not spec.get("fake"):
            args += ["--battery-min", str(BATTERY_MIN_V)]
        if self.cfg["stack"] == "depth":
            args += ["--depth-topic", nav_params.DEPTH_TOPIC]
        cmd = f"{host.env} && exec {py} -u " + " ".join(shlex.quote(a) for a in args)
        rlog = open(cdir / "runner.log", "a")
        rlog.write(f"\n===== part{part}: {cmd}\n")
        runner = host.popen(cmd, subprocess.PIPE, stdin=subprocess.PIPE)
        threading.Thread(target=_pump, args=(runner.stderr, rlog), daemon=True).start()
        lock = threading.Lock()

        def tell_stop(why):
            self.log(f"[{name}] {why} - telling the runner to stop")
            with lock:
                try:
                    runner.stdin.write("stop\n")
                    runner.stdin.flush()
                except Exception:
                    pass

        def watchdog():
            # Isaac or the stack dying freezes /clock: the runner's sim-time leg timeout
            # would never fire, so the session watches the processes itself.
            while runner.poll() is None:
                if isaac is not None and isaac.poll() is not None:
                    return tell_stop(f"Isaac exited (code {isaac.returncode})")
                if stack is not None and stack.poll() is not None:
                    return tell_stop(f"nav stack exited (code {stack.returncode})")
                time.sleep(1.0)
        threading.Thread(target=watchdog, daemon=True).start()
        status = None
        try:
            for line in runner.stdout:
                line = line.strip()
                if not line.startswith("{"):
                    rlog.write(line + "\n")
                    continue
                try:
                    ev = json.loads(line)
                except json.JSONDecodeError:
                    continue
                self._on_event(name, ev)
                if ev["ev"] == "done":
                    status = ev.get("status")
            runner.wait(timeout=30)
        finally:
            if runner.poll() is None:
                tell_stop("session ending")
                try:
                    runner.wait(timeout=15)
                except subprocess.TimeoutExpired:
                    pass
                _stop_group(runner, 5)
            rlog.close()
            got = host.get_dir(out_rel, cdir / "parts" / f"part{part}")
            if got:
                self.merge_parts(cdir)
            else:
                self.log(f"[{name}] WARNING: could not fetch the runner output from {host.name}")
        info["runner_status"] = status
        if status != "done":
            info["status"] = f"runner: {status}"
            return False
        return True

    def _on_event(self, name, ev):
        e = ev["ev"]
        if e == "waiting":
            self.log(f"[{name}] waiting for {ev['what']}")
        elif e == "ready":
            reg = ev["registration"]
            self.log(f"[{name}] ready; course registered at map pose "
                     f"({reg['map_at_start'][0]:.2f}, {reg['map_at_start'][1]:.2f})")
        elif e == "leg_start":
            self.log(f"[{name}] trial {ev['trial']} leg {ev['leg']} ->")
        elif e == "leg_end":
            self.log(f"[{name}] trial {ev['trial']} leg {ev['leg']}: {ev['result']} "
                     f"{ev['duration_s']:.1f} s, final err {ev['final_err_m'] * 100:.1f} cm, "
                     f"recoveries {ev['recoveries']}")
        elif e in ("aborted", "error"):
            self.log(f"[{name}] RUNNER {e.upper()}: {ev.get('reason')}")

    # ── platform steps ───────────────────────────────────────────────────────
    def preflight_real(self, host):
        self.log(f"[real] preflight on {host.name}")
        try:
            r = host.run("true", timeout=20)
        except subprocess.TimeoutExpired:
            raise SessionError(f"{host.name} did not answer ssh within 20 s")
        if r.returncode:
            raise SessionError(f"cannot reach {host.name}: {r.stderr.strip()}")
        r = host.run(f"{REAL_ENV}; echo \"$ROS_LOCALHOST_ONLY|${{ROS_DISCOVERY_SERVER:-}}\"")
        lo, _, ds = r.stdout.strip().partition("|")
        if (lo != "1" or ds) and not os.environ.get("NAVCMP_ALLOW_NETWORKED_ROVER"):
            raise SessionError(
                f"rover ROS is not isolated (ROS_LOCALHOST_ONLY={lo!r}, discovery server={ds!r}): "
                "the desktop's Isaac /cmd_vel could drive the real rover. Fix "
                "rover_nav/local_ros_env.sh (or set NAVCMP_ALLOW_NETWORKED_ROVER=1 if the rover "
                "is on another ROS_DOMAIN_ID).")
        pk = " ".join(REAL_PACKAGES)
        r = host.run(f"{REAL_ENV}; for p in {pk}; do ros2 pkg prefix $p >/dev/null 2>&1 || echo MISSING $p; done",
                     timeout=60)
        missing = [ln.split()[1] for ln in r.stdout.splitlines() if ln.startswith("MISSING")]
        if self.cfg["stack"] != "depth" and "depth_image_proc" in missing:
            missing.remove("depth_image_proc")
        if missing:
            raise SessionError(f"rover is missing ROS packages: {missing} (rover_nav/01_build.sh)")
        # [x]yz: the pattern must not match this very command line (pgrep -f sees the
        # `bash -c "...pgrep..."` that ssh runs, so a plain 'ugv_bringup' always "finds" one)
        r = host.run(f"{REAL_ENV}; fuser ${{UGV_SERIAL_PORT:-/dev/ttyTHS1}} >/dev/null 2>&1 && echo BUSY; "
                     "pgrep -f '[u]gv_bringup |[c]ontroller_server|[s]lam_toolbox|[b]t_navigator' "
                     ">/dev/null && echo NAV; true")
        if "BUSY" in r.stdout or "NAV" in r.stdout:
            raise SessionError(
                "a base driver / nav stack is already running on the rover. The session starts "
                "its own (bringup_lidar + Nav2). Halt it first: ~/rover_nav/stop_goal.sh, then "
                "~/rover_nav/stop_all.sh")
        self.log("[real] preflight OK: reachable, ROS isolated, packages present, base free")

    def prompt_real_start(self):
        s = self.course.start
        print("\n" + "=" * 72)
        print(f" Place the REAL rover on the START mark of course '{self.course.name}':")
        print(f"   base_footprint at x={s[0]:.2f} m, y={s[1]:.2f} m (grid frame), facing "
              f"{math.degrees(s[2]):.0f} deg (0 = along +x)")
        print("   arena walls/boxes built as in the course YAML; nothing else in the arena.")
        print(" The SLAM map is anchored where the rover sits when the stack starts.")
        print("=" * 72)
        input(" Press Enter when the rover is in place (Ctrl-C aborts) ... ")

    def start_isaac(self, name, spec, cdir, info):
        env = dict(os.environ, ISAAC_ROS_LOCALHOST_ONLY="0", ROS_LOCALHOST_ONLY="0",
                   RMW_IMPLEMENTATION="rmw_fastrtps_cpp", SIM_REALTIME="1", PYTHONUNBUFFERED="1")
        for k in ("SIM_ROS", "SIM_TRIAL_SEQ", "SIM_TRIAL_OUT", "SIM_DIAG", "ROS_DISCOVERY_SERVER"):
            env.pop(k, None)
        cmd = [ISAACPY, str(RUN_COURSE), "--course", str(self.dir / "bundle" / "course.yaml")]
        if not self.args.show_isaac:
            cmd.append("--headless")
        if self.cfg["stack"] == "lidar":
            cmd.append("--no-camera-topics")
        cmd += ["--lidar-profile", self.cfg.get("lidar_profile", "ld19")]
        if spec.get("calib"):
            src = yaml.safe_load((CALIB / spec["calib"]).read_text()) or {}
            calib = src.get("sim_calibration_yaml", src)
            if spec.get("perturb"):                 # fake real rover: scaled physics keys
                calib = perturb_calib(calib, spec["perturb"])
                info["perturb"] = spec["perturb"]
            (cdir / "calib.yaml").write_text(yaml.safe_dump(calib, sort_keys=False))
            cmd += ["--calib", str(cdir / "calib.yaml"), "--combine-mode", spec.get("combine_mode", "min")]
        if self.cfg.get("world_from_real_map"):
            real_map = self.dir / "real" / "data" / "map.json"
            if real_map.exists():
                cmd += ["--world-map", str(real_map)]
                info["world_map"] = str(real_map)
            else:
                self.log(f"[{name}] no real map.json yet - Isaac gets the course geometry only")
        info["isaac_cmd"] = cmd
        log = open(cdir / "isaac.log", "w")
        self.log(f"[{name}] launching Isaac -> {cdir.name}/isaac.log")
        proc = subprocess.Popen(cmd, env=env, stdout=log, stderr=subprocess.STDOUT,
                                start_new_session=True)
        deadline = time.monotonic() + self.args.isaac_timeout
        while time.monotonic() < deadline:
            if proc.poll() is not None:
                raise SessionError(f"Isaac exited during startup (code {proc.returncode}); see isaac.log")
            if "Entering simulation loop" in (cdir / "isaac.log").read_text(errors="ignore"):
                self.log(f"[{name}] Isaac is simulating")
                return proc
            time.sleep(2)
        _stop_group(proc, 5)
        raise SessionError(f"Isaac did not start within {self.args.isaac_timeout:.0f} s")

    # ── all ──────────────────────────────────────────────────────────────────
    def run(self):
        self.log(f"session {self.dir.name}: course {self.cfg['course_name']}, stack {self.cfg['stack']}, "
                 f"{self.cfg['trials']} trials, conditions {', '.join(self.cfg['order'])}"
                 + ("  [FAKE REAL ROVER - pipeline test only]" if self.cfg["fake_real"] else ""))
        ok = True
        for name in self.cfg["order"]:
            ok &= bool(self.run_condition(name))
        if not self.args.no_report:
            import report
            try:
                out = report.build(self.dir)
                self.log(f"report: {out}")
            except Exception as exc:
                self.log(f"report failed: {exc}")
                ok = False
        self.log("session " + ("complete" if ok else "finished with problems - see above"))
        return 0 if ok else 1


def perturb_calib(calib, factors):
    """Nested sim_calibration dict with the given physics keys scaled ({key: factor});
    used only for the --fake-real stand-in rover."""
    sys.path.insert(0, str(CALIB.parent / "ugv_controller_v2" / "calibration"))
    import calib_params as CP
    flat = CP.flatten(calib)
    for k, f in factors.items():
        if k not in flat:
            raise SessionError(f"perturb: '{k}' is not a calibration key in this calib")
        flat[k] = float(flat[k]) * float(f)
    def merge(base, upd):
        out = dict(base)
        for k, v in upd.items():
            out[k] = merge(out.get(k) or {}, v) if isinstance(v, dict) else v
        return out
    return merge(calib, CP.nest(flat))


def _pump(stream, f):
    for line in stream:
        f.write(line)
        f.flush()


def _git_rev():
    try:
        return subprocess.run(["git", "-C", str(REPO), "rev-parse", "--short", "HEAD"],
                              capture_output=True, text=True, timeout=5).stdout.strip()
    except Exception:
        return ""


def parse_args(argv=None):
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--course", help="course YAML (navcmp/courses/)")
    ap.add_argument("--stack", choices=("lidar", "depth"), default="lidar",
                    help="lidar: slam_toolbox + Nav2 on the LD19; depth: + OAK depth cloud in the costmaps")
    ap.add_argument("--trials", type=int, default=3, help="trials per condition")
    ap.add_argument("--conditions", default="real,sim_physics,sim_default",
                    help="comma list from navcmp/conditions.yaml, run in this order")
    ap.add_argument("--conditions-file", help="extra/overriding condition definitions (yaml)")
    ap.add_argument("--fake-real", action="store_true",
                    help="no rover: 'real' is played by Isaac with perturbed physics (TESTFAKE_)")
    ap.add_argument("--resume", help="session dir to finish")
    ap.add_argument("--out-root", default=str(HERE / "runs"))
    ap.add_argument("--nav-overrides", help="yaml {node: {param: value}} applied to BOTH rovers")
    ap.add_argument("--on-fail", choices=("continue", "stop-trial"), default="continue")
    ap.add_argument("--leg-pause", type=float, default=1.0)
    ap.add_argument("--world-from-real-map", action="store_true",
                    help="sim conditions also get walls from the real run's slam map (room twin)")
    ap.add_argument("--lidar-profile", choices=("ld19", "native"), default="ld19",
                    help="Isaac lidar range: ld19 = 0.10-12 m like the real LD19 (default); "
                         "native = the USD's Example_Rotary_2D (blind inside 1 m)")
    ap.add_argument("--ready-timeout", type=float, default=300.0)
    ap.add_argument("--isaac-timeout", type=float, default=420.0)
    ap.add_argument("--show-isaac", action="store_true", help="windowed Isaac (default headless)")
    ap.add_argument("--yes", action="store_true", help="skip the place-the-rover prompt")
    ap.add_argument("--no-report", action="store_true")
    a = ap.parse_args(argv)
    if not a.resume and not a.course:
        ap.error("--course is required (or --resume)")
    return a


def main():
    args = parse_args()
    s = Session(args)
    try:
        return s.run()
    except KeyboardInterrupt:
        s.log("interrupted - rover stopped by the runner; finish later with --resume " + str(s.dir))
        return 130
    except SessionError as exc:
        s.log(f"ERROR: {exc}")
        return 1


if __name__ == "__main__":
    sys.exit(main())
