#!/usr/bin/env python3
"""The real-rover transport (SSHHost + Session.preflight_real) against fake ssh/scp shims
that execute in a fake rover $HOME - no rover, no keys, no network."""
from __future__ import annotations

import json
import os
import stat
import subprocess
import sys
import time
from pathlib import Path

import pytest

NAVCMP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAVCMP))

SSH = r'''#!/bin/bash
# fake ssh: drop options, take the host, run the command in $FAKE_HOME
args=()
while [ $# -gt 0 ]; do
  case "$1" in -o) shift 2;; -tt|-t|-q) shift;; *) args+=("$1"); shift;; esac
done
cd "$FAKE_HOME" && HOME="$FAKE_HOME" exec bash -c "${args[*]:1}"
'''
SCP = r'''#!/bin/bash
args=()
while [ $# -gt 0 ]; do
  case "$1" in -o) shift 2;; -q|-r) shift;; *) args+=("$1"); shift;; esac
done
src="${args[0]}"; dst="${args[1]}"
src="${src/#*:/$FAKE_HOME/}"; dst="${dst/#*:/$FAKE_HOME/}"
mkdir -p "$(dirname "$dst")"; exec cp -r "$src" "$dst"
'''


@pytest.fixture
def fake_rover(tmp_path, monkeypatch):
    home = tmp_path / "rover_home"
    (home / "rover_nav").mkdir(parents=True)
    bin_ = tmp_path / "bin"
    bin_.mkdir()
    for name, body in (("ssh", SSH), ("scp", SCP)):
        f = bin_ / name
        f.write_text(body)
        f.chmod(f.stat().st_mode | stat.S_IEXEC)
    # the rover's env script + a fake `ros2` that knows some packages
    (home / "rover_nav" / "local_ros_env.sh").write_text(
        f'export ROS_LOCALHOST_ONLY=1; unset ROS_DISCOVERY_SERVER; export PATH="{home}/rbin:$PATH"\n')
    (home / "rbin").mkdir()
    ros2 = home / "rbin" / "ros2"
    ros2.write_text('#!/bin/bash\n[ "$1 $2" = "pkg prefix" ] || exit 1\n'
                    'grep -qx "$3" "$HOME/installed" && echo /opt/ros/humble || exit 1\n')
    ros2.chmod(0o755)
    (home / "installed").write_text("nav2_bringup\nslam_toolbox\nteb_local_planner\nugv_bringup\n")
    # fuser / pgrep on the "rover": busy only if $HOME/busy names it
    for tool in ("fuser", "pgrep"):
        f = home / "rbin" / tool
        f.write_text(f'#!/bin/bash\ngrep -qx {tool} "$HOME/busy" 2>/dev/null\n')
        f.chmod(0o755)
    monkeypatch.setenv("FAKE_HOME", str(home))
    monkeypatch.setenv("PATH", f"{bin_}:{os.environ['PATH']}")
    return home


def _session(tmp_path, stack="lidar"):
    import course as C
    import session as S
    s = S.Session.__new__(S.Session)
    s.cfg = {"stack": stack}
    s.dir = tmp_path
    s.logf = open(tmp_path / "log.txt", "w")
    s.course = C.load(NAVCMP / "courses" / "grid2x2_box.yaml")
    return s


def test_stage_run_fetch(fake_rover, tmp_path):
    import session as S
    host = S.SSHHost("rover-lan")
    src = tmp_path / "bundle"
    (src / "stack").mkdir(parents=True)
    (src / "stack" / "a.txt").write_text("hello")
    host.put_dir(src, "bundle")
    host.put_dir(src, "bundle")                       # re-stage replaces, never nests
    assert (fake_rover / "navcmp_run" / "bundle" / "stack" / "a.txt").read_text() == "hello"
    assert not (fake_rover / "navcmp_run" / "bundle" / "bundle").exists()
    r = host.run(f"{host.env}; echo $ROS_LOCALHOST_ONLY; pwd")
    assert r.stdout.split() == ["1", str(fake_rover)]
    out = fake_rover / "navcmp_run" / "out" / "real_part1" / "trial_01"
    out.mkdir(parents=True)
    (out / "trace.csv").write_text("t\n0\n")
    assert host.get_dir("out/real_part1", tmp_path / "got")
    assert (tmp_path / "got" / "trial_01" / "trace.csv").exists()


def test_runner_link_watchdog(fake_rover):
    """The runner sees EOF when the desktop side closes stdin (the link died)."""
    import session as S
    host = S.SSHHost("rover-lan")
    code = ("import sys,json; print(json.dumps({'ev':'ready'}), flush=True); "
            "sys.stdin.read(); print(json.dumps({'ev':'done','status':'eof'}), flush=True)")
    p = host.popen(f"exec python3 -u -c {json.dumps(code)}", subprocess.PIPE, stdin=subprocess.PIPE)
    assert json.loads(p.stdout.readline())["ev"] == "ready"
    p.stdin.close()
    assert json.loads(p.stdout.readline())["status"] == "eof"
    p.wait(timeout=10)


def test_held_open_stack_stops_on_ctrl_c(fake_rover):
    import session as S
    host = S.SSHHost("rover-lan")
    log = open(fake_rover / "stack.log", "w")
    p = host.popen("trap 'echo got-int; exit 0' INT; while true; do sleep 0.2; done", log, tty=True)
    time.sleep(0.5)
    t = time.monotonic()
    host.stop(p, grace=3)          # fake ssh has no tty: falls back to killing the group
    assert p.poll() is not None and time.monotonic() - t < 15


def test_preflight(fake_rover, tmp_path):
    import session as S
    s = _session(tmp_path)
    host = S.SSHHost("rover-lan")
    s.preflight_real(host)                                         # all good
    (fake_rover / "installed").write_text("nav2_bringup\nslam_toolbox\nugv_bringup\n")
    with pytest.raises(S.SessionError, match="teb_local_planner"):
        s.preflight_real(host)
    (fake_rover / "installed").write_text("nav2_bringup\nslam_toolbox\nteb_local_planner\nugv_bringup\n")
    s_depth = _session(tmp_path, "depth")
    with pytest.raises(S.SessionError, match="depth_image_proc"):
        s_depth.preflight_real(host)
    for tool in ("fuser", "pgrep"):                               # a stack already up
        (fake_rover / "busy").write_text(tool + "\n")
        with pytest.raises(S.SessionError, match="already running"):
            s.preflight_real(host)
    (fake_rover / "busy").unlink()
    env = fake_rover / "rover_nav" / "local_ros_env.sh"
    env.write_text(env.read_text().replace("ROS_LOCALHOST_ONLY=1", "ROS_LOCALHOST_ONLY=0"))
    with pytest.raises(S.SessionError, match="not isolated"):
        s.preflight_real(host)
