#!/usr/bin/env python3
"""Offline tests for navcmp (no rover, no Isaac). Needs system ROS python for the runner
import:  source /opt/ros/humble/setup.bash && python3 -m pytest navcmp/tests -q"""
from __future__ import annotations

import json
import math
import random
import sys
from pathlib import Path

import pytest
import yaml

NAVCMP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAVCMP))
sys.path.insert(0, str(Path(__file__).resolve().parent))
import course as C  # noqa: E402
import metrics as M  # noqa: E402
import nav_params  # noqa: E402

COURSES = sorted((NAVCMP / "courses").glob("*.yaml"))
BOX = NAVCMP / "courses" / "grid2x2_box.yaml"


# ── course ───────────────────────────────────────────────────────────────────
@pytest.mark.parametrize("path", COURSES, ids=lambda p: p.stem)
def test_shipped_courses_validate(path):
    c = C.load(path)
    assert c.legs()[-1].name == "return"
    assert not c.warnings()


def test_se2_roundtrip():
    rng = random.Random(0)
    for _ in range(50):
        a = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3))
        b = (rng.uniform(-3, 3), rng.uniform(-3, 3), rng.uniform(-3, 3))
        r = C.compose(C.inverse(a), C.compose(a, b))
        assert r[0] == pytest.approx(b[0], abs=1e-9) and r[1] == pytest.approx(b[1], abs=1e-9)
        assert C.wrap(r[2] - b[2]) == pytest.approx(0, abs=1e-9)


def test_registration_maps_start_to_map_pose():
    start = (0.35, 0.35, 0.0)
    map_pose = (0.02, -0.01, 0.05)            # slam map origin ~ where the stack started
    T_mc = C.compose(map_pose, C.inverse(start))
    assert C.compose(T_mc, start) == pytest.approx(map_pose)
    g = C.compose(T_mc, (1.65, 0.35, math.pi / 2))      # 1.3 m ahead of start
    assert math.hypot(g[0] - map_pose[0], g[1] - map_pose[1]) == pytest.approx(1.3)


def test_box_distance():
    b = C.Box("b", 1.0, 1.0, 0.4, 0.2, 0.3, yaw=0.0)
    assert b.distance(1.0, 1.0) < 0
    assert b.distance(1.5, 1.0) == pytest.approx(0.3)
    b90 = C.Box("b", 1.0, 1.0, 0.4, 0.2, 0.3, yaw=math.pi / 2)
    assert b90.distance(1.0, 1.5) == pytest.approx(0.3)


def test_validation_catches_bad_courses():
    d = yaml.safe_load(BOX.read_text())
    d["goals"][0].update(x=1.0, y=1.0)        # inside the centre box
    d["boxes"][0]["size"][2] = 0.1             # under the lidar plane
    problems = C.from_dict(d).validate()
    assert any("from an obstacle" in p for p in problems)
    assert any("scan plane" in p for p in problems)


def test_occupancy_to_boxes():
    w, h = 10, 8
    grid = [0] * (w * h)
    for r in range(2, 5):                      # a 3x4-cell block
        for c in range(3, 7):
            grid[r * w + c] = 100
    boxes = C.boxes_from_occupancy(grid, w, h, 0.05, (1.0, 2.0, 0.0))
    assert len(boxes) == 1
    b = boxes[0]
    assert (b.sx, b.sy) == pytest.approx((0.20, 0.15))
    assert (b.cx, b.cy) == pytest.approx((1.0 + 0.25, 2.0 + 0.175))


# ── shared nav params ────────────────────────────────────────────────────────
def test_params_identical_and_self_contained(tmp_path):
    lidar = nav_params.build("lidar")
    depth = nav_params.build("depth")
    text = yaml.safe_dump(lidar)
    assert "find-pkg-share ugv_nav" not in text            # rover's old ugv_nav never consulted
    assert "keepout_filter" not in text and "speed_filter" not in text
    for cm in ("local_costmap", "global_costmap"):
        assert "depth_cloud" not in lidar[cm][cm]["ros__parameters"]["obstacle_layer"]
        ol = depth[cm][cm]["ros__parameters"]["obstacle_layer"]
        assert ol["depth_cloud"]["topic"] == nav_params.DEPTH_TOPIC
        assert ol["observation_sources"] == "scan depth_cloud"
    # planner/controller untouched
    src = yaml.safe_load(nav_params.SOURCE_PARAMS.read_text())
    assert lidar["controller_server"] == src["controller_server"]
    hashes = nav_params.write_stack(tmp_path, "lidar")
    assert (tmp_path / "stack" / "navcmp_stack.launch.py").exists()
    assert all(len(h) == 64 for h in hashes.values())


def test_params_overrides():
    p = nav_params.build("lidar", {"controller_server": {"FollowPath": {"max_vel_x": 0.2}},
                                   "local_costmap": {"inflation_layer": {"inflation_radius": 0.3}}})
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_x"] == 0.2
    assert p["controller_server"]["ros__parameters"]["FollowPath"]["max_vel_theta"] == 0.5
    assert p["local_costmap"]["local_costmap"]["ros__parameters"]["inflation_layer"]["inflation_radius"] == 0.3


# ── metrics ──────────────────────────────────────────────────────────────────
def test_frechet():
    p = [(0.0, 0.0), (1.0, 0.0), (2.0, 0.0)]
    assert M.frechet(p, p) == 0
    q = [(x, y + 0.1) for x, y in p]
    assert M.frechet(p, q) == pytest.approx(0.1)
    assert M.mean_deviation(p, q) == pytest.approx(0.1)
    r = M.resample([(0, 0.0, 0.0), (1, 2.0, 0.0)], n=5)
    assert r[2] == pytest.approx((1.0, 0.0))


@pytest.fixture(scope="module")
def synthetic(tmp_path_factory):
    pytest.importorskip("rclpy")
    import synth
    root = tmp_path_factory.mktemp("navcmp") / "SYNTH_session"
    return synth.make_session(root, BOX, {
        "real": dict(v=0.20, v_sd=0.01),
        "sim_physics": dict(v=0.20, v_sd=0.01),                 # same distribution as real
        "sim_default": dict(v=0.13, v_sd=0.003, lateral=0.25),   # slower + bows out
    }, trials=5)


def test_metrics_rank_the_better_simulator(synthetic):
    conds = {n: M.load_condition(synthetic / n) for n in ("real", "sim_physics", "sim_default")}
    res = M.compare(conds)
    good, bad = res["conditions"]["sim_physics"], res["conditions"]["sim_default"]
    assert good["rms_z"] < bad["rms_z"]
    assert good["rms_z"] < 2.0 < bad["rms_z"]
    assert good["path_gap_ratio"] < bad["path_gap_ratio"]
    assert math.isfinite(res["real_floor_rms_z"])
    assert good["success_rate"] == 1.0
    assert bad["per_metric_mean_z"]["time_s"] > 2        # slower -> longer legs


def test_report_builds(synthetic):
    import report
    md = report.build(synthetic)
    text = md.read_text()
    assert "closest** — " in text and "sim_physics" in text
    for f in ("paths.png", "agreement.png", "metric_z.png", "speed.png", "leg_metrics.csv",
              "summary.json"):
        assert (md.parent / f).exists(), f
    s = json.loads((md.parent / "summary.json").read_text())
    assert s["conditions"]["sim_physics"]["rms_z"] < s["conditions"]["sim_default"]["rms_z"]


def test_resume_merge_and_trial_counting(tmp_path):
    pytest.importorskip("rclpy")
    import session as S
    import synth
    course = C.load(BOX)
    rng = random.Random(1)
    cdir = tmp_path / "real"
    synth.write_part(cdir / "parts" / "part1", course, 2, rng, first=1)
    # part 2 = resumed run: trial 3 complete, trial 4 aborted after one leg (fence/battery)
    synth.write_part(cdir / "parts" / "part2", course, 2, rng, first=3)
    import csv
    lf = cdir / "parts" / "part2" / "legs.csv"
    rows = list(csv.DictReader(lf.open()))
    keep = [r for r in rows if not (r["trial"] == "4" and r["leg_index"] != "0")]
    with lf.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(keep)
    s = S.Session.__new__(S.Session)
    s.course = course
    s.cfg = {"runner_args": {"on_fail": "continue"}}
    s.merge_parts(cdir)
    assert s._trial_stats(cdir) == (3, 4)    # 3 complete, next resume starts at trial 5
    merged = list(csv.DictReader((cdir / "data" / "legs.csv").open()))
    assert len(merged) == 4 * 3 + 1
    assert sorted(p.name for p in (cdir / "data").glob("trial_*")) == [f"trial_0{i}" for i in range(1, 5)]


def test_stopped_legs_dropped_fence_legs_count(tmp_path):
    """STOPPED (operator/link/battery) is not the rover's doing: dropped from metrics and
    not a driven leg for resume. FENCE (left the arena) is a navigation failure."""
    pytest.importorskip("rclpy")
    import csv
    import session as S
    import synth
    course = C.load(BOX)
    cdir = tmp_path / "real"
    part = cdir / "parts" / "part1"
    synth.write_part(part, course, 3, random.Random(2), first=1)
    rows = list(csv.DictReader((part / "legs.csv").open()))
    out = []
    for r in rows:
        if r["trial"] == "2" and r["leg_index"] == "1":
            out.append(dict(r, result="FENCE"))          # trial 2 ends at a fence exit
        elif r["trial"] == "2" and int(r["leg_index"]) > 1:
            continue
        elif r["trial"] == "3" and r["leg_index"] == "2":
            out.append(dict(r, result="STOPPED"))        # operator stop mid trial 3
        elif r["trial"] == "3" and int(r["leg_index"]) > 2:
            continue
        else:
            out.append(r)
    with (part / "legs.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=rows[0].keys())
        w.writeheader()
        w.writerows(out)
    s = S.Session.__new__(S.Session)
    s.course, s.cfg = course, {"runner_args": {"on_fail": "continue"}}
    s.merge_parts(cdir)
    assert s._trial_stats(cdir) == (2, 3)     # trial 1 + fence-ended trial 2 are complete
    m = M.condition_metrics(M.load_condition(cdir))
    results = [r["result"] for r in m]
    assert "STOPPED" not in results and results.count("FENCE") == 1
    assert len(m) == 4 + 2 + 2
