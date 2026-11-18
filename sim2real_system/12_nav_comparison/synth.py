"""Synthetic navcmp sessions (no ROS, no Isaac) for the metric/report tests.

A unicycle drives each course leg (turn toward the goal, then go), with per-trial speed
scatter and per-sample noise, and writes exactly the files mission_runner.py writes.
"""
from __future__ import annotations

import csv
import json
import math
import random
import shutil
import sys
from pathlib import Path

NAVCMP = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(NAVCMP))
import course as C  # noqa: E402
from mission_runner import LEG_FIELDS, TRACE_FIELDS  # noqa: E402  (needs rclpy importable)


def drive_trial(course, rng, v=0.2, w=0.6, v_sd=0.01, lateral=0.0, hz=10.0, fail_leg=None):
    x, y, th = course.start
    t = 0.0
    traces, legs = [], []
    for idx, leg in enumerate(course.legs()):
        t0, sx, sy, sth = t, x, y, th
        vt = max(0.05, rng.gauss(v, v_sd))
        bend = rng.gauss(lateral, 0.01)
        ok = fail_leg != leg.name
        for _ in range(int(120 * hz)):
            dx, dy = leg.x - x, leg.y - y
            dist = math.hypot(dx, dy)
            if dist < 0.05 or (not ok and t - t0 > 20):
                break
            err = C.wrap(math.atan2(dy, dx) + bend * min(dist, 1.0) - th)
            if abs(err) > 0.3:
                cv, cw = 0.0, math.copysign(w, err)
            else:
                cv, cw = vt, 2.0 * err
            th = C.wrap(th + cw / hz)
            x += cv * math.cos(th) / hz
            y += cv * math.sin(th) / hz
            t += 1.0 / hz
            traces.append([round(t, 3), 0, leg.name, x, y, th, x, y, th, x, y, th, cv, cw, cv, cw,
                           round(min(course.clearance(x, y), 3.0), 3), round(dist, 3), 0, 12.0])
        legs.append({"leg": leg.name, "leg_index": idx, "goal_x": leg.x, "goal_y": leg.y,
                     "goal_yaw": leg.yaw, "result": "SUCCEEDED" if ok else "ABORTED", "reason": "",
                     "t_start": round(t0, 3), "duration_s": round(t - t0, 3), "wall_s": round(t - t0, 3),
                     "recoveries": 0 if ok else 2, "final_x": x, "final_y": y, "final_yaw": th,
                     "final_err_m": math.hypot(leg.x - x, leg.y - y),
                     "final_yaw_err_deg": abs(math.degrees(C.wrap(th - leg.yaw))),
                     "start_x": sx, "start_y": sy, "start_yaw": sth, "n_samples": 0})
    return traces, legs


def write_part(part_dir, course, trials, rng, first=1, **kw):
    part_dir.mkdir(parents=True, exist_ok=True)
    with (part_dir / "legs.csv").open("w", newline="") as lf:
        lw = csv.DictWriter(lf, fieldnames=LEG_FIELDS)
        lw.writeheader()
        for trial in range(first, first + trials):
            traces, legs = drive_trial(course, rng, **kw)
            td = part_dir / f"trial_{trial:02d}"
            td.mkdir(exist_ok=True)
            with (td / "trace.csv").open("w", newline="") as f:
                w = csv.writer(f)
                w.writerow(TRACE_FIELDS)
                w.writerows(traces)
            (td / "cmd.csv").write_text("t,wall,v,w\n")
            for r in legs:
                lw.writerow(dict(r, trial=trial))
    reg = {"T_course_map": [0, 0, 0], "T_map_course": [0, 0, 0], "map_at_start": list(course.start)}
    (part_dir / "meta.json").write_text(json.dumps({"status": "done", "registration": reg,
                                                    "trials": []}))


def make_session(root, course_file, specs, trials=5, seed=3, fake=False):
    """specs: {condition: drive_trial kwargs}. Returns the session dir (merged data/)."""
    import session as S
    course = C.load(course_file)
    sd = Path(root)
    (sd / "bundle").mkdir(parents=True, exist_ok=True)
    shutil.copyfile(course_file, sd / "bundle" / "course.yaml")
    cfg = {"course": str(course_file), "course_name": course.name, "stack": "lidar",
           "trials": trials, "order": list(specs), "fake_real": fake, "created": "synthetic",
           "git": "", "runner_args": {"on_fail": "continue", "leg_pause": 1.0},
           "conditions": {n: {"platform": "real" if n == "real" else "sim", "note": f"synthetic {n}"}
                          for n in specs}}
    (sd / "session.json").write_text(json.dumps(cfg))
    rng = random.Random(seed)
    s = S.Session.__new__(S.Session)
    s.cfg, s.course, s.dir = cfg, course, sd
    for n, kw in specs.items():
        write_part(sd / n / "parts" / "part1", course, trials, rng, **kw)
        s.merge_parts(sd / n)
        (sd / n / "condition.json").write_text(json.dumps({"status": "done"}))
    return sd
