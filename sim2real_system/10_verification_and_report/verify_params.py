#!/usr/bin/env python3
"""Held-out check: did the calibration make the sim match the real rover better?

Drives a FRESH real battery (station starts, same as the loop) that the optimiser never
saw, then simulates the same battery under each parameter set (default: the schema
baseline vs a calibrated surfaces/*.yaml) and scores every set against the same real
data: per-feature z, rms z, and start-aligned path error per test. Writes
loop/runs/verify_<stamp>/report.md (+ path maps / compare plots per test and set).

    python3 loop/verify_params.py --calibrated ../surfaces/carpet_loop.yaml
    python3 loop/verify_params.py --calibrated X.yaml --real-from loop/runs/<s>/iter_02/real
"""
from __future__ import annotations

import argparse
import datetime
import json
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE))
import closed_loop as CLP          # noqa: E402  (shares all the machinery)
from closed_loop import C, F, CAL, ST   # noqa: E402


# Full simulator parameter set for verification: 'baseline' = the schema start values
# (+ the surface's floor-friction band), otherwise a surfaces/*.yaml file's calibrated
# values layered over that start.
def load_theta(path, surface, measured):
    import yaml
    if path in (None, "baseline"):
        return CAL.start_theta(measured, surface, None)
    surface_doc = yaml.safe_load(Path(path).read_text()) or {}
    seed_params = surface_doc.get("calibrated") or C.flatten(surface_doc.get("sim_calibration_yaml", surface_doc))
    return CAL.start_theta(measured, surface, seed_params)


# 1) real trials: recorded (--real-from) or freshly driven with homing to each start;
# 2) targets = real feature mean +- spread per test;
# 3) simulate the battery under the baseline and the calibrated parameters and score
#    both on the SAME real data (rms z, start-aligned path errors);
# 4) write report.md, results.json, compare plots and path maps.
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--surface", default="carpet")
    parser.add_argument("--calibrated", required=True, help="calibrated surfaces/*.yaml")
    parser.add_argument("--baseline", default="baseline",
                    help="'baseline' (schema start + floor band) or another yaml")
    parser.add_argument("--tests", default="T1,T2,T4")
    parser.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"],
                    help="friction combine mode for the CALIBRATED set (use the one it was "
                         "fitted with, e.g. average for concrete_physics.yaml)")
    parser.add_argument("--baseline-combine", default="min", choices=["min", "average", "multiply"],
                    help="combine mode for the baseline set (min = how the sim ran before)")
    parser.add_argument("--repeats", type=int, default=3)
    parser.add_argument("--real-from", default=None)
    parser.add_argument("--keep-bringup", action="store_true")
    parser.add_argument("--allow-approximate", action="store_true")
    args = parser.parse_args()

    config = CLP.load_config(str(CLP.CONFIG))
    test_paths = [CLP.resolve_test(test.strip()) for test in args.tests.split(",") if test.strip()]
    plan = {test_stem: F.load_sequence(str(sequence_path)) for test_stem, sequence_path in test_paths}
    scoring = {test: config["scoring"].get(test, []) for test in plan}
    out_dir = HERE / "runs" / f"verify_{datetime.datetime.now():%Y%m%d_%H%M%S}"
    out_dir.mkdir(parents=True)
    log = CLP.Log(out_dir / "verify.log")
    measured = CAL.load_measured(str(CLP.SC / "optimize" / "measured.yaml"))
    param_sets = {"baseline": load_theta(args.baseline, args.surface, measured),
            "calibrated": load_theta(args.calibrated, args.surface, measured)}
    changed_keys = [key for key in C.OPTIMISABLE if abs(param_sets["baseline"][key] - param_sets["calibrated"][key]) > 1e-9]
    log(f"[verify] parameter sets differ in: "
        + ", ".join(f"{key} {param_sets['baseline'][key]:.4g} -> {param_sets['calibrated'][key]:.4g}" for key in changed_keys))

    # ---- fresh real data ----
    rover = camera = tracker = None
    try:
        if args.real_from:
            real_trials = CLP.load_real_from(args.real_from, plan)
            sys.path.insert(0, str(CLP.SC / "viz"))
            from metric_overlay import load_calib
            H_m2img = load_calib(str(CLP.SC / "calib" / "grid_calib.json"))[1]
        else:
            from camera_track import CameraTrack
            from rover_link import RoverLink
            from localizer import FusedTracker
            fusion_cfg = config.get("fusion", {})
            camera = CameraTrack(allow_approximate=args.allow_approximate)
            camera.latency_s = fusion_cfg.get("cam_latency_s", 0.0)
            CLP.camera_check(camera, log)
            H_m2img = camera.H_m2img
            rover = RoverLink(log=log, logfile=str(out_dir / "rover_agent.log"), fusion_cfg=fusion_cfg)
            rover.open()
            ST.set_guard(rover, config)
            tracker = FusedTracker(camera, rover, cam_latency_s=fusion_cfg.get("cam_latency_s", 0.2),
                                   gain=fusion_cfg.get("gain", 0.3))
            live_view = CLP.LiveView(camera, out_dir, False)
            real_trials = CLP.real_phase(rover, camera, tracker, config, plan, args.repeats, out_dir, log, live_view)
            park_pose = config.get("park")
            if park_pose:
                try:
                    CLP.go_to_pose(rover, tracker, (park_pose["x"], park_pose["y"], math.radians(park_pose["yaw"])),
                                   config, log=log)
                except Exception as exc:
                    log(f"[real] park failed: {exc}")
    finally:
        if tracker is not None and tracker is not camera:
            tracker.close()
        if rover is not None:
            rover.close(stop_bringup=not args.keep_bringup)
        if camera is not None:
            camera.close()
    targets = CLP.build_targets(real_trials, plan, scoring, log)

    # ---- sim under each parameter set, scored on the SAME real data ----
    results = {}
    run_plan = {test: plan[test] for test in plan if real_trials.get(test)}
    for set_name, theta in param_sets.items():
        log(f"[verify] sim with {set_name} parameters")
        combine_mode = args.combine_mode if set_name == "calibrated" else args.baseline_combine
        log(f"[verify]   friction combine mode {combine_mode}")
        sim_csvs = CLP.run_battery(theta, run_plan, out_dir / f"sim_{set_name}", combine_mode)
        sim_features = {test: F.extract(*F.load_csv(sim_csvs[test]), run_plan[test]) for test in run_plan}
        loss, feature_count, z_scores = CLP.score(sim_features, targets)
        path_errors = {}
        for test in run_plan:
            test_dir = out_dir / test
            test_dir.mkdir(exist_ok=True)
            per_repeat = []
            for rep_index, real_trial in enumerate(real_trials[test]):
                metrics = CLP.compare_png(real_trial["csv"], sim_csvs[test], test_dir / f"compare_{set_name}_rep{rep_index+1}.png",
                                    test_dir / f"metrics_{set_name}_rep{rep_index+1}.json", f"{test} {set_name}")
                if metrics:
                    per_repeat.append(metrics)
            path_errors[test] = {key: float(np.mean([metrics[key] for metrics in per_repeat]))
                        for key in ("endpoint_err_m", "mean_err_m", "heading_err_deg")} if per_repeat else {}
        results[set_name] = {"loss": loss, "rms_z": math.sqrt(loss / max(feature_count, 1)), "z": z_scores,
                         "paths": path_errors, "csvs": sim_csvs,
                         "feats": {test: {key: value for key, value in feature_values.items() if CLP.is_scored(key, scoring.get(test, []))}
                                   for test, feature_values in sim_features.items()}}
        log(f"[verify] {set_name}: rms_z={results[set_name]['rms_z']:.2f}  z={z_scores}")
    for test in run_plan:                   # one map per test with both sim paths
        try:
            CLP.path_maps(test, real_trials[test], results["baseline"]["csvs"][test],
                          results["calibrated"]["csvs"][test], out_dir / test, H_m2img, log,
                          "red = baseline sim, cyan = calibrated sim")
        except Exception as exc:
            log(f"[viz] {test}: {exc}")

    # ---- report ----
    baseline, calibrated = results["baseline"], results["calibrated"]
    report_lines = [f"# Held-out verification — {args.surface}", "",
         f"{datetime.datetime.now():%Y-%m-%d %H:%M} · fresh real trials: "
         + ", ".join(f"{test} ×{len(real_trials[test])}" for test in run_plan), "",
         "Changed parameters: " + ", ".join(f"`{key}` {param_sets['baseline'][key]:.4g} → "
                                            f"{param_sets['calibrated'][key]:.4g}" for key in changed_keys), "",
         "| metric | baseline sim | calibrated sim |", "|---|---|---|",
         f"| rms feature error (σ units) | {baseline['rms_z']:.2f} | {calibrated['rms_z']:.2f} |"]
    for test in run_plan:
        for key, label, scale in (("endpoint_err_m", "endpoint error", 100), ("mean_err_m", "mean path error", 100),
                            ("heading_err_deg", "final heading error", 1)):
            unit = "cm" if scale == 100 else "°"
            if baseline["paths"].get(test) and calibrated["paths"].get(test):
                report_lines.append(f"| {test} {label} | {baseline['paths'][test][key]*scale:.1f} {unit} | {calibrated['paths'][test][key]*scale:.1f} {unit} |")
    report_lines += ["", "## Per-feature (real mean vs sim)", "",
          "| test.feature | real | baseline sim | calibrated sim | z baseline | z calibrated |",
          "|---|---|---|---|---|---|"]
    for test, target in targets.items():
        for feature, value in target["features"].items():
            report_lines.append(f"| {test}.{feature} | {value['mean']:.4f} ± {value['sigma']:.4f} | "
                     f"{baseline['feats'].get(test, {}).get(feature, float('nan')):.4f} | "
                     f"{calibrated['feats'].get(test, {}).get(feature, float('nan')):.4f} | "
                     f"{baseline['z'].get(f'{test}.{feature}')} | {calibrated['z'].get(f'{test}.{feature}')} |")
    report_lines += ["", "Path errors are start-aligned and averaged over the real repeats. "
          "`<test>/paths_map.png|mp4`: real (green) vs baseline sim (red) vs calibrated sim (cyan) "
          "on the overhead camera view."]
    (out_dir / "report.md").write_text("\n".join(report_lines) + "\n")
    (out_dir / "results.json").write_text(json.dumps(
        {key: {field: field_value for field, field_value in value.items() if field != "csvs"} for key, value in results.items()},
        indent=2, default=float))
    log(f"[verify] rms_z baseline {baseline['rms_z']:.2f} -> calibrated {calibrated['rms_z']:.2f}; "
        f"report {out_dir/'report.md'}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
