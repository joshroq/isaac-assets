#!/usr/bin/env python3
"""Staged sim->real calibration optimiser for the baked skid-steer rover.

Closes the loop in PARAMETER_MAP.md: drive the SAME command sequences on the real rover
(camera ground truth) and in sim, extract per-maneuver features, and search the SIM
parameters so the sim features match the real ones — IN STAGES, so coupled knobs are
identified together. All non-geometric motion parameters are available to the Isaac search;
the reduced analytic model only warm-starts parameters it represents.

Wheel friction, drive damping and chassis damping are searched even where sensitivity is
weak. Static/passive-wheel friction and damping require Isaac for a meaningful fit.

GEOMETRY IS LOCKED. Wheel radius, track, wheelbase, wheel placement and collider shapes are
never parameters (calib_params refuses to author them); wheel_distance is the only
geometric-looking quantity searched, because it is a controller turning gain. Masses are
MEASURED (--measure-mass), never searched. Floor friction is searched only inside the
literature rubber-on-floor band of the named --surface (calib_params.SURFACE_FRICTION).

Backends:
  analytic : optimize/forward_model.py (no Isaac) — fast; warm-start + machinery self-test.
  isaac    : launch headless Isaac per trial (optimize/sim_eval.py) — the real calibration.
Default flow: warm-start each stage on the analytic model (only the keys it actually responds
to), then (isaac backend) locally refine all stage keys against Isaac.

Typical use:
  # 0. one-time: enter measured masses (geometry is never entered)
  python3 calibrate.py --measure-mass
  # 1. collect >= 5 real repeats per test, build targets.yaml (build_targets.py), then:
  python3 calibrate.py --surface heavy_carpet --targets targets/heavy_carpet.yaml --backend isaac
  # validate the machinery with no Isaac / no real data:
  python3 calibrate.py --self-test
"""
from __future__ import annotations

import argparse
import math
import sys
from pathlib import Path

import numpy as np

HERE = Path(__file__).resolve().parent
sys.path.insert(0, str(HERE.parents[1] / "ugv_controller_v2" / "calibration"))
import calib_params as C            # noqa: E402  single source of truth (pure Python)
import forward_model as fm          # noqa: E402
import features as F                # noqa: E402

try:
    import yaml
except Exception:
    yaml = None

# ── Stage schedule. Each stage: the free keys and, per test (sequence stem in
# parallel/sequences/), the features that identify them. Stages run in order; later stages
# keep the earlier stages' features in their loss at CARRY_WEIGHT so they cannot undo them.
STAGES = [
    dict(id=2, name="drivetrain-ramps",
         keys=["max_acceleration", "max_deceleration", "max_angular_acceleration", "drive_damping"],
         tests={"T1_straight": ["tau_v", "v_ss", "coast_time", "coast_dist"],
                "T5_stop": ["coast_time", "coast_dist"],
                "T2_spin": ["tau_omega", "spin_coast_angle"]}),
    dict(id=3, name="speed-clamps", keys=["max_linear_speed", "max_angular_speed"],
         tests={"T3b_top_speed": ["v_ss_max"], "T6_spin_sweep": ["omega_max"]},
         readoff="top_speed"),
    dict(id=4, name="turning",
         keys=["wheel_distance", "corner_mu_dynamic", "corner_mu_static",
               "middle_mu_dynamic", "middle_mu_static",
               "floor_mu_dynamic", "floor_mu_static", "drive_max_force"],
         tests={"T1_straight": ["v_ss", "tau_v"],
                "T3_speed_sweep": ["v_ss_s0", "v_ss_s1", "v_ss_s2", "v_ss_s3"],
                "T2_spin": ["omega_mean", "dyaw", "turn_walk"],
                "T4_arc": ["arc_radius"],
                "T6_spin_sweep": ["omega_s0", "omega_s1", "omega_s2", "omega_s3"]}),
    dict(id=5, name="chassis-damping",
         keys=["base_linear_damping", "base_angular_damping"],
         tests={"T1_straight": ["v_ss", "coast_time", "coast_dist"],
                "T3_speed_sweep": ["v_ss_s0", "v_ss_s1", "v_ss_s2", "v_ss_s3"],
                "T5_stop": ["coast_time", "coast_dist"],
                "T2_spin": ["omega_mean", "spin_coast_angle", "spin_coast_time"],
                "T4_arc": ["arc_radius"],
                "T6_spin_sweep": ["spin_coast_angle", "spin_coast_time"]}),
]
CARRY_WEIGHT = 0.25
MISSING_PENALTY = 25.0          # missing feature = 5 sigma (keeps the search in a valid region)
INFEASIBLE_PENALTY = float("inf")  # an invalid material must never beat a large valid loss

# Isaac's own run-to-run scatter (relative std over 7 null runs, 2026-10-14 sweep). The sim
# can't be fit tighter than its own noise, so sigma_eff = hypot(sigma_real, rel * |target|).
# Features not listed here (new latency-corrected ones) have no measured sim noise yet.
SIM_NOISE_REL = {"v_ss": 0.02, "dist": 0.02, "omega_ss": 0.15, "dyaw": 0.07,
                 "turn_walk": 0.55, "arc_radius": 0.05, "v_k": 0.03, "w_k": 0.10}


# Isaac's own run-to-run noise for a feature, as a fraction of its value (added to
# the real sigma so the optimiser doesn't chase sim noise).
def _sim_noise(feature):
    if feature.startswith(("v_k", "w_k")):
        return SIM_NOISE_REL[feature[:3]]
    base_name = feature.split("_s")[0] if feature.startswith(("v_ss_s", "omega_s")) else feature
    return SIM_NOISE_REL.get(feature, SIM_NOISE_REL.get(base_name, 0.0))


# ── search-space transforms (log for multi-decade knobs; floor mu per surface) ──
# Search bounds per key in OPTIMISER coordinates (log10 for log-scale keys).
def _bounds(keys, surface=None):
    search_bounds = []
    for key in keys:
        low, high = C.bounds(key, surface)
        search_bounds.append((math.log10(low), math.log10(high)) if C.SCHEMA_BY_KEY[key]["scale"] == "log" else (low, high))
    return search_bounds


# Optimiser vector -> {key: value} (undoing log10 for log-scale keys).
def _to_theta(keys, x):
    theta = {}
    for key, coordinate in zip(keys, np.atleast_1d(x)):
        theta[key] = float(10 ** coordinate) if C.SCHEMA_BY_KEY[key]["scale"] == "log" else float(coordinate)
    return theta


# Current values -> optimiser vector, clipped into the bounds.
def _x0(keys, theta, surface=None):
    coords = []
    for key, (low, high) in zip(keys, _bounds(keys, surface)):
        schema = C.SCHEMA_BY_KEY[key]
        value = float(theta.get(key, schema.get("start", schema["default"])))
        if value <= 0 and "start" in schema:          # 'unset' (0) ramp limit -> start at the loose end
            value = schema["start"]
        coordinate = math.log10(max(value, 1e-9)) if schema["scale"] == "log" else value
        coords.append(min(max(coordinate, low), high))
    return np.array(coords)


# Initial parameter set (see docstring), clipped into this surface's bounds and with
# each dynamic friction <= its static partner.
def start_theta(measured, surface=None, seed_theta=None):
    """Initial theta: measured masses + every searchable key at its start value; floor friction
    at the middle of the surface's literature band; fixed keys at their schema value."""
    theta = dict(measured)
    for key in C.OPTIMISABLE + C.FIXED:
        schema = C.SCHEMA_BY_KEY[key]
        theta.setdefault(key, schema.get("start", schema["default"]))
    theta.update(C.surface_start(surface))
    if seed_theta:
        theta.update({key: value for key, value in seed_theta.items() if key in C.OPTIMISABLE})
    for key in C.OPTIMISABLE:      # defaults and prior seeds must lie in the search bounds
        low, high = C.bounds(key, surface)
        theta[key] = min(max(float(theta[key]), low), high)
    for static, dynamic in C.FRICTION_PAIRS:
        theta[dynamic] = min(theta[dynamic], theta[static])
    C.validate(theta, surface)
    return theta


# ── objective: sigma-weighted feature mismatch over one or more tests ─────────
# Staged-fit loss: sum over terms of weight * z^2 per target feature (see `score` in
# loop/closed_loop.py), predicted by the analytic model or by an Isaac run.
class Objective:
    """terms = [(test_name, steps, {feature: {mean, sigma}}, weight)]."""

    def __init__(self, base_theta, keys, terms, backend, combine_mode, surface):
        self.base = dict(base_theta)
        self.keys = keys
        self.terms = terms
        self.backend = backend
        self.combine_mode = combine_mode
        self.surface = surface
        self.n_eval = 0
        self.enforce_mu_order = True

    # Features of one sequence under theta (analytic model or one Isaac launch).
    def predict(self, theta, steps):
        if self.backend == "analytic":
            track = fm.simulate(theta, steps, combine_mode=self.combine_mode)
            return F.extract(np.array(track["t"]), np.array(track["x"]), np.array(track["y"]),
                             np.array(track["yaw"]), steps)
        import sim_eval
        csv_path = sim_eval.run_trial(theta, steps, self.surface, self.combine_mode)
        return F.extract(*F.load_csv(csv_path), steps)

    # Loss of one optimiser vector; INFEASIBLE_PENALTY for an invalid material set.
    def __call__(self, x):
        self.n_eval += 1
        theta = dict(self.base)
        theta.update(_to_theta(self.keys, x))
        if self.enforce_mu_order:
            try:
                C.validate(theta, self.surface)
            except ValueError:
                return INFEASIBLE_PENALTY
        loss = 0.0
        for _, steps, targets, weight in self.terms:
            predicted = self.predict(theta, steps)
            for feature, target in targets.items():
                if feature in predicted and np.isfinite(predicted[feature]):
                    sigma = math.hypot(max(target["sigma"], 1e-6), _sim_noise(feature) * abs(target["mean"]))
                    loss += weight * ((predicted[feature] - target["mean"]) / sigma) ** 2
                else:
                    loss += weight * MISSING_PENALTY
        return loss


# Global search: bounded Brent for one key, differential evolution for several.
def optimize_stage(obj, keys, max_evals, seed=0, surface=None):
    from scipy.optimize import differential_evolution, minimize_scalar
    bounds = _bounds(keys, surface)
    if len(keys) == 1:
        low, high = bounds[0]
        result = minimize_scalar(lambda xi: obj([xi]), bounds=(low, high), method="bounded",
                              options={"maxiter": max_evals, "xatol": 1e-3})
        return _to_theta(keys, [result.x]), float(result.fun)
    result = differential_evolution(obj, bounds, maxiter=max_evals, popsize=8, tol=1e-3,
                                 seed=seed, polish=True, init="sobol")
    return _to_theta(keys, result.x), float(result.fun)


# Wide starting simplex for Nelder-Mead (see docstring).
def initial_simplex(keys, x0, base_theta, surface=None):
    """Span each bounded coordinate, including friction below the floor and low drive gain.

    SciPy's default 5% simplex stays on the flat min-friction plateau at the wheel defaults.
    Explore toward the farther bound (staying inside it to avoid simplex collapse), and
    explicitly cross below the floor for wheel coefficients on the plateau. Repair
    static/dynamic pairs within the free keys rather than penalising the initial vertices.
    """
    bounds = _bounds(keys, surface)
    key_index = {k: axis for axis, k in enumerate(keys)}

    def feasible(x):
        x = np.clip(np.array(x, dtype=float), *np.array(bounds).T)
        theta = dict(base_theta)
        theta.update(_to_theta(keys, x))
        for static, dynamic in C.FRICTION_PAIRS:
            if theta[dynamic] <= theta[static]:
                continue
            if dynamic in key_index:
                axis = key_index[dynamic]
                x[axis] = np.clip(theta[static], *bounds[axis])
                theta[dynamic] = x[axis]
            if static in key_index and theta[dynamic] > theta[static]:
                axis = key_index[static]
                x[axis] = np.clip(theta[dynamic], *bounds[axis])
                theta[static] = x[axis]
        return x

    start = feasible(x0)
    theta = dict(base_theta)
    theta.update(_to_theta(keys, start))
    vertices = [start]
    for axis, (low, high) in enumerate(bounds):
        vertex = start.copy()
        far_bound = low if start[axis] - low >= high - start[axis] else high
        vertex[axis] = start[axis] + .9 * (far_bound - start[axis])
        key = keys[axis]
        if key.startswith(("corner_mu_", "middle_mu_")):
            floor_mu = theta["floor_mu_" + key.rsplit("_", 1)[1]]
            if theta[key] >= floor_mu:
                vertex[axis] = np.clip(.8 * floor_mu, low, high)
        vertices.append(feasible(vertex))
    return np.array(vertices)


# Local Nelder-Mead against Isaac with bound reflection (see docstring).
def refine_isaac(obj, keys, x0, max_evals, surface=None):
    """Local Nelder-Mead refine against the (expensive) Isaac backend from a warm start.
    Reflect candidates at their bounds: clipping collapses the simplex when a coefficient
    starts at zero, and can leave that coefficient stuck there for the entire search.
    Every evaluated candidate remains inside its bounds, including the floor-friction band.
    """
    from scipy.optimize import minimize
    bounds = _bounds(keys, surface)
    lower, upper = np.array(bounds).T
    span = upper - lower

    def reflected(x):
        return lower + span - np.abs((np.asarray(x) - lower) % (2 * span) - span)

    result = minimize(lambda x: obj(reflected(x)), x0, method="Nelder-Mead",
                   options={"maxfev": max_evals, "xatol": 1e-3, "fatol": 1e-2,
                            "initial_simplex": initial_simplex(keys, x0, obj.base, surface)})
    x_reflected = reflected(result.x)
    return _to_theta(keys, x_reflected), float(result.fun)


# Keys the analytic model actually responds to (see docstring).
def analytic_sensitive_keys(theta, keys, terms, combine_mode, surface):
    """Keys whose lo->hi sweep changes the ANALYTIC loss. The analytic model omits static
    friction, floor-mu scrub and torque-limited turning, so warm-starting those on it would
    move them on model artefacts; they start from their current value instead."""
    sensitive = []
    for key, (low, high) in zip(keys, _bounds(keys, surface)):
        if C.SCHEMA_BY_KEY[key].get("isaac_only"):
            continue
        losses = []
        for coordinate in (low, high):
            probe = Objective(theta, [key], terms, "analytic", combine_mode, surface)
            probe.enforce_mu_order = False      # probe the model, not the feasibility penalty
            losses.append(probe([coordinate]))
        if abs(losses[0] - losses[1]) > 1e-6 * max(1.0, abs(losses[0])):
            sensitive.append(key)
    return sensitive


# ── measured constants (masses). Geometry is LOCKED and never entered here. ──
# Measured masses from measured.yaml (schema defaults otherwise); geometry ignored.
def load_measured(path):
    theta = {key: C.SCHEMA_BY_KEY[key]["default"] for key in C.MEASURED}
    if path and Path(path).exists() and yaml:
        measured_doc = yaml.safe_load(Path(path).read_text()) or {}
        theta.update({key: value for key, value in C.flatten(measured_doc).items() if key in C.MEASURED})
        theta.update({key: value for key, value in measured_doc.items() if key in C.MEASURED})
        ignored = [key for key in list(C.flatten(measured_doc)) + list(measured_doc) if key in C.LOCKED]
        if ignored:
            print(f"[calibrate] ignoring locked geometry in {path}: {sorted(set(ignored))}")
    return {key: theta[key] for key in C.MEASURED}


# Prompt for the real rover's masses and save them to measured.yaml.
def measure_mass(path):
    print("\n=== Enter the REAL rover's measured masses (never optimised) ===")
    print("Geometry (wheel radius, track, wheelbase) is locked in the USD and is not entered.")
    print("Press Enter to keep the current/default value shown in [brackets].\n")
    masses = {}
    for key in C.MEASURED:
        schema = C.SCHEMA_BY_KEY[key]
        default_value = schema["default"]
        try:
            typed = input(f"  {key:12s} [{default_value}]  ({schema['note'][:60]}...): ").strip()
        except EOFError:
            typed = ""
        masses[key] = float(typed) if typed else default_value
    if yaml and path:
        Path(path).write_text(yaml.safe_dump(masses, sort_keys=False))
        print(f"\nwrote {path}")
    return masses


# ── targets / sequences ──────────────────────────────────────────────────────
# Sequence given as a yaml path or a step list -> step list with defaults filled.
def _as_steps(seq):
    if isinstance(seq, str):
        return F.load_sequence(seq if Path(seq).is_absolute() else str(HERE / seq))
    for step in seq:
        step.setdefault("linear", 0.0); step.setdefault("angular", 0.0); step.setdefault("duration", 0.0)
    return seq


# targets.yaml with each test's sequence loaded as steps.
def load_targets(path):
    targets_doc = yaml.safe_load(Path(path).read_text())
    for name, test in targets_doc.get("tests", {}).items():
        test["steps"] = _as_steps(test["sequence"])
    return targets_doc


# Objective terms of one stage: (test, steps, its stage features, weight).
def _stage_terms(stage, tests, weight=1.0):
    terms = []
    for name, feature_names in stage["tests"].items():
        test = tests.get(name)
        if not test:
            continue
        test_features = {feature: test["features"][feature] for feature in feature_names if feature in test.get("features", {})}
        if test_features:
            terms.append((name, test["steps"], test_features, weight))
    return terms


# ── driver ────────────────────────────────────────────────────────────────────
# The staged fit: for each stage, fit only its keys on its own tests plus the earlier
# stages' terms at CARRY_WEIGHT (so later stages can't undo earlier fits). Analytic
# warm start for the keys the model represents, then Isaac Nelder-Mead (backend
# 'isaac') or the analytic result. Speed clamps are read straight off the targets.
def run_calibration(targets, measured, backend, combine_mode, surface, max_evals, seed, log,
                    only_stages=None, seed_theta=None):
    theta = start_theta(measured, surface, seed_theta)
    floor_band = C.surface_band(surface)
    log(f"[surface {surface}] floor mu bands: static {floor_band['static']}, dynamic {floor_band['dynamic']}"
        f"; start static {theta['floor_mu_static']:.3f}, dynamic {theta['floor_mu_dynamic']:.3f}")
    tests = targets.get("tests", {})
    results = {}
    carried = []                                     # earlier stages' terms (down-weighted)
    for stage in STAGES:
        if only_stages and stage["id"] not in only_stages:
            continue
        keys = C.stage_keys(stage["id"])
        if not keys:
            continue
        # read-off stages (speed clamps) set directly from measured top speeds
        if stage.get("readoff") and targets.get(stage["readoff"]):
            top_speeds = targets[stage["readoff"]]
            if "linear" in top_speeds:
                theta["max_linear_speed"] = float(top_speeds["linear"])
            if "angular" in top_speeds:
                theta["max_angular_speed"] = float(top_speeds["angular"])
            log(f"[stage {stage['id']} {stage['name']}] read-off: "
                f"max_linear={theta['max_linear_speed']:.3f} max_angular={theta['max_angular_speed']:.3f}")
            results[stage["name"]] = {key: theta[key] for key in ("max_linear_speed", "max_angular_speed")}
            continue
        own_terms = _stage_terms(stage, tests)
        if not own_terms:
            log(f"[stage {stage['id']} {stage['name']}] SKIP (none of {list(stage['tests'])} with their "
                f"features in targets)")
            continue
        terms = own_terms + [(term_name, term_steps, term_features, CARRY_WEIGHT) for term_name, term_steps, term_features, _ in carried]
        # warm-start on the analytic model, but only the keys it responds to
        warm_keys = analytic_sensitive_keys(theta, keys, own_terms, combine_mode, surface)
        warm, warm_loss, warm_evals = {}, float("nan"), 0
        if warm_keys:
            warm_obj = Objective(theta, warm_keys, terms, "analytic", combine_mode, surface)
            warm, warm_loss = optimize_stage(warm_obj, warm_keys, max_evals=40, seed=seed, surface=surface)
            warm_evals = warm_obj.n_eval
        cold_keys = [key for key in keys if key not in warm_keys]
        if backend == "isaac":
            warm_theta = dict(theta); warm_theta.update(warm)
            objective = Objective(theta, keys, terms, "isaac", combine_mode, surface)
            best_values, best_loss = refine_isaac(objective, keys, _x0(keys, warm_theta, surface), max_evals, surface)
            log(f"[stage {stage['id']} {stage['name']}] warm L={warm_loss:.2f} ({warm_evals} analytic evals on "
                f"{warm_keys}; not modelled: {cold_keys}) -> isaac L={best_loss:.2f} ({objective.n_eval} evals): "
                + ", ".join(f"{key}={best_values[key]:.4g}" for key in keys))
        else:
            best_values = {key: warm.get(key, _to_theta([key], _x0([key], theta, surface))[key]) for key in keys}
            best_loss = warm_loss
            log(f"[stage {stage['id']} {stage['name']}] L={best_loss:.2f} ({warm_evals} evals): "
                + ", ".join(f"{key}={best_values[key]:.4g}" for key in keys)
                + (f"  (no analytic warm start for {cold_keys}: left at start)" if cold_keys else ""))
        theta.update(best_values)
        C.validate(theta, surface)
        results[stage["name"]] = {key: float(best_values[key]) for key in keys} | {"loss": best_loss}
        carried += own_terms
    return theta, results


# Write surfaces/<surface>.yaml: bands, measured / calibrated / fixed values, stage
# results and the ready-to-bake nested sim_calibration_yaml block.
def write_surface(path, surface, theta, results, combine_mode):
    doc = {"surface": surface, "combine_mode": combine_mode,
           "floor_friction_band": {key: list(value) for key, value in C.surface_band(surface).items()},
           "measured": {key: float(theta[key]) for key in C.MEASURED},
           "calibrated": {key: float(theta[key]) for key in C.OPTIMISABLE},
           "fixed": {key: float(theta[key]) for key in C.FIXED},
           "stage_results": results,
           "sim_calibration_yaml": C.nest({key: theta[key] for key in theta if key not in C.LOCKED})}
    if yaml and path:
        Path(path).write_text(yaml.safe_dump(doc, sort_keys=False))
    return doc


# ── self-test: recover a known theta from synthetic targets (no Isaac) ───────
# Recover known parameters from synthetic analytic targets (within 15 %).
def self_test():
    print("=== self-test: recover known parameters from synthetic targets (analytic) ===")
    sequences = {name: F.load_sequence(str(HERE.parent / "parallel" / "sequences" / f"{name}.yaml"))
            for name in ("T1_straight", "T2_spin", "T4_arc", "T5_stop", "T6_spin_sweep",
                         "T3b_top_speed")}
    # only knobs the analytic model represents (static mu / floor-mu scrub / torque-limited
    # turning are Isaac-only and are deliberately not warm-started)
    theta_true = {"wheel_distance": 0.68, "max_acceleration": 0.8, "max_deceleration": 1.5,
                  "max_angular_acceleration": 3.0}
    surface = "concrete"
    measured = {key: C.SCHEMA_BY_KEY[key]["default"] for key in C.MEASURED}
    full_true = start_theta(measured, surface)
    full_true.update(theta_true)
    targets = {"surface": "synthetic", "combine_mode": "min",
               "top_speed": {"linear": 0.42, "angular": 1.9}, "tests": {}}
    for name, steps in sequences.items():
        track = fm.simulate(full_true, steps, combine_mode="min")
        features = F.extract(np.array(track["t"]), np.array(track["x"]), np.array(track["y"]),
                      np.array(track["yaw"]), steps)
        targets["tests"][name] = {"steps": steps, "sequence": steps,
                                  "features": {key: {"mean": value, "sigma": max(0.02 * abs(value), 1e-3)}
                                               for key, value in features.items() if np.isfinite(value)}}
    theta, results = run_calibration(targets, measured, "analytic", "min", surface,
                                     max_evals=40, seed=1, log=print)
    print("\nrecovery (key: true -> found):")
    ok = True
    for key, true_value in sorted(theta_true.items()):
        found_value = theta[key]
        rel_error = abs(found_value - true_value) / max(abs(true_value), 1e-6)
        status = "OK " if rel_error < 0.15 else "OFF"
        ok &= rel_error < 0.15
        print(f"  {status} {key:26s} {true_value:8.4g} -> {found_value:8.4g}  ({rel_error*100:4.1f}%)")
    floor_band = C.surface_band(surface)
    in_band = (floor_band["static"][0] <= theta["floor_mu_static"] <= floor_band["static"][1]
              and floor_band["dynamic"][0] <= theta["floor_mu_dynamic"] <= floor_band["dynamic"][1])
    print(f"  {'OK ' if in_band else 'OFF'} floor mu stayed in the {surface} band "
          f"(static {theta['floor_mu_static']:.3f}, dynamic {theta['floor_mu_dynamic']:.3f})")
    print(f"  {'OK ' if theta['max_linear_speed'] == 0.42 else 'OFF'} speed clamp read-off")
    ok &= in_band and theta["max_linear_speed"] == 0.42
    print("\nSELF-TEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


# Command line for the staged fit (targets.yaml -> surfaces/<surface>.yaml).
def main():
    parser = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    parser.add_argument("--surface", default="surface",
                    help=f"floor material; sets the floor-mu band ({', '.join(C.SURFACE_FRICTION)} "
                         f"or an alias; unknown names use the all-materials envelope)")
    parser.add_argument("--targets", help="targets YAML (build_targets.py output)")
    parser.add_argument("--backend", choices=["analytic", "isaac"], default="analytic")
    parser.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    parser.add_argument("--measured", "--geometry", dest="measured", default=str(HERE / "measured.yaml"),
                    help="measured masses yaml (geometry entries are ignored: geometry is locked)")
    parser.add_argument("--measure-mass", "--measure-geometry", dest="measure_mass", action="store_true")
    parser.add_argument("--no-prompt-geometry", action="store_true")
    parser.add_argument("--max-evals", type=int, default=25, help="per-stage Isaac refine budget")
    parser.add_argument("--seed", type=int, default=0)
    parser.add_argument("--out", help="write surfaces/<surface>.yaml here")
    parser.add_argument("--stages", help="comma list of stage ids to run, e.g. '2,3' on concrete "
                    "then '4,5' per surface (default: all of 2,3,4,5)")
    parser.add_argument("--base", help="seed calibrated knobs from a prior surfaces/*.yaml "
                    "(the concrete reference for stages 2-3) before running")
    parser.add_argument("--self-test", action="store_true")
    args = parser.parse_args()

    if args.self_test:
        return self_test()
    if not yaml:
        sys.exit("PyYAML required for the real workflow (pip install pyyaml).")
    if args.measure_mass:
        measured = measure_mass(args.measured)
    else:
        measured = load_measured(args.measured)
        if not Path(args.measured).exists() and not args.no_prompt_geometry and sys.stdin.isatty():
            if input(f"No {args.measured}. Enter measured masses now? [y/N] ").strip().lower() == "y":
                measured = measure_mass(args.measured)
    if not args.targets:
        sys.exit("--targets is required (or use --self-test).")
    targets = load_targets(args.targets)
    combine_mode = targets.get("combine_mode", args.combine_mode)
    only_stages = {int(stage_text) for stage_text in args.stages.split(",")} if args.stages else None
    seed_theta = None
    if args.base and Path(args.base).exists():
        base_doc = yaml.safe_load(Path(args.base).read_text()) or {}
        seed_theta = base_doc.get("calibrated", {})
    theta, results = run_calibration(targets, measured, args.backend, combine_mode, args.surface,
                                     args.max_evals, args.seed, log=print,
                                     only_stages=only_stages, seed_theta=seed_theta)
    out_path = args.out or str(HERE.parents[0] / "surfaces" / f"{args.surface}.yaml")
    Path(out_path).parent.mkdir(parents=True, exist_ok=True)
    write_surface(out_path, args.surface, theta, results, combine_mode)
    print(f"\nwrote {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
