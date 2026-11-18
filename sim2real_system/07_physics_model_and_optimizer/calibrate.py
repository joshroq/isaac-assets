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


def _sim_noise(feature):
    if feature.startswith(("v_k", "w_k")):
        return SIM_NOISE_REL[feature[:3]]
    base = feature.split("_s")[0] if feature.startswith(("v_ss_s", "omega_s")) else feature
    return SIM_NOISE_REL.get(feature, SIM_NOISE_REL.get(base, 0.0))


# ── search-space transforms (log for multi-decade knobs; floor mu per surface) ──
def _bounds(keys, surface=None):
    b = []
    for k in keys:
        lo, hi = C.bounds(k, surface)
        b.append((math.log10(lo), math.log10(hi)) if C.SCHEMA_BY_KEY[k]["scale"] == "log" else (lo, hi))
    return b


def _to_theta(keys, x):
    out = {}
    for k, xi in zip(keys, np.atleast_1d(x)):
        out[k] = float(10 ** xi) if C.SCHEMA_BY_KEY[k]["scale"] == "log" else float(xi)
    return out


def _x0(keys, theta, surface=None):
    x = []
    for k, (lo, hi) in zip(keys, _bounds(keys, surface)):
        s = C.SCHEMA_BY_KEY[k]
        v = float(theta.get(k, s.get("start", s["default"])))
        if v <= 0 and "start" in s:          # 'unset' (0) ramp limit -> start at the loose end
            v = s["start"]
        xi = math.log10(max(v, 1e-9)) if s["scale"] == "log" else v
        x.append(min(max(xi, lo), hi))
    return np.array(x)


def start_theta(measured, surface=None, seed_theta=None):
    """Initial theta: measured masses + every searchable key at its start value; floor friction
    at the middle of the surface's literature band; fixed keys at their schema value."""
    theta = dict(measured)
    for k in C.OPTIMISABLE + C.FIXED:
        s = C.SCHEMA_BY_KEY[k]
        theta.setdefault(k, s.get("start", s["default"]))
    theta.update(C.surface_start(surface))
    if seed_theta:
        theta.update({k: v for k, v in seed_theta.items() if k in C.OPTIMISABLE})
    for k in C.OPTIMISABLE:      # defaults and prior seeds must lie in the search bounds
        lo, hi = C.bounds(k, surface)
        theta[k] = min(max(float(theta[k]), lo), hi)
    for static, dynamic in C.FRICTION_PAIRS:
        theta[dynamic] = min(theta[dynamic], theta[static])
    C.validate(theta, surface)
    return theta


# ── objective: sigma-weighted feature mismatch over one or more tests ─────────
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

    def predict(self, theta, steps):
        if self.backend == "analytic":
            tr = fm.simulate(theta, steps, combine_mode=self.combine_mode)
            return F.extract(np.array(tr["t"]), np.array(tr["x"]), np.array(tr["y"]),
                             np.array(tr["yaw"]), steps)
        import sim_eval
        csv = sim_eval.run_trial(theta, steps, self.surface, self.combine_mode)
        return F.extract(*F.load_csv(csv), steps)

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
            got = self.predict(theta, steps)
            for f, t in targets.items():
                if f in got and np.isfinite(got[f]):
                    sigma = math.hypot(max(t["sigma"], 1e-6), _sim_noise(f) * abs(t["mean"]))
                    loss += weight * ((got[f] - t["mean"]) / sigma) ** 2
                else:
                    loss += weight * MISSING_PENALTY
        return loss


def optimize_stage(obj, keys, max_evals, seed=0, surface=None):
    from scipy.optimize import differential_evolution, minimize_scalar
    bounds = _bounds(keys, surface)
    if len(keys) == 1:
        lo, hi = bounds[0]
        res = minimize_scalar(lambda xi: obj([xi]), bounds=(lo, hi), method="bounded",
                              options={"maxiter": max_evals, "xatol": 1e-3})
        return _to_theta(keys, [res.x]), float(res.fun)
    res = differential_evolution(obj, bounds, maxiter=max_evals, popsize=8, tol=1e-3,
                                 seed=seed, polish=True, init="sobol")
    return _to_theta(keys, res.x), float(res.fun)


def initial_simplex(keys, x0, base_theta, surface=None):
    """Span each bounded coordinate, including friction below the floor and low drive gain.

    SciPy's default 5% simplex stays on the flat min-friction plateau at the wheel defaults.
    Explore toward the farther bound (staying inside it to avoid simplex collapse), and
    explicitly cross below the floor for wheel coefficients on the plateau. Repair
    static/dynamic pairs within the free keys rather than penalising the initial vertices.
    """
    bounds = _bounds(keys, surface)
    index = {k: i for i, k in enumerate(keys)}

    def feasible(x):
        x = np.clip(np.array(x, dtype=float), *np.array(bounds).T)
        theta = dict(base_theta)
        theta.update(_to_theta(keys, x))
        for static, dynamic in C.FRICTION_PAIRS:
            if theta[dynamic] <= theta[static]:
                continue
            if dynamic in index:
                i = index[dynamic]
                x[i] = np.clip(theta[static], *bounds[i])
                theta[dynamic] = x[i]
            if static in index and theta[dynamic] > theta[static]:
                i = index[static]
                x[i] = np.clip(theta[dynamic], *bounds[i])
                theta[static] = x[i]
        return x

    start = feasible(x0)
    theta = dict(base_theta)
    theta.update(_to_theta(keys, start))
    vertices = [start]
    for i, (lo, hi) in enumerate(bounds):
        vertex = start.copy()
        endpoint = lo if start[i] - lo >= hi - start[i] else hi
        vertex[i] = start[i] + .9 * (endpoint - start[i])
        key = keys[i]
        if key.startswith(("corner_mu_", "middle_mu_")):
            floor = theta["floor_mu_" + key.rsplit("_", 1)[1]]
            if theta[key] >= floor:
                vertex[i] = np.clip(.8 * floor, lo, hi)
        vertices.append(feasible(vertex))
    return np.array(vertices)


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

    res = minimize(lambda x: obj(reflected(x)), x0, method="Nelder-Mead",
                   options={"maxfev": max_evals, "xatol": 1e-3, "fatol": 1e-2,
                            "initial_simplex": initial_simplex(keys, x0, obj.base, surface)})
    xc = reflected(res.x)
    return _to_theta(keys, xc), float(res.fun)


def analytic_sensitive_keys(theta, keys, terms, combine_mode, surface):
    """Keys whose lo->hi sweep changes the ANALYTIC loss. The analytic model omits static
    friction, floor-mu scrub and torque-limited turning, so warm-starting those on it would
    move them on model artefacts; they start from their current value instead."""
    out = []
    for k, (lo, hi) in zip(keys, _bounds(keys, surface)):
        if C.SCHEMA_BY_KEY[k].get("isaac_only"):
            continue
        losses = []
        for xi in (lo, hi):
            obj = Objective(theta, [k], terms, "analytic", combine_mode, surface)
            obj.enforce_mu_order = False      # probe the model, not the feasibility penalty
            losses.append(obj([xi]))
        if abs(losses[0] - losses[1]) > 1e-6 * max(1.0, abs(losses[0])):
            out.append(k)
    return out


# ── measured constants (masses). Geometry is LOCKED and never entered here. ──
def load_measured(path):
    theta = {k: C.SCHEMA_BY_KEY[k]["default"] for k in C.MEASURED}
    if path and Path(path).exists() and yaml:
        data = yaml.safe_load(Path(path).read_text()) or {}
        theta.update({k: v for k, v in C.flatten(data).items() if k in C.MEASURED})
        theta.update({k: v for k, v in data.items() if k in C.MEASURED})
        ignored = [k for k in list(C.flatten(data)) + list(data) if k in C.LOCKED]
        if ignored:
            print(f"[calibrate] ignoring locked geometry in {path}: {sorted(set(ignored))}")
    return {k: theta[k] for k in C.MEASURED}


def measure_mass(path):
    print("\n=== Enter the REAL rover's measured masses (never optimised) ===")
    print("Geometry (wheel radius, track, wheelbase) is locked in the USD and is not entered.")
    print("Press Enter to keep the current/default value shown in [brackets].\n")
    out = {}
    for k in C.MEASURED:
        s = C.SCHEMA_BY_KEY[k]
        cur = s["default"]
        try:
            raw = input(f"  {k:12s} [{cur}]  ({s['note'][:60]}...): ").strip()
        except EOFError:
            raw = ""
        out[k] = float(raw) if raw else cur
    if yaml and path:
        Path(path).write_text(yaml.safe_dump(out, sort_keys=False))
        print(f"\nwrote {path}")
    return out


# ── targets / sequences ──────────────────────────────────────────────────────
def _as_steps(seq):
    if isinstance(seq, str):
        return F.load_sequence(seq if Path(seq).is_absolute() else str(HERE / seq))
    for s in seq:
        s.setdefault("linear", 0.0); s.setdefault("angular", 0.0); s.setdefault("duration", 0.0)
    return seq


def load_targets(path):
    data = yaml.safe_load(Path(path).read_text())
    for name, t in data.get("tests", {}).items():
        t["steps"] = _as_steps(t["sequence"])
    return data


def _stage_terms(stage, tests, weight=1.0):
    terms = []
    for name, feats in stage["tests"].items():
        test = tests.get(name)
        if not test:
            continue
        tf = {f: test["features"][f] for f in feats if f in test.get("features", {})}
        if tf:
            terms.append((name, test["steps"], tf, weight))
    return terms


# ── driver ────────────────────────────────────────────────────────────────────
def run_calibration(targets, measured, backend, combine_mode, surface, max_evals, seed, log,
                    only_stages=None, seed_theta=None):
    theta = start_theta(measured, surface, seed_theta)
    band = C.surface_band(surface)
    log(f"[surface {surface}] floor mu bands: static {band['static']}, dynamic {band['dynamic']}"
        f"; start static {theta['floor_mu_static']:.3f}, dynamic {theta['floor_mu_dynamic']:.3f}")
    tests = targets.get("tests", {})
    results = {}
    carried = []                                     # earlier stages' terms (down-weighted)
    for st in STAGES:
        if only_stages and st["id"] not in only_stages:
            continue
        keys = C.stage_keys(st["id"])
        if not keys:
            continue
        # read-off stages (speed clamps) set directly from measured top speeds
        if st.get("readoff") and targets.get(st["readoff"]):
            top = targets[st["readoff"]]
            if "linear" in top:
                theta["max_linear_speed"] = float(top["linear"])
            if "angular" in top:
                theta["max_angular_speed"] = float(top["angular"])
            log(f"[stage {st['id']} {st['name']}] read-off: "
                f"max_linear={theta['max_linear_speed']:.3f} max_angular={theta['max_angular_speed']:.3f}")
            results[st["name"]] = {k: theta[k] for k in ("max_linear_speed", "max_angular_speed")}
            continue
        own = _stage_terms(st, tests)
        if not own:
            log(f"[stage {st['id']} {st['name']}] SKIP (none of {list(st['tests'])} with their "
                f"features in targets)")
            continue
        terms = own + [(n, s, tf, CARRY_WEIGHT) for n, s, tf, _ in carried]
        # warm-start on the analytic model, but only the keys it responds to
        warm_keys = analytic_sensitive_keys(theta, keys, own, combine_mode, surface)
        warm, fwarm, n_warm = {}, float("nan"), 0
        if warm_keys:
            warm_obj = Objective(theta, warm_keys, terms, "analytic", combine_mode, surface)
            warm, fwarm = optimize_stage(warm_obj, warm_keys, max_evals=40, seed=seed, surface=surface)
            n_warm = warm_obj.n_eval
        cold = [k for k in keys if k not in warm_keys]
        if backend == "isaac":
            theta_try = dict(theta); theta_try.update(warm)
            obj = Objective(theta, keys, terms, "isaac", combine_mode, surface)
            best, fbest = refine_isaac(obj, keys, _x0(keys, theta_try, surface), max_evals, surface)
            log(f"[stage {st['id']} {st['name']}] warm L={fwarm:.2f} ({n_warm} analytic evals on "
                f"{warm_keys}; not modelled: {cold}) -> isaac L={fbest:.2f} ({obj.n_eval} evals): "
                + ", ".join(f"{k}={best[k]:.4g}" for k in keys))
        else:
            best = {k: warm.get(k, _to_theta([k], _x0([k], theta, surface))[k]) for k in keys}
            fbest = fwarm
            log(f"[stage {st['id']} {st['name']}] L={fbest:.2f} ({n_warm} evals): "
                + ", ".join(f"{k}={best[k]:.4g}" for k in keys)
                + (f"  (no analytic warm start for {cold}: left at start)" if cold else ""))
        theta.update(best)
        C.validate(theta, surface)
        results[st["name"]] = {k: float(best[k]) for k in keys} | {"loss": fbest}
        carried += own
    return theta, results


def write_surface(path, surface, theta, results, combine_mode):
    out = {"surface": surface, "combine_mode": combine_mode,
           "floor_friction_band": {k: list(v) for k, v in C.surface_band(surface).items()},
           "measured": {k: float(theta[k]) for k in C.MEASURED},
           "calibrated": {k: float(theta[k]) for k in C.OPTIMISABLE},
           "fixed": {k: float(theta[k]) for k in C.FIXED},
           "stage_results": results,
           "sim_calibration_yaml": C.nest({k: theta[k] for k in theta if k not in C.LOCKED})}
    if yaml and path:
        Path(path).write_text(yaml.safe_dump(out, sort_keys=False))
    return out


# ── self-test: recover a known theta from synthetic targets (no Isaac) ───────
def self_test():
    print("=== self-test: recover known parameters from synthetic targets (analytic) ===")
    seqs = {name: F.load_sequence(str(HERE.parent / "parallel" / "sequences" / f"{name}.yaml"))
            for name in ("T1_straight", "T2_spin", "T4_arc", "T5_stop", "T6_spin_sweep",
                         "T3b_top_speed")}
    # only knobs the analytic model represents (static mu / floor-mu scrub / torque-limited
    # turning are Isaac-only and are deliberately not warm-started)
    theta_true = {"wheel_distance": 0.68, "max_acceleration": 0.8, "max_deceleration": 1.5,
                  "max_angular_acceleration": 3.0}
    surface = "concrete"
    measured = {k: C.SCHEMA_BY_KEY[k]["default"] for k in C.MEASURED}
    full_true = start_theta(measured, surface)
    full_true.update(theta_true)
    targets = {"surface": "synthetic", "combine_mode": "min",
               "top_speed": {"linear": 0.42, "angular": 1.9}, "tests": {}}
    for name, steps in seqs.items():
        tr = fm.simulate(full_true, steps, combine_mode="min")
        f = F.extract(np.array(tr["t"]), np.array(tr["x"]), np.array(tr["y"]),
                      np.array(tr["yaw"]), steps)
        targets["tests"][name] = {"steps": steps, "sequence": steps,
                                  "features": {k: {"mean": v, "sigma": max(0.02 * abs(v), 1e-3)}
                                               for k, v in f.items() if np.isfinite(v)}}
    theta, results = run_calibration(targets, measured, "analytic", "min", surface,
                                     max_evals=40, seed=1, log=print)
    print("\nrecovery (key: true -> found):")
    ok = True
    for k, tv in sorted(theta_true.items()):
        fv = theta[k]
        rel = abs(fv - tv) / max(abs(tv), 1e-6)
        flag = "OK " if rel < 0.15 else "OFF"
        ok &= rel < 0.15
        print(f"  {flag} {k:26s} {tv:8.4g} -> {fv:8.4g}  ({rel*100:4.1f}%)")
    band = C.surface_band(surface)
    inside = (band["static"][0] <= theta["floor_mu_static"] <= band["static"][1]
              and band["dynamic"][0] <= theta["floor_mu_dynamic"] <= band["dynamic"][1])
    print(f"  {'OK ' if inside else 'OFF'} floor mu stayed in the {surface} band "
          f"(static {theta['floor_mu_static']:.3f}, dynamic {theta['floor_mu_dynamic']:.3f})")
    print(f"  {'OK ' if theta['max_linear_speed'] == 0.42 else 'OFF'} speed clamp read-off")
    ok &= inside and theta["max_linear_speed"] == 0.42
    print("\nSELF-TEST", "PASSED" if ok else "FAILED")
    return 0 if ok else 1


def main():
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--surface", default="surface",
                    help=f"floor material; sets the floor-mu band ({', '.join(C.SURFACE_FRICTION)} "
                         f"or an alias; unknown names use the all-materials envelope)")
    ap.add_argument("--targets", help="targets YAML (build_targets.py output)")
    ap.add_argument("--backend", choices=["analytic", "isaac"], default="analytic")
    ap.add_argument("--combine-mode", default="min", choices=["min", "average", "multiply"])
    ap.add_argument("--measured", "--geometry", dest="measured", default=str(HERE / "measured.yaml"),
                    help="measured masses yaml (geometry entries are ignored: geometry is locked)")
    ap.add_argument("--measure-mass", "--measure-geometry", dest="measure_mass", action="store_true")
    ap.add_argument("--no-prompt-geometry", action="store_true")
    ap.add_argument("--max-evals", type=int, default=25, help="per-stage Isaac refine budget")
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--out", help="write surfaces/<surface>.yaml here")
    ap.add_argument("--stages", help="comma list of stage ids to run, e.g. '2,3' on concrete "
                    "then '4,5' per surface (default: all of 2,3,4,5)")
    ap.add_argument("--base", help="seed calibrated knobs from a prior surfaces/*.yaml "
                    "(the concrete reference for stages 2-3) before running")
    ap.add_argument("--self-test", action="store_true")
    args = ap.parse_args()

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
    combine = targets.get("combine_mode", args.combine_mode)
    only = {int(s) for s in args.stages.split(",")} if args.stages else None
    seed_theta = None
    if args.base and Path(args.base).exists():
        base_doc = yaml.safe_load(Path(args.base).read_text()) or {}
        seed_theta = base_doc.get("calibrated", {})
    theta, results = run_calibration(targets, measured, args.backend, combine, args.surface,
                                     args.max_evals, args.seed, log=print,
                                     only_stages=only, seed_theta=seed_theta)
    out = args.out or str(HERE.parents[0] / "surfaces" / f"{args.surface}.yaml")
    Path(out).parent.mkdir(parents=True, exist_ok=True)
    write_surface(out, args.surface, theta, results, combine)
    print(f"\nwrote {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
