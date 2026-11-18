#!/usr/bin/env python3
"""Single source of truth for the sim rover's calibratable physics parameters.

ONE schema, three consumers:
  1. apply_sim_calibration.py  — BAKE a calibration into jetson_rover_v2.usd (saves).
  2. run_baked_rover.py        — apply a calibration IN-MEMORY before play (never saves),
                                 so the optimiser can try a theta in seconds.
  3. sim2real_calib/optimize/  — the search space (bounds/scale/stage) + yaml round-trip.

The SCHEMA block below is dependency-free (pure Python) so the optimiser can import it
under system python3.10. The apply_to_stage() helper imports pxr LAZILY, so it only needs
Isaac's USD libs when actually authoring a stage.

Parameter classes (2026-10-15 revision, see sim2real_calib/optimize/README.md):
  calibrated  finite lo/hi -> searched by the optimiser.
  measured    weighed once (masses); never searched; authoring one rescales that body's inertia.
  fixed       explicitly held constant (currently no motion parameters are fixed).
  locked      rover GEOMETRY. Never authored by any path; a value in a calibration file is
              ignored with a warning. wheel_distance is the ONLY geometric quantity that may be
              changed (it is a controller turning gain, not the physical track).

Prim paths / current values confirmed by dumping jetson_rover_v2.usd (USD crate v0.9):
  - DiffController inputs, 4 corner-wheel revolute drives, 2 passive middle wheels,
    corner/middle wheel physics materials, base_link mass/COM/inertia, 6 wheel masses.
  - There is NO explicit FLOOR material in the stage; author_floor() creates one and binds
    it to the ground collider so friction becomes a clean per-surface knob.
"""
from __future__ import annotations

import math

# ── Prim paths (jetson_rover_v2.usd) ─────────────────────────────────────────
DIFF = "/World/ugv_rover/Drive_Graph/SkidSteer_Controller/DiffController"
TICK = "/World/ugv_rover/Drive_Graph/SkidSteer_Controller/OnPlaybackTick"
MAT_CORNER = "/World/Physics_Materials/corner_wheel"
MAT_MIDDLE = "/World/Physics_Materials/middle_wheel"
MAT_FLOOR = "/World/Physics_Materials/floor"          # created by author_floor()
BASE_LINK = "/World/ugv_rover/base_link"
CORNER_JOINTS = [
    "/World/ugv_rover/joints/left_up_wheel_link_joint",
    "/World/ugv_rover/joints/left_down_wheel_link_joint",
    "/World/ugv_rover/joints/right_up_wheel_link_joint",
    "/World/ugv_rover/joints/right_down_wheel_link_joint",
]
WHEEL_LINKS = [
    "/World/ugv_rover/left_up_wheel_link", "/World/ugv_rover/left_down_wheel_link",
    "/World/ugv_rover/left_middle_wheel_link", "/World/ugv_rover/right_up_wheel_link",
    "/World/ugv_rover/right_down_wheel_link", "/World/ugv_rover/right_middle_wheel_link",
]

# ── Geometry lock ────────────────────────────────────────────────────────────
# Attributes that define the rover's geometry. No schema entry may target them and
# _set_scalar() refuses them, so a future schema edit cannot silently make geometry tunable.
# (Wheel radius / track / wheelbase / wheel placement / collider shapes all live here.)
GEOMETRY_ATTR_PREFIXES = ("xformOp", "physics:localPos", "physics:localRot", "physics:axis",
                          "points", "extent", "radius", "height", "size",
                          "physics:approximation", "physxConvexHullCollision",
                          "physics:centerOfMass", "physics:principalAxes")

# ── Floor friction: literature bands for RUBBER on each floor material ───────
# Floor and wheel coefficients are searched independently. Under combine_mode=min the lower
# coefficient sets the contact friction; values above the other material form a flat region.
# Every floor value authored or searched must stay inside its band (validate_floor).
#   concrete: Engineering Toolbox / Misumi rubber-dry-concrete 0.6-0.85 (kinetic); Serway 1.0
#             static / 0.8 kinetic; HowWhy 1.0 / 0.7; ingdemurtas 0.65 / 0.5; UF design lab
#             0.6 (hard rubber) - 0.85 (soft rubber) static.
#   carpet:   no published rubber-on-carpet table; FRC experience ~0.6 "decent", >0.9
#             "excellent" (anecdotal) -> wide band, narrow it with a tilt test.
#   foam:     no published rubber-on-EVA value; EVA-foam-backed grip tape 0.85-1.2 static
#             (system value), EVA vs sled kinetic 0.5-0.7 -> wide band, measure to narrow.
SURFACE_FRICTION = {
    "concrete": {"static": (0.60, 1.00), "dynamic": (0.50, 0.85)},
    "carpet":   {"static": (0.60, 1.00), "dynamic": (0.50, 0.90)},
    "foam":     {"static": (0.70, 1.20), "dynamic": (0.50, 1.00)},
}
SURFACE_ALIASES = {"sealed_concrete": "concrete", "light_carpet": "carpet",
                   "heavy_carpet": "carpet", "low_pile_carpet": "carpet",
                   "high_pile_carpet": "carpet", "eva": "foam", "eva_foam": "foam",
                   "foam_padding": "foam", "padded": "foam"}   # padded = garage EVA mats
# Union of all bands: the hard limit when the surface is unknown (e.g. 'live', ad-hoc runs).
FRICTION_ENVELOPE = {
    "static": (min(b["static"][0] for b in SURFACE_FRICTION.values()),
               max(b["static"][1] for b in SURFACE_FRICTION.values())),
    "dynamic": (min(b["dynamic"][0] for b in SURFACE_FRICTION.values()),
                max(b["dynamic"][1] for b in SURFACE_FRICTION.values())),
}
# Preserve the existing wheel defaults; optimisation may vary both wheel materials.
WHEEL_MU_STATIC = FRICTION_ENVELOPE["static"][1]
WHEEL_MU_DYNAMIC = FRICTION_ENVELOPE["dynamic"][1]
FRICTION_PAIRS = (("corner_mu_static", "corner_mu_dynamic"),
                  ("middle_mu_static", "middle_mu_dynamic"),
                  ("floor_mu_static", "floor_mu_dynamic"))


def surface_band(surface: str | None) -> dict:
    """{'static': (lo, hi), 'dynamic': (lo, hi)} for a surface name (aliases ok); the
    all-materials envelope when the surface is unknown or None."""
    if surface:
        name = SURFACE_ALIASES.get(surface.lower(), surface.lower())
        if name in SURFACE_FRICTION:
            return SURFACE_FRICTION[name]
    return FRICTION_ENVELOPE


def surface_start(surface: str | None) -> dict:
    """Band midpoints: where a floor-friction search starts for this surface."""
    b = surface_band(surface)
    return {"floor_mu_static": round(sum(b["static"]) / 2, 4),
            "floor_mu_dynamic": round(sum(b["dynamic"]) / 2, 4)}


def validate_floor(mu_static, mu_dynamic, surface: str | None = None) -> None:
    """Raise ValueError unless the floor coefficients lie inside the surface's literature band
    and dynamic <= static (kinetic friction never exceeds static friction)."""
    b = surface_band(surface)
    for name, val in (("static", mu_static), ("dynamic", mu_dynamic)):
        if val is None:
            continue
        lo, hi = b[name]
        if not math.isfinite(float(val)) or not (lo - 1e-9 <= float(val) <= hi + 1e-9):
            raise ValueError(f"floor {name} friction {val} is outside the {surface or 'all-materials'} "
                             f"rubber-on-floor band [{lo}, {hi}] (calib_params.SURFACE_FRICTION)")
    if mu_static is not None and mu_dynamic is not None and float(mu_dynamic) > float(mu_static) + 1e-9:
        raise ValueError(f"floor dynamic friction {mu_dynamic} exceeds static {mu_static}")


# ── Parameter schema ─────────────────────────────────────────────────────────
# Each entry: canonical flat `key`, the nested `yaml` path (for sim_calibration.yaml),
# the `targets` to author as (prim_path, attribute_name), a `default` (current USD value),
# optimiser `lo`/`hi` bounds (+ `scale` lin|log, optional `start`), the calibration `stage`
# (calibrate.STAGES) and its class flag (locked / measured / fixed). lo=hi=None => not an
# optimiser dimension. Multi-prim params list every target so all are authored together.
_ENV_S, _ENV_D = FRICTION_ENVELOPE["static"], FRICTION_ENVELOPE["dynamic"]
_CONCRETE = surface_start("concrete")
SCHEMA = [
    # --- geometry (LOCKED: read for audits, never authored) ------------------------------
    dict(key="wheel_radius", yaml=("differential_controller", "wheelRadius"),
         targets=[(DIFF, "inputs:wheelRadius")], default=0.04,
         lo=None, hi=None, scale="lin", stage=None, locked=True,
         note="LOCKED GEOMETRY: controller R = physical wheel radius (0.04 m collision mesh). "
              "Never authored; a value in a calibration file is ignored with a warning"),
    # --- turning gain: the one geometric-looking quantity that IS calibrated ---------------
    dict(key="wheel_distance", yaml=("differential_controller", "wheelDistance"),
         targets=[(DIFF, "inputs:wheelDistance")], default=0.5,
         lo=0.25, hi=0.9, scale="lin", stage=4,
         note="controller turning gain (physical track 0.1745 is LOCKED geometry). Fit jointly "
              "with floor mu + torque cap on spins AND arcs"),
    # --- speed / yaw ceilings --------------------------------------------------------------
    dict(key="max_linear_speed", yaml=("differential_controller", "maxLinearSpeed"),
         targets=[(DIFF, "inputs:maxLinearSpeed")], default=0.5,
         lo=0.2, hi=0.8, scale="lin", stage=3,
         note="clamp on v; read off the measured top speed or fit on T3b_top_speed"),
    dict(key="max_angular_speed", yaml=("differential_controller", "maxAngularSpeed"),
         targets=[(DIFF, "inputs:maxAngularSpeed")], default=1.9,
         lo=0.8, hi=3.0, scale="lin", stage=3,
         note="clamp on omega; read off the measured top yaw rate or fit on T6_spin_sweep"),
    # --- command ramps (DifferentialController; 0 = unset). Linear limits only act when BOTH
    # are > 0, and need a valid dt: apply_to_stage() wires the controller to the playback tick
    # whenever any of these is set (see enable_tick_dt). CONTROLLER UNITS: the tick's
    # deltaSeconds sums to ~0.57 of sim time under run_baked_rover (physics 1/240, render every
    # 4th step), so the achieved ramp is ~0.57 x the value (forward_model.TICK_DT_SCALE). The
    # fit is done in the same runner, so a fitted value reproduces the real ramp there.
    dict(key="max_acceleration", yaml=("differential_controller", "maxAcceleration"),
         targets=[(DIFF, "inputs:maxAcceleration")], default=0.0,
         lo=0.2, hi=5.0, scale="log", start=5.0, stage=2,
         note="linear command ramp-up [m/s^2]; the real ESP32 firmware/motor ramp"),
    dict(key="max_deceleration", yaml=("differential_controller", "maxDeceleration"),
         targets=[(DIFF, "inputs:maxDeceleration")], default=0.0,
         lo=0.2, hi=8.0, scale="log", start=8.0, stage=2,
         note="linear command ramp-down [m/s^2] (commanded stops)"),
    dict(key="max_angular_acceleration", yaml=("differential_controller", "maxAngularAcceleration"),
         targets=[(DIFF, "inputs:maxAngularAcceleration")], default=0.0,
         lo=0.5, hi=20.0, scale="log", start=20.0, stage=2,
         note="yaw command ramp [rad/s^2] (spin-up / spin stop)"),
    # --- wheel friction: fit driven and passive contacts jointly on spins AND arcs -------
    # The reduced model omits static/middle friction and contact scrub. These parameters
    # are fitted by Isaac, without an analytic warm start that would invent sensitivities.
    dict(key="corner_mu_dynamic", yaml=("wheel_friction", "corner_wheel", "dynamicFriction"),
         targets=[(MAT_CORNER, "physics:dynamicFriction")], default=WHEEL_MU_DYNAMIC,
         lo=0.5, hi=1.5, scale="lin", stage=4, isaac_only=True,
         note="driven-wheel sliding friction; fit jointly with floor friction on spins/arcs"),
    dict(key="corner_mu_static", yaml=("wheel_friction", "corner_wheel", "staticFriction"),
         targets=[(MAT_CORNER, "physics:staticFriction")], default=WHEEL_MU_STATIC,
         lo=0.5, hi=1.8, scale="lin", stage=4, isaac_only=True,
         note="driven-wheel static friction; must be >= corner_mu_dynamic"),
    dict(key="middle_mu_dynamic", yaml=("wheel_friction", "middle_wheel", "dynamicFriction"),
         targets=[(MAT_MIDDLE, "physics:dynamicFriction")], default=WHEEL_MU_DYNAMIC,
         lo=0.3, hi=1.2, scale="lin", stage=4, isaac_only=True,
         note="passive-wheel sliding friction; fit on arcs plus spin behavior"),
    dict(key="middle_mu_static", yaml=("wheel_friction", "middle_wheel", "staticFriction"),
         targets=[(MAT_MIDDLE, "physics:staticFriction")], default=WHEEL_MU_STATIC,
         lo=0.3, hi=1.4, scale="lin", stage=4, isaac_only=True,
         note="passive-wheel static friction; must be >= middle_mu_dynamic"),
    # --- floor friction: per-surface, bounded by the literature bands ----------------------
    dict(key="floor_mu_dynamic", yaml=("floor_friction", "dynamicFriction"),
         targets=[(MAT_FLOOR, "physics:dynamicFriction")], default=_CONCRETE["floor_mu_dynamic"],
         lo=_ENV_D[0], hi=_ENV_D[1], scale="lin", stage=4, surface_bounded=True,
         note="rubber-on-floor kinetic mu; bounds = SURFACE_FRICTION[surface]['dynamic']"),
    dict(key="floor_mu_static", yaml=("floor_friction", "staticFriction"),
         targets=[(MAT_FLOOR, "physics:staticFriction")], default=_CONCRETE["floor_mu_static"],
         lo=_ENV_S[0], hi=_ENV_S[1], scale="lin", stage=4, surface_bounded=True,
         note="rubber-on-floor static mu; bounds = SURFACE_FRICTION[surface]['static']"),
    # --- drivetrain ------------------------------------------------------------------------
    dict(key="drive_damping", yaml=("wheel_drive", "damping"),
         targets=[(j, "drive:angular:physics:damping") for j in CORNER_JOINTS], default=10000.0,
         lo=1e-6, hi=20000.0, scale="log", stage=2, isaac_only=True,
         note="velocity-drive gain; logarithmic bounds include the low-gain response regime"),
    dict(key="drive_max_force", yaml=("wheel_drive", "maxForce"),
         targets=[(j, "drive:angular:physics:maxForce") for j in CORNER_JOINTS], default=3.4028235e38,
         lo=0.05, hi=50.0, scale="log", stage=4,
         note="per-wheel torque cap [N m]; a TURNING knob (spins stall at 0.05, invisible on "
              "straights above ~0.11)"),
    # --- chassis damping: fit translation AND yaw, after the drive/traction stages --------
    dict(key="base_linear_damping", yaml=("drag", "base_linear_damping"),
         targets=[(BASE_LINK, "physxRigidBody:linearDamping")], default=0.0,
         lo=0.0, hi=8.0, scale="lin", stage=5, isaac_only=True,
         note="linear velocity decay [1/s]; fit on straight speeds and stopping behavior"),
    dict(key="base_angular_damping", yaml=("drag", "base_angular_damping"),
         targets=[(BASE_LINK, "physxRigidBody:angularDamping")], default=0.05,
         lo=0.0, hi=8.0, scale="lin", stage=5, isaac_only=True,
         note="angular velocity decay [1/s]; fit on spins, spin stops and arcs"),
    # --- masses: MEASURED (weighed), never searched; inertia follows the mass --------------
    dict(key="base_mass", yaml=("mass", "base_link"),
         targets=[(BASE_LINK, "physics:mass")], default=1.40495,
         lo=None, hi=None, scale="lin", stage=None, measured=True,
         note="MEASURED: weigh the rover with battery in; authoring rescales base inertia"),
    dict(key="wheel_mass", yaml=("mass", "wheel"),
         targets=[(w, "physics:mass") for w in WHEEL_LINKS], default=0.08,
         lo=None, hi=None, scale="lin", stage=None, measured=True,
         note="MEASURED: weigh a wheel; authoring rescales wheel inertia"),
]

SCHEMA_BY_KEY = {s["key"]: s for s in SCHEMA}

# Keys the optimiser may vary (finite bounds). Locked, measured and fixed keys have lo=hi=None
# and are therefore EXCLUDED by construction.
OPTIMISABLE = [s["key"] for s in SCHEMA if s["lo"] is not None and s["hi"] is not None]
LOCKED = [s["key"] for s in SCHEMA if s.get("locked")]
MEASURED = [s["key"] for s in SCHEMA if s.get("measured")]
FIXED = [s["key"] for s in SCHEMA if s.get("fixed")]
FROZEN = LOCKED + MEASURED            # the per-rover constants the optimiser carries along
ACCEL_KEYS = ("max_acceleration", "max_deceleration", "max_angular_acceleration")

for _s in SCHEMA:   # import-time guard: nothing may target a geometry attribute
    for _p, _a in _s["targets"]:
        if not _s.get("locked") and _a.startswith(GEOMETRY_ATTR_PREFIXES):
            raise RuntimeError(f"schema key {_s['key']} targets geometry attribute {_a}")


def stage_keys(stage: int) -> list[str]:
    """Optimisable parameter keys belonging to a calibration stage (calibrate.STAGES)."""
    return [s["key"] for s in SCHEMA
            if s["stage"] == stage and s["lo"] is not None and s["hi"] is not None]


def bounds(key: str, surface: str | None = None) -> tuple[float, float]:
    """(lo, hi) search bounds; floor friction uses the surface's literature band."""
    s = SCHEMA_BY_KEY[key]
    if s.get("surface_bounded"):
        return surface_band(surface)["static" if key.endswith("static") else "dynamic"]
    return s["lo"], s["hi"]


def classify(key: str) -> str:
    """'locked' | 'measured' | 'fixed' | 'calibrated' — for display and to enforce the freeze."""
    s = SCHEMA_BY_KEY.get(key, {})
    for flag in ("locked", "measured", "fixed"):
        if s.get(flag):
            return flag
    return "calibrated"


# ── yaml <-> flat round-trip (pure Python) ───────────────────────────────────
def flatten(cfg: dict) -> dict:
    """Nested sim_calibration.yaml dict -> {canonical_key: value} for keys we know."""
    flat = {}
    for s in SCHEMA:
        node = cfg
        ok = True
        for k in s["yaml"]:
            if isinstance(node, dict) and k in node:
                node = node[k]
            else:
                ok = False
                break
        if ok and not isinstance(node, dict):
            flat[s["key"]] = node
    return flat


def nest(flat: dict) -> dict:
    """{canonical_key: value} -> nested dict shaped like sim_calibration.yaml."""
    out: dict = {}
    for key, val in flat.items():
        s = SCHEMA_BY_KEY.get(key)
        if not s:
            continue
        node = out
        for k in s["yaml"][:-1]:
            node = node.setdefault(k, {})
        node[s["yaml"][-1]] = val
    return out


def defaults() -> dict:
    """{key: default} for every parameter."""
    return {s["key"]: s["default"] for s in SCHEMA}


def validate(flat: dict, surface: str | None = None) -> None:
    """Check friction coefficients/order, floor bands and paired linear ramp limits."""
    validate_floor(flat.get("floor_mu_static"), flat.get("floor_mu_dynamic"), surface)
    for static, dynamic in FRICTION_PAIRS[:2]:
        if static not in flat and dynamic not in flat:
            continue
        s = float(flat.get(static, SCHEMA_BY_KEY[static]["default"]))
        d = float(flat.get(dynamic, SCHEMA_BY_KEY[dynamic]["default"]))
        if not all(math.isfinite(v) and v >= 0 for v in (s, d)):
            raise ValueError(f"{static} and {dynamic} must be finite and nonnegative")
        if d > s + 1e-9:
            raise ValueError(f"{dynamic} {d} exceeds {static} {s}")
    acc, dec = float(flat.get("max_acceleration") or 0), float(flat.get("max_deceleration") or 0)
    if (acc > 0) != (dec > 0):
        raise ValueError("max_acceleration and max_deceleration must both be set (> 0) or both "
                         "unset: the DifferentialController ignores linear limits otherwise")


# ── authoring onto a USD stage (needs pxr; imported lazily) ───────────────────
def _set_scalar(stage, prim_path, attr_name, value, changes, create_type=None):
    if attr_name.startswith(GEOMETRY_ATTR_PREFIXES):
        raise ValueError(f"refusing to author geometry attribute {prim_path}.{attr_name}")
    prim = stage.GetPrimAtPath(prim_path)
    if not prim or not prim.IsValid():
        changes.append(("WARN missing prim", prim_path, attr_name, None, None))
        return
    attr = prim.GetAttribute(attr_name)
    if (not attr or not attr.IsValid()) and create_type is not None:
        attr = prim.CreateAttribute(attr_name, create_type)
    if not attr or not attr.IsValid():
        changes.append(("WARN missing attr", prim_path, attr_name, None, None))
        return
    before = attr.Get()
    new = float(value) if (isinstance(before, float) or before is None) else type(before)(value)
    if isinstance(before, float) and before is not None and \
            abs(before - new) <= max(1e-9, abs(before) * 1e-6):
        return  # float32 noise -> no-op (keeps re-bake idempotent)
    if before == new:
        return
    changes.append(("set", prim_path, attr_name, before, new))
    attr.Set(new)


def _set_mass_keep_density(stage, prim_path, mass, changes):
    """Author a body's mass and scale its authored diagonalInertia by the same factor, so a
    measured mass never leaves an inconsistent inertia behind (the 2026-10-14 wheel bug)."""
    from pxr import Gf
    prim = stage.GetPrimAtPath(prim_path)
    m_attr = prim.GetAttribute("physics:mass") if prim and prim.IsValid() else None
    old = m_attr.Get() if m_attr else None
    _set_scalar(stage, prim_path, "physics:mass", mass, changes)
    if not old or not m_attr or abs(m_attr.Get() - old) <= max(1e-9, abs(old) * 1e-6):
        return
    i_attr = prim.GetAttribute("physics:diagonalInertia")
    inertia = i_attr.Get() if i_attr else None
    if inertia is not None and any(v > 0 for v in inertia):
        k = float(m_attr.Get()) / float(old)
        new = Gf.Vec3f(*(float(v) * k for v in inertia))
        i_attr.Set(new)
        changes.append(("set", prim_path, "physics:diagonalInertia", inertia, new))


def enable_tick_dt(stage, changes) -> None:
    """Run the DifferentialController every playback tick with dt = tick deltaSeconds.

    The baked graph fires the controller only when a Twist ARRIVES (execIn <- SubscribeTwist
    execOut) and leaves inputs:dt unwired. Ramp limits then make the node skip every compute
    (dt <= 0). On the tick, the controller re-reads the latest Twist each frame, so with no
    limits its output is identical; with limits the ramp advances in real (sim) time.

    In a running Kit app the graph is already instantiated and ignores raw USD connection
    edits, so the rewire goes through the OmniGraph API there; standalone pxr (the bake)
    edits the USD connections, which a later stage load picks up.
    """
    from pxr import Sdf
    diff, tick = stage.GetPrimAtPath(DIFF), stage.GetPrimAtPath(TICK)
    if not diff or not diff.IsValid() or not tick or not tick.IsValid():
        changes.append(("WARN missing prim", DIFF, "tick/dt rewire", None, None))
        return
    try:
        import omni.graph.core as og
    except ImportError:
        og = None
    for name, src in (("inputs:execIn", f"{TICK}.outputs:tick"),
                      ("inputs:dt", f"{TICK}.outputs:deltaSeconds")):
        attr = diff.GetAttribute(name)
        before = [str(c) for c in attr.GetConnections()]
        if before == [src]:
            continue
        if og is not None:
            for old in before:
                og.Controller.disconnect(old, f"{DIFF}.{name}")
            og.Controller.connect(src, f"{DIFF}.{name}")
        else:
            attr.SetConnections([Sdf.Path(src)])
        changes.append(("connect", DIFF, name, before, src))


def find_ground_collider(stage) -> str | None:
    """Best-effort path to the ground collider to bind a floor material onto.

    Handles the warehouse envs (`.../GroundPlane/CollisionPlane`) and the default
    ground plane added by World.scene.add_default_ground_plane().
    """
    from pxr import UsdPhysics
    candidates = []
    for prim in stage.Traverse():
        p = str(prim.GetPath())
        if p.endswith("GroundPlane/CollisionPlane") or p.endswith("/CollisionPlane"):
            return p
        if "groundPlane" in p or "defaultGroundPlane" in p or p.endswith("/GroundPlane"):
            candidates.append(p)
        if prim.HasAPI(UsdPhysics.CollisionAPI) and prim.GetTypeName() == "Plane":
            candidates.append(p)
    return candidates[0] if candidates else None


def author_floor(stage, mu_dynamic=None, mu_static=None, combine_mode=None,
                 ground_path=None, verbose=True, surface=None) -> list:
    """Create /World/Physics_Materials/floor (if missing), set its friction, and bind it
    to the ground collider with the 'physics' material purpose. Returns a change list.

    Unset coefficients fall back to the schema defaults (band midpoints): a floor material is
    never bound with unauthored friction (UsdPhysics' fallback is 0 -> near-frictionless).
    """
    from pxr import Sdf, UsdShade, UsdPhysics
    if mu_dynamic is None:
        mu_dynamic = SCHEMA_BY_KEY["floor_mu_dynamic"]["default"]
    if mu_static is None:
        mu_static = SCHEMA_BY_KEY["floor_mu_static"]["default"]
    validate_floor(mu_static, mu_dynamic, surface)
    changes: list = []
    mat_prim = stage.GetPrimAtPath(MAT_FLOOR)
    if not mat_prim or not mat_prim.IsValid():
        UsdShade.Material.Define(stage, MAT_FLOOR)
        mat_prim = stage.GetPrimAtPath(MAT_FLOOR)
        changes.append(("create", MAT_FLOOR, "Material", None, None))
    if not mat_prim.HasAPI(UsdPhysics.MaterialAPI):
        UsdPhysics.MaterialAPI.Apply(mat_prim)
    _set_scalar(stage, MAT_FLOOR, "physics:dynamicFriction", mu_dynamic, changes,
                create_type=Sdf.ValueTypeNames.Float)
    _set_scalar(stage, MAT_FLOOR, "physics:staticFriction", mu_static, changes,
                create_type=Sdf.ValueTypeNames.Float)
    if combine_mode is not None:
        for mp in (MAT_FLOOR, MAT_CORNER, MAT_MIDDLE):
            prim = stage.GetPrimAtPath(mp)
            if prim and prim.IsValid():
                a = prim.GetAttribute("physxMaterial:frictionCombineMode")
                if not a or not a.IsValid():
                    a = prim.CreateAttribute("physxMaterial:frictionCombineMode",
                                             Sdf.ValueTypeNames.Token)
                if a.Get() != combine_mode:
                    a.Set(combine_mode)
                    changes.append(("set", mp, "frictionCombineMode", None, combine_mode))
    gp = ground_path or find_ground_collider(stage)
    if gp:
        ground = stage.GetPrimAtPath(gp)
        if ground and ground.IsValid():
            mat = UsdShade.Material(stage.GetPrimAtPath(MAT_FLOOR))
            UsdShade.MaterialBindingAPI.Apply(ground)
            UsdShade.MaterialBindingAPI(ground).Bind(
                mat, UsdShade.Tokens.weakerThanDescendants, "physics")
            changes.append(("bind-floor", gp, MAT_FLOOR, None, None))
    elif verbose:
        changes.append(("WARN no ground collider found to bind floor", "", "", None, None))
    return changes


def apply_to_stage(stage, flat: dict, *, author_floor_material=True,
                   combine_mode=None, ground_path=None, verbose=True, surface=None) -> list:
    """Author a flat {key: value} calibration onto an open USD stage (in-memory; caller
    decides whether to Save()). Returns a list of change tuples for reporting.

    Locked geometry keys are never authored (WARN). The calibration is validated first
    (finite/ordered friction, floor friction inside its band, ramp limits paired) and raises ValueError
    otherwise. Floor-friction keys are routed through author_floor() (which also
    creates+binds the material); masses rescale inertia; any ramp limit rewires the
    controller onto the playback tick.
    """
    from pxr import Sdf
    validate(flat, surface)
    changes: list = []
    floor_mu_d = flat.get("floor_mu_dynamic")
    floor_mu_s = flat.get("floor_mu_static")
    want_floor = author_floor_material and (floor_mu_d is not None or floor_mu_s is not None
                                            or combine_mode is not None)
    for key, value in flat.items():
        if key in ("floor_mu_dynamic", "floor_mu_static"):
            continue  # handled by author_floor()
        spec = SCHEMA_BY_KEY.get(key)
        if spec is None:
            continue
        if spec.get("locked"):
            if value is not None and abs(float(value) - spec["default"]) > 1e-9:
                changes.append(("WARN locked geometry ignored", key, f"value {value!r}", None, None))
            continue
        if spec.get("measured"):
            for prim_path, _ in spec["targets"]:
                _set_mass_keep_density(stage, prim_path, value, changes)
            continue
        create_type = (Sdf.ValueTypeNames.Float
                       if any(a.startswith("physxRigidBody") for _, a in spec["targets"])
                       else None)
        for prim_path, attr_name in spec["targets"]:
            _set_scalar(stage, prim_path, attr_name, value, changes, create_type=create_type)
    if any(float(flat.get(k) or 0) > 0 for k in ACCEL_KEYS):
        enable_tick_dt(stage, changes)
    if want_floor:
        changes += author_floor(stage, mu_dynamic=floor_mu_d, mu_static=floor_mu_s,
                                combine_mode=combine_mode, ground_path=ground_path,
                                verbose=verbose, surface=surface)
    return changes
