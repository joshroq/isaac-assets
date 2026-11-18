"""Sim-parameter sheet for the session window (viz/dashboard.py).

Splits the simulator's parameters the way the optimiser treats them, straight from the
schema in ugv_controller_v2/calibration/calib_params.py:

  changing   - the keys the optimiser is fitting this session (start, current, best,
               latest candidate, bounds, this iteration's trust region, per-iteration history)
  held       - other calibratable keys, carried at their current value this session
  measured   - weighed per rover (masses); never searched
  geometry   - locked physical geometry; never authored (wheel radius, track, wheelbase)

`wheel_distance` is shown with its ratio to the physical track (D / B). In the skid-steer
model (docs/MODELING.md eq. 4.3) D = chi * B lumps the tyre-scrub factor chi into the
controller gain, so D is NOT expected to equal the physical track; the ratio is the number
to reason about.
"""
from __future__ import annotations

import sys
from pathlib import Path

_CALIB = Path(__file__).resolve().parents[2] / "ugv_controller_v2" / "calibration"
if str(_CALIB) not in sys.path:
    sys.path.insert(0, str(_CALIB))
import calib_params as C  # noqa: E402

UNITS = {
    "wheel_distance": "m", "wheel_radius": "m", "max_linear_speed": "m/s",
    "max_angular_speed": "rad/s", "max_acceleration": "m/s^2", "max_deceleration": "m/s^2",
    "max_angular_acceleration": "rad/s^2", "drive_damping": "", "drive_max_force": "N m",
    "base_linear_damping": "1/s", "base_angular_damping": "1/s", "base_mass": "kg",
    "wheel_mass": "kg",
}

# Physical geometry that is not a simulator attribute the calibration touches. Values are
# the USD link poses (PARAMETER_MAP.md: wheels at x = +/-0.0855 m, |y| = 0.08726 m) - the
# same constants optimize/forward_model.py uses.
TRACK_PHYS = 0.1745
WHEELBASE_PHYS = 0.171
REFERENCE_GEOMETRY = [
    ("physical track B", TRACK_PHYS, "m", "wheel |y| 0.08726 x 2, USD link poses"),
    ("wheelbase L", WHEELBASE_PHYS, "m", "corner wheels x = +/-0.0855"),
    ("wheels / driven", "6 / 4", "", "4 driven corners, 2 passive middles"),
]


def fmt(v):
    if isinstance(v, str):
        return v
    if v is None or v != v:
        return "-"
    a = abs(v)
    if a >= 1e5 or (0 < a < 1e-3):
        return f"{v:.3g}"
    if a >= 100:
        return f"{v:.1f}"
    return f"{v:.4g}"


def _schema_default(key):
    return C.SCHEMA_BY_KEY[key]["default"]


def row(key, theta, surface=None, **extra):
    s = C.SCHEMA_BY_KEY[key]
    value = theta.get(key, _schema_default(key))
    try:
        lo, hi = C.bounds(key, surface) if s["lo"] is not None else (None, None)
    except Exception:
        lo, hi = s["lo"], s["hi"]
    r = dict(key=key, value=value, unit=UNITS.get(key, ""), lo=lo, hi=hi,
             scale=s.get("scale", "lin"), cls=C.classify(key), note=s.get("note", ""))
    if key == "wheel_distance":
        r["derived"] = f"D/B = {value / TRACK_PHYS:.2f} x physical track {TRACK_PHYS} m"
    r.update(extra)
    return r


def build(theta, changing_keys=(), surface=None, start=None, best=None, cand=None,
          trust=None, history=None, header=""):
    """-> sheet dict for Dashboard.params()."""
    changing = []
    for k in changing_keys:
        changing.append(row(k, theta, surface,
                            start=(start or {}).get(k), best=(best or {}).get(k),
                            cand=(cand or {}).get(k), trust=(trust or {}).get(k),
                            history=list((history or {}).get(k, []))))
    held = [row(k, theta, surface) for k in C.OPTIMISABLE if k not in changing_keys]
    measured = [row(k, theta, surface) for k in C.MEASURED]
    geometry = [row(k, theta, surface) for k in C.LOCKED]
    geometry += [dict(key=name, value=v, unit=u, note=n, cls="reference")
                 for name, v, u, n in REFERENCE_GEOMETRY]
    held_title = ("Calibrated, held at current value this session" if changing_keys
                  else "Sim parameters (fixed for this run)")
    groups = [(held_title, held), ("Measured (weighed per rover)", measured),
              ("Locked geometry (never authored)", geometry)]
    return dict(header=header, changing=changing, groups=groups)
