#!/usr/bin/env python3
"""Reduced-order analytic forward model of the baked skid-steer rover.

This is the *equations* from docs/MODELING.md turned into code. Given a parameter set
(theta, the calib_params flat dict) and a /cmd_vel sequence, it predicts the body
trajectory (t, x, y, yaw). It is deliberately a lumped 1-D longitudinal + planar-yaw model,
NOT a contact simulation — its jobs are:

  1. define the SIGN of every parameter->observable sensitivity (the "increase/decrease"
     rules in the paper and the §5 map),
  2. warm-start the Isaac-in-the-loop optimiser into the right basin cheaply,
  3. let the optimiser machinery be unit-tested with NO Isaac (closed-loop recovery).

Static and passive-wheel friction are omitted, and drive/chassis damping sensitivities are
not reliable proxies for Isaac. These seven restored search parameters are marked isaac_only
in calib_params: they require the Isaac backend and are excluded from analytic warm starts.

Fidelity to Isaac/real is the Isaac backend's job (optimize/sim_eval.py); this model only
has to be monotone in the right directions. Where it is intentionally rough (skid-steer yaw
efficiency), it is flagged.

Key physics baked in (see MODELING.md for the derivations):
  * Linear scale:  v_body = (r_phys / R) * v_cmd, with r_phys the PHYSICAL wheel radius
    (collision mesh, fixed ~0.04) and R = inputs:wheelRadius. => raising R SLOWS the sim.
  * Velocity-servo droop: a finite drive damping b leaves a steady-state speed deficit under
    load, so carpet/foam drag (b_wheel) + low b both reduce v_ss (the "sag").
  * Coast: finite maxForce caps the braking force => longer coast; drag bleeds the rest.
  * Skid-steer yaw: ideal diff-drive yaw * (D/track_phys) * eta(mu), eta<1 falls with grip
    (more grip => more scrub resistance => you raise D to compensate).
"""
from __future__ import annotations

import math
import sys
from pathlib import Path

# calib_params defaults (single source of truth) — pure-Python import, no pxr needed.
_CALIB = Path(__file__).resolve().parents[2] / "ugv_controller_v2" / "calibration"
if str(_CALIB) not in sys.path:
    sys.path.insert(0, str(_CALIB))
import calib_params as C  # noqa: E402

# ── Fixed physical constants (geometry/inertia measured from the USD; NOT tuned) ──
R_PHYS = 0.04          # m   physical wheel contact radius (collision mesh), fixed
TRACK_PHYS = 0.1745    # m   physical track (|y| of wheels = 0.08726 each side)
N_DRIVEN = 4           # driven corner wheels
N_CONTACT = 6          # wheels touching the ground (drag acts on all)
I_WHEEL = 6.6e-5       # kg m^2  wheel spin inertia (diagonalInertia major axis; was 1.7e-4
                       #         before tools/fix_wheel_physics.py rescaled it to the 0.08 kg mass)
IZZ_BODY = 0.0122      # kg m^2  base_link yaw inertia (diagonalInertia z)
G = 9.81
C_RR = 0.015           # baseline rolling-resistance coefficient (firm floor); small
KAPPA_SCRUB = 0.8      # skid-steer yaw-efficiency falloff with grip
# The DifferentialController's ramp limits integrate the playback tick's deltaSeconds, which
# under run_baked_rover (physics 1/240, render every 4th step) sums to ~0.57 of simulated time
# (measured 2026-10-15: 0.5 m/s^2 set -> 0.285 achieved; 1.0 -> 0.562). Model it the same way.
TICK_DT_SCALE = 0.57
REF_MU = 0.75          # nominal grip the scrub factor is evaluated at (kept FIXED in this
                       # analytic model so wheel_distance is the clean yaw knob; the true
                       # mu<->yaw coupling is resolved in Isaac during the stage-6 re-fit)


def _combine(mu_a, mu_b, mode):
    if mode == "min":
        return min(mu_a, mu_b)
    if mode == "multiply":
        return mu_a * mu_b
    return 0.5 * (mu_a + mu_b)  # 'average' (PhysX default)


def _params(theta: dict, combine_mode: str | None):
    p = C.defaults()
    p.update({k: v for k, v in theta.items() if k in p})
    R = max(p["wheel_radius"], 1e-4)
    m = p["base_mass"] + N_CONTACT * p["wheel_mass"]
    m_eff = m + N_CONTACT * I_WHEEL / R_PHYS**2          # + reflected wheel inertia
    izz_eff = IZZ_BODY + N_CONTACT * p["wheel_mass"] * (TRACK_PHYS / 2) ** 2
    b = max(p["drive_damping"], 1e-6)
    tau_max = p["drive_max_force"]
    d_lin = p["base_linear_damping"]
    d_ang = p["base_angular_damping"]
    mu_eff = _combine(p["corner_mu_dynamic"], p["floor_mu_dynamic"], combine_mode or "average")
    f_max = (N_DRIVEN * tau_max / R_PHYS if tau_max < 1e30 else 1e30)   # drive/brake FORCE cap [N]
    # traction ceiling: driven wheels can't push harder than mu*N on their share of weight
    f_trac = mu_eff * m * G * (N_DRIVEN / N_CONTACT)
    return dict(
        R=R, k_v=R_PHYS / R, m=m, m_eff=m_eff, izz_eff=izz_eff,
        D=p["wheel_distance"], v_max=p["max_linear_speed"], w_max=p["max_angular_speed"],
        mu_eff=mu_eff,
        c0=C_RR * m * G,                                  # constant rolling-resistance force [N]
        # chassis linearDamping d acts as F = m*d*v (linear-momentum decay): drag slope = m*d.
        c1=m * d_lin,                                     # viscous linear drag slope [N/(m/s)]
        c1_w=izz_eff * d_ang,                             # viscous yaw drag slope [N*m/(rad/s)]
        # USD angular drive gains are per DEGREE: b_SI = b * 180/pi [N m s/rad]
        b_lin=N_DRIVEN * b * (180.0 / math.pi) / R_PHYS**2,   # linear-domain servo gain [N/(m/s)]
        acc=(p.get("max_acceleration") or 0.0) * TICK_DT_SCALE,
        dec=(p.get("max_deceleration") or 0.0) * TICK_DT_SCALE,
        acc_w=(p.get("max_angular_acceleration") or 0.0) * TICK_DT_SCALE,
        f_max=f_max,
        f_cap=min(f_max, f_trac),                         # the binding force limit (torque OR grip)
        eta=1.0 / (1.0 + KAPPA_SCRUB * REF_MU),           # FIXED nominal skid-steer yaw efficiency
    )


def simulate(theta: dict, sequence: list, dt: float = 0.01, combine_mode: str | None = None):
    """Integrate the reduced force model. `sequence` = list of {linear, angular, duration}.

    Dynamics (per driven axis); the servo is integrated implicitly (stable at the stiff USD
    gains) and then force-capped, after the controller's ramp limits shape the command:
        F_servo = b_lin * (v_target - v)          # stiff velocity servo
        F_cmd   = clip(F_servo, -F_cap, +F_cap)   # F_cap = min(torque cap, traction mu*N)
        m_eff v' = F_cmd - sign(v)*(c0 + c1|v|)   # minus rolling resistance + wheel drag
    With the authored maxForce=inf this gives an ~instant ramp (as the real sim does); LOWERING
    maxForce makes the accel linear (a≈F_cap/m_eff) and the coast longer — the dynamics knob.
    Yaw is the rotational analogue about izz_eff with moment cap F_cap*(track/2).

    Returns dict of lists: t, x, y, yaw, v, omega.
    """
    P = _params(theta, combine_mode)
    v = w = x = y = yaw = t = 0.0
    k_rot = P["b_lin"] * (TRACK_PHYS / 2) ** 2            # yaw-servo gain [N*m/(rad/s)]
    m_cap = P["f_cap"] * (TRACK_PHYS / 2)                 # yaw moment cap [N*m]
    v_ref = w_ref = 0.0                                   # controller's ramped command
    T, X, Y, YAW, V, W = [], [], [], [], [], []

    def servo(state, target, gain, cap, drag, inertia):
        """One step of a stiff velocity servo, integrated IMPLICITLY (stable for any gain;
        the old explicit step chattered +-24 % because gain*dt/m >> 2), then force-capped."""
        new = (inertia * state + dt * (gain * target - drag)) / (inertia + dt * gain)
        force = gain * (target - new)
        if abs(force) > cap:
            force = math.copysign(cap, force)
            new = state + (force - drag) / inertia * dt
        return new

    for step in sequence:
        vcmd = max(-P["v_max"], min(P["v_max"], float(step.get("linear", 0.0))))
        wcmd = max(-P["w_max"], min(P["w_max"], float(step.get("angular", 0.0))))
        n = max(1, int(round(float(step.get("duration", 0.0)) / dt)))
        for _ in range(n):
            # DifferentialController ramp limits (linear needs BOTH acc and dec, as in Isaac)
            if P["acc"] > 0 and P["dec"] > 0:
                accel = abs(vcmd) > abs(v_ref) and vcmd * v_ref >= 0
                lim = (P["acc"] if accel else P["dec"]) * dt
                v_ref = max(v_ref - lim, min(vcmd, v_ref + lim))
            else:
                v_ref = vcmd
            if P["acc_w"] > 0:
                lim = P["acc_w"] * dt
                w_ref = max(w_ref - lim, min(wcmd, w_ref + lim))
            else:
                w_ref = wcmd
            v_target = P["k_v"] * v_ref                   # k_v = r_phys/R (=1 when geometry matched)
            w_target = P["k_v"] * (P["D"] / TRACK_PHYS) * P["eta"] * w_ref
            # longitudinal
            f_drag = (P["c0"] + P["c1"] * abs(v)) * (1.0 if v > 0 else (-1.0 if v < 0 else 0.0))
            v_new = servo(v, v_target, P["b_lin"], P["f_cap"], f_drag, P["m_eff"])
            v = 0.0 if (v_ref == 0.0 and v_new * v < 0) else v_new   # don't reverse past 0 on coast
            # yaw
            sgn_w = (1.0 if w > 0 else (-1.0 if w < 0 else 0.0))
            m_drag = (0.5 * P["c0"] * (TRACK_PHYS / 2)) * sgn_w + P["c1_w"] * w   # Coulomb + viscous
            w_new = servo(w, w_target, k_rot, m_cap, m_drag, P["izz_eff"])
            w = 0.0 if (w_ref == 0.0 and w_new * w < 0) else w_new
            yaw += w * dt
            x += v * math.cos(yaw) * dt
            y += v * math.sin(yaw) * dt
            t += dt
            T.append(t); X.append(x); Y.append(y); YAW.append(yaw); V.append(v); W.append(w)
    return dict(t=T, x=X, y=Y, yaw=YAW, v=V, omega=W)


if __name__ == "__main__":
    # tiny smoke test: straight + spin + coast, print endpoint and key features
    seq = [
        {"linear": 0.0, "angular": 0.0, "duration": 1.0},
        {"linear": 0.12, "angular": 0.0, "duration": 3.0},
        {"linear": 0.0, "angular": 0.0, "duration": 2.0},
        {"linear": 0.0, "angular": 0.5, "duration": 3.14},
    ]
    for R in (0.035, 0.04, 0.045):
        tr = simulate({"wheel_radius": R}, seq, combine_mode="min")  # R is locked in the USD; model what-if only
        print(f"R={R}: end x={tr['x'][-1]:+.3f} y={tr['y'][-1]:+.3f} "
              f"yaw={math.degrees(tr['yaw'][-1]):+.1f}deg  max v={max(tr['v']):.3f}")
