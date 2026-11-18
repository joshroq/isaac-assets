# Dynamic & Kinematic Model for Sim→Real Calibration

Paper-facing companion to `PARAMETER_MAP.md`. It writes out the **actual multivariable
systems** that connect each calibratable simulation parameter to a measurable behaviour of the
real rover, so every test in the battery can be pointed at the parameters it identifies, **with
the sign of the effect** (increase/decrease). It is organised as four physical subsystems —
*actuator → wheel–ground contact → skid-steer kinematics → chassis rigid-body dynamics* — then a
per-test reduced model, then the **sensitivity Jacobian** (the increase/decrease table), then
identifiability/staging, a full parameter-accounting checklist, and references + figure sources.

Equations are grounded in the *actual* baked model: the Isaac `DifferentialController` formula
(verified in source), the PhysX implicit velocity-drive, and the authored values dumped from
`jetson_rover_v2.usd`. Render the math with any MathJax/pandoc viewer; the free-body and
block diagrams are provided as compilable TikZ in `figures.tex`.

> **Geometry is frozen.** The physical constants $r,\,B,\,L,\,m,\,I_{zz}$ are measured from the
> real rover and never calibrated. The optimiser varies only the *behavioural* parameters
> (friction, drag, drive limits, effective track). Symbols below mark which is which.

---

## 0. Notation

| Symbol | Meaning | Kind | `calib_params` key / USD attr |
|---|---|---|---|
| $v,\ \omega$ | body longitudinal speed, yaw rate (the observed state) | state | — (from camera / `/odom`) |
| $v_c,\ \omega_c$ | commanded body twist (post-clamp) | input | `/cmd_vel` |
| $r$ | **physical** wheel radius (collision mesh) | geom (frozen) | mesh ≈ 0.04 m |
| $R$ | controller wheel radius | geom (frozen) | `inputs:wheelRadius` |
| $B$ | **physical** track (wheel lateral spacing ×2) | geom (frozen) | 0.1745 m (from link poses) |
| $L$ | wheelbase (fore–aft corner spacing) | geom (frozen) | 0.171 m |
| $D$ | controller "wheel distance" = effective track | **calibrated** | `inputs:wheelDistance` |
| $k_v=r/R$ | linear scale gain | derived | — (=1 when $R$ set to $r$) |
| $b$ | drive damping (velocity-servo gain $K_d$) | **calibrated** | `drive:angular:physics:damping` |
| $\tau_{\max}$ | per-wheel drive torque cap | **calibrated** | `drive:angular:physics:maxForce` |
| $v_{\lim},\ \omega_{\lim}$ | speed / yaw-rate clamps | **calibrated** | `inputs:maxLinear/AngularSpeed` |
| $\mu_w,\ \mu_f$ | wheel-, floor-material friction | **calibrated** | `physics:dynamic/staticFriction` |
| $\mu$ | effective contact friction | derived | $=\mathrm{comb}(\mu_w,\mu_f)$ |
| $d_\ell,\ d_a$ | chassis linear / angular damping (drag) | **calibrated** | `physxRigidBody:linear/angularDamping` |
| $m$ | total mass (chassis + wheels) | measured (frozen) | `physics:mass` |
| $I_{zz}$ | chassis yaw inertia | measured (frozen) | `physics:diagonalInertia_z` |
| $N_i$ | normal load at wheel $i$ | derived | $\approx mg/6$ |
| $s$ | longitudinal slip ratio | derived | — |
| $\chi\ge 1$ | skid-steer under-rotation factor | derived | — (lumped into $D$) |
| $n_d=4,\ n_c=6$ | driven, ground-contacting wheel counts | geom | — |

Reflected quantities: $m_{\mathrm{eff}} = m + n_c I_w/r^2$ (wheel spin inertia reflected to
translation), $I_{zz,\mathrm{eff}} = I_{zz} + n_c m_w (B/2)^2$.

---

## 1. System overview

The rover is a cascade: `/cmd_vel` → **controller** (kinematics, inverts to wheel speeds) →
**actuators** (4 velocity-servo wheel drives, torque-capped) → **wheel–ground contact** (friction
cone + slip) → **chassis** (Newton–Euler planar body) → pose, measured by the overhead camera.
Each parameter lives in exactly one block, which is why a *well-chosen maneuver* isolates it:

$$
\underbrace{(v_c,\omega_c)}_{\text{cmd}}
\xrightarrow[\text{§4: } R,D]{\text{controller}}
(\omega_L^\*,\omega_R^\*)
\xrightarrow[\text{§2: } b,\tau_{\max}]{\text{actuator}}
\tau_i
\xrightarrow[\text{§3: } \mu,C_{rr}]{\text{contact}}
F_{x,i}
\xrightarrow[\text{§5: } m,I_{zz},d_\ell,d_a]{\text{chassis}}
(v,\omega)\!\rightarrow\!(x,y,\theta).
$$

Block diagram: `figures.tex` **Fig. 1**.

---

## 2. Subsystem 1 — Actuator: motor → axle → wheel

**What the sim actually does.** Each driven wheel joint is a PhysX *implicit velocity drive*
(stiffness 0, so pure velocity mode). Over a step it applies the joint torque

$$
\boxed{\ \tau_i = \operatorname{clamp}\!\big(b\,(\omega_i^\* - \omega_i),\ -\tau_{\max},\ +\tau_{\max}\big)\ }
\tag{2.1}
$$

with $\omega_i^\*$ the controller's target wheel speed (§4) and $\omega_i$ the actual wheel speed.
$b$ is the authored `damping` (=10000, very stiff), $\tau_{\max}$ the `maxForce` (=∞ as authored).

**Why this matters (the stiff-servo consequence).** Reflect (2.1) to the body. The closed-loop
longitudinal force is $F = \sum_i \tau_i/r$. With the authored $b$, the velocity error needed to
produce any force is $\Delta\omega = \tau r/b \approx 0$: the drive is effectively an *ideal speed
source*. The accel time-constant is

$$
\tau_v \;=\; \frac{m_{\mathrm{eff}}}{\,n_d\,b/r^2\,} \;\approx\; \frac{m_{\mathrm{eff}} r^2}{n_d b}\ \sim 10^{-7}\,\text{s}
\quad\text{at } b=10^4,
\tag{2.2}
$$

i.e. the sim reaches commanded speed essentially instantly. **Therefore the acceleration ramp of
the real rover is not matched by $b$** (it would need $b\!\sim\!10^{-3}$, which then can't hold
speed). The physically meaningful ramp knob is the **torque cap** $\tau_{\max}$: when it binds, the
acceleration is force-limited,

$$
a \approx \frac{n_d \tau_{\max}/r - F_{\text{drag}}}{m_{\mathrm{eff}}},
\qquad
\tau_v \approx \frac{m_{\mathrm{eff}}\, v_{ss}}{n_d \tau_{\max}/r}.
\tag{2.3}
$$

$b$ remains a *secondary* knob: a finite $b$ produces steady-state **droop** under load,
$v_{ss} = v^\* - F_{\text{load}} r^2/(n_d b)$, which is how a soft motor loses a little speed on a
resistive floor.

**Correspondence to a real DC gearmotor** (for the paper's "motor turning the axle"). The lumped
$(b,\tau_{\max})$ stand in for the full electromechanical model

$$
L_a\dot i + R_a i = V - K_e\,\omega_m,\qquad
J_m\dot\omega_m = K_t\, i - \tfrac{1}{G}\tau_{\text{load}} - b_m\omega_m,\qquad
\omega_{\text{wheel}} = \omega_m/G,
\tag{2.4}
$$

whose closed velocity loop (the Waveshare driver's onboard PID) behaves like a stiff velocity
source saturating at the reflected stall torque $K_t i_{\max}/G \equiv \tau_{\max}$. So: **measure
the real stall/no-load behaviour → set $\tau_{\max}$; measure any residual ramp → trim $b$.** FBD +
control block: `figures.tex` **Fig. 2**.

---

## 3. Subsystem 2 — Wheel against the ground

This is the "wheel turning against the ground" the paper needs, with every contact parameter.

**Rolling + slip (longitudinal).** A driven wheel of physical radius $r$ at wheel-speed
$\omega_{\text{wheel}}$ moves the ground point at $r\,\omega_{\text{wheel}}$; the body advances at
$v$. The **slip ratio** is

$$
s = \frac{r\,\omega_{\text{wheel}} - v}{\max(r\,\omega_{\text{wheel}},\,v)},
\qquad v = r\,\omega_{\text{wheel}}\,(1-s).
\tag{3.1}
$$

Substituting the controller target $\omega_{\text{wheel}}=v_c/R$ gives the **linear scale law**

$$
\boxed{\ v = \frac{r}{R}\,v_c\,(1-s) = k_v\,v_c\,(1-s)\ }
\tag{3.2}
$$

Two consequences for calibration: (i) with geometry matched, $R=r\Rightarrow k_v=1$, so on a
**high-grip** surface ($s\approx0$) the sim travels exactly the commanded distance — any real/sim
gap there is slip $s$ or resistance, **not** a radius to fudge (and note $\partial v/\partial R<0$:
raising the controller $R$ *slows* the sim, the opposite of the naive "bigger wheel" intuition,
because $R$ only rescales the commanded wheel speed). (ii) $s$ grows when demanded traction
exceeds grip (below), which is the real distance-deficit on slick floors.

**Traction limit (friction cone).** The tangential force a wheel can transmit is bounded by the
Coulomb cone with the **combined** material friction:

$$
|F_{x,i}| \le \mu\,N_i,\qquad
\mu = \operatorname{comb}(\mu_w,\mu_f)=
\begin{cases}
\tfrac12(\mu_w+\mu_f) & \text{average (PhysX default)}\\
\min(\mu_w,\mu_f) & \text{min (recommended)}\\
\mu_w\,\mu_f & \text{multiply}
\end{cases}
\tag{3.3}
$$

Choosing `min` makes $\mu\approx\mu_f$, so the **floor material is a clean single per-surface
knob**. When the drive demands $F>\mu N$, the wheel slips ($s\uparrow$) and (3.2) loses distance —
the signature of a slippery surface. FBD: `figures.tex` **Fig. 3**.

**Rolling resistance & deformable terrain (carpet/foam).** Rigid Coulomb contact does **not**
capture the energy lost deforming carpet pile or compressing foam. The classical model adds

$$
F_{rr} = C_{rr}\,N \quad(\text{hard floor}),\qquad
F_{\text{terr}} = f(\text{sinkage},k_c,k_\phi,n)\ \ (\text{Bekker, soft terrain}),
\tag{3.4}
$$

the latter out of rigid-body scope. The affordable **proxy** is a viscous drag lumped at the
chassis, $F_{\text{drag}} = m\,d_\ell\,v$ (and $M_{\text{drag}}=I_{zz}d_a\,\omega$), which bleeds
speed/coast. Crucially, because the drive is a stiff servo (§2), this drag only *sags steady
speed* once the drive is **force-limited** (low $\tau_{\max}$ or low-grip traction); otherwise it
only shortens **coast**. That is why the carpet signature is matched with $d_\ell$ (coast) + a
per-surface $v_{\lim}$/$\tau_{\max}$ (speed) and **not** by lowering friction (which would wrongly
add wheel-spin slip).

---

## 4. Subsystem 3 — Skid-steer kinematics & slip

**Controller (verified in Isaac source).** `DifferentialController` inverts the body twist to
wheel speeds using its own $R,D$:

$$
\omega_L^\* = \frac{2v_c - \omega_c D}{2R},\qquad
\omega_R^\* = \frac{2v_c + \omega_c D}{2R}.
\tag{4.1}
$$

**Ideal body motion** (physical track $B$, no slip) recovered from the *actual* wheel speeds:

$$
v = \frac{r(\omega_R+\omega_L)}{2}=k_v v_c,\qquad
\omega = \frac{r(\omega_R-\omega_L)}{B}=k_v\frac{D}{B}\,\omega_c.
\tag{4.2}
$$

**Skid-steer reality.** A 4/6-wheel skid-steer cannot turn without lateral tyre scrub, so it
**under-rotates** versus (4.2). Following the experimental skid-steer kinematics of Mandow et al.
(ICR model), introduce an effective track $B_{\text{eff}}=\chi B$ with $\chi\ge1$ increasing with
grip; the achieved yaw is

$$
\boxed{\ \omega = \frac{k_v\,D}{\,\chi B\,}\,\omega_c\ }
\tag{4.3}
$$

Calibration lumps $\chi$ into $D$: to match a measured $\omega$, set
$D = \chi B\,\omega/(k_v\omega_c)$. Hence **$D$ is not geometry** (it is $B$ scaled by a
surface-dependent slip factor), and $\partial D/\partial\mu>0$ — more grip ⇒ more scrub ⇒ larger
$D$, so $D$ is re-fit *after* friction per surface. Top-view ICR diagram: `figures.tex` **Fig. 4**.

**Turn "walk".** If the mass centre is offset from the geometric centre by $(e_x,e_y)$, spinning
about the ICR translates the body centre, $\Delta p \approx |e|\,\Delta\theta$ to first order —
the `turn_walk` feature, tuned by COM (frozen geometry) and the middle-wheel friction asymmetry.

---

## 5. Subsystem 4 — Chassis rigid-body dynamics

Planar Newton–Euler for the body (FBD: `figures.tex` **Fig. 5**):

$$
m_{\mathrm{eff}}\,\dot v = \underbrace{\textstyle\sum_i F_{x,i}}_{F_{\text{drive}},\ \le\, \mu N\ \&\ \le n_d\tau_{\max}/r}
\;-\; C_{rr} m g\;-\; m\,d_\ell\,v,
\tag{5.1}
$$

$$
I_{zz,\mathrm{eff}}\,\dot\omega =
\underbrace{\tfrac{B}{2}\textstyle\sum_i \pm F_{x,i}}_{M_{\text{drive}},\ \le\, \tfrac{B}{2}\,n_d\tau_{\max}/r}
\;-\; M_{\text{scrub}}(\mu)\;-\; I_{zz,\mathrm{eff}}\,d_a\,\omega.
\tag{5.2}
$$

**Steady states** (the T1/T2 observables):

$$
v_{ss} = \min\!\Big(\,\underbrace{k_v v_c}_{\text{kinematic}},\ \underbrace{v_{\lim}}_{\text{clamp}},\ \underbrace{\tfrac{\mu m g\,(n_d/n_c)-C_{rr}mg}{m d_\ell}}_{\text{traction/drag-limited}},\ \underbrace{\tfrac{n_d\tau_{\max}/r-C_{rr}mg}{m d_\ell}}_{\text{torque-limited}}\Big),
\tag{5.3}
$$

$$
\omega_{ss} = \min\!\big(k_v D\,\omega_c/(\chi B),\ \ \omega_{\lim}\big).
\tag{5.4}
$$

**Coast-down** ($v_c=0$, T5). The servo now brakes, torque-capped, plus drag:

$$
m_{\mathrm{eff}}\,\dot v = -\operatorname{clamp}\!\big(\tfrac{n_d b}{r^2}v,\,F_{\max}\big) - C_{rr}mg - m d_\ell v,
\quad F_{\max}=\tfrac{n_d\tau_{\max}}{r}.
\tag{5.5}
$$

With $\tau_{\max}=\infty$ the brake is unbounded ⇒ near-instant stop (as the current sim does);
finite $\tau_{\max}$ gives a constant-decel coast with **stopping distance**

$$
d_{\text{coast}} \approx \frac{v_0^2}{2\,a_{\text{brake}}},\qquad
a_{\text{brake}}=\frac{F_{\max}+C_{rr}mg}{m_{\mathrm{eff}}}+d_\ell v_0\ \text{(viscous part)} .
\tag{5.6}
$$

So **coast length is set by $\tau_{\max}$ (brake cap), $m$ (momentum, frozen), $d_\ell$ (drag), and
$\mu$ (once sliding)** — exactly the §E map entry.

---

## 6. Per-test reduced models

Each canonical test (PARAMETER_MAP.md §4) collapses the full system to a handful of parameters:

| Test | Command | Governing eqs | Observable → closed form | Parameters that enter |
|---|---|---|---|---|
| **T1** straight | $v_c$ const, then 0 | (3.2),(5.1),(5.3),(5.5) | $v_{ss}$, $d=\!\int v$, $\tau_v$ (2.3), $d_{\text{coast}}$ (5.6) | $k_v,\mu,\tau_{\max},b,d_\ell,m$ |
| **T2** spin | $\omega_c$ const | (4.1)–(4.3),(5.2),(5.4) | $\omega_{ss}$ (5.4), $\Delta\theta=\!\int\omega$ | $D,k_v,\chi(\mu),\omega_{\lim},I_{zz},d_a$ |
| **T3** speed sweep | $v_c\in\{...\}$ | (5.3) | $v_{ss}(v_c)$ saturation knee | which of $v_{\lim},\mu,\tau_{\max},d_\ell$ binds |
| **T4** arc | $v_c,\omega_c$ | (3.2),(4.3) | $R_{\text{arc}}=\dfrac{v}{\omega}=\dfrac{v_c}{\omega_c}\dfrac{\chi B}{D}$ | $D,\chi(\mu)$ (ratio) |
| **T5** stop | $v_0$→0 | (5.5),(5.6) | $d_{\text{coast}},t_{\text{coast}}$ | $\tau_{\max},d_\ell,\mu,m$ |

T3 is the discriminator: a **flat** plateau ⇒ clamp $v_{\lim}$; a **soft** shoulder that scales
with load ⇒ traction $\mu$ / torque $\tau_{\max}$ / drag $d_\ell$ (distinguished by whether wheel
slip is present and by the coast signature).

---

## 7. Sensitivity map (the increase/decrease table)

The calibration rule is first-order: if a sim feature $y$ is **below** the measured real value,
move parameter $\theta$ by $\operatorname{sign}(\partial y/\partial\theta)$ (and vice-versa),

$$
\Delta\theta \;\propto\; \operatorname{sign}\!\Big(\tfrac{\partial y}{\partial\theta}\Big)\,\big(y_{\text{real}}-y_{\text{sim}}\big).
$$

Signs of $\partial(\text{feature})/\partial(\text{parameter})$ (`+` raise param → feature up, `−`
down, `0` ≈ no effect in the identifiable regime, `()` only when that limit binds):

| Parameter \ Feature | $v_{ss}$ | dist | $\tau_v$ | $d_{\text{coast}}$ | $\omega_{ss}$ | $\Delta\theta_{yaw}$ | $R_{\text{arc}}$ | turn_walk |
|---|:--:|:--:|:--:|:--:|:--:|:--:|:--:|:--:|
| $D$ `wheel_distance` | 0 | 0 | 0 | 0 | **+** | **+** | **−** | + |
| $v_{\lim}$ `max_linear_speed` | (+) | (+) | 0 | 0 | 0 | 0 | (+) | 0 |
| $\omega_{\lim}$ `max_angular_speed` | 0 | 0 | 0 | 0 | (+) | (+) | (−) | 0 |
| $\mu_f$ `floor_mu_dynamic` | (+) | (+) | (−) | (−) | (−)$^\chi$ | (−)$^\chi$ | (+)$^\chi$ | + |
| $\mu_f^{s}$ `floor_mu_static` | 0 | 0 | 0 | (−) | 0 | 0 | 0 | + |
| $\mu_w$ `corner_mu_dynamic` | (+) | (+) | (−) | (−) | (−)$^\chi$ | (−)$^\chi$ | (+)$^\chi$ | + |
| $\mu_m$ `middle_mu_*` | 0 | 0 | 0 | 0 | − | − | + | **+** |
| $\tau_{\max}$ `drive_max_force` | (+) | (+) | **−** | **−** | (+) | (+) | 0 | 0 |
| $b$ `drive_damping` | (+)droop | (+) | (−)small | (−)small | (+)droop | (+) | 0 | 0 |
| $d_\ell$ `base_linear_damping` | (−) | (−) | + | **−** | 0 | 0 | 0 | 0 |
| $d_a$ `base_angular_damping` | 0 | 0 | 0 | 0 | (−) | (−) | (+) | 0 |
| $m$ (frozen) | (−) | (−) | **+** | **+** | 0 | (−) | 0 | + |
| $r,R,B$ (frozen geom) | $\partial v/\partial R<0$; $k_v=r/R$ | — | — | — | $\propto k_v D/B$ | — | $\propto B/D$ | — |

$^\chi$ = via the skid-steer scrub factor $\chi(\mu)$ (more grip ⇒ more under-rotation ⇒ *less*
yaw for fixed $D$ ⇒ you raise $D$). This is the one place friction feeds yaw, and it is why
$D$ is re-fit after $\mu$.

Reading example: real rover **coasts farther** than sim ⇒ $d_{\text{coast}}$ is low in sim ⇒ the
`−` entries say **lower $\tau_{\max}$** (and/or **lower $d_\ell$**, **raise $m$** if un-measured),
matching §E of the map.

---

## 8. Identifiability & staging

A parameter is identifiable from a test only where its column entry above is non-zero *and* not
collinear with another free parameter in that test. The coupling structure dictates the order
(derived in PARAMETER_MAP.md §6):

1. $v_{ss}$ depends on $k_v$ **and** $\mu$ **and** $\tau_{\max}$ **and** $d_\ell$ (5.3) → they are
   collinear in one run. Break it: fix $k_v$ by geometry; run $\tau_{\max},b$ on **concrete**
   (high $\mu$, no slip/resistance, so only the torque terms move $\tau_v,d_{\text{coast}}$); then
   per surface let $\mu$ (slip) and $d_\ell$ (drag) absorb the rest.
2. $\omega_{ss}$ depends on $D$ **and** $\chi(\mu)$ (5.4) → fit $D$ *after* $\mu$.
3. Static frictions $\mu^s$ and middle-wheel $\mu_m$ are weakly identifiable (they touch only
   launch/stiction and turn-walk) → tight priors, last.

The staged optimiser (`optimize/calibrate.py`) enforces exactly this freeze order; the Jacobian
above is the formal statement of *why*.

---

## 9. Parameter-accounting checklist

Every parameter the tests vary, the equation it lives in, the test that identifies it, and the
move rule. (Geometry rows are frozen — shown for completeness, never optimised.)

| Parameter | Appears in | Identified by | If real > sim on its feature |
|---|---|---|---|
| `wheel_distance` $D$ | (4.1)–(4.3),(5.4) | T2 $\Delta\theta,\omega_{ss}$ | **raise** $D$ |
| `max_linear_speed` $v_{\lim}$ | (5.3) | T3 plateau | **raise** to real top speed |
| `max_angular_speed` $\omega_{\lim}$ | (5.4) | yaw sweep | **raise** to real top yaw |
| `floor_mu_dynamic` $\mu_f$ | (3.3),(5.3) | T1/T3 slip deficit | **raise** $\mu_f$ (less slip ⇒ more dist) |
| `floor_mu_static` $\mu_f^s$ | launch/(5.5) | T1 launch, T5 | raise for stickier launch / shorter coast |
| `corner_mu_dynamic` $\mu_w$ | (3.3) | T1/T3 | raise (keep driven-wheel grip high) |
| `middle_mu_*` $\mu_m$ | (5.2) scrub | T2 turn_walk | raise to resist yaw / add walk |
| `drive_max_force` $\tau_{\max}$ | (2.1),(2.3),(5.5) | T1 $\tau_v$, T5 coast | **lower** (finite cap ⇒ ramp + coast) |
| `drive_damping` $b$ | (2.1),(2.2) | T1 ramp/droop | lower for softer ramp / more droop |
| `base_linear_damping` $d_\ell$ | (3.4),(5.1),(5.6) | T3 sag, T5 coast | **raise** to slow + shorten coast (carpet) |
| `base_angular_damping` $d_a$ | (5.2) | T2 spin coast | raise to damp spin |
| `friction_combine_mode` | (3.3) | design choice | set `min` so $\mu_f$ is the knob |
| `wheel_radius` $R$,$r$ (geom) | (3.2),(4.2) | **measured** | set $R=r$ (k_v=1); never fit |
| track $B$, base $m$, $I_{zz}$ (geom) | (4.2),(5.1),(5.2) | **measured** | enter once; never fit |

---

## 10. References & where to get the figures

**Kinematics (differential drive).** R. Siegwart, I. Nourbakhsh, D. Scaramuzza, *Introduction to
Autonomous Mobile Robots*, 2nd ed., MIT Press, 2011 — ch. 3 (diff-drive forward/inverse
kinematics, the exact (4.1)–(4.2) forms, with figures you can redraw).

**Skid-steer slip / effective track (the $\chi$, $D$ story — core citation).**
- A. Mandow, J. L. Martínez, J. Morales, J. L. Blanco, A. García-Cerezo, J. Serón, "Experimental
  kinematics for wheeled skid-steer mobile robots," *IROS 2007* — ICR model, instantaneous
  centres, effective track; the empirical basis for inflating $D$.
- J. L. Martínez, A. Mandow, et al., "Approximating Kinematics for Tracked Mobile Robots,"
  *Int. J. Robotics Research*, 2005.
- K. Kozłowski, D. Pazderski, "Modeling and control of a 4-wheel skid-steering mobile robot,"
  *Int. J. Appl. Math. Comput. Sci.*, 2004 — dynamics (5.1)–(5.2) with scrub moments.

**Tire friction & slip.** H. Pacejka, *Tire and Vehicle Dynamics*, 3rd ed., Butterworth-Heinemann,
2012 — slip ratio (3.1), friction cone, the Magic Formula if you later want a nonlinear
$F_x(s)$. For the stick–slip / Stribeck launch feel: B. Armstrong-Hélouvry, *Control of Machines
with Friction*, Kluwer, 1991; C. Canudas de Wit et al., "A new model for control of systems with
friction," *IEEE TAC*, 1995.

**Ground vehicles / rolling resistance / deformable terrain (carpet & foam, eq. 3.4).** J. Y.
Wong, *Theory of Ground Vehicles*, 4th ed., Wiley, 2008 — rolling resistance, Bekker/terramechanics
pressure–sinkage ($k_c,k_\phi,n$).

**Actuator / DC motor (eq. 2.4) and response.** G. Franklin, J. Powell, A. Emami-Naeini,
*Feedback Control of Dynamic Systems*, Pearson — armature model, first/second-order time constants,
how a velocity PID becomes a stiff speed source.

**The simulator's own models (authoritative for 2.1, 3.3, 5.x).** NVIDIA PhysX & Isaac Sim docs:
"Articulations / Joint Drives" (the stiffness–damping drive of 2.1), "Rigid Body Dynamics"
(linear/angular damping), "Physics Materials" (static/dynamic friction + `frictionCombineMode`),
and the TGS solver notes. R. Featherstone, *Rigid Body Dynamics Algorithms*, Springer, 2008 — the
articulated-body dynamics PhysX's reduced-coordinate articulation implements.

**Pre-existing equation/figure sources you can cite or adapt.** ROS 2 `diff_drive_controller`
documentation (kinematics + odometry equations); Gazebo `skid_steer_drive` plugin docs; the Isaac
Sim "Wheeled Robots" tutorial (the `DifferentialController` interface).

**Making the diagrams.**
- `figures.tex` in this folder — compilable **TikZ** sources for Fig. 1 (system block diagram),
  Fig. 2 (drivetrain FBD + velocity-servo block), Fig. 3 (wheel–ground contact FBD with the
  friction cone and slip), Fig. 4 (skid-steer ICR top view with $B$, $D$, $\chi$), Fig. 5 (chassis
  planar FBD). Compile: `pdflatex figures.tex`.
- Tooling: **TikZ/PGFPlots** for publication vector figures; **draw.io**/**Excalidraw** for quick
  block diagrams; **Inkscape** to trace/relabel; **matplotlib** for the measured-vs-model overlays
  (the calibration already emits trajectory + error plots via `trials/compare_paths.py`).

---

*Code embodiment:* `optimize/forward_model.py` implements the reduced forms (2.3), (3.2), (5.1)–
(5.6) and (4.3); `optimize/calibrate.py` performs the staged identification of §8; the signs in §7
are exactly what the optimiser's loss gradient follows.
