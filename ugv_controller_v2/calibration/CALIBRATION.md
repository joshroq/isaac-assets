# Sim → real motion calibration (bake into the USD, no runtime scaler)

**Goal:** make the *simulated* rover move like the *real* rover, so a command that moves
the sim X also moves the real rover X — for straight driving, turning, and a recorded
obstacle‑avoidance sequence that must end at the same pose. This is done by tuning the v2
rover's **physics in the USD** and baking it once — **not** by scaling `/cmd_vel` at runtime.

## Why physics, not a scaler
A runtime `/cmd_vel` scaler only rescales one number and hides the real mismatch (slip,
skid‑steer under‑rotation, motor ramp). Tuning the sim's physics fixes the *model*, so the
same command stream produces matching motion open‑loop — which is exactly what "replay the
sim's obstacle sequence and land in the same spot" needs. It also means the sim you validate
Nav2 against is physically faithful, which is the Part‑1 sim‑to‑real thesis point.

## Workflow
1. **Measure the real rover** (batteries in, on the floor, tape measure + protractor or a
   marked 90°). Use the rover‑side helper to command a known motion and read odom:
   ```bash
   # on the rover (~/rover_nav/calib/):
   python3 calibrate_motion.py --linear 0.15 --time 4    # then tape-measure the real distance
   python3 calibrate_motion.py --angular 0.5 --time 4    # then measure the real angle
   ```
2. **Edit** `sim_calibration.yaml` toward the real numbers (guide below).
3. **Bake it into the USD (once):**
   ```bash
   ugv_controller_v2/calibration/run_apply_calibration.sh --dry-run   # preview
   ugv_controller_v2/calibration/run_apply_calibration.sh             # writes + backs up the USD
   ```
4. **Re‑verify in sim:** launch Isaac, command the same `0.15 m/s × 4 s` / `0.5 rad/s × 4 s`,
   confirm the sim now travels/turns the same as the real rover. Iterate 2–4.
5. **End‑to‑end check:** record the sim's `/cmd_vel` during an obstacle run and replay it on
   the real rover; with good calibration it lands near the same spot:
   ```bash
   python3 record_cmdvel.py run1.csv --topic /cmd_vel      # on the sim/desktop
   python3 replay_cmdvel.py run1.csv --topic /cmd_vel      # on the rover (no scaler)
   ```

## Parameters to fine‑tune (current USD values in `sim_calibration.yaml`)

### Primary — forward/back distance
- **`differential_controller.wheelRadius`** (0.04 m). Sim linear speed = wheelRadius × wheel
  angular velocity. If the real rover drives *farther* than sim for the same command+time,
  **raise** it (and vice‑versa). This is the main straight‑line knob.
- **`wheel_friction.corner_wheel`** (dyn 1.0 / stat 1.2). If the sim wheels *slip* (travel
  less than wheelRadius predicts) lower‑friction makes it worse; keep corner grip high so the
  driven wheels don't slip and wheelRadius stays the clean lever. `staticFriction` also sets
  launch/deadband feel.

### Primary — turning (skid‑steer)
- **`differential_controller.wheelDistance`** (0.5 m) — the **effective track width**. A
  4‑wheel skid‑steer physically resists yaw, so it turns *slower* than an ideal diff‑drive of
  its real track. **Raise `wheelDistance`** until the sim's degrees‑turned per command matches
  the real rover (this is your "distance between the wheels to overcome skid‑steering" knob).
- **`wheel_friction.middle_wheel`** (dyn 0.65 / stat 0.8). The passive middle wheels set how
  much the chassis fights rotation. **Raise** their friction → slower, more realistic skid
  turns; **lower** → freer turns. Tune the *ratio* vs `corner_wheel` alongside `wheelDistance`.

### Secondary — dynamics / accel / obstacle sequences
- **`wheel_drive.damping`** (10000) — velocity‑drive gain. **Lower** for softer acceleration
  and more lag (models the real motor ramp / deadband); this affects how quickly the rover
  reaches commanded speed, which matters for short segments in an obstacle sequence.
- **`wheel_drive.maxForce`** (3.4e38 ≈ ∞) — per‑wheel torque cap. **Lower** to model real
  motor torque limits / stalling against bumps and heavy turns.
- **`mass.base_link`** (1.405 kg) — raise toward the **measured** real mass (battery adds
  weight). More mass = more momentum (overshoot on stops), more normal force (traction).
- **`mass.wheel`** (0.08 kg each) — wheel inertia; affects spin‑up.
- **`maxLinearSpeed` / `maxAngularSpeed`** (0.5 / 1.9) — clamp to the real rover's true top
  speeds so Nav2 can't command motion the hardware can't deliver.

### Floor friction — lives in the ENVIRONMENT USD, not this one
The rover USD only holds the *wheel* materials. The **floor** physics material (dynamic/
static friction) is on the ground prim of `small_warehouse.usd` / the matched‑room USD. Tune
it there (same `physics:dynamicFriction` / `physics:staticFriction` attributes); wheel‑vs‑floor
friction combine at the contact, so adjust whichever side keeps the other reusable across rooms.

### Advanced (only if contact is unstable / jitter)
- Rigid‑body **solver iteration counts** (`physxRigidBody:solver*IterationCount`) and joint
  **contact/rest offset** on the wheel colliders — raise iterations for stiffer, less‑penetrating
  contact; usually leave alone unless the wheels visibly sink or buzz.
- **Center of mass** offset on `base_link` — shift if the real rover's weight bias (battery
  location) changes traction front/rear.

## Notes
- `apply_sim_calibration.py` backs up the USD (`*.bak-<ts>-calib`) before saving, and is
  idempotent (re‑running with the same yaml is a no‑op).
- Editing values in the Isaac Sim GUI and saving the USD is equivalent; the yaml + script
  just make the calibration **reproducible and version‑controlled**. Prim paths are in
  `apply_sim_calibration.py` (e.g. `/World/ugv_rover/Drive_Graph/SkidSteer_Controller/DiffController`).
