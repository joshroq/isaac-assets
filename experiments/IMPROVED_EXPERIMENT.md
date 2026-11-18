# Improved experiment design — visual fidelity → nav performance → sim-to-real

## The real thesis question (reframed)
> *How does the **visual fidelity** of the simulated environment affect the **performance of
> the navigation system** driving the virtual rover, and how does that compare to the **real
> rover in the real environment**?*

The current matrix answers a *different* question — it compares nav **stacks** across goals and
cold/known-map. That's useful, but it treats the sim as fixed. The thesis wants the sim's
**visual fidelity as the independent variable** and the **real rover as the anchor**. So the
experiment should be restructured around fidelity, not around goals.

## Key insight that should drive the design
Visual fidelity only matters for perception that *uses the rendered image*:
- **Camera-based stacks (depth, rtab, cuVSLAM, ORB-SLAM3)** — performance should degrade as
  fidelity drops (fewer/worse features, noisier depth, bad relocalization). **These are the
  signal.**
- **LiDAR (slam_toolbox/AMCL)** — raycasts geometry, essentially fidelity-invariant. **Use it
  as the control/baseline**: whatever changes in lidar across fidelity levels is *not* a
  fidelity effect, so it calibrates out sim-side confounds (physics, timing, DDS).

So: **vary fidelity, measure the camera stacks, hold lidar as the invariant control, anchor
everything to the real rover.**

## Independent variable — visual fidelity levels (concrete levers)
Define 3–4 discrete levels by combining Isaac render + asset knobs (script them so a level is
one flag). Suggested levers, low→high:
| lever | Low | Medium | High |
|---|---|---|---|
| renderer | path-traced off / fewer SPP | RTX real-time | RTX + more SPP / DLSS quality |
| textures | flat albedo, no normal maps | base textures | full PBR (albedo+normal+roughness) |
| lighting | ambient only | 1 directional | GI / area lights + shadows |
| materials | matte, single colour | default warehouse | reflective/again realistic |
| obstacle detail | primitive boxes | current shapes | detailed meshes |
| camera | 640×360, no noise | 640×360 | native + sensor noise/blur model |

Keep **geometry identical** across levels (same collision, same map) so only *appearance*
changes — that isolates the fidelity effect from a navigation-difficulty change.

## Dependent variables (per trial)
Keep the current outcome metrics **and add perception metrics** that expose fidelity effects:
- outcome: success, path_length, path_efficiency, final_pos_error, duration, n_recoveries.
- perception: SLAM **tracking-loss events**, feature/keypoint count, relocalization count,
  map→odom drift vs ground truth, camera effective Hz. (These are where fidelity shows up
  before success/failure does.)

## Fixed factors (hold constant — the current experiment didn't)
- **Protocol: independent per-trial runs, NO return-home.** The return-home leg corrupts
  localization on long drives (confirmed: `known_map_lidar_G2` trial02 had 26% of its path
  inside obstacles because AMCL was in a wrong frame after returning from the far goal). Use
  `experiments/run_independent.sh` (fresh Isaac + stack + gated init per trial) for **every**
  stack — this is the fix and it removes the artifact entirely.
- **Clean environment per session.** A ~13 h marathon left stale `/dev/shm/fastrtps_*` +
  zombie `/clock` publishers that made Isaac "boot" without real data → the rtab cold-start
  cells all failed with `no /clock`. Reboot (or a hard
  pkill + `rm /dev/shm/fastrtps_*` + `ros2 daemon stop`) between sessions; verify a *fresh*
  `/clock` at ≥ real rate before trusting a run.
- Same Nav2 planner/controller/costmap/footprint across all stacks and levels.
- Same start pose, same goals, same trial count. Calibrate sim physics first
  (`ugv_controller_v2/calibration/`) so motion matches the real rover.

## The sim-to-real anchor (the point of the thesis)
1. Run the **real rover** in the **real environment** on the same goal set, N trials — this is
   ground truth (`~/rover_nav/` scripts already exist).
2. For each fidelity level in sim, compute the **sim-to-real gap** per metric
   (|sim − real|), per nav stack.
3. Result: a curve of gap vs fidelity → identifies the **"sufficient fidelity" threshold**
   (the cheapest level whose camera-nav performance matches real). That's a genuinely novel,
   defensible thesis result: *how much visual realism does a nav system actually need for the
   sim to predict real behaviour?*

## Reduced, well-powered cell set (vs the sprawling current matrix)
- Fewer goals (2 representative: one near/open, one far/cluttered), **more trials** (≥ 8–10)
  for statistics.
- Stacks: **lidar (control), depth, rtab** now; **cuVSLAM** once a **≥ 16 GB GPU** is
  available (it heap-crashes on the 8 GB card — see `RESULTS_STATUS.md`); ORB-SLAM3 optional.
- Conditions: keep cold-start vs known-map only if relevant to your claim; fidelity × stack ×
  sim/real is already 3-D.

## Hardware / repro
- **≥ 16 GB VRAM** for the camera-heavy + cuVSLAM runs (the current 8 GB RTX 5060 Ti is the
  binding constraint). The RTX 5000 Ada (32 GB) box would run the whole thing comfortably.

## What already exists to build on
- `run_independent.sh` (the no-return-home protocol) — the fix, generalised to any stack.
- `play_replay.py` (media-remote viewer) + the Stage-Recorder sampled-take capture in
  `run_baked_rover.py` — physics-neutralised sampled replays in Isaac.
- `replay3d/` — offline 3D trajectory replays with the real URDF robot.
- `make_occupancy_map.py`, `plot_paths.py`, `summarize.py`, `collect_results.py` — mapping,
  figures, aggregation.
What's **new** for the thesis: a `--fidelity {low,med,high}` knob in the Isaac runner wiring
the render/asset levers above, and a sim-vs-real gap analysis.
