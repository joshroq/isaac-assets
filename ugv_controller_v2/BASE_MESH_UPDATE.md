# Shared chassis geometry

The active Isaac rover uses `base_link_fixed2.stl` (2026-10-07 replacement).
The canonical file is in
`ugv_ws/src/ugv_main/ugv_description/meshes/ugv_rover/`.
`base_link_fixed.stl` is an identical compatibility copy for existing mesh users.

## Where the update reaches

Both `jetson_rover.usd` and `jetson_rover_v2.usd` reference
`urdf/ugv_rover_isaac_fixed/ugv_rover_isaac_fixed.usd` in `ugv_description`.
Its physics variant sublayers `configuration/ugv_rover_isaac_fixed_base.usd`.
That base layer contains one shared mesh at
`/meshes/base_link_fixed/node_STL_BINARY_/mesh`, referenced by both chassis
visuals and colliders. The prim name deliberately stays stable: downstream USD
overrides, materials and collision schemas continue to resolve.

Updating this shared mesh reaches the v1 controller, every v2 environment runner,
`sim2real_calib/optimize/sim_eval.py`, and experiment renderers that open the rover
USD. Environments do not contain separate rover copies. The v2 navigation
loaders already read `ugv_rover_isaac_fixed.urdf`; the older Isaac navigation
loaders now use it too, retaining their `3d_camera_link` TF alias. The fixed URDF
uses the new STL for both visual and collision geometry, and its camera mesh
filename now resolves directly to the existing `3d_camera_link.stl`.

The local colcon install is synchronized as well. The hardware viewer still
uses the separate hardware URDF. Historical pre-fix imports, Gazebo/hardware
models, backup files and recorded experiment stages are not active fixed-rover
dependencies and are preserved.

**New launches pick up the update without rebuilding or reimporting the rover.**
An already-open Isaac stage or RViz robot description must be reloaded/restarted;
an STL/USD file change cannot safely recook a running articulation automatically.

## Geometry and controller audit

All coordinates below are in metres in the existing `base_link` frame.

| Property | Previous mesh | New mesh |
| --- | ---: | ---: |
| Triangle count | 374 | 4574 |
| X bounds | -0.122598983 to 0.122598983 | unchanged |
| Y bounds | -0.115209997 to 0.115209997 | unchanged |
| Z bounds | -0.056611992 to 0.040000000 | -0.056611992 to 0.085700005 |

332 triangles are exactly shared. All 42 removed upper-deck triangles reappear
translated by **+0.0457 m in Z**, matching within 1 micrometre. The new mesh adds
four standoffs and an intermediate board. Lower chassis geometry and the base
coordinate frame are preserved. Wheel geometry stays fixed, while deck-mounted
parts must move up with the translated deck.

The URDF camera, lidar and pan-base joint origins are raised by 45.7 mm. In USD,
all six deck/arm link poses are raised (the importer stores them as siblings),
along with the three base-to-deck joint anchors. Both top-level rover scenes
also have a stronger camera-position override; it is raised by the same amount,
preserving its existing adjustment. Attached lights, optical frames, lidar and
camera children follow their parent links. The chassis IMU frame stays at the
existing base-frame origin.

The USD points, triangle topology, normals, per-vertex color array and extent
are regenerated together. Normals are computed from triangle winding because
seven facet normals in the downloaded STL oppose their winding. Existing
collision approximation (`convexHull`), material bindings and prim paths remain
intact; PhysX cooks the new shape when the stage loads.

| Dependent setting | Audited result |
| --- | --- |
| Wheel joint X | -0.0855, 0, +0.0855; unchanged |
| Wheel joint Y | +/-0.08726; physical track 0.17452; unchanged |
| Wheel joint Z | -0.041212; unchanged |
| Corner wheelbase | 0.171; unchanged |
| Physical/controller wheel radius | 0.04; unchanged |
| v1 controller separation | 0.175 (existing rounded hardware nominal); unchanged |
| v2 `DiffController.inputs:wheelDistance` | 0.5 effective skid-steer track; unchanged |
| Surface calibration `wheelDistance` | Per-surface slip compensation; preserved |
| Camera mount joint Z relative to base | 0.021953 -> 0.067653 |
| Lidar and pan-base mount joint Z relative to base | 0.04 -> 0.0857 |
| ROS TF graphs | Read raised USD mounts on playback; no hardcoded offsets to patch |
| Nav2 planar footprint / calibration planar model | XY geometry unchanged |
| Base mass, COM, inertia | Existing authored values preserved |

STL contains a surface, not component densities or measured mass properties.
Mount movement comes from matching the translated deck triangles, not just the
bounding box. Mass properties of each moved sensor/arm link remain local to that
link and follow it. The base mesh also gains standoffs and board detail, but the
STL does not supply their masses: the base's authored mass, COM and inertia are
preserved rather than replaced with a uniform-density estimate. Measured base
mass properties and surface calibration should be revalidated if the physical
hardware changed too.

## Regenerate and verify

From the repository root, after an audited edit to the canonical STL:

```bash
bash ugv_controller_v2/tools/sync_base_mesh.sh
bash ugv_controller_v2/tools/sync_base_mesh.sh --check
```

The first command updates the shared USD mesh, compatibility STL and existing
colcon package copies. The deck lift is recorded in each affected layer's
metadata and applied only once, so repeated runs do not accumulate offsets.
The second is read-only: it checks URDF mesh resolution,
source hashes, geometry arrays, both composed rover scenes, the installed asset,
all six wheel joint offsets, and raised deck link poses/joint anchors. Both use Isaac's standalone USD libraries;
override `ISAAC_ROOT`, `ISAAC_PY` or `PXR` for another installation.

Source SHA-256:
`d01fa17a2cfb67b9241908b3533acded821b47f233ed3ffd01d6f536cbaa6fe8`.

## Validation performed

- Source and installed USD composition checks passed for visuals, collisions,
  all six wheel anchors, all six raised deck/arm link poses and three raised
  mounting joints, including both top-level rover scenes.
- All ten legacy/v2 Isaac navigation description loaders resolve every mesh and
  load the corrected lidar height, with legacy camera frame names preserved.
- A property comparison against the previous USD layers found only the mesh
  arrays, specified mount translations/anchors and provenance metadata changed.
  Drive/ROS graph properties, sensor-local offsets and physics parameters match.
- A second synchronization produced byte-identical USD files: no double lift.
- A headless Isaac/PhysX test settled the rover for two seconds, then drove the
  four corner wheels at 3 rad/s for two seconds. The rover stayed level and moved
  0.237 m. Network sensor assets and ROS graphs were disabled for this isolated
  physics test; full ROS topic delivery and navigation were not exercised.
