# jetson_rover.usd manual checklist

Use this when editing `jetson_rover.usd` by hand in Isaac Sim. Do not use an
apply script for this pass.

These settings target the published rover spec:

- total mass: `2.190 kg`
- tire diameter: `80 mm`
- wheel radius in code: `0.040 m`
- max linear speed in code: `1.3 m/s`
- number of driving wheels: `4`
- number of wheels touching ground: `6`

## Code-side values

These are already set in `ugv_controller/common/rover_drive_controller.py` and
`ugv_controller/common/ros2_bridge_drive_graph.py`:

- `WHEEL_RADIUS = 0.04`
- `WHEEL_SEPARATION` / `WHEEL_DISTANCE = 0.175`
- `MAX_LINEAR_VEL` / `MAX_LINEAR_SPEED = 1.3`
- `MAX_ANGULAR_VEL` / `MAX_ANGULAR_SPEED = 1.0`
- corner wheel joints only:
  - `left_up_wheel_link_joint`
  - `left_down_wheel_link_joint`
  - `right_up_wheel_link_joint`
  - `right_down_wheel_link_joint`

The Python controller preserves authored USD corner-wheel
`drive:angular:physics:maxForce` values. If the attribute is missing, the
fallback is now `3.4028235e38`, matching the effectively unlimited value already
used by the asset instead of capping the wheel drives.

## Do not change

- Do not change wheel dimensions, wheel scale, chassis dimensions, or collider
  shape for this pass. You said the dimensions are correct.
- Do not lower the four corner-wheel `drive:angular:physics:maxForce` values.
  Leave the high authored values exactly as they are.
- Do not set `physics:breakForce` or `physics:breakTorque` to zero. Leave them
  unset/default/infinite so the joints do not break.
- Do not use compliant wheel contact. Leave
  `physxMaterial:compliantContactStiffness` and
  `physxMaterial:compliantContactDamping` unset.

## 1. Free the two center wheel joints

These are idler wheels. They should spin freely but remain attached and
colliding with the ground.

Edit these joint prims:

- `left_middle_wheel_link_joint`
- `right_middle_wheel_link_joint`

For each one, in the Property panel:

Angular Drive:

- `drive:angular:physics:type` -> `velocity` is fine, or remove Angular Drive
  entirely.
- `drive:angular:physics:stiffness` -> `0`
- `drive:angular:physics:damping` -> `0`
- `drive:angular:physics:maxForce` -> `0`

Physx Joint:

- `physxJoint:jointFriction` -> `0`
- `physxJoint:armature` -> `0`

Leave these alone:

- `physics:breakForce`
- `physics:breakTorque`
- all collider enable/disable settings

## 2. Keep the four corner drive joints strong

Verify these four joints:

- `left_up_wheel_link_joint`
- `left_down_wheel_link_joint`
- `right_up_wheel_link_joint`
- `right_down_wheel_link_joint`

Angular Drive:

- `drive:angular:physics:type` -> `velocity`
- `drive:angular:physics:stiffness` -> `0`
- `drive:angular:physics:damping` -> `10000`
- `drive:angular:physics:maxForce` -> leave the existing very high authored USD
  value unchanged.

## 3. Set realistic tire rubber materials

Use the same rubber friction values for corner and middle wheels. The middle
wheels are idlers, but they are still the same rubber tires.

Create or edit physics materials under a clear location such as:

- `/World/Physics_Materials/CornerWheel`
- `/World/Physics_Materials/MiddleWheel`

CornerWheel material:

- `physics:staticFriction` -> `1.0`
- `physics:dynamicFriction` -> `0.8`
- `physics:restitution` -> `0.0`
- `physxMaterial:frictionCombineMode` -> `average`
- `physxMaterial:restitutionCombineMode` -> `average`
- clear/unset `physxMaterial:compliantContactStiffness`
- clear/unset `physxMaterial:compliantContactDamping`

MiddleWheel material:

- `physics:staticFriction` -> `1.0`
- `physics:dynamicFriction` -> `0.8`
- `physics:restitution` -> `0.0`
- `physxMaterial:frictionCombineMode` -> `average`
- `physxMaterial:restitutionCombineMode` -> `average`
- clear/unset `physxMaterial:compliantContactStiffness`
- clear/unset `physxMaterial:compliantContactDamping`

Bind materials to wheel collision prims, not only visual meshes:

- collision prims under `left_up_wheel_link` -> `CornerWheel`
- collision prims under `left_down_wheel_link` -> `CornerWheel`
- collision prims under `right_up_wheel_link` -> `CornerWheel`
- collision prims under `right_down_wheel_link` -> `CornerWheel`
- collision prims under `left_middle_wheel_link` -> `MiddleWheel`
- collision prims under `right_middle_wheel_link` -> `MiddleWheel`

If skid turns chatter or bind after this, lower only the middle wheel material
to `staticFriction = 0.8`, `dynamicFriction = 0.65`. Do not start with lower
values; start with realistic matching rubber first.

## 4. Set realistic ground material

For concrete, warehouse floor, or default ground plane:

- `physics:staticFriction` -> `1.0`
- `physics:dynamicFriction` -> `0.8`
- `physics:restitution` -> `0.0`
- `physxMaterial:frictionCombineMode` -> `average`
- `physxMaterial:restitutionCombineMode` -> `average`

These values represent dry rubber on a normal hard indoor floor. Avoid very low
ground friction because it causes the rover to slide and fail to stop cleanly.

## 5. Reset wheel contact offsets

For each actual wheel collision prim, in Physx Collision:

- `physxCollision:contactOffset` -> `0.005`
- `physxCollision:restOffset` -> `0.0`

Apply this to collision prims under:

- `left_up_wheel_link`
- `left_down_wheel_link`
- `right_up_wheel_link`
- `right_down_wheel_link`
- `left_middle_wheel_link`
- `right_middle_wheel_link`

Do not use large contact offsets such as `0.02` on these 80 mm tires. That can
make the rover feel like it is riding on inflated invisible geometry.

## 6. Set the rover mass to 2.190 kg

Your current provided masses sum to about `4.33134 kg`:

- main chassis piece: `2.78375`
- lidar mesh: `0.0418`
- front camera link: `0.04332`
- each wheel: `0.20709` x 6
- rover arm base link: `0.10953`
- arm hinge part 1: `0.00711`
- arm hinge part 2: `0.09705`
- top camera link: `0.00624`

Set target masses manually in Mass API so the total is `2.190 kg`:

- main chassis/base link: `1.40495 kg`
- lidar mesh: `0.04180 kg`
- front camera link: `0.04332 kg`
- each wheel link: `0.08000 kg`
- rover arm base link: `0.10953 kg`
- arm hinge part 1: `0.00711 kg`
- arm hinge part 2: `0.09705 kg`
- top camera link: `0.00624 kg`

Check: `1.40495 + 0.04180 + 0.04332 + 6*0.08000 + 0.10953 + 0.00711 + 0.09705 + 0.00624 = 2.19000 kg`.

For every prim where you manually edit mass:

- set `physics:mass` to the value above.
- set diagonal inertia to auto-compute if Isaac exposes that option.
- if auto-compute is not available, clear any stale custom inertia values rather
  than keeping inertia from the old 4.33 kg model.

## 7. Set articulation solver iterations

Select the articulation root prim:

- `/World/ugv_rover/base_footprint`

In Physx Articulation:

- `physxArticulation:solverPositionIterationCount` -> `32`
- `physxArticulation:solverVelocityIterationCount` -> `4`

This improves contact stability without changing geometry or reducing corner
wheel drive force.

## Verification

After saving the USD:

1. Start the sim and let the rover settle before driving.
2. Send `cmd_vel` forward at `linear.x = 0.2`. It should start promptly and stop
   promptly when command velocity returns to zero.
3. Send a pivot command such as `linear.x = 0.0`, `angular.z = 0.3`. It should
   yaw without the center wheels acting like powered brakes.
4. If it still slides while stopping, first confirm all four corner wheel
   colliders are actually bound to `CornerWheel` and the floor has friction.
5. If it refuses to pivot, lower only `MiddleWheel` friction to
   `staticFriction = 0.8`, `dynamicFriction = 0.65`.
