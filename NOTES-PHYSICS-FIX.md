# Physics corrections

Baseline: 155aebd. Run `npm test` for assertions and measurements or
`node test/measure.mjs` for the common before/after measurement protocol.
Three.js 0.170.0 was already installed locally; no dependencies were changed.

| Measurement | Before | After |
| --- | ---: | ---: |
| 0–100 km/h, full throttle | 4.2925 s | 4.5450 s |
| 100–0 braking, distance-derived mean | 0.9559 g | 0.9536 g |
| 100–0 braking distance | 41.160 m | 41.251 m |
| Largest lateral g in tested circle sweep | 0.8361 g | 0.8342 g |
| 10° slope drift, 10 s | 255.141 mm | <0.000001 mm |
| 20° slope drift, 10 s | 502.529 mm | <0.000001 mm |

Tests integrate at 400 Hz. Braking starts at the first step reaching 100 km/h
and ends below 0.01 m/s. Circles use a 20 m/s speed controller, 12 s at each
front steer command (0.01, 0.02, 0.04, 0.08 rad); the table reports the largest
final acceleration, not a measured absolute handling limit. Final circle speeds
are 19.58–19.99 m/s. Slopes start aligned with the plane and settle for 5 s
before measuring 10 s of CG displacement. Baseline slopes use baked geometry
to isolate stiction from the separate mesh-normal bug. Regression tests also
cover rotated meshes, manual braking, Auto braking, and P.

## Changes

- Normalize the pure-slip force pair onto a mu*N circle. Reapply the cap after
  rolling resistance so the actual applied tangential force obeys the limit.
  This is the minimum friction-circle model; no new load sensitivity is claimed.
- Solve low-speed tangential contact velocities after integrating gravity,
  suspension and other forces. Thirty-two projected sequential impulse sweeps
  use chassis mass and rotational inertia. Accumulated impulses are limited by
  the friction circle and longitudinal brake torque; unbraked wheels roll freely.
- Transform mesh normals with the object's world normal matrix before converting
  to map coordinates. Tests include a transformed parent and nonuniform scaling.
- Share motor limits across manual and Auto modes: 239 kW, 3200 Nm per axle,
  40/60 torque split, reverse restrictions and powerScale. The rear axle limit
  reduces total launch torque from 6400 to 5333 Nm and explains the slower launch.
- Replace the maximum-penetration body lift with per-contact normal position
  correction, capped at a combined 0.5 m/s. Retain suspension/bump-stop impulses.
  Remove the unconditional below-ground body teleport.
- Report CG acceleration projected on the actual body-left axis from the
  substep velocity derivative; latAcc is no longer v*yawRate. Existing exports,
  constructor, commands, step/place methods and report fields remain available.
  Other report conventions are unchanged.

## Validation and limits

`npm test` asserts braking in a 0.90–1.00 g band around the nominal 0.95 g,
increasing steady-circle response, slope drift below 1 mm, torque/power bounds,
sampled combined friction limits with aids on/off, transformed contact response,
true transient lateral acceleration, bounded asymmetric curb response and
finite state before the existing reset fallback can hide a failure.

The ISO 7401-like 0.04 rad step at nominal 20 m/s has a 10–90% yaw rise time
of 0.3675 s and 2.80% overshoot. These and the ISO 4138-like circle checks are
regression scenarios, not formal ISO certification or real-vehicle validation.
The straight neutral coasting test checks total kinetic, wheel, spring and
gravitational energy every substep with 0.01 J numerical tolerance; this is not
a proof of energy stability for arbitrary impacts or terrain.

Read the simulator's web/js/main.js import and its extended web/js/vehicle.js
implementation to check integration conventions. No files in
/home/kim/korea-autoware-sim were modified. No push was performed.
