# kdrive-physics

Vehicle dynamics for browser driving simulators, in one file (`src/vehicle.js`, depends only on three.js
for vectors and BVH-friendly raycasts). Written for **korea-autoware-sim** and kept here as its own piece.

- **Chassis**: 6-DOF rigid body (position, quaternion, linear/angular velocity, body inertia tensor with the
  gyroscopic term), semi-implicit integration at 400 Hz
- **Corners**: four independently sprung wheels; each casts a ray from its hardpoint onto your drivable
  meshes, so kerbs, crowns and slopes push the body. Spring, bump/rebound damping, anti-roll bars, bump stop
- **Tyres**: slip ratio and slip angle → Pacejka-style Fx/Fy combined on a friction ellipse; below walking pace
  the tyre becomes a velocity constraint (a braked car holds on a slope instead of creeping)
- **Wheels**: spin from drive (motor torque and power limits, front/rear split), brake torque and the tyre's own
  reaction, integrated semi-implicitly (the tyre stiffens with slip at low speed and an explicit step oscillates)
- **Driver aids**: TCS and ABS by per-wheel slip-ratio feedback; Ackermann steering with a rate-limited actuator
- **Commands**: manual (throttle/brake/steer, one-pedal regen) or auto (acceleration + front-wheel angle, as
  Autoware's `control_cmd` gives them)
- Reports at **base_link** (rear axle on the ground), the frame Autoware uses

```js
import { Vehicle } from 'kdrive-physics';
const veh = new Vehicle({ drivable: meshes, frame, groundZ, collides });   // see test/physics.mjs
veh.place(x, y, yaw); veh.gear = 'D'; veh.cmd.throttle = 1;
veh.step(1 / 60);   // x, y, z, yaw, pitch, roll, v, yawRate, latAcc, wheels[].Fz/omega
```

`npm test` drives it on a flat plane (numbers below from that run):

| | |
|---|---|
| 0–100 km/h | 4.3 s |
| 0.78 g steady turn | roll 3.6°, outer/inner load 7.9 / 2.1 kN |
| full brake from 117 km/h | 0.95 g, nose down 1.6° |
| 1.4 m/s² request from rest | 1.26 m/s² delivered (actuator lag + resistance) |
