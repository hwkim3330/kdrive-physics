// kdrive-physics: vehicle dynamics for browser driving simulators (three.js only).
//
// A 6-DOF rigid chassis on four independently sprung wheels:
//   chassis   position/quaternion/velocities in the map frame (x east, y north, z up), inertia
//             tensor in the body frame, integrated semi-implicitly at 400 Hz
//   corners   each wheel casts a ray from its hardpoint down the body's -z onto the drivable
//             meshes (BVH): spring + damper + anti-roll bar give the normal load, the hit face's
//             normal gives the contact frame, so curbs, crowns and slopes act on the body
//   tyres     slip ratio and slip angle -> Pacejka-style Fx/Fy, combined on a friction ellipse;
//             at walking pace the tyre becomes a velocity constraint (impulse friction capped by
//             mu*Fz) so a braked car sits still on a slope instead of creeping
//   wheels    spin from drive torque (motor torque/power limits, front/rear split), brake
//             torque and the tyre's own Fx; Ackermann steering on the front pair
//   commands  manual: throttle/brake/steer; auto (Autoware or the pilot): acceleration and
//             front-wheel angle, turned into torques with a first-order actuator lag
// Everything the rest of the sim reads is reported at base_link (rear axle on the ground).
import * as THREE from 'three';
// geometry of the car (IONIQ 6 / Model 3 class). Edit before constructing for another car.
export const SPEC = { wheelbase: 2.875, track: 1.58, tyreR: 0.335 };

const G = 9.81, MASS = 2000;
const I_BODY = [650, 3100, 3400];                   // roll, pitch, yaw [kg m^2]
const L = SPEC.wheelbase, LR = 1.45, LF = L - LR;   // CG ahead of the rear axle
const CG_H = 0.55, TRACK = SPEC.track, R_W = SPEC.tyreR, I_W = 1.3;
const REST = 0.34, K_S = 42000, C_BUMP = 3400, C_REB = 4600, ARB = 8000, TRAVEL = 0.17;
const SAG = (MASS * G) / 4 / K_S;                   // static compression (~0.12 m)
const MU = 1.0, PAC = { Bx: 11, Cx: 1.65, By: 9, Cy: 1.35 };
const P_MAX = 239e3, T_MAX = 3200;                  // motor power [W], per-axle wheel torque [Nm]
const DRIVE_SPLIT = 0.4;                             // front share (dual motor)
const T_BRAKE = MASS * 9.5 * R_W;                    // full brake: ~0.97 g
const BRAKE_SPLIT = 0.62;
const CDA = 0.23 * 2.3, RHO = 1.2, CRR = 0.010;
const MAX_STEER = 0.61, STEER_RATE = 0.9, STEER_TAU = 0.10, ACC_TAU = 0.15;
const SUB = 1 / 400;

// corners relative to the CG in the body frame (x fwd, y left, z up)
const CORNERS = [
  { x: LF, y: TRACK / 2, front: true }, { x: LF, y: -TRACK / 2, front: true },
  { x: -LR, y: TRACK / 2, front: false }, { x: -LR, y: -TRACK / 2, front: false },
];
const HARD_Z = R_W + REST - SAG - CG_H;              // hardpoint height above the CG, so the CG rests at CG_H

const v3 = () => new THREE.Vector3();

export class Vehicle {
  constructor(world) {
    this.world = world;
    this.p = v3(); this.q = new THREE.Quaternion(); this.vel = v3(); this.w = v3();
    this.wheels = CORNERS.map((c) => ({ ...c, omega: 0, comp: 0, compPrev: 0, Fz: 0, slip: 0, ground: false, steer: 0, kappa: 0, tcs: 1, abs: 1 }));
    this.aids = true;                                  // TCS + ABS, as every modern EV has
    this.gear = 'P'; this.mode = 'manual';
    this.cmd = { throttle: 0, brake: 0, steer: 0, accel: 0, delta: 0 };
    this.delta = 0; this.a = 0; this.aCmd = 0;
    this.braking = false; this.odometer = 0; this.collided = 0; this.slip = 0;
    this.ray = new THREE.Raycaster(); this.ray.firstHitOnly = true;
    this._down = new THREE.Vector3(0, -1, 0);
    this._acc = 0;
    this._report();
  }

  // ---------------------------------------------------------------- placement / ground queries
  place(x, y, yaw) {
    const gz = this.heightAt(x, y) ?? this.world.groundZ(x, y);
    this.q.setFromAxisAngle(new THREE.Vector3(0, 0, 1), yaw);
    const c = Math.cos(yaw), s = Math.sin(yaw);
    this.p.set(x + LR * c, y + LR * s, gz + CG_H);
    this.vel.set(0, 0, 0); this.w.set(0, 0, 0);
    for (const wh of this.wheels) { wh.omega = 0; wh.comp = wh.compPrev = SAG; }
    this.delta = 0; this.a = 0;
    this._report();
  }

  // ground under a map point: {z, n} (n in map frame) or null
  _ground(x, y) {
    const f = this.world.frame, o = f.toThree(x, y, 300);
    this.ray.set(o, this._down); this.ray.far = 800;
    const hit = this.ray.intersectObjects(this.world.drivable, false)[0];
    if (!hit) return null;
    const n = hit.face ? hit.face.normal : { x: 0, y: 1, z: 0 };
    let nx = n.x, ny = -n.z, nz = n.y;                // three (x, y-up, z-south) -> map
    if (nz < 0) { nx = -nx; ny = -ny; nz = -nz; }     // double-sided meshes
    return { z: hit.point.y, n: new THREE.Vector3(nx, ny, nz) };
  }
  heightAt(x, y) { const g = this._ground(x, y); return g ? g.z : null; }

  // ---------------------------------------------------------------- one frame
  step(dt) {
    this._acc += dt;
    let n = 0;
    while (this._acc >= SUB && n < 40) { this._sub(SUB); this._acc -= SUB; n++; }
    // never let a bad step poison the sim: fall back to the last finite pose, at rest
    const ok = [this.p.x, this.p.y, this.p.z, this.vel.x, this.vel.y, this.vel.z, this.w.x, this.w.y, this.w.z, this.q.w].every(Number.isFinite);
    if (!ok) {
      const g = this._good || { x: this.x || 0, y: this.y || 0, yaw: this.yaw || 0 };
      console.warn('vehicle physics reset at', g);
      this.place(g.x, g.y, g.yaw);
    } else if (Math.abs(this.vel.z) < 5) this._good = { x: this.x, y: this.y, yaw: this.yaw };
    this._report();
  }

  _commands(dt) {
    const c = this.cmd;
    // steering actuator (rate-limited first order), same for driver and autopilot
    const want = THREE.MathUtils.clamp(this.mode === 'auto' ? c.delta : c.steer * MAX_STEER, -MAX_STEER, MAX_STEER);
    this.delta += THREE.MathUtils.clamp((want - this.delta) * Math.min(1, dt / STEER_TAU), -STEER_RATE * dt, STEER_RATE * dt);
    const vx = this.vx;
    let drive = 0, brake = 0;                           // total wheel torques [Nm], drive signed by gear
    if (this.mode === 'auto') {
      // acceleration command -> torque, lagged like a real powertrain/brake actuator
      this.aCmd += (c.accel - this.aCmd) * Math.min(1, dt / ACC_TAU);
      const resist = (0.5 * RHO * CDA * vx * vx + CRR * MASS * G) * Math.sign(vx || 1);
      const F = MASS * this.aCmd + (Math.abs(vx) > 0.2 ? resist : 0);
      const dir = this.gear === 'R' ? -1 : 1;
      if (F * dir > 0 && this.gear !== 'P' && this.gear !== 'N') drive = F * R_W;
      else brake = Math.min(T_BRAKE, Math.abs(F) * R_W + (Math.abs(vx) < 0.3 && c.accel <= 0.05 ? T_BRAKE * 0.4 : 0));
      this.braking = c.accel < -0.4;
    } else {
      const sgn = this.gear === 'R' ? -1 : 1;
      if (this.gear === 'D' || this.gear === 'R') {
        const tAxle = Math.min(T_MAX * 2, P_MAX / Math.max(Math.abs(vx), 1) * R_W);
        drive = sgn * c.throttle * tAxle * (this.gear === 'R' ? 0.35 : 1) * (this.powerScale ?? 1);
        if (this.gear === 'R' && -vx > 3.2) drive = 0;                          // reverse capped ~11 km/h
        // one-pedal regen when the accelerator is lifted
        // lift-off regen: level 0..3, or i-Pedal (4) which brakes all the way to a stop and holds
        const lvl = this.regenLevel ?? 4;
        const regen = [0, 0.6, 1.1, 1.6, 2.2][lvl];
        if (c.throttle < 0.02 && (Math.abs(vx) > 0.3 || lvl === 4)) brake += MASS * Math.min(regen, (lvl === 4 ? 0.8 : 0.35) * regen + Math.abs(vx) * 0.06) * R_W + (lvl === 4 && Math.abs(vx) < 0.5 ? T_BRAKE * 0.3 : 0);
      }
      brake += c.brake * T_BRAKE;
      if (this.gear === 'P') brake = T_BRAKE;
      this.braking = c.brake > 0.05 || (c.throttle < 0.02 && Math.abs(vx) > 0.5 && (this.regenLevel ?? 4) >= 2);
    }
    return { drive, brake };
  }

  _sub(dt) {
    const { drive, brake } = this._commands(dt);
    const R = new THREE.Matrix3().setFromMatrix4(new THREE.Matrix4().makeRotationFromQuaternion(this.q));
    const ex = new THREE.Vector3(1, 0, 0).applyMatrix3(R), ey = new THREE.Vector3(0, 1, 0).applyMatrix3(R), ez = new THREE.Vector3(0, 0, 1).applyMatrix3(R);
    const F = new THREE.Vector3(0, 0, -MASS * G), T = v3();
    // aero + rolling resistance on the body
    const vxb = this.vel.dot(ex);
    F.addScaledVector(ex, -0.5 * RHO * CDA * vxb * Math.abs(vxb));

    // suspension compression first (anti-roll bars need both sides)
    const hit = [];
    for (const wh of this.wheels) {
      const hp = this.p.clone().addScaledVector(ex, wh.x).addScaledVector(ey, wh.y).addScaledVector(ez, HARD_Z);
      const g = this._ground(hp.x, hp.y);
      let comp = 0, gpt = null;
      if (g) {
        // distance along -ez from the hardpoint to the ground plane through the hit
        const gp = new THREE.Vector3(hp.x, hp.y, g.z);
        const denom = -ez.dot(g.n) || -1;
        const d = gp.clone().sub(hp).dot(g.n) / denom;
        comp = REST + R_W - d;
        gpt = hp.clone().addScaledVector(ez, -d);
      }
      wh.compPrev = wh.comp; wh.comp = THREE.MathUtils.clamp(comp, -1, TRAVEL + 0.2);
      hit.push({ hp, g, gpt });
    }
    // penetration past the bump stop is resolved as a position constraint, not a force: a tile that
    // loads under the car, or a kerb met at speed, lifts the body instead of launching it
    const over = Math.max(...this.wheels.map((w) => w.comp - TRAVEL));
    if (over > 0.01) {
      this.p.addScaledVector(ez, over);
      const vn = this.vel.dot(ez); if (vn < 0) this.vel.addScaledVector(ez, -vn);
      for (const w of this.wheels) { w.comp -= over; w.compPrev = Math.min(w.compPrev, w.comp); }
    }
    const arbF = [(this.wheels[0].comp - this.wheels[1].comp) * ARB, (this.wheels[2].comp - this.wheels[3].comp) * ARB];

    let slipMax = 0;
    this.wheels.forEach((wh, i) => {
      const { g, gpt } = hit[i];
      wh.ground = !!g && wh.comp > 0;
      // drive/brake torque at this wheel
      const front = wh.front;
      // traction control and ABS: per-wheel slip-ratio feedback on drive and brake torque
      if (this.aids) {
        const k = wh.kappa * Math.sign(this.vx || 1);
        wh.tcs = k > 0.10 ? Math.max(0.05, wh.tcs * 0.92) : Math.min(1, wh.tcs + 0.02);
        wh.abs = k < -0.12 ? Math.max(0.1, wh.abs * 0.85) : Math.min(1, wh.abs + 0.04);
      }
      const tDrive = drive * (front ? DRIVE_SPLIT : 1 - DRIVE_SPLIT) / 2 * (this.aids ? wh.tcs : 1);
      const tBrake = brake * (front ? BRAKE_SPLIT : 1 - BRAKE_SPLIT) / 2 * (this.aids ? wh.abs : 1);
      if (!wh.ground) {
        wh.Fz = 0;
        wh.omega += (tDrive / I_W) * dt;
        wh.omega -= Math.sign(wh.omega) * Math.min(Math.abs(wh.omega), (tBrake / I_W) * dt);
        return;
      }
      // normal load: spring + damper (bump/rebound) + anti-roll bar, never pulling
      const rate = (wh.comp - wh.compPrev) / dt;
      const side = i % 2 === 0 ? 1 : -1, arb = arbF[front ? 0 : 1] * side;
      let Fz = K_S * Math.min(wh.comp, TRAVEL) + (rate > 0 ? C_BUMP : C_REB) * rate + arb;
      if (wh.comp > TRAVEL) Fz += 200000 * (wh.comp - TRAVEL);            // bump stop (soft; the constraint above does the rest)
      Fz = Math.min(Math.max(0, Fz), MASS * G * 2.5);
      wh.Fz = Fz;
      // contact frame: wheel heading projected on the ground plane
      const n = g.n;
      const st = front ? this._ackermann(wh.y) : 0;
      wh.steer = st;
      const head = ex.clone().multiplyScalar(Math.cos(st)).addScaledVector(ey, Math.sin(st));
      const fw = head.addScaledVector(n, -head.dot(n)).normalize();
      const lat = new THREE.Vector3().crossVectors(n, fw);
      const r = gpt.clone().sub(this.p);
      const vc = this.vel.clone().add(new THREE.Vector3().crossVectors(this.w, r));
      const vx = vc.dot(fw), vy = vc.dot(lat);
      const mu = MU * Fz;
      let Fx, Fy;
      const speed = Math.abs(vx);
      if (speed > 2.0 || Math.abs(wh.omega * R_W) > 2.5) {
        // Pacejka-style magic formula on slip ratio / slip angle, combined on the ellipse
        const kappa = (wh.omega * R_W - vx) / Math.max(speed, 0.5);
        wh.kappa = kappa;
        const alpha = Math.atan2(vy, Math.max(speed, 0.5));
        const fx0 = Math.sin(PAC.Cx * Math.atan(PAC.Bx * kappa)), fy0 = -Math.sin(PAC.Cy * Math.atan(PAC.By * alpha));
        const sx = kappa, sy = Math.tan(alpha), s = Math.hypot(sx, sy) || 1e-6;
        const comb = Math.hypot(fx0 * sx / s, fy0 * sy / s) > 1 ? 1 / Math.hypot(fx0 * sx / s, fy0 * sy / s) : 1;
        Fx = mu * fx0 * comb; Fy = mu * fy0 * comb;
        slipMax = Math.max(slipMax, Math.abs(alpha), Math.min(1, Math.abs(kappa)) * 0.5);
      } else {
        // walking pace: the tyre is a velocity constraint (stiction), capped by the friction circle
        // longitudinal: drive passes straight through; the brake is a constraint that stops the car
        // and then holds it (on a slope too) up to the brake's torque; free rolling has no hold
        const c = MASS / 4 / (dt * 6);
        const bMax = tBrake / R_W;
        Fx = tDrive / R_W + (bMax > 0 ? THREE.MathUtils.clamp(-vx * c, -bMax, bMax) : 0);
        Fy = -vy * c;
        const mag = Math.hypot(Fx, Fy);
        if (mag > mu) { Fx *= mu / mag; Fy *= mu / mag; }
        wh.omega = vx / R_W;                                              // rolling, no slip
        wh.kappa = 0;
      }
      // wheel spin: drive - brake - tyre reaction
      if (speed > 2.0 || Math.abs(wh.omega * R_W) > 2.5) {
        // semi-implicit: the tyre's Fx stiffens fast with slip at low speed (dFx/domega ~ mu*Fz*B*C*R/v),
        // which an explicit step at 400 Hz turns into a wheel-speed oscillation; divide it out
        const kw = (mu * PAC.Bx * PAC.Cx * R_W) / Math.max(speed, 0.5);
        wh.omega += (((tDrive - Fx * R_W) / I_W) * dt) / (1 + (dt * R_W * kw) / I_W);
        const bd = (tBrake / I_W) * dt;
        wh.omega = Math.abs(wh.omega) <= bd ? 0 : wh.omega - Math.sign(wh.omega) * bd;
      }
      // rolling resistance
      Fx -= Math.sign(vx) * Math.min(Math.abs(vx) * 50, CRR * Fz);
      const Fw = n.clone().multiplyScalar(Fz).addScaledVector(fw, Fx).addScaledVector(lat, Fy);
      F.add(Fw);
      T.add(new THREE.Vector3().crossVectors(r, Fw));
    });
    this.slip = slipMax;

    // integrate: linear, then angular with the body-frame inertia (gyroscopic term included)
    this.vel.addScaledVector(F, dt / MASS);
    const Rt = R.clone().transpose();
    const wb = this.w.clone().applyMatrix3(Rt), Tb = T.clone().applyMatrix3(Rt);
    const Iw = new THREE.Vector3(I_BODY[0] * wb.x, I_BODY[1] * wb.y, I_BODY[2] * wb.z);
    const gyro = new THREE.Vector3().crossVectors(wb, Iw);
    wb.x += ((Tb.x - gyro.x) / I_BODY[0]) * dt; wb.y += ((Tb.y - gyro.y) / I_BODY[1]) * dt; wb.z += ((Tb.z - gyro.z) / I_BODY[2]) * dt;
    this.w.copy(wb.applyMatrix3(R));
    if (this.w.length() > 8) this.w.setLength(8);

    const np = this.p.clone().addScaledVector(this.vel, dt);
    // buildings: test the base_link pose of the next step; a hit kills the motion into the wall
    const yaw = Math.atan2(ex.y, ex.x);
    const bx = np.x - LR * Math.cos(yaw), by = np.y - LR * Math.sin(yaw);
    if (this.world.collides && this.world.collides(bx, by, yaw, this)) {
      this.collided = performance.now();
      this.vel.multiplyScalar(-0.15); this.w.multiplyScalar(0.3);
      for (const wh of this.wheels) wh.omega *= -0.15;
    } else {
      this.odometer += np.distanceTo(this.p) * Math.cos(Math.asin(Math.min(1, Math.abs(this.vel.z) / (this.vel.length() || 1))));
      this.p.copy(np);
    }
    const dq = new THREE.Quaternion(this.w.x * dt * 0.5, this.w.y * dt * 0.5, this.w.z * dt * 0.5, 0).multiply(this.q);
    this.q.x += dq.x; this.q.y += dq.y; this.q.z += dq.z; this.q.w += dq.w; this.q.normalize();
    // fell through the world (tile not loaded yet): put it back on whatever is below
    const g0 = this._ground(this.p.x, this.p.y);
    if (g0 && this.p.z < g0.z + 0.1) { this.p.z = g0.z + CG_H; this.vel.z = Math.max(0, this.vel.z); }
  }

  _ackermann(y) {
    const d = this.delta;
    if (Math.abs(d) < 1e-4) return d;
    const Rr = L / Math.tan(Math.abs(d)), inner = (d > 0) === (y > 0);
    return Math.sign(d) * Math.atan(L / (Rr + (inner ? -1 : 1) * TRACK / 2));
  }

  // what the rest of the sim reads, at base_link
  _report() {
    const e = new THREE.Euler().setFromQuaternion(this.q, 'ZYX');
    this.yaw = e.z; this.pitch = -e.y; this.roll = e.x;              // pitch: nose up +, roll: left side up +
    const c = Math.cos(this.yaw), s = Math.sin(this.yaw);
    const R = new THREE.Matrix4().makeRotationFromQuaternion(this.q);
    const base = new THREE.Vector3(-LR, 0, -CG_H).applyMatrix4(R).add(this.p);
    this.x = base.x; this.y = base.y; this.z = base.z;
    const ex = new THREE.Vector3(c, s, 0), ey = new THREE.Vector3(-s, c, 0);
    const vPrev = this.v ?? 0;
    this.vx = this.vel.dot(ex);
    this.v = this.vx;
    this.vy = this.vel.dot(ey) - LR * this.w.z;
    this.yawRate = this.w.z;
    this.latAcc = this.v * this.yawRate;
    this.a = this._aFilt = (this._aFilt ?? 0) * 0.9 + 0.1 * ((this.v - vPrev) / (1 / 60));
  }

  get kmh() { return this.v * 3.6; }
  get steerNorm() { return this.delta / MAX_STEER; }
  static get MAX_STEER() { return MAX_STEER; }
}
