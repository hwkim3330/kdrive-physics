// Physics on a flat plane, no browser: node test/physics.mjs
import * as THREE from 'three';
import { Vehicle } from '../src/vehicle.js';

const plane = new THREE.Mesh(new THREE.PlaneGeometry(20000, 20000).rotateX(-Math.PI / 2), new THREE.MeshBasicMaterial());
plane.updateMatrixWorld();
const world = { drivable: [plane], frame: { toThree: (x, y, z = 0, o = new THREE.Vector3()) => o.set(x, z, -y) }, groundZ: () => 0, collides: () => false };
const v = new Vehicle(world);
v.place(0, 0, 0);
const run = (s, f) => { for (let t = 0; t < s; t += 1 / 60) { f?.(t); v.step(1 / 60); } };
const show = (tag) => console.log(tag.padEnd(14), `v ${v.v.toFixed(2)} z ${v.z.toFixed(3)} pitch ${(v.pitch * 57.3).toFixed(2)} roll ${(v.roll * 57.3).toFixed(2)} r ${v.yawRate.toFixed(3)} lat ${v.latAcc.toFixed(2)} Fz ${v.wheels.map((w) => Math.round(w.Fz)).join('/')} wR ${v.wheels.map((w) => (w.omega * 0.335).toFixed(1)).join('/')}`);
run(1); show('settle');
v.gear = 'D'; v.cmd.throttle = 1;
let t = 0; while (v.v < 27.78 && t < 20) { v.step(1 / 60); t += 1 / 60; } console.log('0-100 km/h', t.toFixed(2), 's'); show('at 100');
v.cmd.throttle = 0.25; v.cmd.steer = 0.03; run(4); show('turn 100 (d=.03)');
v.cmd.steer = 0.06; run(4); show('turn 100 (d=.06)');
v.cmd.steer = 0; run(3); show('straight');
v.cmd.throttle = 0; v.cmd.brake = 1; t = 0; const v0 = v.v; while (v.v > 0.2 && t < 15) { v.step(1 / 60); t += 1 / 60; }
console.log('stop from', v0.toFixed(1), 'm/s in', t.toFixed(2), 's  ->', (v0 / t / 9.81).toFixed(2), 'g'); show('stopped');
// low-speed creep under a small acceleration request (autopilot pulling away)
v.place(0, 0, 0); run(1); v.mode = 'auto'; v.gear = 'D'; v.cmd.accel = 1.4; v.cmd.delta = 0;
for (const T of [1, 2, 4, 6]) { run(T === 1 ? 1 : T - [1, 2, 4, 6][[1, 2, 4, 6].indexOf(T) - 1]); console.log(`auto 1.4 m/s2 t=${T}s`.padEnd(18), 'v', v.v.toFixed(2), 'expect', (1.4 * T).toFixed(1), 'tcs', v.wheels.map((w) => w.tcs.toFixed(2)).join('/')); }
