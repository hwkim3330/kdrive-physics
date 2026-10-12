import assert from 'node:assert/strict';
import * as THREE from 'three';
import { car, run, metrics } from './measure.mjs';

const m = metrics();
assert.ok(m.accel > 4 && m.accel < 6);
assert.ok(m.brakeG > 0.90 && m.brakeG < 1.0, '100–0 distance-derived braking g');
assert.ok(m.distance > 39 && m.distance < 44);
for (let i = 0; i < m.circles.length; i++) {
 const c=m.circles[i];
 assert.ok(c.speed > 19 && c.speed < 20.1);
 assert.ok(c.g > 0 && c.g < 1);
 if(i) assert.ok(c.g > m.circles[i-1].g, 'steady circle lateral acceleration increases with steer');
}
for(const s of m.slopes) assert.ok(s.drift < 0.001, 'park slope hold');

// World normal, including parent rotation and nonuniform scale.
{
 const v=car(20); const mesh=v.world.drivable[0];
 const parent=new THREE.Group();parent.rotation.y=0.3;parent.scale.set(1.2,0.8,1.1);parent.add(mesh);parent.updateMatrixWorld(true);
 const n=new THREE.Vector3(0,1,0).applyNormalMatrix(new THREE.Matrix3().getNormalMatrix(mesh.matrixWorld));
 const expected=new THREE.Vector3(n.x,-n.z,n.y);
 assert.ok(v._ground(0,0).n.distanceTo(expected)<1e-10);
}
// Rotated mesh and baked geometry produce the same contact response.
{
 const a=car(20), b=car(20,true);run(a,2);run(b,2);
 assert.ok(a.vel.distanceTo(b.vel)<1e-7);
 assert.ok(a.p.distanceTo(b.p)<1e-7);
}
for(const deg of [10,20]) for(const mode of ['manual','auto']) for(const gear of ['P','D']) {
 const v=car(deg);v.mode=mode;v.gear=gear;v.cmd.brake=1;v.cmd.accel=-1;
 run(v,5);const start=v.p.clone();run(v,10);
 assert.ok(v.p.distanceTo(start)<0.001, `${deg} degree ${mode}/${gear} hold`);
}
// No artificial anchoring of a freely rolling car.
{
 const v=car(20);run(v,5);v.gear='N';v.regenLevel=0;
 const start=v.p.clone();run(v,2);
 assert.ok(v.p.distanceTo(start)>0.1);
}
// Torque, per-axle and power caps must hold in both command paths.
for(const speed of [0,10,30,60]) for(const gear of ['D','R']) for(const scale of [0.5,1,2]) {
 const v=car();v.gear=gear;v.powerScale=scale;v.vx=gear==='R'?-speed:speed;
 v.mode='auto';v.aCmd=gear==='R'?-1e6:1e6;v.cmd.accel=v.aCmd;
 const auto=v._commands(0.0025).drive;
 v.mode='manual';v.cmd.throttle=1;const manual=v._commands(0.0025).drive;
 assert.ok(Math.abs(auto-manual)<1e-8);
 assert.ok(Math.abs(auto)*0.6<=3200+1e-8);
 assert.ok(Math.abs(auto)*speed/0.335<=239000+1e-8);
}
// Sample actual applied forces under combined slip, with and without aids.
for(const aids of [false,true]) {
 const v=car();run(v,2);v.gear='D';v.aids=aids;v.regenLevel=0;
 for(let i=0;i<1600;i++){
  const speed=3+(i%70);v.vel.set(speed,speed*Math.tan(0.14+0.3*Math.sin(i)),0);
  for(const w of v.wheels)w.omega=speed*(1+0.12*Math.cos(i))/0.335;
  v.cmd.throttle=i%2;v.cmd.brake=1-i%2;v.step(0.0025);
  for(const w of v.wheels)if(w.ground) assert.ok(Math.hypot(w.Fx,w.Fy)<=w.Fz+1e-7);
  assert.ok([...v.p.toArray(),...v.vel.toArray(),...v.q.toArray(),...v.w.toArray(),...v.wheels.map(w=>w.omega)].every(Number.isFinite));
 }
}
// ISO 7401-like step steer at nominal 20 m/s.
const step=car();run(step,2);step.gear='D';step.mode='auto';step.vel.x=20;
for(const w of step.wheels)w.omega=20/0.335;
const samples=[];run(step,8,t=>{step.cmd.accel=2*(20-step.v);step.cmd.delta=0.04;samples.push({t,r:step.yawRate});});
const steady=samples.slice(-400).reduce((s,x)=>s+x.r,0)/400;
const t10=samples.find(x=>x.r>=0.1*steady).t,t90=samples.find(x=>x.r>=0.9*steady).t;
const overshoot=Math.max(...samples.map(x=>x.r))/steady-1;
assert.ok(t90-t10>0.05&&t90-t10<1);
assert.ok(overshoot>=0&&overshoot<0.25);
// Lateral acceleration is a velocity derivative, including a lateral slide with zero yaw.
{
 const v=car();run(v,2);v.vel.y=1;v.w.set(0,0,0);const before=v.vel.clone();v.step(0.0025);
 const expected=v.vel.clone().sub(before).multiplyScalar(400).dot(new THREE.Vector3(0,1,0).applyQuaternion(v.q));
 assert.ok(Math.abs(v.latAcc-expected)<1e-8);
 assert.ok(Math.abs(v.latAcc-v.v*v.yawRate)>1);
}
// Coast: total mechanical energy (translation, rotation, wheels, gravity, springs).
function energy(v){
 const w=v.w.clone().applyQuaternion(v.q.clone().invert());
 return 1000*v.vel.lengthSq()+0.5*(650*w.x*w.x+3100*w.y*w.y+3400*w.z*w.z)+2000*9.81*v.p.z+
 v.wheels.reduce((s,w)=>s+0.65*w.omega*w.omega+21000*Math.max(0,w.comp)**2,0);
}
{
 const v=car();run(v,2);v.gear='N';v.regenLevel=0;v.vel.x=25;for(const w of v.wheels)w.omega=25/0.335;
 let previous=energy(v);run(v,10,()=>{const e=energy(v);assert.ok(e<=previous+0.01,'coast energy must not increase');previous=e;});
}
// A newly encountered 15 cm curb produces an impulse, not an instantaneous lift.
{
 const v=car();run(v,2);const ground=v._ground.bind(v);
 v._ground=(x,y)=>{const g=ground(x,y);if(x>v.p.x&&y>0)g.z+=0.15;return g;};
 const p=v.p.clone();v.step(0.0025);
 assert.ok(v.p.distanceTo(p)<0.003);
 assert.ok(v.vel.z>0&&v.w.length()>0,'asymmetric curb gives vertical and angular jolt');
}
console.log(JSON.stringify({...m,stepSteer:{riseTime:t90-t10,overshoot}},null,2));
console.log('All physics assertions passed.');
