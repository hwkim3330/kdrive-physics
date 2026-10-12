import assert from 'node:assert/strict';
import * as THREE from 'three';
import { Vehicle } from '../src/vehicle.js';
export function car(deg=0, baked=false) {
 const geometry=new THREE.PlaneGeometry(20000,20000).rotateX(-Math.PI/2);
 if(baked) geometry.rotateZ(deg*Math.PI/180);
 const plane=new THREE.Mesh(geometry,new THREE.MeshBasicMaterial({side:THREE.DoubleSide}));
 if(!baked) plane.rotation.z=deg*Math.PI/180;
 plane.updateMatrixWorld();
 const world={drivable:[plane],frame:{toThree:(x,y,z=0)=>new THREE.Vector3(x,z,-y)},groundZ:()=>0,collides:()=>false};
 const v=new Vehicle(world);
 const sub=v._sub.bind(v);
 v._sub=dt=>{
  sub(dt);
  assert.ok([...v.p.toArray(),...v.q.toArray(),...v.vel.toArray(),...v.w.toArray(),
   ...v.wheels.flatMap(w=>[w.omega,w.comp,w.Fz,w.kappa])].every(Number.isFinite), 'finite state before fallback');
 };
 v.place(0,0,0);
 if(deg) {v.q.setFromAxisAngle(new THREE.Vector3(0,1,0),-deg*Math.PI/180);v.p.set(0,0,0.55/Math.cos(deg*Math.PI/180));}
 return v;
}
export function run(v,s,f) {for(let i=0;i<Math.round(s*400);i++){f?.(i/400);v.step(1/400);}}
export function metrics() {
 const v=car();run(v,2);v.gear='D';v.cmd.throttle=1;
 let t=0;while(v.v<100/3.6&&t<30){v.step(1/400);t+=1/400;}
 const accel=t, v0=v.v, p=v.p.clone();v.cmd.throttle=0;v.cmd.brake=1;t=0;
 while(v.v>0.01&&t<15){v.step(1/400);t+=1/400;}
 const distance=v.p.clone().sub(p).length();
 const circles=[];
 for(const delta of [0.01,0.02,0.04,0.08]){
 const c=car();run(c,2);c.gear='D';c.mode='auto';c.vel.x=20;for(const w of c.wheels)w.omega=20/0.335;
 run(c,12,()=>{c.cmd.accel=2*(20-c.v);c.cmd.delta=delta;});
 circles.push({delta,g:Math.abs(c.latAcc)/9.81,speed:c.v,yaw:c.yawRate});
 }
 const slopes=[];
 for(const deg of [10,20]){const c=car(deg,true);run(c,5);const p=c.p.clone();run(c,10);slopes.push({deg,drift:c.p.distanceTo(p)});}
 return {accel,brakeG:v0*v0/(2*distance*9.81),distance,circles,slopes};
}
if(process.argv[1]?.endsWith('measure.mjs')) console.log(JSON.stringify(metrics(),null,2));
