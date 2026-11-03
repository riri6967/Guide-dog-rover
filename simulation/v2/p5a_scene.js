
/* ================= 3D world (three.js r128) ================= */
// sim (x, y) -> three (x, height, -y); yaw th -> rotation.y = th
let G = null;
const LAYER_OVERLAY=1, LAYER_FEEDONLY=2;
function canvasTex(w,h,draw,rx,ry){
  const c=document.createElement('canvas'); c.width=w; c.height=h; draw(c.getContext('2d'),w,h);
  const t=new THREE.CanvasTexture(c); t.wrapS=t.wrapT=THREE.RepeatWrapping; t.encoding=THREE.sRGBEncoding; t.anisotropy=4;
  if (rx) t.repeat.set(rx,ry); return t;
}
function rng(seed){ let s=Math.max(1,Math.floor(seed)%2147483646); return ()=>{ s=(s*16807)%2147483647; return (s-1)/2147483646; }; }
const TEX = {
  wood:(c,w,h)=>{ const r=rng(7); for (let i=0;i<8;i++){ const y=i*h/8, t=0.85+r()*0.2; c.fillStyle=`rgb(${Math.round(150*t)},${Math.round(104*t)},${Math.round(66*t)})`; c.fillRect(0,y,w,h/8);
      c.strokeStyle='rgba(60,35,18,.35)'; c.lineWidth=2; c.strokeRect(0,y,w,h/8); const sx=r()*w; c.beginPath(); c.moveTo(sx,y); c.lineTo(sx,y+h/8); c.stroke();
      c.strokeStyle='rgba(90,55,30,.18)'; c.lineWidth=1; for (let g=0;g<6;g++){ const gy=y+r()*h/8; c.beginPath(); c.moveTo(0,gy); c.bezierCurveTo(w*.3,gy+3,w*.6,gy-3,w,gy+1); c.stroke(); } } },
  wood2:(c,w,h)=>{ const r=rng(11); for (let i=0;i<8;i++){ const y=i*h/8, t=0.9+r()*0.15; c.fillStyle=`rgb(${Math.round(190*t)},${Math.round(160*t)},${Math.round(120*t)})`; c.fillRect(0,y,w,h/8);
      c.strokeStyle='rgba(80,60,40,.3)'; c.lineWidth=2; c.strokeRect(0,y,w,h/8); const sx=r()*w; c.beginPath(); c.moveTo(sx,y); c.lineTo(sx,y+h/8); c.stroke(); } },
  tile:(c,w,h)=>{ c.fillStyle='#CFD3D6'; c.fillRect(0,0,w,h); const r=rng(3); for (let i=0;i<4;i++) for (let j=0;j<4;j++){ const t=0.96+r()*0.06; c.fillStyle=`rgb(${Math.round(214*t)},${Math.round(218*t)},${Math.round(221*t)})`; c.fillRect(i*w/4+2,j*h/4+2,w/4-4,h/4-4); } },
  tile2:(c,w,h)=>{ for (let i=0;i<4;i++) for (let j=0;j<4;j++){ c.fillStyle=(i+j)%2?'#E8E4DC':'#B9B3A8'; c.fillRect(i*w/4,j*h/4,w/4,h/4); } },
  carpet:(c,w,h)=>{ c.fillStyle='#5D6B7A'; c.fillRect(0,0,w,h); const r=rng(5); for (let i=0;i<4000;i++){ c.fillStyle=`rgba(${r()>.5?255:0},${r()>.5?255:0},255,${r()*0.05})`; c.fillRect(r()*w,r()*h,2,2); } },
  vinyl:(c,w,h)=>{ c.fillStyle='#AEB7BF'; c.fillRect(0,0,w,h); const r=rng(19); for (let i=0;i<5000;i++){ c.fillStyle=`rgba(${r()>.5?255:40},${r()>.5?255:60},${r()>.5?255:80},${r()*0.12})`; c.fillRect(r()*w,r()*h,2,2); } c.strokeStyle='rgba(60,70,80,.15)'; c.strokeRect(0,0,w,h); },
  raised:(c,w,h)=>{ c.fillStyle='#C9CDD1'; c.fillRect(0,0,w,h); c.strokeStyle='#8E959C'; c.lineWidth=3; c.strokeRect(1,1,w-2,h-2); c.fillStyle='rgba(60,70,80,.35)'; for (let i=12;i<w-8;i+=14) for (let j=12;j<h-8;j+=14) { c.beginPath(); c.arc(i,j,2.2,0,TAU); c.fill(); } },
  plaster:(c,w,h)=>{ c.fillStyle='#E7E2D8'; c.fillRect(0,0,w,h); const r=rng(9); for (let i=0;i<2500;i++){ c.fillStyle=`rgba(120,110,95,${r()*0.05})`; c.fillRect(r()*w,r()*h,2,2); } },
  labwall:(c,w,h)=>{ c.fillStyle='#E3E7EA'; c.fillRect(0,0,w,h); const r=rng(4); for (let i=0;i<2000;i++){ c.fillStyle=`rgba(90,100,110,${r()*0.04})`; c.fillRect(r()*w,r()*h,2,2); } },
  rug:(c,w,h)=>{ c.fillStyle='#7A3E3A'; c.fillRect(0,0,w,h); c.strokeStyle='#D9B77A'; c.lineWidth=10; c.strokeRect(14,14,w-28,h-28); c.lineWidth=3; c.strokeRect(34,34,w-68,h-68);
      c.fillStyle='rgba(217,183,122,.55)'; for (let i=0;i<5;i++) for (let j=0;j<3;j++){ c.beginPath(); c.arc(70+i*(w-140)/4,70+j*(h-140)/2,10,0,TAU); c.fill(); } },
  rug2:(c,w,h)=>{ c.fillStyle='#2F5D62'; c.fillRect(0,0,w,h); c.strokeStyle='#E8DCC2'; c.lineWidth=8; for (let i=0;i<6;i++){ c.beginPath(); c.moveTo(0,i*h/6+h/12); c.lineTo(w,i*h/6+h/12); c.stroke(); } },
  art:(c,w,h,seed)=>{ const r=rng(seed||13); c.fillStyle='#F3EEE4'; c.fillRect(0,0,w,h); for (let i=0;i<6;i++){ c.fillStyle=['#D9643B','#2F5D62','#E9B949','#3D4F7A','#8A9A5B'][i%5]; c.globalAlpha=0.85; c.beginPath(); c.arc(r()*w,r()*h,20+r()*50,0,TAU); c.fill(); } c.globalAlpha=1; },
};
function mat(color, o){ return new THREE.MeshStandardMaterial(Object.assign({color, roughness:0.8, metalness:0.0}, o||{})); }
function boxMesh(sx,sy,sz,m,x,y,z){ const g=new THREE.Mesh(new THREE.BoxGeometry(sx,sy,sz),m); g.position.set(x,y,z); g.castShadow=true; g.receiveShadow=true; return g; }
function cyl(rt,rb,h,m,seg){ const g=new THREE.Mesh(new THREE.CylinderGeometry(rt,rb,h,seg||16),m); g.castShadow=true; g.receiveShadow=true; return g; }
function setLayer(o,l){ o.traverse(c=>c.layers.set(l)); return o; }
function stick(mesh, a, b){
  const d=new THREE.Vector3().subVectors(b,a), L=d.length();
  mesh.position.copy(a).addScaledVector(d,0.5); mesh.scale.set(1,Math.max(L,1e-3),1);
  mesh.quaternion.setFromUnitVectors(new THREE.Vector3(0,1,0), d.normalize());
}
function makeHuman(shirt, trousers, skin, hair, glasses){
  const g=new THREE.Group(), ms=mat(shirt), mt=mat(trousers), mk_=mat(skin,{roughness:0.6}), mh=mat(hair), mshoe=mat('#1B1D21');
  const legs=[];
  for (const z of [-0.1,0.1]) {
    const hip=new THREE.Group(); hip.position.set(0,0.92,z);
    const leg=cyl(0.07,0.06,0.86,mt); leg.position.y=-0.43; hip.add(leg);
    hip.add(boxMesh(0.24,0.07,0.1,mshoe,0.05,-0.88,0)); g.add(hip); legs.push(hip);
  }
  const torso=cyl(0.19,0.16,0.6,ms,14); torso.scale.z=1.25; torso.scale.x=0.75; torso.position.y=1.22; g.add(torso);
  const neck=cyl(0.05,0.05,0.08,mk_); neck.position.y=1.55; g.add(neck);
  const head=new THREE.Mesh(new THREE.SphereGeometry(0.11,20,16),mk_); head.position.y=1.65; head.castShadow=true; g.add(head);
  const hairM=new THREE.Mesh(new THREE.SphereGeometry(0.115,20,12,0,TAU,0,Math.PI/2),mh); hairM.position.y=1.67; hairM.rotation.z=0.25; g.add(hairM);
  if (glasses) g.add(boxMesh(0.03,0.04,0.17,mat('#0D0F12',{roughness:0.2,metalness:0.5}),0.105,1.67,0));
  const arms=[];
  for (const z of [-0.23,0.23]) {
    const sh=new THREE.Group(); sh.position.set(0,1.47,z);
    const a=cyl(0.045,0.04,0.62,ms); a.position.y=-0.31; sh.add(a);
    const hand=new THREE.Mesh(new THREE.SphereGeometry(0.045,10,8),mk_); hand.position.y=-0.64; sh.add(hand);
    g.add(sh); arms.push(sh);
  }
  return {g, legs, arms, shirtMat:ms};
}

/* ---------- the rover, built from the URDF meshes ---------- */
function urdfGeometry(m) {
  const bin=s=>{ const b=atob(s), u=new Uint8Array(b.length); for (let i=0;i<b.length;i++) u[i]=b.charCodeAt(i); return u.buffer; };
  const q=new Int16Array(bin(m.p)), pos=new Float32Array(q.length); for (let i=0;i<q.length;i++) pos[i]=q[i]*0.0001;
  const g=new THREE.BufferGeometry(); g.setAttribute('position',new THREE.BufferAttribute(pos,3));
  g.setIndex(new THREE.BufferAttribute(m.i32?new Uint32Array(bin(m.i)):new Uint16Array(bin(m.i)),1)); g.computeVertexNormals(); return g;
}
function urdfMesh(name, emissive) {
  const m=URDF_MESHES[name];
  const material=new THREE.MeshStandardMaterial({color:m.c, roughness:0.55, metalness:name.startsWith('handle_rod')?0.6:0.1, flatShading:true, side:THREE.DoubleSide,
    emissive: emissive||'#000000', emissiveIntensity: emissive?0:1});
  const mesh=new THREE.Mesh(urdfGeometry(m), material); mesh.castShadow=true; mesh.receiveShadow=true; return mesh;
}
function buildRover() {
  const rover=new THREE.Group(), base=new THREE.Group(); base.position.y=URDF_FRAMES.wheel_radius; rover.add(base);
  for (const n of ['chassis','encoder_holders','encoder_sensors','lidar','imu','camera','handle_mount','handle_rod','handle_frame','handle_grip']) base.add(urdfMesh(n));
  const haptic=['haptic_front','haptic_back','haptic_left','haptic_right'].map(n=>{ const m=urdfMesh(n,'#FF6B57'); base.add(m); return m; });
  const wheels=[];
  for (const [x,side,front] of [[URDF_FRAMES.half_base,1,true],[URDF_FRAMES.half_base,-1,true],[-URDF_FRAMES.half_base,1,false],[-URDF_FRAMES.half_base,-1,false]]) {
    const w=new THREE.Group(); w.position.set(x,0,-side*URDF_FRAMES.half_track);
    w.add(urdfMesh('tire')); w.add(urdfMesh('extender_'+(side>0?'L':'R'))); if (front) w.add(urdfMesh('disc_'+(side>0?'L':'R')));
    w.userData.side=side; base.add(w); wheels.push(w);
  }
  return {rover, wheels, haptic};
}

function init3D(){
  if (!window.THREE) { $('vpMsg').hidden=false; $('vpMsg').textContent='The 3D view could not start. The simulation, sensors and reasoning still run.'; return; }
  const cv=$('c3d');
  const renderer=new THREE.WebGLRenderer({canvas:cv, antialias:true});
  renderer.setPixelRatio(Math.min(window.devicePixelRatio||1,2));
  renderer.outputEncoding=THREE.sRGBEncoding; renderer.toneMapping=THREE.ACESFilmicToneMapping; renderer.toneMappingExposure=1.05;
  renderer.shadowMap.enabled=true; renderer.shadowMap.type=THREE.PCFSoftShadowMap;
  const scene=new THREE.Scene(); scene.background=new THREE.Color('#07090D'); scene.fog=new THREE.Fog('#07090D',26,48);
  const cam=new THREE.PerspectiveCamera(48,1,0.03,120); cam.layers.enable(LAYER_OVERLAY);
  scene.add(new THREE.HemisphereLight('#E4ECF5','#5A4E42',0.55));
  const sun=new THREE.DirectionalLight('#FFF4E2',0.95); sun.position.set(7,16,6); sun.castShadow=true;
  sun.shadow.mapSize.set(2048,2048); Object.assign(sun.shadow.camera,{left:-14,right:14,top:10,bottom:-10,near:1,far:40}); sun.shadow.bias=-0.0005;
  scene.add(sun); scene.add(new THREE.AmbientLight('#8090A0',0.18));
  mats();
  const R_=buildRover(); scene.add(R_.rover);
  const feedCam=new THREE.PerspectiveCamera(52,4/3,0.03,30); feedCam.position.set(0.15,CAM_H,0); feedCam.rotation.order='YXZ'; feedCam.rotation.y=-Math.PI/2;
  feedCam.layers.enable(LAYER_FEEDONLY); R_.rover.add(feedCam);
  const glow=new THREE.PointLight('#FF6B57',0,0.9); scene.add(glow);
  const person=makeHuman('#6C5CB8','#2A2F38','#B9825A','#2A1D15',true);
  const rArm=cyl(0.045,0.04,1,person.shirtMat); scene.add(rArm); person.arms[1].visible=false; scene.add(person.g);
  // overlays
  const lidarGeo=new THREE.BufferGeometry(); lidarGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(LIDAR_N*3),3)); lidarGeo.setAttribute('color',new THREE.BufferAttribute(new Float32Array(LIDAR_N*3),3));
  const lidarPts=new THREE.Points(lidarGeo,new THREE.PointsMaterial({size:0.05,vertexColors:true,sizeAttenuation:true})); lidarPts.frustumCulled=false; setLayer(lidarPts,LAYER_OVERLAY); scene.add(lidarPts);
  const rayGeo=new THREE.BufferGeometry(); rayGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(LIDAR_N*6),3));
  const rays=new THREE.LineSegments(rayGeo,new THREE.LineBasicMaterial({color:'#5CC6E6',transparent:true,opacity:0.1})); rays.frustumCulled=false; setLayer(rays,LAYER_OVERLAY); scene.add(rays);
  const pathMesh=new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({color:'#F2B33D',transparent:true,opacity:0.9,depthWrite:false})); pathMesh.frustumCulled=false; setLayer(pathMesh,LAYER_OVERLAY); scene.add(pathMesh);
  const lane=new THREE.Mesh(new THREE.BufferGeometry(), new THREE.MeshBasicMaterial({color:'#F2B33D',transparent:true,opacity:0.08,depthWrite:false})); lane.frustumCulled=false; setLayer(lane,LAYER_OVERLAY); scene.add(lane);
  const trajLine=new THREE.Line(new THREE.BufferGeometry(), new THREE.LineBasicMaterial({color:'#5CC6E6'})); trajLine.frustumCulled=false; setLayer(trajLine,LAYER_OVERLAY); scene.add(trajLine);
  const goal=new THREE.Group();
  const ring=new THREE.Mesh(new THREE.RingGeometry(0.24,0.32,40), new THREE.MeshBasicMaterial({color:'#F2B33D',side:THREE.DoubleSide,transparent:true,opacity:0.95})); ring.rotation.x=-Math.PI/2; ring.position.y=0.02; goal.add(ring);
  const beam=new THREE.Mesh(new THREE.CylinderGeometry(0.03,0.03,1.6,8,1,true), new THREE.MeshBasicMaterial({color:'#F2B33D',transparent:true,opacity:0.35})); beam.position.y=0.8; goal.add(beam);
  setLayer(goal,LAYER_OVERLAY); scene.add(goal);
  const costCv=document.createElement('canvas'); costCv.width=NX; costCv.height=NY;
  const costTex=new THREE.CanvasTexture(costCv); costTex.magFilter=THREE.NearestFilter;
  const costPlane=new THREE.Mesh(new THREE.PlaneGeometry(NX*RES,NY*RES), new THREE.MeshBasicMaterial({map:costTex,transparent:true,depthWrite:false}));
  costPlane.rotation.x=-Math.PI/2; costPlane.position.set(OX+NX*RES/2,0.012,-(OY+NY*RES/2)); setLayer(costPlane,LAYER_OVERLAY); scene.add(costPlane);
  const coneGeo=new THREE.BufferGeometry(); coneGeo.setAttribute('position',new THREE.BufferAttribute(new Float32Array(16*3),3));
  const cone=new THREE.LineSegments(coneGeo,new THREE.LineBasicMaterial({color:'#AE92FF',transparent:true,opacity:0.6})); cone.frustumCulled=false; setLayer(cone,LAYER_OVERLAY); scene.add(cone);
  const feedR=new THREE.WebGLRenderer({canvas:$('cFeed'), antialias:false});
  feedR.outputEncoding=THREE.sRGBEncoding; feedR.toneMapping=THREE.ACESFilmicToneMapping; feedR.setSize(480,360,false);
  G={renderer,scene,cam,feedR,feedCam,rover:R_.rover,wheels:R_.wheels,haptic:R_.haptic,glow,person,rArm,lidarPts,lidarGeo,rays,rayGeo,pathMesh,lane,trajLine,goal,costCv,costTex,costPlane,cone,coneGeo,
     world:null, walls:[], lintels:[], deco:[], walkerM:new Map(), objM:new Map(), view:'follow', orbit:{yaw:-0.9,pitch:0.75,r:9,tx:0,ty:0,tz:0}, camPos:new THREE.Vector3(6,6,6), camLook:new THREE.Vector3(), pathKey:''};
  hookViewport();
}

/* ---------- static world: floors, walls, furniture, doors, decor (one group per level) ---------- */
function buildScene(id) {
  if (!G) return;
  if (G.world) { G.scene.remove(G.world); G.world.traverse(o=>{ if (o.geometry) o.geometry.dispose(); }); }
  const W_=new THREE.Group(); G.world=W_; G.scene.add(W_);
  const ground=new THREE.Mesh(new THREE.PlaneGeometry(70,70), mat('#0E141B',{roughness:1})); ground.rotation.x=-Math.PI/2; ground.position.y=-0.02; W_.add(ground);
  const keep=LEVEL, n=WORLDS[id].levels ? WORLDS[id].levels.length : 1;
  G.levelGroups=[]; G.doorMeshes=[]; G.walls=[]; G.lintels=[]; G.deco=[];
  for (let L=0; L<n; L++) { buildWorld(id, L); const g=new THREE.Group(); W_.add(g); G.levelGroups.push(g); buildLevel(g, id, L); }
  buildWorld(id, keep); showLevel(keep);
  const [bx0,by0,bx1,by1]=WORLD.bounds;
  const sun=G.scene.children.find(o=>o.isDirectionalLight); if (sun) { Object.assign(sun.shadow.camera,{left:bx0-2,right:bx1+2,top:by1+2,bottom:by0-2}); sun.shadow.camera.updateProjectionMatrix(); }
  costDirty=true;
}
function showLevel(L) { if (G && G.levelGroups) G.levelGroups.forEach((g,i)=>{ g.visible = i===L; }); }
function buildLevel(W_, id, L) {
  const [bx0,by0,bx1,by1]=WORLD.bounds, bw=bx1-bx0, bh=by1-by0, cx=(bx0+bx1)/2, cy=(by0+by1)/2;
  // base floor, with a hole where stairs go down
  const holes=STATIC.filter(b=>b.cls==='stairs' && b.opt.dir==='down');
  const tileT=canvasTex(256,256,TEX.tile,1,1);
  let base;
  if (holes.length) {
    const sh=new THREE.Shape(); sh.moveTo(bx0-0.1,by0-0.1); sh.lineTo(bx1+0.1,by0-0.1); sh.lineTo(bx1+0.1,by1+0.1); sh.lineTo(bx0-0.1,by1+0.1); sh.closePath();
    for (const h of holes) { const p=new THREE.Path(); p.moveTo(h.x0,h.y0); p.lineTo(h.x0,h.y1); p.lineTo(h.x1,h.y1); p.lineTo(h.x1,h.y0); p.closePath(); sh.holes.push(p); }
    base=new THREE.Mesh(new THREE.ShapeGeometry(sh), mat('#ffffff',{map:tileT,roughness:0.55}));
    base.rotation.x=-Math.PI/2;
  } else {
    tileT.repeat.set(bw,bh);
    base=new THREE.Mesh(new THREE.PlaneGeometry(bw+0.2,bh+0.2), mat('#ffffff',{map:tileT,roughness:0.55})); base.rotation.x=-Math.PI/2; base.position.set(cx,0,-cy);
  }
  base.receiveShadow=true; W_.add(base);
  for (const a of AREAS) {
    if (a.floor==='tile') continue;
    for (const [x0,y0,x1,y1] of (a.floorRects || [[a.x0,a.y0,a.x1,a.y1]])) {
      const w=x1-x0, h=y1-y0, rep={carpet:3,raised:0.6,tile2:0.8,vinyl:2,hosp:2,hosp2:2,liftcar:1}[a.floor]||2.4;
      const f=new THREE.Mesh(new THREE.PlaneGeometry(w,h), mat('#ffffff',{map:canvasTex(512,512,TEX[a.floor], w/rep, h/rep),roughness:a.floor==='carpet'?1:0.6}));
      f.rotation.x=-Math.PI/2; f.position.set((x0+x1)/2,0.004+(a.floor==='liftcar'?0.002:0),-(y0+y1)/2); f.receiveShadow=true; W_.add(f);
    }
  }
  const wallM=mat('#ffffff',{map:canvasTex(256,256,id==='house'?TEX.plaster:TEX.labwall,4,4),roughness:0.92}), capM=mat('#3A4654',{roughness:0.7}), skirtM=mat(id==='house'?'#5E5144':id==='hospital'?'#5F8F86':'#4A5562',{roughness:0.6});
  for (const b of STATIC) {
    if (b.kind==='furn') continue;
    const w=new THREE.Mesh(new THREE.BoxGeometry(b.sx,1,b.sy), b.kind==='post'?mat('#6E5A48',{roughness:0.5}):wallM);
    w.position.set(b.cx,0.5,-b.cy); w.castShadow=true; w.receiveShadow=true; W_.add(w);
    const cap=new THREE.Mesh(new THREE.BoxGeometry(b.sx+0.005,0.02,b.sy+0.005), capM); cap.position.set(b.cx,b.h,-b.cy); W_.add(cap);
    G.walls.push({w,cap,h:b.h});
    if (b.kind==='wall') W_.add(boxMesh(b.sx+0.025,0.1,b.sy+0.025,skirtM,b.cx,0.05,-b.cy));
  }
  for (const d of DOORS) { const l=d.vert?boxMesh(0.2,0.35,d.w+0.2,wallM,d.x,2.425,-d.y):boxMesh(d.w+0.2,0.35,0.2,wallM,d.x,2.425,-d.y); W_.add(l); G.lintels.push(l); }
  for (const b of STATIC) if (b.kind==='furn' && BUILD[b.cls]) { const g=BUILD[b.cls](b, W_); g.traverse(o=>{ if (o.isMesh) { o.castShadow=true; o.receiveShadow=true; } }); W_.add(g); }
  G.doorMeshes[L] = DOOR_DYN.map(d=>buildDoorMesh(d, W_, id));
  G.deco.push(...decor(W_, id));
  const ceil=new THREE.Mesh(new THREE.PlaneGeometry(bw,bh), mat(id==='house'?'#EFECE6':'#F4F5F6',{roughness:1,side:THREE.DoubleSide})); ceil.rotation.x=Math.PI/2; ceil.position.set(cx,2.6,-cy); ceil.layers.set(LAYER_FEEDONLY); W_.add(ceil);
  for (const a of AREAS) for (let x=a.x0+1.5; x<a.x1-0.5; x+=3) for (let y=a.y0+1.2; y<a.y1-0.4; y+=2.6) {
    const l=new THREE.Mesh(id!=='house'?new THREE.PlaneGeometry(1.2,0.3):new THREE.CircleGeometry(0.22,24), new THREE.MeshBasicMaterial({color:'#FFF6E0'})); l.rotation.x=Math.PI/2; l.position.set(x,2.59,-y); l.layers.set(LAYER_FEEDONLY); W_.add(l); }
  for (const a of AREAS) {
    if (a.name==='Lift car') continue;
    const c=document.createElement('canvas'); c.width=512; c.height=96; const x=c.getContext('2d');
    x.font='700 50px Atkinson Hyperlegible, sans-serif'; x.fillStyle='rgba(232,238,244,.85)'; x.textAlign='center'; x.textBaseline='middle'; x.fillText(a.name.toUpperCase(),256,50);
    const sp=new THREE.Sprite(new THREE.SpriteMaterial({map:new THREE.CanvasTexture(c),transparent:true,depthTest:false}));
    sp.position.set((a.x0+a.x1)/2,2.0,-(a.y0+a.y1)/2); sp.scale.set(2.4,0.45,1); setLayer(sp,LAYER_OVERLAY); W_.add(sp);
  }
}
// doors: hinged leaves swing on a pivot, sliding and lift doors part in the middle
function buildDoorMesh(d, parent, id) {
  const g=new THREE.Group(); parent.add(g);
  const horiz=!d.vert, P=(along,h,across)=>horiz ? [d.x+along,h,-(d.y+across)] : [d.x+across,h,-(d.y+along)];
  const frameM=mat(d.type==='lift'?'#AEB6BF':'#8C9199',{roughness:0.35,metalness:0.5});
  const jamb=o=>{ const [x,y,z]=P(o,1.075,0); g.add(boxMesh(horiz?0.06:0.22,2.15,horiz?0.22:0.06,frameM,x,y,z)); };
  if (d.group===d.id) { jamb(-d.w/2-0.03); jamb(d.w/2+0.03); } else jamb(d.hinge*(d.w/2+0.03));
  if (d.type==='hinged') {
    const pivot=new THREE.Group(), [hx,,hz]=P(d.hinge*d.w/2,0,0); pivot.position.set(hx,0,hz); g.add(pivot);
    const leafM=mat(id==='hospital'?'#D9C8A9':'#B9A27E',{roughness:0.55}), dir=-d.hinge, L=d.w-0.03;
    const leaf=new THREE.Group(); pivot.add(leaf);
    if (horiz) leaf.position.set(dir*L/2,0,0); else leaf.position.set(0,0,-dir*L/2);
    const bx=(sx,sy,sz,m,x,y,z)=>leaf.add(horiz?boxMesh(sx,sy,sz,m,x,y,z):boxMesh(sz,sy,sx,m,z,y,-x));
    bx(L,2.05,0.045,leafM,0,1.04,0);
    bx(L*0.9,0.25,0.05,M.steel,0,0.15,0);                                      // kick plate
    if (id==='hospital') bx(0.14,0.6,0.05,M.glassBlue,dir*-L*0.25,1.45,0);       // vision panel
    for (const s of [-1,1]) bx(0.14,0.025,0.03,M.steel,dir*(L/2-0.12),1.0,s*0.045); // lever handles both faces
    return {update(f){ pivot.rotation.y = horiz ? f*Math.PI/2*dir*d.swing : -f*Math.PI/2*dir*d.swing; }};
  }
  // sliding (automatic or lift): two leaves part from the middle
  const steel=d.type==='lift', leafM=steel?mat('#B5BCC4',{roughness:0.25,metalness:0.85}):M.glassBlue, half=d.w/2;
  const mkLeaf=s=>{ const l=new THREE.Group(); const [x,y,z]=P(s*half/2,0,0); l.position.set(x,y,z);
    l.add(horiz?boxMesh(half,2.1,steel?0.04:0.025,leafM,0,1.05,0):boxMesh(steel?0.04:0.025,2.1,half,leafM,0,1.05,0));
    if (!steel) l.add(horiz?boxMesh(half,0.08,0.04,frameM,0,0.04,0):boxMesh(0.04,0.08,half,frameM,0,0.04,0));
    g.add(l); return l; };
  const a=mkLeaf(-1), b=mkLeaf(1), a0=a.position.clone(), b0=b.position.clone();
  { const [x,y,z]=P(0,2.2,0); g.add(horiz?boxMesh(d.w+0.2,0.16,0.18,frameM,x,y,z):boxMesh(0.18,0.16,d.w+0.2,frameM,x,y,z));
    const s=new THREE.Mesh(new THREE.SphereGeometry(0.03,10,8),new THREE.MeshBasicMaterial({color:steel?'#FFB13C':'#3CFF8A'})); s.position.set(x,y-0.09,z); g.add(s); }
  return {update(f){ const o=f*half*0.95; if (horiz) { a.position.x=a0.x-o; b.position.x=b0.x+o; } else { a.position.z=a0.z+o; b.position.z=b0.z-o; } }};
}
