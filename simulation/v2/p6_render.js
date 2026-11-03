
/* ================= 3D per-frame sync ================= */
const layers={lidar:true, path:true, cost:false, cam:true, cut:true};
let tool='goal';
const V3=(x,h,y)=>new THREE.Vector3(x,h,-y);
function ribbon(mesh, pts, half, y) {
  const n=pts.length; if (n<2) { mesh.visible=false; return; }
  const pos=new Float32Array(n*2*3), idx=[];
  for (let i=0;i<n;i++) {
    const a=pts[Math.max(0,i-1)], b=pts[Math.min(n-1,i+1)], dx=b[0]-a[0], dy=b[1]-a[1], L=hyp(dx,dy)||1, nx=-dy/L*half, ny=dx/L*half;
    pos.set([pts[i][0]+nx,y,-(pts[i][1]+ny), pts[i][0]-nx,y,-(pts[i][1]-ny)], i*6);
    if (i<n-1) idx.push(2*i,2*i+1,2*i+2, 2*i+1,2*i+3,2*i+2);
  }
  const g=new THREE.BufferGeometry(); g.setAttribute('position',new THREE.BufferAttribute(pos,3)); g.setIndex(idx);
  mesh.geometry.dispose(); mesh.geometry=g; mesh.visible=true;
}
function buildCost() {
  const x=G.costCv.getContext('2d'), img=x.createImageData(NX,NY), d=img.data;
  for (let j=0;j<NY;j++) for (let i=0;i<NX;i++) {
    const k=j*NX+i, p=((NY-1-j)*NX+i)*4, c=dist[k]; let a=0, r=255, g=107, b=87;
    if (marks[k]===2) { a=230; r=174; g=146; b=255; }
    else if (marks[k]) a=230;
    else if (!occ[k]) { if (c<R_LETHAL) a=80; else if (c<1.3) a=Math.round(40*(1-(c-R_LETHAL)/0.8)); }
    d[p]=r; d[p+1]=g; d[p+2]=b; d[p+3]=a;
  }
  x.putImageData(img,0,0); G.costTex.needsUpdate=true; costDirty=false;
}
const KCOL=[[0,0,0],[0.36,0.78,0.9],[0.95,0.7,0.24],[1,0.42,0.34],[0.68,0.57,1],[0.68,0.57,1],[0.91,0.93,0.96]];
function setWalls(cut) {
  const ch=1.05;
  for (const w of G.walls) { const h=cut?Math.min(ch,w.h):w.h; w.w.scale.y=h; w.w.position.y=h/2; w.cap.position.y=h+0.01; }
  for (const l of G.lintels) l.visible=!cut;
  for (const d of G.deco) d.visible=!cut;
}
function animHuman(H, walk, moving, holdArm) {
  const ph=walk*TAU/1.25, sw=moving?Math.sin(ph)*0.45:0;
  H.legs[0].rotation.z=sw; H.legs[1].rotation.z=-sw;
  H.arms[0].rotation.z=-sw*0.6; if (!holdArm) H.arms[1].rotation.z=sw*0.6;
}
function sync3D(now, dtReal) {
  if (!G) return;
  const c=Math.cos(S.th), s=Math.sin(S.th);
  G.rover.position.set(S.x,0,-S.y); G.rover.rotation.y=S.th;
  if (G.doorMeshes && G.doorMeshes[LEVEL]) S.doors.forEach((d,i)=>{ const m=G.doorMeshes[LEVEL][i]; if (m) m.update(d.f); });
  for (const w of G.wheels) w.rotation.z = -((w.userData.side>0 ? S.wheelL : S.wheelR)||0);
  // the four vibration modules in the URDF grip glow with their duty
  let gl=0;
  hap.out.forEach((d,i)=>{ const f=d/255, m=G.haptic[i].material; m.emissiveIntensity=f*2.2; gl=Math.max(gl,f); });
  const gx=S.x+GRIP_X*c, gy=S.y+GRIP_X*s, gm=V3(gx,GRIP_H,gy);
  G.glow.position.copy(gm); G.glow.intensity=gl*1.4;
  // the person faces the grip; right hand on it
  G.person.g.position.set(S.px,0,-S.py); G.person.g.rotation.y=S.pth;
  animHuman(G.person, S.pWalk, (S.pSpeed||0)>0.03, true);
  const pr=S.pth-Math.PI/2, sh=V3(S.px+Math.cos(pr)*0.21, 1.45, S.py+Math.sin(pr)*0.21);
  stick(G.rArm, sh, gm);
  // walkers
  const seen=new Set();
  for (const w of S.walkers) {
    let H=G.walkerM.get(w.id);
    if (!H) { const looks=[['#D9643B','#3A3F47'],['#2F8F8A','#2C2C34'],['#C9A227','#433A33'],['#8A9AA8','#25303B'],['#B04A6A','#2A2F38'],['#7A8450','#2A2F38'],['#3E8C95','#3E8C95'],['#F2F4F6','#3A4654']][w.look%8];
      H=makeHuman(looks[0],looks[1],['#8D5A3B','#E0B892','#C68A5E'][w.id%3],'#1E1A17',false); G.scene.add(H.g); G.walkerM.set(w.id,H); }
    H.g.position.set(w.x,0,-w.y); H.g.rotation.y=w.face; animHuman(H,w.walk,hyp(w.vx,w.vy)>0.05,false); seen.add(w.id);
  }
  for (const [id,H] of G.walkerM) if (!seen.has(id)) { G.scene.remove(H.g); G.walkerM.delete(id); }
  // dropped objects
  const seenO=new Set();
  for (const o of S.objs) {
    let m=G.objM.get(o.id);
    if (!m) {
      m=new THREE.Group();
      if (o.cls==='backpack') { m.add(boxMesh(o.sx,o.h*0.85,o.sy,mat('#2E5E8C',{roughness:0.9}),0,o.h*0.425,0)); m.add(boxMesh(o.sx*0.7,o.h*0.4,0.06,mat('#244A6E'),0,o.h*0.3,-o.sy/2-0.02)); const hd=new THREE.Mesh(new THREE.TorusGeometry(0.07,0.015,8,16,Math.PI),mat('#1B2A3A')); hd.position.y=o.h*0.85; m.add(hd); }
      else { m.add(boxMesh(o.sx,o.h,o.sy,mat('#B98A55',{roughness:0.95}),0,o.h/2,0)); m.add(boxMesh(o.sx+0.005,0.05,0.06,mat('#D9C9A8'),0,o.h-0.02,0)); }
      m.position.set(o.cx,0,-o.cy); m.traverse(q=>{ if (q.isMesh) { q.castShadow=true; q.receiveShadow=true; } }); G.scene.add(m); G.objM.set(o.id,m);
    }
    seenO.add(o.id);
  }
  for (const [id,m] of G.objM) if (!seenO.has(id)) { G.scene.remove(m); G.objM.delete(id); }
  // LiDAR
  G.lidarPts.visible=G.rays.visible=layers.lidar;
  if (layers.lidar) {
    const P=G.lidarGeo.attributes.position.array, C=G.lidarGeo.attributes.color.array, R=G.rayGeo.attributes.position.array;
    const lx0=S.x+LIDAR_OFF*c, ly0=S.y+LIDAR_OFF*s;
    for (let n=0;n<LIDAR_N;n++) {
      const a=S.th-Math.PI+n*TAU/LIDAR_N, r=S.scan[n], k=S.scanK[n], hit=r<LIDAR_MAX, rr=hit?r:0;
      const hx=lx0+Math.cos(a)*rr, hy=ly0+Math.sin(a)*rr;
      P[n*3]=hx; P[n*3+1]=hit?LIDAR_H:-5; P[n*3+2]=-hy;
      const col=KCOL[k]||KCOL[1]; C[n*3]=col[0]; C[n*3+1]=col[1]; C[n*3+2]=col[2];
      const rl=Math.min(r,6);
      R.set([lx0,LIDAR_H,-ly0, lx0+Math.cos(a)*rl,LIDAR_H,-(ly0+Math.sin(a)*rl)], n*6);
    }
    G.lidarGeo.attributes.position.needsUpdate=true; G.lidarGeo.attributes.color.needsUpdate=true; G.rayGeo.attributes.position.needsUpdate=true;
  }
  // path, lane and local plan
  const pk=S.path?`${S.path.length}:${S.pathIdx>>2}:${S.path[S.path.length-1]}`:'none';
  if (pk!==G.pathKey) { G.pathKey=pk;
    if (S.path) { const pts=S.path.slice(S.pathIdx); pts.unshift([S.x,S.y]); ribbon(G.pathMesh,pts,0.035,0.02); ribbon(G.lane,pts,0.3,0.015); }
    else { G.pathMesh.visible=false; G.lane.visible=false; } }
  if (!layers.path) { G.pathMesh.visible=false; G.lane.visible=false; } else if (S.path) { G.pathMesh.visible=true; G.lane.visible=true; }
  if (S.traj && layers.path) { const arr=new Float32Array((S.traj.length+1)*3); arr.set([S.x,0.03,-S.y],0); S.traj.forEach((p,i)=>arr.set([p[0],0.03,-p[1]],(i+1)*3));
    G.trajLine.geometry.dispose(); G.trajLine.geometry=new THREE.BufferGeometry(); G.trajLine.geometry.setAttribute('position',new THREE.BufferAttribute(arr,3)); G.trajLine.visible=true; }
  else G.trajLine.visible=false;
  G.goal.visible=!!S.goal; if (S.goal) { G.goal.position.set(S.goal.x,0,-S.goal.y); G.goal.children[0].scale.setScalar(1+0.08*Math.sin(now/300)); }
  G.costPlane.visible=layers.cost; if (layers.cost && costDirty) buildCost();
  // camera cone
  G.cone.visible=layers.cam;
  if (layers.cam) {
    const cx=S.x+CAM_OFF*c, cy=S.y+CAM_OFF*s, A=G.coneGeo.attributes.position.array, L=3.2, hf=CAM_HFOV/2;
    const cs=[[-hf,0.25],[hf,0.25],[hf,-0.55],[-hf,-0.55]].map(([da,dz])=>[cx+Math.cos(S.th+da)*L, CAM_H+dz*L*0.75, cy+Math.sin(S.th+da)*L]);
    let k=0; const put=(x,h,y)=>{ A[k++]=x; A[k++]=h; A[k++]=-y; };
    for (let i=0;i<4;i++) { put(cx,CAM_H,cy); put(cs[i][0],cs[i][1],cs[i][2]); }
    for (let i=0;i<4;i++) { const a=cs[i], b=cs[(i+1)%4]; put(a[0],a[1],a[2]); put(b[0],b[1],b[2]); }
    G.coneGeo.attributes.position.needsUpdate=true;
  }
}

/* ================= Cameras ================= */
function placeCamera() {
  const cam=G.cam, o=G.orbit;
  let pos, look;
  if (G.view==='follow') {
    const bx=S.px-S.x, by=S.py-S.y, L=hyp(bx,by)||1;   // camera behind the person, looking past them to the rover
    pos=V3(S.px+bx/L*2.2, 2.3, S.py+by/L*2.2); look=V3(S.x+Math.cos(S.th)*1.1, 0.3, S.y+Math.sin(S.th)*1.1);
  } else if (G.view==='top') { const [x0,y0,x1,y1]=WORLD.bounds, cx=(x0+x1)/2, cy=(y0+y1)/2, k=(x1-x0)/24; pos=new THREE.Vector3(cx,23*k,-cy+8.5*k); look=new THREE.Vector3(cx,0,-cy+0.6); }
  else if (G.view==='rover') {   // close-up of the URDF rover from its front-left quarter
    const a=S.th+0.75; pos=V3(S.x+Math.cos(a)*1.05, 0.62, S.y+Math.sin(a)*1.05); look=V3(S.x-0.18*Math.cos(S.th), 0.28, S.y-0.18*Math.sin(S.th));
  } else if (G.view==='pov') {
    pos=V3(S.px+Math.cos(S.pth)*0.12, 1.62, S.py+Math.sin(S.pth)*0.12); look=V3(S.x+Math.cos(S.pth)*2.5, 0.6, S.y+Math.sin(S.pth)*2.5);
  } else {
    look=new THREE.Vector3(o.tx,o.ty,o.tz);
    pos=new THREE.Vector3(o.tx+o.r*Math.cos(o.pitch)*Math.cos(o.yaw), o.ty+o.r*Math.sin(o.pitch), o.tz+o.r*Math.cos(o.pitch)*Math.sin(o.yaw));
  }
  const k = G.view==='orbit' ? 1 : 0.09;
  G.camPos.lerp(pos,k); G.camLook.lerp(look,k);
  cam.position.copy(G.camPos); cam.lookAt(G.camLook);
}
function hookViewport() {
  const cv=$('c3d'); let drag=null;
  cv.addEventListener('contextmenu', e=>e.preventDefault());
  cv.addEventListener('pointerdown', e=>{ drag={x:e.clientX,y:e.clientY,btn:e.button,moved:false,shift:e.shiftKey}; cv.setPointerCapture(e.pointerId); });
  cv.addEventListener('pointermove', e=>{
    if (!drag) return; const dx=e.clientX-drag.x, dy=e.clientY-drag.y;
    if (!drag.moved && hyp(dx,dy)<5) return;
    if (!drag.moved) { drag.moved=true; if (G.view!=='orbit') toOrbit(); }
    const o=G.orbit;
    if (drag.btn===2||drag.shift) { const f=o.r*0.0016, sy=Math.sin(o.yaw), cy=Math.cos(o.yaw); o.tx+=(dx*sy - dy*cy)*f*-1; o.tz+=(-dx*cy - dy*sy)*f*-1; }
    else { o.yaw+=dx*0.006; o.pitch=clamp(o.pitch+dy*0.005,0.12,1.5); }
    drag.x=e.clientX; drag.y=e.clientY;
  });
  cv.addEventListener('pointerup', e=>{ if (drag && !drag.moved && drag.btn===0) floorClick(e); drag=null; });
  cv.addEventListener('wheel', e=>{ e.preventDefault(); if (G.view!=='orbit') toOrbit(); G.orbit.r=clamp(G.orbit.r*Math.pow(1.0015,e.deltaY),2.5,40); }, {passive:false});
}
function toOrbit() {
  const o=G.orbit, d=new THREE.Vector3().subVectors(G.cam.position,G.camLook);
  o.tx=G.camLook.x; o.ty=G.camLook.y; o.tz=G.camLook.z; o.r=d.length(); o.pitch=Math.asin(clamp(d.y/o.r,-1,1)); o.yaw=Math.atan2(d.z,d.x);
  setView('orbit');
}
function setView(v) { G.view=v; document.querySelectorAll('[data-view]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.view===v))); }
function floorClick(e) {
  if (!SC.free) { flash('Switch to the Free roam scenario to send the rover somewhere or place things.'); return; }
  const r=$('c3d').getBoundingClientRect(), ndc=new THREE.Vector2((e.clientX-r.left)/r.width*2-1, -((e.clientY-r.top)/r.height*2-1));
  const ray=new THREE.Raycaster(); ray.setFromCamera(ndc,G.cam);
  const t=-ray.ray.origin.y/ray.ray.direction.y; if (!(t>0)) return;
  const p=ray.ray.origin.clone().addScaledVector(ray.ray.direction,t), x=p.x, y=-p.z;
  freeAction(x,y);
}
let flashT=0;
function flash(msg){ tl('event',msg); }

/* ================= Camera feed overlay ================= */
function drawFeedOverlay() {
  const cv=$('cFeedOv'), x=cv.getContext('2d'), W=cv.width, H=cv.height;
  x.clearRect(0,0,W,H);
  const cam=G.feedCam; cam.updateMatrixWorld();
  const proj=(px,h,py)=>{ const v=V3(px,h,py).project(cam); return {x:(v.x+1)/2*W, y:(1-v.y)/2*H, ok:v.z<1 && v.z>-1}; };
  // drivable lane ahead
  if (S.path && S.path.length>S.pathIdx+2) {
    const pts=S.path.slice(S.pathIdx, S.pathIdx+45), L=[], R=[];
    for (let i=0;i<pts.length;i++) {
      const a=pts[Math.max(0,i-1)], b=pts[Math.min(pts.length-1,i+1)], dx=b[0]-a[0], dy=b[1]-a[1], l=hyp(dx,dy)||1;
      const rel=(pts[i][0]-S.x)*Math.cos(S.th)+(pts[i][1]-S.y)*Math.sin(S.th); if (rel<CAM_OFF+0.25) continue;
      const pl=proj(pts[i][0]-dy/l*0.38,0,pts[i][1]+dx/l*0.38), pr=proj(pts[i][0]+dy/l*0.38,0,pts[i][1]-dx/l*0.38);
      if (pl.ok&&pr.ok) { L.push(pl); R.push(pr); }
    }
    if (L.length>1) { x.beginPath(); L.forEach((p,i)=>i?x.lineTo(p.x,p.y):x.moveTo(p.x,p.y)); for (let i=R.length-1;i>=0;i--) x.lineTo(R[i].x,R[i].y); x.closePath();
      x.fillStyle='rgba(76,195,138,.22)'; x.fill(); x.strokeStyle='rgba(76,195,138,.8)'; x.lineWidth=1.5; x.stroke(); }
  }
  // detections
  x.font='600 11px IBM Plex Mono, monospace'; x.textBaseline='top';
  for (const d of S.dets) {
    let x0=1e9,y0=1e9,x1=-1e9,y1=-1e9,any=false;
    const r=d.rad, hgt=d.cls==='doorway'?2.1:d.h;
    for (const [ox,oy] of [[-r,-r],[r,-r],[-r,r],[r,r]]) for (const hh of [0,hgt]) { const p=proj(d.cx+ox,hh,d.cy+oy); if (!p.ok) continue; any=true; x0=Math.min(x0,p.x); y0=Math.min(y0,p.y); x1=Math.max(x1,p.x); y1=Math.max(y1,p.y); }
    if (!any) continue;
    x0=clamp(x0,1,W-1); x1=clamp(x1,1,W-1); y0=clamp(y0,1,H-1); y1=clamp(y1,1,H-1); if (x1-x0<4||y1-y0<4) continue;
    const col=d.cls==='walker'?'#FF6B57':d.cls==='doorway'?'#5CC6E6':(d.ref&&d.ref.isObj)?'#AE92FF':'#F2B33D', k=Math.min(14,(x1-x0)/3,(y1-y0)/3);
    x.strokeStyle=col; x.lineWidth=2; x.beginPath();
    x.moveTo(x0,y0+k); x.lineTo(x0,y0); x.lineTo(x0+k,y0); x.moveTo(x1-k,y0); x.lineTo(x1,y0); x.lineTo(x1,y0+k);
    x.moveTo(x1,y1-k); x.lineTo(x1,y1); x.lineTo(x1-k,y1); x.moveTo(x0+k,y1); x.lineTo(x0,y1); x.lineTo(x0,y1-k); x.stroke();
    const txt=`${d.name} ${Math.round(d.conf*100)}% · ${d.d.toFixed(1)}m`, tw=x.measureText(txt).width+8, ty=y0>16?y0-16:y0+2;
    x.fillStyle=col; x.fillRect(x0,ty,tw,15); x.fillStyle='#0A0F15'; x.fillText(txt,x0+4,ty+2);
  }
  // HUD
  x.fillStyle='rgba(10,15,21,.65)'; x.fillRect(0,0,W,20); x.fillRect(0,H-20,W,20);
  x.fillStyle='#E8EEF4'; x.font='500 11px IBM Plex Mono, monospace'; x.textBaseline='middle';
  x.fillText('PI CAM v2 · 640×480 · 62° HFOV', 8, 10);
  x.textAlign='right'; x.fillText(`t ${S.t.toFixed(1)} s · ${feedFps} fps`, W-8, 10);
  x.fillText(`yaw ${Math.round(S.th*57.3)}°`, W-8, H-10);
  x.textAlign='left'; x.fillText(`${S.dets.length} detections · green = drivable lane`, 8, H-10);
  x.strokeStyle='rgba(232,238,244,.45)'; x.lineWidth=1; x.beginPath(); x.moveTo(W/2-8,H/2); x.lineTo(W/2+8,H/2); x.moveTo(W/2,H/2-8); x.lineTo(W/2,H/2+8); x.stroke();
  if (S.linkLost) { x.fillStyle='rgba(255,94,94,.85)'; x.fillRect(W/2-90,30,180,22); x.fillStyle='#fff'; x.textAlign='center'; x.fillText('HANDLE LINK LOST', W/2, 41); x.textAlign='left'; }
}
let feedFps=0;
