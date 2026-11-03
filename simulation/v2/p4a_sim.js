
/* ================= Simulation state ================= */
let S, SC, SV, cueMode='v2', nextId=1;
function fresh(sc, v) {
  const p=PROF[sc.profile], st=v.start;
  const [hx,hy]=[st.x+GRIP_X*Math.cos(st.th), st.y+GRIP_X*Math.sin(st.th)];
  return {
    profile:sc.profile, t:0, x:st.x, y:st.y, th:st.th, cmdV:0, cmdW:0,
    px:hx-ARM*Math.cos(st.th), py:hy-ARM*Math.sin(st.th), pth:st.th, pWalk:0, pReactT:0, pReacting:false, pBlocked:false,
    ticksL:0, ticksR:0, accL:0, accR:0,
    objs:[], walkers:[], nearRects:[],
    scan:new Float32Array(LIDAR_N).fill(LIDAR_MAX), scanK:new Uint8Array(LIDAR_N),
    gap:Infinity, gapK:0, lidarGap:Infinity, nearest:Infinity, nearestK:0, nearestA:0, sect:{F:9,L:9,R:9,B:9}, userPts:0,
    dets:[], camNew:null, camNewT:-9, lastScanT:-1, lastCamT:-1, ctrlAcc:0,
    mode:'ready', modeT:0, goal:null, goalIdx:-1, goalsDone:[], path:null, pathIdx:0, navStart:0, lastReplan:-9, lastPathCheck:0, replanWhy:'', replanT:-9,
    blockedT:0, blockReplanned:false, backups:0, backupDist:0, backupT:0, progT:0, progX:0, progY:0, progTh:0,
    inPlace:false, spinDir:0, announced:false, announceUntil:-1, rel:0, traj:null, dwaBlocked:false,
    stopReason:null, waitWhy:'', obstUrg:0, cap:VMAX, capWhy:'', bumped:false,
    hold:false, linkLost:false, introStep:-1,
    lastTurnCue:{L:-9,R:-9}, lastDirCue:-9, lastSlowCue:-9, lastDoorCue:-9, lastHeart:-9, lastObstT:-9, sideCueT:{}, waitT:0,
    lastSpeechT:-9, lastSpokenTurn:-9, lastSpokenWait:-9, speechLog:[], heardSpeech:false, lastArea:null, pendingVoice:null,
    turnInfo:null,
    psy:{trust:sc.trust0??p.trust0, stress:sc.stress0??p.stress0, frozen:false, dragT:0, slowT:0, flowT:0, lastV:0, turning:false, waitVoiced:false, cd:{}},
    voiceCount:{}, voice:'', voiceT:-9, cueLog:[], firedEv:new Set(),
    stats:{turnsCued:0, surprises:0, cues:0, spoken:0, closeCalls:0, dragT:0, peak:sc.stress0??p.stress0, trust0:sc.trust0??p.trust0, stressSum:0, n:0, arrived:0, failed:0},
    hist:[], tl:[], think:{}, thinkKey:'',
    floorT:0, lift:null, liftGoal:null, doors:[], doorRects:[], doorAheadInfo:null, stairsNear:null,
  };
}

/* ================= Timeline & voice ================= */
function tl(tag, text) {
  const last=S.tl[0];
  if (last && last.text===text) return;
  S.tl.unshift({t:S.t, tag, text}); if (S.tl.length>140) S.tl.pop();
  S.tlDirty=true;
}
function say(key, force) {
  const line=voiceLine(key, S.profile); if (!line) return;
  if (!force && S.t-S.voiceT < 3.2) return;
  S.voice=line; S.voiceT=S.t; tl('user', line);
  if (narrateInner) utter(line,'inner',0);
}
function cue(key) {
  if (!SHOT[key]) return;
  hap.play(key); S.cueLog.push(S.t); S.stats.cues++;
  if (key.startsWith('preTurn')) { S.lastDirCue=S.t; tl('tell', `${SHOT[key].name}: ${SHOT[key].say}`); }
  else if (key==='door'||key.startsWith('arrived')) tl('tell', `${SHOT[key].name}: ${SHOT[key].say}`);
  if (key==='slow') S.lastSlowCue=S.t;
  if (key==='heart') S.lastHeart=S.t;
}

/* ================= Sensing ================= */
function markObj(o, v) {
  for (let j=cy2j(o.y0-0.05); j<=cy2j(o.y1+0.05); j++) for (let i=cx2i(o.x0-0.05); i<=cx2i(o.x1+0.05); i++)
    if (i>=0&&j>=0&&i<NX&&j<NY) marks[j*NX+i]=v;
  costDirty=true;
}
// RPLIDAR A1 at 0.23 m: walls, cabinets and the bases of solid furniture, but only the legs of tables, desks and chairs
function sense() {
  const c=Math.cos(S.th), s=Math.sin(S.th), lx=S.x+LIDAR_OFF*c, ly=S.y+LIDAR_OFF*s;
  S.nearRects = STATIC.filter(b => rectDist(S.x,S.y,b) < 3.2);
  const scanSet = SCAN_ALL.concat(S.doorRects).filter(b => rectDist(lx,ly,b) < LIDAR_MAX);
  let near=Infinity, nearK=0, nearA=0, gap=Infinity, gapK=0, user=0;
  const sect={F:Infinity,L:Infinity,R:Infinity,B:Infinity};
  for (let n=0;n<LIDAR_N;n++) {
    const a=S.th-Math.PI+n*TAU/LIDAR_N, dx=Math.cos(a), dy=Math.sin(a);
    let r=LIDAR_MAX, k=0, hitObj=null;
    for (const b of scanSet) { const t=rayBox(lx,ly,dx,dy,b,r); if (t<r&&t>=LIDAR_MIN) { r=t; k=b.kind==='door'?6:1; } }
    for (const o of S.objs) { if (o.h<LIDAR_H) continue; const t=rayBox(lx,ly,dx,dy,o,r); if (t<r&&t>=LIDAR_MIN) { r=t; k=2; hitObj=o; } }
    for (const w of S.walkers) { const t=rayCircle(lx,ly,dx,dy,w.x,w.y,WALKER_LEG_R); if (t<r&&t>=LIDAR_MIN) { r=t; k=3; hitObj=null; } }
    { const t=rayCircle(lx,ly,dx,dy,S.px,S.py,0.15); if (t<r&&t>=LIDAR_MIN) { r=t; k=4; hitObj=null; } }
    if (hitObj && !hitObj.seenL) { hitObj.seenL=true; S.needReplan=true; S.replanWhy=hitObj.cls; }
    S.scan[n]=r; S.scanK[n]=k;
    const lim=Math.min(r, MARK_RANGE);
    for (let t=0.2; t<lim-0.15; t+=RES) { const i=cx2i(lx+dx*t), j=cy2j(ly+dy*t); if (i>=0&&j>=0&&i<NX&&j<NY) { const kk=j*NX+i; if (marks[kk]===1) { marks[kk]=0; costDirty=true; } } }
    if (k===4) { user++; continue; }
    if (r>=LIDAR_MAX) continue;
    if (k===2 && r<=MARK_RANGE) {   // people are waited for, not planned around
      const i=cx2i(lx+dx*(r+0.03)), j=cy2j(ly+dy*(r+0.03));
      if (i>0&&j>0&&i<NX-1&&j<NY-1 && !occ[j*NX+i] && !marks[j*NX+i]) { marks[j*NX+i]=1; costDirty=true; }
    }
    const hx=lx+dx*r-S.x, hy=ly+dy*r-S.y, rx=hx*c+hy*s, ry=-hx*s+hy*c, dc=hyp(rx,ry);
    if (dc<near) { near=dc; nearK=k; nearA=Math.atan2(ry,rx); }
    if (k!==6 && Math.abs(ry)<HW+0.05 && rx>HL) { const g=rx-HL; if (g<gap) { gap=g; gapK=k; } }   // doors are handled by the door logic
    const ra=Math.atan2(ry,rx), sec=Math.abs(ra)<Math.PI/4?'F':Math.abs(ra)>3*Math.PI/4?'B':ra>0?'L':'R';
    if (dc<sect[sec]) sect[sec]=dc;
  }
  let camGap=Infinity;
  for (const o of S.objs) {
    if (!o.seenC || o.h>=LIDAR_H) continue;
    for (const [qx,qy] of [[o.x0,o.y0],[o.x1,o.y0],[o.x0,o.y1],[o.x1,o.y1],[o.cx,o.cy]]) {
      const hx=qx-S.x, hy=qy-S.y, rx=hx*c+hy*s, ry=-hx*s+hy*c;
      if (Math.abs(ry)<HW+0.1 && rx>HL) camGap=Math.min(camGap, rx-HL);
    }
  }
  S.nearest=near; S.nearestK=nearK; S.nearestA=nearA; S.sect=sect; S.userPts=user; S.lidarGap=gap;
  S.gap=Math.min(gap,camGap); S.gapK = camGap<gap ? 5 : gapK;
}
function camDetect() {
  const c=Math.cos(S.th), s=Math.sin(S.th), camx=S.x+CAM_OFF*c, camy=S.y+CAM_OFF*s, out=[];
  const occl=WALL_RECTS.filter(b=>rectDist(camx,camy,b)<CAM_RANGE);
  const consider=(cls,label,cx,cy,rad,h,ref,id)=>{
    const dx=cx-camx, dy=cy-camy, d=hyp(dx,dy); if (d>CAM_RANGE||d<0.25) return;
    const rel=wrap(Math.atan2(dy,dx)-S.th), half=Math.atan2(rad,d);
    if (Math.abs(rel)-half > CAM_HFOV/2) return;
    for (const b of occl) { if (b===ref) continue; const t=rayBox(camx,camy,dx/d,dy/d,b,d); if (t<d-rad-0.05) return; }
    let conf=0.97-0.05*d-(Math.abs(rel)>CAM_HFOV/2-0.05?0.12:0)+0.025*Math.sin(S.t*2.7+id*1.7);
    conf=clamp(conf,0,0.98); if (conf<0.45) return;
    out.push({cls, name:DET_NAME[cls]||cls, label, d, rel, conf, ref, id, cx, cy, h, rad});
  };
  for (const b of STATIC) if (b.kind==='furn') consider(b.cls,b.label,b.cx,b.cy,Math.max(b.sx,b.sy)/2,b.h,b,b.id);
  for (const o of S.objs) consider(o.cls,o.cls,o.cx,o.cy,Math.max(o.sx,o.sy)/2,o.h,o,o.id);
  for (const w of S.walkers) consider('walker','person',w.x,w.y,WALKER_R,1.72,w,w.id);
  DOORS.forEach((dr,i) => { if (!S.doors.some(q=>hyp(q.x-dr.x,q.y-dr.y)<0.8)) consider('doorway',dr.label,dr.x,dr.y,dr.w/2,2.1,dr,200+i); });
  S.doors.forEach((d,i) => consider(d.type==='lift'?'liftdoor':d.type==='auto'?'autodoor':'door', d.label, d.x, d.y, d.w/2, 2.1, d, 300+i));
  for (const o of out) { if (o.ref && o.ref.type) o.name = `${o.name} (${o.ref.f>0.9?'open':o.ref.f>0.1?'opening':'closed'})`; if (o.cls==='stairs') o.name=`stairs going ${o.ref.opt.dir}`; }
  S.dets=out.sort((a,b)=>a.d-b.d).slice(0,14);
  for (const d of out) if (d.ref && d.ref.isObj && !d.ref.seenC) {
    d.ref.seenC=true; markObj(d.ref,2); S.camNew=d.ref; S.camNewT=S.t; S.needReplan=true; S.replanWhy=d.ref.cls;
    tl('see', `Camera: ${d.name} at ${d.d.toFixed(1)} m (${pct(d.conf)})${d.ref.h<LIDAR_H?`. It is ${d.ref.h.toFixed(2)} m tall, under the LiDAR's 0.23 m scan plane, so the camera marks it on the costmap`:''}.`);
    rsay(`${cap1(d.name)} on the floor, ${sayDist(d.d)} ahead. I will take us around it.`, 1, 'stop');
  }
}

/* ================= Physics ================= */
function physics(dt) {
  const prof=PROF[S.profile], v=S.cmdV, w=S.cmdW;
  const nth=S.th+w*dt, nx=S.x+v*Math.cos(S.th)*dt, ny=S.y+v*Math.sin(S.th)*dt;
  const [hx,hy]=gripOf(nx,ny,nth), ang=Math.atan2(S.py-hy,S.px-hx), da=Math.abs(wrap(nth+Math.PI-ang));
  const told=Math.max(S.lastDirCue,S.lastSpokenTurn);
  const cued=S.t-told<3.5 || (cueMode==='v2' && S.inPlace && S.announced);
  const delay=cued ? 0.15 : prof.react*(1+0.6*S.psy.stress);
  if (da>0.12) { S.pReactT+=dt; if (S.pReactT>=delay) S.pReacting=true; }
  else if (da<0.05) { S.pReactT=0; S.pReacting=false; }
  const spd = S.psy.frozen ? 0 : (S.pReacting ? prof.walk*(1-0.4*S.psy.stress) : 0);
  const pref = S.inPlace ? S.spinDir : 0;
  updateDoors(dt);
  const ob=S.doorRects.length ? S.objs.concat(S.doorRects) : S.objs;
  const tryMove=(x,y,th)=>{
    if (roverClr(x,y,th,STATIC,ob,S.walkers,0) < 0) return null;
    return personStep(x,y,th,S.px,S.py,dt,spd,STATIC,ob,S.walkers,0,pref);
  };
  S.bumped=false;
  const ox=S.x, oy=S.y, oth=S.th, opx=S.px, opy=S.py;
  const anchored = S.psy.frozen && v>0.01;      // a frozen person holds the handle back
  let P=null;
  if (!anchored && (P=tryMove(nx,ny,nth))) { S.x=nx; S.y=ny; S.th=wrap(nth); }
  else if (Math.abs(w)>1e-3 && (P=tryMove(S.x,S.y,nth))) { S.th=wrap(nth); S.bumped=Math.abs(v)>0.02 && !anchored; }
  else if (!anchored && Math.abs(v)>1e-3 && (P=tryMove(nx,ny,S.th))) { S.x=nx; S.y=ny; S.bumped=Math.abs(w)>0.05; }
  else { P=tryMove(S.x,S.y,S.th); S.bumped = !anchored && (Math.abs(v)>1e-3||Math.abs(w)>1e-3); }
  if (P) { S.px=P[0]; S.py=P[1]; S.pBlocked=P[2]; }
  const [ghx,ghy]=gripOf(S.x,S.y,S.th);
  S.pth=Math.atan2(ghy-S.py, ghx-S.px);
  const moved=hyp(S.x-ox,S.y-oy), turned=wrap(S.th-oth), pmoved=hyp(S.px-opx,S.py-opy);
  S.pWalk+=pmoved; S.pSpeed=pmoved/dt; S.rSpeed=moved/dt;
  if (S.mode==='backup') S.backupDist+=moved;
  const realV=(v>=0?1:-1)*moved/dt, realW=turned/dt;
  S.realV=realV; S.realW=realW;
  const vl=realV-realW*TRACK/2, vr=realV+realW*TRACK/2, per=2*Math.PI*WHEEL_R/TICKS_PER_REV;
  S.accL+=vl*dt; S.accR+=vr*dt; S.wheelL=(S.wheelL||0)+vl*dt/WHEEL_R; S.wheelR=(S.wheelR||0)+vr*dt/WHEEL_R;
  while (Math.abs(S.accL)>=per) { const g=Math.sign(S.accL); S.ticksL+=g; S.accL-=g*per; }
  while (Math.abs(S.accR)>=per) { const g=Math.sign(S.accR); S.ticksR+=g; S.accR-=g*per; }
  stepWalkers(dt);
}

/* ================= Other people ================= */
function addWalker(path, speed, look, loop, life) {
  S.walkers.push({id:nextId++, path:path.map(p=>p.slice()), idx:1, bx:path[0][0], by:path[0][1], x:path[0][0], y:path[0][1],
    vx:0, vy:0, off:0, offT:0, speed, look:look||1, loop:!!loop, life:life||150, waitT:0, walk:0, face:0, done:false});
}
function stepWalkers(dt) {
  const [hx,hy]=gripOf(S.x,S.y,S.th);
  for (const w of S.walkers) {
    w.life-=dt; if (w.idx>=w.path.length) { w.done=true; continue; }
    const tg=w.path[w.idx], dx=tg[0]-w.bx, dy=tg[1]-w.by, d=hyp(dx,dy);
    if (d<0.05) { w.idx++; if (w.idx>=w.path.length && w.loop) { w.path.reverse(); w.idx=1; } continue; }
    const ux=dx/d, uy=dy/d, st=Math.min(d,w.speed*dt), nbx=w.bx+ux*st, nby=w.by+uy*st;
    const nx=nbx-uy*w.off, ny=nby+ux*w.off;
    let block=false;
    for (const o of ROVER_CIRC) if (hyp(nx-(S.x+o*Math.cos(S.th)), ny-(S.y+o*Math.sin(S.th))) < ROVER_R+WALKER_R+0.12 && hyp(w.x-S.x,w.y-S.y)>hyp(nx-S.x,ny-S.y)) block=true;
    if (hyp(nx-S.px,ny-S.py) < PR+WALKER_R+0.15 && hyp(w.x-S.px,w.y-S.py)>hyp(nx-S.px,ny-S.py)) block=true;
    if (hyp(nx-hx,ny-hy) < WALKER_R+0.1) block=true;
    for (const o of S.walkers) if (o!==w && hyp(nx-o.x,ny-o.y)<2*WALKER_R+0.1 && hyp(w.x-o.x,w.y-o.y)>hyp(nx-o.x,ny-o.y)) block=true;
    if (walkerDoorBlock(w,nx,ny,dt)) block=true;
    const ox=w.x, oy=w.y;
    if (block) {
      w.waitT+=dt;
      if (w.waitT>2.0 && w.offT===0) {   // step aside, away from the rover
        const side = ((S.x-w.x)*(-uy)+(S.y-w.y)*ux) > 0 ? -1 : 1;
        for (const sd of [side,-side]) { const tx=w.bx-uy*sd*0.6, ty=w.by+ux*sd*0.6; if (STATIC.every(b=>rectDist(tx,ty,b)>WALKER_R+0.05)) { w.offT=sd*0.6; break; } }
      }
      if (w.waitT>5) {   // no room to pass: turn back the way they came
        const back=[[w.bx,w.by], ...w.path.slice(0,w.idx).reverse()]; w.path=back; w.idx=1; w.waitT=0; w.offT=0;
      }
    } else { w.bx=nbx; w.by=nby; w.x=nx; w.y=ny; w.waitT=0; w.face=Math.atan2(uy,ux); }
    if (w.off!==w.offT) {
      const no=w.off+clamp(w.offT-w.off,-0.6*dt,0.6*dt), tx=w.bx-uy*no, ty=w.by+ux*no;
      if (STATIC.every(b=>rectDist(tx,ty,b)>WALKER_R) && hyp(tx-S.px,ty-S.py)>PR+WALKER_R+0.05) { w.off=no; w.x=tx; w.y=ty; }
      if (!block && w.waitT===0 && Math.abs(w.offT)>0 && hyp(w.x-S.x,w.y-S.y)>2) w.offT=0;
    }
    w.vx=(w.x-ox)/dt; w.vy=(w.y-oy)/dt; w.walk+=hyp(w.x-ox,w.y-oy);
  }
  S.walkers=S.walkers.filter(w=>!w.done && w.life>0);
}
