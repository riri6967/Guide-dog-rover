(() => {
'use strict';
const $ = id => document.getElementById(id);
const TAU = Math.PI*2;
const clamp = (v,a,b) => v<a?a:v>b?b:v;
const wrap = a => { a = (a+Math.PI) % TAU; if (a<0) a += TAU; return a-Math.PI; };
const hyp = Math.hypot;
const pct = v => Math.round(v*100)+'%';
const fmtT = t => { const m=Math.floor(t/60), s=Math.floor(t%60); return m+':'+String(s).padStart(2,'0'); };

/* ================= The rover, from rover.urdf.xacro ================= */
// chassis 0.274 x 0.208 m, track 0.290 m, wheels r 0.055 m; with wheels the footprint is 0.27 x 0.33 m
const VMAX=0.30, WMAX=1.0, ACC_V=0.5, ACC_W=2.0, XY_TOL=0.15, GOAL_TIMEOUT=260;
const HL=0.14, HW=0.17, ROVER_CIRC=[-0.07,0.07], ROVER_R=0.18;
// handle is rigid in yaw (it only pitches on its hinge): the grip sits 0.52 m behind base_link, 0.77 m up
const GRIP_X=URDF_FRAMES.grip[0], GRIP_H=URDF_FRAMES.grip[2]+URDF_FRAMES.wheel_radius;
const ARM=0.38, PR=0.24;                  // hand to body centre, body radius
const LIDAR_H=0.173+0.055, LIDAR_OFF=0.088, LIDAR_N=360, LIDAR_MIN=0.15, LIDAR_MAX=12, MARK_RANGE=3.5;
const CAM_OFF=0.14, CAM_H=0.0469+0.055, CAM_HFOV=66*Math.PI/180, CAM_RANGE=7;
const STOP_GAP=0.25, SLOW_GAP=1.2, WARN_GAP=1.4;
const WALKER_R=0.25, WALKER_LEG_R=0.15, WHEEL_R=0.055, TRACK=0.29, TICKS_PER_REV=32;

/* ================= Grid, costmap, planner ================= */
const RES=0.1, OX=-12.5, OY=-8.5, NX=250, NY=170, N=NX*NY;
const R_LETHAL=0.40;                     // the person follows the rover's line, so plan for the person's width
const occ=new Uint8Array(N), marks=new Uint8Array(N), dist=new Float32Array(N);   // marks: 1 LiDAR, 2 camera
const cx2i = x => Math.floor((x-OX)/RES), cy2j = y => Math.floor((y-OY)/RES);
const cellX = i => OX+(i+0.5)*RES, cellY = j => OY+(j+0.5)*RES;
let costDirty = true;
function rasterize(rects) {
  occ.fill(0);
  for (const b of rects) {
    const i0=Math.max(0,cx2i(b.x0-0.05)), i1=Math.min(NX-1,cx2i(b.x1+0.05)), j0=Math.max(0,cy2j(b.y0-0.05)), j1=Math.min(NY-1,cy2j(b.y1+0.05));
    for (let j=j0;j<=j1;j++) for (let i=i0;i<=i1;i++) { const x=cellX(i), y=cellY(j); if (x>b.x0-0.05&&x<b.x1+0.05&&y>b.y0-0.05&&y<b.y1+0.05) occ[j*NX+i]=1; }
  }
}
function computeDist() {
  const INF=1e9, d=dist, S2=Math.SQRT2;
  for (let k=0;k<N;k++) d[k] = (occ[k]||marks[k]) ? 0 : INF;
  for (let j=0;j<NY;j++) for (let i=0;i<NX;i++) { const k=j*NX+i; let v=d[k]; if (!v) continue;
    if (i>0) v=Math.min(v,d[k-1]+1);
    if (j>0) { v=Math.min(v,d[k-NX]+1); if (i>0) v=Math.min(v,d[k-NX-1]+S2); if (i<NX-1) v=Math.min(v,d[k-NX+1]+S2); }
    d[k]=v; }
  for (let j=NY-1;j>=0;j--) for (let i=NX-1;i>=0;i--) { const k=j*NX+i; let v=d[k]; if (!v) continue;
    if (i<NX-1) v=Math.min(v,d[k+1]+1);
    if (j<NY-1) { v=Math.min(v,d[k+NX]+1); if (i<NX-1) v=Math.min(v,d[k+NX+1]+S2); if (i>0) v=Math.min(v,d[k+NX-1]+S2); }
    d[k]=v; }
  for (let k=0;k<N;k++) d[k]=Math.max(0, d[k]*RES-0.05);
  costDirty = true;
}
const clearanceAt = (x,y) => { const i=cx2i(x), j=cy2j(y); if (i<0||j<0||i>=NX||j>=NY) return 0; return dist[j*NX+i]; };

const gS=new Float32Array(N), came=new Int32Array(N), closed=new Uint8Array(N);
const heapI=new Int32Array(N*4), heapF=new Float32Array(N*4);
function plan(sx, sy, gx, gy) {
  const si=cx2i(sx), sj=cy2j(sy); let gi=cx2i(gx), gj=cy2j(gy);
  if (si<1||sj<1||si>=NX-1||sj>=NY-1||gi<1||gj<1||gi>=NX-1||gj>=NY-1) return null;
  let snapped=false;
  if (dist[gj*NX+gi] < R_LETHAL) {
    let best=null, bd=1e9;
    for (let dj=-10;dj<=10;dj++) for (let di=-10;di<=10;di++) {
      const i=gi+di, j=gj+dj; if (i<1||j<1||i>=NX-1||j>=NY-1) continue;
      const d2=di*di+dj*dj; if (d2>100||d2>=bd) continue;
      if (dist[j*NX+i] >= R_LETHAL) { bd=d2; best=[i,j]; }
    }
    if (!best) return null; gi=best[0]; gj=best[1]; snapped=true;
  }
  const passable = (i,j) => { const c=dist[j*NX+i]; if (c>=R_LETHAL) return true;
    const dx=(i-si)*RES, dy=(j-sj)*RES; return c>=0.15 && dx*dx+dy*dy<0.8*0.8; };
  gS.fill(Infinity); came.fill(-1); closed.fill(0);
  let hn=0;
  const push=(k,f)=>{ let c=hn++; heapI[c]=k; heapF[c]=f;
    while (c>0) { const p=(c-1)>>1; if (heapF[p]<=heapF[c]) break; [heapI[p],heapI[c]]=[heapI[c],heapI[p]]; [heapF[p],heapF[c]]=[heapF[c],heapF[p]]; c=p; } };
  const pop=()=>{ const top=heapI[0]; hn--; heapI[0]=heapI[hn]; heapF[0]=heapF[hn];
    let c=0; for(;;){ const l=2*c+1, r=l+1; let m=c; if (l<hn&&heapF[l]<heapF[m]) m=l; if (r<hn&&heapF[r]<heapF[m]) m=r; if (m===c) break;
      [heapI[m],heapI[c]]=[heapI[c],heapI[m]]; [heapF[m],heapF[c]]=[heapF[c],heapF[m]]; c=m; } return top; };
  const h=(i,j)=>{ const dx=Math.abs(i-gi), dy=Math.abs(j-gj); return dx+dy+(Math.SQRT2-2)*Math.min(dx,dy); };
  const pen = d => d>=1.1 ? 0 : 4.0*((1.1-d)/(1.1-R_LETHAL))**2;   // keep to the middle of aisles
  const sk=sj*NX+si, gk=gj*NX+gi; gS[sk]=0; push(sk,h(si,sj));
  const D=[[1,0,1],[-1,0,1],[0,1,1],[0,-1,1],[1,1,Math.SQRT2],[1,-1,Math.SQRT2],[-1,1,Math.SQRT2],[-1,-1,Math.SQRT2]];
  let found=false;
  while (hn>0 && hn<heapI.length-8) {
    const k=pop(); if (closed[k]) continue; closed[k]=1;
    if (k===gk) { found=true; break; }
    const i=k%NX, j=(k/NX)|0;
    for (const [di,dj,len] of D) {
      const ni=i+di, nj=j+dj; if (ni<1||nj<1||ni>=NX-1||nj>=NY-1) continue;
      const nk=nj*NX+ni; if (closed[nk]||!passable(ni,nj)) continue;
      if (di&&dj&&(!passable(i+di,j)||!passable(i,j+dj))) continue;
      const g=gS[k]+len*(1+pen(dist[nk]));
      if (g<gS[nk]) { gS[nk]=g; came[nk]=k; push(nk, g+h(ni,nj)); }
    }
  }
  if (!found) return null;
  const cells=[]; for (let k=gk;k!==-1;k=came[k]) cells.push(k); cells.reverse();
  let pts=cells.map(k=>[cellX(k%NX), cellY((k/NX)|0)]);
  pts[0]=[sx,sy]; if (!snapped) pts[pts.length-1]=[gx,gy];
  for (let it=0;it<40;it++) for (let n=1;n<pts.length-1;n++) {
    const a=pts[n-1], p=pts[n], b=pts[n+1];
    const nx=p[0]+0.3*(a[0]+b[0]-2*p[0]), ny=p[1]+0.3*(a[1]+b[1]-2*p[1]);
    if (clearanceAt(nx,ny) >= Math.min(clearanceAt(p[0],p[1]), 0.7)) pts[n]=[nx,ny];
  }
  const out=[pts[0]];
  for (let n=1;n<pts.length;n++) {
    const last=out[out.length-1], p=pts[n], L=hyp(p[0]-last[0],p[1]-last[1]);
    if (L<0.1) { if (n===pts.length-1) out.push(p); continue; }
    const steps=Math.floor(L/0.1);
    for (let s=1;s<=steps;s++) out.push([last[0]+(p[0]-last[0])*s/steps, last[1]+(p[1]-last[1])*s/steps]);
  }
  return {pts:out, snapped, goal:[cellX(gi),cellY(gj)]};
}

/* ================= Geometry ================= */
function rectDist(px,py,b){ const dx=Math.max(b.x0-px,0,px-b.x1), dy=Math.max(b.y0-py,0,py-b.y1); return Math.sqrt(dx*dx+dy*dy); }
function rayBox(ox,oy,dx,dy,b,maxR){
  let tmin=0, tmax=maxR;
  if (Math.abs(dx)<1e-9) { if (ox<b.x0||ox>b.x1) return Infinity; }
  else { let t1=(b.x0-ox)/dx, t2=(b.x1-ox)/dx; if (t1>t2){const t=t1;t1=t2;t2=t;} if (t1>tmin) tmin=t1; if (t2<tmax) tmax=t2; if (tmin>tmax) return Infinity; }
  if (Math.abs(dy)<1e-9) { if (oy<b.y0||oy>b.y1) return Infinity; }
  else { let t1=(b.y0-oy)/dy, t2=(b.y1-oy)/dy; if (t1>t2){const t=t1;t1=t2;t2=t;} if (t1>tmin) tmin=t1; if (t2<tmax) tmax=t2; if (tmin>tmax) return Infinity; }
  return tmin;
}
function rayCircle(ox,oy,dx,dy,cx,cy,r){
  const fx=ox-cx, fy=oy-cy, b=fx*dx+fy*dy, c=fx*fx+fy*fy-r*r, disc=b*b-c;
  if (disc<0) return Infinity; const t=-b-Math.sqrt(disc); return t>=0?t:Infinity;
}
function roverClr(x,y,th,rects,objs,wk,tOff){
  const c=Math.cos(th), s=Math.sin(th); let m=9;
  for (const o of ROVER_CIRC) {
    const cx=x+o*c, cy=y+o*s;
    for (const b of rects) { const d=rectDist(cx,cy,b)-ROVER_R; if (d<m) m=d; }
    for (const b of objs) { const d=rectDist(cx,cy,b)-ROVER_R; if (d<m) m=d; }
    for (const w of wk) { const d=hyp(cx-w.x-w.vx*tOff, cy-w.y-w.vy*tOff)-ROVER_R-WALKER_R; if (d<m) m=d; }
  }
  return m;
}
function personClr(px,py,rects,objs,wk,tOff){
  let m=9;
  for (const b of rects) { const d=rectDist(px,py,b)-PR; if (d<m) m=d; }
  for (const b of objs) { const d=rectDist(px,py,b)-PR; if (d<m) m=d; }
  for (const w of wk) { const d=hyp(px-w.x-w.vx*tOff, py-w.y-w.vy*tOff)-PR-WALKER_R; if (d<m) m=d; }
  return m;
}
const gripOf = (x,y,th) => [x+GRIP_X*Math.cos(th), y+GRIP_X*Math.sin(th)];
// The grip is fixed to the rover; the person's body hangs off it at arm's length and steps round
// toward "straight behind" at speed spd. prefDir forces the long way round.
function personStep(cx,cy,th,px,py,dt,spd,rects,objs,wk,tOff,prefDir){
  const [hx,hy]=gripOf(cx,cy,th);
  const ang=Math.atan2(py-hy,px-hx);
  const rx=hx+ARM*Math.cos(ang), ry=hy+ARM*Math.sin(ang);
  if (personClr(rx,ry,rects,objs,wk,tOff) < 0) return null;
  if (spd<=0) return [rx,ry,false];
  let da=wrap(th+Math.PI-ang);
  if (prefDir && Math.abs(da)>0.3 && Math.sign(da)!==prefDir) da = da>0 ? da-TAU : da+TAU;
  const maxDa=spd*dt/ARM; let st=clamp(da,-maxDa,maxDa);
  for (let k=0;k<3&&Math.abs(st)>1e-4;k++) {
    const a=ang+st, x=hx+ARM*Math.cos(a), y=hy+ARM*Math.sin(a);
    if (personClr(x,y,rects,objs,wk,tOff)>=0) return [x,y,false];
    st*=0.5;
  }
  return [rx,ry,Math.abs(da)>0.05];
}
const personRel = () => { const [hx,hy]=gripOf(S.x,S.y,S.th); return wrap(Math.atan2(S.py-hy,S.px-hx)-S.th-Math.PI); };
