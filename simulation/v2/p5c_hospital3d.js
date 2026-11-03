
/* ================= Hospital furniture and decor ================= */
TEX.hosp = (c,w,h)=>{ c.fillStyle='#CFE3DD'; c.fillRect(0,0,w,h); const r=rng(23); for (let i=0;i<5000;i++){ c.fillStyle=`rgba(${r()>.5?255:60},${r()>.5?255:110},${r()>.5?255:100},${r()*0.1})`; c.fillRect(r()*w,r()*h,2,2); } };
TEX.hosp2 = (c,w,h)=>{ c.fillStyle='#E2DDD3'; c.fillRect(0,0,w,h); const r=rng(29); for (let i=0;i<5000;i++){ c.fillStyle=`rgba(${r()>.5?255:90},${r()>.5?255:80},${r()>.5?255:70},${r()*0.08})`; c.fillRect(r()*w,r()*h,2,2); } };
TEX.liftcar = (c,w,h)=>{ c.fillStyle='#3A4048'; c.fillRect(0,0,w,h); c.fillStyle='#4A515A'; for (let i=8;i<w;i+=16) for (let j=8;j<h;j+=16) { c.beginPath(); c.arc(i,j,4,0,TAU); c.fill(); } };
function labelTex(txt, bg, fg) { return canvasTex(512,96,(c,W,H)=>{ c.fillStyle=bg; c.fillRect(0,0,W,H); c.fillStyle=fg||'#fff'; c.font='bold 46px Atkinson Hyperlegible, sans-serif'; c.textAlign='center'; c.textBaseline='middle'; c.fillText(txt,W/2,H/2+2); }); }
function lyingPatient(g, d, w) {   // under the blanket, head on the raised pillow
  const skin=mat(['#8D5A3B','#E0B892','#C68A5E'][(d*10|0)%3],{roughness:0.6});
  const head=new THREE.Mesh(new THREE.SphereGeometry(0.11,16,12),skin); head.position.set(-d/2+0.35,0.93,0); g.add(head);
  const body=boxMesh(d*0.55,0.16,w*0.42,mat('#9FC3D9',{roughness:1}),d*0.08,0.7,0); g.add(body);
  const torso=boxMesh(0.5,0.18,w*0.42,mat('#B8D4E6',{roughness:1}),-d/2+0.75,0.78,0); torso.rotation.z=-0.35; g.add(torso);
}
Object.assign(BUILD, {
  hbed(f){ const g=grp(f), {d,w}=dims(f), steel=M.steel, frame=mat('#D5DADF',{roughness:0.4,metalness:0.3});
    for (const [a,b] of [[-1,-1],[1,-1],[-1,1],[1,1]]) { g.add(boxMesh(0.05,0.32,0.05,frame,a*(d/2-0.08),0.2,b*(w/2-0.08))); const c=cyl(0.05,0.05,0.04,M.black,12); c.rotation.x=Math.PI/2; c.position.set(a*(d/2-0.08),0.05,b*(w/2-0.08)); g.add(c); }
    g.add(boxMesh(d,0.1,w,frame,0,0.4,0));
    g.add(boxMesh(d*0.62,0.16,w-0.06,mat('#EAF2F7',{roughness:0.9}),d*0.17,0.53,0));
    const back=boxMesh(d*0.36,0.16,w-0.06,mat('#EAF2F7',{roughness:0.9}),-d*0.27,0.64,0); back.rotation.z=-0.45; g.add(back);
    if (!f.opt.patient) g.add(boxMesh(d*0.5,0.05,w-0.03,mat('#9FC3D9',{roughness:1}),d*0.2,0.63,0));
    const pil=boxMesh(0.4,0.1,w*0.6,M.pillow,-d/2+0.32,0.82,0); pil.rotation.z=-0.45; g.add(pil);
    g.add(boxMesh(0.05,0.62,w,mat('#C8BBA6',{roughness:0.6}),-d/2+0.02,0.72,0)); g.add(boxMesh(0.05,0.4,w,mat('#C8BBA6',{roughness:0.6}),d/2-0.02,0.6,0));
    g.add(boxMesh(0.01,0.22,0.16,M.offwhite,d/2+0.005,0.66,0.2));
    for (const z of [-1,1]) g.add(boxMesh(d*0.4,0.18,0.03,steel,-d*0.15,0.74,z*(w/2+0.015)));
    if (f.opt.patient) lyingPatient(g,d,w);
    // bed-head services panel: oxygen, suction, call bell, light
    g.add(boxMesh(0.06,0.22,w+0.6,mat('#E8ECEF'),-d/2-0.12,1.45,0));
    for (const [z,c] of [[-0.35,'#FFFFFF'],[-0.2,'#2F6FDE'],[0.2,'#3BA55D'],[0.35,'#E9B949']]) { const o=cyl(0.025,0.025,0.02,mat(c),10); o.rotation.z=Math.PI/2; o.position.set(-d/2-0.085,1.42,z); g.add(o); }
    const mon=boxMesh(0.06,0.24,0.32,M.black,-d/2-0.05,1.85,w*0.45); g.add(mon); const ms=new THREE.Mesh(new THREE.PlaneGeometry(0.28,0.2),M.screens[3]); ms.rotation.y=Math.PI/2; ms.position.set(-d/2-0.015,1.85,w*0.45); g.add(ms);
    return g; },
  overbed(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,0.04,0.08,M.plastic,0,0.04,-w/2+0.06)); g.add(boxMesh(0.05,h-0.05,0.05,M.steel,0,h/2,-w/2+0.06));
    g.add(boxMesh(d,0.03,w,mat('#D9CBB3'),0,h,0)); const cup=cyl(0.035,0.03,0.1,M.white,12); cup.position.set(0,h+0.065,w*0.2); g.add(cup); g.add(boxMesh(0.12,0.08,0.22,mat('#9FC3D9'),0.05,h+0.055,-w*0.15)); return g; },
  ivpole(f){ const g=grp(f); for (let i=0;i<5;i++) { const a=i*TAU/5, l=boxMesh(0.24,0.025,0.03,M.steel,Math.cos(a)*0.12,0.05,Math.sin(a)*0.12); l.rotation.y=-a; g.add(l); }
    const p=cyl(0.012,0.012,1.85,M.steel); p.position.y=0.95; g.add(p); g.add(boxMesh(0.12,0.16,0.18,mat('#E8ECEF'),0.05,1.15,0)); g.add(boxMesh(0.03,0.04,0.12,M.screens[3],0.11,1.18,0));
    g.add(boxMesh(0.04,0.2,0.12,mat('#DDEEF8',{transparent:true,opacity:0.7}),0,1.72,0.06)); return g; },
  vending(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,h,w,mat('#B03A3A',{roughness:0.4,metalness:0.2}),0,h/2,0));
    const gl=new THREE.Mesh(new THREE.PlaneGeometry(w*0.65,h*0.6),mat('#1A2A33',{emissive:'#2A4A5A',emissiveIntensity:0.6,roughness:0.1})); gl.rotation.y=Math.PI/2; gl.position.set(d/2+0.002,h*0.6,-w*0.12); g.add(gl);
    const r=rng(f.id); for (let i=0;i<5;i++) for (let j=0;j<5;j++) g.add(boxMesh(0.02,0.08,0.06,M.books[(r()*M.books.length)|0],d/2+0.004,h*0.38+i*0.17,-w*0.38+j*0.11));
    g.add(boxMesh(0.02,0.3,0.14,M.plastic,d/2+0.01,h*0.6,w*0.34)); return g; },
  pcounter(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,h-0.05,w,mat('#E7E3DA',{roughness:0.6}),0,(h-0.05)/2,0)); g.add(boxMesh(d+0.06,0.05,w+0.04,mat('#6E8E9A',{roughness:0.4}),0.02,h-0.02,0));
    const sign=new THREE.Mesh(new THREE.PlaneGeometry(Math.min(w*0.6,1.6),0.28),new THREE.MeshBasicMaterial({map:labelTex(f.label.toUpperCase(),'#1F6F8B')})); sign.rotation.y=Math.PI/2; sign.position.set(d/2+0.004,h*0.6,0); g.add(sign);
    g.add(boxMesh(d*0.5,0.03,w*0.25,mat('#6E8E9A'),d*0.25,0.75,-w*0.3));   // lower wheelchair-height section
    if (f.opt.glass) { g.add(boxMesh(0.015,0.55,w,M.glassBlue,d/2-0.05,h+0.28,0)); for (const z of [-w*0.25,w*0.25]) g.add(boxMesh(0.02,0.006,0.4,M.steel,d/2-0.05,h+0.02,z)); }
    if (f.opt.pc) for (const z of [-w*0.22,w*0.22]) { const it=new THREE.Group(); it.position.set(-0.05,h,z); monitor(it,0,0,0,Math.PI,2); g.add(it); }
    return g; },
  couch(f){ const g=grp(f), {d,w}=dims(f), h=f.h; for (const [a,b] of [[-1,-1],[1,-1],[-1,1],[1,1]]) g.add(boxMesh(0.05,h-0.1,0.05,M.steel,a*(d/2-0.06),(h-0.1)/2,b*(w/2-0.08)));
    g.add(boxMesh(d,0.1,w*0.7,mat('#2F6F78',{roughness:0.8}),0,h-0.05,w*0.15)); const hd=boxMesh(d,0.1,w*0.3,mat('#2F6F78',{roughness:0.8}),0,h+0.08,-w*0.36); hd.rotation.x=0.45; g.add(hd);
    g.add(boxMesh(d*0.7,0.005,w*0.8,M.white,0,h+0.003,0.05)); return g; },
  scanner(f){ const g=grp(f), {d,w}=dims(f), white=mat('#EEF1F3',{roughness:0.35});
    const ring=new THREE.Mesh(new THREE.TorusGeometry(0.62,0.26,16,40),white); ring.rotation.y=Math.PI/2; ring.position.set(-d/2+0.45,0.95,0); g.add(ring);
    g.add(boxMesh(0.55,0.5,w*0.95,white,-d/2+0.45,0.25,0)); const bore=new THREE.Mesh(new THREE.CylinderGeometry(0.38,0.38,0.56,32,1,true),mat('#C9D3DB',{side:THREE.DoubleSide})); bore.rotation.z=Math.PI/2; bore.position.set(-d/2+0.45,0.95,0); g.add(bore);
    g.add(boxMesh(d*0.6,0.5,0.4,white,d*0.12,0.25,0)); g.add(boxMesh(d*0.75,0.08,0.5,mat('#DDE4EA'),d*0.08,0.78,0)); g.add(boxMesh(d*0.6,0.03,0.42,mat('#5F7F8C'),d*0.12,0.835,0)); return g; },
  seatbench(f){ const g=grp(f), {d,w}=dims(f); addLegs(g,d,w,0.42,M.steel,0.05,0.04); g.add(boxMesh(d,0.05,w,mat('#7A8A99'),0,0.45,0)); g.add(boxMesh(0.05,0.4,w,mat('#7A8A99'),-d/2+0.03,0.7,0)); return g; },
  wheelchair(f){ const g=grp(f), {d,w}=dims(f);
    for (const z of [-1,1]) { const wh=new THREE.Mesh(new THREE.TorusGeometry(0.28,0.02,8,28),M.black); wh.position.set(-0.05,0.3,z*(w/2-0.02)); g.add(wh); const c=cyl(0.04,0.04,0.03,M.black,10); c.rotation.x=Math.PI/2; c.position.set(d/2-0.06,0.04,z*(w/2-0.08)); g.add(c); }
    g.add(boxMesh(0.42,0.05,w-0.12,mat('#2A3440'),0.02,0.5,0)); g.add(boxMesh(0.04,0.42,w-0.12,mat('#2A3440'),-0.2,0.75,0));
    for (const z of [-1,1]) { g.add(boxMesh(0.04,0.04,0.04,M.steel,-0.24,0.95,z*(w/2-0.1))); g.add(boxMesh(0.3,0.03,0.04,M.steel,0.05,0.68,z*(w/2-0.06))); }
    g.add(boxMesh(0.12,0.02,w-0.2,M.steel,d/2-0.05,0.12,0)); return g; },
  trolley(f){ const g=grp(f), {d,w}=dims(f); addLegs(g,d,w,1.0,M.steel,0.03,0.025);
    for (const y of [0.15,0.55,0.95]) g.add(boxMesh(d,0.02,w,mat('#DDE2E6',{metalness:0.4,roughness:0.4}),0,y,0));
    const r=rng(f.id); for (let i=0;i<5;i++) g.add(boxMesh(0.14,0.1,0.18,[M.white,mat('#9FC3D9'),M.offwhite][i%3],(r()-0.5)*d*0.6,0.62+(i>2?0.4:0),(r()-0.5)*w*0.8)); return g; },
  sinkunit(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,h,w,M.white,0,h/2,0)); g.add(boxMesh(d*0.7,0.04,w*0.7,M.steel,0,h+0.01,0));
    const tap=cyl(0.012,0.012,0.22,M.steel); tap.position.set(-d/2+0.06,h+0.11,0); g.add(tap); g.add(boxMesh(0.08,0.14,0.1,M.white,-d/2+0.02,h+0.45,w*0.3));
    const mir=new THREE.Mesh(new THREE.PlaneGeometry(Math.min(w,0.5),0.6),mat('#C9DDE8',{metalness:0.9,roughness:0.05})); mir.rotation.y=Math.PI/2; mir.position.set(-d/2+0.005,h+0.7,0); g.add(mir); return g; },
  kiosk(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d*0.6,f.h*0.65,w*0.7,mat('#1F6F8B',{roughness:0.4}),-0.05,f.h*0.33,0));
    const hd=boxMesh(0.06,0.42,w,M.black,0.05,f.h-0.2,0); hd.rotation.z=0.25; g.add(hd); const sc=new THREE.Mesh(new THREE.PlaneGeometry(w*0.85,0.34),M.screens[2]); sc.rotation.y=Math.PI/2; sc.rotation.x=-0.25; sc.position.set(0.085,f.h-0.2,0); g.add(sc); return g; },
  medtrolley(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.85,w,mat('#2F6FDE',{roughness:0.5}),0,0.5,0)); for (let i=0;i<5;i++) g.add(boxMesh(0.005,0.01,w-0.06,M.white,d/2+0.003,0.2+i*0.15,0));
    for (const [a,b] of [[-1,-1],[1,-1],[-1,1],[1,1]]) { const c=cyl(0.035,0.035,0.03,M.black,10); c.rotation.x=Math.PI/2; c.position.set(a*(d/2-0.06),0.04,b*(w/2-0.06)); g.add(c); } g.add(boxMesh(0.03,0.03,w,M.steel,-d/2-0.04,0.95,0)); return g; },
  crashcart(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.9,w,mat('#C0392B',{roughness:0.45}),0,0.52,0)); for (let i=0;i<5;i++) g.add(boxMesh(0.005,0.01,w-0.06,M.white,d/2+0.003,0.2+i*0.16,0));
    g.add(boxMesh(0.25,0.15,0.3,mat('#E9B949'),0,1.05,0)); g.add(boxMesh(0.2,0.12,0.25,M.black,0,1.2,0)); return g; },
  parallelbars(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.04,w,mat('#3A4654'),0,0.02,0));
    for (const zs of [-1,1]) { const z=zs*0.3; const bar=cyl(0.025,0.025,d-0.1,mat('#C9A27A',{roughness:0.4}),10); bar.rotation.z=Math.PI/2; bar.position.set(0,0.95,z); g.add(bar);
      for (const x of [-d/2+0.1,0,d/2-0.1]) g.add(boxMesh(0.04,0.95,0.04,M.steel,x,0.48,z)); } return g; },
  treadmill(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.2,w,M.plastic,0,0.1,0)); g.add(boxMesh(d-0.15,0.01,w-0.2,M.black,0.05,0.205,0));
    for (const z of [-1,1]) g.add(boxMesh(0.05,1.1,0.05,M.steel,-d/2+0.1,0.75,z*(w/2-0.06))); g.add(boxMesh(0.25,0.3,w,M.plastic,-d/2+0.1,1.3,0)); return g; },
  exbike(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.06,0.5,M.plastic,0,0.03,0)); g.add(boxMesh(0.08,0.8,0.08,M.plastic,-0.1,0.45,0)); g.add(boxMesh(0.28,0.06,0.22,M.black,-0.12,0.86,0));
    g.add(boxMesh(0.08,1.0,0.08,M.plastic,d/2-0.15,0.55,0)); g.add(boxMesh(0.06,0.04,0.45,M.steel,d/2-0.15,1.05,0)); const fw=new THREE.Mesh(new THREE.TorusGeometry(0.18,0.04,8,20),M.steel); fw.position.set(d/2-0.25,0.3,0); g.add(fw); return g; },
  stairs(f){ const g=new THREE.Group(), n=13, rise=3.2/n, dp=(f.y1-f.y0)/n, up=f.opt.dir==='up';
    const stepM=mat('#B9BEC4',{roughness:0.7}), nose=mat('#E9C23B',{roughness:0.5}), railM=mat('#C9A27A',{roughness:0.5});
    for (let i=0;i<n;i++) {
      const y1=f.y1-i*dp, top = up ? (i+1)*rise : -(i+1)*rise, bottom = up ? 0 : -3.25;
      const h=top-bottom; g.add(boxMesh(f.sx,h,dp,stepM,f.cx,bottom+h/2,-(y1-dp/2)));
      g.add(boxMesh(f.sx,0.012,0.05,nose,f.cx,top+0.006,-(y1-0.03)));        // contrasting nosing on every step
    }
    for (const x of [f.x0+0.08,f.x1-0.08]) {   // handrails both sides, following the pitch
      const r=cyl(0.022,0.022,1,railM,10), a=V3s(x,(up?rise:-rise)+0.9,f.y1), b=V3s(x,(up?3.2:-3.2)+0.9,f.y0); stick(r,a,b); g.add(r);
      const r2=cyl(0.022,0.022,0.3,railM,10); stick(r2,V3s(x,0.9,f.y1+0.3),a); g.add(r2);
    }
    // tactile warning strip (corduroy paving) on the landing at the top of a flight going down, and at the foot of one going up
    const strip=new THREE.Mesh(new THREE.PlaneGeometry(f.sx,0.4), mat('#fff',{map:canvasTex(256,64,(c,W,H)=>{ c.fillStyle='#E9C23B'; c.fillRect(0,0,W,H); c.fillStyle='rgba(0,0,0,.25)'; for (let y=4;y<H;y+=10) c.fillRect(0,y,W,3); }),roughness:0.9}));
    strip.rotation.x=-Math.PI/2; strip.position.set(f.cx,0.011,-(f.y1+0.2)); g.add(strip);
    const sign=new THREE.Mesh(new THREE.PlaneGeometry(1.2,0.24),new THREE.MeshBasicMaterial({map:labelTex(up?'STAIRS UP ▲':'STAIRS DOWN ▼','#2A3440')})); sign.position.set(f.cx,2.3,-(f.y1+0.02)); sign.rotation.y=Math.PI; g.add(sign);
    if (!up) { const pit=mat('#6E757D',{roughness:0.9}); g.add(boxMesh(0.05,3.25,f.sy,pit,f.x0+0.02,-1.625,-f.cy)); g.add(boxMesh(0.05,3.25,f.sy,pit,f.x1-0.02,-1.625,-f.cy)); g.add(boxMesh(f.sx,3.25,0.05,pit,f.cx,-1.625,-(f.y0+0.02))); }
    return g; },
});
const V3s=(x,h,y)=>new THREE.Vector3(x,h,-y);
function seatedPerson(x,y,face,shirt,trousers,skin) {
  const g=new THREE.Group(); g.position.set(x,0,-y); g.rotation.y=FACE_ROT[face]||0;
  const ms=mat(shirt), mt=mat(trousers), mk_=mat(skin,{roughness:0.6});
  const t=cyl(0.17,0.15,0.55,ms,12); t.scale.z=1.2; t.scale.x=0.75; t.position.set(-0.08,0.8,0); g.add(t);
  const hd=new THREE.Mesh(new THREE.SphereGeometry(0.105,16,12),mk_); hd.position.set(-0.06,1.2,0); g.add(hd);
  for (const z of [-0.1,0.1]) { g.add(boxMesh(0.42,0.13,0.13,mt,0.12,0.55,z)); g.add(boxMesh(0.12,0.48,0.12,mt,0.33,0.26,z)); g.add(boxMesh(0.22,0.06,0.1,mat('#1B1D21'),0.38,0.03,z)); g.add(boxMesh(0.32,0.08,0.08,ms,0.1,0.68,z*2.1)); }
  g.traverse(o=>{ if (o.isMesh) o.castShadow=true; });
  return g;
}
function decorHospital(scene) {
  const out=[], L=LEVEL, D=levelData('hospital',L);
  const add=o=>{ scene.add(o); out.push(o); return o; };
  const art=(x,y,ry,w,h,seed,yy)=>{ const g=new THREE.Group(); g.add(boxMesh(w+0.06,h+0.06,0.03,mat('#E8E4DC'),0,0,0)); const p=new THREE.Mesh(new THREE.PlaneGeometry(w,h), mat('#fff',{map:canvasTex(256,192,(c,W,H)=>TEX.art(c,W,H,seed)),roughness:0.9})); p.position.z=0.017; g.add(p); g.position.set(x,yy||1.55,-y); g.rotation.y=ry; add(g); };
  const win=(x,y,ry,w)=>{ const g=new THREE.Group(); g.add(boxMesh(w+0.1,1.3,0.06,M.white,0,0,0)); const gl=new THREE.Mesh(new THREE.PlaneGeometry(w,1.2),mat('#BFE0F5',{emissive:'#9CC7E8',emissiveIntensity:0.6})); gl.position.z=0.032; g.add(gl); g.add(boxMesh(0.04,1.2,0.07,M.white,0,0,0.01)); g.position.set(x,1.55,-y); g.rotation.y=ry; add(g); };
  const sign=(x,y,ry,txt,bg)=>{ const s=new THREE.Mesh(new THREE.PlaneGeometry(1.1,0.2),new THREE.MeshBasicMaterial({map:labelTex(txt,bg||'#1F6F8B')})); s.position.set(x,2.25,-y); s.rotation.y=ry; add(s); };
  const hang=(x,y,txt)=>{ const g=new THREE.Group(); g.add(boxMesh(1.8,0.3,0.04,mat('#1F6F8B'),0,0,0)); for (const s of [-1,1]) { const p=new THREE.Mesh(new THREE.PlaneGeometry(1.74,0.26),new THREE.MeshBasicMaterial({map:labelTex(txt,'#1F6F8B')})); p.position.z=s*0.022; if (s<0) p.rotation.y=Math.PI; g.add(p); }
    for (const sx of [-0.8,0.8]) g.add(boxMesh(0.01,0.25,0.01,M.steel,sx,0.27,0)); g.position.set(x,2.3,-y); g.rotation.y=Math.PI/2; add(g); };
  // wayfinding lines, handrails and protective rails, hand sanitiser
  for (const [y,c] of [[-0.35,'#2F6FDE'],[0.35,'#3BA55D']]) { const l=new THREE.Mesh(new THREE.PlaneGeometry(23.6,0.1),mat(c,{roughness:0.7})); l.rotation.x=-Math.PI/2; l.position.set(0,0.009,-y); add(l); }
  const railM=mat('#C9A27A',{roughness:0.5}), bumpM=mat('#5F8F86',{roughness:0.5});
  for (const w of D.walls) if (Math.abs(Math.abs(w[1])-1.5)<1e-6 && w[3]<0.5 && w[2]>0.8) {
    const side=w[1]>0?-1:1, y=w[1]+side*0.13, r=cyl(0.022,0.022,w[2]-0.3,railM,10); r.rotation.z=Math.PI/2; r.position.set(w[0],0.9,-y); add(r);
    add(boxMesh(w[2]-0.2,0.12,0.03,bumpM,w[0],0.32,-(w[1]+side*0.115)));
    if (w[2]>2.5) add(boxMesh(0.12,0.2,0.08,M.white,w[0],1.25,-(w[1]+side*0.14)));
  }
  // the lift: steel car lining, handrail, mirror, buttons; landing floor indicator and call panel
  const steel=mat('#B5BCC4',{roughness:0.3,metalness:0.8});
  add(boxMesh(2.75,2.4,0.02,steel,7.5,1.2,7.88)); add(boxMesh(0.02,2.4,2.6,steel,6.12,1.2,6.55)); add(boxMesh(0.02,2.4,2.6,steel,8.88,1.2,6.55));
  const mir=new THREE.Mesh(new THREE.PlaneGeometry(1.4,1.2),mat('#C9DDE8',{metalness:0.9,roughness:0.05})); mir.position.set(7.5,1.4,7.86); mir.rotation.y=Math.PI; add(mir);
  for (const [x,z,ry,len] of [[7.5,7.83,0,2.4],[6.17,6.55,Math.PI/2,2.2],[8.83,6.55,Math.PI/2,2.2]]) { const r=cyl(0.02,0.02,len,steel,10); r.rotation.z=Math.PI/2; r.rotation.y=ry; r.position.set(x,0.9,z); add(r); }
  add(boxMesh(0.03,0.6,0.25,M.black,8.86,1.15,5.5));
  const ind=new THREE.Mesh(new THREE.PlaneGeometry(0.5,0.22),new THREE.MeshBasicMaterial({map:labelTex(`▲▼  ${L}`,'#0D1117','#FFB13C')})); ind.position.set(7.5,2.45,4.99); ind.rotation.y=Math.PI; add(ind);
  add(boxMesh(0.14,0.3,0.03,steel,6.6,1.0,4.99)); for (const [yy,c] of [[1.06,'#3CFF8A'],[0.94,'#FFB13C']]) { const b=cyl(0.025,0.025,0.02,new THREE.MeshBasicMaterial({color:c}),12); b.rotation.x=Math.PI/2; b.position.set(6.6,yy,4.975); add(b); }
  sign(7.5,-5.0,Math.PI,`LIFT · ${L===0?'GROUND FLOOR':'FLOOR 1'}`,'#2A3440');
  add(boxMesh(0.25,0.35,0.12,mat('#3BA55D'),6.12,1.4,2.6));   // AED cabinet
  const ext=cyl(0.07,0.07,0.45,M.red,14); ext.position.set(-11.88+0.1,0.5,0.9); add(ext);
  if (L===0) {
    sign(-8.25,1.38,0,'WAITING AREA'); sign(-2.3,1.38,0,'PATIENT ROOM 1'); sign(3.9,1.38,0,'PATIENT ROOM 2'); sign(9.6,1.38,0,'PHARMACY','#3BA55D');
    sign(-8.5,-1.38,Math.PI,'RECEPTION'); sign(-2.1,-1.38,Math.PI,'CONSULTATION'); sign(3.3,-1.38,Math.PI,'IMAGING','#2F6FDE'); sign(9,-1.38,Math.PI,'LIFTS · STAIRS');
    hang(-4.0,0,'← RECEPTION · LIFTS →'); hang(5.0,0,'PHARMACY · IMAGING →');
    const tv=new THREE.Group(); tv.add(boxMesh(1.3,0.75,0.05,M.black,0,0,0)); const sc=new THREE.Mesh(new THREE.PlaneGeometry(1.24,0.69),M.screens[3]); sc.position.z=0.03; tv.add(sc); tv.position.set(-8.5,1.95,-7.88); add(tv);
    const q=new THREE.Mesh(new THREE.PlaneGeometry(0.9,0.4),new THREE.MeshBasicMaterial({map:labelTex('NOW SERVING  A-042','#0D1117','#3CFF8A')})); q.position.set(-9.5,2.0,1.62); add(q);
    const looks=[['#B04A6A','#2A2F38','#8D5A3B'],['#E9B949','#3A3F47','#E0B892'],['#4F79A8','#25303B','#C68A5E'],['#7A8450','#433A33','#8D5A3B'],['#D9643B','#2C2C34','#E0B892'],['#8A9AA8','#3A3F47','#C68A5E'],['#2F8F8A','#2A2F38','#B9825A']];
    let k=0; for (const b of STATIC) if (b.kind==='furn' && b.opt && b.opt.seated) { const l=looks[k++%looks.length]; add(seatedPerson(b.cx,b.cy,b.face,l[0],l[1],l[2])); }
    add(seatedPerson(9.6,3.75,'S','#F2F4F6','#2F8F8A','#B9825A')); add(seatedPerson(-9.2,-5.2,'N','#2F8F8A','#2F8F8A','#8D5A3B')); add(seatedPerson(-3.5,-7.3,'N','#F2F4F6','#3A4654','#C68A5E'));
    const ent=new THREE.Group(); ent.add(boxMesh(2.4,2.3,0.05,M.glassBlue,0,0,0)); ent.add(boxMesh(0.04,2.3,0.07,M.steel,0,0,0)); ent.position.set(-11.88,1.15,4.5); ent.rotation.y=Math.PI/2; add(ent);
    const entS=new THREE.Mesh(new THREE.PlaneGeometry(1.4,0.26),new THREE.MeshBasicMaterial({map:labelTex('MAIN ENTRANCE','#1F6F8B')})); entS.position.set(-11.87,2.45,4.5); entS.rotation.y=Math.PI/2; add(entS);
    for (const [x,y] of [[-4.25,5.6],[0.35,5.6],[1.55,5.6]]) add(boxMesh(0.02,1.7,1.4,mat('#BFD8C8',{roughness:1,transparent:true,opacity:0.92}),x,1.45,-y));
    win(-2.0,7.89,0,1.6); win(4.0,7.89,0,1.6); win(-2.0,-7.89,Math.PI,1.4); win(10.0,7.89,0,1.4);
    art(-11.89,6.0,Math.PI/2,0.9,0.6,61); art(-4.89,-4.0,Math.PI/2,0.7,0.5,66); art(5.89,-4.0,-Math.PI/2,0.7,0.5,71);
  } else {
    sign(-8.5,1.38,0,'WARD BAY 1–3'); sign(-1.5,1.38,0,'NURSES\' STATION','#3BA55D'); sign(3.7,1.38,0,'ROOM A'); sign(9.3,1.38,0,'ROOM B');
    sign(-8.5,-1.38,Math.PI,'DAY ROOM'); sign(-2.1,-1.38,Math.PI,'STAFF ONLY','#C0392B'); sign(3.3,-1.38,Math.PI,'PHYSIOTHERAPY','#2F6FDE'); sign(9,-1.38,Math.PI,'LIFTS · STAIRS');
    hang(-5.5,0,'WARD 1 · FLOOR 1'); hang(5.0,0,'← NURSES · LIFTS →');
    add(seatedPerson(-2.6,3.6,'S','#3E8C95','#3E8C95','#8D5A3B')); add(seatedPerson(-10.9,-4.0,'E','#B04A6A','#2A2F38','#E0B892')); add(seatedPerson(-11.45,4.4,'E','#4F79A8','#25303B','#C68A5E'));
    const wb=new THREE.Group(); wb.add(boxMesh(2.0,0.9,0.03,M.steel,0,0,0)); const s=new THREE.Mesh(new THREE.PlaneGeometry(1.94,0.84),mat('#fff',{map:canvasTex(512,224,(c,W,H)=>{ c.fillStyle='#FAFAF8'; c.fillRect(0,0,W,H); c.fillStyle='#1F6F8B'; c.fillRect(0,0,W,30); c.fillStyle='#fff'; c.font='bold 22px sans-serif'; c.fillText('WARD 1 · BED STATE',12,22); c.fillStyle='#222'; c.font='18px sans-serif'; ['Bay 1 · J. K. · discharge pm','Bay 2 · empty, clean','Bay 3 · R. M. · obs 4-hourly','Room A · S. P. · physio 2pm','Room B · visitors welcome'].forEach((t,i)=>c.fillText(t,14,60+i*32)); })})); s.position.z=0.017; wb.add(s); wb.position.set(-1.5,1.6,-7.88); add(wb);
    for (const [x,y] of [[-9.75,6.15],[-7.25,6.15],[1.55,5.8],[11.2,5.6]]) add(boxMesh(0.02,1.7,1.4,mat('#BFD8C8',{roughness:1,transparent:true,opacity:0.92}),x,1.45,-y));
    win(-8.5,7.89,0,2.0); win(4.0,7.89,0,1.6); win(9.5,7.89,0,1.4); win(-8.5,-7.89,Math.PI,2.0); win(3.5,-7.89,Math.PI,1.6);
    const mat_=new THREE.Mesh(new THREE.PlaneGeometry(1.8,1.0),mat('#2F6FDE',{roughness:1})); mat_.rotation.x=-Math.PI/2; mat_.position.set(1.9,0.012,3.4); add(mat_);
    art(-11.89,4.5,Math.PI/2,0.9,0.6,81); art(-4.11,4.5,-Math.PI/2,0.8,0.5,83); art(-5.11,-5.5,-Math.PI/2,0.9,0.6,85);
  }
  return out;
}
