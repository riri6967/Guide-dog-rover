
/* ================= Furniture builders ================= */
// local frame: front faces +x, depth along x, width along z, y up
const FACE_ROT = {E:0, N:Math.PI/2, W:Math.PI, S:-Math.PI/2, '':0};
let M = null;   // shared materials, made once three.js is up
function mats() {
  if (M) return M;
  M = {
    wood:mat('#7A5234',{roughness:0.55}), darkWood:mat('#4A3222',{roughness:0.6}), lightWood:mat('#C9A27A',{roughness:0.6}),
    white:mat('#EDEDEA',{roughness:0.5}), offwhite:mat('#E4DED2',{roughness:0.7}), metal:mat('#9AA3AD',{roughness:0.35,metalness:0.7}),
    black:mat('#17191C',{roughness:0.5}), plastic:mat('#2A2F36',{roughness:0.6}), fabric:mat('#3E4A57',{roughness:0.95}),
    sheet:mat('#F1EEE6',{roughness:0.9}), pillow:mat('#FAFAF7',{roughness:1}), steel:mat('#C5CAD0',{roughness:0.25,metalness:0.8}),
    glassBlue:mat('#9CC7E8',{roughness:0.1,metalness:0.1,transparent:true,opacity:0.45}), screenOff:mat('#0D1117',{roughness:0.25,metalness:0.3}),
    lampShade:mat('#F3E7CF',{emissive:'#7A6230',emissiveIntensity:0.7}), leaf:mat('#3E7A43',{roughness:0.85}), pot:mat('#B0643C',{roughness:0.8}),
    rackBody:mat('#1E232A',{roughness:0.5,metalness:0.4}), ledG:new THREE.MeshBasicMaterial({color:'#3CFF8A'}), ledB:new THREE.MeshBasicMaterial({color:'#4DA3FF'}),
    ledA:new THREE.MeshBasicMaterial({color:'#FFB13C'}), locker:mat('#5B6F86',{roughness:0.5,metalness:0.3}), red:mat('#C0392B',{roughness:0.4}),
  };
  const bookCols=['#8C2F39','#2F5D62','#E9B949','#3D4F7A','#7A8450','#B5653C','#4B3B5C','#D9D2C5'];
  M.books=bookCols.map(c=>mat(c,{roughness:0.8}));
  M.screens=[0,1,2,3].map(k=>mat('#ffffff',{map:canvasTex(256,160,(c,w,h)=>screenArt(c,w,h,k)),emissive:'#ffffff',emissiveMap:canvasTex(256,160,(c,w,h)=>screenArt(c,w,h,k)),emissiveIntensity:0.85,roughness:0.3}));
  return M;
}
function screenArt(c,w,h,k){
  c.fillStyle=['#0E1726','#101418','#F4F6F8','#14202E'][k]; c.fillRect(0,0,w,h);
  const r=rng(31+k*7);
  if (k===2) { c.fillStyle='#2F6FDE'; c.fillRect(0,0,w,18); for (let i=0;i<9;i++){ c.fillStyle='#C9D3DE'; c.fillRect(14,28+i*13,60+r()*150,6); } return; }
  if (k===3) { c.strokeStyle='#4CC38A'; c.lineWidth=2; c.beginPath(); for (let x=0;x<w;x+=4){ const y=h/2+Math.sin(x/12)*28*Math.sin(x/70); x?c.lineTo(x,y):c.moveTo(x,y);} c.stroke(); c.strokeStyle='#F2B33D'; c.beginPath(); for (let x=0;x<w;x+=4){ const y=h/2+((x/20|0)%2?24:-24); x?c.lineTo(x,y):c.moveTo(x,y);} c.stroke(); return; }
  const cols=['#5CC6E6','#F2B33D','#AE92FF','#4CC38A','#E8EEF4','#FF6B57'];
  for (let i=0;i<12;i++){ const ind=(r()*5|0)*10; c.fillStyle=cols[(r()*cols.length)|0]; c.fillRect(10+ind,10+i*12,30+r()*120,5); }
}
function grp(f){ const g=new THREE.Group(); g.position.set(f.x,0,-f.y); g.rotation.y=FACE_ROT[f.face]||0; return g; }
function dims(f){ return (f.face==='N'||f.face==='S') ? {d:f.sy,w:f.sx} : {d:f.sx,w:f.sy}; }
function addLegs(g,d,w,h,m,inset,r){ for (const [a,b] of [[-1,-1],[1,-1],[-1,1],[1,1]]) g.add(boxMesh(r||0.05,h,r||0.05,m,a*(d/2-inset),h/2,b*(w/2-inset))); }
function lampOn(g,x,y,z,s){ s=s||1; const b=cyl(0.06*s,0.08*s,0.03,M.metal); b.position.set(x,y+0.015,z); g.add(b); const p=cyl(0.01,0.01,0.3*s,M.metal); p.position.set(x,y+0.17*s,z); g.add(p); const sh=cyl(0.07*s,0.12*s,0.14*s,M.lampShade,20); sh.position.set(x,y+0.36*s,z); g.add(sh); }
function laptop(g,x,y,z,ry,k){ const l=new THREE.Group(); l.position.set(x,y,z); l.rotation.y=ry; l.add(boxMesh(0.24,0.015,0.34,M.steel,0,0.0075,0)); const sc=new THREE.Group(); sc.position.set(-0.12,0.015,0); sc.rotation.z=-1.25; sc.add(boxMesh(0.008,0.23,0.33,M.steel,0,0.115,0)); const disp=new THREE.Mesh(new THREE.PlaneGeometry(0.3,0.2),M.screens[k%4]); disp.position.set(0.006,0.115,0); disp.rotation.y=Math.PI/2; sc.add(disp); l.add(sc); g.add(l); }
function monitor(g,x,y,z,ry,k){ const m=new THREE.Group(); m.position.set(x,y,z); m.rotation.y=ry; m.add(boxMesh(0.16,0.015,0.22,M.plastic,0,0.008,0)); m.add(boxMesh(0.03,0.2,0.04,M.plastic,-0.03,0.11,0)); m.add(boxMesh(0.025,0.34,0.58,M.black,-0.02,0.36,0)); const s=new THREE.Mesh(new THREE.PlaneGeometry(0.55,0.31),M.screens[k%4]); s.position.set(-0.006,0.36,0); s.rotation.y=Math.PI/2; m.add(s); g.add(m); }
function keyboard(g,x,y,z,ry){ const k=boxMesh(0.14,0.018,0.42,M.plastic,x,y+0.009,z); k.rotation.y=ry; g.add(k); const ms=boxMesh(0.1,0.025,0.06,M.plastic,x,y+0.012,z+0.3); g.add(ms); }
function books(g,x0,y,z0,len,depth,h,seed){ const r=rng(seed); let z=z0; while (z<z0+len-0.03) { const bw=0.025+r()*0.03, bh=h*(0.7+r()*0.3); g.add(boxMesh(depth*(0.8+r()*0.2),bh,bw,M.books[(r()*M.books.length)|0],x0,y+bh/2,z+bw/2)); z+=bw+0.003; } }

const BUILD = {
  bed(f){ const g=grp(f), {d,w}=dims(f);
    g.add(boxMesh(d,0.3,w,M.darkWood,0,0.15,0)); g.add(boxMesh(d-0.1,0.22,w-0.08,M.sheet,0,0.41,0));
    g.add(boxMesh(d*0.66,0.08,w-0.02,mat(f.opt.duvet||'#4F79A8',{roughness:0.95}),d*0.15,0.55,0));
    g.add(boxMesh(0.3,0.06,w-0.02,mat(f.opt.duvet||'#4F79A8',{roughness:0.95}),d*0.15-d*0.33+0.02,0.6,0));
    for (const z of [-w*0.24,w*0.24]) g.add(boxMesh(0.42,0.12,w*0.4,M.pillow,-(d/2-0.33),0.58,z));
    g.add(boxMesh(0.1,1.05,w+0.06,M.darkWood,-(d/2+0.02),0.52,0)); g.add(boxMesh(0.08,0.45,w+0.04,M.darkWood,d/2+0.01,0.22,0)); return g; },
  table(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,0.045,w,M.wood,0,h-0.022,0)); addLegs(g,d,w,h-0.045,M.darkWood,0.07,0.055);
    const it=f.opt.items, top=h;
    if (it==='study') { lampOn(g,-d*0.3,top,w*0.3); g.add(boxMesh(0.24,0.04,0.17,M.books[1],0.05,top+0.02,-w*0.2)); const mug=cyl(0.04,0.035,0.1,mat('#E9B949'),14); mug.position.set(d*0.25,top+0.05,w*0.15); g.add(mug); laptop(g,0.05,top,0.1,Math.PI,1); }
    if (it==='laptop') { laptop(g,0,top,0,Math.PI,0); g.add(boxMesh(0.22,0.01,0.3,M.offwhite,d*0.25,top+0.005,w*0.25)); }
    if (it==='dining') { for (const [a,b] of [[-0.25,-0.5],[0.25,-0.5],[-0.25,0.5],[0.25,0.5],[0,-0.82],[0,0.82]]) { if (Math.abs(b)*w/2>w/2-0.1&&Math.abs(b)<0.8) continue; const pm=boxMesh(0.3,0.005,0.4,mat('#C7B299',{roughness:1}),a*d*0.9,top+0.003,b*w*0.35); g.add(pm); }
      const bowl=new THREE.Mesh(new THREE.SphereGeometry(0.13,16,8,0,TAU,Math.PI/2,Math.PI/2),mat('#E8E2D6')); bowl.position.set(0,top+0.13,0); g.add(bowl);
      for (const [a,b,c] of [[0.04,0.03,'#D9643B'],[-0.05,0.02,'#E9B949'],[0.0,-0.05,'#7A9A3C']]) { const fr_=new THREE.Mesh(new THREE.SphereGeometry(0.045,10,8),mat(c)); fr_.position.set(a,top+0.06,b); g.add(fr_); } }
    if (it==='pc') { monitor(g,-d*0.25,top,0,0,0); keyboard(g,d*0.12,top,0,0); g.add(boxMesh(0.42,0.42,0.18,M.black,0,0.21,w/2-0.2)); }
    if (it==='meeting') { const r=rng(5); for (let i=0;i<3;i++) laptop(g,(i-1)*d*0.3,top,(i%2?1:-1)*w*0.22,i%2?Math.PI/2:-Math.PI/2,i); const sp=cyl(0.09,0.1,0.03,M.black,20); sp.position.set(0,top+0.015,0); g.add(sp); g.add(boxMesh(0.2,0.01,0.28,M.offwhite,d*0.35,top+0.005,-w*0.1)); }
    if (it==='arm') { const b=cyl(0.1,0.12,0.08,M.plastic,20); b.position.set(0,top+0.04,0); g.add(b); const s1=cyl(0.04,0.04,0.35,mat('#E8EEF4',{metalness:0.3,roughness:0.4})); s1.position.set(0,top+0.25,0); g.add(s1); const s2=cyl(0.035,0.035,0.32,mat('#E8EEF4',{metalness:0.3,roughness:0.4})); s2.position.set(0.12,top+0.5,0); s2.rotation.z=-1.0; g.add(s2); g.add(boxMesh(0.06,0.06,0.1,M.red,0.26,top+0.58,0)); }
    return g; },
  coffee(f){ const g=BUILD.table(Object.assign({},f,{opt:{}})); const {d,w}=dims(f); books(g,0,f.h,-w*0.35,0.25,0.18,0.05,8); const p=cyl(0.06,0.05,0.1,M.pot); p.position.set(d*0.25,f.h+0.05,w*0.2); g.add(p); const l=new THREE.Mesh(new THREE.SphereGeometry(0.09,10,8),M.leaf); l.position.set(d*0.25,f.h+0.16,w*0.2); g.add(l); return g; },
  desk(f){ return BUILD.table(f); },
  sidetable(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.lightWood,0,f.h/2,0)); lampOn(g,0,f.h,0,0.8); return g; },
  chair(f){ const g=grp(f), {d,w}=dims(f), s=Math.min(d,w)*0.75, m=f.opt.color?mat(f.opt.color):M.fabric;
    g.add(boxMesh(s,0.06,s,m,0,0.46,0)); addLegs(g,s,s,0.44,M.darkWood,0.03,0.04); g.add(boxMesh(0.06,0.5,s,M.darkWood,-s/2+0.03,0.72,0)); g.add(boxMesh(0.04,0.35,s-0.04,m,-s/2+0.07,0.74,0)); return g; },
  ochair(f){ const g=grp(f); if (f.opt.pulled) g.rotation.y += 0.5;
    for (let i=0;i<5;i++) { const a=i*TAU/5, l=boxMesh(0.26,0.03,0.04,M.plastic,Math.cos(a)*0.13,0.06,Math.sin(a)*0.13); l.rotation.y=-a; g.add(l); const wh=new THREE.Mesh(new THREE.SphereGeometry(0.025,8,6),M.black); wh.position.set(Math.cos(a)*0.26,0.025,Math.sin(a)*0.26); g.add(wh); }
    const col=cyl(0.025,0.025,0.36,M.steel); col.position.y=0.25; g.add(col);
    g.add(boxMesh(0.46,0.08,0.46,M.fabric,0,0.47,0)); g.add(boxMesh(0.06,0.52,0.42,M.fabric,-0.22,0.8,0));
    for (const z of [-0.24,0.24]) g.add(boxMesh(0.28,0.04,0.04,M.plastic,0,0.66,z)); return g; },
  armchair(f){ const g=grp(f), {d,w}=dims(f), m=mat(f.opt.color||'#7A5C8E',{roughness:0.95});
    g.add(boxMesh(d,0.38,w,m,0,0.19,0)); g.add(boxMesh(d-0.2,0.12,w-0.24,m,0.06,0.44,0)); g.add(boxMesh(0.2,0.5,w,m,-d/2+0.1,0.62,0));
    for (const z of [-1,1]) g.add(boxMesh(d,0.22,0.13,m,0,0.49,z*(w/2-0.065))); return g; },
  sofa(f){ const g=grp(f), {d,w}=dims(f), m=mat(f.opt.color||'#56657A',{roughness:0.95}), m2=mat('#6B7B91',{roughness:0.95});
    g.add(boxMesh(d,0.36,w,m,0,0.18,0)); const n=3, cw=(w-0.3)/n;
    for (let i=0;i<n;i++) { const z=-w/2+0.15+cw*(i+0.5); g.add(boxMesh(d-0.25,0.13,cw-0.02,m2,0.08,0.43,z)); g.add(boxMesh(0.2,0.42,cw-0.02,m2,-d/2+0.18,0.66,z)); }
    g.add(boxMesh(0.16,0.5,w,m,-d/2+0.08,0.6,0)); for (const z of [-1,1]) g.add(boxMesh(d,0.25,0.15,m,0,0.5,z*(w/2-0.075)));
    for (const z of [-w*0.32,w*0.3]) { const c=boxMesh(0.12,0.3,0.32,mat(z<0?'#E9B949':'#D9643B',{roughness:1}),-d/2+0.32,0.62,z); c.rotation.x=0.15; g.add(c); } return g; },
  wardrobe(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,h,w,M.offwhite,0,h/2,0)); g.add(boxMesh(d+0.01,0.08,w+0.01,M.darkWood,0,0.04,0));
    g.add(boxMesh(0.005,h-0.15,0.008,M.darkWood,d/2+0.003,h/2+0.04,0));
    for (const z of [-0.05,0.05]) g.add(boxMesh(0.03,0.25,0.02,M.steel,d/2+0.015,h*0.55,z)); return g; },
  nightstand(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.lightWood,0,f.h/2,0)); g.add(boxMesh(0.005,0.01,w-0.04,M.darkWood,d/2+0.003,f.h*0.55,0)); g.add(boxMesh(0.02,0.02,0.1,M.steel,d/2+0.01,f.h*0.75,0)); lampOn(g,-0.02,f.h,0,0.75); return g; },
  dresser(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.wood,0,f.h/2,0));
    for (let i=1;i<3;i++) g.add(boxMesh(0.005,0.01,w-0.04,M.darkWood,d/2+0.003,f.h*i/3,0));
    for (let i=0;i<3;i++) g.add(boxMesh(0.02,0.02,0.14,M.steel,d/2+0.01,f.h*(i+0.5)/3,0));
    const v=cyl(0.05,0.07,0.22,mat('#2F5D62',{roughness:0.3}),14); v.position.set(0,f.h+0.11,w*0.3); g.add(v); books(g,0,f.h,-w*0.4,0.25,d*0.6,0.2,3); return g; },
  bookshelf(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(0.02,h,w,M.wood,-d/2+0.01,h/2,0)); for (const z of [-1,1]) g.add(boxMesh(d,h,0.03,M.wood,0,h/2,z*(w/2-0.015)));
    const n=5; for (let i=0;i<=n;i++) { const y=i*h/n; g.add(boxMesh(d,0.025,w,M.wood,0,Math.min(y,h-0.0125)+0.0125,0)); if (i<n) books(g,0.01,y+0.025,-w/2+0.04,w-0.08,d*0.85,h/n*0.8,f.id*7+i); } return g; },
  tvunit(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.darkWood,0,f.h/2,0)); for (const z of [-w/4,w/4]) g.add(boxMesh(0.005,f.h-0.1,w/2-0.04,M.plastic,d/2+0.003,f.h/2,z));
    g.add(boxMesh(0.12,0.04,0.3,M.black,-0.05,f.h+0.02,0)); g.add(boxMesh(0.04,0.1,0.06,M.black,-0.05,f.h+0.07,0)); g.add(boxMesh(0.05,0.66,1.16,M.black,-0.05,f.h+0.45,0));
    const s=new THREE.Mesh(new THREE.PlaneGeometry(1.1,0.6),M.screens[3]); s.position.set(-0.02,f.h+0.45,0); s.rotation.y=Math.PI/2; g.add(s); return g; },
  lamp(f){ const g=grp(f); const b=cyl(0.15,0.15,0.03,M.metal,20); b.position.y=0.015; g.add(b); const p=cyl(0.015,0.015,1.4,M.metal); p.position.y=0.72; g.add(p); const sh=cyl(0.14,0.2,0.26,M.lampShade,20); sh.position.y=1.48; g.add(sh); return g; },
  cabinet(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,f.label==='Power unit'?M.rackBody:M.offwhite,0,f.h/2,0));
    const n=Math.max(1,Math.round(w/0.5)); for (let i=0;i<n;i++) { const z=-w/2+(i+0.5)*w/n; g.add(boxMesh(0.005,f.h-0.08,w/n-0.02,M.lightWood,d/2+0.003,f.h/2,z)); g.add(boxMesh(0.02,0.08,0.02,M.steel,d/2+0.012,f.h*0.75,z+(i%2?-1:1)*(w/n/2-0.06))); }
    if (f.label==='Power unit') g.add(boxMesh(0.005,0.06,0.25,M.ledG,d/2+0.006,f.h-0.12,0)); return g; },
  counter(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,0.86,w,M.offwhite,0,0.43,0)); g.add(boxMesh(d+0.03,0.04,w+0.02,mat('#3B3F45',{roughness:0.3}),0.015,0.88,0));
    const n=Math.round(w/0.6); for (let i=0;i<n;i++) { const z=-w/2+(i+0.5)*w/n; g.add(boxMesh(0.005,0.7,w/n-0.02,mat('#D4CDBF'),d/2+0.003,0.45,z)); g.add(boxMesh(0.02,0.02,0.12,M.steel,d/2+0.012,0.74,z)); }
    if (f.opt.sink!=null) { g.add(boxMesh(0.4,0.02,0.55,M.steel,0,0.905,f.opt.sink)); const tap=cyl(0.012,0.012,0.25,M.steel); tap.position.set(-d/2+0.1,1.02,f.opt.sink); g.add(tap); }
    if (f.opt.hob!=null) { g.add(boxMesh(0.5,0.012,0.6,M.black,0,0.905,f.opt.hob)); for (const [a,b] of [[-0.1,-0.13],[0.1,-0.13],[-0.1,0.13],[0.1,0.13]]) { const r_=new THREE.Mesh(new THREE.RingGeometry(0.05,0.065,20),mat('#5A5F66')); r_.rotation.x=-Math.PI/2; r_.position.set(a,0.913,f.opt.hob+b); g.add(r_); }
      g.add(boxMesh(0.45,0.25,0.65,M.steel,-d/2+0.25,1.85,f.opt.hob)); }
    if (f.opt.coffee!=null) { g.add(boxMesh(0.3,0.38,0.25,M.black,-0.05,1.09,f.opt.coffee)); const mw=boxMesh(0.36,0.28,0.5,M.steel,-0.05,1.04,f.opt.coffee-0.8); g.add(mw); }
    g.add(boxMesh(0.35,0.7,w,M.offwhite,-d/2+0.175,1.85,0)); for (let i=0;i<n;i++) g.add(boxMesh(0.005,0.66,w/n-0.02,mat('#D4CDBF'),-d/2+0.353,1.85,-w/2+(i+0.5)*w/n));
    return g; },
  fridge(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,mat('#D9DCDF',{roughness:0.3,metalness:0.4}),0,f.h/2,0)); g.add(boxMesh(0.005,0.01,w,M.plastic,d/2+0.003,f.h*0.62,0));
    for (const y of [f.h*0.8,f.h*0.35]) g.add(boxMesh(0.03,0.3,0.03,M.steel,d/2+0.02,y,w/2-0.08)); return g; },
  washer(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.white,0,f.h/2,0)); const r_=new THREE.Mesh(new THREE.TorusGeometry(0.17,0.03,10,24),mat('#9AA3AD',{metalness:0.6,roughness:0.3})); r_.rotation.y=Math.PI/2; r_.position.set(d/2+0.01,f.h*0.45,0); g.add(r_);
    const gl=new THREE.Mesh(new THREE.CircleGeometry(0.15,24),M.glassBlue); gl.rotation.y=Math.PI/2; gl.position.set(d/2+0.012,f.h*0.45,0); g.add(gl); g.add(boxMesh(0.005,0.06,w-0.1,M.plastic,d/2+0.003,f.h-0.06,0)); return g; },
  basket(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,mat('#B89B6E',{roughness:1}),0,f.h/2,0)); g.add(boxMesh(d-0.06,0.08,w-0.06,mat('#E8E2D6'),0,f.h,0)); return g; },
  plant(f){ const g=grp(f), r=Math.min(f.sx,f.sy)/2; const pot=cyl(r*0.8,r*0.6,0.4,M.pot,18); pot.position.y=0.2; g.add(pot); const rr=rng(f.id||3);
    for (let i=0;i<9;i++) { const s=new THREE.Mesh(new THREE.SphereGeometry(r*(0.5+rr()*0.35),10,8),M.leaf); s.position.set((rr()-0.5)*r*1.3,0.55+rr()*(f.h-0.6),(rr()-0.5)*r*1.3); g.add(s); } return g; },
  bench(f, scene){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,0.04,w,mat('#E4E7EA',{roughness:0.5}),0,h-0.02,0));
    for (const z of [-w/2+0.05, w/2-0.05, ...(w>3?[0]:[])]) { g.add(boxMesh(d-0.1,0.05,0.05,M.plastic,0,0.08,z)); g.add(boxMesh(0.05,h-0.04,0.05,M.plastic,d/2-0.08,(h-0.04)/2,z)); g.add(boxMesh(0.05,h-0.04,0.05,M.plastic,-d/2+0.08,(h-0.04)/2,z)); }
    if (f.opt.island) g.add(boxMesh(0.03,0.35,w-0.1,mat('#8A9AA8',{roughness:0.9}),0,h+0.175,0));
    if (f.opt.wall) g.add(boxMesh(0.03,0.6,w,mat('#8A9AA8',{roughness:0.9}),-d/2+0.015,h+0.3,0));
    (f.opt.seats||[]).forEach(([sx,sy,face],i)=>{   // seat items in world coordinates
      const dir={N:[0,-1],S:[0,1],E:[-1,0],W:[1,0]}[face], ry=Math.atan2(-dir[1],-dir[0]);   // local +x (screens, keyboards) faces the person
      const k=(f.id*3+i);
      if (f.opt.electronics) {
        const it=new THREE.Group(); it.position.set(sx+dir[0]*0.15,h,-(sy+dir[1]*0.15)); it.rotation.y=ry;
        it.add(boxMesh(0.3,0.18,0.36,mat('#3A4654'),0,0.09,0)); const sc=new THREE.Mesh(new THREE.PlaneGeometry(0.2,0.13),M.screens[3]); sc.position.set(0.151,0.1,-0.05); sc.rotation.y=Math.PI/2; it.add(sc);
        it.add(boxMesh(0.18,0.01,0.26,mat('#F2F2EE'),0.25,0.005,0.15)); it.add(boxMesh(0.12,0.06,0.1,M.red,0.2,0.03,-0.25)); scene.add(it);
      } else if (k%3===1) { const it=new THREE.Group(); it.position.set(sx-dir[0]*0.02,h,-(sy-dir[1]*0.02)); laptop(it,0,0,0,ry,k); scene.add(it); }
      else { const it=new THREE.Group(); it.position.set(sx+dir[0]*0.22,h,-(sy+dir[1]*0.22)); monitor(it,0,0,0,ry,k); scene.add(it);
        const kb=new THREE.Group(); kb.position.set(sx-dir[0]*0.12,h,-(sy-dir[1]*0.12)); keyboard(kb,0,0,0,ry); scene.add(kb); }
      const tw=new THREE.Group(); tw.position.set(clamp(sx+0.35,f.x0+0.15,f.x1-0.15),0,-clamp(sy+dir[1]*-0.1,f.y0+0.15,f.y1-0.15)); tw.add(boxMesh(0.45,0.42,0.2,M.black,0,0.21,0)); tw.add(boxMesh(0.005,0.01,0.05,M.ledB,0.226,0.36,0)); scene.add(tw);
    });
    return g; },
  printer3d(f){ const g=BUILD.table(Object.assign({},f,{h:0.75,opt:{}})); g.add(boxMesh(0.5,0.5,0.5,M.glassBlue,0,1.0,0)); g.add(boxMesh(0.52,0.04,0.52,M.plastic,0,0.77,0)); g.add(boxMesh(0.52,0.04,0.52,M.plastic,0,1.25,0));
    const p=cyl(0.06,0.08,0.12,mat('#F2B33D'),6); p.position.set(0,0.85,0); g.add(p); return g; },
  rack(f){ const g=grp(f), {d,w}=dims(f), h=f.h; g.add(boxMesh(d,h,w,M.rackBody,0,h/2,0)); const r=rng(f.id);
    for (let i=0;i<14;i++) { const y=0.15+i*0.125; g.add(boxMesh(0.005,0.09,w-0.1,mat('#2A313A',{metalness:0.5}),d/2+0.003,y,0)); for (let j=0;j<3;j++) g.add(boxMesh(0.006,0.012,0.012,[M.ledG,M.ledB,M.ledA][(r()*3)|0],d/2+0.006,y+0.02,-w/2+0.12+j*0.04)); }
    return g; },
  printer(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h*0.8,w,M.white,0,f.h*0.4,0)); g.add(boxMesh(d*0.7,0.12,w*0.8,mat('#D5D8DC'),0,f.h*0.86,0)); g.add(boxMesh(0.08,0.06,0.18,M.screens[2],d/2-0.02,f.h*0.82,w*0.3)); g.add(boxMesh(0.3,0.02,w*0.5,M.offwhite,d/2+0.1,f.h*0.55,0)); return g; },
  lockers(f){ const g=grp(f), {d,w}=dims(f); g.add(boxMesh(d,f.h,w,M.locker,0,f.h/2,0)); const n=Math.round(w/0.4);
    for (let i=0;i<n;i++) { const z=-w/2+(i+0.5)*w/n; g.add(boxMesh(0.005,f.h-0.08,0.006,M.plastic,d/2+0.003,f.h/2,z+w/n/2-0.003)); for (let v=0;v<3;v++) g.add(boxMesh(0.005,0.008,0.18,M.plastic,d/2+0.003,f.h-0.2-v*0.03,z)); g.add(boxMesh(0.02,0.08,0.02,M.steel,d/2+0.012,f.h*0.55,z+0.12)); } return g; },
  cooler(f){ const g=grp(f); g.add(boxMesh(0.32,0.95,0.32,M.white,0,0.475,0)); const b=cyl(0.13,0.13,0.4,mat('#7FB8E6',{transparent:true,opacity:0.6,roughness:0.1}),18); b.position.y=1.15; g.add(b); return g; },
  bins(f){ const g=grp(f), {d,w}=dims(f); ['#2F6FDE','#4CC38A','#E9B949'].forEach((c,i)=>g.add(boxMesh(d,f.h,w/3-0.02,mat(c,{roughness:0.6}),0,f.h/2,-w/2+(i+0.5)*w/3))); return g; },
};

/* ================= Decor (visual only, nothing the rover can hit) ================= */
function decor(scene, id) {
  if (id==='hospital') return decorHospital(scene);
  const art=(x,y,ry,w,h,seed,yy)=>{ const g=new THREE.Group(); g.add(boxMesh(w+0.06,h+0.06,0.03,mat('#2B2622'),0,0,0)); const p=new THREE.Mesh(new THREE.PlaneGeometry(w,h), mat('#fff',{map:canvasTex(256,192,(c,W,H)=>TEX.art(c,W,H,seed)),roughness:0.9})); p.position.z=0.017; g.add(p); g.position.set(x,yy||1.55,-y); g.rotation.y=ry; scene.add(g); return g; };
  const win=(x,y,ry,w)=>{ const g=new THREE.Group(); g.add(boxMesh(w+0.1,1.3,0.06,M.white,0,0,0)); const gl=new THREE.Mesh(new THREE.PlaneGeometry(w,1.2),mat('#BFE0F5',{emissive:'#9CC7E8',emissiveIntensity:0.6})); gl.position.z=0.032; g.add(gl); g.add(boxMesh(0.04,1.2,0.07,M.white,0,0,0.01));
    for (const s of [-1,1]) g.add(boxMesh(0.35,1.6,0.04,mat('#C9B79C',{roughness:1}),s*(w/2+0.12),-0.05,0.09)); g.position.set(x,1.55,-y); g.rotation.y=ry; scene.add(g); return g; };
  const flat=(x,y,w,h,tex,ry)=>{ const r=new THREE.Mesh(new THREE.PlaneGeometry(w,h), mat('#fff',{map:canvasTex(512,256,TEX[tex]),roughness:1})); r.rotation.x=-Math.PI/2; r.rotation.z=ry||0; r.position.set(x,0.008,-y); r.receiveShadow=true; scene.add(r); };
  const out=[];
  if (id==='house') {
    flat(-5.6,3.4,2.4,1.5,'rug'); flat(5.3,-3.2,2.2,1.4,'rug2'); flat(5.0,5.4,3.2,2.2,'rug'); flat(-5,-4.4,2.8,2.0,'rug2');
    out.push(win(-6.5,7.79,0,1.6), win(3.5,-7.79,Math.PI,1.4), win(8.5,7.89,0,1.8), win(2.5,7.89,0,1.4), win(-3.2,-7.89,Math.PI,1.6), win(-11.89,4.5,Math.PI/2,1.2), win(11.89,-5,-Math.PI/2,1.2));
    out.push(art(-8.89,4.4,Math.PI/2,0.7,0.5,41), art(-6.5,-1.39,Math.PI,0.8,0.55,8), art(-2.2,-1.39,Math.PI,0.6,0.8,17), art(4,1.39,0,1.2,0.6,29), art(-0.89,4.0,Math.PI/2,0.9,0.6,13), art(8.89,-5.5,-Math.PI/2,0.9,0.6,21), art(11.89,5.5,-Math.PI/2,0.8,0.6,5));
    const clock=new THREE.Group(); const face=new THREE.Mesh(new THREE.CircleGeometry(0.17,32),mat('#F5F2EA')); clock.add(face); const rim=new THREE.Mesh(new THREE.TorusGeometry(0.17,0.012,8,32),M.plastic); clock.add(rim); clock.add(boxMesh(0.012,0.12,0.005,M.black,0,0.05,0.01)); clock.position.set(-9.5,1.9,-1.39); clock.rotation.y=Math.PI; scene.add(clock); out.push(clock);
    const mirror=new THREE.Mesh(new THREE.PlaneGeometry(0.6,1.0),mat('#C9DDE8',{metalness:0.9,roughness:0.05})); mirror.position.set(-11.89,1.4,-0.4); mirror.rotation.y=Math.PI/2; scene.add(mirror); out.push(mirror);
  } else {
    // lab: floor tape along the aisles, whiteboards, signs, a TV in the meeting room
    const tape=(x0,y0,x1,y1)=>{ const L=hyp(x1-x0,y1-y0), m_=new THREE.Mesh(new THREE.PlaneGeometry(L,0.05),mat('#E9C23B',{roughness:0.8})); m_.rotation.x=-Math.PI/2; m_.rotation.z=Math.atan2(y1-y0,x1-x0); m_.position.set((x0+x1)/2,0.009,-(y0+y1)/2); scene.add(m_); };
    tape(-9.8,-1.45,3.8,-1.45); tape(-9.8,2.25,1.0,2.25); tape(-9.8,4.35,1.0,4.35);
    const wb=(x,y,ry,w)=>{ const g=new THREE.Group(); g.add(boxMesh(w+0.06,1.06,0.03,M.steel,0,0,0)); const s=new THREE.Mesh(new THREE.PlaneGeometry(w,1.0),mat('#fff',{map:canvasTex(512,256,(c,W,H)=>{ c.fillStyle='#FAFAF8'; c.fillRect(0,0,W,H); c.strokeStyle='#2F6FDE'; c.lineWidth=3; c.beginPath(); c.moveTo(40,180); c.bezierCurveTo(120,40,220,220,320,90); c.stroke(); c.strokeStyle='#C0392B'; c.strokeRect(330,40,140,90); c.fillStyle='#222'; c.font='22px sans-serif'; c.fillText('SLAM → nav2 → /cmd_vel',40,40); c.fillText('handle haptics v2',340,170); }),roughness:0.4})); s.position.z=0.017; g.add(s); g.position.set(x,1.5,-y); g.rotation.y=ry; scene.add(g); return g; };
    out.push(wb(-9.89,2.0,Math.PI/2,2.0), wb(9.89,-0.5,-Math.PI/2,1.8));
    const tv=new THREE.Group(); tv.add(boxMesh(1.4,0.8,0.05,M.black,0,0,0)); const sc=new THREE.Mesh(new THREE.PlaneGeometry(1.34,0.74),M.screens[0]); sc.position.z=0.03; tv.add(sc); tv.position.set(8.5,1.6,-1.88); scene.add(tv); out.push(tv);
    const sign=(x,y,ry,txt,col)=>{ const s=new THREE.Mesh(new THREE.PlaneGeometry(0.5,0.18),new THREE.MeshBasicMaterial({map:canvasTex(256,96,(c,W,H)=>{ c.fillStyle=col; c.fillRect(0,0,W,H); c.fillStyle='#fff'; c.font='bold 40px sans-serif'; c.textAlign='center'; c.textBaseline='middle'; c.fillText(txt,W/2,H/2); })})); s.position.set(x,2.25,-y); s.rotation.y=ry; scene.add(s); out.push(s); };
    sign(-5.75,-3.12,0,'LAB A','#1E8A4C'); sign(0.75,-3.12,0,'LAB B','#1E8A4C'); sign(6.75,-3.12,0,'MEETING','#2F6FDE'); sign(3.89,4.65,-Math.PI/2,'SERVERS','#C0392B'); sign(-9.89,-4,Math.PI/2,'EXIT','#1E8A4C');
    const ext=cyl(0.07,0.07,0.45,M.red,14); ext.position.set(-2,0.5,3.11); scene.add(ext); out.push(ext);
    out.push(art(-4,-4.89,Math.PI,1.2,0.8,52,1.5), art(3,-3.11,0,0.9,0.6,9), art(-8.5,6.89,0,0.8,0.5,77,1.9));
  }
  return out;
}
