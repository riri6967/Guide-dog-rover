
/* ================= Doors, lift and stairs ================= */
const closedRect = d => d.vert ? mk(d.x,d.y,0.08,d.w,{kind:'door',door:d}) : mk(d.x,d.y,d.w,0.08,{kind:'door',door:d});
// a hinged leaf standing open at 90 degrees, along the jamb on its swing side
function leafRect(d) {
  if (d.vert) { const hy=d.y+d.hinge*(d.w/2-0.04); return mk(d.x+d.swing*d.w/2, hy, d.w, 0.08, {kind:'door',door:d}); }
  const hx=d.x+d.hinge*(d.w/2-0.04); return mk(hx, d.y+d.swing*d.w/2, 0.08, d.w, {kind:'door',door:d});
}
function initDoors() {
  S.doors = DOOR_DYN.map(d=>Object.assign({}, d, {f:0, openUntil:-9, openAt:null, opened:false, announced:false, liftOpen:false}));
  S.doorRects = S.doors.map(closedRect);
}
function updateDoors(dt) {
  if (!S.doors.length) { S.doorRects=[]; return; }
  const people=[[S.x,S.y],[S.px,S.py],...S.walkers.map(w=>[w.x,w.y])];
  for (const d of S.doors) {
    const near=r=>people.some(([x,y])=>hyp(x-d.x,y-d.y)<r);
    if (d.type==='auto' && near(1.9)) d.openUntil=Math.max(d.openUntil,S.t+2.0);           // sensor
    if (d.type==='hinged' && d.f>0.5 && near(1.5)) d.openUntil=Math.max(d.openUntil,S.t+2.5); // held while people pass
    let want = d.type==='lift' ? (d.liftOpen?1:0) : (S.t<d.openUntil?1:0);
    if (near(0.75) && d.f>0.05) want=1;    // never close on someone in the doorway
    const rate = d.type==='hinged'?0.85 : d.type==='lift'?0.9 : 1.4;
    d.f = clamp(d.f + Math.sign(want-d.f)*Math.min(Math.abs(want-d.f), rate*dt), 0, 1);
    // ready for the next visit once it has been opened and has swung shut again
    if (d.f<0.02 && d.type!=='lift' && ((d.opened && S.t>d.openUntil) || (d.type==='auto' && S.t>d.openUntil+3))) { d.openAt=null; d.opened=false; d.announced=false; }
    if (d.type==='lift' && d.liftOpen && d.closeAt && S.t>d.closeAt) { d.liftOpen=false; d.closeAt=0; }
  }
  const R=[]; for (const d of S.doors) { if (d.f<0.9) R.push(closedRect(d)); if (d.type==='hinged' && d.f>0.1) R.push(leafRect(d)); }
  S.doorRects=R;
}
// first door the route goes through in the next ~3 m
function doorAhead() {
  if (!S.path || !S.doors.length) return null;
  const pts=S.path;
  for (let k=S.pathIdx; k<Math.min(pts.length-1, S.pathIdx+32); k++) {
    const a=pts[k], b=pts[k+1];
    for (const d of S.doors) {
      const pa = d.vert ? a[0]-d.x : a[1]-d.y, pb = d.vert ? b[0]-d.x : b[1]-d.y;
      if (pa*pb>0 || (pa===0&&pb===0)) continue;
      const t=pa/((pa-pb)||1e-9), cx=a[0]+(b[0]-a[0])*t, cy=a[1]+(b[1]-a[1])*t, along = d.vert ? cy-d.y : cx-d.x;
      if (Math.abs(along) > d.w/2+0.05) continue;
      return {door:d, dist:hyp(cx-S.x,cy-S.y), side:Math.sign(d.vert ? S.x-d.x : S.y-d.y)||1, cx, cy};
    }
  }
  return null;
}
function announceDoor(d, grp, swingSide) {
  const hx = grp.length>1 ? grp.reduce((s,q)=>s+q.x,0)/grp.length : (d.vert ? d.x : d.x-d.hinge*(d.w/2-0.1));
  const hy = grp.length>1 ? grp.reduce((s,q)=>s+q.y,0)/grp.length : (d.vert ? d.y-d.hinge*(d.w/2-0.1) : d.y);
  const rel=relToPerson(hx,hy);
  S.lastSpeechT=-9;
  rsay(`Door ahead, closed. ${grp.length>1?'Double doors; the handles are':'The handle is'} ${sayDir(rel)}, at waist height. ${swingSide?'Pull it towards you':'Push it open'}, and I will lead you through.`, 2, 'door');
  if (cueMode==='v2') cue('doorAhead');
  tl('think', `The ${d.label} is closed and I cannot open doors. Stopped ${swingSide?'clear of its swing':'within reach of the handle'}; waiting for ${PROF[S.profile].name} to open it.`);
}
// returns adjusted [v, w]: the rover never drives into a door that is not fully open
function doorControl(prof, vd, wd) {
  const da=doorAhead(); S.doorAheadInfo=da;
  if (!da) return [vd,wd];
  const d=da.door, grp=S.doors.filter(q=>q.group===d.group);
  if (d.type==='auto' && !d.announced && da.dist<2.6) { d.announced=true; rsay(`Automatic door ahead, ${sayDist(Math.max(0,da.dist-0.9))}. It opens by itself.`, 1, 'door'); }
  if (grp.every(q=>q.f>0.92)) return [vd,wd];
  if (d.type!=='hinged') { if (da.dist<0.85) { vd=0; S.waitWhy='door'; } else if (da.dist<1.6) vd=Math.min(vd,0.12); return [vd,wd]; }
  const swingSide = da.side===d.swing, stopAt = swingSide ? d.w+0.35 : 0.8;
  if (da.dist < stopAt+0.5) vd=Math.min(vd,0.1);
  if (da.dist < stopAt) {
    vd=0; wd=0; S.waitWhy='door';
    if (d.openAt==null) { const delay={new:5.0,cane:3.2,exp:2.0,dog:2.2}[prof.key]*(1+S.psy.stress*0.6); grp.forEach(q=>q.openAt=S.t+delay); announceDoor(d,grp,swingSide); }
    else if (S.t>=d.openAt && !S.psy.frozen) {
      if (!d.opened) { grp.forEach(q=>q.opened=true); tl('event',`${prof.name} opens the ${d.label}.`); feel('doorOpened',-0.02,0.02); rsay('Thank you. Walking through.',1,'door'); }
      grp.forEach(q=>q.openUntil=Math.max(q.openUntil,S.t+4));
    }
  }
  return [vd,wd];
}
function walkerDoorBlock(w, nx, ny, dt) {
  for (const d of S.doors) {
    if (d.f>=0.9 || rectDist(nx,ny,closedRect(d))>WALKER_R+0.02) continue;
    if (d.type==='hinged') { w.doorT=(w.doorT||0)+dt; if (w.doorT>1.2) S.doors.filter(q=>q.group===d.group).forEach(q=>q.openUntil=Math.max(q.openUntil,S.t+4)); }
    return true;
  }
  w.doorT=0; return false;
}

/* ---------- the lift ---------- */
const levelName = L => levelData(WORLD_ID,L).name;
const levelSay = L => levelData(WORLD_ID,L).spoken || levelName(L);
function switchLevel(level) {
  buildWorld(WORLD_ID, level);
  initDoors(); S.walkers=[]; S.objs=[]; S.lastArea=null; S.floorT=S.t; S.path=null;
  S.nearRects = STATIC.filter(b => rectDist(S.x,S.y,b) < 3.2);
  if (G) showLevel(level);
  tl('event', `Now on ${levelName(level)}.`); S.uiDirty=true;
}
const LIFT_TEXT = {
  align:['At the lift','Turning to face the lift doors, so the call button is exactly where I say it is.'],
  call:['Waiting for the call button','I cannot press buttons, so I have told NAME where the call button is and wait for NAME to press it.'],
  wait:['Lift on its way','Lift called. I hold still and pulse a slow heartbeat so the wait never feels like a fault.'],
  opening:['Lift doors opening','The doors are opening. I go in only when they are fully open.'],
  enter:['Walking into the lift','The car is 2.8 m by 2.7 m, enough for NAME and me to turn round inside. Going straight through the middle of the 1.2 m doorway, slowly.'],
  turn:['Turning round in the lift','We turn to face the doors now, so we can walk straight out. NAME walks round me holding the handle; I turn only as fast as NAME steps.'],
  close:['Doors closing','The doors close behind us. They will not close while anyone is in the doorway.'],
  ride:['Riding the lift','NAME cannot see the floor display, so I say the floor out loud when the doors open.'],
  arrive:['Doors opening','We are on the new floor. Waiting for the doors to open fully before walking out.'],
};
function liftControl(dt) {
  const L=WORLD.lift, prof=PROF[S.profile], v2=cueMode==='v2', lf=S.lift, d=S.doors.find(q=>q.id===L.door);
  S.stopReason=null; S.obstUrg=0; S.waitWhy='lift'; S.dwaBlocked=false; S.rel=personRel();
  if (S.linkLost||S.hold) { setCmd(0,0,dt); return; }
  let vd=0, wd=0;
  const ph=lf.phase, since=S.t-lf.t0, go=p=>{ lf.phase=p; lf.t0=S.t; lf.said=false; }, up=lf.target>LEVEL;
  switch (ph) {
    case 'align': { const e=wrap(-Math.PI/2-S.th); if (Math.abs(e)>0.12) wd=clamp(1.5*e,-0.5,0.5); else go('call'); break; }
    case 'call':
      if (!lf.said) { lf.said=true; const [bx,by]=L.button; S.lastSpeechT=-9;
        rsay(`We are at the lift. The call button is ${sayDir(relToPerson(bx,by))}, ${sayDist(Math.max(0.3,hyp(bx-S.px,by-S.py)-0.45))}, at waist height. Please press ${up?'up':'down'}.`, 2, 'lift');
        lf.pressAt=S.t+{new:5,cane:3.5,exp:2.2,dog:2.4}[prof.key]; }
      if (S.t>=lf.pressAt && !S.psy.frozen) { tl('event',`${prof.name} presses the ${up?'up':'down'} button.`); rsay('Thank you. The lift is on its way.',1,'lift'); lf.arriveAt=S.t+5; go('wait'); }
      break;
    case 'wait': if (S.t>=lf.arriveAt) { d.liftOpen=true; tl('event','Ding. The lift arrives and its doors open.'); go('opening'); } break;
    case 'opening': if (d.f>0.95) { rsay('The lift is here and the doors are open. Walking in, straight ahead.',1,'lift'); go('enter'); } break;
    case 'enter': { const [cx,cy]=L.car, dist=hyp(cx-S.x,cy-S.y), e=wrap(Math.atan2(cy-S.y,cx-S.x)-S.th);
      if (dist<0.12) { go('turn'); break; }
      vd=Math.min(0.15,0.6*dist)*Math.max(0,Math.cos(e)); wd=clamp(1.6*e,-0.6,0.6); break; }
    case 'turn': { const e=wrap(Math.PI/2-S.th);
      if (!S.inPlace) { S.inPlace=true; S.spinDir=chooseSpin(e); hap.sweep=S.spinDir; S.lastDirCue=S.t; S.announced=true;
        rsay(`We are in. Now we turn round to face the doors. Walk round to your ${S.spinDir>0?'right':'left'} as I turn.`,1,'turn'); }
      if (Math.abs(e)<0.1) { S.inPlace=false; go('close'); break; }
      wd=S.spinDir*0.5; if (Math.abs(S.rel)>1.15 && Math.sign(wd)===-Math.sign(S.rel)) wd=0; break; }
    case 'close':
      if (!lf.said) { lf.said=true; d.liftOpen=false; rsay(`Doors closing. Going ${up?'up':'down'} to ${levelSay(lf.target)}.`,1,'lift'); }
      if (d.f<0.02) { feel('liftIn', (prof.needsSpeech||prof.key==='cane')?0.05:0.02, 0); go('ride'); }
      break;
    case 'ride':
      if (since>=3 && !lf.switched) { lf.switched=true; switchLevel(lf.target); }
      if (since>=6) { S.doors.find(q=>q.id===L.door).liftOpen=true; tl('event','Ding. The doors open.'); go('arrive'); }
      break;
    case 'arrive': { const nd=S.doors.find(q=>q.id===L.door);
      if (nd.f>0.95) { S.lastSpeechT=-9; rsay(`${cap1(levelSay(LEVEL))}. The doors are open. Walking out, straight ahead.`,2,'lift'); feel('liftOut',-0.04,0.03);
        nd.closeAt=S.t+6; S.mode='nav'; S.lift=null; startGoal(S.goalIdx, S.liftGoal); return; }
      break; }
  }
  if (vd>0 || Math.abs(wd)>0) { const best=dwa(vd,wd); if (best) { vd=best.v; wd=best.w; S.traj=best.traj; } else { vd=0; wd=0; } }
  S.waitT = (ph==='call'||ph==='wait'||ph==='ride') ? S.waitT+dt : 0;
  if (v2 && S.waitT>1.5 && S.t-S.lastHeart>2.2 && ph!=='ride') cue('heart');
  setCmd(vd,wd,dt);
}

/* ---------- stairs: a drop the LiDAR cannot see ---------- */
function stairsCues(prof) {
  const st=STATIC.find(b=>b.cls==='stairs'); S.stairsNear=null; if (!st) return;
  const d=Math.min(rectDist(S.px,S.py,st), rectDist(S.x,S.y,st)), down=st.opt.dir==='down';
  if (d<3.5) S.stairsNear={d, down};
  if (d<3.2 && S.t-(S.lastStairsSay||-99)>25) {
    const rel=relToPerson(clamp(S.px,st.x0,st.x1), clamp(S.py,st.y0,st.y1));
    const ok = rsay(down ? `Careful: stairs going down ${sayDir(rel)}, ${sayDist(d)}. I will keep you well away from the edge.`
                         : `Stairs going up ${sayDir(rel)}, ${sayDist(d)}. I cannot climb stairs, so we use the lift.`, down?2:1, 'stop');
    if (ok) { S.lastStairsSay=S.t; feel('stairsNear', down?0.01:-0.01, 0.01); }
    else if (speechMode==='off' && down) { S.lastStairsSay=S.t; feel('stairsNear', 0.05, -0.01); }
  }
}
Object.assign(VOICE, {
  doorOpened:{cane:["Handle where it said, push... there. Door's open.","Found the handle first try. That's new."], new:["A door. I found the handle where it said it would be.","I opened a door by myself."], dog:["Doors I can do. Juno always showed me the handle side."], exp:["Door, got it."]},
  liftIn:{cane:["Into the lift. Hearing the doors close behind me always makes me tense."], new:["The doors are closing. It's so small in here. At least it told me where we're going."], dog:["Lift. Juno loved lifts, for some reason."], exp:["Up we go."]},
  liftOut:{cane:["It told me the floor. I'd never have known otherwise."], new:["Floor 1. It said so. Okay, I know where I am."], dog:["Right floor, straight out. Fine."], exp:["Out we go."]},
  stairsNear:{cane:["Stairs nearby. I can hear the echo change. Glad it keeps me away from the edge."], new:["Stairs? Please don't let me near the edge."], dog:["Stairs. Juno would have stopped dead at the top."], exp:["Stairs on that side, noted."]},
});
SHOT.doorAhead = {dur:900, f:[fr(0,350,180,0,0,0), fr(550,900,180,0,0,0)], name:'Two long front pulses', say:'Closed door ahead: wait.'};
