
/* ================= Navigation ================= */
function replan(first) {
  computeDist(); S.lastReplan=S.t; S.needReplan=false;
  const p=plan(S.x,S.y,S.goal.x,S.goal.y);
  if (!p) { S.path=null; if (first) tl('think', `No route to ${S.goal.label} yet. Waiting for the way to clear.`); return false; }
  const oldLen = S.path ? (S.path.length-S.pathIdx)*0.1 : 0;
  S.path=p.pts; S.pathIdx=0;
  if (p.snapped && first) { S.goal.x=p.goal[0]; S.goal.y=p.goal[1]; }
  if (first) tl('think', `Route to ${S.goal.label}: ${(S.path.length*0.1).toFixed(1)} m. Planned 0.5 m clear of everything, so the person behind me has room too.`);
  else {
    const what = S.replanWhy ? (DET_NAME[S.replanWhy]||S.replanWhy) : 'obstacle';
    S.replanT=S.t; S.replanTxt=`New route around the ${what}: ${(S.path.length*0.1).toFixed(1)} m${oldLen?`, ${Math.max(0,S.path.length*0.1-oldLen).toFixed(1)} m longer`:''}.`;
    tl('think', S.replanTxt); say('replan'); { const o=[...S.objs].reverse().find(q=>q.cls===S.replanWhy); rsay(o?`Going around the ${what}, ${passSide(o)}.`:`Going around the ${what}.`, 1, 'stop'); } S.replanWhy='';
  }
  return true;
}
function startGoal(idx, goal) {
  S.goalIdx=idx; S.mode='nav'; S.modeT=0; S.navStart=S.t;
  if (goal.floor!=null && goal.floor!==LEVEL && WORLD.lift) { S.liftGoal=goal; S.goal={x:WORLD.lift.call[0], y:WORLD.lift.call[1], label:'the lift', isLift:true, toFloor:goal.floor}; }
  else S.goal=Object.assign({}, goal);
  S.blockedT=0; S.blockReplanned=false; S.backups=0; S.inPlace=false; S.announced=false;
  S.progT=S.t; S.progX=S.x; S.progY=S.y; S.progTh=S.th;
  S.run={driven:0}; replan(true); S.uiDirty=true;
  if (S.path) routePreview(goal);
}
function arrive() {
  if (S.goal && S.goal.isLift) { S.mode='lift'; S.lift={phase:'align', t0:S.t, target:S.goal.toFloor}; S.path=null; S.cmdV=0; tl('do','At the lift. Lining up with its doors.'); return; }
  const g=S.goal, err=hyp(S.x-g.x,S.y-g.y);
  S.cmdV=0; S.cmdW=0; S.path=null; S.stats.arrived++;
  S.goalsDone[S.goalIdx]='done'; S.arriveSide=null;
  let side='';
  if (g.near) {
    let t=STATIC.filter(b=>b.label===g.near).sort((p,q)=>rectDist(S.px,S.py,p)-rectDist(S.px,S.py,q))[0];
    if (t && rectDist(S.px,S.py,t)>3) t=null;
    if (t) { const fx=Math.cos(S.pth), fy=Math.sin(S.pth), cr=fx*(t.cy-S.py)-fy*(t.cx-S.px); side=cr>0?'L':'R'; S.arriveSide={side, label:g.near, d:Math.max(0,rectDist(S.px,S.py,t)-PR), rel:relToPerson(t.cx,t.cy)}; }
  }
  if (cueMode==='v2') cue(side?('arrived'+side):'arrived'); else { hap.play('arrived'); }
  rsay(arrivalPhrase(g, side?S.arriveSide:null), 1, 'arrive');
  tl('event', `Arrived: ${g.label} (${err.toFixed(2)} m from target, ${(S.t-S.navStart).toFixed(0)} s).`);
  feel('arrived', -0.12, 0.07, true);
  if (side && (cueMode==='v2'||speechMode!=='off')) S.pendingVoice={key:'arrivedSide', at:S.t+2};
  S.goal=null; S.mode='between'; S.modeT=0; S.uiDirty=true;
}
function failGoal(why) {
  S.goalsDone[S.goalIdx]='fail'; S.stats.failed++;
  tl('event', `${S.goal.label}: ${why}. Stopped ${hyp(S.x-S.goal.x,S.y-S.goal.y).toFixed(2)} m away.`);
  S.goal=null; S.path=null; S.mode='between'; S.modeT=0; S.cmdV=0; S.cmdW=0; S.uiDirty=true;
}
function setCmd(v,w,dt) {
  S.cmdV+=clamp(v-S.cmdV,-ACC_V*dt,ACC_V*dt);
  S.cmdW+=clamp(w-S.cmdW,-ACC_W*dt,ACC_W*dt);
  if (S.stopReason==='obstacle') S.cmdV=Math.min(S.cmdV,0);
}
function pathBlocked() {
  if (!S.path) return false;
  const end=Math.min(S.path.length-1,S.pathIdx+40);
  for (let k=S.pathIdx;k<=end;k+=2) {
    const [px,py]=S.path[k], ci=cx2i(px), cj=cy2j(py);
    for (let dj=-5;dj<=5;dj++) for (let di=-5;di<=5;di++) {
      const i=ci+di, j=cj+dj; if (i<0||j<0||i>=NX||j>=NY||!marks[j*NX+i]) continue;
      if (hyp(cellX(i)-px,cellY(j)-py) < R_LETHAL) return true;
    }
  }
  return false;
}
// first sharp bend on the route ahead
function upcomingTurn() {
  if (!S.path) return null;
  const pts=S.path; let s=0;
  for (let k=S.pathIdx+1;k<pts.length-4&&s<3.2;k++) {
    s+=0.1;
    const dir=Math.atan2(pts[k+4][1]-pts[k][1], pts[k+4][0]-pts[k][0]), d=wrap(dir-S.th);
    if (Math.abs(d)>0.55) return {side:d>0?'L':'R', dist:s, angle:d};
  }
  return null;
}
function narrowAhead() {
  if (!S.path) return null;
  for (let k=S.pathIdx+5;k<Math.min(S.path.length-2,S.pathIdx+20);k+=2) {
    const a=S.path[k], b=S.path[k+2], dir=Math.atan2(b[1]-a[1],b[0]-a[0]), nx=-Math.sin(dir), ny=Math.cos(dir);
    let l=0, r=0;
    for (let t=0.3;t<=1.3;t+=0.1) { if (!l && occ[cy2j(a[1]+ny*t)*NX+cx2i(a[0]+nx*t)]) l=t; if (!r && occ[cy2j(a[1]-ny*t)*NX+cx2i(a[0]-nx*t)]) r=t; }
    if (l && r && l+r<2.4) return {dist:(k-S.pathIdx)*0.1, width:l+r, x:a[0], y:a[1]};
  }
  return null;
}
// pick the in-place turn direction whose arc the person can actually walk
function chooseSpin(e) {
  const a0=Math.atan2(S.py-S.y,S.px-S.x), R0=hyp(S.py-S.y,S.px-S.x), rects=S.nearRects;
  const ok=dir=>{ const total=dir===Math.sign(e)?Math.abs(e):TAU-Math.abs(e);
    for (let t=0.15;t<total;t+=0.15) { const a=a0+dir*t; if (personClr(S.x+R0*Math.cos(a),S.y+R0*Math.sin(a),rects,S.objs,S.walkers,0)<0.03) return false; } return true; };
  const d=Math.sign(e)||1; if (ok(d)) return d; if (ok(-d)) return -d; return d;
}
function dwa(vd, wd) {
  if (vd<0.005 && Math.abs(wd)<0.005) return {v:0,w:0,traj:null};
  const rects=S.nearRects, objs=S.objs.filter(o=>o.seenL||o.seenC).concat(S.doorRects), wk=S.walkers, walk=PROF[S.profile].walk;
  const rc0=roverClr(S.x,S.y,S.th,rects,objs,wk,0), pc0=personClr(S.px,S.py,rects,objs,wk,0);
  const rMin=Math.min(0.03,rc0-0.01), pMin=Math.min(0.05,pc0-0.01);
  const vs=[vd,vd*0.66,vd*0.33,0].filter((v,i,a)=>a.indexOf(v)===i);
  const ws=[wd,0]; for (const dw of [-0.6,-0.3,-0.15,0.15,0.3,0.6]) ws.push(clamp(wd+dw,-WMAX,WMAX));
  let best=null;
  for (const v of vs) for (const w of ws) {
    if (v===0 && Math.abs(w)<0.02) continue;
    let x=S.x, y=S.y, th=S.th, px=S.px, py=S.py, rc=9, pc=9, ok=true; const traj=[];
    for (let k=1;k<=8;k++) {
      const dt=0.2; th+=w*dt; x+=v*Math.cos(th)*dt; y+=v*Math.sin(th)*dt;
      const r=roverClr(x,y,th,rects,objs,wk,k*dt); if (r<rc) rc=r; if (r<rMin) { ok=false; break; }
      const P=personStep(x,y,th,px,py,dt,walk,rects,objs,wk,k*dt,S.inPlace?S.spinDir:0); if (!P) { ok=false; break; }
      px=P[0]; py=P[1];
      const q=personClr(px,py,rects,objs,wk,k*dt); if (q<pc) pc=q; if (q<pMin) { ok=false; break; }
      { const [gx,gy]=gripOf(x,y,th); if (Math.abs(wrap(Math.atan2(py-gy,px-gx)-th-Math.PI))>1.9) { ok=false; break; } }
      traj.push([x,y]);
    }
    if (!ok) continue;
    const cost=3*Math.abs(v-vd)/0.3+Math.abs(w-wd)-0.5*Math.min(rc,0.4)-0.8*Math.min(pc,0.35);
    if (!best||cost<best.cost) best={v,w,cost,traj,rc,pc};
  }
  return best;
}

function control(dt) {
  const prof=PROF[S.profile], v2=cueMode==='v2';
  S.stopReason=null; S.obstUrg=0; S.waitWhy=''; S.dwaBlocked=false;
  S.rel=personRel();
  if (S.mode==='lift') return liftControl(dt);
  if (S.linkLost||S.hold) { S.waitWhy=S.linkLost?'link':'hold'; setCmd(0,0,dt); return; }
  if (S.mode==='backup') {
    S.backupT+=dt; setCmd(-0.08,0,dt);
    if (S.backupDist>=0.25||S.backupT>5||(S.bumped&&S.backupT>0.4)) { S.mode='nav'; S.blockedT=0; S.blockReplanned=false; replan(false); }
    return;
  }
  if (S.mode!=='nav') { setCmd(0,0,dt); return; }
  const g=S.goal, dGoal=hyp(g.x-S.x,g.y-S.y);
  if (dGoal<XY_TOL) return arrive();
  if (S.t-S.navStart>GOAL_TIMEOUT) return failGoal('timed out');
  if (S.needReplan && S.t-S.lastReplan>0.5) replan(false);
  if (!S.path) { if (S.t-S.lastReplan>2) replan(false); }
  else if (S.t-S.lastPathCheck>0.5) { S.lastPathCheck=S.t; if (S.t-S.lastReplan>1 && pathBlocked()) replan(false); }

  // speed cap: the rover adapts to the person in the adaptive set; the original firmware does not know them
  let cap=VMAX, why='full speed';
  if (v2) {
    const tc=prof.vcapLow+(VMAX-prof.vcapLow)*clamp((S.psy.trust-0.35)/0.45,0,1);
    if (tc<cap-0.005) { cap=tc; why=`${prof.name} is new to this (trust ${pct(S.psy.trust)})`; }
    const pl=prof.pace*(1-0.7*S.psy.stress);
    if (pl<cap-0.005) { cap=Math.max(0.06,pl); why=`matching ${prof.name}'s pace (stress ${pct(S.psy.stress)})`; }
  }
  if (S.psy.frozen) { cap=0; why=`${prof.name} has stopped walking`; }
  const rf=clamp((2.4-Math.abs(S.rel))/1.4,0,1);
  if (rf<1) { cap*=rf; why=`waiting for ${prof.name} to get behind me`; }
  S.cap=cap; S.capWhy=why;

  let vd=0, wd=0;
  if (S.path) {
    const pts=S.path; let best=S.pathIdx, bd=Infinity;
    for (let k=S.pathIdx;k<Math.min(pts.length,S.pathIdx+40);k++) { const d=(pts[k][0]-S.x)**2+(pts[k][1]-S.y)**2; if (d<bd) { bd=d; best=k; } }
    S.pathIdx=best;
    const tk=Math.min(pts.length-1,best+8), e=wrap(Math.atan2(pts[tk][1]-S.y,pts[tk][0]-S.x)-S.th);
    // corners are taken as arcs so the person keeps walking; only real turnarounds pivot on the spot
    if (!S.inPlace && Math.abs(e)>1.6) { S.inPlace=true; S.spinDir=chooseSpin(e); S.announced=false; }
    if (S.inPlace && Math.abs(e)<0.45) { S.inPlace=false; S.announced=false; }
    if (S.inPlace) { vd=0; wd=S.spinDir*0.6; }
    else { vd=cap*Math.max(0.3,1-0.7*Math.abs(e)); wd=clamp(1.6*e,-WMAX,WMAX); }
    vd=Math.min(vd, 0.45*dGoal+0.05);
    // say it, then do it
    if (v2 && S.inPlace && !S.announced) {
      S.announced=true; S.announceUntil=S.t+1.0; S.lastDirCue=S.t;
      rsay(turnaroundPhrase(S.spinDir), 1, 'turn');
      hap.sweep=S.spinDir; tl('tell', `Turnaround sweep: I'm about to turn ${S.spinDir>0?'left':'right'} on the spot; ${prof.name} walks round to stay behind me.`);
      S.stats.turnsCued++; S.cueLog.push(S.t);
    }
    if (S.t<S.announceUntil) { vd=0; wd=0; S.waitWhy='announce'; }
    // the person sets how fast the rover may pivot
    if (Math.abs(S.rel)>1.15 && Math.sign(wd)===-Math.sign(S.rel)) { wd=0; S.waitWhy='person'; }
  }
  [vd,wd] = doorControl(prof, vd, wd);
  // fused forward clearance
  if (vd>0.01 && S.gap<WARN_GAP) { S.obstUrg=clamp(1-(S.gap-STOP_GAP)/(WARN_GAP-STOP_GAP),0,1); S.lastObstT=S.t; }
  if (vd>0.01 && S.gap<SLOW_GAP) vd=Math.max(Math.min(vd,0.03), vd*clamp((S.gap-STOP_GAP)/(SLOW_GAP-STOP_GAP),0,1));
  // stop moving forward, but keep steering: turning away from a wall end is how the rover gets clear (DWA checks it)
  if (vd>0.01 && S.gap<STOP_GAP) { vd=0; S.stopReason='obstacle'; S.waitWhy = S.gapK===3?'walker':'obstacle'; }

  const waiting0 = S.cmdV<0.02 && (S.stopReason||S.dwaBlocked||!S.path||S.psy.frozen||S.waitWhy==='person'||S.waitWhy==='door');
  S.waitT = waiting0 ? S.waitT+dt : 0;
  if (v2) adaptiveCues(prof, vd);
  speechCues(prof);
  const best=dwa(vd,wd);
  if (!best) { vd=0; wd=0; S.dwaBlocked=true; S.traj=null; if (!S.waitWhy) S.waitWhy='tight'; }
  else { vd=best.v; wd=best.w; S.traj=best.traj; S.dwaPc=best.pc; }
  setCmd(vd,wd,dt);

  // stuck handling and Nav2-style progress check
  const waitingOnPerson = S.waitWhy==='person'||S.waitWhy==='announce'||S.waitWhy==='door'||S.psy.frozen;
  const stuck=!waitingOnPerson && (S.stopReason==='obstacle'||S.dwaBlocked||!S.path||S.bumped);
  S.blockedT = stuck ? S.blockedT+dt : 0;
  if (S.blockedT>3 && dGoal<0.3) return arrive();
  if (S.blockedT>3 && !S.blockReplanned && S.path) { S.blockReplanned=true; S.replanWhy=S.waitWhy==='walker'?'walker':''; replan(false); }
  if (S.t-S.progT>12) {
    const moved=hyp(S.x-S.progX,S.y-S.progY)+0.3*Math.abs(wrap(S.th-S.progTh)), walkerNear=S.walkers.some(w=>hyp(w.x-S.x,w.y-S.y)<2.2);
    S.progT=S.t; S.progX=S.x; S.progY=S.y; S.progTh=S.th;
    if (moved<0.15 && !walkerNear && !waitingOnPerson && S.path) {
      if (S.backups>=3) return failGoal('stuck');
      tl('think','No progress in 12 s. Backing up 0.25 m, slowly, then planning again.');
      S.mode='backup'; S.backupDist=0; S.backupT=0; S.backups++;
    }
  }
}

function adaptiveCues(prof, vd) {
  const v=Math.max(S.cmdV,0.12);
  const ut=upcomingTurn(); S.turnInfo=ut;
  if (ut && !S.inPlace && (!prof.quiet || Math.abs(ut.angle)>0.9)) {
    if (ut.dist/v<=prof.anticip && S.t-S.lastTurnCue[ut.side]>4 && S.t-S.lastDirCue>2) {
      S.lastTurnCue[ut.side]=S.t; cue('preTurn'+ut.side); S.stats.turnsCued++;
    }
  }
  if (prof.info) {
    const na=narrowAhead(); S.narrow=na;
    if (na && na.dist<1.6 && S.t-S.lastDoorCue>8 && S.cmdV>0.05) { S.lastDoorCue=S.t; cue('door'); say('doorInfo'); }
    // things passing close to the person's side
    let best=null;
    for (const b of S.nearRects) if (b.kind==='furn') { const d=rectDist(S.px,S.py,b)-PR; if (d<0.45&&(!best||d<best.d)) best={d,x:b.cx,y:b.cy,id:b.id}; }
    for (const o of S.objs) if (o.seenL||o.seenC) { const d=rectDist(S.px,S.py,o)-PR; if (d<0.45&&(!best||d<best.d)) best={d,x:o.cx,y:o.cy,id:o.id}; }
    if (best && S.t-(S.sideCueT[best.id]||-99)>10 && S.cmdV>0.05) {
      S.sideCueT[best.id]=S.t; const fx=Math.cos(S.pth), fy=Math.sin(S.pth), side=fx*(best.y-S.py)-fy*(best.x-S.px)>0?'L':'R';
      cue('side'+side); S.lastSideCue=S.t; say('sideInfo');
    }
  }
  if (S.cmdV>0.12 && vd<0.55*S.cmdV && S.t-S.lastSlowCue>4) cue('slow');
  if (S.waitT>1.5 && S.t-S.lastHeart>1.8 && !S.inPlace) cue('heart');
}

/* ================= The person's mind ================= */
function feel(key, dS, dT, force) {
  const m=S.psy;
  m.stress=clamp(m.stress+dS,0.04,1); m.trust=clamp(m.trust+dT,0,1);
  if (key) say(key, force);
}
function psychTick(dt) {
  const prof=PROF[S.profile], m=S.psy, v2=cueMode==='v2';
  const moving=S.mode==='nav'||S.mode==='backup';
  m.cd=m.cd||{}; const ready=(k,s)=>{ if ((m.cd[k]||-99)+s<=S.t) { m.cd[k]=S.t; return true; } return false; };
  // turns: was the person told before the handle swung?
  const w=Math.abs(S.realW||0), turning = m.turning ? w>0.1 : w>0.22;
  if (turning && !m.turning && moving) {
    const spoke = S.t-S.lastSpokenTurn<4, cued = S.t-S.lastDirCue<3.5 || spoke;
    if (cued && !spoke && prof.needsSpeech && !S.inPlace) { feel('speechMissing',0.05,-0.01); S.hist.push({t:S.t,ev:'s'}); S.stats.surprises++; }
    else if (S.inPlace) { if (cued) feel('turnaround',-0.02,0.02); else { feel('turnaroundConfused',0.13,-0.04); S.stats.surprises++; S.hist.push({t:S.t,ev:'s'}); } }
    else if (cued) { feel('turnCued',-0.025,0.02); S.hist.push({t:S.t,ev:'c'}); }
    else { feel('turnSurprise',0.09+0.05*(prof.react>0.8),-0.04); S.stats.surprises++; S.hist.push({t:S.t,ev:'s'}); }
  }
  m.turning=turning;
  // sudden stops
  const v=S.realV||0;
  if (m.lastV>0.15 && v<0.03 && moving) {
    const told = S.t-S.lastSlowCue<2.5 || S.t-S.lastObstT<2 || S.t-S.lastSpokenWait<2.5;
    if (told) feel('stopExplained',0.01,0.005); else { feel('stopSurprise',0.09,-0.03); S.stats.surprises++; S.hist.push({t:S.t,ev:'s'}); }
  }
  m.lastV = v>m.lastV ? v : m.lastV*0.6+v*0.4;
  // waiting without knowing why
  if (moving && Math.abs(v)<0.02 && !S.inPlace) {
    m.waitT=(m.waitT||0)+dt;
    if (m.waitT>2.5) {
      const reassured = S.t-S.lastHeart<2.5 || S.t-S.lastSpeechT<5;
      if (!m.frozen) m.stress=clamp(m.stress+(reassured?0.002:(prof.key==='cane'?0.03:0.02))*dt,0,1);
      if (!m.waitVoiced) { say(reassured?'waitReassured':'waitConfused', true); m.waitVoiced=true; }
    }
  } else { m.waitT=0; m.waitVoiced=false; }
  // something close to the body
  let close=9;
  for (const b of S.nearRects) if (b.kind==='furn') close=Math.min(close, rectDist(S.px,S.py,b)-PR);
  for (const o of S.objs) close=Math.min(close, rectDist(S.px,S.py,o)-PR);
  for (const wk of S.walkers) close=Math.min(close, hyp(S.px-wk.x,S.py-wk.y)-PR-WALKER_R);
  S.personClose=close;
  if (close<0.28 && moving && ready('close',6)) {
    const told=S.t-(S.lastSideCue||-9)<3;
    if (told) feel(null,0.01,0); else { feel('closeCall',0.06*(prof.info?1.3:0.8),-0.02); S.stats.closeCalls++; }
  }
  const walkerNear=S.walkers.some(wk=>hyp(wk.x-S.x,wk.y-S.y)<1.8);
  if (walkerNear && ready('wk',10)) say('personNear');
  const passer=S.walkers.find(wk=>hyp(wk.x-S.px,wk.y-S.py)<1.0 && hyp(wk.vx,wk.vy)>0.2);
  if (passer && ready('pass',8)) { const told=S.t-(S.lastSpokenPass||-9)<2.5; feel('passedBy', told?0.005:(prof.needsSpeech?0.06:0.03), told?0.005:-0.005); }
  if (speechMode!=='off' && prof.talkTol && S.speechLog.filter(t=>S.t-t<60).length>prof.talkTol && ready('talky',20)) feel('tooTalky',0.04,-0.01);
  if (S.pendingVoice && S.t>=S.pendingVoice.at) { say(S.pendingVoice.key,true); S.pendingVoice=null; }
  // a dog handler finds the pace frustrating
  if (prof.pace>0.9 && S.mode==='nav' && v>0.05) { m.slowT+=dt; if (m.slowT>14) { m.slowT=0; feel('tooSlow',0.05,-0.02); } }
  // too many buzzes
  const recent=S.cueLog.filter(t=>S.t-t<60).length;
  if (recent>prof.cueTol && ready('many',18)) feel('tooMany',0.04*(prof.quiet?1.6:0.8),-0.01);
  // being pulled faster than they want to walk
  const lim=prof.pace*(1-0.7*m.stress);
  if (moving && v>lim+0.03) { m.dragT+=dt; S.stats.dragT+=dt; m.stress=clamp(m.stress+0.04*dt,0,1); if (m.dragT>1.5 && ready('drag',8)) say('dragged',true); }
  else { if (m.dragT>2 && v2) say('slowedForMe'); m.dragT=0; }
  // freezing and recovering
  if (!m.frozen && m.stress>0.86 && moving) { m.frozen=true; say('freeze',true); rsay('Take your time. I will wait until you are ready.', 1, 'wait'); tl('event',`${prof.name} has stopped walking and is holding the handle back.`); }
  if (m.frozen && m.stress<0.6) { m.frozen=false; say('unfreeze',true); }
  // calm travel lowers stress and builds trust
  if (moving && v>0.08 && !turning) { m.flowT+=dt; if (m.flowT>6) { m.flowT=0; feel(ready('flowv',14)?'flow':null,-0.02,0.01); } } else m.flowT=Math.max(0,m.flowT-dt);
  const base=prof.base*(1-0.5*m.trust)+0.04; m.stress += (base-m.stress)*0.006*dt; if (m.frozen) m.stress -= 0.035*dt;   // standing still, breathing
  if (S.mode==='intro'||S.mode==='between') m.stress += (base-m.stress)*0.012*dt;
  m.stress=Math.max(m.stress, base*0.7);
  S.stats.peak=Math.max(S.stats.peak,m.stress); S.stats.stressSum+=m.stress; S.stats.n++;
}

/* ================= Scenario script ================= */
const INTRO_GRIP = [
  {d:2.6, shot:'introF', txt:'FRONT: something is ahead.', speak:'This is the front motor. It means something is ahead.'},
  {d:2.6, shot:'introL', txt:'LEFT: the rover is turning left.', speak:'Left. I am turning left.'},
  {d:2.6, shot:'introR', txt:'RIGHT: the rover is turning right.', speak:'Right. I am turning right.'},
  {d:2.6, shot:'introB', txt:'BACK: slowing down, or step back.', speak:'Back. Slowing down, or step back.'},
  {d:2.8, shot:'preTurnL', txt:'Double tap on one side: a turn is coming in about two seconds.', speak:'Two taps on one side: a turn is coming.'},
  {d:2.6, shot:'introStop', txt:'All four pulsing: stop.', speak:'All four together: stop walking.'},
  {d:2.4, shot:null, txt:'Silence: all clear, keep walking.', speak:'And silence means all clear.'},
];
const INTRO_TALK = [
  {d:4.2, shot:null, txt:'The rover introduces itself out loud.', speak:'Hello. I am your guide rover. I will tell you everything before I do it.'},
  {d:3.8, shot:null, txt:'It explains how the handle works.', speak:'Keep a light hold on the handle. I go slowly, and I stop whenever you stop.'},
  {d:5.2, shot:null, txt:'It explains how it will describe distances and directions.', speak:()=>unitIntro()},
];
function unitIntro() {
  const stride=Math.round((STRIDE[S.profile]||0.65)*100);
  const u={m:'I will give distances in metres.', ft:'I will give distances in feet.', steps:`I will give distances in your steps. One of your steps is about ${stride} centimetres.`,
    'm+steps':`I will give distances in metres and in your steps. One of your steps is about ${stride} centimetres.`, 'ft+steps':'I will give distances in feet and in your steps.'}[unitMode];
  return u + (dirMode==='clock' ? ' I describe where things are like a clock: 12 is straight ahead, 3 on your right, 9 on your left.' : ' Left and right always mean your left and your right.');
}
const introFor = p => p==='new' ? [...INTRO_TALK, ...INTRO_GRIP] : INTRO_GRIP;
function scriptTick(dt) {
  S.modeT+=dt;
  if (S.mode==='intro') {
    const INTRO=introFor(S.profile);
    let acc=0, step=-1;
    for (let i=0;i<INTRO.length;i++) { if (S.modeT>=acc) step=i; acc+=INTRO[i].d; }
    if (S.modeT>=acc) { S.mode='ready'; S.modeT=0; say('introDone',true); feel(null,-0.04,0.06); tl('do','Introduction done. Starting at a gentle pace.'); }
    else if (step!==S.introStep) {
      S.introStep=step; const st=INTRO[step];
      if (st.shot) hap.play(st.shot);
      tl('tell', `Introduction ${step+1}/${INTRO.length}: ${st.txt}`);
      if (st.speak) { S.lastSpeechT=-9; rsay(typeof st.speak==='function'?st.speak():st.speak, 1, 'intro'); }
      const line=(VOICE.intro[S.profile]||VOICE.intro.cane)[step]; if (line) { S.voice=line; S.voiceT=S.t; tl('user',line); if (narrateInner) utter(line,'inner',0); }
      feel(null,-0.012,0.02);
    }
  }
  if (S.mode==='ready' && S.modeT>2.4) {
    if (S.goalIdx<0 && SV.goals.length) { if (cueMode==='v2') hap.play('ready'); startGoal(0, SV.goals[0]); }
    else if (!SV.goals.length) { S.mode='idle'; }
  }
  if (S.mode==='between' && S.modeT>3) {
    if (S.goalIdx+1<SV.goals.length) startGoal(S.goalIdx+1, SV.goals[S.goalIdx+1]);
    else { S.mode='done'; S.modeT=0; S.uiDirty=true; tl('event','Scenario complete.'); }
  }
  const evs=[...(SV.walkers||[]).map((w,i)=>({...w,key:'w'+i})), ...(SV.events||[]).map((e,i)=>({...e,key:'e'+i}))];
  for (const e of evs) {
    if (S.firedEv.has(e.key)) continue;
    const a=e.at; let go=false;
    if (a.t!=null && S.t>=a.t) go=true;
    if (a.roverX!=null && S.x>=a.roverX && S.mode!=='intro') go=true;
    if (a.navT!=null && S.mode==='nav' && S.t-S.navStart>=a.navT) go=true;
    if (a.floor!=null) go = LEVEL===a.floor && S.floorT>0 && S.t-S.floorT>=(a.delay||0);
    if (!go) continue;
    S.firedEv.add(e.key);
    if (e.path) { addWalker(e.path, e.speed, e.look, e.loop); tl('event','Someone starts walking nearby.'); }
    if (e.spawn) { addObj(e.spawn); }
    if (e.link!=null) setLink(e.link);
    if (e.say) tl('event', e.say);
  }
}
function addObj(sp) {
  const o=mk(sp.x,sp.y,sp.sx,sp.sy,{kind:'obj', isObj:true, cls:sp.cls, h:sp.h, id:nextId++, seenL:false, seenC:false});
  S.objs.push(o); return o;
}
function setLink(lost) {
  if (S.linkLost===lost) return;
  S.linkLost=lost;
  if (lost) { const sp=rsay('The handle has lost its connection. Please stand still, I have stopped.', 2, 'link'); feel('linkLost',sp?0.15:0.25,-0.08,true); tl('think','No packets from the handle for 2 s. If I keep moving, the person walks blind. Stopping until the link is back.'); }
  else { rsay('The connection is back. Walking on.', 2, 'link'); feel('linkBack',-0.05,0.02,true); tl('do','Link back. Resuming at reduced speed.'); }
  S.uiDirty=true;
}

/* ================= Thought process ================= */
function think() {
  const prof=PROF[S.profile], nm=prof.name, R={}, v2=cueMode==='v2';
  const kname=['nothing','structure','object','person','',''][S.nearestK]||'object';
  const sideOf=a=>Math.abs(a)<0.6?'ahead':Math.abs(a)>2.4?'behind':a>0?'to my left':'to my right';
  const top=S.dets.filter(d=>d.cls!=='doorway').slice(0,2).map(d=>`${d.name} ${pct(d.conf)} at ${d.d.toFixed(1)} m`);
  const door=S.dets.find(d=>d.cls==='doorway');
  S.overhang=null;
  for (const b of S.nearRects) if (b.kind==='furn' && (b.scan==='l'||b.scan==='d') && b.h>0.6) { const d=rectDist(S.px,S.py,b)-PR; if (d<0.5 && (!S.overhang||d<S.overhang.d)) S.overhang={id:b.id,label:b.label,d}; }
  R.see = `LiDAR: nearest ${isFinite(S.nearest)?`${kname} ${S.nearest.toFixed(2)} m ${sideOf(S.nearestA)}`:'nothing in range'}, ${isFinite(S.gap)?S.gap.toFixed(1)+' m clear ahead':'open ahead'}; ${S.userPts} returns are ${nm}, ignored. Camera: ${top.length?top.join(', '):'nothing of note'}${door?`, ${door.label} ${door.d.toFixed(1)} m`:''}.`;
  let key='idle', head='Waiting', th='Waiting for a destination.';
  const ut=S.turnInfo, na=S.narrow;
  if (S.linkLost) { key='link'; head='Handle link lost'; th=`No packets from the handle for 2 s. ${nm} would be walking with a silent grip that looks like "all clear". I stop and stay put until it reconnects.`; }
  else if (S.hold) { key='hold'; head='Holding'; th='Told to hold. Motors off; the grip says stop.'; }
  else if (S.mode==='intro') { key='intro'+S.introStep; head='Introducing the grip'; th=`${nm} is standing still while I play each motor once. Learning the four positions now means nothing is a surprise later.`; }
  else if (S.mode==='ready') { key='ready'; head='Getting ready'; th=`Giving ${nm} a moment to settle before moving.`; }
  else if (S.mode==='done') { key='done'; head='Walk complete'; th=`All goals done. ${nm} ends with trust ${pct(S.psy.trust)} and stress ${pct(S.psy.stress)}.`; }
  else if (S.mode==='between') { key='between'; head='Arrived'; th=S.arriveSide?`Parked so the ${S.arriveSide.label.toLowerCase()} is ${S.arriveSide.d.toFixed(1)} m to ${nm}'s ${S.arriveSide.side==='L'?'left':'right'}. Telling ${nm} which side.`:`At the goal. A short pause before the next leg.`; }
  else if (S.mode==='lift') { const T=LIFT_TEXT[S.lift.phase]; key='lift'+S.lift.phase; head=T[0]; th=T[1].replace(/NAME/g,nm); }
  else if (S.waitWhy==='door' && S.doorAheadInfo) { const d=S.doorAheadInfo.door; key='door'+d.id;
    head = d.type==='hinged' ? 'Closed door' : 'Automatic door';
    th = d.type==='hinged' ? `The ${d.label} is closed and I have no arm to open it. I stopped ${S.doorAheadInfo.side===d.swing?'well back, because it swings towards us':`where ${nm} can reach the handle`}, told ${nm} where the handle is, and go through only once it is fully open.`
                           : `The ${d.label} opens by itself when its sensor sees us. I wait until it is fully open, so ${nm} is never caught by a moving door.`; }
  else if (S.stairsNear && S.stairsNear.d<3) { key='stairs'+S.stairsNear.down; head = S.stairsNear.down ? 'Stairs: a drop the LiDAR cannot see' : 'Stairs going up';
    th = S.stairsNear.down ? `The stairwell is ${S.stairsNear.d.toFixed(1)} m away: a 3.2 m drop. My LiDAR scans at 0.23 m, so where the floor ends it simply sees nothing. The map keeps the stairwell as a no-go zone and the camera watches the tactile strip at its edge; I keep ${nm} at least 0.4 m from it.`
                           : `Stairs going up, ${S.stairsNear.d.toFixed(1)} m away. To a wheeled robot they are a wall. For other floors I use the lift.`; }
  else if (S.psy.frozen) { key='frozen'; head=`${nm} has stopped`; th=`The handle is pulling back: ${nm} has frozen (stress ${pct(S.psy.stress)}). Pulling now would break trust. I wait${v2?' and pulse a slow heartbeat so the silence does not feel like a fault':''}.`; }
  else if (S.mode==='backup') { key='backup'; head='Backing up'; th=`I'm stuck, so I reverse 0.25 m at walking-backwards pace. ${nm} is behind me, so I check ${nm} has room to step back first.`; }
  else if (S.inPlace) { key='spin'; head='Turning on the spot'; th=`The route doubles back. I pivot ${S.spinDir>0?'left':'right'} and ${nm} walks round me holding the handle. I only turn as fast as ${nm} steps (handle angle ${Math.round(Math.abs(S.rel)*57.3)}°). I chose the side where ${nm}'s path round me is clear.`; }
  else if (S.waitWhy==='walker') { key='walker'; head='Letting someone pass'; th=`A person is crossing ${S.gap.toFixed(1)} m ahead. People get right of way; squeezing past with ${nm} on the handle would leave no margin.`; }
  else if (S.waitWhy==='obstacle') { key='obst'; head='Blocked'; th=`Something is ${S.gap.toFixed(2)} m in front. Stopping, then I'll look for a way round.`; }
  else if (S.dwaBlocked) { key='tight'; head='Too tight for both of us'; th=`Every move I can make right now would bring me or ${nm} within a few centimetres of something. Holding still while I re-plan.`; }
  else if (S.camNew && S.t-S.camNewT<6 && S.camNew.h<LIDAR_H) { key='cam'+S.camNew.id; head='Camera caught a low object'; th=`The camera sees a ${DET_NAME[S.camNew.cls]} that my LiDAR misses: it scans at 0.23 m and this is only ${S.camNew.h.toFixed(2)} m tall. Marked it on the costmap from the camera.`; }
  else if (S.t-S.replanT<4) { key='replan'+Math.floor(S.replanT); head='Re-routing'; th=`${S.replanTxt} The new line keeps ${nm} at least ${Math.max(0,S.personClose||0).toFixed(2)} m from it.`; }
  else if (ut && ut.dist<3) { key='turn'+ut.side; head=`${ut.side==='L'?'Left':'Right'} turn coming`; th=`Route bends ${ut.side==='L'?'left':'right'} by ${Math.round(Math.abs(ut.angle)*57.3)}° in ${ut.dist.toFixed(1)} m. ${v2?`${nm} needs about ${prof.anticip} s of warning, so the double tap goes out ${(prof.anticip*Math.max(S.cmdV,0.12)).toFixed(1)} m before the bend.`:`The original firmware only buzzes once the turn starts.`}`; }
  else if (S.overhang) { key='over'+S.overhang.id; head='Tabletop at hip height'; th=`My LiDAR scans at 0.23 m, so it only sees the legs of the ${S.overhang.label.toLowerCase()}. Its top is at ${nm}'s hip. I plan from the map, which knows the whole top, and keep ${nm} ${Math.max(0,S.overhang.d).toFixed(2)} m clear of it.`; }
  else if (na) { key='narrow'; head='Doorway ahead'; th=`Gap ${na.width.toFixed(1)} m wide in ${na.dist.toFixed(1)} m. Rover 0.76 m plus ${nm} swinging behind: keep to the centre line and go straight through.`; }
  else if (S.gap<SLOW_GAP) { key='slow'; head='Slowing'; th=`Something ${S.gap.toFixed(1)} m ahead. Slowing smoothly so the stop never jolts ${nm}'s arm.`; }
  else if (S.cap<VMAX-0.01) { key='cap'; head='Walking at their pace'; th=`Holding ${S.cap.toFixed(2)} m/s: ${S.capWhy}.${prof.key==='dog'?' Sam is used to a dog at 1.2 m/s, so I keep turns smooth to make up for it.':''}`; }
  else if (S.goal) { key='cruise'; head='Clear route'; th=`Nothing within ${Math.min(S.gap,9).toFixed(1)} m ahead. Holding speed. Silence on the grip tells ${nm} all is well.${prof.key==='dog'?' Still slower than the 1.2 m/s Sam expects.':''}`; }
  R.head=head; R.think=th;
  R.do = `v ${S.cmdV.toFixed(2)} m/s · ω ${S.cmdW>=0?'+':''}${S.cmdW.toFixed(2)} rad/s · cap ${S.cap.toFixed(2)} (${S.capWhy}) · ${S.traj?'local plan: '+S.traj.length*0.2+' s arc checked for rover and person':'no motion'}`;
  S.think=R;
  if (key!==S.thinkKey) { S.thinkKey=key; if (!['idle','cruise'].includes(key)) tl('think', `${head}. ${th}`); }
}
