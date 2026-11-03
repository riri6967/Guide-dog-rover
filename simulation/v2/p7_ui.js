
/* ================= Panels ================= */
const VOCAB = [
  {k:'none', n:'Silence', d:'All clear, keep walking'},
  {k:'turn', n:'Side buzz', d:'Turning that way; stronger = sharper'},
  {k:'pre', n:'Double tap', d:'Turn coming in ~2 s', v2:true},
  {k:'slow', n:'Back tap', d:'Slowing down', v2:true},
  {k:'door', n:'Both sides tap', d:'Doorway, walls close', v2:true},
  {k:'side', n:'Single side tap', d:'Passing something there', v2:true},
  {k:'obstacle', n:'Front pulsing', d:'Obstacle ahead, faster = closer'},
  {k:'stop', n:'All four pulsing', d:'Stop walking'},
  {k:'heart', n:'Heartbeat', d:'Waiting on purpose, still working', v2:true},
  {k:'doorwait', n:'Two long front pulses', d:'Closed door ahead: wait', v2:true},
  {k:'turnaround', n:'Sweep', d:'Turning on the spot: walk round with it', v2:true},
  {k:'backup', n:'Back steady', d:'Step back'},
  {k:'arrived', n:'Two soft pulses', d:'Arrived (+ tap: which side)'},
  {k:'lost', n:'Three short pulses', d:'Link lost: the handle is blind'},
];
const SHOT_VOCAB = {doorAhead:'doorwait',preTurnL:'pre',preTurnR:'pre',slow:'slow',door:'door',sideL:'side',sideR:'side',heart:'heart',ready:'heart',arrived:'arrived',arrivedL:'arrived',arrivedR:'arrived',introF:'obstacle',introL:'turn',introR:'turn',introB:'backup',introStop:'stop'};
const motorEls = (()=>{ const MOT=[['Front',120,56,120,22],['Back',120,204,120,244],['Left',46,130,32,176],['Right',194,130,208,176]];
  const g=$('motors');
  return MOT.map(([name,x,y,lx,ly])=>{ const e=document.createElementNS('http://www.w3.org/2000/svg','g');
    e.innerHTML=`<circle class="halo" cx="${x}" cy="${y}" r="16" fill="#FF6B57" opacity="0"/><circle cx="${x}" cy="${y}" r="13" fill="#0F1720" stroke="#8FA0B3" stroke-width="1.5"/><circle class="core" cx="${x}" cy="${y}" r="9" fill="#FF6B57" opacity="0"/><text x="${lx}" y="${ly}" text-anchor="middle" font-size="12" fill="#E8EEF4" font-family="IBM Plex Mono, monospace">${name==='Front'?'Front ▲':name} <tspan class="duty" fill="#8FA0B3">0</tspan></text>`;
    g.appendChild(e); return {halo:e.querySelector('.halo'), core:e.querySelector('.core'), duty:e.querySelector('.duty')}; }); })();
const cache={};
const setT=(id,t)=>{ if (cache[id]!==t) { cache[id]=t; $(id).textContent=t; } };
const setH=(id,h)=>{ if (cache[id]!==h) { cache[id]=h; $(id).innerHTML=h; } };
const esc=s=>String(s).replace(/[&<>]/g,c=>({'&':'&amp;','<':'&lt;','>':'&gt;'}[c]));
function renderVocab() {
  $('vocab').innerHTML = VOCAB.map(v=>`<li data-v="${v.k}" class="${v.v2?'v2':''} ${v.v2&&cueMode!=='v2'?'off':''}"><b>${v.n}</b><span>${v.d}</span></li>`).join('');
  cache.vocab='';
}
function renderGrip() {
  const o=hap.out;
  o.forEach((d,i)=>{ const m=motorEls[i], f=d/255; m.halo.setAttribute('opacity',(f*0.35).toFixed(2)); m.halo.setAttribute('r',(16+f*12).toFixed(1)); m.core.setAttribute('opacity',f.toFixed(2)); m.duty.textContent=d; });
  let vk='none', name='Silence', sayTxt='Carry on straight. Nothing to do.';
  const p=hap.pat;
  if (p===P.LOST) { vk='lost'; name='Link lost'; sayTxt='Three short pulses on all four. The handle has stopped hearing the rover.'; }
  else if (p===P.STOP) { vk='stop'; name='Stop'; sayTxt='All four motors pulsing hard. Stop walking.'; }
  else if (p===P.BACKUP) { vk='backup'; name='Step back'; sayTxt='Back motor steady. The rover is reversing.'; }
  else if (p===P.TURN) { vk='turnaround'; name='Turnaround sweep'; sayTxt=`Front, ${hap.sweep>0?'right':'left'}, back: walk round to your ${hap.sweep>0?'right':'left'} as the rover turns.`; }
  else if (hap.shot) { vk=SHOT_VOCAB[hap.shotKey]||'none'; name=hap.shot.name; sayTxt=hap.shot.say; }
  else if (p===P.OBSTACLE) { vk='obstacle'; name='Obstacle ahead'; sayTxt=`Front pulsing: something ${sayDist(S.gap)} ahead.`; }
  else if (o[2]||o[3]) { vk='turn'; name=o[2]?'Left buzz':'Right buzz'; sayTxt=`The rover is steering ${o[2]?'left':'right'} now.`; }
  else if (o[1]) { vk='backup'; name='Back buzz'; sayTxt='The rover is reversing.'; }
  setT('patName',name); setT('patSay',sayTxt);
  if (cache.vocab!==vk) { cache.vocab=vk; document.querySelectorAll('#vocab li').forEach(li=>li.classList.toggle('on',li.dataset.v===vk)); }
  const hx=n=>n.toString(16).toUpperCase().padStart(2,'0');
  setH('packet', S.linkLost ? 'ESP-NOW · <b>no packet for 2 s</b> → handle plays LINK LOST' :
    `ESP-NOW → F <b>${hx(o[0])}</b> · B <b>${hx(o[1])}</b> · L <b>${hx(o[2])}</b> · R <b>${hx(o[3])}</b> · pattern <b>${p}</b> ${PNAME[p]}${hap.shot?' + '+hap.shotKey:''}`);
  setT('speakerTxt', S.lastSpeech && S.t-S.lastSpeechT<8 ? `“${S.lastSpeech}”` : speechMode==='off' ? 'Voice off' : '—');
  sound(o);
}
function renderScenario() {
  const prof=PROF[S.profile];
  const avail=SCEN.filter(s=>s[WORLD_ID]); setT('scIdx', `${avail.indexOf(SC)+1} of ${avail.length}`); setT('scTitle',SC.title); setT('scBrief',SC.brief[WORLD_ID]);
  setH('scWhy', SC.why.map(w=>`<li>${esc(w)}</li>`).join(''));
  const steps=[];
  if (SC.intro) steps.push({t:'Meet the grip: each motor once', s:S.mode==='intro'?'cur':'done'});
  SV.goals.forEach((g,i)=>steps.push({t:g.label, s:S.goalsDone[i]==='done'?'done':S.goalsDone[i]==='fail'?'fail':S.goalIdx===i&&S.mode!=='between'?'cur':''}));
  if (SC.free) steps.push({t:S.goal?`Driving to ${S.goal.label}`:'Click the floor to choose a destination', s:S.goal?'cur':''});
  setH('scSteps', steps.map(s=>`<li class="${s.s}"><i></i><span>${esc(s.t)}</span></li>`).join(''));
  const db=$('scDebrief');
  if (S.mode==='done') {
    const st=S.stats, avg=st.n?st.stressSum/st.n:0;
    const tips=[];
    if (cueMode==='orig' && st.surprises>0) tips.push(`${st.surprises} turns or stops came without warning: the original firmware only buzzes once the rover is already turning. Switch to Adaptive cues and restart to compare.`);
    if (cueMode==='v2' && st.surprises===0) tips.push(`Every turn was announced before it happened, so ${prof.name} never had to guess.`);
    if (st.dragT>2) tips.push(`${prof.name} was pulled faster than they wanted to walk for ${st.dragT.toFixed(0)} s. A handle force sensor would let the rover match their pace.`);
    if (prof.key==='dog') tips.push('For former dog handlers, fewer cues and smooth turns matter more than information. The adaptive set keeps gentle turns silent for Sam.');
    if (prof.needsSpeech && speechMode==='off') tips.push(`${prof.name} has not learned the grip yet. With the rover voice off, every buzz is a puzzle. Turn the voice on and restart to compare.`);
    if (prof.needsSpeech && speechMode!=='off') tips.push(`The rover's voice did most of the work for ${prof.name}. Once the grip vocabulary sticks, it can say less.`);
    if (prof.key==='dog' && speechMode==='describe') tips.push('Room descriptions are too much for Sam. Key moments only suits a former dog handler better.');
    if (st.closeCalls) tips.push(`${st.closeCalls} close passes near ${prof.name}'s body without a side tap.`);
    db.hidden=false;
    setH('scDebrief', `<div class="eyebrow">Debrief · ${cueMode==='v2'?'Adaptive cues':'Original firmware'}</div>
      <div>${WORLD.name} · time <b>${fmtT(S.t)}</b> · goals <b>${st.arrived}/${SV.goals.length}</b> · surprises <b>${st.surprises}</b> · grip cues <b>${st.cues}</b> · spoken <b>${st.spoken}</b></div>
      <div>Trust <b>${pct(st.trust0)} → ${pct(S.psy.trust)}</b> · stress avg <b>${pct(avg)}</b>, peak <b>${pct(st.peak)}</b></div>
      ${tips.map(t=>`<div>${esc(t)}</div>`).join('')}`);
  } else db.hidden=true;
  setT('pName',prof.name); setT('pInit',prof.init); setT('pTag',prof.tag);
  setH('pChips', prof.chips.map(c=>`<span class="chip">${esc(c)}</span>`).join(''));
  $('voice').dataset.who=`${prof.name}'s inner voice`;
  $('tools').hidden=!SC.free;
  setT('thTellLbl', `Tells ${prof.name}`);
}
function renderPerson() {
  const m=S.psy, prof=PROF[S.profile];
  $('gTrust').style.width=pct(m.trust); setT('vTrust',pct(m.trust));
  $('gStress').style.width=pct(m.stress); setT('vStress',pct(m.stress));
  const pace=prof.pace*(1-0.7*m.stress)*(m.frozen?0:1);
  $('gPace').style.width=pct(clamp(pace/1.2,0,1)); setT('vPace',pace.toFixed(2));
  setT('voice', S.voice ? `“${S.voice}”` : '');
}
function renderThink() {
  const R=S.think||{};
  setT('thHead',R.head||''); setT('thSee',R.see||''); setT('thThink',R.think||''); setT('thDo',R.do||'');
  setT('thTell', `${$('patName').textContent}: ${$('patSay').textContent}`);
  if (S.tlDirty) {
    S.tlDirty=false;
    $('timeline').innerHTML = S.tl.slice(0,60).map(e=>`<li><time>${fmtT(e.t)}</time><span class="tag k-${e.tag}">${e.tag==='user'?PROF[S.profile].name:e.tag==='say'?'rover says':e.tag}</span><span class="${e.tag==='user'?'u':''}">${esc(e.text)}</span></li>`).join('');
  }
}
function renderStatus() {
  const prof=PROF[S.profile];
  let pill='Ready', s='nav', txt='';
  if (S.linkLost) { pill='Link lost'; s='bad'; txt='Handle cannot hear the rover. Stopped.'; }
  else if (S.hold) { pill='Holding'; s='bad'; txt='Stopped on request.'; }
  else if (S.mode==='intro') { pill='Introduction'; s='ok'; txt=`Teaching ${prof.name} the grip, one motor at a time.`; }
  else if (S.mode==='nav') { pill=S.psy.frozen?'Person stopped':S.stopReason?'Waiting':S.inPlace?'Turning':'Guiding'; s=S.psy.frozen||S.stopReason?'warn':'nav'; txt=`${S.goal.label} · ${hyp(S.goal.x-S.x,S.goal.y-S.y).toFixed(1)} m · ${S.cmdV.toFixed(2)} m/s`; }
  else if (S.mode==='backup') { pill='Backing up'; s='warn'; txt='Reversing slowly.'; }
  else if (S.mode==='lift') { pill='Lift'; s='nav'; txt=`${LIFT_TEXT[S.lift.phase][0]} · going to ${levelName(S.lift.target)}`; }
  else if (S.mode==='between') { pill='Arrived'; s='ok'; txt='Short pause.'; }
  else if (S.mode==='done') { pill='Complete'; s='ok'; txt='See the debrief on the left. Restart or try the other cue set.'; }
  else if (S.mode==='idle') { pill='Idle'; s=''; txt='Click the floor to choose a destination.'; }
  if (WORLD.levels) txt = `${levelName(LEVEL)} · ${txt}`;
  const el=$('stPill'); if (el.dataset.s!==s) el.dataset.s=s; setT('stPill',pill); setT('stTxt',txt);
  const riding = S.mode==='lift' && (S.lift.phase==='ride'||S.lift.phase==='close');
  $('liftHud').hidden=!riding;
  if (riding) setH('liftHud', `<span class="eyebrow">Lift</span><b>${S.lift.target>LEVEL||S.lift.switched&&S.lift.target===LEVEL&&S.lift.target>0?'▲':'▼'} ${LEVEL}</b><span>${S.lift.switched?'arriving':S.lift.phase==='close'?'doors closing':'moving'}</span>`);
}
function renderSensors() {
  setH('dets', S.dets.length ? S.dets.slice(0,8).map(d=>`<li><b>${esc(d.name)}${d.cls==='doorway'?' · '+esc(d.label):''}</b><span>${Math.round(d.conf*100)}%</span><span>${d.d.toFixed(2)} m</span></li>`).join('') : '<li><b>no detections</b><span></span><span></span></li>');
  const f=v=>isFinite(v)&&v<20?v.toFixed(2)+' m':'—';
  setH('lidKv', [['front',f(S.sect.F)],['left',f(S.sect.L)],['right',f(S.sect.R)],['rear',f(S.sect.B)],['clear ahead',f(S.gap)],['user mask',S.userPts+' pts']].map(([a,b])=>`<div><span>${a}</span><span>${b}</span></div>`).join(''));
  const pc=S.personClose, deg=Math.round(S.rel*57.3);
  setH('tele', [['/cmd_vel linear.x',S.cmdV.toFixed(2)+' m/s'],['/cmd_vel angular.z',S.cmdW.toFixed(2)+' rad/s'],['/odom x, y',`${S.x.toFixed(2)}, ${S.y.toFixed(2)}`],['/odom yaw',Math.round(S.th*57.3)+'°'],
    ['/ticks_left',S.ticksL],['/ticks_right',S.ticksR],['speed cap',S.cap.toFixed(2)+' m/s'],['handle angle',(deg>0?'+':'')+deg+'°'],
    ['person clearance',pc!=null&&pc<9?Math.max(0,pc).toFixed(2)+' m':'—'],['rover clearance',S.traj&&best_rc()!=null?best_rc():'—'],['costmap marks',marksCount()],['spoken lines',S.stats.spoken],['sim speed',speed+'×']]
    .map(([a,b])=>`<div><span>${a}</span><span>${b}</span></div>`).join(''));
}
function best_rc(){ const r=roverClr(S.x,S.y,S.th,S.nearRects,S.objs,S.walkers,0); return r<9?Math.max(0,r).toFixed(2)+' m':null; }
let mcT=0, mcV=0; function marksCount(){ if (performance.now()-mcT>1000) { mcT=performance.now(); mcV=0; for (let k=0;k<N;k++) if (marks[k]) mcV++; } return mcV+' cells'; }
function drawPolar() {
  const cv=$('cPolar'), x=cv.getContext('2d'), W=cv.width, H=cv.height, cx=W/2, cy=H/2+20, sc=(W/2-14)/4;
  x.clearRect(0,0,W,H);
  x.strokeStyle='#1E2B39'; x.lineWidth=1; x.fillStyle='#5D6E81'; x.font='10px IBM Plex Mono, monospace';
  for (let r=1;r<=4;r++) { x.beginPath(); x.arc(cx,cy,r*sc,0,TAU); x.stroke(); x.fillText(r+' m',cx+3,cy-r*sc+11); }
  x.beginPath(); x.moveTo(cx,cy-4.2*sc); x.lineTo(cx,cy+4.2*sc); x.moveTo(cx-4.2*sc,cy); x.lineTo(cx+4.2*sc,cy); x.stroke();
  // zones: forward is up
  x.fillStyle='rgba(240,164,58,.10)'; x.fillRect(cx-(HW+0.1)*sc, cy-(HL+SLOW_GAP)*sc, 2*(HW+0.1)*sc, SLOW_GAP*sc);
  x.fillStyle='rgba(255,94,94,.22)'; x.fillRect(cx-(HW+0.1)*sc, cy-(HL+STOP_GAP)*sc, 2*(HW+0.1)*sc, STOP_GAP*sc);
  x.fillStyle='#F2B33D'; x.fillRect(cx-HW*sc, cy-HL*sc, 2*HW*sc, 2*HL*sc);
  // person, from the handle angle
  const pa=Math.atan2(S.py-S.y,S.px-S.x)-S.th, pd=hyp(S.py-S.y,S.px-S.x); x.fillStyle='#AE92FF'; x.beginPath(); x.arc(cx-Math.sin(pa)*pd*sc, cy-Math.cos(pa)*pd*sc, PR*sc,0,TAU); x.fill();
  x.strokeStyle='#C5CAD0'; x.lineWidth=2; x.beginPath(); x.moveTo(cx,cy); x.lineTo(cx, cy-GRIP_X*sc); x.stroke();
  const lo=LIDAR_OFF;
  for (let n=0;n<LIDAR_N;n++) {
    const r=S.scan[n]; if (r>=LIDAR_MAX) continue; const k=S.scanK[n];
    const a=-Math.PI+n*TAU/LIDAR_N, rx=lo+Math.cos(a)*r, ry=Math.sin(a)*r; if (hyp(rx,ry)>4.2) continue;
    const col=KCOL[k]||KCOL[1]; x.fillStyle=`rgb(${col.map(v=>Math.round(v*255)).join(',')})`;
    x.fillRect(cx-ry*sc-1.5, cy-rx*sc-1.5, 3, 3);
  }
  if (isFinite(S.nearest) && S.nearest<4) { const a=S.nearestA; x.strokeStyle='#E8EEF4'; x.setLineDash([3,3]); x.beginPath(); x.moveTo(cx,cy); x.lineTo(cx-Math.sin(a)*S.nearest*sc, cy-Math.cos(a)*S.nearest*sc); x.stroke(); x.setLineDash([]); }
  x.fillStyle='#8FA0B3'; x.fillText('▲ forward', cx-26, 12);
  x.fillStyle='#5CC6E6'; x.fillText('■ wall', 8, H-8); x.fillStyle='#F2B33D'; x.fillText('■ object', 64, H-8); x.fillStyle='#FF6B57'; x.fillText('■ person', 132, H-8); x.fillStyle='#AE92FF'; x.fillText('■ user (ignored)', 200, H-8);
}
function drawChart() {
  const cv=$('cChart'), W=cv.clientWidth||700, H=170; if (cv.width!==W) cv.width=W; cv.height=H;
  const x=cv.getContext('2d'), L=34, R=10, T=10, B=22, h=S.hist, tmax=Math.max(60,S.t);
  x.clearRect(0,0,W,H); x.font='10px IBM Plex Mono, monospace'; x.fillStyle='#5D6E81'; x.strokeStyle='#1E2B39';
  for (const v of [0,0.5,1]) { const y=T+(1-v)*(H-T-B); x.beginPath(); x.moveTo(L,y); x.lineTo(W-R,y); x.stroke(); x.fillText(Math.round(v*100)+'%',4,y+3); }
  const step=tmax>240?60:30; for (let t=0;t<=tmax;t+=step) { const X=L+t/tmax*(W-L-R); x.fillText(fmtT(t),X-10,H-6); }
  const X=t=>L+t/tmax*(W-L-R), Y=v=>T+(1-v)*(H-T-B);
  for (const [key,col] of [['trust','#4CC38A'],['stress','#FF6B57']]) {
    x.strokeStyle=col; x.lineWidth=2; x.beginPath(); let first=true;
    for (const p of h) if (p[key]!=null) { first?x.moveTo(X(p.t),Y(p[key])):x.lineTo(X(p.t),Y(p[key])); first=false; } x.stroke();
  }
  for (const p of h) if (p.ev) { x.fillStyle=p.ev==='c'?'#F2B33D':'#FF5E5E'; x.fillRect(X(p.t)-1,T,2,6); }
}

/* ================= Sound: each motor its own tone and ear ================= */
let audio=null, soundOn=false;
function initAudio(){ if (audio) return; try { const ctx=new (window.AudioContext||window.webkitAudioContext)(), master=ctx.createGain(); master.gain.value=0.6; master.connect(ctx.destination);
  audio={ctx, ch:[[200,0],[110,0],[150,-1],[150,1]].map(([f,pan])=>{ const o=ctx.createOscillator(); o.type='sawtooth'; o.frequency.value=f; const lp=ctx.createBiquadFilter(); lp.type='lowpass'; lp.frequency.value=500; const g=ctx.createGain(); g.gain.value=0; const p=ctx.createStereoPanner?ctx.createStereoPanner():null; o.connect(lp); lp.connect(g); if (p) { p.pan.value=pan; g.connect(p); p.connect(master); } else g.connect(master); o.start(); return g; })}; } catch(e) { audio=null; } }
function sound(o){ if (!audio) return; const t=audio.ctx.currentTime; audio.ch.forEach((g,i)=>g.gain.setTargetAtTime(soundOn?(o[i]/255)*0.12:0,t,0.01)); }

/* ================= Free roam ================= */
function freeAction(x,y) {
  const [bx0,by0,bx1,by1]=WORLD.bounds, inside=(m)=>STATIC.some(b=>rectDist(x,y,b)<m) || x<bx0+0.3 || x>bx1-0.3 || y<by0+0.3 || y>by1-0.3;
  if (tool==='goal') {
    if (inside(0.2)) { flash('That spot is inside a wall or furniture. Pick open floor.'); return; }
    S.goalsDone=[]; SV.goals=[{x,y,label:`the spot at (${x.toFixed(1)}, ${y.toFixed(1)})`}]; S.goalIdx=-1; startGoal(0,SV.goals[0]); S.uiDirty=true;
  } else if (tool==='bag'||tool==='box') {
    const hit=S.objs.findIndex(o=>rectDist(x,y,o)<0.05);
    if (hit>=0) { const o=S.objs.splice(hit,1)[0]; markObj(o,0); flash('Removed.'); return; }
    if (inside(0.3) || hyp(x-S.x,y-S.y)<1 || hyp(x-S.px,y-S.py)<0.6) { flash('No room there.'); return; }
    addObj(tool==='bag'?{cls:'backpack',x,y,sx:0.45,sy:0.32,h:0.18}:{cls:'box',x,y,sx:0.55,sy:0.55,h:0.6});
    flash(tool==='bag'?'Dropped a backpack lying flat (0.18 m): below the LiDAR, only the camera can see it.':'Dropped a 0.6 m box: the LiDAR will see it.');
  } else {
    if (inside(0.35) || hyp(x-S.x,y-S.y)<1.2) { flash('Pick open floor at least 1.2 m from the rover for a walker.'); return; }
    const reach=(dx,dy)=>{ let r=2.4; for (const b of STATIC) { const t=rayBox(x,y,dx,dy,b,r); if (t<r) r=t; } return Math.max(0,Math.min(2,r-0.4)); };
    const ex=[reach(1,0),reach(-1,0)], ey=[reach(0,1),reach(0,-1)], tx=ex[0]+ex[1], ty=ey[0]+ey[1];
    const useX = ty<1 ? true : tx<1 ? false : tx<ty, e=useX?ex:ey;
    if (e[0]+e[1]<1) { flash('Too tight for a walker there.'); return; }
    addWalker(useX?[[x-e[1],y],[x+e[0],y]]:[[x,y-e[1]],[x,y+e[0]]], 0.8, (nextId%5)+1, true, 60);
    flash('A person is walking back and forth there for a minute.');
  }
}

/* ================= Scenario loading & main loop ================= */
function loadScenario(id) {
  SC=SCEN.find(s=>s.id===id && s[WORLD_ID]) || SCEN[0];
  fillScenarioList();
  $('selScen').value=SC.id;
  SV=Object.assign({}, SC[WORLD_ID], {goals:SC[WORLD_ID].goals.slice()});
  buildWorld(WORLD_ID, SV.start.floor||0); if (G) showLevel(LEVEL);
  S=fresh(SC,SV); S.mode=SC.intro?'intro':'ready'; initDoors();
  if (synth) try { synth.cancel(); } catch(e) {}
  hap.shot=null; hap.setPattern(P.NONE);
  sense(); camDetect();
  say('start',true);
  tl('event', `${WORLD.name} · ${SC.title}: ${PROF[S.profile].name} takes the handle.`);
  if (G) { for (const [,H] of G.walkerM) G.scene.remove(H.g); G.walkerM.clear(); for (const [,m] of G.objM) G.scene.remove(m); G.objM.clear(); G.pathKey=''; G.camPos.set(S.px-Math.cos(S.th)*4,4,-(S.py-Math.sin(S.th)*4)); }
  renderVocab(); renderScenario(); S.tlDirty=true;
  try { localStorage.setItem('gdr-scen', SC.id); localStorage.setItem('gdr-world', WORLD_ID); } catch(e) {}
}
function decideHaptics() {
  const v2=cueMode==='v2', prof=PROF[S.profile];
  hap.guide(S.cmdV,S.cmdW, v2&&prof.quiet);
  let want=P.NONE;
  if (S.linkLost) want=P.LOST;
  else if (S.hold) want=P.STOP;
  else if (S.stopReason==='obstacle' && S.cmdV<0.02 && !(v2 && S.waitT>1.5)) want=P.STOP;
  else if (S.mode==='backup') want=P.BACKUP;
  else if (v2 && S.inPlace && S.mode==='nav') want=P.TURN;
  else if (S.obstUrg>0 && S.cmdV>0.02) { want=P.OBSTACLE; hap.urg=S.obstUrg; }
  hap.setPattern(want);
}
let speed=2, paused=false, last=performance.now(), frame=0, feedFrames=0, feedT0=performance.now();
function simStep(dt) {
  S.t+=dt;
  if (S.t-S.lastScanT>=0.1) { S.lastScanT=S.t; sense(); }
  if (S.t-S.lastCamT>=0.2) { S.lastCamT=S.t; camDetect(); }
  S.ctrlAcc+=dt;
  if (S.ctrlAcc>=0.1) { const c=S.ctrlAcc; S.ctrlAcc=0; scriptTick(c); control(c); psychTick(c); think(); decideHaptics();
    if (!S.hist.length || S.t-S.hist[S.hist.length-1].t>=1) S.hist.push({t:S.t,trust:S.psy.trust,stress:S.psy.stress}); }
  physics(dt);
}
function tick(now) {
  const real=Math.min(0.1,(now-last)/1000); last=now;
  if (!paused) { let left=real*speed; while (left>1e-6) { const dt=Math.min(0.02,left); left-=dt; simStep(dt); } }
  hap.update(now);
  if (G) {
    sync3D(now, real); placeCamera();
    const vp=$('vp'), w=vp.clientWidth, h=vp.clientHeight;
    if (G.w!==w||G.h!==h) { G.w=w; G.h=h; G.renderer.setSize(w,h,false); G.cam.aspect=w/h; G.cam.updateProjectionMatrix(); }
    setWalls(layers.cut && G.view!=='pov'); G.renderer.render(G.scene,G.cam);
    if (frame%3===0) { setWalls(false); G.feedR.render(G.scene,G.feedCam); feedFrames++; drawFeedOverlay(); }
    if (now-feedT0>1000) { feedFps=Math.round(feedFrames*1000/(now-feedT0)); feedFrames=0; feedT0=now; }
  }
  renderGrip();
  if (frame%3===0) { renderPerson(); renderThink(); renderStatus(); drawPolar(); }
  if (frame%15===0) { renderSensors(); drawChart(); }
  if (S.uiDirty) { S.uiDirty=false; renderScenario(); }
  frame++;
  requestAnimationFrame(tick);
}

/* ================= Controls ================= */
function fillScenarioList() { const list=SCEN.filter(s=>s[WORLD_ID]); const html=list.map((s,i)=>`<option value="${s.id}">${i+1}. ${esc(s.title)} · ${PROF[s.profile].name}</option>`).join(''); if (cache.scenList!==html) { cache.scenList=html; $('selScen').innerHTML=html; } }
$('selScen').addEventListener('change', e=>loadScenario(e.target.value));
$('btnRestart').addEventListener('click', ()=>loadScenario(SC.id));
$('btnPlay').addEventListener('click', ()=>{ paused=!paused; $('btnPlay').textContent=paused?'Play':'Pause'; });
document.querySelectorAll('[data-speed]').forEach(b=>b.addEventListener('click',()=>{ speed=+b.dataset.speed; document.querySelectorAll('[data-speed]').forEach(x=>x.setAttribute('aria-pressed',String(x===b))); }));
document.querySelectorAll('[data-cue]').forEach(b=>b.addEventListener('click',()=>{ cueMode=b.dataset.cue; document.querySelectorAll('[data-cue]').forEach(x=>x.setAttribute('aria-pressed',String(x===b))); renderVocab(); tl('event', cueMode==='v2'?'Switched to Adaptive cues.':'Switched to the original firmware cues (haptics.cpp).'); }));
document.querySelectorAll('[data-view]').forEach(b=>b.addEventListener('click',()=>{ if (G) setView(b.dataset.view); }));
document.querySelectorAll('[data-tool]').forEach(b=>b.addEventListener('click',()=>{ tool=b.dataset.tool; document.querySelectorAll('[data-tool]').forEach(x=>x.setAttribute('aria-pressed',String(x===b))); }));
for (const [id,k] of [['lyLidar','lidar'],['lyPath','path'],['lyCost','cost'],['lyCam','cam'],['lyCut','cut']]) $(id).addEventListener('change',e=>{ layers[k]=e.target.checked; if (k==='cost') costDirty=true; });
function switchWorld(w) {
  document.querySelectorAll('[data-world]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.world===w)));
  buildWorld(w); buildScene(w); loadScenario(SC?SC.id:'cane');
}
document.querySelectorAll('[data-world]').forEach(b=>b.addEventListener('click',()=>{ if (b.dataset.world!==WORLD_ID) switchWorld(b.dataset.world); }));
$('selUnits').addEventListener('change', e=>{ unitMode=e.target.value; try { localStorage.setItem('gdr-units',unitMode); } catch(err) {} tl('event', 'The rover now says distances like this: '+sayDist(2.4)+'.'); });
$('selDir').addEventListener('change', e=>{ dirMode=e.target.value; try { localStorage.setItem('gdr-dir',dirMode); } catch(err) {} tl('event', 'The rover now says directions like this: a chair '+sayDir(-1.4)+'.'); });
try { const u=localStorage.getItem('gdr-units'), d=localStorage.getItem('gdr-dir'); if (u) { unitMode=u; $('selUnits').value=u; } if (d) { dirMode=d; $('selDir').value=d; } } catch(e) {}
$('selVoice').addEventListener('change', e=>{ speechMode=e.target.value; tl('event', speechMode==='off'?'Rover voice off: guidance through the grip only.':speechMode==='key'?'Rover voice: key moments.':'Rover voice: key moments and room descriptions.'); });
$('chkSpeak').addEventListener('change', e=>{ speakAloud=e.target.checked; if (!synth) { tl('event','This browser has no speech output; spoken lines still appear in the timeline.'); return; } if (!speakAloud) synth.cancel(); else utter('Voice on.','rover',2); });
$('chkInner').addEventListener('change', e=>{ narrateInner=e.target.checked; if (narrateInner && synth) utter('Inner voice on.','inner',2); });
$('chkSound').addEventListener('change', e=>{ soundOn=e.target.checked; if (soundOn) { initAudio(); audio&&audio.ctx.resume&&audio.ctx.resume(); } });
$('btnHold').addEventListener('click', ()=>{ S.hold=!S.hold; $('btnHold').textContent=S.hold?'Release':'Hold'; });
$('btnLink').addEventListener('click', ()=>{ setLink(!S.linkLost); $('btnLink').textContent=S.linkLost?'Restore link':'Cut link'; });

let startId='cane', startW='house'; try { startId=localStorage.getItem('gdr-scen')||'cane'; startW=localStorage.getItem('gdr-world')||'house'; } catch(e) {}
if (!SCEN.some(s=>s.id===startId)) startId='cane'; if (!WORLDS[startW]) startW='house';
buildWorld(startW);
try { init3D(); } catch(e) { G=null; $('vpMsg').hidden=false; $('vpMsg').textContent='This computer could not start WebGL for the 3D view. The simulation, sensors, voice and reasoning still run.'; }
buildScene(startW);
document.querySelectorAll('[data-world]').forEach(b=>b.setAttribute('aria-pressed',String(b.dataset.world===startW)));
$('selScen').value=startId; loadScenario(startId);
requestAnimationFrame(t=>{ last=t; requestAnimationFrame(tick); });
window.__sim = { advance(sec){ for (let t=0;t<sec;t+=0.02) simStep(0.02); hap.update(performance.now()); renderGrip(); renderPerson(); renderThink(); renderStatus(); renderSensors(); renderScenario(); return {t:S.t,x:S.x,y:S.y,px:S.px,py:S.py,mode:S.mode,goal:S.goal&&S.goal.label,trust:S.psy.trust,stress:S.psy.stress,stats:S.stats}; },
  load: loadScenario, world: switchWorld, get S(){ return S; }, voice(m){ speechMode=m; $('selVoice').value=m; }, setCue(m){ document.querySelector(`[data-cue="${m}"]`).click(); },
  check(){ let worst=9; const p=personClr(S.px,S.py,STATIC,S.objs,S.walkers,0), r=roverClr(S.x,S.y,S.th,STATIC,S.objs,S.walkers,0); return {person:p, rover:r}; } };
})();
