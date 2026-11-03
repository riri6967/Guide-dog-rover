
/* ================= Rover voice: a small speaker on the rover the person hears ================= */
// speechMode: what the rover says (it drives the person model); speakAloud: whether *you* hear it through the browser
let speechMode='key', speakAloud=false, narrateInner=false;
// how distances and directions are put into words
let unitMode='m+steps', dirMode='lr';
const synth = ('speechSynthesis' in window) ? window.speechSynthesis : null;
let voices=[];
function loadVoices(){ try { voices = synth ? synth.getVoices() : []; } catch(e) { voices=[]; } }
if (synth) { loadVoices(); try { synth.addEventListener('voiceschanged', loadVoices); } catch(e) {} }
function pickVoice(kind){
  const en=voices.filter(v=>/^en/i.test(v.lang)); if (!en.length) return null;
  if (kind==='inner') return en.find(v=>/female|zira|samantha|karen|susan/i.test(v.name)) || en[Math.min(1,en.length-1)];
  return en.find(v=>/google uk english male|daniel|david|mark|alex/i.test(v.name)) || en[0];
}
// a new user hears the rover a little slower; a former dog handler wants it brisk
const SPEECH_RATE = {new:0.86, cane:0.95, exp:1.05, dog:1.12};
function utter(text, kind, prio){
  if (!synth) return;
  try {
    if (prio>=2) synth.cancel(); else if ((synth.speaking||synth.pending) && prio<1) return;
    const u=new SpeechSynthesisUtterance(text), v=pickVoice(kind); if (v) u.voice=v;
    u.rate = kind==='inner' ? 1.05 : (S ? SPEECH_RATE[S.profile]||1 : 1); u.pitch = kind==='inner' ? 1.12 : 0.92; u.volume=1;
    synth.speak(u);
  } catch(e) {}
}

/* ---------- words for distances and directions ---------- */
// average step length while being guided: shorter for someone new or anxious
const STRIDE = {new:0.5, cane:0.6, exp:0.65, dog:0.7};
function sayDist(d) {
  if (!isFinite(d)) return 'some way';
  if (d<0.6) return "within arm's reach";
  const stride=STRIDE[S?S.profile:'exp']||0.65, n=Math.max(1,Math.round(d/stride));
  const steps = n===1 ? '1 step' : `${n} steps`;
  const mv = d<3 ? Math.round(d*2)/2 : Math.round(d);
  const m = mv===1 ? '1 metre' : `${mv} metres`;
  const fv = d*3.281, ft = fv<10 ? `${Math.round(fv)} feet` : `${Math.round(fv/5)*5} feet`;
  switch (unitMode) {
    case 'm': return m;
    case 'ft': return ft;
    case 'steps': return `about ${steps}`;
    case 'ft+steps': return `${ft}, about ${steps}`;
    default: return `${m}, about ${steps}`;
  }
}
// rel: angle from the person's facing, + = left. Clock face is the orientation-and-mobility standard
function sayDir(rel, verb) {
  if (dirMode==='clock') { let h=(12-Math.round(rel*6/Math.PI))%12; if (h<=0) h+=12; return `at your ${h} o'clock`; }
  const a=Math.abs(rel);
  if (a<0.4) return verb?'straight ahead':'ahead';
  if (a<1.1) return rel>0?'ahead on your left':'ahead on your right';
  if (a<2.3) return rel>0?'on your left':'on your right';
  return rel>0?'behind you on the left':'behind you on the right';
}
const handFor = rel => rel>0 ? 'left' : 'right';
const relToPerson = (x,y) => wrap(Math.atan2(y-S.py,x-S.px)-S.pth);
const cap1 = s => s.charAt(0).toUpperCase()+s.slice(1);

// prio 0 description, 1 guidance, 2 safety. kind feeds the person model.
function rsay(text, prio, kind){
  if (speechMode==='off' || !S) return false;
  if (prio===0 && speechMode!=='describe') return false;
  const gap = S.t-S.lastSpeechT;
  if (prio===0 && gap<5) return false;
  if (prio===1 && gap<1.4) return false;
  S.lastSpeechT=S.t; S.lastSpeech=text; S.lastSpeechKind=kind; S.speechLog.push(S.t); S.stats.spoken++;
  if (kind==='turn') S.lastSpokenTurn=S.t;
  if (kind==='stop'||kind==='wait') S.lastSpokenWait=S.t;
  tl('say', text);
  if (speakAloud) utter(text,'rover',prio);
  if (!S.heardSpeech) { S.heardSpeech=true; S.pendingVoice={key:'speechHeard', at:S.t+2.5}; }
  // steps are a distance the body already knows; metres have to be translated first
  const prof=PROF[S.profile];
  if (/step|arm's reach/.test(text) && (prof.needsSpeech||prof.key==='cane')) feel(null,-0.004,0.003);
  return true;
}

/* ---------- what the rover says, and when ---------- */
function routeTurns(pts) {
  const out=[]; let ref=null;
  for (let i=0;i+10<pts.length;i+=5) {
    const h=Math.atan2(pts[i+10][1]-pts[i][1], pts[i+10][0]-pts[i][0]);
    if (ref===null) { const d0=wrap(h-S.th); if (Math.abs(d0)>2.3) out.push('around'); else if (Math.abs(d0)>0.75) out.push(d0>0?'left':'right'); ref=h; continue; }
    const d=wrap(h-ref); if (Math.abs(d)>0.75) { out.push(d>0?'left':'right'); ref=h; }
  }
  return out;
}
const NUM = ['no','one','two','three','four','five','six','seven','eight','nine','ten'];
const listWords = a => a.length<2 ? a.join('') : a.slice(0,-1).join(', ')+', then '+a[a.length-1];
// ['right','right','left'] -> "right twice, then left"
function turnWords(turns) {
  const g=[]; for (const t of turns) { if (g.length && g[g.length-1].t===t && t!=='around') g[g.length-1].n++; else g.push({t,n:1}); }
  return listWords(g.map(x=>x.t==='around'?'first we turn around':x.n===1?x.t:x.n===2?`${x.t} twice`:`${x.t} ${NUM[x.n]||x.n} times`));
}
function routePreview(goal) {
  const prof=PROF[S.profile], len=S.path.length*0.1, turns=routeTurns(S.path);
  S.legLen=len; S.saidHalf=false; S.saidAlmost=false;
  if (S.goal.isLift) {
    const t=S.liftGoal;
    return rsay(prof.quiet ? `${t.label}, on ${levelSay(t.floor)}. Lift first: ${sayDist(len)}.` : `Next stop: ${t.label}, on ${levelSay(t.floor)}. First we go to the lift, ${sayDist(len)}. I cannot climb stairs, so we always use the lift.`, 1, 'start');
  }
  if (prof.quiet) return rsay(`${goal.label}. ${cap1(sayDist(len))}.`, 1, 'start');
  const n=turns.length, t = n===0 ? 'Straight ahead all the way.' : `${cap1(NUM[n]||String(n))} turn${n>1?'s':''}: ${turnWords(turns)}.`;
  if (rsay(`Next stop: ${goal.label}. ${cap1(sayDist(len))}. ${t}`, 1, 'start')) feel('routePreview',-0.03,0.02);
}
function arrivalPhrase(g, as) {
  if (!as) return `Arrived: ${g.label}.`;
  const what=as.label.toLowerCase();
  const reach = Math.abs(as.rel)<0.4 ? 'Reach straight out in front of you.' : `Reach out with your ${handFor(as.rel)} hand.`;
  return `Arrived. The ${what} is ${sayDir(as.rel)}, ${sayDist(as.d)}. ${reach}`;
}
function turnaroundPhrase(dir) {
  return `Pause. We are turning around to the ${dir>0?'left':'right'}. Walk round to your ${dir>0?'right':'left'} as I turn.`;
}
function describeArea(a) {
  const seen=new Set(), items=[];
  const list=STATIC.filter(b=>b.kind==='furn' && b.cx>=a.x0 && b.cx<=a.x1 && b.cy>=a.y0 && b.cy<=a.y1)
    .map(b=>({b, d:Math.max(0,rectDist(S.px,S.py,b)-PR), rel:relToPerson(b.cx,b.cy)})).filter(o=>o.d<6).sort((p,q)=>p.d-q.d);
  for (const o of list) { const k=o.b.label.toLowerCase(); if (seen.has(k)) continue; seen.add(k); items.push(`${k} ${sayDir(o.rel)}, ${sayDist(o.d)}`); if (items.length>=2) break; }
  if (rsay(`Entering ${a.says}.${items.length?' '+items.map(cap1).join('. ')+'.':''}`, 0, 'describe')) feel(null, PROF[S.profile].needsSpeech?-0.04:-0.015, 0.01);
}
// which way the new route passes an object, from the person's point of view
function passSide(o) {
  if (!S.path) return '';
  let best=0, bd=Infinity;
  for (let k=S.pathIdx;k<S.path.length;k++) { const d=hyp(S.path[k][0]-o.cx,S.path[k][1]-o.cy); if (d<bd) { bd=d; best=k; } }
  const a=S.path[Math.max(0,best-2)], b=S.path[Math.min(S.path.length-1,best+2)];
  const cr=(b[0]-a[0])*(o.cy-a[1])-(b[1]-a[1])*(o.cx-a[0]);
  return cr>0 ? 'keeping it on your left' : 'keeping it on your right';
}
// guidance spoken in either cue set
function speechCues(prof) {
  if (S.mode==='nav') stairsCues(prof);
  if (speechMode==='off' || S.mode!=='nav') return;
  const a=areaAt(S.x,S.y);
  if (a && S.lastArea && a!==S.lastArea) describeArea(a);
  if (a) S.lastArea=a;
  const ut=upcomingTurn(), v=Math.max(S.cmdV,0.12);
  if (ut && !S.inPlace && ut.dist/v<=prof.anticip+0.4 && S.t-S.lastSpokenTurn>5 && (!prof.quiet||Math.abs(ut.angle)>0.9))
    rsay(ut.dist<0.6 ? `${ut.side==='L'?'Left':'Right'} turn now.` : `${ut.side==='L'?'Left':'Right'} turn in ${sayDist(ut.dist)}.`, 1, 'turn');
  // the moment of turning, for people still learning the grip
  if (ut && ut.dist<0.45 && S.t-S.lastSpokenTurn<9 && S.t-(S.lastTurnNow||-9)>6 && (prof.needsSpeech||prof.key==='cane'))
    if (rsay(`Turning ${ut.side==='L'?'left':'right'} now.`, 1, 'turn')) S.lastTurnNow=S.t;
  const na=narrowAhead();
  if (na && na.dist<1.8 && S.t-(S.lastSpokenDoor||-9)>10 && S.cmdV>0.05 && (prof.info||prof.needsSpeech) && !(S.doorAheadInfo && S.doorAheadInfo.dist<2.2) && !(areaAt(S.x,S.y)||{}).name?.startsWith('Lift car')) {
    const dr=DOORS.map(d=>({d, e:hyp(d.x-na.x,d.y-na.y)})).filter(o=>o.e<1.6).sort((p,q)=>p.e-q.e).map(o=>o.d)[0], tight=na.width<1.6;
    if (rsay(`${dr?cap1(dr.label):'A gap'} in ${sayDist(na.dist)}.${tight?' It is narrow, stay close behind me.':''}`, 1, 'door')) S.lastSpokenDoor=S.t;
  }
  // stopping, waiting, and setting off again
  if (S.stopReason==='obstacle' && S.cmdV<0.05 && S.t-S.lastSpokenWait>7) {
    const near=S.gap<0.6;
    rsay(S.waitWhy==='walker' ? (near?'Stop. Someone is crossing right in front of us. I will wait.':`Stop. Someone is crossing ${sayDist(S.gap)} ahead. I will wait.`)
                              : (near?'Stop. Something is right in front of us. Finding a way around.':`Stop. Something is ${sayDist(S.gap)} ahead. Finding a way around.`), 2, 'stop');
    S.saidWaiting=true;
  } else if (S.waitT>4 && S.t-S.lastSpokenWait>8) {
    rsay(S.waitWhy==='person' ? 'Take your time. Step round with the handle.' : 'Still waiting. The way is not clear yet.', 1, 'wait'); S.saidWaiting=true;
  } else if (S.saidWaiting && S.cmdV>0.1) { rsay('Clear now. Walking on.', 1, 'wait'); S.saidWaiting=false; }
  // never leave a blocked route unexplained
  if (!S.path && S.blockedT>5 && !S.saidNoRoute) { if (rsay('The way is blocked and I cannot find a safe route. Please wait here with me.', 2, 'stop')) S.saidNoRoute=true; }
  if (S.path) S.saidNoRoute=false;
  // people passing close
  const wk=S.walkers.find(w=>hyp(w.x-S.px,w.y-S.py)<1.3 && hyp(w.vx,w.vy)>0.2);
  if (wk && S.t-(S.lastSpokenPass||-9)>9 && (prof.info||prof.needsSpeech))
    if (rsay(`Someone passing ${sayDir(relToPerson(wk.x,wk.y))}.`, 1, 'wait')) S.lastSpokenPass=S.t;
  // progress on longer legs, so the distance left is never a mystery
  if (S.path && S.legLen>6 && !prof.quiet) {
    const left=(S.path.length-S.pathIdx)*0.1;
    if (!S.saidHalf && left<S.legLen/2) { if (rsay(`Halfway. ${cap1(sayDist(left))} to go.`, 1, 'progress')) { S.saidHalf=true; feel('progress',-0.02,0.01); } }
    else if (!S.saidAlmost && left<2.2 && left>0.8) { if (rsay(`Almost there. ${cap1(sayDist(left))} to go.`, 1, 'progress')) S.saidAlmost=true; }
  }
}
Object.assign(VOICE, {
  routePreview:{cane:["It told me the whole route before we moved. Now I have a picture in my head."], new:["It told me how far, and which way we'll turn. I can hold that in my head."], exp:["Route preview. Good."]},
  progress:{cane:["Halfway. Good to know where I am."], new:["Halfway. I can do the other half."], exp:[]},
});
