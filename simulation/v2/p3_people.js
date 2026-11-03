
/* ================= People ================= */
// walk: how fast they step round; react: seconds before responding to an unannounced change; needsSpeech: has not learned the grip yet
const PROF = {
  cane: {key:'cane', name:'Ravi', init:'R', tag:'Long-cane user for 12 years · first days on the rover',
    trust0:0.30, stress0:0.50, base:0.24, pace:0.45, walk:0.70, react:0.95, info:true, quiet:false, cueTol:18, vcapLow:0.16, anticip:2.2,
    chips:['Wants to know what is around','Used to setting the pace','Reacts in ~1 s to surprises']},
  dog: {key:'dog', name:'Sam', init:'S', tag:'Guide dog handler for 8 years · Juno retired last month',
    trust0:0.45, stress0:0.30, base:0.18, pace:1.05, walk:1.0, react:0.5, info:false, quiet:true, cueTol:8, vcapLow:0.30, anticip:1.3, talkTol:4,
    chips:['Expects ~1.2 m/s','Reads direction from the handle','Finds constant buzzing noisy']},
  exp: {key:'exp', name:'Lena', init:'L', tag:'Rover user for 3 months',
    trust0:0.78, stress0:0.15, base:0.10, pace:0.70, walk:0.9, react:0.45, info:true, quiet:false, cueTol:14, vcapLow:0.30, anticip:1.6,
    chips:['Knows the vocabulary','Relaxed at full speed']},
  new: {key:'new', name:'Asha', init:'A', tag:'Lost their sight four months ago · no cane training yet',
    trust0:0.22, stress0:0.68, base:0.30, pace:0.32, walk:0.45, react:1.3, info:true, quiet:false, cueTol:24, vcapLow:0.12, anticip:2.8, needsSpeech:true,
    chips:['Still pictures the room','Afraid of tripping','Needs words, not just buzzes','Self-conscious around people']},
};

const VOICE = {
  start:{cane:["Twelve years of tapping a cane, and now I'm holding a handle. I can't feel the floor anymore.","My hand keeps wanting to sweep left and right. There's nothing to sweep."],
         dog:["No warm body at my knee. Just a handle and a motor hum.","Juno would already be leaning toward the door."],
         exp:["Same handle as yesterday. Let's go."],
         new:["Four months ago I could see this room. Now I keep turning my head toward sounds.","I haven't walked anywhere without someone's arm since it happened."]},
  intro:{cane:["Front. I can feel exactly where that is.","Left side, like someone tapping my palm.","Right. Different spot, easy to tell apart.","Back means slow down or step back. Got it.","Two quick taps: a turn is coming. So it warns me first.","All four, hard. That's unmistakable: stop.","And nothing means all clear. I like that it's quiet."],
         new:["A voice. Okay. It's talking to me, not about me.","It says it will tell me everything before it moves.","Steps. I can picture steps better than metres.","Front. Near my thumb.","Left... yes, left.","Right. They feel different.","Back. Slow down.","Two taps means a turn is coming. I'll try to remember.","All of them at once. Stop. That one I won't forget.","Quiet means fine. I can live with quiet."]},
  introDone:{cane:["Okay. Four spots, each with a meaning. I can work with that."], new:["I'm scared. But I want to try."]},
  turnCued:{cane:["A double tap on the left... and there's the turn. It warned me first.","It told me before it moved. That's what I needed.","Tap, then turn. I'm starting to trust the rhythm."],
            dog:["Tap, then turn. Juno used to drop her shoulder just like that.","Fair enough, a heads-up before the turn."],
            exp:["Right on cue."], new:["It said there'd be a turn, and there it is. Okay.","Warned, then turned. I can do this."]},
  turnSurprise:{cane:["Whoa! It swung me without any warning.","Where are we going? It just turned under my hand."],
                dog:["That was abrupt. A dog eases into a turn.","It yanked sideways. Juno never did that."],
                exp:["Bit of a sudden turn there."], new:["Wait, wait! Where are we going?","No, no, it just pulled me sideways."]},
  stopSurprise:{cane:["It stopped dead. Is something there?","Why did we stop? I didn't feel anything."],
                dog:["Sudden stop. Juno only did that for a real hazard."], exp:["Hard stop. Unusual."],
                new:["Why did it stop? Is there a step? I hate not knowing."]},
  stopExplained:{cane:["Front pulsing faster, then the stop. Something's ahead and it saw it first."],
                 dog:["It refused to go on, and told me why. Like a dog's intelligent disobedience."],
                 exp:["Obstacle, stopping. Fine."], new:["It told me something's in front. Fine. I can wait."]},
  waitConfused:{cane:["We've been standing here a while. Is it broken? Am I in the way?"],
                dog:["Why are we waiting? Juno would at least shift her weight."], exp:["Waiting... for what?"],
                new:["We're just standing here. Everyone must be staring at me."]},
  waitReassured:{cane:["Slow pulses, like a heartbeat. It's waiting on purpose, it hasn't frozen."],
                 dog:["Fine, it's letting me know it's still working."], exp:["Heartbeat pulse. It's just waiting."],
                 new:["It keeps telling me it's waiting. That helps more than I expected."]},
  closeCall:{cane:["Something was right beside my hip. With a cane I'd have found it first.","I could feel the air change. Something's very close on my side."],
             dog:["Tight squeeze."], exp:["That was close to my shoulder."], new:["Something's right next to me. I can feel it."]},
  sideInfo:{cane:["A tap on the side as we passed. So there's something there. Good to know."], exp:["Passing something on my side, noted."], new:["A tap on the side... something there. Okay."]},
  doorInfo:{cane:["Both sides at once. A doorway. I can almost picture the frame."], new:["A doorway. I remember this doorway."]},
  tooSlow:{dog:["Juno would have been at the door by now.","This pace is killing me. I walk faster than this on my own.","Slow, but I suppose it never gets distracted by a sandwich."]},
  tooMany:{dog:["Buzz, buzz, buzz. I can feel the handle turn; I don't need a running commentary."], cane:["That's a lot of signals at once."], exp:["Chatty today."]},
  tooTalky:{dog:["It won't stop talking. I just want to walk."]},
  dragged:{cane:["Too fast. It's pulling me.","Slow down, I'm not ready."], dog:["Easy..."], exp:["It's dragging a bit."], new:["Too fast, please, too fast."]},
  slowedForMe:{cane:["It slowed down when I hesitated. It's paying attention to me."], dog:["At least it noticed when I eased off."], exp:["It matched my pace."], new:["It slowed down for me. Thank you."]},
  freeze:{cane:["I'm stopping. I don't know where I am."], dog:["Hold on. I need a second."], exp:["Let me stop a sec."], new:["I can't. I need to stop."]},
  unfreeze:{cane:["Okay. Breathe. Let's keep going."], dog:["Right, carry on."], exp:["Okay, go."], new:["Okay... one more try."]},
  flow:{cane:["Silence. Straight ahead. My shoulders are dropping a bit.","This is actually smooth."],
        dog:["The rhythm isn't bad. Different from Juno, not worse.","Smooth enough."], exp:["Easy walking."],
        new:["We're actually walking. On my own, sort of.","I'm not holding anyone's elbow. Just a handle."]},
  turnaround:{cane:["The grip sweeps round my hand, so I walk round with it."], dog:["Turning on the spot. Juno would circle me instead. Odd, but clear."],
              exp:["Turnaround. Walk round it."], new:["It said we're turning around. I'll follow the handle."]},
  turnaroundConfused:{cane:["It's spinning and dragging my arm sideways. Which way do I go?"], dog:["What's it doing? I'm getting pulled sideways."], exp:["Spinning without warning."], new:["It's pulling me in a circle. I don't understand."]},
  linkLost:{cane:["Three short pulses... the handle has lost the rover. I'm not moving."], dog:["Signal's gone. Standing still."], exp:["Link lost. Stay put."], new:["What's happening? Something's wrong."]},
  linkBack:{cane:["It's back. Quiet again."], dog:["Back on."], exp:["Back."], new:["It's back. Okay. Breathe."]},
  personNear:{cane:["Footsteps right in front. Glad it stopped."], dog:["Someone crossing. Juno would have waited too."], exp:["Someone walking past."], new:["People walking past. Do they think I'm helpless?"]},
  passedBy:{cane:["Footsteps passing close. The rover kept to its side."], dog:["Busy. Juno would have threaded through this."], exp:["Busy today."], new:["So many people. I hope I'm not in their way."]},
  replan:{cane:["We're curving around something I can't feel."], dog:["Detour. Fine."], exp:["Going around something."], new:["We're going around something. It said so."]},
  arrived:{cane:["Two soft pulses. We're here. My heart's still racing, but we made it.","We made it, and nothing touched me."],
           dog:["We're here. Slower than Juno, but steady."], exp:["Arrived."], new:["We're here. I did that.","Arrived. I didn't need anyone's arm."]},
  arrivedSide:{cane:["And a tap telling me which side it's on. No searching with my hands."], dog:["It even told me which side it's on."], exp:["Handy, telling me the side."], new:["It told me where it is. I can reach for it."]},
  speechHeard:{cane:["It talks too. Handy when I'm not sure what a buzz meant."], dog:["It talks. Juno never needed to, but fine."], exp:["Voice on. Useful in new places."], new:["Its voice tells me what's around. Like having my eyes narrate."]},
  speechMissing:{new:["A tap on my hand... what does that one mean again?","It buzzed, but I don't know what it wants."]},
};
function voiceLine(key, prof) {
  const v=VOICE[key]; if (!v) return null;
  const arr=v[prof]||v.any; if (!arr||!arr.length) return null;
  const n=(S.voiceCount[key]=(S.voiceCount[key]||0)+1)-1;
  return arr[n%arr.length];
}

/* ================= Scenarios: same stories in both worlds ================= */
const W = (x,y)=>[x,y];
const SCEN = [
  {id:'cane', title:'First walk after the cane', profile:'cane', intro:true,
   brief:{house:'Ravi has used a long cane for 12 years and is trying the rover for the first time: a straight walk along the corridor, then into Room 1 to the table.',
          lab:'Ravi works in the lab and tries the rover on the way in: straight along the corridor, then through the lab door and between the desk rows to a workstation.'},
   why:['A cane reports the ground one step ahead. The rover asks Ravi to give that up.','Control moves from Ravi\'s hand to the machine. Trust comes from being predictable, not from being fast.','So the rover teaches each motor first, warns before every move, and keeps silence meaning "all clear".'],
   house:{start:{x:3,y:0,th:Math.PI}, goals:[{x:-3,y:0,label:'Straight walk along the corridor'},{x:-5,y:4.1,label:'Room 1, by the table',near:'Table'}]},
   lab:{start:{x:4.0,y:-4.3,th:Math.PI}, goals:[{x:-1.5,y:-4.3,label:'Straight along the corridor'},{x:-6.6,y:1.5,label:'Desk row 2, by the monitors',near:'Desk'}]}},
  {id:'newblind', title:'Recently blind: first steps', profile:'new', intro:true,
   brief:{house:'Asha lost their sight four months ago and has not had cane training yet. This is the first walk in their own home without holding someone\'s arm: out of the guest room and across to the living room.',
          lab:'Asha lost their sight four months ago and is back at work for the first time. The rover takes Asha from the corridor to their own desk.'},
   why:['Asha still thinks visually: they turn their head toward sounds and try to picture the room.','The grip vocabulary is new and easily forgotten under stress, so words matter more than buzzes for now.','Fear of tripping and of looking helpless in front of others is often stronger than any obstacle.','So the rover talks, goes slowly, explains every stop, and never lets a silence go unexplained.'],
   house:{start:{x:5.5,y:-5.2,th:Math.PI/2}, goals:[{x:4.5,y:0,label:'Out into the corridor'},{x:3.2,y:4.7,label:'Living room, by the armchair',near:'Armchair'}]},
   lab:{start:{x:4.0,y:-4.3,th:Math.PI}, goals:[{x:-1.5,y:-4.3,label:'Along the corridor'},{x:-6.6,y:5.1,label:'Your desk, north aisle',near:'Desk'}]}},
  {id:'dog', title:'Switching from a guide dog', profile:'dog',
   brief:{house:'Sam worked with guide dog Juno for 8 years until Juno retired last month. Sam walks the length of the corridor and into Room 2.',
          lab:'Sam worked with guide dog Juno for 8 years until Juno retired last month. Sam crosses the building to a meeting.'},
   why:['A dog walks at about 1.2 m/s; the rover tops out at 0.3 m/s. Expect frustration.','Handlers read direction from the harness itself, so constant buzzing and talking feel like noise.','No warmth or companionship, and a mechanical stop can feel like the dog refusing.'],
   house:{start:{x:-8.5,y:0,th:0}, goals:[{x:5,y:-4.2,label:'Room 2, middle'}], walkers:[{at:{t:9}, path:[W(9.5,0.95),W(-9.5,0.95)], speed:0.95, look:2}]},
   lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[{x:5.0,y:1.2,label:'Meeting room, corner seat',near:'Chair'}], walkers:[{at:{t:10}, path:[W(9.6,-3.45),W(-9.6,-3.45)], speed:0.95, look:2}]}},
  {id:'crowd', title:'People walking by', profile:'cane', trust0:0.5, stress0:0.35,
   brief:{house:'A few weeks in. Ravi walks the corridor while the household is busy: people overtake, come the other way, and cut across.',
          lab:'A few weeks in. Ravi heads to a desk at the start of the working day: colleagues walk the aisles, come down the corridor and stop to let the rover past.'},
   why:['Footsteps all around make it hard to tell which ones matter.','Being overtaken or brushed past feels exposing when you cannot see who it is.','The rover keeps to one side, waits for people instead of weaving, and tells Ravi when someone is passing.'],
   house:{start:{x:-8.5,y:0,th:0}, goals:[{x:8.6,y:0,label:'East end of the corridor'}],
          walkers:[{at:{t:6}, path:[W(-9.5,-0.95),W(9.5,-0.95)], speed:1.1, look:1},{at:{t:10}, path:[W(9.5,0.95),W(-9.5,0.95)], speed:1.0, look:3},
                   {at:{roverX:-3}, path:[W(0.1,3.4),W(0.1,1.3),W(0.45,-1.3),W(0.45,-3.2)], speed:0.8, look:4},{at:{t:22}, path:[W(-5,3),W(-5,1.0),W(9.5,0.95)], speed:1.0, look:5},
                   {at:{t:30}, path:[W(9.5,-0.95),W(-9.5,-0.95)], speed:0.9, look:2}]},
   lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[{x:-0.7,y:5.1,label:'Desk row 1, north aisle',near:'Desk'}],
        walkers:[{at:{t:2}, path:[W(9.6,-3.45),W(-9.6,-3.45)], speed:1.0, look:3},{at:{t:4}, path:[W(-9.3,1.5),W(2.0,1.5)], speed:0.8, look:1, loop:true},
                 {at:{t:12}, path:[W(-3.8,-2.2),W(-3.8,5.1)], speed:0.9, look:4, loop:true},{at:{t:20}, path:[W(2.2,5.1),W(-9.3,5.1)], speed:0.9, look:5}]}},
  {id:'busy', title:'Busy corridor', profile:'exp',
   brief:{house:'Lena crosses the house at a busy moment: one person cuts across the corridor, another walks past the other way, a third comes out of Room 2.',
          lab:'Lena walks the lab corridor at lunchtime: someone heads from the lab to the printer, another comes the other way, a third leaves the meeting room.'},
   why:['People have right of way. Squeezing past with a person on the handle leaves no margin.','A long wait with no explanation feels like a breakdown.','So the rover stops early and pulses a slow heartbeat while it waits.'],
   house:{start:{x:-9,y:0,th:0}, goals:[{x:8.6,y:0,label:'East end of the corridor'}],
          walkers:[{at:{roverX:-2.2}, path:[W(0.5,-3.2),W(0.5,-1.3),W(0.1,1.3),W(0.1,3.6)], speed:0.8, look:1},{at:{roverX:-1.0}, path:[W(9.5,1.0),W(-9.5,1.0)], speed:1.0, look:3},
                   {at:{roverX:1.8}, path:[W(5,-3.6),W(5,-1.6),W(6.5,-0.9),W(11,-0.9)], speed:0.8, look:4}]},
   lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[{x:8.8,y:-4.3,label:'East end of the corridor'}],
        walkers:[{at:{roverX:-3.5}, path:[W(0.75,-1.6),W(0.75,-3.4),W(0.2,-4.9),W(-0.9,-6.8)], speed:0.8, look:1},{at:{roverX:-1}, path:[W(9.6,-3.45),W(-9.6,-3.45)], speed:1.0, look:3},
                 {at:{roverX:2.5}, path:[W(6.75,-1.6),W(6.75,-3.6),W(9.6,-4.9)], speed:0.8, look:4}]}},
  {id:'bag', title:'A bag in the doorway', profile:'cane', trust0:0.55, stress0:0.32,
   brief:{house:'Two weeks in. Ravi heads into Room 2, and someone has left a backpack lying flat in the doorway. At 0.18 m it is below the LiDAR\'s scan plane.',
          lab:'Two weeks in. Ravi heads into the lab, and someone has left a backpack lying flat in door B. At 0.18 m it is below the LiDAR\'s scan plane.'},
   why:['The LiDAR scans at 0.23 m, so a bag lying flat is invisible to it. Only the camera can see it.','Ravi walks behind the rover, so the detour has to clear Ravi\'s feet too, not just the rover.','A cane user would normally find the bag with the cane; now Ravi relies on the rover to.'],
   house:{start:{x:-2,y:0,th:0}, goals:[{x:5,y:-4.2,label:'Room 2, past the doorway'}], events:[{at:{roverX:0.6}, spawn:{cls:'backpack',x:4.45,y:-1.55,sx:0.45,sy:0.32,h:0.18}, say:'Someone has left a backpack in the Room 2 doorway.'}]},
   lab:{start:{x:-4.5,y:-4.3,th:0}, goals:[{x:2.0,y:1.0,label:'East aisle, by the electronics bench',near:'Electronics bench'}], events:[{at:{roverX:-2.6}, spawn:{cls:'backpack',x:0.95,y:-3.05,sx:0.45,sy:0.32,h:0.18}, say:'Someone has left a backpack in lab door B.'}]}},
  {id:'chair', title:'Find a seat', profile:'cane', trust0:0.6, stress0:0.28,
   brief:{house:'Ravi wants to sit at the Room 1 table. The rover parks so the chair ends up right beside Ravi, and says which side it is on.',
          lab:'Ravi needs a free desk. The rover finds the chair someone left pulled out and parks so it ends up beside Ravi.'},
   why:['Arriving "near" a chair still means searching with your hands.','The useful target is where the person stands, not where the rover stops.'],
   house:{start:{x:-1.6,y:0,th:Math.PI}, goals:[{x:-4.25,y:6.45,label:'Beside the chair in Room 1',near:'Chair'}]},
   lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[{x:-0.7,y:1.75,label:'Beside the free chair, desk row 2',near:'Free chair'}]}},
  {id:'link', title:'Handle link drops', profile:'exp',
   brief:{house:'Lena is on the way to Room 1 when the radio link between the rover and the handle drops for a few seconds.',
          lab:'Lena is on the way to the server room when the radio link between the rover and the handle drops for a few seconds.'},
   why:['The handle going silent would normally mean "all clear". Here it could mean the link is gone.','So link loss gets its own unmistakable pattern, and the rover stops until the link is back.'],
   house:{start:{x:6,y:0,th:Math.PI}, goals:[{x:-5,y:4.1,label:'Room 1'}], events:[{at:{navT:13}, link:true, say:'Radio interference: the handle stops receiving packets.'},{at:{navT:20}, link:false, say:'Link restored.'}]},
   lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[{x:6.3,y:4.6,label:'Server room'}], events:[{at:{navT:16}, link:true, say:'Radio interference near the server racks path: the handle stops receiving packets.'},{at:{navT:23}, link:false, say:'Link restored.'}]}},
  {id:'free', title:'Free roam', profile:'exp', free:true,
   brief:{house:'Your house, your rules. Click the floor to send the rover somewhere, drop a flat bag or a tall box, or add people walking around.',
          lab:'Your lab, your rules. Click the floor to send the rover somewhere, drop a flat bag or a tall box, or add people walking around.'},
   why:['Drop a bag to see camera-only detection, or a box to see the LiDAR pick it up.','Switch between Adaptive cues and Original firmware, or turn the rover voice on and off, to compare how the person copes.'],
   house:{start:{x:0,y:0,th:0}, goals:[]}, lab:{start:{x:-8.5,y:-4.3,th:0}, goals:[]}},
];

/* ================= Haptics ================= */
const P = {NONE:0, STOP:1, OBSTACLE:2, BACKUP:3, LOST:4, ARRIVED:5, TURN:6};
const PNAME = ['NONE','STOP','OBSTACLE','BACKUP','LOST','ARRIVED','TURNAROUND'];
const fr = (a,b,F,B,L,R) => [a,b,F,B,L,R];
const SHOT = {
  preTurnL:{dur:420, f:[fr(0,130,0,0,210,0), fr(260,390,0,0,210,0)], name:'Left double tap', say:'A left turn is coming.'},
  preTurnR:{dur:420, f:[fr(0,130,0,0,0,210), fr(260,390,0,0,0,210)], name:'Right double tap', say:'A right turn is coming.'},
  slow:{dur:200, f:[fr(0,170,0,150,0,0)], name:'Back tap', say:'Slowing down.'},
  door:{dur:240, f:[fr(0,200,0,0,150,150)], name:'Both sides tap', say:'Doorway: walls close on both sides.'},
  sideL:{dur:170, f:[fr(0,150,0,0,120,0)], name:'Left side tap', say:'Passing something on the left.'},
  sideR:{dur:170, f:[fr(0,150,0,0,0,120)], name:'Right side tap', say:'Passing something on the right.'},
  heart:{dur:420, f:[fr(0,110,140,0,0,0), fr(220,320,100,0,0,0)], name:'Heartbeat', say:'Waiting on purpose. Still working.'},
  arrived:{dur:1200, f:[fr(0,300,150,150,150,150), fr(600,900,150,150,150,150)], name:'Arrived', say:'Two soft pulses: you are there.'},
  arrivedL:{dur:1750, f:[fr(0,300,150,150,150,150), fr(600,900,150,150,150,150), fr(1250,1700,0,0,220,0)], name:'Arrived, left', say:'Arrived. The target is on your left.'},
  arrivedR:{dur:1750, f:[fr(0,300,150,150,150,150), fr(600,900,150,150,150,150), fr(1250,1700,0,0,0,220)], name:'Arrived, right', say:'Arrived. The target is on your right.'},
  introF:{dur:1100, f:[fr(0,1100,200,0,0,0)], name:'Front', say:'This is front.'},
  introL:{dur:1100, f:[fr(0,1100,0,0,200,0)], name:'Left', say:'This is left.'},
  introR:{dur:1100, f:[fr(0,1100,0,0,0,200)], name:'Right', say:'This is right.'},
  introB:{dur:1100, f:[fr(0,1100,0,200,0,0)], name:'Back', say:'This is back.'},
  introStop:{dur:1200, f:[0,200,400,600,800,1000].map(t=>fr(t,t+100,255,255,255,255)), name:'Stop', say:'All four pulsing: stop.'},
  ready:{dur:420, f:[fr(0,110,140,0,0,0), fr(220,320,100,0,0,0)], name:'Ready', say:'Ready to go.'},
};
const hap = {
  out:[0,0,0,0], pat:P.NONE, patStart:0, urg:0, guideV:[0,0,0,0], shot:null, shotKey:'', shotStart:0, sweep:1,
  setPattern(p){ if (p!==this.pat) { this.pat=p; this.patStart=performance.now(); } },
  play(key){ this.shot=SHOT[key]; this.shotKey=key; this.shotStart=performance.now(); },
  // haptics.cpp fromVelocity(); the adaptive set can stay quiet on gentle turns
  guide(lin, ang, quiet){
    const g=this.guideV; g[0]=g[1]=g[2]=g[3]=0;
    const db = quiet ? 0.35 : 0.10;
    if (Math.abs(ang)>db) { const d=70+Math.floor(80*clamp(Math.abs(ang)/1.0,0,1)); if (ang>0) g[2]=d; else g[3]=d; }
    else if (lin < -0.03) g[1]=150;
  },
  update(now){
    const t=now-this.patStart; let o=null;
    switch (this.pat) {
      case P.LOST: { const m=t%1600, on=m<120||(m>240&&m<360)||(m>480&&m<600); o=on?[255,255,255,255]:[0,0,0,0]; break; }
      case P.STOP: o = Math.floor(t/100)%2===0 ? [255,255,255,255] : [0,0,0,0]; break;
      case P.BACKUP: o=[0,255,0,0]; break;
      case P.TURN: { const m=t%1000, sd=this.sweep>0?3:2; o=[0,0,0,0]; if (m<220) o[0]=200; else if (m<440) o[sd]=200; else if (m<660) o[1]=200; break; }
    }
    if (!o && this.shot) {
      const ts=now-this.shotStart;
      if (ts>this.shot.dur) this.shot=null;
      else { o=[0,0,0,0]; for (const f of this.shot.f) if (ts>=f[0]&&ts<f[1]) o=[f[2],f[3],f[4],f[5]]; }
    }
    if (!o && this.pat===P.OBSTACLE) { const per=600-Math.floor(450*this.urg); o = Math.floor(t/(per/2))%2===0 ? [255,0,0,0] : [0,0,0,0]; }
    if (!o) o=this.guideV;
    for (let i=0;i<4;i++) this.out[i] = o[i]<70 ? 0 : o[i];
  },
};
