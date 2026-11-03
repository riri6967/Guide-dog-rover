
/* ================= Hospital: two floors joined by a bed lift and stairs ================= */
// Doors that move: hinged (the person opens them; the rover has no arm), auto (sliding, sensor), lift (driven by the lift).
// hinge: which end of the opening carries the hinge (-1 = lower x / lower y); swing: which side the leaf opens into (+1 = +y / +x).
const DOOR = (id,type,x,y,w,o) => Object.assign({id,type,x,y,w,vert:false,hinge:-1,swing:1,group:id,label:'door'}, o||{});
// shared core on both floors: main corridor, lift lobby, bed lift (2.8 x 2.7 m car, room to turn round), stairwell
const CORE_WALLS = [[-12,0,0.2,16],[12,0,0.2,16],[0,8,24,0.2],[0,-8,24,0.2],
  [-11.25,-1.5,1.5,0.2],[-4.65,-1.5,3.7,0.2],[0.6,-1.5,4.0,0.2],[5.5,-1.5,3.0,0.2],[11.5,-1.5,1.0,0.2],
  [-5,-4.75,0.2,6.5],[1,-4.75,0.2,6.5],[6,-4.75,0.2,6.5],
  [9.0,-6.55,0.2,2.9],[6.45,-5.1,0.9,0.2],[8.55,-5.1,0.9,0.2]];        // lift shaft: east wall, front wall either side of the 1.2 m door
const LIFT_DOOR = DOOR('lift','lift',7.5,-5.1,1.2,{label:'lift door'});
const L0 = {
  name:'Ground floor', spoken:'the ground floor',
  walls:[...CORE_WALLS, [-11,1.5,2,0.2],[-4.75,1.5,3.5,0.2],[0.7,1.5,4.6,0.2],[6.6,1.5,3.6,0.2],[11.4,1.5,1.2,0.2],
    [-5,4.75,0.2,6.5],[1,4.75,0.2,6.5],[7,4.75,0.2,6.5]],
  posts:[],
  furn:(()=>{
    const F=[];
    const taken=new Set(['B-11','B-9.8','B-7.3','B-6.1','A-10.4','A-9.2','A-6.7']);
    for (const [row,y] of [['A',7.35],['B',4.4]]) for (const x of [-11.0,-10.4,-9.8,-9.2,-7.9,-7.3,-6.7,-6.1]) {
      const free = row==='B' && x===-7.9;
      F.push(fu('chair',x,y,0.55,0.55,0.85,'S',free?'Free chair':'Seat','c',{color:'#2F6FDE',seated:taken.has(row+x)}));
    }
    F.push(fu('coffee',-8.55,5.9,0.6,0.5,0.45,'','Magazine table','l'));
    F.push(fu('vending',-5.55,5.6,0.8,0.8,1.8,'W','Vending machine'));
    F.push(fu('cooler',-5.5,7.5,0.35,0.35,1.2,'','Water cooler'));
    F.push(fu('plant',-11.5,2.0,0.45,0.45,1.2,'','Plant'));
    for (const x of [-3.6,-0.3]) F.push(fu('hbed',x,5.7,1.0,2.1,0.6,'S','Hospital bed','l',{patient:x<-1}));
    F.push(fu('cabinet',-4.55,6.9,0.45,0.45,0.8,'E','Bedside cabinet')); F.push(fu('cabinet',0.6,6.9,0.45,0.45,0.8,'W','Bedside cabinet'));
    F.push(fu('ivpole',-4.55,4.6,0.3,0.3,1.9,'','IV pole','l'));
    F.push(fu('overbed',-2.8,5.9,0.4,0.8,0.85,'W','Overbed table','l'));
    F.push(fu('chair',-2.6,4.3,0.6,0.6,0.9,'W','Visitor chair','c',{color:'#7A8450'}));
    F.push(fu('sinkunit',0.5,2.3,0.5,0.45,0.9,'W','Sink'));
    F.push(fu('hbed',2.2,5.7,1.0,2.1,0.6,'S','Hospital bed','l',{patient:true}));
    F.push(fu('cabinet',1.45,6.95,0.45,0.45,0.8,'E','Bedside cabinet'));
    F.push(fu('overbed',3.15,5.6,0.4,0.8,0.85,'W','Overbed table','l'));
    F.push(fu('chair',3.6,4.3,0.6,0.6,0.9,'W','Visitor chair','c',{color:'#7A8450'}));
    F.push(fu('sofa',6.45,5.0,0.8,1.8,0.8,'W','Sofa bed','s',{color:'#5F7F8C'}));
    F.push(fu('ivpole',1.45,4.4,0.3,0.3,1.9,'','IV pole','l'));
    F.push(fu('pcounter',9.6,3.0,4.0,0.7,1.05,'S','Pharmacy counter','s',{glass:true}));
    F.push(fu('bookshelf',9.5,7.6,4.5,0.45,2.0,'S','Medicine shelves','s',{meds:true}));
    F.push(fu('plant',7.45,2.0,0.45,0.45,1.2,'','Plant'));
    F.push(fu('pcounter',-8.5,-4.3,3.0,0.8,1.05,'N','Reception desk','s',{pc:true}));
    F.push(fu('ochair',-9.2,-5.2,0.55,0.55,1.0,'N','Chair','c')); F.push(fu('ochair',-7.8,-5.2,0.55,0.55,1.0,'N','Chair','c'));
    F.push(fu('plant',-11.5,-2.1,0.45,0.45,1.2,'','Plant'));
    F.push(fu('seatbench',-6.0,-7.45,1.8,0.5,0.45,'N','Bench','l'));
    F.push(fu('cabinet',-11.6,-3.4,0.4,0.6,1.4,'E','Leaflet stand'));
    F.push(fu('kiosk',-11.55,-6.0,0.5,0.6,1.5,'E','Check-in kiosk'));
    F.push(fu('trolley',-11.0,-1.1,1.0,0.5,1.0,'N','Trolley','l'));
    F.push(fu('desk',-3.5,-6.6,1.4,0.7,0.75,'S','Desk','l',{items:'pc'}));
    F.push(fu('ochair',-3.5,-7.3,0.55,0.55,1.0,'N','Doctor\'s chair','c'));
    F.push(fu('chair',-3.3,-5.7,0.55,0.55,0.9,'S','Patient chair','c',{color:'#2F6FDE'}));
    F.push(fu('couch',0.2,-5.0,0.7,1.9,0.7,'W','Examination couch','l'));
    F.push(fu('sinkunit',-4.55,-3.0,0.6,1.0,0.9,'E','Sink'));
    F.push(fu('cabinet',0.45,-7.45,0.6,0.5,1.8,'N','Supply cabinet'));
    F.push(fu('scanner',3.5,-6.0,1.9,2.6,1.9,'N','CT scanner'));
    F.push(fu('desk',5.45,-2.6,0.7,1.4,0.75,'W','Control desk','l',{items:'pc'}));
    F.push(fu('seatbench',6.4,-3.0,0.5,1.6,0.45,'E','Bench','l'));
    F.push(fu('wheelchair',11.4,-2.9,0.6,0.7,0.9,'W','Wheelchair','l'));
    F.push(fu('plant',11.5,-2.1,0.45,0.45,1.2,'','Plant'));
    F.push(fu('stairs',10.5,-6.05,2.8,3.7,2.6,'N','Stairs','s',{dir:'up'}));
    return F;
  })(),
  areas:[
    {name:'Waiting area', x0:-12, y0:1.5, x1:-5, y1:8, floor:'hosp', says:'the waiting area'},
    {name:'Patient room 1', x0:-5, y0:1.5, x1:1, y1:8, floor:'hosp2', says:'patient room 1'},
    {name:'Patient room 2', x0:1, y0:1.5, x1:7, y1:8, floor:'hosp2', says:'patient room 2'},
    {name:'Pharmacy', x0:7, y0:1.5, x1:12, y1:8, floor:'hosp', says:'the pharmacy'},
    {name:'Reception', x0:-12, y0:-8, x1:-5, y1:-1.5, floor:'hosp', says:'reception'},
    {name:'Consultation', x0:-5, y0:-8, x1:1, y1:-1.5, floor:'hosp2', says:'the consultation room'},
    {name:'Imaging', x0:1, y0:-8, x1:6, y1:-1.5, floor:'hosp2', says:'the imaging room'},
    {name:'Lift car', x0:6, y0:-8, x1:9, y1:-5.1, floor:'liftcar', says:'the lift'},
    {name:'Lift lobby', x0:6, y0:-8, x1:12, y1:-1.5, floor:'hosp', says:'the lift lobby', floorRects:[[6,-5.1,12,-1.5]]},
    {name:'Corridor', x0:-12, y0:-1.5, x1:12, y1:1.5, floor:'tile', says:'the main corridor'},
  ],
  doors:[{label:'waiting area',x:-8.25,y:1.5,w:3.5},{label:'Patient room 1 door',x:-2.3,y:1.5,w:1.4},{label:'Patient room 2 doors',x:3.9,y:1.5,w:1.8},
         {label:'pharmacy',x:9.6,y:1.5,w:2.4},{label:'reception',x:-8.5,y:-1.5,w:4.0},{label:'Consultation room door',x:-2.1,y:-1.5,w:1.4},
         {label:'Imaging room door',x:3.3,y:-1.5,w:1.4},{label:'lift lobby',x:9,y:-1.5,w:4.0},{label:'lift',x:7.5,y:-5.1,w:1.2}],
  doorDyn:[DOOR('pr1','hinged',-2.3,1.5,1.4,{hinge:-1,swing:1,label:'patient room 1 door'}),
           DOOR('pr2a','hinged',3.45,1.5,0.9,{hinge:-1,swing:1,group:'pr2',label:'patient room 2 doors'}),
           DOOR('pr2b','hinged',4.35,1.5,0.9,{hinge:1,swing:1,group:'pr2',label:'patient room 2 doors'}),
           DOOR('cons','hinged',-2.1,-1.5,1.4,{hinge:1,swing:-1,label:'consultation room door'}),
           DOOR('img','auto',3.3,-1.5,1.4,{label:'imaging room door'}),
           LIFT_DOOR],
};
const L1 = {
  name:'Floor 1 · Ward 1', spoken:'floor 1, Ward 1',
  walls:[...CORE_WALLS, [-10.75,1.5,2.5,0.2],[-5.55,1.5,3.9,0.2],[1.8,1.5,2.4,0.2],[6.5,1.5,4.2,0.2],[11,1.5,2,0.2],
    [-4,4.75,0.2,6.5],[1,4.75,0.2,6.5],[7,4.75,0.2,6.5]],
  posts:[],
  furn:(()=>{
    const F=[];
    for (const [x,p] of [[-11.0,true],[-8.5,false],[-6.0,true]]) F.push(fu('hbed',x,6.15,1.0,2.1,0.6,'S','Hospital bed','l',{patient:p}));
    for (const x of [-10.15,-7.65,-4.85]) F.push(fu('cabinet',x,6.95,0.45,0.45,0.8,'W','Bedside cabinet'));
    F.push(fu('chair',-11.45,4.4,0.55,0.55,0.9,'E','Chair','c',{color:'#7A8450'})); F.push(fu('chair',-7.0,4.4,0.55,0.55,0.9,'W','Chair','c',{color:'#7A8450'}));
    F.push(fu('ivpole',-9.6,5.0,0.3,0.3,1.9,'','IV pole','l'));
    F.push(fu('pcounter',-1.5,2.7,4.2,0.7,1.05,'S','Nurses\' station','s',{pc:true}));
    F.push(fu('ochair',-2.6,3.6,0.55,0.55,1.0,'S','Chair','c')); F.push(fu('ochair',-0.4,3.6,0.55,0.55,1.0,'S','Chair','c'));
    F.push(fu('medtrolley',-3.4,5.8,0.6,0.5,1.0,'E','Medication trolley','l'));
    F.push(fu('crashcart',0.4,5.8,0.6,0.5,1.0,'W','Crash cart','l'));
    F.push(fu('bookshelf',-1.5,7.6,3.0,0.45,2.0,'S','Supply shelves','s',{meds:true}));
    F.push(fu('hbed',2.2,5.8,1.0,2.1,0.6,'S','Hospital bed','l',{patient:true}));
    F.push(fu('cabinet',1.45,7.05,0.45,0.45,0.8,'E','Bedside cabinet'));
    F.push(fu('chair',3.6,4.4,0.6,0.6,0.9,'W','Visitor chair','c',{color:'#7A8450'}));
    F.push(fu('overbed',3.15,5.7,0.4,0.8,0.85,'W','Overbed table','l'));
    F.push(fu('sofa',6.45,5.0,0.8,1.8,0.8,'W','Sofa bed','s',{color:'#5F7F8C'}));
    F.push(fu('ivpole',1.45,4.4,0.3,0.3,1.9,'','IV pole','l'));
    F.push(fu('hbed',10.6,5.8,1.0,2.1,0.6,'S','Hospital bed','l',{patient:true}));
    F.push(fu('cabinet',11.55,7.05,0.45,0.45,0.8,'W','Bedside cabinet'));
    F.push(fu('chair',9.0,4.6,0.6,0.6,0.9,'E','Visitor chair','c',{color:'#7A8450'}));
    F.push(fu('overbed',9.65,6.0,0.4,0.8,0.85,'E','Overbed table','l'));
    F.push(fu('sinkunit',7.4,2.3,0.45,0.5,0.9,'E','Sink'));
    F.push(fu('sofa',-8.5,-7.4,2.2,0.85,0.85,'N','Sofa','s',{color:'#8E6A4F'}));
    F.push(fu('armchair',-10.9,-4.0,0.8,0.8,0.85,'E','Armchair','s',{color:'#7A5C8E'})); F.push(fu('armchair',-10.9,-5.6,0.8,0.8,0.85,'E','Armchair','s',{color:'#4F6F5C'}));
    F.push(fu('coffee',-8.3,-5.2,1.1,0.6,0.45,'','Coffee table','l'));
    F.push(fu('tvunit',-5.35,-4.7,0.45,1.6,0.55,'W','TV'));
    F.push(fu('bookshelf',-11.65,-7.2,0.4,1.2,1.8,'E','Bookcase'));
    F.push(fu('plant',-5.5,-7.5,0.45,0.45,1.2,'','Plant'));
    F.push(fu('bookshelf',-3.5,-7.5,2.4,0.45,2.0,'N','Linen shelves','s',{meds:true}));
    F.push(fu('sinkunit',0.45,-4.5,0.6,1.0,0.9,'W','Sluice sink'));
    F.push(fu('trolley',-4.3,-4.0,0.6,1.0,1.0,'E','Linen trolley','l'));
    F.push(fu('parallelbars',3.5,-5.6,0.8,2.6,0.95,'N','Parallel bars','l'));
    F.push(fu('treadmill',1.7,-6.8,0.8,1.8,1.3,'N','Treadmill'));
    F.push(fu('exbike',5.2,-6.8,0.6,1.2,1.1,'N','Exercise bike','l'));
    F.push(fu('couch',5.3,-3.6,0.7,1.9,0.7,'W','Treatment couch','l'));
    F.push(fu('seatbench',6.4,-3.0,0.5,1.6,0.45,'E','Bench','l'));
    F.push(fu('plant',11.5,-2.1,0.45,0.45,1.2,'','Plant'));
    F.push(fu('stairs',10.5,-6.05,2.8,3.7,0.0,'N','Stairs','n',{dir:'down'}));
    return F;
  })(),
  areas:[
    {name:'Ward bay', x0:-12, y0:1.5, x1:-4, y1:8, floor:'hosp2', says:'the ward bay'},
    {name:'Nurses\' station', x0:-4, y0:1.5, x1:1, y1:8, floor:'hosp', says:'the nurses\' station'},
    {name:'Room A', x0:1, y0:1.5, x1:7, y1:8, floor:'hosp2', says:'room A'},
    {name:'Room B', x0:7, y0:1.5, x1:12, y1:8, floor:'hosp2', says:'room B'},
    {name:'Day room', x0:-12, y0:-8, x1:-5, y1:-1.5, floor:'carpet', says:'the day room'},
    {name:'Utility', x0:-5, y0:-8, x1:1, y1:-1.5, floor:'hosp', says:'the utility room'},
    {name:'Physio gym', x0:1, y0:-8, x1:6, y1:-1.5, floor:'hosp2', says:'the physio gym'},
    {name:'Lift car', x0:6, y0:-8, x1:9, y1:-5.1, floor:'liftcar', says:'the lift'},
    {name:'Lift lobby', x0:6, y0:-8, x1:12, y1:-1.5, floor:'hosp', says:'the lift lobby', floorRects:[[6,-4.2,12,-1.5],[6,-5.1,9.1,-4.2]]},
    {name:'Corridor', x0:-12, y0:-1.5, x1:12, y1:1.5, floor:'tile', says:'the ward corridor'},
  ],
  doors:[{label:'ward bay',x:-8.5,y:1.5,w:2.0},{label:'nurses\' station',x:-1.5,y:1.5,w:4.2},{label:'Room A door',x:3.7,y:1.5,w:1.4},{label:'Room B door',x:9.3,y:1.5,w:1.4},
         {label:'day room',x:-8.5,y:-1.5,w:4.0},{label:'Utility room door',x:-2.1,y:-1.5,w:1.4},{label:'Physio gym door',x:3.3,y:-1.5,w:1.4},{label:'lift lobby',x:9,y:-1.5,w:4.0},{label:'lift',x:7.5,y:-5.1,w:1.2}],
  doorDyn:[DOOR('ra','hinged',3.7,1.5,1.4,{hinge:-1,swing:1,label:'room A door'}),
           DOOR('rb','hinged',9.3,1.5,1.4,{hinge:1,swing:1,label:'room B door'}),
           DOOR('util','hinged',-2.1,-1.5,1.4,{hinge:1,swing:-1,label:'utility room door'}),
           DOOR('gym','auto',3.3,-1.5,1.4,{label:'physio gym door'}),
           LIFT_DOOR],
};
WORLDS.hospital = {name:'Hospital', bounds:[-12,-8,12,8], levels:[L0,L1],
  lift:{call:[7.5,-4.25], car:[7.5,-6.55], button:[6.6,-4.98], door:'lift', floorH:3.2}};
// the lab gets automatic doors on both lab entrances and a hinged meeting-room door; the house is open plan
WORLDS.lab.doorDyn = [DOOR('la','auto',-5.75,-3,1.5,{label:'lab door A'}), DOOR('lb','auto',0.75,-3,1.5,{label:'lab door B'}),
                      DOOR('meet','hinged',6.75,-3,1.5,{hinge:1,swing:1,label:'meeting room door'})];
WORLDS.house.doorDyn = [];
Object.assign(DET_NAME, {hbed:'hospital bed', overbed:'table', ivpole:'IV pole', vending:'vending machine', pcounter:'counter', couch:'examination couch',
  scanner:'CT scanner', seatbench:'bench', wheelchair:'wheelchair', trolley:'trolley', sinkunit:'sink', kiosk:'kiosk', medtrolley:'medication trolley',
  crashcart:'crash cart', parallelbars:'parallel bars', treadmill:'treadmill', exbike:'exercise bike', stairs:'stairs'});

const HOSP = {
  cane:{brief:'Ravi is visiting a relative and tries the rover in the hospital: straight along the main corridor, then through the door of patient room 1 to the visitor chair.',
        v:{start:{x:8.5,y:0,th:Math.PI}, goals:[{x:3.0,y:0,label:'Straight along the corridor'},{x:-1.7,y:3.6,label:'Patient room 1, by the visitor chair',near:'Visitor chair'}]}},
  newblind:{brief:'Asha lost their sight four months ago and is back for a follow-up appointment. The rover takes Asha down the corridor and through the door to the consultation room.',
        v:{start:{x:8.5,y:0,th:Math.PI}, goals:[{x:3.0,y:0,label:'Along the corridor'},{x:-2.3,y:-4.4,label:'Consultation room, by the patient chair',near:'Patient chair'}]}},
  dog:{brief:'Sam is collecting a prescription. Juno knew this hospital well; the rover has to walk the full corridor to the pharmacy counter.',
        v:{start:{x:-10,y:0,th:0}, goals:[{x:9.6,y:2.05,label:'Pharmacy counter',near:'Pharmacy counter'}], walkers:[{at:{t:10}, path:[W(11.5,-0.95),W(-10,-0.95)], speed:0.95, look:6}]}},
  crowd:{brief:'Visiting hours. Ravi walks the main corridor while staff, patients and visitors overtake, come the other way and go in and out of rooms.',
        v:{start:{x:-10,y:0,th:0}, goals:[{x:10.2,y:0,label:'East end of the corridor'}],
           walkers:[{at:{t:6}, path:[W(-11.5,0.95),W(11.5,0.95)], speed:1.1, look:6},{at:{t:10}, path:[W(11.5,-0.95),W(-10,-0.95)], speed:1.0, look:7},
                    {at:{roverX:-5}, path:[W(-2.3,3.2),W(-2.3,1.3),W(-2.1,-1.3),W(-2.1,-3.2)], speed:0.8, look:6},{at:{t:25}, path:[W(9,-3.4),W(9,-1.0),W(-10,-0.9)], speed:0.9, look:3},
                    {at:{t:32}, path:[W(9.6,2.2),W(9.6,0.9),W(-10,0.9)], speed:1.0, look:7}]}},
  busy:{brief:'Lena walks the main corridor at a busy moment: a patient crosses from reception to the waiting area, a nurse comes the other way, and someone leaves imaging through its automatic door.',
        v:{start:{x:-10,y:0,th:0}, goals:[{x:10.2,y:0,label:'East end of the corridor'}],
           walkers:[{at:{t:3}, path:[W(-8.2,-3.0),W(-8.2,-1.3),W(-8.4,1.3),W(-8.4,3.0)], speed:0.7, look:1},{at:{roverX:-3}, path:[W(11.5,0.95),W(-10,0.95)], speed:1.0, look:6},
                    {at:{roverX:0.5}, path:[W(3.3,-3.0),W(3.3,-1.3),W(4.5,-0.9),W(11.5,-0.9)], speed:0.8, look:4}]}},
  bag:{brief:'Two weeks in. Ravi visits patient room 2 through its double doors, and a visitor has left a backpack lying flat just inside. At 0.18 m it is below the LiDAR\'s scan plane.',
        v:{start:{x:-1.0,y:0,th:0}, goals:[{x:4.6,y:3.4,label:'Patient room 2'}], events:[{at:{roverX:0.5}, spawn:{cls:'backpack',x:4.25,y:2.45,sx:0.4,sy:0.3,h:0.18}, say:'A visitor has left a backpack on the floor just inside patient room 2.'}]}},
  chair:{brief:'Ravi has checked in and needs somewhere to wait. Most seats are taken; the rover finds the free one and parks so it ends up beside Ravi.',
        v:{start:{x:-3,y:0,th:Math.PI}, goals:[{x:-8.7,y:3.3,label:'Beside the free seat in the waiting area',near:'Free chair'}]}},
  link:{brief:'Lena is on the way to the imaging room when the radio link between the rover and the handle drops for a few seconds, near the scanners.',
        v:{start:{x:-10,y:0,th:0}, goals:[{x:3.3,y:-3.6,label:'Imaging room'}], events:[{at:{navT:15}, link:true, say:'Interference near imaging: the handle stops receiving packets.'},{at:{navT:22}, link:false, say:'Link restored.'}]}},
  free:{brief:'Your hospital, your rules. Click the floor to send the rover somewhere, drop a flat bag or a tall box, or add people walking around.',
        v:{start:{x:0,y:0,th:0}, goals:[]}},
};
for (const s of SCEN) { s.brief.hospital=HOSP[s.id].brief; s.hospital=HOSP[s.id].v; }
SCEN.splice(SCEN.findIndex(s=>s.id==='free'), 0, {id:'upstairs', title:'Upstairs by lift', profile:'cane', trust0:0.5, stress0:0.35,
  brief:{hospital:'Ravi is visiting someone in room B on Ward 1, upstairs. The stairs are right beside the lift, but the rover cannot climb stairs. It takes Ravi to the bed lift: Ravi presses the call button, they go in and turn round, ride up, and walk to room B through its door.'},
  why:['Stairs are the most dangerous thing in a building for someone who cannot see, and a 0.23 m LiDAR cannot see a drop at all.','A lift means a closed box, doors shutting behind you and no idea what floor you are on, unless someone tells you.','So the rover keeps the stairwell as a no-go zone, describes every step of the lift, and says the floor out loud when the doors open.'],
  hospital:{start:{x:-6,y:0,th:0,floor:0}, goals:[{x:9.3,y:3.3,label:'Ward 1, room B',floor:1,near:'Visitor chair'}],
            walkers:[{at:{floor:1,delay:5}, path:[W(-3.5,-0.9),W(11.5,-0.9)], speed:0.9, look:6},{at:{floor:1,delay:9}, path:[W(10.5,0.95),W(-11,0.95)], speed:1.0, look:7}]}});
