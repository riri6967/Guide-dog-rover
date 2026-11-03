
/* ================= Worlds ================= */
// furniture: cls, x, y, sx, sy, height, facing (front side: N/S/E/W), label, LiDAR shape, options
// LiDAR shape (the scan plane is only 0.23 m up): s solid to the floor, l four legs, c chair, d desk legs + PC towers, n none
const fu = (cls,x,y,sx,sy,h,face,label,scan,opt) => ({cls,x,y,sx,sy,h,face:face||'',label,scan:scan||'s',opt:opt||{}});
const WORLDS = {
  house: {
    name:'House', bounds:[-12,-8,12,8],
    walls:[[-12,0,0.2,16],[12,0,0.2,16],[0,8,24,0.2],[0,-8,24,0.2],
      [-8,1.5,4,0.2],[5.3,1.5,9.4,0.2],[-5,-1.5,10,0.2],[8,-1.5,4,0.2],
      [-9,4.75,0.2,6.5],[-1,4.75,0.2,6.5],[-5,7.9,8,0.2],[-7.5,1.5,3,0.2],[-2.5,1.5,3,0.2],
      [1,-4.75,0.2,6.5],[9,-4.75,0.2,6.5],[5,-7.9,8,0.2],[2,-1.5,2,0.2],[8,-1.5,2,0.2]],
    posts:[[-6,1.5],[-4,1.5],[6,-1.5]],
    furn:[
      // Room 1 (bedroom): the SDF bed, table and chair, plus the rest of a lived-in room
      fu('bed',-7.3,5.5,2.8,1.5,0.7,'E','Bed','s',{duvet:'#4F79A8'}),
      fu('table',-3,4,1.3,1.0,0.76,'','Table','l',{items:'study'}),
      fu('chair',-3,5.6,0.7,0.7,0.9,'S','Chair','c'),
      fu('wardrobe',-4.6,7.45,1.6,0.6,2.0,'S','Wardrobe'),
      fu('nightstand',-8.55,4.35,0.45,0.42,0.55,'E','Nightstand'),
      fu('bookshelf',-1.35,6.4,0.4,1.2,1.8,'W','Bookshelf'),
      fu('armchair',-8.3,2.3,0.8,0.8,0.85,'E','Armchair','s',{color:'#7A5C8E'}),
      fu('dresser',-2.0,7.5,1.0,0.5,0.9,'S','Dresser'),
      fu('plant',-1.45,2.0,0.45,0.45,1.1,'','Plant'),
      // Room 2 (guest bedroom / study)
      fu('bed',7.3,-5.5,2.8,1.5,0.7,'W','Bed','s',{duvet:'#B0675A'}),
      fu('table',3,-4,1.3,1.0,0.76,'','Table','l',{items:'laptop'}),
      fu('chair',3,-5.6,0.7,0.7,0.9,'N','Chair','c'),
      fu('wardrobe',5.0,-7.45,1.6,0.6,2.0,'N','Wardrobe'),
      fu('nightstand',8.55,-4.35,0.45,0.42,0.55,'W','Nightstand'),
      fu('dresser',8.55,-2.4,0.6,1.1,0.9,'W','Dresser'),
      fu('bookshelf',1.35,-6.6,0.4,1.2,1.8,'E','Bookshelf'),
      fu('armchair',6.9,-2.3,0.8,0.8,0.85,'S','Armchair','s',{color:'#4F6F5C'}),
      fu('plant',1.5,-2.0,0.45,0.45,1.1,'','Plant'),
      // North hall (living room)
      fu('sofa',5.0,7.35,2.4,0.9,0.85,'S','Sofa','s',{color:'#56657A'}),
      fu('coffee',5.0,5.95,1.2,0.6,0.45,'','Coffee table','l'),
      fu('armchair',2.6,5.9,0.85,0.85,0.85,'E','Armchair','s',{color:'#8E6A4F'}),
      fu('armchair',7.4,5.9,0.85,0.85,0.85,'W','Armchair','s',{color:'#8E6A4F'}),
      fu('tvunit',5.0,1.9,1.8,0.45,0.55,'N','TV unit'),
      fu('bookshelf',-0.6,6.6,0.4,1.4,1.9,'E','Bookshelf'),
      fu('sidetable',3.6,7.4,0.5,0.5,0.55,'','Side table'),
      fu('lamp',6.6,7.5,0.35,0.35,1.6,'','Floor lamp'),
      fu('cabinet',11.6,4.0,0.45,1.6,0.9,'W','Cabinet'),
      fu('desk',10.0,2.2,1.4,0.7,0.75,'N','Desk','l',{items:'pc'}),
      fu('ochair',10.0,2.95,0.55,0.55,1.0,'S','Desk chair','c'),
      fu('plant',11.2,7.2,0.6,0.6,1.3,'','Plant'),
      // kitchen and dining (south-west)
      fu('counter',-6.0,-7.55,5.0,0.65,0.9,'N','Kitchen counter','s',{sink:-1.2,hob:1.0}),
      fu('fridge',-9.0,-7.5,0.75,0.7,1.85,'N','Fridge'),
      fu('counter',-11.55,-5.6,0.65,2.4,0.9,'E','Counter'),
      fu('table',-5.0,-4.4,1.8,1.0,0.75,'','Dining table','l',{items:'dining'}),
      fu('chair',-5.5,-3.55,0.5,0.5,0.9,'S','Chair','c'), fu('chair',-4.5,-3.55,0.5,0.5,0.9,'S','Chair','c'),
      fu('chair',-5.5,-5.25,0.5,0.5,0.9,'N','Chair','c'), fu('chair',-4.5,-5.25,0.5,0.5,0.9,'N','Chair','c'),
      fu('chair',-6.25,-4.4,0.5,0.5,0.9,'E','Chair','c'), fu('chair',-3.75,-4.4,0.5,0.5,0.9,'W','Chair','c'),
      fu('plant',-11.2,-7.2,0.6,0.6,1.3,'','Plant'),
      fu('plant',-11.4,-2.0,0.45,0.45,1.1,'','Plant'),
      // laundry (south-east), store (north-west), corridor ends
      fu('washer',11.55,-6.8,0.6,0.6,0.85,'W','Washing machine'),
      fu('washer',11.55,-6.1,0.6,0.6,0.85,'W','Dryer'),
      fu('basket',10.4,-7.55,0.5,0.4,0.4,'','Laundry basket'),
      fu('bookshelf',11.65,-3.5,0.4,1.2,1.8,'W','Shelf'),
      fu('cabinet',-11.6,5.2,0.45,1.6,0.9,'E','Cabinet'),
      fu('plant',-11.3,7.4,0.45,0.45,1.1,'','Plant'),
      fu('cabinet',-11.65,0.4,0.35,1.0,0.5,'E','Shoe rack'),
      fu('cabinet',11.65,-0.4,0.35,1.0,0.8,'W','Console'),
    ],
    areas:[
      {name:'Room 1', x0:-9, y0:1.5, x1:-1, y1:7.9, floor:'wood', says:'the bedroom'},
      {name:'Room 2', x0:1, y0:-7.9, x1:9, y1:-1.5, floor:'wood2', says:'the guest room'},
      {name:'Living room', x0:-1, y0:1.5, x1:12, y1:8, floor:'carpet', says:'the living room'},
      {name:'Kitchen', x0:-12, y0:-8, x1:1, y1:-1.5, floor:'tile2', says:'the kitchen'},
      {name:'Laundry', x0:9, y0:-8, x1:12, y1:-1.5, floor:'tile2', says:'the laundry corner'},
      {name:'Corridor', x0:-12, y0:-1.5, x1:12, y1:1.5, floor:'tile', says:'the corridor'},
      {name:'Store', x0:-12, y0:1.5, x1:-9, y1:8, floor:'tile', says:'the store corner'},
    ],
    doors:[{label:'Room 1 door',x:-5,y:1.5,w:1.9},{label:'Room 2 door',x:4.5,y:-1.5,w:2.9},{label:'living room opening',x:-0.15,y:1.5,w:1.5},{label:'kitchen passage',x:0.45,y:-1.5,w:0.9}],
  },
  lab: {
    name:'Laboratory', bounds:[-10,-8,10,7],
    walls:[[-10,-0.5,0.2,15],[10,-0.5,0.2,15],[0,7,20,0.2],[0,-8,20,0.2],
      [-8.25,-3,3.5,0.2],[-2.5,-3,5,0.2],[2.75,-3,2.5,0.2],[5,-3,2,0.2],[8.75,-3,2.5,0.2],   // corridor north wall, 3 doors
      [-5.5,-5.6,9,0.2],[5.5,-5.6,9,0.2],                                                   // corridor south wall (2.4 m corridor), print bay opening
      [4,0.5,0.2,7],[4,6.15,0.2,1.7],                                                       // lab | meeting + server, server door
      [7,2,6,0.2]],
    posts:[],
    furn:(()=>{
      const F=[];
      for (const cx of [-8.1,-5.3,-2.5]) {
        F.push(fu('bench',cx,6.55,2.6,0.7,0.75,'S','Desk','d',{seats:[[cx-0.65,6.55,'S']],wall:true}));
        for (const sx of [cx-0.65,cx+0.65]) F.push(fu('ochair',sx,6.05,0.5,0.5,1.0,'N','Chair','c'));
      }
      F[0].opt.seats=[[-8.75,6.55,'S'],[-7.45,6.55,'S']]; F[3].opt.seats=[[-5.95,6.55,'S'],[-4.65,6.55,'S']]; F[6].opt.seats=[[-3.15,6.55,'S'],[-1.85,6.55,'S']];
      for (const yc of [3.3,-0.3]) for (const xc of [-6.6,-1.0]) {
        const seats=[]; for (const o of [-1.5,-0.5,0.5,1.5]) { seats.push([xc+o,yc+0.35,'N'],[xc+o,yc-0.35,'S']); }
        F.push(fu('bench',xc,yc,4.0,1.4,0.75,'','Desk','d',{seats,island:true}));
        for (const o of [-1.5,-0.5,0.5,1.5]) {
          const x=xc+o;
          if (!(yc<0 && x===-1.5)) F.push(fu('ochair',x,yc+0.85,0.5,0.5,1.0,'S','Chair','c'));
          F.push(fu('ochair',x,yc-0.85,0.5,0.5,1.0,'N','Chair','c'));
        }
      }
      F.push(fu('ochair',-1.5,1.0,0.5,0.5,1.0,'S','Free chair','c',{pulled:true}));
      F.push(fu('bench',3.45,0.8,0.7,2.6,0.9,'W','Electronics bench','d',{seats:[[3.45,1.4,'W'],[3.45,0.2,'W']],electronics:true}));
      F.push(fu('printer3d',3.45,-1.9,0.7,0.8,1.2,'W','3D printer','l'));
      F.push(fu('table',2.4,6.2,1.2,0.9,0.8,'','Robot arm','l',{items:'arm'}));
      F.push(fu('plant',-9.6,-2.55,0.45,0.45,1.2,'','Plant'));
      // server room
      for (const y of [2.8,3.85,4.9,5.95]) F.push(fu('rack',9.45,y,0.6,1.0,2.0,'W','Server rack'));
      for (const y of [3.4,4.45]) F.push(fu('rack',7.6,y,0.6,1.0,2.0,'W','Server rack'));
      F.push(fu('table',5.0,6.5,1.2,0.6,0.75,'','Work table','l',{items:'laptop'}));
      F.push(fu('cabinet',5.0,2.45,0.6,0.4,0.9,'N','Power unit'));
      // meeting room
      F.push(fu('table',7.2,-0.5,2.6,1.2,0.75,'','Meeting table','l',{items:'meeting'}));
      for (const x of [6.4,7.2,8.0]) F.push(fu('ochair',x,0.45,0.5,0.5,1.0,'S','Chair','c'));
      for (const x of [6.4,8.0]) F.push(fu('ochair',x,-1.45,0.5,0.5,1.0,'N','Chair','c'));
      F.push(fu('ochair',5.35,-0.5,0.5,0.5,1.0,'E','Chair','c')); F.push(fu('ochair',9.05,-0.5,0.5,0.5,1.0,'W','Chair','c'));
      F.push(fu('cabinet',8.5,1.6,1.6,0.45,0.8,'S','Credenza'));
      F.push(fu('plant',9.5,-2.5,0.45,0.45,1.2,'','Plant'));
      // print bay and kitchenette
      F.push(fu('printer',0,-7.55,1.0,0.6,1.0,'N','Printer'));
      F.push(fu('lockers',-6,-7.65,5.0,0.45,1.9,'N','Lockers'));
      F.push(fu('cooler',1.6,-7.7,0.35,0.35,1.2,'','Water cooler'));
      F.push(fu('counter',5.5,-7.6,4.0,0.6,0.9,'N','Kitchenette','s',{sink:-0.8,coffee:1.2}));
      F.push(fu('bins',-2.0,-7.7,0.9,0.35,0.7,'N','Recycling bins'));
      return F;
    })(),
    areas:[
      {name:'Server room', x0:4, y0:2, x1:10, y1:7, floor:'raised', says:'the server room'},
      {name:'Meeting room', x0:4, y0:-3, x1:10, y1:2, floor:'carpet', says:'the meeting room'},
      {name:'Main lab', x0:-10, y0:-3, x1:4, y1:7, floor:'vinyl', says:'the main lab'},
      {name:'Corridor', x0:-10, y0:-5.6, x1:10, y1:-3, floor:'tile', says:'the corridor'},
      {name:'Print bay', x0:-10, y0:-8, x1:10, y1:-5.6, floor:'tile2', says:'the print bay'},
    ],
    doors:[{label:'Lab door A',x:-5.75,y:-3,w:1.5},{label:'Lab door B',x:0.75,y:-3,w:1.5},{label:'Meeting room door',x:6.75,y:-3,w:1.5},
           {label:'Server room door',x:4,y:4.65,w:1.3,vert:true},{label:'Print bay opening',x:0,y:-5.6,w:2.0}],
  },
};
const DET_NAME = {bed:'bed', table:'table', chair:'chair', ochair:'office chair', plant:'potted plant', walker:'person', backpack:'backpack', box:'box', doorway:'doorway',
  wardrobe:'wardrobe', nightstand:'nightstand', bookshelf:'bookshelf', armchair:'armchair', dresser:'dresser', sofa:'couch', coffee:'coffee table', tvunit:'tv',
  sidetable:'side table', lamp:'floor lamp', cabinet:'cabinet', desk:'desk', counter:'counter', fridge:'refrigerator', washer:'washing machine', basket:'basket',
  bench:'desk + monitors', printer3d:'3D printer', rack:'server rack', printer:'printer', lockers:'lockers', cooler:'water cooler', bins:'bins'};

let WORLD, WORLD_ID, LEVEL=0, STATIC=[], WALL_RECTS=[], SCAN_ALL=[], AREAS=[], DOORS=[], DOOR_DYN=[];
const mk = (cx,cy,sx,sy,o) => Object.assign({x0:cx-sx/2,y0:cy-sy/2,x1:cx+sx/2,y1:cy+sy/2,cx,cy,sx,sy}, o);
function scanShapes(b) {
  const out=[], leg=(x,y,s)=>out.push(mk(x,y,s,s,{kind:'leg',of:b}));
  if (b.scan==='s') out.push(b);
  else if (b.scan==='l'||b.scan==='d') {
    const ix=Math.min(0.06,b.sx/4), iy=Math.min(0.06,b.sy/4);
    for (const [px,py] of [[b.x0+ix,b.y0+iy],[b.x1-ix,b.y0+iy],[b.x0+ix,b.y1-iy],[b.x1-ix,b.y1-iy]]) leg(px,py,0.05);
    if (b.sx>2.2) for (let x=b.x0+2; x<b.x1-0.5; x+=2) { leg(x,b.y0+iy,0.05); leg(x,b.y1-iy,0.05); }
    if (b.scan==='d') for (const [sx,sy,f] of (b.opt.seats||[])) {   // PC towers under the desk at each seat
      const ox = f==='E'?-0.1:f==='W'?0.1:0, oy = f==='N'?-0.15:f==='S'?0.15:0;
      out.push(mk(sx+0.35+ox, sy+oy, f==='E'||f==='W'?0.45:0.2, f==='E'||f==='W'?0.2:0.45, {kind:'leg',of:b}));
    }
  } else if (b.scan==='c') {
    if (b.cls==='ochair') out.push(mk(b.cx,b.cy,0.07,0.07,{kind:'leg',of:b}));
    else { const i=0.05; for (const [px,py] of [[b.x0+i,b.y0+i],[b.x1-i,b.y0+i],[b.x0+i,b.y1-i],[b.x1-i,b.y1-i]]) leg(px,py,0.035); }
  }
  return out;
}
// a world can have several levels (the hospital); the simulation only ever holds the level the rover is on
const levelData = (id, level) => WORLDS[id].levels ? WORLDS[id].levels[level||0] : WORLDS[id];
function buildWorld(id, level) {
  WORLD_ID=id; WORLD=WORLDS[id]; LEVEL=WORLD.levels ? (level||0) : 0;
  const D=levelData(id, LEVEL);
  STATIC=[];
  D.walls.forEach(w => STATIC.push(mk(w[0],w[1],w[2],w[3],{kind:'wall',h:2.6})));
  D.posts.forEach(p => STATIC.push(mk(p[0],p[1],0.12,0.12,{kind:'post',h:2.4})));
  D.furn.forEach((f,i) => STATIC.push(mk(f.x,f.y,f.sx,f.sy,Object.assign({kind:'furn',id:100+i+LEVEL*1000},f))));
  WALL_RECTS = STATIC.filter(b => b.kind!=='furn' || b.h>1.5);
  SCAN_ALL = STATIC.flatMap(b => b.kind==='furn' ? scanShapes(b) : [b]);
  AREAS = D.areas; DOORS = D.doors; DOOR_DYN = D.doorDyn || WORLD.doorDyn || [];
  // the planner treats open hinged leaves as solid, so routes never cut through a door's swing
  rasterize(STATIC.concat(DOOR_DYN.filter(d=>d.type==='hinged').map(leafRect))); marks.fill(0); computeDist();
}
const areaAt = (x,y) => AREAS.find(a => x>=a.x0 && x<=a.x1 && y>=a.y0 && y<=a.y1);
