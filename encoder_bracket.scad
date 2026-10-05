// encoder_bracket.scad — holds an LM393 slot sensor against the encoder disc
//
// Open in OpenSCAD, set the numbers from your own hardware, F5 to preview,
// F6 then Export as STL.
//
// GEOMETRY — get this clear before touching the numbers:
//
//   The disc is on the axle, so it spins in a VERTICAL plane. The sensor's
//   two posts must sit either side of that plane, with the beam crossing the
//   slotted ring. So the sensor PCB lies FLAT (posts pointing up) somewhere
//   below the axle, and the bottom edge of the disc dips down into the gap.
//
//        axle  ●
//              |
//         ,----+----.        <- disc, vertical, spinning
//        (           )
//         `----+----'
//          ║   |   ║         <- sensor posts, disc passing between them
//        ──╨───┴───╨──       <- sensor PCB, flat
//            shelf
//            │
//            │ upright
//          ──┴──  base -> chassis
//
// Every hole is a SLOT, because getting this right first try does not happen.
// Loosen, nudge, retighten.

// ---------------------------------------------------------------- measure these
// Sensor PCB — measure centre-to-centre between its two mounting holes.
sensor_hole_spacing = 28;    // mm
sensor_hole_d       = 3.2;   // M3 clearance

// How high the sensor PCB must sit above the chassis surface it bolts to.
// = (axle height) - (disc radius) + (depth from PCB up to the beam)
// Measure the last one: it is how far the beam sits above the PCB, often 8-12mm.
shelf_height        = 35;    // mm

// How far the shelf reaches out from the upright, to get under the disc.
shelf_reach         = 30;    // mm

// Chassis mounting — spacing of the two bolts holding this bracket down.
base_hole_spacing   = 24;    // mm
base_hole_d         = 3.2;   // M3 clearance

// ---------------------------------------------------------------- build
wall       = 3;      // plate thickness. 3mm in PLA is stiff enough
width      = 24;     // bracket width
slot_range = 6;      // how far each slot lets you slide things, mm
$fn        = 48;

// A slot: a hole you can slide along. Oriented along X.
module slot(d, travel, h) {
    hull() {
        translate([-travel/2, 0, 0]) cylinder(d = d, h = h, center = true);
        translate([ travel/2, 0, 0]) cylinder(d = d, h = h, center = true);
    }
}

module bracket() {
    difference() {
        union() {
            // base — bolts to the chassis
            translate([0, 0, wall/2])
                cube([width, base_hole_spacing + 16, wall], center = true);

            // upright
            translate([0, -(base_hole_spacing + 16)/2 + wall/2, shelf_height/2])
                cube([width, wall, shelf_height], center = true);

            // shelf — sensor sits here
            translate([0, -(base_hole_spacing + 16)/2 + shelf_reach/2, shelf_height])
                cube([width, shelf_reach, wall], center = true);

            // gussets, so the shelf does not flex and let the disc touch a post
            for (s = [-1, 1])
                translate([s * (width/2 - wall/2), 0, 0])
                    rotate([0, 0, 0])
                    translate([0, -(base_hole_spacing + 16)/2 + wall, 0])
                    linear_extrude(wall, center = true)
                        polygon([[0,0], [0, shelf_height - wall],
                                 [shelf_reach * 0.6, 0]]);
        }

        // chassis bolt slots — slide fore/aft
        for (s = [-1, 1])
            translate([0, s * base_hole_spacing/2, wall/2])
                slot(base_hole_d, slot_range, wall + 2);

        // sensor bolt slots — slide the sensor toward/away from the disc
        for (s = [-1, 1])
            translate([s * sensor_hole_spacing/2,
                       -(base_hole_spacing + 16)/2 + shelf_reach/2,
                       shelf_height])
                rotate([0, 0, 90])
                    slot(sensor_hole_d, slot_range, wall + 2);
    }
}

bracket();

// ---------------------------------------------------------------- notes
//
// PRINTING
//   Layer height 0.2, walls 3, infill 30%, no supports if printed base-down.
//   Print ONE, check it against the real wheel and sensor, then print the
//   second. The first one is a measuring tool.
//
// IF THE GEOMETRY IS WRONG
//   shelf too low  -> disc never reaches the beam, no pulses
//   shelf too high -> disc bottoms out on the PCB, or rubs a post
//   Both are fixed by changing shelf_height and reprinting, which is why the
//   first print is cheap and small.
//
// THE THING THAT ACTUALLY MATTERS
//   The disc must pass through the gap WITHOUT TOUCHING, with the slots
//   crossing the beam. Spin the wheel by hand and watch for wobble before
//   you bolt anything down permanently.
