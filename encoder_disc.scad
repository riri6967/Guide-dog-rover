// encoder_disc.scad — slotted disc for an LM393 slot-gate encoder
//
// Open in OpenSCAD (free, openscad.org), change the numbers below, press F5
// to preview and F6 then "Export as STL" to slice and print.
//
// MEASURE THESE FIRST, off your sensor and axle:
//   - the gap between the two black posts  -> disc_thickness must be less
//   - how deep the beam sits inside the slot -> the slot ring must reach it
//   - your axle diameter                   -> bore_d

// ---------------------------------------------------------------- geometry
slots            = 20;    // how many slots. More = finer resolution, harder to print
disc_d           = 40;    // outer diameter, mm
disc_thickness   = 2;     // must clear the sensor gap (usually ~5mm) with room to spare

// The ring the slots live in. The sensor beam must fall inside this band.
slot_inner_d     = 24;    // where slots start
slot_outer_d     = 36;    // where slots end

duty             = 0.5;   // 0.5 = slot and tooth equally wide. Keep near 0.5

// ---------------------------------------------------------------- hub
bore_d           = 6.0;   // axle diameter. Print 0.2 UNDER for a press fit
bore_flat        = 0;     // D-shaft: depth of the flat, mm. 0 for a round shaft
hub_d            = 16;    // solid area around the bore, keeps the disc stiff

// ---------------------------------------------------------------- derived
pitch_deg        = 360 / slots;
slot_deg         = pitch_deg * duty;
$fn              = 120;

// Printed slot width at the middle of the slot ring — sanity check this.
// Below about 1.2mm a 0.4mm nozzle starts closing the gaps up.
mid_r            = (slot_inner_d + slot_outer_d) / 4;
echo(str("Slot width at mid-radius: ",
         round(2 * 3.14159 * mid_r / slots * duty * 100) / 100, " mm"));
echo(str("Pulses per revolution (RISING edge only): ", slots));

// ---------------------------------------------------------------- parts

// One angular slot, cut from slot_inner_d out to slot_outer_d.
module slot_cut() {
    r_in  = slot_inner_d / 2;
    r_out = slot_outer_d / 2;
    rotate([0, 0, -slot_deg / 2])
        linear_extrude(disc_thickness + 2)
            polygon(concat(
                [for (a = [0 : 1 : slot_deg]) [r_out * cos(a), r_out * sin(a)]],
                [for (a = [slot_deg : -1 : 0]) [r_in  * cos(a), r_in  * sin(a)]]
            ));
}

// The axle hole, with an optional flat for a D-shaft.
module bore_cut() {
    translate([0, 0, -1])
        difference() {
            cylinder(d = bore_d, h = disc_thickness + 2);
            if (bore_flat > 0)
                translate([bore_d/2 - bore_flat, -bore_d/2, -1])
                    cube([bore_d, bore_d, disc_thickness + 4]);
        }
}

module encoder_disc() {
    difference() {
        union() {
            cylinder(d = disc_d, h = disc_thickness);
            // Slightly taller hub: more grip on the axle, less wobble.
            cylinder(d = hub_d, h = disc_thickness * 2);
        }
        for (i = [0 : slots - 1])
            rotate([0, 0, i * pitch_deg]) slot_cut();
        bore_cut();
    }
}

encoder_disc();

// ---------------------------------------------------------------- printing
// Layer height   0.2 mm
// Walls          3
// Infill         40% (it is thin, infill barely applies)
// Supports       none needed — every cut goes straight through
// Orientation    flat on the bed, as modelled
//
// Print one, test the fit on the axle, then print the second. A bore that is
// 0.2mm too tight can be opened with a drill bit; one that is too loose is
// scrap.
