"""Prepare the handle parts for 3D printing.

    python print_prep.py <grip STEP folder> <out folder>

For every printed part: tries the six ways of laying it on the bed, measures
how much surface would need support (faces steeper than 45 degrees overhang,
not resting on the bed), picks the best, and exports an STL already rotated
that way and sitting on Z = 0. Also re-checks that no two parts overlap.
"""
import itertools
import sys
from pathlib import Path

import numpy as np
import cadquery as cq

import build_handle as bh
from OCP.GProp import GProp_GProps
from OCP.BRepGProp import BRepGProp

# One filament for every part: PETG (tough, a little flexible, prints cleanly).
DENSITY = 1.27   # g/cm3, PETG
NOZZLE = 0.4
# name: (walls, top/bottom layers, infill %, supports, brim, why)
SETTINGS = {
    'Frame': (4, 5, 20, 'tree / organic, touching build plate only where possible', 'no',
              "carries the rod's pull and twist; gyroid infill is light and strong in every direction"),
    'Hinge Bracket': (4, 5, 30, 'none needed (base down)', 'no', 'takes the most force, at the pivot bolt'),
    'Rod Plug': (4, 5, 40, 'none (its 6 and 12 mm side holes bridge fine)', '5 mm', 'small and tall; holds the cross-bolt'),
    'Bail': (4, 4, 20, 'none or minimal', 'no', 'squeezed thousands of times; PETG flexes instead of snapping'),
    'Cap Right (switch)': (3, 4, 15, 'tree, only under the switch housing', 'no', 'light clamping part'),
    'Cap Left': (3, 4, 15, 'tree, minimal', 'no', 'light clamping part'),
    'Pod Lid': (2, 3, 15, 'none', 'no', 'a cover, carries no load'),
    'Grip Top Part A (wire hole)': (3, 4, 15, 'tree, inside the channel only', 'no', 'your grip; its 2.5 mm walls print solid at 3 walls anyway'),
    'Grip Top Part B': (3, 4, 15, 'tree, minimal', 'no', 'your back panel'),
}

ORIENTS = {  # name: the part's direction that points UP on the bed
    '+Y up (as used)': (0, 1, 0), '-Y up (upside down)': (0, -1, 0),
    '+Z up (front up)': (0, 0, 1), '-Z up (back up)': (0, 0, -1),
    '+X up (right end up)': (1, 0, 0), '-X up (left end up)': (-1, 0, 0),
}
BED = (220, 220, 250)
# Parts modelled at an angle get their own way up: the rod plug stands on its round end.
CUSTOM = {'Rod Plug': ('standing on its round end', (1, 0, 0), -bh.ROD_ANGLE),
          # a 4 mm strip 115 long would wobble on its end: lay it on its flat outer face
          'Grip Top Part B': ('flat, outer face down', (1, 0, 0), 0)}


def triangles(wp):
    vs, fs = wp.val().tessellate(0.1, 0.15)
    V = np.array([[v.x, v.y, v.z] for v in vs]); return V[np.array(fs)]


def support_area(T, up):
    up = np.array(up, float)
    n = np.cross(T[:, 1] - T[:, 0], T[:, 2] - T[:, 0])
    area = np.linalg.norm(n, axis=1) / 2
    n = n / (np.linalg.norm(n, axis=1)[:, None] + 1e-12)
    h = T @ up
    on_bed = h.max(axis=1) < h.min() + 0.2
    overhang = (n @ up < -np.cos(np.radians(45))) & ~on_bed
    bed_area = area[on_bed & (n @ up < -0.99)].sum()
    return area[overhang].sum(), bed_area, h.max() - h.min()


def rotate_up(wp, up):
    """Rotate so `up` points to +Z, then sit it on Z = 0."""
    s = wp.val()
    rot = {(0, 1, 0): ((1, 0, 0), 90), (0, -1, 0): ((1, 0, 0), -90), (0, 0, 1): (None, 0),
           (0, 0, -1): ((1, 0, 0), 180), (1, 0, 0): ((0, 1, 0), -90), (-1, 0, 0): ((0, 1, 0), 90)}[up]
    if rot[0]: s = s.rotate(cq.Vector(0, 0, 0), cq.Vector(*rot[0]), rot[1])
    bb = s.BoundingBox()
    return s.translate(cq.Vector(-(bb.xmin + bb.xmax) / 2, -(bb.ymin + bb.ymax) / 2, -bb.zmin))


def main(grip_dir, out_dir):
    out = Path(out_dir); stl = out / 'STL (print-ready orientation)'; stl.mkdir(parents=True, exist_ok=True)
    gA, gB, parts, tube, body, lidar = bh.main(grip_dir, out_dir)
    # the user's grip: Part A gets the 6 mm wire hole in its bottom wall, under the right cradle
    gA_hole = gA.cut(bh.cyl(bh.WIRE_D, 6, (54, -bh.GRIP_Y / 2 - 1, 0), (0, 1, 0)))
    cq.exporters.export(gA_hole, str(out / 'Grip Top Part A (wire hole).step'))
    printable = {'Grip Top Part A (wire hole)': gA_hole, 'Grip Top Part B': gB, **parts}

    print('\n=== interference (overlapping volume between parts) ===')
    solids = {'Grip A': gA_hole, 'Grip B': gB, **parts, 'Tube': tube}
    clashes = 0
    for (a, sa), (b, sb) in itertools.combinations(solids.items(), 2):
        v = sa.val().intersect(sb.val()).Volume()
        if v > 0.5: print(f'   {a} x {b}: {v:.1f} mm3'); clashes += 1
    print('   none' if not clashes else f'   {clashes} clashes')

    print('\n=== orientation (support area in mm2 for each way up; lower is better) ===')
    rows = []
    for name, wp in printable.items():
        if name in CUSTOM:
            label, axis, ang = CUSTOM[name]
            wp = cq.Workplane(obj=wp.val().rotate(cq.Vector(0, 0, 0), cq.Vector(*axis), ang))
            ORIENTS_ = {label: (0, 0, 1)}
        else:
            ORIENTS_ = ORIENTS
        T = triangles(wp)
        scores = {o: support_area(T, up) for o, up in ORIENTS_.items()}
        best = min(scores, key=lambda o: (scores[o][0] - 0.3 * scores[o][1], scores[o][2]))
        s = rotate_up(wp, ORIENTS_[best])
        bb = s.BoundingBox()
        fits = bb.xlen <= BED[0] and bb.ylen <= BED[1] and bb.zlen <= BED[2]
        cq.exporters.export(cq.Workplane(obj=s), str(stl / f'{name}.stl'), tolerance=0.05, angularTolerance=0.1)
        row = '  '.join(f'{o.split()[0]} {v[0]:6.0f}' for o, v in scores.items())
        print(f'{name:20s} best: {best:22s} support {scores[best][0]:6.0f} mm2, on bed {scores[best][1]:6.0f} mm2, '
              f'size {bb.xlen:.0f}x{bb.ylen:.0f}x{bb.zlen:.0f} {"fits" if fits else "TOO BIG"}')
        print(f'{"":20s} ({row})')
        print(f'{"":20s} volume {wp.val().Volume() / 1000:5.1f} cm3')
        rows.append((name, best, scores[best][0], bb, grams(wp, *SETTINGS[name][:3])))
    write_guide(out, rows)


def grams(wp, walls, tb, infill):
    """Rough printed mass: a solid skin (walls / top-bottom) plus infill inside."""
    g = GProp_GProps(); BRepGProp.SurfaceProperties_s(wp.val().wrapped, g)
    area, vol = g.Mass(), wp.val().Volume()
    skin = min(vol, area * (walls * NOZZLE * 0.6 + tb * 0.2 * 0.4))   # walls on sides, layers on top/bottom
    solid_g = vol / 1000 * DENSITY
    return (skin + infill / 100 * (vol - skin)) / 1000 * DENSITY, solid_g


def write_guide(out, rows):
    total = sum(r[4][0] for r in rows); solid = sum(r[4][1] for r in rows)
    L = ['# Haptic handle: print guide', '',
         'All parts in **PETG**, 0.4 mm nozzle, **0.2 mm layers**, **gyroid infill**. STLs in',
         '`STL (print-ready orientation)` are already the right way up: drop them in and slice.', '',
         '| Part | Qty | Walls | Top/bottom | Infill | Supports | Brim | Est. weight | Why |',
         '|---|---|---|---|---|---|---|---|---|']
    for name, best, sup, bb, (g, gs) in rows:
        w, tb, inf, sp, brim, why = SETTINGS[name]
        L.append(f'| {name} | 1 | {w} | {tb} | {inf}% | {sp} | {brim} | ~{g:.0f} g (solid {gs:.0f} g) | {why} |')
    L += ['', f'**Total: about {total:.0f} g of PETG** for all printed parts '
              f'(about {solid:.0f} g if printed solid). Add roughly 10-15 % for supports. One 1 kg spool is plenty.', '',
          '## Same for every part', '',
          '- Nozzle 240 °C, bed 75-80 °C (PETG). Glue stick or textured PEI so PETG releases cleanly.',
          '- Seam: rear or aligned. No ironing, no fuzzy skin.',
          '- Supports: tree/organic, 45° threshold, "on build plate only" where the slicer allows.',
          '- Heat-set inserts: set them after printing with a soldering iron at about 220 °C.', '',
          '## Keeping it light', '',
          '- Gyroid at 15-30 % instead of high infill: it is strong in every direction for its weight.',
          '- Strength comes from walls, not infill: 4 walls on the load-bearing parts, 2-3 on covers.',
          '- 0.2 mm layers; finer layers add time, not strength.',
          '- Do not print anything at 100 %. The rod plug (40 %) is the densest part.', '',
          '## Hardware', '',
          '| Item | Qty | Where |', '|---|---|---|',
          '| M3 x 12 socket screw | 4 | caps to cradles |',
          '| M3 heat-set insert (hole 4.0) | 4 | cradle ears |',
          '| M3 x 25 screw + M3 nyloc nut | 2 + 2 | bail pivots through the legs |',
          '| M2.5 x 6 screw + M2.5 heat-set insert (hole 3.6) | 2 + 2 | pod lid |',
          '| M4 x 6 grub screw + M4 heat-set insert (hole 5.6) | 1 + 1 | rod socket |',
          '| M3 x 20 screw + M3 nyloc nut | 1 + 1 | cross-bolt, rod plug in tube (drill the tube 3.2) |',
          '| M6 x 30 bolt, 2 washers, M6 nyloc nut | 1 set | hinge pivot |',
          '| Rubber grommet, ID 6, OD 12 | 1 | in the rod plug tongue |',
          '| M4 x 25 bolt + M4 nyloc nut | 2 + 2 | hinge stops |',
          '| M3 x 12 screw, nyloc nut, large washers | 4 | bracket to rover body |',
          '| Aluminium tube 16 x 1 mm | ~605 mm | rod (length by user height, see CANE.md) |',
          '| Subminiature lever microswitch (D2F size) | 1 | right leg pocket, pressed by the bail |',
          '| 5-way navigation switch | 1 | right cap pocket |',
          '| Thin foam tape (~1 mm) | 1 strip | inside each cradle, so the caps clamp the grip |',
          '| Epoxy | - | rod plug into the tube |']
    (out / 'PRINT_GUIDE.md').write_text('\n'.join(L) + '\n', encoding='utf-8')
    print(f'PRINT_GUIDE.md written. Estimated total {total:.0f} g PETG (solid would be {solid:.0f} g).')


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
