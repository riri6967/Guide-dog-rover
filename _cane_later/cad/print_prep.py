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

ORIENTS = {  # name: the part's direction that points UP on the bed
    '+Y up (as used)': (0, 1, 0), '-Y up (upside down)': (0, -1, 0),
    '+Z up (front up)': (0, 0, 1), '-Z up (back up)': (0, 0, -1),
    '+X up (right end up)': (1, 0, 0), '-X up (left end up)': (-1, 0, 0),
}
BED = (220, 220, 250)
# Parts modelled at an angle get their own way up: the rod plug stands on its round end.
CUSTOM = {'Rod Plug': ('standing on its round end', (1, 0, 0), -bh.ROD_ANGLE)}


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

    print('\n=== interference (overlapping volume between parts) ===')
    solids = {'Grip A': gA, 'Grip B': gB, **parts, 'Tube': tube}
    clashes = 0
    for (a, sa), (b, sb) in itertools.combinations(solids.items(), 2):
        v = sa.val().intersect(sb.val()).Volume()
        if v > 0.5: print(f'   {a} x {b}: {v:.1f} mm3'); clashes += 1
    print('   none' if not clashes else f'   {clashes} clashes')

    print('\n=== orientation (support area in mm2 for each way up; lower is better) ===')
    for name, wp in parts.items():
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


if __name__ == '__main__':
    main(sys.argv[1], sys.argv[2])
