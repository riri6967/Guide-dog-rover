#!/usr/bin/env python3
"""Footprint-overlap check from ground-truth traces (diag_failures.py output). Needed because furniture below the LiDAR
plane is invisible to the scan-based collision test. A step is a contact if any point of a 2.5 cm lattice over the
0.90 x 0.76 m footprint lies inside a static collision box (+ margin). Boxes from the SDF via the Nav2-map parser.
usage: /usr/bin/python3 training/gz_env/contact_check.py TRACES.pkl WORLD.sdf [--margin 0.0]
"""
import argparse, contextlib, io, math, pickle, re, sys, xml.etree.ElementTree as ET
from pathlib import Path
import numpy as np
PKG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG / 'scripts'))
from v9_global_planner import StructuralOccupancyMap  # noqa: E402


def named_boxes(sdf):
    """(name, xmin, xmax, ymin, ymax, top_z) for every static box collision (axis-aligned worlds only)."""
    w = ET.parse(sdf).getroot().find('world'); out = []
    for m in w.findall('model'):
        if (m.findtext('static') or '').strip() not in ('true', '1') or 'floor' in m.get('name', ''): continue
        mp = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
        for l in m.findall('link'):
            # visual-only boxes (e.g. assistive_world doorframe posts) count too: physics lets the rover pass, a real one would not
            for c in (l.findall('collision') or l.findall('visual')):
                b = c.find('geometry/box')
                if b is None: continue
                cp = [float(v) for v in (c.findtext('pose') or '0 0 0 0 0 0').split()]
                sx, sy, sz = map(float, b.findtext('size').split()); x, y, z = mp[0] + cp[0], mp[1] + cp[1], mp[2] + cp[2]
                out.append((m.get('name'), x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2, z + sz / 2))
    return out


if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('traces'); ap.add_argument('world'); ap.add_argument('--margin', type=float, default=0.0)
    a = ap.parse_args()
    boxes = named_boxes(a.world)
    gx, gy = np.meshgrid(np.linspace(-0.45, 0.45, 37), np.linspace(-0.38, 0.38, 31)); L = np.c_[gx.ravel(), gy.ravel()]
    res = pickle.load(open(a.traces, 'rb')); eps = [e for v in res.values() for e in v]
    per_obj = {}; ep_contact = 0; first = []
    for k, e in enumerate(eps):
        hit_names = set()
        for t in e['trace']:
            x, y, yaw = t[:3]; c, s = math.cos(yaw), math.sin(yaw)
            P = L @ np.array([[c, s], [-s, c]]) + [x, y]
            for name, x0, x1, y0, y1, top in boxes:
                if x1 + a.margin < P[:, 0].min() or x0 - a.margin > P[:, 0].max() or y1 + a.margin < P[:, 1].min() or y0 - a.margin > P[:, 1].max(): continue
                if np.any((P[:, 0] >= x0 - a.margin) & (P[:, 0] <= x1 + a.margin) & (P[:, 1] >= y0 - a.margin) & (P[:, 1] <= y1 + a.margin)):
                    hit_names.add((name, round(top, 2)))
        for h in hit_names: per_obj[h] = per_obj.get(h, 0) + 1
        ep_contact += bool(hit_names)
        if hit_names and len(first) < 10: first.append((k, e.get('leg'), sorted(hit_names), 'success' if e['is_success'] else 'fail'))
    n = len(eps)
    print(f'{n} episodes, success {sum(e["is_success"] for e in eps)}, scan-collision {sum(e["collision"] for e in eps)}, '
          f'footprint contact (margin {a.margin} m) {ep_contact}')
    for (name, top), cnt in sorted(per_obj.items(), key=lambda kv: -kv[1]):
        print(f'  {name:24s} top {top:.2f} m {"(below LiDAR)" if top < 0.96 else ""}: {cnt} episodes')
    for f in first: print('  e.g. episode', f)
