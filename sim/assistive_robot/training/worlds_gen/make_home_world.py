#!/usr/bin/env python3
"""Training-format copy of worlds/assistive_world.sdf (held-out HOME test world: never train or tune on it).
Same walls and furniture; world name train_world, 4 ms unthrottled physics, rover model assistive_rover_train (no camera).
Grid: exact rasterization (cell centre inside) of the SDF collision AABBs read by the same parser as the Nav2 map,
INCLUDING the furniture, which is all below the LiDAR plane (0.96 m) and therefore invisible to the scan.
usage: /usr/bin/python3 training/worlds_gen/make_home_world.py
"""
import contextlib, io, re, sys
from pathlib import Path
import numpy as np
PKG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG / 'scripts'))
from v9_global_planner import StructuralOccupancyMap  # noqa: E402
BOUNDS, RES = (-12.0, 12.0, -8.0, 8.0), 0.05

if __name__ == '__main__':
    src = (PKG / 'worlds' / 'assistive_world.sdf').read_text()
    s = src.replace('<world name="assistive_world">', '<world name="train_world">')
    s = re.sub(r'(<gravity>[^<]*</gravity>\n)', r'\1    <physics name="train" type="ignored">\n      <max_step_size>0.004'
               r'</max_step_size>\n      <real_time_factor>0</real_time_factor>\n    </physics>\n', s, count=1)
    s = s.replace('<uri>model://assistive_rover</uri>', '<uri>model://assistive_rover_train</uri>')
    out = PKG / 'training' / 'worlds' / 'home'; out.mkdir(parents=True, exist_ok=True)
    sdf = out / 'assistive_home.sdf'; sdf.write_text(s)
    with contextlib.redirect_stdout(io.StringIO()):
        om = StructuralOccupancyMap(world_path=sdf, resolution=RES, inflation_radius=0.0, bounds=BOUNDS)
    nx, ny = int(round((BOUNDS[1] - BOUNDS[0]) / RES)), int(round((BOUNDS[3] - BOUNDS[2]) / RES))
    X, Y = np.meshgrid(BOUNDS[0] + (np.arange(nx) + 0.5) * RES, BOUNDS[2] + (np.arange(ny) + 0.5) * RES)
    g = np.zeros((ny, nx), bool)
    boxes = list(om.raw_boxes)
    # visual-only static boxes that reach the LiDAR plane (the 4 doorframe posts): no collision, so the parser skips
    # them, but the LiDAR sees them and Nav2's costmap marks them, so the planner/subgoal must know them too
    import xml.etree.ElementTree as ET
    for m in ET.parse(sdf).getroot().find('world').findall('model'):
        if (m.findtext('static') or '').strip() not in ('true', '1'): continue
        for l in m.findall('link'):
            if l.find('collision') is not None: continue
            mp = [float(v) for v in (m.findtext('pose') or '0 0 0 0 0 0').split()]
            for v in l.findall('visual'):
                b = v.find('geometry/box')
                if b is None: continue
                vp = [float(t) for t in (v.findtext('pose') or '0 0 0 0 0 0').split()]
                sx, sy, sz = map(float, b.findtext('size').split()); x, y, z = mp[0] + vp[0], mp[1] + vp[1], mp[2] + vp[2]
                if z - sz / 2 <= 0.96 <= z + sz / 2:
                    boxes.append((x - sx / 2, x + sx / 2, y - sy / 2, y + sy / 2)); print('visual-only obstacle added:', m.get('name'))
    for x0, x1, y0, y1 in boxes: g |= (X >= x0) & (X <= x1) & (Y >= y0) & (Y <= y1)
    np.savez_compressed(sdf.with_suffix('.npz'), grid=g.astype(np.uint8), res=RES, xmin=BOUNDS[0], ymin=BOUNDS[2], centred=True, tier=-1)
    print(sdf, 'boxes', len(om.raw_boxes), 'occupied cells', int(g.sum()))
