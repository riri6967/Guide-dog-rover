#!/usr/bin/env python3
"""Trial-only world generator (precursor of PLAN.md Phase 3). One layout -> Gazebo SDF + occupancy grid from the SAME SDF.
Layout: rectangular room W x H, one dividing wall with a door (1.0-1.6 m), 3-7 random boxes.
World name is "train_world"; systems, lights and floor are copied from worlds/assistive_world.sdf; 4 ms physics, unthrottled.
These worlds are for plumbing/throughput trials only. assistive_world stays a held-out test world.

usage: /usr/bin/python3 training/worlds_gen/gen_trial_worlds.py --n 3 --seed 0 [--out training/worlds/trial]
"""
import argparse, random, re, sys
from pathlib import Path
import numpy as np

PKG = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(PKG / 'scripts'))
from v9_global_planner import StructuralOccupancyMap  # noqa: E402  (same parser as the Nav2 map)

WALL_T, WALL_H = 0.2, 2.0
BOUNDS = (-12.0, 12.0, -8.0, 8.0)  # floor is 24 x 16


def box_model(name, x, y, sx, sy, sz, color='0.6 0.6 0.6 1'):
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {sz / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry></collision>
        <visual name="visual"><geometry><box><size>{sx:.3f} {sy:.3f} {sz:.3f}</size></box></geometry>
          <material><ambient>{color}</ambient><diffuse>{color}</diffuse></material></visual>
      </link>
    </model>'''


def layout(rng):
    W, H = rng.uniform(8, 12), rng.uniform(6, 9)
    m = []
    m.append(('wall_n', 0, H / 2, W + WALL_T, WALL_T, WALL_H)); m.append(('wall_s', 0, -H / 2, W + WALL_T, WALL_T, WALL_H))
    m.append(('wall_e', W / 2, 0, WALL_T, H, WALL_H)); m.append(('wall_w', -W / 2, 0, WALL_T, H, WALL_H))
    # dividing wall parallel to y at x = xd, with a door of width dw centred at yd
    xd = rng.uniform(-W / 6, W / 6); dw = rng.uniform(1.0, 1.6); yd = rng.uniform(-H / 2 + 1.0 + dw / 2, H / 2 - 1.0 - dw / 2)
    y_lo, y_hi = -H / 2, H / 2
    a_len = (yd - dw / 2) - y_lo; b_len = y_hi - (yd + dw / 2)
    m.append(('div_a', xd, y_lo + a_len / 2, WALL_T, a_len, WALL_H)); m.append(('div_b', xd, y_hi - b_len / 2, WALL_T, b_len, WALL_H))
    boxes = []
    for i in range(rng.randint(3, 7)):
        for _ in range(50):
            sx, sy = rng.uniform(0.4, 1.2), rng.uniform(0.4, 1.2)
            x, y = rng.uniform(-W / 2 + 0.3 + sx / 2, W / 2 - 0.3 - sx / 2), rng.uniform(-H / 2 + 0.3 + sy / 2, H / 2 - 0.3 - sy / 2)
            if abs(x - xd) < sx / 2 + 1.2 and abs(y - yd) < sy / 2 + dw:  # keep the door approach clear
                continue
            if abs(x - xd) < sx / 2 + 0.2:  # not merged into the dividing wall
                continue
            boxes.append((f'box_{i}', x, y, sx, sy, rng.uniform(0.5, 1.0))); break
    return dict(W=W, H=H, door_x=xd, door_y=yd, door_w=dw), m + boxes


def free_spawn(models, clearance=0.7):
    for x in np.arange(-3.0, 3.01, 0.25):
        for y in np.arange(-2.0, 2.01, 0.25):
            if all(abs(x - mx) > sx / 2 + clearance or abs(y - my) > sy / 2 + clearance for _, mx, my, sx, sy, _ in models):
                return float(x), float(y)
    raise RuntimeError('no free spawn point')


def write_world(path, models, step=0.004):
    src = (PKG / 'worlds' / 'assistive_world.sdf').read_text()
    head = src[:src.index('<!-- OUTER WALLS')]
    head = head.replace('<world name="assistive_world">', '<world name="train_world">')
    head = re.sub(r'(<gravity>[^<]*</gravity>\n)', r'\1    <physics name="train" type="ignored">\n      <max_step_size>'
                  f'{step}' r'</max_step_size>\n      <real_time_factor>0</real_time_factor>\n    </physics>\n', head, count=1)
    head = head.rsplit('<!-- =====', 1)[0]  # drop the dangling section banner
    body = ''.join(box_model(*mm) for mm in models)
    rover = ('\n    <include>\n      <uri>model://assistive_rover_train</uri>\n      <name>assistive_rover</name>\n'
             '      <pose>%.2f %.2f 0 0 0 0</pose>\n    </include>\n  </world>\n</sdf>\n' % free_spawn(models))
    path.write_text(head + body + rover)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--n', type=int, default=3); ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--out', default=str(PKG / 'training' / 'worlds' / 'trial'))
    a = ap.parse_args(); out = Path(a.out); out.mkdir(parents=True, exist_ok=True)
    for k in range(a.n):
        rng = random.Random(a.seed * 1000 + k)
        info, models = layout(rng)
        sdf = out / f'trial_{a.seed}_{k}.sdf'
        write_world(sdf, models)
        omap = StructuralOccupancyMap(world_path=sdf, resolution=0.05, inflation_radius=0.0, bounds=BOUNDS)
        np.savez_compressed(sdf.with_suffix('.npz'), grid=omap.grid.astype(np.uint8), res=0.05,
                            xmin=omap.xmin, ymin=omap.ymin, **{kk: v for kk, v in info.items()})
        print(f"{sdf.name}: room {info['W']:.1f} x {info['H']:.1f} m, door {info['door_w']:.2f} m at ({info['door_x']:.1f},{info['door_y']:.1f}), "
              f"{len(models) - 6} boxes, occupied cells {int(omap.grid.sum())}")


if __name__ == '__main__':
    main()
