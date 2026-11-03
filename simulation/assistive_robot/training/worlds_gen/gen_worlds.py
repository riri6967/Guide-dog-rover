#!/usr/bin/env python3
"""PLAN.md Phase 3 world generator. One layout (list of static boxes/cylinders) -> Gazebo SDF + exact 0.05 m occupancy grid
(cell centre inside a primitive = occupied). Both come from the same primitive list, so they cannot drift apart.

Tiers (curriculum, PLAN.md: empty -> sparse obstacles -> rooms + doors -> clutter):
  0 open     one room 6-14 x 5-10 m, 0-3 obstacles
  1 sparse   one room 8-16 x 6-12 m, 4-10 obstacles (boxes, cylinders, thin poles, free-standing wall pieces)
  2 rooms    building 12-20 x 8-13 m: corridor 1.2-3.0 m (straight / dead-end / L / T), rooms 2.8-6 m on the sides,
             doors 0.9-2.0 m (85% >= 1.0 m, 15% hard 0.9-1.0 m), 0-1 obstacles per room
  3 clutter  as 2 plus 2-5 obstacles per room and some in wide corridors
All obstacles are 1.2-1.8 m tall: the LiDAR is 0.96 m above the floor, anything lower is invisible to it.
World name "train_world"; systems/lights/floor copied from worlds/assistive_world.sdf (floor 24 x 16 m); 4 ms physics.

usage: /usr/bin/python3 training/worlds_gen/gen_worlds.py [--out training/worlds/gen] [--check-parser]
"""
import argparse, json, math, random, re, sys
from pathlib import Path
import numpy as np
from scipy import ndimage

PKG = Path(__file__).resolve().parents[2]
WALL_T, WALL_H = 0.2, 2.0
RES = 0.05
BOUNDS = (-12.0, 12.0, -8.0, 8.0)
HALF_W = 0.38                       # rover half width (footprint 0.90 x 0.76)
SPLITS = {'train': (10000, {0: 30, 1: 50, 2: 60, 3: 60}),
          'val': (20000, {0: 6, 1: 8, 2: 8, 3: 8}),
          'test': (30000, {0: 6, 1: 8, 2: 8, 3: 8})}


# ---------------------------------------------------------------- primitives: ('box', x, y, sx, sy, h) | ('cyl', x, y, r, h)
def aabb(p):
    if p[0] == 'box': return p[1] - p[3] / 2, p[1] + p[3] / 2, p[2] - p[4] / 2, p[2] + p[4] / 2
    return p[1] - p[3], p[1] + p[3], p[2] - p[3], p[2] + p[3]


def gap(a, b):
    """Euclidean separation of two AABBs (0 if they touch/overlap)."""
    dx = max(a[0] - b[1], b[0] - a[1], 0.0); dy = max(a[2] - b[3], b[2] - a[3], 0.0)
    return math.hypot(dx, dy)


def door_width(rng):
    return rng.uniform(0.9, 1.0) if rng.random() < 0.15 else rng.uniform(1.0, 2.0)


def random_obstacle(rng, x0, x1, y0, y1, kinds=('box', 'box', 'cyl', 'pole')):
    k = rng.choice(kinds); h = rng.uniform(1.2, 1.8)
    if k == 'box':
        sx, sy = rng.uniform(0.3, 1.2), rng.uniform(0.3, 1.2)
        return ('box', rng.uniform(x0 + sx / 2, x1 - sx / 2), rng.uniform(y0 + sy / 2, y1 - sy / 2), sx, sy, h)
    if k == 'wallpiece':
        L = rng.uniform(1.0, 3.0); sx, sy = (L, WALL_T) if rng.random() < 0.5 else (WALL_T, L)
        if sx > x1 - x0 or sy > y1 - y0: return None
        return ('box', rng.uniform(x0 + sx / 2, x1 - sx / 2), rng.uniform(y0 + sy / 2, y1 - sy / 2), sx, sy, WALL_H)
    r = rng.uniform(0.15, 0.4) if k == 'cyl' else rng.uniform(0.04, 0.08)
    return ('cyl', rng.uniform(x0 + r, x1 - r), rng.uniform(y0 + r, y1 - r), r, h)


def place(rng, prims, keepout, n, x0, x1, y0, y1, kinds=('box', 'box', 'cyl', 'pole')):
    """Add up to n obstacles inside [x0,x1]x[y0,y1]: >= 0.1 m from walls/obstacles (no hidden slivers), outside keepouts."""
    added = 0
    for _ in range(n * 30):
        if added >= n: break
        if x1 - x0 < 1.0 or y1 - y0 < 1.0: break
        p = random_obstacle(rng, x0, x1, y0, y1, kinds)
        if p is None: continue
        a = aabb(p)
        if any(gap(a, aabb(q)) < 0.1 for q in prims): continue
        if any(gap(a, k) < 0.05 for k in keepout): continue
        prims.append(p); added += 1
    return added


# ---------------------------------------------------------------- tier 0/1: single room
def room_layout(rng, tier):
    W, H = (rng.uniform(6, 14), rng.uniform(5, 10)) if tier == 0 else (rng.uniform(8, 16), rng.uniform(6, 12))
    prims = outer_walls(W, H)
    n = rng.randint(0, 3) if tier == 0 else rng.randint(4, 10)
    kinds = ('box', 'box', 'cyl', 'pole') if tier == 0 else ('box', 'box', 'cyl', 'pole', 'wallpiece')
    i = 0.1 + WALL_T / 2
    place(rng, prims, [], n, -W / 2 + i, W / 2 - i, -H / 2 + i, H / 2 - i, kinds)
    return dict(tier=tier, kind='room', W=W, H=H, doors=[]), prims


def outer_walls(W, H):
    return [('box', 0, H / 2, W + WALL_T, WALL_T, WALL_H), ('box', 0, -H / 2, W + WALL_T, WALL_T, WALL_H),
            ('box', W / 2, 0, WALL_T, H, WALL_H), ('box', -W / 2, 0, WALL_T, H, WALL_H)]


# ---------------------------------------------------------------- tier 2/3: corridor building from tiles
def cuts(rng, a, b, forced=()):
    """Split [a, b] into rooms 2.8-6 m wide; forced positions are always cuts."""
    pts = sorted({a, b, *forced}); out = [a]
    for lo, hi in zip(pts[:-1], pts[1:]):
        x = lo
        while hi - x > 6.0:
            x += rng.uniform(2.8, min(6.0, hi - x - 2.8)) if hi - x - 2.8 > 2.8 else (hi - x) / 2
            out.append(x)
        out.append(hi)
    return sorted(set(round(v, 4) for v in out))


def building_layout(rng, tier):
    W, H = rng.uniform(12, 20), rng.uniform(8, 13)
    cw = rng.uniform(1.2, 3.0)
    x0, x1, y0, y1 = -W / 2, W / 2, -H / 2, H / 2
    two_sided = H - cw >= 2 * 2.8
    yc = rng.uniform(y0 + 2.8 + cw / 2, y1 - 2.8 - cw / 2) if two_sided else y0 + cw / 2
    b0, b1 = yc - cw / 2, yc + cw / 2
    kind = rng.choice(['straight', 'dead_end', 'L', 'T']) if two_sided else rng.choice(['straight', 'dead_end'])
    tiles = []  # (x0, x1, y0, y1, space)
    no_door = set()  # (space_a, space_b, orientation, coord) boundaries that must stay closed
    if kind in ('L', 'T'):
        cw2 = rng.uniform(1.2, 3.0); xb = rng.uniform(x0 + 3.0 + cw2 / 2, x1 - 3.0 - cw2 / 2)
        bx0, bx1 = xb - cw2 / 2, xb + cw2 / 2
        tiles.append((bx0, bx1, b1, y1, 'C'))
        if kind == 'T':
            tiles.append((x0, x1, b0, b1, 'C')); top_forced = (bx0, bx1); bot_forced = ()
        else:
            tiles.append((x0, bx1, b0, b1, 'C')); top_forced = (bx0, bx1); bot_forced = (bx1,)
    elif kind == 'dead_end':
        xe = rng.uniform(x0 + 0.55 * W, x0 + 0.8 * W)
        tiles.append((x0, xe, b0, b1, 'C')); top_forced = (); bot_forced = (xe,) if two_sided else ()
        if not two_sided: top_forced = (xe,)
    else:
        tiles.append((x0, x1, b0, b1, 'C')); top_forced = bot_forced = ()
    rid = 0
    top = cuts(rng, x0, x1, top_forced)
    for lo, hi in zip(top[:-1], top[1:]):
        if kind in ('L', 'T') and abs(lo - top_forced[0]) < 1e-3: continue  # the vertical corridor
        tiles.append((lo, hi, b1, y1, f'R{rid}')); rid += 1
    if two_sided:
        bot = cuts(rng, x0, x1, bot_forced)
        for lo, hi in zip(bot[:-1], bot[1:]):
            tiles.append((lo, hi, y0, b0, f'R{rid}')); rid += 1
    # leftover piece of the corridor band (dead end / L): joins the room below (or above if one-sided)
    if kind == 'dead_end' or kind == 'L':
        xs = xe if kind == 'dead_end' else bx1
        below = [t for t in tiles if t[4] != 'C' and t[2] == (y0 if two_sided else b1) and abs(t[0] - xs) < 1e-3]
        sp = below[0][4]; tiles.append((xs, x1, b0, b1, sp))
        no_door.add(('C', sp))  # corridor end stays a wall: a real dead end (room keeps a door elsewhere)
    return assemble(rng, tier, dict(tier=tier, kind=kind, W=W, H=H, corridor_w=cw), tiles, no_door)


def shared_edges(tiles):
    """Boundary segments between tiles of different spaces: (orient, coord, lo, hi, space_a, space_b)."""
    out = []
    for i, a in enumerate(tiles):
        for b in tiles[i + 1:]:
            if a[4] == b[4]: continue
            for (p, q) in ((a, b), (b, a)):
                if abs(p[1] - q[0]) < 1e-6:  # vertical edge at x = p.x1
                    lo, hi = max(p[2], q[2]), min(p[3], q[3])
                    if hi - lo > 1e-6: out.append(('v', p[1], lo, hi, a[4], b[4]))
                if abs(p[3] - q[2]) < 1e-6:  # horizontal edge at y = p.y1
                    lo, hi = max(p[0], q[0]), min(p[1], q[1])
                    if hi - lo > 1e-6: out.append(('h', p[3], lo, hi, a[4], b[4]))
    return out


def assemble(rng, tier, info, tiles, no_door):
    W, H = info['W'], info['H']
    edges = shared_edges(tiles)
    rooms = sorted({t[4] for t in tiles if t[4] != 'C'})
    doors = {}  # edge index -> (centre, width)
    m = 0.35 + WALL_T / 2                        # door edge to wall junction
    def ok_edge(e, dw): return e[3] - e[2] >= dw + 2 * m
    for r in rooms:  # one corridor door per room (random eligible edge)
        cand = [k for k, e in enumerate(edges) if {e[4], e[5]} == {'C', r} and frozenset(('C', r)) not in
                {frozenset(x) for x in no_door}]
        rng.shuffle(cand)
        for k in cand:
            dw = door_width(rng)
            if ok_edge(edges[k], dw): doors[k] = (rng.uniform(edges[k][2] + m + dw / 2, edges[k][3] - m - dw / 2), dw); break
    for k, e in enumerate(edges):  # some room-room doors (always one for rooms with no corridor door)
        if 'C' in (e[4], e[5]) or k in doors: continue
        need = any(not any(r in (edges[j][4], edges[j][5]) for j in doors) for r in (e[4], e[5]))
        if need or rng.random() < 0.3:
            dw = door_width(rng)
            if ok_edge(e, dw): doors[k] = (rng.uniform(e[2] + m + dw / 2, e[3] - m - dw / 2), dw)
    prims = outer_walls(W, H)
    keepout = []
    for k, e in enumerate(edges):
        segs = [(e[2], e[3])]
        if k in doors:
            c, dw = doors[k]; segs = [(e[2], c - dw / 2), (c + dw / 2, e[3])]
            # keep 1.2 m in front of the door clear on both sides (furniture placement only)
            if e[0] == 'v': keepout.append((e[1] - 1.2, e[1] + 1.2, c - dw / 2 - 0.4, c + dw / 2 + 0.4))
            else: keepout.append((c - dw / 2 - 0.4, c + dw / 2 + 0.4, e[1] - 1.2, e[1] + 1.2))
        for lo, hi in segs:
            if hi - lo < 1e-3: continue
            if e[0] == 'v': prims.append(('box', e[1], (lo + hi) / 2, WALL_T, hi - lo, WALL_H))
            else: prims.append(('box', (lo + hi) / 2, e[1], hi - lo, WALL_T, WALL_H))
    i = 0.1 + WALL_T / 2
    for t in tiles:
        if t[4] == 'C':
            if tier == 3 and min(t[1] - t[0], t[3] - t[2]) >= 2.4:  # wide corridor: a few small items, >= 1.2 m passage left
                place(rng, prims, keepout, rng.randint(0, 2), t[0] + i, t[1] - i, t[2] + i, t[3] - i, ('pole', 'cyl'))
            continue
        n = rng.randint(0, 1) if tier == 2 else rng.randint(2, 5)
        place(rng, prims, keepout, n, t[0] + i, t[1] - i, t[2] + i, t[3] - i)
    info['doors'] = [round(dw, 3) for _, dw in doors.values()]
    info['rooms'] = len(rooms)
    return info, prims


# ---------------------------------------------------------------- grid, spawn, SDF
def rasterize(prims):
    xmin, xmax, ymin, ymax = BOUNDS
    nx, ny = int(round((xmax - xmin) / RES)), int(round((ymax - ymin) / RES))
    X, Y = np.meshgrid(xmin + (np.arange(nx) + 0.5) * RES, ymin + (np.arange(ny) + 0.5) * RES)
    g = np.zeros((ny, nx), bool)
    for p in prims:
        if p[0] == 'box': g |= (np.abs(X - p[1]) <= p[3] / 2) & (np.abs(Y - p[2]) <= p[4] / 2)
        else: g |= (X - p[1]) ** 2 + (Y - p[2]) ** 2 <= p[3] ** 2
    return g


def analyse(g):
    """Same passability test as gz_nav_env.WorldGrid (fine clearance, min over 0.1 m cells, > half width + 1 cm), so the
    generator rejects exactly the layouts the env would see as cut off. Returns the largest inner region of 0.1 m cells."""
    f = int(round(0.1 / RES)); h, w = (g.shape[0] // f) * f, (g.shape[1] // f) * f
    fine = ndimage.distance_transform_edt(~g[:h, :w]) * RES - RES / 2
    dist = fine.reshape(h // f, f, w // f, f).min(axis=(1, 3))
    passable = dist > HALF_W + 0.01
    lab, n = ndimage.label(passable)
    edge = set(np.unique(np.r_[lab[0], lab[-1], lab[:, 0], lab[:, -1]]))  # labels touching the bounds are outside
    inside = [k for k in range(1, n + 1) if k not in edge]
    ok = dist > 0.65
    sizes = {k: int((ok & (lab == k)).sum()) for k in inside}
    if not sizes: return None
    best = max(sizes, key=sizes.get)
    return dict(dist=dist, region=lab == best, frac=sizes[best] / max(1, sum(sizes.values())), n_ok=sizes[best], res=f * RES)


def pick_spawn(rng, an):
    rr, cc = np.nonzero(an['region'] & (an['dist'] > 0.8))
    k = rng.randrange(len(rr))
    return BOUNDS[0] + (cc[k] + 0.5) * an['res'], BOUNDS[2] + (rr[k] + 0.5) * an['res']


def sdf_prim(name, p):
    if p[0] == 'box':
        _, x, y, sx, sy, h = p; geo = f'<box><size>{sx:.3f} {sy:.3f} {h:.3f}</size></box>'; col = '0.6 0.6 0.6 1'
    else:
        _, x, y, r, h = p; geo = f'<cylinder><radius>{r:.3f}</radius><length>{h:.3f}</length></cylinder>'; col = '0.5 0.4 0.3 1'
    return f'''
    <model name="{name}">
      <static>true</static>
      <pose>{x:.3f} {y:.3f} {h / 2:.3f} 0 0 0</pose>
      <link name="link">
        <collision name="collision"><geometry>{geo}</geometry></collision>
        <visual name="visual"><geometry>{geo}</geometry>
          <material><ambient>{col}</ambient><diffuse>{col}</diffuse></material></visual>
      </link>
    </model>'''


def write_world(path, prims, spawn, step=0.004):
    src = (PKG / 'worlds' / 'assistive_world.sdf').read_text()
    head = src[:src.index('<!-- OUTER WALLS')]
    head = head.replace('<world name="assistive_world">', '<world name="train_world">')
    head = re.sub(r'(<gravity>[^<]*</gravity>\n)', r'\1    <physics name="train" type="ignored">\n      <max_step_size>'
                  f'{step}' r'</max_step_size>\n      <real_time_factor>0</real_time_factor>\n    </physics>\n', head, count=1)
    head = head.rsplit('<!-- =====', 1)[0]
    body = ''.join(sdf_prim(f'obj_{i}', p) for i, p in enumerate(prims))
    rover = ('\n    <include>\n      <uri>model://assistive_rover_train</uri>\n      <name>assistive_rover</name>\n'
             '      <pose>%.2f %.2f 0 0 0 0</pose>\n    </include>\n  </world>\n</sdf>\n' % spawn)
    path.write_text(head + body + rover)


def generate(seed, tier):
    for attempt in range(100):
        rng = random.Random(seed * 100 + attempt)
        info, prims = room_layout(rng, tier) if tier <= 1 else building_layout(rng, tier)
        g = rasterize(prims); an = analyse(g)
        # reject layouts where a room is cut off (e.g. by a hard door the grid calls impassable, or by furniture)
        if an is None or an['frac'] < 0.97 or an['n_ok'] < 500: continue
        info.update(attempt=attempt, reachable_frac=round(an['frac'], 3))
        return info, prims, g, pick_spawn(rng, an)
    raise RuntimeError(f'no valid layout for seed {seed} tier {tier}')


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default=str(PKG / 'training' / 'worlds' / 'gen'))
    ap.add_argument('--check-parser', action='store_true', help='cross-check grids against the SDF parser used for the Nav2 map')
    a = ap.parse_args()
    manifest = []
    for split, (base, counts) in SPLITS.items():
        d = Path(a.out) / split; d.mkdir(parents=True, exist_ok=True)
        for tier, n in counts.items():
            for k in range(n):
                seed = base + tier * 1000 + k
                info, prims, g, spawn = generate(seed, tier)
                sdf = d / f'w{tier}_{k:03d}.sdf'
                write_world(sdf, prims, spawn)
                np.savez_compressed(sdf.with_suffix('.npz'), grid=g.astype(np.uint8), res=RES, xmin=BOUNDS[0], ymin=BOUNDS[2],
                                    centred=True, tier=tier)
                info.update(split=split, file=str(sdf.relative_to(PKG)), seed=seed, spawn=[round(spawn[0], 2), round(spawn[1], 2)],
                            n_prims=len(prims))
                manifest.append(info)
                if a.check_parser:
                    sys.path.insert(0, str(PKG / 'scripts'))
                    from v9_global_planner import StructuralOccupancyMap
                    import contextlib, io
                    with contextlib.redirect_stdout(io.StringIO()):
                        om = StructuralOccupancyMap(world_path=sdf, resolution=RES, inflation_radius=0.0, bounds=BOUNDS)
                    pg = om.grid.astype(bool)[:g.shape[0], :g.shape[1]]
                    miss = int((g & ~pg).sum())  # cells occupied in our grid but free in the parser's (should be ~0)
                    info['parser_missing_cells'] = miss
        print(split, {t: sum(1 for m in manifest if m['split'] == split and m['tier'] == t) for t in counts})
    (Path(a.out) / 'manifest.json').write_text(json.dumps(manifest, indent=1))
    for t in range(4):
        ms = [m for m in manifest if m['tier'] == t]
        drs = [d for m in ms for d in m['doors']]
        print(f"tier {t}: {len(ms)} worlds, mean prims {np.mean([m['n_prims'] for m in ms]):.0f}, "
              f"kinds {sorted({m['kind'] for m in ms})}, doors {len(drs)}"
              + (f" ({min(drs):.2f}-{max(drs):.2f} m, {sum(d < 1.0 for d in drs)} < 1.0 m)" if drs else '')
              + (f", parser-missing cells max {max(m.get('parser_missing_cells', 0) for m in ms)}" if a.check_parser else ''))


if __name__ == '__main__':
    main()
