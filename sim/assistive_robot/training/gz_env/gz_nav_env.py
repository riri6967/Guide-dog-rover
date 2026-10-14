#!/usr/bin/env python3
"""Gazebo navigation env (system python 3.12 only: needs gz-transport bindings). One instance = one headless Gazebo server
in lockstep (paused; every step advances exactly CONTROL_DT). PLAN.md Phase 4 spec, starting values.

State for training comes from Gazebo ground truth (not odom). Collision = a scan point inside the footprint + 2 cm.
Goal/progress use the geodesic distance to the goal on the world's occupancy grid; the observation gets a subgoal
~1.5 m ahead along that distance field (stand-in for the planner path used at deployment).
"""
import math, os, random, signal, subprocess, threading, time
from collections import deque
from pathlib import Path
import numpy as np
from scipy import ndimage
from scipy.sparse import coo_matrix
from scipy.sparse.csgraph import dijkstra
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl

PKG = Path(__file__).resolve().parents[2]
CONTROL_DT = 0.1
V_MAX, W_MAX = 0.3, 1.0                   # same limits as Nav2
HALF_L, HALF_W, MARGIN = 0.45, 0.38, 0.02  # footprint 0.90 x 0.76 m
N_BEAMS, POOL, STACK, R_MAX = 360, 4, 4, 6.0
OBS_DIM = (N_BEAMS // POOL) * STACK + 3 + 2 + 2  # 367
PLAN_RES = 0.1
CFG = dict(max_steps=400, goal_tol=0.25, goal_min=2.0, goal_max=10.0, subgoal_ahead=1.5, stuck_steps=60, stuck_progress=0.10,
           r_progress=10.0, r_goal=10.0, r_collision=-10.0, r_step=-0.01, r_prox=-0.2, prox_dist=0.25, r_smooth=-0.02,
           prox_from='footprint', stop_zone=(0.0, 0.55, 0.33, 4), v_min=0.0, near_wall_goal_p=0.0, tour=None, tour_start=(0.0, 0.0, 0.0), collision_terminates=True)
# stop_zone (x_min, x_max, half_width, min_points): also a collision when >= min_points scan points are inside this robot-frame
# box = Nav2 Collision Monitor PolygonStop (nav2_params.yaml). On the real stack it freezes the robot for good (run 2 was
# trapped twice); None = old rule (footprint + 2 cm only, runs 1 and 2).
# v_min: lowest commanded speed (action -1 -> v_min, +1 -> V_MAX); 0.0 = runs 1-3 (no reverse). Measured v is signed
# (forward projection of the GT displacement), identical to the old unsigned value when driving forward.
# near_wall_goal_p: share of episodes whose goal is drawn from cells with only > 0.42 m clearance (near walls, like
# assistive_world candidate_5 at 0.40 m) instead of > 0.65 m. 0.0 = runs 1-3.
# tour: list of (x, y) goals visited back-to-back without teleporting (first reset teleports to tour_start); like the Nav2
# evaluator, a leg is judged at its end and the next leg starts wherever the rover is. Set via env var GZNAV_CFG (JSON).
# prox_from: 'footprint' = clearance from the 0.90 x 0.76 m body edge (2026-10-10, after sac_gen_20261010_113224 showed 35/39
# collisions were side hits on door jambs); 'centre' = old rule (min scan range from the LiDAR, r_prox -0.1, prox_dist 0.5),
# which barely fired: a point 2 cm from the side is 0.40 m from the centre.


def _requester(conn, world):
    """Service requests from a process without subscriptions (gz Node.request() keeps the GIL while waiting)."""
    n = Node()
    while True:
        msg = conn.recv()
        if msg is None: return
        kind, payload = msg
        if kind == 'wait':
            t0 = time.time()
            while time.time() - t0 < 60 and not set(payload) <= set(n.service_list()): time.sleep(0.2)
            conn.send(set(payload) <= set(n.service_list())); continue
        if kind == 'marker':  # viewer only: flat green disc at the goal (2 cm tall, far below the LiDAR plane at 0.96 m)
            from gz.msgs10.entity_factory_pb2 import EntityFactory
            from gz.msgs10.entity_pb2 import Entity
            e = Entity(); e.name = 'goal_marker'; e.type = Entity.MODEL
            n.request(f'/world/{world}/remove', e, Entity, Boolean, 2000)
            f = EntityFactory(); f.sdf = ('<sdf version="1.9"><model name="goal_marker"><static>true</static><pose>%.3f %.3f 0.01 0 0 0</pose>'
                                         '<link name="l"><visual name="v"><geometry><cylinder><radius>0.25</radius><length>0.02</length>'
                                         '</cylinder></geometry><material><ambient>0 1 0 1</ambient><diffuse>0 1 0 1</diffuse></material>'
                                         '</visual></link></model></sdf>' % payload)
            ok, _ = n.request(f'/world/{world}/create', f, EntityFactory, Boolean, 2000)
            conn.send(ok); continue
        if kind == 'control':
            req = WorldControl(); req.pause = True; req.multi_step = payload
            ok, _ = n.request(f'/world/{world}/control', req, WorldControl, Boolean, 3000)
        else:
            x, y, yaw = payload; p = Pose(); p.name = 'assistive_rover'
            p.position.x, p.position.y, p.position.z = x, y, 0.02
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            ok, _ = n.request(f'/world/{world}/set_pose', p, Pose, Boolean, 3000)
        conn.send(ok)


class WorldGrid:
    """Occupancy grid of one world: free-space sampling and geodesic distance fields."""

    def __init__(self, npz, spawn_xy):
        z = np.load(npz); g = z['grid'].astype(bool); res = float(z['res'])
        f = int(round(PLAN_RES / res))
        h, w = (g.shape[0] // f) * f, (g.shape[1] // f) * f
        self.occ = g[:h, :w].reshape(h // f, f, w // f, f).any(axis=(1, 3))  # 0.1 m cells, conservative
        self.x0, self.y0 = float(z['xmin']), float(z['ymin'])
        # clearance on the fine grid (m from a cell centre to the nearest obstacle surface), then min over each 0.1 m cell:
        # the coarse any() grid alone narrows a 0.9 m door to ~0.8 m and would call it impassable
        fine = ndimage.distance_transform_edt(~g[:h, :w]) * res - res / 2
        dist = fine.reshape(h // f, f, w // f, f).min(axis=(1, 3))
        self.passable = dist > 0.39          # rover half-width 0.38 + 1 cm
        # subgoal field cost: up to 5x per metre within 0.9 m of obstacles, so paths keep to the middle of free space
        cost = 1.0 + 4.0 * np.clip((0.9 - dist) / 0.5, 0.0, 1.0)
        sx, sy = self.cell(*spawn_xy)
        lab, _ = ndimage.label(self.passable)
        self.region = lab == lab[sy, sx]     # the room the rover is in (excludes the outside of the walls)
        self.sample_ok = self.region & (dist > 0.65)  # start/goal cells: clear of the corner reach (0.59 m)
        self.cells = np.argwhere(self.sample_ok)
        self.near_cells = np.argwhere(self.region & (dist > 0.42) & (dist <= 0.65))  # near-wall goal candidates
        idx = -np.ones(self.occ.shape, int); rr, cc = np.nonzero(self.region); idx[rr, cc] = np.arange(len(rr))
        rows, cols, wts, wcs = [], [], [], []
        H, W = idx.shape
        for dr, dc in ((0, 1), (1, 0), (1, 1), (1, -1)):  # each undirected 8-neighbour pair once
            c0, c1 = max(0, -dc), W - max(0, dc)
            a = idx[0:H - dr, c0:c1]; b = idx[dr:H, c0 + dc:c1 + dc]
            m = (a >= 0) & (b >= 0)
            L = math.hypot(dr, dc) * PLAN_RES
            rows.append(a[m]); cols.append(b[m]); wts.append(np.full(m.sum(), L))
            ca = cost[0:H - dr, c0:c1][m]; cb = cost[dr:H, c0 + dc:c1 + dc][m]; wcs.append(L * (ca + cb) / 2)
        r, c, wt, wc = np.concatenate(rows), np.concatenate(cols), np.concatenate(wts), np.concatenate(wcs)
        self.graph = coo_matrix((np.r_[wt, wt], (np.r_[r, c], np.r_[c, r])), shape=(len(rr), len(rr))).tocsr()      # metres
        self.graph_c = coo_matrix((np.r_[wc, wc], (np.r_[r, c], np.r_[c, r])), shape=(len(rr), len(rr))).tocsr()  # clearance-weighted
        self.idx, self.rr, self.cc = idx, rr, cc

    def cell(self, x, y):
        return (int(np.clip((x - self.x0) / PLAN_RES, 0, self.occ.shape[1] - 1)),
                int(np.clip((y - self.y0) / PLAN_RES, 0, self.occ.shape[0] - 1)))

    def xy(self, r, c):
        return self.x0 + (c + 0.5) * PLAN_RES, self.y0 + (r + 0.5) * PLAN_RES

    def field(self, gx, gy, weighted=False):
        cx, cy = self.cell(gx, gy); src = self.idx[cy, cx]
        if src < 0:  # goal cell not passable (e.g. a goal close to a wall): start the field from the nearest region cell
            k = np.argmin((self.rr - cy) ** 2 + (self.cc - cx) ** 2); src = self.idx[self.rr[k], self.cc[k]]
        d = dijkstra(self.graph_c if weighted else self.graph, indices=src)
        D = np.full(self.occ.shape, np.inf); D[self.rr, self.cc] = d
        return D

    def geo(self, D, x, y):
        cx, cy = self.cell(x, y); v = D[cy, cx]
        if np.isfinite(v): return float(v)
        win = D[max(0, cy - 3):cy + 4, max(0, cx - 3):cx + 4]  # centre in the 0.30 m wall band: use nearest valid
        return float(win[np.isfinite(win)].min()) if np.isfinite(win).any() else float('inf')

    def subgoal(self, D, x, y, ahead):
        cx, cy = self.cell(x, y)
        if not np.isfinite(D[cy, cx]):
            win = np.where(np.isfinite(D[max(0, cy - 3):cy + 4, max(0, cx - 3):cx + 4]))
            if len(win[0]) == 0: return None
            cy, cx = max(0, cy - 3) + win[0][0], max(0, cx - 3) + win[1][0]
        travelled = 0.0
        while travelled < ahead and D[cy, cx] > 0:
            best = None
            for dr in (-1, 0, 1):
                for dc in (-1, 0, 1):
                    r, c = cy + dr, cx + dc
                    if (dr or dc) and 0 <= r < D.shape[0] and 0 <= c < D.shape[1] and D[r, c] < (D[best] if best else D[cy, cx]):
                        best = (r, c)
            if best is None: break
            travelled += math.hypot(best[0] - cy, best[1] - cx) * PLAN_RES; cy, cx = best
        return self.xy(cy, cx)


class GzNavEnv:
    def __init__(self, world_sdf, world_npz, instance=0, seed=0, phys_step=0.004, world_name='train_world', cfg=None):
        import json
        self.cfg = dict(CFG, **json.loads(os.environ.get('GZNAV_CFG', '{}')), **(cfg or {}))
        self.world = world_name; self.rng = random.Random(seed); self.leg = 0
        self.trace_on = os.environ.get('GZNAV_TRACE') == '1'  # diagnostics: per-step trace in the terminal info
        self.iters = max(1, round(CONTROL_DT / phys_step)); assert abs(self.iters * phys_step - CONTROL_DT) < 1e-9
        part = f'gzenv{instance}_{os.getpid()}'
        env = dict(os.environ, GZ_PARTITION=part, GZ_IP='127.0.0.1',
                   GZ_SIM_RESOURCE_PATH=f"{PKG/'training'/'models'}:{PKG/'models'}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}")
        os.environ.update(GZ_PARTITION=part, GZ_IP='127.0.0.1')
        logdir = Path('/tmp/claude-1000') if Path('/tmp/claude-1000').is_dir() else Path('/tmp')
        self.srv = subprocess.Popen(['gz', 'sim', '-s', '--headless-rendering', str(world_sdf)], env=env, start_new_session=True,
                                    stdout=open(logdir / f'gzenv_{instance}.log', 'w'), stderr=subprocess.STDOUT)
        import multiprocessing as mp
        ctx = mp.get_context('spawn'); self._pc, child = ctx.Pipe()
        self._helper = ctx.Process(target=_requester, args=(child, world_name), daemon=True); self._helper.start()
        self.cv = threading.Condition(); self.S = {'scan': None, 'scan_t': -1.0, 'gt': None}
        self.node = Node()
        self.node.subscribe(LaserScan, '/scan', self._on_scan)
        self.node.subscribe(Pose_V, f'/world/{world_name}/dynamic_pose/info', self._on_pose)
        self.pub = self.node.advertise('/cmd_vel', Twist)
        spawn = self._spawn_xy(world_sdf)
        self.grid = WorldGrid(world_npz, spawn)
        if not self._rq('wait', [f'/world/{world_name}/control', f'/world/{world_name}/set_pose']):
            raise RuntimeError('gz services did not appear')
        t0 = time.time()
        while time.time() - t0 < 60:
            if self._advance(3) and self._wait_scan(0.0, 1.0) is not None: break
        if self.S['scan'] is None: raise RuntimeError('no scan from gz')
        self.frames = deque(maxlen=STACK); self.D = None

    @staticmethod
    def _spawn_xy(world_sdf):
        txt = Path(world_sdf).read_text(); seg = txt[txt.index('assistive_rover_train'):]
        x, y = seg.split('<pose>')[1].split()[:2]; return float(x), float(y)

    # ---- gz plumbing
    def _rq(self, kind, payload):
        self._pc.send((kind, payload)); return self._pc.recv()

    def _advance(self, k=1):
        return self._rq('control', self.iters * k)

    def _on_scan(self, m):
        with self.cv:
            self.S['scan'], self.S['scan_t'] = m, m.header.stamp.sec + m.header.stamp.nsec * 1e-9; self.cv.notify_all()

    def _on_pose(self, m):
        for p in m.pose:
            if p.name == 'assistive_rover':
                q = p.orientation
                with self.cv:
                    self.S['gt'] = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)),
                                    m.header.stamp.sec + m.header.stamp.nsec * 1e-9)
                    self.cv.notify_all()
                return

    def _wait_scan(self, t_min, timeout=5.0):
        with self.cv:
            if not self.cv.wait_for(lambda: self.S['scan_t'] >= t_min - 1e-6, timeout): return None
            return self.S['scan']

    def _gt_at(self, t, timeout=1.0):
        with self.cv:
            self.cv.wait_for(lambda: self.S['gt'] is not None and self.S['gt'][3] >= t - 1e-6, timeout); return self.S['gt']

    # ---- observation pieces
    @staticmethod
    def _ranges(scan):
        r = np.asarray(scan.ranges, np.float32)
        return np.nan_to_num(r, nan=R_MAX, posinf=R_MAX, neginf=R_MAX), scan.angle_min, scan.angle_step

    def _collided(self, r, a0, da):
        ang = a0 + da * np.arange(len(r)); near = r < 1.0
        x, y = r[near] * np.cos(ang[near]), r[near] * np.sin(ang[near])
        inside = (np.abs(x) <= HALF_L + MARGIN) & (np.abs(y) <= HALF_W + MARGIN)
        if inside.any():
            k = np.argmin(r[near][inside]); self.hit = (float(x[inside][k]), float(y[inside][k]))  # robot frame
            return True
        sz = self.cfg['stop_zone']
        if sz is not None:
            zone = (x >= sz[0]) & (x <= sz[1]) & (np.abs(y) <= sz[2])
            if zone.sum() >= sz[3]:
                k = np.argmin(r[near][zone]); self.hit = (float(x[zone][k]), float(y[zone][k])); self.stop_hit = True
                return True
        return False

    def _obs(self, r, gt, vw, prev_a):
        pooled = np.clip(r, 0.12, R_MAX).reshape(-1, POOL).min(axis=1) / R_MAX
        self.frames.append(pooled.astype(np.float32))
        while len(self.frames) < STACK: self.frames.append(self.frames[-1])
        sg = self.grid.subgoal(self.Dc, gt[0], gt[1], self.cfg['subgoal_ahead']) or (self.goal[0], self.goal[1])
        dx, dy = sg[0] - gt[0], sg[1] - gt[1]; d = math.hypot(dx, dy); b = math.atan2(dy, dx) - gt[2]
        return np.concatenate([np.concatenate(self.frames), [min(d / R_MAX, 1.0), math.sin(b), math.cos(b)],
                               [vw[0] / V_MAX, vw[1] / W_MAX], prev_a]).astype(np.float32)

    # ---- gym API
    def reset(self):
        self.pub.publish(Twist())
        if self.cfg['tour'] and self.leg > 0:
            return self._next_leg()
        for _ in range(200):
            if self.cfg['tour']: break
            sr, sc = self.grid.cells[self.rng.randrange(len(self.grid.cells))]
            if self.cfg['near_wall_goal_p'] > 0 and len(self.grid.near_cells) and self.rng.random() < self.cfg['near_wall_goal_p']:
                gr, gc = self.grid.near_cells[self.rng.randrange(len(self.grid.near_cells))]
            else:
                gr, gc = self.grid.cells[self.rng.randrange(len(self.grid.cells))]
            start, goal = self.grid.xy(sr, sc), self.grid.xy(gr, gc)
            D = self.grid.field(*goal); d0 = self.grid.geo(D, *start)
            if self.cfg['goal_min'] <= d0 <= self.cfg['goal_max']: break
        yaw = self.rng.uniform(-math.pi, math.pi)
        if self.cfg['tour']:
            start, yaw = tuple(self.cfg['tour_start'][:2]), self.cfg['tour_start'][2]
            goal = tuple(self.cfg['tour'][0]); self.leg = 1
            D = self.grid.field(*goal); d0 = self.grid.geo(D, *start)
        if os.environ.get('GZNAV_MARKER') == '1': self._rq('marker', goal)
        self.goal, self.D, self.d_start = goal, D, d0; self.Dc = self.grid.field(*goal, weighted=True)
        self._rq('set_pose', (start[0], start[1], yaw))
        t_before = self.S['scan_t']; self._advance(2)
        scan = self._wait_scan(t_before + 2 * CONTROL_DT); gt = self._gt_at(self.S['scan_t'])
        r, a0, da = self._ranges(scan)
        self.frames.clear(); self.prev_a = np.zeros(2, np.float32); self.prev_gt = gt
        self.d_prev = self.grid.geo(D, gt[0], gt[1]); self.hist = deque([self.d_prev], maxlen=self.cfg['stuck_steps'] + 1)
        self.trace = []; self.hit = None; self.travel = 0.0; self.cmd_v = 0.0; self.n_coll = 0; self.stop_hit = False
        self.t = 0; self.ep_ret = 0.0; self.reset_err = math.hypot(gt[0] - start[0], gt[1] - start[1])
        return self._obs(r, gt, (0.0, 0.0), self.prev_a)

    def _next_leg(self):
        """Tour mode: next goal from the current pose (no teleport). Keeps the scan history and previous action."""
        goal = tuple(self.cfg['tour'][self.leg % len(self.cfg['tour'])]); self.leg += 1
        if os.environ.get('GZNAV_MARKER') == '1': self._rq('marker', goal)
        t_before = self.S['scan_t']; self._advance(1)
        scan = self._wait_scan(t_before + CONTROL_DT); gt = self._gt_at(self.S['scan_t'])
        r, a0, da = self._ranges(scan)
        self.goal = goal; self.D = self.grid.field(*goal); self.Dc = self.grid.field(*goal, weighted=True)
        self.d_start = self.grid.geo(self.D, gt[0], gt[1]); self.prev_gt = gt
        self.d_prev = self.d_start; self.hist = deque([self.d_prev], maxlen=self.cfg['stuck_steps'] + 1)
        self.trace = []; self.hit = None; self.travel = 0.0; self.cmd_v = 0.0; self.n_coll = 0; self.stop_hit = False
        self.t = 0; self.ep_ret = 0.0; self.reset_err = 0.0; self.leg_start = gt[:2]
        return self._obs(r, gt, (0.0, 0.0), self.prev_a)

    def step(self, action):
        a = np.clip(np.asarray(action, np.float32), -1, 1)
        vmin = self.cfg['v_min']; v, w = vmin + (a[0] + 1) / 2 * (V_MAX - vmin), a[1] * W_MAX
        cmd = Twist(); cmd.linear.x, cmd.angular.z = float(v), float(w); self.pub.publish(cmd)
        last = self.S['scan_t']; self._advance(1)
        scan = self._wait_scan(last + CONTROL_DT)
        if scan is None: raise RuntimeError('scan timeout')
        gt = self._gt_at(self.S['scan_t'])
        r, a0, da = self._ranges(scan); self.t += 1
        dyaw = (gt[2] - self.prev_gt[2] + math.pi) % (2 * math.pi) - math.pi
        ddx, ddy = gt[0] - self.prev_gt[0], gt[1] - self.prev_gt[1]
        vf = math.hypot(ddx, ddy) / CONTROL_DT
        if self.cfg['v_min'] < 0 and (ddx * math.cos(gt[2]) + ddy * math.sin(gt[2])) < 0: vf = -vf  # signed only with reverse
        vw = (vf, dyaw / CONTROL_DT); self.prev_gt = gt
        d = self.grid.geo(self.D, gt[0], gt[1]); c = self.cfg
        progress = (self.d_prev - d) if math.isfinite(d) and math.isfinite(self.d_prev) else 0.0
        self.d_prev = d if math.isfinite(d) else self.d_prev; self.hist.append(self.d_prev)
        dist_goal = math.hypot(gt[0] - self.goal[0], gt[1] - self.goal[1])
        collided = self._collided(r, a0, da); reached = dist_goal <= c['goal_tol']
        rmin = float(r.min())
        ang = a0 + da * np.arange(len(r)); px, py = r * np.cos(ang), r * np.sin(ang)
        clear = float(np.hypot(np.maximum(np.abs(px) - HALF_L, 0), np.maximum(np.abs(py) - HALF_W, 0)).min())  # body edge
        self.travel += abs(vw[0]) * CONTROL_DT; self.cmd_v += float(v)
        rew = c['r_progress'] * progress + c['r_step'] + c['r_smooth'] * float(np.abs(a - self.prev_a).sum())
        prox = clear if c['prox_from'] == 'footprint' else rmin
        if prox < c['prox_dist']: rew += c['r_prox'] * (c['prox_dist'] - prox) / c['prox_dist']
        if reached: rew += c['r_goal']
        if collided: rew += c['r_collision']
        stuck = len(self.hist) > c['stuck_steps'] and (self.hist[0] - self.hist[-1]) < c['stuck_progress']
        self.n_coll += int(collided)
        terminated = reached or (collided and c['collision_terminates'])
        truncated = (not terminated) and (self.t >= c['max_steps'] or stuck)
        self.ep_ret += rew
        obs = self._obs(r, gt, vw, a); self.prev_a = a
        if self.trace_on: self.trace.append((gt[0], gt[1], gt[2], float(v), float(w), vw[0], vw[1], rmin))
        info = {}
        if terminated or truncated:
            info = dict(is_success=bool(reached), collision=bool(self.n_coll > 0), collision_steps=self.n_coll, stop_zone=bool(self.stop_hit), leg=self.leg, stuck=bool(stuck and not terminated),
                        timeout=bool(self.t >= c['max_steps'] and not terminated), ep_len=self.t, ep_return=self.ep_ret,
                        d_start=self.d_start, d_end_geo=d, dist_goal=dist_goal, reset_err=self.reset_err,
                        # frozen sim suspected: stuck while moving < 25% of the commanded distance (mean cmd v > 0.1 m/s)
                        frozen=bool(stuck and not terminated and self.cmd_v / self.t > 0.1
                                    and self.travel < 0.25 * self.cmd_v * CONTROL_DT))
            if self.trace_on: info.update(trace=self.trace, goal=self.goal, hit=self.hit)
        return obs, float(rew), bool(terminated), bool(truncated), info

    def close(self):
        try: self.pub.publish(Twist()); self._pc.send(None)
        except Exception: pass
        try: os.killpg(self.srv.pid, signal.SIGINT); self.srv.wait(10)
        except Exception:
            try: os.killpg(self.srv.pid, signal.SIGKILL)
            except Exception: pass
