#!/usr/bin/env python3
"""Lockstep env-loop bench talking to Gazebo directly (gz-transport, no ROS, no bridge).
Gazebo starts paused; each step = publish (v, w), request exactly one control period of physics, wait for that scan.
reset = set_pose to a random free pose + zero cmd + step until a scan after the teleport.

usage: /usr/bin/python3 training/phase1/gz_lockstep_bench.py WORLD.sdf --phys-step 0.004 [--instance I] [--steps 3000] [--episode 200]
"""
import argparse, math, os, random, signal, subprocess, sys, threading, time
from pathlib import Path
import numpy as np
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.pose_pb2 import Pose
from gz.msgs10.boolean_pb2 import Boolean
from gz.msgs10.world_control_pb2 import WorldControl

sys.path.insert(0, str(Path(__file__).resolve().parent))
from env_loop_bench import free_poses, CONTROL_DT, PKG  # noqa: E402


def _requester(conn):
    # Separate process with no subscriptions: gz Node.request() keeps the GIL while waiting, so calling it from a
    # process that also has Python subscription callbacks can deadlock (the callback blocks the transport thread).
    n = Node()
    while True:
        msg = conn.recv()
        if msg is None: return
        kind, payload = msg
        if kind == 'wait':
            t0 = time.time()
            while time.time() - t0 < 60 and not set(payload) <= set(n.service_list()): time.sleep(0.2)
            conn.send(set(payload) <= set(n.service_list())); continue
        if kind == 'control':
            req = WorldControl(); req.pause = True; req.multi_step = payload
            ok, _ = n.request('/world/assistive_world/control', req, WorldControl, Boolean, 2000)
        else:  # set_pose
            x, y, yaw = payload; p = Pose(); p.name = 'assistive_rover'
            p.position.x, p.position.y, p.position.z = x, y, 0.02
            p.orientation.z, p.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
            ok, _ = n.request('/world/assistive_world/set_pose', p, Pose, Boolean, 2000)
        conn.send(ok)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world'); ap.add_argument('--instance', type=int, default=0)
    ap.add_argument('--phys-step', type=float, default=0.004)
    ap.add_argument('--steps', type=int, default=3000); ap.add_argument('--episode', type=int, default=200)
    a = ap.parse_args()
    part = f'gzls{a.instance}_{os.getpid()}'
    env = dict(os.environ, GZ_PARTITION=part, GZ_IP='127.0.0.1',
               GZ_SIM_RESOURCE_PATH=f"{PKG/'training'/'models'}:{PKG/'models'}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}")
    os.environ.update(GZ_PARTITION=part, GZ_IP='127.0.0.1')
    logdir = Path('/tmp/claude-1000') if Path('/tmp/claude-1000').is_dir() else Path('/tmp')
    srv = subprocess.Popen(['gz', 'sim', '-s', '--headless-rendering', a.world], env=env, start_new_session=True,
                           stdout=open(logdir / f'gzls_{a.instance}.log', 'w'), stderr=subprocess.STDOUT)
    try:
        run(a)
    finally:
        os.killpg(srv.pid, signal.SIGINT)
        try: srv.wait(10)
        except subprocess.TimeoutExpired: os.killpg(srv.pid, signal.SIGKILL)


def run(a):
    cv = threading.Condition(); S = {'scan': None, 'scan_t': -1.0, 'gt': None}
    st = lambda h: h.stamp.sec + h.stamp.nsec * 1e-9

    def on_scan(m):
        with cv: S['scan'], S['scan_t'] = m, st(m.header); cv.notify_all()

    def on_pose(m):
        for p in m.pose:
            if p.name == 'assistive_rover':
                q = p.orientation
                with cv:
                    S['gt'] = (p.position.x, p.position.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), st(m.header))
                    cv.notify_all()
                break

    import multiprocessing as mp
    parent, child = mp.get_context('spawn').Pipe()
    helper = mp.get_context('spawn').Process(target=_requester, args=(child,), daemon=True); helper.start()

    def rq(kind, payload):
        parent.send((kind, payload)); return parent.recv()

    n = Node()
    n.subscribe(LaserScan, '/scan', on_scan)
    n.subscribe(Pose_V, '/world/assistive_world/dynamic_pose/info', on_pose)
    pub = n.advertise('/cmd_vel', Twist)
    iters = max(1, round(CONTROL_DT / a.phys_step))
    t_rt = []

    def advance(k=1):
        t0 = time.time(); ok = rq('control', iters * k); t_rt.append(time.time() - t0)
        return ok

    def wait_scan(t_min, timeout=5.0):
        with cv:
            if not cv.wait_for(lambda: S['scan_t'] >= t_min - 1e-6, timeout): return None
            return S['scan']

    t0 = time.time()  # wait until the server advertises its services (a request before that blocks)
    rq('wait', ['/world/assistive_world/control', '/world/assistive_world/set_pose'])
    while time.time() - t0 < 60:  # then produce the first scan
        if advance(3) and wait_scan(0.0, 1.0) is not None: break
        time.sleep(0.5)
    if S['scan'] is None: print(f'instance {a.instance}: NO DATA'); return
    poses = free_poses(); rng = random.Random(a.instance)

    def reset():
        pub.publish(Twist())
        x, y = rng.choice(poses); yaw = rng.uniform(-math.pi, math.pi)
        rq('set_pose', (x, y, yaw))
        t_before = S['scan_t']; advance(2)
        sc = wait_scan(t_before + 2 * CONTROL_DT)
        with cv: cv.wait_for(lambda: S['gt'] is not None and S['gt'][3] >= S['scan_t'] - 1e-6, 1.0); gt = S['gt']
        return sc, math.hypot(gt[0] - x, gt[1] - y)

    steps = 0; resets = 0; reset_wall = []; tele = []; prox = 0; timeouts = 0; pss = []
    t_start = time.time(); sim0 = S['scan_t']
    sc, e = reset(); resets += 1; tele.append(e); ep = 0
    while steps < a.steps:
        last = S['scan_t']
        cmd = Twist(); cmd.linear.x = rng.uniform(0.0, 0.3); cmd.angular.z = rng.uniform(-1.0, 1.0)
        pub.publish(cmd); advance(1)
        nsc = wait_scan(last + CONTROL_DT)
        if nsc is None: timeouts += 1; sc, e = reset(); resets += 1; continue
        pss.append(S['scan_t'] - last); sc = nsc; steps += 1; ep += 1
        r = np.asarray(sc.ranges, np.float32)
        obs = np.minimum.reduceat(np.clip(np.nan_to_num(r, posinf=6.0), 0.12, 6.0), np.arange(0, len(r), 4))
        if obs.min() < 0.50 or ep >= a.episode:
            prox += obs.min() < 0.50
            t1 = time.time(); sc, e = reset(); reset_wall.append(time.time() - t1); resets += 1; tele.append(e); ep = 0
    wall = time.time() - t_start; sim = S['scan_t'] - sim0; pss = np.array(pss)
    print(f"instance {a.instance}: {steps} steps in {wall:.1f} s wall = {steps / wall:.1f} steps/s = {3600 * steps / wall:,.0f} steps/h | "
          f"sim {sim:.0f} s (RTF {sim / wall:.1f}) | resets {resets} (mean {1000 * np.mean(reset_wall) if reset_wall else 0:.0f} ms, "
          f"teleport err max {max(tele):.3f} m) | proximity ends {prox} | scan timeouts {timeouts} | "
          f"sim per step {pss.mean():.3f} s (min {pss.min():.3f} max {pss.max():.3f}) | control rtt median {1000 * np.median(t_rt):.1f} ms")
    pub.publish(Twist()); parent.send(None)


if __name__ == '__main__':
    main()
