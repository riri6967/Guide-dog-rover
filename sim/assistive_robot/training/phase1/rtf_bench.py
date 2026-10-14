#!/usr/bin/env python3
"""Headless Gazebo speed + stability bench (no ROS). Starts one headless gz server, drives a constant arc,
reports real-time factor, LiDAR rate and how the motion compares across physics step sizes.

usage: /usr/bin/python3 training/phase1/rtf_bench.py WORLD.sdf [--partition P] [--sim-seconds 30] [--wall-timeout 120]
Needs GZ_SIM_RESOURCE_PATH with training/models and models/. Uses GZ_IP=127.0.0.1.
"""
import argparse, math, os, signal, subprocess, sys, threading, time
from pathlib import Path
from gz.transport13 import Node
from gz.msgs10.twist_pb2 import Twist
from gz.msgs10.world_stats_pb2 import WorldStatistics
from gz.msgs10.laserscan_pb2 import LaserScan
from gz.msgs10.pose_v_pb2 import Pose_V
from gz.msgs10.contacts_pb2 import Contacts
CONTACT_TOPIC = '/world/assistive_world/model/assistive_rover/link/base_link/sensor/rover_contact/contact'

PKG = Path(__file__).resolve().parents[2]
V_CMD, W_CMD = 0.12, 0.5  # small constant arc (real radius ~0.2 m) so the 0.59 m corner reach stays clear of walls (y +-1.4)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world'); ap.add_argument('--partition', default=f'bench{os.getpid()}')
    ap.add_argument('--sim-seconds', type=float, default=30.0); ap.add_argument('--wall-timeout', type=float, default=120.0)
    a = ap.parse_args()
    env = dict(os.environ, GZ_PARTITION=a.partition, GZ_IP='127.0.0.1',
               GZ_SIM_RESOURCE_PATH=f"{PKG/'training'/'models'}:{PKG/'models'}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}")
    os.environ.update(GZ_PARTITION=a.partition, GZ_IP='127.0.0.1')
    log = open(f"/tmp/claude-1000/rtf_bench_{a.partition}.log", 'w') if os.path.isdir('/tmp/claude-1000') else subprocess.DEVNULL
    srv = subprocess.Popen(['gz', 'sim', '-s', '-r', '--headless-rendering', a.world], env=env, stdout=log, stderr=log,
                           start_new_session=True)
    lock = threading.Lock(); st = {'stats': [], 'scans': [], 'poses': [], 'wall_contacts': 0}

    def on_stats(m):
        with lock: st['stats'].append((time.time(), m.sim_time.sec + m.sim_time.nsec * 1e-9, m.real_time_factor))

    def on_scan(m):
        with lock: st['scans'].append((time.time(), m.header.stamp.sec + m.header.stamp.nsec * 1e-9))

    def on_pose(m):
        t = m.header.stamp.sec + m.header.stamp.nsec * 1e-9
        for p in m.pose:
            if p.name == 'assistive_rover':
                q = p.orientation
                yaw = math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))
                roll = math.atan2(2 * (q.w * q.x + q.y * q.z), 1 - 2 * (q.x * q.x + q.y * q.y))
                pitch = math.asin(max(-1, min(1, 2 * (q.w * q.y - q.z * q.x))))
                with lock: st['poses'].append((t, p.position.x, p.position.y, p.position.z, yaw, roll, pitch))
                break

    def on_contact(m):
        for c in m.contact:
            names = (c.collision1.name + ' ' + c.collision2.name).lower()
            if 'floor' not in names and 'ground' not in names:
                with lock: st['wall_contacts'] += 1

    n = Node()
    n.subscribe(Contacts, CONTACT_TOPIC, on_contact)
    n.subscribe(WorldStatistics, '/stats', on_stats)
    n.subscribe(LaserScan, '/scan', on_scan)
    n.subscribe(Pose_V, '/world/assistive_world/dynamic_pose/info', on_pose)
    pub = n.advertise('/cmd_vel', Twist)
    cmd = Twist(); cmd.linear.x, cmd.angular.z = V_CMD, W_CMD
    t0 = time.time()
    try:
        while time.time() - t0 < a.wall_timeout:
            pub.publish(cmd); time.sleep(0.02)
            with lock:
                if st['stats'] and st['stats'][-1][1] >= a.sim_seconds: break
    finally:
        os.killpg(srv.pid, signal.SIGINT)
        try: srv.wait(10)
        except subprocess.TimeoutExpired: os.killpg(srv.pid, signal.SIGKILL)
    S, L, P = st['stats'], st['scans'], st['poses']
    if len(S) < 5 or not P:
        print(f'{Path(a.world).name}: NO DATA (stats {len(S)}, scans {len(L)}, poses {len(P)})'); return
    # measure over the part where the server was clearly running (skip first 2 sim-seconds of startup)
    S2 = [s for s in S if s[1] >= 2.0] or S
    wall = S2[-1][0] - S2[0][0]; sim = S2[-1][1] - S2[0][1]; rtf = sim / wall if wall > 0 else float('nan')
    L2 = [l for l in L if l[1] >= 2.0]
    scan_wall_hz = (len(L2) - 1) / (L2[-1][0] - L2[0][0]) if len(L2) > 2 else 0
    scan_sim_hz = (len(L2) - 1) / (L2[-1][1] - L2[0][1]) if len(L2) > 2 else 0
    def rates(seg):
        if len(seg) < 10: return float('nan'), float('nan')
        dyaw = sum((seg[i][4] - seg[i - 1][4] + math.pi) % (2 * math.pi) - math.pi for i in range(1, len(seg)))
        dist = sum(math.hypot(seg[i][1] - seg[i - 1][1], seg[i][2] - seg[i - 1][2]) for i in range(1, len(seg)))
        dt = seg[-1][0] - seg[0][0]; return dist / dt, dyaw / dt
    P2 = [p for p in P if p[0] >= 5.0]; mid = len(P2) // 2
    speed, yaw_rate = rates(P2); s1, w1 = rates(P2[:mid]); s2, w2 = rates(P2[mid:])
    xs = [p[1] for p in P2]; ys = [p[2] for p in P2]
    zs = [p[3] for p in P]; tilt = max(max(abs(p[5]), abs(p[6])) for p in P)
    last = P[-1]
    print(f"{Path(a.world).name:22s} RTF {rtf:6.2f} | lidar {scan_wall_hz:6.1f} Hz wall {scan_sim_hz:4.1f} sim | "
          f"v {speed:.3f} (1st {s1:.3f} 2nd {s2:.3f}) w {yaw_rate:.3f} (1st {w1:.3f} 2nd {w2:.3f}) | "
          f"x {min(xs):.2f}..{max(xs):.2f} y {min(ys):.2f}..{max(ys):.2f} | tilt {math.degrees(tilt):.1f} deg | wall contacts {st['wall_contacts']}")

if __name__ == '__main__':
    main()
