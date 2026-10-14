#!/usr/bin/env python3
"""Nav2 evaluator v2: every trial is judged by Gazebo ground-truth pose, not Nav2's own estimate.
usage: /usr/bin/python3 scripts/auto_nav2_eval_v2.py [--repeats N] [--timeout S] [--tol M] [--goals CSV]
"""
import argparse, csv, math, re, subprocess, threading, time
from datetime import datetime
from pathlib import Path

import rclpy
from action_msgs.msg import GoalStatus
from nav2_msgs.action import NavigateToPose
from rclpy.action import ActionClient
from rclpy.node import Node
from rclpy.qos import qos_profile_sensor_data
from sensor_msgs.msg import LaserScan
try:
    from ros_gz_interfaces.msg import Contacts
except ImportError:
    Contacts = None

GZ = "gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A14 'name: \"assistive_rover\"'"
STATUS = {GoalStatus.STATUS_SUCCEEDED: 'SUCCEEDED', GoalStatus.STATUS_ABORTED: 'ABORTED',
          GoalStatus.STATUS_CANCELED: 'CANCELED'}
COLS = ['repeat', 'goal', 'goal_x', 'goal_y', 'status', 'success_real', 'duration_s', 'recoveries',
        'start_x', 'start_y', 'start_yaw', 'end_x', 'end_y', 'end_yaw',
        'real_dist_to_goal_m', 'yaw_error_deg', 'path_len_m', 'straight_m', 'efficiency',
        'min_lidar_m', 'contact_pairs']


def gt_pose():
    try:
        out = subprocess.run(GZ, shell=True, capture_output=True, text=True, timeout=5).stdout
        pos = out.split('position')[1].split('orientation')[0]
        ori = out.split('orientation')[1]
        x = float(re.search(r'x: (\S+)', pos).group(1))
        y = float(re.search(r'y: (\S+)', pos).group(1))
        z = float(re.search(r'z: (\S+)', ori).group(1))
        w = float(re.search(r'w: (\S+)', ori).group(1))
        return x, y, math.degrees(2 * math.atan2(z, w))
    except Exception:
        return None


class Sampler(threading.Thread):
    def __init__(self):
        super().__init__(daemon=True)
        self.samples, self.running = [], False

    def run(self):
        while True:
            if self.running:
                p = gt_pose()
                if p:
                    self.samples.append(p)
            else:
                time.sleep(0.05)


class Eval(Node):
    def __init__(self):
        super().__init__('nav2_eval_v2')
        self.client = ActionClient(self, NavigateToPose, 'navigate_to_pose')
        self.min_range, self.contacts, self.recoveries = float('inf'), set(), 0
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data)
        if Contacts:
            self.create_subscription(Contacts, '/rover_contacts', self.on_contacts, 10)

    def on_scan(self, m):
        v = [r for r in m.ranges if math.isfinite(r) and r > m.range_min]
        if v:
            self.min_range = min(self.min_range, min(v))

    def on_contacts(self, m):
        for c in m.contacts:
            a, b = c.collision1.name, c.collision2.name
            if 'ground' not in a.lower() and 'ground' not in b.lower():
                self.contacts.add(f"{a}|{b}")

    def on_feedback(self, fb):
        self.recoveries = fb.feedback.number_of_recoveries

    def spin_for(self, secs):
        t = time.time()
        while time.time() - t < secs:
            rclpy.spin_once(self, timeout_sec=0.1)

    def run_trial(self, sampler, g, timeout, tol):
        gx, gy, gyaw = float(g['x']), float(g['y']), float(g.get('yaw_deg') or 0)
        start = gt_pose()
        self.min_range, self.contacts, self.recoveries = float('inf'), set(), 0
        goal = NavigateToPose.Goal()
        goal.pose.header.frame_id = 'map'
        goal.pose.pose.position.x, goal.pose.pose.position.y = gx, gy
        goal.pose.pose.orientation.z = math.sin(math.radians(gyaw) / 2)
        goal.pose.pose.orientation.w = math.cos(math.radians(gyaw) / 2)
        sampler.samples = []
        sampler.running = True
        t0 = time.time()
        fut = self.client.send_goal_async(goal, feedback_callback=self.on_feedback)
        rclpy.spin_until_future_complete(self, fut, timeout_sec=10)
        handle = fut.result()
        if handle is None or not handle.accepted:
            status = 'REJECTED'
        else:
            rf = handle.get_result_async()
            while not rf.done() and time.time() - t0 < timeout:
                rclpy.spin_once(self, timeout_sec=0.1)
            if rf.done():
                status = STATUS.get(rf.result().status, str(rf.result().status))
            else:
                status = 'TIMEOUT'
                rclpy.spin_until_future_complete(self, handle.cancel_goal_async(), timeout_sec=5)
        duration = time.time() - t0
        sampler.running = False
        self.spin_for(3)                      # let the rover settle
        end = gt_pose()
        pts = ([start] if start else []) + list(sampler.samples) + ([end] if end else [])
        path = sum(math.hypot(b[0] - a[0], b[1] - a[1]) for a, b in zip(pts, pts[1:]))
        r = dict(goal=g['name'], goal_x=gx, goal_y=gy, status=status, duration_s=round(duration, 1),
                 recoveries=self.recoveries, success_real=False,
                 min_lidar_m=round(self.min_range, 2) if self.min_range < 1e9 else '',
                 contact_pairs=';'.join(sorted(self.contacts)), path_len_m=round(path, 2))
        if start:
            r.update(start_x=round(start[0], 2), start_y=round(start[1], 2), start_yaw=round(start[2], 0),
                     straight_m=round(math.hypot(gx - start[0], gy - start[1]), 2))
            if path > 0.05:
                r['efficiency'] = round(r['straight_m'] / path, 2)
        if end:
            d = math.hypot(gx - end[0], gy - end[1])
            r.update(end_x=round(end[0], 2), end_y=round(end[1], 2), end_yaw=round(end[2], 0),
                     real_dist_to_goal_m=round(d, 2),
                     yaw_error_deg=round((end[2] - gyaw + 180) % 360 - 180, 0),
                     success_real=(status == 'SUCCEEDED' and d <= tol))
        return r


def main():
    base = Path(__file__).resolve().parents[1]
    ap = argparse.ArgumentParser()
    ap.add_argument('--goals', default=str(base / 'nav2' / 'goals.csv'))
    ap.add_argument('--repeats', type=int, default=1)
    ap.add_argument('--timeout', type=float, default=150)
    ap.add_argument('--tol', type=float, default=0.25)
    a = ap.parse_args()
    goals = [{k.strip(): (v or '').strip() for k, v in row.items()} for row in csv.DictReader(open(a.goals))]

    rclpy.init()
    node = Eval()
    if not node.client.wait_for_server(timeout_sec=15):
        print('navigate_to_pose action server not available'); return
    sampler = Sampler(); sampler.start()
    out = base / 'results'; out.mkdir(exist_ok=True)
    path = out / f"nav2_eval_v2_{datetime.now():%Y%m%d_%H%M%S}.csv"
    rows = []
    with open(path, 'w', newline='') as f:
        w = csv.DictWriter(f, fieldnames=COLS, restval='')
        w.writeheader()
        for rep in range(1, a.repeats + 1):
            for g in goals:
                r = node.run_trial(sampler, g, a.timeout, a.tol)
                r['repeat'] = rep
                w.writerow(r); f.flush(); rows.append(r)
                print(f"[{rep}] {r['goal']:12s} {r['status']:10s} real_dist={r.get('real_dist_to_goal_m', '?')} m "
                      f"t={r['duration_s']}s rec={r['recoveries']} success_real={r['success_real']}", flush=True)
    n = len(rows); ok = [r for r in rows if r['success_real']]
    print('\n=== SUMMARY (success = Nav2 SUCCEEDED and real distance <= %.2f m) ===' % a.tol)
    print(f"trials: {n}   real successes: {len(ok)} ({100 * len(ok) / max(n, 1):.0f}%)")
    if ok:
        print(f"mean time (successes): {sum(r['duration_s'] for r in ok) / len(ok):.1f} s")
        effs = [r['efficiency'] for r in ok if r.get('efficiency') != '' and r.get('efficiency') is not None]
        if effs:
            print(f"mean path efficiency (successes): {sum(effs) / len(effs):.2f}")
    print(f"trials with non-ground contacts (unvalidated): {sum(1 for r in rows if r['contact_pairs'])}")
    print(f"csv: {path}")
    rclpy.shutdown()


if __name__ == '__main__':
    main()
