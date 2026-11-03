#!/usr/bin/env python3
"""ROS 2 env-loop throughput bench: one isolated Gazebo instance (own GZ_PARTITION + ROS_DOMAIN_ID), bridge,
and a gym-style loop: reset = teleport to a random free pose + zero cmd + wait for a fresh scan;
step = publish (v, w), wait for the next scan one control period later, read ground-truth pose.
Random actions. Reports env steps/s (wall), reset cost, and action latency in SIM time.

usage: /usr/bin/python3 training/phase1/env_loop_bench.py WORLD.sdf --instance I [--steps 3000] [--episode 200]
"""
import argparse, math, os, random, re, signal, subprocess, sys, time
from pathlib import Path
import numpy as np

PKG = Path(__file__).resolve().parents[2]
CONTROL_DT = 0.1  # 10 Hz = LiDAR rate


def free_poses(clearance=0.7):
    d = open(PKG / 'nav2' / 'maps' / 'assistive_world.pgm', 'rb').read()
    m = re.match(rb'P5\s+(\d+)\s+(\d+)\s+(\d+)\s', d); w, h = int(m[1]), int(m[2])
    img = np.flipud(np.frombuffer(d[m.end():], np.uint8).reshape(h, w))  # row 0 = y min
    occ = img < 128; k = int(clearance / 0.05); cur = occ.copy()
    for _ in range(k):
        p = np.pad(cur, 1); cur = cur | p[:-2, 1:-1] | p[2:, 1:-1] | p[1:-1, :-2] | p[1:-1, 2:]
    ys, xs = np.nonzero(~cur)
    return [(-13 + x * 0.05, -8 + y * 0.05) for x, y in zip(xs[::7], ys[::7])]


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('world'); ap.add_argument('--instance', type=int, default=0)
    ap.add_argument('--steps', type=int, default=3000); ap.add_argument('--episode', type=int, default=200)
    ap.add_argument('--lockstep', action='store_true', help='start paused; advance exactly one control period per step')
    ap.add_argument('--phys-step', type=float, default=0.001, help='world max_step_size (for lockstep iteration count)')
    a = ap.parse_args()
    part, dom = f'env{a.instance}_{os.getpid()}', 20 + a.instance
    env = dict(os.environ, GZ_PARTITION=part, GZ_IP='127.0.0.1', ROS_DOMAIN_ID=str(dom),
               ROS_AUTOMATIC_DISCOVERY_RANGE='LOCALHOST',
               GZ_SIM_RESOURCE_PATH=f"{PKG/'training'/'models'}:{PKG/'models'}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}")
    os.environ.update({k: env[k] for k in ('GZ_PARTITION', 'GZ_IP', 'ROS_DOMAIN_ID', 'ROS_AUTOMATIC_DISCOVERY_RANGE')})
    logdir = Path('/tmp/claude-1000') if Path('/tmp/claude-1000').is_dir() else Path('/tmp')
    procs = [subprocess.Popen(['gz', 'sim', '-s'] + ([] if a.lockstep else ['-r']) + ['--headless-rendering', a.world], env=env, start_new_session=True,
                              stdout=open(logdir / f'envbench_gz_{a.instance}.log', 'w'), stderr=subprocess.STDOUT)]
    procs.append(subprocess.Popen(['ros2', 'run', 'ros_gz_bridge', 'parameter_bridge',
        '/clock@rosgraph_msgs/msg/Clock[gz.msgs.Clock',
        '/scan@sensor_msgs/msg/LaserScan[gz.msgs.LaserScan',
        '/cmd_vel@geometry_msgs/msg/Twist]gz.msgs.Twist',
        '/world/assistive_world/dynamic_pose/info@tf2_msgs/msg/TFMessage[gz.msgs.Pose_V',
        '/world/assistive_world/set_pose@ros_gz_interfaces/srv/SetEntityPose',
        '/world/assistive_world/control@ros_gz_interfaces/srv/ControlWorld',
        '--ros-args', '-r', '/world/assistive_world/dynamic_pose/info:=/gt_poses'],
        env=env, start_new_session=True, stdout=open(logdir / f'envbench_bridge_{a.instance}.log', 'w'), stderr=subprocess.STDOUT))
    try:
        run(a)
    finally:
        for p in procs:
            try: os.killpg(p.pid, signal.SIGINT)
            except ProcessLookupError: pass
        for p in procs:
            try: p.wait(10)
            except subprocess.TimeoutExpired: os.killpg(p.pid, signal.SIGKILL)


def run(a):
    import rclpy
    from rclpy.node import Node
    from rclpy.qos import qos_profile_sensor_data
    from sensor_msgs.msg import LaserScan
    from geometry_msgs.msg import Twist
    from rosgraph_msgs.msg import Clock
    from tf2_msgs.msg import TFMessage
    from ros_gz_interfaces.srv import SetEntityPose, ControlWorld
    rclpy.init(); n = Node('env_bench')
    S = {'scan': None, 'clock': 0.0, 'gt': None}
    stamp = lambda h: h.stamp.sec + h.stamp.nanosec * 1e-9
    n.create_subscription(LaserScan, '/scan', lambda m: S.__setitem__('scan', m), qos_profile_sensor_data)
    n.create_subscription(Clock, '/clock', lambda m: S.__setitem__('clock', m.clock.sec + m.clock.nanosec * 1e-9), 10)

    def on_gt(m):
        # the Pose_V -> TFMessage bridge drops entity names; gz lists the model first (checked by the teleport error)
        if not m.transforms: return
        t = m.transforms[0]; q = t.transform.rotation
        S['gt'] = (t.transform.translation.x, t.transform.translation.y,
                   math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z)), stamp(t.header))
    n.create_subscription(TFMessage, '/gt_poses', on_gt, 10)
    pub = n.create_publisher(Twist, '/cmd_vel', 10)
    cli = n.create_client(SetEntityPose, '/world/assistive_world/set_pose')
    ctl = n.create_client(ControlWorld, '/world/assistive_world/control')
    iters = max(1, round(CONTROL_DT / a.phys_step))

    def advance(k=1):
        # lockstep: run exactly k control periods while staying paused
        req = ControlWorld.Request(); req.world_control.pause = True; req.world_control.multi_step = iters * k
        f = ctl.call_async(req)
        while not f.done(): spin()
    spin = lambda: rclpy.spin_once(n, timeout_sec=0.001)

    t_wait = time.time()
    if a.lockstep:
        while not ctl.service_is_ready() and time.time() - t_wait < 60: spin()
        advance(3)
    while (S['scan'] is None or S['gt'] is None or not cli.service_is_ready()) and time.time() - t_wait < 60: spin()
    if S['scan'] is None or S['gt'] is None:
        print(f'instance {a.instance}: NO DATA after 60 s'); return
    poses = free_poses(); rng = random.Random(a.instance)

    def wait_scan_after(t_sim, timeout=5.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            spin()
            if S['scan'] is not None and stamp(S['scan'].header) >= t_sim - 1e-6: return S['scan']
        return None

    def reset():
        pub.publish(Twist())
        x, y = rng.choice(poses); yaw = rng.uniform(-math.pi, math.pi)
        req = SetEntityPose.Request(); req.entity.name = 'assistive_rover'; req.entity.type = 2
        req.pose.position.x, req.pose.position.y, req.pose.position.z = x, y, 0.02
        req.pose.orientation.z, req.pose.orientation.w = math.sin(yaw / 2), math.cos(yaw / 2)
        f = cli.call_async(req)
        while not f.done(): spin()
        t_req = S['clock']
        if a.lockstep:
            advance(2); sc = wait_scan_after(t_req + 2 * CONTROL_DT - 1e-3)
            t_g = time.time()
            while S['gt'] is None or S['gt'][3] < stamp(sc.header) - 1e-6:
                spin()
                if time.time() - t_g > 2: break
        else:
            sc = wait_scan_after(t_req + CONTROL_DT)  # first scan fully taken after the teleport
        gt = S['gt']; err = math.hypot(gt[0] - x, gt[1] - y) if gt else float('nan')
        return sc, err

    steps = resets = timeouts = 0; reset_wall = []; lat = []; per_step_sim = []; tele_err = []; collisions = 0
    t_start = time.time(); sim_start = S['clock']
    sc, err = reset(); resets += 1; tele_err.append(err); ep = 0
    while steps < a.steps:
        last = stamp(sc.header)
        cmd = Twist(); cmd.linear.x = rng.uniform(0.0, 0.3); cmd.angular.z = rng.uniform(-1.0, 1.0)
        pub.publish(cmd)
        if a.lockstep:
            lat.append(S['clock'] - last); advance(1)
        else:
            lat.append(S['clock'] - last)  # sim time elapsed between scan stamp and action
        nsc = wait_scan_after(last + CONTROL_DT - 1e-3)
        if nsc is None: timeouts += 1; sc, err = reset(); resets += 1; continue
        per_step_sim.append(stamp(nsc.header) - last); sc = nsc; steps += 1; ep += 1
        r = np.asarray(sc.ranges, dtype=np.float32); r = r[np.isfinite(r)]
        obs = np.minimum.reduceat(np.clip(np.nan_to_num(np.asarray(sc.ranges, np.float32), posinf=6.0), 0.12, 6.0), np.arange(0, len(sc.ranges), 4))
        crashed = r.size and r.min() < 0.50
        if crashed: collisions += 1
        if crashed or ep >= a.episode:
            t0 = time.time(); sc, err = reset(); reset_wall.append(time.time() - t0); resets += 1; tele_err.append(err); ep = 0
    wall = time.time() - t_start; sim = S['clock'] - sim_start
    lat = np.array(lat); pss = np.array(per_step_sim)
    print(f"instance {a.instance}: {steps} steps in {wall:.1f} s wall = {steps / wall:.1f} steps/s = {3600 * steps / wall:,.0f} steps/h | "
          f"sim {sim:.0f} s (RTF {sim / wall:.1f}) | resets {resets} (mean {1000 * np.mean(reset_wall) if reset_wall else 0:.0f} ms wall, "
          f"teleport err max {np.nanmax(tele_err):.3f} m) | episodes ended by proximity {collisions} | scan timeouts {timeouts} | "
          f"sim per step mean {pss.mean():.3f} s (max {pss.max():.3f}) | action latency in sim: median {1000 * np.median(lat):.0f} ms, p95 {1000 * np.percentile(lat, 95):.0f} ms")
    pub.publish(Twist()); n.destroy_node(); rclpy.shutdown()


if __name__ == '__main__':
    main()
