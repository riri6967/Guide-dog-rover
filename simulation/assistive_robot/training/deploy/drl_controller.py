#!/usr/bin/env python3
"""DRL local controller for Nav2 (system python 3.12: rclpy + numpy, no torch). Replaces controller_server (MPPI) ONLY:
serves the nav2_msgs/FollowPath action that bt_navigator calls, so planner, BT recoveries, velocity smoother,
Collision Monitor, AMCL and the evaluator stay exactly as in the frozen baseline. Launch Nav2 with
nav2/launch/navigation_drl.launch.py (same as navigation.launch.py minus controller_server).

Observation = training env (gz_nav_env.py), built from ROS data: /scan (4 stacked frames, min-pooled to 90, /6),
subgoal ~1.5 m ahead along the received path in the robot frame (pose = AMCL via TF map->base_link, NOT ground truth),
measured v/w from /odom, previous action. Acts once per new scan (10 Hz, as the 0.1 s training step).
Output: /cmd_vel_nav (-> velocity smoother -> collision monitor -> /cmd_vel).
Goal check like Nav2's general_goal_checker xy tolerance (0.15 m, yaw ignored: the policy never learned a final turn).
Progress check like Nav2's progress_checker (0.20 m per 10 s) -> FAILED_TO_MAKE_PROGRESS, so the BT runs the same recoveries.

usage: /usr/bin/python3 training/deploy/drl_controller.py [--actor training/deploy/actor_run2.npz] [--tol 0.15]
"""
import argparse, math, os, threading, time, traceback
from collections import deque
from pathlib import Path
import numpy as np
import rclpy
from rclpy.action import ActionServer, CancelResponse, GoalResponse
from rclpy.callback_groups import MutuallyExclusiveCallbackGroup, ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.node import Node
from rclpy.parameter import Parameter
from rclpy.qos import qos_profile_sensor_data
from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from nav2_msgs.action import FollowPath
import tf2_ros

HERE = Path(__file__).resolve().parent
V_MAX, W_MAX, R_MAX, POOL, STACK = 0.3, 1.0, 6.0, 4, 4


class Actor:
    def __init__(self, path):
        z = np.load(path); self.W = [z[f'W{i}'] for i in range(3)]; self.b = [z[f'b{i}'] for i in range(3)]

    def __call__(self, obs):
        h = np.maximum(self.W[0] @ obs + self.b[0], 0); h = np.maximum(self.W[1] @ h + self.b[1], 0)
        return np.tanh(self.W[2] @ h + self.b[2])


class DrlController(Node):
    def __init__(self, a):
        super().__init__('drl_controller', parameter_overrides=[Parameter('use_sim_time', Parameter.Type.BOOL, True)])
        self.a = a; self.actor = Actor(a.actor); cg = ReentrantCallbackGroup(); sg = MutuallyExclusiveCallbackGroup()
        self.tf = tf2_ros.Buffer(); self.tfl = tf2_ros.TransformListener(self.tf, self)
        self.scan_ev = threading.Event(); self.scan = None; self.odom = (0.0, 0.0)
        self.create_subscription(LaserScan, '/scan', self.on_scan, qos_profile_sensor_data, callback_group=sg)
        self.create_subscription(Odometry, '/odom', self.on_odom, 10, callback_group=sg)
        self.pub = self.create_publisher(Twist, '/cmd_vel_nav', 10)
        self.active = None; self.path = None; self.path_xy = None; self.idx = 0; self.lock = threading.Lock()
        self.frames = deque(maxlen=STACK); self.prev_a = np.zeros(2, np.float32); self.busy = False
        self.prog_ref, self.prog_t = None, time.time()
        ActionServer(self, FollowPath, 'follow_path', execute_callback=self.execute, goal_callback=lambda g: GoalResponse.ACCEPT,
                     cancel_callback=lambda g: CancelResponse.ACCEPT, callback_group=cg)
        self.get_logger().info(f'DRL controller ready, actor {a.actor}, tol {a.tol} m, lookahead {a.lookahead} m')

    def on_scan(self, m): self.scan = m; self.scan_ev.set()
    def on_odom(self, m):  # signed forward speed only for policies trained with reverse (v_min < 0), as in gz_nav_env
        v = m.twist.twist.linear.x if self.a.v_min < 0 else math.hypot(m.twist.twist.linear.x, m.twist.twist.linear.y)
        self.odom = (v, m.twist.twist.angular.z)

    def pose(self):
        t = self.tf.lookup_transform('map', 'base_link', rclpy.time.Time())
        q = t.transform.rotation
        return t.transform.translation.x, t.transform.translation.y, math.atan2(2 * (q.w * q.z + q.x * q.y), 1 - 2 * (q.y * q.y + q.z * q.z))

    def subgoal(self, x, y, P):
        lo = min(self.idx, len(P) - 1); hi = min(len(P), lo + 200)  # monotone search window: never jump back along the path
        d = np.hypot(P[lo:hi, 0] - x, P[lo:hi, 1] - y); self.idx = lo + int(np.argmin(d))
        acc = 0.0; k = self.idx
        while k + 1 < len(P) and acc < self.a.lookahead:
            acc += float(np.hypot(*(P[k + 1] - P[k]))); k += 1
        return P[k]

    def observation(self, x, y, yaw, P):
        r = np.asarray(self.scan.ranges, np.float32)
        r = np.nan_to_num(r, nan=R_MAX, posinf=R_MAX, neginf=R_MAX)
        pooled = np.clip(r, 0.12, R_MAX).reshape(-1, POOL).min(axis=1) / R_MAX
        self.frames.append(pooled.astype(np.float32))
        while len(self.frames) < STACK: self.frames.append(self.frames[-1])
        sx, sy = self.subgoal(x, y, P); dx, dy = sx - x, sy - y; d = math.hypot(dx, dy); b = math.atan2(dy, dx) - yaw
        v, w = self.odom
        return np.concatenate([np.concatenate(self.frames), [min(d / R_MAX, 1.0), math.sin(b), math.cos(b)],
                               [v / V_MAX, w / W_MAX], self.prev_a]).astype(np.float32)

    def stop(self): self.pub.publish(Twist())

    def execute(self, gh):
        try:
            return self._execute(gh)
        except Exception:
            self.get_logger().error('execute failed:\n' + traceback.format_exc())
            self.stop()
            with self.lock:
                if self.active == bytes(gh.goal_id.uuid): self.active = None
            res = FollowPath.Result(); res.error_code = FollowPath.Result.UNKNOWN; res.error_msg = 'DRL controller exception'
            try: gh.abort()
            except Exception: pass
            return res

    def _execute(self, gh):
        my = bytes(gh.goal_id.uuid)
        with self.lock:
            new_nav = self.active is None
            self.active = my
            self.path_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in gh.request.path.poses], float)
            self.idx = 0
            if new_nav:  # progress state survives path updates (BT replans ~1 Hz), like controller_server's progress checker
                self.frames.clear(); self.prev_a = np.zeros(2, np.float32); self.prog_ref, self.prog_t = None, time.time()
        res = FollowPath.Result()
        if len(self.path_xy) == 0:
            res.error_code = FollowPath.Result.INVALID_PATH; gh.abort(); self.active = None; return res
        tf_fail_t = None; fb = FollowPath.Feedback()
        while rclpy.ok():
            with self.lock:
                if self.active != my:  # superseded by a newer path from the BT (replanning): it takes over the loop
                    gh.abort(); return res
                P = self.path_xy
            goal = P[-1]
            if gh.is_cancel_requested:
                self.stop(); self.active = None; gh.canceled(); return res
            if not self.scan_ev.wait(1.0): continue
            self.scan_ev.clear()
            try:
                x, y, yaw = self.pose(); tf_fail_t = None
            except Exception:
                tf_fail_t = tf_fail_t or time.time()
                if time.time() - tf_fail_t > 2.0:
                    self.stop(); self.active = None; res.error_code = FollowPath.Result.TF_ERROR; gh.abort(); return res
                continue
            dist = math.hypot(goal[0] - x, goal[1] - y)
            if dist <= self.a.tol:
                self.stop(); self.active = None; gh.succeed(); return res
            if self.prog_ref is None or math.hypot(x - self.prog_ref[0], y - self.prog_ref[1]) >= 0.20:
                self.prog_ref, self.prog_t = (x, y), time.time()
            elif time.time() - self.prog_t > 10.0:  # same rule as Nav2's progress_checker (sim time ~ wall time at RTF 1)
                self.get_logger().warn('no progress for 10 s: aborting FollowPath (BT recoveries)')
                self.stop(); self.active = None; res.error_code = FollowPath.Result.FAILED_TO_MAKE_PROGRESS
                res.error_msg = 'DRL: failed to make progress'; gh.abort(); return res
            with self.lock:
                if self.active != my: continue
                obs = self.observation(x, y, yaw, P)
            act = self.actor(obs); self.prev_a = act.astype(np.float32)
            cmd = Twist(); cmd.linear.x = float(self.a.v_min + (act[0] + 1) / 2 * (V_MAX - self.a.v_min)); cmd.angular.z = float(act[1] * W_MAX)
            self.pub.publish(cmd)
            fb.distance_to_goal = float(dist); gh.publish_feedback(fb)
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--actor', default=str(HERE / 'actor_run2.npz')); ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--lookahead', type=float, default=1.5)
    ap.add_argument('--v-min', type=float, default=0.0, help='must match the training CFG v_min (0.0 for runs 1-3)')
    a, _ = ap.parse_known_args()
    import signal, sys
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)  # own handlers: log WHICH signal stopped the node
    def on_sig(signum, frame):
        print(f'{time.strftime("%H:%M:%S")} received signal {signal.Signals(signum).name}, shutting down', flush=True)
        try: node.stop()
        except Exception: pass
        rclpy.try_shutdown()
        os._exit(0)  # the multithreaded executor can hang after shutdown (seen 2026-10-11): exit for sure
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)
    import faulthandler; faulthandler.register(signal.SIGUSR1, all_threads=True)  # kill -USR1 <pid>: dump all thread stacks to the log
    node = DrlController(a); ex = MultiThreadedExecutor(num_threads=8); ex.add_node(node)
    try: ex.spin()
    except KeyboardInterrupt: pass
    except Exception:
        print('executor crashed:\n' + traceback.format_exc(), flush=True)
    finally:
        try: node.stop()
        except Exception: pass
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
