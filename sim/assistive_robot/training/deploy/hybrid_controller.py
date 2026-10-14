#!/usr/bin/env python3
"""Hybrid local controller for Nav2: the DRL policy drives by default; Nav2's MPPI (controller_server, already running under
the DRL launch as action 'follow_path_mppi_unused') takes over only in tight spots and for the final approach.
Serves the same nav2_msgs/FollowPath action ('follow_path') as drl_controller.py, so the BT, planner, recoveries, velocity
smoother, Collision Monitor and evaluator are unchanged.

Switching (checked on every new scan, with hysteresis and a minimum dwell time in MPPI mode):
  DRL -> MPPI  when the planned path in the next --ahead m (2.0) passes a narrow region (static-map clearance < --map-c,
               0.9 m, i.e. openings narrower than ~1.8 m: doors, gaps), so MPPI takes over while the rover is still lined up;
               OR scan clearance from the footprint edge < --c-in (0.25 m, safety); OR distance to the goal < --g-in (1.0 m)
  MPPI -> DRL  when none of these holds (scan clearance > --c-out 0.40 m, goal > --g-in + 0.3 m) for --dwell s in MPPI mode
Scan clearance = min distance from any scan point to the 0.90 x 0.76 m footprint rectangle (LiDAR at the base_link origin).
Map clearance = Euclidean distance transform of /map (occupied or unknown = obstacle), so it works for any world/map.
In MPPI mode the no-progress decision is left to MPPI's own progress checker (same 0.2 m / 10 s rule).
Escape (at most --escapes per navigation, before handing a no-progress failure to the BT): reverse straight back at
0.10 m/s up to 0.30 m if the scan shows > 0.35 m free behind the footprint (stop at < 0.15 m), then turn in place toward
the path while no scan point is inside the turning circle (0.64 m). Uses the real scan, not the costmap (Nav2's BackUp
was refused with "Collision Ahead" in these spots).
The node logs every switch and, per FollowPath navigation, the distance driven in each mode (DRL share for the report).

usage: /usr/bin/python3 training/deploy/hybrid_controller.py --actor training/deploy/actor_run3.npz
"""
import argparse, math, os, signal, threading, time, traceback
import numpy as np
import rclpy
from rclpy.action import ActionClient
from rclpy.callback_groups import ReentrantCallbackGroup
from rclpy.executors import MultiThreadedExecutor
from rclpy.qos import QoSProfile, DurabilityPolicy, ReliabilityPolicy
from nav_msgs.msg import OccupancyGrid
from scipy.ndimage import distance_transform_edt
from action_msgs.msg import GoalStatus
from geometry_msgs.msg import Twist
from nav2_msgs.action import FollowPath
from drl_controller import DrlController, HERE, V_MAX, W_MAX

HALF_L, HALF_W = 0.45, 0.38


def scan_xy(scan):
    r = np.asarray(scan.ranges, np.float32)
    a = scan.angle_min + np.arange(len(r), dtype=np.float32) * scan.angle_increment
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    return r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])


def rear_clearance(scan):
    x, y = scan_xy(scan); m = (x < -HALF_L) & (np.abs(y) < HALF_W + 0.05)
    return float((-x[m] - HALF_L).min()) if m.any() else 10.0


def turn_clearance(scan):  # free distance beyond the turning circle of the footprint corners (radius 0.59 m)
    x, y = scan_xy(scan)
    return float(np.hypot(x, y).min() - math.hypot(HALF_L, HALF_W)) if len(x) else 10.0


def footprint_clearance(scan):
    r = np.asarray(scan.ranges, np.float32)
    a = scan.angle_min + np.arange(len(r), dtype=np.float32) * scan.angle_increment
    ok = np.isfinite(r) & (r >= scan.range_min) & (r <= scan.range_max)
    if not ok.any(): return 10.0
    x, y = r[ok] * np.cos(a[ok]), r[ok] * np.sin(a[ok])
    dx, dy = np.maximum(np.abs(x) - HALF_L, 0), np.maximum(np.abs(y) - HALF_W, 0)
    return float(np.hypot(dx, dy).min())


class HybridController(DrlController):
    def __init__(self, a):
        super().__init__(a)
        self.mppi = ActionClient(self, FollowPath, 'follow_path_mppi_unused', callback_group=ReentrantCallbackGroup())
        self.mode, self.mode_t = 'drl', 0.0
        self.mppi_gh, self.mppi_res, self.mppi_for = None, None, None
        self.split = {'drl': 0.0, 'mppi': 0.0}; self.total = {'drl': 0.0, 'mppi': 0.0}; self.last_xy = None; self.n_switch = 0
        self.escapes = 0; self.n_escape = 0
        self.edt = None; self.mode_lock = threading.Lock()  # one execute thread at a time may switch modes / talk to MPPI
        self.create_subscription(OccupancyGrid, '/map', self.on_map, QoSProfile(depth=1, durability=DurabilityPolicy.TRANSIENT_LOCAL,
                                 reliability=ReliabilityPolicy.RELIABLE))
        self.get_logger().info(f'HYBRID mode: map_c {a.map_c} ahead {a.ahead} c_in {a.c_in} c_out {a.c_out} g_in {a.g_in} dwell {a.dwell} s')

    def on_map(self, m):
        g = np.asarray(m.data, np.int16).reshape(m.info.height, m.info.width)
        self.edt = (distance_transform_edt(~((g >= 65) | (g < 0))) * m.info.resolution,
                    m.info.origin.position.x, m.info.origin.position.y, m.info.resolution)
        self.get_logger().info(f'map {m.info.width}x{m.info.height} received, clearance field ready')

    def map_clear_ahead(self, x, y, P):
        if self.edt is None: return 10.0
        E, ox, oy, res = self.edt; pts = [(x, y)]; acc = 0.0; k = self.idx
        while k + 1 < len(P) and acc < self.a.ahead:
            acc += float(np.hypot(*(P[k + 1] - P[k]))); k += 1; pts.append(P[k])
        q = np.asarray(pts); i = np.clip(((q[:, 1] - oy) / res).astype(int), 0, E.shape[0] - 1)
        j = np.clip(((q[:, 0] - ox) / res).astype(int), 0, E.shape[1] - 1)
        return float(E[i, j].min())

    # ---- MPPI action client helpers (called from the execute thread; the executor has free threads for the responses) ----
    def _wait(self, fut, timeout):
        t0 = time.time()
        while not fut.done() and time.time() - t0 < timeout: time.sleep(0.005)
        return fut.done()

    def mppi_send(self, path_msg, my):
        if not self.mppi.wait_for_server(timeout_sec=1.0):
            self.get_logger().error('MPPI server follow_path_mppi_unused not available'); return False
        g = FollowPath.Goal(); g.path = path_msg; g.controller_id = ''; g.goal_checker_id = ''
        f = self.mppi.send_goal_async(g)
        if not self._wait(f, 2.0) or not f.result().accepted:
            self.get_logger().error('MPPI goal not accepted'); return False
        self.mppi_gh = f.result(); self.mppi_res = self.mppi_gh.get_result_async(); self.mppi_for = my
        return True

    def mppi_cancel(self):
        if self.mppi_gh is not None:
            try: self._wait(self.mppi_gh.cancel_goal_async(), 1.0)
            except Exception: pass
        self.mppi_gh, self.mppi_res, self.mppi_for = None, None, None

    def set_mode(self, m, why, x, y):
        self.n_switch += 1
        self.get_logger().info(f'switch {self.mode} -> {m} at ({x:.2f}, {y:.2f}): {why}')
        self.mode, self.mode_t = m, time.time()

    def escape(self, gh, P):
        """Scan-based escape; runs in the calling execute thread while it holds mode_lock (newer BT paths wait for it).
        Returns True if the rover moved or turned."""
        self.escapes += 1; self.n_escape += 1
        if self.mode == 'mppi': self.mppi_cancel()
        self.mode, self.mode_t = 'drl', 0.0
        rear = rear_clearance(self.scan); x0, y0, yaw0 = self.pose(); moved = 0.0
        self.get_logger().info(f'ESCAPE {self.escapes} at ({x0:.2f}, {y0:.2f}): rear clearance {rear:.2f} m')
        t0 = time.time()
        if rear > 0.35:
            while time.time() - t0 < 5.0 and not gh.is_cancel_requested:
                if rear_clearance(self.scan) < 0.15: break
                x, y, _ = self.pose(); moved = math.hypot(x - x0, y - y0)
                if moved >= 0.30: break
                c = Twist(); c.linear.x = -0.10; self.pub.publish(c); time.sleep(0.1)
            self.stop(); time.sleep(0.3)
        t0 = time.time(); turned = 0.0
        while time.time() - t0 < 6.0 and not gh.is_cancel_requested:
            x, y, yaw = self.pose(); sx, sy = self.subgoal(x, y, P)
            b = math.atan2(math.sin(math.atan2(sy - y, sx - x) - yaw), math.cos(math.atan2(sy - y, sx - x) - yaw))
            if abs(b) < 0.35 or turn_clearance(self.scan) < 0.05: break
            c = Twist(); c.angular.z = 0.6 * math.copysign(1.0, b); self.pub.publish(c); time.sleep(0.1)
            turned = abs(math.atan2(math.sin(yaw - yaw0), math.cos(yaw - yaw0)))
        self.stop(); self.frames.clear(); self.prev_a = np.zeros(2, np.float32); self.prog_ref = None
        self.get_logger().info(f'ESCAPE done: reversed {moved:.2f} m, turned {math.degrees(turned):.0f} deg')
        return moved > 0.05 or turned > 0.2

    def end_nav(self, how):
        s = self.split; tot = s['drl'] + s['mppi']
        self.get_logger().info(f'navigation {how}: drl {s["drl"]:.2f} m, mppi {s["mppi"]:.2f} m'
                               f' -> DRL share {100 * s["drl"] / tot if tot > 0 else 0:.0f}% | cumulative drl {self.total["drl"]:.1f} m,'
                               f' mppi {self.total["mppi"]:.1f} m')

    def _execute(self, gh):
        a = self.a; my = bytes(gh.goal_id.uuid)
        with self.lock:
            new_nav = self.active is None
            self.active = my
            self.path_xy = np.array([[p.pose.position.x, p.pose.position.y] for p in gh.request.path.poses], float)
            self.idx = 0
            if new_nav:
                self.frames.clear(); self.prev_a = np.zeros(2, np.float32); self.prog_ref, self.prog_t = None, time.time()
                self.split = {'drl': 0.0, 'mppi': 0.0}; self.last_xy = None; self.escapes = 0
                if self.mode == 'mppi': self.mppi_cancel()
                self.mode, self.mode_t = 'drl', 0.0
        res = FollowPath.Result()
        if len(self.path_xy) == 0:
            res.error_code = FollowPath.Result.INVALID_PATH; gh.abort(); self.active = None; return res

        def finish(kind, code=None, msg=''):
            if self.mode == 'mppi': self.mppi_cancel()
            self.stop(); self.active = None; self.end_nav(kind)
            if kind == 'succeeded': gh.succeed()
            elif kind == 'canceled': gh.canceled()
            else:
                res.error_code = code; res.error_msg = msg; gh.abort()
            return res

        tf_fail_t = None; fb = FollowPath.Feedback()
        while rclpy.ok():
            with self.lock:
                if self.active != my:  # superseded by a newer path: the newer call takes over (and re-sends to MPPI if needed)
                    gh.abort(); return res
                P = self.path_xy
            goal = P[-1]
            if gh.is_cancel_requested: return finish('canceled')
            if not self.scan_ev.wait(1.0): continue
            self.scan_ev.clear()
            try:
                x, y, yaw = self.pose(); tf_fail_t = None
            except Exception:
                tf_fail_t = tf_fail_t or time.time()
                if time.time() - tf_fail_t > 2.0: return finish('aborted', FollowPath.Result.TF_ERROR, 'TF')
                continue
            if self.last_xy is not None:
                d = math.hypot(x - self.last_xy[0], y - self.last_xy[1]); self.split[self.mode] += d; self.total[self.mode] += d
            self.last_xy = (x, y)
            dist = math.hypot(goal[0] - x, goal[1] - y)
            if dist <= a.tol: return finish('succeeded')
            if self.prog_ref is None or math.hypot(x - self.prog_ref[0], y - self.prog_ref[1]) >= 0.20:
                self.prog_ref, self.prog_t = (x, y), time.time()
            elif self.mode == 'drl' and time.time() - self.prog_t > 10.0:
                with self.mode_lock:
                    if self.active != my: continue
                    if self.escapes < self.a.escapes and self.escape(gh, P): continue
                self.get_logger().warn('no progress for 10 s in drl mode: aborting FollowPath (BT recoveries)')
                return finish('aborted', FollowPath.Result.FAILED_TO_MAKE_PROGRESS, 'hybrid: failed to make progress')

            with self.mode_lock:
                if self.active != my: continue  # superseded while waiting: the newer call owns the mode and MPPI
                clear = footprint_clearance(self.scan); mclear = self.map_clear_ahead(x, y, P)
                if self.mode == 'drl' and (dist < a.g_in or clear < a.c_in or mclear < a.map_c):
                    why = (f'goal {dist:.2f} m' if dist < a.g_in else f'scan clearance {clear:.2f} m' if clear < a.c_in
                           else f'narrow ahead (map clearance {mclear:.2f} m)')
                    self.set_mode('mppi', why, x, y); self.prog_ref = None
                elif (self.mode == 'mppi' and time.time() - self.mode_t >= a.dwell and clear > a.c_out and dist > a.g_in + 0.3
                      and mclear >= a.map_c + 0.1):
                    self.mppi_cancel(); self.frames.clear(); self.prev_a = np.zeros(2, np.float32)
                    self.set_mode('drl', f'open ahead (map {mclear:.2f} m, scan {clear:.2f} m)', x, y); self.prog_ref = None

                if self.mode == 'mppi':
                    if self.mppi_for != my:  # entered MPPI mode, or the BT sent a new path: (re)send it to MPPI (preempts)
                        if not self.mppi_send(gh.request.path, my):
                            return finish('aborted', FollowPath.Result.UNKNOWN, 'hybrid: MPPI unavailable')
                    elif self.mppi_res is not None and self.mppi_res.done():
                        st = self.mppi_res.result().status; code = self.mppi_res.result().result.error_code
                        self.mppi_gh, self.mppi_res, self.mppi_for = None, None, None
                        if st == GoalStatus.STATUS_SUCCEEDED: return finish('succeeded')
                        if code == FollowPath.Result.FAILED_TO_MAKE_PROGRESS and self.escapes < a.escapes and self.escape(gh, P):
                            continue
                        self.get_logger().warn(f'MPPI ended with status {st}, error {code}: aborting FollowPath (BT recoveries)')
                        return finish('aborted', code or FollowPath.Result.UNKNOWN, 'hybrid: MPPI failed')
                else:
                    with self.lock:
                        if self.active != my: continue
                        obs = self.observation(x, y, yaw, P)
                    act = self.actor(obs); self.prev_a = act.astype(np.float32)
                    cmd = Twist(); cmd.linear.x = float(a.v_min + (act[0] + 1) / 2 * (V_MAX - a.v_min)); cmd.angular.z = float(act[1] * W_MAX)
                    self.pub.publish(cmd)
            fb.distance_to_goal = float(dist); gh.publish_feedback(fb)
        return res


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--actor', default=str(HERE / 'actor_run3.npz')); ap.add_argument('--tol', type=float, default=0.15)
    ap.add_argument('--lookahead', type=float, default=1.5); ap.add_argument('--v-min', type=float, default=0.0)
    ap.add_argument('--c-in', type=float, default=0.25); ap.add_argument('--c-out', type=float, default=0.40)
    ap.add_argument('--g-in', type=float, default=1.0); ap.add_argument('--dwell', type=float, default=2.0)
    ap.add_argument('--escapes', type=int, default=2)
    ap.add_argument('--map-c', type=float, default=0.9); ap.add_argument('--ahead', type=float, default=2.0)
    a, _ = ap.parse_known_args()
    from rclpy.signals import SignalHandlerOptions
    rclpy.init(signal_handler_options=SignalHandlerOptions.NO)
    def on_sig(signum, frame):
        print(f'{time.strftime("%H:%M:%S")} received signal {signal.Signals(signum).name}, shutting down', flush=True)
        try: node.stop(); node.end_nav('at shutdown'); print(f'escapes total {node.n_escape}', flush=True)
        except Exception: pass
        rclpy.try_shutdown(); os._exit(0)
    signal.signal(signal.SIGINT, on_sig); signal.signal(signal.SIGTERM, on_sig)
    import faulthandler; faulthandler.register(signal.SIGUSR1, all_threads=True)
    node = HybridController(a); ex = MultiThreadedExecutor(num_threads=8); ex.add_node(node)
    try: ex.spin()
    except Exception: print('executor crashed:\n' + traceback.format_exc(), flush=True)
    finally:
        try: node.stop()
        except Exception: pass
        node.destroy_node(); rclpy.try_shutdown()


if __name__ == '__main__':
    main()
