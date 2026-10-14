#!/usr/bin/env python3
"""Env sanity check: a scripted controller (turn to the subgoal, then drive) should reach most goals.
usage: /usr/bin/python3 training/gz_env/sanity_scripted.py WORLD.sdf [--episodes 30]
"""
import argparse, math, sys, time
from pathlib import Path
import numpy as np
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gz_nav_env import GzNavEnv, OBS_DIM, N_BEAMS, POOL, STACK, V_MAX  # noqa: E402


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('world'); ap.add_argument('--episodes', type=int, default=30); ap.add_argument('--ahead', type=float, default=0.8)
    a = ap.parse_args()
    sdf = Path(a.world); env = GzNavEnv(sdf, sdf.with_suffix('.npz'), instance=90, seed=1, cfg=dict(subgoal_ahead=a.ahead))
    k = (N_BEAMS // POOL) * STACK
    res = []; steps = 0; t0 = time.time()
    try:
        for ep in range(a.episodes):
            obs = env.reset(); assert obs.shape == (OBS_DIM,), obs.shape
            while True:
                b = math.atan2(obs[k + 1], obs[k + 2])
                w = float(np.clip(2.0 * b, -1, 1))
                v = V_MAX * max(0.0, math.cos(b)) if abs(b) < 0.6 else 0.0
                beams = obs[(STACK - 1) * N_BEAMS // POOL:STACK * N_BEAMS // POOL] * 6.0  # newest frame, 90 x 4 deg, index 45 = ahead
                front = float(beams[45 - 8:45 + 9].min())  # +-32 deg
                v *= float(np.clip((front - 0.55) / 0.6, 0.15, 1.0))
                obs, r, term, trunc, info = env.step([2 * v / V_MAX - 1, w]); steps += 1
                if term or trunc:
                    res.append(info); break
    finally:
        wall = time.time() - t0; env.close()
    ok = sum(i['is_success'] for i in res); col = sum(i['collision'] for i in res)
    print(f"episodes {len(res)}: success {ok} ({100 * ok / len(res):.0f}%), collisions {col}, stuck {sum(i['stuck'] for i in res)}, "
          f"timeouts {sum(i['timeout'] for i in res)} | {steps} steps in {wall:.1f} s = {steps / wall:.0f} steps/s | "
          f"mean len {np.mean([i['ep_len'] for i in res]):.0f}, mean return {np.mean([i['ep_return'] for i in res]):.1f}, "
          f"reset err max {max(i['reset_err'] for i in res):.3f} m")
    for i in res:
        if not i['is_success']:
            print(f"  fail: start geo {i['d_start']:.1f} m, end geo {i['d_end_geo']:.2f} m, dist {i['dist_goal']:.2f} m, "
                  f"{'collision' if i['collision'] else 'stuck' if i['stuck'] else 'timeout'} after {i['ep_len']} steps")


if __name__ == '__main__':  # required: the env's helper process uses 'spawn'
    main()
