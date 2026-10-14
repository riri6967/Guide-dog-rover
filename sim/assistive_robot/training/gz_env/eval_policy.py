#!/usr/bin/env python3
"""Evaluate a saved SAC policy (deterministic) on a set of worlds (pyenv python3 3.10).
usage: python3 training/gz_env/eval_policy.py RUN_DIR/model.zip WORLD_DIR [--episodes-per-world 30] [--seed 123]
"""
import argparse, math, sys
from pathlib import Path
import numpy as np
from stable_baselines3 import SAC
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import GzVecEnv  # noqa: E402


def wilson(k, n, z=1.96):
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('model'); ap.add_argument('world_dir')
    ap.add_argument('--episodes-per-world', type=int, default=30); ap.add_argument('--seed', type=int, default=123)
    a = ap.parse_args()
    worlds = sorted(Path(a.world_dir).glob('*.sdf'))
    venv = GzVecEnv(worlds, seed=a.seed)
    model = SAC.load(a.model, device='cpu')
    res = {i: [] for i in range(len(worlds))}
    try:
        obs = venv.reset()
        while min(len(r) for r in res.values()) < a.episodes_per_world:
            act, _ = model.predict(obs, deterministic=True)
            venv.step_async(act); obs, _, dones, infos = venv.step_wait()
            for i, (d, info) in enumerate(zip(dones, infos)):
                if d and len(res[i]) < a.episodes_per_world: res[i].append(info)
    finally:
        venv.close()
    allr = [x for r in res.values() for x in r]
    for i, w in enumerate(worlds):
        r = res[i]; k = sum(x['is_success'] for x in r)
        print(f"{w.name}: success {k}/{len(r)}, collisions {sum(x['collision'] for x in r)}, stuck {sum(x['stuck'] for x in r)}, "
              f"timeouts {sum(x['timeout'] for x in r)}")
    k, n = sum(x['is_success'] for x in allr), len(allr); lo, hi = wilson(k, n)
    succ = [x for x in allr if x['is_success']]
    print(f"TOTAL success {k}/{n} = {100 * k / n:.0f}% (Wilson 95% {100 * lo:.0f}-{100 * hi:.0f}%), "
          f"collisions {sum(x['collision'] for x in allr)}/{n}, stuck {sum(x['stuck'] for x in allr)}/{n} | "
          f"successful episodes: mean {np.mean([x['ep_len'] for x in succ]) / 10 if succ else float('nan'):.1f} s, "
          f"geodesic start dist {np.mean([x['d_start'] for x in succ]) if succ else float('nan'):.1f} m")


if __name__ == '__main__':
    main()
