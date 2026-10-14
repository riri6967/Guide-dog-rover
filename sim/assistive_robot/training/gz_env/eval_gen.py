#!/usr/bin/env python3
"""Evaluate a saved SAC policy (deterministic) on generated worlds, exactly N episodes per world, per-tier summary.
pyenv python3 (3.10). Uses rotating workers so 30 worlds do not need 30 Gazebo instances at once.
Episodes beyond N on a world (when no spare was ready to switch) are discarded.

usage: python3 training/gz_env/eval_gen.py MODEL.zip training/worlds/gen/test [--episodes 10] [--n-envs 6] [--out CSV]
"""
import argparse, csv, math, sys, time
from collections import defaultdict
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import SAC
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import RotatingGzVecEnv  # noqa: E402


def wilson(k, n, z=1.96):
    if n == 0: return float('nan'), float('nan')
    p = k / n; d = 1 + z * z / n; c = (p + z * z / (2 * n)) / d; h = z * math.sqrt(p * (1 - p) / n + z * z / (4 * n * n)) / d
    return c - h, c + h


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('model'); ap.add_argument('world_dir')
    ap.add_argument('--episodes', type=int, default=10); ap.add_argument('--n-envs', type=int, default=6)
    ap.add_argument('--seed', type=int, default=4242); ap.add_argument('--out', default=None)
    a = ap.parse_args()
    torch.set_num_threads(4)
    worlds = sorted(Path(a.world_dir).glob('*.sdf')); it = iter(worlds * 10)
    venv = RotatingGzVecEnv(min(a.n_envs, len(worlds)), lambda: next(it), episodes_per_world=a.episodes, n_spare=3, seed=a.seed,
                            log_dir=Path(a.out).with_suffix('') if a.out else None)
    model = SAC.load(a.model, device='cpu')
    res = defaultdict(list); t0 = time.time()
    try:
        obs = venv.reset()
        while any(len(res[w.stem]) < a.episodes for w in worlds):
            act, _ = model.predict(obs, deterministic=True)
            obs, _, dones, infos = venv.step(act)
            for d, info in zip(dones, infos):
                if d and len(res[info['world']]) < a.episodes: res[info['world']].append(info)
    finally:
        venv.close()
    rows = [dict(world=w, tier=int(w[1]), **{k: info[k] for k in ('is_success', 'collision', 'stuck', 'timeout', 'ep_len', 'd_start', 'dist_goal')})
            for w in sorted(res) for info in res[w]]
    if a.out:
        with open(a.out, 'w', newline='') as f:
            wr = csv.DictWriter(f, fieldnames=list(rows[0])); wr.writeheader(); wr.writerows(rows)
    print(f'{a.model} on {a.world_dir}: {len(rows)} episodes in {(time.time() - t0) / 60:.1f} min')
    for t in sorted({r['tier'] for r in rows}) + ['all']:
        rr = [r for r in rows if t == 'all' or r['tier'] == t]; k = sum(r['is_success'] for r in rr); lo, hi = wilson(k, len(rr))
        succ = [r for r in rr if r['is_success']]
        print(f"tier {t}: success {k}/{len(rr)} = {100 * k / len(rr):.0f}% (Wilson {100 * lo:.0f}-{100 * hi:.0f}%), "
              f"collision {sum(r['collision'] for r in rr)}, stuck {sum(r['stuck'] for r in rr)}, timeout {sum(r['timeout'] for r in rr)}"
              f" | successful: mean {np.mean([r['ep_len'] for r in succ]) / 10 if succ else float('nan'):.1f} s for "
              f"{np.mean([r['d_start'] for r in succ]) if succ else float('nan'):.1f} m geodesic")


if __name__ == '__main__':
    main()
