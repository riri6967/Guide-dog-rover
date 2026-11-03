#!/usr/bin/env python3
"""Diagnostics (pyenv python3 3.10): run a policy deterministically on worlds with per-step traces (GZNAV_TRACE=1) and
pickle all terminal infos. Use on VALIDATION worlds only (test worlds stay untouched).
usage: python3 training/gz_env/diag_failures.py MODEL.zip OUT.pkl WORLD.sdf [WORLD.sdf ...] [--episodes 10]
"""
import argparse, os, pickle, sys
from collections import defaultdict
from pathlib import Path
os.environ['GZNAV_TRACE'] = '1'
from stable_baselines3 import SAC
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import RotatingGzVecEnv  # noqa: E402

if __name__ == '__main__':
    ap = argparse.ArgumentParser(); ap.add_argument('model'); ap.add_argument('out'); ap.add_argument('worlds', nargs='+')
    ap.add_argument('--episodes', type=int, default=10); ap.add_argument('--n-envs', type=int, default=6)
    a = ap.parse_args(); worlds = [Path(w) for w in a.worlds]; it = iter(worlds * 10)
    venv = RotatingGzVecEnv(min(a.n_envs, len(worlds)), lambda: next(it), episodes_per_world=a.episodes, n_spare=3, seed=999)
    model = SAC.load(a.model, device='cpu'); res = defaultdict(list)
    try:
        obs = venv.reset()
        while any(len(res[w.stem]) < a.episodes for w in worlds):
            act, _ = model.predict(obs, deterministic=True); obs, _, dones, infos = venv.step(act)
            for d, info in zip(dones, infos):
                if d and len(res[info['world']]) < a.episodes:
                    info.pop('terminal_observation', None); res[info['world']].append(info)
    finally:
        venv.close()
    pickle.dump(dict(res), open(a.out, 'wb')); print('episodes', sum(len(v) for v in res.values()))
