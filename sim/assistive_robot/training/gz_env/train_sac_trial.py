#!/usr/bin/env python3
"""Short SAC trial (pyenv python3 = 3.10 with torch/SB3): measures training throughput with a learner running and
checks that success rises. Trial worlds only (training/worlds/trial); assistive_world is never used for training.

usage: python3 training/gz_env/train_sac_trial.py --minutes 30 [--n-envs 3] [--gradient-steps -1]
"""
import argparse, csv, json, subprocess, sys, time
from collections import deque
from datetime import datetime
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import GzVecEnv  # noqa: E402

PKG = Path(__file__).resolve().parents[2]


class Log(BaseCallback):
    def __init__(self, out, minutes, every=2000):
        super().__init__(); self.out, self.deadline, self.every = out, time.time() + 60 * minutes, every
        self.win = deque(maxlen=100); self.t0 = time.time(); self.last_t, self.last_n = time.time(), 0; self.rows = []

    def _on_step(self):
        for info in self.locals['infos']:
            if 'is_success' in info:
                self.win.append((info['is_success'], info['collision'], info['stuck'], info['timeout'], info['ep_len']))
        n = self.num_timesteps
        if n - self.last_n >= self.every:
            now = time.time(); sps = (n - self.last_n) / (now - self.last_t); self.last_t, self.last_n = now, n
            w = np.array(self.win, float) if self.win else np.zeros((1, 5))
            gpu = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used', '--format=csv,noheader,nounits'],
                                 capture_output=True, text=True).stdout.strip()
            row = dict(steps=n, minutes=round((now - self.t0) / 60, 1), steps_per_s=round(sps, 1), episodes=len(self.win),
                       success=round(w[:, 0].mean(), 3), collision=round(w[:, 1].mean(), 3), stuck=round(w[:, 2].mean(), 3),
                       timeout=round(w[:, 3].mean(), 3), ep_len=round(w[:, 4].mean(), 1), gpu_util_mem=gpu)
            self.rows.append(row); print(json.dumps(row), flush=True)
            with open(self.out / 'progress.csv', 'w', newline='') as f:
                wr = csv.DictWriter(f, fieldnames=list(row)); wr.writeheader(); wr.writerows(self.rows)
        return time.time() < self.deadline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=30); ap.add_argument('--n-envs', type=int, default=3)
    ap.add_argument('--gradient-steps', type=int, default=-1); ap.add_argument('--seed', type=int, default=0)
    a = ap.parse_args()
    out = PKG / 'training' / 'runs' / f"sac_trial_{datetime.now():%Y%m%d_%H%M%S}"; out.mkdir(parents=True)
    worlds = sorted((PKG / 'training' / 'worlds' / 'trial').glob('trial_*.sdf'))
    worlds = [worlds[i % len(worlds)] for i in range(a.n_envs)]
    (out / 'config.json').write_text(json.dumps(dict(vars(a), worlds=[w.name for w in worlds], torch=torch.__version__,
                                                     cuda=torch.cuda.is_available()), indent=1))
    venv = GzVecEnv(worlds, seed=a.seed, log_dir=out)
    try:
        model = SAC('MlpPolicy', venv, buffer_size=300_000, learning_starts=5_000, batch_size=256, gamma=0.99,
                    train_freq=1, gradient_steps=a.gradient_steps, policy_kwargs=dict(net_arch=[256, 256]),
                    device='cuda' if torch.cuda.is_available() else 'cpu', seed=a.seed, verbose=0)
        model.learn(total_timesteps=10_000_000, callback=Log(out, a.minutes))
        model.save(out / 'model')
    finally:
        venv.close()
    print('run dir:', out)


if __name__ == '__main__':
    main()
