#!/usr/bin/env python3
"""SAC on generated worlds with a curriculum (PLAN.md Phases 3/5). pyenv python3 (3.10: torch/SB3).

Train worlds: training/worlds/gen/train (tiers 0-3). Each worker switches world every --epw episodes (spares pre-load).
Curriculum: stage k trains on tier k (50%) + tiers < k (50%). Validation runs concurrently on --eval-envs extra workers
with the deterministic policy, on the val worlds of the current tier; promote when the last --window val episodes of
that tier reach --promote success. In the last stage, validation cycles through all 30 val worlds (all tiers).
Test worlds and assistive_world are never touched here.

usage: python3 training/gz_env/train_sac_gen.py --minutes 150
"""
import argparse, csv, json, os, random, subprocess, sys, threading, time
from collections import deque, defaultdict
from datetime import datetime
from pathlib import Path
import numpy as np
import torch
from stable_baselines3 import SAC
from stable_baselines3.common.callbacks import BaseCallback
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import RotatingGzVecEnv  # noqa: E402

PKG = Path(__file__).resolve().parents[2]
GEN = PKG / 'training' / 'worlds' / 'gen'
N_TIERS = 4


def tier_of(name): return int(Path(name).stem[1])


class Curriculum:
    def __init__(self, seed):
        self.stage = 0; self.rng = random.Random(seed)
        self.train = defaultdict(list); self.val = defaultdict(list)
        for f in sorted((GEN / 'train').glob('*.sdf')): self.train[tier_of(f)].append(f)
        for f in sorted((GEN / 'val').glob('*.sdf')): self.val[tier_of(f)].append(f)
        self._val_i = 0

    def train_world(self):
        k = self.stage
        t = k if (k == 0 or self.rng.random() < 0.5) else self.rng.randrange(k)
        return self.rng.choice(self.train[t])

    def val_world(self):
        pool = self.val[self.stage] if self.stage < N_TIERS - 1 else [w for t in range(N_TIERS) for w in self.val[t]]
        w = pool[self._val_i % len(pool)]; self._val_i += 1; return w


class BgTrainer(threading.Thread):
    """Gradient steps in a background thread, overlapping the (slower) Gazebo env steps. Keeps
    updates ~= utd * (env steps - learning_starts); the main loop waits if the trainer falls > max_lag behind.
    SB3's own training is disabled (gradient_steps=0). pause() is used around model.save()."""

    def __init__(self, model, utd, batch, learning_starts, max_lag=2000):
        super().__init__(daemon=True); self.m, self.utd, self.batch, self.ls, self.max_lag = model, utd, batch, learning_starts, max_lag
        self.done = 0; self.stop = False; self.idle = threading.Event(); self.run_ok = threading.Event(); self.run_ok.set()
        self.error = None

    @staticmethod
    def lock_actor(model):
        """SB3's actor keeps its action distribution as state (set and read within forward/action_log_prob), so the
        main thread's predict and this thread's train() must not interleave inside those calls."""
        lock = threading.Lock(); actor = model.actor
        for name in ('forward', 'action_log_prob'):
            orig = getattr(actor, name)
            def locked(*a, _orig=orig, **k):
                with lock: return _orig(*a, **k)
            setattr(actor, name, locked)

    def target(self): return self.utd * max(0, self.m.num_timesteps - self.ls)

    def run(self):
        try:
            while not self.stop:
                if not self.run_ok.is_set() or self.done >= self.target():
                    self.idle.set(); time.sleep(0.002); continue
                self.idle.clear(); self.m.train(gradient_steps=1, batch_size=self.batch); self.done += 1
        except Exception as e:  # surfaced by the callback
            self.error = e; raise

    def pause(self):
        self.run_ok.clear(); self.idle.wait(5)

    def resume(self): self.run_ok.set()


class Log(BaseCallback):
    def __init__(self, out, a, cur, eval_env):
        super().__init__(); self.out, self.a, self.cur, self.ev = out, a, cur, eval_env; self.bg = None
        self.deadline = time.time() + 60 * a.minutes; self.t0 = time.time(); self.last_t, self.last_n = time.time(), 0
        self.train_win = defaultdict(lambda: deque(maxlen=100)); self.val_eps = []; self.rows = []
        self.next_ckpt = time.time() + 60 * a.ckpt_minutes; self.stage_log = [dict(stage=0, steps=0, minutes=0.0)]
        self.ev_csv = open(out / 'val_episodes.csv', 'w', newline=''); self.ev_w = None

    def _on_training_start(self):
        self.ev_obs = self.ev.reset(); self.ev_pending = False

    def _eval_tick(self):
        if self.ev_pending:
            self.ev_obs, _, dones, infos = self.ev.step_wait()
            for d, info in zip(dones, infos):
                if d:
                    row = dict(steps=self.num_timesteps, minutes=round((time.time() - self.t0) / 60, 2), stage=self.cur.stage,
                               world=info['world'], tier=tier_of(info['world']), **{k: info[k] for k in
                               ('is_success', 'collision', 'stuck', 'timeout', 'ep_len', 'd_start', 'dist_goal')})
                    self.val_eps.append(row)
                    if self.ev_w is None: self.ev_w = csv.DictWriter(self.ev_csv, fieldnames=list(row)); self.ev_w.writeheader()
                    self.ev_w.writerow(row); self.ev_csv.flush()
                    self._maybe_promote()
        act, _ = self.model.predict(self.ev_obs, deterministic=True)
        self.ev.step_async(act); self.ev_pending = True

    def _maybe_promote(self):
        k = self.cur.stage
        if k >= N_TIERS - 1: return
        since = self.stage_log[-1]['steps']
        eps = [e for e in self.val_eps if e['tier'] == k and e['steps'] >= since]
        if len(eps) >= self.a.window and np.mean([e['is_success'] for e in eps[-self.a.window:]]) >= self.a.promote:
            self._save(self.out / f'model_stage{k}_promoted')
            self.cur.stage = k + 1
            self.stage_log.append(dict(stage=k + 1, steps=self.num_timesteps, minutes=round((time.time() - self.t0) / 60, 1),
                                       val_success_prev_tier=float(np.mean([e['is_success'] for e in eps[-self.a.window:]]))))
            print('PROMOTED', json.dumps(self.stage_log[-1]), flush=True)
            (self.out / 'stages.json').write_text(json.dumps(self.stage_log, indent=1))

    def _save(self, path):
        if self.bg: self.bg.pause()
        self.model.save(path)
        if self.bg: self.bg.resume()

    def _on_step(self):
        if self.bg:
            if self.bg.error: raise self.bg.error
            while self.bg.done < self.bg.target() - self.bg.max_lag and not self.bg.error: time.sleep(0.001)
        for info in self.locals['infos']:
            if 'is_success' in info:
                self.train_win[tier_of(info['world'])].append((info['is_success'], info['collision'], info['stuck'], info['timeout']))
        self._eval_tick()
        n = self.num_timesteps
        if n - self.last_n >= self.a.log_every:
            now = time.time(); sps = (n - self.last_n) / (now - self.last_t); self.last_t, self.last_n = now, n
            gpu = subprocess.run(['nvidia-smi', '--query-gpu=utilization.gpu,memory.used', '--format=csv,noheader,nounits'],
                                 capture_output=True, text=True).stdout.strip()
            row = dict(steps=n, minutes=round((now - self.t0) / 60, 1), steps_per_s=round(sps, 1), stage=self.cur.stage)
            for t in range(N_TIERS):
                w = np.array(self.train_win[t], float) if self.train_win[t] else np.full((1, 4), np.nan)
                row[f'train_succ_t{t}'] = round(float(w[:, 0].mean()), 3); row[f'train_coll_t{t}'] = round(float(w[:, 1].mean()), 3)
                v = [e['is_success'] for e in self.val_eps if e['tier'] == t][-self.a.window:]
                row[f'val_succ_t{t}'] = round(float(np.mean(v)), 3) if v else float('nan'); row[f'val_n_t{t}'] = len(v)
            row.update(grad_updates=self.bg.done if self.bg else self.model._n_updates, swaps=self.training_env.n_swaps, swap_miss=self.training_env.n_swap_miss, gpu_util_mem=gpu)
            self.rows.append(row); print(json.dumps(row), flush=True)
            with open(self.out / 'progress.csv', 'w', newline='') as f:
                wr = csv.DictWriter(f, fieldnames=list(row)); wr.writeheader(); wr.writerows(self.rows)
        if time.time() > self.next_ckpt:
            self._save(self.out / f'ckpt_{n // 1000}k'); self.next_ckpt = time.time() + 60 * self.a.ckpt_minutes
        return time.time() < self.deadline


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--minutes', type=float, default=150); ap.add_argument('--n-envs', type=int, default=6)
    ap.add_argument('--n-spare', type=int, default=2); ap.add_argument('--eval-envs', type=int, default=2)
    ap.add_argument('--epw', type=int, default=10, help='episodes per world before a worker switches world')
    ap.add_argument('--batch', type=int, default=1024); ap.add_argument('--grad-steps', type=int, default=2, help='per vec step')
    ap.add_argument('--buffer', type=int, default=1_000_000); ap.add_argument('--learning-starts', type=int, default=10_000)
    ap.add_argument('--promote', type=float, default=0.8); ap.add_argument('--window', type=int, default=40)
    ap.add_argument('--ckpt-minutes', type=float, default=20); ap.add_argument('--log-every', type=int, default=6000)
    ap.add_argument('--seed', type=int, default=0)
    ap.add_argument('--utd', type=float, default=1 / 3, help='gradient updates per env step (background trainer)')
    ap.add_argument('--no-bg', action='store_true', help='SB3 sequential training with --grad-steps per vec step instead')
    a = ap.parse_args()
    torch.set_num_threads(4)
    out = PKG / 'training' / 'runs' / f"sac_gen_{datetime.now():%Y%m%d_%H%M%S}"; out.mkdir(parents=True)
    cur = Curriculum(a.seed)
    (out / 'config.json').write_text(json.dumps(dict(vars(a), torch=torch.__version__, cuda=torch.cuda.is_available(),
                                                     utd_effective=a.grad_steps / a.n_envs if a.no_bg else a.utd,
                                                     env_cfg_override=os.environ.get('GZNAV_CFG', '')), indent=1))
    venv = RotatingGzVecEnv(a.n_envs, cur.train_world, episodes_per_world=a.epw, n_spare=a.n_spare, seed=a.seed, log_dir=out / 'train_workers')
    eenv = None
    try:
        eenv = RotatingGzVecEnv(a.eval_envs, cur.val_world, episodes_per_world=5, n_spare=1, seed=a.seed + 777, log_dir=out / 'val_workers')
        model = SAC('MlpPolicy', venv, buffer_size=a.buffer, learning_starts=a.learning_starts, batch_size=a.batch, gamma=0.99,
                    train_freq=1, gradient_steps=a.grad_steps if a.no_bg else 0, policy_kwargs=dict(net_arch=[256, 256]),
                    device='cuda' if torch.cuda.is_available() else 'cpu', seed=a.seed, verbose=0)
        cb = Log(out, a, cur, eenv)
        if not a.no_bg:
            BgTrainer.lock_actor(model)
            cb.bg = BgTrainer(model, a.utd, a.batch, a.learning_starts); cb.bg.start()
        model.learn(total_timesteps=100_000_000, callback=cb, log_interval=10 ** 9)
        if cb.bg: cb.bg.stop = True; cb.bg.join(10)
        model.save(out / 'model'); model.save_replay_buffer(out / 'replay_buffer')
        (out / 'stages.json').write_text(json.dumps(cb.stage_log, indent=1))
    finally:
        venv.close()
        if eenv is not None: eenv.close()
    print('run dir:', out)


if __name__ == '__main__':
    main()
