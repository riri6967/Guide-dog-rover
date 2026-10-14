#!/usr/bin/env python3
"""Learner-side VecEnv (pyenv python 3.10: torch/SB3). Each sub-env is a worker.py process under /usr/bin/python3
(gz bindings) running its own headless Gazebo in lockstep. Talks over a unix socket (no authkey; socket in a 0700 dir).
"""
import os, subprocess, tempfile, time
from multiprocessing.connection import Listener
from pathlib import Path
import numpy as np
from gymnasium import spaces
from stable_baselines3.common.vec_env.base_vec_env import VecEnv

HERE = Path(__file__).resolve().parent
OBS_DIM = 367  # must match gz_nav_env.OBS_DIM


class GzVecEnv(VecEnv):
    def __init__(self, worlds, seed=0, log_dir=None):
        n = len(worlds)
        self._dir = tempfile.mkdtemp(prefix='gzvec_'); os.chmod(self._dir, 0o700)
        addr = os.path.join(self._dir, 'sock'); self._listener = Listener(addr, family='AF_UNIX')
        log_dir = Path(log_dir or self._dir); log_dir.mkdir(parents=True, exist_ok=True)
        self.procs = [subprocess.Popen(['/usr/bin/python3', str(HERE / 'worker.py'), '--world', str(w), '--index', str(i),
                                        '--seed', str(seed * 100 + i), '--addr', addr],
                                       stdout=open(log_dir / f'worker_{i}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
                      for i, w in enumerate(worlds)]
        conns = {}
        while len(conns) < n:
            c = self._listener.accept(); msg = c.recv()
            if msg[0] != 'ready': raise RuntimeError(f'worker failed:\n{msg[1]}')
            conns[msg[1]] = c
        self.conns = [conns[i] for i in range(n)]
        super().__init__(n, spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32), spaces.Box(-1.0, 1.0, (2,), np.float32))
        self._actions = None

    def _recv(self, c):
        m = c.recv()
        if isinstance(m, tuple) and len(m) == 2 and m[0] == 'error': raise RuntimeError(f'worker error:\n{m[1]}')
        return m

    def reset(self):
        for c in self.conns: c.send(('reset',))
        return np.stack([self._recv(c) for c in self.conns])

    def step_async(self, actions):
        for c, a in zip(self.conns, actions): c.send(('step', np.asarray(a, np.float32)))

    def step_wait(self):
        out = [self._recv(c) for c in self.conns]
        obs, rew, term, trunc, infos = zip(*out)
        dones = np.array(term) | np.array(trunc)
        return np.stack(obs), np.array(rew, np.float32), dones, list(infos)

    def close(self):
        for c in self.conns:
            try: c.send(('close',))
            except Exception: pass
        for p in self.procs:
            try: p.wait(20)
            except subprocess.TimeoutExpired: p.kill()
        self._listener.close()

    # minimal VecEnv API
    def get_attr(self, name, indices=None): return [None] * self.num_envs
    def set_attr(self, name, value, indices=None): pass
    def env_method(self, name, *args, indices=None, **kw): return [None] * self.num_envs
    def env_is_wrapped(self, wrapper_class, indices=None): return [False] * self.num_envs
    def seed(self, seed=None): return [None] * self.num_envs


class RotatingGzVecEnv(VecEnv):
    """n_envs active Gazebo workers; each switches to a new world (from sampler()) after episodes_per_world episodes.
    The next worlds are pre-loaded by n_spare spare workers in the background, so a switch costs one reset, not a Gazebo
    start. If no spare is ready yet, the slot keeps its current world. Swaps happen only at episode boundaries; the
    terminal info of the finished episode is kept, the returned obs is the spare's fresh reset obs.
    info['world'] names the world of every finished episode."""

    def __init__(self, n_envs, sampler, episodes_per_world=10, n_spare=2, seed=0, log_dir=None, start_timeout=180):
        import threading
        self.sampler, self.epw, self.n_spare, self._seed = sampler, episodes_per_world, n_spare, seed
        self._dir = tempfile.mkdtemp(prefix='gzrot_'); os.chmod(self._dir, 0o700)
        self.addr = os.path.join(self._dir, 'sock'); self._listener = Listener(self.addr, family='AF_UNIX')
        self.log_dir = Path(log_dir or self._dir); self.log_dir.mkdir(parents=True, exist_ok=True)
        self._lock = threading.Lock(); self._ready = {}; self._closing = False
        self._next_id = 0; self.pending = {}; self.retiring = []; self.spares = []; self.n_swaps = 0; self.n_swap_miss = 0
        self._wlog = open(self.log_dir / 'world_log.csv', 'a'); self._wlog.write('time,event,worker,world\n')
        threading.Thread(target=self._accept_loop, daemon=True).start()
        for _ in range(n_envs + n_spare): self._launch()
        self.slots = []; t0 = time.time()
        while len(self.slots) < n_envs:
            self._housekeeping()
            for wid in list(self.spares):
                if len(self.slots) < n_envs: self.spares.remove(wid); self.slots.append(self._activate(wid))
            if time.time() - t0 > start_timeout: self.close(); raise RuntimeError('workers did not start')
            time.sleep(0.2)
        super().__init__(n_envs, spaces.Box(-np.inf, np.inf, (OBS_DIM,), np.float32), spaces.Box(-1.0, 1.0, (2,), np.float32))

    # ---- worker management
    def _log(self, event, wid, world):
        self._wlog.write(f'{time.time():.1f},{event},{wid},{Path(world).stem}\n'); self._wlog.flush()

    def _launch(self):
        wid = self._next_id; self._next_id += 1; world = Path(self.sampler())
        p = subprocess.Popen(['/usr/bin/python3', str(HERE / 'worker.py'), '--world', str(world), '--index', str(wid),
                              '--seed', str(self._seed * 100000 + wid), '--addr', self.addr],
                             stdout=open(self.log_dir / f'worker_{wid % 50}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
        self.pending[wid] = dict(proc=p, world=world, t=time.time()); self._log('launch', wid, world)

    def _accept_loop(self):
        while True:  # until the listener is closed (also during close(), so late starters can still be shut down)
            try: c = self._listener.accept(); msg = c.recv()
            except Exception: return
            if msg[0] == 'ready':
                with self._lock: self._ready[msg[1]] = c

    def _housekeeping(self):
        with self._lock: ready = dict(self._ready); self._ready.clear()
        for wid, c in ready.items():
            self.pending[wid]['conn'] = c; self.spares.append(wid)
        for wid in list(self.pending):  # a worker that died before 'ready': replace it
            w = self.pending[wid]
            if 'conn' not in w and w['proc'].poll() is not None:
                self._log('failed', wid, w['world']); del self.pending[wid]; self._launch()
        self.retiring = [p for p in self.retiring if p.poll() is None]

    def _activate(self, wid):
        w = self.pending.pop(wid); w['id'] = wid; w['eps'] = 0
        w['conn'].send(('reset',)); w['obs'] = self._recv(w['conn']); self._log('active', wid, w['world'])
        return w

    def _retire(self, s):
        try: s['conn'].send(('close',))
        except Exception: pass
        self.retiring.append(s['proc']); self._log('retire', s['id'], s['world'])

    def _recv(self, c):
        m = c.recv()
        if isinstance(m, tuple) and len(m) == 2 and m[0] == 'error': raise RuntimeError(f'worker error:\n{m[1]}')
        return m

    # ---- VecEnv API
    def reset(self):
        for s in self.slots: s['conn'].send(('reset',))
        return np.stack([self._recv(s['conn']) for s in self.slots])

    def step_async(self, actions):
        for s, a in zip(self.slots, actions): s['conn'].send(('step', np.asarray(a, np.float32)))

    def step_wait(self):
        out = [self._recv(s['conn']) for s in self.slots]
        obs, rew, term, trunc, infos = (list(x) for x in zip(*out))
        dones = np.array(term) | np.array(trunc)
        self._housekeeping()
        for i, s in enumerate(self.slots):
            if not dones[i]: continue
            infos[i]['world'] = s['world'].stem; s['eps'] += 1
            if s['eps'] >= self.epw:
                if self.spares:
                    new = self._activate(self.spares.pop(0)); obs[i] = new['obs']; self._retire(s); self.slots[i] = new
                    self.n_swaps += 1; self._launch()
                else:
                    self.n_swap_miss += 1
        return np.stack(obs), np.array(rew, np.float32), dones, infos

    def close(self):
        self._closing = True
        t0 = time.time()  # let still-starting workers finish starting, so their Gazebo servers get shut down cleanly
        while any('conn' not in w for w in self.pending.values()) and time.time() - t0 < 120:
            self._housekeeping(); time.sleep(0.5)
        for s in self.slots + [w for w in self.pending.values() if 'conn' in w]:
            try: s['conn'].send(('close',))
            except Exception: pass
        procs = [s['proc'] for s in self.slots] + [w['proc'] for w in self.pending.values()] + self.retiring
        for p in procs:
            try: p.wait(30)
            except subprocess.TimeoutExpired:
                p.kill(); print(f'WARNING: killed worker pid {p.pid}; its Gazebo server may be orphaned', flush=True)
        self._listener.close(); self._wlog.close()

    def get_attr(self, name, indices=None): return [None] * self.num_envs
    def set_attr(self, name, value, indices=None): pass
    def env_method(self, name, *args, indices=None, **kw): return [None] * self.num_envs
    def env_is_wrapped(self, wrapper_class, indices=None): return [False] * self.num_envs
    def seed(self, seed=None): return [None] * self.num_envs
