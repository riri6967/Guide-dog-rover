#!/usr/bin/env python3
"""Env worker (system python 3.12): runs one GzNavEnv and serves it to the learner (pyenv 3.10) over a unix socket.
Started by vec_env.GzVecEnv; not meant to be run by hand.
Protocol (pickled tuples): ('reset',) -> obs | ('step', action) -> (obs, rew, terminated, truncated, info) with auto-reset
(info['terminal_observation'] holds the last obs) | ('close',).
"""
import argparse, sys, traceback
from multiprocessing.connection import Client
from pathlib import Path
sys.path.insert(0, str(Path(__file__).resolve().parent))
from gz_nav_env import GzNavEnv  # noqa: E402


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument('--world', required=True); ap.add_argument('--index', type=int, required=True)
    ap.add_argument('--seed', type=int, default=0); ap.add_argument('--addr', required=True)
    a = ap.parse_args()
    conn = Client(a.addr, family='AF_UNIX')
    env = None
    try:
        sdf = Path(a.world)
        env = GzNavEnv(sdf, sdf.with_suffix('.npz'), instance=100 + a.index % 50, seed=a.seed)
        conn.send(('ready', a.index))
        while True:
            msg = conn.recv()
            if msg[0] == 'reset':
                conn.send(env.reset())
            elif msg[0] == 'step':
                obs, rew, term, trunc, info = env.step(msg[1])
                if term or trunc:
                    info['terminal_observation'] = obs; info['TimeLimit.truncated'] = bool(trunc and not term)
                    if info.get('frozen'):  # restart this world's Gazebo (seen once: a server where the rover never moved)
                        print('frozen sim detected, restarting Gazebo', flush=True)
                        env.close(); env = None; env = GzNavEnv(sdf, sdf.with_suffix('.npz'), instance=100 + a.index % 50, seed=a.seed + 1)
                    obs = env.reset()
                conn.send((obs, rew, term, trunc, info))
            elif msg[0] == 'close':
                break
    except Exception:
        try: conn.send(('error', traceback.format_exc()))
        except Exception: pass
        raise
    finally:
        if env is not None: env.close()


if __name__ == '__main__':  # required: GzNavEnv's helper process uses 'spawn'
    main()
