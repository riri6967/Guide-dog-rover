#!/usr/bin/env python3
"""Watch a trained policy drive in the Gazebo GUI (pyenv python3 3.10). One world at a time: a headless lockstep server (same
env as training/eval) plus a GUI client in the same gz partition, paced to --speed x real time. Green disc = goal.

usage: python3 training/gz_env/watch_policy.py MODEL.zip WORLD.sdf [WORLD.sdf ...] [--episodes 4] [--speed 1.0]
"""
import argparse, os, subprocess, sys, time
from pathlib import Path
os.environ['GZNAV_MARKER'] = '1'
from stable_baselines3 import SAC
sys.path.insert(0, str(Path(__file__).resolve().parent))
from vec_env import GzVecEnv  # noqa: E402

PKG = Path(__file__).resolve().parents[2]


def main():
    ap = argparse.ArgumentParser(); ap.add_argument('model'); ap.add_argument('worlds', nargs='+')
    ap.add_argument('--episodes', type=int, default=4); ap.add_argument('--speed', type=float, default=1.0)
    ap.add_argument('--seed', type=int, default=7)
    a = ap.parse_args()
    model = SAC.load(a.model, device='cpu'); total = []
    for wpath in a.worlds:
        w = Path(wpath); venv = GzVecEnv([w], seed=a.seed); gui = None
        try:
            part = f'gzenv100_{venv.procs[0].pid}'  # GzNavEnv partition: gzenv{instance}_{worker pid}, instance = 100 + index
            env = dict(os.environ, GZ_PARTITION=part, GZ_IP='127.0.0.1',
                       GZ_SIM_RESOURCE_PATH=f"{PKG / 'training' / 'models'}:{PKG / 'models'}:{os.environ.get('GZ_SIM_RESOURCE_PATH', '')}")
            for k in [k for k in env if k.startswith('QT_')]: env.pop(k)  # cv2 (imported via SB3) points Qt at its own plugins
            gui = subprocess.Popen(['gz', 'sim', '-g', '-v', '3'], env=env, stdout=open(f'/tmp/claude-1000/watch_gui_{w.stem}.log', 'w'), stderr=subprocess.STDOUT, start_new_session=True)
            print(f'\n=== {w.stem} (tier {w.stem[1]}): GUI starting, first episode in 8 s ===', flush=True); time.sleep(8)
            obs = venv.reset(); done_eps = 0; t_next = time.time()
            while done_eps < a.episodes:
                act, _ = model.predict(obs, deterministic=True)
                venv.step_async(act); obs, _, dones, infos = venv.step_wait()
                t_next += 0.1 / a.speed; time.sleep(max(0.0, t_next - time.time()))
                if dones[0]:
                    i = infos[0]; done_eps += 1; total.append(i)
                    res = 'SUCCESS' if i['is_success'] else 'COLLISION' if i['collision'] else 'STUCK' if i['stuck'] else 'TIMEOUT'
                    print(f"  episode {done_eps}: {res:9s} {i['ep_len'] / 10:5.1f} s, path {i['d_start']:.1f} m, "
                          f"ended {i['dist_goal']:.2f} m from goal", flush=True)
                    time.sleep(1.5); t_next = time.time()
        finally:
            if gui is not None:
                try: os.killpg(gui.pid, 2); gui.wait(10)
                except Exception: gui.kill()
            venv.close()
    k = sum(i['is_success'] for i in total)
    print(f'\nTOTAL: {k}/{len(total)} successes, {sum(i["collision"] for i in total)} collisions')


if __name__ == '__main__':
    main()
