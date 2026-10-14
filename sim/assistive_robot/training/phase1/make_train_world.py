#!/usr/bin/env python3
"""Make training variants of the world and rover. Originals in worlds/ and models/ are never modified.

usage: /usr/bin/python3 training/phase1/make_train_world.py --step 0.001 --rtf 0 [--keep-camera] [--name NAME]
  writes training/worlds/NAME.sdf (default name: train_s<step>_rtf<rtf>[_cam])
  and (once) training/models/assistive_rover_train/ = rover without the RGB-D camera.
rtf 0 = run as fast as possible (gz-sim does not throttle when the desired real-time factor is 0).
Run with GZ_SIM_RESOURCE_PATH including training/models and models/.
"""
import argparse, re
from pathlib import Path

PKG = Path(__file__).resolve().parents[2]


def make_model():
    src = PKG / 'models' / 'assistive_rover'
    dst = PKG / 'training' / 'models' / 'assistive_rover_train'
    dst.mkdir(parents=True, exist_ok=True)
    sdf = (src / 'model.sdf').read_text()
    sdf, n = re.subn(r'\s*<sensor name="rgbd_camera".*?</sensor>', '', sdf, flags=re.S)
    assert n == 1, 'rgbd_camera block not found exactly once'
    (dst / 'model.sdf').write_text(sdf)
    cfg = (src / 'model.config').read_text().replace('<name>assistive_rover</name>', '<name>assistive_rover_train</name>')
    (dst / 'model.config').write_text(cfg)


def make_world(step, rtf, keep_camera, name):
    w = (PKG / 'worlds' / 'assistive_world.sdf').read_text()
    physics = (f'\n    <physics name="train" type="ignored">\n      <max_step_size>{step}</max_step_size>\n'
               f'      <real_time_factor>{rtf}</real_time_factor>\n    </physics>\n')
    w, n = re.subn(r'(<gravity>[^<]*</gravity>\n)', r'\1' + physics, w, count=1)
    assert n == 1
    if not keep_camera:
        w, n = re.subn(r'<uri>model://assistive_rover</uri>', '<uri>model://assistive_rover_train</uri>', w)
        assert n == 1
    out = PKG / 'training' / 'worlds' / f'{name}.sdf'
    out.write_text(w)
    print(out)


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--step', type=float, default=0.001)
    ap.add_argument('--rtf', type=float, default=0.0)
    ap.add_argument('--keep-camera', action='store_true')
    ap.add_argument('--name')
    a = ap.parse_args()
    make_model()
    make_world(a.step, a.rtf, a.keep_camera,
               a.name or f"train_s{a.step:g}_rtf{a.rtf:g}" + ('_cam' if a.keep_camera else ''))
