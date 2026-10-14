#!/usr/bin/env python3
"""Export the deterministic SAC actor (MLP: obs -> 256 -> 256 -> mu, action = tanh(mu)) to an .npz for numpy inference
in the ROS 2 node (system python 3.12 has rclpy but no torch). pyenv python3 3.10.
usage: python3 training/deploy/export_actor.py MODEL.zip OUT.npz
"""
import sys
import numpy as np, torch
from stable_baselines3 import SAC

m = SAC.load(sys.argv[1], device='cpu'); a = m.actor
lin = [l for l in a.latent_pi if isinstance(l, torch.nn.Linear)] + [a.mu]
assert len(lin) == 3 and all(isinstance(l, torch.nn.ReLU) for l in a.latent_pi if not isinstance(l, torch.nn.Linear))
W = {f'W{i}': l.weight.detach().numpy().astype(np.float32) for i, l in enumerate(lin)}
B = {f'b{i}': l.bias.detach().numpy().astype(np.float32) for i, l in enumerate(lin)}
np.savez(sys.argv[2], **W, **B, source=sys.argv[1])
# check against SB3's deterministic predict
obs = np.random.default_rng(0).uniform(0, 1, (64, m.observation_space.shape[0])).astype(np.float32)
ref, _ = m.predict(obs, deterministic=True)
h = np.maximum(obs @ W['W0'].T + B['b0'], 0); h = np.maximum(h @ W['W1'].T + B['b1'], 0); out = np.tanh(h @ W['W2'].T + B['b2'])
print('max |numpy - SB3| =', float(np.abs(out - ref).max()), '| obs dim', obs.shape[1])
