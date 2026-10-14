# SAC trial 2026-10-10 (30 min, plumbing + throughput check; NOT a result to quote as performance)

Setup: 3 headless Gazebo workers (lockstep, 4 ms physics, 10 Hz control), one trial world each (training/worlds/trial, seed 0).
SAC MlpPolicy [256,256], buffer 300k, learning_starts 5k, batch 256, gamma 0.99, train_freq 1, gradient_steps -1 (3 updates per vec step), CUDA.
Obs 367 (4 x 90 pooled LiDAR, subgoal 1.5 m ahead, v/w, previous action). Reward per training/gz_env/gz_nav_env.py CFG.

## Throughput
- before learning starts: ~165 env steps/s (3 workers), ~590k/h
- with learner: 60.3 env steps/s mean (~217k/h). GPU ~20% util, 0.56 GB. Learner-bound (sequential SAC updates), not Gazebo-bound.

## Learning (rolling last 100 training episodes, on the 3 training worlds)
| min | steps | success | collision | stuck |
|---|---|---|---|---|
| 4.7 | 20k | 16% | 23% | 59% |
| 10.8 | 42k | 57% | 22% | 20% |
| 25.7 | 96k | 83% | 13% | 4% |
| 29.6 | 110k | 82% | 12% | 6% |

## Held-out worlds (training/worlds/trial_heldout, seed 1, never trained on), deterministic policy, 30 episodes each
| world | door | SAC policy | scripted controller (0.8 m lookahead) |
|---|---|---|---|
| trial_1_0 | 1.21 m | 24/30 (5 coll, 1 stuck) | 27/30 (3 coll) |
| trial_1_1 | 1.52 m | 21/30 (5 coll, 4 stuck) | 30/30 |
| trial_1_2 | 1.08 m | 11/30 (11 coll, 8 stuck) | 17/30 (13 coll) |
| total | | **56/90 = 62% (Wilson 95% 52-72%)**, 21 coll | 74/90 = 82% |

Reading: it learns (0 -> 82% on training worlds in 30 min), but generalizes only partly (62% held-out) after 3 training worlds.
The 1.08 m door world is hard for both. Episodes start/goal 2-10 m geodesic; collision = scan point inside footprint + 2 cm.
