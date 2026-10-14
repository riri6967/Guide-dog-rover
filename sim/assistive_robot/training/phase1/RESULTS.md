# Phase 1: Gazebo training speed (measured 2026-10-10)

Machine: Ryzen 7 7435HS (16 threads), 23 GB RAM, RTX 4050 Laptop 6 GB. Gazebo Sim 8.15 (Jazzy), headless (`gz sim -s --headless-rendering`).
Training variants (originals untouched): `make_train_world.py` writes `training/worlds/train_s<step>_rtf<rtf>.sdf` (adds `<physics>`)
and `training/models/assistive_rover_train` (rover without the RGB-D camera). LiDAR on (gpu_lidar, 360 beams, 10 Hz).

## 1. Raw simulation speed (rtf_bench.py, constant arc v=0.12 w=0.5, 120 sim-s, 2 trials each, no wall contact)

| physics step | RTF (unthrottled) | measured v / w (m/s, rad/s) | w vs 1 ms |
|---|---|---|---|
| 1 ms (default) | 4.9 | 0.129 / 0.666 | ref |
| 4 ms | 18.3 | 0.127 / 0.676 | +1.5% |
| 8 ms | 34.2 | 0.131 / 0.696 | +4.5% |
| 16 ms | 55.3 | 0.128 / 0.714 | +7.2% |

- Speed is physics-bound (~4,700 physics steps/s); the RGB-D camera costs nothing when unsubscribed (4.69 vs 4.73).
- Trials were repeatable to the 3rd decimal. No tilt, LiDAR stays 10 Hz in sim time at every step size.
- **Chosen: 4 ms** (1.5% turn-rate difference). 8 ms also breaks lockstep at 10 Hz (0.1 s is not a whole number of 8 ms steps).
- Side finding for any 2D sim / calibration: commanded w=0.5 gives real w~0.67 (+33%), commanded (0.25, 0.4) gives (0.27, 0.57). The cmd -> real mapping must be measured, not assumed.

## 2. Env loop (reset + step + scan, random actions, 10 Hz control, episodes <= 200 steps or min scan < 0.5 m)

Free-running (ROS 2 bridge, sim not paused) is NOT usable above real time:
| step | steps/s | action latency in sim (median / p95) | problem |
|---|---|---|---|
| 1 ms | 51 | 15 / 28 ms | ok |
| 4 ms | 186 | 48 / 100 ms | actions up to a full period late; stale pose after some resets (11.7 m error) |
| 8 ms | 168 | 72 / 96 ms | misses every other scan (5 Hz control) |

Lockstep (sim paused; each step advances exactly 0.1 s; action latency 0, every step exactly 0.1 s sim):
| loop | step | steps/s (one instance) | steps/h | reset cost |
|---|---|---|---|---|
| ROS 2 bridge services (env_loop_bench.py --lockstep) | 4 ms | 29 | 105k | ~2 s (pose wait) |
| gz-transport direct (gz_lockstep_bench.py) | 1 ms | 54 | 194k | 40 ms |
| gz-transport direct | 4 ms | 109-121 | 393-434k | 20 ms |

Note: gz Python `Node.request()` holds the GIL while waiting; with subscriptions in the same process it deadlocks.
gz_lockstep_bench.py sends requests from a helper process (no subscriptions). Keep that pattern.

## 3. Parallel instances (gz-transport lockstep, 4 ms, own GZ_PARTITION each)

| instances | per instance steps/s | total steps/s | total steps/h |
|---|---|---|---|
| 1 | 109-121 | 109-121 | 393-434k |
| 2 | 93 | 186 | 669k |
| 3 | 91 | 274 | 985k |
| 4 | 80 | 322 | 1.16M |

Teleport (reset) error <= 0.06 m everywhere; 0 scan timeouts.

## Gate (PLAN.md section 3)
Threshold ~100k steps/h. Measured 393k/h with one instance, ~1M/h with 3, 1.16M/h with 4: far above.
Not measured yet: throughput with a learner running (SAC gradient updates compete for CPU/GPU), GPU memory per instance,
cost of switching between generated worlds (needs a server restart per world, ~5-10 s).
