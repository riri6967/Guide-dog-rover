# Assistive indoor rover: Nav2 baseline vs DRL local navigation (and a DRL + Nav2 hybrid)

DRAFT 2026-10-11. All numbers are measured in simulation (ROS 2 Jazzy + Gazebo Sim 8) on one laptop (RTX 4050).
Success is always judged by Gazebo ground truth, never by Nav2's own status alone. Intervals are Wilson 95%.


## 1. Goal
An assistive rover (0.90 x 0.76 m, skid-steer, 360-degree LiDAR) must navigate a home: corridor, Room 1, Room 2.
Plan: freeze a defensible Nav2 baseline, train a deep-reinforcement-learning (DRL) local controller that replaces only
Nav2's local controller (MPPI), and compare both on the same start/goal pairs with the same evaluator.

## 2. System
- Nav2: SmacPlanner2D (global path), MPPI (local controller), AMCL (localization), velocity smoother, Collision Monitor.
- DRL controller: SAC policy (2 x 256 MLP). Observation: 4 stacked LiDAR scans (90 bins each), a subgoal 1.5 m ahead on the
  Nav2 path (distance, bearing), measured velocity, previous action. Action: linear 0..0.3 m/s, angular -1..1 rad/s.
- Deployment: the DRL node serves Nav2's FollowPath action, so planner, behaviour tree, recoveries, smoother, Collision
  Monitor and evaluator are identical for every controller. Training pose = ground truth; deployment pose = AMCL.

## 3. Nav2 baseline (frozen 2026-10-10, baseline_20261010/)
- Fixes needed before the baseline was defensible: odometry yaw (wheel separation), AMCL instead of a fixed map->odom,
  real footprint (0.76 m, not 0.60 m), a 0.9 m wall gap widened to 1.5 m, obstacle range 10 -> 3.5 m (phantom obstacles
  1209 -> 10 per trip), localhost-only networking after a Wi-Fi drop crashed a run.
- Frozen result: 12/18 = 67% [44, 84] over 3 passes of 6 goals. candidate_5 (5, 2) fails always: the goal is 0.40 m from
  a wall, about 2 cm side clearance for the real rover.
- The final comparison uses the current world (one visual-only pillar removed at the user's request) and a Collision
  Monitor stop zone that allows reversing; Nav2 was re-run under exactly those conditions (section 6).

## 4. DRL training in generated worlds
- Gazebo only, lockstep (4 ms physics), about 170-200 environment steps per second (6 training instances + 2 spares for world rotation + 2 validation).
- World generator: 260 worlds (train 200 / val 30 / test 30) in 4 tiers: open, sparse obstacles, corridor buildings
  with doors 0.9-2.0 m, clutter. Map and Gazebo world come from the same primitive list. The house is never trained on.
- Curriculum SAC, 150 min per run, one seed per run. Test = 30 unseen worlds x 10 episodes.

| Run | One change vs previous | Val | Test (tiers 0/1/2/3) | Test collisions |
|---|---|---|---|---|
| 1 | baseline reward | n/a | 79% (98/98/65/60) | 60 |
| 2 | proximity penalty from the footprint edge | 90% | 85% (100/95/78/71) | 42 |
| 3 | Collision Monitor stop zone counted as collision | 91% | 85% (100/98/79/69) | 43 |
| 4 | policy may reverse (v_min -0.1) | 84% | 78% (97/91/70/58) | 64 |
| scripted controller (reference) | | | 90% (100/98/90/75) | 27 |

Run 3 is the best DRL model. Reverse (run 4) made it worse and was dropped.

## 5. Bugs found in the deployment chain (worth reporting)
1. Nav2's behaviour tree waited only 200 ms for the Python controller to accept each path (re-sent about once per second);
   measured acceptance latency p99 152 ms, max 224 ms, so long navigations failed. Raised to 1000 ms (DRL launch only).
2. The DRL node's "no progress" timer restarted on every path update, so Nav2 never ran recoveries. Fixed to work like
   Nav2's controller (timer kept per navigation).
Both affected every DRL house result before 2026-10-11; those results are marked invalid and not used below.

## 6. House comparison (same 6 goals, protocol: run A 1 pass, back to start, run B 2 passes = 18 trials)

| Controller | Successes | Mean time (successes) | DRL share of distance |
|---|---|---|---|
| Nav2 MPPI | 14/18 = 78% [55, 91] | 84 s | 0% |
| Pure DRL (run 3) | 4/18 = 22% [9, 45] | 74 s | 100% |
| Hybrid DRL + MPPI + escape | 13/18 = 72% [49, 88] | 73 s | 25% |
| Nav2 MPPI, repeat run | 12/18 = 67% [44, 84] | 75 s (run B) | 0% |
| Hybrid, repeat run | 12/18 = 67% [44, 84] | 77 s (run B) | 25% |
| **Nav2, 2 runs combined** | **26/36 = 72% [56, 84]** | 77 s | 0% |
| **Hybrid, 2 runs combined** | **25/36 = 69% [53, 82]** | 74 s | 25% |
| Hybrid, narrow threshold 0.7 m (more DRL) | 10/18 = 56% [34, 75] | 83 s | 51% |

Per goal over 36 trials, hybrid vs Nav2: c1 5/6 vs 6/6, c2 5/6 vs 4/6, c3 6/6 vs 6/6, c4 6/6 vs 6/6, c5 0/6 vs 0/6,
c6 3/6 vs 4/6. (An earlier hybrid repeat lost its run B to an evaluator problem; it was rerun in full and is not counted.)

- Pure DRL is fast and accurate on c1-c4 (run A 4/4), then drives into the wall corner near c5. There the Collision
  Monitor stops forward motion and turning, Nav2's backup is refused by the costmap, and every later goal fails.
- Pure DRL does NOT beat Nav2 on the house.

## 7. Hybrid DRL + Nav2 controller (training/deploy/hybrid_controller.py)
- DRL drives by default. MPPI takes over when the next 2 m of the planned path pass a narrow region (map clearance
  < 0.9 m: doors, gaps, near-wall goals) or an obstacle comes within 0.25 m of the footprint; DRL takes back over when
  the path ahead is open again. The final approach stays with DRL (it stops more accurately than a cold-started MPPI).
- Escape: before reporting "no progress", back straight out 0.30 m if the LiDAR shows free space behind, then turn toward
  the path. A scripted recovery, not learned.
- Development, each step measured on the same protocol: scan-only handover (too late; smoke test) -> map look-ahead with
  MPPI final approach (1/6, partial) -> DRL final approach (5/18) -> + escape (13/18).
- Per goal, hybrid vs Nav2: c1 3/3 vs 3/3 (43 s vs 67 s), c2 3/3 vs 2/3, c3 3/3 vs 3/3, c4 3/3 vs 3/3, c5 0/3 vs 0/3,
  c6 1/3 vs 3/3.

## 8. Conclusions (draft)
- A DRL policy trained only on generated worlds reaches 85% on unseen generated worlds and drives the house's open parts
  well, but alone it fails on the house (22%) because of tight spots it does not handle.
- Combining DRL with MPPI for tight spots and a simple escape gives a controller level with Nav2: 25/36 = 69% vs
  26/36 = 72% over two complete runs each (intervals almost identical), similar speed (74 s vs 77 s), with DRL driving
  about a quarter of the distance. Giving DRL half the distance (threshold 0.7 m) drops success to 56%: a measured share/success trade-off.
- The near-wall goal c5 is not solved by any controller.

## 9. Limitations
- One house, 6 goals, 36 back-to-back trials per controller (two runs) (not independent: a failure changes the next start pose).
- One training seed per model; the plan's 3 seeds were not run.
- Nav2 was not run on the generated worlds, so the DRL vs Nav2 comparison rests on the house only.
- The hybrid's DRL share is 25%; MPPI drives the hard parts. The escape is scripted.
- Collisions are not measured by a validated sensor (/rover_contacts is unvalidated); no collision-free claim.
- Simulation only; training pose is ground truth, deployment pose is AMCL.

## 10. Reproduce
- Nav2: HEADLESS=1 NAV2_OVERRIDES=cm_reverse_override.yaml MODE=nav2 training/deploy/compare_protocol.sh TAG
- Hybrid: same plus MODE=drl ACTOR=training/deploy/actor_run3.npz DRL_NODE=training/deploy/hybrid_controller.py
  DRL_ARGS="--g-in 0"
- Training: training/gz_env/train_sac_gen.py --minutes 150; evaluation: training/gz_env/eval_gen.py MODEL WORLDS_DIR
