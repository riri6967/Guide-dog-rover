# MASTER PLAN: Nav2 baseline -> DRL local navigation that generalizes (Gazebo only, no Isaac)
Tick a box only when its check passes. Stop at every STOP line and report measured numbers.

## 0. Target (definition of done)
1. A frozen Nav2 baseline with honest, documented numbers.
2. A DRL local policy running inside: global planner -> subgoal -> policy -> safety layer -> /cmd_vel.
3. Evaluation on HELD-OUT worlds never used in training, against Nav2-MPPI on the SAME start/goal pairs.
4. Proposed pass bars (adjust together): held-out standard worlds (doors >= 1.0 m): success >= 90%,
   collision episodes <= 5%. Harder tiers: report the numbers, no bar. Never claim "better than Nav2" unless measured.

Scope levels:
- L1 (this plan): local navigation that generalizes to unseen layouts, given a map + global path.
- L2 (stretch): unseen building -> build a map with SLAM Toolbox first, then L1.
- L3 (stretch): frontier exploration, then L1.

## 1. Architecture (deployment)
Global goal -> Nav2 SmacPlanner2D on the map -> subgoal ~1.5 m ahead on the path -> DRL policy
(LiDAR history + subgoal + velocity + previous action -> v, w) -> velocity smoother -> Collision Monitor -> /cmd_vel.
The policy replaces MPPI only. Localization is AMCL at deployment.

## 2. Phase 0: finish the baseline (needed as comparison and as validated plumbing)
- [x] Door fix per CLAUDE.md (gap 5.3/9.4, regenerate map, footprints +-0.38), backups first. (done 2026-10-09: 60-cell map diff verified, footprints confirmed live)
- [x] STOP: user restarts Gazebo + Nav2, checks costmap gap (dump_at.py -0.5 1.5 2.0). (gap open: cost 0 at x~-0.1; found phantom obstacles -> obstacle_max_range 10 -> 3.5, open-floor phantoms 1209 -> 10)
- [x] Run scripts/auto_nav2_eval_v2.py --repeats 1; compare candidate_4/5 with the old CSV (4/6). (gap fixed: candidate_4 3/3; then 3 passes = 12/18, see baseline_20261010/)
- [x] Freeze: baseline_YYYYMMDD/ (params, launch, model, world, map, goals, CSV) + LIMITATIONS.md
      (n=6 back-to-back, contacts unvalidated, recoveries, borderline passes).
- [x] STOP: user reviews. (approved 2026-10-10)

## 3. Phase 1: measure training speed (decision gate)
- [x] Run Gazebo headless: gz sim -s -r --headless-rendering <world>. (4 ms step: RTF 18, turn rate +1.5% vs 1 ms; see training/phase1/RESULTS.md) Make physics run faster than real time
      (real_time_factor / update rate in the world's <physics>) and record the highest stable RTF with the LiDAR on.
- [x] Measure env steps/sec through a ROS 2 env loop (reset + step + scan). (free-running unusable above RT; lockstep: ROS 29/s, gz-transport direct 109-121/s) steps/hour = control_hz * 3600 * RTF.
- [x] Test 2-3 parallel Gazebo instances: (2: 669k/h, 3: 985k/h, 4: 1.16M/h) separate GZ_PARTITION and ROS_DOMAIN_ID per instance. Record total steps/hour.
- DECISION 2026-10-10 (user): Gazebo-only first. Measured ~400k steps/h per instance, ~1M/h with 3 (lockstep, 4 ms).
  Stage A (section 4) is skipped for now; GPU trains the network. Revisit a batched GPU 2D sim only if a measured SAC trial
  shows Gazebo throughput is the real limit.
- GATE: if total Gazebo throughput is under ~100k steps/hour, do Stage A (section 4) for the bulk of training.
       If it is far higher, Stage A can be skipped or shortened. Report the numbers and decide with the user.
- Training-time state comes from Gazebo ground truth (pose from /world/<w>/pose/info); odom is not used for pose.
  This removes the old reset/odometry bug. Reset = set_pose + zero velocity + wait for a fresh scan.
  Deployment uses AMCL pose instead, so test that swap in Phase 7.

## 4. Phase 2: Stage A, fast 2D training simulator (recommended)
- [ ] Gymnasium env: occupancy-grid world, unicycle kinematics (v <= 0.3 m/s, w <= 1.0 rad/s, same as Nav2 limits),
      numpy/numba ray casting that matches the Gazebo LiDAR (360 beams, 0.12-15 m, 360 deg FOV).
- [ ] Randomize: scan noise and dropouts, action delay, wheel-slip factor (the real rover under-rotates ~10%).
- [ ] Footprint-aware collision: terminate if any scan point falls inside the rover footprint (0.90 x 0.76 m) + 2 cm.
      Do not rely on /rover_contacts (unvalidated).
- [ ] Check: thousands of steps/sec; a scripted controller reaches goals in simple maps (proves the env is sane).
- [ ] Validation: replay a few scripted trajectories in Gazebo and compare scans/poses with the 2D sim.

## 5. Phase 3: world generator (the core of generalization)
- [x] One layout description produces BOTH a 2D occupancy map and a Gazebo SDF (so one world works in both sims).
- [x] Parameters: building size, corridor width 1.2-3 m, number of rooms, L/T/dead-end corridors,
      door widths 0.9-2.0 m (most >= 1.0 m, a hard tier at 0.9-1.0), random furniture boxes, random start/goal.
- [x] Splits: ~200 train, ~30 validation, ~30 test worlds (suggested). Test worlds are never used for tuning.
      The current assistive_world is an extra held-out "home" test.
- [x] Curriculum: empty -> sparse obstacles -> rooms + doors -> clutter. Promote a stage when validation success >= 80%.
- [ ] Goals come from the planner path: subgoal = point ~1.5 m ahead along the path (as at deployment).
      (2026-10-10: subgoal currently from the geodesic distance field, not the Nav2 planner. Results: training/runs/sac_gen_20261010_113224/RESULTS.md)

## 6. Phase 4: environment spec (shared by both sims). Starting values; change ONE at a time.
- Observation: 360-beam scan min-pooled to 90 beams, clipped to 0.12-6 m, divided by 6; 4 stacked frames (360 values);
  subgoal in the robot frame (distance/6, sin, cos); measured v, w; previous action. About 367 dims.
  Network: small 1D CNN or plain MLP over the scan. Try MLP first.
- Action: v in [0, 0.3] m/s, w in [-1, 1] rad/s (same limits as Nav2, so the comparison is fair).
- Reward: + progress toward the subgoal; + goal bonus; - collision penalty (ends episode); - small step penalty;
  - penalty when the closest scan point is under 0.5 m; - action-change penalty (smoothness); stuck detection ends the episode.
- Termination: goal reached (<= 0.25 m), collision, timeout, stuck (no progress for N seconds).
- Log is_success in info so TensorBoard shows success rate, not just reward.

## 7. Phase 5: training (Stage A)
- [x] Algorithm: SAC (sample efficient, existing code). PPO only if many parallel 2D envs are vectorized.
- [ ] Settings to start: gamma 0.99 (short subgoal horizon), buffer 1M (check RAM), learning_starts 10k, seed fixed,
      EvalCallback on the validation worlds, checkpoints with replay buffer, resume support.
- [ ] Short trial runs (15-30 min) first to catch reward bugs; watch episodes (replay videos), not only curves.
- [ ] Final claims need 3 seeds. Keep a run log (config, result, notes).
- [ ] STOP: user reviews validation success per curriculum stage.

## 8. Phase 6: Stage B, Gazebo fine-tune and check
- [ ] Run the Stage A policy in Gazebo (ground-truth state), generated SDF worlds, random obstacles.
- [ ] Compare behavior with the 2D sim (turn lag, slip, scan differences). Fine-tune 100-300k steps if needed.
- [ ] Gate: success on Gazebo validation worlds within ~10 points of the 2D sim, or find the gap first.

## 9. Phase 7: deployment node
- [ ] ROS 2 node: subscribe /scan, odom, TF; get the path (compute_path_to_pose), compute the subgoal,
      run inference on CPU, publish cmd_vel_nav. Keep the velocity smoother and the Collision Monitor active.
- [ ] Switch pose from ground truth to AMCL and re-test. Fix the footprint-aware stop polygon (0.33 -> 0.38).

## 10. Phase 8: evaluation protocol (what you can show)
- [ ] Same start/goal pairs per world for Nav2-MPPI and for the policy; >= 30 pairs per tier.
- [ ] Success = Nav2/policy finished AND real distance to goal <= 0.25 m by Gazebo ground truth.
- [ ] Metrics: success, collision episodes, timeouts, time, path efficiency, smoothness (angular jerk), recoveries (Nav2).
- [ ] Report with Wilson 95% intervals; list failure cases with videos; also run the 6 original goals.
- [ ] Tiers: standard worlds, tight doors, clutter, the home world. Policy-only (no planner) is optional.

## 11. Phase 9: new areas (stretch)
- L2: teleop once with SLAM Toolbox in an unseen world, save the map, then run the full stack.
- L3: frontier exploration (Nav2 exploration packages) feeding goals to the stack.

## 12. Honest time estimates (full-time, ranges, depend on measured speed)
Phase 0: 1-2 days | Phase 1: 1-2 | Phases 2-3 (2D sim + generator): 4-7 | Phases 4-5 (debug + training): 4-8
Phase 6-7: 3-5 | Phase 8: 2-4. Total about 3-5 weeks. Training itself is hours to a few days; the rest is building and debugging.

## 13. Risks
- Reward hacking (spinning, creeping): watch rollouts, penalize oscillation, keep the progress term.
- Sim-to-sim gap (2D -> Gazebo): domain randomization + Stage B fine-tune + the Phase 6 gate.
- Overfitting to few worlds: enforce splits; never tune on test worlds.
- Narrow doors: the rover is 0.76 m wide; keep 0.9-1.0 m doors in a separate tier and report them separately.
- Scope creep: do not start L2/L3 until L1 passes on held-out worlds.
