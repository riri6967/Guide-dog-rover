# Assistive Rover: Nav2 baseline, then DRL local navigation (2D fast sim + Gazebo)

## Project
Assistive indoor navigation rover (corridor + Room 1 + Room 2). ROS 2 Jazzy + Gazebo Sim, Ubuntu 22.04, RTX 4050 Laptop (6 GB).
Goal: freeze a defensible Nav2 baseline, then train a DRL local policy (replaces MPPI only) in a fast 2D sim + Gazebo and compare both on the SAME start/goal pairs and metrics. Full plan and checkboxes: PLAN.md (follow its STOP lines).
I do NOT want to debug Nav2 forever: fix only what blocks a defensible baseline, document the rest.

## Paths (workspace ~/assistive_robot_ws, package src/assistive_robot)
- worlds/assistive_world.sdf, models/assistive_rover/model.sdf, config/bridge.yaml, assistive_rover.rviz
- nav2/params/nav2_params.yaml, nav2/launch/navigation.launch.py, nav2/goals.csv
- nav2/maps/assistive_world.{pgm,yaml} and generate_map.py (SDF and map must always change together)
- scripts/auto_nav2_eval_v2.py (current evaluator), scripts/*v9* (old SAC work, archived)
- results/ (CSV outputs), helper scripts in package root: gt_check_map.py, track.py, show_path.py, dump_at.py, dump_patch.py, scan_vs_map.py, spin_test.sh, drive_test.sh

## Startup (one terminal each; fresh Gazebo start puts rover at 0,0 facing +x, which AMCL assumes)
0. In EVERY terminal (Gazebo, bridge, Nav2, RViz, evaluator, any ros2/gz command): source /opt/ros/jazzy/setup.bash && export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1 ; then ros2 daemon stop once so the CLI daemon picks it up.
   Why: on 2026-10-10 a Wi-Fi drop (00:05:28-00:08:58) stalled DDS + gz transport on this same machine; Nav2 lifecycle manager killed the stack (collision_monitor heartbeat), the rover drove into bed_room1 with the safety chain down, and the eval run was invalid (results/nav2_eval_v2_20261010_000519_INVALID_wifi_drop.csv).
1. cd ~/assistive_robot_ws && source /opt/ros/jazzy/setup.bash && export GZ_SIM_RESOURCE_PATH="$HOME/assistive_robot_ws/src/assistive_robot/models:${GZ_SIM_RESOURCE_PATH:-}" && gz sim -r src/assistive_robot/worlds/assistive_world.sdf
2. ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$HOME/assistive_robot_ws/src/assistive_robot/config/bridge.yaml"
3. cd ~/assistive_robot_ws/src/assistive_robot && ros2 launch nav2/launch/navigation.launch.py
4. optional: rviz2 -d ~/assistive_robot_ws/src/assistive_robot/assistive_rover.rviz
Wait ~30 s, check: ros2 lifecycle get /amcl and /planner_server both "active [3]".
Gazebo reads model/world files only at startup: restart it after any SDF edit. Restart Nav2 after params/map edits.

## Stack
Nav2: SmacPlanner2D (GridBased), MPPI (FollowPath), AMCL (replaced the old static map->odom), Collision Monitor, velocity smoother.
Velocity chain: /cmd_vel_nav -> /cmd_vel_smoothed -> /cmd_vel -> ros_gz_bridge. Topics: /scan, /odom, /map, /rover_contacts.
LiDAR frame assistive_rover/base_link/lidar, static transform z=0.68 from base_link.

## Hard rules
- Use /usr/bin/python3 (system 3.12) for any rclpy script. Default python3 is pyenv 3.10 and fails. Do not touch pyenv or reinstall ROS.
- Back up a file before editing it (cp file file.backup_$(date +%Y%m%d_%H%M%S)). One change at a time, with evidence first.
- Judge every navigation result against Gazebo ground truth (gz topic -e -n 1 -t /world/assistive_world/pose/info), never Nav2's own pose or status alone. Early "successes" were false: odometry had drifted 0.5-10 m.
- Do not quote old V9/SAC evaluation numbers (e.g. "83% collision"); they came from a buggy reset. The 50k V9 model predates the fix.
- Contact data (/rover_contacts) is NOT validated as a collision count. Never claim "collision-free" from it.
- Don't teleport/reset the rover without handling odom/TF/AMCL consistency. Current evaluator runs goals back-to-back and logs each start pose instead.
- Ask before deleting files or running anything destructive.

## What was fixed / verified
- Odometry yaw was ~2x real (skid-steer, 4 wheels). wheel_separation changed 0.68 -> 1.33 -> 1.46 in model.sdf. Spin test ratio odom/gazebo ~1.06-1.09, straight drive ratio 1.05. Good enough; don't tune further.
- Fixed identity map->odom caused scan projection errors (diagonal phantom obstacle band at origin). Replaced with AMCL (nav2_params.yaml amcl block, max_beams 360).
- LiDAR mount and map verified: 100% of scan hits land on map walls at the real pose.
- AMCL at rest is about 0.08 m and about 6 deg off Gazebo (time-aligned probe, 2026-10-09; driving: mean 4.2 deg, max 29 deg in fast turns). The earlier ~0.03 m / 1 deg was measured right after one trip. (An earlier "5 m error" was a checker artifact: it read raw odom before AMCL's transform arrived. gt_check_map.py now waits.)

## FROZEN BASELINE: baseline_20261010/ (README.md = numbers, LIMITATIONS.md = caveats; awaiting user review, PLAN.md Phase 0 last STOP)
12/18 = 67% real successes, Wilson 95% [44%, 84%] over 3 back-to-back passes (3/6, 5/6, 4/6). candidate_2/3/4 3/3, candidate_6 2/3, candidate_1 1/3, candidate_5 0/3 (goal 0.40 m from wall, ~2 cm clearance for the 0.76 m rover).
Quote the frozen numbers, not the single-pass history below.

## Pre-freeze history (evaluator v2, one pass each, 6 goals back-to-back, NOT independent trials)
results/nav2_eval_v2_20261009_161509.csv: 4/6 real successes (success = Nav2 SUCCEEDED and real distance <= 0.25 m).
candidate_1 (-5,4) ok 5 recoveries; candidate_2 (5,-4) ok (0.24 m, borderline); candidate_3 (-3,0) ok; candidate_6 (-5,2.5) ok 10 recoveries;
candidate_4 (3,4) and candidate_5 (5,2) TIMEOUT, both stuck at about (-0.5, 0.8). Planner finds valid paths to both.

## Current diagnosis and OPEN TASK
Both failures stall at a gap in the top corridor wall at x in [-1, 0], y=1.5. Real clear opening was 0.9 m, not 1.0: room1_right_wall (x=-1, 0.2 thick, y 1.5..8) puts a face at x=-0.9; corridor_top_right started at x=0. Map showed free x -0.85..-0.05.
Real rover is ~0.76 m wide (wheels at y=+-0.34, 0.08 m wide; body 0.90x0.60), but the nav2_params.yaml footprint was +-0.30 (0.60 m) in both costmaps. Inflation 0.65, cost_scaling 4.0 (unchanged).
Fix APPLIED and tested 2026-10-09 (backups *.backup_20261009_230101). results/nav2_eval_v2_20261009_230625.csv: 4/6; candidate_4/5 now reach the goal (5 misses by 0.01 m: 0.26 m real, AMCL error), candidate_6 TIMEOUT:
1. corridor_top_right pose x 5 -> 5.3, size 10 -> 9.4 (collision and visual): wall now spans x 0.6..10, clear opening 1.5 m (-0.9..0.6). Checked: no other model in x 0..0.8, y 1.2..1.8.
2. Map regenerated with /usr/bin/python3 nav2/maps/generate_map.py (do NOT hand-edit the PGM; generator output matched the old PGM exactly before the edit). Diff: 60 cells occupied->free, x 0.00..0.55, y 1.40..1.60; yaml unchanged.
3. Both footprints now [[0.45,0.38],[0.45,-0.38],[-0.45,-0.38],[-0.45,0.38]] (lines 351, 411); comment fixed to 0.76 m.
4. Restarted and evaluated (see line above).
Phantom obstacles in the global costmap (found 2026-10-09; phantom = free in /map but lethal in costmap):
- 2820 phantom cells after the 6-goal run. After clearing, one trip (3,4)->(-5,2.5) rebuilt 114 -> 2769: 681 near walls (<=0.10 m), 879 at 0.15-0.30 m, 1209 on open floor (>0.30 m), 1385 behind walls (no line of sight from any true pose).
- New phantoms ~37/sample at AMCL yaw error < 5 deg, ~4x that at > 10 deg (fast turns). Hypothesis: obstacle_max_range 10 m turns a few degrees of yaw error into 0.3-0.8 m misplaced hits.
- APPLIED and PASSED: obstacle_max_range 10.0 -> 3.5 in both costmaps (raytrace_max_range stays 12; backup nav2_params.yaml.backup_20261009_235701). Same probe (3,4)->(-5,2.5), 10 m vs 3.5 m: open-floor phantoms 1209 -> 10, behind walls 1385 -> 58, new per driving sample 42.9 -> 8.0, total 2769 -> 207; candidate_6 SUCCEEDED 0.07 m, 0 recoveries (one run each; the 3.5 m run had no recovery spins). AMCL error itself unchanged.
- Evaluator with 3.5 m: first attempt invalid (Wi-Fi drop, see Startup step 0). Rerun (localhost-only, no network errors) results/nav2_eval_v2_20261010_002625.csv: 3/6 (vs 4/6 at 23:06). c2/c3/c4 ok (c3, c4 now 0.03 m, 0 recoveries); c1 TIMEOUT 0.22 m real, 8 rec; c5 TIMEOUT 0.34 m, 13 rec; c6 TIMEOUT 3.0 m away (still in corridor at (-2.62,0.66) after 8 rec). n=6 back-to-back: 3/6 vs 4/6 is within noise; no evidence the range change improved or hurt success.
- Open: final-approach stalls (c1, c5, c6 earlier) stop 0.2-0.35 m short; "Failed to make progress" every ~10 s matches progress_checker (0.20 m per 10 s) firing near the goal. Hypothesis only, not tested.
candidate_6 (-5,2.5) is flaky: TIMEOUT in the eval run, SUCCEEDED (0.10 m, 5 recoveries) in the probe run. Both times it stalled at about (-4.93, 2.38) real, 0.14 m from the goal, with "Failed to make progress" (AMCL ~0.08 m off vs xy_goal_tolerance 0.15); recovery spins (yaw error up to 29 deg) dropped phantom marks on the goal.
Then freeze the baseline: dated folder with params, launch, model, world, map, goals, CSV, and a limitations note.

## Known loose ends (low priority)
- Peak AMCL error 0.45 m during one trip (cause unknown, short transient or sustained?).
- Collision Monitor stop polygon half-width 0.33 is narrower than the real rover (0.38).
- Room 1 door goals needed 5-10 recoveries; door may also be tight.
- Nav2 heading lags real heading ~20 deg during turns (odom over-reports rotation ~10%), position stays fine.
- Contacts unvalidated (see rules).

## DRL phase (after baseline freeze; details in PLAN.md Phases 1-8)
Phase 1 measured (training/phase1/RESULTS.md): headless Gazebo, 4 ms physics step, LOCKSTEP via gz-transport directly (not the ROS bridge):
~110-120 env steps/s per instance (~400k/h), 3 instances ~1M/h. Free-running fast sim gives late actions/stale poses: always lockstep for training.
gz Python Node.request() holds the GIL: issue requests from a helper process without subscriptions (pattern in gz_lockstep_bench.py).
Gate DECIDED 2026-10-10: Gazebo-only first (no 2D sim for now); GPU used for the network. Next: short SAC trial to measure throughput with a learner.
Python split: gz bindings only in /usr/bin/python3 (3.12); torch 2.14 (CUDA ok), SB3 2.9, gymnasium 1.3 only in pyenv python3 (3.10). Env workers run under 3.12, learner under 3.10, connected by a socket. Do not install torch into system python; do not touch pyenv.
Gazebo RL stack (training/): worlds_gen/gen_trial_worlds.py (SDF + grid from the same SDF), gz_env/gz_nav_env.py (lockstep env, GT state,
footprint collision, geodesic progress, centred subgoal), worker.py + vec_env.py (3.12 <-> 3.10 bridge), train_sac_trial.py, eval_policy.py.
Scripts that create GzNavEnv need an if __name__ == '__main__' guard (helper process uses spawn). Never pkill -f a pattern that appears in your own command.
SAC trial 2026-10-10 (training/runs/sac_trial_20261010_021952/RESULTS.md): 3 trial worlds, 30 min, 110k steps: train 82%, held-out 56/90 = 62%
(scripted controller 82% on the same held-out worlds). With learner ~60 env steps/s (learner-bound, GPU ~20%).
Trial-world caveat: boxes in training/worlds/trial* are 0.5-1.0 m tall, below the LiDAR (0.96 m above floor): mostly invisible. Trial numbers above are affected.
Phase 3 generated worlds (training/worlds_gen/gen_worlds.py -> training/worlds/gen/{train,val,test} 200/30/30, tiers 0 open, 1 sparse, 2 corridor building + doors 0.9-2.0 m, 3 clutter; obstacles 1.2-1.8 m tall; SDF + exact grid from one primitive list).
Curriculum run 2026-10-10 (training/runs/sac_gen_20261010_113224/RESULTS.md): train_sac_gen.py, 150 min, 1.53M steps (~170 steps/s, background-thread SAC, UTD 1/3, batch 1024, world rotation with spare workers).
TEST (10 eps/world, deterministic): SAC 237/300 = 79% (tiers 98/98/65/60%), scripted 270/300 = 90% (100/98/90/75%). Failures are collisions in buildings. Not compared with Nav2 on these worlds.
Run 2 2026-10-10 (training/runs/sac_gen_20261010_142340/RESULTS.md): ONE change, proximity penalty from the footprint edge (CFG prox_from='footprint', r_prox -0.2 below 0.25 m clearance); run-1 collisions were 35/39 side hits on door jambs/wall ends.
TEST: run 2 255/300 = 85% (tiers 100/95/78/71%), 42 collisions (run 1: 79%, 60). Scripted 90%. Val 90% (used for choice). One seed each.
DRL deployment (training/deploy/): export_actor.py (numpy actor), drl_controller.py (serves follow_path; AMCL pose via TF, subgoal 1.5 m along the Nav2 path, tol 0.15, progress check 0.2 m/10 s), run_drl_stack.sh / stop_drl_stack.sh, nav2/launch/navigation_drl.launch.py (MPPI action renamed follow_path_mppi_unused; drl_overrides.yaml: bt_navigator default_server_timeout 200 ms because rclpy acks take 17-50 ms).
assistive_world has 4 VISUAL-ONLY doorframe posts (0.12 m, 2.4 m tall) at (-6,1.5), (-4,1.5), (4,-1.5), (6,-1.5): LiDAR sees them, physics does not; (4,-1.5) stands inside the 3 m Room 2 opening (x 3..6). Not in the Nav2 map (generate_map uses collisions).
First DRL Nav2 run 2026-10-10 (results/drl_eval_run2_20261010_214914_passA_trapped_at_doorpost.csv): pass A c1, c2 OK (49 s, 132 s), then leaving Room 2 the rover stopped ~3 cm from the (4,-1.5) post, Collision Monitor stop zone held it, the policy kept commanding forward: trapped, remaining 4 goals failed. Run stopped: 2/6, not a full 18-trial comparison.
WORLD CHANGE 2026-10-10 (user request): removed visual-only pillar doorframe_room2_left (4,-1.5) from worlds/assistive_world.sdf (backup *.backup_20261010_*). Nav2 map grid unchanged (visual-only; verified identical). The frozen baseline (12/18) was measured WITH it: rerun Nav2 on the current world before quoting a DRL vs Nav2 comparison. 3 posts remain (both Room 1 posts, Room 2 post at (6,-1.5)), all at wall ends.
COMPARISON 2026-10-10 on the no-pillar world (results/COMPARISON_DRL_vs_NAV2_20261010.md): Nav2 13/18 = 72% (49-88%), DRL run 2 4/18 = 22% (9-45%). DRL run A 4/6 (faster: 57 vs 76 s) then trapped facing the wall after c5 inside the Collision Monitor stop polygon (x 0..0.55, |y| <= 0.33): stop zeroes all motion incl. BT backup, policy has v >= 0 only. DRL does NOT beat Nav2.
Run 3 2026-10-11 (training/runs/sac_gen_20261010_234158/RESULTS.md): stop_zone=(0,0.55,0.33,4) as collision in training. Gen test 85% (same as run 2), val 91%. House via Nav2: 4/18 again: c1-c4 OK (mean 50 s), then trapped facing the wall near c5 (0.40 m from wall) inside the stop polygon. Any controller inside PolygonStop is stuck for good (stop zeroes turns, reverse, BT backup).
OVERNIGHT 2026-10-11 (results/OVERNIGHT_SUMMARY_20261011.md): drl_controller.py had TWO bugs affecting every earlier DRL house run (incl. the 4/18 above): (1) BT ack timeout: rclpy ack p99 152 ms, max 224 ms vs 200 ms limit -> drl_overrides.yaml now 1000 ms; (2) progress checker reset on every ~1 Hz path update, so the BT never ran recoveries -> progress state now per navigation. Invalid runs renamed *_INVALID_ack_timeout / *_INVALID_progress_reset.
Valid house numbers, both with NAV2_OVERRIDES=cm_reverse_override.yaml (stop box replaced by a velocity polygon: front box stops forward/turns, rear box stops reverse) via nav2/launch/navigation_variant.launch.py: Nav2 MPPI 14/18 = 78% (55-91%, c5 0/3); DRL run 3 4/18 = 22% (9-45%), wedged near c5 (spin stopped by collision monitor, backup "Collision Ahead").
Run 4 (training/runs/sac_gen_20261011_055331, ONE change GZNAV_CFG {"v_min": -0.1}, deploy DRL_ARGS="--v-min -0.1"): WORSE: test 78% (run 3 85%), 64 collisions; house 2/18, corner pivoted onto the Room 2 wall end (3.62,-1.49). Run 3 stays the best DRL model. Next step awaits user decision (options in the overnight summary).
run_drl_stack.sh: DRL_ARGS env passes controller options; stop_drl_stack.sh now SIGKILLs groups that survive TERM (gz sim hung once). Use absolute paths in background commands (cwd may be results/).
HYBRID 2026-10-11 (results/HYBRID_DRL_NAV2_20261011.md, user's faculty requires DRL): training/deploy/hybrid_controller.py = DRL run 3 drives by default, MPPI (follow_path_mppi_unused) in narrow regions (map look-ahead 2 m, clearance < 0.9 m) or scan clearance < 0.25 m, + scan-based escape (reverse 0.3 m if rear clear, turn to path). Run with DRL_NODE=training/deploy/hybrid_controller.py DRL_ARGS="--g-in 0". House: 13/18 = 72% (49-88%), mean 73 s, DRL 25% of distance; Nav2 14/18 (84 s). Failures c5 0/3 (Nav2 too), c6 1/3. MPPI final approach (--g-in 1) and no escape were worse (1/6 partial, 5/18). Nav2 velocity smoother and MPPI DO allow reverse (-0.10).
REPEATS 2026-10-11 afternoon (results/AFTERNOON_SUMMARY_20261011.md): Nav2 combined 26/36 = 72% (56-84); hybrid combined 17/24 = 71% (51-85; repeat run B lost, evaluator output now saved to results/<TAG>_<A|B>_evaluator_raw.log); hybrid --map-c 0.7: DRL share 51%, success 10/18 = 56%. Draft report: results/FINAL_REPORT_DRAFT.md.
FINAL 36 vs 36 (2026-10-11 22:00): hybrid 25/36 = 69% (53-82), 74 s, DRL 25%; Nav2 26/36 = 72% (56-84), 77 s. Runs: hybrid_run3_esc + hybrid_run3_esc_r3; nav2_cmrev + nav2_cmrev_r2.
Known env issue: one Gazebo instance occasionally gets stuck (rover does not move); rerun of the world is fine. No guard yet.
No Isaac Lab (the Isaac plan is dropped). Training stays in simulation on this machine: Stage A fast 2D Gymnasium sim (bulk of training), Stage B Gazebo fine-tune/check. Phase 1 measures Gazebo steps/hour first; that gate decides how much Stage A is needed.
Deployment: SmacPlanner2D path -> subgoal ~1.5 m ahead -> policy (v, w) -> velocity smoother -> Collision Monitor -> /cmd_vel. Training pose = Gazebo ground truth; deployment pose = AMCL (test the swap).
Generalization: generated worlds (one layout -> 2D map + SDF), train/val/test splits; assistive_world is an extra held-out "home" test.
Reuse: rover geometry (0.90x0.76 m, wheel radius 0.12, effective wheel_separation ~1.46 for skid-steer, under-rotates ~10%), LiDAR spec (360 beams, 360 deg, range 0.12-15 m), Nav2 limits (v 0..0.3 m/s, w +-1.0 rad/s), observation idea from V9 (LiDAR history, subgoal, velocity, previous action), the same 6 goals and metrics.
Collision for training = scan point inside footprint + 2 cm, not /rover_contacts.
Do not claim DRL beats Nav2 until measured with the same start/goal pairs and evaluator logic.
