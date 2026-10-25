# Simulation: Nav2 baseline and DRL local navigation

`assistive_robot/` is a ROS 2 Jazzy + Gazebo Sim package by **pushkar**, handed
over on 2026-10-12. It compares Nav2 with a deep-reinforcement-learning (SAC)
local controller, and with hybrids of the two, in an indoor house world. Every
result is checked against Gazebo's true robot pose, not the robot's own estimate.

Start with `assistive_robot/HANDOFF.md`, then `results/FINAL_REPORT_DRAFT.md`.
The full log of what was tried and why is `assistive_robot/CLAUDE.md`.

## Headline results (house world, same 6 goals)

| Controller | Success | 95% interval |
|---|---|---|
| Nav2 (SmacPlanner2D + MPPI) | 26/36 = 72% | 56–84% |
| Hybrid: DRL drives, MPPI in narrow spots | 25/36 = 69% | 53–82% |
| DRL alone | 4/18 = 22% | 9–45% |

The hybrid ties Nav2; it does not beat it. Goal c5 sits 0.40 m from a wall and
failed for every controller.

## Read before using this on the real rover

**The simulated robot is not this rover.** It is about three times larger, so the
trained policies and the Nav2 tuning do not transfer as they are.

| | Simulated (`models/assistive_rover`) | Real rover (`ros/rover_description`) |
|---|---|---|
| Footprint | 0.90 × 0.76 m | 0.27 × 0.26 m |
| Wheel radius | 0.12 m | 0.0325 m |
| LiDAR height above floor | 0.96 m | 0.21 m |
| Camera | RGB-D | Pi Camera v3 (no depth) |

- **Obstacle heights.** Training obstacles are 1.2–1.8 m tall so a LiDAR at 0.96 m
  can see them. The real LiDAR at 0.21 m sees low obstacles too, and misses table
  tops above it.
- **The policy expects the big footprint.** Collision and the safety-stop zone are
  defined from it, so it learned clearances for a 0.76 m-wide robot.
- **Nav2 parameters.** Footprints, inflation and the collision-monitor polygons
  are sized for 0.90 × 0.76 m. Speeds (0.3 m/s, 1 rad/s) suit guiding a person.

What carries over: the evaluation method (ground-truth success, Wilson
intervals, one change per run), the world generator, the lockstep Gazebo
training environment, and the overall Nav2 / DRL / hybrid architecture. To use
any of it here, rebuild the Gazebo model from `ros/rover_description` and
retrain.

## Not included

- **Confidence hybrid.** The results PDF of 2026-10-13 reports a "confidence
  hybrid" at 5/6 (preliminary) and an uncertainty study
  (`training/runs/unc_study_run3_val/`). Neither is in this handoff, which predates
  them.
- **Removed from the handoff to keep the repo small:** replay buffers, file
  backups, empty worker logs, and the trained models of runs other than the best
  one (run 3, `training/runs/sac_gen_20261010_234158/model.zip`). The deployable
  numpy actors for runs 2–4 are in `training/deploy/`.
- **Paths.** Scripts use absolute paths under `/home/pushkar/assistive_robot_ws`.
  Adjust them for another machine.
