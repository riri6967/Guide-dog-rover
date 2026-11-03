# Nav2 baseline, frozen 2026-10-10

Assistive rover, ROS 2 Jazzy + Gazebo Sim, assistive_world (corridor + Room 1 + Room 2).
Stack: SmacPlanner2D (global) + MPPI (local) + AMCL + velocity smoother + Collision Monitor.
This folder is a copy of the exact configuration that produced the numbers below. Do not edit it; change the live files and freeze a new folder instead.

## Contents
- params/nav2_params.yaml: footprint 0.90 x 0.76 m (+-0.45, +-0.38), inflation 0.65 / cost_scaling 4.0, obstacle_max_range 3.5 m (raytrace 12 m) in both costmaps, AMCL initial pose (0,0,0)
- launch/navigation.launch.py, launch/bridge.yaml, launch/assistive_rover.rviz
- model/model.sdf, model.config (wheel_separation 1.46 effective for skid-steer)
- world/assistive_world.sdf (corridor_top_right pose x 5.3, size 9.4: 1.5 m gap at x -0.9..0.6, y 1.5)
- map/assistive_world.{pgm,yaml} generated from the world by map/generate_map.py (0.05 m, origin -13,-8)
- goals/goals.csv (6 goals), auto_nav2_eval_v2.py (evaluator used)
- results/nav2_eval_v2_20261010_002625.csv (run A, 1 pass) and nav2_eval_v2_20261010_004313.csv (runs B1, B2, 2 passes)

## How the runs were made
Fresh Gazebo start (rover at 0,0 facing +x), bridge, Nav2, all with ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST and GZ_IP=127.0.0.1.
Each pass runs the 6 goals back-to-back (each goal starts where the previous one ended); the rover was driven back to about (0,0) before each evaluator call.
Success = Nav2 reports SUCCEEDED AND Gazebo ground-truth distance to goal <= 0.25 m. Timeout 150 s per goal.
No Nav2 critical failures, no Gazebo network errors, no Wi-Fi drops during these runs.

## Results (3 passes, 18 goal attempts)

| goal | A | B1 | B2 | success |
|---|---|---|---|---|
| candidate_1 (-5, 4) | TIMEOUT 0.22 m, 8 rec | ok 0.21 m, 0 rec | TIMEOUT 0.15 m, 10 rec | 1/3 |
| candidate_2 (5, -4) | ok 0.13 m, 1 rec | ok 0.19 m, 0 rec | ok 0.09 m, 0 rec | 3/3 |
| candidate_3 (-3, 0) | ok 0.03 m, 0 rec | ok 0.22 m, 1 rec | ok 0.19 m, 0 rec | 3/3 |
| candidate_4 (3, 4) | ok 0.03 m, 0 rec | ok 0.10 m, 0 rec | ok 0.17 m, 0 rec | 3/3 |
| candidate_5 (5, 2) | TIMEOUT 0.34 m, 13 rec | TIMEOUT 0.23 m, 14 rec | TIMEOUT 0.28 m, 14 rec | 0/3 |
| candidate_6 (-5, 2.5) | TIMEOUT 3.01 m, 8 rec | ok 0.14 m, 4 rec | ok 0.19 m, 0 rec | 2/3 |
| per pass | 3/6 | 5/6 | 4/6 | |

- **Overall: 12/18 = 67%, Wilson 95% interval [44%, 84%].**
- Per pass 3/6 to 5/6: a single pass of 6 goals is not enough to compare configurations.
- Successful trials: mean time 88.8 s, mean real distance 0.14 m (max 0.22 m), mean path efficiency 0.83, mean 0.5 recoveries.
- Without candidate_5 (see LIMITATIONS.md): 12/15 = 80%, Wilson 95% [55%, 93%]. Reported for information only; the headline number includes it.
- Minimum LiDAR range in any trial: 0.55 m. Contact sensor reported no non-ground contacts, but it is unvalidated (no collision claim is made).

See LIMITATIONS.md before quoting any of these numbers.
