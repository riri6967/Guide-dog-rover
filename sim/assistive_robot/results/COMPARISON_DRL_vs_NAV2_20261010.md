# DRL (run 2) vs Nav2 MPPI on assistive_world without the Room 2 pillar (2026-10-10)

Same world (pillar doorframe_room2_left removed at user request), same stack (planner, BT recoveries, AMCL, velocity smoother,
Collision Monitor), same evaluator (scripts/auto_nav2_eval_v2.py), same protocol as baseline_20261010: fresh start, run A
(1 pass of the 6 goals), NavigateToPose back to (0,0), run B (2 passes); success = SUCCEEDED and Gazebo GT distance <= 0.25 m.
Only difference: who serves follow_path (MPPI controller_server vs training/deploy/drl_controller.py, policy
training/runs/sac_gen_20261010_142340, AMCL pose). Script: training/deploy/compare_protocol.sh.

| | Nav2 MPPI | DRL run 2 |
|---|---|---|
| CSVs | nav2_nopillar_A_20261010_220450, nav2_nopillar_B_20261010_221545 | drl_run2_nopillar_A_20261010_224422, drl_run2_nopillar_B_20261010_225340 |
| run A | 4/6 | 4/6 |
| run B | 9/12 | 0/12 |
| **total** | **13/18 = 72% (Wilson 49-88%)** | **4/18 = 22% (Wilson 9-45%)** |
| mean time of successes | 75.6 s (A), 88.0 s (B) | 57.2 s (A) |
| min LiDAR range in run A | 0.63 m | 0.34 m (drives much closer to obstacles) |

Per goal, Nav2: c1 3/3, c2 3/3, c3 3/3, c4 3/3, c5 0/3 (stalls 0.34-0.50 m short), c6 1/3.
DRL run A: c1 0.04 m (38.6 s), c2 0.20 m (101.6 s), c3 0.09 m (49.6 s), c4 0.08 m (38.9 s) OK; c5 reported SUCCEEDED but
0.28 m real (AMCL error; tolerance 0.15 in AMCL frame); c6 failed.
After c5 the rover faced the wall inside the Collision Monitor stop polygon (x 0..0.55, |y| <= 0.33 m, i.e. 10 cm ahead of
the bumper). A stop zeroes every command (turning, BT backup too), the policy can only command v >= 0: trapped for the
remaining 13 attempts (every later trial is a failure from the same pose). Same mechanism as the earlier doorpost trap.

Reading: when it is not trapped the DRL controller reaches the goals faster than MPPI (4/4 of c1-c4 in run A, mean 57 vs 76 s),
but it drives into the Collision Monitor stop zone and cannot get out; MPPI keeps away because of costmap inflation.
DRL is NOT better than Nav2 here. One pass of 6 is not enough to compare the untrapped behaviour.

Invalid attempts (not counted): drl_eval_20261010_214608_INVALID_bt_server_timeout, drl_run2_nopillar_*_INVALID_controller_crashed,
drl_eval_run2_20261010_214914_passA_trapped_at_doorpost (world still had the pillar).

## Update 2026-10-11: run 3 (stop zone = collision in training)
results/drl_run3_A_20261011_022206.csv, drl_run3_B_20261011_023121.csv: run A 4/6 (c1-c4 OK, mean 49.9 s), c5 TIMEOUT 0.30 m,
then trapped facing the wall near c5 inside the stop polygon: run B 0/12. TOTAL 4/18 = 22% (9-45%). Nav2 13/18 = 72%.
