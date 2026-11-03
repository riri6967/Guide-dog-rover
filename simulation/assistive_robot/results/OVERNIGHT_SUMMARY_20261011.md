# Overnight summary, 2026-10-11 (autonomous run, 03:10 to 09:25)

All house numbers: assistive_world without the Room 2 pillar, baseline protocol (fresh start, run A = 1 pass of the 6 goals,
NavigateToPose back to (0,0), run B = 2 passes; success = SUCCEEDED and Gazebo ground-truth distance <= 0.25 m, timeout 150 s).
Intervals are Wilson 95%. One run each: back-to-back trials, not independent.

## 1. Headline

| Controller (same goals, same evaluator, same Nav2 overrides) | Successes | Notes |
|---|---|---|
| Nav2 MPPI, stop zone allows reversing (`nav2_cmrev`) | **14/18 = 78% [55, 91]** | c1, c3, c4, c6 3/3; c2 2/3 (one SUCCEEDED at 0.26 m real); c5 0/3 (stops 0.2-0.3 m short, as in the frozen baseline) |
| Nav2 MPPI, original stop zone (yesterday, `COMPARISON_DRL_vs_NAV2_20261010.md`) | 13/18 = 72% [49, 88] | for reference |
| DRL run 3, stop zone allows reversing, both controller-node bugs fixed (`drl_run3_cmrev3`) | **4/18 = 22% [9, 45]** | run A: c1-c4 OK (mean 74 s), c5 timeout, then wedged near (4.4, 2.2) for the remaining 13 trials |
| DRL run 4 (policy may reverse, v_min -0.1; `drl_run4`) | **2/18 = 11% [3, 33]** | c1, c2 OK, then corner pivoted onto the Room 2 wall end at (3.62, -1.49); never moved again |

**Short version: Nav2 is clearly better on the house (14/18 vs best DRL 4/18). Run 4 (reverse) made things worse; run 3
stays the best DRL model. DRL is faster when it works (run 3 run A mean 74 s incl. one 136 s goal; Nav2 81 s).**

DRL does NOT beat Nav2 on the house. Nav2 14/18 vs DRL run 3 4/18 is a clear gap (intervals do not overlap).
The reverse-aware stop zone did not change the Nav2 result meaningfully (13/18 -> 14/18, within noise).

## 2. Two bugs in MY DRL controller node (training/deploy/drl_controller.py), found and fixed tonight

Both affected EVERY earlier DRL house run, including yesterday's 4/18 (run 2) and 4/18 (run 3). Those numbers measured the
bugs as much as the policy. The valid DRL number is the `drl_run3_cmrev3` row above (by coincidence also 4/18, but now the
failure is a real driving failure, see section 3).

1. **Goal-acknowledge timeout.** bt_navigator waits `default_server_timeout` for the controller to accept each path; it re-sends
   the path about once per second. Probe under the full stack (60 goals at 1 Hz): ack median 18 ms, p90 81 ms, p99 152 ms,
   max 224 ms. With 200 ms, a long navigation almost surely hits one timeout, and each one fails the whole goal (15 in the
   first fix-1 DRL run, 0/18). Fix: `nav2/params/drl_overrides.yaml` 200 -> 1000 ms (DRL launch only; does not change
   control, planning or recoveries; Nav2/MPPI runs do not load the file). Result: 0 ack timeouts since.
2. **Progress checker reset on every path update.** The "no progress for 10 s" timer lived inside each FollowPath call, and
   the BT's ~1 Hz replans start a new call, so it never fired: a blocked rover sat forever with 0 recoveries and the BT never
   got FAILED_TO_MAKE_PROGRESS. Nav2's controller_server keeps its progress checker across path updates. Fix: progress
   state kept per navigation (reset only when a new navigation starts or after an abort). Verified: warnings fire and the
   BT runs spin/backup (193 no-progress aborts in `drl_run3_cmrev3`).

Also added: SIGUSR1 stack dump in the node (diagnostic, `kill -USR1 <pid>`), `DRL_ARGS` pass-through in run_drl_stack.sh,
SIGKILL fallback in stop_drl_stack.sh (one headless gz ignored INT and TERM at the end of a run; it started with that run,
so no overlap with another run).

Invalid outputs renamed (not deleted): `results/drl_run3_cmrev_*_INVALID_ack_timeout.*`,
`results/drl_run3_cmrev2_*_INVALID_progress_reset.*`.

## 3. Why DRL fails on the house (valid run, `drl_run3_cmrev3`)

- The policy drives itself into poses close to a wall (around c5 (5,2), whose goal is 0.40 m from the wall; and in one
  repro at the top-corridor gap wall end (0.3, 1.8)).
- There: the Collision Monitor front stop box blocks forward motion AND turning (8 stop events); the policy can only command
  v >= 0, so it keeps pushing forward.
- BT recoveries now run (17-18 per goal) but cannot free it: 31/31 spins "Exceeded time allowance" (turning is stopped),
  29 backups failed, 26 of them "Collision Ahead" (Nav2's costmap sees an obstacle in the 0.3 m reverse path).
- Nav2/MPPI also fails c5 (0/3) but never wedges itself; it goes on to c6 afterwards.

## 4. Run 4 (one training change): the policy may reverse slowly

- Change: `GZNAV_CFG={"v_min": -0.1}` (action -1 -> -0.1 m/s; measured v in the observation becomes signed). Everything else
  as run 3 (stop zone as collision, footprint proximity penalty, 150 min, same curriculum, same worlds).
- Deployment: `drl_controller.py --v-min -0.1`, same `cm_reverse_override.yaml` as the Nav2 row (the rear stop box still
  stops reversing into obstacles).
- Run dir: `training/runs/sac_gen_20261011_055331/`. Pipeline: `training/deploy/pipeline_run4.sh`.

RESULTS (details: training/runs/sac_gen_20261011_055331/RESULTS.md):

| | run 3 val | run 4 val | run 3 test | run 4 test | scripted test |
|---|---|---|---|---|---|
| all tiers | 91% | 84% | 85% (81-89) | 78% (73-82) | 90% |
| tier 2 / tier 3 | 89 / 79% | 76 / 65% | 79 / 69% | 70 / 58% | 90 / 75% |
| collisions | 25 | 48 | 43 | 64 | 27 |

House: 2/18 = 11% [3, 33]. After c2 the rover stopped at (3.62, -1.49) yaw 143 deg in the Room 2 opening; its front-left
corner (about (3.03, -1.52)) sits on the Room 2 wall end. Spin timed out 35/35, backup "Collision Ahead" 32/32, and the policy
never reversed out. Reverse added collisions and gave no escapes: do not continue this line.

## 4b. My recommendation for what next (your call; I did not start anything)

Both valid DRL house failures are near-wall / wall-end clipping at doorways, which the generated-world tests do not punish as
hard. Options, cheapest first:
1. **Accept Nav2 as the deployment controller** and report DRL honestly as weaker on the house (78% vs 22%). The comparison is
   now defensible: same goals, evaluator, Nav2 overrides, ground truth, both controller-node bugs fixed.
2. **Pose-noise training (run 5, one change):** training uses ground-truth pose; deployment uses AMCL (about 0.08 m / 6 deg at
   rest, up to 29 deg in fast turns). Add that noise to the subgoal pose in training. Untested hypothesis for the clipping.
3. **Hybrid:** DRL in open space, MPPI near obstacles/goals. More engineering, muddier comparison.

## 5. Files changed tonight
No leftover processes (checked 09:20: no gz, Nav2, bridge or DRL node running).
 (all backed up as *.backup_20261011_*)
- nav2/params/drl_overrides.yaml (ack timeout 1000 ms)
- training/deploy/drl_controller.py (progress state per navigation; SIGUSR1 dump)
- training/deploy/run_drl_stack.sh (DRL_ARGS), training/deploy/stop_drl_stack.sh (KILL fallback)
- new: training/deploy/pipeline_run4.sh, nav2/params/cm_reverse_override.yaml (from last night)
- training/runs/sac_gen_20261010_234158/RESULTS.md: correction note appended (house run had the node bugs)
- training/runs/sac_gen_20261011_055331/RESULTS.md: new (run 4); training/deploy/actor_run4.npz: new
- Frozen baseline files (navigation.launch.py, nav2_params.yaml, world, map) NOT edited.
