# Run 2: SAC on generated worlds, footprint-clearance proximity reward (2026-10-10, seed 0)

Identical to sac_gen_20261010_113224 (run 1: same worlds, curriculum, 150 min, 6 workers, batch 1024, UTD 1/3, seed 0) except ONE
reward change in gz_nav_env.py CFG: proximity penalty measured from the footprint edge (prox_from='footprint', r_prox -0.2 when
clearance < 0.25 m, linear) instead of from the LiDAR centre (r_prox -0.1 when min range < 0.5 m).
Why: run-1 failure analysis (diag_failures.py, val tiers 2-3, 160 episodes): 39 collisions, 37 on walls, 35 of them within 0.3 m
of a wall end (door jamb / corner), 30/39 side hits, 95% at cmd v > 0.2 m/s. The old penalty barely fired: a point 2 cm from the
side is 0.40 m from the centre (-0.02 per step vs ~+0.3 progress).
Also added (infrastructure, no behaviour change): frozen-Gazebo guard (worker restarts Gazebo; not triggered in this run) and
optional per-step traces (GZNAV_TRACE=1).

Curriculum promotions: tier 0 at 4.0 min (47k), tier 1 at 6.6 min (76k), tier 2 at 26.4 min (284k) [run 1: 4.5 / 14.5 / 74.3 min].
1.5M steps, ~170 steps/s.

## Validation (300 episodes, 10 per world; val is used for promotion and for choosing run 2 over run 1)
| tier | run 1 | run 2 | scripted |
|---|---|---|---|
| 0 | 49/60 (82%, incl. 10 frozen-sim episodes) | 59/60 = 98% | 59/60 |
| 1 | 75/80 = 94% | 77/80 = 96% | 77/80 |
| 2 | 60/80 = 75% | 73/80 = 91% | 72/80 |
| 3 | 51/80 = 64% | 60/80 = 75% | 64/80 |
| all | 235/300 = 78% (73-83%) | 269/300 = 90% (86-93%) | 272/300 = 91% |
| collisions | 52 | 27 | 23 |

## Test (300 episodes, 10 per world; run 2 evaluated once on test, after choosing it on val)
| tier | run 2 | Wilson 95% | collision | run 1 | scripted |
|---|---|---|---|---|---|
| 0 | 60/60 = 100% | 94-100% | 0 | 98% | 100% |
| 1 | 76/80 = 95% | 88-98% | 3 | 98% | 98% |
| 2 | 62/80 = 78% | 67-85% | 16 | 65% | 90% |
| 3 | 57/80 = 71% | 61-80% | 23 | 60% | 75% |
| all | **255/300 = 85%** | 81-89% | 42 | 79% (74-83%) | **90%** |

## Reading
- The change helped: collisions roughly halved on val (52 -> 27) and test (60 -> 42); test success 79% -> 85%.
- Still below the scripted controller on test (85 vs 90%), mainly tier 2 (78 vs 90%). Val tier 2 (91%) is higher than test
  tier 2 (78%): the val worlds were used for promotion and model choice, so test is the number to quote.
- One training seed per variant. The run-1 vs run-2 difference is beyond eval noise (non-overlapping CIs) but training-seed
  variance is not measured. PLAN.md wants 3 seeds for final claims.
- No Nav2 comparison on these worlds yet.
