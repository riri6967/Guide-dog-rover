# Run 4: policy may reverse slowly (2026-10-11, seed 0)

ONE change vs run 3 (sac_gen_20261010_234158): GZNAV_CFG {"v_min": -0.1} (action -1 -> -0.1 m/s, +1 -> 0.3 m/s; measured v in
the observation becomes signed). Recorded in config.json env_cfg_override. Why: in the house run 3 wedged facing a wall and
could only command v >= 0. Same worlds, curriculum, 150 min, seed, stop zone as collision. Promotions: tier 0 4.5 min,
tier 1 7.5 min, tier 2 25.3 min (run 3: 6.3 / 14.6 / 61.2). Pipeline: training/deploy/pipeline_run4.sh.
Deployment: drl_controller.py --v-min -0.1 (verified on the process command line), NAV2_OVERRIDES=cm_reverse_override.yaml.

## Generated worlds (10 episodes per world, deterministic, same GZNAV_CFG)
| | run 3 val | run 4 val | run 3 test | run 4 test | scripted test |
|---|---|---|---|---|---|
| tier 0 | 100% | 100% | 100% | 97% | 100% |
| tier 1 | 98% | 98% | 98% | 91% | 98% |
| tier 2 | 89% | 76% | 79% | 70% | 90% |
| tier 3 | 79% | 65% | 69% | 58% | 75% |
| all | 91% (87-93) | 84% (79-87) | 85% (81-89) | 78% (73-82) | 90% |
| collisions | 25 | 48 | 43 | 64 | 27 |

## assistive_world through Nav2 (results/drl_run4_{A,B}_*.csv)
Run A: c1 OK 45.8 s, c2 OK 73.9 s, then c3 TIMEOUT: rover stopped at (3.62, -1.49) yaw 143 deg (ground truth) in the Room 2
opening and never moved again (16 trials, identical end pose). TOTAL 2/18 = 11% (Wilson 3-33%).
At that pose the front-left footprint corner is at about (3.03, -1.52), on the face of the Room 2 wall end (x = 3): the
policy pivoted the corner onto the wall end. BT recoveries ran (211 no-progress aborts) but spin timed out 35/35 (corner
cannot swing through the wall) and backup failed 32/32 "Collision Ahead" (costmap). The policy did not reverse out.

## Reading
- Worse than run 3 on every split (one seed each; test 85% -> 78%, Wilson intervals barely overlap; collisions 43 -> 64).
- Reverse ability did not produce escapes; it added collisions. Run 3 stays the best DRL model. Do not continue this line.
- Both house failures (run 3 at c5, run 4 at the Room 2 opening) are wall-end / near-wall clipping. Hypothesis (untested):
  training uses ground-truth pose, deployment uses AMCL (about 0.08 m / 6 deg at rest, up to 29 deg in fast turns), and the
  subgoal geometry shifts with that error near door edges.
