# Run 3: stop zone = collision (2026-10-10/11, seed 0)

ONE change vs run 2 (sac_gen_20261010_142340): gz_nav_env CFG stop_zone=(0.0, 0.55, 0.33, 4): an episode also ends as a
collision when >= 4 scan points are inside the Nav2 Collision Monitor PolygonStop box (robot frame x 0..0.55, |y| <= 0.33).
Why: on the real stack run 2 was trapped twice inside that zone (stop zeroes all commands, policy has v >= 0 only).
Same worlds, curriculum, 150 min, seed. Promotions: tier 0 6.3 min, tier 1 14.6 min, tier 2 61.2 min (run 2: 4.0 / 6.6 / 26.4).
Pipeline: training/deploy/pipeline_run3.sh.

## Generated worlds (10 episodes per world, deterministic)
| | run 2 val | run 3 val | run 2 test | run 3 test | scripted test |
|---|---|---|---|---|---|
| tier 0 | 98% | 100% | 100% | 100% | 100% |
| tier 1 | 96% | 98% | 95% | 98% | 98% |
| tier 2 | 91% | 89% | 78% | 79% | 90% |
| tier 3 | 75% | 79% | 71% | 69% | 75% |
| all | 90% | 91% (87-93) | 85% | 85% (81-89) | 90% |
| collisions | 27 | 25 | 42 | 43 | 27 |
Run 3 collisions include stop-zone entries (stricter), so equal counts mean more clearance at equal success.

## assistive_world through Nav2 (no pillar; compare_protocol.sh; results/drl_run3_{A,B}_*.csv)
Run A: c1 0.13 m 62.9 s, c2 0.09 m 63.0 s, c3 0.05 m 41.9 s, c4 0.05 m 31.9 s (all OK, mean 49.9 s; Nav2 75.6 s);
c5 TIMEOUT 0.30 m (6 recoveries); c6 failed. Then trapped at (4.77, 2.18) facing the corridor wall (yaw about -83 deg)
inside the stop polygon from 02:36: run B 0/12. TOTAL 4/18 = 22% (Wilson 9-45%), same as run 2. Nav2 13/18 = 72%.
c5 (5, 2) is 0.40 m from the wall (2 cm side clearance); training goals are >= 0.65 m from obstacles, so the policy never
practised a near-wall goal. BT recoveries near the wall (spin) can also bring the wall into the front polygon.

## Reading
- In training worlds the change keeps success equal with more clearance.
- In the house it does not prevent the trap at the near-wall goal; any controller inside the stop polygon is stuck for good
  (stop zeroes turns, reverse and BT backup). MPPI avoids it only by stalling 0.34-0.50 m short of c5.

## CORRECTION 2026-10-11 (morning)
The house run above was affected by two bugs in training/deploy/drl_controller.py (BT ack timeout 200 ms; progress checker
reset on every path update, so the BT never ran recoveries). Both fixed; see results/OVERNIGHT_SUMMARY_20261011.md.
Valid rerun with both fixes and the reverse-aware stop zone (results/drl_run3_cmrev3_*.csv): 4/18 = 22% (9-45%), again
c1-c4 OK then wedged near (4.4, 2.2); BT spin/backup now run but fail (turning stopped by the collision monitor, backup
"Collision Ahead"). Nav2 MPPI with the same overrides: 14/18 = 78% (55-91%).
