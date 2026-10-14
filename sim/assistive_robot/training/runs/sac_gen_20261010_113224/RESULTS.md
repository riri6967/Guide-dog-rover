# SAC on generated worlds with curriculum (2026-10-10, PLAN.md Phases 3 + 5, seed 0)

## Setup
- Worlds: `training/worlds_gen/gen_worlds.py` -> `training/worlds/gen/{train,val,test}` (200 / 30 / 30), manifest.json, preview/.
  Tiers: 0 open room, 1 sparse obstacles, 2 corridor building (straight / dead-end / L / T, corridor 1.2-3 m, doors 0.9-2.0 m,
  15% hard 0.9-1.0 m), 3 = 2 + clutter. All obstacles 1.2-1.8 m tall (LiDAR is at 0.96 m; the old trial-world boxes were
  0.5-1.0 m, mostly invisible to it). SDF and 0.05 m grid come from the same primitive list (parser cross-check: 0 missing cells).
- Env: gz_nav_env.py unchanged except the grid clearance (fine-grid EDT, min per 0.1 m cell, passable > 0.39 m), so 0.9 m doors
  are not closed by downsampling. Generator rejects layouts the env would see as cut off (region identical in all 260 worlds).
- Training: `train_sac_gen.py --minutes 150`: 6 workers, world switch every 10 episodes (2 pre-loaded spares), SAC MLP 256x256,
  batch 1024, UTD 1/3 in a background thread, buffer 1M, learning_starts 10k, gamma 0.99. Validation concurrently on 2 extra
  workers (deterministic policy). Curriculum: stage k = 50% tier k + 50% lower tiers; promote at >= 80% over the last 40
  val episodes of tier k.
- Throughput: 1.53M env steps in 150 min, ~170 steps/s with learner (sac_trial: 60/s). 510k gradient updates.

## Curriculum (val success that triggered promotion, last 40 episodes)
| stage | promoted at | steps |
|---|---|---|
| 0 -> 1 | 4.5 min | 51k |
| 1 -> 2 | 14.5 min | 162k |
| 2 -> 3 | 74.3 min | 788k |
Tier 2 hovered at 55-72% for ~45 min before one 40-episode window hit 80%: the promotion was on a noisy high point.

## Final model, deterministic, 10 episodes per world
Validation worlds (also used for promotion, so not unbiased):
| tier | success | Wilson 95% | collision | stuck |
|---|---|---|---|---|
| 0 | 49/60 = 82% | 70-89% | 1 | 10 |
| 1 | 75/80 = 94% | 86-97% | 4 | 1 |
| 2 | 60/80 = 75% | 65-83% | 20 | 0 |
| 3 | 51/80 = 64% | 53-73% | 27 | 3 |
| all | 235/300 = 78% | 73-83% | 52 | 14 |
All 10 tier-0 stuck episodes were one world (w0_003) on one Gazebo instance, 60 steps each with no motion. Rerun of that world
alone: 20/20. Env-side fault (one worker), not the policy; cause not found. Without it tier 0 is 49/50.

Test worlds (never used for training, tuning or promotion), with the scripted controller on the same worlds:
| tier | SAC | Wilson 95% | collision | scripted (lookahead 0.8 m) | collision |
|---|---|---|---|---|---|
| 0 open | 59/60 = 98% | 91-100% | 1 | 60/60 = 100% | 0 |
| 1 sparse | 78/80 = 98% | 91-99% | 2 | 78/80 = 98% | 1 |
| 2 rooms + doors | 52/80 = 65% | 54-75% | 25 | 72/80 = 90% | 7 |
| 3 clutter | 48/80 = 60% | 49-70% | 32 | 60/80 = 75% | 19 |
| all | **237/300 = 79%** | 74-83% | 60 | **270/300 = 90%** | 27 |
SAC successful episodes: mean 17.7 s for 5.3 m geodesic. No timeouts; failures are almost all collisions.
Note the scripted controller is not a learned baseline: it uses the same GT subgoal with a 0.8 m lookahead and front slowdown.

## Conclusions
- Open and sparse rooms are solved and generalize (98% test, same as scripted).
- Buildings with doors and clutter are not: 65% / 60% vs scripted 90% / 75%, failures are collisions (25 + 32 of 160).
  Validation (75 / 64%) and test (65 / 60%) agree within noise: this is under-training/capability, not overfitting to train worlds.
- DRL does not beat the scripted controller yet. No comparison with Nav2 has been made on these worlds.

## Known issues / not done
- Occasional stuck Gazebo instance (above). Needs a guard (e.g. detect zero motion under non-zero command and restart the worker).
- One seed only (PLAN.md wants 3 for final claims). No resume from checkpoint implemented (replay buffer is saved: 2.9 GB).
- Subgoal comes from the geodesic distance field, not the Nav2 planner path (PLAN.md Phase 3 last item).
