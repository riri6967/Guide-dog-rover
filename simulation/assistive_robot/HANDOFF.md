# Assistive Rover: Nav2 baseline + DRL (SAC) local navigation — handoff

ROS 2 Jazzy + Gazebo Sim, Ubuntu. Package: `assistive_robot/` (put it in `<ws>/src/`).

## Read first
1. `results/FINAL_REPORT_DRAFT.md`: final numbers and conclusions
2. `CLAUDE.md`: full project log, startup steps, pitfalls
3. `PLAN.md`: phase plan

## Final results (house world, same 6 goals, judged against Gazebo ground truth)
| Controller | Success | 95% CI | Mean time |
|---|---|---|---|
| Nav2 (SmacPlanner2D + MPPI) | 26/36 = 72% | 56–84% | 77 s |
| Hybrid (DRL run 3 + MPPI in narrow spots + escape) | 25/36 = 69% | 53–82% | 74 s |
| Pure DRL (SAC run 3) | 4/18 = 22% | 9–45% | – |

The hybrid is statistically tied with Nav2; it does not beat it.

## What's where
- `worlds/`, `models/assistive_rover/`, `config/bridge.yaml`: Gazebo world, rover, ROS-Gazebo bridge
- `nav2/`: params, launch files (incl. `navigation_drl.launch.py`, `navigation_variant.launch.py`), map + `generate_map.py`, `goals.csv`
- `scripts/auto_nav2_eval_v2.py`: the evaluator
- `baseline_20261010/`: frozen Nav2 baseline snapshot
- `training/gz_env/`, `training/worlds_gen/`, `training/worlds/`: lockstep Gazebo RL env + generated worlds (train/val/test)
- `training/runs/<run>/`: config, final `model.zip` (SB3 SAC), progress, eval CSVs, RESULTS.md. Best = `sac_gen_20261010_234158` (run 3)
- `training/deploy/`: `actor_run3.npz` (numpy actor), `drl_controller.py`, `hybrid_controller.py`, `run_drl_stack.sh` / `stop_drl_stack.sh`
- `results/`: every evaluation CSV/log + summaries. Files with `INVALID` in the name are known-bad runs, kept for the record.

## Not included (size)
Replay buffers (2.8 GB each), intermediate checkpoints, file backups, legacy YOLO weights.

## Python
- rclpy / gz bindings: system `/usr/bin/python3` (3.12)
- Training (torch, stable-baselines3 2.x, gymnasium 1.x): separate Python 3.10 env; the two are joined by a socket (`training/worker.py`, `vec_env.py`).
