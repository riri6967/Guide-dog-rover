#!/usr/bin/env bash
# Run 4: ONE training change set via GZNAV_CFG (recorded in the run's config.json): train 150 min -> eval val + test (same
# GZNAV_CFG) -> export actor -> Nav2 protocol comparison with NAV2_OVERRIDES (same file Nav2 was re-run with) and DRL_ARGS.
# usage: GZNAV_CFG='{"near_wall_goal_p":0.3}' NAV2_OVERRIDES=cm_reverse_override.yaml DRL_ARGS="" training/deploy/pipeline_run4.sh
set -e
PKG="$HOME/assistive_robot_ws/src/assistive_robot"; cd "$PKG"
echo "=== GZNAV_CFG=$GZNAV_CFG NAV2_OVERRIDES=$NAV2_OVERRIDES DRL_ARGS=$DRL_ARGS"
export GZNAV_CFG
echo "=== train start $(date +%H:%M)"
python3 training/gz_env/train_sac_gen.py --minutes 150 2>&1 | grep -v Warning | grep -E "PROMOTED|run dir|Traceback|Error" || true
R=$(ls -td training/runs/sac_gen_* | head -1); echo "run: $R"; [ -f "$R/model.zip" ] || { echo "no model"; exit 1; }
echo "=== eval val $(date +%H:%M)"; python3 training/gz_env/eval_gen.py $R/model.zip training/worlds/gen/val --episodes 10 --out $R/eval_val.csv 2>&1 | grep -v Warning | tail -5
echo "=== eval test $(date +%H:%M)"; python3 training/gz_env/eval_gen.py $R/model.zip training/worlds/gen/test --episodes 10 --out $R/eval_test.csv 2>&1 | grep -v Warning | tail -5
python3 training/deploy/export_actor.py $R/model.zip training/deploy/actor_run4.npz 2>&1 | grep -v Warning
unset GZNAV_CFG  # deploy stack must not see it (run_drl_stack also unsets it)
echo "=== nav2 comparison $(date +%H:%M)"; HEADLESS=1 ACTOR="$PKG/training/deploy/actor_run4.npz" MODE=drl training/deploy/compare_protocol.sh drl_run4 2>&1
cp /tmp/claude-1000/drl_stack/drl.log $R/deploy_drl.log; cp /tmp/claude-1000/drl_stack/nav2.log $R/deploy_nav2.log
echo "=== all done $(date +%H:%M)"
