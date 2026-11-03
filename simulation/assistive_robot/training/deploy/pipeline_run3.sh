#!/usr/bin/env bash
# Run 3 (stop zone = collision): train 150 min -> eval val + test -> export actor -> Nav2 protocol comparison (18 goals).
set -e
PKG="$HOME/assistive_robot_ws/src/assistive_robot"; cd "$PKG"
echo "=== train start $(date +%H:%M)"
python3 training/gz_env/train_sac_gen.py --minutes 150 2>&1 | grep -v Warning | grep -E "PROMOTED|run dir|Traceback|Error" || true
R=$(ls -td training/runs/sac_gen_* | head -1); echo "run: $R"; [ -f "$R/model.zip" ] || { echo "no model"; exit 1; }
echo "=== eval val $(date +%H:%M)"; python3 training/gz_env/eval_gen.py $R/model.zip training/worlds/gen/val --episodes 10 --out $R/eval_val.csv 2>&1 | grep -v Warning | tail -5
echo "=== eval test $(date +%H:%M)"; python3 training/gz_env/eval_gen.py $R/model.zip training/worlds/gen/test --episodes 10 --out $R/eval_test.csv 2>&1 | grep -v Warning | tail -5
python3 training/deploy/export_actor.py $R/model.zip training/deploy/actor_run3.npz 2>&1 | grep -v Warning
echo "=== nav2 comparison $(date +%H:%M)"; ACTOR="$PKG/training/deploy/actor_run3.npz" MODE=drl training/deploy/compare_protocol.sh drl_run3 2>&1
cp /tmp/claude-1000/drl_stack/drl.log $R/deploy_drl.log; cp /tmp/claude-1000/drl_stack/nav2.log $R/deploy_nav2.log
echo "=== all done $(date +%H:%M)"
