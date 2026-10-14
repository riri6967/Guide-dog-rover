#!/usr/bin/env bash
# Afternoon 2026-10-11: repeat runs (hybrid, Nav2) + one DRL-share variant (narrow threshold 0.9 -> 0.7 m). Same protocol,
# same NAV2_OVERRIDES for all. Logs copied to results/<TAG>_{nav2,drl}.log.
P=/home/pushkar/assistive_robot_ws/src/assistive_robot; cd $P
export HEADLESS=1 NAV2_OVERRIDES=cm_reverse_override.yaml
run() {  # $1 tag, $2 MODE, $3 DRL_ARGS
  echo "##### $1 start $(date +%H:%M)"
  ACTOR=$P/training/deploy/actor_run3.npz DRL_NODE=training/deploy/hybrid_controller.py DRL_ARGS="$3" MODE=$2 $P/training/deploy/compare_protocol.sh $1
  cp /tmp/claude-1000/drl_stack/nav2.log $P/results/$1_nav2.log; [ "$2" = drl ] && cp /tmp/claude-1000/drl_stack/drl.log $P/results/$1_drl.log
  echo "##### $1 end $(date +%H:%M)"
}
run hybrid_run3_esc_r2 drl "--g-in 0"
run nav2_cmrev_r2 nav2 ""
run hybrid_run3_esc_mc07 drl "--g-in 0 --map-c 0.7"
echo "=== queue end $(date +%H:%M)"
