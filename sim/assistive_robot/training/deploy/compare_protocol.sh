#!/usr/bin/env bash
# Baseline protocol (baseline_20261010/README.md) for one controller: fresh stack -> run A (1 pass of the 6 goals) ->
# NavigateToPose back to (0,0) -> run B (2 passes). Same evaluator (scripts/auto_nav2_eval_v2.py), timeout 150 s, tol 0.25 m.
# usage: MODE=nav2|drl training/deploy/compare_protocol.sh TAG     (CSV files renamed results/<TAG>_{A,B}_<ts>.csv)
set -e
PKG="$HOME/assistive_robot_ws/src/assistive_robot"; cd "$PKG"; TAG="$1"
training/deploy/stop_drl_stack.sh >/dev/null
training/deploy/run_drl_stack.sh | tail -1
source /opt/ros/jazzy/setup.bash; export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1
run() {  # $1 = repeats, $2 = A/B
  raw="results/${TAG}_$2_evaluator_raw.log"  # full evaluator output (2026-10-11: a run B ended with no results and no trace)
  /usr/bin/python3 scripts/auto_nav2_eval_v2.py --repeats "$1" > "$raw" 2>&1; echo "evaluator exit $?" >> "$raw"
  out=$(grep -v "^\[INFO\]" "$raw")
  echo "$out" | grep -E "^\[|trials:|mean time"
  csv=$(echo "$out" | grep -o "/.*nav2_eval_v2_[0-9_]*\.csv" | tail -1)
  [ -n "$csv" ] && mv "$csv" "results/${TAG}_$2_$(basename "$csv" | sed 's/nav2_eval_v2_//')" && echo "csv: results/${TAG}_$2_$(basename "$csv" | sed 's/nav2_eval_v2_//')"
}
echo "=== $TAG run A"; run 1 A
echo "=== back to origin"; timeout 200 ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 0.0, y: 0.0}, orientation: {w: 1.0}}}}" 2>&1 | grep "finished" || echo "return goal did not finish"
timeout 5 gz topic -e -n 1 -t /world/assistive_world/pose/info | grep -A4 'name: "assistive_rover"' | grep -A2 position | tr -s ' \n' ' '; echo
echo "=== $TAG run B"; run 2 B
training/deploy/stop_drl_stack.sh >/dev/null; echo "=== $TAG done"
