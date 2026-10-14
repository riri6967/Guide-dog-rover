#!/usr/bin/env bash
# Stop the processes started by run_drl_stack.sh (by their process groups, from the saved PIDs; no pattern kills).
LOG="${LOG:-/tmp/claude-1000/drl_stack}"
for n in drl nav2 bridge gazebo; do
  f="$LOG/$n.pid"; [ -f "$f" ] || continue; pid=$(cat "$f")
  kill -INT -- -"$pid" 2>/dev/null; sleep 2; kill -TERM -- -"$pid" 2>/dev/null
  sleep 3; kill -0 -- -"$pid" 2>/dev/null && { echo "$n group $pid survived TERM: KILL"; kill -KILL -- -"$pid" 2>/dev/null; }  # gz sim can hang on shutdown (seen 2026-10-11)
  rm -f "$f"
done
f="$LOG/gazebo_children.pid"  # gz server/gui started by the gz wrapper (own process groups in GUI mode)
if [ -f "$f" ]; then
  for pid in $(cat "$f"); do kill -TERM "$pid" 2>/dev/null; done; sleep 2
  for pid in $(cat "$f"); do kill -0 "$pid" 2>/dev/null && { echo "gz child $pid survived TERM: KILL"; kill -KILL "$pid" 2>/dev/null; }; done
  rm -f "$f"
fi
sleep 2; echo stopped
