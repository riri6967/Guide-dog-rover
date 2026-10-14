#!/usr/bin/env bash
# Start the DRL comparison stack (same startup as CLAUDE.md, step 3 uses navigation_drl.launch.py) + the DRL controller.
# Logs in $LOG (default /tmp/claude-1000/drl_stack). Gazebo starts WITH GUI so the run can be watched (use HEADLESS=1 for -s).
# MODE=nav2 starts the frozen baseline instead (navigation.launch.py, MPPI, no DRL controller).
# DRL_NODE=training/deploy/hybrid_controller.py runs the DRL+MPPI hybrid instead of pure DRL.
# DRL_ARGS="--v-min -0.1" passes extra drl_controller.py options (compare_protocol.sh does not forward args).
# Stop with: training/deploy/stop_drl_stack.sh
set -e
PKG="$HOME/assistive_robot_ws/src/assistive_robot"; LOG="${LOG:-/tmp/claude-1000/drl_stack}"; mkdir -p "$LOG"
source /opt/ros/jazzy/setup.bash
export ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST GZ_IP=127.0.0.1
unset GZ_PARTITION GZNAV_CFG GZNAV_MARKER GZNAV_TRACE
ros2 daemon stop >/dev/null 2>&1 || true
export GZ_SIM_RESOURCE_PATH="$PKG/models:${GZ_SIM_RESOURCE_PATH:-}"
cd "$HOME/assistive_robot_ws"
if [ "${HEADLESS:-0}" = 1 ]; then GZ_ARGS="-s -r --headless-rendering"; else GZ_ARGS="-r"; fi
setsid gz sim $GZ_ARGS src/assistive_robot/worlds/assistive_world.sdf > "$LOG/gazebo.log" 2>&1 & echo $! > "$LOG/gazebo.pid"
sleep 8
pgrep -P "$(cat "$LOG/gazebo.pid")" > "$LOG/gazebo_children.pid" || true  # GUI mode: gz server/gui run in their own process groups
setsid ros2 run ros_gz_bridge parameter_bridge --ros-args -p config_file:="$PKG/config/bridge.yaml" > "$LOG/bridge.log" 2>&1 & echo $! > "$LOG/bridge.pid"
sleep 3
cd "$PKG"
if [ -n "${NAV2_OVERRIDES:-}" ]; then  # variant launch (frozen launch files untouched)
  if [ "${MODE:-drl}" = nav2 ]; then export NAV2_CONTROLLER=mppi; else export NAV2_CONTROLLER=drl; fi
  setsid ros2 launch nav2/launch/navigation_variant.launch.py > "$LOG/nav2.log" 2>&1 & echo $! > "$LOG/nav2.pid"
  if [ "$NAV2_CONTROLLER" = drl ]; then
    setsid /usr/bin/python3 ${DRL_NODE:-training/deploy/drl_controller.py} --actor "${ACTOR:-$PKG/training/deploy/actor_run2.npz}" ${DRL_ARGS:-} "$@" > "$LOG/drl.log" 2>&1 & echo $! > "$LOG/drl.pid"
  fi
elif [ "${MODE:-drl}" = nav2 ]; then
  setsid ros2 launch nav2/launch/navigation.launch.py > "$LOG/nav2.log" 2>&1 & echo $! > "$LOG/nav2.pid"
else
  setsid ros2 launch nav2/launch/navigation_drl.launch.py > "$LOG/nav2.log" 2>&1 & echo $! > "$LOG/nav2.pid"
  setsid /usr/bin/python3 ${DRL_NODE:-training/deploy/drl_controller.py} --actor "${ACTOR:-$PKG/training/deploy/actor_run2.npz}" ${DRL_ARGS:-} "$@" > "$LOG/drl.log" 2>&1 & echo $! > "$LOG/drl.pid"
fi
for i in $(seq 1 60); do
  a=$(ros2 lifecycle get /amcl 2>/dev/null || true); p=$(ros2 lifecycle get /bt_navigator 2>/dev/null || true)
  if [[ "$a" == active* && "$p" == active* ]]; then echo "stack up after $((i*2)) s: amcl $a, bt_navigator $p"; exit 0; fi
  sleep 2
done
echo "stack did NOT come up; see $LOG"; exit 1
