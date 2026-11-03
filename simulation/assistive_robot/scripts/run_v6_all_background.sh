#!/usr/bin/env bash

set -u

ROOT="$HOME/assistive_robot_ws/src/assistive_robot"
SCRIPTS="$ROOT/scripts"
LOGDIR="$HOME/assistive_robot_ws/logs/v6_background"

mkdir -p "$LOGDIR"

MAIN_LOG="$LOGDIR/main.log"
GAZEBO_LOG="$LOGDIR/gazebo.log"
BRIDGE_LOG="$LOGDIR/bridge.log"
FUSION_LOG="$LOGDIR/state_fusion.log"
EVAL_LOG="$LOGDIR/evaluation.log"

exec > >(tee -a "$MAIN_LOG") 2>&1

echo "=============================================="
echo "V6 BACKGROUND RUN"
echo "$(date)"
echo "=============================================="

# ---------------------------------------------------------
# CLEAN OLD PROJECT PROCESSES
# ---------------------------------------------------------

echo "[CLEAN] Stopping old project processes..."

pkill -f "gz sim -r -s worlds/assistive_world.sdf" 2>/dev/null || true
pkill -f "parameter_bridge.*assistive_robot.*bridge.yaml" 2>/dev/null || true
pkill -f "python3 state_fusion.py" 2>/dev/null || true

sleep 3

# ---------------------------------------------------------
# START GAZEBO HEADLESS
# ---------------------------------------------------------

echo "[START] Gazebo headless..."

cd "$ROOT"

export GZ_SIM_RESOURCE_PATH="$ROOT/models:${GZ_SIM_RESOURCE_PATH:-}"

nohup gz sim -r -s worlds/assistive_world.sdf \
    > "$GAZEBO_LOG" 2>&1 &

GAZEBO_PID=$!

echo "[GAZEBO] PID=$GAZEBO_PID"

sleep 8

# ---------------------------------------------------------
# START BRIDGE
# ---------------------------------------------------------

echo "[START] ros_gz_bridge..."

source /opt/ros/jazzy/setup.bash

nohup ros2 run ros_gz_bridge parameter_bridge \
    --ros-args \
    -p config_file:="$ROOT/config/bridge.yaml" \
    > "$BRIDGE_LOG" 2>&1 &

BRIDGE_PID=$!

echo "[BRIDGE] PID=$BRIDGE_PID"

sleep 5

# ---------------------------------------------------------
# START STATE FUSION
# ---------------------------------------------------------

echo "[START] state_fusion..."

cd "$SCRIPTS"

nohup /usr/bin/python3 state_fusion.py \
    > "$FUSION_LOG" 2>&1 &

FUSION_PID=$!

echo "[FUSION] PID=$FUSION_PID"

# ---------------------------------------------------------
# CLEANUP FUNCTION
# ---------------------------------------------------------

cleanup() {
    echo
    echo "=============================================="
    echo "CLEANUP"
    echo "$(date)"
    echo "=============================================="

    kill "$FUSION_PID" 2>/dev/null || true
    kill "$BRIDGE_PID" 2>/dev/null || true
    kill "$GAZEBO_PID" 2>/dev/null || true

    sleep 3

    pkill -f "python3 state_fusion.py" 2>/dev/null || true
    pkill -f "parameter_bridge.*assistive_robot.*bridge.yaml" 2>/dev/null || true
    pkill -f "gz sim -r -s worlds/assistive_world.sdf" 2>/dev/null || true

    echo "[CLEANUP] complete."
}

trap cleanup EXIT INT TERM

# ---------------------------------------------------------
# WAIT FOR ROS DATA
# ---------------------------------------------------------

echo
echo "[WAIT] Waiting for /odom..."

for i in $(seq 1 30); do

    if timeout 2 ros2 topic echo /odom --once >/dev/null 2>&1; then
        echo "[OK] /odom is alive."
        break
    fi

    echo "[WAIT] /odom attempt $i/30..."
    sleep 2

    if [ "$i" -eq 30 ]; then
        echo "[ERROR] /odom never became available."
        exit 1
    fi

done

echo
echo "[WAIT] Waiting for /dr_state..."

for i in $(seq 1 30); do

    if timeout 2 ros2 topic echo /dr_state --once >/dev/null 2>&1; then
        echo "[OK] /dr_state is alive."
        break
    fi

    echo "[WAIT] /dr_state attempt $i/30..."
    sleep 2

    if [ "$i" -eq 30 ]; then
        echo "[ERROR] /dr_state never became available."
        exit 1
    fi

done

# ---------------------------------------------------------
# RUN FULL EVALUATION
# ---------------------------------------------------------

echo
echo "=============================================="
echo "STARTING FULL V6 EVALUATION"
echo "$(date)"
echo "=============================================="

cd "$SCRIPTS"

source ~/assistive_v6_env/bin/activate

python run_everything_v6.py \
    2>&1 | tee "$EVAL_LOG"

RESULT=$?

echo
echo "=============================================="
echo "EVALUATION FINISHED"
echo "$(date)"
echo "EXIT CODE: $RESULT"
echo "=============================================="

exit "$RESULT"
