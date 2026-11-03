#!/bin/bash
# Start the whole rover on the Pi: micro-ROS agent (ESP32), URDF, wheel
# odometry, MPU6050, EKF, RPLIDAR and camera - ros/rover_bringup.
#
#     ./run_rover.sh                   everything
#     ./run_rover.sh --phone           also serve the phone driving page on :8080
#     ./run_rover.sh camera:=false     any bringup switch passes through:
#                                      agent, odometry, imu, ekf, lidar, camera
#
# Ctrl+C stops it all. The ESP32 stops the motors by itself 0.9 s later.
#
# --phone publishes /cmd_vel all the time (zero when idle), so do not drive
# with teleop_twist_keyboard or `ros2 topic pub` while it is running.

HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

PHONE=0
ARGS=()
for a in "$@"; do
  if [ "$a" = "--phone" ]; then PHONE=1; else ARGS+=("$a"); fi
done

source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/local_setup.bash
source ~/ros2_ws/install/setup.bash

# A stale daemon hides the ESP32's topics from `ros2 topic list` (BUILD.md 9).
ros2 daemon stop >/dev/null 2>&1

if [ "$PHONE" = 1 ]; then
  python3 "$HERE/scripts/web_teleop.py" &
  PHONE_PID=$!
  trap 'kill $PHONE_PID 2>/dev/null' EXIT
fi

ros2 launch rover_bringup bringup.launch.py \
  agent_script:="$HERE/scripts/start_agent.sh" "${ARGS[@]}"
