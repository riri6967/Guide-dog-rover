#!/bin/bash
# Self-healing micro-ROS agent launcher.
#
# Three problems this works around, all found the hard way:
#
#  1. Opening the serial port asserts DTR/RTS, and on a DevKit the RTS line
#     drives EN through the auto-reset circuit — so the agent's own open()
#     holds the ESP32 in reset. The release has to happen AFTER the agent
#     opens the port, or it simply gets undone.
#
#  2. Motor EMI can reset the ESP32 hard enough that its USB-serial bridge
#     re-enumerates, moving /dev/ttyUSB0 to ttyUSB1. The agent then waits
#     forever on a device node that no longer exists, reporting nothing.
#
#  3. Hard-coding the device means plugging the board into a different port
#     silently breaks everything. Auto-detect instead.
#
#  4. The RPLIDAR is also a /dev/ttyUSB*. With the udev rules from
#     ros/rover_bringup/udev/99-rover.rules installed, /dev/esp32 is used;
#     without them, the first ttyUSB that is not /dev/rplidar.
#
# Usage:  ./start_agent.sh [baud]

BAUD=${1:-115200}

source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/local_setup.bash

cleanup() { kill $HOLDER 2>/dev/null; exit 0; }
trap cleanup INT TERM

while true; do
  if [ -e /dev/esp32 ]; then
    DEV=/dev/esp32
  else
    LIDAR=$(readlink -f /dev/rplidar 2>/dev/null)
    DEV=$(ls /dev/ttyUSB* 2>/dev/null | grep -vx "${LIDAR:-none}" | head -1)
  fi

  if [ -z "$DEV" ]; then
    echo "[$(date +%T)] waiting for ESP32 to appear..."
    sleep 2
    continue
  fi

  echo "[$(date +%T)] starting agent on $DEV @ $BAUD"

  # Release DTR/RTS a few seconds after the agent has opened the port.
  ( sleep 3
    exec python3 -c "
import serial, time
try:
    s = serial.Serial('$DEV', $BAUD)
    s.dtr = False
    s.rts = True; time.sleep(0.1); s.rts = False
    while True: time.sleep(3600)
except Exception:
    pass
" ) &
  HOLDER=$!

  ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b "$BAUD" -v4

  kill $HOLDER 2>/dev/null
  echo "[$(date +%T)] agent exited — restarting in 2s"
  sleep 2
done
