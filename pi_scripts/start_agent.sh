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
# Usage:  ./start_agent.sh [baud]

BAUD=${1:-115200}

source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/local_setup.bash

cleanup() { kill $HOLDER 2>/dev/null; exit 0; }
trap cleanup INT TERM

while true; do
  DEV=$(ls /dev/ttyUSB* 2>/dev/null | head -1)

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
