# Guide Dog Rover

A skid-steer rover intended to guide a blind person, built around an ESP32 for
real-time motor control and a Raspberry Pi 5 for perception and navigation.

The ESP32 runs micro-ROS and appears on the Pi's ROS 2 Jazzy graph as the node
`rover_base`. The person is guided by a **rigid handle** on the rover, with
vibration motors in the grip telling them which way they are about to be
steered — the way a real guide dog harness works.

---

## Hardware

| | |
|---|---|
| Controller | ESP32 DevKit (ESP-WROOM-32) |
| Motor drivers | 2 × L298N, one per side |
| Motors | 4 × 12 V brushed DC, two per side in parallel |
| Encoders | 2 × LM393 slot optical sensors, one per side |
| Battery | 4S LiPo, 14.8 V |
| Companion computer | Raspberry Pi 5 — LiDAR, IMU, camera, nav2 |

Wiring is in **[WIRING.md](WIRING.md)**, including the pin map and the mistakes
that cost the most time.

---

## Layout

```
rover_microros/     ESP32 firmware — the real one. micro-ROS node.
rover_control/      Standalone WiFi + phone web UI. No ROS. Useful for testing.
rover_base/         Serial keyboard control over USB. Bench testing.
motor_smoketest/    Raw diagnostic — no PWM, no logic. For when nothing works.
encoder_test/       Encoder calibration. Measures ticks per revolution.
drive_and_measure/  Drives motors AND reads encoders together. No ROS needed.

pi_scripts/         Runs on the Raspberry Pi
  start_agent.sh      self-healing micro-ROS agent launcher
  rover_monitor.py    live view of commands vs encoder response

encoder_disc.scad     parametric slotted disc
encoder_bracket.scad  parametric sensor mount

_cane_later/        Haptic handle code — written, not yet wired in
```

---

## Documentation

| | |
|---|---|
| **[BUILD.md](BUILD.md)** | Full handbook — status, setup, running it, troubleshooting. **Start here** |
| **[INTEGRATION.md](INTEGRATION.md)** | ROS topic contract. Read before writing Pi-side nodes |
| [WIRING.md](WIRING.md) | Pin map, power, L298N jumpers |
| [MICROROS.md](MICROROS.md) | Toolchain versions, agent build, the DTR/RTS trap |
| [WIRELESS.md](WIRELESS.md) | Phone control via `rover_control` |
| [CANE.md](CANE.md) | Haptic handle design |

---

## ROS interface

Node **`rover_base`**:

| Direction | Topic | Type |
|---|---|---|
| subscribes | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| publishes | `/ticks_left` | `std_msgs/msg/Int32` |
| publishes | `/ticks_right` | `std_msgs/msg/Int32` |

Motors stop if no `/cmd_vel` arrives within 900 ms. Every way of losing control
looks the same from the rover's side — commands stop arriving — so silence is
treated as "stop", never as "carry on".

Odometry is computed **on the Pi** from the ticks (`ros/rover_bringup`), not on
the ESP32, so the constants can be tuned without reflashing — INTEGRATION.md
decision 2.

## Starting the whole robot on the Pi

`ros/rover_bringup` starts everything with one command: the micro-ROS agent,
the URDF, wheel odometry, the MPU6050, the EKF, the RPLIDAR and the camera.

```bash
ros2 launch rover_bringup bringup.launch.py
```

Setup, the udev rules for the two USB serial devices, and the calibration
tests are in [ros/rover_bringup/README.md](ros/rover_bringup/README.md).

---

## Toolchain — versions are strict

| | |
|---|---|
| ESP32 Arduino core | **2.0.2** — 3.x will not link against the precompiled micro-ROS libs |
| micro_ros_arduino | **v2.0.8-jazzy** |
| ROS 2 | Jazzy on Ubuntu 24.04 |

---

## Quick start

**Flash:** open `rover_microros/rover_microros.ino`, board = ESP32 Dev Module,
upload. All five files in that folder must stay together.

**On the Pi:**

```bash
./pi_scripts/start_agent.sh
```

```bash
ros2 topic list          # /cmd_vel /ticks_left /ticks_right
```

```bash
python3 pi_scripts/rover_monitor.py
```

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

Wheels off the ground for the first run.

**The blue LED on GPIO2 means connected to the agent.** It is the only debug
output the firmware has — the USB port belongs to micro-ROS, so `Serial.print`
would corrupt the protocol.

---

## Current state

Working: drivetrain, encoder reading, micro-ROS link, phone control, failsafes,
agent auto-recovery.

Outstanding:

- **Motor EMI** is the top blocker. It drops the micro-ROS session *and* injects
  false encoder pulses, so counts taken while driving are inflated several-fold.
  100 nF across each motor's terminals is the fix.
- **Top speed still unmeasured** (`MAX_WHEEL_SPEED_MS`). Measured and in:
  `TICKS_PER_REV = 140`, 110 mm wheels, 240 mm track.
- **L298N has a ~23 % deadband.** The firmware now starts its PWM range at the
  deadband so slow commands still move the wheels; a MOSFET driver (BTS7960)
  would remove it properly.
- **Closed-loop speed PID** is written but switched off (`USE_SPEED_PID`) until
  the motor capacitors stop the EMI.
- Pi-side bringup (odometry, IMU, EKF, LiDAR, camera) is written but not yet run
  on the robot. SLAM, Nav2, haptic handle and autonomous behaviour not started.
