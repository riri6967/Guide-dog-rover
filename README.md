# Guide Dog Rover

A skid-steer rover intended to guide a blind person, built around an ESP32 for
real-time motor control and a Raspberry Pi 5 for perception and navigation.

The ESP32 runs micro-ROS and appears on the Pi's ROS 2 Jazzy graph as the node
`rover_base`. The person is guided by a **rigid handle** on the rover, with
vibration motors in the grip telling them which way they are about to be
steered — the way a real guide dog harness works.

---

## Where everything is

Each top-level folder is one part of the robot. The first two are all you need
to make it drive.

```
esp32/                  ESP32 firmware — flashed from the Arduino IDE on Windows
  rover_microros/         THE firmware: motors + encoders, micro-ROS node
  rover_control/          standalone WiFi + phone web UI, no ROS (drivetrain testing)
  tests/                  bench sketches: rover_base, motor_smoketest,
                          encoder_test, drive_and_measure, vibrator_test

raspberry_pi/           everything that runs on the Pi 5 — START HERE for the Pi
  README.md               step-by-step: wiring to the Pi, setup, run, checks
  setup_pi.sh             one-time setup (ROS 2, drivers, agent, workspace, udev)
  run_rover.sh            starts the whole robot
  scripts/                start_agent.sh, web_teleop.py (phone), rover_monitor.py
  ros/rover_bringup/      LiDAR, camera, IMU, wheel odometry, EKF launch + nodes
  ros/rover_description/  URDF, meshes and TF tree of the rover

haptic_handle/          the guide handle: haptics.cpp/.h (not wired in yet),
                        CadQuery model (cad/), drawings, grip vibration sim

cad/                    printed parts for the encoders: shaft extender + disc
                        (CadQuery), OpenSCAD disc and bracket

simulation/             Gazebo / Nav2 / DRL simulation and browser simulators

docs/                   the handbook and reference notes
```

### Which computer runs what

```
 phone / keyboard ──/cmd_vel──►  Raspberry Pi 5  ──USB──►  ESP32  ──►  2 × L298N  ──►  4 motors
                                 (ROS 2 Jazzy)   ◄──────   (micro-ROS) ◄── 2 × LM393 encoders
                                    ▲   ▲   ▲      ticks
                         RPLIDAR A1 ┘   │   └ Pi Camera 3
                         (USB)          │     (ribbon)
                                  MPU6050 (Pi I2C)
```

| Runs on the ESP32 (`esp32/`) | Runs on the Pi (`raspberry_pi/`) |
|---|---|
| motor PWM and direction | RPLIDAR → `/scan` |
| encoder tick counting → `/ticks_left`, `/ticks_right` | Pi Camera → `/camera/image_raw` |
| `/cmd_vel` → wheel speeds | MPU6050 → `/imu/data_raw` |
| 900 ms command-timeout failsafe | ticks → `/wheel/odom`; EKF → `/odometry/filtered` |

---

## Quick start

1. **Flash the ESP32** — `esp32/rover_microros/rover_microros.ino`, ESP32 core
   **2.0.2**, micro_ros_arduino **v2.0.8-jazzy**. Details: [docs/MICROROS.md](docs/MICROROS.md) Part A.
2. **On the Pi** (Ubuntu 24.04), clone into `~/Guide-dog-rover` and set up once:

   ```bash
   git clone https://github.com/riri6967/Guide-dog-rover.git ~/Guide-dog-rover
   ```

   ```bash
   cd ~/Guide-dog-rover/raspberry_pi && bash setup_pi.sh
   ```

3. **Run:**

   ```bash
   ~/Guide-dog-rover/raspberry_pi/run_rover.sh
   ```

Full steps, sensor wiring and checks: **[raspberry_pi/README.md](raspberry_pi/README.md)**.
Wheels off the ground for the first run.

**The blue LED on GPIO2 means connected to the agent.** It is the only debug
output the firmware has — the USB port belongs to micro-ROS, so `Serial.print`
would corrupt the protocol.

---

## Documentation

| | |
|---|---|
| **[raspberry_pi/README.md](raspberry_pi/README.md)** | Getting the Pi running, start to finish |
| **[docs/BUILD.md](docs/BUILD.md)** | Full handbook — status, setup, running it, troubleshooting |
| **[docs/INTEGRATION.md](docs/INTEGRATION.md)** | ESP32 ↔ Pi ROS topic contract |
| [docs/WIRING.md](docs/WIRING.md) | Pin map, power, L298N jumpers |
| [docs/MICROROS.md](docs/MICROROS.md) | Toolchain versions, agent build, the DTR/RTS trap |
| [docs/WIRELESS.md](docs/WIRELESS.md) | Phone control via `esp32/rover_control` (no Pi) |
| [docs/CANE.md](docs/CANE.md) | Haptic handle design |
| [raspberry_pi/ros/rover_bringup/README.md](raspberry_pi/ros/rover_bringup/README.md) | Calibration checks on the real robot |
| [simulation/README.md](simulation/README.md) | Simulation work |

---

## Hardware

| | |
|---|---|
| Controller | ESP32 DevKit (ESP-WROOM-32) |
| Motor drivers | 2 × L298N, one per side |
| Motors | 4 × 12 V brushed DC, two per side in parallel |
| Encoders | 2 × LM393 slot optical sensors, one per side, 32-slot discs |
| Wheels / track | 110 mm wheels, 290 mm track (240 + 2 × 25 mm shaft extenders) |
| Battery | 4S LiPo, 14.8 V |
| Companion computer | Raspberry Pi 5 — RPLIDAR A1, MPU6050, Pi Camera 3 |

The geometry (wheel 110 mm, track 290 mm, 32 ticks/turn) lives in three places
that must agree: `esp32/rover_microros/rover_microros.ino`,
`raspberry_pi/ros/rover_bringup/rover_bringup/wheel_odometry.py` and
`raspberry_pi/ros/rover_description/urdf/rover.urdf.xacro`.

---

## ROS interface

Node **`rover_base`** (the ESP32):

| Direction | Topic | Type |
|---|---|---|
| subscribes | `/cmd_vel` | `geometry_msgs/msg/Twist` |
| publishes | `/ticks_left` | `std_msgs/msg/Int32` |
| publishes | `/ticks_right` | `std_msgs/msg/Int32` |

Motors stop if no `/cmd_vel` arrives within 900 ms. Every way of losing control
looks the same from the rover's side — commands stop arriving — so silence is
treated as "stop", never as "carry on".

Odometry is computed **on the Pi** from the ticks (`raspberry_pi/ros/rover_bringup`),
not on the ESP32, so the constants can be tuned without reflashing —
docs/INTEGRATION.md decision 2.

---

## Toolchain — versions are strict

| | |
|---|---|
| ESP32 Arduino core | **2.0.2** — 3.x will not link against the precompiled micro-ROS libs |
| micro_ros_arduino | **v2.0.8-jazzy** |
| ROS 2 | Jazzy on Ubuntu 24.04 |

---

## Current state

Working: drivetrain, encoder reading, micro-ROS link, phone control, failsafes,
agent auto-recovery.

Outstanding:

- **Motor EMI** is the top blocker. It drops the micro-ROS session *and* injects
  false encoder pulses, so counts taken while driving are inflated several-fold.
  100 nF across each motor's terminals is the fix.
- **Top speed still unmeasured** (`MAX_WHEEL_SPEED_MS`). Confirm 32 ticks per
  wheel turn with `esp32/tests/encoder_test` once the extender discs are fitted.
- **L298N has a ~23 % deadband.** The firmware now starts its PWM range at the
  deadband so slow commands still move the wheels; a MOSFET driver (BTS7960)
  would remove it properly.
- **Closed-loop speed PID** is written but switched off (`USE_SPEED_PID`) until
  the motor capacitors stop the EMI.
- Pi-side bringup (odometry, IMU, EKF, LiDAR, camera) is written but not yet run
  on the robot. SLAM, Nav2, haptic handle and autonomous behaviour not started.
