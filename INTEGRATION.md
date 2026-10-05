# ESP32 ↔ Raspberry Pi integration contract

What the ESP32 firmware (`rover_microros`) puts on the ROS 2 graph, and the two
decisions that need settling before the Pi-side nodes connect to it.

---

## The interface

Node name: **`rover_base`**

### Subscribes

| Topic | Type | Notes |
|---|---|---|
| `/cmd_vel` | `geometry_msgs/msg/Twist` | `linear.x` m/s, `angular.z` rad/s. Motors stop if nothing arrives for 900 ms |

### Publishes

| Topic | Type | Rate | Notes |
|---|---|---|---|
| `/ticks_left` | `std_msgs/msg/Int32` | 20 Hz | cumulative, signed |
| `/ticks_right` | `std_msgs/msg/Int32` | 20 Hz | cumulative, signed |
| `/odom` | `nav_msgs/msg/Odometry` | 20 Hz | **see decision 2 — probably disable this** |

---

## Decision 1 — two encoders, not four

The Pi-side wheel odometry node expects `[FL, FR, RL, RR]`. The rover has
**two** optical sensors, one per side.

That is not a shortcoming. Both wheels on a side are driven in parallel from one
L298N channel and are mechanically coupled through the chassis and the ground —
they turn at the same rate. A second sensor per side would produce a duplicate
reading, not new information.

Three ways to resolve it, in order of preference:

**A. Pi node accepts two inputs.** Cleanest. Differential-drive odometry needs
left-side and right-side travel; four wheels do not change that maths.

**B. ESP32 publishes four topics**, with `FL == RL` and `FR == RR`. Honest
about where the data comes from, requires no change on the Pi. Say the word and
I will add it — about ten lines.

**C. Pi node duplicates on receipt** — subscribe to two, feed four. Same as B
but the fiction lives on the Pi side.

**Not recommended: fitting four sensors.** It would give two pairs of readings
that should always agree, and nothing useful to do when they disagree. Spend
that effort on quadrature instead (see below).

---

## Decision 2 — who computes odometry

Right now **both** ends compute it, which must not reach the EKF twice.

| | |
|---|---|
| ESP32 publishes `/odom` | integrated on the microcontroller from its own ticks |
| Pi node publishes odometry | computed from the same ticks, feeds `robot_localization` |

**Recommendation: the Pi wins, and the ESP32's `/odom` gets disabled.**

Reasons:

- The EKF already fuses wheel odometry with the IMU. Adding a second,
  unfiltered odometry source would double-count the same wheel data.
- Calibration constants are easier to tune in a Python node than by reflashing
  firmware.
- The ESP32 should do what only it can do — real-time motor control and
  interrupt-driven tick counting — and leave estimation to the Pi.

Disabling it is a few lines; the tick publishers stay. Ask and I will cut it.

Until then, **do not subscribe the EKF to `/odom`** from the ESP32.

---

## Calibration — unmeasured so far

Top of `rover_microros.ino`:

```cpp
const float TICKS_PER_REV       = 20.0f;    // <-- MEASURE
const float WHEEL_DIAMETER_M    = 0.065f;   // <-- MEASURE
const float TRACK_WIDTH_M       = 0.200f;   // <-- MEASURE
const float MAX_WHEEL_SPEED_MS  = 0.55f;    // <-- MEASURE
```

These are placeholders. They affect:

- `/odom` from the ESP32 — wrong until measured (another reason to disable it)
- the `cmd_vel` m/s → PWM conversion, so commanded speeds are only approximate

**`/ticks_left` and `/ticks_right` are unaffected.** They are raw counts and
need no calibration, which is why the Pi-side node should work from those.

Measure `TICKS_PER_REV` with `encoder_test.ino`: press `c`, turn one wheel
exactly ten revolutions by hand, divide the count by ten. Do not trust the slot
count printed on the disc.

---

## Direction sensing caveat

The sensors are **single-channel**, so they cannot tell which way a wheel turned.
The firmware signs each tick by the direction it last commanded.

That is correct unless something turns a wheel against the motor — being pushed
by hand, rolling backwards down a slope, a wheel dragged by an obstacle. In
those cases ticks are counted with the wrong sign and odometry drifts.

The fix is quadrature: a second sensor per wheel offset by a quarter slot-pitch,
so the two signals reveal direction. Worth doing if EKF drift turns out to
correlate with pushing the rover around.

---

## Known EKF-relevant issues

**Encoder discs are not fitted yet.** The sensors work on the bench but have no
disc to read, so ticks only change when the gate is triggered by hand. Real
wheel rotation produces nothing until discs are mounted.

**Electrical noise.** Brushed motors throw EMI that corrupts the USB serial
link; the micro-ROS session drops and reconnects under load. 100 nF capacitors
across each motor's terminals is the fix. Until then expect gaps in the tick
stream, which will look like odometry glitches.

**The L298N has a ~23 % deadband.** Commanded velocities below roughly a quarter
of full scale produce no movement at all. Relevant to nav2 tuning — the
controller will ask for small corrections and get nothing. Replacing the driver
with a MOSFET H-bridge (BTS7960) removes this.

---

## Flashing the firmware

Requires a specific toolchain; newer versions will not link.

1. **ESP32 Arduino core 2.0.2** — Boards Manager, select the version explicitly.
   Core 3.x fails against the precompiled micro-ROS libraries.
2. **micro_ros_arduino v2.0.8-jazzy** —
   `https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-jazzy.zip`
   installed via Sketch → Include Library → Add .ZIP Library.
3. Board: **ESP32 Dev Module**. Open `rover_microros/rover_microros.ino`, upload.

All five files in that folder must stay together — the IDE compiles the whole
directory.

---

## Running the agent

There is **no apt package** for `micro_ros_agent` on any distro. The snap is
amd64-only. The Docker image exists for arm64 but its entrypoint exits silently
on Ubuntu for Pi. Build from source — see `MICROROS.md`.

### The trap that costs an afternoon

**Opening the serial port holds the ESP32 in reset.** The USB-serial bridge's
RTS line drives the EN pin through the auto-reset circuit, so the agent's own
`open()` puts the board into reset and leaves it there. The agent reports
`running... fd: N` and then nothing, forever, with an empty `ros2 topic list`.

`start_agent.sh` works around it by releasing DTR/RTS **after** the agent has
opened the port. Order matters — doing it before is undone by the agent.

### Other things that look like failures

| Symptom | Cause |
|---|---|
| Topics missing but ESP32's blue LED is lit | stale `ros2` daemon. `ros2 daemon stop`, then run any publisher to force rediscovery |
| Agent waits forever on a port that exists | ESP32 re-enumerated to a different `ttyUSB` number after a reset |
| Everything dies a few seconds into driving | battery protection tripping under motor load |

**Blue LED on GPIO2 = connected to the agent.** It is the only debug output the
firmware has, since the USB port belongs to micro-ROS and `Serial.print` would
corrupt the protocol.

---

## Suggested split

| ESP32 | Raspberry Pi |
|---|---|
| motor PWM and direction | LiDAR, IMU, camera |
| encoder interrupt counting | odometry from ticks |
| `/cmd_vel` → wheel speeds | `robot_localization` EKF |
| command-timeout failsafe | SLAM, nav2 |
| *(later)* closed-loop speed PID | *(later)* guide-dog behaviour |

The dividing line: the ESP32 does what needs hard real-time guarantees and
direct hardware access. Everything else belongs on the Pi, where it is easier
to write, tune and debug.
