# Guide Dog Rover — build handbook

Everything built so far, what has actually been tested on hardware, what is
still outstanding, and the steps to reproduce the whole thing from nothing.

---

## 1. What this is

A skid-steer rover intended to guide a blind person. An **ESP32** handles
real-time motor control and encoder counting; a **Raspberry Pi** handles
perception and navigation. They talk over USB using micro-ROS, so the ESP32
appears on the Pi's ROS 2 graph as an ordinary node.

```
         ┌──────────────────── Raspberry Pi (ROS 2 Jazzy) ────────────────────┐
         │                                                                     │
         │   LiDAR ──► /scan                                                   │
         │   IMU   ──► /imu/data_raw        robot_localization EKF             │
         │   camera                              ▲         │                   │
         │                                       │         ▼                   │
         │   micro-ROS agent ◄──USB──┐      wheel odom    nav2                 │
         │          ▲                 │                     │                  │
         └──────────┼─────────────────┼─────────────────────┼──────────────────┘
                    │                 │                     │
                    │            /ticks_left           /cmd_vel
                    │            /ticks_right               │
                    │                 │                     │
         ┌──────────┴─────────────────┴─────────────────────▼──────────────────┐
         │  ESP32  (node: rover_base)                                           │
         │    • /cmd_vel → differential drive → PWM                             │
         │    • encoder interrupts → tick counts                                │
         │    • 900 ms command-timeout failsafe                                 │
         │    • automatic agent reconnection                                    │
         └──────────┬───────────────────────────────────────────────────────────┘
                    │
          2 × L298N ──► 4 × 12 V motors
          2 × LM393 slot sensors ◄── encoder discs
```

**Division of labour:** the ESP32 does what needs hard real-time guarantees and
direct hardware access. Everything else belongs on the Pi, where it is far
easier to write, tune and debug.

---

## 2. Status — what is proven, and what is not

### Verified on real hardware

| | Evidence |
|---|---|
| Motors driven from ESP32 | all four turn, directions calibrated |
| Two L298N boards, one per side | both channels confirmed |
| Encoder sensors counting | both channels independently, correct pin mapping |
| micro-ROS session | `client_key` exchange, agent log |
| Messages reaching DDS | `Odometry` writes in the agent log (the old on-board `/odom`, since moved to the Pi) |
| `/ticks_left` `/ticks_right` | live counts visible via `ros2 topic echo` |
| `/cmd_vel` driving motors | motors respond to `ros2 topic pub` |
| Agent auto-recovery | reconnects after link loss |
| Phone web control (`rover_control`) | drove the rover over WiFi |
| TFT status display | live system stats on a 2.4" ILI9341 |
| **Encoder discs on the shaft** | printed slots, sensors read real rotation |
| **`TICKS_PER_REV` measured** | **140** — 1392 and 1402 pulses over ten hand-turned wheel revolutions |
| Motors + encoders together | `drive_and_measure` drives and reports RPM in one sketch |

### Built but not yet tested on hardware

- **Closed-loop speed PID** — written in `rover_microros` and compiles, but off
  (`USE_SPEED_PID = false`) until the motor capacitors stop the EMI.
- **Pi bringup** (`ros/rover_bringup`) — wheel odometry, MPU6050, EKF, RPLIDAR,
  camera and one launch file for all of it. Not yet run on the robot.
- **Haptic handle** (`_cane_later/`) — code written, hardware not built.

### Known outstanding

| | Impact |
|---|---|
| **Motor EMI — now the top blocker** | drops the micro-ROS session *and* injects false encoder pulses. Counts taken while driving are inflated several-fold and cannot be trusted. Fix: 100 nF across each motor |
| **Two calibration constants still unmeasured** | track width (firmware 0.200 m vs CAD 0.2146 m) and top speed. The 65 mm wheel diameter is in. Raw tick topics unaffected |
| **L298N ~23 % deadband** | small velocity commands produce no movement. Matters for nav2 tuning |
| **Battery protection trips under load** | fixed by moving to a 4S LiPo; still check the Pi's supply with `vcgencmd get_throttled` (see WIRING.md) |

---

## 3. Hardware

| Part | Notes |
|---|---|
| ESP32 DevKit (ESP-WROOM-32) | 30-pin board, labels read `D13` / `G13` — both mean GPIO 13 |
| 2 × L298N | one per side. Each motor gets its own H-bridge channel |
| 4 × 12 V brushed DC gearmotors | two per side, wired in parallel |
| 2 × LM393 slot optical sensors | one per side, powered at **3.3 V** |
| 4S LiPo, 14.8 V | main leads to the L298Ns; balance plug to a 5 V USB-C module for the Pi — see WIRING.md |
| Raspberry Pi 5 | LiDAR, IMU, camera, nav2 |

### Pin map

| ESP32 | Connects to |
|---|---|
| `D13` | right board `ENA` + `ENB` |
| `D26` | right board `IN1` + `IN3` |
| `D27` | right board `IN2` + `IN4` |
| `D14` | left board `ENA` + `ENB` |
| `D25` | left board `IN1` + `IN3` |
| `D33` | left board `IN2` + `IN4` |
| `D18` | left encoder `OUT` |
| `D19` | right encoder `OUT` |
| `GND` | L298N `GND` — **mandatory**, both boards |
| `3V3` | both encoder modules `VCC` |

Both channels of each board are tied to the same GPIO because the two wheels on
a side are mechanically coupled — they must turn together. Tying the *inputs*
does not tie the *outputs*: each motor still gets its own H-bridge and its own
2 A budget.

### L298N jumpers

| | |
|---|---|
| `ENA` cap | **off** both boards — the ESP32 drives those pins |
| `ENB` cap | **off** both boards |
| `5V-EN` cap | **on** both boards — each powers its own logic |

Never run a wire between the two boards' `5V` pins; that ties two regulators
together and they fight. With `5V-EN` fitted, the `5V` screw terminal stays
empty.

Full detail including the mistakes that cost the most time: **WIRING.md**

---

## 4. Software — the sketches

| Folder | Purpose |
|---|---|
| `rover_microros/` | **The real firmware.** micro-ROS node `rover_base` |
| `rover_control/` | Standalone WiFi + phone web UI. No ROS. Good for testing the drivetrain alone |
| `rover_base/` | Serial keyboard control over USB. Bench testing |
| `motor_smoketest/` | Raw `digitalWrite` diagnostic — no PWM, no logic. For when nothing works |
| `encoder_test/` | Measures ticks per revolution |
| `drive_and_measure/` | **Drives the motors and reports encoder RPM together.** No ROS, no Pi — the quickest way to prove the drivetrain and sensors cooperate, and where the calibration constants come from |

Each is a self-contained Arduino sketch. `rover_microros` and `rover_control`
have several files that **must stay in their folder together** — the IDE
compiles the whole directory.

---

## 5. Setup — Windows / Arduino side

Versions are strict. Newer will not work.

### 5.1 ESP32 core 2.0.2

Boards Manager → `esp32 by Espressif Systems` → version dropdown → **2.0.2**.

Core 3.x will not link against the precompiled micro-ROS libraries. The rover
sketches handle both cores via an `ESP_ARDUINO_VERSION_MAJOR` guard, so nothing
else breaks on the downgrade.

### 5.2 micro_ros_arduino v2.0.8-jazzy

Download:

```
https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-jazzy.zip
```

**Sketch → Include Library → Add .ZIP Library.** Do not unzip it first; it is
not in Library Manager.

### 5.3 Flash

Open `rover_microros/rover_microros.ino`, board = **ESP32 Dev Module**, upload.

A successful build reports roughly **418 KB (31 %)**. A bare sketch without
micro-ROS is ~230 KB — that extra 190 KB is the library, statically linked in.

Afterwards the USB port is the micro-ROS transport. The Serial Monitor shows
binary garbage; that is the protocol working, not a fault.

---

## 6. Setup — Raspberry Pi side

### 6.1 ROS 2 Jazzy

Needs **Ubuntu 24.04**. "Jazzy" is a ROS distribution, not an operating system.

```bash
sudo apt install -y software-properties-common curl && sudo add-apt-repository universe -y
```

```bash
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg
```

```bash
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list
```

```bash
sudo apt update && sudo apt install -y ros-jazzy-ros-base
```

`ros-base`, not `ros-desktop` — desktop adds ~3 GB of RViz and demos you do not
need on a headless robot.

> **If `apt` reports unmet dependencies on `liblz4-dev` or `libzstd-dev`**, the
> Pi image shipped with `noble-updates` disabled. Check
> `/etc/apt/sources.list.d/ubuntu.sources` — the first `Suites:` line should
> read `noble noble-updates noble-backports`. Fix it, `apt update`,
> `apt upgrade`, retry.

### 6.2 Build the micro-ROS agent

There is **no apt package** for `micro_ros_agent` on any distro. The snap is
amd64-only. The Docker image exists for arm64 but its entrypoint exits silently
on Ubuntu for Pi. Build from source.

```bash
sudo apt install -y python3-rosdep python3-colcon-common-extensions git build-essential python3-serial
```

```bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

```bash
sudo rosdep init 2>/dev/null; rosdep update
source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -y
```

```bash
cd ~/uros_ws && colcon build && source install/local_setup.bash
ros2 run micro_ros_setup create_agent_ws.sh
ros2 run micro_ros_setup build_agent.sh
```

The last step takes 3–5 minutes. "1 package had stderr output" is git clone
progress, not an error.

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/uros_ws/install/local_setup.bash" >> ~/.bashrc
```

### 6.3 Serial permission

```bash
sudo usermod -aG dialout $USER
```

Log out and back in. Over SSH that means `exit` and reconnect. Verify with
`groups`.

### 6.4 The agent launcher

Copy `pi_scripts/start_agent.sh` to the Pi and `chmod +x` it. **Do not run the
bare `ros2 run micro_ros_agent` command** — see §9.

---

## 7. Running it

### Step by step

**1.** Plug the ESP32's USB into the **Pi**. Not your laptop — SSH does not
carry USB devices.

```bash
ls /dev/ttyUSB*
```

**2.** Start the agent. Leave this window alone.

```bash
~/start_agent.sh
```

**3.** Check the ESP32's **blue LED on GPIO 2**. Lit means connected to the
agent. It is the only debug output the firmware has, since the USB port belongs
to micro-ROS and `Serial.print` would corrupt the protocol.

**4.** Second terminal:

```bash
ros2 topic list
```

Expect `/cmd_vel`, `/ticks_left`, `/ticks_right`. (Odometry comes from the Pi —
see `ros/rover_bringup`.)

> If the blue LED is lit but topics are missing, it is the stale `ros2` daemon,
> not the rover. See §9.

**5.** Watch commands and encoder response together:

```bash
python3 pi_scripts/rover_monitor.py
```

**6.** Drive it. **Wheels off the ground for the first run.**

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

`topic pub` with `-r 10` publishes continuously, so the failsafe never fires.
`teleop_twist_keyboard` only publishes on keystrokes, which makes it stutter.

Turning:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{angular: {z: 0.8}}" -r 10
```

### What a working run looks like

```
cmd +0.30 m/s +0.00 rad/s | ticks L   1247 (+38)  R   1193 (+41) | MOVING | odom ...
```

Command non-zero **and** both tick deltas climbing means the full loop works.

---

## 8. ROS interface

Node **`rover_base`**.

| Direction | Topic | Type | Rate |
|---|---|---|---|
| subscribes | `/cmd_vel` | `geometry_msgs/msg/Twist` | — |
| publishes | `/ticks_left` | `std_msgs/msg/Int32` | 20 Hz |
| publishes | `/ticks_right` | `std_msgs/msg/Int32` | 20 Hz |

**Failsafe:** motors stop if no `/cmd_vel` arrives within 900 ms. Every way of
losing control — agent killed, cable pulled, Pi crashed, teleop closed — looks
identical from the rover's side: commands stop arriving. Silence is treated as
"stop", never as "carry on".

**Odometry is computed on the Pi** from the ticks, by `ros/rover_bringup`'s
`wheel_odometry` — both integration decisions are settled in **INTEGRATION.md**.

### Calibration

Top of `rover_microros.ino`:

```cpp
const float TICKS_PER_REV       = 140.0f;   // measured
const float WHEEL_DIAMETER_M    = 0.065f;   // the 65 mm wheels fitted
const float TRACK_WIDTH_M       = 0.200f;   // <-- MEASURE
const float MAX_WHEEL_SPEED_MS  = 0.55f;    // <-- MEASURE
```

| Constant | How to get it |
|---|---|
| `TICKS_PER_REV` | flash `encoder_test`, press `c`, turn one wheel exactly ten revolutions by hand, divide by ten. **Do not trust the slot count printed on the disc** |
| `WHEEL_DIAMETER_M` | ruler, in metres |
| `TRACK_WIDTH_M` | centre of left wheel to centre of right wheel |
| `MAX_WHEEL_SPEED_MS` | drive flat out over a measured 2 m, time it |

On the ESP32 these set the m/s → PWM conversion and the speed loop. The Pi's
`wheel_odometry` node takes the same values as parameters — keep them in step.
**Raw tick topics are unaffected** — they need no calibration.

---

## 9. Troubleshooting

These cost real time to find and none of them appear in upstream documentation.

### Agent prints `running... fd: N` then nothing, forever

**Opening the serial port holds the ESP32 in reset.** The USB-serial bridge's
RTS line drives the EN pin through the auto-reset circuit, so the agent's own
`open()` puts the board into reset and leaves it there. Pressing EN does
nothing — the circuit is already holding it down.

`start_agent.sh` releases DTR/RTS **after** the agent opens the port. Order
matters: doing it first gets undone by the agent.

### Agent waits forever on a port that exists

Motor EMI can reset the ESP32 hard enough that the USB-serial bridge
**re-enumerates**, moving `/dev/ttyUSB0` to `ttyUSB1`. The agent sits on a dead
device node. `start_agent.sh` re-detects and restarts automatically.

### Topics missing but the blue LED is lit

Stale `ros2` daemon. It caches the discovery graph and can miss a node that
appears while it is idle.

```bash
ros2 topic pub /kick std_msgs/msg/String "{data: x}" -r 1
```

Leave that running, check `ros2 topic list` in another window — everything
appears — then Ctrl+C the kick.

**Rule: blue LED on = the rover is working**, regardless of what `topic list`
says.

### Everything dies a few seconds into driving

Battery protection tripping. A 2S pack cannot supply four 12 V motors, and
7.4 V minus the L298N's ~2 V drop leaves 5.4 V for motors rated at 12 V — they
strain, draw more current, and trip the protection sooner. Use the 4S pack.

Nothing getting warm rules out thermal shutdown.

### Session drops and reconnects under load

Brushed motor EMI corrupting the serial link. Fix: **100 nF ceramic capacitor
across each motor's terminals**, soldered at the motor. Plus a second GND wire
between ESP32 and L298N, and keep motor leads away from signal wires.

### Motors do not move but commands arrive

Check `cmd` in `rover_monitor.py`. If it reads `+0.00`, nothing is publishing —
`teleop_twist_keyboard` needs its terminal focused, as it reads keys directly.

If `cmd` is non-zero and ticks stay flat, it is voltage or a driver fault.

### Serial stream has no `0x7E` frames

micro-XRCE-DDS frames every message with `0x7E`. A stream with none of them is
not micro-ROS. Either the wrong sketch is on the board, or you are reading at
the wrong baud rate — a perfectly repeating byte pattern means the latter.

### Two identical ESP32s

Cost several hours, twice. One board runs `rover_microros`, the other usually
has a bench sketch on it; plugged into the Pi they look identical, the agent
starts, and nothing works. **Label them.**

The blue LED is the tell: only `rover_microros` drives GPIO 2, so a board that
never lights it is running something else.

### Encoder counts implausibly high while driving

Motor EMI false-triggering the LM393 comparators. If the counts imply a wheel
speed of thousands of RPM, they include noise. Hand-turned calibration with
the motors off is unaffected — which is why `TICKS_PER_REV` is trustworthy and
anything measured under power is not.

### One side dead

In order: `ENA`/`ENB` jumper caps must be **off**; the EN wire must be on the
**inner** pin of each 2-pin header (the outer is +5 V); each signal wire must
fan out to **both** of its pins. Then swap the signal trios between boards — if
the fault follows the wires it is the ESP32 pins, if it stays it is the board.

---

## 10. What comes next

In the order that unblocks the most.

**1. Capacitors on the motors.** 100 nF across each motor's terminals, soldered
at the motor. Plus a second GND wire and keeping encoder leads away from motor
leads. This is now the top blocker: EMI corrupts both the micro-ROS link and the
encoder counts, so nothing measured under power is trustworthy until it is done.

**2. Take the remaining three measurements.** Wheel diameter and track width
with a ruler; top speed from the `m` sweep in `drive_and_measure` — but only
after the capacitors, or the sweep measures noise.

**4. Turn on the speed PID.** Written and compiling; set `USE_SPEED_PID = true`
after the capacitors, then tune `PID_KP` / `PID_KI` (see
`ros/rover_bringup/README.md`). Gives matched wheel speeds, constant speed
under varying load, and usable low-speed control.

**5. Replace the L298N.** Two BTS7960 boards, ~₹700. MOSFET, 0.1 V drop, 43 A,
no thermal shutdown, no deadband. The code change is one function.

**6. The guide-dog half.** Rigid handle with four vibration motors in the grip,
telling the person which way they are about to be steered. Code written in
`_cane_later/`; design in **CANE.md**.
