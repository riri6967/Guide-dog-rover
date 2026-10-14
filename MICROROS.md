# micro-ROS: ESP32 ↔ Raspberry Pi 5

Target: ROS 2 **Jazzy** on Ubuntu 24.04, Raspberry Pi 5. ESP32 over USB serial.
Library on the ESP32: `micro_ros_arduino` **v2.0.8-jazzy**.

This is the procedure that actually worked, rebuilt from a fresh Jazzy install.
Two parts: the Windows/Arduino side (done once) and the Pi side (redo after any
OS reinstall).

---

## Part A — Windows / Arduino IDE

Only needed once, or after reinstalling Arduino IDE.

### A1. ESP32 core must be 2.0.2

`micro_ros_arduino` ships precompiled static libraries built against **ESP32
Arduino core v2.0.2**. Core 3.x will not link.

Boards Manager → `esp32 by Espressif Systems` → version dropdown → **2.0.2** →
Install.

The rover sketches handle both cores via an `ESP_ARDUINO_VERSION_MAJOR` guard,
so nothing else breaks on the downgrade.

### A2. Install the library

Download:

```
https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-jazzy.zip
```

**Sketch → Include Library → Add .ZIP Library...** → pick the zip. Do not unzip
it first; it is not in Library Manager.

### A3. Flash the rover

Open `rover_microros/rover_microros.ino`, confirm board = ESP32 Dev Module,
upload.

Afterwards the USB port is the micro-ROS transport, not a console. The Serial
Monitor will show binary garbage with `XRCE` visible in it — that is the
protocol handshake and means it is working.

---

## Part B — Raspberry Pi 5

Redo this whole part after an OS reinstall.

### B1. Confirm the distro

```bash
echo $ROS_DISTRO
```

Must print `jazzy`. (`ros2 --version` is not a valid command — it has no such
flag.)

### B2. Serial port permission

```bash
sudo usermod -aG dialout $USER
```

Then **log out and back in** — over SSH that means `exit` and reconnect. Check
with `groups` that `dialout` is listed.

### B3. Build the agent from source

There is **no apt package** for `micro_ros_agent` on any distro, and the snap
is amd64-only so it will not install on a Pi. The Docker image exists for arm64
but its entrypoint silently exits on this setup. Source build is the reliable
route.

```bash
sudo apt update && sudo apt install -y python3-rosdep python3-colcon-common-extensions git build-essential
```

```bash
mkdir -p ~/uros_ws/src && cd ~/uros_ws
```

```bash
git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
```

```bash
sudo rosdep init 2>/dev/null; rosdep update
```

```bash
source /opt/ros/jazzy/setup.bash && rosdep install --from-paths src --ignore-src -y
```

```bash
cd ~/uros_ws && colcon build && source install/local_setup.bash
```

```bash
ros2 run micro_ros_setup create_agent_ws.sh
```

```bash
ros2 run micro_ros_setup build_agent.sh
```

That last step takes 3–5 minutes and prints `git clone` progress on stderr —
which colcon reports as "1 package had stderr output". That is not an error.

### B4. Source automatically

```bash
echo "source /opt/ros/jazzy/setup.bash" >> ~/.bashrc
echo "source ~/uros_ws/install/local_setup.bash" >> ~/.bashrc
```

### B5. Reclaim the build space

```bash
cd ~/uros_ws && rm -rf build log
```

Saves 1–2 GB. `install/` is what actually runs.

---

## ⚠️ The thing that will waste your afternoon

**Opening the serial port holds the ESP32 in reset.**

On a DevKit, the USB-serial chip's RTS line drives the EN pin through the
auto-reset transistors. Asserted RTS = EN held low = the chip never boots.
Every normal way of opening the port — the agent, `cat`, `minicom` — asserts
it, and from that moment the ESP32 is dead. Pressing EN does nothing, because
the circuit is already holding it down.

The symptom is a perfectly healthy-looking agent that reports
`TermiosAgentLinux.cpp | init | running... | fd: 10` and then nothing, forever,
with an empty `ros2 topic list`.

Windows drivers handle DTR/RTS differently, which is why the same board works
fine there.

### The fix

Deassert both lines and hold them deasserted while the agent runs. The script
below does it.

---

## Part C — Starting the agent

Create this once:

```bash
cat > ~/start_agent.sh <<'EOF'
#!/bin/bash
DEV=${1:-/dev/ttyUSB0}
BAUD=${2:-115200}

# Hold DTR/RTS deasserted — otherwise they keep the ESP32 in reset.
python3 -c "
import serial, time
s = serial.Serial('$DEV', $BAUD)
s.dtr = False
s.rts = True; time.sleep(0.1); s.rts = False
while True: time.sleep(3600)
" &
HOLDER=$!
trap "kill $HOLDER 2>/dev/null" EXIT

sleep 1
source /opt/ros/jazzy/setup.bash
source ~/uros_ws/install/local_setup.bash
ros2 run micro_ros_agent micro_ros_agent serial --dev "$DEV" -b "$BAUD" -v6
EOF
chmod +x ~/start_agent.sh
```

Needs pyserial:

```bash
sudo apt install -y python3-serial
```

Then every session is one command:

```bash
~/start_agent.sh
```

---

## Verifying

**The ESP32's USB cable must be plugged into the Pi**, not your laptop. SSH does
not carry USB devices.

```bash
ls /dev/ttyUSB*
```

Expect `/dev/ttyUSB0`. Nothing listed means the cable is in the wrong machine,
or it is a charge-only cable.

Start the agent, then in a second terminal:

```bash
ros2 topic list
```

```
/cmd_vel
/parameter_events
/rosout
/ticks_left
/ticks_right
```

```bash
ros2 topic echo /ticks_left
```

Publishing at 20 Hz. Spin a wheel by hand and the count moves. (Odometry is
computed on the Pi from these counts, by `ros/rover_bringup`.)

The ESP32's onboard LED lights when connected to the agent — the only debug
output available, since the USB port belongs to micro-ROS.

---

## Driving it

```bash
sudo apt install -y ros-jazzy-teleop-twist-keyboard
```

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

Keep that terminal focused; it reads keys directly. `i` forward, `,` back,
`j`/`l` turn, `k` stop. Wheels off the ground for the first run.

---

## Troubleshooting

| Symptom | Cause |
|---|---|
| Agent prints `fd: N` then nothing | DTR/RTS holding the ESP32 in reset — use `start_agent.sh` |
| `/dev/ttyUSB0` missing | Cable in the laptop not the Pi, or a charge-only cable |
| `Permission denied` on the port | Not in `dialout`, or did not log out and back in |
| Topics missing but agent connected | `ROS_DOMAIN_ID` differs between terminals. Default is 0 |
| Agent connects, then drops | Loose USB, or the ESP32 browning out from motor current |
| ESP32 LED off | No agent session. Start the agent, it reconnects on its own |

---

## Reflashing later

The ESP32 can only be flashed from the machine running Arduino IDE. To change
the firmware: unplug from the Pi, plug into Windows, flash, plug back into the
Pi. Windows will not see the board while it is attached to the Pi.
