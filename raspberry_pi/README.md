# Raspberry Pi 5 — getting the rover running

Everything in this folder runs on the Pi. The ESP32 firmware is flashed
separately, from Windows (`esp32/`, step 1).

```
raspberry_pi/
  setup_pi.sh            one-time setup — run once after cloning
  run_rover.sh           starts the robot
  scripts/
    start_agent.sh       micro-ROS agent for the ESP32 (started by run_rover.sh)
    web_teleop.py        phone driving page (run_rover.sh --phone)
    rover_monitor.py     live view: commands vs encoder ticks
  ros/
    rover_bringup/       one launch for agent + URDF + odometry + IMU + EKF + LiDAR + camera
      launch/lidar.launch.py      RPLIDAR A1  -> /scan
      launch/camera.launch.py     Pi Camera 3 -> /camera/image_raw
      rover_bringup/mpu6050_imu.py      MPU6050 -> /imu/data_raw
      rover_bringup/wheel_odometry.py   ESP32 ticks -> /wheel/odom
      config/ekf.yaml             odometry + IMU -> /odometry/filtered
      udev/99-rover.rules         fixed names /dev/esp32 and /dev/rplidar
    rover_description/   URDF + meshes: where every sensor sits on the rover
```

| Sensor | Plugs into | Read by |
|---|---|---|
| RPLIDAR A1 | Pi USB | Pi — `rplidar_ros` driver |
| Pi Camera 3 | Pi camera ribbon port | Pi — `camera_ros` driver |
| MPU6050 IMU | Pi I2C pins | Pi — `mpu6050_imu.py` |
| Wheel encoders | **ESP32** GPIO 18/19 | ESP32 counts; Pi computes odometry |

---

## 1. Flash the ESP32 (Windows, once)

Full detail: `docs/MICROROS.md` Part A. Versions are strict:

1. Arduino IDE → Boards Manager → **esp32 by Espressif, version 2.0.2**
   (3.x will not link).
2. Sketch → Include Library → Add .ZIP Library →
   `https://github.com/micro-ROS/micro_ros_arduino/archive/refs/tags/v2.0.8-jazzy.zip`
3. Open `esp32/rover_microros/rover_microros.ino`, board **ESP32 Dev Module**,
   Upload.

Then move its USB cable to the **Pi**.

---

## 2. Prepare the Pi

**Operating system: Ubuntu Server 24.04 LTS (64-bit).** Not Raspberry Pi OS —
ROS 2 Jazzy has no packages for it. Flash it with Raspberry Pi Imager and turn
on SSH and Wi-Fi in the Imager's settings.

### Wire the sensors to the Pi

| Part | Connects to |
|---|---|
| ESP32 | Pi USB port (data cable, not charge-only) |
| RPLIDAR A1 | Pi USB port, through its USB adapter board |
| Pi Camera 3 | Pi **CAM/DISP** ribbon port (the Pi 5 needs the narrow 22-pin cable) |
| MPU6050 VCC | Pi pin **1** (3.3 V) |
| MPU6050 GND | Pi pin **6** |
| MPU6050 SDA | Pi pin **3** |
| MPU6050 SCL | Pi pin **5** |

Motors, battery and ESP32 pins: `docs/WIRING.md`.

### Get the code

```bash
sudo apt install -y git
```

```bash
git clone https://github.com/riri6967/Guide-dog-rover.git ~/Guide-dog-rover
```

Keep it at `~/Guide-dog-rover`; the launch file looks for the agent script there.

---

## 3. One-time setup

With the ESP32 and LiDAR plugged in:

```bash
cd ~/Guide-dog-rover/raspberry_pi && bash setup_pi.sh
```

It installs ROS 2 Jazzy if missing, the LiDAR / camera / EKF packages, builds
the micro-ROS agent (the slow part, 10–20 min) and the rover packages into
`~/ros2_ws`, sets permissions, and installs the udev rules. Safe to run again.

**Reboot when it says so:**

```bash
sudo reboot
```

Then check:

```bash
ls -l /dev/esp32 /dev/rplidar
```

```bash
i2cdetect -y 1
```

Both names should exist, and `68` should appear in the I2C table.

---

## 4. Run it

**Wheels off the ground the first time.** Keep the rover still for the first
two seconds — the IMU measures its gyro bias at start-up.

```bash
~/Guide-dog-rover/raspberry_pi/run_rover.sh
```

The ESP32's **blue LED** lights when it connects. Then, in a second SSH window,
drive it one of these ways:

**Keyboard** — `i` forward, `,` back, `j`/`l` turn, `k` stop:

```bash
ros2 run teleop_twist_keyboard teleop_twist_keyboard
```

**Fixed command** — 0.3 m/s forward; Ctrl+C stops it:

```bash
ros2 topic pub /cmd_vel geometry_msgs/msg/Twist "{linear: {x: 0.3}}" -r 10
```

**Phone** — start with the phone page, then open the address it prints on a
phone on the same Wi-Fi:

```bash
~/Guide-dog-rover/raspberry_pi/run_rover.sh --phone
```

Use one way of driving at a time. Stopping the driver, closing SSH or pulling
the cable stops the motors within 0.9 s.

### Check each sensor

```bash
ros2 topic list
```

```bash
python3 ~/Guide-dog-rover/raspberry_pi/scripts/rover_monitor.py
```

```bash
ros2 topic hz /scan
```

```bash
ros2 topic echo /imu/data_raw --once
```

```bash
ros2 topic hz /camera/image_raw
```

Expected: `/cmd_vel /ticks_left /ticks_right /wheel/odom /odometry/filtered
/imu/data_raw /scan /camera/image_raw /tf`.

Switch a part off for testing: `run_rover.sh camera:=false` (also `lidar`,
`imu`, `ekf`, `odometry`, `agent`).

Then do the five calibration checks in `ros/rover_bringup/README.md`.

### After pulling new code

```bash
cd ~/Guide-dog-rover && git pull
```

```bash
cd ~/ros2_ws && colcon build --symlink-install
```

---

## Troubleshooting

| Symptom | Fix |
|---|---|
| Blue LED never lights | ESP32 not on the Pi's USB, or a charge-only cable. `ls /dev/ttyUSB*` |
| LED lit but no `/ticks_left` | `ros2 daemon stop`, then `ros2 topic list` again |
| `Permission denied` on a port | The reboot after setup was skipped |
| `/dev/esp32` or `/dev/rplidar` missing | Both boards may use the same USB chip: add serial numbers as described in `ros/rover_bringup/udev/99-rover.rules`, re-run `setup_pi.sh` |
| IMU node fails / no `68` | Check the 4 IMU wires; reboot after setup |
| Camera node fails | `run_rover.sh camera:=false` and drive without it |
| Everything dies a few seconds into driving | Battery protection or motor EMI — `docs/BUILD.md` section 9 |

More: `docs/BUILD.md` section 9 and `docs/MICROROS.md`.
