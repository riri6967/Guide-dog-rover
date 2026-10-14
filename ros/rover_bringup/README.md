# rover_bringup

Starts the robot on the Raspberry Pi with one command:

```bash
ros2 launch rover_bringup bringup.launch.py
```

| Piece | Gives | Frame |
|---|---|---|
| micro-ROS agent (`pi_scripts/start_agent.sh`) | `/ticks_left`, `/ticks_right`; takes `/cmd_vel` | — |
| URDF (`rover_description`) | the fixed transforms base_link -> sensors | — |
| `wheel_odometry` | `/wheel/odom`, `/joint_states` | odom -> base_footprint |
| `mpu6050_imu` | `/imu/data_raw` (gyro + accel, no orientation) | `imu_link` |
| EKF (`robot_localization`, `config/ekf.yaml`) | `/odometry/filtered` and the odom -> base_footprint transform | — |
| RPLIDAR A1M8 (`rplidar_ros`) | `/scan` | `laser` |
| Pi Camera v3 (`camera_ros`) | `/camera/image_raw`, `/camera/camera_info` | `camera_link_optical` |

Turn any piece off with `<name>:=false`: `agent`, `odometry`, `imu`, `ekf`,
`lidar`, `camera`. SLAM and Nav2 go on top: they need `/scan`,
`/odometry/filtered` and the TF tree map -> odom -> base_footprint -> sensors.

**Not yet run on the robot.** The Python nodes and launch files are
syntax-checked and the odometry maths is tested on a PC; nothing here has
talked to real hardware yet.

## Setup (once)

```bash
sudo apt install -y ros-jazzy-robot-localization ros-jazzy-rplidar-ros ros-jazzy-camera-ros ros-jazzy-xacro ros-jazzy-robot-state-publisher python3-smbus i2c-tools
```

```bash
ln -s ~/Guide-dog-rover/ros/rover_bringup ~/ros2_ws/src/rover_bringup
```

```bash
cd ~/ros2_ws && colcon build --symlink-install && source install/setup.bash
```

`rover_description` must be linked into the same workspace (see the main
README). The launch file expects the repo at `~/Guide-dog-rover`; otherwise
pass `agent_script:=/path/to/pi_scripts/start_agent.sh`.

### I2C for the MPU6050

```bash
sudo usermod -aG i2c $USER
```

Enable I2C (`dtparam=i2c_arm=on` in `/boot/firmware/config.txt`), reboot, then
`i2cdetect -y 1` should show `68`. Wiring: VCC -> pin 1 (3.3 V), GND -> pin 6,
SDA -> pin 3, SCL -> pin 5.

### Fixed names for the two USB serial devices

The ESP32 and the RPLIDAR both appear as `/dev/ttyUSB*`, and the numbers swap
with plug order. `udev/99-rover.rules` creates `/dev/esp32` and `/dev/rplidar`;
the file explains how to fill in your boards' IDs. `start_agent.sh` uses
`/dev/esp32` when it exists and never picks `/dev/rplidar`.

## Checks on the real robot

Do these once, in this order. Each wrong result is one number to change.

### 1. LiDAR direction
Put a box about 1 m straight in front of the rover and view `/scan` in RViz
(fixed frame `base_link`). It must appear in front (+X). If it appears behind,
set `laser_yaw` to `0` in `rover_description/urdf/rover.urdf.xacro`.

### 2. IMU axes
`/imu/data_raw` is in the chip's own axes. The URDF assumes the GY-521's pin
header faces the rear, which puts the chip's +Y forward and +X to the robot's
right (`imu_yaw` = -pi/2). An accelerometer at rest reads 1 g *upward*, so
raising one end makes the axis pointing that way read positive.

`ros2 topic echo /imu/data_raw`, then:

| Do this | Should see if the URDF is right |
|---|---|
| Rover flat and still | `linear_acceleration.z` about +9.8, x and y about 0 |
| Lift the front | `linear_acceleration.y` goes **positive** |
| Lift the left side | `linear_acceleration.x` goes **negative** |
| Turn left (anticlockwise from above) | `angular_velocity.z` positive |

If the front lift moves x instead of y, or a sign is flipped, note which and
set `imu_yaw` to match (0, pi/2, pi or -pi/2). If z reads -9.8, the board is
upside down.

### 3. Wheel odometry: distance
Mark the floor, drive 2 m straight (`ros2 topic pub /cmd_vel ...`), and compare
with `ros2 topic echo /wheel/odom`. A consistent error means `wheel_radius` is
off: real / reported x old radius.

### 4. Wheel odometry: turning (skid-steer)
Skid-steer wheels slide sideways in a turn, so the rover turns less than the
wheel difference says. Spin in place exactly 5 turns (1800 deg) and read the yaw
the odometry reports. Then:

    effective_track_width = track_width x (reported angle / real angle)

Pass it as a parameter (`effective_track_width:=...`). It is typically 1.3-2x
the real track width. The EKF leans on the IMU for heading anyway, but a good
value keeps the two from fighting.

### 5. Keep the three copies of the geometry in step
Wheel diameter (110 mm) and track width (240 mm, centre of the left wheel to
centre of the right wheel) are set in three places: the firmware
(`WHEEL_DIAMETER_M`, `TRACK_WIDTH_M`), this package's `wheel_odometry`
parameters, and the URDF. Change one, change all three.

## Turning on the speed PID (ESP32)

The firmware has a closed-loop wheel-speed controller, off by default because
motor EMI currently adds false encoder pulses. After the 100 nF capacitors:

1. Check the ticks are clean: wheels off the ground, drive at a steady speed;
   `/ticks_left` should rise at a believable rate (0.2 m/s with 110 mm wheels
   is about 81 ticks/s).
2. Set `USE_SPEED_PID = true` in `rover_microros.ino`, flash.
3. Wheels off the ground, command 0.2 m/s. The measured speed from
   `/wheel/odom` should settle on it within about a second. Oscillating:
   halve `PID_KP`. Slow to settle or stays short: raise `PID_KI`.
4. Then on the floor, with a load, at 0.1 m/s - slow speeds are where it matters.
