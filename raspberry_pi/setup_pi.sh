#!/bin/bash
# One-time setup of the Guide Dog Rover on a Raspberry Pi 5.
#
#     cd ~/Guide-dog-rover/raspberry_pi && bash setup_pi.sh
#
# Needs Ubuntu 24.04 (64-bit) on the Pi - ROS 2 Jazzy has no packages for
# Raspberry Pi OS. Safe to run again: every step skips what is already done.
#
# What it does:
#   1. ROS 2 Jazzy (ros-base), if not installed
#   2. apt packages for the LiDAR, IMU, camera, EKF and URDF
#   3. serial / I2C / camera permissions for your user
#   4. I2C and camera switched on in /boot/firmware/config.txt
#   5. builds the micro-ROS agent from source in ~/uros_ws  (10-20 min)
#   6. builds rover_bringup + rover_description in ~/ros2_ws
#   7. installs the udev rules: /dev/esp32 and /dev/rplidar
#   8. adds the ROS setup lines to ~/.bashrc

set -e
HERE="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REBOOT=0

step() { echo; echo "=================== $*"; }
warn() { echo "!!! $*"; }

if [ "$EUID" -eq 0 ]; then
  echo "Run this as your normal user, not with sudo. It asks for sudo itself."
  exit 1
fi

. /etc/os-release
if [ "$ID" != "ubuntu" ] || [ "$VERSION_ID" != "24.04" ]; then
  echo "This Pi runs $PRETTY_NAME."
  echo "ROS 2 Jazzy needs Ubuntu 24.04 64-bit. Flash 'Ubuntu Server 24.04 LTS (64-bit)'"
  echo "with Raspberry Pi Imager, then run this script again."
  exit 1
fi

# Files copied from Windows may carry CRLF line endings and lose the execute
# bit; either one stops the scripts running.
find "$HERE" -type f \( -name '*.sh' -o -name '*.py' -o -name '*.yaml' -o -name '*.rules' \) \
  -exec sed -i 's/\r$//' {} +
chmod +x "$HERE"/*.sh "$HERE"/scripts/*.sh "$HERE"/scripts/*.py

# ---------------------------------------------------------------- 1. ROS 2
step "1/8  ROS 2 Jazzy"
if [ -f /opt/ros/jazzy/setup.bash ]; then
  echo "already installed"
else
  sudo apt update
  sudo apt install -y software-properties-common curl
  sudo add-apt-repository universe -y
  sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
    -o /usr/share/keyrings/ros-archive-keyring.gpg
  echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $UBUNTU_CODENAME main" \
    | sudo tee /etc/apt/sources.list.d/ros2.list >/dev/null
  sudo apt update
  # If this fails on liblz4-dev / libzstd-dev: docs/BUILD.md section 6.1.
  sudo apt install -y ros-jazzy-ros-base
fi

# ---------------------------------------------------------------- 2. packages
step "2/8  packages"
sudo apt update
sudo apt install -y \
  python3-rosdep python3-colcon-common-extensions git build-essential \
  python3-serial python3-smbus i2c-tools \
  ros-jazzy-robot-localization ros-jazzy-rplidar-ros \
  ros-jazzy-xacro ros-jazzy-robot-state-publisher ros-jazzy-joint-state-publisher \
  ros-jazzy-teleop-twist-keyboard
# Kept separate: if the camera driver is missing, everything else still runs.
sudo apt install -y ros-jazzy-camera-ros \
  || warn "camera_ros did not install - start the rover with: ./run_rover.sh camera:=false"

# ---------------------------------------------------------------- 3. groups
step "3/8  permissions (dialout = USB serial, i2c = IMU, video = camera)"
for g in dialout i2c video; do
  if getent group "$g" >/dev/null && ! id -nG "$USER" | grep -qw "$g"; then
    sudo usermod -aG "$g" "$USER"
    echo "added $USER to $g"
    REBOOT=1
  fi
done

# ---------------------------------------------------------------- 4. boot config
step "4/8  I2C and camera in config.txt"
CONFIG=/boot/firmware/config.txt
if [ -f "$CONFIG" ]; then
  for line in dtparam=i2c_arm=on camera_auto_detect=1; do
    if ! grep -q "^$line" "$CONFIG"; then
      echo "$line" | sudo tee -a "$CONFIG" >/dev/null
      echo "added $line"
      REBOOT=1
    fi
  done
else
  warn "$CONFIG not found - enable I2C and the camera by hand"
fi

# ---------------------------------------------------------------- 5. agent
step "5/8  micro-ROS agent (~/uros_ws)"
source /opt/ros/jazzy/setup.bash
if [ -d ~/uros_ws/install/micro_ros_agent ]; then
  echo "already built"
else
  mkdir -p ~/uros_ws/src
  cd ~/uros_ws
  [ -d src/micro_ros_setup ] || \
    git clone -b jazzy https://github.com/micro-ROS/micro_ros_setup.git src/micro_ros_setup
  sudo rosdep init 2>/dev/null || true
  rosdep update
  rosdep install --from-paths src --ignore-src -y
  colcon build
  source install/local_setup.bash
  ros2 run micro_ros_setup create_agent_ws.sh
  # "1 package had stderr output" here is git progress, not an error.
  ros2 run micro_ros_setup build_agent.sh
  rm -rf build log
fi

# ---------------------------------------------------------------- 6. rover packages
step "6/8  rover packages (~/ros2_ws)"
mkdir -p ~/ros2_ws/src
ln -sfn "$HERE/ros/rover_bringup"     ~/ros2_ws/src/rover_bringup
ln -sfn "$HERE/ros/rover_description" ~/ros2_ws/src/rover_description
cd ~/ros2_ws
colcon build --symlink-install

# ---------------------------------------------------------------- 7. udev
step "7/8  udev rules (/dev/esp32, /dev/rplidar)"
sudo cp "$HERE/ros/rover_bringup/udev/99-rover.rules" /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger
echo "USB serial devices plugged in now:"
for d in /dev/ttyUSB* /dev/ttyACM*; do
  [ -e "$d" ] || continue
  vid=$(udevadm info -q property -n "$d" | sed -n 's/^ID_VENDOR_ID=//p')
  pid=$(udevadm info -q property -n "$d" | sed -n 's/^ID_MODEL_ID=//p')
  echo "  $d  $vid:$pid"
done
ls -l /dev/esp32 /dev/rplidar 2>/dev/null || true
echo "LiDAR is expected as 10c4:ea60 (CP2102), the ESP32 as 1a86:7523 (CH340)."
echo "If the ESP32 also shows 10c4:ea60, add serial numbers to the rules:"
echo "see the comments in ros/rover_bringup/udev/99-rover.rules."

# ---------------------------------------------------------------- 8. bashrc
step "8/8  ~/.bashrc"
for line in "source /opt/ros/jazzy/setup.bash" \
            "source ~/uros_ws/install/local_setup.bash" \
            "source ~/ros2_ws/install/setup.bash"; do
  grep -qxF "$line" ~/.bashrc || echo "$line" >> ~/.bashrc
done
echo "done"

echo
echo "================================================================"
echo " Setup finished."
if [ "$REBOOT" = 1 ]; then
  echo " REBOOT NOW (sudo reboot) - new permissions and config.txt need it."
fi
echo " Then:   cd $HERE && ./run_rover.sh"
echo "================================================================"
