"""MPU6050 (GY-521 board) on the Pi's I2C bus -> /imu/data_raw.

Publishes sensor_msgs/Imu in frame imu_link with angular velocity (rad/s) and
linear acceleration (m/s^2, gravity included). The MPU6050 has no magnetometer
and this node does no fusion, so orientation is marked unknown
(orientation_covariance[0] = -1); the EKF uses the gyro's yaw rate.

Axes are the chip's own. The URDF's imu_link rotation (imu_yaw) maps them onto
the robot, so nothing is rotated here. Check it with the tilt tests in the
package README before trusting the EKF.

The gyro's bias is measured at start-up: keep the rover still for the first
couple of seconds after launch.

Wiring (Pi header): VCC -> pin 1 (3.3 V), GND -> pin 6, SDA -> pin 3 (GPIO2),
SCL -> pin 5 (GPIO3). Enable I2C, then `i2cdetect -y 1` should show 68.
Needs python3-smbus (`sudo apt install python3-smbus`) or smbus2; either works.
"""
import math
import time

import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Imu

REG_SMPLRT_DIV, REG_CONFIG, REG_GYRO_CONFIG, REG_ACCEL_CONFIG = 0x19, 0x1A, 0x1B, 0x1C
REG_ACCEL_XOUT_H, REG_PWR_MGMT_1, REG_WHO_AM_I = 0x3B, 0x6B, 0x75
GYRO_LSB_PER_DPS = 131.0     # +-250 deg/s range
ACCEL_LSB_PER_G = 16384.0    # +-2 g range
G = 9.80665


def s16(hi, lo):
    v = (hi << 8) | lo
    return v - 65536 if v & 0x8000 else v


class Mpu6050Imu(Node):
    def __init__(self):
        super().__init__('mpu6050_imu')
        self.bus_num = self.declare_parameter('i2c_bus', 1).value
        self.addr = self.declare_parameter('address', 0x68).value
        self.frame = self.declare_parameter('frame_id', 'imu_link').value
        rate = self.declare_parameter('rate', 100.0).value
        self.calib_samples = self.declare_parameter('calibration_samples', 200).value
        # Noise densities from the datasheet (0.005 deg/s/rtHz, 400 ug/rtHz) at a
        # 44 Hz bandwidth, rounded up: variances in (rad/s)^2 and (m/s^2)^2.
        self.gyro_var = self.declare_parameter('gyro_variance', 2e-5).value
        self.accel_var = self.declare_parameter('accel_variance', 5e-3).value

        try:
            from smbus2 import SMBus
        except ImportError:
            from smbus import SMBus  # python3-smbus from apt: same calls
        self.bus = SMBus(self.bus_num)
        self._init_chip()
        self.bias = self._calibrate()
        self.pub = self.create_publisher(Imu, 'imu/data_raw', 20)
        self.errors = 0
        self.create_timer(1.0 / rate, self._tick)

    def _init_chip(self):
        who = self.bus.read_byte_data(self.addr, REG_WHO_AM_I)
        if who not in (0x68, 0x72, 0x98):  # 0x68 genuine; clones report others
            self.get_logger().warn(f'WHO_AM_I = 0x{who:02x}, expected 0x68 - continuing anyway')
        self.bus.write_byte_data(self.addr, REG_PWR_MGMT_1, 0x01)  # wake, PLL on gyro X
        time.sleep(0.1)
        self.bus.write_byte_data(self.addr, REG_CONFIG, 0x03)        # DLPF ~44 Hz
        self.bus.write_byte_data(self.addr, REG_SMPLRT_DIV, 4)       # 1 kHz / 5 = 200 Hz
        self.bus.write_byte_data(self.addr, REG_GYRO_CONFIG, 0x00)   # +-250 deg/s
        self.bus.write_byte_data(self.addr, REG_ACCEL_CONFIG, 0x00)  # +-2 g
        time.sleep(0.05)

    def _read(self):
        b = self.bus.read_i2c_block_data(self.addr, REG_ACCEL_XOUT_H, 14)
        acc = [s16(b[i], b[i + 1]) / ACCEL_LSB_PER_G * G for i in (0, 2, 4)]
        gyr = [math.radians(s16(b[i], b[i + 1]) / GYRO_LSB_PER_DPS) for i in (8, 10, 12)]
        return acc, gyr

    def _calibrate(self):
        self.get_logger().info(f'measuring gyro bias over {self.calib_samples} samples - keep the rover still')
        sums = [0.0, 0.0, 0.0]
        for _ in range(self.calib_samples):
            _, g = self._read()
            sums = [s + v for s, v in zip(sums, g)]
            time.sleep(0.005)
        bias = [s / self.calib_samples for s in sums]
        self.get_logger().info('gyro bias (deg/s): ' + ', '.join(f'{math.degrees(b):+.3f}' for b in bias))
        return bias

    def _tick(self):
        try:
            acc, gyr = self._read()
        except OSError as e:
            self.errors += 1
            self.get_logger().warn(f'I2C read failed ({self.errors} so far): {e}', throttle_duration_sec=5.0)
            return
        m = Imu()
        m.header.stamp = self.get_clock().now().to_msg()
        m.header.frame_id = self.frame
        m.orientation_covariance[0] = -1.0
        m.angular_velocity.x, m.angular_velocity.y, m.angular_velocity.z = (g - b for g, b in zip(gyr, self.bias))
        m.linear_acceleration.x, m.linear_acceleration.y, m.linear_acceleration.z = acc
        for i in (0, 4, 8):
            m.angular_velocity_covariance[i] = self.gyro_var
            m.linear_acceleration_covariance[i] = self.accel_var
        self.pub.publish(m)


def main():
    rclpy.init()
    node = Mpu6050Imu()
    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.try_shutdown()


if __name__ == '__main__':
    main()
