#!/usr/bin/env python3

import math
import time
import numpy as np

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import LaserScan
from nav_msgs.msg import Odometry


class LidarDebug(Node):

    def __init__(self):
        super().__init__('lidar_debug_v8')

        self.scan = None
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = 0.0

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.last_print = 0.0

        self.get_logger().info(
            'V8 LiDAR diagnostic started.'
        )

    def scan_callback(self, msg):
        self.scan = msg

    def odom_callback(self, msg):
        self.odom_x = float(
            msg.pose.pose.position.x
        )
        self.odom_y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        siny_cosp = (
            2.0 *
            (q.w * q.z + q.x * q.y)
        )

        cosy_cosp = (
            1.0 -
            2.0 * (
                q.y * q.y +
                q.z * q.z
            )
        )

        self.odom_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

    def sector_min(
        self,
        center_deg,
        width_deg
    ):

        if self.scan is None:
            return float('nan')

        center = math.radians(center_deg)
        half = math.radians(width_deg)

        values = []

        for i, r in enumerate(
            self.scan.ranges
        ):

            if not math.isfinite(r):
                continue

            if not (
                self.scan.range_min
                <= r
                <= self.scan.range_max
            ):
                continue

            angle = (
                self.scan.angle_min
                +
                i *
                self.scan.angle_increment
            )

            diff = (
                (angle - center + math.pi)
                %
                (2.0 * math.pi)
            ) - math.pi

            if abs(diff) <= half:
                values.append(float(r))

        if not values:
            return self.scan.range_max

        return min(values)

    def print_scan(self):

        if self.scan is None:
            print('Waiting for /scan...')
            return

        s = self.scan

        valid = [
            float(r)
            for r in s.ranges
            if math.isfinite(r)
            and s.range_min <= r <= s.range_max
        ]

        print()
        print('=' * 72)
        print('V8 RAW 2D LIDAR')
        print('=' * 72)

        print(
            f'Rays           : {len(s.ranges)}'
        )

        print(
            f'Angle range    : '
            f'{math.degrees(s.angle_min):.1f}° '
            f'to '
            f'{math.degrees(s.angle_max):.1f}°'
        )

        print(
            f'Angle increment: '
            f'{math.degrees(s.angle_increment):.3f}°'
        )

        print(
            f'Range limits   : '
            f'{s.range_min:.2f} m '
            f'to '
            f'{s.range_max:.2f} m'
        )

        if valid:
            print(
                f'Min range      : '
                f'{min(valid):.2f} m'
            )

        print(
            f'Robot position : '
            f'({self.odom_x}, {self.odom_y})'
        )

        print(
            f'Robot yaw      : '
            f'{math.degrees(self.odom_yaw):.1f}°'
        )

        print()
        print(
            'Directional minimum distances:'
        )

        directions = [
            ('FRONT', 0),
            ('FRONT-RIGHT', -30),
            ('RIGHT', -90),
            ('REAR-RIGHT', -135),
            ('REAR', 180),
            ('REAR-LEFT', 135),
            ('LEFT', 90),
            ('FRONT-LEFT', 30),
        ]

        for name, angle in directions:

            distance = self.sector_min(
                angle,
                12
            )

            print(
                f'  {name:12s} '
                f'{distance:6.2f} m'
            )

        print()
        print(
            'Interpretation:'
        )
        print(
            '  High distance = open space'
        )
        print(
            '  Low distance  = obstacle/wall'
        )
        print(
            '  Opening should appear as a '
            'continuous angular region of high distance'
        )

        print('=' * 72)

    def run(self):

        while rclpy.ok():

            rclpy.spin_once(
                self,
                timeout_sec=0.05
            )

            now = time.monotonic()

            if now - self.last_print >= 1.0:

                self.last_print = now

                self.print_scan()


def main():

    rclpy.init()

    node = LidarDebug()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
