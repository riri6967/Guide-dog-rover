#!/usr/bin/env python3

import math
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray


class LidarProcessor(Node):

    def __init__(self):
        super().__init__('lidar_processor')

        self.subscription = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.publisher = self.create_publisher(
            Float32MultiArray,
            '/lidar_features',
            10
        )

        self.num_sectors = 12

        self.get_logger().info(
            'LiDAR processor started: 12 sectors'
        )

    def scan_callback(self, msg):

        ranges = list(msg.ranges)
        features = []

        sector_size = len(ranges) // self.num_sectors

        for i in range(self.num_sectors):

            start = i * sector_size

            if i == self.num_sectors - 1:
                sector = ranges[start:]
            else:
                sector = ranges[start:start + sector_size]

            valid = [
                r for r in sector
                if math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
            ]

            if valid:
                minimum_distance = min(valid)
            else:
                minimum_distance = msg.range_max

            features.append(minimum_distance)

        output = Float32MultiArray()
        output.data = features

        self.publisher.publish(output)


def main(args=None):

    rclpy.init(args=args)

    node = LidarProcessor()

    try:
        rclpy.spin(node)
    except KeyboardInterrupt:
        pass

    node.destroy_node()
    rclpy.shutdown()


if __name__ == '__main__':
    main()
