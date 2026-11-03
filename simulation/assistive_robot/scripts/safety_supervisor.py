#!/usr/bin/env python3

import json
import math
import time

import rclpy

from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from std_msgs.msg import String
from rclpy.node import Node


class SafetySupervisor(Node):

    def __init__(self):

        super().__init__('safety_supervisor')

        self.desired_linear = 0.0
        self.desired_angular = 0.0
        self.last_sac_command_time = 0.0

        self.min_lidar = float('inf')
        self.last_scan_time = 0.0

        self.nearest_human = None

        self.create_subscription(
            Twist,
            '/sac_cmd_vel',
            self.command_callback,
            10
        )

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.create_subscription(
            String,
            '/semantic_objects',
            self.semantic_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.publish_safe_command
        )

        self.get_logger().info(
            'Safety supervisor started'
        )

    def command_callback(self, msg):

        self.desired_linear = float(
            msg.linear.x
        )

        self.desired_angular = float(
            msg.angular.z
        )

        self.last_sac_command_time = time.monotonic()

    def scan_callback(self, msg):

        valid = [
            r for r in msg.ranges
            if math.isfinite(r)
            and r >= msg.range_min
            and r <= msg.range_max
        ]

        if valid:

            self.min_lidar = min(valid)

        else:

            self.min_lidar = float('inf')

        self.last_scan_time = time.monotonic()

    def semantic_callback(self, msg):

        try:

            data = json.loads(
                msg.data
            )

            nearest = None

            for obj in data.get(
                'objects',
                []
            ):

                if (
                    obj.get('class', '').lower()
                    == 'person'
                ):

                    distance = float(
                        obj.get(
                            'distance_m',
                            99.0
                        )
                    )

                    confidence = float(
                        obj.get(
                            'confidence',
                            0.0
                        )
                    )

                    if confidence < 0.3:
                        continue

                    if (
                        nearest is None
                        or distance < nearest
                    ):

                        nearest = distance

            self.nearest_human = nearest

        except Exception:
            pass

    def publish_safe_command(self):

        linear = self.desired_linear
        angular = self.desired_angular

        now = time.monotonic()

        # SAC command timeout = STOP
        # Prevent stale commands from continuing after
        # the SAC controller stops or crashes.
        if (
            self.last_sac_command_time == 0.0
            or now - self.last_sac_command_time > 0.3
        ):
            linear = 0.0
            angular = 0.0

        # Sensor timeout = STOP
        if (
            now - self.last_scan_time
            > 0.5
        ):

            linear = 0.0
            angular = 0.0

        # ---------------------------------------------------------
        # LiDAR safety filter
        # ---------------------------------------------------------

        if self.min_lidar < 0.35:

            linear = 0.0
            angular = 0.0

        elif self.min_lidar < 1.0:

            scale = (
                self.min_lidar - 0.35
            ) / 0.65

            scale = max(
                0.0,
                min(1.0, scale)
            )

            linear *= scale

        # ---------------------------------------------------------
        # Human safety filter
        # ---------------------------------------------------------

        if self.nearest_human is not None:

            if self.nearest_human < 0.6:

                linear = 0.0
                angular = 0.0

            elif self.nearest_human < 1.5:

                linear *= 0.5

        # Final command limits
        linear = max(
            0.0,
            min(0.6, linear)
        )

        angular = max(
            -1.5,
            min(1.5, angular)
        )

        msg = Twist()

        msg.linear.x = linear
        msg.angular.z = angular

        self.cmd_pub.publish(
            msg
        )


def main(args=None):

    rclpy.init(args=args)

    node = SafetySupervisor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        msg = Twist()
        node.cmd_pub.publish(msg)

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
