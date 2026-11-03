#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class RecoverySupervisor(Node):

    def __init__(self):
        super().__init__('recovery_supervisor')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.max_linear = 0.60
        self.max_angular = 1.50

        # Safety distances
        self.stop_distance = 0.35
        self.slow_distance = 0.75
        self.reverse_clearance = 0.45

        # Stuck detection
        self.stuck_command_speed = 0.15
        self.stuck_time = 1.2
        self.min_progress = 0.06

        # Recovery motion
        self.reverse_speed = -0.25
        self.reverse_duration = 0.80

        self.turn_speed = 0.90
        self.turn_duration = 0.90

        self.cooldown_duration = 1.50

        # ---------------------------------------------------------
        # State
        # ---------------------------------------------------------

        self.sac_linear = 0.0
        self.sac_angular = 0.0

        self.front_min = 15.0
        self.left_min = 15.0
        self.right_min = 15.0
        self.rear_min = 15.0
        self.global_min = 15.0

        self.odom_x = None
        self.odom_y = None

        self.last_movement_x = None
        self.last_movement_y = None
        self.last_progress_time = time.monotonic()

        self.last_sac_time = 0.0

        self.mode = "NORMAL"
        self.recovery_start = 0.0
        self.turn_direction = 1.0

        self.recovery_count = 0

        # ---------------------------------------------------------
        # ROS
        # ---------------------------------------------------------

        self.sac_sub = self.create_subscription(
            Twist,
            '/sac_cmd_vel',
            self.sac_callback,
            10
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        # 20 Hz supervisor
        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Recovery supervisor started'
        )

    # -------------------------------------------------------------
    # SAC command
    # -------------------------------------------------------------

    def sac_callback(self, msg):

        self.sac_linear = float(msg.linear.x)
        self.sac_angular = float(msg.angular.z)

        self.last_sac_time = time.monotonic()

    # -------------------------------------------------------------
    # Odometry
    # -------------------------------------------------------------

    def odom_callback(self, msg):

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

        if self.last_movement_x is None:

            self.last_movement_x = self.odom_x
            self.last_movement_y = self.odom_y
            self.last_progress_time = time.monotonic()

            return

        movement = math.hypot(
            self.odom_x - self.last_movement_x,
            self.odom_y - self.last_movement_y
        )

        if movement >= self.min_progress:

            self.last_movement_x = self.odom_x
            self.last_movement_y = self.odom_y
            self.last_progress_time = time.monotonic()

    # -------------------------------------------------------------
    # LiDAR
    # -------------------------------------------------------------

    def sector_min(
        self,
        msg,
        center_deg,
        half_width_deg
    ):

        values = []

        count = len(msg.ranges)

        for i, r in enumerate(msg.ranges):

            if not math.isfinite(r):
                continue

            angle = (
                msg.angle_min
                + i * msg.angle_increment
            )

            angle_deg = math.degrees(angle)

            difference = (
                (angle_deg - center_deg + 180.0)
                % 360.0
            ) - 180.0

            if abs(difference) <= half_width_deg:

                if (
                    msg.range_min
                    <= r
                    <= msg.range_max
                ):
                    values.append(r)

        if not values:
            return 15.0

        return min(values)

    def scan_callback(self, msg):

        self.global_min = min(
            [
                r for r in msg.ranges
                if math.isfinite(r)
                and msg.range_min <= r <= msg.range_max
            ],
            default=15.0
        )

        # Robot forward = 0 degrees
        self.front_min = self.sector_min(
            msg,
            0.0,
            35.0
        )

        self.left_min = self.sector_min(
            msg,
            90.0,
            50.0
        )

        self.right_min = self.sector_min(
            msg,
            -90.0,
            50.0
        )

        self.rear_min = self.sector_min(
            msg,
            180.0,
            35.0
        )

    # -------------------------------------------------------------
    # Publishing
    # -------------------------------------------------------------

    def publish(
        self,
        linear,
        angular
    ):

        msg = Twist()

        msg.linear.x = float(
            max(-self.max_linear,
                min(self.max_linear, linear))
        )

        msg.angular.z = float(
            max(-self.max_angular,
                min(self.max_angular, angular))
        )

        self.cmd_pub.publish(msg)

    # -------------------------------------------------------------
    # Recovery
    # -------------------------------------------------------------

    def start_recovery(self):

        self.mode = "REVERSE"
        self.recovery_start = time.monotonic()

        self.recovery_count += 1

        # Turn toward the side with MORE free space.
        if self.left_min > self.right_min:
            self.turn_direction = 1.0
        else:
            self.turn_direction = -1.0

        self.get_logger().warn(
            f'RECOVERY #{self.recovery_count} '
            f'front={self.front_min:.2f} '
            f'left={self.left_min:.2f} '
            f'right={self.right_min:.2f} '
            f'rear={self.rear_min:.2f} '
            f'turn={("LEFT" if self.turn_direction > 0 else "RIGHT")}'
        )

    def recovery_loop(self):

        elapsed = (
            time.monotonic()
            - self.recovery_start
        )

        # ---------------------------------------------------------
        # Reverse
        # ---------------------------------------------------------

        if self.mode == "REVERSE":

            if elapsed < self.reverse_duration:

                if self.rear_min > self.reverse_clearance:

                    self.publish(
                        self.reverse_speed,
                        0.0
                    )

                else:

                    self.get_logger().warn(
                        'Rear blocked; skipping reverse'
                    )

                    self.mode = "TURN"
                    self.recovery_start = time.monotonic()

            else:

                self.mode = "TURN"
                self.recovery_start = time.monotonic()

            return

        # ---------------------------------------------------------
        # Turn
        # ---------------------------------------------------------

        if self.mode == "TURN":

            if elapsed < self.turn_duration:

                self.publish(
                    0.0,
                    self.turn_direction * self.turn_speed
                )

            else:

                self.mode = "COOLDOWN"
                self.recovery_start = time.monotonic()

            return

        # ---------------------------------------------------------
        # Cooldown
        # ---------------------------------------------------------

        if self.mode == "COOLDOWN":

            self.publish(
                0.0,
                0.0
            )

            if elapsed >= self.cooldown_duration:

                self.mode = "NORMAL"

                self.last_progress_time = (
                    time.monotonic()
                )

                self.get_logger().info(
                    'Recovery complete; returning control to SAC'
                )

    # -------------------------------------------------------------
    # Main control loop
    # -------------------------------------------------------------

    def control_loop(self):

        now = time.monotonic()

        # ---------------------------------------------------------
        # Never allow stale SAC commands to continue forever
        # ---------------------------------------------------------

        if (
            self.last_sac_time == 0.0
            or now - self.last_sac_time > 0.50
        ):

            self.publish(0.0, 0.0)

            return

        # ---------------------------------------------------------
        # Already recovering
        # ---------------------------------------------------------

        if self.mode != "NORMAL":

            self.recovery_loop()

            return

        # ---------------------------------------------------------
        # Detect low-clearance situation
        # ---------------------------------------------------------

        obstacle_ahead = (
            self.front_min
            < self.stop_distance
        )

        caution_ahead = (
            self.front_min
            < self.slow_distance
        )

        # ---------------------------------------------------------
        # Detect actual stagnation
        # ---------------------------------------------------------

        no_progress = (
            now - self.last_progress_time
            >= self.stuck_time
        )

        trying_to_move = (
            abs(self.sac_linear)
            >= self.stuck_command_speed
        )

        stuck = (
            trying_to_move
            and no_progress
        )

        # ---------------------------------------------------------
        # Strong obstacle protection
        # ---------------------------------------------------------

        if obstacle_ahead:

            # If SAC wants forward motion into an obstacle,
            # stop linear motion and turn toward free space.
            if self.sac_linear > 0.0:

                turn = (
                    1.0
                    if self.left_min > self.right_min
                    else -1.0
                )

                self.publish(
                    0.0,
                    turn * min(
                        self.turn_speed,
                        abs(self.sac_angular) + 0.35
                    )
                )

                return

        # ---------------------------------------------------------
        # STUCK → RECOVERY
        # ---------------------------------------------------------

        if stuck and (
            self.front_min < 1.0
            or self.global_min < 0.75
        ):

            self.start_recovery()

            return

        # ---------------------------------------------------------
        # Normal safety scaling
        # ---------------------------------------------------------

        linear = self.sac_linear
        angular = self.sac_angular

        if caution_ahead and linear > 0.0:

            ratio = (
                self.front_min
                - self.stop_distance
            ) / (
                self.slow_distance
                - self.stop_distance
            )

            ratio = max(
                0.0,
                min(1.0, ratio)
            )

            linear *= ratio

        self.publish(
            linear,
            angular
        )


def main():

    rclpy.init()

    node = RecoverySupervisor()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:
        node.publish(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == '__main__':
    main()
