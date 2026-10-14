#!/usr/bin/env python3

import math
import time

import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan


class LocalNavigationSupervisor(Node):

    def __init__(self):
        super().__init__('local_navigation_supervisor')

        # ---------------------------------------------------------
        # Robot limits
        # ---------------------------------------------------------

        self.max_linear = 0.60
        self.max_angular = 1.50

        # ---------------------------------------------------------
        # Obstacle distances
        # ---------------------------------------------------------

        self.hard_stop_distance = 0.55
        self.avoid_distance = 1.00
        self.full_speed_distance = 1.60

        # ---------------------------------------------------------
        # Stuck detection
        # ---------------------------------------------------------

        self.stuck_time = 1.20
        self.minimum_progress = 0.06
        self.minimum_command_speed = 0.18

        # ---------------------------------------------------------
        # Recovery
        # ---------------------------------------------------------

        self.reverse_speed = -0.25
        self.reverse_duration = 0.75

        self.turn_speed = 1.0
        self.turn_tolerance = math.radians(15.0)

        self.cooldown_duration = 0.8

        # ---------------------------------------------------------
        # Robot state
        # ---------------------------------------------------------

        self.odom_x = None
        self.odom_y = None
        self.yaw = None

        # ---------------------------------------------------------
        # Goal state
        # ---------------------------------------------------------

        self.goal_x = None
        self.goal_y = None

        # ---------------------------------------------------------
        # SAC command
        # ---------------------------------------------------------

        self.sac_linear = 0.0
        self.sac_angular = 0.0
        self.last_sac_time = 0.0

        # ---------------------------------------------------------
        # LiDAR
        # ---------------------------------------------------------

        self.scan = None

        # ---------------------------------------------------------
        # Progress tracking
        # ---------------------------------------------------------

        self.last_progress_x = None
        self.last_progress_y = None
        self.last_progress_time = time.monotonic()

        # ---------------------------------------------------------
        # Recovery state
        # ---------------------------------------------------------

        self.mode = "NORMAL"
        self.recovery_start = 0.0
        self.recovery_turn = 1.0
        self.recovery_count = 0

        # ---------------------------------------------------------
        # ROS subscriptions
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

        self.goal_sub = self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/cmd_vel',
            10
        )

        self.timer = self.create_timer(
            0.05,
            self.control_loop
        )

        self.get_logger().info(
            'Goal-aware local navigation supervisor started'
        )

    # =============================================================
    # SAC COMMAND
    # =============================================================

    def sac_callback(self, msg):

        self.sac_linear = float(
            msg.linear.x
        )

        self.sac_angular = float(
            msg.angular.z
        )

        self.last_sac_time = time.monotonic()

    # =============================================================
    # GOAL
    # =============================================================

    def goal_callback(self, msg):

        self.goal_x = float(
            msg.pose.position.x
        )

        self.goal_y = float(
            msg.pose.position.y
        )

    # =============================================================
    # ODOMETRY
    # =============================================================

    def odom_callback(self, msg):

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        siny = (
            2.0 *
            (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy = (
            1.0
            -
            2.0 *
            (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        self.yaw = math.atan2(
            siny,
            cosy
        )

        if self.last_progress_x is None:

            self.last_progress_x = self.odom_x
            self.last_progress_y = self.odom_y
            self.last_progress_time = time.monotonic()

            return

        movement = math.hypot(
            self.odom_x - self.last_progress_x,
            self.odom_y - self.last_progress_y
        )

        if movement >= self.minimum_progress:

            self.last_progress_x = self.odom_x
            self.last_progress_y = self.odom_y
            self.last_progress_time = time.monotonic()

    # =============================================================
    # LIDAR
    # =============================================================

    def scan_callback(self, msg):

        self.scan = msg

    def valid_ranges(self):

        if self.scan is None:
            return []

        values = []

        for i, r in enumerate(
            self.scan.ranges
        ):

            if not math.isfinite(r):
                continue

            if (
                self.scan.range_min
                <= r
                <= self.scan.range_max
            ):

                angle = (
                    self.scan.angle_min
                    +
                    i * self.scan.angle_increment
                )

                values.append(
                    (angle, r)
                )

        return values

    def sector_min(
        self,
        center_deg,
        width_deg
    ):

        center = math.radians(
            center_deg
        )

        half = math.radians(
            width_deg
        )

        values = []

        for angle, distance in self.valid_ranges():

            diff = (
                (angle - center + math.pi)
                %
                (2.0 * math.pi)
            ) - math.pi

            if abs(diff) <= half:

                values.append(
                    distance
                )

        if not values:
            return 15.0

        return min(values)

    # =============================================================
    # ROVER FOOTPRINT / CLEARANCE
    # =============================================================

    # Rover chassis: 0.90 m x 0.60 m
    ROVER_HALF_LENGTH = 0.45
    ROVER_HALF_WIDTH = 0.30

    # Extra clearance for localization/control uncertainty.
    ROVER_SAFETY_MARGIN = 0.18

    @classmethod
    def required_clearance_for_angle(cls, angle_deg):
        """
        Distance from robot center to the rectangular footprint
        boundary along a LiDAR ray at angle_deg, plus safety margin.
        """
        a = math.radians(angle_deg)

        support = (
            abs(cls.ROVER_HALF_LENGTH * math.cos(a))
            +
            abs(cls.ROVER_HALF_WIDTH * math.sin(a))
        )

        return support + cls.ROVER_SAFETY_MARGIN

    def direction_is_safe(self, angle_deg, width_deg=12.0):
        """
        Check whether a proposed motion direction has enough
        clearance for the complete rover footprint.
        """
        clearance = self.sector_min(
            angle_deg,
            width_deg
        )

        required = self.required_clearance_for_angle(
            angle_deg
        )

        return clearance > required

    def report_blocked_direction(self, angle_deg):
        clearance = self.sector_min(
            angle_deg,
            12.0
        )

        required = self.required_clearance_for_angle(
            angle_deg
        )

        self.get_logger().warn(
            f"FOOTPRINT BLOCK: angle={angle_deg:+.0f}deg "
            f"lidar={clearance:.2f}m "
            f"required={required:.2f}m"
        )

    # =============================================================
    # ANGLE HELPERS
    # =============================================================

    @staticmethod
    def angle_diff(
        target,
        current
    ):

        return (
            (target - current + math.pi)
            %
            (2.0 * math.pi)
        ) - math.pi

    def goal_heading_error(self):

        if (
            self.goal_x is None
            or self.goal_y is None
            or self.odom_x is None
            or self.odom_y is None
            or self.yaw is None
        ):

            return None

        dx = (
            self.goal_x
            -
            self.odom_x
        )

        dy = (
            self.goal_y
            -
            self.odom_y
        )

        if math.hypot(dx, dy) < 0.05:

            return 0.0

        goal_heading = math.atan2(
            dy,
            dx
        )

        return self.angle_diff(
            goal_heading,
            self.yaw
        )

    # =============================================================
    # PUBLISH
    # =============================================================

    def publish_command(
        self,
        linear,
        angular
    ):

        msg = Twist()

        msg.linear.x = max(
            -self.max_linear,
            min(
                self.max_linear,
                linear
            )
        )

        msg.angular.z = max(
            -self.max_angular,
            min(
                self.max_angular,
                angular
            )
        )

        self.cmd_pub.publish(msg)

    # =============================================================
    # ESCAPE DIRECTION
    # =============================================================

    def choose_escape_turn(self):

        left = self.sector_min(
            90.0,
            45.0
        )

        right = self.sector_min(
            -90.0,
            45.0
        )

        if left >= right:
            return 1.0

        return -1.0

    # =============================================================
    # START RECOVERY
    # =============================================================

    def start_recovery(
        self,
        reason
    ):

        self.mode = "REVERSE"

        self.recovery_start = (
            time.monotonic()
        )

        self.recovery_count += 1

        error = self.goal_heading_error()

        if (
            error is not None
            and
            abs(error)
            >
            math.radians(100.0)
        ):

            # Goal is behind the robot.
            # Perform a goal-directed U-turn.
            self.recovery_turn = (
                1.0
                if error > 0.0
                else -1.0
            )

            recovery_type = "GOAL_U_TURN"

        else:

            # Goal is not directly behind;
            # escape toward the more open side.
            self.recovery_turn = (
                self.choose_escape_turn()
            )

            recovery_type = "ESCAPE_TURN"

        if error is None:

            error_deg = 0.0

        else:

            error_deg = math.degrees(
                error
            )

        direction = (
            "LEFT"
            if self.recovery_turn > 0
            else
            "RIGHT"
        )

        self.get_logger().warn(
            f"RECOVERY #{self.recovery_count} "
            f"type={recovery_type} "
            f"reason={reason} "
            f"goal_error={error_deg:.1f}deg "
            f"turn={direction}"
        )

    # =============================================================
    # RECOVERY EXECUTION
    # =============================================================

    def run_recovery(self):

        elapsed = (
            time.monotonic()
            -
            self.recovery_start
        )

        # ---------------------------------------------------------
        # PHASE 1: REVERSE
        # ---------------------------------------------------------

        if self.mode == "REVERSE":

            rear = self.sector_min(
                180.0,
                35.0
            )

            if (
                elapsed
                <
                self.reverse_duration
                and
                rear
                >
                0.70
            ):

                self.publish_command(
                    self.reverse_speed,
                    0.0
                )

                return

            self.mode = "TURN"

            self.recovery_start = (
                time.monotonic()
            )

            return

        # ---------------------------------------------------------
        # PHASE 2: TURN TOWARD GOAL
        # ---------------------------------------------------------

        if self.mode == "TURN":

            error = (
                self.goal_heading_error()
            )

            if (
                error is not None
                and
                abs(error)
                <=
                self.turn_tolerance
            ):

                self.get_logger().info(
                    f"U-turn complete; "
                    f"heading error="
                    f"{math.degrees(error):.1f}deg"
                )

                self.mode = "COOLDOWN"

                self.recovery_start = (
                    time.monotonic()
                )

                self.publish_command(
                    0.0,
                    0.0
                )

                return

            # Safety timeout
            if elapsed >= 4.0:

                self.get_logger().warn(
                    "Turn timeout; restoring SAC"
                )

                self.mode = "COOLDOWN"

                self.recovery_start = (
                    time.monotonic()
                )

                return

            # Turn in place.
            if error is not None:

                direction = (
                    1.0
                    if error > 0.0
                    else -1.0
                )

            else:

                direction = (
                    self.recovery_turn
                )

            self.publish_command(
                0.0,
                direction * self.turn_speed
            )

            return

        # ---------------------------------------------------------
        # PHASE 3: COOLDOWN
        # ---------------------------------------------------------

        if self.mode == "COOLDOWN":

            self.publish_command(
                0.0,
                0.0
            )

            if (
                elapsed
                >=
                self.cooldown_duration
            ):

                self.mode = "NORMAL"

                self.last_progress_time = (
                    time.monotonic()
                )

                self.get_logger().info(
                    "Recovery complete; "
                    "SAC control restored"
                )

    # =============================================================
    # LOCAL AVOIDANCE
    # =============================================================

    def choose_avoidance_angle(self):

        goal_error = (
            self.goal_heading_error()
        )

        if goal_error is not None:

            desired = math.degrees(
                goal_error
            )

        else:

            desired = math.degrees(
                math.atan2(
                    self.sac_angular,
                    max(
                        self.sac_linear,
                        0.05
                    )
                )
            )

        desired = max(
            -75.0,
            min(
                75.0,
                desired
            )
        )

        best_angle = desired
        best_score = -999.0

        for angle in range(
            -75,
            76,
            10
        ):

            clearance = (
                self.sector_min(
                    angle,
                    8.0
                )
            )

            if (
                clearance
                <
                self.hard_stop_distance
            ):
                continue

            angle_error = abs(
                angle - desired
            )

            alignment = math.cos(
                math.radians(
                    min(
                        angle_error,
                        180.0
                    )
                )
            )

            clearance_score = (
                min(
                    clearance,
                    3.0
                )
                /
                3.0
            )

            turn_cost = (
                abs(angle)
                /
                75.0
            )

            score = (
                2.5 * alignment
                +
                2.0 * clearance_score
                -
                0.35 * turn_cost
            )

            if score > best_score:

                best_score = score
                best_angle = angle

        return best_angle

    # =============================================================
    # MAIN CONTROL LOOP
    # =============================================================

    def control_loop(self):

        now = time.monotonic()

        # ---------------------------------------------------------
        # Wait for LiDAR
        # ---------------------------------------------------------

        if self.scan is None:

            self.publish_command(
                0.0,
                0.0
            )

            return

        # ---------------------------------------------------------
        # Stale SAC protection
        # ---------------------------------------------------------

        if (
            self.last_sac_time == 0.0
            or
            now - self.last_sac_time
            >
            0.50
        ):

            self.publish_command(
                0.0,
                0.0
            )

            return

        # ---------------------------------------------------------
        # Continue recovery
        # ---------------------------------------------------------

        if self.mode != "NORMAL":

            self.run_recovery()

            return

        front = self.sector_min(
            0.0,
            25.0
        )

        front_wide = self.sector_min(
            0.0,
            40.0
        )

        trying_to_move = (
            self.sac_linear
            >
            self.minimum_command_speed
        )

        no_progress = (
            now
            -
            self.last_progress_time
            >=
            self.stuck_time
        )

        stuck = (
            trying_to_move
            and
            no_progress
            and
            front_wide < 1.20
        )

        goal_error = (
            self.goal_heading_error()
        )

        strongly_wrong_way = (
            goal_error is not None
            and
            abs(goal_error)
            >
            math.radians(120.0)
            and
            trying_to_move
        )

        # ---------------------------------------------------------
        # Goal-aware recovery
        # ---------------------------------------------------------

        if strongly_wrong_way:

            self.start_recovery(
                "heading-away-from-goal"
            )

            return

        if stuck:

            self.start_recovery(
                "stagnation-near-obstacle"
            )

            return

        # ---------------------------------------------------------
        # Hard obstacle
        # ---------------------------------------------------------

        if (
            front
            <
            self.hard_stop_distance
        ):

            angle = (
                self.choose_avoidance_angle()
            )

            if angle is None:
                self.start_recovery(
                    "no-footprint-safe-direction"
                )
                return

            clearance = (
                self.sector_min(
                    angle,
                    12.0
                )
            )

            if not self.direction_is_safe(
                angle,
                12.0
            ):
                self.report_blocked_direction(
                    angle
                )
                self.start_recovery(
                    "footprint-blocked"
                )
                return

                self.start_recovery(
                    "no-safe-direction"
                )

                return

            self.publish_command(
                0.0,
                math.radians(
                    angle
                ) * 1.4
            )

            return

        # ---------------------------------------------------------
        # Local obstacle avoidance
        # ---------------------------------------------------------

        if (
            self.sac_linear > 0.05
            and
            front_wide
            <
            self.avoid_distance
        ):

            angle = (
                self.choose_avoidance_angle()
            )

            if angle is None:
                self.start_recovery(
                    "no-footprint-safe-direction"
                )
                return

            clearance = (
                self.sector_min(
                    angle,
                    12.0
                )
            )

            if not self.direction_is_safe(
                angle,
                12.0
            ):
                self.report_blocked_direction(
                    angle
                )
                self.start_recovery(
                    "footprint-blocked"
                )
                return

            # Slow down based on the clearance remaining
            # after accounting for the rover footprint.
            required = self.required_clearance_for_angle(
                angle
            )

            safe_margin = max(
                0.0,
                clearance - required
            )

            scale = min(
                1.0,
                safe_margin / 0.80
            )

            scale = max(
                0.15,
                min(
                    1.0,
                    scale
                )
            )

            self.publish_command(
                self.sac_linear * scale,
                math.radians(
                    angle
                ) * 1.35
            )

            return

        # ---------------------------------------------------------
        # Normal SAC control
        # ---------------------------------------------------------

        linear = self.sac_linear
        angular = self.sac_angular

        if (
            front
            <
            self.full_speed_distance
        ):

            scale = (
                front
                -
                self.hard_stop_distance
            ) / (
                self.full_speed_distance
                -
                self.hard_stop_distance
            )

            scale = max(
                0.25,
                min(
                    1.0,
                    scale
                )
            )

            linear *= scale

        self.publish_command(
            linear,
            angular
        )

    # =============================================================
    # SHUTDOWN
    # =============================================================

    def stop_robot(self):

        for _ in range(5):

            self.publish_command(
                0.0,
                0.0
            )

            time.sleep(0.02)

    def destroy_node(self):

        self.stop_robot()

        super().destroy_node()


def main():

    rclpy.init()

    node = LocalNavigationSupervisor()

    try:

        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.stop_robot()
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
