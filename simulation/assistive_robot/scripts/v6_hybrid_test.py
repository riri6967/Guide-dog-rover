#!/usr/bin/env python3

import math
import re
import subprocess
import time
import argparse

import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray

from rclpy.node import Node
from stable_baselines3 import SAC
from v6_route import generate_room_route


MODEL_PATH = (
    "/home/pushkar/assistive_robot_ws/models/"
    "sac_v5/assistive_sac_v5_30000_steps.zip"
)

parser = argparse.ArgumentParser()
parser.add_argument("--goal-x", type=float, default=-5.0)
parser.add_argument("--goal-y", type=float, default=3.0)
args = parser.parse_args()

FINAL_GOAL = (args.goal_x, args.goal_y)

WAYPOINTS = generate_room_route(
    "room1",
    FINAL_GOAL,
)

WAYPOINT_TOLERANCE = 0.35

MAX_LINEAR = 0.6
MAX_ANGULAR = 1.5

MAX_STEPS_PER_WAYPOINT = 350


class V6WaypointTest(Node):

    def __init__(self):
        super().__init__("v6_waypoint_test")

        self.state = None
        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = None
        self.min_lidar = 15.0

        self.state_sub = self.create_subscription(
            Float32MultiArray,
            "/dr_state",
            self.state_cb,
            10,
        )

        self.odom_sub = self.create_subscription(
            Odometry,
            "/odom",
            self.odom_cb,
            10,
        )

        self.scan_sub = self.create_subscription(
            LaserScan,
            "/scan",
            self.scan_cb,
            10,
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )

        self.control_service = self.find_control_service()

    def find_control_service(self):
        output = subprocess.check_output(
            ["gz", "service", "-l"],
            text=True,
        )

        matches = re.findall(
            r"/world/[^ \n]+/control",
            output,
        )

        if not matches:
            raise RuntimeError("Gazebo world control service not found.")

        return matches[0]

    def state_cb(self, msg):
        values = np.asarray(msg.data, dtype=np.float32)

        if values.size == 48:
            self.state = values

    def odom_cb(self, msg):
        self.odom_x = float(msg.pose.pose.position.x)
        self.odom_y = float(msg.pose.pose.position.y)

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.odom_yaw = math.atan2(siny, cosy)

    def scan_cb(self, msg):
        valid = [
            r
            for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        self.min_lidar = min(valid) if valid else 15.0

    def publish_cmd(self, linear, angular):
        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)
        self.cmd_pub.publish(msg)

    def publish_goal(self, x, y):
        msg = PoseStamped()
        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def reset_gazebo(self):
        self.publish_cmd(0.0, 0.0)

        result = subprocess.run(
            [
                "gz",
                "service",
                "-s",
                self.control_service,
                "--reqtype",
                "gz.msgs.WorldControl",
                "--reptype",
                "gz.msgs.Boolean",
                "--timeout",
                "3000",
                "--req",
                "reset: {all: true}",
            ],
            text=True,
            capture_output=True,
            timeout=6,
        )

        if result.returncode != 0:
            raise RuntimeError(
                "Gazebo reset failed:\n" + result.stderr
            )

        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.odom_x is not None
                and self.odom_y is not None
                and abs(self.odom_x) < 0.15
                and abs(self.odom_y) < 0.15
            ):
                return

        raise RuntimeError("Robot did not return to origin.")

    def wait_for_sensors(self):
        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self, timeout_sec=0.05)

            if (
                self.state is not None
                and self.odom_x is not None
                and self.odom_y is not None
            ):
                return

        raise RuntimeError("Timed out waiting for sensors.")

    def distance_to(self, x, y):
        return math.hypot(
            x - self.odom_x,
            y - self.odom_y,
        )

    def run(self):

        self.get_logger().info("Loading 30k SAC checkpoint...")
        model = SAC.load(MODEL_PATH)

        self.get_logger().info("Resetting Gazebo...")
        self.reset_gazebo()
        self.wait_for_sensors()

        print()
        print("=" * 60)
        print("V6 HIERARCHICAL WAYPOINT TEST")
        print("30k SAC policy")
        print("=" * 60)

        for waypoint_index, (gx, gy) in enumerate(WAYPOINTS, start=1):

            print()
            print(
                f"WAYPOINT {waypoint_index}/{len(WAYPOINTS)} "
                f"-> ({gx:.2f}, {gy:.2f})"
            )

            # Give state fusion the new waypoint.
            for _ in range(10):
                self.publish_goal(gx, gy)
                rclpy.spin_once(self, timeout_sec=0.02)
                time.sleep(0.05)

            reached = False

            for step in range(1, MAX_STEPS_PER_WAYPOINT + 1):

                self.publish_goal(gx, gy)

                rclpy.spin_once(
                    self,
                    timeout_sec=0.03,
                )

                if self.state is None:
                    continue

                distance = self.distance_to(gx, gy)

                state_distance = float(self.state[12]) * 20.0
                state_bearing = math.degrees(
                    float(self.state[13]) * math.pi
                )

                # Hard emergency stop only.
                if self.min_lidar < 0.28:
                    self.publish_cmd(0.0, 0.0)

                    print(
                        f"[{step:03d}] "
                        f"DIST={distance:5.2f}m "
                        f"STATE={state_distance:5.2f}m "
                        f"BEAR={state_bearing:+6.1f}° "
                        f"LIDAR={self.min_lidar:4.2f} "
                        f"!!! EMERGENCY STOP !!!",
                        flush=True,
                    )

                    time.sleep(0.2)
                    continue

                if distance <= WAYPOINT_TOLERANCE:

                    self.publish_cmd(0.0, 0.0)

                    print(
                        f"[SUCCESS] Waypoint {waypoint_index} reached "
                        f"at ({self.odom_x:.2f}, {self.odom_y:.2f})"
                    )

                    reached = True
                    time.sleep(0.4)
                    break

                # -------------------------------------------------
                # HYBRID CONTROL
                # SAC handles navigation.
                # Precision controller handles final approach.
                # -------------------------------------------------

                if distance < 1.0:

                    dx = gx - self.odom_x
                    dy = gy - self.odom_y

                    target_heading = math.atan2(dy, dx)

                    heading_error = (
                        target_heading - self.odom_yaw + math.pi
                    ) % (2.0 * math.pi) - math.pi

                    # Turn in place when badly misaligned.
                    if abs(heading_error) > math.radians(18.0):

                        linear = 0.0

                        angular = float(
                            np.clip(
                                2.0 * heading_error,
                                -1.2,
                                1.2,
                            )
                        )

                    else:

                        linear = float(
                            np.clip(
                                0.65 * distance,
                                0.08,
                                0.30,
                            )
                        )

                        angular = float(
                            np.clip(
                                2.2 * heading_error,
                                -0.8,
                                0.8,
                            )
                        )

                else:

                    action, _ = model.predict(
                        self.state,
                        deterministic=True,
                    )

                    action = np.asarray(
                        action,
                        dtype=np.float32,
                    )

                    linear = float(
                        np.clip(action[0], -1.0, 1.0)
                        * MAX_LINEAR
                    )

                    angular = float(
                        np.clip(action[1], -1.0, 1.0)
                        * MAX_ANGULAR
                    )

                self.publish_cmd(
                    linear,
                    angular,
                )

                if step % 10 == 0:

                    print(
                        f"[{step:03d}] "
                        f"DIST={distance:5.2f}m "
                        f"STATE={state_distance:5.2f}m "
                        f"BEAR={state_bearing:+6.1f}° "
                        f"CMD=({linear:+.2f},{angular:+.2f})",
                        flush=True,
                    )

                time.sleep(0.10)

            self.publish_cmd(0.0, 0.0)

            if not reached:

                print(
                    f"[FAILED] Waypoint {waypoint_index} "
                    f"not reached."
                )

                print("Stopping V6 test.")
                return

        self.publish_cmd(0.0, 0.0)

        print()
        print("=" * 60)
        print("V6 ROUTE COMPLETE")
        print("=" * 60)


def main():

    rclpy.init()

    node = V6WaypointTest()

    try:
        node.run()

    except KeyboardInterrupt:
        pass

    finally:
        node.publish_cmd(0.0, 0.0)
        node.destroy_node()
        rclpy.shutdown()


if __name__ == "__main__":
    main()
