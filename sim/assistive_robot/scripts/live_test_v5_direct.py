import math
import subprocess
import time

import numpy as np
import rclpy
from rclpy.node import Node

from geometry_msgs.msg import Twist, PoseStamped
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray

from stable_baselines3 import SAC


MODEL = "/home/pushkar/assistive_robot_ws/models/sac_v5/assistive_sac_v5_30000_steps.zip"

GOAL_X = -5.0
GOAL_Y = 3.0

MAX_STEPS = 300
DT = 0.10


class LiveSAC(Node):

    def __init__(self):
        super().__init__("live_sac_v5_test")

        self.model = SAC.load(MODEL)

        self.state = None
        self.x = None
        self.y = None
        self.yaw = None

        self.cmd_pub = self.create_publisher(
            Twist,
            "/cmd_vel",
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10
        )

        self.create_subscription(
            Float32MultiArray,
            "/dr_state",
            self.state_callback,
            10
        )

        self.create_subscription(
            Odometry,
            "/odom",
            self.odom_callback,
            10
        )

        self.get_logger().info("Live V5 30k test started.")

    def state_callback(self, msg):
        if len(msg.data) == 48:
            self.state = np.asarray(
                msg.data,
                dtype=np.float32
            )

    def odom_callback(self, msg):

        self.x = msg.pose.pose.position.x
        self.y = msg.pose.pose.position.y

        q = msg.pose.pose.orientation

        siny = 2.0 * (q.w * q.z + q.x * q.y)
        cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)

        self.yaw = math.atan2(siny, cosy)

    def publish_goal(self):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = "odom"

        msg.pose.position.x = GOAL_X
        msg.pose.position.y = GOAL_Y
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def publish_action(self, action):

        msg = Twist()

        msg.linear.x = float(action[0]) * 0.60
        msg.angular.z = float(action[1]) * 1.50

        self.cmd_pub.publish(msg)

    def stop(self):

        msg = Twist()
        self.cmd_pub.publish(msg)


def reset_gazebo():

    print("Resetting Gazebo...")

    result = subprocess.run(
        [
            "gz",
            "service",
            "-s",
            "/world/assistive_world/control",
            "--reqtype",
            "gz.msgs.WorldControl",
            "--reptype",
            "gz.msgs.Boolean",
            "--timeout",
            "3000",
            "--req",
            "reset: {all: true}",
        ],
        capture_output=True,
        text=True,
        timeout=6,
    )

    if result.returncode != 0:
        print(result.stderr)
        raise RuntimeError("Gazebo reset failed")


rclpy.init()

node = LiveSAC()

reset_gazebo()

# Give Gazebo time to settle.
time.sleep(1.0)

# Publish the fixed ROOM1 goal repeatedly.
for _ in range(20):

    node.publish_goal()

    rclpy.spin_once(
        node,
        timeout_sec=0.05
    )

    time.sleep(0.05)

print()
print("======================================")
print("LIVE SAC V5 TEST")
print(f"Goal = ({GOAL_X}, {GOAL_Y})")
print("======================================")
print()

# Wait for state.
deadline = time.monotonic() + 5.0

while (
    rclpy.ok()
    and node.state is None
    and time.monotonic() < deadline
):

    rclpy.spin_once(
        node,
        timeout_sec=0.05
    )

if node.state is None:
    node.stop()
    node.destroy_node()
    rclpy.shutdown()
    raise RuntimeError("No /dr_state received.")


for step in range(1, MAX_STEPS + 1):

    # Keep goal alive.
    node.publish_goal()

    rclpy.spin_once(
        node,
        timeout_sec=0.02
    )

    obs = node.state.copy()

    action, _ = node.model.predict(
        obs,
        deterministic=True
    )

    distance = float(obs[12]) * 20.0
    bearing = float(obs[13]) * 180.0
    yaw = float(obs[47]) * 180.0

    linear = float(action[0]) * 0.60
    angular = float(action[1]) * 1.50

    node.publish_action(action)

    if step % 5 == 0:

        print(
            f"[{step:03d}] "
            f"dist={distance:6.2f}m | "
            f"bearing={bearing:+7.2f}° | "
            f"yaw={yaw:+7.2f}° | "
            f"action=({action[0]:+.3f}, {action[1]:+.3f}) | "
            f"cmd=({linear:+.3f}, {angular:+.3f})"
        )

    if distance < 0.45:

        print()
        print("========== SUCCESS ==========")
        print(f"Reached goal in {step} steps.")
        break

    time.sleep(DT)

else:

    print()
    print("========== TIMEOUT ==========")
    print(f"Goal not reached after {MAX_STEPS} steps.")


node.stop()
time.sleep(0.2)

node.destroy_node()
rclpy.shutdown()
