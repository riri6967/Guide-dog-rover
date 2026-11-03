#!/usr/bin/env python3

import json
import math
import random
import re
import subprocess
import time

import gymnasium as gym
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from rclpy.node import Node


class SACROSBridge(Node):

    def __init__(self):
        super().__init__('sac_ros_bridge_v2')

        self.state = None
        self.state_seq = 0

        self.odom_x = None
        self.odom_y = None

        self.min_lidar = 15.0
        self.nearest_human = None

        self.create_subscription(
            Float32MultiArray,
            '/dr_state',
            self.state_callback,
            10
        )

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
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
            '/sac_cmd_vel',
            10
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

    def state_callback(self, msg):

        values = np.asarray(
            msg.data,
            dtype=np.float32
        )

        if values.size == 48:
            self.state = values
            self.state_seq += 1

    def odom_callback(self, msg):

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

    def scan_callback(self, msg):

        valid = [
            r for r in msg.ranges
            if math.isfinite(r)
            and msg.range_min <= r <= msg.range_max
        ]

        if valid:
            self.min_lidar = min(valid)
        else:
            self.min_lidar = 15.0

    def semantic_callback(self, msg):

        try:
            data = json.loads(msg.data)

            nearest = None

            for obj in data.get('objects', []):

                if (
                    obj.get('class', '').lower()
                    != 'person'
                ):
                    continue

                confidence = float(
                    obj.get('confidence', 0.0)
                )

                if confidence < 0.3:
                    continue

                distance = float(
                    obj.get('distance_m', 99.0)
                )

                if nearest is None or distance < nearest:
                    nearest = distance

            self.nearest_human = nearest

        except Exception:
            pass

    def publish_action(self, linear, angular):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def publish_goal(self, x, y):

        msg = PoseStamped()

        msg.header.stamp = self.get_clock().now().to_msg()
        msg.header.frame_id = 'odom'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def stop(self):

        self.publish_action(0.0, 0.0)


class AssistiveNavEnvV2(gym.Env):

    metadata = {'render_modes': []}

    def __init__(self):

        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros = SACROSBridge()

        # ---------------------------------------------------------
        # Multiple reachable target points around the initial area.
        # This is randomized goal selection rather than random
        # arbitrary points inside unknown obstacles.
        # ---------------------------------------------------------

        # =========================================================
        # V3 MAP-WIDE CURRICULUM GOALS
        # =========================================================

        # Main corridor
        self.corridor_goals = [
            (-9.0, -0.7), (-9.0, 0.0), (-9.0, 0.7),
            (-7.0, -0.7), (-7.0, 0.0), (-7.0, 0.7),
            (-5.0, -0.7), (-5.0, 0.0), (-5.0, 0.7),
            (-3.0, -0.7), (-3.0, 0.0), (-3.0, 0.7),
            (-1.0, -0.7), (-1.0, 0.0), (-1.0, 0.7),
            (1.0, -0.7), (1.0, 0.0), (1.0, 0.7),
            (3.0, -0.7), (3.0, 0.0), (3.0, 0.7),
            (5.0, -0.7), (5.0, 0.0), (5.0, 0.7),
            (7.0, -0.7), (7.0, 0.0), (7.0, 0.7),
            (9.0, -0.7), (9.0, 0.0), (9.0, 0.7),
        ]

        # Upper room
        self.room1_goals = [
            (-8.5, 2.3),
            (-7.0, 2.3),
            (-5.5, 2.3),
            (-4.0, 2.3),
            (-2.0, 2.3),

            (-8.5, 3.1),
            (-7.0, 3.1),
            (-5.5, 3.1),
            (-4.0, 3.1),
            (-2.0, 3.1),

            (-8.5, 7.0),
            (-7.0, 7.0),
            (-5.5, 7.0),
            (-4.0, 7.0),
            (-2.0, 7.0),

            (-8.5, 7.2),
            (-7.0, 7.2),
            (-5.5, 7.2),
            (-4.0, 7.2),
            (-2.0, 7.2),
        ]

        # Lower room
        self.room2_goals = [
            (1.7, -2.3),
            (3.9, -2.3),
            (5.2, -2.3),
            (6.5, -2.3),
            (8.3, -2.3),

            (1.7, -3.0),
            (4.0, -3.0),
            (5.2, -3.0),
            (6.5, -3.0),
            (8.3, -3.0),

            (1.7, -7.0),
            (4.0, -7.0),
            (5.2, -7.0),
            (6.5, -7.0),
            (8.3, -7.0),

            (1.7, -7.2),
            (4.0, -7.5),
            (5.2, -7.2),
            (6.5, -7.5),
            (8.3, -7.2),
        ]

        # Full map pool
        self.goal_candidates = (
            self.corridor_goals
            + self.room1_goals
            + self.room2_goals
        )

        # Automatic curriculum counter
        self.episode_count = 0


        self.goal_x = 5.0
        self.goal_y = 0.0

        self.max_episode_steps = 600

        self.max_linear_speed = 0.6
        self.max_angular_speed = 1.5

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(48,),
            dtype=np.float32
        )

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        self.episode_steps = 0

        self.previous_goal_distance = None
        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.path_length = 0.0

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        self.control_service = (
            self.find_gazebo_control_service()
        )

    def get_logger(self):
        return self.ros.get_logger()

    def find_gazebo_control_service(self):

        try:

            output = subprocess.check_output(
                ['gz', 'service', '-l'],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=5
            )

        except Exception as e:

            raise RuntimeError(
                f'Could not list Gazebo services: {e}'
            )

        matches = re.findall(
            r'/world/[^ \n]+/control',
            output
        )

        if not matches:

            raise RuntimeError(
                'No Gazebo world control service found.'
            )

        return matches[0]

    def reset_gazebo(self):

        self.ros.stop()

        request = 'reset: {all: true}'

        result = subprocess.run(
            [
                'gz',
                'service',
                '-s',
                self.control_service,
                '--reqtype',
                'gz.msgs.WorldControl',
                '--reptype',
                'gz.msgs.Boolean',
                '--timeout',
                '3000',
                '--req',
                request
            ],
            text=True,
            capture_output=True,
            timeout=6
        )

        if result.returncode != 0:

            raise RuntimeError(
                'Gazebo reset failed:\n'
                + result.stderr
            )

        # Do not continue until Gazebo has actually returned the
        # rover to the reset pose. A fixed sleep can accept stale odometry.
        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:
            rclpy.spin_once(self.ros, timeout_sec=0.05)

            if (
                self.ros.odom_x is not None
                and self.ros.odom_y is not None
                and abs(self.ros.odom_x) < 0.15
                and abs(self.ros.odom_y) < 0.15
            ):
                break

        else:
            raise RuntimeError(
                "Gazebo reset completed, but rover did not return "
                "near the origin within 5 seconds."
            )

        time.sleep(0.1)

    def publish_goal_reliably(self):

        # Publish repeatedly with a small delay so the separate
        # state_fusion node has time to receive the new goal.
        for _ in range(10):

            self.ros.publish_goal(
                self.goal_x,
                self.goal_y
            )

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.02
            )

            time.sleep(0.05)

    def wait_for_state(self, timeout=5.0):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.03
            )

            if (
                self.ros.state is not None
                and self.ros.odom_x is not None
            ):
                return self.ros.state.copy()

        raise RuntimeError(
            'Timed out waiting for /dr_state and /odom.'
        )

    def wait_for_goal_state(self, timeout=5.0):

        expected_distance = math.hypot(
            self.goal_x,
            self.goal_y
        )

        expected_bearing = math.atan2(
            self.goal_y,
            self.goal_x
        )

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.03
            )

            if (
                self.ros.state is None
                or self.ros.odom_x is None
                or self.ros.odom_y is None
            ):
                continue

            state = self.ros.state.copy()

            # State layout:
            # state[12] = normalized goal distance
            # state[13] = normalized goal bearing
            actual_distance = float(state[12]) * 20.0
            actual_bearing = float(state[13]) * math.pi

            bearing_error = math.atan2(
                math.sin(actual_bearing - expected_bearing),
                math.cos(actual_bearing - expected_bearing)
            )

            robot_at_reset = (
                abs(self.ros.odom_x) < 0.15
                and abs(self.ros.odom_y) < 0.15
            )

            goal_matches = (
                abs(actual_distance - expected_distance) < 0.20
                and abs(bearing_error) < 0.08
            )

            if robot_at_reset and goal_matches:
                return state

        raise RuntimeError(
            'Timed out waiting for DR state with the current goal.'
        )

    def wait_for_new_state(
        self,
        previous_seq,
        timeout=0.5
    ):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.02
            )

            if self.ros.state_seq > previous_seq:
                return self.ros.state.copy()

        if self.ros.state is not None:
            return self.ros.state.copy()

        raise RuntimeError(
            'No /dr_state received.'
        )

    def action_to_velocity(self, action):

        action = np.clip(
            action,
            -1.0,
            1.0
        )

        # Forward-only assistive rover
        linear = (
            (float(action[0]) + 1.0)
            * 0.5
            * self.max_linear_speed
        )

        angular = (
            float(action[1])
            * self.max_angular_speed
        )

        return linear, angular

    @staticmethod
    def goal_distance_from_state(state):

        return float(state[12]) * 20.0

    def calculate_reward(
        self,
        state,
        action,
        linear_velocity
    ):

        goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        reward = -0.01

        # ---------------------------------------------------------
        # 1. Progress toward goal
        # ---------------------------------------------------------

        progress = 0.0

        if self.previous_goal_distance is not None:

            progress = (
                self.previous_goal_distance
                - goal_distance
            )

            reward += 10.0 * progress

        self.previous_goal_distance = goal_distance

        # ---------------------------------------------------------
        # 2. Goal heading alignment
        #
        # state[13] is normalized goal bearing [-1,1]
        # ---------------------------------------------------------

        goal_bearing = (
            float(state[13]) * math.pi
        )

        heading_alignment = math.cos(
            goal_bearing
        )

        # Reward forward motion when pointing toward goal.
        reward += (
            0.30 *
            linear_velocity *
            heading_alignment
        )

        # Discourage travelling strongly away from goal.
        if heading_alignment < 0.0:

            reward -= (
                0.20 *
                linear_velocity
            )

        # ---------------------------------------------------------
        # 3. LiDAR proximity
        # ---------------------------------------------------------

        min_lidar = float(
            self.ros.min_lidar
        )

        if min_lidar < 1.0:

            reward -= (
                2.0 *
                (1.0 - min_lidar)
            )

        # ---------------------------------------------------------
        # 4. Human proximity
        # ---------------------------------------------------------

        if self.ros.nearest_human is not None:

            human_distance = (
                self.ros.nearest_human
            )

            if human_distance < 1.5:

                reward -= (
                    2.5 *
                    (1.5 - human_distance)
                )

            if human_distance < 0.6:

                reward -= 8.0

        # ---------------------------------------------------------
        # 5. Smoothness
        # ---------------------------------------------------------

        action_change = np.linalg.norm(
            action -
            self.previous_action
        )

        reward -= (
            0.04 *
            float(action_change)
        )

        # ---------------------------------------------------------
        # 6. Excessive steering
        # ---------------------------------------------------------

        reward -= (
            0.015 *
            abs(float(action[1]))
        )

        # ---------------------------------------------------------
        # 7. Stagnation
        # ---------------------------------------------------------

        if progress < 0.002:

            self.stuck_steps += 1

        else:

            self.stuck_steps = 0

        if self.stuck_steps > 20:

            reward -= 0.05

        self.previous_action = action.copy()

        return reward

    def reset(
        self,
        *,
        seed=None,
        options=None
    ):

        super().reset(seed=seed)

        self.ros.state = None
        self.ros.state_seq = 0
        self.ros.odom_x = None
        self.ros.odom_y = None

        self.episode_steps = 0

        self.previous_goal_distance = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.path_length = 0.0

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        # =========================================================
        # AUTOMATIC CURRICULUM GOAL SELECTION
        # =========================================================

        self.episode_count += 1

        if self.episode_count <= 200:

            # Stage 1: learn the main corridor
            goal_pool = self.corridor_goals

        elif self.episode_count <= 500:

            # Stage 2: learn corridor + both rooms
            goal_pool = (
                self.corridor_goals
                + self.room1_goals
                + self.room2_goals
            )

        else:

            # Stage 3: full map
            goal_pool = self.goal_candidates

        self.goal_x, self.goal_y = random.choice(
            goal_pool
        )

        self.reset_gazebo()

        self.publish_goal_reliably()

        # Do not accept a stale DR state from the previous episode.
        state = self.wait_for_goal_state()

        self.previous_x = self.ros.odom_x
        self.previous_y = self.ros.odom_y

        self.previous_goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        info = {
            'goal_x': self.goal_x,
            'goal_y': self.goal_y,
            'goal_distance': (
                self.previous_goal_distance
            ),
            'path_length': 0.0
        }

        return state, info

    def step(self, action):

        action = np.asarray(
            action,
            dtype=np.float32
        )

        action = np.clip(
            action,
            -1.0,
            1.0
        )

        linear, angular = (
            self.action_to_velocity(action)
        )

        previous_seq = self.ros.state_seq

        self.ros.publish_action(
            linear,
            angular
        )

        state = self.wait_for_new_state(
            previous_seq
        )

        # ---------------------------------------------------------
        # Path length
        # ---------------------------------------------------------

        if (
            self.ros.odom_x is not None
            and self.previous_x is not None
        ):

            dx = (
                self.ros.odom_x
                - self.previous_x
            )

            dy = (
                self.ros.odom_y
                - self.previous_y
            )

            movement = math.sqrt(
                dx * dx + dy * dy
            )

            self.path_length += movement

            self.previous_x = self.ros.odom_x
            self.previous_y = self.ros.odom_y

        reward = self.calculate_reward(
            state,
            action,
            linear
        )

        goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        min_lidar = float(
            self.ros.min_lidar
        )

        goal_reached = (
            goal_distance < 0.45
        )

        collision = (
            min_lidar < 0.28
        )

        terminated = (
            goal_reached
            or collision
        )

        if goal_reached:

            reward += 100.0

        if collision:

            reward -= 100.0

        self.episode_steps += 1

        truncated = (
            self.episode_steps
            >= self.max_episode_steps
        )

        if terminated or truncated:

            self.ros.stop()

        info = {
            'goal_x': self.goal_x,
            'goal_y': self.goal_y,
            'goal_distance': goal_distance,
            'min_lidar_distance': min_lidar,
            'goal_reached': goal_reached,
            'collision': collision,
            'path_length': self.path_length,
            'linear_velocity': linear,
            'angular_velocity': angular
        }

        return (
            state,
            float(reward),
            terminated,
            truncated,
            info
        )

    def close(self):

        try:

            self.ros.stop()

            if self.ros is not None:
                self.ros.destroy_node()

            if rclpy.ok():
                rclpy.shutdown()

        except Exception:
            pass


if __name__ == '__main__':

    env = AssistiveNavEnvV2()

    try:

        obs, info = env.reset()

        print(
            'Observation shape:',
            obs.shape
        )

        print(
            'Selected goal:',
            info['goal_x'],
            info['goal_y']
        )

        print(
            'Initial goal distance:',
            info['goal_distance']
        )

    finally:

        env.close()
