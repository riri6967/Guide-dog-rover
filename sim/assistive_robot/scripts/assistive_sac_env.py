#!/usr/bin/env python3

import math
import re
import subprocess
import time

import gymnasium as gym
import numpy as np
import rclpy

from geometry_msgs.msg import Twist
from std_msgs.msg import Float32MultiArray
from rclpy.node import Node


class SACROSBridge(Node):

    def __init__(self):
        super().__init__('sac_ros_bridge')

        self.state = None
        self.state_seq = 0

        self.state_sub = self.create_subscription(
            Float32MultiArray,
            '/dr_state',
            self.state_callback,
            10
        )

        self.cmd_pub = self.create_publisher(
            Twist,
            '/sac_cmd_vel',
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

    def publish_action(self, linear, angular):

        msg = Twist()
        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop(self):

        self.publish_action(0.0, 0.0)


class AssistiveNavEnv(gym.Env):

    metadata = {
        'render_modes': []
    }

    def __init__(
        self,
        max_episode_steps=600,
        max_linear_speed=0.6,
        max_angular_speed=1.5
    ):

        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros = SACROSBridge()

        self.max_episode_steps = max_episode_steps
        self.max_linear_speed = max_linear_speed
        self.max_angular_speed = max_angular_speed

        # 48-dimensional multimodal state
        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(48,),
            dtype=np.float32
        )

        # Action:
        # a[0] -> forward velocity
        # a[1] -> angular velocity
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

        self.control_service = self.find_gazebo_control_service()

        self.get_logger().info(
            f'Gazebo reset service: {self.control_service}'
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
                'No Gazebo world control service found. '
                'Make sure Gazebo is running.'
            )

        return matches[0]

    def reset_gazebo(self):

        self.ros.stop()

        request = 'reset: {all: true}'

        try:

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

        except Exception as e:

            raise RuntimeError(
                f'Gazebo reset failed: {e}'
            )

        if result.returncode != 0:

            raise RuntimeError(
                'Gazebo reset returned an error:\n'
                + result.stderr
            )

        time.sleep(0.3)

    def wait_for_state(self, timeout=5.0):

        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:

            if self.ros.state is not None:
                return self.ros.state.copy()

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.05
            )

        raise RuntimeError(
            'Timed out waiting for /dr_state. '
            'Make sure state_fusion.py is running.'
        )

    def step_until_new_state(
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

        # Return the latest state if simulation is slightly slower.
        if self.ros.state is not None:
            return self.ros.state.copy()

        raise RuntimeError(
            'No state received from /dr_state.'
        )

    def action_to_velocity(self, action):

        action = np.clip(
            action,
            -1.0,
            1.0
        )

        # [-1,1] -> [0, max_linear_speed]
        linear = (
            (float(action[0]) + 1.0)
            * 0.5
            * self.max_linear_speed
        )

        # [-1,1] -> [-max_angular, max_angular]
        angular = (
            float(action[1])
            * self.max_angular_speed
        )

        return linear, angular

    @staticmethod
    def goal_distance_from_state(state):

        # State index 12 = normalized goal distance.
        return float(state[12]) * 20.0

    @staticmethod
    def minimum_lidar_distance(state):

        # First 12 values are normalized 0-15m LiDAR sectors.
        lidar = np.asarray(
            state[:12],
            dtype=np.float32
        )

        return float(
            np.clip(
                np.min(lidar) * 15.0,
                0.0,
                15.0
            )
        )

    def human_information(self, state):

        nearest_human = None

        # Semantic objects:
        # 14 + i*5
        #
        # class
        # confidence
        # distance
        # bearing
        # vertical

        person_class_norm = 1.0 / 11.0

        for i in range(5):

            idx = 14 + i * 5

            class_norm = float(state[idx])
            confidence = float(state[idx + 1])
            distance = float(state[idx + 2]) * 10.0

            if (
                abs(
                    class_norm -
                    person_class_norm
                ) < 0.02
                and confidence > 0.3
            ):

                if (
                    nearest_human is None
                    or distance <
                    nearest_human
                ):
                    nearest_human = distance

        return nearest_human

    def calculate_reward(
        self,
        state,
        action
    ):

        goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        reward = -0.01

        # ---------------------------------------------------------
        # Goal progress
        # ---------------------------------------------------------

        if self.previous_goal_distance is not None:

            progress = (
                self.previous_goal_distance
                - goal_distance
            )

            reward += 5.0 * progress

        self.previous_goal_distance = goal_distance

        # ---------------------------------------------------------
        # LiDAR safety penalty
        # ---------------------------------------------------------

        min_lidar = (
            self.minimum_lidar_distance(
                state
            )
        )

        if min_lidar < 1.0:

            reward -= 3.0 * (
                1.0 - min_lidar
            )

        # ---------------------------------------------------------
        # Human proximity penalty
        # ---------------------------------------------------------

        nearest_human = (
            self.human_information(
                state
            )
        )

        if nearest_human is not None:

            if nearest_human < 1.5:

                reward -= 3.0 * (
                    1.5 -
                    nearest_human
                )

            if nearest_human < 0.6:

                reward -= 10.0

        # ---------------------------------------------------------
        # Smoothness penalty
        # ---------------------------------------------------------

        action_change = np.linalg.norm(
            action -
            self.previous_action
        )

        reward -= (
            0.05 *
            float(action_change)
        )

        # ---------------------------------------------------------
        # Turning penalty
        # ---------------------------------------------------------

        reward -= (
            0.02 *
            abs(float(action[1]))
        )

        self.previous_action = (
            action.copy()
        )

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

        self.episode_steps = 0

        self.previous_goal_distance = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.reset_gazebo()

        state = self.wait_for_state()

        self.previous_goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        info = {
            'goal_distance': (
                self.previous_goal_distance
            )
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
            self.action_to_velocity(
                action
            )
        )

        previous_seq = self.ros.state_seq

        self.ros.publish_action(
            linear,
            angular
        )

        state = self.step_until_new_state(
            previous_seq
        )

        reward = self.calculate_reward(
            state,
            action
        )

        goal_distance = (
            self.goal_distance_from_state(
                state
            )
        )

        min_lidar = (
            self.minimum_lidar_distance(
                state
            )
        )

        # Goal reached
        goal_reached = (
            goal_distance < 0.4
        )

        # Extremely close LiDAR return
        collision = (
            min_lidar < 0.25
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
            'goal_distance': goal_distance,
            'min_lidar_distance': min_lidar,
            'goal_reached': goal_reached,
            'collision': collision,
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

            if rclpy.ok():
                self.ros.destroy_node()
                rclpy.shutdown()

        except Exception:
            pass


if __name__ == '__main__':

    env = AssistiveNavEnv()

    try:

        obs, info = env.reset()

        print(
            'Observation shape:',
            obs.shape
        )

        print(
            'Initial goal distance:',
            info['goal_distance']
        )

    finally:

        env.close()
