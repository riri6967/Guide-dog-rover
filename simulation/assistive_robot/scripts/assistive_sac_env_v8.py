#!/usr/bin/env python3

import math
import random
import re
import subprocess
import time
from collections import deque

import gymnasium as gym
import numpy as np
import rclpy
import torch
import torch.nn as nn

from gymnasium import spaces

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan

from rclpy.node import Node

from stable_baselines3.common.torch_layers import BaseFeaturesExtractor


# ============================================================
# CONFIG
# ============================================================

NUM_RAYS = 180
HISTORY = 4

LIDAR_MAX_RANGE = 15.0

MAX_LINEAR_SPEED = 0.60
MAX_ANGULAR_SPEED = 1.50

TARGET_REACHED_RADIUS = 0.55

MISSION_TIMEOUT_STEPS = 3000

CONTROL_DT = 0.10


# ============================================================
# 1D CNN FEATURE EXTRACTOR
# ============================================================

class LidarCNNExtractor(
    BaseFeaturesExtractor
):

    def __init__(
        self,
        observation_space: spaces.Box,
        features_dim: int = 256
    ):

        super().__init__(
            observation_space,
            features_dim
        )

        self.lidar_dim = (
            NUM_RAYS * HISTORY
        )

        self.aux_dim = 7

        # Four channels = four time frames.
        self.cnn = nn.Sequential(

            nn.Conv1d(
                HISTORY,
                32,
                kernel_size=7,
                stride=2,
                padding=3
            ),

            nn.ReLU(),

            nn.Conv1d(
                32,
                64,
                kernel_size=7,
                stride=2,
                padding=3
            ),

            nn.ReLU(),

            nn.Conv1d(
                64,
                96,
                kernel_size=5,
                stride=2,
                padding=2
            ),

            nn.ReLU(),

            nn.Conv1d(
                96,
                128,
                kernel_size=5,
                stride=2,
                padding=2
            ),

            nn.ReLU(),

            nn.AdaptiveAvgPool1d(1)
        )

        self.aux_net = nn.Sequential(

            nn.Linear(
                self.aux_dim,
                64
            ),

            nn.ReLU(),

            nn.Linear(
                64,
                64
            ),

            nn.ReLU()
        )

        self.fusion = nn.Sequential(

            nn.Linear(
                128 + 64,
                features_dim
            ),

            nn.ReLU(),

            nn.Linear(
                features_dim,
                features_dim
            ),

            nn.ReLU()
        )

    def forward(self, observations):

        lidar = observations[
            :,
            :self.lidar_dim
        ]

        aux = observations[
            :,
            self.lidar_dim:
        ]

        lidar = lidar.view(
            -1,
            HISTORY,
            NUM_RAYS
        )

        lidar_features = self.cnn(
            lidar
        )

        lidar_features = (
            lidar_features
            .squeeze(-1)
        )

        aux_features = self.aux_net(
            aux
        )

        fused = torch.cat(
            [
                lidar_features,
                aux_features
            ],
            dim=1
        )

        return self.fusion(
            fused
        )


# ============================================================
# ROS BRIDGE
# ============================================================

class SACROSBridgeV8(Node):

    def __init__(self):

        super().__init__(
            'sac_ros_bridge_v8'
        )

        self.odom_x = None
        self.odom_y = None
        self.odom_yaw = 0.0

        self.robot_v = 0.0
        self.robot_w = 0.0

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.scan = np.ones(
            NUM_RAYS,
            dtype=np.float32
        )

        self.scan_ready = False
        self.odom_ready = False

        self.scan_seq = 0
        self.odom_seq = 0

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

        self.create_subscription(
            PoseStamped,
            '/goal_pose',
            self.goal_callback,
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

    # --------------------------------------------------------
    # SCAN
    # --------------------------------------------------------

    def scan_callback(
        self,
        msg: LaserScan
    ):

        ranges = np.asarray(
            msg.ranges,
            dtype=np.float32
        )

        if ranges.size == 0:
            return

        ranges = np.nan_to_num(
            ranges,
            nan=msg.range_max,
            posinf=msg.range_max,
            neginf=msg.range_max
        )

        ranges = np.clip(
            ranges,
            msg.range_min,
            msg.range_max
        )

        angles = (
            msg.angle_min
            +
            np.arange(
                ranges.size,
                dtype=np.float32
            )
            * msg.angle_increment
        )

        # Wrap to [-pi, pi)
        wrapped = (
            angles + math.pi
        ) % (
            2.0 * math.pi
        ) - math.pi

        order = np.argsort(
            wrapped
        )

        wrapped = wrapped[order]
        ranges = ranges[order]

        # Remove possible duplicate endpoint.
        keep = np.ones(
            len(wrapped),
            dtype=bool
        )

        if len(wrapped) > 1:

            keep[1:] = (
                np.diff(wrapped) > 1e-6
            )

        wrapped = wrapped[keep]
        ranges = ranges[keep]

        if len(wrapped) < 2:
            return

        # Circular extension.
        extended_angles = np.concatenate([
            wrapped[-1:] - 2.0 * math.pi,
            wrapped,
            wrapped[:1] + 2.0 * math.pi
        ])

        extended_ranges = np.concatenate([
            ranges[-1:],
            ranges,
            ranges[:1]
        ])

        target_angles = np.linspace(
            -math.pi,
            math.pi,
            NUM_RAYS,
            endpoint=False,
            dtype=np.float32
        )

        resampled = np.interp(
            target_angles,
            extended_angles,
            extended_ranges
        )

        resampled = np.nan_to_num(
            resampled,
            nan=LIDAR_MAX_RANGE,
            posinf=LIDAR_MAX_RANGE,
            neginf=0.0
        )

        resampled = np.clip(
            resampled,
            0.0,
            LIDAR_MAX_RANGE
        )

        self.scan = (
            resampled / LIDAR_MAX_RANGE
        ).astype(
            np.float32
        )

        self.scan_ready = True
        self.scan_seq += 1

    # --------------------------------------------------------
    # ODOM
    # --------------------------------------------------------

    def odom_callback(
        self,
        msg: Odometry
    ):

        self.odom_x = float(
            msg.pose.pose.position.x
        )

        self.odom_y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        siny_cosp = (
            2.0 *
            (
                q.w * q.z
                +
                q.x * q.y
            )
        )

        cosy_cosp = (
            1.0 -
            2.0 *
            (
                q.y * q.y
                +
                q.z * q.z
            )
        )

        self.odom_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.robot_v = float(
            msg.twist.twist.linear.x
        )

        self.robot_w = float(
            msg.twist.twist.angular.z
        )

        self.odom_ready = True
        self.odom_seq += 1

    # --------------------------------------------------------
    # GOAL
    # --------------------------------------------------------

    def goal_callback(
        self,
        msg: PoseStamped
    ):

        self.goal_x = float(
            msg.pose.position.x
        )

        self.goal_y = float(
            msg.pose.position.y
        )

    def publish_goal(
        self,
        x,
        y
    ):

        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock()
            .now()
            .to_msg()
        )

        msg.header.frame_id = 'odom'

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    # --------------------------------------------------------
    # COMMAND
    # --------------------------------------------------------

    def publish_action(
        self,
        linear,
        angular
    ):

        msg = Twist()

        msg.linear.x = float(linear)
        msg.angular.z = float(angular)

        self.cmd_pub.publish(msg)

    def stop(self):

        self.publish_action(
            0.0,
            0.0
        )


# ============================================================
# ENVIRONMENT
# ============================================================

class AssistiveNavEnvV8(gym.Env):

    metadata = {
        'render_modes': []
    }

    # This route is PRIVATE to the environment.
    # The policy never receives these simulator coordinates.
    MISSION_ROUTE = [

        (4.0, 0.0),

        (5.0, -1.0),
        (5.0, -2.2),
        (5.0, -3.0),
        (5.0, -4.0),
        (5.0, -5.0),

        (5.0, -4.0),
        (5.0, -2.2),
        (5.0, -1.0),

        (2.0, 0.0),

        (-3.0, 0.0),

        (-5.0, 1.2),
        (-5.0, 2.2),
        (-5.0, 3.0),
        (-5.0, 4.5),
        (-5.0, 5.0),

        (-7.0, 5.0),
        (-3.0, 5.0),

        (-5.0, 3.0),
        (-5.0, 2.2),
        (-5.0, 1.2),

        (-3.0, 0.0),

        (0.0, 0.0)
    ]

    def __init__(self):

        super().__init__()

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros = SACROSBridgeV8()

        self.max_linear_speed = (
            MAX_LINEAR_SPEED
        )

        self.max_angular_speed = (
            MAX_ANGULAR_SPEED
        )

        self.max_episode_steps = (
            MISSION_TIMEOUT_STEPS
        )

        # ----------------------------------------------------
        # OBSERVATION
        # ----------------------------------------------------

        self.lidar_dim = (
            NUM_RAYS * HISTORY
        )

        self.aux_dim = 7

        self.observation_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(
                self.lidar_dim
                +
                self.aux_dim,
            ),
            dtype=np.float32
        )

        # ----------------------------------------------------
        # ACTION
        # ----------------------------------------------------

        self.action_space = spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        self.scan_history = deque(
            maxlen=HISTORY
        )

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.goal_x = 0.0
        self.goal_y = 0.0

        self.target_index = 0

        self.episode_steps = 0

        self.previous_goal_distance = None

        self.previous_x = None
        self.previous_y = None

        self.path_length = 0.0

        self.control_service = (
            self.find_gazebo_control_service()
        )

    # ========================================================
    # GAZEBO RESET
    # ========================================================

    def find_gazebo_control_service(self):

        try:

            output = subprocess.check_output(
                [
                    'gz',
                    'service',
                    '-l'
                ],
                text=True,
                stderr=subprocess.STDOUT,
                timeout=5
            )

        except Exception as e:

            raise RuntimeError(
                f'Could not list Gazebo services: {e}'
            )

        matches = re.findall(
            r'/world/[^\s\n]+/control',
            output
        )

        if not matches:

            raise RuntimeError(
                'No Gazebo world control service found.'
            )

        return matches[0]

    def reset_gazebo(self):

        self.ros.stop()

        # Force fresh post-reset sensor observations.
        self.ros.scan_ready = False
        self.ros.odom_ready = False

        self.ros.scan_seq = 0
        self.ros.odom_seq = 0

        request = (
            'reset: {all: true}'
        )

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
                +
                result.stderr
            )

        deadline = (
            time.monotonic() + 5.0
        )

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.05
            )

            if (
                self.ros.odom_ready
                and
                self.ros.scan_ready
                and
                self.ros.odom_x is not None
                and
                self.ros.odom_y is not None
            ):

                return

        raise RuntimeError(
            'Gazebo reset completed, but '
            'the rover did not return to '
            'the expected initial pose.'
        )

    # ========================================================
    # GOAL
    # ========================================================

    def publish_goal_reliably(self):

        for _ in range(10):

            self.ros.publish_goal(
                self.goal_x,
                self.goal_y
            )

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.02
            )

            time.sleep(0.03)

    # ========================================================
    # GOAL GEOMETRY
    # ========================================================

    @staticmethod
    def normalize_angle(
        angle
    ):

        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def goal_geometry(self):

        dx = (
            self.goal_x
            -
            self.ros.odom_x
        )

        dy = (
            self.goal_y
            -
            self.ros.odom_y
        )

        distance = math.hypot(
            dx,
            dy
        )

        world_angle = math.atan2(
            dy,
            dx
        )

        bearing = self.normalize_angle(
            world_angle
            -
            self.ros.odom_yaw
        )

        return (
            distance,
            bearing
        )

    # ========================================================
    # OBSERVATION
    # ========================================================

    def build_observation(self):

        lidar = np.concatenate(
            list(self.scan_history)
        )

        distance, bearing = (
            self.goal_geometry()
        )

        goal_distance_norm = min(
            distance / 20.0,
            1.0
        )

        aux = np.array(
            [
                goal_distance_norm,

                math.sin(bearing),

                math.cos(bearing),

                np.clip(
                    self.ros.robot_v
                    /
                    self.max_linear_speed,
                    -1.0,
                    1.0
                ),

                np.clip(
                    self.ros.robot_w
                    /
                    self.max_angular_speed,
                    -1.0,
                    1.0
                ),

                self.previous_action[0],

                self.previous_action[1]
            ],
            dtype=np.float32
        )

        observation = np.concatenate(
            [
                lidar,
                aux
            ]
        )

        return np.clip(
            observation,
            -1.0,
            1.0
        ).astype(
            np.float32
        )

    # ========================================================
    # WAIT FOR SENSOR
    # ========================================================

    def wait_for_sensor_data(
        self,
        timeout=5.0
    ):

        deadline = (
            time.monotonic()
            +
            timeout
        )

        while (
            time.monotonic()
            <
            deadline
        ):

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.03
            )

            if (
                self.ros.odom_ready
                and
                self.ros.scan_ready
            ):

                return

        raise RuntimeError(
            'Timed out waiting for '
            'odom and raw /scan.'
        )

    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
        *,
        seed=None,
        options=None
    ):

        super().reset(
            seed=seed
        )

        self.reset_gazebo()

        self.target_index = 0

        self.goal_x, self.goal_y = (
            self.MISSION_ROUTE[
                self.target_index
            ]
        )

        self.publish_goal_reliably()

        self.wait_for_sensor_data()

        # Allow reset transients, supervisor state, and Gazebo
        # physics to settle before the first RL transition.
        settle_deadline = (
            time.monotonic()
            + 1.0
        )

        while time.monotonic() < settle_deadline:
            rclpy.spin_once(
                self.ros,
                timeout_sec=0.02
            )

        current_scan = (
            self.ros.scan.copy()
        )

        self.scan_history.clear()

        for _ in range(HISTORY):
            self.scan_history.append(
                current_scan.copy()
            )

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.episode_steps = 0

        self.path_length = 0.0

        self.previous_x = (
            self.ros.odom_x
        )

        self.previous_y = (
            self.ros.odom_y
        )

        distance, _ = (
            self.goal_geometry()
        )

        self.previous_goal_distance = (
            distance
        )

        observation = (
            self.build_observation()
        )

        info = {
            'target_index':
                self.target_index,
            'targets_total':
                len(self.MISSION_ROUTE),
            'goal_distance':
                distance,
            'mission_complete':
                False,
            'path_length':
                0.0
        }

        print()
        print(
            '=' * 72
        )
        print(
            '[V8 MISSION START]'
        )
        print(
            f'Target 1/'
            f'{len(self.MISSION_ROUTE)}'
        )
        print(
            f'Goal: '
            f'({self.goal_x:.2f}, '
            f'{self.goal_y:.2f})'
        )
        print(
            '=' * 72
        )

        return (
            observation,
            info
        )

    # ========================================================
    # ACTION SMOOTHING
    # ========================================================

    def smooth_action(
        self,
        action
    ):

        action = np.asarray(
            action,
            dtype=np.float32
        )

        action = np.clip(
            action,
            -1.0,
            1.0
        )

        max_delta = 0.18

        delta = (
            action
            -
            self.previous_action
        )

        delta = np.clip(
            delta,
            -max_delta,
            max_delta
        )

        return np.clip(
            self.previous_action
            +
            delta,
            -1.0,
            1.0
        )

    # ========================================================
    # REWARD
    # ========================================================

    def compute_reward(
        self,
        action,
        linear_velocity,
        angular_velocity
    ):

        distance, bearing = (
            self.goal_geometry()
        )

        reward = -0.008

        # ----------------------------------------------------
        # Progress
        # ----------------------------------------------------

        if (
            self.previous_goal_distance
            is not None
        ):

            progress = (
                self.previous_goal_distance
                -
                distance
            )

            reward += (
                8.0 *
                np.clip(
                    progress,
                    -0.20,
                    0.20
                )
            )

        self.previous_goal_distance = (
            distance
        )

        # ----------------------------------------------------
        # Alignment
        # ----------------------------------------------------

        reward += (
            0.35
            *
            linear_velocity
            *
            math.cos(bearing)
        )

        # ----------------------------------------------------
        # Smoothness
        # ----------------------------------------------------

        action_change = np.linalg.norm(
            action
            -
            self.previous_action
        )

        reward -= (
            0.04 *
            action_change
        )

        # ----------------------------------------------------
        # Spinning penalty
        # ----------------------------------------------------

        if (
            abs(linear_velocity) < 0.08
            and
            abs(angular_velocity) > 0.70
        ):

            reward -= 0.02

        # ----------------------------------------------------
        # Proximity penalty
        # ----------------------------------------------------

        min_scan = float(
            np.min(
                self.ros.scan
            )
            *
            LIDAR_MAX_RANGE
        )

        if min_scan < 1.0:

            reward -= (
                0.30
                *
                (1.0 - min_scan)
            )

        if min_scan < 0.55:

            reward -= 0.75

        return reward

    # ========================================================
    # STEP
    # ========================================================

    def step(
        self,
        action
    ):

        action = self.smooth_action(
            action
        )

        linear = (
            float(action[0])
            *
            self.max_linear_speed
        )

        angular = (
            float(action[1])
            *
            self.max_angular_speed
        )

        previous_scan_seq = self.ros.scan_seq
        previous_odom_seq = self.ros.odom_seq

        step_start = time.monotonic()

        self.ros.publish_action(
            linear,
            angular
        )

        # ----------------------------------------------------
        # Hold the action for exactly CONTROL_DT.
        # ----------------------------------------------------

        deadline = (
            step_start
            +
            CONTROL_DT
        )

        fresh_scan = False
        fresh_odom = False

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.005
            )

            if (
                self.ros.scan_seq
                >
                previous_scan_seq
            ):
                fresh_scan = True

            if (
                self.ros.odom_seq
                >
                previous_odom_seq
            ):
                fresh_odom = True

        # Consume the latest sensor state after the fixed
        # action interval.
        current_scan = (
            self.ros.scan.copy()
        )

        self.scan_history.append(
            current_scan
        )

        # ----------------------------------------------------
        # Path length
        # ----------------------------------------------------

        if (
            self.previous_x
            is not None
            and
            self.ros.odom_x
            is not None
        ):

            dx = (
                self.ros.odom_x
                -
                self.previous_x
            )

            dy = (
                self.ros.odom_y
                -
                self.previous_y
            )

            self.path_length += (
                math.hypot(
                    dx,
                    dy
                )
            )

            self.previous_x = (
                self.ros.odom_x
            )

            self.previous_y = (
                self.ros.odom_y
            )

        measured_linear = float(
            self.ros.robot_v
        )

        measured_angular = float(
            self.ros.robot_w
        )

        reward = self.compute_reward(
            action,
            measured_linear,
            measured_angular
        )

        distance, bearing = (
            self.goal_geometry()
        )

        goal_reached = (
            distance
            <
            TARGET_REACHED_RADIUS
        )

        mission_complete = False

        # ----------------------------------------------------
        # TARGET REACHED
        #
        # IMPORTANT:
        # No reset.
        # No teleport.
        # No episode termination.
        # ----------------------------------------------------

        if goal_reached:

            reward += 35.0

            self.target_index += 1

            if (
                self.target_index
                >= len(self.MISSION_ROUTE)
            ):

                mission_complete = True
                reward += 150.0

                print()
                print(
                    '[V8 MISSION COMPLETE]'
                )

            else:

                (
                    self.goal_x,
                    self.goal_y
                ) = (
                    self.MISSION_ROUTE[
                        self.target_index
                    ]
                )

                self.publish_goal_reliably()

                distance, bearing = (
                    self.goal_geometry()
                )

                self.previous_goal_distance = (
                    distance
                )

                print(
                    f'[TARGET REACHED] '
                    f'next='
                    f'{self.target_index + 1}/'
                    f'{len(self.MISSION_ROUTE)} '
                    f'goal='
                    f'({self.goal_x:.2f}, '
                    f'{self.goal_y:.2f})'
                )

        self.previous_action = (
            action.copy()
        )

        self.episode_steps += 1

        truncated = (
            self.episode_steps
            >=
            self.max_episode_steps
        )

        if mission_complete or truncated:

            self.ros.stop()

        observation = (
            self.build_observation()
        )

        info = {

            'target_index':
                self.target_index,

            'targets_total':
                len(self.MISSION_ROUTE),

            'goal_distance':
                distance,

            'goal_bearing':
                bearing,

            'min_lidar_distance':
                float(
                    np.min(
                        self.ros.scan
                    )
                    *
                    LIDAR_MAX_RANGE
                ),

            'goal_reached':
                goal_reached,

            'mission_complete':
                mission_complete,

            'path_length':
                self.path_length,

            'linear_velocity_command':
                linear,

            'angular_velocity_command':
                angular,

            'linear_velocity':
                measured_linear,

            'angular_velocity':
                measured_angular
        }

        return (
            observation,
            float(reward),
            mission_complete,
            truncated,
            info
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        try:

            self.ros.stop()

            self.ros.destroy_node()

        except Exception:
            pass


if __name__ == '__main__':

    env = AssistiveNavEnvV8()

    try:

        obs, info = env.reset()

        print(
            'Observation shape:',
            obs.shape
        )

        print(
            'Observation space:',
            env.observation_space
        )

    finally:

        env.close()
