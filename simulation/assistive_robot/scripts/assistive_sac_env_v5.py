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


# ============================================================
# ROS BRIDGE
# ============================================================

class SACROSBridge(Node):

    def __init__(self):
        super().__init__('sac_ros_bridge_v2')

        self.state = None
        self.state_seq = 0

        self.odom_x = None
        self.odom_y = None

        self.min_lidar = 15.0
        self.nearest_human = None

        # ----------------------------------------------------
        # DR STATE
        # ----------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            '/dr_state',
            self.state_callback,
            10
        )

        # ----------------------------------------------------
        # ODOMETRY
        # ----------------------------------------------------

        self.create_subscription(
            Odometry,
            '/odom',
            self.odom_callback,
            10
        )

        # ----------------------------------------------------
        # LIDAR
        # ----------------------------------------------------

        self.create_subscription(
            LaserScan,
            '/scan',
            self.scan_callback,
            10
        )

        # ----------------------------------------------------
        # SEMANTIC OBJECTS
        # ----------------------------------------------------

        self.create_subscription(
            String,
            '/semantic_objects',
            self.semantic_callback,
            10
        )

        # ----------------------------------------------------
        # COMMAND VELOCITY
        # ----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            '/sac_cmd_vel',
            10
        )

        # ----------------------------------------------------
        # GOAL
        # ----------------------------------------------------

        self.goal_pub = self.create_publisher(
            PoseStamped,
            '/goal_pose',
            10
        )

    # ========================================================
    # CALLBACKS
    # ========================================================

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
            r
            for r in msg.ranges
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

    # ========================================================
    # COMMANDS
    # ========================================================

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

        self.publish_action(
            0.0,
            0.0
        )


# ============================================================
# ASSISTIVE NAVIGATION ENVIRONMENT V5
# ============================================================

class AssistiveNavEnvV5(gym.Env):

    metadata = {
        'render_modes': []
    }

    def __init__(self):

        super().__init__()

        # ----------------------------------------------------
        # ROS INIT
        # ----------------------------------------------------

        if not rclpy.ok():
            rclpy.init(args=None)

        self.ros = SACROSBridge()

        # ====================================================
        # CURRICULUM STATE
        # ====================================================
        #
        # IMPORTANT:
        #
        # curriculum_episode is incremented inside reset(),
        # NOT here.
        #
        # This fixes the previous bug where the curriculum
        # remained stuck at CORRIDOR.
        #
        # ====================================================

        self.curriculum_episode = 0

        self.goal_candidates = []

        self.curriculum_stage = "CORRIDOR"

        # Initialize first curriculum stage.
        self._update_curriculum()

        # ----------------------------------------------------
        # DEFAULT GOAL
        # ----------------------------------------------------

        self.goal_x = 5.0
        self.goal_y = 0.0

        # ----------------------------------------------------
        # EPISODE SETTINGS
        # ----------------------------------------------------

        self.max_episode_steps = 600

        # ----------------------------------------------------
        # ROBOT SPEED
        # ----------------------------------------------------

        self.max_linear_speed = 0.6
        self.max_angular_speed = 1.5

        # ----------------------------------------------------
        # OBSERVATION SPACE
        # ----------------------------------------------------

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(48,),
            dtype=np.float32
        )

        # ----------------------------------------------------
        # ACTION SPACE
        # ----------------------------------------------------

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32
        )

        # ----------------------------------------------------
        # EPISODE VARIABLES
        # ----------------------------------------------------

        self.episode_steps = 0

        self.previous_goal_distance = None

        self.previous_goal_bearing = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.path_length = 0.0

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        # ----------------------------------------------------
        # GAZEBO CONTROL SERVICE
        # ----------------------------------------------------

        self.control_service = (
            self.find_gazebo_control_service()
        )

    # ========================================================
    # CURRICULUM
    # ========================================================

    def _update_curriculum(self):

        # ----------------------------------------------------
        # STAGE 1 — CORRIDOR
        # ----------------------------------------------------

        if self.curriculum_episode <= 80:

            self.goal_candidates = [
                (4.0, 0.0),
                (5.0, 0.0),
                (5.0, 1.0),
                (5.0, -1.0),
                (6.0, 0.0),
            ]

            self.curriculum_stage = "CORRIDOR"

        # ----------------------------------------------------
        # STAGE 2 — ROOM 1
        # ----------------------------------------------------

        elif self.curriculum_episode <= 160:

            self.goal_candidates = [
                (-5.0, 3.0),
                (-5.0, 5.0),
                (-7.0, 5.0),
                (-3.0, 5.0),
            ]

            self.curriculum_stage = "ROOM1"

        # ----------------------------------------------------
        # STAGE 3 — ROOM 2
        # ----------------------------------------------------

        elif self.curriculum_episode <= 260:

            self.goal_candidates = [
                (5.0, -3.0),
                (5.0, -4.0),
                (5.0, -5.0),
                (5.0, -6.0),
                (3.0, -4.0),
                (7.0, -4.0),
                (3.0, -6.0),
                (7.0, -6.0),
            ]

            self.curriculum_stage = "ROOM2"

        # ----------------------------------------------------
        # STAGE 4 — FULL MAP
        # ----------------------------------------------------

        else:

            self.goal_candidates = [

                # Corridor
                (4.0, 0.0),
                (5.0, 0.0),
                (6.0, 0.0),

                # Room 1
                (-5.0, 3.0),
                (-5.0, 5.0),
                (-7.0, 5.0),
                (-3.0, 5.0),

                # Room 2
                (5.0, -3.0),
                (5.0, -4.0),
                (5.0, -5.0),
                (5.0, -6.0),
                (3.0, -4.0),
                (7.0, -4.0),
                (3.0, -6.0),
                (7.0, -6.0),
            ]

            self.curriculum_stage = "FULL_MAP"

    # ========================================================
    # LOGGER
    # ========================================================

    def get_logger(self):
        return self.ros.get_logger()

    # ========================================================
    # FIND GAZEBO CONTROL SERVICE
    # ========================================================

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

    # ========================================================
    # RESET GAZEBO
    # ========================================================

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

        # Wait until Gazebo actually returns rover
        # to reset position.

        deadline = time.monotonic() + 5.0

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.05
            )

            if (
                self.ros.odom_x is not None
                and self.ros.odom_y is not None
                and abs(self.ros.odom_x) < 0.15
                and abs(self.ros.odom_y) < 0.15
            ):

                break

        else:

            raise RuntimeError(
                "Gazebo reset completed, but rover did not "
                "return near the origin within 5 seconds."
            )

        time.sleep(0.1)

    # ========================================================
    # PUBLISH GOAL
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

            time.sleep(0.05)

    # ========================================================
    # WAIT FOR STATE
    # ========================================================

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

    # ========================================================
    # WAIT FOR GOAL STATE
    # ========================================================

    def wait_for_goal_state(self, timeout=10.0):

        expected_distance = math.hypot(
            self.goal_x,
            self.goal_y
        )

        # /dr_state[13] is ROBOT-RELATIVE goal bearing.
        # Do not assume reset yaw is exactly zero.
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
            #
            # state[12] = normalized goal distance
            # state[13] = normalized goal bearing

            actual_distance = (
                float(state[12]) * 20.0
            )

            actual_bearing = (
                float(state[13]) * math.pi
            )

            # state[13] = world_goal_bearing - robot_yaw
            actual_yaw = (
                float(state[47]) * math.pi
            )

            expected_world_bearing = math.atan2(
                self.goal_y - self.ros.odom_y,
                self.goal_x - self.ros.odom_x
            )

            expected_bearing = math.atan2(
                math.sin(
                    expected_world_bearing - actual_yaw
                ),
                math.cos(
                    expected_world_bearing - actual_yaw
                )
            )

            bearing_error = math.atan2(
                math.sin(
                    actual_bearing - expected_bearing
                ),
                math.cos(
                    actual_bearing - expected_bearing
                )
            )

            robot_at_reset = (
                abs(self.ros.odom_x) < 0.15
                and abs(self.ros.odom_y) < 0.15
            )

            goal_matches = (
                abs(
                    actual_distance
                    - expected_distance
                ) < 0.20
                and abs(bearing_error) < 0.08
            )

            if robot_at_reset and goal_matches:

                return state

        raise RuntimeError(
            'Timed out waiting for DR state with '
            'the current goal.'
        )

    # ========================================================
    # WAIT FOR NEW STATE
    # ========================================================

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

    # ========================================================
    # ACTION → VELOCITY
    # ========================================================

    def action_to_velocity(self, action):

        action = np.clip(
            action,
            -1.0,
            1.0
        )

        # -1 = full reverse
        #  0 = stop
        # +1 = full forward

        linear = (
            float(action[0])
            * self.max_linear_speed
        )

        angular = (
            float(action[1])
            * self.max_angular_speed
        )

        return linear, angular

    # ========================================================
    # GOAL DISTANCE
    # ========================================================

    @staticmethod
    def goal_distance_from_state(state):

        return float(
            state[12]
        ) * 20.0

    # ========================================================
    # REWARD
    # ========================================================

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

        goal_bearing = (
            float(state[13])
            * math.pi
        )

        reward = -0.01

        # ----------------------------------------------------
        # 1. PROGRESS TOWARD GOAL
        # ----------------------------------------------------

        progress = 0.0

        if self.previous_goal_distance is not None:

            progress = (
                self.previous_goal_distance
                - goal_distance
            )

            reward += (
                10.0
                * progress
            )

        self.previous_goal_distance = goal_distance

        # ----------------------------------------------------
        # 2. GOAL HEADING IMPROVEMENT
        # ----------------------------------------------------

        if self.previous_goal_bearing is not None:

            previous_abs_bearing = abs(
                self.previous_goal_bearing
            )

            current_abs_bearing = abs(
                goal_bearing
            )

            bearing_progress = (
                previous_abs_bearing
                - current_abs_bearing
            )

            reward += (
                2.5
                * bearing_progress
            )

        self.previous_goal_bearing = (
            goal_bearing
        )

        # ----------------------------------------------------
        # 3. MOTION IN GOAL DIRECTION
        # ----------------------------------------------------

        heading_alignment = math.cos(
            goal_bearing
        )

        goal_directed_motion = (
            linear_velocity
            * heading_alignment
        )

        reward += (
            0.50
            * goal_directed_motion
        )

        if goal_directed_motion < 0.0:

            reward += (
                0.25
                * goal_directed_motion
            )

        # ----------------------------------------------------
        # 4. LIDAR PROXIMITY
        # ----------------------------------------------------

        min_lidar = float(
            self.ros.min_lidar
        )

        if min_lidar < 1.0:

            reward -= (
                2.0
                * (1.0 - min_lidar)
            )

        # ----------------------------------------------------
        # 5. HUMAN PROXIMITY
        # ----------------------------------------------------

        if self.ros.nearest_human is not None:

            human_distance = (
                self.ros.nearest_human
            )

            if human_distance < 1.5:

                reward -= (
                    2.5
                    * (1.5 - human_distance)
                )

            if human_distance < 0.6:

                reward -= 8.0

        # ----------------------------------------------------
        # 6. SMOOTHNESS
        # ----------------------------------------------------

        action_change = np.linalg.norm(
            action
            - self.previous_action
        )

        reward -= (
            0.04
            * float(action_change)
        )

        # ----------------------------------------------------
        # 7. STEERING REGULARIZATION
        # ----------------------------------------------------

        reward -= (
            0.05
            * abs(float(action[1]))
        )

        # ----------------------------------------------------
        # 8. ANTI-SPIN NEAR CORRECT HEADING
        # ----------------------------------------------------

        if abs(goal_bearing) < math.radians(15.0):

            excess_turn = max(
                0.0,
                abs(float(action[1]))
                - 0.20
            )

            reward -= (
                0.25
                * excess_turn
            )

        # ----------------------------------------------------
        # 9. STAGNATION
        # ----------------------------------------------------

        if progress < 0.002:

            self.stuck_steps += 1

        else:

            self.stuck_steps = 0

        if self.stuck_steps > 10:

            reward -= 0.25

        # ----------------------------------------------------
        # SAVE ACTION
        # ----------------------------------------------------

        self.previous_action = (
            action.copy()
        )

        return reward

    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
        *,
        seed=None,
        options=None
    ):

        super().reset(seed=seed)

        # ====================================================
        # ADVANCE CURRICULUM
        # ====================================================

        self.curriculum_episode += 1

        self._update_curriculum()

        print(
            f"[CURRICULUM] "
            f"Episode {self.curriculum_episode} "
            f"-> {self.curriculum_stage}"
        )

        # ----------------------------------------------------
        # CLEAR ROS STATE
        # ----------------------------------------------------

        self.ros.state = None
        self.ros.state_seq = 0

        self.ros.odom_x = None
        self.ros.odom_y = None

        # ----------------------------------------------------
        # RESET EPISODE VARIABLES
        # ----------------------------------------------------

        self.episode_steps = 0

        self.previous_goal_distance = None

        self.previous_goal_bearing = None

        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.path_length = 0.0

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        # ----------------------------------------------------
        # RANDOM GOAL
        # ----------------------------------------------------

        self.goal_x, self.goal_y = random.choice(
            self.goal_candidates
        )

        print(
            f"[GOAL] "
            f"({self.goal_x:.2f}, "
            f"{self.goal_y:.2f})"
        )

        # ----------------------------------------------------
        # RESET GAZEBO
        # ----------------------------------------------------

        self.reset_gazebo()

        # ----------------------------------------------------
        # PUBLISH GOAL
        # ----------------------------------------------------

        self.publish_goal_reliably()

        # ----------------------------------------------------
        # WAIT FOR CORRECT STATE
        # ----------------------------------------------------

        state = self.wait_for_goal_state()

        # ----------------------------------------------------
        # INITIAL ODOM
        # ----------------------------------------------------

        self.previous_x = self.ros.odom_x
        self.previous_y = self.ros.odom_y

        # ----------------------------------------------------
        # INITIAL DISTANCE
        # ----------------------------------------------------

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

            'path_length': 0.0,

            'curriculum_episode': (
                self.curriculum_episode
            ),

            'curriculum_stage': (
                self.curriculum_stage
            )
        }

        return state, info

    # ========================================================
    # STEP
    # ========================================================

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

        # ----------------------------------------------------
        # ACTION → VELOCITY
        # ----------------------------------------------------

        linear, angular = (
            self.action_to_velocity(
                action
            )
        )

        # ----------------------------------------------------
        # PUBLISH ACTION
        # ----------------------------------------------------

        previous_seq = (
            self.ros.state_seq
        )

        self.ros.publish_action(
            linear,
            angular
        )

        # ----------------------------------------------------
        # WAIT FOR NEW STATE
        # ----------------------------------------------------

        state = self.wait_for_new_state(
            previous_seq
        )

        # ----------------------------------------------------
        # PATH LENGTH
        # ----------------------------------------------------

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
                dx * dx
                + dy * dy
            )

            self.path_length += movement

            self.previous_x = (
                self.ros.odom_x
            )

            self.previous_y = (
                self.ros.odom_y
            )

        # ----------------------------------------------------
        # REWARD
        # ----------------------------------------------------

        reward = self.calculate_reward(
            state,
            action,
            linear
        )

        # ----------------------------------------------------
        # GOAL / COLLISION
        # ----------------------------------------------------

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

        # ----------------------------------------------------
        # TERMINAL REWARD
        # ----------------------------------------------------

        if goal_reached:

            reward += 100.0

            print(
                f"[SUCCESS] "
                f"Episode "
                f"{self.curriculum_episode} "
                f"reached goal!"
            )

        if collision:

            reward -= 100.0

            print(
                f"[COLLISION] "
                f"Episode "
                f"{self.curriculum_episode}"
            )

        # ----------------------------------------------------
        # EPISODE TIME
        # ----------------------------------------------------

        self.episode_steps += 1

        truncated = (
            self.episode_steps
            >= self.max_episode_steps
        )

        # ----------------------------------------------------
        # STOP ROBOT
        # ----------------------------------------------------

        if terminated or truncated:

            self.ros.stop()

        # ----------------------------------------------------
        # INFO
        # ----------------------------------------------------

        info = {

            'goal_x': self.goal_x,

            'goal_y': self.goal_y,

            'goal_distance': goal_distance,

            'min_lidar_distance': min_lidar,

            'goal_reached': goal_reached,

            'collision': collision,

            'path_length': self.path_length,

            'linear_velocity': linear,

            'angular_velocity': angular,

            'curriculum_episode': (
                self.curriculum_episode
            ),

            'curriculum_stage': (
                self.curriculum_stage
            )
        }

        return (
            state,
            float(reward),
            terminated,
            truncated,
            info
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        try:

            self.ros.stop()

            if self.ros is not None:

                self.ros.destroy_node()

            if rclpy.ok():

                rclpy.shutdown()

        except Exception:

            pass


# ============================================================
# DIRECT TEST
# ============================================================

if __name__ == '__main__':

    env = AssistiveNavEnvV5()

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

        print(
            'Curriculum episode:',
            info['curriculum_episode']
        )

        print(
            'Curriculum stage:',
            info['curriculum_stage']
        )

    finally:

        env.close()
