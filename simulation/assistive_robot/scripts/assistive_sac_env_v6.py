#!/usr/bin/env python3

import json
import math
import random
import re
import subprocess
import time
import threading

from rclpy.executors import SingleThreadedExecutor

import gymnasium as gym
import numpy as np
import rclpy

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from std_msgs.msg import Float32MultiArray, String
from ros_gz_interfaces.msg import Contacts
from rclpy.node import Node


# ============================================================
# DEDICATED CONTACT MONITOR
# ============================================================

class ContactMonitor(Node):

    def __init__(self):
        super().__init__('v6_contact_monitor')

        self.contact_detected = False

        self.create_subscription(
            Contacts,
            '/rover_contacts',
            self.contact_callback,
            10,
        )

    def contact_callback(self, msg):

        # Episode-latched collision.
        # Empty Contacts messages never clear a collision.

        for contact in msg.contacts:

            names = [
                str(contact.collision1.name).lower(),
                str(contact.collision2.name).lower()
            ]

            # Ignore ground / floor contact.
            if any(
                "floor" in name or "ground" in name
                for name in names
            ):
                continue

            self.contact_detected = True
            return


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

        # Simulator ground-truth collision state.
        # NEVER exposed to the SAC observation.
        self.contact_detected = False

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
        # CONTACT SENSOR
        # ----------------------------------------------------

        self.create_subscription(
            Contacts,
            "/rover_contacts",
            self.contact_callback,
            10,
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

    def contact_callback(self, msg):

        # Latch any real collision for the rest of the episode.
        # reset() clears contact_detected.
        #
        # Empty Contacts messages must NOT clear a collision that
        # has already happened.

        for contact in msg.contacts:

            names = [
                str(contact.collision1.name).lower(),
                str(contact.collision2.name).lower()
            ]

            # Ignore floor / ground contacts.
            if any(
                "floor" in name or "ground" in name
                for name in names
            ):
                continue

            self.contact_detected = True
            return

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

class AssistiveNavEnvV6(gym.Env):

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

        # ----------------------------------------------------
        # DEDICATED CONTACT MONITOR
        # ----------------------------------------------------

        # Collision detection runs in its own executor thread so
        # high-rate odom / lidar / state callbacks cannot starve it.

        self.contact_monitor = ContactMonitor()

        self.contact_executor = SingleThreadedExecutor()

        self.contact_executor.add_node(
            self.contact_monitor
        )

        self.contact_thread = threading.Thread(
            target=self.contact_executor.spin,
            daemon=True
        )

        self.contact_thread.start()

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
        # V6 TRAINING TASKS
        # ----------------------------------------------------
        #
        # Each episode uses a coherent simulator-only
        # (spawn, goal) pair.
        #
        # SAC never receives these world coordinates.
        #

        self.task_candidates = []

        self.training_spawns = []

        self.current_spawn = (0.0, 0.0)


        # ----------------------------------------------------
        # OBSERVATION SPACE
        # ----------------------------------------------------

        # ----------------------------------------------------
        # TEMPORAL OBSERVATION HISTORY
        # ----------------------------------------------------
        #
        # Keep the latest 4 perception/state frames.
        # This gives SAC short-term temporal context instead
        # of forcing it to react only to one instantaneous state.
        #
        self.history_length = 4
        self.state_history = []

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(48 * self.history_length,),
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
        # SAFE TRAINING LOCATIONS
        # ----------------------------------------------------

        corridor = [
            (0.0, 0.0),
            (2.0, 0.0),
            (4.0, 0.0),
            (6.0, 0.0),
        ]

        room1 = [
            (-5.0, 2.2),
            (-5.0, 3.0),
            (-5.0, 4.5),
            (-5.0, 6.0),
            (-7.0, 5.0),
            (-3.0, 5.0),
        ]

        room2 = [
            (5.0, -2.2),
            (5.0, -3.0),
            (5.0, -4.5),
            (5.0, -6.0),
            (3.0, -4.0),
            (7.0, -4.0),
        ]

        tasks = []

        # ----------------------------------------------------
        # STAGE 1 — CORRIDOR
        # ----------------------------------------------------

        if self.curriculum_episode <= 100:

            goals = [
                (2.0, 0.0),
                (4.0, 0.0),
                (5.0, 1.0),
                (5.0, -1.0),
                (6.0, 0.0),
            ]

            for spawn in corridor:
                for goal in goals:

                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

            self.curriculum_stage = "CORRIDOR_LOCAL"

        # ----------------------------------------------------
        # STAGE 2 — ROOM 1 DOORWAY
        # ----------------------------------------------------

        elif self.curriculum_episode <= 200:

            goals = [
                (-5.0, 0.6),
                (-5.0, 1.2),
                (-5.0, 1.8),
                (-5.0, 2.2),
            ]

            for spawn in corridor:
                for goal in goals:
                    tasks.append((spawn, goal))

            self.curriculum_stage = "DOORWAY_ROOM1"

        # ----------------------------------------------------
        # STAGE 3 — ROOM 1 LOCAL
        # ----------------------------------------------------

        elif self.curriculum_episode <= 300:

            goals = [
                (-5.0, 3.0),
                (-5.0, 4.5),
                (-5.0, 5.0),
                (-7.0, 5.0),
                (-3.0, 5.0),
            ]

            for spawn in room1:
                for goal in goals:

                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

            self.curriculum_stage = "ROOM1_LOCAL"

        # ----------------------------------------------------
        # STAGE 4 — ROOM 2 DOORWAY
        # ----------------------------------------------------

        elif self.curriculum_episode <= 400:

            goals = [
                (5.0, -0.6),
                (5.0, -1.2),
                (5.0, -1.8),
                (5.0, -2.2),
            ]

            for spawn in corridor:
                for goal in goals:
                    tasks.append((spawn, goal))

            self.curriculum_stage = "DOORWAY_ROOM2"

        # ----------------------------------------------------
        # STAGE 5 — ROOM 2 LOCAL
        # ----------------------------------------------------

        elif self.curriculum_episode <= 500:

            goals = [
                (5.0, -3.0),
                (5.0, -4.0),
                (5.0, -5.0),
                (5.0, -6.0),
                (3.0, -4.0),
                (7.0, -4.0),
            ]

            for spawn in room2:
                for goal in goals:

                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

            self.curriculum_stage = "ROOM2_LOCAL"

        # ----------------------------------------------------
        # STAGE 6 — MIXED LOCAL NAVIGATION
        # ----------------------------------------------------

        else:

            corridor_goals = [
                (2.0, 0.0),
                (4.0, 0.0),
                (5.0, 1.0),
                (5.0, -1.0),
            ]

            room1_goals = [
                (-5.0, 3.0),
                (-5.0, 4.5),
                (-5.0, 5.0),
                (-7.0, 5.0),
                (-3.0, 5.0),
            ]

            room2_goals = [
                (5.0, -3.0),
                (5.0, -4.0),
                (5.0, -5.0),
                (5.0, -6.0),
                (3.0, -4.0),
                (7.0, -4.0),
            ]

            for spawn in corridor:

                for goal in corridor_goals:
                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

                for goal in [
                    (-5.0, 0.6),
                    (-5.0, 1.8),
                    (-5.0, 2.2),
                ]:
                    tasks.append((spawn, goal))

                for goal in [
                    (5.0, -0.6),
                    (5.0, -1.8),
                    (5.0, -2.2),
                ]:
                    tasks.append((spawn, goal))

            for spawn in room1:
                for goal in room1_goals:

                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

            for spawn in room2:
                for goal in room2_goals:

                    if math.dist(spawn, goal) >= 0.8:
                        tasks.append((spawn, goal))

            self.curriculum_stage = "MIXED_LOCAL"

        if not tasks:
            raise RuntimeError(
                f"No V6 tasks for stage {self.curriculum_stage}"
            )

        self.task_candidates = tasks

        self.training_spawns = [
            spawn for spawn, _ in tasks
        ]

        self.goal_candidates = [
            goal for _, goal in tasks
        ]

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
    # SET RANDOM TRAINING SPAWN
    # ========================================================

    def set_training_spawn(self, x, y):
        """Robust simulator-only training spawn with verified pose."""

        request = (
            'name: "assistive_rover" '
            f'position: {{x: {float(x)}, y: {float(y)}, z: 0.15}} '
            'orientation: {x: 0.0, y: 0.0, z: 0.0, w: 1.0}'
        )

        service = "/world/assistive_world/set_pose/blocking"

        for attempt in range(1, 11):

            # -------------------------------------------------
            # 1. Request simulator pose
            # -------------------------------------------------
            try:
                result = subprocess.run(
                    [
                        "gz",
                        "service",
                        "-s",
                        service,
                        "--reqtype",
                        "gz.msgs.Pose",
                        "--reptype",
                        "gz.msgs.Boolean",
                        "--timeout",
                        "5000",
                        "--req",
                        request,
                    ],
                    text=True,
                    capture_output=True,
                    timeout=8,
                )

                if result.returncode != 0:
                    print(
                        f"[SPAWN] attempt={attempt} "
                        f"service failed: {result.stderr.strip()}"
                    )
                    time.sleep(0.25)
                    continue

            except subprocess.TimeoutExpired:
                print(
                    f"[SPAWN] attempt={attempt} "
                    "set_pose timed out; retrying."
                )
                time.sleep(0.25)
                continue

            except Exception as exc:
                print(
                    f"[SPAWN] attempt={attempt} "
                    f"spawn exception: {exc}"
                )
                time.sleep(0.25)
                continue

            # Give Gazebo a moment to apply the pose.
            time.sleep(0.15)

            # -------------------------------------------------
            # 2. Verify pose without changing it
            #    Retry verification before issuing another
            #    set_pose command.
            # -------------------------------------------------
            verified = False

            for verify_attempt in range(1, 4):

                try:
                    verify = subprocess.run(
                        [
                            "gz",
                            "model",
                            "-m",
                            "assistive_rover",
                            "-p",
                        ],
                        text=True,
                        capture_output=True,
                        timeout=5,
                    )

                    if verify.returncode != 0:
                        print(
                            f"[SPAWN] attempt={attempt} "
                            f"verify={verify_attempt} "
                            "pose query failed; retrying verification."
                        )
                        time.sleep(0.25)
                        continue

                    match = re.search(
                        r"\[\s*([-+0-9.eE]+)\s+"
                        r"([-+0-9.eE]+)\s+"
                        r"([-+0-9.eE]+)\s*\]",
                        verify.stdout,
                    )

                    if match is None:
                        print(
                            f"[SPAWN] attempt={attempt} "
                            f"verify={verify_attempt} "
                            "could not parse pose; retrying verification."
                        )
                        time.sleep(0.25)
                        continue

                    actual_x = float(match.group(1))
                    actual_y = float(match.group(2))

                    error = math.hypot(
                        actual_x - float(x),
                        actual_y - float(y),
                    )

                    print(
                        f"[SPAWN] attempt={attempt} "
                        f"verify={verify_attempt} "
                        f"requested=({float(x):.2f},{float(y):.2f}) "
                        f"actual=({actual_x:.2f},{actual_y:.2f}) "
                        f"error={error:.3f}m"
                    )

                    if error <= 0.75:
                        self.current_spawn = (
                            float(x),
                            float(y),
                        )

                        print(
                            f"[SPAWN] VERIFIED "
                            f"({actual_x:.2f}, {actual_y:.2f})"
                        )

                        return

                    print(
                        "[SPAWN] Verification mismatch; "
                        "retrying verification."
                    )

                except subprocess.TimeoutExpired:
                    print(
                        f"[SPAWN] attempt={attempt} "
                        f"verify={verify_attempt} "
                        "pose verification timed out; "
                        "retrying verification."
                    )

                except Exception as exc:
                    print(
                        f"[SPAWN] attempt={attempt} "
                        f"verify={verify_attempt} "
                        f"verification exception: {exc}"
                    )

                time.sleep(0.25)

            # -------------------------------------------------
            # 3. Verification failed all attempts.
            #    Reissue set_pose.
            # -------------------------------------------------
            print(
                f"[SPAWN] attempt={attempt} "
                "verification failed; reissuing set_pose."
            )

            time.sleep(0.25)

        raise RuntimeError(
            "Gazebo spawn failed verification after 10 attempts."
        )
    def reset_gazebo(self):
        """Reset simulator state without assuming odom origin.

        Gazebo world reset and DiffDrive odometry are separate state
        machines. The reset service is authoritative here; the
        environment later captures the current odom reference after
        the training teleport.
        """

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
            timeout=8
        )

        if result.returncode != 0:

            raise RuntimeError(
                'Gazebo reset failed:\n'
                + result.stderr
            )

        # Give Gazebo / DiffDrive a little time to settle.
        # We intentionally do NOT require odom=(0,0).
        for _ in range(30):

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.05
            )

            time.sleep(0.03)

        # Reset simulator-side collision latch.
        self.ros.contact_detected = False

        if hasattr(self, 'contact_monitor'):

            self.contact_monitor.contact_detected = False

        time.sleep(0.15)


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

    def wait_for_goal_state(self, timeout=15.0):
        """
        Wait for a fresh DR state after reset.

        The goal is stored in the current odom frame. State fusion
        already computes the robot-to-goal relationship, so we do
        not validate the state against self.goal_x/self.goal_y here.
        """
        deadline = time.monotonic() + timeout

        while time.monotonic() < deadline:
            rclpy.spin_once(self.ros, timeout_sec=0.03)

            if (
                self.ros.state is not None
                and self.ros.odom_x is not None
                and self.ros.odom_y is not None
            ):
                return self.ros.state.copy()

            time.sleep(0.02)

        if self.ros.state is not None and self.ros.odom_x is not None:
            return self.ros.state.copy()

        raise RuntimeError(
            "Timed out waiting for /dr_state and /odom."
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
    # TEMPORAL OBSERVATION
    # ========================================================

    def reset_state_history(self, state):
        """Initialize the temporal history with a valid state."""

        state = np.asarray(
            state,
            dtype=np.float32
        )

        self.state_history = [
            state.copy()
            for _ in range(self.history_length)
        ]

    def update_state_history(self, state):
        """Append the newest state and return the stacked observation."""

        state = np.asarray(
            state,
            dtype=np.float32
        )

        if not self.state_history:
            self.reset_state_history(state)

        else:
            self.state_history.append(
                state.copy()
            )

            self.state_history = (
                self.state_history[-self.history_length:]
            )

        return np.concatenate(
            self.state_history
        ).astype(np.float32)

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
        self.ros.contact_detected = False
        self.contact_monitor.contact_detected = False

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

        selected_spawn, selected_goal = random.choice(
            self.task_candidates
        )

        spawn_x, spawn_y = selected_spawn

        self.goal_x, self.goal_y = selected_goal

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
        # RANDOM TRAINING SPAWN
        # ----------------------------------------------------

        self.set_training_spawn(
            spawn_x,
            spawn_y
        )

        print(
            f"[SPAWN] World position "
            f"({spawn_x:.2f}, {spawn_y:.2f})"
        )

        # ----------------------------------------------------
        # CONVERT WORLD GOAL -> ODOM FRAME
        # ----------------------------------------------------
        #
        # Gazebo teleports the physical robot, while DiffDrive
        # keeps the odometry origin unchanged.
        #
        # Therefore:
        #
        #   odom_goal = world_goal - world_spawn
        #
        # The learned policy only receives this relative goal.
        #

        self.world_goal_x = float(self.goal_x)
        self.world_goal_y = float(self.goal_y)

        # Capture the robot's current odom reference AFTER
        # the simulator teleport.
        #
        # This makes training independent of whether Gazebo's
        # DiffDrive odometry happened to reset to exactly zero.

        deadline = time.monotonic() + 3.0

        while time.monotonic() < deadline:

            rclpy.spin_once(
                self.ros,
                timeout_sec=0.03
            )

            if (
                self.ros.odom_x is not None
                and self.ros.odom_y is not None
            ):
                break

        odom_reference_x = (
            float(self.ros.odom_x)
            if self.ros.odom_x is not None
            else 0.0
        )

        odom_reference_y = (
            float(self.ros.odom_y)
            if self.ros.odom_y is not None
            else 0.0
        )

        # World goal -> goal expressed in the CURRENT odom frame.
        self.goal_x = (
            self.world_goal_x
            - float(spawn_x)
            + odom_reference_x
        )

        self.goal_y = (
            self.world_goal_y
            - float(spawn_y)
            + odom_reference_y
        )

        print(
            f"[ODOM REF] "
            f"({odom_reference_x:.2f}, "
            f"{odom_reference_y:.2f})"
        )

        print(
            f"[GOAL] World: "
            f"({self.world_goal_x:.2f}, "
            f"{self.world_goal_y:.2f})"
        )

        print(
            f"[GOAL] Relative/Odom: "
            f"({self.goal_x:.2f}, "
            f"{self.goal_y:.2f})"
        )

        # ----------------------------------------------------
        # PUBLISH GOAL
        # ----------------------------------------------------

        self.publish_goal_reliably()

        # ----------------------------------------------------
        # WAIT FOR CORRECT STATE
        # ----------------------------------------------------

        state = self.wait_for_goal_state()

        # ----------------------------------------------------
        # INITIALIZE TEMPORAL HISTORY
        # ----------------------------------------------------

        observation = self.update_state_history(
            state
        )

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

        return observation, info

    def close(self):

        try:
            self.ros.stop()
        except Exception:
            pass

        try:
            self.contact_executor.shutdown(
                timeout_sec=1.0
            )
        except Exception:
            pass

        try:
            self.contact_monitor.destroy_node()
        except Exception:
            pass

        try:
            self.ros.destroy_node()
        except Exception:
            pass

        super().close()


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

        # ----------------------------------------------------
        # Process pending ROS callbacks before evaluating
        # terminal collision state.
        for _ in range(8):
            rclpy.spin_once(
                self.ros,
                timeout_sec=0.0
            )

            if self.ros.contact_detected:
                break

        # COLLISION / GOAL — collision has priority
        # ----------------------------------------------------

        collision = (
            self.contact_monitor.contact_detected
            or self.ros.contact_detected
            or min_lidar < 0.28
        )

        goal_reached = (
            goal_distance < 0.45
            and not collision
        )

        terminated = (
            collision
            or goal_reached
        )

        # ----------------------------------------------------
        # TERMINAL REWARD
        # ----------------------------------------------------

        if collision:

            reward -= 100.0

            print(
                f"[COLLISION] "
                f"Episode "
                f"{self.curriculum_episode} "
                f"contact={self.ros.contact_detected} "
                f"lidar={min_lidar:.3f}m"
            )

        elif goal_reached:

            reward += 100.0

            print(
                f"[SUCCESS] "
                f"Episode "
                f"{self.curriculum_episode} "
                f"reached goal safely!"
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

            'contact_collision': (
                self.contact_monitor.contact_detected
                or self.ros.contact_detected
            ),

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

        # ----------------------------------------------------
        # UPDATE TEMPORAL HISTORY
        # ----------------------------------------------------

        observation = self.update_state_history(
            state
        )

        return (
            observation,
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


# ============================================================
# FINAL V6 CLEAN SHUTDOWN OVERRIDE
# ============================================================

def _v6_clean_close(self):

    # Stop robot commands first.
    try:
        self.ros.stop()
    except Exception:
        pass

    # Stop dedicated contact executor before destroying its node.
    try:
        executor = getattr(
            self,
            "contact_executor",
            None
        )

        if executor is not None:
            executor.shutdown(
                timeout_sec=2.0
            )
    except Exception:
        pass

    # Wait for contact thread to actually exit.
    try:
        thread = getattr(
            self,
            "contact_thread",
            None
        )

        if thread is not None and thread.is_alive():
            thread.join(
                timeout=2.0
            )
    except Exception:
        pass

    # Destroy dedicated contact monitor.
    try:
        monitor = getattr(
            self,
            "contact_monitor",
            None
        )

        if monitor is not None:
            monitor.destroy_node()
    except Exception:
        pass

    # Destroy main ROS node.
    try:
        if self.ros is not None:
            self.ros.destroy_node()
    except Exception:
        pass

    # Only shut down ROS after all executor threads are stopped.
    try:
        if rclpy.ok():
            rclpy.shutdown()
    except Exception:
        pass


# This overrides the duplicate close() definitions above.
AssistiveNavEnvV6.close = _v6_clean_close


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
