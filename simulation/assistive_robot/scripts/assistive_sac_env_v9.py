#!/usr/bin/env python3

import math
import random
import subprocess
import time
from collections import deque

import gymnasium as gym
import numpy as np
import rclpy
import torch
import torch.nn as nn

from geometry_msgs.msg import PoseStamped, Twist
from nav_msgs.msg import Odometry
from sensor_msgs.msg import LaserScan
from rclpy.node import Node
from stable_baselines3.common.torch_layers import BaseFeaturesExtractor

from v9_global_planner import StructuralOccupancyMap


# ============================================================
# UTILITIES
# ============================================================

def _yaw_from_quaternion(q):
    siny = 2.0 * (q.w * q.z + q.x * q.y)
    cosy = 1.0 - 2.0 * (q.y * q.y + q.z * q.z)
    return math.atan2(siny, cosy)


def _wrap_angle(a):
    return math.atan2(
        math.sin(a),
        math.cos(a),
    )


# ============================================================
# ROS / GAZEBO BRIDGE
# ============================================================

class V9ROSBridge(Node):

    def __init__(self):

        super().__init__(
            "assistive_sac_v9_bridge"
        )

        # ----------------------------------------------------
        # SENSOR STATE
        # ----------------------------------------------------

        self.scan = None

        self.scan_seq = 0
        self.odom_seq = 0

        # Raw Gazebo DiffDrive odometry.

        self.raw_x = None

        self.raw_y = None

        self.raw_yaw = None


        # Per-episode odometry reference.

        self.odom_ref_x = None

        self.odom_ref_y = None

        self.odom_ref_yaw = None

        self.x = None
        self.y = None
        self.yaw = None

        self.v = 0.0
        self.w = 0.0

        self.min_range = 15.0

        self.range_min = 0.12
        self.range_max = 15.0

        # ----------------------------------------------------
        # SAC COMMAND
        # ----------------------------------------------------

        self.cmd_pub = self.create_publisher(
            Twist,
            "/sac_cmd_vel",
            10,
        )

        self.goal_pub = self.create_publisher(
            PoseStamped,
            "/goal_pose",
            10,
        )

        # ----------------------------------------------------
        # SCAN
        # ----------------------------------------------------

        self.create_subscription(
            LaserScan,
            "/scan",
            self._scan_cb,
            10,
        )

        # ----------------------------------------------------
        # ODOM
        # ----------------------------------------------------

        self.create_subscription(
            Odometry,
            "/odom",
            self._odom_cb,
            10,
        )

    # ========================================================
    # CALLBACKS
    # ========================================================

    def _scan_cb(self, msg):

        ranges = np.asarray(
            msg.ranges,
            dtype=np.float32,
        )

        rmin = float(
            msg.range_min
        )

        rmax = float(
            msg.range_max
        )

        ranges = np.nan_to_num(
            ranges,
            nan=rmax,
            posinf=rmax,
            neginf=rmin,
        )

        ranges = np.clip(
            ranges,
            rmin,
            rmax,
        )

        self.scan = ranges

        self.range_min = rmin
        self.range_max = rmax

        if ranges.size > 0:
            self.min_range = float(
                ranges.min()
            )
        else:
            self.min_range = rmax

        self.scan_seq += 1

    def _odom_cb(self, msg):

        # ----------------------------------------------------
        # Raw Gazebo odometry
        # ----------------------------------------------------

        self.raw_x = float(
            msg.pose.pose.position.x
        )

        self.raw_y = float(
            msg.pose.pose.position.y
        )

        self.raw_yaw = _yaw_from_quaternion(
            msg.pose.pose.orientation
        )

        self.v = float(
            msg.twist.twist.linear.x
        )

        self.w = float(
            msg.twist.twist.angular.z
        )

        # ----------------------------------------------------
        # Episode-local odometry
        # ----------------------------------------------------

        if self.odom_ref_x is None:

            self.x = self.raw_x
            self.y = self.raw_y
            self.yaw = self.raw_yaw

        else:

            # Express odometry displacement in the episode-start frame.
            dx = self.raw_x - self.odom_ref_x
            dy = self.raw_y - self.odom_ref_y

            cos_ref = math.cos(self.odom_ref_yaw)
            sin_ref = math.sin(self.odom_ref_yaw)

            # Rotate displacement by -reference yaw.
            self.x = cos_ref * dx + sin_ref * dy
            self.y = -sin_ref * dx + cos_ref * dy

            # Express heading relative to the episode-start heading.
            self.yaw = _wrap_angle(
                self.raw_yaw - self.odom_ref_yaw
            )

        self.odom_seq += 1

    def reset_odometry_reference(self):

        if self.raw_x is None:
            raise RuntimeError(
                "Cannot reset odometry reference: "
                "no /odom received yet."
            )

        self.odom_ref_x = self.raw_x
        self.odom_ref_y = self.raw_y
        self.odom_ref_yaw = self.raw_yaw

        self.x = 0.0
        self.y = 0.0
        self.yaw = 0.0

        print(
            "[V9 ODOM RESET] "
            f"reference=("
            f"{self.odom_ref_x:.3f}, "
            f"{self.odom_ref_y:.3f}, "
            f"{self.odom_ref_yaw:.3f})"
        )

    # ========================================================
    # COMMANDS
    # ========================================================

    def publish(
        self,
        linear,
        angular,
    ):

        msg = Twist()

        msg.linear.x = float(
            linear
        )

        msg.angular.z = float(
            angular
        )

        self.cmd_pub.publish(
            msg
        )

    def publish_goal(self, x, y):
        msg = PoseStamped()

        msg.header.stamp = (
            self.get_clock().now().to_msg()
        )
        msg.header.frame_id = "odom"

        msg.pose.position.x = float(x)
        msg.pose.position.y = float(y)
        msg.pose.position.z = 0.0

        msg.pose.orientation.x = 0.0
        msg.pose.orientation.y = 0.0
        msg.pose.orientation.z = 0.0
        msg.pose.orientation.w = 1.0

        self.goal_pub.publish(msg)

    def stop(self):

        for _ in range(3):

            self.publish(
                0.0,
                0.0,
            )

    # ========================================================
    # SENSOR READINESS
    # ========================================================

    def wait_ready(
        self,
        timeout=10.0,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while rclpy.ok():

            if (
                self.scan is not None
                and self.x is not None
                and self.y is not None
                and self.yaw is not None
            ):

                return True

            if (
                time.monotonic()
                >= deadline
            ):

                return False

            rclpy.spin_once(
                self,
                timeout_sec=0.05,
            )

        return False

    def wait_fresh(
        self,
        old_scan_seq,
        old_odom_seq,
        timeout=0.25,
    ):

        deadline = (
            time.monotonic()
            + timeout
        )

        while rclpy.ok():

            if (
                self.scan_seq
                > old_scan_seq
                and
                self.odom_seq
                > old_odom_seq
            ):

                return True

            if (
                time.monotonic()
                >= deadline
            ):

                return False

            rclpy.spin_once(
                self,
                timeout_sec=0.02,
            )

        return False


# ============================================================
# LIDAR CNN FEATURE EXTRACTOR
# ============================================================

class LidarCNNExtractor(
    BaseFeaturesExtractor
):

    def __init__(
        self,
        observation_space,
        features_dim=256,
    ):

        super().__init__(
            observation_space,
            features_dim,
        )

        self.lidar_dim = 720
        self.aux_dim = 7

        if observation_space.shape != (727,):

            raise ValueError(
                "V9 expects observation shape "
                "(727,)"
            )

        # ----------------------------------------------------
        # 4 x 180 TEMPORAL LIDAR
        # ----------------------------------------------------

        self.cnn = nn.Sequential(

            nn.Conv1d(
                4,
                32,
                kernel_size=7,
                stride=2,
                padding=3,
            ),

            nn.ReLU(),

            nn.Conv1d(
                32,
                64,
                kernel_size=5,
                stride=2,
                padding=2,
            ),

            nn.ReLU(),

            nn.Conv1d(
                64,
                96,
                kernel_size=5,
                stride=2,
                padding=2,
            ),

            nn.ReLU(),

            nn.Conv1d(
                96,
                128,
                kernel_size=3,
                stride=2,
                padding=1,
            ),

            nn.ReLU(),

            nn.AdaptiveAvgPool1d(
                1
            ),

            nn.Flatten(),
        )

        # ----------------------------------------------------
        # AUXILIARY FEATURES
        # ----------------------------------------------------

        self.aux = nn.Sequential(

            nn.Linear(
                self.aux_dim,
                64,
            ),

            nn.ReLU(),

            nn.Linear(
                64,
                64,
            ),

            nn.ReLU(),
        )

        # ----------------------------------------------------
        # FUSION
        # ----------------------------------------------------

        self.fusion = nn.Sequential(

            nn.Linear(
                128 + 64,
                features_dim,
            ),

            nn.ReLU(),
        )

    def forward(self, obs):

        lidar = obs[
            :,
            :self.lidar_dim
        ].reshape(
            -1,
            4,
            180,
        )

        aux = obs[
            :,
            self.lidar_dim:
        ]

        z_lidar = self.cnn(
            lidar
        )

        z_aux = self.aux(
            aux
        )

        return self.fusion(
            torch.cat(
                [
                    z_lidar,
                    z_aux,
                ],
                dim=1,
            )
        )


# ============================================================
# ASSISTIVE NAVIGATION ENVIRONMENT V9
# ============================================================

class AssistiveNavEnvV9(
    gym.Env
):

    metadata = {
        "render_modes": []
    }

    def __init__(
        self,
        world_path=None,
        max_episode_steps=2500,
        goal_lookahead=1.5,
    ):

        super().__init__()

        # ----------------------------------------------------
        # ROS
        # ----------------------------------------------------

        if not rclpy.ok():

            rclpy.init(
                args=None
            )

        self.ros = V9ROSBridge()

        # ----------------------------------------------------
        # GLOBAL PLANNER
        # ----------------------------------------------------

        self.planner = (
            StructuralOccupancyMap(
                world_path=world_path
            )
        )

        self.goal_lookahead = float(
            goal_lookahead
        )

        # ----------------------------------------------------
        # EPISODE
        # ----------------------------------------------------

        self.max_episode_steps = int(
            max_episode_steps
        )

        # ----------------------------------------------------
        # SPEED LIMITS
        # ----------------------------------------------------

        self.max_linear_speed = 0.6
        self.max_angular_speed = 1.5

        # ----------------------------------------------------
        # ROVER FOOTPRINT
        # ----------------------------------------------------

        self.rover_half_length = 0.45
        self.rover_half_width = 0.30

        self.safety_margin = 0.18

        # ----------------------------------------------------
        # ACTION SPACE
        # ----------------------------------------------------

        self.action_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(2,),
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # OBSERVATION
        #
        # 4 x 180 LIDAR = 720
        # local goal distance = 1
        # sin bearing = 1
        # cos bearing = 1
        # v = 1
        # w = 1
        # previous linear action = 1
        # previous angular action = 1
        #
        # TOTAL = 727
        # ----------------------------------------------------

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(727,),
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # EPISODE STATE
        # ----------------------------------------------------

        self.episode_steps = 0

        self.previous_action = np.zeros(
            2,
            dtype=np.float32,
        )

        self.scan_history = deque(
            maxlen=4
        )

        # ----------------------------------------------------
        # GLOBAL PATH
        # ----------------------------------------------------

        self.global_goal = None

        self.global_path = None

        self.global_path_length = 0.0

        self.path_segment = 0

        self.previous_path_remaining = None

        # ----------------------------------------------------
        # MOTION HISTORY
        # ----------------------------------------------------

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        # ----------------------------------------------------
        # FIXED GLOBAL-PLANNER START
        # ----------------------------------------------------

        self._start = (
            0.0,
            0.0,
        )

        # ----------------------------------------------------
        # MISSION GOALS
        # ----------------------------------------------------

        self.goal_pool = [

            (-5.0, 4.0),

            (5.0, -4.0),

            (-3.0, 0.0),

            (3.0, 4.0),

            (5.0, 2.0),

            (-5.0, 2.5),

        ]

        # ----------------------------------------------------
        # 180 ANGLES
        # ----------------------------------------------------

        self._angles = np.linspace(
            -math.pi,
            math.pi,
            180,
            endpoint=False,
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # GAZEBO WORLD
        # ----------------------------------------------------

        self.world_name = (
            "assistive_world"
        )

        self.robot_name = (
            "assistive_rover"
        )

    # ========================================================
    # GAZEBO RESET
    # ========================================================

    def _reset_sim_pose(self):

        self.ros.stop()

        # ----------------------------------------------------
        # Physically move rover to episode start
        # ----------------------------------------------------

        request = (
            f'name: "{self.robot_name}", '
            f'position: {{'
            f'x: 0.0, '
            f'y: 0.0, '
            f'z: 0.20'
            f'}}, '
            f'orientation: {{'
            f'w: 1.0, '
            f'x: 0.0, '
            f'y: 0.0, '
            f'z: 0.0'
            f'}}'
        )

        service = (
            "/world/"
            + self.world_name
            + "/set_pose/blocking"
        )

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
                "3000",
                "--req",
                request,
            ],
            capture_output=True,
            text=True,
            timeout=5.0,
            check=False,
        )

        if "true" not in result.stdout.lower():

            raise RuntimeError(
                "Gazebo set_pose failed.\n"
                f"stdout={result.stdout}\n"
                f"stderr={result.stderr}"
            )

        print(
            "[V9 RESET] Gazebo set_pose accepted by "
            "/world/assistive_world/set_pose/blocking"
        )

        # ----------------------------------------------------
        # Require new sensor messages after the Gazebo pose reset.
        old_scan_seq = self.ros.scan_seq
        old_odom_seq = self.ros.odom_seq

        if not self.ros.wait_fresh(
            old_scan_seq, old_odom_seq, timeout=2.0
        ):
            raise RuntimeError(
                "No fresh /scan and /odom after Gazebo set_pose."
            )

        # ----------------------------------------------------
        # CRITICAL:
        # Gazebo pose resets, but DiffDrive odom is cumulative.
        # Treat current raw odom as the new episode origin.
        # ----------------------------------------------------

        self.ros.reset_odometry_reference()

        # ----------------------------------------------------
        # Let physics settle
        # ----------------------------------------------------

        settle_until = time.monotonic() + 0.40

        while time.monotonic() < settle_until:
            self.ros.publish(0.0, 0.0)
            rclpy.spin_once(self.ros, timeout_sec=0.02)

        self.ros.publish(
            0.0,
            0.0
        )

        # ----------------------------------------------------
        # Verify episode-local pose
        # ----------------------------------------------------

        px = float(self.ros.x)
        py = float(self.ros.y)
        pyaw = float(self.ros.yaw)

        pos_error = (
            px * px
            + py * py
        ) ** 0.5

        yaw_error = abs(
            _wrap_angle(pyaw)
        )

        if pos_error > 0.20 or yaw_error > 0.20:

            raise RuntimeError(
                "Episode-local odometry failed reset.\n"
                f"local pose="
                f"({px:.3f}, {py:.3f}, {pyaw:.3f})\n"
                f"raw pose="
                f"({self.ros.raw_x:.3f}, "
                f"{self.ros.raw_y:.3f}, "
                f"{self.ros.raw_yaw:.3f})"
            )

        print(
            "[V9 RESET] "
            f"local pose=("
            f"{px:.3f}, "
            f"{py:.3f}, "
            f"{pyaw:.3f}) "
            f"raw=("
            f"{self.ros.raw_x:.3f}, "
            f"{self.ros.raw_y:.3f}, "
            f"{self.ros.raw_yaw:.3f})"
        )

    def _publish_supervisor_goal(self):
        if self.global_goal is None:
            raise RuntimeError(
                "Cannot publish supervisor goal: "
                "global_goal is None."
            )

        ref_x = self.ros.odom_ref_x
        ref_y = self.ros.odom_ref_y
        ref_yaw = self.ros.odom_ref_yaw

        if (
            ref_x is None
            or ref_y is None
            or ref_yaw is None
        ):
            raise RuntimeError(
                "Cannot publish supervisor goal: "
                "odometry reference is not initialized."
            )

        gx, gy = self.global_goal

        # Convert episode-local V9 coordinates
        # into raw Gazebo /odom coordinates.
        raw_gx = (
            ref_x
            + math.cos(ref_yaw) * gx
            - math.sin(ref_yaw) * gy
        )

        raw_gy = (
            ref_y
            + math.sin(ref_yaw) * gx
            + math.cos(ref_yaw) * gy
        )

        self.ros.publish_goal(
            raw_gx,
            raw_gy,
        )

        print(
            "[V9 GOAL BRIDGE] "
            f"local=({gx:.2f},{gy:.2f}) "
            f"raw=({raw_gx:.2f},{raw_gy:.2f})"
        )

    def _select_goal(self):

        candidates = (
            self.goal_pool[:]
        )

        random.shuffle(
            candidates
        )

        for goal in candidates:

            try:

                path, length = (
                    self.planner.plan(
                        self._start,
                        goal,
                    )
                )

            except Exception:

                continue

            if length > 2.0:

                return (
                    goal,
                    path,
                    length,
                )

        raise RuntimeError(
            "No valid mission goal "
            "could be planned."
        )

    # ========================================================
    # LIDAR RESAMPLING
    # ========================================================

    def _resample_scan(self):

        raw = self.ros.scan

        if (
            raw is None
            or raw.size == 0
        ):

            return np.full(
                180,
                self.ros.range_max,
                dtype=np.float32,
            )

        x_old = np.linspace(
            0.0,
            1.0,
            raw.size,
            dtype=np.float32,
        )

        x_new = np.linspace(
            0.0,
            1.0,
            180,
            dtype=np.float32,
        )

        scan = np.interp(
            x_new,
            x_old,
            raw,
        )

        scan = np.clip(
            scan,
            self.ros.range_min,
            self.ros.range_max,
        ).astype(
            np.float32
        )

        return scan

    # ========================================================
    # LOCAL SUBGOAL
    # ========================================================

    def _local_goal_features(self):

        (
            target,
            progress,
            remaining,
            seg,
        ) = self.planner.local_subgoal(

            self.global_path,

            (
                self.ros.x,
                self.ros.y,
            ),

            lookahead=(
                self.goal_lookahead
            ),

            start_segment=(
                self.path_segment
            ),
        )

        self.path_segment = max(
            self.path_segment,
            seg,
        )

        dx = (
            target[0]
            - self.ros.x
        )

        dy = (
            target[1]
            - self.ros.y
        )

        distance = math.hypot(
            dx,
            dy,
        )

        bearing = _wrap_angle(
            math.atan2(
                dy,
                dx,
            )
            - self.ros.yaw
        )

        goal_dx = (
            self.global_goal[0]
            - self.ros.x
        )

        goal_dy = (
            self.global_goal[1]
            - self.ros.y
        )

        goal_distance = math.hypot(
            goal_dx,
            goal_dy,
        )

        return (
            target,
            progress,
            remaining,
            distance,
            bearing,
            goal_distance,
        )

    # ========================================================
    # OBSERVATION
    # ========================================================

    def _build_observation(self):

        scan = (
            self._resample_scan()
        )

        self.scan_history.append(
            scan
        )

        while (
            len(self.scan_history)
            < 4
        ):

            self.scan_history.appendleft(
                scan.copy()
            )

        lidar = np.stack(
            list(
                self.scan_history
            ),
            axis=0,
        ).astype(
            np.float32
        )

        lidar_norm = (
            2.0
            * (
                lidar
                - self.ros.range_min
            )
            / max(
                1e-6,
                (
                    self.ros.range_max
                    - self.ros.range_min
                ),
            )
            - 1.0
        )

        (
            _,
            _,
            _,
            local_distance,
            local_bearing,
            _,
        ) = (
            self._local_goal_features()
        )

        aux = np.array(
            [

                # Local subgoal distance
                np.clip(
                    local_distance / 5.0,
                    0.0,
                    1.0,
                )
                * 2.0
                - 1.0,

                # Bearing
                math.sin(
                    local_bearing
                ),

                math.cos(
                    local_bearing
                ),

                # Linear velocity
                np.clip(
                    self.ros.v
                    / self.max_linear_speed,
                    -1.0,
                    1.0,
                ),

                # Angular velocity
                np.clip(
                    self.ros.w
                    / self.max_angular_speed,
                    -1.0,
                    1.0,
                ),

                # Previous linear action
                float(
                    self.previous_action[0]
                ),

                # Previous angular action
                float(
                    self.previous_action[1]
                ),

            ],
            dtype=np.float32,
        )

        obs = np.concatenate(
            [
                lidar_norm.reshape(-1),
                aux,
            ]
        ).astype(
            np.float32
        )

        if obs.shape != (727,):

            raise RuntimeError(
                f"Invalid V9 observation "
                f"shape: {obs.shape}"
            )

        return obs

    # ========================================================
    # SAFETY SHIELD
    # ========================================================

    def _safety_shield(
        self,
        action,
    ):

        action = np.clip(
            np.asarray(
                action,
                dtype=np.float32,
            ),
            -1.0,
            1.0,
        )

        # ----------------------------------------------------
        # ACTION SMOOTHING
        # ----------------------------------------------------

        max_delta = 0.18

        executed = (
            self.previous_action
            + np.clip(
                action
                - self.previous_action,
                -max_delta,
                max_delta,
            )
        )

        linear = float(
            executed[0]
            * self.max_linear_speed
        )

        angular = float(
            executed[1]
            * self.max_angular_speed
        )

        # ----------------------------------------------------
        # LIDAR
        # ----------------------------------------------------

        scan = (
            self._resample_scan()
        )

        angles = (
            self._angles
        )

        front_mask = (
            np.abs(angles)
            <= math.radians(35.0)
        )

        left_mask = (
            (angles >= math.radians(45.0))
            &
            (angles <= math.radians(105.0))
        )

        right_mask = (
            (angles <= -math.radians(45.0))
            &
            (angles >= -math.radians(105.0))
        )

        rear_mask = (
            np.abs(
                np.abs(angles)
                - math.pi
            )
            <= math.radians(30.0)
        )

        front_clear = float(
            scan[
                front_mask
            ].min()
        )

        left_clear = float(
            scan[
                left_mask
            ].min()
        )

        right_clear = float(
            scan[
                right_mask
            ].min()
        )

        rear_clear = float(
            scan[
                rear_mask
            ].min()
        )

        # ----------------------------------------------------
        # FOOTPRINT
        # ----------------------------------------------------

        footprint_front = (
            self.rover_half_length
            + self.safety_margin
        )

        footprint_side = (
            self.rover_half_width
            + self.safety_margin
        )

        # ----------------------------------------------------
        # PREDICTIVE BUFFER
        # ----------------------------------------------------

        speed_buffer = max(
            0.08,
            abs(linear) * 0.45
            +
            (
                linear * linear
            )
            /
            (
                2.0 * 0.8
            ),
        )

        # ----------------------------------------------------
        # FORWARD
        # ----------------------------------------------------

        if linear > 0.0:

            threshold = (
                footprint_front
                + speed_buffer
            )

            if (
                front_clear
                < threshold
            ):

                available = max(
                    0.0,
                    front_clear
                    - footprint_front,
                )

                linear = min(
                    linear,
                    available / 0.45,
                )

                if (
                    front_clear
                    <
                    footprint_front
                    + 0.08
                ):

                    linear = 0.0

                    if (
                        left_clear
                        >
                        right_clear
                    ):

                        angular = max(
                            angular,
                            0.65,
                        )

                    else:

                        angular = min(
                            angular,
                            -0.65,
                        )

        # ----------------------------------------------------
        # REVERSE
        # ----------------------------------------------------

        elif linear < 0.0:

            threshold = (
                footprint_front
                + 0.10
            )

            if (
                rear_clear
                < threshold
            ):

                available = max(
                    0.0,
                    rear_clear
                    - footprint_front,
                )

                linear = max(
                    linear,
                    -available / 0.35,
                )

                if (
                    rear_clear
                    <
                    footprint_front
                    + 0.05
                ):

                    linear = 0.0

        # ----------------------------------------------------
        # TURNING
        # ----------------------------------------------------

        if (
            angular > 0.15
            and
            left_clear
            < footprint_side
        ):

            angular = min(
                angular,
                0.15,
            )

        if (
            angular < -0.15
            and
            right_clear
            < footprint_side
        ):

            angular = max(
                angular,
                -0.15,
            )

        # ----------------------------------------------------
        # NORMALIZE BACK TO ACTION SPACE
        # ----------------------------------------------------

        executed_linear = np.clip(
            linear
            /
            self.max_linear_speed,
            -1.0,
            1.0,
        )

        executed_angular = np.clip(
            angular
            /
            self.max_angular_speed,
            -1.0,
            1.0,
        )

        executed_action = np.array(
            [
                executed_linear,
                executed_angular,
            ],
            dtype=np.float32,
        )

        return (
            executed_action,
            float(
                min(
                    front_clear,
                    left_clear,
                    right_clear,
                    rear_clear,
                )
            ),
        )

    # ========================================================
    # REWARD
    # ========================================================

    def _reward(
        self,
        executed_action,
        path_remaining,
        local_bearing,
        min_clearance,
        goal_reached,
    ):

        reward = -0.01

        # ----------------------------------------------------
        # PATH PROGRESS
        # ----------------------------------------------------

        if (
            self.previous_path_remaining
            is not None
        ):

            progress = (
                self.previous_path_remaining
                - path_remaining
            )

            reward += (
                12.0
                * progress
            )

        # ----------------------------------------------------
        # FORWARD ALIGNMENT
        # ----------------------------------------------------

        heading_alignment = max(
            0.0,
            math.cos(
                local_bearing
            ),
        )

        reward += (
            0.25
            * self.ros.v
            * heading_alignment
        )

        # ----------------------------------------------------
        # CLEARANCE
        # ----------------------------------------------------

        if (
            min_clearance
            < 1.0
        ):

            reward -= (
                2.0
                * (
                    1.0
                    - min_clearance
                )
            )

        # ----------------------------------------------------
        # ACTION SMOOTHNESS
        # ----------------------------------------------------

        action_delta = np.linalg.norm(
            executed_action
            - self.previous_action
        )

        reward -= (
            0.05
            * float(
                action_delta
            )
        )

        # Slight penalty on excessive turning.
        reward -= (
            0.01
            * abs(
                float(
                    executed_action[1]
                )
            )
        )

        # ----------------------------------------------------
        # STUCK DETECTION
        # ----------------------------------------------------

        movement = 0.0

        if (
            self.previous_x
            is not None
        ):

            movement = math.hypot(
                self.ros.x
                - self.previous_x,

                self.ros.y
                - self.previous_y,
            )

        if (
            movement
            < 0.002
            and
            path_remaining
            > 0.8
        ):

            self.stuck_steps += 1

            reward -= 0.04

        else:

            self.stuck_steps = 0

        if (
            self.stuck_steps
            > 30
        ):

            reward -= 1.0

        # ----------------------------------------------------
        # SUCCESS
        # ----------------------------------------------------

        if goal_reached:

            reward += 100.0

        return float(
            reward
        )

    # ========================================================
    # RESET
    # ========================================================

    def reset(
        self,
        *,
        seed=None,
        options=None,
    ):

        super().reset(
            seed=seed
        )

        self.episode_steps = 0

        self.previous_action = np.zeros(
            2,
            dtype=np.float32,
        )

        self.scan_history.clear()

        self.path_segment = 0

        self.previous_path_remaining = None

        self.previous_x = None
        self.previous_y = None

        self.stuck_steps = 0

        # ----------------------------------------------------
        # REAL GAZEBO RESET
        # ----------------------------------------------------

        self._reset_sim_pose()

        # ----------------------------------------------------
        # SENSOR READINESS
        # ----------------------------------------------------

        if not self.ros.wait_ready(
            timeout=5.0
        ):

            raise RuntimeError(
                "ROS /scan or /odom "
                "did not become ready."
            )

        # ----------------------------------------------------
        # GLOBAL MISSION
        # ----------------------------------------------------

        (
            self.global_goal,
            self.global_path,
            self.global_path_length,
        ) = (
            self._select_goal()
        )

        
        
        # Synchronize the independent safety supervisor with V9.
        self._publish_supervisor_goal()

        # Allow ROS to deliver the goal message.
        rclpy.spin_once(
            self.ros,
            timeout_sec=0.05,
        )
# ----------------------------------------------------
        # INITIAL POSITION
        # ----------------------------------------------------

        self.previous_x = (
            self.ros.x
        )

        self.previous_y = (
            self.ros.y
        )

        # ----------------------------------------------------
        # LOCAL GOAL
        # ----------------------------------------------------

        (
            local_goal,
            progress,
            remaining,
            local_distance,
            local_bearing,
            global_distance,
        ) = (
            self._local_goal_features()
        )

        self.previous_path_remaining = (
            remaining
        )

        # ----------------------------------------------------
        # MISSION LOG
        # ----------------------------------------------------

        print(
            "\n"
            + "=" * 72
        )

        print(
            "[V9 MISSION START]"
        )

        print(
            f"Global goal: "
            f"("
            f"{self.global_goal[0]:.2f}, "
            f"{self.global_goal[1]:.2f}"
            f")"
        )

        print(
            f"Global path: "
            f"{len(self.global_path)} points, "
            f"{self.global_path_length:.2f} m"
        )

        print(
            f"Local subgoal: "
            f"("
            f"{local_goal[0]:.2f}, "
            f"{local_goal[1]:.2f}"
            f")"
        )

        print(
            f"Start pose: "
            f"("
            f"{self.ros.x:.2f}, "
            f"{self.ros.y:.2f}"
            f")"
        )

        print(
            f"Initial clearance: "
            f"{self.ros.min_range:.2f} m"
        )

        print(
            "=" * 72
        )

        # ----------------------------------------------------
        # OBS
        # ----------------------------------------------------

        obs = (
            self._build_observation()
        )

        info = {

            "global_goal":
                self.global_goal,

            "global_path_length":
                self.global_path_length,

            "local_goal":
                local_goal,

            "path_remaining":
                remaining,

            "global_goal_distance":
                global_distance,

            "min_clearance":
                self.ros.min_range,
        }

        return (
            obs,
            info,
        )

    # ========================================================
    # STEP
    # ========================================================

    def step(
        self,
        action,
    ):

        action = np.asarray(
            action,
            dtype=np.float32,
        )

        # ----------------------------------------------------
        # SAFETY SHIELD
        # ----------------------------------------------------

        (
            executed_action,
            min_clearance,
        ) = (
            self._safety_shield(
                action
            )
        )

        linear = float(
            executed_action[0]
            * self.max_linear_speed
        )

        angular = float(
            executed_action[1]
            * self.max_angular_speed
        )

        # ----------------------------------------------------
        # COMMAND
        # ----------------------------------------------------

        old_scan_seq = (
            self.ros.scan_seq
        )

        old_odom_seq = (
            self.ros.odom_seq
        )

        self.ros.publish(
            linear,
            angular,
        )

        sensor_fresh = self.ros.wait_fresh(
            old_scan_seq, old_odom_seq, timeout=0.5
        )

        if sensor_fresh:
            # Recalculate clearance using the updated LiDAR scan.
            _, min_clearance = self._safety_shield(
                np.zeros(2, dtype=np.float32)
            )
        else:
            # Stop when required sensor updates are missing.
            self.ros.stop()

        # ----------------------------------------------------
        # LOCAL GOAL
        # ----------------------------------------------------

        (
            local_goal,
            progress,
            path_remaining,
            local_distance,
            local_bearing,
            global_distance,
        ) = (
            self._local_goal_features()
        )

        self.episode_steps += 1

        # ----------------------------------------------------
        # TERMINAL CONDITIONS
        # ----------------------------------------------------

        goal_reached = bool(
            sensor_fresh and global_distance <= 0.45
        )

        collision = bool(
            sensor_fresh and min_clearance <= 0.34
        )

        terminated = bool(
            goal_reached
            or collision
        )

        truncated = bool(
            self.episode_steps >= self.max_episode_steps
            or not sensor_fresh
        )

        # ----------------------------------------------------
        # REWARD
        # ----------------------------------------------------

        reward = self._reward(
            executed_action,
            path_remaining,
            local_bearing,
            min_clearance,
            goal_reached,
        )

        if collision:
            reward -= 100.0
        elif not sensor_fresh:
            reward -= 20.0

        # ----------------------------------------------------
        # UPDATE HISTORY
        # ----------------------------------------------------

        self.previous_action = (
            executed_action.copy()
        )

        self.previous_path_remaining = (
            path_remaining
        )

        self.previous_x = (
            self.ros.x
        )

        self.previous_y = (
            self.ros.y
        )

        # ----------------------------------------------------
        # STOP ON TERMINATION
        # ----------------------------------------------------

        if (
            terminated
            or truncated
        ):

            self.ros.stop()

        # ----------------------------------------------------
        # NEXT OBS
        # ----------------------------------------------------

        obs = (
            self._build_observation()
        )

        # ----------------------------------------------------
        # INFO
        # ----------------------------------------------------

        info = {

            "global_goal_x":
                self.global_goal[0],

            "global_goal_y":
                self.global_goal[1],

            "global_path_length":
                self.global_path_length,

            "path_remaining":
                path_remaining,

            "local_goal_x":
                local_goal[0],

            "local_goal_y":
                local_goal[1],

            "local_goal_distance":
                local_distance,

            "local_goal_bearing":
                local_bearing,

            "global_goal_distance":
                global_distance,

            "min_clearance":
                min_clearance,

            "collision":
                collision,

            "goal_reached":
                goal_reached,

            "linear_velocity":
                linear,

            "angular_velocity":
                angular,

            "executed_action":
                executed_action.copy(),
        }

        info["sensor_fresh"] = sensor_fresh
        info["sensor_timeout"] = not sensor_fresh

        return (
            obs,
            reward,
            terminated,
            truncated,
            info,
        )

    # ========================================================
    # CLOSE
    # ========================================================

    def close(self):

        try:

            self.ros.stop()

            self.ros.destroy_node()

            if rclpy.ok():

                rclpy.shutdown()

        except Exception:

            pass


# ============================================================
# STANDALONE TEST
# ============================================================

if __name__ == "__main__":

    env = AssistiveNavEnvV9(
        max_episode_steps=100
    )

    try:

        obs, info = (
            env.reset()
        )

        print(
            "\nObservation shape:",
            obs.shape
        )

        for i in range(20):

            (
                obs,
                reward,
                terminated,
                truncated,
                info,
            ) = env.step(
                np.array(
                    [0.0, 0.0],
                    dtype=np.float32,
                )
            )

            print(
                f"step={i + 1:03d} "
                f"reward={reward:8.3f} "
                f"remaining="
                f"{info['path_remaining']:6.2f} "
                f"clearance="
                f"{info['min_clearance']:5.2f}"
            )

            if (
                terminated
                or truncated
            ):

                break

    finally:

        env.close()
