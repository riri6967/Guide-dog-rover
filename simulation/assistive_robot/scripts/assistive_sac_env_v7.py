import math
import time
from collections import deque

import gymnasium as gym
import numpy as np
import rclpy
from sensor_msgs.msg import LaserScan

import assistive_sac_env_v6 as v6


BaseV6 = getattr(v6, "AssistiveNavEnvV6", None)

if BaseV6 is None:
    raise RuntimeError(
        "Could not find AssistiveNavEnvV6 in assistive_sac_env_v6.py"
    )


class AssistiveNavEnvV7(BaseV6):
    """
    V7 CONTINUOUS NAVIGATION ENVIRONMENT

    - One Gazebo reset/spawn per mission.
    - Targets change without resetting the rover.
    - Policy sees generic sensor/relative-goal information only.
    - No room/doorway labels are exposed to the policy.
    - Target reaching is NOT an episode termination.
    - Existing safety supervisor remains below SAC:
          SAC -> /sac_cmd_vel -> supervisor -> /cmd_vel
    """

    # ============================================================
    # CONTINUOUS MISSION ROUTE
    # ============================================================
    #
    # These coordinates are simulator-side mission targets.
    # They are NOT exposed to SAC as coordinates.
    #
    # They form a continuous traversal through the existing map:
    # corridor -> lower area -> corridor -> upper area -> back.
    #
    # We deliberately use intermediate targets so that the local
    # policy does not need to solve a huge jump around an obstacle.
    #
    # ============================================================

    MISSION_ROUTE = [
        (4.0,  0.0),
        (5.0, -1.0),

        (5.0, -2.2),
        (5.0, -3.0),
        (5.0, -4.0),
        (5.0, -5.0),
        (5.0, -4.0),
        (5.0, -2.2),

        (5.0, -1.0),
        (2.0,  0.0),

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
        (0.0, 0.0),
    ]

    TARGET_REACHED_RADIUS = 0.45

    MAX_MISSION_STEPS = 5000

    # LiDAR/contact terminal safety.
    COLLISION_LIDAR_THRESHOLD = 0.28

    # A waypoint is considered reached, but the mission continues.
    # This is intentionally much different from V6.

    def __init__(self):

        super().__init__()

        # --------------------------------------------------------
        # V7 observation stays compatible with the V6 checkpoint:
        # four 48-D states stacked => 192-D.
        # --------------------------------------------------------

        self.observation_space = gym.spaces.Box(
            low=-1.0,
            high=1.0,
            shape=(192,),
            dtype=np.float32,
        )

        self.state_history = deque(maxlen=4)

        # --------------------------------------------------------
        # Mission state
        # --------------------------------------------------------

        self.target_index = 0
        self.targets_reached = 0

        self.mission_active = False
        self.mission_complete = False

        # --------------------------------------------------------
        # Checkpoint bookkeeping.
        # Training recovery/teleport will be added after this
        # continuous architecture is proven.
        # --------------------------------------------------------

        self.last_checkpoint_x = None
        self.last_checkpoint_y = None
        self.last_checkpoint_yaw = None

        self.checkpoint_spacing = 0.75

        # --------------------------------------------------------
        # Extra stuck monitoring.
        # The supervisor remains responsible for physical recovery.
        # --------------------------------------------------------

        self.last_motion_check_x = None
        self.last_motion_check_y = None
        self.last_motion_check_time = None

        self.stuck_detected = False

        # --------------------------------------------------------
        # Action smoothing state.
        # Keep semantics close to V6 for the first transfer test.
        # The stronger V7 acceleration controller comes during
        # fresh training.
        # --------------------------------------------------------

        self.previous_linear_cmd = 0.0
        self.previous_angular_cmd = 0.0

        # --------------------------------------------------------
        # RAW LIDAR FOR V7 RECOVERY
        # --------------------------------------------------------

        self.raw_scan = None

        self.scan_subscription = self.ros.create_subscription(
            LaserScan,
            "/scan",
            self._v7_scan_callback,
            10
        )

        # --------------------------------------------------------
        # LOOP / STAGNATION DETECTION
        # --------------------------------------------------------

        self.progress_window_start_time = None
        self.progress_window_start_distance = None
        self.progress_window_start_path = 0.0

        self.best_goal_distance = None
        self.best_goal_distance_time = None

        self.loop_detected = False

        # Recovery state:
        # 0 = normal
        # 1 = reverse
        # 2 = turn
        # 3 = escape-forward
        # 4 = cooldown

        self.recovery_mode = 0
        self.recovery_start_time = 0.0
        self.recovery_direction = 1.0

        self.RECOVERY_REVERSE_TIME = 1.2
        self.RECOVERY_TURN_TIME = 2.0
        self.RECOVERY_FORWARD_TIME = 1.2
        self.RECOVERY_COOLDOWN_TIME = 1.0

        self.LOOP_WINDOW_TIME = 6.0
        self.LOOP_MIN_PROGRESS = 0.20
        self.LOOP_MIN_PATH = 0.80

        self.REAR_SAFE_DISTANCE = 0.90
        self.SIDE_SAFE_DISTANCE = 0.75

        print("[V7] loop detector + recovery enabled")

    # ============================================================
    # HELPERS
    # ============================================================

    @staticmethod
    def wrap_angle(angle):
        return math.atan2(
            math.sin(angle),
            math.cos(angle)
        )

    def _v7_scan_callback(self, msg):
        self.raw_scan = msg

    def _sector_min(self, center_deg, width_deg):
        if self.raw_scan is None:
            return 15.0

        center = math.radians(center_deg)
        half = math.radians(width_deg)

        best = 15.0

        for i, r in enumerate(self.raw_scan.ranges):

            if not math.isfinite(r):
                continue

            if not (
                self.raw_scan.range_min
                <= r
                <= self.raw_scan.range_max
            ):
                continue

            angle = (
                self.raw_scan.angle_min
                + i * self.raw_scan.angle_increment
            )

            diff = (
                (angle - center + math.pi)
                % (2.0 * math.pi)
            ) - math.pi

            if abs(diff) <= half:
                best = min(best, float(r))

        return best

    def _choose_escape_direction(self):

        left = self._sector_min(90.0, 35.0)
        right = self._sector_min(-90.0, 35.0)

        if left >= self.SIDE_SAFE_DISTANCE and right >= self.SIDE_SAFE_DISTANCE:

            if left > right:
                return 1.0
            return -1.0

        if left >= self.SIDE_SAFE_DISTANCE:
            return 1.0

        if right >= self.SIDE_SAFE_DISTANCE:
            return -1.0

        # Neither side is comfortably open.
        # Choose the less obstructed side anyway.
        return 1.0 if left >= right else -1.0

    def _reset_loop_monitor(self, state):

        self.progress_window_start_time = time.monotonic()

        self.progress_window_start_distance = (
            self.goal_distance_from_state(state)
        )

        self.progress_window_start_path = self.path_length

        self.best_goal_distance = (
            self.progress_window_start_distance
        )

        self.best_goal_distance_time = time.monotonic()

        self.loop_detected = False

    def _update_loop_detection(self, state):

        distance = self.goal_distance_from_state(state)
        now = time.monotonic()

        if self.progress_window_start_time is None:
            self._reset_loop_monitor(state)
            return False

        self.best_goal_distance = min(
            self.best_goal_distance,
            distance
        )

        elapsed = (
            now
            - self.progress_window_start_time
        )

        window_path = (
            self.path_length
            - self.progress_window_start_path
        )

        actual_progress = (
            self.progress_window_start_distance
            - self.best_goal_distance
        )

        loop = bool(
            elapsed >= self.LOOP_WINDOW_TIME
            and window_path >= self.LOOP_MIN_PATH
            and actual_progress < self.LOOP_MIN_PROGRESS
            and distance > self.TARGET_REACHED_RADIUS + 0.5
        )

        if loop and not self.loop_detected:

            self.loop_detected = True

            print(
                "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            )
            print(
                "[V7 LOOP DETECTED]"
            )
            print(
                f"window={elapsed:.1f}s "
                f"path_added={window_path:.2f}m "
                f"distance_start="
                f"{self.progress_window_start_distance:.2f}m "
                f"best="
                f"{self.best_goal_distance:.2f}m "
                f"progress="
                f"{actual_progress:.2f}m"
            )
            print(
                "Starting physical escape recovery."
            )
            print(
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!\n"
            )

            self._start_v7_recovery()

            return True

        return False

    def _start_v7_recovery(self):

        self.recovery_mode = 1
        self.recovery_start_time = time.monotonic()

        self.recovery_direction = (
            self._choose_escape_direction()
        )

        print(
            "[V7 RECOVERY] "
            f"direction="
            f"{'LEFT' if self.recovery_direction > 0 else 'RIGHT'}"
        )

    def _run_v7_recovery(self):

        elapsed = (
            time.monotonic()
            - self.recovery_start_time
        )

        rear = self._sector_min(
            180.0,
            35.0
        )

        front = self._sector_min(
            0.0,
            35.0
        )

        # --------------------------------------------------------
        # Phase 1: reverse only when rear is clear
        # --------------------------------------------------------

        if self.recovery_mode == 1:

            if (
                elapsed < self.RECOVERY_REVERSE_TIME
                and rear > self.REAR_SAFE_DISTANCE
            ):

                self.ros.publish_action(
                    -0.22,
                    0.0
                )

                return True

            self.recovery_mode = 2
            self.recovery_start_time = time.monotonic()

            return True

        # --------------------------------------------------------
        # Phase 2: turn toward the more open side
        # --------------------------------------------------------

        if self.recovery_mode == 2:

            if elapsed < self.RECOVERY_TURN_TIME:

                self.ros.publish_action(
                    0.0,
                    self.recovery_direction * 0.85
                )

                return True

            self.recovery_mode = 3
            self.recovery_start_time = time.monotonic()

            return True

        # --------------------------------------------------------
        # Phase 3: short escape-forward movement
        # --------------------------------------------------------

        if self.recovery_mode == 3:

            if (
                elapsed < self.RECOVERY_FORWARD_TIME
                and front > 0.80
            ):

                self.ros.publish_action(
                    0.20,
                    self.recovery_direction * 0.20
                )

                return True

            self.recovery_mode = 4
            self.recovery_start_time = time.monotonic()

            return True

        # --------------------------------------------------------
        # Phase 4: cooldown
        # --------------------------------------------------------

        if self.recovery_mode == 4:

            self.ros.publish_action(
                0.0,
                0.0
            )

            if elapsed >= self.RECOVERY_COOLDOWN_TIME:

                self.recovery_mode = 0
                self._reset_loop_monitor(
                    self.ros.state
                )

                print(
                    "[V7 RECOVERY] complete; "
                    "returning control to SAC"
                )

            return True

        return False

    def current_robot_yaw(self, state=None):
        if state is None:
            state = self.ros.state

        if state is None or len(state) < 48:
            return 0.0

        # State fusion stores normalized yaw at state[47].
        return float(state[47]) * math.pi

    def stacked_observation(self):
        if not self.state_history:
            raise RuntimeError("State history is empty.")

        while len(self.state_history) < 4:
            self.state_history.appendleft(
                self.state_history[-1].copy()
            )

        return np.concatenate(
            list(self.state_history),
            axis=0
        ).astype(np.float32)

    def current_target(self):
        return self.MISSION_ROUTE[self.target_index]

    def publish_current_target(self):
        self.goal_x, self.goal_y = self.current_target()

        print(
            f"[TARGET] {self.target_index + 1}/"
            f"{len(self.MISSION_ROUTE)} "
            f"-> ({self.goal_x:.2f}, {self.goal_y:.2f})"
        )

        # Publish repeatedly exactly like the proven V6 mechanism.
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

    def wait_for_current_target_state(self, timeout=5.0):

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

            robot_x = float(self.ros.odom_x)
            robot_y = float(self.ros.odom_y)

            dx = self.goal_x - robot_x
            dy = self.goal_y - robot_y

            expected_distance = math.hypot(dx, dy)

            yaw = self.current_robot_yaw(state)

            expected_bearing = self.wrap_angle(
                math.atan2(dy, dx) - yaw
            )

            actual_distance = float(state[12]) * 20.0
            actual_bearing = float(state[13]) * math.pi

            distance_error = abs(
                actual_distance - expected_distance
            )

            bearing_error = abs(
                self.wrap_angle(
                    actual_bearing - expected_bearing
                )
            )

            if (
                distance_error < 0.30
                and bearing_error < 0.15
            ):
                return state

        if self.ros.state is not None:
            return self.ros.state.copy()

        raise RuntimeError(
            "Timed out waiting for current V7 target state."
        )

    def collision_now(self):

        min_lidar = float(
            getattr(self.ros, "min_lidar", 15.0)
        )

        contact = False

        monitor = getattr(
            self,
            "contact_monitor",
            None
        )

        if monitor is not None:
            contact = bool(
                getattr(
                    monitor,
                    "contact_detected",
                    False
                )
            )

        ros_contact = bool(
            getattr(
                self.ros,
                "contact_detected",
                False
            )
        )

        return (
            contact
            or ros_contact
            or min_lidar < self.COLLISION_LIDAR_THRESHOLD
        )

    def update_checkpoint(self):

        if (
            self.ros.odom_x is None
            or self.ros.odom_y is None
            or self.ros.state is None
        ):
            return

        x = float(self.ros.odom_x)
        y = float(self.ros.odom_y)

        if (
            self.last_checkpoint_x is None
            or math.hypot(
                x - self.last_checkpoint_x,
                y - self.last_checkpoint_y
            ) >= self.checkpoint_spacing
        ):

            clearance = float(
                getattr(
                    self.ros,
                    "min_lidar",
                    15.0
                )
            )

            # Only update a checkpoint when the area is reasonably
            # clear. This is simulator-side bookkeeping.
            if clearance > 1.0:

                self.last_checkpoint_x = x
                self.last_checkpoint_y = y
                self.last_checkpoint_yaw = (
                    self.current_robot_yaw()
                )

                print(
                    "[CHECKPOINT] "
                    f"({x:.2f}, {y:.2f})"
                )

    def update_stuck_monitor(self, commanded_linear):

        now = time.monotonic()

        if (
            self.last_motion_check_x is None
            or self.last_motion_check_y is None
        ):

            self.last_motion_check_x = float(
                self.ros.odom_x
            )
            self.last_motion_check_y = float(
                self.ros.odom_y
            )
            self.last_motion_check_time = now

            self.stuck_detected = False
            return

        elapsed = (
            now - self.last_motion_check_time
        )

        displacement = math.hypot(
            float(self.ros.odom_x)
            - self.last_motion_check_x,
            float(self.ros.odom_y)
            - self.last_motion_check_y
        )

        # Actual "I am commanding movement but not progressing"
        # detector. The supervisor already has its own recovery logic.
        self.stuck_detected = bool(
            commanded_linear > 0.15
            and elapsed >= 3.0
            and displacement < 0.05
            and float(
                getattr(
                    self.ros,
                    "min_lidar",
                    15.0
                )
            ) < 1.2
        )

        if elapsed >= 1.0:

            self.last_motion_check_x = float(
                self.ros.odom_x
            )
            self.last_motion_check_y = float(
                self.ros.odom_y
            )
            self.last_motion_check_time = now

    # ============================================================
    # RESET
    # ============================================================

    def reset(
        self,
        *,
        seed=None,
        options=None
    ):

        # IMPORTANT:
        # Call Gym's reset directly.
        # Never call BaseV6.reset(), because V6 reset performs
        # the old independent-task episode behavior.

        gym.Env.reset(
            self,
            seed=seed
        )

        self.mission_active = True
        self.mission_complete = False

        self.target_index = 0
        self.targets_reached = 0

        self.episode_steps = 0
        self.path_length = 0.0

        self.previous_goal_distance = None
        self.previous_goal_bearing = None
        self.previous_action = np.zeros(
            2,
            dtype=np.float32
        )

        self.previous_x = None
        self.previous_y = None
        self.stuck_steps = 0

        self.state_history.clear()

        self.last_checkpoint_x = None
        self.last_checkpoint_y = None
        self.last_checkpoint_yaw = None

        self.last_motion_check_x = None
        self.last_motion_check_y = None
        self.last_motion_check_time = None
        self.stuck_detected = False

        self.previous_linear_cmd = 0.0
        self.previous_angular_cmd = 0.0

        # Clear stale state from the old mission.
        self.ros.state = None
        self.ros.state_seq = 0
        self.ros.odom_x = None
        self.ros.odom_y = None

        # --------------------------------------------------------
        # THE ONLY PHYSICAL RESET OF THE MISSION
        # --------------------------------------------------------

        print("\n================================================")
        print("V7 NEW CONTINUOUS MISSION")
        print("================================================")
        print("[MISSION] Gazebo reset/spawn: ONCE")
        print(
            f"[MISSION] Targets: {len(self.MISSION_ROUTE)}"
        )

        self.reset_gazebo()

        # --------------------------------------------------------
        # First target
        # --------------------------------------------------------

        self.publish_current_target()

        state = self.wait_for_current_target_state()

        # Initialize history without introducing artificial
        # temporal discontinuity.
        for _ in range(4):
            self.state_history.append(
                state.copy()
            )

        self.previous_x = self.ros.odom_x
        self.previous_y = self.ros.odom_y

        self.previous_goal_distance = (
            self.goal_distance_from_state(state)
        )

        self.previous_goal_bearing = (
            float(state[13]) * math.pi
        )

        self.last_checkpoint_x = float(
            self.ros.odom_x
        )
        self.last_checkpoint_y = float(
            self.ros.odom_y
        )
        self.last_checkpoint_yaw = (
            self.current_robot_yaw(state)
        )

        self.last_motion_check_x = float(
            self.ros.odom_x
        )
        self.last_motion_check_y = float(
            self.ros.odom_y
        )
        self.last_motion_check_time = time.monotonic()

        self._reset_loop_monitor(
            state
        )

        info = {
            "mission_started": True,
            "mission_complete": False,
            "target_index": self.target_index,
            "targets_reached": self.targets_reached,
            "target_count": len(self.MISSION_ROUTE),
            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_distance": (
                self.previous_goal_distance
            ),
            "path_length": 0.0,
        }

        print(
            "[MISSION] "
            f"Target 1/{len(self.MISSION_ROUTE)} active"
        )

        return (
            self.stacked_observation(),
            info
        )

    # ============================================================
    # STEP
    # ============================================================

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

        # --------------------------------------------------------
        # V7 PHYSICAL RECOVERY OVERRIDES SAC TEMPORARILY
        # --------------------------------------------------------

        if self.recovery_mode != 0:

            handled = self._run_v7_recovery()

            if handled:

                state = self.wait_for_new_state(
                    self.ros.state_seq
                )

                self.state_history.append(
                    state.copy()
                )

                self.episode_steps += 1

                if self.episode_steps >= self.MAX_MISSION_STEPS:

                    self.ros.stop()

                    return (
                        self.stacked_observation(),
                        -1.0,
                        False,
                        True,
                        {
                            "mission_complete": False,
                            "target_index": self.target_index,
                            "targets_reached": self.targets_reached,
                            "target_reached": False,
                            "collision": self.collision_now(),
                            "stuck_detected": True,
                            "recovery_active": True,
                            "path_length": self.path_length,
                        }
                    )

                return (
                    self.stacked_observation(),
                    -0.20,
                    False,
                    False,
                    {
                        "mission_complete": False,
                        "target_index": self.target_index,
                        "targets_reached": self.targets_reached,
                        "target_reached": False,
                        "collision": self.collision_now(),
                        "stuck_detected": True,
                        "recovery_active": True,
                        "path_length": self.path_length,
                    }
                )

        # V6-compatible velocity mapping.
        linear, angular = (
            self.action_to_velocity(action)
        )

        previous_seq = self.ros.state_seq

        # SAC -> /sac_cmd_vel
        # Safety supervisor -> /cmd_vel
        self.ros.publish_action(
            linear,
            angular
        )

        state = self.wait_for_new_state(
            previous_seq
        )

        # --------------------------------------------------------
        # PATH LENGTH
        # --------------------------------------------------------

        if (
            self.ros.odom_x is not None
            and self.ros.odom_y is not None
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

            movement = math.hypot(
                dx,
                dy
            )

            self.path_length += movement

            self.previous_x = self.ros.odom_x
            self.previous_y = self.ros.odom_y

        # --------------------------------------------------------
        # BASE V6 REWARD SHAPING
        # --------------------------------------------------------

        reward = self.calculate_reward(
            state,
            action,
            linear
        )

        goal_distance = (
            self.goal_distance_from_state(state)
        )

        min_lidar = float(
            getattr(
                self.ros,
                "min_lidar",
                15.0
            )
        )

        # --------------------------------------------------------
        # CHECKPOINT / STUCK MONITOR
        # --------------------------------------------------------

        self.update_checkpoint()

        self.update_stuck_monitor(
            linear
        )

        loop_detected_now = (
            self._update_loop_detection(
                state
            )
        )

        self.stuck_detected = bool(
            self.stuck_detected
            or loop_detected_now
            or self.recovery_mode != 0
        )

        if self.stuck_detected:

            # Physical recovery is handled by V7 recovery + shield.
            # The mission remains alive.
            reward -= 0.5

        # --------------------------------------------------------
        # COLLISION
        # --------------------------------------------------------

        collision = self.collision_now()

        # --------------------------------------------------------
        # TARGET REACHED
        # --------------------------------------------------------

        target_reached = (
            goal_distance
            < self.TARGET_REACHED_RADIUS
        )

        mission_complete = False
        target_changed = False

        if target_reached and not collision:

            self.targets_reached += 1

            print(
                "\n--------------------------------------------"
            )
            print(
                "[TARGET REACHED]"
                f" {self.target_index + 1}/"
                f"{len(self.MISSION_ROUTE)}"
            )
            print(
                f"[TARGET REACHED] "
                f"path={self.path_length:.2f} m"
            )
            print(
                "--------------------------------------------"
            )

            reward += 100.0

            # ----------------------------------------------------
            # CRITICAL:
            # DO NOT terminate.
            # DO NOT reset Gazebo.
            # DO NOT respawn.
            # ----------------------------------------------------

            if (
                self.target_index
                >= len(self.MISSION_ROUTE) - 1
            ):

                mission_complete = True
                self.mission_complete = True
                self.mission_active = False

                print(
                    "\n================================================"
                )
                print(
                    "✅ CONTINUOUS MISSION COMPLETE"
                )
                print(
                    f"Targets reached: "
                    f"{self.targets_reached}/"
                    f"{len(self.MISSION_ROUTE)}"
                )
                print(
                    f"Path length: "
                    f"{self.path_length:.2f} m"
                )
                print(
                    "================================================"
                )

            else:

                self.target_index += 1
                target_changed = True

                self.publish_current_target()

                # Get state after state_fusion has accepted the
                # new target. This keeps SAC's observation aligned
                # with the target currently being pursued.
                state = self.wait_for_current_target_state()

                self.state_history.clear()

                for _ in range(4):
                    self.state_history.append(
                        state.copy()
                    )

                self.previous_goal_distance = (
                    self.goal_distance_from_state(
                        state
                    )
                )

                self.previous_goal_bearing = (
                    float(state[13]) * math.pi
                )

                self._reset_loop_monitor(
                    state
                )

                print(
                    "[MISSION CONTINUE] "
                    f"Target {self.target_index + 1}/"
                    f"{len(self.MISSION_ROUTE)}"
                )

        # --------------------------------------------------------
        # COLLISION = TRUE TERMINAL
        # --------------------------------------------------------

        terminated = bool(
            collision
            or mission_complete
        )

        if collision:

            reward -= 100.0

            print(
                "\n!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            )
            print(
                "❌ COLLISION TERMINATED MISSION"
            )
            print(
                f"Target index: {self.target_index + 1}"
            )
            print(
                f"Path length: {self.path_length:.2f} m"
            )
            print(
                "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"
            )

        # --------------------------------------------------------
        # MISSION TIMEOUT
        # --------------------------------------------------------

        self.episode_steps += 1

        truncated = (
            self.episode_steps
            >= self.MAX_MISSION_STEPS
        )

        if truncated and not mission_complete:

            print(
                "\n[MISSION TIMEOUT]"
                f" steps={self.episode_steps}"
            )

        if terminated or truncated:

            self.ros.stop()

        # --------------------------------------------------------
        # TEMPORAL OBSERVATION
        # --------------------------------------------------------

        self.state_history.append(
            state.copy()
        )

        observation = (
            self.stacked_observation()
        )

        info = {
            "mission_started": True,
            "mission_complete": mission_complete,
            "target_index": self.target_index,
            "targets_reached": self.targets_reached,
            "target_count": len(self.MISSION_ROUTE),

            "target_changed": target_changed,
            "target_reached": target_reached,

            "goal_x": self.goal_x,
            "goal_y": self.goal_y,
            "goal_distance": goal_distance,

            "min_lidar_distance": min_lidar,
            "collision": collision,

            "stuck_detected": self.stuck_detected,

            "checkpoint_x": self.last_checkpoint_x,
            "checkpoint_y": self.last_checkpoint_y,

            "path_length": self.path_length,

            "linear_velocity": linear,
            "angular_velocity": angular,
        }

        return (
            observation,
            float(reward),
            terminated,
            truncated,
            info
        )


if __name__ == "__main__":

    print(
        "V7 module loaded. "
        "Run evaluate_v7_continuous.py for the mission test."
    )
