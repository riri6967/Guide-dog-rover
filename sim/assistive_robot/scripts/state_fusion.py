#!/usr/bin/env python3

import json
import math

import rclpy
from rclpy.node import Node

from sensor_msgs.msg import Imu
from nav_msgs.msg import Odometry
from std_msgs.msg import Float32MultiArray
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String


class StateFusion(Node):

    def __init__(self):
        super().__init__('state_fusion')

        # ---------------------------------------------------------
        # Parameters
        # ---------------------------------------------------------

        self.declare_parameter('goal_x', 5.0)
        self.declare_parameter('goal_y', 0.0)

        self.goal_x = float(
            self.get_parameter('goal_x').value
        )
        self.goal_y = float(
            self.get_parameter('goal_y').value
        )

        self.max_objects = 5

        # ---------------------------------------------------------
        # Latest sensor data
        # ---------------------------------------------------------

        self.lidar = [15.0] * 12

        self.objects = []

        self.robot_x = 0.0
        self.robot_y = 0.0
        self.robot_yaw = 0.0

        self.robot_v = 0.0
        self.robot_w = 0.0

        self.imu_angular = [0.0, 0.0, 0.0]
        self.imu_accel = [0.0, 0.0, 9.81]

        # ---------------------------------------------------------
        # Subscribers
        # ---------------------------------------------------------

        self.create_subscription(
            Float32MultiArray,
            '/lidar_features',
            self.lidar_callback,
            10
        )

        self.create_subscription(
            String,
            '/semantic_objects',
            self.semantic_callback,
            10
        )

        self.create_subscription(
            Imu,
            '/imu',
            self.imu_callback,
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

        # ---------------------------------------------------------
        # Publishers
        # ---------------------------------------------------------

        self.state_pub = self.create_publisher(
            Float32MultiArray,
            '/dr_state',
            10
        )

        self.debug_pub = self.create_publisher(
            String,
            '/dr_state_debug',
            10
        )

        # 10 Hz state generation
        self.timer = self.create_timer(
            0.1,
            self.publish_state
        )

        self.frame_count = 0

        self.get_logger().info(
            'State fusion started'
        )

    # -------------------------------------------------------------
    # Callbacks
    # -------------------------------------------------------------

    def lidar_callback(self, msg):

        values = list(msg.data)

        if len(values) >= 12:
            self.lidar = values[:12]

    def semantic_callback(self, msg):

        try:
            data = json.loads(msg.data)

            objects = data.get(
                'objects',
                []
            )

            # Nearest first
            objects.sort(
                key=lambda obj: float(
                    obj.get('distance_m', 99.0)
                )
            )

            self.objects = objects[
                :self.max_objects
            ]

        except Exception as e:

            self.get_logger().warning(
                f'Semantic JSON error: {e}'
            )

    def imu_callback(self, msg):

        self.imu_angular = [
            float(msg.angular_velocity.x),
            float(msg.angular_velocity.y),
            float(msg.angular_velocity.z)
        ]

        self.imu_accel = [
            float(msg.linear_acceleration.x),
            float(msg.linear_acceleration.y),
            float(msg.linear_acceleration.z)
        ]

    def odom_callback(self, msg):

        self.robot_x = float(
            msg.pose.pose.position.x
        )

        self.robot_y = float(
            msg.pose.pose.position.y
        )

        q = msg.pose.pose.orientation

        # Quaternion → yaw
        siny_cosp = (
            2.0 *
            (q.w * q.z + q.x * q.y)
        )

        cosy_cosp = (
            1.0 -
            2.0 *
            (q.y * q.y + q.z * q.z)
        )

        self.robot_yaw = math.atan2(
            siny_cosp,
            cosy_cosp
        )

        self.robot_v = float(
            msg.twist.twist.linear.x
        )

        self.robot_w = float(
            msg.twist.twist.angular.z
        )

    def goal_callback(self, msg):

        self.goal_x = float(
            msg.pose.position.x
        )

        self.goal_y = float(
            msg.pose.position.y
        )

    # -------------------------------------------------------------
    # Utility functions
    # -------------------------------------------------------------

    def normalize_angle(self, angle):

        while angle > math.pi:
            angle -= 2.0 * math.pi

        while angle < -math.pi:
            angle += 2.0 * math.pi

        return angle

    def object_class_id(self, class_name):

        # Temporary semantic vocabulary.
        # This will later be replaced by the project's
        # custom HARP/environment class vocabulary.

        vocabulary = {
            'person': 1,
            'chair': 2,
            'table': 3,
            'door': 4,
            'bed': 5,
            'stairs': 6,
            'dustbin': 7,
            'shelf': 8,
            'window': 9,
            'wall': 10,
            'sports ball': 11,
        }

        return vocabulary.get(
            class_name.lower(),
            0
        )

    # -------------------------------------------------------------
    # State construction
    # -------------------------------------------------------------

    def build_state(self):

        state = []

        # ---------------------------------------------------------
        # 1. LiDAR
        # ---------------------------------------------------------

        for distance in self.lidar:

            distance = float(distance)

            if not math.isfinite(distance):
                distance = 15.0

            distance = max(
                0.0,
                min(15.0, distance)
            )

            # Normalize 0-15 m → 0-1
            state.append(
                distance / 15.0
            )

        # ---------------------------------------------------------
        # 2. Goal
        # ---------------------------------------------------------

        dx = self.goal_x - self.robot_x
        dy = self.goal_y - self.robot_y

        goal_distance = math.sqrt(
            dx * dx + dy * dy
        )

        goal_angle_world = math.atan2(
            dy,
            dx
        )

        goal_bearing = self.normalize_angle(
            goal_angle_world - self.robot_yaw
        )

        # Normalize to approximately -1..1
        goal_distance_norm = min(
            goal_distance / 20.0,
            1.0
        )

        goal_bearing_norm = (
            goal_bearing / math.pi
        )

        state.append(goal_distance_norm)
        state.append(goal_bearing_norm)

        # ---------------------------------------------------------
        # 3. Semantic objects
        #
        # Five objects × five features:
        #
        # class ID
        # confidence
        # distance
        # bearing
        # vertical image-plane offset
        # ---------------------------------------------------------

        for i in range(self.max_objects):

            if i < len(self.objects):

                obj = self.objects[i]

                class_id = self.object_class_id(
                    obj.get(
                        'class',
                        'unknown'
                    )
                )

                confidence = float(
                    obj.get(
                        'confidence',
                        0.0
                    )
                )

                distance = float(
                    obj.get(
                        'distance_m',
                        10.0
                    )
                )

                bearing = float(
                    obj.get(
                        'bearing_deg',
                        0.0
                    )
                )

                vertical = float(
                    obj.get(
                        'y_m',
                        0.0
                    )
                )

                # Normalize
                class_norm = (
                    class_id / 11.0
                )

                distance_norm = min(
                    max(distance, 0.0) / 10.0,
                    1.0
                )

                bearing_norm = max(
                    -1.0,
                    min(
                        1.0,
                        bearing / 90.0
                    )
                )

                vertical_norm = max(
                    -1.0,
                    min(
                        1.0,
                        vertical / 2.0
                    )
                )

                state.extend([
                    class_norm,
                    confidence,
                    distance_norm,
                    bearing_norm,
                    vertical_norm
                ])

            else:

                # Empty object slot
                state.extend([
                    0.0,
                    0.0,
                    1.0,
                    0.0,
                    0.0
                ])

        # ---------------------------------------------------------
        # 4. Robot motion
        # ---------------------------------------------------------

        state.append(
            max(
                -1.0,
                min(
                    1.0,
                    self.robot_v / 1.0
                )
            )
        )

        state.append(
            max(
                -1.0,
                min(
                    1.0,
                    self.robot_w / 2.0
                )
            )
        )

        # ---------------------------------------------------------
        # 5. IMU angular velocity
        # ---------------------------------------------------------

        for value in self.imu_angular:

            state.append(
                max(
                    -1.0,
                    min(
                        1.0,
                        value / 5.0
                    )
                )
            )

        # ---------------------------------------------------------
        # 6. IMU acceleration
        # ---------------------------------------------------------

        for value in self.imu_accel:

            state.append(
                max(
                    -1.0,
                    min(
                        1.0,
                        value / 20.0
                    )
                )
            )

        # ---------------------------------------------------------
        # 7. Robot yaw
        # ---------------------------------------------------------

        state.append(
            self.robot_yaw / math.pi
        )

        return state

    # -------------------------------------------------------------
    # Publish state
    # -------------------------------------------------------------

    def publish_state(self):

        state = self.build_state()

        msg = Float32MultiArray()

        msg.data = [
            float(value)
            for value in state
        ]

        self.state_pub.publish(msg)

        debug = {
            'state_dimension': len(state),
            'lidar_features': 12,
            'semantic_slots': self.max_objects,
            'objects_detected': len(
                self.objects
            ),
            'goal': {
                'x': self.goal_x,
                'y': self.goal_y
            },
            'robot': {
                'x': self.robot_x,
                'y': self.robot_y,
                'yaw': self.robot_yaw,
                'v': self.robot_v,
                'w': self.robot_w
            }
        }

        debug_msg = String()

        debug_msg.data = json.dumps(
            debug
        )

        self.debug_pub.publish(
            debug_msg
        )

        self.frame_count += 1

        if self.frame_count % 100 == 0:

            self.get_logger().info(
                f'DRL state published | '
                f'dimension={len(state)} | '
                f'objects={len(self.objects)}'
            )


def main(args=None):

    rclpy.init(args=args)

    node = StateFusion()

    try:
        rclpy.spin(node)

    except KeyboardInterrupt:
        pass

    finally:

        node.destroy_node()

        if rclpy.ok():
            rclpy.shutdown()


if __name__ == '__main__':
    main()
