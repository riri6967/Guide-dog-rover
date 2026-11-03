"""RPLIDAR A1M8 -> /scan in frame `laser` (matches the URDF).

    ros2 launch rover_bringup lidar.launch.py port:=/dev/rplidar

The default port is the udev symlink from udev/99-rover.rules. Without it, the
LiDAR and the ESP32 are both /dev/ttyUSB*, and which is which changes with plug
order.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('port', default_value='/dev/rplidar'),
        Node(
            package='rplidar_ros',
            executable='rplidar_node',
            name='rplidar_node',
            parameters=[{
                'channel_type': 'serial',
                'serial_port': LaunchConfiguration('port'),
                'serial_baudrate': 115200,   # A1
                'frame_id': 'laser',
                'inverted': False,
                'angle_compensate': True,
            }],
            output='screen',
        ),
    ])
