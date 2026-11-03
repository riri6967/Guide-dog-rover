"""Pi Camera Module 3 -> /camera/image_raw, /camera/camera_info.

Images are stamped with frame camera_link_optical, the URDF's optical frame
(+Z out of the lens), which is what image pipelines expect.

640 x 480 keeps the Pi 5's CPU free for SLAM and Nav2; raise it if the
perception model needs more.
"""
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    return LaunchDescription([
        DeclareLaunchArgument('width', default_value='640'),
        DeclareLaunchArgument('height', default_value='480'),
        Node(
            package='camera_ros',
            executable='camera_node',
            name='camera',
            parameters=[{
                'frame_id': 'camera_link_optical',
                'width': ParameterValue(LaunchConfiguration('width'), value_type=int),
                'height': ParameterValue(LaunchConfiguration('height'), value_type=int),
            }],
            output='screen',
        ),
    ])
