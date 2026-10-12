"""Publish the rover's static TF tree from the URDF.

    ros2 launch rover_description description.launch.py

robot_state_publisher turns the URDF into TF (base_footprint -> base_link ->
laser / imu_link / camera_link / wheels). joint_state_publisher supplies zero
angles for the four wheel joints; without it the wheels never appear in TF.
Once something publishes real /joint_states from the encoders, launch with
joint_states:=false.
"""

import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    xacro_file = os.path.join(
        get_package_share_directory('rover_description'), 'urdf', 'rover.urdf.xacro')

    robot_description = ParameterValue(Command(['xacro ', xacro_file]), value_type=str)

    return LaunchDescription([
        DeclareLaunchArgument('use_sim_time', default_value='false'),
        DeclareLaunchArgument('joint_states', default_value='true',
                              description='Publish zero wheel angles'),

        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            parameters=[{
                'robot_description': robot_description,
                'use_sim_time': LaunchConfiguration('use_sim_time'),
            }],
        ),

        Node(
            package='joint_state_publisher',
            executable='joint_state_publisher',
            condition=IfCondition(LaunchConfiguration('joint_states')),
        ),
    ])
