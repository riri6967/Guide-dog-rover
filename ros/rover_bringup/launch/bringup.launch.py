"""Start the whole robot with one command.

    ros2 launch rover_bringup bringup.launch.py

Starts, each switchable with <name>:=false:
    agent     micro-ROS agent for the ESP32 (pi_scripts/start_agent.sh)
    URDF      robot_state_publisher (always on)
    odometry  /ticks_left, /ticks_right -> /wheel/odom and the wheel joints
    imu       MPU6050 -> /imu/data_raw   (keep the rover still while it starts)
    ekf       wheel odometry + IMU -> /odometry/filtered, odom -> base_footprint
    lidar     RPLIDAR A1M8 -> /scan
    camera    Pi Camera v3 -> /camera/image_raw

SLAM and Nav2 go on top of this; they need /scan, /odometry/filtered and the TF
tree map -> odom -> base_footprint -> base_link -> sensors.
"""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, ExecuteProcess, IncludeLaunchDescription
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():
    here = get_package_share_directory('rover_bringup')
    description = get_package_share_directory('rover_description')
    on = lambda name: IfCondition(LaunchConfiguration(name))

    def include(pkg_dir, name, args=None):
        return IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(pkg_dir, 'launch', name)),
            launch_arguments=(args or {}).items())

    switches = [DeclareLaunchArgument(n, default_value='true')
                for n in ('agent', 'odometry', 'imu', 'ekf', 'lidar', 'camera')]

    return LaunchDescription(switches + [
        DeclareLaunchArgument('agent_script',
                              default_value=os.path.expanduser('~/Guide-dog-rover/pi_scripts/start_agent.sh')),
        DeclareLaunchArgument('lidar_port', default_value='/dev/rplidar'),

        ExecuteProcess(cmd=[LaunchConfiguration('agent_script')], output='screen',
                       condition=on('agent')),

        # Wheel joints come from the odometry node, so no zero-angle publisher.
        include(description, 'description.launch.py', {'joint_states': 'false'}),

        Node(package='rover_bringup', executable='wheel_odometry', output='screen',
             condition=on('odometry')),
        Node(package='rover_bringup', executable='mpu6050_imu', output='screen',
             condition=on('imu')),
        Node(package='robot_localization', executable='ekf_node', name='ekf_filter_node',
             parameters=[os.path.join(here, 'config', 'ekf.yaml')], output='screen',
             condition=on('ekf')),

        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'lidar.launch.py')),
            launch_arguments={'port': LaunchConfiguration('lidar_port')}.items(),
            condition=on('lidar')),
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(os.path.join(here, 'launch', 'camera.launch.py')),
            condition=on('camera')),
    ])
