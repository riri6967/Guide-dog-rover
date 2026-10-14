#!/usr/bin/env python3
"""Nav2 launch for the DRL controller comparison: identical to navigation.launch.py (frozen baseline) except that
controller_server's follow_path action is renamed to follow_path_mppi_unused, so bt_navigator's FollowPath goals go to
training/deploy/drl_controller.py. controller_server still runs because the BT's recoveries use its local costmap services."""
from pathlib import Path

from launch import LaunchDescription
from launch_ros.actions import Node

# Resolve configuration relative to this launch file, avoiding a hard-coded home path.
NAV2_DIR = Path(__file__).resolve().parents[1]
PARAMS = str(NAV2_DIR / "params" / "nav2_params.yaml")
DRL_OVERRIDES = str(NAV2_DIR / "params" / "drl_overrides.yaml")  # bt_navigator default_server_timeout 200 ms


def nav_node(package, executable, name, remappings=None):
    return Node(
        package=package,
        executable=executable,
        name=name,
        output="screen",
        parameters=[PARAMS, DRL_OVERRIDES] if name == "bt_navigator" else [PARAMS],
        remappings=remappings or [],
    )


def generate_launch_description():
    lifecycle_nodes = [
        "map_server",
        "amcl",
        "controller_server",
        "planner_server",
        "smoother_server",
        "behavior_server",
        "bt_navigator",
        "velocity_smoother",
        "collision_monitor",
    ]

    nodes = [
        nav_node("nav2_map_server", "map_server", "map_server"),
        nav_node("nav2_amcl", "amcl", "amcl"),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node(
            "nav2_controller",
            "controller_server",
            "controller_server",
            remappings=[("cmd_vel", "cmd_vel_nav"), ("follow_path", "follow_path_mppi_unused")],
        ),
        nav_node("nav2_smoother", "smoother_server", "smoother_server"),
        nav_node(
            "nav2_behaviors",
            "behavior_server",
            "behavior_server",
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        nav_node(
            "nav2_velocity_smoother",
            "velocity_smoother",
            "velocity_smoother",
            remappings=[("cmd_vel", "cmd_vel_nav")],
        ),
        nav_node(
            "nav2_collision_monitor",
            "collision_monitor",
            "collision_monitor",
        ),
        Node(
            package="tf2_ros",
            executable="static_transform_publisher",
            name="base_to_lidar",
            output="screen",
            arguments=[
                "--x", "0.0", "--y", "0.0", "--z", "0.68",
                "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                "--frame-id", "base_link",
                "--child-frame-id", "assistive_rover/base_link/lidar",
            ],
        ),
        Node(
            package="nav2_lifecycle_manager",
            executable="lifecycle_manager",
            name="lifecycle_manager_navigation",
            output="screen",
            parameters=[
                {
                    "use_sim_time": True,
                    "autostart": True,
                    "node_names": lifecycle_nodes,
                }
            ],
        ),
    ]
    return LaunchDescription(nodes)
