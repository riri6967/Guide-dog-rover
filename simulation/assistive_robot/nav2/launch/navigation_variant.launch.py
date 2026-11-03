#!/usr/bin/env python3
"""Nav2 launch for controlled variants of the frozen baseline (navigation.launch.py is NOT modified).
Environment variables:
  NAV2_CONTROLLER=mppi|drl  drl: controller_server's follow_path renamed to follow_path_mppi_unused (training/deploy/
                            drl_controller.py serves follow_path) and drl_overrides.yaml applied (BT ack timeout 200 ms)
  NAV2_OVERRIDES=a.yaml,b.yaml  extra parameter files (paths relative to nav2/params) layered on nav2_params.yaml for all nodes
"""
import os
from pathlib import Path
from launch import LaunchDescription
from launch_ros.actions import Node

NAV2_DIR = Path(__file__).resolve().parents[1]
PARAMS = str(NAV2_DIR / "params" / "nav2_params.yaml")
CONTROLLER = os.environ.get("NAV2_CONTROLLER", "mppi")
EXTRA = [str(NAV2_DIR / "params" / f) for f in os.environ.get("NAV2_OVERRIDES", "").split(",") if f]
if CONTROLLER == "drl":
    EXTRA.append(str(NAV2_DIR / "params" / "drl_overrides.yaml"))


def nav_node(package, executable, name, remappings=None):
    return Node(package=package, executable=executable, name=name, output="screen",
                parameters=[PARAMS] + EXTRA, remappings=remappings or [])


def generate_launch_description():
    lifecycle_nodes = ["map_server", "amcl", "controller_server", "planner_server", "smoother_server",
                       "behavior_server", "bt_navigator", "velocity_smoother", "collision_monitor"]
    ctrl_remap = [("cmd_vel", "cmd_vel_nav")] + ([("follow_path", "follow_path_mppi_unused")] if CONTROLLER == "drl" else [])
    nodes = [
        nav_node("nav2_map_server", "map_server", "map_server"),
        nav_node("nav2_amcl", "amcl", "amcl"),
        nav_node("nav2_planner", "planner_server", "planner_server"),
        nav_node("nav2_controller", "controller_server", "controller_server", remappings=ctrl_remap),
        nav_node("nav2_smoother", "smoother_server", "smoother_server"),
        nav_node("nav2_behaviors", "behavior_server", "behavior_server", remappings=[("cmd_vel", "cmd_vel_nav")]),
        nav_node("nav2_bt_navigator", "bt_navigator", "bt_navigator"),
        nav_node("nav2_velocity_smoother", "velocity_smoother", "velocity_smoother", remappings=[("cmd_vel", "cmd_vel_nav")]),
        nav_node("nav2_collision_monitor", "collision_monitor", "collision_monitor"),
        Node(package="tf2_ros", executable="static_transform_publisher", name="base_to_lidar", output="screen",
             arguments=["--x", "0.0", "--y", "0.0", "--z", "0.68", "--roll", "0.0", "--pitch", "0.0", "--yaw", "0.0",
                        "--frame-id", "base_link", "--child-frame-id", "assistive_rover/base_link/lidar"]),
        Node(package="nav2_lifecycle_manager", executable="lifecycle_manager", name="lifecycle_manager_navigation",
             output="screen", parameters=[{"use_sim_time": True, "autostart": True, "node_names": lifecycle_nodes}]),
    ]
    return LaunchDescription(nodes)
