"""
yardbot_bringup/launch/nav2_bringup.launch.py

Nav2 bringup for Yardbot driveway testing (phase 1).

Starts:
  1. tags_bringup.launch.py  — URDF, RealSense (with pointcloud ON), AprilTag,
                                depth_to_grid
  2. RTAB-Map                — visual odometry + SLAM, publishes map→odom TF
  3. sabertooth_node         — Sabertooth 2x32 serial interface, subscribes
                                /cmd_vel
  4. Nav2                    — full navigation stack with nav2_params.yaml

Usage:
    ros2 launch yardbot_bringup nav2_bringup.launch.py

    # Override serial port if Sabertooth appears on a different device:
    ros2 launch yardbot_bringup nav2_bringup.launch.py serial_port:=/dev/ttyACM1

    # Use a pre-built map instead of live SLAM:
    ros2 launch yardbot_bringup nav2_bringup.launch.py \
        use_existing_map:=true map:=/path/to/map.yaml
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import (
    DeclareLaunchArgument,
    IncludeLaunchDescription,
    TimerAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────
    bringup_dir  = get_package_share_directory("yardbot_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────
    serial_port       = LaunchConfiguration("serial_port")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/sabertooth",
        description="Serial port for Sabertooth 2x32.",
    )
    
    sabertooth_node = Node(
        package="yardbot_bringup",
        executable="sabertooth_node",
        name="sabertooth",
        output="screen",
        parameters=[
            os.path.join(bringup_dir, "config", "sabertooth_node_params.yaml"),
            {
                "serial_port": serial_port,
            }
        ],
        remappings=[
            ("cmd_vel", "cmd_vel"),  # Nav2 → velocity_smoother → collision_monitor → here
        ],
    )


    return LaunchDescription([
        # Arguments
        declare_serial_port,
        
        sabertooth_node,
    ])
