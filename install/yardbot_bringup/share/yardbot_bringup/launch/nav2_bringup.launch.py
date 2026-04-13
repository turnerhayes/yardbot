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
    GroupAction,
)
from launch.conditions import IfCondition, UnlessCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution
from launch_ros.actions import Node, PushRosNamespace
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():

    # ── Package directories ───────────────────────────────────────────────
    bringup_dir  = get_package_share_directory("yardbot_bringup")
    nav2_bringup = get_package_share_directory("nav2_bringup")

    # ── Launch arguments ──────────────────────────────────────────────────
    serial_port       = LaunchConfiguration("serial_port")
    use_existing_map  = LaunchConfiguration("use_existing_map")
    map_yaml          = LaunchConfiguration("map")

    declare_serial_port = DeclareLaunchArgument(
        "serial_port",
        default_value="/dev/sabertooth",
        description="Serial port for Sabertooth 2x32.",
    )
    declare_use_existing_map = DeclareLaunchArgument(
        "use_existing_map",
        default_value="false",
        description="If true, load a saved map and run RTAB-Map in localization "
                    "mode instead of full SLAM.",
    )
    declare_map = DeclareLaunchArgument(
        "map",
        default_value="",
        description="Path to map YAML file. Only used when use_existing_map:=true.",
    )

    # ── 1. tags_bringup (URDF + RealSense + AprilTag + depth_to_grid) ─────
    # NOTE: pointcloud.enable is overridden to true here so the Nav2
    # obstacle layer can consume /camera/camera/depth/color/points.
    # tags_bringup passes launch_arguments down to the RealSense launch.
    tags_bringup = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(bringup_dir, "launch", "tags_bringup.launch.py")
        ),
        launch_arguments={
            "pointcloud.enable": "true",   # override tags_bringup default
        }.items(),
    )

    # ── 2a. RTAB-Map odometry ─────────────────────────────────────────────
    rtabmap_odom = Node(
        condition=UnlessCondition(use_existing_map),
        package="rtabmap_odom",
        executable="rgbd_odometry",
        name="rtabmap_odom",
        output="screen",
        parameters=[{
            "frame_id":          "base_link",
            "odom_frame_id":     "odom",
            "approx_sync":       True,
            "queue_size":        10,
            "Vis/MinInliers":    "10",
            "OdomF2M/MaxSize":   "1000",
        }],
        remappings=[
            ("rgb/image",       "/camera/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("depth/image",     "/camera/camera/depth/image_rect_raw"),
        ],
    )

    # ── 2b. RTAB-Map SLAM ─────────────────────────────────────────────────
    rtabmap_slam = Node(
        condition=UnlessCondition(use_existing_map),
        package="rtabmap_slam",
        executable="rtabmap",
        name="rtabmap",
        output="screen",
        arguments=["--delete_db_on_start"],
        parameters=[{
            "frame_id":              "base_link",
            "odom_frame_id":         "odom",
            "subscribe_depth":       True,
            "subscribe_rgb":         True,
            "subscribe_odom_info":   True,
            "approx_sync":           True,
            "queue_size":            10,
            "Rtabmap/DetectionRate": "1",
            "Mem/STMSize":           "30",
            "publish_tf":            False,  # odom node owns the odom→base_link TF
        }],
        remappings=[
            ("rgb/image",       "/camera/camera/color/image_raw"),
            ("rgb/camera_info", "/camera/camera/color/camera_info"),
            ("depth/image",     "/camera/camera/depth/image_rect_raw"),
            ("odom",            "/odom"),
        ],
    )

    # ── 3. Sabertooth node ────────────────────────────────────────────────
    sabertooth_node = Node(
        package="yardbot_bringup",
        executable="sabertooth_node",
        name="sabertooth",
        output="screen",
        parameters=[{
            "serial_port": serial_port,
            "baud_rate":   9600,
            "max_speed":   30,       # Sabertooth 0-127 value at max_linear_vel
        }],
        remappings=[
            ("cmd_vel", "cmd_vel"),  # Nav2 → velocity_smoother → collision_monitor → here
        ],
    )

    # ── 4. Nav2 ───────────────────────────────────────────────────────────
    # Use nav2_bringup's navigation_launch.py which starts the full stack
    # (controller, planner, behavior, costmaps, bt_navigator, etc.) but
    # NOT the map server — RTAB-Map owns the map.
    nav2_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(nav2_bringup, "launch", "navigation_launch.py")
        ),
        launch_arguments={
            "use_sim_time":  "false",
            "params_file":   os.path.join(bringup_dir, "config", "nav2_params.yaml"),
            "autostart":     "true",
        }.items(),
    )

    # ── 4b. Map server (only when loading existing map) ───────────────────
    map_server = Node(
        condition=IfCondition(use_existing_map),
        package="nav2_map_server",
        executable="map_server",
        name="map_server",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "yaml_filename": map_yaml,
        }],
    )

    lifecycle_manager_map = Node(
        condition=IfCondition(use_existing_map),
        package="nav2_lifecycle_manager",
        executable="lifecycle_manager",
        name="lifecycle_manager_map",
        output="screen",
        parameters=[{
            "use_sim_time": False,
            "autostart":    True,
            "node_names":   ["map_server"],
        }],
    )

    return LaunchDescription([
        # Arguments
        declare_serial_port,
        declare_use_existing_map,
        declare_map,

        # Bringup in order, with delays to let each layer stabilise
        tags_bringup,                                   # t=0  sensors + TF
        TimerAction(period=3.0, actions=[              # t=3  RTAB-Map
            rtabmap_slam,
            rtabmap_odom,
        ]),
        TimerAction(period=4.0, actions=[              # t=4  Sabertooth
            sabertooth_node,
        ]),
        TimerAction(period=6.0, actions=[              # t=6  Nav2
            nav2_launch,
            map_server,
            lifecycle_manager_map,
        ]),
    ])
