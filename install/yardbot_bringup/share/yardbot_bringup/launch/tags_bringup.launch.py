"""
yardbot_bringup/launch/tags_bringup.launch.py

Top-level bringup for early-stage Yardbot development.

Starts:
  1. yardbot_description    — URDF + full TF tree stubs
  2. RealSense D455          — RGB + depth streams
  3. apriltag_node           — tag36h11 detection on RGB stream
  4. depth_to_grid_node      — depth image → OccupancyGrid on /occupancy

Usage:
    # Normal mode (grid in base_link frame):
    ros2 launch yardbot_bringup tags_bringup.launch.py

    # Tripod/manual-pan testing (grid in camera frame, no base_link TF needed):
    ros2 launch yardbot_bringup tags_bringup.launch.py target_frame:=camera_link

    # Override tag size (measured black-square edge in metres):
    ros2 launch yardbot_bringup tags_bringup.launch.py tag_size:=0.154
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def generate_launch_description():

    # ── Launch arguments ──────────────────────────────────────────────────

    tag_size          = LaunchConfiguration("tag_size")
    image_rect_topic  = LaunchConfiguration("image_rect_topic")
    camera_info_topic = LaunchConfiguration("camera_info_topic")
    depth_topic       = LaunchConfiguration("depth_topic")
    depth_info_topic  = LaunchConfiguration("depth_info_topic")
    target_frame      = LaunchConfiguration("target_frame")

    declare_tag_size = DeclareLaunchArgument(
        "tag_size",
        default_value="0.16",
        description="AprilTag black-square edge length in metres. "
                    "Measure the black square only, not the white border.",
    )
    declare_image_rect_topic = DeclareLaunchArgument(
        "image_rect_topic",
        default_value="/camera/camera/color/image_raw",
        description="RGB image topic for AprilTag detection.",
    )
    declare_camera_info_topic = DeclareLaunchArgument(
        "camera_info_topic",
        default_value="/camera/camera/color/camera_info",
        description="CameraInfo topic matching image_rect_topic.",
    )
    declare_depth_topic = DeclareLaunchArgument(
        "depth_topic",
        default_value="/camera/camera/depth/image_rect_raw",
        description="Depth image topic for occupancy grid.",
    )
    declare_depth_info_topic = DeclareLaunchArgument(
        "depth_info_topic",
        default_value="/camera/camera/depth/camera_info",
        description="CameraInfo topic matching depth_topic.",
    )
    declare_target_frame = DeclareLaunchArgument(
        "target_frame",
        default_value="base_link",
        description=(
            "Frame to project the occupancy grid into. "
            "Use 'base_link' for normal robot operation. "
            "Use 'camera_link' for tripod/manual-pan testing "
            "when the full TF tree is not yet meaningful."
        ),
    )

    # ── 1. Robot description (URDF + TF stubs) ────────────────────────────
    description_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yardbot_description"),
                "launch",
                "robot_description.launch.py",
            )
        )
    )

    # ── 2. RealSense D455 ─────────────────────────────────────────────────
    realsense_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("realsense2_camera"),
                "launch",
                "rs_launch.py",
            )
        ),
        launch_arguments={
            "enable_color":      "true",
            "enable_depth":      "true",
            "enable_gyro":       "true",
            "enable_accel":      "true",
            "pointcloud.enable": "true",
        }.items(),
    )

    # ── 3a. QoS relay ─────────────────────────────────────────────────────
    # The RealSense publishes image_raw as RELIABLE + KEEP_LAST(1).  FastDDS
    # silently drops messages to BEST_EFFORT subscribers from such publishers.
    # This relay subscribes RELIABLY (so it receives every frame) and
    # republishes as BEST_EFFORT (compatible with apriltag_ros's image_transport
    # subscription).
    qos_relay = Node(
        package="yardbot_bringup",
        executable="qos_relay",
        name="qos_relay",
        output="screen",
        remappings=[
            ("~/in/image",       image_rect_topic),
            ("~/in/camera_info", camera_info_topic),
        ],
    )

    # ── 3b. AprilTag detector ─────────────────────────────────────────────
    apriltag_node = Node(
        package="apriltag_ros",
        executable="apriltag_node",
        name="apriltag",
        output="screen",
        remappings=[
            ("image_rect",  "/qos_relay/out/image"),
            ("camera_info", "/qos_relay/out/camera_info"),
        ],
        parameters=[{
            "family": "36h11",
            "size":   tag_size,
        }],
    )

    # ── 4. Depth → OccupancyGrid ──────────────────────────────────────────
    depth_to_grid_node = Node(
        package="yardbot_bringup",
        executable="depth_to_grid",
        name="depth_to_grid",
        output="screen",
        parameters=[{
            "depth_topic":       depth_topic,
            "camera_info_topic": depth_info_topic,
            "target_frame":      target_frame,
            # 10×10 m grid centred on the robot / camera
            "resolution":  0.05,
            "width_m":    10.0,
            "height_m":   10.0,
            "origin_x_m": -5.0,
            "origin_y_m": -5.0,
            # Only mark points between 5 cm and 40 cm above the target frame
            # as obstacles — filters out ground and overhead items.
            "z_min_m": 0.05,
            "z_max_m": 0.40,
            # Process every 4th pixel row/column for speed
            "stride": 4,
        }],
    )

    return LaunchDescription([
        declare_tag_size,
        declare_image_rect_topic,
        declare_camera_info_topic,
        declare_depth_topic,
        declare_depth_info_topic,
        declare_target_frame,

        description_launch,
        realsense_launch,
        qos_relay,
        TimerAction(period=3.0, actions=[apriltag_node]),
        depth_to_grid_node,
    ])