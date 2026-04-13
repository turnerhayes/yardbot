"""
yardbot_description/launch/robot_description.launch.py

Starts:
  1. robot_state_publisher  — reads the URDF/xacro, publishes the
                              base_link → camera_link TF chain.
  2. joint_state_publisher  — publishes zero joint states so
                              robot_state_publisher is happy (no real
                              joints yet, but needed to suppress warnings).
  3. static_transform_publisher (odom → base_link)
                            — placeholder identity transform so the full
                              map → odom → base_link → camera TF tree
                              exists.  Replace with real odometry later.

Usage (standalone):
    ros2 launch yardbot_description robot_description.launch.py

Usually you will include this from your top-level bringup launch:
    from launch.actions import IncludeLaunchDescription
    from launch.launch_description_sources import PythonLaunchDescriptionSource
    from ament_index_python.packages import get_package_share_directory
    import os

    desc = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("yardbot_description"),
                "launch", "robot_description.launch.py"
            )
        )
    )
"""

import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import LaunchConfiguration, Command
from launch_ros.actions import Node
from launch_ros.parameter_descriptions import ParameterValue


def generate_launch_description():
    pkg_share = get_package_share_directory("yardbot_description")
    urdf_xacro = os.path.join(pkg_share, "urdf", "yardbot.urdf.xacro")

    use_sim_time = LaunchConfiguration("use_sim_time")

    declare_use_sim_time = DeclareLaunchArgument(
        "use_sim_time",
        default_value="false",
        description="Use simulation (Gazebo) clock if true",
    )

    # Expand xacro → URDF string at launch time
    robot_description_content = ParameterValue(Command(["xacro ", urdf_xacro]), value_type=str)

    # ── 1. robot_state_publisher ──────────────────────────────────────────
    # Reads the URDF and publishes TF for every joint.
    # For fixed joints (all we have for now) it publishes static TFs.
    robot_state_publisher = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        name="robot_state_publisher",
        output="screen",
        parameters=[
            {"robot_description": robot_description_content},
            {"use_sim_time": use_sim_time},
        ],
    )

    # ── 2. joint_state_publisher ──────────────────────────────────────────
    # Publishes zero-valued joint states so robot_state_publisher
    # doesn't complain about missing joint data.
    # (We have no movable joints yet — remove when pan servo is added
    #  and replaced with the real joint_states source.)
    joint_state_publisher = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        name="joint_state_publisher",
        output="screen",
        parameters=[{"use_sim_time": use_sim_time}],
    )

    # ── 3. Static transform: odom → base_link (identity placeholder) ─────
    # This gives Nav2 / RViz a complete TF tree even before real
    # odometry exists.  Replace with your odometry node later.
    #
    # Format: x y z qx qy qz qw  parent_frame  child_frame
    static_odom_to_base = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_odom_to_base_link",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1",
                   "odom", "base_link"],
    )

    # ── 4. Static transform: map → odom (identity placeholder) ───────────
    # Lets map-frame nodes (Nav2, RViz fixed frame = "map") work without
    # a real localizer running.  Replace with AMCL / RTAB-Map / AprilTag
    # pose output later.
    static_map_to_odom = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        name="static_map_to_odom",
        output="screen",
        arguments=["0", "0", "0", "0", "0", "0", "1",
                   "map", "odom"],
    )

    return LaunchDescription([
        declare_use_sim_time,
        robot_state_publisher,
        joint_state_publisher,
        static_odom_to_base,
        static_map_to_odom,
    ])
