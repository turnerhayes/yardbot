# In yardbot_bringup/launch/teleop_rc.launch.py

from launch import LaunchDescription
from launch_ros.actions import Node

def generate_launch_description():
    rc_teleop_node = Node(
        package='yardbot_bringup',
        executable='rc_teleop_node',
        name='rc_teleop_node',
        output='screen',
    )

    return LaunchDescription([
        rc_teleop_node,
    ])