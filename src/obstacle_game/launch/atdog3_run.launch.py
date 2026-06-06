from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    rl_real_atdog3_node = Node(
        package="rl_sar",
        executable="rl_real_atdog3",
        output="screen",
    )

    return LaunchDescription([
        rl_real_atdog3_node,
    ])
