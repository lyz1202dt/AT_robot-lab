from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    remote_test_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            os.path.join(
                get_package_share_directory("obstacle_game"),
                "launch",
                "remote_test.launch.py",
            )
        )
    )

    rl_real_atdog2_node = Node(
        package="rl_sar",
        executable="rl_real_atdog2",
        output="screen",
    )

    return LaunchDescription([
        #remote_test_launch,
        rl_real_atdog2_node,
    ])
