from launch import LaunchDescription
from launch_ros.actions import Node


def generate_launch_description():
    return LaunchDescription(
        [
            Node(
                package="task_game_ui",
                executable="task_game_ui_node",
                name="task_game_ui",
                output="screen",
            ),
        ]
    )
