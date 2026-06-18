from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory

import os


def generate_launch_description():
    task_game_share = get_package_share_directory("task_game")
    launch_pack_share = get_package_share_directory("launch_pack")

    point_yaml_path = os.path.join(task_game_share, "config", "point.yaml")
    arm_vision_launch_path = os.path.join(
        launch_pack_share,
        "launch",
        "arm_vision_task.launch.py",
    )

    point_yaml_arg = DeclareLaunchArgument(
        "point_yaml_path",
        default_value=point_yaml_path,
        description="Path to the point.yaml file updated by measure_node.",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="false",
        description="Whether to start RViz2 from arm_vision_task.launch.py.",
    )

    arm_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_vision_launch_path),
        launch_arguments={
            "show_rviz": LaunchConfiguration("show_rviz"),
        }.items(),
    )

    rl_real_atdog2_node = Node(
        package="rl_sar",
        executable="rl_real_atdog2",
        name="rl_real_atdog2",
        output="screen",
    )

    measure_node = Node(
        package="task_game",
        executable="measure_node",
        name="measure_node",
        output="screen",
        emulate_tty=True,
        parameters=[
            {
                "point_yaml_path": LaunchConfiguration("point_yaml_path"),
            }
        ],
    )

    static_tf_arm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.146",
            "0.0",
            "0.073",
            "0.0",
            "0.0",
            "0.0",
            "1.0",
            "base_link",
            "arm_base_link",
        ],
        output="screen",
    )

    return LaunchDescription(
        [
            point_yaml_arg,
            show_rviz_arg,
            arm_vision_launch,
            rl_real_atdog2_node,
            static_tf_arm,
            measure_node,
        ]
    )
