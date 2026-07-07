from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration, PathJoinSubstitution, TextSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    launch_pack_share = get_package_share_directory("launch_pack")

    plan_config_path = PathJoinSubstitution([
        FindPackageShare("task_game"),
        "config",
        [TextSubstitution(text="generate_plan_"), LaunchConfiguration("side"), TextSubstitution(text=".yaml")],
    ])
    arm_vision_launch_path = os.path.join(
        launch_pack_share,
        "launch",
        "arm_vision_task.launch.py",
    )

    side_arg = DeclareLaunchArgument(
        "side",
        default_value="left",
        description="Which side config to load: 'left' or 'right'.",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 from arm_vision_task.launch.py.",
    )

    arm_vision_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(arm_vision_launch_path),
        launch_arguments={
            "show_rviz": LaunchConfiguration("show_rviz"),
        }.items(),
    )

    robot_control_node = Node(
        package="task_game",
        executable="robot_control",
        output="screen",
        parameters=[{"generate_plan_config": plan_config_path}],
    )

    static_tf_arm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.146", "0.0", "0.073",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "arm_base_link",
        ],
        output="screen",
    )

    return LaunchDescription([
        side_arg,
        show_rviz_arg,
        arm_vision_launch,
        static_tf_arm,
        TimerAction(period=2.0, actions=[robot_control_node]),
    ])
