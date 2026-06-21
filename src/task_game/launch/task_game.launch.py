from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    task_game_share = get_package_share_directory("task_game")
    arm_share = get_package_share_directory("arm")
    arm_task_share = get_package_share_directory("arm_task")
    launch_pack_share = get_package_share_directory("launch_pack")

    plan_config_path = os.path.join(
        task_game_share,
        "config",
        "generate_plan.yaml",
    )
    urdf_path = os.path.join(arm_share, "model", "arm4.urdf")
    task_config = os.path.join(arm_task_share, "config", "task_config.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2",
    )

    robot_control_node = Node(
        package="task_game",
        executable="robot_control",
        output="screen",
        parameters=[{"generate_plan_config": plan_config_path}],
    )

    robot_state_pub = Node(
        package="robot_state_publisher",
        executable="robot_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    joint_state_pub = Node(
        package="joint_state_publisher",
        executable="joint_state_publisher",
        parameters=[{"robot_description": robot_desc}],
        output="screen",
    )

    arm_calc = Node(
        package="arm_calc",
        executable="arm_calc",
        output="screen",
    )

    rviz2 = Node(
        package="rviz2",
        executable="rviz2",
        arguments=["-d", rviz_path],
        condition=IfCondition(LaunchConfiguration("show_rviz")),
    )

    arm_task = Node(
        package="arm_task",
        executable="arm_task",
        parameters=[task_config],
        output="screen",
    )

    arm_driver = Node(
        package="robot_driver",
        executable="robot_driver",
        output="screen",
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

    static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "-0.03165", "0.0", "0.042",
            "0.7071", "0.0", "0.7071", "0.0",
            "joint5",
            "camera_link",
        ],
        output="screen",
    )

    return LaunchDescription([
        show_rviz_arg,
        robot_control_node,
        robot_state_pub,
        joint_state_pub,
        arm_calc,
        rviz2,
        arm_task,
        arm_driver,
        static_tf_arm,
        static_tf_camera,
    ])
