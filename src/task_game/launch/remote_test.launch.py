from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    plan_config_path = os.path.join(
        get_package_share_directory("task_game"),
        "config",
        "generate_plan.yaml",
    )

    robot_control_node = Node(
        package="task_game",
        executable="robot_control",
        output="screen",
        parameters=[{"generate_plan_config": plan_config_path}],
    )

    remote_node = Node(
        package="remote_node",
        executable="remote_node",
        output="screen",
    )
    
    static_tf_arm = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.146", "0.0", "0.073",
            "0.0", "0.0", "0.0", "1.0",
            "base_link",
            "arm_base_link"
        ],
        output="screen",
    )

    test_node = Node(
        package="task_game",
        executable="test_node",
        output="screen",
    )

    return LaunchDescription([
        test_node,
        robot_control_node,
        static_tf_arm
        #remote_node,
    ])
