from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():
    arm_share = get_package_share_directory("arm")
    arm_task_share = get_package_share_directory("arm_task")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_path = os.path.join(arm_share, "model", "arm4.urdf")
    task_config = os.path.join(arm_task_share, "config", "task_config.yaml")
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
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

    # Static transform from link4 to camera_link
    # Based on robotic_arm.xml: camera at pos="0.1 0.09 -0.03" relative to link4
    # xyaxes="0 0 1 0 1 0" means x-axis points in z direction, y-axis points in y direction
    # This is a 90-degree rotation about y-axis
    # Quaternion for 90-degree rotation about y-axis: (0, 0.7071, 0, 0.7071)
    static_tf_camera = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "-0.03165", "0.0", "0.042",
        "0.7071", "0.0", "0.7071", "0.0",
        "joint5",
        "camera_link"
    ],
    output="screen",
)
    #模拟发布出来的箱子的坐标
    static_tf_object_test = Node(
    package="tf2_ros",
    executable="static_transform_publisher",
    arguments=[
        "0.4", "0.0", "-0.1",
        "0.7071", "0.0", "0.7071", "0.0",
        "arm_base_link",
        "object_frame"
    ],
    output="screen",
)

    return LaunchDescription([
        show_rviz_arg,
        robot_state_pub,
        joint_state_pub,
        arm_calc,
        rviz2,
        arm_task,
        arm_driver,
        static_tf_camera,
        static_tf_object_test
    ])
