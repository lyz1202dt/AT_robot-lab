from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, OpaqueFunction
from launch.conditions import IfCondition
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def launch_setup(context, *args, **kwargs):
    model_package = LaunchConfiguration("model_package").perform(context)
    arm_task_share = get_package_share_directory("arm_task")
    launch_pack_share = get_package_share_directory("launch_pack")

    urdf_filename = {
        "arm": "arm4.urdf",
        "arm2": "arm2.urdf",
    }[model_package]

    model_share = get_package_share_directory(model_package)
    urdf_path = os.path.join(model_share, "model", urdf_filename)
    task_config_filename = {
        "arm": "task_config.yaml",
        "arm2": "task_config2.yaml",
    }[model_package]
    task_config = os.path.join(arm_task_share, "config", task_config_filename)
    rviz_path = os.path.join(launch_pack_share, "rviz", "display_config.rviz")

    with open(urdf_path, "r", encoding="utf-8") as inf:
        robot_desc = inf.read()

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
        parameters=[{
            "model_package": model_package,
            "urdf_filename": urdf_filename,
        }],
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
        arguments=["--ros-args", "--params-file", task_config],
        output="screen",
    )

    arm_driver = Node(
        package="robot_driver",
        executable="robot_driver",
        output="screen",
    )

    # Static transform from joint1 to camera_link.
    # The orientation includes an additional -90-degree rotation about camera_link's
    # current z-axis.
    if model_package=="arm2":       #连杆狗（01云台）的ARM
        static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.32", "-0.02", "-0.03",
            "-1.570796", "0.0", "-2.356194",
            "joint1",
            "camera_link",
        ],
        output="screen",
        )
    else:       #同步带狗(05云台)的ARM
        static_tf_camera = Node(
        package="tf2_ros",
        executable="static_transform_publisher",
        arguments=[
            "0.232", "-0.022", "-0.03",
            "-1.570796", "0.0", "-2.356194",
            "joint1",
            "camera_link",
        ],
        output="screen",
        )
    

    return [
        robot_state_pub,
        joint_state_pub,
        arm_calc,
        rviz2,
        arm_task,
        arm_driver,
        static_tf_camera,
    ]


def generate_launch_description():
    model_package_arg = DeclareLaunchArgument(
        "model_package",
        default_value="arm",
        choices=["arm", "arm2"],
        description="URDF description package to load: arm or arm2",
    )

    show_rviz_arg = DeclareLaunchArgument(
        "show_rviz",
        default_value="true",
        description="Whether to start RViz2 together with MuJoCo simulation",
    )

    return LaunchDescription([
        model_package_arg,
        show_rviz_arg,
        OpaqueFunction(function=launch_setup),
    ])
