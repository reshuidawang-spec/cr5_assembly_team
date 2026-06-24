"""Piston spray real-robot launcher — launches MoveIt and the CR5 controller for the piston spray painting application on the physical robot."""
import os

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, SetEnvironmentVariable
from launch.conditions import IfCondition
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def include_launch(package_name, launch_file_name, *, condition=None):
    """Include launch."""
    launch_path = os.path.join(
        get_package_share_directory(package_name),
        "launch",
        launch_file_name,
    )
    return IncludeLaunchDescription(
        PythonLaunchDescriptionSource(launch_path),
        condition=condition,
    )


def generate_launch_description():
    """Generate launch description."""
    default_robot_ip = os.environ.get("IP_address", "192.168.5.1")
    default_robot_type = os.environ.get("DOBOT_TYPE", "cr5")
    default_qt_platform = os.environ.get("QT_QPA_PLATFORM", "xcb")

    robot_ip = LaunchConfiguration("robot_ip")
    robot_type = LaunchConfiguration("robot_type")
    qt_platform = LaunchConfiguration("qt_platform")
    start_rviz = LaunchConfiguration("start_rviz")
    start_gui = LaunchConfiguration("start_gui")

    bringup_node = Node(
        package="cr_robot_ros2",
        executable="cr_robot_ros2_node",
        name="dobot_bringup_ros2",
        output="screen",
        parameters=[
            {"robot_ip_address": robot_ip},
            {"robot_type": robot_type},
            {"trajectory_duration": 0.3},
            {"robot_node_name": "dobot_bringup_ros2"},
            {"robot_number": 1},
        ],
    )

    joint_bridge_node = include_launch("dobot_moveit", "dobot_joint.launch.py")
    robot_state_publisher_launch = include_launch("cr5_moveit", "rsp.launch.py")
    move_group_launch = include_launch("cr5_moveit", "move_group.launch.py")
    rviz_launch = include_launch(
        "cr5_moveit",
        "moveit_rviz.launch.py",
        condition=IfCondition(start_rviz),
    )

    spray_gui_node = Node(
        package="my_cr5_control",
        executable="piston_spray_gui_node",
        output="screen",
        condition=IfCondition(start_gui),
    )

    return LaunchDescription([
        DeclareLaunchArgument("robot_ip", default_value=default_robot_ip),
        DeclareLaunchArgument("robot_type", default_value=default_robot_type),
        DeclareLaunchArgument("qt_platform", default_value=default_qt_platform),
        DeclareLaunchArgument("start_rviz", default_value="true"),
        DeclareLaunchArgument("start_gui", default_value="true"),
        SetEnvironmentVariable("IP_address", robot_ip),
        SetEnvironmentVariable("DOBOT_TYPE", robot_type),
        SetEnvironmentVariable("QT_QPA_PLATFORM", qt_platform),
        bringup_node,
        joint_bridge_node,
        robot_state_publisher_launch,
        move_group_launch,
        rviz_launch,
        spray_gui_node,
    ])
