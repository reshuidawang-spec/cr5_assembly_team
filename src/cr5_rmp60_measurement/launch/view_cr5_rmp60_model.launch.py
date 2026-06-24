from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument
from launch.substitutions import Command, LaunchConfiguration
from launch_ros.actions import Node


PROJECT_DIR = "/home/zhu/dobot_ws/src/cr5_rmp60_measurement"


def generate_launch_description():
    xacro_file = f"{PROJECT_DIR}/urdf/cr5_with_rmp60.urdf.xacro"
    rviz_config = f"{PROJECT_DIR}/config/rviz/cr5_rmp60_model.rviz"

    declared_arguments = [
        DeclareLaunchArgument("adapter_length", default_value="0.0494"),
        DeclareLaunchArgument("probe_body_length", default_value="0.076"),
        DeclareLaunchArgument("probe_body_radius", default_value="0.0315"),
        DeclareLaunchArgument("stylus_length", default_value="0.083870898"),
        DeclareLaunchArgument("stylus_radius", default_value="0.0015"),
        DeclareLaunchArgument("stylus_ball_radius", default_value="0.001"),
        DeclareLaunchArgument("tool_axis_sign", default_value="1"),
        DeclareLaunchArgument("mount_xyz", default_value="0 0 0"),
        DeclareLaunchArgument("mount_rpy", default_value="0 0 0"),
    ]

    robot_description = {
        "robot_description": Command(
            [
                "xacro ",
                xacro_file,
                " adapter_length:=",
                LaunchConfiguration("adapter_length"),
                " probe_body_length:=",
                LaunchConfiguration("probe_body_length"),
                " probe_body_radius:=",
                LaunchConfiguration("probe_body_radius"),
                " stylus_length:=",
                LaunchConfiguration("stylus_length"),
                " stylus_radius:=",
                LaunchConfiguration("stylus_radius"),
                " stylus_ball_radius:=",
                LaunchConfiguration("stylus_ball_radius"),
                " tool_axis_sign:=",
                LaunchConfiguration("tool_axis_sign"),
                " mount_xyz:='",
                LaunchConfiguration("mount_xyz"),
                "'",
                " mount_rpy:='",
                LaunchConfiguration("mount_rpy"),
                "'",
            ]
        )
    }

    return LaunchDescription(
        declared_arguments
        + [
            Node(
                package="robot_state_publisher",
                executable="robot_state_publisher",
                parameters=[robot_description],
                output="screen",
            ),
            Node(
                package="joint_state_publisher",
                executable="joint_state_publisher",
                output="screen",
            ),
            Node(
                package="rviz2",
                executable="rviz2",
                arguments=["-d", rviz_config],
                output="screen",
            ),
        ]
    )
