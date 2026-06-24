"""Synthetic guidance demo launcher — runs the guide model on synthetically-generated planning problems for controlled evaluation."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, IncludeLaunchDescription, OpaqueFunction, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node


def launch_setup(context, *_args, **_kwargs):
    """Launch setup."""
    my_share = Path(get_package_share_directory("my_cr5_control"))
    cr5_share = Path(get_package_share_directory("cr5_moveit"))

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(str(cr5_share / "launch" / "demo.launch.py")),
        launch_arguments={
            "use_rviz": LaunchConfiguration("use_rviz").perform(context),
            "rviz_config": str(my_share / "config" / "rviz" / "synthetic_guidance_demo.rviz"),
        }.items(),
    )

    synthetic_demo_node = Node(
        package="my_cr5_control",
        executable="synthetic_guidance_demo_node",
        name="synthetic_guidance_demo_node",
        output="screen",
    )

    return [
        demo_launch,
        TimerAction(
            period=float(LaunchConfiguration("start_delay").perform(context)),
            actions=[synthetic_demo_node],
        ),
    ]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_delay", default_value="5.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
