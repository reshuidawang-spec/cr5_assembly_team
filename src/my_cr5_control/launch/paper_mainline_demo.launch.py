"""Legacy compatibility wrapper.

This launch now starts the synthetic guidance demo that used to be called
"paper_mainline_demo". It is not the canonical paper mainline.
"""

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
    return [
        IncludeLaunchDescription(
            PythonLaunchDescriptionSource(str(my_share / "launch" / "synthetic_guidance_demo.launch.py")),
            launch_arguments={
                "use_rviz": LaunchConfiguration("use_rviz").perform(context),
                "start_delay": LaunchConfiguration("start_delay").perform(context),
            }.items(),
        )
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
