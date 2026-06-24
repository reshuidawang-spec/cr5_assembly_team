"""Experimental task-space RRT* demo — launches the CR5 planner with task-space sampling for benchmarking RRT*-based planning algorithms."""
from launch import LaunchDescription
from launch.actions import IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.actions import Node
from launch_ros.substitutions import FindPackageShare
from moveit_configs_utils import MoveItConfigsBuilder


def generate_launch_description():
    """Generate launch description."""
    moveit_config = (
        MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )

    stable_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("my_cr5_control"), "launch", "cr5_moveit_stable_demo.launch.py"]
            )
        )
    )

    experimental_node = TimerAction(
        period=9.0,
        actions=[
            Node(
                package="my_cr5_control",
                executable="cr5_experimental_task_space_rrtstar_demo_node",
                output="screen",
                parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
            )
        ],
    )

    return LaunchDescription(
        [
            stable_demo,
            experimental_node,
        ]
    )
