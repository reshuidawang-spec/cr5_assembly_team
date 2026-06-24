"""Planner benchmark launcher — runs planner comparison benchmarks (simple and V2 task sets) with configurable repeat counts."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import LaunchConfiguration
from launch_ros.actions import Node
from moveit_configs_utils import MoveItConfigsBuilder


def launch_setup(context, *_args, **_kwargs):
    """Launch setup."""
    benchmark = LaunchConfiguration("benchmark").perform(context)
    repeats = LaunchConfiguration("repeats").perform(context)
    start_delay = float(LaunchConfiguration("start_delay").perform(context))
    planners = LaunchConfiguration("planners").perform(context).strip()
    scenes = LaunchConfiguration("scenes").perform(context).strip()

    executable_map = {
        "simple": "planner_comparison_simple_node",
        "v2": "planner_comparison_v2_node",
    }
    if benchmark not in executable_map:
        raise RuntimeError(f"Unsupported benchmark: {benchmark}")

    moveit_config = (
        MoveItConfigsBuilder("cr5_robot", package_name="cr5_moveit")
        .planning_pipelines(default_planning_pipeline="ompl", pipelines=["ompl"], load_all=False)
        .to_moveit_configs()
    )

    demo_launch = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            str(Path(get_package_share_directory("cr5_moveit")) / "launch" / "demo.launch.py")
        )
    )

    additional_env = {
        "MY_CR5_CONTROL_SIMPLE_REPEATS": repeats,
        "MY_CR5_CONTROL_V2_REPEATS": repeats,
    }
    if planners:
        additional_env["MY_CR5_CONTROL_BENCHMARK_PLANNERS"] = planners
    if scenes:
        additional_env["MY_CR5_CONTROL_BENCHMARK_SCENES"] = scenes

    benchmark_node = Node(
        package="my_cr5_control",
        executable=executable_map[benchmark],
        name=executable_map[benchmark],
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
        additional_env=additional_env,
    )

    return [
        demo_launch,
        TimerAction(period=start_delay, actions=[benchmark_node]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=benchmark_node,
                on_exit=[EmitEvent(event=Shutdown(reason="planner benchmark completed"))],
            )
        ),
    ]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument(
                "benchmark",
                default_value="simple",
                description="Which benchmark to run: simple or v2",
            ),
            DeclareLaunchArgument(
                "repeats",
                default_value="1",
                description="Repeat count for the selected benchmark node.",
            ),
            DeclareLaunchArgument(
                "start_delay",
                default_value="5.0",
                description="Delay in seconds before starting the benchmark node.",
            ),
            DeclareLaunchArgument(
                "planners",
                default_value="",
                description="Optional comma-separated planner list passed via MY_CR5_CONTROL_BENCHMARK_PLANNERS.",
            ),
            DeclareLaunchArgument(
                "scenes",
                default_value="",
                description="Optional comma-separated scene list passed via MY_CR5_CONTROL_BENCHMARK_SCENES.",
            ),
            OpaqueFunction(function=launch_setup),
        ]
    )
