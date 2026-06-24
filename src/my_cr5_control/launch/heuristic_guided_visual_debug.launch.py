"""Heuristic-guided visual debug launcher — launches the planning pipeline with RViz visualization of heuristic gate activity for debugging guide models."""
from pathlib import Path

from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch.actions import DeclareLaunchArgument, EmitEvent, IncludeLaunchDescription, OpaqueFunction, RegisterEventHandler, TimerAction
from launch.event_handlers import OnProcessExit
from launch.events import Shutdown
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
            "rviz_config": str(my_share / "config" / "rviz" / "heuristic_guided_visual_debug.rviz"),
        }.items(),
    )

    debug_node = Node(
        package="my_cr5_control",
        executable="heuristic_guided_visual_debug_node",
        name="heuristic_guided_visual_debug_node",
        output="screen",
        additional_env={
            "MY_CR5_CONTROL_DEBUG_BENCHMARK": LaunchConfiguration("benchmark").perform(context),
            "MY_CR5_CONTROL_DEBUG_SCENE": LaunchConfiguration("scene").perform(context),
            "MY_CR5_CONTROL_DEBUG_BASE_PLANNER": LaunchConfiguration("base_planner").perform(context),
            "MY_CR5_CONTROL_DEBUG_PLANNING_BUDGET_S": LaunchConfiguration("budget_s").perform(context),
            "MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS": LaunchConfiguration("slow_direct_threshold_ms").perform(context),
            "MY_CR5_CONTROL_DEBUG_SAMPLE_COUNT": LaunchConfiguration("sample_count").perform(context),
            "MY_CR5_CONTROL_DEBUG_TOP_GUIDES": LaunchConfiguration("top_guides").perform(context),
            "MY_CR5_CONTROL_DEBUG_HOLD_S": LaunchConfiguration("hold_s").perform(context),
            "MY_CR5_CONTROL_DEBUG_ADAPTIVE_ELLIPSOID": LaunchConfiguration("adaptive_ellipsoid").perform(context),
            "MY_CR5_CONTROL_DEBUG_GUIDE_SEED": LaunchConfiguration("guide_seed").perform(context),
            "MY_CR5_CONTROL_DEBUG_EXECUTE": LaunchConfiguration("execute_motion").perform(context),
            "MY_CR5_CONTROL_V2_MESH_PROFILE": LaunchConfiguration("mesh_profile").perform(context),
            "MY_CR5_CONTROL_DEBUG_KEEP_ALIVE": LaunchConfiguration("keep_alive").perform(context),
            "MY_CR5_CONTROL_DEBUG_OUTPUT_DIR": LaunchConfiguration("output_dir").perform(context),
        },
    )

    actions = [
        demo_launch,
        TimerAction(
            period=float(LaunchConfiguration("start_delay").perform(context)),
            actions=[debug_node],
        ),
    ]
    if LaunchConfiguration("shutdown_on_exit").perform(context).lower() in ("1", "true", "yes", "on"):
        actions.append(
            RegisterEventHandler(
                OnProcessExit(
                    target_action=debug_node,
                    on_exit=[EmitEvent(event=Shutdown(reason="visual heuristic debug completed"))],
                )
            )
        )
    return actions


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("benchmark", default_value="simple"),
            DeclareLaunchArgument("scene", default_value="Medium_SideSurface"),
            DeclareLaunchArgument("base_planner", default_value="RRTConnect"),
            DeclareLaunchArgument("budget_s", default_value="10.0"),
            DeclareLaunchArgument("slow_direct_threshold_ms", default_value="800"),
            DeclareLaunchArgument("sample_count", default_value="24"),
            DeclareLaunchArgument("top_guides", default_value="10"),
            DeclareLaunchArgument("hold_s", default_value="12"),
            DeclareLaunchArgument("adaptive_ellipsoid", default_value="true"),
            DeclareLaunchArgument("guide_seed", default_value=""),
            DeclareLaunchArgument("execute_motion", default_value="false"),
            DeclareLaunchArgument("mesh_profile", default_value="ws119"),
            DeclareLaunchArgument("keep_alive", default_value="true"),
            DeclareLaunchArgument("output_dir", default_value=""),
            DeclareLaunchArgument("shutdown_on_exit", default_value="false"),
            DeclareLaunchArgument("use_rviz", default_value="true"),
            DeclareLaunchArgument("start_delay", default_value="5.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
