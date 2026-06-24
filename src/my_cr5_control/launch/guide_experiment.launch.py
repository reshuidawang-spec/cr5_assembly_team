"""Guide-model experiment launcher — starts the full planning pipeline with a learned guide model for planner selection and heuristic injection."""
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


def resolve_optional_path(raw_value: str) -> str:
    """Resolve optional path."""
    value = raw_value.strip()
    if not value:
        return value

    candidate = Path(value).expanduser()
    if candidate.is_absolute():
        return str(candidate)

    cwd_candidate = (Path.cwd() / candidate).resolve()
    if cwd_candidate.exists():
        return str(cwd_candidate)

    package_root_candidate = (Path(__file__).resolve().parents[1] / candidate).resolve()
    if package_root_candidate.exists():
        return str(package_root_candidate)

    for parent in Path(__file__).resolve().parents:
        workspace_candidate = (parent / "src" / "my_cr5_control" / candidate).resolve()
        if workspace_candidate.exists():
            return str(workspace_candidate)

    return value


def launch_setup(context, *_args, **_kwargs):
    """Launch setup."""
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
        "MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE": LaunchConfiguration("mode").perform(context),
        "MY_CR5_CONTROL_GUIDE_BASE_PLANNER": LaunchConfiguration("base_planner").perform(context),
        "MY_CR5_CONTROL_GUIDE_REPEATS": LaunchConfiguration("repeats").perform(context),
        "MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT": LaunchConfiguration("sample_count").perform(context),
        "MY_CR5_CONTROL_GUIDE_BUDGET_S": LaunchConfiguration("budget_s").perform(context),
        "MY_CR5_CONTROL_GUIDE_SCENES": LaunchConfiguration("scenes").perform(context),
        "MY_CR5_CONTROL_GUIDE_SAMPLE_SEED": LaunchConfiguration("sample_seed").perform(context),
        "MY_CR5_CONTROL_GUIDE_MODEL_PATH": resolve_optional_path(
            LaunchConfiguration("model_path").perform(context)
        ),
        "MY_CR5_CONTROL_GUIDE_MODEL_TOP_K": LaunchConfiguration("top_k").perform(context),
        "MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD": LaunchConfiguration("threshold").perform(context),
        "MY_CR5_CONTROL_GUIDE_SELECTION_MODE": LaunchConfiguration("selection_mode").perform(context),
        "MY_CR5_CONTROL_GUIDE_HYBRID_ALPHA": LaunchConfiguration("hybrid_alpha").perform(context),
        "MY_CR5_CONTROL_GUIDE_RETAINED_ORDER": LaunchConfiguration("retained_order").perform(context),
        "MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE": LaunchConfiguration("direct_gate_mode").perform(context),
        "MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE": LaunchConfiguration("reuse_direct_baseline").perform(context),
    }

    guide_node = Node(
        package="my_cr5_control",
        executable="guide_ranking_simple_experiment_node",
        name="guide_ranking_simple_experiment_node",
        output="screen",
        parameters=[moveit_config.to_dict(), {"use_sim_time": False}],
        additional_env=additional_env,
    )

    return [
        demo_launch,
        TimerAction(period=float(LaunchConfiguration("start_delay").perform(context)), actions=[guide_node]),
        RegisterEventHandler(
            OnProcessExit(
                target_action=guide_node,
                on_exit=[EmitEvent(event=Shutdown(reason="guide experiment completed"))],
            )
        ),
    ]


def generate_launch_description():
    """Generate launch description."""
    return LaunchDescription(
        [
            DeclareLaunchArgument("mode", default_value="ablation"),
            DeclareLaunchArgument("base_planner", default_value="RRTConnect"),
            DeclareLaunchArgument("repeats", default_value="2"),
            DeclareLaunchArgument("sample_count", default_value="24"),
            DeclareLaunchArgument("budget_s", default_value="4.0"),
            DeclareLaunchArgument("scenes", default_value=""),
            DeclareLaunchArgument("sample_seed", default_value=""),
            DeclareLaunchArgument("model_path", default_value=""),
            DeclareLaunchArgument("top_k", default_value="1"),
            DeclareLaunchArgument("threshold", default_value="0.55"),
            DeclareLaunchArgument("selection_mode", default_value="top_prob"),
            DeclareLaunchArgument("hybrid_alpha", default_value="2.0"),
            DeclareLaunchArgument("retained_order", default_value="heuristic"),
            DeclareLaunchArgument("direct_gate_mode", default_value="off"),
            DeclareLaunchArgument("reuse_direct_baseline", default_value="1"),
            DeclareLaunchArgument("start_delay", default_value="5.0"),
            OpaqueFunction(function=launch_setup),
        ]
    )
