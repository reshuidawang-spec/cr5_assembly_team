"""CR5 MoveIt stable demo launch file — starts MoveIt with the CR5 robot and automatically load/activates the trajectory controller after startup."""
from launch import LaunchDescription
from launch.actions import ExecuteProcess, IncludeLaunchDescription, TimerAction
from launch.launch_description_sources import PythonLaunchDescriptionSource
from launch.substitutions import PathJoinSubstitution
from launch_ros.substitutions import FindPackageShare


def generate_launch_description():
    """这里在 MoveIt 启动后显式补一次 load + activate，避免每次手工执行 ros2 control 命令。."""
    cr5_moveit_demo = IncludeLaunchDescription(
        PythonLaunchDescriptionSource(
            PathJoinSubstitution(
                [FindPackageShare("cr5_moveit"), "launch", "demo.launch.py"]
            )
        )
    )

    # 当前 demo 环境里 joint_state_broadcaster 能起来，但轨迹控制器经常没有自动进入 active。
    # 这里在 MoveIt 启动后显式补一次 load + activate，避免每次手工执行 ros2 control 命令。
    load_cr5_group_controller = TimerAction(
        period=5.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "run",
                    "controller_manager",
                    "spawner",
                    "cr5_group_controller",
                    "--inactive",
                ],
                output="screen",
            )
        ],
    )

    activate_cr5_group_controller = TimerAction(
        period=7.0,
        actions=[
            ExecuteProcess(
                cmd=[
                    "ros2",
                    "control",
                    "switch_controllers",
                    "--activate",
                    "cr5_group_controller",
                ],
                output="screen",
            )
        ],
    )

    return LaunchDescription(
        [
            cr5_moveit_demo,
            load_cr5_group_controller,
            activate_cr5_group_controller,
        ]
    )
