"""CR5 planner launcher — generic launch file to start the CR5 motion planning pipeline with configurable planner and scene parameters."""
import os
from ament_index_python.packages import get_package_share_directory
from launch import LaunchDescription
from launch_ros.actions import Node
import xacro
import yaml

def load_yaml(package_name, file_path):
    """Load yaml."""
    package_path = get_package_share_directory(package_name)
    absolute_file_path = os.path.join(package_path, file_path)
    try:
        with open(absolute_file_path, 'r') as file:
            return yaml.safe_load(file)
    except EnvironmentError:
        return None

def generate_launch_description():
    """Generate launch description."""
    moveit_config_pkg = "cr5_moveit" 
    my_package_name = "my_cr5_control"

    xacro_file = os.path.join(
        get_package_share_directory(moveit_config_pkg),
        "config",
        "cr5_robot.urdf.xacro" 
    )
    

    doc = xacro.process_file(xacro_file)
    robot_description_config = doc.toxml()
    
    robot_description = {"robot_description": robot_description_config}

    # 读取 SRDF 文件内容
    srdf_file = os.path.join(
        get_package_share_directory(moveit_config_pkg),
        "config",
        "cr5_robot.srdf" 
    )
    
    with open(srdf_file, 'r') as f:
        robot_description_semantic_config = f.read()

    robot_description_semantic = {"robot_description_semantic": robot_description_semantic_config}


    #  加载 Kinematics 
    kinematics_yaml = load_yaml(moveit_config_pkg, "config/kinematics.yaml")


    # 5. 启动节点
    print(" 机器人描述文件加载成功，准备启动规划节点...")

    run_move_group_node = Node(
        package=my_package_name,
        executable="cr5_planner_node",
        output="screen",
        parameters=[
            robot_description,
            robot_description_semantic,
            kinematics_yaml,
            {"use_sim_time": False} # 真实机械臂设为 False
        ]
    )

    return LaunchDescription([
        run_move_group_node
    ])
