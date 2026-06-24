cmake_minimum_required(VERSION 3.8)
project(my_cr5_control)

# 编译器选项
if(CMAKE_COMPILER_IS_GNUCXX OR CMAKE_CXX_COMPILER_ID MATCHES "Clang")
  add_compile_options(-Wall -Wextra -Wpedantic)
endif()

# C++标准
set(CMAKE_CXX_STANDARD 17)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
add_compile_definitions(MY_CR5_CONTROL_SOURCE_DIR="${CMAKE_CURRENT_SOURCE_DIR}")

# 寻找依赖
find_package(ament_cmake REQUIRED)
find_package(ament_index_cpp REQUIRED)
find_package(rclcpp REQUIRED)
find_package(moveit_ros_planning_interface REQUIRED)
find_package(moveit_msgs REQUIRED)
find_package(geometry_msgs REQUIRED)
find_package(visualization_msgs REQUIRED)
find_package(interactive_markers REQUIRED)
find_package(geometric_shapes REQUIRED)
find_package(tf2 REQUIRED)
find_package(tf2_geometry_msgs REQUIRED)

# Qt5依赖
find_package(Qt5 COMPONENTS Core Widgets REQUIRED)
set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTOUIC ON)
set(CMAKE_AUTORCC ON)

include_directories(include)

function(add_my_cr5_executable target_name)
  cmake_parse_arguments(ARG "" "" "SOURCES;LIBRARIES;DEPENDENCIES" ${ARGN})
  add_executable(${target_name} ${ARG_SOURCES})
  if(ARG_LIBRARIES)
    target_link_libraries(${target_name} ${ARG_LIBRARIES})
  endif()
  if(ARG_DEPENDENCIES)
    ament_target_dependencies(${target_name} ${ARG_DEPENDENCIES})
  endif()
  set(MY_CR5_EXECUTABLE_TARGETS ${MY_CR5_EXECUTABLE_TARGETS} ${target_name} PARENT_SCOPE)
endfunction()

# 1. 编译核心库 (cr5_core_lib)
add_library(cr5_core_lib SHARED
  src/core/cr5_robot.cpp
  src/core/measurement_point_generator.cpp
)
ament_target_dependencies(cr5_core_lib
  ament_index_cpp
  rclcpp
  moveit_ros_planning_interface
  moveit_msgs
  geometry_msgs
  geometric_shapes
  tf2
  tf2_geometry_msgs
)

add_library(piston_spray_lib SHARED
  src/core/piston_spray_planner.cpp
  src/core/piston_spray_execution.cpp
)
ament_target_dependencies(piston_spray_lib
  ament_index_cpp
  rclcpp
  moveit_ros_planning_interface
  geometry_msgs
  tf2
  tf2_geometry_msgs
)
target_link_libraries(piston_spray_lib cr5_core_lib)

# 2. 编译业务节点 (cr5_planner)
add_my_cr5_executable(cr5_planner_node
  SOURCES src/operations/cr5_planner.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

# Legacy synthetic scene demo for RViz/debugging. This is not the canonical
# paper mainline; keep the old executable name only as a compatibility alias.
add_my_cr5_executable(synthetic_guidance_demo_node
  SOURCES src/tools/synthetic_guidance_demo.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

add_my_cr5_executable(cr5_paper_mainline_demo_node
  SOURCES src/tools/synthetic_guidance_demo.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

# 1. 库文件安装
install(TARGETS cr5_core_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)
install(TARGETS piston_spray_lib
  ARCHIVE DESTINATION lib
  LIBRARY DESTINATION lib
  RUNTIME DESTINATION bin
)

# 3. 编译箱体测试节点 (新功能)
add_my_cr5_executable(cr5_box_tester_node
  SOURCES src/operations/cr5_box_tester.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    interactive_markers
    tf2
    tf2_geometry_msgs
)

# 4. 编译Qt5 GUI节点
add_my_cr5_executable(cr5_gui_node
  SOURCES
    src/gui/cr5_gui_main.cpp
    src/gui/cr5_gui_window.cpp
    include/my_cr5_control/cr5_gui_window.hpp
  LIBRARIES
    cr5_core_lib
    Qt5::Core
    Qt5::Widgets
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

add_my_cr5_executable(piston_spray_gui_node
  SOURCES
    src/gui/piston_spray_gui_main.cpp
    src/gui/piston_spray_gui_window.cpp
    include/my_cr5_control/piston_spray_gui_window.hpp
  LIBRARIES
    piston_spray_lib
    Qt5::Core
    Qt5::Widgets
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

# 5. 编译规划器对比测试节点
add_my_cr5_executable(planner_comparison_node
  SOURCES src/benchmarks/planner_comparison.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

# 6. 编译规划器对比测试节点V2（基于STL特征提取）
add_my_cr5_executable(planner_comparison_v2_node
  SOURCES src/benchmarks/planner_comparison_v2.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

# 7. 编译规划器对比测试节点 - 简化版
add_my_cr5_executable(planner_comparison_simple_node
  SOURCES src/benchmarks/planner_comparison_simple.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

# 8. 编译 simple 随机任务数据采集节点
add_my_cr5_executable(random_task_dataset_simple_node
  SOURCES src/benchmarks/random_task_dataset_simple.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

# 9. 编译箱体特征提取检查节点
add_my_cr5_executable(inspect_box_features_node
  SOURCES src/tools/inspect_box_features.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    geometry_msgs
)

# 10. 编译箱体特征 RViz 可视化节点
add_my_cr5_executable(visualize_box_features_node
  SOURCES src/tools/visualize_box_features.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    geometry_msgs
    visualization_msgs
)

add_my_cr5_executable(inspect_v2_scenarios_node
  SOURCES src/tools/inspect_v2_scenarios.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    geometry_msgs
)

add_my_cr5_executable(heuristic_guided_visual_debug_node
  SOURCES src/tools/heuristic_guided_visual_debug.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

# 11. 编译论文 improved RRT*-Connect 复现 TEST 节点
add_my_cr5_executable(test_rrtstar_connect_reproduction_node
  SOURCES src/tools/test_rrtstar_connect_reproduction.cpp
  DEPENDENCIES rclcpp
)

# 11.1 编译机械臂 task-space RRT* 实验 TEST 节点
add_my_cr5_executable(cr5_experimental_task_space_rrtstar_demo_node
  SOURCES src/tools/cr5_experimental_task_space_rrtstar_demo.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
    visualization_msgs
    tf2
    tf2_geometry_msgs
)

# 12. 编译 simple guide ranking 数据采集 / ablation 节点
add_my_cr5_executable(guide_ranking_simple_experiment_node
  SOURCES src/benchmarks/guide_ranking_simple_experiment.cpp
  LIBRARIES cr5_core_lib
  DEPENDENCIES
    ament_index_cpp
    rclcpp
    moveit_ros_planning_interface
    geometry_msgs
)

# 3. 配置文件安装 (Launch, include)
install(DIRECTORY include/ DESTINATION include/)
install(DIRECTORY launch DESTINATION share/${PROJECT_NAME}
  PATTERN "__pycache__" EXCLUDE
)
install(DIRECTORY src/meshes DESTINATION share/${PROJECT_NAME})
install(DIRECTORY config DESTINATION share/${PROJECT_NAME})
install(PROGRAMS
  scripts/benchmarks/collect_benchmark_dataset.py
  scripts/benchmarks/analyze_heuristic_gate_activity.py
  scripts/benchmarks/analyze_heuristic_rescue_candidates.py
  scripts/benchmarks/export_benchmark_dataset.py
  scripts/benchmarks/plot_q2_paper_figures.py
  scripts/benchmarks/plot_simple_benchmark.py
  scripts/benchmarks/run_unified_formal_rerun.py
  scripts/benchmarks/test_bitstar.py
  scripts/datasets/analyze_simple_random_dataset.py
  scripts/datasets/plot_simple_random_dataset.py
  scripts/maintenance/build_dataset_manifest.py
  scripts/maintenance/generate_v2_hero_mesh.py
  scripts/maintenance/repair_simple_random_dataset_metrics.py
  scripts/models/prepare_training_table.py
  scripts/models/prepare_oracle_planner_dataset.py
  scripts/models/train_baseline_model.py
  scripts/models/train_guide_ranking_model.py
  scripts/models/train_oracle_planner_model.py
  scripts/models/evaluate_baseline_model.py
  scripts/models/evaluate_planner_selection.py
  scripts/models/evaluate_oracle_planner_model.py
  scripts/models/prepare_triggered_guide_dataset.py
  scripts/models/analyze_guide_ablation_trace.py
  scripts/operations/animate_rrtstar_connect.py
  DESTINATION lib/${PROJECT_NAME}
)
install(FILES
  scripts/models/guide_model_schema.py
  scripts/models/model_pipeline_common.py
  DESTINATION lib/${PROJECT_NAME}
)
install(TARGETS ${MY_CR5_EXECUTABLE_TARGETS}
  DESTINATION lib/${PROJECT_NAME}
)

ament_package()
