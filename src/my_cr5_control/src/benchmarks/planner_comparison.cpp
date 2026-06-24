#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/planner_selection_utils.hpp"
#include "my_cr5_control/result_utils.hpp"
#include <algorithm>
#include <cstdlib>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <string>
#include <vector>

// 测试场景配置
struct TestScenario {
    std::string name;
    geometry_msgs::msg::Pose target_pose;
    std::string difficulty; // "easy", "medium", "hard"
};

// 测试结果
struct PlannerResult {
    std::string planner_name;
    std::string planning_mode;
    std::string planner_id;
    std::string scenario_name;
    std::string difficulty;
    bool success;
    double planning_time_ms;
    double path_length;
    int num_waypoints;
};

enum class PlanningMode {
    OMPL,
    HeuristicGuided
};

struct PlannerConfig {
    std::string name;
    std::string planner_id;
    double planning_time_s;
    PlanningMode mode;
};

namespace {

std::vector<PlannerConfig> getLegacyPlannerConfigs(std::vector<std::string>* unknown_names = nullptr) {
    const std::vector<PlannerConfig> defaults = {
        {"RRTConnect", "RRTConnect", 10.0, PlanningMode::OMPL},
        {"RRTstar", "RRTstar", 10.0, PlanningMode::OMPL},
        {"LBTRRT", "LBTRRT", 10.0, PlanningMode::OMPL},
        {"FMT", "FMT", 10.0, PlanningMode::OMPL},
        {"BFMT", "BFMT", 10.0, PlanningMode::OMPL},
        {"PRMstar", "PRMstar", 10.0, PlanningMode::OMPL},
        {"HeuristicGuided", "custom_two_stage_guided", 10.0, PlanningMode::HeuristicGuided},
    };

    const std::vector<PlannerConfig> supported = {
        {"RRTConnect", "RRTConnect", 10.0, PlanningMode::OMPL},
        {"RRTstar", "RRTstar", 10.0, PlanningMode::OMPL},
        {"LBTRRT", "LBTRRT", 10.0, PlanningMode::OMPL},
        {"FMT", "FMT", 10.0, PlanningMode::OMPL},
        {"BFMT", "BFMT", 10.0, PlanningMode::OMPL},
        {"PRMstar", "PRMstar", 10.0, PlanningMode::OMPL},
        {"BITstar", "BITstar", 10.0, PlanningMode::OMPL},
        {"InformedRRTstar", "InformedRRTstar", 10.0, PlanningMode::OMPL},
        {"ABITstar", "ABITstar", 10.0, PlanningMode::OMPL},
        {"AITstar", "AITstar", 10.0, PlanningMode::OMPL},
        {"EITstar", "EITstar", 10.0, PlanningMode::OMPL},
        {"HeuristicGuided", "custom_two_stage_guided", 10.0, PlanningMode::HeuristicGuided},
    };

    return my_cr5_control::benchmarks::selectPlannerConfigs(
        defaults,
        supported,
        {"MY_CR5_CONTROL_BENCHMARK_PLANNERS"},
        unknown_names);
}

void logLegacyPlannerSelection(
    const rclcpp::Logger& logger,
    const std::vector<PlannerConfig>& planners,
    const std::vector<std::string>& unknown_names) {
    RCLCPP_INFO(
        logger,
        "启用 planner 集合: %s",
        my_cr5_control::benchmarks::joinPlannerNames(planners).c_str());
    if (!unknown_names.empty()) {
        RCLCPP_WARN(
            logger,
            "忽略未识别 planner 名称: %s",
            my_cr5_control::benchmarks::joinTokens(unknown_names).c_str());
    }
}

}  // namespace

// 生成测试场景
std::vector<TestScenario> generateTestScenarios() {
    std::vector<TestScenario> scenarios;

    // 简单场景：开放空间
    {
        TestScenario s;
        s.name = "Easy_OpenSpace";
        s.difficulty = "easy";
        s.target_pose.position.x = 0.4;
        s.target_pose.position.y = 0.2;
        s.target_pose.position.z = 0.5;
        s.target_pose.orientation.x = 0.0;
        s.target_pose.orientation.y = 1.0;
        s.target_pose.orientation.z = 0.0;
        s.target_pose.orientation.w = 0.0;
        scenarios.push_back(s);
    }

    // 中等场景：箱体外表面
    {
        TestScenario s;
        s.name = "Medium_BoxSurface";
        s.difficulty = "medium";
        s.target_pose.position.x = 0.5;
        s.target_pose.position.y = 0.0;
        s.target_pose.position.z = 0.3;
        s.target_pose.orientation.x = 0.0;
        s.target_pose.orientation.y = 1.0;
        s.target_pose.orientation.z = 0.0;
        s.target_pose.orientation.w = 0.0;
        scenarios.push_back(s);
    }

    // 困难场景：箱体内腔
    {
        TestScenario s;
        s.name = "Hard_BoxInterior";
        s.difficulty = "hard";
        s.target_pose.position.x = 0.5;
        s.target_pose.position.y = 0.0;
        s.target_pose.position.z = 0.15;
        s.target_pose.orientation.x = 0.0;
        s.target_pose.orientation.y = 1.0;
        s.target_pose.orientation.z = 0.0;
        s.target_pose.orientation.w = 0.0;
        scenarios.push_back(s);
    }

    return scenarios;
}

// 计算路径长度
[[maybe_unused]] double calculatePathLength(const moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    double length = 0.0;
    const auto& trajectory = plan.trajectory_.joint_trajectory;

    if (trajectory.points.size() < 2) return 0.0;

    for (size_t i = 1; i < trajectory.points.size(); ++i) {
        double segment_length = 0.0;
        for (size_t j = 0; j < trajectory.points[i].positions.size(); ++j) {
            double diff = trajectory.points[i].positions[j] - trajectory.points[i-1].positions[j];
            segment_length += diff * diff;
        }
        length += std::sqrt(segment_length);
    }
    return length;
}

// 测试单个规划器
PlannerResult testPlanner(CR5Robot& robot, const PlannerConfig& planner,
                          const TestScenario& scenario) {
    PlannerResult result;
    result.planner_name = planner.name;
    result.planning_mode = (planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided";
    result.planner_id = planner.planner_id;
    result.scenario_name = scenario.name;
    result.difficulty = scenario.difficulty;

    auto start_time = std::chrono::high_resolution_clock::now();

    bool success = false;
    if (planner.mode == PlanningMode::OMPL) {
        success = robot.moveToPoseWithPlanner(
            scenario.target_pose, planner.planner_id, planner.planning_time_s);
    } else {
        success = robot.moveToPoseImproved(scenario.target_pose);
    }

    auto end_time = std::chrono::high_resolution_clock::now();
    auto duration = std::chrono::duration_cast<std::chrono::milliseconds>(end_time - start_time);

    result.success = success;
    result.planning_time_ms = duration.count();
    result.path_length = 0.0; // 简化版本，实际需要从plan中提取
    result.num_waypoints = 0;

    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("planner_comparison");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "规划器对比实验");
    RCLCPP_INFO(logger, "========================================");

    // 初始化机器人
    CR5Robot robot("planner_comparison_node");
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        return 1;
    }

    // 添加测试环境
    robot.addSimulationEnvironment();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 生成测试场景
    auto scenarios = generateTestScenarios();
    std::vector<std::string> unknown_planner_names;
    const std::vector<PlannerConfig> planners = getLegacyPlannerConfigs(&unknown_planner_names);
    logLegacyPlannerSelection(logger, planners, unknown_planner_names);

    const std::string timestamp = my_cr5_control::results::makeTimestamp();
    const std::string results_path =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_results.csv");
    const std::string summary_path =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_summary.csv");

    std::ofstream csv_file(results_path);
    csv_file << "规划器,模式,规划器ID,场景名称,难度,目标X,目标Y,目标Z,成功,规划时间(ms)\n";
    std::vector<PlannerResult> all_results;

    // 运行测试
    for (const auto& scenario : scenarios) {
        RCLCPP_INFO(logger, "\n>>> 测试场景: %s (%s)",
                    scenario.name.c_str(), scenario.difficulty.c_str());

        for (const auto& planner : planners) {
            RCLCPP_INFO(logger, "  测试规划器: %s", planner.name.c_str());

            // 返回初始位置
            robot.moveToNamedTarget("home");
            std::this_thread::sleep_for(std::chrono::seconds(1));

            // 运行测试
            auto result = testPlanner(robot, planner, scenario);
            all_results.push_back(result);

            // 记录结果
            csv_file << result.planner_name << ","
                     << result.planning_mode << ","
                     << result.planner_id << ","
                     << result.scenario_name << ","
                     << scenario.difficulty << ","
                     << std::fixed << std::setprecision(4)
                     << scenario.target_pose.position.x << ","
                     << scenario.target_pose.position.y << ","
                     << scenario.target_pose.position.z << ","
                     << (result.success ? "成功" : "失败") << ","
                     << std::fixed << std::setprecision(1)
                     << result.planning_time_ms << "\n";

            RCLCPP_INFO(logger, "    结果: %s, 时间: %.2f ms",
                        result.success ? "成功" : "失败",
                        result.planning_time_ms);

            std::this_thread::sleep_for(std::chrono::milliseconds(500));
        }
    }

    csv_file.close();

    std::ofstream summary_file(summary_path);
    summary_file << "规划器,模式,规划器ID,成功率(%),平均时间(ms)\n";

    for (const auto& planner : planners) {
        int success_count = 0;
        int count = 0;
        double total_time = 0.0;

        for (const auto& result : all_results) {
            if (result.planner_name != planner.name || result.planning_mode != ((planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided")) {
                continue;
            }
            if (result.success) {
                success_count++;
            }
            total_time += result.planning_time_ms;
            count++;
        }

        const double success_rate = count > 0 ? (100.0 * success_count / count) : 0.0;
        const double avg_time = count > 0 ? (total_time / count) : 0.0;

        summary_file << planner.name << ","
                     << ((planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided") << ","
                     << planner.planner_id << ","
                     << std::fixed << std::setprecision(1) << success_rate << ","
                     << avg_time << "\n";
    }
    summary_file.close();

    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "测试完成，详细结果已保存到 %s", results_path.c_str());
    RCLCPP_INFO(logger, "统计摘要已保存到 %s", summary_path.c_str());
    RCLCPP_INFO(logger, "========================================");

    rclcpp::shutdown();
    return 0;
}
