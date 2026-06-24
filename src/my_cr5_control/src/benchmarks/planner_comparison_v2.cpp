#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"
#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/paper_mainline/v2_measurement_pose.hpp"
#include "my_cr5_control/paper_mainline/v2_scenario_profile.hpp"
#include "my_cr5_control/planner_selection_utils.hpp"
#include "my_cr5_control/result_utils.hpp"
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <iterator>
#include <cmath>
#include <string>
#include <unistd.h>
#include <vector>

using namespace measurement;

namespace {

inline constexpr char kV2BenchmarkVersion[] = "v2_v20260311_reachable_scenes_metrics";
inline constexpr double kFastSolveThresholdMs = 1000.0;

int getV2RepeatCount() {
    return my_cr5_control::env::getIntClamped("MY_CR5_CONTROL_V2_REPEATS", 1, 1, 1000);
}

void logAdaptiveEllipsoidConfig(
    const rclcpp::Logger& logger,
    const my_cr5_control::benchmarks::AdaptiveEllipsoidEnvConfig& config) {
    if (!config.enabled) {
        return;
    }

    if (config.fixed_difficulty.has_value()) {
        RCLCPP_INFO(
            logger,
            "自适应椭球采样已启用，使用固定难度评分: %.2f",
            *config.fixed_difficulty);
        return;
    }

    RCLCPP_INFO(logger, "自适应椭球采样已启用，默认复用场景 difficulty_score");
}

void applyAdaptiveEllipsoidForDifficulty(
    CR5Robot& robot,
    const my_cr5_control::benchmarks::AdaptiveEllipsoidEnvConfig& config,
    double difficulty_score) {
    if (!config.enabled) {
        return;
    }

    robot.setSceneDifficultyScore(
        my_cr5_control::benchmarks::resolveAdaptiveEllipsoidDifficulty(config, difficulty_score));
}

void writeDetailedHeader(std::ostream& os) {
    os << "基准版本,实验时间戳,总重复次数,重复序号,"
       << "规划器,模式,规划器ID,场景名称,难度,测点描述,难度评分,"
       << "尖端X,尖端Y,尖端Z,法兰X,法兰Y,法兰Z,成功,"
       << "墙钟时间(ms),MoveIt规划时间(ms),预算上限(ms),规划调用次数,触发预算上限,快速求解(<1s),"
       << "guide候选数,guide尝试数,使用direct回退,direct规划成功,direct尝试时间(ms),direct路径代价,"
       << "top guide ID,top guide heuristic_cost,top guide ranking_score,top guide cost_delta_to_direct,"
       << "top guide clearance,top guide manipulability,top guide axial_progress,top guide lateral_offset\n";
}

void writeSummaryHeader(std::ostream& os) {
    os << "基准版本,实验时间戳,总重复次数,"
       << "规划器,模式,规划器ID,成功率(%),成功样本数,总样本数,"
       << "平均时间(ms),中位时间(ms),P25(ms),P75(ms),"
       << "预算命中数,预算命中率(%),快速求解数,快速求解率(%),平均MoveIt规划时间(ms),平均规划调用次数,"
       << "平均guide候选数,平均guide尝试数,direct回退率(%),"
       << "简单场景(ms),中等场景(ms),困难场景(ms),极端场景(ms),详细结果文件\n";
}

double computeQuantile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }

    std::sort(values.begin(), values.end());
    const double clamped_q = std::clamp(q, 0.0, 1.0);
    const double pos = clamped_q * static_cast<double>(values.size() - 1);
    const auto lower = static_cast<std::size_t>(std::floor(pos));
    const auto upper = static_cast<std::size_t>(std::ceil(pos));
    if (lower == upper) {
        return values[lower];
    }
    const double weight = pos - static_cast<double>(lower);
    return values[lower] * (1.0 - weight) + values[upper] * weight;
}

struct DifficultyTiming {
    double total_time_ms{0.0};
    int count{0};
};

std::size_t findV2DifficultyIndex(const std::string& difficulty) {
    const auto& order = my_cr5_control::paper_mainline::canonicalV2DifficultyOrder();
    const auto it = std::find(order.begin(), order.end(), difficulty);
    if (it == order.end()) {
        return order.size();
    }
    return static_cast<std::size_t>(std::distance(order.begin(), it));
}

double averageDifficultyTime(const std::vector<DifficultyTiming>& timings, std::size_t index) {
    if (index >= timings.size() || timings[index].count == 0) {
        return 0.0;
    }
    return timings[index].total_time_ms / static_cast<double>(timings[index].count);
}

}  // namespace

// 测试结果
struct PlannerResult {
    int repeat_index;
    std::string planner_name;
    std::string planning_mode;
    std::string planner_id;
    std::string scenario_name;
    std::string difficulty;
    std::string point_description;
    bool success;
    double wall_time_ms;
    double planner_reported_time_ms;
    double planning_budget_ms;
    int planner_calls;
    bool hit_budget_limit;
    int guide_candidate_count;
    int guide_candidates_attempted;
    bool used_direct_plan;
    bool direct_plan_success;
    double direct_attempt_wall_time_ms;
    double direct_path_cost;
    int top_ranked_candidate_id;
    double top_ranked_candidate_heuristic_cost;
    double top_ranked_candidate_ranking_score;
    double top_ranked_candidate_cost_delta_to_direct;
    double top_ranked_candidate_clearance_margin;
    double top_ranked_candidate_manipulability_score;
    double top_ranked_candidate_axial_progress;
    double top_ranked_candidate_lateral_offset;
    double difficulty_score;
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Pose flange_pose;
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

std::vector<PlannerConfig> getV2PlannerConfigs(std::vector<std::string>* unknown_names = nullptr) {
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
        {"MY_CR5_CONTROL_V2_PLANNERS", "MY_CR5_CONTROL_BENCHMARK_PLANNERS"},
        unknown_names);
}

void logV2PlannerSelection(
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

std::vector<MeasurementPointGenerator::TestScenario> filterV2Scenes(
    const std::vector<MeasurementPointGenerator::TestScenario>& scenarios,
    std::vector<std::string>* unknown_names = nullptr) {
    const char* raw = my_cr5_control::benchmarks::getFirstPlannerEnvValue(
        {"MY_CR5_CONTROL_V2_SCENES", "MY_CR5_CONTROL_BENCHMARK_SCENES"});
    if (raw == nullptr) {
        return scenarios;
    }

    const std::vector<std::string> selected_names =
        my_cr5_control::benchmarks::splitCommaTokens(raw);
    std::vector<MeasurementPointGenerator::TestScenario> filtered;
    filtered.reserve(selected_names.size());

    for (const auto& name : selected_names) {
        const auto it = std::find_if(
            scenarios.begin(), scenarios.end(),
            [&](const MeasurementPointGenerator::TestScenario& scenario) {
                return scenario.name == name;
            });
        if (it == scenarios.end()) {
            if (unknown_names != nullptr &&
                std::find(unknown_names->begin(), unknown_names->end(), name) == unknown_names->end()) {
                unknown_names->push_back(name);
            }
            continue;
        }

        const auto duplicate = std::find_if(
            filtered.begin(), filtered.end(),
            [&](const MeasurementPointGenerator::TestScenario& scenario) {
                return scenario.name == it->name;
            });
        if (duplicate == filtered.end()) {
            filtered.push_back(*it);
        }
    }

    return filtered;
}

void logV2SceneSelection(const rclcpp::Logger& logger,
                         const std::vector<MeasurementPointGenerator::TestScenario>& scenarios,
                         const std::vector<std::string>& unknown_names) {
    std::vector<std::string> scene_names;
    scene_names.reserve(scenarios.size());
    for (const auto& scenario : scenarios) {
        scene_names.push_back(scenario.name);
    }
    RCLCPP_INFO(logger, "启用场景子集: %s",
                my_cr5_control::benchmarks::joinTokens(scene_names).c_str());
    if (!unknown_names.empty()) {
        RCLCPP_WARN(logger, "忽略未识别场景名称: %s",
                    my_cr5_control::benchmarks::joinTokens(unknown_names).c_str());
    }
}

}  // namespace

// 测试单个规划器
PlannerResult testPlanner(
    CR5Robot& robot,
    const PlannerConfig& planner,
    const MeasurementPointGenerator::TestScenario& scenario,
    int repeat_index,
    const my_cr5_control::benchmarks::AdaptiveEllipsoidEnvConfig& adaptive_ellipsoid_config) {
    PlannerResult result;
    result.repeat_index = repeat_index;
    result.planner_name = planner.name;
    result.planning_mode = (planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided";
    result.planner_id = planner.planner_id;
    result.scenario_name = scenario.name;
    result.difficulty = scenario.difficulty;

    if (scenario.points.empty()) {
        result.success = false;
        result.wall_time_ms = 0.0;
        result.planner_reported_time_ms = 0.0;
        result.planning_budget_ms = planner.planning_time_s * 1000.0;
        result.planner_calls = 0;
        result.hit_budget_limit = false;
        result.difficulty_score = 0.0;
        result.point_description = "无测点";
        return result;
    }

    const auto& point = scenario.points[0];
    result.point_description = point.description;
    result.difficulty_score = point.difficulty_score;
    result.tip_point = point.position;

    geometry_msgs::msg::Pose target_pose =
        my_cr5_control::paper_mainline::buildV2MeasurementFlangePose(point);
    result.flange_pose = target_pose;
    result.wall_time_ms = 0.0;
    result.planner_reported_time_ms = 0.0;
    result.planning_budget_ms = planner.planning_time_s * 1000.0;
    result.planner_calls = 0;
    result.hit_budget_limit = false;
    result.guide_candidate_count = 0;
    result.guide_candidates_attempted = 0;
    result.used_direct_plan = false;
    result.direct_plan_success = false;
    result.direct_attempt_wall_time_ms = 0.0;
    result.direct_path_cost = -1.0;
    result.top_ranked_candidate_id = -1;
    result.top_ranked_candidate_heuristic_cost = -1.0;
    result.top_ranked_candidate_ranking_score = -1.0;
    result.top_ranked_candidate_cost_delta_to_direct = -1.0;
    result.top_ranked_candidate_clearance_margin = -1.0;
    result.top_ranked_candidate_manipulability_score = -1.0;
    result.top_ranked_candidate_axial_progress = -1.0;
    result.top_ranked_candidate_lateral_offset = -1.0;

    CR5Robot::PlanningMetrics metrics;
    if (planner.mode == PlanningMode::OMPL) {
        result.success = robot.planToPoseWithPlanner(
            target_pose, planner.planner_id, planner.planning_time_s, "home", &metrics);
    } else {
        // HeuristicGuided 当前对应外部两阶段采样引导原型，不走 OMPL planner_id 分发。
        applyAdaptiveEllipsoidForDifficulty(
            robot, adaptive_ellipsoid_config, point.difficulty_score);
        result.success = robot.planToPoseImproved(
            target_pose, "home", planner.planning_time_s, &metrics);
    }

    result.wall_time_ms = metrics.wall_time_ms;
    result.planner_reported_time_ms = metrics.planner_reported_time_ms;
    result.planning_budget_ms = metrics.planning_budget_ms;
    result.planner_calls = metrics.planner_calls;
    result.hit_budget_limit = metrics.hit_budget_limit;
    result.guide_candidate_count = metrics.guide_candidate_count;
    result.guide_candidates_attempted = metrics.guide_candidates_attempted;
    result.used_direct_plan = metrics.used_direct_plan;
    result.direct_plan_success = metrics.direct_plan_success;
    result.direct_attempt_wall_time_ms = metrics.direct_attempt_wall_time_ms;
    result.direct_path_cost = metrics.direct_path_cost;
    result.top_ranked_candidate_id = metrics.top_ranked_candidate_id;
    result.top_ranked_candidate_heuristic_cost = metrics.top_ranked_candidate_heuristic_cost;
    result.top_ranked_candidate_ranking_score = metrics.top_ranked_candidate_ranking_score;
    result.top_ranked_candidate_cost_delta_to_direct = metrics.top_ranked_candidate_cost_delta_to_direct;
    result.top_ranked_candidate_clearance_margin = metrics.top_ranked_candidate_clearance_margin;
    result.top_ranked_candidate_manipulability_score = metrics.top_ranked_candidate_manipulability_score;
    result.top_ranked_candidate_axial_progress = metrics.top_ranked_candidate_axial_progress;
    result.top_ranked_candidate_lateral_offset = metrics.top_ranked_candidate_lateral_offset;

    return result;
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("planner_comparison_v2");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "规划器对比实验 V2（基于STL特征提取）");
    RCLCPP_INFO(logger, "========================================");

    // 初始化机器人
    const std::string robot_node_name =
        "planner_comparison_v2_node_" + std::to_string(static_cast<long long>(getpid()));
    CR5Robot robot(robot_node_name);
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        return 1;
    }
    const auto adaptive_ellipsoid_config =
        my_cr5_control::benchmarks::getAdaptiveEllipsoidEnvConfig();
    robot.enableAdaptiveEllipsoidSampling(adaptive_ellipsoid_config.enabled);
    logAdaptiveEllipsoidConfig(logger, adaptive_ellipsoid_config);

    // 添加测试环境（包含地板）
    robot.addSimulationEnvironment();
    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 生成测试场景（基于STL模型）
    MeasurementPointGenerator generator;
    auto scenarios = generator.generateTestScenarios();
    std::vector<std::string> unknown_scene_names;
    scenarios = filterV2Scenes(scenarios, &unknown_scene_names);

    if (scenarios.empty()) {
        RCLCPP_ERROR(logger, "场景过滤后为空，请检查 MY_CR5_CONTROL_V2_SCENES / MY_CR5_CONTROL_BENCHMARK_SCENES");
        return 1;
    }
    logV2SceneSelection(logger, scenarios, unknown_scene_names);

    std::vector<std::string> unknown_planner_names;
    const std::vector<PlannerConfig> planners = getV2PlannerConfigs(&unknown_planner_names);

    const std::string timestamp = my_cr5_control::results::makeTimestamp();
    const int repeat_count = getV2RepeatCount();
    RCLCPP_INFO(logger, "基准版本: %s", kV2BenchmarkVersion);
    RCLCPP_INFO(logger, "重复次数: %d", repeat_count);
    logV2PlannerSelection(logger, planners, unknown_planner_names);

    const std::string results_path =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_v2_results.csv");
    const std::string summary_path =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_v2_summary.csv");
    const std::string plot_data_path =
        my_cr5_control::results::makeSharedOutputPath("planner_comparison_v2_plot_data_metrics.csv");
    const std::string plot_summary_path =
        my_cr5_control::results::makeSharedOutputPath("planner_comparison_v2_plot_summary_metrics.csv");

    // 打开CSV文件（Excel可以打开）
    std::ofstream csv_file(results_path);
    writeDetailedHeader(csv_file);

    const bool plot_data_exists =
        std::filesystem::exists(plot_data_path) && std::filesystem::file_size(plot_data_path) > 0;
    std::ofstream plot_data_file(plot_data_path, std::ios::app);
    if (!plot_data_exists) {
        writeDetailedHeader(plot_data_file);
    }

    // 运行测试
    std::vector<PlannerResult> all_results;
    auto writeDetailedRow = [&](std::ostream& os, const PlannerResult& result) {
        os << kV2BenchmarkVersion << ","
           << timestamp << ","
           << repeat_count << ","
           << result.repeat_index << ","
           << result.planner_name << ","
           << result.planning_mode << ","
           << result.planner_id << ","
           << result.scenario_name << ","
           << result.difficulty << ","
           << "\"" << result.point_description << "\","
           << std::fixed << std::setprecision(2) << result.difficulty_score << ","
           << std::fixed << std::setprecision(4)
           << result.tip_point.x << ","
           << result.tip_point.y << ","
           << result.tip_point.z << ","
           << result.flange_pose.position.x << ","
           << result.flange_pose.position.y << ","
           << result.flange_pose.position.z << ","
           << (result.success ? "成功" : "失败") << ","
           << std::fixed << std::setprecision(1) << result.wall_time_ms << ","
           << result.planner_reported_time_ms << ","
           << result.planning_budget_ms << ","
           << result.planner_calls << ","
           << (result.hit_budget_limit ? "是" : "否") << ","
           << (result.wall_time_ms < kFastSolveThresholdMs ? "是" : "否") << ","
           << result.guide_candidate_count << ","
           << result.guide_candidates_attempted << ","
           << (result.used_direct_plan ? "是" : "否") << ","
           << (result.direct_plan_success ? "是" : "否") << ","
           << result.direct_attempt_wall_time_ms << ","
           << std::fixed << std::setprecision(4) << result.direct_path_cost << ","
           << result.top_ranked_candidate_id << ","
           << result.top_ranked_candidate_heuristic_cost << ","
           << result.top_ranked_candidate_ranking_score << ","
           << result.top_ranked_candidate_cost_delta_to_direct << ","
           << result.top_ranked_candidate_clearance_margin << ","
           << result.top_ranked_candidate_manipulability_score << ","
           << result.top_ranked_candidate_axial_progress << ","
           << result.top_ranked_candidate_lateral_offset << "\n";
    };

    for (int repeat_index = 1; repeat_index <= repeat_count; ++repeat_index) {
        RCLCPP_INFO(logger, "\n========================================");
        RCLCPP_INFO(logger, "重复实验 %d / %d", repeat_index, repeat_count);
        RCLCPP_INFO(logger, "========================================");

        for (const auto& scenario : scenarios) {
            RCLCPP_INFO(logger, "\n>>> 测试场景: %s (%s)",
                        scenario.name.c_str(), scenario.difficulty.c_str());

            if (!scenario.points.empty()) {
                RCLCPP_INFO(logger, "    测点: %s (难度: %.2f)",
                            scenario.points[0].description.c_str(),
                            scenario.points[0].difficulty_score);
            }

            for (const auto& planner : planners) {
                RCLCPP_INFO(logger, "  测试规划器: %s", planner.name.c_str());

                // 纯规划 benchmark：固定以 home 命名状态作为起点，不执行轨迹。
                auto result = testPlanner(
                    robot, planner, scenario, repeat_index, adaptive_ellipsoid_config);
                all_results.push_back(result);

                writeDetailedRow(csv_file, result);
                writeDetailedRow(plot_data_file, result);

                RCLCPP_INFO(logger, "    结果: %s, 时间: %.1f ms",
                            result.success ? "✓ 成功" : "✗ 失败",
                            result.wall_time_ms);

                std::this_thread::sleep_for(std::chrono::milliseconds(500));
            }
        }
    }

    csv_file.close();
    plot_data_file.close();

    // 生成统计摘要
    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "统计摘要");
    RCLCPP_INFO(logger, "========================================");

    for (const auto& planner : planners) {
        int success_count = 0;
        double total_time = 0.0;
        double total_reported_time = 0.0;
        int total_planner_calls = 0;
        int total_guide_candidate_count = 0;
        int total_guide_attempt_count = 0;
        int direct_fallback_count = 0;
        int count = 0;
        int budget_hit_count = 0;
        int fast_solve_count = 0;
        std::vector<double> time_samples;

        for (const auto& result : all_results) {
            if (result.planner_name == planner.name &&
                result.planning_mode == ((planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided")) {
                if (result.success) success_count++;
                total_time += result.wall_time_ms;
                total_reported_time += result.planner_reported_time_ms;
                total_planner_calls += result.planner_calls;
                total_guide_candidate_count += result.guide_candidate_count;
                total_guide_attempt_count += result.guide_candidates_attempted;
                if (result.used_direct_plan) {
                    ++direct_fallback_count;
                }
                count++;
                time_samples.push_back(result.wall_time_ms);
                if (result.hit_budget_limit) budget_hit_count++;
                if (result.wall_time_ms < kFastSolveThresholdMs) fast_solve_count++;
            }
        }

        double success_rate = count > 0 ? (100.0 * success_count / count) : 0.0;
        double avg_time = count > 0 ? (total_time / count) : 0.0;
        const double median_time = computeQuantile(time_samples, 0.5);
        const double p25_time = computeQuantile(time_samples, 0.25);
        const double p75_time = computeQuantile(time_samples, 0.75);
        const double budget_hit_rate = count > 0 ? (100.0 * budget_hit_count / count) : 0.0;
        const double fast_solve_rate = count > 0 ? (100.0 * fast_solve_count / count) : 0.0;
        const double avg_reported_time = count > 0 ? (total_reported_time / count) : 0.0;
        const double avg_planner_calls = count > 0 ? (static_cast<double>(total_planner_calls) / count) : 0.0;
        const double avg_guide_candidates =
            count > 0 ? (static_cast<double>(total_guide_candidate_count) / count) : 0.0;
        const double avg_guide_attempts =
            count > 0 ? (static_cast<double>(total_guide_attempt_count) / count) : 0.0;
        const double direct_fallback_rate = count > 0 ? (100.0 * direct_fallback_count / count) : 0.0;

        RCLCPP_INFO(logger, "%s: 成功率 %.1f%%, 平均时间 %.1f ms",
                    planner.name.c_str(), success_rate, avg_time);
        RCLCPP_INFO(logger,
                    "  - 中位数 %.1f ms, P25/P75 = %.1f / %.1f ms, 预算命中 %d/%d (%.1f%%), "
                    "快速求解 %d/%d (%.1f%%), 平均MoveIt时间 %.1f ms, 平均调用 %.2f, "
                    "平均guide候选 %.2f, 平均guide尝试 %.2f, direct回退率 %.1f%%",
                    median_time, p25_time, p75_time,
                    budget_hit_count, count, budget_hit_rate,
                    fast_solve_count, count, fast_solve_rate,
                    avg_reported_time, avg_planner_calls,
                    avg_guide_candidates,
                    avg_guide_attempts, direct_fallback_rate);
    }

    std::ofstream summary_file(summary_path);
    writeSummaryHeader(summary_file);

    const bool plot_summary_exists =
        std::filesystem::exists(plot_summary_path) && std::filesystem::file_size(plot_summary_path) > 0;
    std::ofstream plot_summary_file(plot_summary_path, std::ios::app);
    if (!plot_summary_exists) {
        writeSummaryHeader(plot_summary_file);
    }

    for (const auto& planner : planners) {
        int success_count = 0;
        double total_time = 0.0;
        double total_reported_time = 0.0;
        int total_planner_calls = 0;
        int total_guide_candidate_count = 0;
        int total_guide_attempt_count = 0;
        int direct_fallback_count = 0;
        int count = 0;
        int budget_hit_count = 0;
        int fast_solve_count = 0;
        std::vector<double> time_samples;

        std::vector<DifficultyTiming> difficulty_timings(
            my_cr5_control::paper_mainline::canonicalV2DifficultyOrder().size());

        for (const auto& result : all_results) {
            if (result.planner_name == planner.name &&
                result.planning_mode == ((planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided")) {
                if (result.success) success_count++;
                total_time += result.wall_time_ms;
                total_reported_time += result.planner_reported_time_ms;
                total_planner_calls += result.planner_calls;
                total_guide_candidate_count += result.guide_candidate_count;
                total_guide_attempt_count += result.guide_candidates_attempted;
                if (result.used_direct_plan) {
                    ++direct_fallback_count;
                }
                count++;
                time_samples.push_back(result.wall_time_ms);
                if (result.hit_budget_limit) budget_hit_count++;
                if (result.wall_time_ms < kFastSolveThresholdMs) fast_solve_count++;

                const std::size_t difficulty_index = findV2DifficultyIndex(result.difficulty);
                if (difficulty_index < difficulty_timings.size()) {
                    difficulty_timings[difficulty_index].total_time_ms += result.wall_time_ms;
                    difficulty_timings[difficulty_index].count++;
                }
            }
        }

        double success_rate = count > 0 ? (100.0 * success_count / count) : 0.0;
        double avg_time = count > 0 ? (total_time / count) : 0.0;
        const double median_time = computeQuantile(time_samples, 0.5);
        const double p25_time = computeQuantile(time_samples, 0.25);
        const double p75_time = computeQuantile(time_samples, 0.75);
        const double budget_hit_rate = count > 0 ? (100.0 * budget_hit_count / count) : 0.0;
        const double fast_solve_rate = count > 0 ? (100.0 * fast_solve_count / count) : 0.0;
        const double avg_reported_time = count > 0 ? (total_reported_time / count) : 0.0;
        const double avg_planner_calls = count > 0 ? (static_cast<double>(total_planner_calls) / count) : 0.0;
        const double avg_guide_candidates =
            count > 0 ? (static_cast<double>(total_guide_candidate_count) / count) : 0.0;
        const double avg_guide_attempts =
            count > 0 ? (static_cast<double>(total_guide_attempt_count) / count) : 0.0;
        const double direct_fallback_rate = count > 0 ? (100.0 * direct_fallback_count / count) : 0.0;

        auto writeSummaryRow = [&](std::ostream& os) {
            os << kV2BenchmarkVersion << ","
               << timestamp << ","
               << repeat_count << ","
               << planner.name << ","
               << ((planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided") << ","
               << planner.planner_id << ","
               << std::fixed << std::setprecision(1) << success_rate << ","
               << success_count << ","
               << count << ","
               << avg_time << ","
               << median_time << ","
               << p25_time << ","
               << p75_time << ","
               << budget_hit_count << ","
               << budget_hit_rate << ","
               << fast_solve_count << ","
               << fast_solve_rate << ","
               << avg_reported_time << ","
               << avg_planner_calls << ","
               << avg_guide_candidates << ","
               << avg_guide_attempts << ","
               << direct_fallback_rate << ","
               << averageDifficultyTime(difficulty_timings, 0) << ","
               << averageDifficultyTime(difficulty_timings, 1) << ","
               << averageDifficultyTime(difficulty_timings, 2) << ","
               << averageDifficultyTime(difficulty_timings, 3) << ","
               << results_path << "\n";
        };
        writeSummaryRow(summary_file);
        writeSummaryRow(plot_summary_file);
    }

    summary_file.close();
    plot_summary_file.close();

    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "测试完成");
    RCLCPP_INFO(logger, "详细结果: %s", results_path.c_str());
    RCLCPP_INFO(logger, "统计摘要: %s", summary_path.c_str());
    RCLCPP_INFO(logger, "绘图明细: %s", plot_data_path.c_str());
    RCLCPP_INFO(logger, "绘图摘要: %s", plot_summary_path.c_str());
    RCLCPP_INFO(logger, "========================================");

    rclcpp::shutdown();
    return 0;
}
