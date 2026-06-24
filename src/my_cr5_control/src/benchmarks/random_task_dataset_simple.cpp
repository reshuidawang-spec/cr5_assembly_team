#include <rclcpp/rclcpp.hpp>

#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"
#include "my_cr5_control/planner_selection_utils.hpp"
#include "my_cr5_control/result_utils.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>

#include <algorithm>
#include <chrono>
#include <cctype>
#include <cmath>
#include <cstdlib>
#include <cstdint>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <random>
#include <sstream>
#include <string>
#include <thread>
#include <unistd.h>
#include <vector>

namespace {

inline constexpr char kDatasetVersion[] = "simple_random_dataset_v20260311_stage2";
inline constexpr double kFastSolveThresholdMs = 1000.0;

struct SimpleBox {
    double center_x = 0.45;
    double center_y = 0.0;
    double center_z = 0.15;
    double width = 0.20;
    double depth = 0.20;
    double height = 0.20;
    double hole_radius = 0.04;
    double hole_depth = 0.12;
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

struct RandomTask {
    int task_index;
    std::string family;
    std::string difficulty;
    std::string description;
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Point normal;
    geometry_msgs::msg::Quaternion orientation;
    geometry_msgs::msg::Pose flange_pose;
    double difficulty_score;
    double depth_from_top_m;
    double depth_ratio;
    double lateral_offset_x_m;
    double lateral_offset_y_m;
    double radial_offset_m;
    double opening_size_m;
    double estimated_clearance_m;
};

struct TaskPlannerResult {
    int task_index;
    std::string family;
    std::string difficulty;
    std::string planner_name;
    std::string planning_mode;
    std::string planner_id;
    bool success;
    double wall_time_ms;
    double planner_reported_time_ms;
    double planning_budget_ms;
    int planner_calls;
    bool hit_budget_limit;
};

int getEnvInt(const char* key, int default_value, int min_value, int max_value) {
    return my_cr5_control::env::getIntClamped(key, default_value, min_value, max_value);
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

    RCLCPP_INFO(logger, "自适应椭球采样已启用，默认复用任务 difficulty_score");
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

std::uint32_t getDatasetSeed() {
    const auto seed = my_cr5_control::env::getUint32("MY_CR5_CONTROL_RANDOM_SIMPLE_SEED");
    if (seed.has_value()) {
        return *seed;
    }

    return static_cast<std::uint32_t>(
        std::chrono::steady_clock::now().time_since_epoch().count() & 0xffffffffu);
}

std::vector<PlannerConfig> getPlannerConfigs() {
    const std::vector<PlannerConfig> defaults = {
        {"FMT", "FMT", 10.0, PlanningMode::OMPL},
        {"LBTRRT", "LBTRRT", 10.0, PlanningMode::OMPL},
        {"RRTConnect", "RRTConnect", 10.0, PlanningMode::OMPL},
        {"HeuristicGuided", "custom_two_stage_guided", 10.0, PlanningMode::HeuristicGuided},
    };

    const std::string raw =
        my_cr5_control::env::getString("MY_CR5_CONTROL_RANDOM_SIMPLE_PLANNERS", "");
    if (raw.empty()) {
        return defaults;
    }

    const std::vector<PlannerConfig> all = {
        {"RRTConnect", "RRTConnect", 10.0, PlanningMode::OMPL},
        {"RRTstar", "RRTstar", 10.0, PlanningMode::OMPL},
        {"LBTRRT", "LBTRRT", 10.0, PlanningMode::OMPL},
        {"FMT", "FMT", 10.0, PlanningMode::OMPL},
        {"BFMT", "BFMT", 10.0, PlanningMode::OMPL},
        {"PRMstar", "PRMstar", 10.0, PlanningMode::OMPL},
        {"HeuristicGuided", "custom_two_stage_guided", 10.0, PlanningMode::HeuristicGuided},
    };

    std::vector<PlannerConfig> selected;
    std::stringstream ss(raw);
    std::string token;
    while (std::getline(ss, token, ',')) {
        token.erase(
            std::remove_if(token.begin(), token.end(), [](unsigned char ch) { return std::isspace(ch); }),
            token.end());
        if (token.empty()) {
            continue;
        }
        auto it = std::find_if(all.begin(), all.end(), [&](const PlannerConfig& config) {
            return config.name == token;
        });
        if (it != all.end()) {
            selected.push_back(*it);
        }
    }

    return selected.empty() ? defaults : selected;
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

geometry_msgs::msg::Quaternion makeQuaternion(double x, double y, double z, double w) {
    geometry_msgs::msg::Quaternion quat;
    quat.x = x;
    quat.y = y;
    quat.z = z;
    quat.w = w;
    return quat;
}

void writeDetailedHeader(std::ostream& os) {
    os << "数据集版本,实验时间戳,随机种子,总任务数,任务序号,"
       << "任务族,难度,任务描述,"
       << "规划器,模式,规划器ID,"
       << "尖端X,尖端Y,尖端Z,法向X,法向Y,法向Z,法兰X,法兰Y,法兰Z,"
       << "箱体宽,箱体深,箱体高,孔半径,孔深,"
       << "顶部深度(m),深度比例,横向偏移X(m),横向偏移Y(m),距孔中心径向偏移(m),局部开口尺寸(m),估计净空(m),难度评分,"
       << "成功,墙钟时间(ms),MoveIt规划时间(ms),预算上限(ms),规划调用次数,触发预算上限,快速求解(<1s)\n";
}

void writeSummaryHeader(std::ostream& os) {
    os << "数据集版本,实验时间戳,随机种子,总任务数,"
       << "规划器,模式,规划器ID,成功率(%),成功样本数,总样本数,"
       << "平均时间(ms),中位时间(ms),P25(ms),P75(ms),"
       << "预算命中数,预算命中率(%),快速求解数,快速求解率(%),平均MoveIt规划时间(ms),平均规划调用次数,详细结果文件\n";
}

void applySimpleScene(const SimpleBox& box, const rclcpp::Logger& logger) {
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    const std::vector<std::string> object_ids = {
        "floor",
        "box_bottom",
        "box_front",
        "box_back",
        "box_left",
        "box_right",
        "box_top_front",
        "box_top_back",
        "box_top_left",
        "box_top_right",
    };
    planning_scene.removeCollisionObjects(object_ids);
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    std::vector<moveit_msgs::msg::CollisionObject> objects;
    objects.reserve(10);

    const double wall_thickness = 0.02;
    const double hole_size = box.hole_radius * 2.0;

    auto addBoxObject = [&](const std::string& id, double dx, double dy, double dz,
                            double px, double py, double pz) {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "base_link";
        object.id = id;

        shape_msgs::msg::SolidPrimitive primitive;
        primitive.type = primitive.BOX;
        primitive.dimensions = {dx, dy, dz};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = px;
        pose.position.y = py;
        pose.position.z = pz;

        object.primitives.push_back(primitive);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        objects.push_back(object);
    };

    addBoxObject("floor", 10.0, 10.0, 1.0, 0.0, 0.0, -0.51);
    addBoxObject("box_bottom", box.width, box.depth, wall_thickness,
                 box.center_x, box.center_y, box.center_z + wall_thickness / 2.0);
    addBoxObject("box_front", box.width, wall_thickness, box.height,
                 box.center_x, box.center_y + box.depth / 2.0 - wall_thickness / 2.0,
                 box.center_z + box.height / 2.0);
    addBoxObject("box_back", box.width, wall_thickness, box.height,
                 box.center_x, box.center_y - box.depth / 2.0 + wall_thickness / 2.0,
                 box.center_z + box.height / 2.0);
    addBoxObject("box_left", wall_thickness, box.depth, box.height,
                 box.center_x - box.width / 2.0 + wall_thickness / 2.0, box.center_y,
                 box.center_z + box.height / 2.0);
    addBoxObject("box_right", wall_thickness, box.depth, box.height,
                 box.center_x + box.width / 2.0 - wall_thickness / 2.0, box.center_y,
                 box.center_z + box.height / 2.0);
    addBoxObject("box_top_front", box.width, (box.depth - hole_size) / 2.0, wall_thickness,
                 box.center_x,
                 box.center_y + hole_size / 2.0 + (box.depth - hole_size) / 4.0,
                 box.center_z + box.height - wall_thickness / 2.0);
    addBoxObject("box_top_back", box.width, (box.depth - hole_size) / 2.0, wall_thickness,
                 box.center_x,
                 box.center_y - hole_size / 2.0 - (box.depth - hole_size) / 4.0,
                 box.center_z + box.height - wall_thickness / 2.0);
    addBoxObject("box_top_left", (box.width - hole_size) / 2.0, hole_size, wall_thickness,
                 box.center_x - hole_size / 2.0 - (box.width - hole_size) / 4.0,
                 box.center_y,
                 box.center_z + box.height - wall_thickness / 2.0);
    addBoxObject("box_top_right", (box.width - hole_size) / 2.0, hole_size, wall_thickness,
                 box.center_x + hole_size / 2.0 + (box.width - hole_size) / 4.0,
                 box.center_y,
                 box.center_z + box.height - wall_thickness / 2.0);

    planning_scene.applyCollisionObjects(objects);
    RCLCPP_INFO(logger, "已加载 simple 随机任务场景，共 %zu 个碰撞对象", objects.size());
}

double clearanceToBoxEdges(const SimpleBox& box, double offset_x, double offset_y) {
    const double clear_x = box.width * 0.5 - std::abs(offset_x);
    const double clear_y = box.depth * 0.5 - std::abs(offset_y);
    return std::max(0.0, std::min(clear_x, clear_y));
}

RandomTask sampleTask(const SimpleBox& box, std::mt19937& rng, int task_index) {
    std::uniform_int_distribution<int> family_dist(0, 5);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * M_PI);

    RandomTask task;
    task.task_index = task_index;

    const double top_z = box.center_z + box.height;
    const int family_id = family_dist(rng);
    const double theta = angle_dist(rng);

    switch (family_id) {
        case 0: {
            std::uniform_real_distribution<double> x_dist(-0.04, 0.04);
            std::uniform_real_distribution<double> y_dist(-0.04, 0.04);
            std::uniform_real_distribution<double> z_dist(0.04, 0.08);
            task.family = "top_open";
            task.difficulty = "easy";
            task.description = "顶部开放接近";
            task.tip_point.x = box.center_x + x_dist(rng);
            task.tip_point.y = box.center_y + y_dist(rng);
            task.tip_point.z = top_z + z_dist(rng);
            task.normal.x = 0.0;
            task.normal.y = 0.0;
            task.normal.z = -1.0;
            task.orientation = makeQuaternion(0.0, 1.0, 0.0, 0.0);
            task.difficulty_score = 0.20 + 0.15 * unit_dist(rng);
            task.opening_size_m = std::min(box.width, box.depth);
            task.estimated_clearance_m = clearanceToBoxEdges(
                box, task.tip_point.x - box.center_x, task.tip_point.y - box.center_y);
            break;
        }
        case 1: {
            std::uniform_real_distribution<double> x_dist(-0.06, 0.06);
            std::uniform_real_distribution<double> z_ratio_dist(0.35, 0.80);
            task.family = "front_side";
            task.difficulty = "medium";
            task.description = "前侧面接触";
            task.tip_point.x = box.center_x + x_dist(rng);
            task.tip_point.y = box.center_y + box.depth / 2.0 + 0.01;
            task.tip_point.z = box.center_z + box.height * z_ratio_dist(rng);
            task.normal.x = 0.0;
            task.normal.y = -1.0;
            task.normal.z = 0.0;
            task.orientation = makeQuaternion(0.7071068, 0.0, 0.0, 0.7071068);
            task.difficulty_score = 0.45 + 0.15 * unit_dist(rng);
            task.opening_size_m = box.width;
            task.estimated_clearance_m = 0.01 + 0.02 * unit_dist(rng);
            break;
        }
        case 2: {
            std::uniform_real_distribution<double> y_dist(-0.03, 0.03);
            std::uniform_real_distribution<double> z_ratio_dist(0.65, 0.90);
            task.family = "right_upper_angled";
            task.difficulty = "medium";
            task.description = "右上侧斜向接近";
            task.tip_point.x = box.center_x + box.width / 2.0 + 0.01;
            task.tip_point.y = box.center_y + y_dist(rng);
            task.tip_point.z = box.center_z + box.height * z_ratio_dist(rng);
            task.normal.x = -0.7071068;
            task.normal.y = 0.0;
            task.normal.z = -0.7071068;
            task.orientation = makeQuaternion(0.0, -0.9238795, 0.0, 0.3826834);
            task.difficulty_score = 0.55 + 0.15 * unit_dist(rng);
            task.opening_size_m = box.height * 0.35;
            task.estimated_clearance_m = 0.01 + 0.015 * unit_dist(rng);
            break;
        }
        case 3: {
            std::uniform_real_distribution<double> radius_dist(0.0, box.hole_radius * 0.55);
            std::uniform_real_distribution<double> depth_dist(0.02, 0.05);
            const double radius = radius_dist(rng);
            task.family = "hole_shallow";
            task.difficulty = "hard";
            task.description = "孔内浅层";
            task.tip_point.x = box.center_x + radius * std::cos(theta);
            task.tip_point.y = box.center_y + radius * std::sin(theta);
            task.tip_point.z = top_z - depth_dist(rng);
            task.normal.x = 0.0;
            task.normal.y = 0.0;
            task.normal.z = -1.0;
            task.orientation = makeQuaternion(0.0, 1.0, 0.0, 0.0);
            task.difficulty_score = 0.65 + 0.10 * unit_dist(rng);
            task.opening_size_m = box.hole_radius * 2.0;
            task.estimated_clearance_m = std::max(0.0, box.hole_radius - radius);
            break;
        }
        case 4: {
            std::uniform_real_distribution<double> radius_dist(box.hole_radius * 0.65, box.hole_radius * 0.95);
            std::uniform_real_distribution<double> depth_dist(0.04, 0.08);
            const double radius = radius_dist(rng);
            task.family = "hole_edge";
            task.difficulty = "hard";
            task.description = "孔边缘偏置";
            task.tip_point.x = box.center_x + radius * std::cos(theta);
            task.tip_point.y = box.center_y + radius * std::sin(theta);
            task.tip_point.z = top_z - depth_dist(rng);
            task.normal.x = 0.0;
            task.normal.y = 0.0;
            task.normal.z = -1.0;
            task.orientation = makeQuaternion(0.0, 1.0, 0.0, 0.0);
            task.difficulty_score = 0.78 + 0.10 * unit_dist(rng);
            task.opening_size_m = box.hole_radius * 2.0;
            task.estimated_clearance_m = std::max(0.0, box.hole_radius - radius);
            break;
        }
        default: {
            std::uniform_real_distribution<double> radius_dist(0.0, box.hole_radius * 0.35);
            std::uniform_real_distribution<double> depth_dist(0.085, 0.11);
            const double radius = radius_dist(rng);
            task.family = "hole_deep";
            task.difficulty = "extreme";
            task.description = "孔内深层";
            task.tip_point.x = box.center_x + radius * std::cos(theta);
            task.tip_point.y = box.center_y + radius * std::sin(theta);
            task.tip_point.z = top_z - depth_dist(rng);
            task.normal.x = 0.0;
            task.normal.y = 0.0;
            task.normal.z = -1.0;
            task.orientation = makeQuaternion(0.0, 1.0, 0.0, 0.0);
            task.difficulty_score = 0.88 + 0.10 * unit_dist(rng);
            task.opening_size_m = box.hole_radius * 2.0;
            task.estimated_clearance_m = std::max(0.0, box.hole_radius - radius);
            break;
        }
    }

    task.flange_pose =
        my_cr5_control::tool::buildFlangePoseFromTipPoint(task.tip_point, task.orientation);

    task.depth_from_top_m = std::max(0.0, top_z - task.tip_point.z);
    task.depth_ratio = std::clamp(task.depth_from_top_m / std::max(1e-6, box.hole_depth), 0.0, 1.5);
    task.lateral_offset_x_m = task.tip_point.x - box.center_x;
    task.lateral_offset_y_m = task.tip_point.y - box.center_y;
    task.radial_offset_m = std::hypot(task.lateral_offset_x_m, task.lateral_offset_y_m);
    return task;
}

std::vector<RandomTask> generateTasks(const SimpleBox& box, int task_count, std::uint32_t seed) {
    std::mt19937 rng(seed);
    std::vector<RandomTask> tasks;
    tasks.reserve(task_count);
    for (int task_index = 1; task_index <= task_count; ++task_index) {
        tasks.push_back(sampleTask(box, rng, task_index));
    }
    return tasks;
}

TaskPlannerResult evaluateTask(
    CR5Robot& robot,
    const PlannerConfig& planner,
    const RandomTask& task,
    const my_cr5_control::benchmarks::AdaptiveEllipsoidEnvConfig& adaptive_ellipsoid_config) {
    TaskPlannerResult result;
    result.task_index = task.task_index;
    result.family = task.family;
    result.difficulty = task.difficulty;
    result.planner_name = planner.name;
    result.planning_mode = planner.mode == PlanningMode::OMPL ? "ompl" : "heuristic_guided";
    result.planner_id = planner.planner_id;

    CR5Robot::PlanningMetrics metrics;
    if (planner.mode == PlanningMode::OMPL) {
        result.success = robot.planToPoseWithPlanner(
            task.flange_pose, planner.planner_id, planner.planning_time_s, "home", &metrics);
    } else {
        // HeuristicGuided 当前对应外部两阶段采样引导原型，不走 OMPL planner_id 分发。
        applyAdaptiveEllipsoidForDifficulty(
            robot, adaptive_ellipsoid_config, task.difficulty_score);
        result.success = robot.planToPoseImproved(
            task.flange_pose, "home", planner.planning_time_s, &metrics);
    }

    result.wall_time_ms = metrics.wall_time_ms;
    result.planner_reported_time_ms = metrics.planner_reported_time_ms;
    result.planning_budget_ms = metrics.planning_budget_ms;
    result.planner_calls = metrics.planner_calls;
    result.hit_budget_limit = metrics.hit_budget_limit;
    return result;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("random_task_dataset_simple");

    const int task_count = getEnvInt("MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS", 60, 1, 5000);
    const std::uint32_t seed = getDatasetSeed();
    const auto planners = getPlannerConfigs();
    const std::string timestamp = my_cr5_control::results::makeTimestamp();

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "simple 随机任务数据采集器");
    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "数据集版本: %s", kDatasetVersion);
    RCLCPP_INFO(logger, "任务数: %d", task_count);
    RCLCPP_INFO(logger, "随机种子: %u", seed);
    RCLCPP_INFO(logger, "规划器数: %zu", planners.size());

    const std::string robot_node_name =
        "random_task_dataset_simple_node_" + std::to_string(static_cast<long long>(getpid()));
    CR5Robot robot(robot_node_name);
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        return 1;
    }
    const auto adaptive_ellipsoid_config =
        my_cr5_control::benchmarks::getAdaptiveEllipsoidEnvConfig();
    robot.enableAdaptiveEllipsoidSampling(adaptive_ellipsoid_config.enabled);
    logAdaptiveEllipsoidConfig(logger, adaptive_ellipsoid_config);

    const SimpleBox box;
    robot.setGuideEnvironmentBoxHint(
        box.center_x, box.center_y, box.center_z + box.height * 0.5, box.width, box.depth, box.height);
    applySimpleScene(box, logger);
    std::this_thread::sleep_for(std::chrono::seconds(2));

    const auto tasks = generateTasks(box, task_count, seed);
    RCLCPP_INFO(logger, "已生成 %zu 个随机任务", tasks.size());

    const std::string results_path =
        my_cr5_control::results::makeOutputPath(timestamp, "simple_random_task_dataset_results.csv");
    const std::string summary_path =
        my_cr5_control::results::makeOutputPath(timestamp, "simple_random_task_dataset_summary.csv");

    std::ofstream results_file(results_path);
    writeDetailedHeader(results_file);

    std::vector<TaskPlannerResult> all_results;
    all_results.reserve(tasks.size() * planners.size());

    for (const auto& task : tasks) {
        RCLCPP_INFO(logger, "\n任务 %d / %d: %s (%s), tip=(%.3f, %.3f, %.3f)",
                    task.task_index, task_count, task.family.c_str(), task.difficulty.c_str(),
                    task.tip_point.x, task.tip_point.y, task.tip_point.z);

        for (const auto& planner : planners) {
            auto result = evaluateTask(robot, planner, task, adaptive_ellipsoid_config);
            all_results.push_back(result);

            results_file << kDatasetVersion << ","
                         << timestamp << ","
                         << seed << ","
                         << task_count << ","
                         << task.task_index << ","
                         << task.family << ","
                         << task.difficulty << ","
                         << "\"" << task.description << "\","
                         << planner.name << ","
                         << (planner.mode == PlanningMode::OMPL ? "ompl" : "heuristic_guided") << ","
                         << planner.planner_id << ","
                         << std::fixed << std::setprecision(4)
                         << task.tip_point.x << ","
                         << task.tip_point.y << ","
                         << task.tip_point.z << ","
                         << task.normal.x << ","
                         << task.normal.y << ","
                         << task.normal.z << ","
                         << task.flange_pose.position.x << ","
                         << task.flange_pose.position.y << ","
                         << task.flange_pose.position.z << ","
                         << box.width << ","
                         << box.depth << ","
                         << box.height << ","
                         << box.hole_radius << ","
                         << box.hole_depth << ","
                         << task.depth_from_top_m << ","
                         << task.depth_ratio << ","
                         << task.lateral_offset_x_m << ","
                         << task.lateral_offset_y_m << ","
                         << task.radial_offset_m << ","
                         << task.opening_size_m << ","
                         << task.estimated_clearance_m << ","
                         << std::setprecision(2) << task.difficulty_score << ","
                         << (result.success ? "成功" : "失败") << ","
                         << std::setprecision(1)
                         << result.wall_time_ms << ","
                         << result.planner_reported_time_ms << ","
                         << result.planning_budget_ms << ","
                         << result.planner_calls << ","
                         << (result.hit_budget_limit ? "是" : "否") << ","
                         << (result.wall_time_ms < kFastSolveThresholdMs ? "是" : "否") << "\n";

            RCLCPP_INFO(logger, "  - %s: %s, wall=%.1f ms, budget_hit=%s",
                        planner.name.c_str(),
                        result.success ? "成功" : "失败",
                        result.wall_time_ms,
                        result.hit_budget_limit ? "是" : "否");
        }
    }
    results_file.close();

    std::ofstream summary_file(summary_path);
    writeSummaryHeader(summary_file);

    for (const auto& planner : planners) {
        int success_count = 0;
        int count = 0;
        int budget_hit_count = 0;
        int fast_count = 0;
        double total_wall_time = 0.0;
        double total_reported_time = 0.0;
        double total_planner_calls = 0.0;
        std::vector<double> wall_times;

        for (const auto& result : all_results) {
            if (result.planner_name != planner.name) {
                continue;
            }
            ++count;
            if (result.success) {
                ++success_count;
            }
            total_wall_time += result.wall_time_ms;
            total_reported_time += result.planner_reported_time_ms;
            total_planner_calls += static_cast<double>(result.planner_calls);
            wall_times.push_back(result.wall_time_ms);
            if (result.hit_budget_limit) {
                ++budget_hit_count;
            }
            if (result.wall_time_ms < kFastSolveThresholdMs) {
                ++fast_count;
            }
        }

        const double success_rate = count > 0 ? 100.0 * success_count / count : 0.0;
        const double avg_wall_time = count > 0 ? total_wall_time / count : 0.0;
        const double median_wall_time = computeQuantile(wall_times, 0.5);
        const double p25_wall_time = computeQuantile(wall_times, 0.25);
        const double p75_wall_time = computeQuantile(wall_times, 0.75);
        const double budget_hit_rate = count > 0 ? 100.0 * budget_hit_count / count : 0.0;
        const double fast_rate = count > 0 ? 100.0 * fast_count / count : 0.0;
        const double avg_reported_time = count > 0 ? total_reported_time / count : 0.0;
        const double avg_planner_calls = count > 0 ? total_planner_calls / count : 0.0;

        summary_file << kDatasetVersion << ","
                     << timestamp << ","
                     << seed << ","
                     << task_count << ","
                     << planner.name << ","
                     << (planner.mode == PlanningMode::OMPL ? "ompl" : "heuristic_guided") << ","
                     << planner.planner_id << ","
                     << std::fixed << std::setprecision(1)
                     << success_rate << ","
                     << success_count << ","
                     << count << ","
                     << avg_wall_time << ","
                     << median_wall_time << ","
                     << p25_wall_time << ","
                     << p75_wall_time << ","
                     << budget_hit_count << ","
                     << budget_hit_rate << ","
                     << fast_count << ","
                     << fast_rate << ","
                     << avg_reported_time << ","
                     << avg_planner_calls << ","
                     << results_path << "\n";

        RCLCPP_INFO(logger,
                    "%s: 成功率 %.1f%%, 中位数 %.1f ms, 预算命中 %d/%d, 快速求解 %d/%d",
                    planner.name.c_str(), success_rate, median_wall_time,
                    budget_hit_count, count, fast_count, count);
    }
    summary_file.close();

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "随机数据采集完成");
    RCLCPP_INFO(logger, "详细结果: %s", results_path.c_str());
    RCLCPP_INFO(logger, "统计摘要: %s", summary_path.c_str());
    RCLCPP_INFO(logger, "========================================");

    rclcpp::shutdown();
    return 0;
}
