#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"
#include "my_cr5_control/planner_selection_utils.hpp"
#include "my_cr5_control/result_utils.hpp"
#include "my_cr5_control/scene_utils.hpp"
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <cstdlib>
#include <algorithm>
#include <filesystem>
#include <fstream>
#include <chrono>
#include <iomanip>
#include <cmath>
#include <string>
#include <unistd.h>
#include <vector>

namespace {

inline constexpr char kSimpleBenchmarkVersion[] = "simple_v20260311_6scene_metrics";
inline constexpr double kFastSolveThresholdMs = 1000.0;

int getSimpleRepeatCount() {
    return my_cr5_control::env::getIntClamped("MY_CR5_CONTROL_SIMPLE_REPEATS", 1, 1, 1000);
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

}  // namespace

// 简单箱体几何定义
struct SimpleBox {
    double center_x = 0.45;  // 箱体中心X
    double center_y = 0.0;   // 箱体中心Y
    double center_z = 0.15;  // 箱体中心Z（底部高度）
    double width = 0.20;     // X方向宽度
    double depth = 0.20;     // Y方向深度
    double height = 0.20;    // Z方向高度

    // 孔特征
    double hole_radius = 0.04;  // 孔半径4cm
    double hole_depth = 0.12;   // 孔深度12cm
};

// 测点结构
struct TestPoint {
    std::string name;
    std::string difficulty;
    std::string description;
    double difficulty_score;
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Pose pose;
};

// 生成简单的测试点
std::vector<TestPoint> generateSimpleTestPoints(const SimpleBox& box) {
    std::vector<TestPoint> points;

    // 1. 简单场景：箱体顶部中心（开放空间）
    {
        TestPoint p;
        p.name = "Easy_TopCenter";
        p.difficulty = "easy";
        p.description = "箱体顶部中心（开放空间）";
        p.difficulty_score = 0.3;

        double tip_z = box.center_z + box.height + 0.05;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = tip_z;
        p.tip_point = tip_point;

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    // 2. 中等场景：箱体侧面
    {
        TestPoint p;
        p.name = "Medium_SideSurface";
        p.difficulty = "medium";
        p.description = "箱体侧面（需要特定姿态）";
        p.difficulty_score = 0.5;
    
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x - 0.04;
        tip_point.y = box.center_y + box.depth/2 + 0.01;
        tip_point.z = box.center_z + box.height * 0.55;
        p.tip_point = tip_point;

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.7071068;
        orientation.y = 0.0;
        orientation.z = 0.0;
        orientation.w = 0.7071068;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    // 3. 困难场景：孔内部（浅层）
    {
        TestPoint p;
        p.name = "MediumPlus_RightUpperAngled";
        p.difficulty = "medium";
        p.description = "箱体右侧上沿（侧向斜入）";
        p.difficulty_score = 0.6;

        // 保留右侧接触物理意义，但改为上沿斜向下探，避免旧版水平右侧点完全不可达
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x + box.width/2 + 0.01;
        tip_point.y = box.center_y;
        tip_point.z = box.center_z + box.height * 0.75;
        p.tip_point = tip_point;

        // 测针局部 +Z 指向世界 (-X, 0, -Z)，表示从右上外侧斜向接近右侧面
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = -0.9238795;
        orientation.z = 0.0;
        orientation.w = 0.3826834;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    // 4. 困难场景：孔内部（浅层）
    {
        TestPoint p;
        p.name = "Hard_HoleShallow";
        p.difficulty = "hard";
        p.description = "孔内部浅层（狭窄空间）";
        p.difficulty_score = 0.7;

        // 测针尖端目标：孔内4cm深（从顶部向下）
        double tip_z = box.center_z + box.height - 0.04;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = tip_z;
        p.tip_point = tip_point;

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    // 5. 困难增强场景：孔口边缘偏置
    {
        TestPoint p;
        p.name = "HardPlus_HoleEdgeOffset";
        p.difficulty = "hard";
        p.description = "孔口边缘偏置（入孔余量更小）";
        p.difficulty_score = 0.8;

        // 目标偏向孔口边缘，要求在穿过开口时保留更小的横向裕量
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x - 0.03;
        tip_point.y = box.center_y + 0.03;
        tip_point.z = box.center_z + box.height - 0.07;
        p.tip_point = tip_point;

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    // 6. 极端场景：孔内部（深层）
    {
        TestPoint p;
        p.name = "Extreme_HoleDeep";
        p.difficulty = "extreme";
        p.description = "孔内部深层（极度狭窄）";
        p.difficulty_score = 0.9;

        // 测针尖端目标：孔内10cm深（从顶部向下）
        double tip_z = box.center_z + box.height - 0.10;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = tip_z;
        p.tip_point = tip_point;

        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

        points.push_back(p);
    }

    return points;
}

// 测试结果
struct PlannerResult {
    int repeat_index;
    std::string planner_name;
    std::string planning_mode;
    std::string planner_id;
    std::string scenario_name;
    std::string difficulty;
    std::string description;
    double difficulty_score;
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

std::vector<PlannerConfig> getSimplePlannerConfigs(std::vector<std::string>* unknown_names = nullptr) {
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
        {"MY_CR5_CONTROL_SIMPLE_PLANNERS", "MY_CR5_CONTROL_BENCHMARK_PLANNERS"},
        unknown_names);
}

void logSimplePlannerSelection(
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

std::vector<TestPoint> filterSimpleScenes(const std::vector<TestPoint>& points,
                                          std::vector<std::string>* unknown_names = nullptr) {
    const char* raw = my_cr5_control::benchmarks::getFirstPlannerEnvValue(
        {"MY_CR5_CONTROL_SIMPLE_SCENES", "MY_CR5_CONTROL_BENCHMARK_SCENES"});
    if (raw == nullptr) {
        return points;
    }

    const std::vector<std::string> selected_names =
        my_cr5_control::benchmarks::splitCommaTokens(raw);
    std::vector<TestPoint> filtered;
    filtered.reserve(selected_names.size());

    for (const auto& name : selected_names) {
        const auto it = std::find_if(points.begin(), points.end(), [&](const TestPoint& point) {
            return point.name == name;
        });
        if (it == points.end()) {
            if (unknown_names != nullptr &&
                std::find(unknown_names->begin(), unknown_names->end(), name) == unknown_names->end()) {
                unknown_names->push_back(name);
            }
            continue;
        }

        const auto duplicate = std::find_if(filtered.begin(), filtered.end(), [&](const TestPoint& point) {
            return point.name == it->name;
        });
        if (duplicate == filtered.end()) {
            filtered.push_back(*it);
        }
    }

    return filtered;
}

void logSimpleSceneSelection(const rclcpp::Logger& logger,
                             const std::vector<TestPoint>& points,
                             const std::vector<std::string>& unknown_names) {
    std::vector<std::string> scene_names;
    scene_names.reserve(points.size());
    for (const auto& point : points) {
        scene_names.push_back(point.name);
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
    const TestPoint& point,
    int repeat_index,
    const my_cr5_control::benchmarks::AdaptiveEllipsoidEnvConfig& adaptive_ellipsoid_config) {
    PlannerResult result;
    result.repeat_index = repeat_index;
    result.planner_name = planner.name;
    result.planning_mode = (planner.mode == PlanningMode::OMPL) ? "ompl" : "heuristic_guided";
    result.planner_id = planner.planner_id;
    result.scenario_name = point.name;
    result.difficulty = point.difficulty;
    result.description = point.description;
    result.difficulty_score = point.difficulty_score;
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
            point.pose, planner.planner_id, planner.planning_time_s, "home", &metrics);
    } else {
        // HeuristicGuided 当前对应外部两阶段采样引导原型，不走 OMPL planner_id 分发。
        applyAdaptiveEllipsoidForDifficulty(
            robot, adaptive_ellipsoid_config, point.difficulty_score);
        result.success = robot.planToPoseImproved(
            point.pose, "home", planner.planning_time_s, &metrics);
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
    auto logger = rclcpp::get_logger("planner_comparison_simple");

    RCLCPP_INFO(logger, "========================================");
    RCLCPP_INFO(logger, "规划器对比实验 - 简化版（带孔箱体，纯规划）");
    RCLCPP_INFO(logger, "========================================");

    // 初始化机器人
    const std::string robot_node_name =
        "planner_comparison_simple_node_" + std::to_string(static_cast<long long>(getpid()));
    CR5Robot robot(robot_node_name);
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        return 1;
    }
    const auto adaptive_ellipsoid_config =
        my_cr5_control::benchmarks::getAdaptiveEllipsoidEnvConfig();
    robot.enableAdaptiveEllipsoidSampling(adaptive_ellipsoid_config.enabled);
    logAdaptiveEllipsoidConfig(logger, adaptive_ellipsoid_config);

    SimpleBox box;
    robot.setGuideEnvironmentBoxHint(
        box.center_x, box.center_y, box.center_z + box.height * 0.5, box.width, box.depth, box.height);

    // 2. 添加地板
    moveit_msgs::msg::CollisionObject floor_obj;
    floor_obj.header.frame_id = "base_link";
    floor_obj.id = "floor";

    shape_msgs::msg::SolidPrimitive floor_prim;
    floor_prim.type = floor_prim.BOX;
    floor_prim.dimensions = {10.0, 10.0, 1.0};  // 10m x 10m x 1m

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.x = 0.0;
    floor_pose.position.y = 0.0;
    floor_pose.position.z = -0.51;  // 地板顶面在Z=-0.01，避免与底座接触

    floor_obj.primitives.push_back(floor_prim);
    floor_obj.primitive_poses.push_back(floor_pose);
    floor_obj.operation = floor_obj.ADD;

    // 获取planning_scene_interface（需要通过robot对象）
    // 由于CR5Robot没有暴露planning_scene_interface_，我们需要另一种方式
    // 最简单的方法是调用robot的方法来添加地板

    // 临时方案：创建一个临时的planning_scene_interface
    moveit::planning_interface::PlanningSceneInterface planning_scene;
    planning_scene.applyCollisionObject(floor_obj);
    RCLCPP_INFO(logger, "✓ 地板已添加");

    // 3. 创建带孔的箱体（用多个Box组合）
    // 箱体结构：底面 + 4个侧面 + 顶面（顶面中间有孔）

    double wall_thickness = 0.02;  // 壁厚2cm

    // 3.1 底面
    {
        moveit_msgs::msg::CollisionObject bottom;
        bottom.header.frame_id = "base_link";
        bottom.id = "box_bottom";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {box.width, box.depth, wall_thickness};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z + wall_thickness/2;

        bottom.primitives.push_back(prim);
        bottom.primitive_poses.push_back(pose);
        bottom.operation = bottom.ADD;

        planning_scene.applyCollisionObject(bottom);
    }

    // 3.2 前侧面
    {
        moveit_msgs::msg::CollisionObject front;
        front.header.frame_id = "base_link";
        front.id = "box_front";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {box.width, wall_thickness, box.height};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y + box.depth/2 - wall_thickness/2;
        pose.position.z = box.center_z + box.height/2;

        front.primitives.push_back(prim);
        front.primitive_poses.push_back(pose);
        front.operation = front.ADD;

        planning_scene.applyCollisionObject(front);
    }

    // 3.3 后侧面
    {
        moveit_msgs::msg::CollisionObject back;
        back.header.frame_id = "base_link";
        back.id = "box_back";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {box.width, wall_thickness, box.height};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y - box.depth/2 + wall_thickness/2;
        pose.position.z = box.center_z + box.height/2;

        back.primitives.push_back(prim);
        back.primitive_poses.push_back(pose);
        back.operation = back.ADD;

        planning_scene.applyCollisionObject(back);
    }

    // 3.4 左侧面
    {
        moveit_msgs::msg::CollisionObject left;
        left.header.frame_id = "base_link";
        left.id = "box_left";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {wall_thickness, box.depth, box.height};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x - box.width/2 + wall_thickness/2;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z + box.height/2;

        left.primitives.push_back(prim);
        left.primitive_poses.push_back(pose);
        left.operation = left.ADD;

        planning_scene.applyCollisionObject(left);
    }

    // 3.5 右侧面
    {
        moveit_msgs::msg::CollisionObject right;
        right.header.frame_id = "base_link";
        right.id = "box_right";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {wall_thickness, box.depth, box.height};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x + box.width/2 - wall_thickness/2;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z + box.height/2;

        right.primitives.push_back(prim);
        right.primitive_poses.push_back(pose);
        right.operation = right.ADD;

        planning_scene.applyCollisionObject(right);
    }

    // 3.6 顶面（环形，中间有孔）
    // 用4个矩形条组成环形顶面
    double hole_size = box.hole_radius * 2;  // 孔的尺寸

    // 顶面前条
    {
        moveit_msgs::msg::CollisionObject top_front;
        top_front.header.frame_id = "base_link";
        top_front.id = "box_top_front";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {box.width, (box.depth - hole_size)/2, wall_thickness};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y + hole_size/2 + (box.depth - hole_size)/4;
        pose.position.z = box.center_z + box.height - wall_thickness/2;

        top_front.primitives.push_back(prim);
        top_front.primitive_poses.push_back(pose);
        top_front.operation = top_front.ADD;

        planning_scene.applyCollisionObject(top_front);
    }

    // 顶面后条
    {
        moveit_msgs::msg::CollisionObject top_back;
        top_back.header.frame_id = "base_link";
        top_back.id = "box_top_back";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {box.width, (box.depth - hole_size)/2, wall_thickness};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x;
        pose.position.y = box.center_y - hole_size/2 - (box.depth - hole_size)/4;
        pose.position.z = box.center_z + box.height - wall_thickness/2;

        top_back.primitives.push_back(prim);
        top_back.primitive_poses.push_back(pose);
        top_back.operation = top_back.ADD;

        planning_scene.applyCollisionObject(top_back);
    }

    // 顶面左条
    {
        moveit_msgs::msg::CollisionObject top_left;
        top_left.header.frame_id = "base_link";
        top_left.id = "box_top_left";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {(box.width - hole_size)/2, hole_size, wall_thickness};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x - hole_size/2 - (box.width - hole_size)/4;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z + box.height - wall_thickness/2;

        top_left.primitives.push_back(prim);
        top_left.primitive_poses.push_back(pose);
        top_left.operation = top_left.ADD;

        planning_scene.applyCollisionObject(top_left);
    }

    // 顶面右条
    {
        moveit_msgs::msg::CollisionObject top_right;
        top_right.header.frame_id = "base_link";
        top_right.id = "box_top_right";

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {(box.width - hole_size)/2, hole_size, wall_thickness};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = box.center_x + hole_size/2 + (box.width - hole_size)/4;
        pose.position.y = box.center_y;
        pose.position.z = box.center_z + box.height - wall_thickness/2;

        top_right.primitives.push_back(prim);
        top_right.primitive_poses.push_back(pose);
        top_right.operation = top_right.ADD;

        planning_scene.applyCollisionObject(top_right);
    }

    RCLCPP_INFO(logger, "✓ 带孔箱体已创建:");
    RCLCPP_INFO(logger, "  - 箱体尺寸: %.2f x %.2f x %.2f m", box.width, box.depth, box.height);
    RCLCPP_INFO(logger, "  - 孔直径: %.2f m", hole_size);
    RCLCPP_INFO(logger, "  - 孔深度: %.2f m", box.hole_depth);

    std::this_thread::sleep_for(std::chrono::seconds(2));

    // 生成测试点
    auto test_points = generateSimpleTestPoints(box);
    std::vector<std::string> unknown_scene_names;
    test_points = filterSimpleScenes(test_points, &unknown_scene_names);
    if (test_points.empty()) {
        RCLCPP_ERROR(logger, "场景过滤后为空，请检查 MY_CR5_CONTROL_SIMPLE_SCENES / MY_CR5_CONTROL_BENCHMARK_SCENES");
        return 1;
    }
    RCLCPP_INFO(logger, "生成了 %d 个测试点", static_cast<int>(test_points.size()));
    logSimpleSceneSelection(logger, test_points, unknown_scene_names);

    std::vector<std::string> unknown_planner_names;
    const std::vector<PlannerConfig> planners = getSimplePlannerConfigs(&unknown_planner_names);

    const std::string timestamp = my_cr5_control::results::makeTimestamp();
    const int repeat_count = getSimpleRepeatCount();
    RCLCPP_INFO(logger, "基准版本: %s", kSimpleBenchmarkVersion);
    RCLCPP_INFO(logger, "重复次数: %d", repeat_count);
    logSimplePlannerSelection(logger, planners, unknown_planner_names);

    // 构建带时间戳的文件路径
    std::string results_filename =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_simple_results.csv");
    std::string summary_filename =
        my_cr5_control::results::makeOutputPath(timestamp, "planner_comparison_simple_summary.csv");
    const std::string plot_data_filename =
        my_cr5_control::results::makeSharedOutputPath("planner_comparison_simple_plot_data_metrics.csv");
    const std::string plot_summary_filename =
        my_cr5_control::results::makeSharedOutputPath("planner_comparison_simple_plot_summary_metrics.csv");

    // 打开CSV文件
    std::ofstream csv_file(results_filename);
    writeDetailedHeader(csv_file);

    const bool plot_data_exists =
        std::filesystem::exists(plot_data_filename) && std::filesystem::file_size(plot_data_filename) > 0;
    std::ofstream plot_data_file(plot_data_filename, std::ios::app);
    if (!plot_data_exists) {
        writeDetailedHeader(plot_data_file);
    }

    // 运行测试
    std::vector<PlannerResult> all_results;
    auto writeDetailedRow = [&](std::ostream& os, const PlannerResult& result, const TestPoint& point) {
        os << kSimpleBenchmarkVersion << ","
           << timestamp << ","
           << repeat_count << ","
           << result.repeat_index << ","
           << result.planner_name << ","
           << result.planning_mode << ","
           << result.planner_id << ","
           << result.scenario_name << ","
           << result.difficulty << ","
           << "\"" << result.description << "\","
           << std::fixed << std::setprecision(2) << result.difficulty_score << ","
           << std::fixed << std::setprecision(4)
           << point.tip_point.x << ","
           << point.tip_point.y << ","
           << point.tip_point.z << ","
           << point.pose.position.x << ","
           << point.pose.position.y << ","
           << point.pose.position.z << ","
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

        for (const auto& point : test_points) {
            RCLCPP_INFO(logger, "\n>>> 测试场景: %s (%s)",
                        point.name.c_str(), point.difficulty.c_str());
            RCLCPP_INFO(logger, "    测点: %s (难度: %.2f)",
                        point.description.c_str(), point.difficulty_score);
            RCLCPP_INFO(logger, "    尖端目标: (%.3f, %.3f, %.3f)",
                        point.tip_point.x, point.tip_point.y, point.tip_point.z);
            RCLCPP_INFO(logger, "    法兰目标: (%.3f, %.3f, %.3f)",
                        point.pose.position.x, point.pose.position.y, point.pose.position.z);

            for (const auto& planner : planners) {
                RCLCPP_INFO(logger, "  测试规划器: %s", planner.name.c_str());

                // 纯规划 benchmark：固定以 home 命名状态作为起点，不执行轨迹。
                auto result = testPlanner(
                    robot, planner, point, repeat_index, adaptive_ellipsoid_config);
                all_results.push_back(result);

                writeDetailedRow(csv_file, result, point);
                writeDetailedRow(plot_data_file, result, point);

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

    std::ofstream summary_file(summary_filename);
    writeSummaryHeader(summary_file);

    const bool plot_summary_exists =
        std::filesystem::exists(plot_summary_filename) && std::filesystem::file_size(plot_summary_filename) > 0;
    std::ofstream plot_summary_file(plot_summary_filename, std::ios::app);
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

        double easy_time = 0.0, medium_time = 0.0, hard_time = 0.0, extreme_time = 0.0;
        int easy_count = 0, medium_count = 0, hard_count = 0, extreme_count = 0;

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

                if (result.difficulty == "easy") {
                    easy_time += result.wall_time_ms;
                    easy_count++;
                } else if (result.difficulty == "medium") {
                    medium_time += result.wall_time_ms;
                    medium_count++;
                } else if (result.difficulty == "hard") {
                    hard_time += result.wall_time_ms;
                    hard_count++;
                } else if (result.difficulty == "extreme") {
                    extreme_time += result.wall_time_ms;
                    extreme_count++;
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

        RCLCPP_INFO(logger, "%s: 成功率 %.1f%%, 平均时间 %.1f ms",
                    planner.name.c_str(), success_rate, avg_time);

        auto writeSummaryRow = [&](std::ostream& os) {
            os << kSimpleBenchmarkVersion << ","
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
               << (easy_count > 0 ? easy_time / easy_count : 0.0) << ","
               << (medium_count > 0 ? medium_time / medium_count : 0.0) << ","
               << (hard_count > 0 ? hard_time / hard_count : 0.0) << ","
               << (extreme_count > 0 ? extreme_time / extreme_count : 0.0) << ","
               << results_filename << "\n";
        };
        writeSummaryRow(summary_file);
        writeSummaryRow(plot_summary_file);
    }

    summary_file.close();
    plot_summary_file.close();

    RCLCPP_INFO(logger, "\n========================================");
    RCLCPP_INFO(logger, "测试完成");
    RCLCPP_INFO(logger, "详细结果: %s", results_filename.c_str());
    RCLCPP_INFO(logger, "统计摘要: %s", summary_filename.c_str());
    RCLCPP_INFO(logger, "绘图明细: %s", plot_data_filename.c_str());
    RCLCPP_INFO(logger, "绘图摘要: %s", plot_summary_filename.c_str());
    RCLCPP_INFO(logger, "========================================");

    rclcpp::shutdown();
    return 0;
}
