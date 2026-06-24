#include <ament_index_cpp/get_package_share_directory.hpp>
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
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <cstdint>
#include <limits>
#include <numeric>
#include <sstream>
#include <string>
#include <thread>
#include <unordered_map>
#include <unordered_set>
#include <unistd.h>
#include <vector>

namespace {

inline constexpr char kGuideDatasetVersion[] = "guide_ranking_simple_v20260317";
inline constexpr char kAblationVersion[] = "learned_guidance_simple_v20260317";
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

struct TestPoint {
    std::string name;
    std::string difficulty;
    std::string description;
    double difficulty_score{0.0};
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Pose pose;
};

struct LinearFeatureSpec {
    std::string name;
    double weight{0.0};
    double mean{0.0};
    double std{1.0};
};

struct LinearRankingModel {
    std::string model_path;
    std::string target_name;
    double bias{0.0};
    std::vector<LinearFeatureSpec> features;
};

struct LearnedRankingPolicy {
    int top_k{1};
    double probability_threshold{0.55};
    bool fallback_to_heuristic_best{true};
    std::string selection_mode{"top_prob"};
    double hybrid_alpha{2.0};
    std::string retained_order{"heuristic"};
    std::string direct_gate_mode{"off"};
};

struct RankingFeatureContext {
    double difficulty_score{0.0};
    double planning_budget_ms{0.0};
    bool direct_success{false};
    double direct_wall_time_ms{0.0};
    double direct_moveit_time_ms{0.0};
    int direct_planner_calls{0};
    bool direct_hit_budget{false};
};

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

    RCLCPP_INFO(logger, "自适应椭球采样已启用，默认复用 guide scene difficulty_score");
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

std::filesystem::path guideModelSchemaRoot() {
    return std::filesystem::path(
               ament_index_cpp::get_package_share_directory("my_cr5_control")) /
           "config" / "guide_model_schema";
}

std::string normalizeCsvCell(std::string cell) {
    if (cell.size() >= 3 &&
        static_cast<unsigned char>(cell[0]) == 0xEF &&
        static_cast<unsigned char>(cell[1]) == 0xBB &&
        static_cast<unsigned char>(cell[2]) == 0xBF) {
        cell.erase(0, 3);
    }

    const auto is_space = [](unsigned char ch) {
        return std::isspace(ch) != 0;
    };

    while (!cell.empty() && is_space(static_cast<unsigned char>(cell.front()))) {
        cell.erase(cell.begin());
    }
    while (!cell.empty() && is_space(static_cast<unsigned char>(cell.back()))) {
        cell.pop_back();
    }
    return cell;
}

std::vector<std::string> splitCsvLine(const std::string& line) {
    std::vector<std::string> cells;
    std::stringstream ss(line);
    std::string cell;
    while (std::getline(ss, cell, ',')) {
        cells.push_back(normalizeCsvCell(cell));
    }
    return cells;
}

const std::unordered_set<std::string>& knownGuideFeatureNames() {
    static const std::unordered_set<std::string> features = []() {
        const auto path = guideModelSchemaRoot() / "feature_profiles.csv";
        std::ifstream handle(path);
        if (!handle.is_open()) {
            throw std::runtime_error("无法打开 guide feature schema: " + path.string());
        }

        std::unordered_set<std::string> names;
        std::string line;
        bool is_header = true;
        while (std::getline(handle, line)) {
            if (line.empty()) {
                continue;
            }
            if (is_header) {
                is_header = false;
                continue;
            }
            const auto cells = splitCsvLine(line);
            if (cells.size() < 3) {
                continue;
            }
            names.insert(cells[2]);
        }
        return names;
    }();
    return features;
}

const std::unordered_map<std::string, bool>& knownGuideTargetNames() {
    static const std::unordered_map<std::string, bool> targets = []() {
        const auto path = guideModelSchemaRoot() / "targets.csv";
        std::ifstream handle(path);
        if (!handle.is_open()) {
            throw std::runtime_error("无法打开 guide target schema: " + path.string());
        }

        std::unordered_map<std::string, bool> names;
        std::string line;
        bool is_header = true;
        while (std::getline(handle, line)) {
            if (line.empty()) {
                continue;
            }
            if (is_header) {
                is_header = false;
                continue;
            }
            const auto cells = splitCsvLine(line);
            if (cells.size() < 3) {
                continue;
            }
            names[cells[0]] = (cells[2] == "1");
        }
        return names;
    }();
    return targets;
}

int envToInt(const char* name, int default_value, int min_value = 1, int max_value = 100000) {
    return my_cr5_control::env::getIntClamped(name, default_value, min_value, max_value);
}

double envToDouble(const char* name, double default_value, double min_value = 0.0, double max_value = 1e9) {
    return my_cr5_control::env::getDoubleClamped(name, default_value, min_value, max_value);
}

std::string envToString(const char* name, const std::string& default_value) {
    return my_cr5_control::env::getString(name, default_value);
}

bool tryParseUint32(const std::string& raw, std::uint32_t* value) {
    if (raw.empty() || value == nullptr) {
        return false;
    }

    const auto parsed = my_cr5_control::env::parseUint32(raw.c_str());
    if (!parsed.has_value()) {
        return false;
    }
    *value = *parsed;
    return true;
}

std::vector<std::string> splitCommaList(const std::string& raw) {
    std::vector<std::string> items;
    std::stringstream ss(raw);
    std::string item;
    while (std::getline(ss, item, ',')) {
        item.erase(std::remove_if(item.begin(), item.end(), ::isspace), item.end());
        if (!item.empty()) {
            items.push_back(item);
        }
    }
    return items;
}

std::uint32_t hashSceneName(const std::string& raw) {
    std::uint32_t hash = 2166136261u;
    for (const unsigned char ch : raw) {
        hash ^= static_cast<std::uint32_t>(ch);
        hash *= 16777619u;
    }
    return hash;
}

std::uint32_t deriveGuideScenarioSeed(std::uint32_t base_seed,
                                      int repeat_index,
                                      const std::string& scene_name) {
    const std::uint32_t repeat_mix =
        static_cast<std::uint32_t>(repeat_index) * 2654435761u;
    return base_seed ^ repeat_mix ^ hashSceneName(scene_name);
}

std::vector<TestPoint> generateSimpleTestPoints(const SimpleBox& box) {
    std::vector<TestPoint> points;

    {
        TestPoint p;
        p.name = "Easy_TopCenter";
        p.difficulty = "easy";
        p.description = "箱体顶部中心（开放空间）";
        p.difficulty_score = 0.3;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = box.center_z + box.height + 0.05;
        p.tip_point = tip_point;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);
        points.push_back(p);
    }

    {
        TestPoint p;
        p.name = "Medium_SideSurface";
        p.difficulty = "medium";
        p.description = "箱体侧面（需要特定姿态）";
        p.difficulty_score = 0.5;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x - 0.04;
        tip_point.y = box.center_y + box.depth / 2 + 0.01;
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

    {
        TestPoint p;
        p.name = "MediumPlus_RightUpperAngled";
        p.difficulty = "medium";
        p.description = "箱体右侧上沿（侧向斜入）";
        p.difficulty_score = 0.6;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x + box.width / 2 + 0.01;
        tip_point.y = box.center_y;
        tip_point.z = box.center_z + box.height * 0.75;
        p.tip_point = tip_point;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = -0.9238795;
        orientation.z = 0.0;
        orientation.w = 0.3826834;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);
        points.push_back(p);
    }

    {
        TestPoint p;
        p.name = "Hard_HoleShallow";
        p.difficulty = "hard";
        p.description = "孔内部浅层（狭窄空间）";
        p.difficulty_score = 0.7;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = box.center_z + box.height - 0.04;
        p.tip_point = tip_point;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = 1.0;
        orientation.z = 0.0;
        orientation.w = 0.0;
        p.pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);
        points.push_back(p);
    }

    {
        TestPoint p;
        p.name = "HardPlus_HoleEdgeOffset";
        p.difficulty = "hard";
        p.description = "孔口边缘偏置（入孔余量更小）";
        p.difficulty_score = 0.8;
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

    {
        TestPoint p;
        p.name = "Extreme_HoleDeep";
        p.difficulty = "extreme";
        p.description = "孔内部深层（极度狭窄）";
        p.difficulty_score = 0.9;
        geometry_msgs::msg::Point tip_point;
        tip_point.x = box.center_x;
        tip_point.y = box.center_y;
        tip_point.z = box.center_z + box.height - 0.10;
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

void setupSimpleBenchmarkScene(const rclcpp::Logger& logger, const SimpleBox& box) {
    moveit::planning_interface::PlanningSceneInterface planning_scene;

    moveit_msgs::msg::CollisionObject floor_obj;
    floor_obj.header.frame_id = "base_link";
    floor_obj.id = "floor";

    shape_msgs::msg::SolidPrimitive floor_prim;
    floor_prim.type = floor_prim.BOX;
    floor_prim.dimensions = {10.0, 10.0, 1.0};

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.orientation.w = 1.0;
    floor_pose.position.z = -0.51;

    floor_obj.primitives.push_back(floor_prim);
    floor_obj.primitive_poses.push_back(floor_pose);
    floor_obj.operation = floor_obj.ADD;
    planning_scene.applyCollisionObject(floor_obj);

    const double wall_thickness = 0.02;
    const double hole_size = box.hole_radius * 2.0;

    auto apply_box = [&](const std::string& id,
                         double dx,
                         double dy,
                         double dz,
                         double px,
                         double py,
                         double pz) {
        moveit_msgs::msg::CollisionObject object;
        object.header.frame_id = "base_link";
        object.id = id;

        shape_msgs::msg::SolidPrimitive prim;
        prim.type = prim.BOX;
        prim.dimensions = {dx, dy, dz};

        geometry_msgs::msg::Pose pose;
        pose.orientation.w = 1.0;
        pose.position.x = px;
        pose.position.y = py;
        pose.position.z = pz;

        object.primitives.push_back(prim);
        object.primitive_poses.push_back(pose);
        object.operation = object.ADD;
        planning_scene.applyCollisionObject(object);
    };

    apply_box("box_bottom", box.width, box.depth, wall_thickness, box.center_x, box.center_y,
              box.center_z + wall_thickness / 2.0);
    apply_box("box_front", box.width, wall_thickness, box.height, box.center_x,
              box.center_y + box.depth / 2.0 - wall_thickness / 2.0, box.center_z + box.height / 2.0);
    apply_box("box_back", box.width, wall_thickness, box.height, box.center_x,
              box.center_y - box.depth / 2.0 + wall_thickness / 2.0, box.center_z + box.height / 2.0);
    apply_box("box_left", wall_thickness, box.depth, box.height,
              box.center_x - box.width / 2.0 + wall_thickness / 2.0, box.center_y, box.center_z + box.height / 2.0);
    apply_box("box_right", wall_thickness, box.depth, box.height,
              box.center_x + box.width / 2.0 - wall_thickness / 2.0, box.center_y, box.center_z + box.height / 2.0);
    apply_box("box_top_front", box.width, (box.depth - hole_size) / 2.0, wall_thickness, box.center_x,
              box.center_y + hole_size / 2.0 + (box.depth - hole_size) / 4.0,
              box.center_z + box.height - wall_thickness / 2.0);
    apply_box("box_top_back", box.width, (box.depth - hole_size) / 2.0, wall_thickness, box.center_x,
              box.center_y - hole_size / 2.0 - (box.depth - hole_size) / 4.0,
              box.center_z + box.height - wall_thickness / 2.0);
    apply_box("box_top_left", (box.width - hole_size) / 2.0, hole_size, wall_thickness,
              box.center_x - hole_size / 2.0 - (box.width - hole_size) / 4.0,
              box.center_y, box.center_z + box.height - wall_thickness / 2.0);
    apply_box("box_top_right", (box.width - hole_size) / 2.0, hole_size, wall_thickness,
              box.center_x + hole_size / 2.0 + (box.width - hole_size) / 4.0,
              box.center_y, box.center_z + box.height - wall_thickness / 2.0);

    RCLCPP_INFO(logger, "✓ simple benchmark 场景已创建");
    std::this_thread::sleep_for(std::chrono::seconds(2));
}

bool keepPoint(const TestPoint& point, const std::vector<std::string>& scene_filter) {
    if (scene_filter.empty()) {
        return true;
    }
    return std::find(scene_filter.begin(), scene_filter.end(), point.name) != scene_filter.end();
}

void writeGuideDatasetHeader(std::ostream& os) {
    os << "数据集版本,实验时间戳,重复序号,场景UID,场景名称,难度,难度评分,测点描述,"
       << "基础规划器,规划预算(s),guide_index,"
       << "尖端X,尖端Y,尖端Z,法兰X,法兰Y,法兰Z,"
       << "guide_x,guide_y,guide_z,"
       << "heuristic_cost,direct_cost,cost_delta_to_direct,"
       << "start_to_guide_distance,guide_to_goal_distance,total_guide_distance,direct_distance,"
       << "axial_progress,lateral_offset,guide_penalty,mid1_penalty,mid2_penalty,guide_height,"
       << "clearance_margin,manipulability_score,safety_penalty,ik_feasible,"
       << "direct_success,direct_wall_time_ms,direct_moveit_time_ms,direct_planner_calls,direct_hit_budget,"
       << "guided_success,guided_wall_time_ms,guided_moveit_time_ms,guided_planner_calls,guided_hit_budget,"
       << "candidate_viable,candidate_fast,candidate_preferred\n";
}

void writeAblationHeader(std::ostream& os) {
    os << "实验版本,实验时间戳,重复序号,模式,ranking_name,model_path,"
       << "场景名称,难度,难度评分,测点描述,"
       << "尖端X,尖端Y,尖端Z,法兰X,法兰Y,法兰Z,"
       << "成功,墙钟时间(ms),MoveIt规划时间(ms),预算上限(ms),规划调用次数,触发预算上限,快速求解(<1s),"
       << "reused_direct_baseline,"
       << "guide_sample_count,guide_candidate_count,guide_candidates_attempted,used_direct_plan,"
       << "selected_candidate_id,selected_candidate_probability,selected_candidate_heuristic_cost,"
       << "selected_candidate_ranking_score,selected_guide_x,selected_guide_y,selected_guide_z\n";
}

void writeAblationSummaryHeader(std::ostream& os) {
    os << "实验版本,实验时间戳,模式,ranking_name,样本数,成功率(%),平均时间(ms),中位时间(ms),"
       << "预算命中数,预算命中率(%),快速求解数,快速求解率(%)\n";
}

double featureValue(const RankingFeatureContext& context,
                    const CR5Robot::GuideCandidate& candidate,
                    const std::string& feature_name) {
    if (feature_name == "heuristic_cost") return candidate.heuristic_cost;
    if (feature_name == "direct_cost") return candidate.direct_cost;
    if (feature_name == "cost_delta_to_direct") return candidate.cost_delta_to_direct;
    if (feature_name == "start_to_guide_distance") return candidate.start_to_guide_distance;
    if (feature_name == "guide_to_goal_distance") return candidate.guide_to_goal_distance;
    if (feature_name == "total_guide_distance") return candidate.total_guide_distance;
    if (feature_name == "direct_distance") return candidate.direct_distance;
    if (feature_name == "axial_progress") return candidate.axial_progress;
    if (feature_name == "lateral_offset") return candidate.lateral_offset;
    if (feature_name == "guide_penalty") return candidate.guide_penalty;
    if (feature_name == "mid1_penalty") return candidate.mid1_penalty;
    if (feature_name == "mid2_penalty") return candidate.mid2_penalty;
    if (feature_name == "guide_height") return candidate.guide_height;
    if (feature_name == "clearance_margin") return candidate.clearance_margin;
    if (feature_name == "manipulability_score") return candidate.manipulability_score;
    if (feature_name == "safety_penalty") return candidate.safety_penalty;
    if (feature_name == "ik_feasible") return candidate.ik_feasible ? 1.0 : 0.0;
    if (feature_name == "difficulty_score_raw") return context.difficulty_score;
    if (feature_name == "direct_success_flag") return context.direct_success ? 1.0 : 0.0;
    if (feature_name == "direct_hit_budget_flag") return context.direct_hit_budget ? 1.0 : 0.0;
    if (feature_name == "direct_bad_flag") return (!context.direct_success || context.direct_hit_budget) ? 1.0 : 0.0;
    if (feature_name == "direct_wall_time_ms") return context.direct_wall_time_ms;
    if (feature_name == "direct_moveit_time_ms") return context.direct_moveit_time_ms;
    if (feature_name == "direct_planner_calls") return static_cast<double>(context.direct_planner_calls);
    if (feature_name == "direct_wall_time_ratio") {
        return context.planning_budget_ms <= 1e-9 ? 0.0 : context.direct_wall_time_ms / context.planning_budget_ms;
    }
    if (feature_name == "direct_moveit_time_ratio") {
        return context.planning_budget_ms <= 1e-9 ? 0.0 : context.direct_moveit_time_ms / context.planning_budget_ms;
    }
    return 0.0;
}

LinearRankingModel loadLinearRankingModel(const std::string& model_path) {
    std::ifstream handle(model_path);
    if (!handle.is_open()) {
        throw std::runtime_error("无法打开模型文件: " + model_path);
    }

    LinearRankingModel model;
    model.model_path = model_path;
    std::string line;
    bool is_header = true;
    while (std::getline(handle, line)) {
        if (line.empty()) {
            continue;
        }
        if (is_header) {
            is_header = false;
            continue;
        }

        const auto cells = splitCsvLine(line);
        if (cells.size() < 6) {
            continue;
        }

        const std::string& kind = cells[0];
        const std::string& feature_name = cells[1];
        const std::string& weight_str = cells[2];
        const std::string& mean_str = cells[3];
        const std::string& std_str = cells[4];
        const std::string& target_name = cells[5];

        if (kind == "bias") {
            model.bias = std::stod(weight_str);
            model.target_name = target_name;
            continue;
        }

        if (kind == "feature") {
            LinearFeatureSpec spec;
            spec.name = feature_name;
            spec.weight = std::stod(weight_str);
            spec.mean = mean_str.empty() ? 0.0 : std::stod(mean_str);
            spec.std = std_str.empty() ? 1.0 : std::max(1e-9, std::stod(std_str));
            model.features.push_back(spec);
        }
    }

    if (model.features.empty()) {
        throw std::runtime_error("模型文件中没有 feature 行: " + model_path);
    }

    const auto& known_targets = knownGuideTargetNames();
    const auto target_it = known_targets.find(model.target_name);
    if (target_it == known_targets.end()) {
        throw std::runtime_error("模型 target 不在共享 schema 中: " + model.target_name);
    }
    if (!target_it->second) {
        throw std::runtime_error("模型 target 当前未标记为 online supported: " + model.target_name);
    }

    const auto& known_features = knownGuideFeatureNames();
    for (const auto& feature : model.features) {
        if (known_features.find(feature.name) == known_features.end()) {
            throw std::runtime_error("模型 feature 不在共享 schema 中: " + feature.name);
        }
    }
    return model;
}

double sigmoid(double x) {
    const double clamped = std::clamp(x, -40.0, 40.0);
    return 1.0 / (1.0 + std::exp(-clamped));
}

double evaluateLinearProbability(const LinearRankingModel& model,
                                 const RankingFeatureContext& context,
                                 const CR5Robot::GuideCandidate& candidate) {
    double logit = model.bias;
    for (const auto& feature : model.features) {
        const double value = featureValue(context, candidate, feature.name);
        const double normalized = (value - feature.mean) / feature.std;
        logit += normalized * feature.weight;
    }
    return sigmoid(logit);
}

void applyTopKLearnedGating(const LinearRankingModel& model,
                            const LearnedRankingPolicy& policy,
                            const RankingFeatureContext& context,
                            std::vector<CR5Robot::GuideCandidate>& candidates) {
    struct CandidateScore {
        std::size_t index{0};
        double probability{0.0};
        double heuristic_cost{0.0};
        double heuristic_norm{0.0};
        double clearance_margin{0.0};
    };

    std::vector<CandidateScore> scored;
    scored.reserve(candidates.size());
    std::size_t heuristic_best_index = 0;
    double heuristic_best_cost = std::numeric_limits<double>::infinity();
    double heuristic_min = std::numeric_limits<double>::infinity();
    double heuristic_max = 0.0;

    for (std::size_t index = 0; index < candidates.size(); ++index) {
        auto& candidate = candidates[index];
        const double probability = evaluateLinearProbability(model, context, candidate);
        candidate.learned_probability = probability;
        scored.push_back({index, probability, candidate.heuristic_cost, 0.0, candidate.clearance_margin});

        candidate.enabled = false;
        candidate.ranking_score = std::numeric_limits<double>::infinity();

        heuristic_min = std::min(heuristic_min, candidate.heuristic_cost);
        heuristic_max = std::max(heuristic_max, candidate.heuristic_cost);
        if (candidate.heuristic_cost < heuristic_best_cost) {
            heuristic_best_cost = candidate.heuristic_cost;
            heuristic_best_index = index;
        }
    }

    const double heuristic_span = std::max(1e-9, heuristic_max - heuristic_min);
    for (auto& item : scored) {
        item.heuristic_norm = (item.heuristic_cost - heuristic_min) / heuristic_span;
    }

    std::stable_sort(scored.begin(), scored.end(),
                     [&](const CandidateScore& lhs, const CandidateScore& rhs) {
                         if (policy.selection_mode == "heuristic_gate") {
                             if (lhs.heuristic_cost != rhs.heuristic_cost) {
                                 return lhs.heuristic_cost < rhs.heuristic_cost;
                             }
                             return lhs.probability > rhs.probability;
                         }

                         if (policy.selection_mode == "hybrid") {
                             const double lhs_score =
                                 lhs.heuristic_norm - policy.hybrid_alpha * lhs.probability;
                             const double rhs_score =
                                 rhs.heuristic_norm - policy.hybrid_alpha * rhs.probability;
                             if (lhs_score != rhs_score) {
                                 return lhs_score < rhs_score;
                             }
                             if (lhs.heuristic_cost != rhs.heuristic_cost) {
                                 return lhs.heuristic_cost < rhs.heuristic_cost;
                             }
                             return lhs.probability > rhs.probability;
                         }

                         if (lhs.probability != rhs.probability) {
                             return lhs.probability > rhs.probability;
                         }
                         return lhs.heuristic_cost < rhs.heuristic_cost;
                     });

    std::vector<CandidateScore> retained;
    retained.reserve(static_cast<std::size_t>(std::max(1, policy.top_k)));
    for (const auto& item : scored) {
        if (static_cast<int>(retained.size()) >= policy.top_k) {
            break;
        }
        if (item.probability < policy.probability_threshold) {
            continue;
        }
        retained.push_back(item);
    }

    auto retainedComparator = [&](const CandidateScore& lhs, const CandidateScore& rhs) {
        if (policy.retained_order == "learned") {
            if (lhs.probability != rhs.probability) {
                return lhs.probability > rhs.probability;
            }
            return lhs.heuristic_cost < rhs.heuristic_cost;
        }

        if (policy.retained_order == "hybrid") {
            const double lhs_score = lhs.heuristic_cost - 0.10 * lhs.probability - 0.05 * lhs.clearance_margin;
            const double rhs_score = rhs.heuristic_cost - 0.10 * rhs.probability - 0.05 * rhs.clearance_margin;
            if (lhs_score != rhs_score) {
                return lhs_score < rhs_score;
            }
            if (lhs.heuristic_cost != rhs.heuristic_cost) {
                return lhs.heuristic_cost < rhs.heuristic_cost;
            }
            return lhs.probability > rhs.probability;
        }

        if (lhs.heuristic_cost != rhs.heuristic_cost) {
            return lhs.heuristic_cost < rhs.heuristic_cost;
        }
        return lhs.probability > rhs.probability;
    };

    std::stable_sort(retained.begin(), retained.end(), retainedComparator);

    for (std::size_t rank = 0; rank < retained.size(); ++rank) {
        auto& candidate = candidates[retained[rank].index];
        candidate.enabled = true;
        candidate.ranking_score = static_cast<double>(rank) * 1000.0 + candidate.heuristic_cost;
    }

    if (retained.empty() && policy.fallback_to_heuristic_best && !candidates.empty()) {
        auto& fallback = candidates[heuristic_best_index];
        fallback.enabled = true;
        fallback.ranking_score = fallback.heuristic_cost;
    }
}

double quantile(std::vector<double> values, double q) {
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

std::string optionalIntField(int value) {
    if (value < 0) {
        return "";
    }
    return std::to_string(value);
}

std::string optionalDoubleField(double value, int precision = 6, bool has_value = true) {
    if (!has_value || !std::isfinite(value)) {
        return "";
    }
    std::ostringstream oss;
    oss << std::fixed << std::setprecision(precision) << value;
    return oss.str();
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("guide_ranking_simple_experiment");

    const std::string mode = envToString("MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE", "collect_dataset");
    const std::string base_planner = envToString("MY_CR5_CONTROL_GUIDE_BASE_PLANNER", "RRTConnect");
    const int repeat_count = envToInt("MY_CR5_CONTROL_GUIDE_REPEATS", mode == "ablation" ? 2 : 2, 1, 100);
    const int sample_count = envToInt("MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT", mode == "ablation" ? 24 : 12, 1, 128);
    const double planning_budget_s = envToDouble("MY_CR5_CONTROL_GUIDE_BUDGET_S", 4.0, 0.5, 30.0);
    const std::vector<std::string> scene_filter = splitCommaList(envToString("MY_CR5_CONTROL_GUIDE_SCENES", ""));
    const std::string model_path = envToString("MY_CR5_CONTROL_GUIDE_MODEL_PATH", "");
    const std::string guide_sample_seed_raw = envToString("MY_CR5_CONTROL_GUIDE_SAMPLE_SEED", "");
    const bool reuse_direct_baseline = envToInt("MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE", 1, 0, 1) != 0;
    const LearnedRankingPolicy learned_policy{
        envToInt("MY_CR5_CONTROL_GUIDE_MODEL_TOP_K", 1, 1, 8),
        envToDouble("MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD", 0.55, 0.0, 1.0),
        true,
        envToString("MY_CR5_CONTROL_GUIDE_SELECTION_MODE", "top_prob"),
        envToDouble("MY_CR5_CONTROL_GUIDE_HYBRID_ALPHA", 2.0, 0.0, 100.0),
        envToString("MY_CR5_CONTROL_GUIDE_RETAINED_ORDER", "heuristic"),
        envToString("MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE", "off")};
    std::uint32_t guide_sample_seed = 0;
    const bool has_guide_sample_seed = tryParseUint32(guide_sample_seed_raw, &guide_sample_seed);

    const std::string robot_node_name =
        "guide_ranking_simple_experiment_node_" + std::to_string(static_cast<long long>(getpid()));
    CR5Robot robot(robot_node_name);
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        return 1;
    }
    const auto adaptive_ellipsoid_config =
        my_cr5_control::benchmarks::getAdaptiveEllipsoidEnvConfig();
    robot.enableAdaptiveEllipsoidSampling(adaptive_ellipsoid_config.enabled);
    logAdaptiveEllipsoidConfig(logger, adaptive_ellipsoid_config);
    robot.setPlanner(base_planner);

    SimpleBox box;
    setupSimpleBenchmarkScene(logger, box);
    robot.setGuideEnvironmentBoxHint(
        box.center_x, box.center_y, box.center_z + box.height * 0.5, box.width, box.depth, box.height);
    const auto all_points = generateSimpleTestPoints(box);
    std::vector<TestPoint> test_points;
    for (const auto& point : all_points) {
        if (keepPoint(point, scene_filter)) {
            test_points.push_back(point);
        }
    }
    if (test_points.empty()) {
        RCLCPP_ERROR(logger, "没有匹配的测试场景");
        return 1;
    }

    if (has_guide_sample_seed) {
        RCLCPP_INFO(logger,
                    "guide candidate 采样已固定 seed: %u (同一 repeat/scene 下 heuristic 与 learned 复用同一候选流)",
                    guide_sample_seed);
    }
    if (!scene_filter.empty()) {
        RCLCPP_INFO(logger, "当前固定场景子集: %s", envToString("MY_CR5_CONTROL_GUIDE_SCENES", "").c_str());
    }

    const std::string timestamp = my_cr5_control::results::makeTimestamp();

    auto resetGuideSamplingSeedForScenario =
        [&](int repeat_index, const TestPoint& point) {
            if (!has_guide_sample_seed) {
                robot.clearGuideSamplingSeed();
                return;
            }
            robot.setGuideSamplingSeed(
                deriveGuideScenarioSeed(guide_sample_seed, repeat_index, point.name));
        };

    if (mode == "collect_dataset") {
        const std::string output_path =
            my_cr5_control::results::makeOutputPath(timestamp, "guide_ranking_simple_dataset_results.csv");
        std::ofstream csv_file(output_path);
        writeGuideDatasetHeader(csv_file);

        RCLCPP_INFO(logger, "开始采集 guide candidate 数据集: repeats=%d sample_count=%d budget=%.1fs",
                    repeat_count, sample_count, planning_budget_s);

        for (int repeat_index = 1; repeat_index <= repeat_count; ++repeat_index) {
            for (const auto& point : test_points) {
                CR5Robot::PlanningMetrics direct_metrics;
                const bool direct_success = robot.planToPoseWithPlanner(
                    point.pose, base_planner, planning_budget_s, "home", &direct_metrics);
                const std::string scenario_uid = timestamp + ":" + std::to_string(repeat_index) + ":" + point.name;

                resetGuideSamplingSeedForScenario(repeat_index, point);
                applyAdaptiveEllipsoidForDifficulty(
                    robot, adaptive_ellipsoid_config, point.difficulty_score);
                const auto candidates = robot.sampleGuideCandidates(point.pose, "home", sample_count, false);
                for (std::size_t guide_index = 0; guide_index < candidates.size(); ++guide_index) {
                    const auto& candidate = candidates[guide_index];
                    CR5Robot::PlanningMetrics guided_metrics;
                    const bool guided_success = robot.planToPoseViaGuide(
                        point.pose, candidate.pose, "home", planning_budget_s, &guided_metrics);

                    const bool candidate_viable = guided_success && !guided_metrics.hit_budget_limit;
                    const bool candidate_fast = guided_success && guided_metrics.wall_time_ms < kFastSolveThresholdMs;
                    const bool candidate_preferred =
                        guided_success &&
                        ((!direct_success) ||
                         (!guided_metrics.hit_budget_limit && direct_metrics.hit_budget_limit) ||
                         (direct_success && guided_metrics.wall_time_ms + 50.0 < direct_metrics.wall_time_ms));

                    csv_file << kGuideDatasetVersion << ","
                             << timestamp << ","
                             << repeat_index << ","
                             << scenario_uid << ","
                             << point.name << ","
                             << point.difficulty << ","
                             << std::fixed << std::setprecision(2) << point.difficulty_score << ","
                             << "\"" << point.description << "\","
                             << base_planner << ","
                             << std::setprecision(1) << planning_budget_s << ","
                             << guide_index << ","
                             << std::setprecision(4)
                             << point.tip_point.x << "," << point.tip_point.y << "," << point.tip_point.z << ","
                             << point.pose.position.x << "," << point.pose.position.y << "," << point.pose.position.z << ","
                             << candidate.pose.position.x << "," << candidate.pose.position.y << "," << candidate.pose.position.z << ","
                             << std::setprecision(6)
                             << candidate.heuristic_cost << "," << candidate.direct_cost << "," << candidate.cost_delta_to_direct << ","
                             << candidate.start_to_guide_distance << "," << candidate.guide_to_goal_distance << ","
                             << candidate.total_guide_distance << "," << candidate.direct_distance << ","
                             << candidate.axial_progress << "," << candidate.lateral_offset << ","
                             << candidate.guide_penalty << "," << candidate.mid1_penalty << "," << candidate.mid2_penalty << ","
                             << candidate.guide_height << ","
                             << candidate.clearance_margin << "," << candidate.manipulability_score << ","
                             << candidate.safety_penalty << "," << (candidate.ik_feasible ? "1" : "0") << ","
                             << (direct_success ? "1" : "0") << ","
                             << std::setprecision(3)
                             << direct_metrics.wall_time_ms << "," << direct_metrics.planner_reported_time_ms << ","
                             << direct_metrics.planner_calls << "," << (direct_metrics.hit_budget_limit ? "1" : "0") << ","
                             << (guided_success ? "1" : "0") << ","
                             << guided_metrics.wall_time_ms << "," << guided_metrics.planner_reported_time_ms << ","
                             << guided_metrics.planner_calls << "," << (guided_metrics.hit_budget_limit ? "1" : "0") << ","
                             << (candidate_viable ? "1" : "0") << ","
                             << (candidate_fast ? "1" : "0") << ","
                             << (candidate_preferred ? "1" : "0") << "\n";
                }
            }
        }

        csv_file.close();
        RCLCPP_INFO(logger, "guide candidate 数据集已写出: %s", output_path.c_str());
    } else if (mode == "ablation") {
        if (model_path.empty()) {
            RCLCPP_ERROR(logger, "ablation 模式需要设置 MY_CR5_CONTROL_GUIDE_MODEL_PATH");
            return 1;
        }

        const LinearRankingModel model = loadLinearRankingModel(model_path);
        const std::string result_path =
            my_cr5_control::results::makeOutputPath(timestamp, "learned_guidance_simple_ablation_results.csv");
        const std::string summary_path =
            my_cr5_control::results::makeOutputPath(timestamp, "learned_guidance_simple_ablation_summary.csv");
        std::ofstream result_file(result_path);
        std::ofstream summary_file(summary_path);
        writeAblationHeader(result_file);
        writeAblationSummaryHeader(summary_file);

        RCLCPP_INFO(logger,
                    "learned ablation ranking policy: top-k gating + retained re-rank, sample_count=%d top_k=%d threshold=%.2f selection_mode=%s hybrid_alpha=%.2f retained_order=%s direct_gate_mode=%s reuse_direct_baseline=%s target=%s",
                    sample_count,
                    learned_policy.top_k,
                    learned_policy.probability_threshold,
                    learned_policy.selection_mode.c_str(),
                    learned_policy.hybrid_alpha,
                    learned_policy.retained_order.c_str(),
                    learned_policy.direct_gate_mode.c_str(),
                    reuse_direct_baseline ? "true" : "false",
                    model.target_name.c_str());

        struct AblationRow {
            std::string mode;
            std::string ranking_name;
            TestPoint point;
            CR5Robot::PlanningMetrics metrics;
            bool reused_direct_baseline{false};
        };
        std::vector<AblationRow> rows;

        const std::string learned_ranking_name =
            "learned_" + learned_policy.selection_mode + "_topk_" +
            learned_policy.retained_order + "_k" + std::to_string(learned_policy.top_k) +
            (learned_policy.direct_gate_mode == "off" ? "" : "_gate_" + learned_policy.direct_gate_mode);

        for (int repeat_index = 1; repeat_index <= repeat_count; ++repeat_index) {
            for (const auto& point : test_points) {
                applyAdaptiveEllipsoidForDifficulty(
                    robot, adaptive_ellipsoid_config, point.difficulty_score);
                robot.clearGuideRankingFunction();
                robot.setGuideDirectCostGateEnabled(false);
                CR5Robot::PlanningMetrics no_guidance_metrics;
                robot.planToPoseWithPlanner(
                    point.pose, base_planner, planning_budget_s, "home", &no_guidance_metrics);
                rows.push_back({"no_guidance", "direct_" + base_planner, point, no_guidance_metrics, false});

                robot.clearGuideRankingFunction();
                robot.setGuideDirectCostGateEnabled(false);
                CR5Robot::PlanningMetrics heuristic_metrics;
                resetGuideSamplingSeedForScenario(repeat_index, point);
                robot.planToPoseImproved(point.pose,
                                         "home",
                                         planning_budget_s,
                                         &heuristic_metrics,
                                         sample_count,
                                         reuse_direct_baseline ? &no_guidance_metrics : nullptr);
                rows.push_back({"heuristic_guided", "heuristic_cost", point, heuristic_metrics,
                                reuse_direct_baseline});

                const RankingFeatureContext learned_context{
                    point.difficulty_score,
                    no_guidance_metrics.planning_budget_ms,
                    no_guidance_metrics.success,
                    no_guidance_metrics.wall_time_ms,
                    no_guidance_metrics.planner_reported_time_ms,
                    no_guidance_metrics.planner_calls,
                    no_guidance_metrics.hit_budget_limit};
                robot.setGuideRankingFunction(
                    [model, learned_policy, learned_context](const geometry_msgs::msg::Pose&,
                                                             const geometry_msgs::msg::Pose&,
                                                             std::vector<CR5Robot::GuideCandidate>& candidates) {
                        applyTopKLearnedGating(model, learned_policy, learned_context, candidates);
                    },
                    learned_ranking_name);
                const bool enable_direct_gate =
                    learned_policy.direct_gate_mode == "beat_direct" ||
                    (learned_policy.direct_gate_mode == "beat_direct_no_budget_hit" &&
                     no_guidance_metrics.success && !no_guidance_metrics.hit_budget_limit);
                robot.setGuideDirectCostGateEnabled(enable_direct_gate);
                CR5Robot::PlanningMetrics learned_metrics;
                resetGuideSamplingSeedForScenario(repeat_index, point);
                robot.planToPoseImproved(point.pose,
                                         "home",
                                         planning_budget_s,
                                         &learned_metrics,
                                         sample_count,
                                         reuse_direct_baseline ? &no_guidance_metrics : nullptr);
                rows.push_back({"learned_guided", learned_ranking_name, point, learned_metrics,
                                reuse_direct_baseline});

                for (const auto& row : std::vector<AblationRow>{
                         rows[rows.size() - 3], rows[rows.size() - 2], rows[rows.size() - 1]}) {
                    result_file << kAblationVersion << ","
                                << timestamp << ","
                                << repeat_index << ","
                                << row.mode << ","
                                << row.ranking_name << ","
                                << model.model_path << ","
                                << row.point.name << ","
                                << row.point.difficulty << ","
                                << std::fixed << std::setprecision(2) << row.point.difficulty_score << ","
                                << "\"" << row.point.description << "\","
                                << std::setprecision(4)
                                << row.point.tip_point.x << "," << row.point.tip_point.y << "," << row.point.tip_point.z << ","
                                << row.point.pose.position.x << "," << row.point.pose.position.y << "," << row.point.pose.position.z << ","
                                << (row.metrics.success ? "成功" : "失败") << ","
                                << std::setprecision(3)
                                << row.metrics.wall_time_ms << "," << row.metrics.planner_reported_time_ms << ","
                                << row.metrics.planning_budget_ms << "," << row.metrics.planner_calls << ","
                                << (row.metrics.hit_budget_limit ? "是" : "否") << ","
                                << (row.metrics.wall_time_ms < kFastSolveThresholdMs ? "是" : "否") << ","
                                << (row.reused_direct_baseline ? "是" : "否") << ","
                                << sample_count << ","
                                << row.metrics.guide_candidate_count << ","
                                << row.metrics.guide_candidates_attempted << ","
                                << (row.metrics.used_direct_plan ? "是" : "否") << ","
                                << optionalIntField(row.metrics.selected_candidate_id) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_learned_probability, 6,
                                                       row.metrics.selected_candidate_learned_probability >= 0.0) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_heuristic_cost, 6,
                                                       row.metrics.selected_candidate_id >= 0) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_ranking_score, 6,
                                                       row.metrics.selected_candidate_id >= 0) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_point.x, 4,
                                                       row.metrics.selected_candidate_id >= 0) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_point.y, 4,
                                                       row.metrics.selected_candidate_id >= 0) << ","
                                << optionalDoubleField(row.metrics.selected_candidate_point.z, 4,
                                                       row.metrics.selected_candidate_id >= 0) << "\n";
                }
            }
        }

        robot.clearGuideRankingFunction();

        for (const auto& mode_name : std::vector<std::string>{"no_guidance", "heuristic_guided", "learned_guided"}) {
            std::vector<double> wall_times;
            int success_count = 0;
            int budget_hit_count = 0;
            int fast_solve_count = 0;
            int sample_count_mode = 0;
            std::string ranking_name = "";

            for (const auto& row : rows) {
                if (row.mode != mode_name) {
                    continue;
                }
                ++sample_count_mode;
                ranking_name = row.ranking_name;
                wall_times.push_back(row.metrics.wall_time_ms);
                if (row.metrics.success) {
                    ++success_count;
                }
                if (row.metrics.hit_budget_limit) {
                    ++budget_hit_count;
                }
                if (row.metrics.wall_time_ms < kFastSolveThresholdMs) {
                    ++fast_solve_count;
                }
            }

            const double mean_time =
                sample_count_mode == 0 ? 0.0 :
                std::accumulate(wall_times.begin(), wall_times.end(), 0.0) / static_cast<double>(sample_count_mode);
            const double median_time = quantile(wall_times, 0.5);

            summary_file << kAblationVersion << ","
                         << timestamp << ","
                         << mode_name << ","
                         << ranking_name << ","
                         << sample_count_mode << ","
                         << std::fixed << std::setprecision(1)
                         << (sample_count_mode == 0 ? 0.0 : 100.0 * success_count / sample_count_mode) << ","
                         << mean_time << ","
                         << median_time << ","
                         << budget_hit_count << ","
                         << (sample_count_mode == 0 ? 0.0 : 100.0 * budget_hit_count / sample_count_mode) << ","
                         << fast_solve_count << ","
                         << (sample_count_mode == 0 ? 0.0 : 100.0 * fast_solve_count / sample_count_mode) << "\n";
        }

        result_file.close();
        summary_file.close();
        RCLCPP_INFO(logger, "ablation 结果已写出: %s", result_path.c_str());
        RCLCPP_INFO(logger, "ablation 摘要已写出: %s", summary_path.c_str());
    } else {
        RCLCPP_ERROR(logger, "不支持的模式: %s", mode.c_str());
        return 1;
    }

    rclcpp::shutdown();
    return 0;
}
