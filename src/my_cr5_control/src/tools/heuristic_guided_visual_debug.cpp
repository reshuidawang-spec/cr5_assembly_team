#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"
#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/paper_mainline/v2_measurement_pose.hpp"
#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/scene_utils.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;
constexpr double kFastSolveThresholdMs = 1000.0;

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

struct SimpleScene {
    std::string name;
    std::string difficulty;
    std::string description;
    double difficulty_score{0.0};
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Pose flange_pose;
};

struct DebugConfig {
    std::string benchmark{"simple"};
    std::string scene{"Medium_SideSurface"};
    std::string base_planner{"RRTConnect"};
    double planning_budget_s{10.0};
    double slow_direct_threshold_ms{0.0};
    int sample_count{24};
    int top_guides{10};
    int hold_s{12};
    bool adaptive_ellipsoid{true};
    bool execute_motion{false};
    bool keep_alive{false};
    std::string output_dir;
    std::optional<std::uint32_t> guide_seed;
};

template <typename T>
T clampParsed(T value, T min_value, T max_value) {
    return std::max(min_value, std::min(max_value, value));
}

std::string getEnvString(const char* key, const std::string& default_value) {
    return my_cr5_control::env::getString(key, default_value);
}

int getEnvInt(const char* key, int default_value, int min_value, int max_value) {
    return my_cr5_control::env::getIntClamped(key, default_value, min_value, max_value);
}

double getEnvDouble(const char* key,
                    double default_value,
                    double min_value,
                    double max_value) {
    return my_cr5_control::env::getDoubleClamped(key, default_value, min_value, max_value);
}

bool getEnvBool(const char* key, bool default_value) {
    return my_cr5_control::env::getBool(key, default_value);
}

std::optional<std::uint32_t> getEnvUint32(const char* key) {
    return my_cr5_control::env::getUint32(key);
}

DebugConfig loadConfig() {
    DebugConfig config;
    config.benchmark = getEnvString("MY_CR5_CONTROL_DEBUG_BENCHMARK", "simple");
    config.scene = getEnvString("MY_CR5_CONTROL_DEBUG_SCENE", "Medium_SideSurface");
    config.base_planner = getEnvString("MY_CR5_CONTROL_DEBUG_BASE_PLANNER", "RRTConnect");
    config.planning_budget_s =
        getEnvDouble("MY_CR5_CONTROL_DEBUG_PLANNING_BUDGET_S", 10.0, 0.5, 30.0);
    config.slow_direct_threshold_ms =
        getEnvDouble("MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS", 0.0, 0.0, 10000.0);
    config.sample_count =
        getEnvInt("MY_CR5_CONTROL_DEBUG_SAMPLE_COUNT", 24, 1, 128);
    config.top_guides =
        getEnvInt("MY_CR5_CONTROL_DEBUG_TOP_GUIDES", 10, 1, 20);
    config.hold_s =
        getEnvInt("MY_CR5_CONTROL_DEBUG_HOLD_S", 12, 1, 120);
    config.adaptive_ellipsoid =
        getEnvBool("MY_CR5_CONTROL_DEBUG_ADAPTIVE_ELLIPSOID", true);
    config.execute_motion =
        getEnvBool("MY_CR5_CONTROL_DEBUG_EXECUTE", false);
    config.keep_alive =
        getEnvBool("MY_CR5_CONTROL_DEBUG_KEEP_ALIVE", false);
    config.output_dir =
        getEnvString("MY_CR5_CONTROL_DEBUG_OUTPUT_DIR", "");
    config.guide_seed = getEnvUint32("MY_CR5_CONTROL_DEBUG_GUIDE_SEED");
    return config;
}

std::string jsonEscape(const std::string& value) {
    std::ostringstream out;
    for (const char c : value) {
        switch (c) {
            case '\\': out << "\\\\"; break;
            case '"': out << "\\\""; break;
            case '\n': out << "\\n"; break;
            case '\r': out << "\\r"; break;
            case '\t': out << "\\t"; break;
            default: out << c; break;
        }
    }
    return out.str();
}

std::string optionalSeedToString(const std::optional<std::uint32_t>& seed) {
    if (!seed.has_value()) {
        return "";
    }
    return std::to_string(*seed);
}

void writeTrajectoryCsv(const std::filesystem::path& path,
                        const std::string& trajectory_name,
                        const std::vector<geometry_msgs::msg::Point>& points) {
    std::ofstream out(path);
    out << "trajectory,index,x,y,z\n";
    out << std::fixed << std::setprecision(9);
    for (std::size_t i = 0; i < points.size(); ++i) {
        out << trajectory_name << ','
            << i << ','
            << points[i].x << ','
            << points[i].y << ','
            << points[i].z << '\n';
    }
}

void writeGuideCandidatesCsv(const std::filesystem::path& path,
                             const std::vector<CR5Robot::GuideCandidate>& candidates) {
    std::ofstream out(path);
    out << "rank,candidate_id,x,y,z,heuristic_cost,ranking_score,ik_feasible,"
           "direct_cost,cost_delta_to_direct,start_to_guide_distance,guide_to_goal_distance,"
           "total_guide_distance,direct_distance,axial_progress,lateral_offset,"
           "clearance_margin,manipulability_score,safety_penalty\n";
    out << std::fixed << std::setprecision(9);
    for (std::size_t i = 0; i < candidates.size(); ++i) {
        const auto& c = candidates[i];
        out << i << ','
            << c.candidate_id << ','
            << c.pose.position.x << ','
            << c.pose.position.y << ','
            << c.pose.position.z << ','
            << c.heuristic_cost << ','
            << c.ranking_score << ','
            << (c.ik_feasible ? "true" : "false") << ','
            << c.direct_cost << ','
            << c.cost_delta_to_direct << ','
            << c.start_to_guide_distance << ','
            << c.guide_to_goal_distance << ','
            << c.total_guide_distance << ','
            << c.direct_distance << ','
            << c.axial_progress << ','
            << c.lateral_offset << ','
            << c.clearance_margin << ','
            << c.manipulability_score << ','
            << c.safety_penalty << '\n';
    }
}

void writeMetricsJson(const std::filesystem::path& path,
                      const DebugConfig& config,
                      const std::string& difficulty,
                      double difficulty_score,
                      const geometry_msgs::msg::Point& tip_point,
                      const geometry_msgs::msg::Pose& target_pose,
                      const CR5Robot::PlanningMetrics& direct_metrics,
                      const CR5Robot::PlanningMetrics& heuristic_metrics,
                      std::size_t direct_point_count,
                      std::size_t heuristic_point_count,
                      std::size_t candidate_count) {
    std::ofstream out(path);
    out << std::fixed << std::setprecision(6);
    out << "{\n";
    out << "  \"benchmark\": \"" << jsonEscape(config.benchmark) << "\",\n";
    out << "  \"scene\": \"" << jsonEscape(config.scene) << "\",\n";
    out << "  \"base_planner\": \"" << jsonEscape(config.base_planner) << "\",\n";
    out << "  \"planning_budget_s\": " << config.planning_budget_s << ",\n";
    out << "  \"sample_count\": " << config.sample_count << ",\n";
    out << "  \"adaptive_ellipsoid\": " << (config.adaptive_ellipsoid ? "true" : "false") << ",\n";
    out << "  \"guide_seed\": \"" << optionalSeedToString(config.guide_seed) << "\",\n";
    out << "  \"difficulty\": \"" << jsonEscape(difficulty) << "\",\n";
    out << "  \"difficulty_score\": " << difficulty_score << ",\n";
    out << "  \"tip_point\": {\"x\": " << tip_point.x << ", \"y\": " << tip_point.y
        << ", \"z\": " << tip_point.z << "},\n";
    out << "  \"target_flange_pose\": {\"x\": " << target_pose.position.x
        << ", \"y\": " << target_pose.position.y
        << ", \"z\": " << target_pose.position.z << "},\n";
    out << "  \"direct\": {\n";
    out << "    \"success\": " << (direct_metrics.success ? "true" : "false") << ",\n";
    out << "    \"wall_time_ms\": " << direct_metrics.wall_time_ms << ",\n";
    out << "    \"budget_hit\": " << (direct_metrics.hit_budget_limit ? "true" : "false") << ",\n";
    out << "    \"planner_calls\": " << direct_metrics.planner_calls << ",\n";
    out << "    \"trajectory_points\": " << direct_point_count << "\n";
    out << "  },\n";
    out << "  \"heuristic\": {\n";
    out << "    \"success\": " << (heuristic_metrics.success ? "true" : "false") << ",\n";
    out << "    \"wall_time_ms\": " << heuristic_metrics.wall_time_ms << ",\n";
    out << "    \"budget_hit\": " << (heuristic_metrics.hit_budget_limit ? "true" : "false") << ",\n";
    out << "    \"planner_calls\": " << heuristic_metrics.planner_calls << ",\n";
    out << "    \"guide_candidate_count\": " << heuristic_metrics.guide_candidate_count << ",\n";
    out << "    \"guide_candidates_attempted\": " << heuristic_metrics.guide_candidates_attempted << ",\n";
    out << "    \"used_direct_plan\": " << (heuristic_metrics.used_direct_plan ? "true" : "false") << ",\n";
    out << "    \"top_ranked_candidate_id\": " << heuristic_metrics.top_ranked_candidate_id << ",\n";
    out << "    \"selected_candidate_id\": " << heuristic_metrics.selected_candidate_id << ",\n";
    out << "    \"trajectory_points\": " << heuristic_point_count << "\n";
    out << "  },\n";
    out << "  \"exported_candidate_count\": " << candidate_count << "\n";
    out << "}\n";
}

geometry_msgs::msg::Quaternion makeDownwardOrientation() {
    tf2::Quaternion q;
    q.setRPY(0.0, kPi, 0.0);
    return tf2::toMsg(q);
}

std::vector<SimpleScene> generateSimpleScenes(const SimpleBox& box) {
    std::vector<SimpleScene> scenes;

    {
        SimpleScene scene;
        scene.name = "Easy_TopCenter";
        scene.difficulty = "easy";
        scene.description = "箱体顶部中心（开放空间）";
        scene.difficulty_score = 0.3;
        scene.tip_point.x = box.center_x;
        scene.tip_point.y = box.center_y;
        scene.tip_point.z = box.center_z + box.height + 0.05;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, makeDownwardOrientation());
        scenes.push_back(scene);
    }

    {
        SimpleScene scene;
        scene.name = "Medium_SideSurface";
        scene.difficulty = "medium";
        scene.description = "箱体侧面（需要特定姿态）";
        scene.difficulty_score = 0.5;
        scene.tip_point.x = box.center_x - 0.04;
        scene.tip_point.y = box.center_y + box.depth / 2.0 + 0.01;
        scene.tip_point.z = box.center_z + box.height * 0.55;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.7071068;
        orientation.y = 0.0;
        orientation.z = 0.0;
        orientation.w = 0.7071068;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, orientation);
        scenes.push_back(scene);
    }

    {
        SimpleScene scene;
        scene.name = "MediumPlus_RightUpperAngled";
        scene.difficulty = "medium";
        scene.description = "箱体右侧上沿（侧向斜入）";
        scene.difficulty_score = 0.6;
        scene.tip_point.x = box.center_x + box.width / 2.0 + 0.01;
        scene.tip_point.y = box.center_y;
        scene.tip_point.z = box.center_z + box.height * 0.75;
        geometry_msgs::msg::Quaternion orientation;
        orientation.x = 0.0;
        orientation.y = -0.9238795;
        orientation.z = 0.0;
        orientation.w = 0.3826834;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, orientation);
        scenes.push_back(scene);
    }

    {
        SimpleScene scene;
        scene.name = "Hard_HoleShallow";
        scene.difficulty = "hard";
        scene.description = "孔内部浅层（狭窄空间）";
        scene.difficulty_score = 0.7;
        scene.tip_point.x = box.center_x;
        scene.tip_point.y = box.center_y;
        scene.tip_point.z = box.center_z + box.height - 0.04;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, makeDownwardOrientation());
        scenes.push_back(scene);
    }

    {
        SimpleScene scene;
        scene.name = "HardPlus_HoleEdgeOffset";
        scene.difficulty = "hard";
        scene.description = "孔口边缘偏置（入孔余量更小）";
        scene.difficulty_score = 0.8;
        scene.tip_point.x = box.center_x - 0.03;
        scene.tip_point.y = box.center_y + 0.03;
        scene.tip_point.z = box.center_z + box.height - 0.07;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, makeDownwardOrientation());
        scenes.push_back(scene);
    }

    {
        SimpleScene scene;
        scene.name = "Extreme_HoleDeep";
        scene.difficulty = "extreme";
        scene.description = "孔内部深层（极度狭窄）";
        scene.difficulty_score = 0.9;
        scene.tip_point.x = box.center_x;
        scene.tip_point.y = box.center_y;
        scene.tip_point.z = box.center_z + box.height - 0.10;
        scene.flange_pose = my_cr5_control::tool::buildFlangePoseFromTipPoint(
            scene.tip_point, makeDownwardOrientation());
        scenes.push_back(scene);
    }

    return scenes;
}

std::optional<SimpleScene> findSimpleScene(const std::string& scene_name) {
    const SimpleBox box;
    for (const auto& scene : generateSimpleScenes(box)) {
        if (scene.name == scene_name) {
            return scene;
        }
    }
    return std::nullopt;
}

bool setupSimpleScene(CR5Robot& robot, const rclcpp::Logger& logger) {
    const SimpleBox box;
    for (const char* object_id : {
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
             "test_box",
             "ws119_mesh"}) {
        robot.removeCollisionObject(object_id);
    }

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;

    pose.position.x = 0.0;
    pose.position.y = 0.0;
    pose.position.z = -0.51;
    if (!robot.addBoxObstacleObject("floor", pose, 10.0, 10.0, 1.0)) {
        return false;
    }

    const double wall_thickness = 0.02;
    const double hole_size = box.hole_radius * 2.0;

    auto add_box = [&](const std::string& id,
                       double x,
                       double y,
                       double z,
                       double dx,
                       double dy,
                       double dz) -> bool {
        geometry_msgs::msg::Pose box_pose;
        box_pose.orientation.w = 1.0;
        box_pose.position.x = x;
        box_pose.position.y = y;
        box_pose.position.z = z;
        return robot.addBoxObstacleObject(id, box_pose, dx, dy, dz);
    };

    if (!add_box("box_bottom",
                 box.center_x,
                 box.center_y,
                 box.center_z + wall_thickness / 2.0,
                 box.width,
                 box.depth,
                 wall_thickness) ||
        !add_box("box_front",
                 box.center_x,
                 box.center_y + box.depth / 2.0 - wall_thickness / 2.0,
                 box.center_z + box.height / 2.0,
                 box.width,
                 wall_thickness,
                 box.height) ||
        !add_box("box_back",
                 box.center_x,
                 box.center_y - box.depth / 2.0 + wall_thickness / 2.0,
                 box.center_z + box.height / 2.0,
                 box.width,
                 wall_thickness,
                 box.height) ||
        !add_box("box_left",
                 box.center_x - box.width / 2.0 + wall_thickness / 2.0,
                 box.center_y,
                 box.center_z + box.height / 2.0,
                 wall_thickness,
                 box.depth,
                 box.height) ||
        !add_box("box_right",
                 box.center_x + box.width / 2.0 - wall_thickness / 2.0,
                 box.center_y,
                 box.center_z + box.height / 2.0,
                 wall_thickness,
                 box.depth,
                 box.height) ||
        !add_box("box_top_front",
                 box.center_x,
                 box.center_y + hole_size / 2.0 + (box.depth - hole_size) / 4.0,
                 box.center_z + box.height - wall_thickness / 2.0,
                 box.width,
                 (box.depth - hole_size) / 2.0,
                 wall_thickness) ||
        !add_box("box_top_back",
                 box.center_x,
                 box.center_y - hole_size / 2.0 - (box.depth - hole_size) / 4.0,
                 box.center_z + box.height - wall_thickness / 2.0,
                 box.width,
                 (box.depth - hole_size) / 2.0,
                 wall_thickness) ||
        !add_box("box_top_left",
                 box.center_x - hole_size / 2.0 - (box.width - hole_size) / 4.0,
                 box.center_y,
                 box.center_z + box.height - wall_thickness / 2.0,
                 (box.width - hole_size) / 2.0,
                 hole_size,
                 wall_thickness) ||
        !add_box("box_top_right",
                 box.center_x + hole_size / 2.0 + (box.width - hole_size) / 4.0,
                 box.center_y,
                 box.center_z + box.height - wall_thickness / 2.0,
                 (box.width - hole_size) / 2.0,
                 hole_size,
                 wall_thickness)) {
        RCLCPP_ERROR(logger, "simple 调试场景创建失败");
        return false;
    }

    robot.setGuideEnvironmentBoxHint(
        box.center_x,
        box.center_y,
        box.center_z + box.height * 0.5,
        box.width,
        box.depth,
        box.height);
    std::this_thread::sleep_for(std::chrono::milliseconds(600));
    return true;
}

std::optional<measurement::MeasurementPointGenerator::TestScenario> findV2Scene(
    const std::string& scene_name) {
    measurement::MeasurementPointGenerator generator;
    const auto scenarios = generator.generateTestScenarios();
    for (const auto& scenario : scenarios) {
        if (scenario.name == scene_name) {
            return scenario;
        }
    }
    return std::nullopt;
}

visualization_msgs::msg::Marker baseMarker(const std::string& ns,
                                           int id,
                                           int type,
                                           const rclcpp::Node::SharedPtr& node) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.header.stamp = node->now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
}

visualization_msgs::msg::Marker sphereMarker(const std::string& ns,
                                             int id,
                                             const geometry_msgs::msg::Pose& pose,
                                             double scale,
                                             float r,
                                             float g,
                                             float b,
                                             float a,
                                             const rclcpp::Node::SharedPtr& node) {
    auto marker = baseMarker(ns, id, visualization_msgs::msg::Marker::SPHERE, node);
    marker.pose = pose;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::msg::Marker arrowMarker(const std::string& ns,
                                            int id,
                                            const geometry_msgs::msg::Pose& pose,
                                            float r,
                                            float g,
                                            float b,
                                            float a,
                                            const rclcpp::Node::SharedPtr& node) {
    auto marker = baseMarker(ns, id, visualization_msgs::msg::Marker::ARROW, node);
    marker.pose = pose;
    marker.scale.x = 0.08;
    marker.scale.y = 0.012;
    marker.scale.z = 0.012;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::msg::Marker lineStripMarker(
    const std::string& ns,
    int id,
    const std::vector<geometry_msgs::msg::Point>& points,
    double width,
    float r,
    float g,
    float b,
    float a,
    const rclcpp::Node::SharedPtr& node) {
    auto marker = baseMarker(ns, id, visualization_msgs::msg::Marker::LINE_STRIP, node);
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.points = points;
    return marker;
}

visualization_msgs::msg::Marker cylinderMarker(const std::string& ns,
                                               int id,
                                               const geometry_msgs::msg::Pose& pose,
                                               double radius,
                                               double height,
                                               float r,
                                               float g,
                                               float b,
                                               float a,
                                               const rclcpp::Node::SharedPtr& node) {
    auto marker = baseMarker(ns, id, visualization_msgs::msg::Marker::CYLINDER, node);
    marker.pose = pose;
    marker.scale.x = 2.0 * radius;
    marker.scale.y = 2.0 * radius;
    marker.scale.z = height;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    return marker;
}

visualization_msgs::msg::Marker textMarker(const std::string& ns,
                                           int id,
                                           const geometry_msgs::msg::Point& position,
                                           const std::string& text,
                                           double size,
                                           float r,
                                           float g,
                                           float b,
                                           float a,
                                           const rclcpp::Node::SharedPtr& node) {
    auto marker = baseMarker(ns, id, visualization_msgs::msg::Marker::TEXT_VIEW_FACING, node);
    marker.pose.position = position;
    marker.scale.z = size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.text = text;
    return marker;
}

geometry_msgs::msg::Pose poseWithLocalOffsetAndRotation(
    const geometry_msgs::msg::Pose& base_pose,
    double local_z_offset_m,
    const tf2::Quaternion& local_rotation) {
    tf2::Quaternion base_quat;
    tf2::fromMsg(base_pose.orientation, base_quat);
    const tf2::Vector3 offset =
        tf2::quatRotate(base_quat, tf2::Vector3(0.0, 0.0, local_z_offset_m));

    geometry_msgs::msg::Pose pose = base_pose;
    pose.position.x += offset.x();
    pose.position.y += offset.y();
    pose.position.z += offset.z();
    pose.orientation = tf2::toMsg(base_quat * local_rotation);
    return pose;
}

void publishDebugMarkers(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& compat_publisher,
    const std::string& benchmark,
    const std::string& scene_name,
    const std::string& difficulty,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const std::vector<CR5Robot::GuideCandidate>& candidates,
    const CR5Robot::PlanningMetrics& direct_metrics,
    const CR5Robot::PlanningMetrics& heuristic_metrics,
    const std::vector<geometry_msgs::msg::Point>& direct_trajectory_points,
    const std::vector<geometry_msgs::msg::Point>& heuristic_trajectory_points,
    int top_guides) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);

    int id = 0;
    markers.markers.push_back(
        sphereMarker("start", id++, start_pose, 0.035, 0.10f, 0.80f, 0.20f, 0.95f, node));
    markers.markers.push_back(
        arrowMarker("start_axis", id++, start_pose, 0.10f, 0.80f, 0.20f, 0.90f, node));
    markers.markers.push_back(
        sphereMarker("goal", id++, goal_pose, 0.035, 0.95f, 0.20f, 0.10f, 0.95f, node));
    markers.markers.push_back(
        arrowMarker("goal_axis", id++, goal_pose, 0.95f, 0.20f, 0.10f, 0.90f, node));

    tf2::Quaternion identity;
    identity.setRPY(0.0, 0.0, 0.0);
    tf2::Quaternion star_x_rotation;
    star_x_rotation.setRPY(0.0, kPi * 0.5, 0.0);
    tf2::Quaternion star_y_rotation;
    star_y_rotation.setRPY(kPi * 0.5, 0.0, 0.0);

    const auto probe_body_pose = poseWithLocalOffsetAndRotation(
        goal_pose, my_cr5_control::probe::kProbeBodyHeight * 0.5, identity);
    const auto probe_stem_pose = poseWithLocalOffsetAndRotation(
        goal_pose,
        my_cr5_control::probe::kProbeBodyHeight + my_cr5_control::probe::kProbeStemHeight * 0.5,
        identity);
    const auto probe_star_x_pose = poseWithLocalOffsetAndRotation(
        goal_pose, my_cr5_control::probe::kStarStylusZOffset, star_x_rotation);
    const auto probe_star_y_pose = poseWithLocalOffsetAndRotation(
        goal_pose, my_cr5_control::probe::kStarStylusZOffset, star_y_rotation);
    const auto probe_tip_pose = poseWithLocalOffsetAndRotation(
        goal_pose, my_cr5_control::probe::kProbeTipZOffset, identity);
    markers.markers.push_back(
        cylinderMarker("probe_body", id++, probe_body_pose,
                       my_cr5_control::probe::kProbeBodyRadius,
                       my_cr5_control::probe::kProbeBodyHeight,
                       0.20f, 0.45f, 1.00f, 0.75f, node));
    markers.markers.push_back(
        cylinderMarker("probe_stem", id++, probe_stem_pose,
                       my_cr5_control::probe::kProbeStemRadius * 2.0,
                       my_cr5_control::probe::kProbeStemHeight,
                       0.10f, 0.95f, 1.00f, 0.90f, node));
    markers.markers.push_back(
        cylinderMarker("probe_star_x", id++, probe_star_x_pose,
                       my_cr5_control::probe::kStarStylusTipRadius * 6.0,
                       my_cr5_control::probe::kStarStylusLength,
                       1.00f, 0.95f, 0.20f, 0.95f, node));
    markers.markers.push_back(
        cylinderMarker("probe_star_y", id++, probe_star_y_pose,
                       my_cr5_control::probe::kStarStylusTipRadius * 6.0,
                       my_cr5_control::probe::kStarStylusLength,
                       1.00f, 0.95f, 0.20f, 0.95f, node));
    markers.markers.push_back(
        cylinderMarker("probe_tip", id++, probe_tip_pose,
                       my_cr5_control::probe::kProbeTipRadius * 6.0,
                       my_cr5_control::probe::kProbeTipHeight,
                       1.00f, 0.95f, 0.20f, 0.95f, node));

    markers.markers.push_back(
        lineStripMarker("direct_path",
                        id++,
                        {start_pose.position, goal_pose.position},
                        0.003,
                        0.85f,
                        0.85f,
                        0.85f,
                        0.65f,
                        node));

    if (direct_trajectory_points.size() >= 2) {
        markers.markers.push_back(
            lineStripMarker("direct_trajectory",
                            id++,
                            direct_trajectory_points,
                            0.006,
                            1.00f,
                            0.18f,
                            0.12f,
                            0.92f,
                            node));
    }

    if (heuristic_trajectory_points.size() >= 2) {
        markers.markers.push_back(
            lineStripMarker("heuristic_trajectory",
                            id++,
                            heuristic_trajectory_points,
                            0.006,
                            0.10f,
                            0.85f,
                            1.00f,
                            0.95f,
                            node));
    }

    const int guide_limit =
        std::min<int>(top_guides, static_cast<int>(candidates.size()));
    for (int i = 0; i < guide_limit; ++i) {
        const auto& candidate = candidates[static_cast<std::size_t>(i)];
        const float alpha = 0.82f - static_cast<float>(0.05 * i);
        const float green = std::max(0.20f, 0.80f - static_cast<float>(0.05 * i));
        markers.markers.push_back(
            sphereMarker("guide_points",
                         id++,
                         candidate.pose,
                         0.022,
                         0.96f,
                         green,
                         0.15f,
                         alpha,
                         node));
        markers.markers.push_back(
            arrowMarker("guide_axes",
                        id++,
                        candidate.pose,
                        0.96f,
                        green,
                        0.15f,
                        alpha,
                        node));
        markers.markers.push_back(
            lineStripMarker("guide_paths",
                            id++,
                            {start_pose.position, candidate.pose.position, goal_pose.position},
                            0.0018,
                            0.96f,
                            green,
                            0.15f,
                            0.50f,
                            node));
        geometry_msgs::msg::Point label_pos = candidate.pose.position;
        label_pos.z += 0.025;
        std::ostringstream oss;
        oss << "#" << candidate.candidate_id
            << " s=" << std::fixed << std::setprecision(3) << candidate.ranking_score
            << " dh=" << candidate.cost_delta_to_direct;
        markers.markers.push_back(
            textMarker("guide_labels", id++, label_pos, oss.str(), 0.010, 1.0f, 0.95f, 0.75f, 0.95f, node));
    }

    if (heuristic_metrics.selected_candidate_id >= 0) {
        geometry_msgs::msg::Pose selected_pose = goal_pose;
        selected_pose.position = heuristic_metrics.selected_candidate_point;
        markers.markers.push_back(
            sphereMarker("selected_candidate",
                         id++,
                         selected_pose,
                         0.030,
                         0.10f,
                         0.95f,
                         0.95f,
                         0.95f,
                         node));
        markers.markers.push_back(
            lineStripMarker("selected_path",
                            id++,
                            {start_pose.position, selected_pose.position, goal_pose.position},
                            0.004,
                            0.10f,
                            0.95f,
                            0.95f,
                            0.85f,
                            node));
    }

    geometry_msgs::msg::Point summary_pos = goal_pose.position;
    summary_pos.y += 0.18;
    summary_pos.z += 0.12;
    std::ostringstream summary;
    summary << benchmark << " / " << scene_name << " [" << difficulty << "]\n"
            << "direct: " << std::fixed << std::setprecision(1) << direct_metrics.wall_time_ms
            << " ms, success=" << (direct_metrics.success ? "true" : "false") << "\n"
            << "heur: " << heuristic_metrics.wall_time_ms
            << " ms, attempted=" << heuristic_metrics.guide_candidates_attempted
            << ", fallback=" << (heuristic_metrics.used_direct_plan ? "true" : "false") << "\n"
            << "red=direct trajectory, cyan=HeuristicGuided trajectory";
    markers.markers.push_back(
        textMarker("summary", id++, summary_pos, summary.str(), 0.018, 0.95f, 0.95f, 1.0f, 0.98f, node));

    publisher->publish(markers);
    if (compat_publisher) {
        compat_publisher->publish(markers);
    }
}

void logTopCandidates(const rclcpp::Logger& logger,
                      const std::vector<CR5Robot::GuideCandidate>& candidates,
                      int top_guides) {
    const int limit = std::min<int>(top_guides, static_cast<int>(candidates.size()));
    for (int i = 0; i < limit; ++i) {
        const auto& candidate = candidates[static_cast<std::size_t>(i)];
        RCLCPP_INFO(
            logger,
            "guide[%d] id=%d score=%.4f heuristic=%.4f delta_direct=%.4f clearance=%.4f manip=%.4f axial=%.3f lateral=%.3f",
            i,
            candidate.candidate_id,
            candidate.ranking_score,
            candidate.heuristic_cost,
            candidate.cost_delta_to_direct,
            candidate.clearance_margin,
            candidate.manipulability_score,
            candidate.axial_progress,
            candidate.lateral_offset);
    }
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    const auto config = loadConfig();
    auto node = rclcpp::Node::make_shared("heuristic_guided_visual_debug");
    auto logger = node->get_logger();

    auto marker_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "heuristic_guided_visual_debug_markers",
        rclcpp::QoS(1).reliable().transient_local());
    auto marker_compat_publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker_array",
        rclcpp::QoS(1).reliable().transient_local());

    RCLCPP_INFO(
        logger,
        "visual debug config: benchmark=%s scene=%s planner=%s budget=%.1fs slow_direct=%.1fms sample_count=%d hold_s=%d adaptive=%s execute=%s keep_alive=%s",
        config.benchmark.c_str(),
        config.scene.c_str(),
        config.base_planner.c_str(),
        config.planning_budget_s,
        config.slow_direct_threshold_ms,
        config.sample_count,
        config.hold_s,
        config.adaptive_ellipsoid ? "true" : "false",
        config.execute_motion ? "true" : "false",
        config.keep_alive ? "true" : "false");
    if (config.guide_seed.has_value()) {
        RCLCPP_INFO(logger, "guide sampling seed fixed to %u", *config.guide_seed);
    }

    CR5Robot robot("heuristic_guided_visual_debug_node");
    if (!robot.init()) {
        RCLCPP_ERROR(logger, "机器人初始化失败");
        rclcpp::shutdown();
        return 1;
    }
    robot.enableAdaptiveEllipsoidSampling(config.adaptive_ellipsoid);
    robot.setPlanner(config.base_planner);

    geometry_msgs::msg::Pose target_pose;
    geometry_msgs::msg::Point tip_point;
    std::string difficulty;
    double difficulty_score = 0.0;

    if (config.benchmark == "simple") {
        if (!setupSimpleScene(robot, logger)) {
            rclcpp::shutdown();
            return 1;
        }
        const auto scene = findSimpleScene(config.scene);
        if (!scene.has_value()) {
            RCLCPP_ERROR(logger, "未找到 simple 场景: %s", config.scene.c_str());
            rclcpp::shutdown();
            return 1;
        }
        target_pose = scene->flange_pose;
        tip_point = scene->tip_point;
        difficulty = scene->difficulty;
        difficulty_score = scene->difficulty_score;
    } else if (config.benchmark == "v2") {
        robot.addSimulationEnvironment();
        std::this_thread::sleep_for(std::chrono::seconds(2));
        const auto scenario = findV2Scene(config.scene);
        if (!scenario.has_value()) {
            RCLCPP_ERROR(logger, "未找到 v2 场景: %s", config.scene.c_str());
            rclcpp::shutdown();
            return 1;
        }
        if (scenario->points.empty()) {
            RCLCPP_ERROR(logger, "v2 场景没有测点: %s", config.scene.c_str());
            rclcpp::shutdown();
            return 1;
        }
        const auto& point = scenario->points.front();
        target_pose = my_cr5_control::paper_mainline::buildV2MeasurementFlangePose(point);
        tip_point = point.position;
        difficulty = scenario->difficulty;
        difficulty_score = point.difficulty_score;
    } else {
        RCLCPP_ERROR(logger, "不支持的 benchmark: %s", config.benchmark.c_str());
        rclcpp::shutdown();
        return 1;
    }

    if (config.adaptive_ellipsoid) {
        robot.setSceneDifficultyScore(difficulty_score);
    }
    if (config.guide_seed.has_value()) {
        robot.setGuideSamplingSeed(*config.guide_seed);
    }

    const bool visual_start_aligned = robot.moveToNamedTarget("home");
    if (visual_start_aligned) {
        RCLCPP_INFO(logger, "robot moved to named target home for visual alignment");
        std::this_thread::sleep_for(std::chrono::milliseconds(500));
    } else {
        RCLCPP_WARN(logger, "moveToNamedTarget(home) 失败，将继续使用当前状态作为可视化起点");
    }
    const geometry_msgs::msg::Pose start_pose = robot.getCurrentPose();

    RCLCPP_INFO(
        logger,
        "scene=%s difficulty=%s score=%.2f tip=(%.4f, %.4f, %.4f) flange=(%.4f, %.4f, %.4f)",
        config.scene.c_str(),
        difficulty.c_str(),
        difficulty_score,
        tip_point.x,
        tip_point.y,
        tip_point.z,
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z);

    CR5Robot::PlanningMetrics direct_metrics;
    robot.planToPoseWithPlanner(
        target_pose, config.base_planner, config.planning_budget_s, "home", &direct_metrics);
    RCLCPP_INFO(
        logger,
        "direct metrics: success=%s wall=%.1fms budget_hit=%s planner_calls=%d",
        direct_metrics.success ? "true" : "false",
        direct_metrics.wall_time_ms,
        direct_metrics.hit_budget_limit ? "true" : "false",
        direct_metrics.planner_calls);

    if (config.guide_seed.has_value()) {
        robot.setGuideSamplingSeed(*config.guide_seed);
    }
    const auto guide_candidates =
        robot.sampleGuideCandidates(target_pose, "home", static_cast<std::size_t>(config.sample_count), true);
    RCLCPP_INFO(logger, "sampled %zu ranked guide candidates", guide_candidates.size());
    logTopCandidates(logger, guide_candidates, config.top_guides);

    moveit_msgs::msg::RobotTrajectory direct_trajectory;
    CR5Robot::PlanningMetrics direct_trajectory_metrics;
    robot.planToPoseWithPlannerTrajectory(
        target_pose,
        config.base_planner,
        config.planning_budget_s,
        "home",
        &direct_trajectory,
        &direct_trajectory_metrics);
    const auto direct_trajectory_points =
        robot.endEffectorPathFromTrajectory(direct_trajectory);
    RCLCPP_INFO(
        logger,
        "direct trajectory visualization points: %zu",
        direct_trajectory_points.size());

    std::vector<geometry_msgs::msg::Point> heuristic_trajectory_points;
    if (!guide_candidates.empty()) {
        std::vector<moveit_msgs::msg::RobotTrajectory> heuristic_trajectories;
        CR5Robot::PlanningMetrics selected_guide_metrics;
        robot.planToPoseViaGuideTrajectories(
            target_pose,
            guide_candidates.front().pose,
            "home",
            config.planning_budget_s,
            &heuristic_trajectories,
            &selected_guide_metrics);
        for (const auto& trajectory : heuristic_trajectories) {
            auto points = robot.endEffectorPathFromTrajectory(trajectory);
            heuristic_trajectory_points.insert(
                heuristic_trajectory_points.end(), points.begin(), points.end());
        }
    }
    RCLCPP_INFO(
        logger,
        "heuristic trajectory visualization points: %zu",
        heuristic_trajectory_points.size());

    if (config.guide_seed.has_value()) {
        robot.setGuideSamplingSeed(*config.guide_seed);
    }
    CR5Robot::PlanningMetrics heuristic_metrics;
    robot.planToPoseImproved(
        target_pose,
        "home",
        config.planning_budget_s,
        &heuristic_metrics,
        static_cast<std::size_t>(config.sample_count),
        nullptr);
    RCLCPP_INFO(
        logger,
        "heuristic metrics: success=%s wall=%.1fms guide_attempted=%d direct_fallback=%s selected_id=%d top_id=%d",
        heuristic_metrics.success ? "true" : "false",
        heuristic_metrics.wall_time_ms,
        heuristic_metrics.guide_candidates_attempted,
        heuristic_metrics.used_direct_plan ? "true" : "false",
        heuristic_metrics.selected_candidate_id,
        heuristic_metrics.top_ranked_candidate_id);

    if (!config.output_dir.empty()) {
        const std::filesystem::path output_dir(config.output_dir);
        std::error_code ec;
        std::filesystem::create_directories(output_dir, ec);
        if (ec) {
            RCLCPP_ERROR(
                logger,
                "failed to create trajectory output dir %s: %s",
                output_dir.string().c_str(),
                ec.message().c_str());
        } else {
            writeTrajectoryCsv(
                output_dir / "direct_ee_trajectory.csv",
                "direct",
                direct_trajectory_points);
            writeTrajectoryCsv(
                output_dir / "heuristic_guided_ee_trajectory.csv",
                "heuristic_guided",
                heuristic_trajectory_points);
            writeGuideCandidatesCsv(
                output_dir / "guide_candidates.csv",
                guide_candidates);
            writeMetricsJson(
                output_dir / "metrics.json",
                config,
                difficulty,
                difficulty_score,
                tip_point,
                target_pose,
                direct_trajectory_metrics,
                heuristic_metrics,
                direct_trajectory_points.size(),
                heuristic_trajectory_points.size(),
                guide_candidates.size());
            RCLCPP_INFO(
                logger,
                "trajectory export written to %s",
                output_dir.string().c_str());
        }
    }

    if (heuristic_metrics.used_direct_plan &&
        heuristic_metrics.guide_candidates_attempted == 0 &&
        heuristic_metrics.wall_time_ms >= kFastSolveThresholdMs) {
        RCLCPP_WARN(
            logger,
            "diagnosis: 该场景表现为 direct-only long tail；当前 guidance 没有真正被触发");
    } else if (!heuristic_metrics.used_direct_plan &&
               heuristic_metrics.guide_candidates_attempted > 0) {
        RCLCPP_INFO(
            logger,
            "diagnosis: 当前 guidance 已被激活，可在 RViz 中核对 selected/top guide 的空间分布");
    }

    publishDebugMarkers(
        node,
        marker_publisher,
        marker_compat_publisher,
        config.benchmark,
        config.scene,
        difficulty,
        start_pose,
        target_pose,
        guide_candidates,
        direct_metrics,
        heuristic_metrics,
        direct_trajectory_points,
        heuristic_trajectory_points,
        config.top_guides);

    if (config.execute_motion && heuristic_metrics.success) {
        if (!visual_start_aligned) {
            RCLCPP_WARN(
                logger,
                "跳过 motion execution：当前可视化起点未能对齐到 home，执行结果将不再代表论文主线起点");
        } else {
            const bool executed = robot.moveToPoseImproved(target_pose);
            RCLCPP_INFO(logger, "motion execution via HeuristicGuided: %s", executed ? "true" : "false");
        }
    }

    for (int i = 0; i < config.hold_s && rclcpp::ok(); ++i) {
        publishDebugMarkers(
            node,
            marker_publisher,
            marker_compat_publisher,
            config.benchmark,
            config.scene,
            difficulty,
            start_pose,
            target_pose,
            guide_candidates,
            direct_metrics,
            heuristic_metrics,
            direct_trajectory_points,
            heuristic_trajectory_points,
            config.top_guides);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    while (config.keep_alive && rclcpp::ok()) {
        publishDebugMarkers(
            node,
            marker_publisher,
            marker_compat_publisher,
            config.benchmark,
            config.scene,
            difficulty,
            start_pose,
            target_pose,
            guide_candidates,
            direct_metrics,
            heuristic_metrics,
            direct_trajectory_points,
            heuristic_trajectory_points,
            config.top_guides);
        rclcpp::spin_some(node);
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }

    rclcpp::shutdown();
    return 0;
}
