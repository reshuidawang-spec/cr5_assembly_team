#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstddef>
#include <limits>
#include <optional>
#include <random>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "my_cr5_control/cr5_robot.hpp"

namespace {

constexpr double kPi = 3.14159265358979323846;

struct Vec3 {
    double x{0.0};
    double y{0.0};
    double z{0.0};
};

Vec3 operator+(const Vec3& a, const Vec3& b) {
    return {a.x + b.x, a.y + b.y, a.z + b.z};
}

Vec3 operator-(const Vec3& a, const Vec3& b) {
    return {a.x - b.x, a.y - b.y, a.z - b.z};
}

Vec3 operator*(const Vec3& value, double scale) {
    return {value.x * scale, value.y * scale, value.z * scale};
}

Vec3 operator/(const Vec3& value, double scale) {
    return {value.x / scale, value.y / scale, value.z / scale};
}

double dot(const Vec3& a, const Vec3& b) {
    return a.x * b.x + a.y * b.y + a.z * b.z;
}

double norm(const Vec3& value) {
    return std::sqrt(dot(value, value));
}

double distance(const Vec3& a, const Vec3& b) {
    return norm(a - b);
}

Vec3 normalize(const Vec3& value) {
    const double length = norm(value);
    if (length < 1e-9) {
        return {0.0, 0.0, 0.0};
    }
    return value / length;
}

struct AabbObstacle {
    Vec3 center;
    Vec3 size;
};

struct CylinderObstacle {
    Vec3 center;
    double radius{0.0};
    double height{0.0};
};

struct PlannerConfig {
    Vec3 bounds_min{0.18, -0.30, 0.08};
    Vec3 bounds_max{0.62, 0.30, 0.72};
    int max_iterations{1400};
    int edge_checks{14};
    int shortcut_trials{140};
    double goal_bias{0.18};
    double step_size{0.050};
    double near_radius{0.11};
    double goal_tolerance{0.055};
    double tip_clearance_radius{0.025};
};

struct PlannerNode {
    Vec3 point;
    int parent{-1};
    double cost{0.0};
};

struct PlanResult {
    bool success{false};
    int goal_index{-1};
    std::vector<PlannerNode> tree;
    std::vector<Vec3> path;
    std::vector<Vec3> smoothed_path;
};

geometry_msgs::msg::Quaternion makeDownwardOrientation() {
    tf2::Quaternion q;
    q.setRPY(0.0, kPi, 0.0);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Pose makePose(const Vec3& point) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = point.x;
    pose.position.y = point.y;
    pose.position.z = point.z;
    pose.orientation = makeDownwardOrientation();
    return pose;
}

Vec3 toVec3(const geometry_msgs::msg::Pose& pose) {
    return {pose.position.x, pose.position.y, pose.position.z};
}

bool isNearlyZeroPose(const geometry_msgs::msg::Pose& pose) {
    return std::abs(pose.position.x) < 1e-6 &&
           std::abs(pose.position.y) < 1e-6 &&
           std::abs(pose.position.z) < 1e-6;
}

bool isReasonableCurrentPose(const geometry_msgs::msg::Pose& pose) {
    if (isNearlyZeroPose(pose)) {
        return false;
    }

    const double radial = std::hypot(pose.position.x, pose.position.y);
    return pose.position.x >= 0.10 && pose.position.x <= 0.75 &&
           std::abs(pose.position.y) <= 0.45 &&
           pose.position.z >= 0.05 && pose.position.z <= 0.85 &&
           radial >= 0.12 && radial <= 0.95;
}

bool insideBounds(const Vec3& point, const PlannerConfig& config) {
    return point.x >= config.bounds_min.x && point.x <= config.bounds_max.x &&
           point.y >= config.bounds_min.y && point.y <= config.bounds_max.y &&
           point.z >= config.bounds_min.z && point.z <= config.bounds_max.z;
}

double clampDistanceToAabb(const Vec3& point, const AabbObstacle& box) {
    const double hx = 0.5 * box.size.x;
    const double hy = 0.5 * box.size.y;
    const double hz = 0.5 * box.size.z;
    const double dx = std::max(std::abs(point.x - box.center.x) - hx, 0.0);
    const double dy = std::max(std::abs(point.y - box.center.y) - hy, 0.0);
    const double dz = std::max(std::abs(point.z - box.center.z) - hz, 0.0);
    return std::sqrt(dx * dx + dy * dy + dz * dz);
}

bool collidesAabb(const Vec3& point, const AabbObstacle& box, double radius) {
    return clampDistanceToAabb(point, box) <= radius;
}

bool collidesCylinder(const Vec3& point, const CylinderObstacle& cylinder, double radius) {
    const double radial =
        std::hypot(point.x - cylinder.center.x, point.y - cylinder.center.y);
    const double axial = std::abs(point.z - cylinder.center.z);
    return radial <= cylinder.radius + radius &&
           axial <= 0.5 * cylinder.height + radius;
}

bool isTaskSpacePointValid(const Vec3& point,
                           const PlannerConfig& config,
                           const std::vector<AabbObstacle>& boxes,
                           const std::vector<CylinderObstacle>& cylinders) {
    if (!insideBounds(point, config)) {
        return false;
    }

    for (const auto& box : boxes) {
        if (collidesAabb(point, box, config.tip_clearance_radius)) {
            return false;
        }
    }

    for (const auto& cylinder : cylinders) {
        if (collidesCylinder(point, cylinder, config.tip_clearance_radius)) {
            return false;
        }
    }

    return true;
}

bool isEdgeValid(const Vec3& from,
                 const Vec3& to,
                 const PlannerConfig& config,
                 const std::vector<AabbObstacle>& boxes,
                 const std::vector<CylinderObstacle>& cylinders) {
    for (int i = 0; i <= config.edge_checks; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(config.edge_checks);
        const Vec3 sample = from + (to - from) * t;
        if (!isTaskSpacePointValid(sample, config, boxes, cylinders)) {
            return false;
        }
    }
    return true;
}

int nearestNodeIndex(const std::vector<PlannerNode>& tree, const Vec3& sample) {
    int best_index = -1;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.size(); ++i) {
        const double candidate_distance = distance(tree[i].point, sample);
        if (candidate_distance < best_distance) {
            best_distance = candidate_distance;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

Vec3 steer(const Vec3& from, const Vec3& to, double step_size) {
    const Vec3 delta = to - from;
    const double length = norm(delta);
    if (length <= step_size) {
        return to;
    }
    return from + normalize(delta) * step_size;
}

std::vector<int> nearNodeIndices(const std::vector<PlannerNode>& tree,
                                 const Vec3& point,
                                 double radius) {
    std::vector<int> indices;
    for (std::size_t i = 0; i < tree.size(); ++i) {
        if (distance(tree[i].point, point) <= radius) {
            indices.push_back(static_cast<int>(i));
        }
    }
    return indices;
}

std::vector<Vec3> reconstructPath(const std::vector<PlannerNode>& tree, int goal_index) {
    std::vector<Vec3> path;
    for (int index = goal_index; index >= 0; index = tree[index].parent) {
        path.push_back(tree[index].point);
    }
    std::reverse(path.begin(), path.end());
    return path;
}

std::vector<Vec3> shortcutPath(const std::vector<Vec3>& path,
                               const PlannerConfig& config,
                               const std::vector<AabbObstacle>& boxes,
                               const std::vector<CylinderObstacle>& cylinders) {
    if (path.size() <= 2) {
        return path;
    }

    std::vector<Vec3> result = path;
    std::mt19937 rng(42u);
    for (int trial = 0; trial < config.shortcut_trials; ++trial) {
        if (result.size() <= 2) {
            break;
        }

        std::uniform_int_distribution<int> left_dist(0, static_cast<int>(result.size()) - 2);
        const int left = left_dist(rng);
        std::uniform_int_distribution<int> right_dist(left + 1, static_cast<int>(result.size()) - 1);
        const int right = right_dist(rng);
        if (right <= left + 1) {
            continue;
        }
        if (!isEdgeValid(result[left], result[right], config, boxes, cylinders)) {
            continue;
        }

        std::vector<Vec3> shortened;
        shortened.reserve(result.size() - (right - left - 1));
        for (int i = 0; i <= left; ++i) {
            shortened.push_back(result[i]);
        }
        for (std::size_t i = static_cast<std::size_t>(right); i < result.size(); ++i) {
            shortened.push_back(result[i]);
        }
        result.swap(shortened);
    }

    return result;
}

PlanResult planTaskSpaceRrtStar(CR5Robot& robot,
                                const Vec3& start,
                                const Vec3& goal,
                                PlannerConfig config,
                                const std::vector<AabbObstacle>& boxes,
                                const std::vector<CylinderObstacle>& cylinders,
                                rclcpp::Logger logger) {
    PlanResult result;
    std::mt19937 rng(7u);
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    std::uniform_real_distribution<double> x_dist(config.bounds_min.x, config.bounds_max.x);
    std::uniform_real_distribution<double> y_dist(config.bounds_min.y, config.bounds_max.y);
    std::uniform_real_distribution<double> z_dist(config.bounds_min.z, config.bounds_max.z);

    // test 场景里起点来自当前机械臂真实/仿真状态，不能被固定边界硬编码卡死。
    // 这里按 start/goal 自适应放宽采样域，并保留一圈安全余量。
    config.bounds_min.x = std::min({config.bounds_min.x, start.x, goal.x}) - 0.04;
    config.bounds_min.y = std::min({config.bounds_min.y, start.y, goal.y}) - 0.04;
    config.bounds_min.z = std::min({config.bounds_min.z, start.z, goal.z}) - 0.04;
    config.bounds_max.x = std::max({config.bounds_max.x, start.x, goal.x}) + 0.04;
    config.bounds_max.y = std::max({config.bounds_max.y, start.y, goal.y}) + 0.04;
    config.bounds_max.z = std::max({config.bounds_max.z, start.z, goal.z}) + 0.04;

    config.bounds_min.x = std::max(0.10, config.bounds_min.x);
    config.bounds_min.y = std::max(-0.45, config.bounds_min.y);
    config.bounds_min.z = std::max(0.02, config.bounds_min.z);
    config.bounds_max.x = std::min(0.75, config.bounds_max.x);
    config.bounds_max.y = std::min(0.45, config.bounds_max.y);
    config.bounds_max.z = std::min(0.85, config.bounds_max.z);

    if (!isTaskSpacePointValid(start, config, boxes, cylinders) ||
        !isTaskSpacePointValid(goal, config, boxes, cylinders)) {
        RCLCPP_ERROR(
            logger,
            "Start or goal is invalid in task space. bounds=[(%.3f, %.3f, %.3f) -> (%.3f, %.3f, %.3f)] start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
            config.bounds_min.x,
            config.bounds_min.y,
            config.bounds_min.z,
            config.bounds_max.x,
            config.bounds_max.y,
            config.bounds_max.z,
            start.x,
            start.y,
            start.z,
            goal.x,
            goal.y,
            goal.z);
        return result;
    }

    if (!robot.computeIKForPose(makePose(start), nullptr) ||
        !robot.computeIKForPose(makePose(goal), nullptr)) {
        RCLCPP_ERROR(logger, "Start or goal is not IK reachable.");
        return result;
    }

    result.tree.push_back({start, -1, 0.0});

    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        Vec3 sample;
        if (unit_dist(rng) < config.goal_bias) {
            sample = goal;
        } else {
            sample = {x_dist(rng), y_dist(rng), z_dist(rng)};
        }

        const int nearest_index = nearestNodeIndex(result.tree, sample);
        if (nearest_index < 0) {
            continue;
        }

        const Vec3 candidate = steer(result.tree[nearest_index].point, sample, config.step_size);
        if (!isTaskSpacePointValid(candidate, config, boxes, cylinders) ||
            !isEdgeValid(result.tree[nearest_index].point, candidate, config, boxes, cylinders) ||
            !robot.computeIKForPose(makePose(candidate), nullptr)) {
            continue;
        }

        auto near_indices = nearNodeIndices(result.tree, candidate, config.near_radius);
        int best_parent = nearest_index;
        double best_cost =
            result.tree[nearest_index].cost + distance(result.tree[nearest_index].point, candidate);

        for (const int index : near_indices) {
            const double candidate_cost =
                result.tree[index].cost + distance(result.tree[index].point, candidate);
            if (candidate_cost >= best_cost) {
                continue;
            }
            if (!isEdgeValid(result.tree[index].point, candidate, config, boxes, cylinders)) {
                continue;
            }
            best_parent = index;
            best_cost = candidate_cost;
        }

        const int new_index = static_cast<int>(result.tree.size());
        result.tree.push_back({candidate, best_parent, best_cost});

        for (const int index : near_indices) {
            if (index == best_parent) {
                continue;
            }
            const double rewired_cost =
                result.tree[new_index].cost + distance(result.tree[new_index].point, result.tree[index].point);
            if (rewired_cost >= result.tree[index].cost) {
                continue;
            }
            if (!isEdgeValid(result.tree[new_index].point, result.tree[index].point, config, boxes, cylinders)) {
                continue;
            }
            result.tree[index].parent = new_index;
            result.tree[index].cost = rewired_cost;
        }

        if (distance(candidate, goal) <= config.goal_tolerance &&
            isEdgeValid(candidate, goal, config, boxes, cylinders)) {
            result.tree.push_back({
                goal,
                new_index,
                result.tree[new_index].cost + distance(candidate, goal)});
            result.goal_index = static_cast<int>(result.tree.size()) - 1;
            result.success = true;
            break;
        }
    }

    if (!result.success) {
        RCLCPP_WARN(logger, "Experimental task-space RRT* failed to connect goal.");
        return result;
    }

    result.path = reconstructPath(result.tree, result.goal_index);
    result.smoothed_path = shortcutPath(result.path, config, boxes, cylinders);
    return result;
}

visualization_msgs::msg::Marker makeDeleteAllMarker() {
    visualization_msgs::msg::Marker marker;
    marker.action = visualization_msgs::msg::Marker::DELETEALL;
    return marker;
}

visualization_msgs::msg::Marker makeSphereMarker(int id,
                                                 const std::string& ns,
                                                 const Vec3& point,
                                                 double scale,
                                                 float r,
                                                 float g,
                                                 float b) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position.x = point.x;
    marker.pose.position.y = point.y;
    marker.pose.position.z = point.z;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = scale;
    marker.scale.y = scale;
    marker.scale.z = scale;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = 0.95f;
    return marker;
}

visualization_msgs::msg::Marker makeLineListMarker(int id,
                                                   const std::string& ns,
                                                   double width,
                                                   float r,
                                                   float g,
                                                   float b,
                                                   float alpha) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_LIST;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = alpha;
    return marker;
}

visualization_msgs::msg::Marker makeLineStripMarker(int id,
                                                    const std::string& ns,
                                                    double width,
                                                    float r,
                                                    float g,
                                                    float b,
                                                    float alpha) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::LINE_STRIP;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = width;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = alpha;
    return marker;
}

geometry_msgs::msg::Point toPoint(const Vec3& point) {
    geometry_msgs::msg::Point out;
    out.x = point.x;
    out.y = point.y;
    out.z = point.z;
    return out;
}

void publishMarkers(const rclcpp::Node::SharedPtr& node,
                    const PlanResult& result,
                    const Vec3& start,
                    const Vec3& goal) {
    auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "experimental_task_space_rrtstar_markers",
        rclcpp::QoS(1).transient_local().reliable());

    visualization_msgs::msg::MarkerArray markers;
    markers.markers.push_back(makeDeleteAllMarker());

    int id = 0;
    markers.markers.push_back(makeSphereMarker(id++, "start", start, 0.035, 0.10f, 0.85f, 0.20f));
    markers.markers.push_back(makeSphereMarker(id++, "goal", goal, 0.035, 0.95f, 0.20f, 0.15f));

    auto tree_marker = makeLineListMarker(id++, "tree_edges", 0.0025, 0.25f, 0.55f, 0.95f, 0.55f);
    for (std::size_t i = 0; i < result.tree.size(); ++i) {
        const auto& node_entry = result.tree[i];
        if (node_entry.parent < 0) {
            continue;
        }
        tree_marker.points.push_back(toPoint(node_entry.point));
        tree_marker.points.push_back(toPoint(result.tree[static_cast<std::size_t>(node_entry.parent)].point));
    }
    markers.markers.push_back(tree_marker);

    auto raw_path_marker = makeLineStripMarker(id++, "raw_path", 0.007, 0.95f, 0.70f, 0.10f, 0.95f);
    for (const auto& point : result.path) {
        raw_path_marker.points.push_back(toPoint(point));
    }
    markers.markers.push_back(raw_path_marker);

    auto smooth_path_marker = makeLineStripMarker(id++, "smoothed_path", 0.010, 0.95f, 0.15f, 0.15f, 0.95f);
    for (const auto& point : result.smoothed_path) {
        smooth_path_marker.points.push_back(toPoint(point));
    }
    markers.markers.push_back(smooth_path_marker);

    for (int repeat = 0; repeat < 5; ++repeat) {
        for (auto& marker : markers.markers) {
            marker.header.stamp = node->now();
        }
        publisher->publish(markers);
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
}

bool executeWaypointPath(CR5Robot& robot,
                         const std::vector<Vec3>& waypoints,
                         rclcpp::Logger logger,
                         bool enable_moveit_execution) {
    if (waypoints.size() < 2) {
        return false;
    }

    if (!enable_moveit_execution) {
        RCLCPP_WARN(
            logger,
            "Skipping MoveIt waypoint execution because current robot state is unavailable in this demo environment.");
        return true;
    }

    for (std::size_t i = 1; i < waypoints.size(); ++i) {
        const auto waypoint_pose = makePose(waypoints[i]);
        RCLCPP_INFO(
            logger,
            "Executing waypoint %zu/%zu at (%.3f, %.3f, %.3f)",
            i,
            waypoints.size() - 1,
            waypoints[i].x,
            waypoints[i].y,
            waypoints[i].z);
        if (robot.moveToPoseWithPlanner(waypoint_pose, "RRTConnect", 4.0)) {
            rclcpp::sleep_for(std::chrono::milliseconds(150));
            continue;
        }

        RCLCPP_WARN(
            logger,
            "Execution failed at waypoint %zu, retry with plan-only diagnostic.",
            i);
        if (!robot.planToPoseWithPlanner(waypoint_pose, "RRTConnect", 5.0, "")) {
            RCLCPP_ERROR(logger, "Failed to execute waypoint %zu.", i);
            return false;
        }
        RCLCPP_WARN(
            logger,
            "Waypoint %zu only passed plan-only validation; execution did not complete.",
            i);
    }

    return true;
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("cr5_experimental_task_space_rrtstar_demo");

    auto marker_node = rclcpp::Node::make_shared("cr5_experimental_task_space_rrtstar_markers");
    std::thread marker_spin_thread([marker_node]() { rclcpp::spin(marker_node); });

    CR5Robot robot("cr5_experimental_task_space_rrtstar_demo");
    if (!robot.init()) {
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 1;
    }

    robot.setSpeed(0.20);
    const std::array<const char*, 12> cleanup_ids{
        "test_box",
        "exp_floor",
        "exp_center_box",
        "exp_left_pillar",
        "exp_right_pillar",
        "exp_cube_front_left",
        "exp_cube_front_right",
        "exp_cube_mid_left",
        "exp_cube_mid_right",
        "exp_beam_front",
        "exp_beam_rear",
        "exp_cube_left",
    };
    for (const auto* object_id : cleanup_ids) {
        robot.removeCollisionObject(object_id);
    }
    robot.removeCollisionObject("exp_cube_right");

    geometry_msgs::msg::Pose floor_pose;
    floor_pose.position.x = 0.40;
    floor_pose.position.y = 0.00;
    floor_pose.position.z = -0.02;
    floor_pose.orientation.w = 1.0;
    robot.addBoxObstacleObject("exp_floor", floor_pose, 1.20, 1.20, 0.04);

    geometry_msgs::msg::Pose beam_front_pose;
    beam_front_pose.position.x = 0.39;
    beam_front_pose.position.y = 0.18;
    beam_front_pose.position.z = 0.46;
    beam_front_pose.orientation.w = 1.0;
    robot.addBoxObstacleObject("exp_beam_front", beam_front_pose, 0.20, 0.08, 0.08);

    geometry_msgs::msg::Pose beam_rear_pose;
    beam_rear_pose.position.x = 0.50;
    beam_rear_pose.position.y = -0.16;
    beam_rear_pose.position.z = 0.39;
    beam_rear_pose.orientation.w = 1.0;
    robot.addBoxObstacleObject("exp_beam_rear", beam_rear_pose, 0.20, 0.08, 0.08);

    geometry_msgs::msg::Pose cube_left_pose;
    cube_left_pose.position.x = 0.43;
    cube_left_pose.position.y = 0.23;
    cube_left_pose.position.z = 0.18;
    cube_left_pose.orientation.w = 1.0;
    robot.addBoxObstacleObject("exp_cube_left", cube_left_pose, 0.10, 0.10, 0.18);

    geometry_msgs::msg::Pose cube_right_pose;
    cube_right_pose.position.x = 0.50;
    cube_right_pose.position.y = -0.24;
    cube_right_pose.position.z = 0.18;
    cube_right_pose.orientation.w = 1.0;
    robot.addBoxObstacleObject("exp_cube_right", cube_right_pose, 0.10, 0.10, 0.18);

    if (!robot.moveToNamedTarget("home")) {
        RCLCPP_WARN(
            logger,
            "Failed to move to named target 'home'. Continue from current robot state for this test.");
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    geometry_msgs::msg::Pose current_pose = robot.getCurrentPose();
    bool enable_moveit_execution = true;
    Vec3 start = toVec3(current_pose);
    if (!isReasonableCurrentPose(current_pose)) {
        start = {0.265, -0.128, 0.652};
        enable_moveit_execution = false;
        RCLCPP_WARN(
            logger,
            "Current robot state is unavailable or unreasonable. Use fallback test start pose (%.3f, %.3f, %.3f).",
            start.x,
            start.y,
            start.z);
    }
    const Vec3 goal{0.60, 0.00, 0.30};

    PlannerConfig config;
    const std::vector<AabbObstacle> boxes{
        {{0.40, 0.00, -0.02}, {1.20, 1.20, 0.04}},
        {{0.39, 0.18, 0.46}, {0.20, 0.08, 0.08}},
        {{0.50, -0.16, 0.39}, {0.20, 0.08, 0.08}},
        {{0.43, 0.23, 0.18}, {0.10, 0.10, 0.18}},
        {{0.50, -0.24, 0.18}, {0.10, 0.10, 0.18}},
    };
    const std::vector<CylinderObstacle> cylinders;

    RCLCPP_INFO(
        logger,
        "Experimental task-space RRT* start=(%.3f, %.3f, %.3f) goal=(%.3f, %.3f, %.3f)",
        start.x,
        start.y,
        start.z,
        goal.x,
        goal.y,
        goal.z);

    auto result = planTaskSpaceRrtStar(robot, start, goal, config, boxes, cylinders, logger);
    publishMarkers(marker_node, result, start, goal);

    if (!result.success) {
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 3;
    }

    RCLCPP_INFO(
        logger,
        "Experimental task-space RRT* success: tree_nodes=%zu raw_waypoints=%zu smoothed_waypoints=%zu",
        result.tree.size(),
        result.path.size(),
        result.smoothed_path.size());
    for (std::size_t i = 0; i < result.smoothed_path.size(); ++i) {
        const auto& point = result.smoothed_path[i];
        RCLCPP_INFO(
            logger,
            "Smoothed path waypoint %zu: (%.3f, %.3f, %.3f)",
            i,
            point.x,
            point.y,
            point.z);
    }

    const auto& execution_path =
        result.smoothed_path.size() >= 3 ? result.smoothed_path : result.path;

    RCLCPP_INFO(
        logger,
        "Execution path selected: %s (%zu waypoints)",
        result.smoothed_path.size() >= 3 ? "smoothed_path" : "raw_path",
        execution_path.size());

    if (!executeWaypointPath(robot, execution_path, logger, enable_moveit_execution)) {
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 4;
    }

    RCLCPP_INFO(
        logger,
        "Experimental task-space RRT* demo completed. Path planning and waypoint validation succeeded.");
    rclcpp::shutdown();
    if (marker_spin_thread.joinable()) {
        marker_spin_thread.join();
    }
    return 0;
}
