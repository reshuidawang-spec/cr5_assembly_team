#include <rclcpp/rclcpp.hpp>

#include <chrono>
#include <cstdint>
#include <cstdlib>
#include <iomanip>
#include <optional>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#include <geometry_msgs/msg/pose.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"

// This file implements a synthetic guide-visualization demo kept for
// debugging and presentation. It is intentionally not the canonical paper
// mainline, which is defined by the HeuristicGuided method on:
// 1. the controlled simple box benchmark
// 2. the v2 / WS119.STL benchmark

namespace {

std::uint32_t getSyntheticGuidanceGuideSeed() {
    const char* raw = my_cr5_control::env::firstValue({
        "MY_CR5_CONTROL_SYNTHETIC_GUIDANCE_GUIDE_SEED",
        "MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_SEED",
    });
    if (raw == nullptr) {
        return 12345u;
    }
    return my_cr5_control::env::parseUint32(raw).value_or(12345u);
}

geometry_msgs::msg::Quaternion makeDownwardOrientation() {
    tf2::Quaternion q;
    q.setRPY(0.0, 3.14159265358979323846, 0.0);
    return tf2::toMsg(q);
}

geometry_msgs::msg::Pose makePose(double x, double y, double z) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation = makeDownwardOrientation();
    return pose;
}

visualization_msgs::msg::Marker makeSphereMarker(
    int id,
    const std::string& ns,
    const geometry_msgs::msg::Pose& pose,
    double scale,
    float r,
    float g,
    float b,
    float a) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::SPHERE;
    marker.action = visualization_msgs::msg::Marker::ADD;
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

visualization_msgs::msg::Marker makeArrowMarker(
    int id,
    const std::string& ns,
    const geometry_msgs::msg::Pose& pose,
    float r,
    float g,
    float b,
    float a) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::ARROW;
    marker.action = visualization_msgs::msg::Marker::ADD;
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

visualization_msgs::msg::Marker makeLineStripMarker(
    int id,
    const std::string& ns,
    const std::vector<geometry_msgs::msg::Point>& points,
    double width,
    float r,
    float g,
    float b,
    float a) {
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
    marker.color.a = a;
    marker.points = points;
    return marker;
}

visualization_msgs::msg::Marker makeTextMarker(
    int id,
    const std::string& ns,
    const geometry_msgs::msg::Point& position,
    const std::string& text,
    double size,
    float r,
    float g,
    float b,
    float a) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = "base_link";
    marker.ns = ns;
    marker.id = id;
    marker.type = visualization_msgs::msg::Marker::TEXT_VIEW_FACING;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.position = position;
    marker.pose.orientation.w = 1.0;
    marker.scale.z = size;
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
    marker.text = text;
    return marker;
}

void publishSceneMarkers(
    const rclcpp::Node::SharedPtr& node,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& publisher,
    const rclcpp::Publisher<visualization_msgs::msg::MarkerArray>::SharedPtr& compat_publisher,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const std::vector<CR5Robot::GuideCandidate>& guides,
    const std::optional<CR5Robot::PlanningMetrics>& metrics = std::nullopt) {
    visualization_msgs::msg::MarkerArray markers;
    visualization_msgs::msg::Marker clear_marker;
    clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
    markers.markers.push_back(clear_marker);
    int id = 0;

    markers.markers.push_back(
        makeSphereMarker(id++, "start", start_pose, 0.035, 0.10f, 0.80f, 0.20f, 0.95f));
    markers.markers.push_back(
        makeArrowMarker(id++, "start_axis", start_pose, 0.10f, 0.80f, 0.20f, 0.90f));
    markers.markers.push_back(
        makeSphereMarker(id++, "goal", goal_pose, 0.035, 0.95f, 0.20f, 0.10f, 0.95f));
    markers.markers.push_back(
        makeArrowMarker(id++, "goal_axis", goal_pose, 0.95f, 0.20f, 0.10f, 0.90f));
    markers.markers.push_back(
        makeLineStripMarker(
            id++,
            "direct_path",
            {start_pose.position, goal_pose.position},
            0.003,
            0.85f,
            0.85f,
            0.85f,
            0.65f));

    const std::size_t limit = std::min<std::size_t>(10, guides.size());
    for (std::size_t i = 0; i < limit; ++i) {
        const auto& guide = guides[i];
        const float alpha = guide.enabled ? 0.75f : 0.25f;
        const float green = std::max(0.25f, 0.80f - static_cast<float>(0.05 * i));
        markers.markers.push_back(
            makeSphereMarker(
                id++,
                "guide_points",
                guide.pose,
                0.022,
                0.95f,
                green,
                0.15f,
                alpha));
        markers.markers.push_back(
            makeArrowMarker(
                id++,
                "guide_axes",
                guide.pose,
                0.95f,
                green,
                0.15f,
                alpha));
        markers.markers.push_back(
            makeLineStripMarker(
                id++,
                "guide_paths",
                {start_pose.position, guide.pose.position, goal_pose.position},
                0.0018,
                0.95f,
                green,
                0.15f,
                0.50f));
        geometry_msgs::msg::Point label_pos = guide.pose.position;
        label_pos.z += 0.025;
        std::ostringstream guide_text;
        guide_text << "#" << guide.candidate_id
                   << " s=" << std::fixed << std::setprecision(3) << guide.ranking_score;
        markers.markers.push_back(
            makeTextMarker(id++, "guide_labels", label_pos, guide_text.str(), 0.010, 1.0f, 0.95f, 0.75f, 0.95f));
    }

    if (metrics.has_value() && metrics->selected_candidate_id >= 0) {
        geometry_msgs::msg::Pose selected_pose = goal_pose;
        selected_pose.position = metrics->selected_candidate_point;
        markers.markers.push_back(
            makeSphereMarker(
                id++,
                "selected_candidate",
                selected_pose,
                0.030,
                0.10f,
                0.95f,
                0.95f,
                0.95f));
        markers.markers.push_back(
            makeLineStripMarker(
                id++,
                "selected_path",
                {start_pose.position, selected_pose.position, goal_pose.position},
                0.004,
                0.10f,
                0.95f,
                0.95f,
                0.85f));
    }

    geometry_msgs::msg::Point summary_pos = goal_pose.position;
    summary_pos.y += 0.20;
    summary_pos.z += 0.12;
    std::ostringstream summary;
    summary << "synthetic guidance demo\n"
            << "legacy synthetic scene\n"
            << "guides=" << guides.size();
    if (metrics.has_value()) {
        summary << "\nwall=" << std::fixed << std::setprecision(1) << metrics->wall_time_ms
                << " ms, attempted=" << metrics->guide_candidates_attempted
                << "\ndirect=" << (metrics->direct_plan_success ? "true" : "false")
                << ", fallback=" << (metrics->used_direct_plan ? "true" : "false")
                << "\nselected=" << metrics->selected_candidate_id
                << ", top=" << metrics->top_ranked_candidate_id;
    }
    markers.markers.push_back(
        makeTextMarker(id++, "summary", summary_pos, summary.str(), 0.018, 0.95f, 0.95f, 1.0f, 0.98f));

    for (int repeat = 0; repeat < 5; ++repeat) {
        for (auto& marker : markers.markers) {
            marker.header.stamp = node->now();
        }
        publisher->publish(markers);
        if (compat_publisher) {
            compat_publisher->publish(markers);
        }
        rclcpp::sleep_for(std::chrono::milliseconds(200));
    }
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("synthetic_guidance_demo");
    const std::uint32_t guide_seed = getSyntheticGuidanceGuideSeed();

    auto marker_node = rclcpp::Node::make_shared("synthetic_guidance_demo_markers");
    std::thread marker_spin_thread([marker_node]() { rclcpp::spin(marker_node); });
    auto marker_publisher = marker_node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "synthetic_guidance_demo_markers",
        rclcpp::QoS(1).transient_local().reliable());
    auto marker_compat_publisher = marker_node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "visualization_marker_array",
        rclcpp::QoS(1).transient_local().reliable());

    CR5Robot robot("synthetic_guidance_demo");
    if (!robot.init()) {
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 1;
    }

    robot.setSpeed(0.25);
    robot.enableAdaptiveEllipsoidSampling(true);
    robot.setSceneDifficultyScore(0.85);
    robot.setGuideMaxAttempts(6);
    robot.setGuideSamplingSeed(guide_seed);
    RCLCPP_INFO(logger, "Synthetic guidance demo guide seed fixed to %u", guide_seed);

    robot.removeCollisionObject("test_box");
    robot.removeCollisionObject("demo_pillar_left");
    robot.removeCollisionObject("demo_pillar_right");

    const geometry_msgs::msg::Pose target_pose = makePose(0.46, 0.00, 0.34);

    robot.addBoxObstacle(0.34, 0.00, 0.26, 0.10, 0.34, 0.20);

    geometry_msgs::msg::Pose left_pillar = makePose(0.36, 0.14, 0.28);
    geometry_msgs::msg::Pose right_pillar = makePose(0.36, -0.14, 0.28);
    robot.addCylinderObstacle("demo_pillar_left", left_pillar, 0.24, 0.035);
    robot.addCylinderObstacle("demo_pillar_right", right_pillar, 0.24, 0.035);

    if (!robot.moveToNamedTarget("home")) {
        RCLCPP_ERROR(logger, "Failed to move to named target 'home'.");
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 1;
    }

    rclcpp::sleep_for(std::chrono::milliseconds(500));
    const geometry_msgs::msg::Pose start_pose = robot.getCurrentPose();
    robot.setGuideSamplingSeed(guide_seed);
    const auto guide_candidates = robot.sampleGuideCandidates(target_pose, "", 24, true);
    publishSceneMarkers(
        marker_node,
        marker_publisher,
        marker_compat_publisher,
        start_pose,
        target_pose,
        guide_candidates);

    RCLCPP_INFO(
        logger,
        "Planning demo started: start=(%.3f, %.3f, %.3f), goal=(%.3f, %.3f, %.3f), guides=%zu",
        start_pose.position.x,
        start_pose.position.y,
        start_pose.position.z,
        target_pose.position.x,
        target_pose.position.y,
        target_pose.position.z,
        guide_candidates.size());

    CR5Robot::PlanningMetrics metrics;
    robot.setGuideSamplingSeed(guide_seed);
    const bool planning_success =
        robot.planToPoseImproved(target_pose, "", 8.0, &metrics, 24, nullptr);

    if (!planning_success) {
        RCLCPP_ERROR(logger, "Synthetic guidance planning failed.");
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 2;
    }

    RCLCPP_INFO(
        logger,
        "Plan success: wall=%.1f ms, candidates=%d, attempted=%d, direct_success=%s, top_candidate=%d",
        metrics.wall_time_ms,
        metrics.guide_candidate_count,
        metrics.guide_candidates_attempted,
        metrics.direct_plan_success ? "true" : "false",
        metrics.top_ranked_candidate_id);
    publishSceneMarkers(
        marker_node,
        marker_publisher,
        marker_compat_publisher,
        start_pose,
        target_pose,
        guide_candidates,
        metrics);

    robot.setGuideSamplingSeed(guide_seed);
    const bool execution_success = robot.moveToPoseImproved(target_pose);
    if (!execution_success) {
        RCLCPP_ERROR(logger, "Execution via HeuristicGuided failed.");
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) {
            marker_spin_thread.join();
        }
        return 3;
    }

    RCLCPP_INFO(logger, "Synthetic guidance demo completed successfully.");
    rclcpp::shutdown();
    if (marker_spin_thread.joinable()) {
        marker_spin_thread.join();
    }
    return 0;
}
