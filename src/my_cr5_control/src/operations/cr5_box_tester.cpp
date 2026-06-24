#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/result_utils.hpp"
#include "my_cr5_control/scene_utils.hpp"
#include <interactive_markers/interactive_marker_server.hpp>
#include <visualization_msgs/msg/interactive_marker.hpp>
#include <visualization_msgs/msg/interactive_marker_control.hpp>
#include <visualization_msgs/msg/interactive_marker_feedback.hpp>
#include <visualization_msgs/msg/marker.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <fstream>
#include <functional>
#include <iostream>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

namespace {

constexpr std::size_t kRequiredTeachPoints = 3;
constexpr double kApproachDistance = 0.08;
constexpr double kSafeHeightMargin = 0.15;

geometry_msgs::msg::Quaternion getVerticalDownOrientation() {
    tf2::Quaternion q;
    q.setRPY(0.0, 3.14159265358979323846, 0.0);
    return tf2::toMsg(q);
}

struct RecordedTeachPoint {
    std::string name;
    geometry_msgs::msg::Point tip_point;
    geometry_msgs::msg::Pose target_flange_pose;
    geometry_msgs::msg::Pose approach_pose;
};

class ProbeTeachInteractiveMarker {
public:
    explicit ProbeTeachInteractiveMarker(const rclcpp::Node::SharedPtr& node) : node_(node) {
        server_ = std::make_shared<interactive_markers::InteractiveMarkerServer>("/probe_teach_marker_server", node_);
        current_tip_pose_.orientation.w = 1.0;
        current_tip_pose_.position.x = 0.45;
        current_tip_pose_.position.y = 0.00;
        current_tip_pose_.position.z = 0.30;
        createMarker();
    }

    geometry_msgs::msg::Pose getCurrentTipPose() const {
        std::lock_guard<std::mutex> lock(pose_mutex_);
        return current_tip_pose_;
    }

private:
    void createMarker() {
        visualization_msgs::msg::InteractiveMarker marker;
        marker.header.frame_id = "base_link";
        marker.name = "probe_tip_teach";
        marker.description = "Drag tip point in RViz then press Enter";
        marker.scale = 0.20;
        marker.pose = current_tip_pose_;

        visualization_msgs::msg::Marker tip_sphere;
        tip_sphere.type = visualization_msgs::msg::Marker::SPHERE;
        tip_sphere.scale.x = 0.016;
        tip_sphere.scale.y = 0.016;
        tip_sphere.scale.z = 0.016;
        tip_sphere.color.r = 1.0;
        tip_sphere.color.g = 0.3;
        tip_sphere.color.b = 0.1;
        tip_sphere.color.a = 0.95;

        visualization_msgs::msg::InteractiveMarkerControl visual_control;
        visual_control.always_visible = true;
        visual_control.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_3D;
        visual_control.markers.push_back(tip_sphere);
        marker.controls.push_back(visual_control);

        visualization_msgs::msg::InteractiveMarkerControl move_x;
        move_x.name = "move_x";
        move_x.orientation.w = 1.0;
        move_x.orientation.x = 1.0;
        move_x.orientation.y = 0.0;
        move_x.orientation.z = 0.0;
        move_x.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(move_x);

        visualization_msgs::msg::InteractiveMarkerControl move_y;
        move_y.name = "move_y";
        move_y.orientation.w = 1.0;
        move_y.orientation.x = 0.0;
        move_y.orientation.y = 1.0;
        move_y.orientation.z = 0.0;
        move_y.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(move_y);

        visualization_msgs::msg::InteractiveMarkerControl move_z;
        move_z.name = "move_z";
        move_z.orientation.w = 1.0;
        move_z.orientation.x = 0.0;
        move_z.orientation.y = 0.0;
        move_z.orientation.z = 1.0;
        move_z.interaction_mode = visualization_msgs::msg::InteractiveMarkerControl::MOVE_AXIS;
        marker.controls.push_back(move_z);

        server_->insert(marker, std::bind(&ProbeTeachInteractiveMarker::handleFeedback, this, std::placeholders::_1));
        server_->applyChanges();
    }

    void handleFeedback(const visualization_msgs::msg::InteractiveMarkerFeedback::ConstSharedPtr& feedback) {
        if (feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::POSE_UPDATE ||
            feedback->event_type == visualization_msgs::msg::InteractiveMarkerFeedback::MOUSE_UP) {
            std::lock_guard<std::mutex> lock(pose_mutex_);
            current_tip_pose_ = feedback->pose;
        }
    }

    rclcpp::Node::SharedPtr node_;
    std::shared_ptr<interactive_markers::InteractiveMarkerServer> server_;
    mutable std::mutex pose_mutex_;
    geometry_msgs::msg::Pose current_tip_pose_;
};

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("box_tester_node");

    auto marker_node = rclcpp::Node::make_shared("probe_teach_marker_node");
    auto marker_server = std::make_shared<ProbeTeachInteractiveMarker>(marker_node);
    std::thread marker_spin_thread([marker_node]() { rclcpp::spin(marker_node); });

    auto robot = std::make_shared<CR5Robot>("box_tester_node");
    if (!robot->init()) {
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) marker_spin_thread.join();
        return 1;
    }

    robot->addSimulationEnvironment();

    RCLCPP_INFO(logger, "RViz示教模式已启动");
    RCLCPP_INFO(logger, "请在 RViz 添加 InteractiveMarkers 显示，更新话题为 /probe_teach_marker_server/update");
    RCLCPP_INFO(logger, "拖动测针点到目标位置后，在终端按 Enter 记录坐标，共需记录 %zu 个点", kRequiredTeachPoints);

    std::vector<RecordedTeachPoint> recorded_points;
    recorded_points.reserve(kRequiredTeachPoints);

    for (std::size_t i = 0; i < kRequiredTeachPoints && rclcpp::ok(); ++i) {
        std::string input;
        std::cout << "\n========================================" << std::endl;
        std::cout << "示教点 " << (i + 1) << "/" << kRequiredTeachPoints << std::endl;
        std::cout << "请在 RViz 中拖动标记到目标位置" << std::endl;
        std::cout << "拖动完成后按 Enter 键记录坐标..." << std::endl;
        std::cout << "========================================" << std::endl;
        std::getline(std::cin, input);

        const auto tip_pose = marker_server->getCurrentTipPose();
        RecordedTeachPoint pt;
        pt.name = "P" + std::to_string(i + 1);
        pt.tip_point = tip_pose.position;
        pt.target_flange_pose =
            my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_pose.position, getVerticalDownOrientation());
        pt.approach_pose = pt.target_flange_pose;
        pt.approach_pose.position.z += kApproachDistance;
        recorded_points.push_back(pt);

        RCLCPP_INFO(logger, "✓ 已记录 %s: tip(%.4f, %.4f, %.4f)",
                    pt.name.c_str(), pt.tip_point.x, pt.tip_point.y, pt.tip_point.z);
    }

    if (recorded_points.size() < kRequiredTeachPoints) {
        RCLCPP_ERROR(logger, "记录点不足，程序终止");
        rclcpp::shutdown();
        if (marker_spin_thread.joinable()) marker_spin_thread.join();
        return 1;
    }

    const std::string timestamp = my_cr5_control::results::makeTimestamp();
    const std::string log_path =
        my_cr5_control::results::makeOutputPath(timestamp, "box_rviz_teaching_log.csv");
    std::ofstream log_file(log_path);
    log_file << "Point,TipX,TipY,TipZ,PlanType,Fraction,FlangeX,FlangeY,FlangeZ,QX,QY,QZ,QW\n";

    // 计算安全位姿：使用第一个点的XY坐标，但使用更保守的Z高度
    geometry_msgs::msg::Pose safe_pose = recorded_points.front().target_flange_pose;
    safe_pose.position.z += kSafeHeightMargin;

    RCLCPP_INFO(logger, ">>> 移动到安全位姿 (%.3f, %.3f, %.3f)...",
                safe_pose.position.x, safe_pose.position.y, safe_pose.position.z);

    robot->setSpeed(0.30);
    bool reached_safe_pose = false;

    // 尝试1：使用改进算法
    if (robot->moveToPoseImproved(safe_pose)) {
        reached_safe_pose = true;
        RCLCPP_INFO(logger, "✓ 使用改进算法到达安全位姿");
    } else {
        RCLCPP_WARN(logger, "改进算法失败，尝试标准规划...");
        // 尝试2：使用标准规划
        if (robot->moveToPose(safe_pose)) {
            reached_safe_pose = true;
            RCLCPP_INFO(logger, "✓ 使用标准算法到达安全位姿");
        } else {
            RCLCPP_WARN(logger, "标准规划失败，尝试降低安全高度...");
            // 尝试3：降低安全高度重试
            safe_pose.position.z -= 0.10;  // 降低10cm
            if (robot->moveToPose(safe_pose)) {
                reached_safe_pose = true;
                RCLCPP_INFO(logger, "✓ 使用降低高度到达安全位姿");
            }
        }
    }

    if (!reached_safe_pose) {
        RCLCPP_WARN(logger, "⚠ 无法到达安全位姿，将直接从当前位置开始执行路径点");
    }

    for (const auto& pt : recorded_points) {
        RCLCPP_INFO(logger, ">>> 执行路径点: %s", pt.name.c_str());

        robot->setSpeed(0.25);
        bool reached_approach = false;

        // 尝试到达预备点
        if (robot->moveToPoseImproved(pt.approach_pose)) {
            reached_approach = true;
        } else {
            RCLCPP_WARN(logger, "改进算法失败，尝试标准规划到预备点...");
            if (robot->moveToPose(pt.approach_pose)) {
                reached_approach = true;
            }
        }

        if (!reached_approach) {
            RCLCPP_WARN(logger, "⚠ 预备点规划失败，跳过该点: %s", pt.name.c_str());
            continue;
        }

        RCLCPP_INFO(logger, "    执行直线触碰...");
        robot->setSpeed(0.05);
        const double fraction = robot->moveLine(pt.target_flange_pose);
        const auto current_pose = robot->getCurrentPose();

        log_file << pt.name << ","
                 << pt.tip_point.x << "," << pt.tip_point.y << "," << pt.tip_point.z << ","
                 << "cartesian," << fraction << ","
                 << current_pose.position.x << "," << current_pose.position.y << "," << current_pose.position.z << ","
                 << current_pose.orientation.x << "," << current_pose.orientation.y << ","
                 << current_pose.orientation.z << "," << current_pose.orientation.w << "\n";

        if (fraction > 0.95) {
            RCLCPP_INFO(logger, "✓ 触碰成功 (%.1f%%)", fraction * 100.0);
        } else {
            RCLCPP_WARN(logger, "⚠ 触碰不完整 (%.1f%%)", fraction * 100.0);
        }

        RCLCPP_INFO(logger, "    后退中...");
        robot->setSpeed(0.20);
        const double retract_fraction = robot->moveLine(pt.approach_pose);
        if (retract_fraction < 0.95) {
            RCLCPP_WARN(logger, "⚠ 后退不完整 (%.1f%%)，使用规划返回", retract_fraction * 100.0);
            if (!robot->moveToPose(pt.approach_pose)) {
                RCLCPP_ERROR(logger, "⚠ 无法返回预备点");
            }
        }
    }

    RCLCPP_INFO(logger, ">>> 返回安全位姿...");
    robot->setSpeed(0.30);
    if (!robot->moveToPose(safe_pose)) {
        RCLCPP_WARN(logger, "⚠ 无法返回安全位姿，保持当前位置");
    }

    log_file.close();
    RCLCPP_INFO(logger, "✓ RViz示教三点路径规划执行完成，日志已写入 %s", log_path.c_str());

    rclcpp::shutdown();
    if (marker_spin_thread.joinable()) marker_spin_thread.join();
    return 0;
}
