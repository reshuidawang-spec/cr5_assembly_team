#include <rclcpp/rclcpp.hpp>
#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/result_utils.hpp"
#include <thread>
#include <fstream>
#include <iomanip>
#include <cmath>
#include <visualization_msgs/msg/marker.hpp>
#include <visualization_msgs/msg/marker_array.hpp>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>

//  标定球与测头参数配置 
const double SPHERE_X = 0.50;
const double SPHERE_Y = 0.00;
const double SPHERE_Z = 0.45;
const double SPHERE_RADIUS = 0.0125; // 12.5mm

// 测头参数 (与 cr5_robot.cpp 中的碰撞模型一致)
const double PROBE_LENGTH = my_cr5_control::probe::kProbeLength;       // 测头尖端总长
const double PROBE_BODY_RADIUS = my_cr5_control::probe::kProbeBodyRadius; // 测头主体半径

//  1: 定义星形测针的几何参数
// 依据 cr5_robot.cpp: star_x_pose.position.z = 0.126
const double STAR_STYLUS_Z_OFFSET = my_cr5_control::probe::kStarStylusZOffset; 
// 依据 cr5_robot.cpp: star_x dimensions, 旋转后半径约为 0.020
const double STAR_STYLUS_RADIUS = my_cr5_control::probe::kStarStylusReach;   

const double SAFE_HEIGHT = 0.75;
const double APPROACH_RETRACT_DIST = 0.10; 

struct CalibrationPoint {
    std::string name;
    geometry_msgs::msg::Pose target_pose;   
    geometry_msgs::msg::Pose approach_pose; 
};

// LookAt (让Z轴指向目标)
geometry_msgs::msg::Quaternion calculateLookAt(
    const geometry_msgs::msg::Point& source,
    const geometry_msgs::msg::Point& target)
{
    tf2::Vector3 position(source.x, source.y, source.z);
    tf2::Vector3 target_pos(target.x, target.y, target.z);
    tf2::Vector3 z_axis = (target_pos - position).normalized();
    
    tf2::Vector3 up(0, 0, 1);
    if (std::abs(z_axis.dot(up)) > 0.99) {
        up = tf2::Vector3(1, 0, 0);
    }

    tf2::Vector3 x_axis = up.cross(z_axis).normalized();
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();

    tf2::Matrix3x3 mat(
        x_axis.x(), y_axis.x(), z_axis.x(),
        x_axis.y(), y_axis.y(), z_axis.y(),
        x_axis.z(), y_axis.z(), z_axis.z()
    );
    tf2::Quaternion q;
    mat.getRotation(q);
    return tf2::toMsg(q);
}

// 垂直向下姿态
geometry_msgs::msg::Quaternion getVerticalDownOrientation() {
    tf2::Quaternion q;
    q.setRPY(0, M_PI, 0); 
    return tf2::toMsg(q);
}

std::vector<CalibrationPoint> generateConePoints() {
    std::vector<CalibrationPoint> points;
    geometry_msgs::msg::Point center;
    center.x = SPHERE_X; center.y = SPHERE_Y; center.z = SPHERE_Z;

    // 1. 顶点：测针尖端向下触碰球顶 (保持不变)
    {
        CalibrationPoint pt;
        pt.name = "Top";
        double flange_dist = SPHERE_RADIUS + PROBE_LENGTH;
        pt.target_pose.position.x = center.x;
        pt.target_pose.position.y = center.y;
        pt.target_pose.position.z = center.z + flange_dist; 
        pt.target_pose.orientation = calculateLookAt(pt.target_pose.position, center);

        pt.approach_pose = pt.target_pose;
        pt.approach_pose.position.z += APPROACH_RETRACT_DIST;
        points.push_back(pt);
    }

    // 2. 赤道4点：使用星形测针侧面触碰
    struct EquatorConfig { std::string name; double azimuth; };
    std::vector<EquatorConfig> equator_configs = {
        {"Equator_Front", 0.0}, 
        {"Equator_Left",  90.0},
        {"Equator_Back",  180.0},
        {"Equator_Right", 270.0}
    };

    for (const auto& cfg : equator_configs) {
        CalibrationPoint pt;
        pt.name = cfg.name;

        // 2: 水平距离计算
    
        double horizontal_dist = STAR_STYLUS_RADIUS + SPHERE_RADIUS;
        double theta = cfg.azimuth * M_PI / 180.0;

        pt.target_pose.position.x = center.x + horizontal_dist * std::cos(theta);
        pt.target_pose.position.y = center.y + horizontal_dist * std::sin(theta);
        
        // 3: Z轴高度补偿 
        // 姿态是垂直向下（Z轴朝下），因此法兰的高度必须高于球心。
        // 法兰高度 = 球心高度 + 星形测针相对于法兰的Z轴距离
        pt.target_pose.position.z = center.z + STAR_STYLUS_Z_OFFSET;
        pt.target_pose.orientation = getVerticalDownOrientation();
        pt.approach_pose = pt.target_pose;
        pt.approach_pose.position.x += APPROACH_RETRACT_DIST * std::cos(theta);
        pt.approach_pose.position.y += APPROACH_RETRACT_DIST * std::sin(theta);

        points.push_back(pt);
    }

    return points;
}

void visualizePoints(rclcpp::Node::SharedPtr node, const std::vector<CalibrationPoint>& points) {
    auto pub = node->create_publisher<visualization_msgs::msg::MarkerArray>("calibration_markers", 10);
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    visualization_msgs::msg::MarkerArray ma;
    int id = 0;
    visualization_msgs::msg::Marker sphere;
    sphere.header.frame_id = "base_link"; sphere.id = id++; sphere.type = visualization_msgs::msg::Marker::SPHERE;
    sphere.action = visualization_msgs::msg::Marker::ADD;
    sphere.pose.position.x = SPHERE_X; sphere.pose.position.y = SPHERE_Y; sphere.pose.position.z = SPHERE_Z;
    sphere.scale.x = SPHERE_RADIUS*2; sphere.scale.y = SPHERE_RADIUS*2; sphere.scale.z = SPHERE_RADIUS*2;
    sphere.color.r = 1.0; sphere.color.a = 0.5;
    ma.markers.push_back(sphere);
    for(const auto& p : points) {
        visualization_msgs::msg::Marker m = sphere; 
        m.id = id++; m.pose = p.target_pose; m.scale.x=0.01; m.scale.y=0.01; m.scale.z=0.01; 
        m.color.r=0; m.color.g=1; m.color.a=1; ma.markers.push_back(m);
        m.id = id++; m.pose = p.approach_pose; m.color.r=1; m.color.g=1; m.color.b=0; ma.markers.push_back(m);
        visualization_msgs::msg::Marker line = m;
        line.id = id++; line.type = visualization_msgs::msg::Marker::ARROW;
        line.scale.x = 0.005; line.scale.y = 0.01; line.scale.z = 0.0;
        line.points.push_back(p.approach_pose.position);
        line.points.push_back(p.target_pose.position);
        line.color.r=0.5; line.color.g=0.5; line.color.b=0.5;
        ma.markers.push_back(line);
    }
    for(int i=0;i<5;i++) { pub->publish(ma); std::this_thread::sleep_for(std::chrono::milliseconds(200)); }
}

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("tcp_calibration_simulation");
    CR5Robot robot("cr5_tcp_calibration_sim");
    if (!robot.init()) return 1;
    robot.setupCalibrationScene(); 
    const std::string timestamp = my_cr5_control::results::makeTimestamp();
    const std::string log_path =
        my_cr5_control::results::makeOutputPath(timestamp, "tcp_calibration_simulation_log.csv");
    std::ofstream log_file(log_path);
    log_file << "Point,X,Y,Z,QX,QY,QZ,QW\n";
    auto points = generateConePoints();

    // 创建可视化节点并在单独线程中运行
    auto node = rclcpp::Node::make_shared("viz_node");
    std::thread viz_thread([node]() { rclcpp::spin(node); });

    visualizePoints(node, points);
    RCLCPP_INFO(logger, ">>> 移动到安全高度...");
    geometry_msgs::msg::Pose safe_pose;
    safe_pose.position.x = SPHERE_X; safe_pose.position.y = SPHERE_Y; safe_pose.position.z = SAFE_HEIGHT;
    safe_pose.orientation = calculateLookAt(safe_pose.position, points[0].target_pose.position);
    robot.setSpeed(0.5);
    if (!robot.moveToPose(safe_pose)) {
        RCLCPP_ERROR(logger, "无法移动到安全高度，程序终止");
        rclcpp::shutdown();
        if (viz_thread.joinable()) viz_thread.join();
        return 1;
    }
    for (const auto& pt : points) {
        RCLCPP_INFO(logger, ">>> 执行标定点: %s", pt.name.c_str());
        robot.setSpeed(0.5);
        if (!robot.moveToPose(pt.approach_pose)) {
            RCLCPP_ERROR(logger, "预备点规划失败: %s", pt.name.c_str());
            continue; 
        }
        RCLCPP_INFO(logger, "    触碰中...");
        robot.setSpeed(0.1);
        double fraction = robot.moveLine(pt.target_pose);
        if (fraction > 0.95) {
            RCLCPP_INFO(logger, "✓ 触碰成功");
            auto p = robot.getCurrentPose();
            log_file << pt.name << "," << p.position.x << "," << p.position.y << "," << p.position.z << ","
                     << p.orientation.x << "," << p.orientation.y << "," << p.orientation.z << "," << p.orientation.w << std::endl;
        } else {
            RCLCPP_WARN(logger, "⚠ 触碰未完全到达 (%.1f%%)", fraction*100);
        }
        RCLCPP_INFO(logger, "    后退中...");
        robot.setSpeed(0.5);
        double retract_fraction = robot.moveLine(pt.approach_pose);
        if (retract_fraction < 0.95) {
            RCLCPP_WARN(logger, "⚠ 后退不完整 (%.1f%%)，使用RRT*规划返回", retract_fraction*100);
            robot.moveToPose(pt.approach_pose);
        }
    }
    RCLCPP_INFO(logger, ">>> TCP标定模拟完成，返回安全点");
    robot.moveToPose(safe_pose);
    log_file.close();
    RCLCPP_INFO(logger, "TCP标定模拟日志已保存到 %s", log_path.c_str());
    rclcpp::shutdown();
    if (viz_thread.joinable()) viz_thread.join();
    return 0;
}
