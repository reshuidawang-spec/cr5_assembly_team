#include "my_cr5_control/cr5_robot.hpp"
#include "my_cr5_control/env_utils.hpp"
#include "my_cr5_control/paper_mainline/guide_bridge.hpp"
#include "my_cr5_control/paper_mainline/guide_geometry.hpp"
#include "my_cr5_control/paper_mainline/guide_policy_params.hpp"
#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/scene_utils.hpp"
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2/LinearMath/Matrix3x3.h>
#include <moveit_msgs/msg/collision_object.hpp>
#include <moveit_msgs/msg/attached_collision_object.hpp>
#include <moveit_msgs/msg/planning_scene_components.hpp>
#include <moveit_msgs/srv/get_planning_scene.hpp>
#include <shape_msgs/msg/solid_primitive.hpp>
#include <moveit_msgs/msg/constraints.hpp>
#include <moveit_msgs/msg/position_constraint.hpp>
#include <geometric_shapes/shape_operations.h>
#include <moveit/collision_detection/collision_common.h>
#include <moveit/planning_scene/planning_scene.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <moveit/trajectory_processing/iterative_time_parameterization.h>
#include <boost/variant/get.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/SVD>
#include <algorithm>
#include <cctype>
#include <chrono>
#include <cmath>
#include <cstdlib>
#include <future>
#include <limits>
#include <memory>
#include <random>

static const std::string PLANNING_GROUP = "cr5_group";

namespace {

namespace paper = my_cr5_control::paper_mainline;

constexpr double kBudgetHitToleranceMs = 100.0;

bool isBudgetHit(double wall_time_ms, double budget_ms) {
    return budget_ms > 0.0 && wall_time_ms >= std::max(0.0, budget_ms - kBudgetHitToleranceMs);
}

double elapsedMs(const std::chrono::steady_clock::time_point& start,
                 const std::chrono::steady_clock::time_point& end) {
    return std::chrono::duration_cast<std::chrono::milliseconds>(end - start).count();
}

double sanitizeReportedPlanningTimeMs(double reported_ms,
                                      double fallback_ms,
                                      double planning_budget_ms) {
    if (!std::isfinite(reported_ms) || reported_ms < 0.0) {
        return fallback_ms;
    }

    const double sane_upper_bound = std::max({
        fallback_ms + 100.0,
        planning_budget_ms + 100.0,
        100.0,
    });

    if (reported_ms > sane_upper_bound) {
        return fallback_ms;
    }

    return reported_ms;
}

std::size_t getGuideMaxAttemptsFromEnv() {
    return static_cast<std::size_t>(my_cr5_control::env::getUnsignedLongClamped(
        "MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS", 0ul, 0ul, 1024ul));
}

double getGuideSlowDirectThresholdMsFromEnv() {
    return my_cr5_control::env::getDoubleClamped(
        "MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS", 0.0, 0.0, 10000.0);
}

bool getGuideBridgeEnabledFromEnv() {
    return my_cr5_control::env::getBool("MY_CR5_CONTROL_HEURISTIC_GUIDE_BRIDGE", false);
}

std::size_t getGuideBridgeMaxSequencesFromEnv() {
    return static_cast<std::size_t>(
        my_cr5_control::env::getUnsignedLongClamped(
            "MY_CR5_CONTROL_HEURISTIC_GUIDE_BRIDGE_MAX_SEQUENCES",
            paper::kGuideBridgeDefaultMaxSequences,
            1ul,
            24ul));
}

enum class HeuristicAblationMode {
    Full,
    DirectOnly,
    FixedGuide,
    NoAnchors,
    NoAdaptiveDifficulty,
    AlwaysGuide,
};

std::string normalizedAblationToken(std::string token) {
    token.erase(
        std::remove_if(token.begin(), token.end(), [](unsigned char ch) {
            return ch == '_' || ch == '-' || std::isspace(ch);
        }),
        token.end());
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return token;
}

HeuristicAblationMode getHeuristicAblationModeFromEnv() {
    const std::string token = normalizedAblationToken(
        my_cr5_control::env::getString("MY_CR5_CONTROL_HEURISTIC_ABLATION_MODE", "full"));
    if (token == "directonly") {
        return HeuristicAblationMode::DirectOnly;
    }
    if (token == "fixedguide") {
        return HeuristicAblationMode::FixedGuide;
    }
    if (token == "noanchors") {
        return HeuristicAblationMode::NoAnchors;
    }
    if (token == "noadaptivedifficulty") {
        return HeuristicAblationMode::NoAdaptiveDifficulty;
    }
    if (token == "alwaysguide") {
        return HeuristicAblationMode::AlwaysGuide;
    }
    return HeuristicAblationMode::Full;
}

bool usesAdaptiveDifficulty(HeuristicAblationMode mode) {
    return mode != HeuristicAblationMode::FixedGuide &&
           mode != HeuristicAblationMode::NoAdaptiveDifficulty;
}

bool usesEnvironmentAnchors(HeuristicAblationMode mode) {
    return mode != HeuristicAblationMode::FixedGuide &&
           mode != HeuristicAblationMode::NoAnchors;
}

bool usesGuideRefinement(HeuristicAblationMode mode) {
    return mode != HeuristicAblationMode::FixedGuide;
}

bool usesAdaptiveGuideFirst(HeuristicAblationMode mode) {
    return mode != HeuristicAblationMode::FixedGuide &&
           mode != HeuristicAblationMode::NoAdaptiveDifficulty;
}

bool forceGuideFirst(HeuristicAblationMode mode) {
    return mode == HeuristicAblationMode::AlwaysGuide;
}

double effectiveDifficultyForMode(HeuristicAblationMode mode, bool adaptive_enabled, double scene_difficulty) {
    if (!adaptive_enabled) {
        return 0.0;
    }
    if (mode == HeuristicAblationMode::NoAdaptiveDifficulty) {
        return 0.5;
    }
    if (mode == HeuristicAblationMode::FixedGuide) {
        return 0.0;
    }
    return scene_difficulty;
}

}  // namespace

CR5Robot::CR5Robot(const std::string& node_name, bool attach_probe_model)
    : attach_probe_model_(attach_probe_model) {
    rclcpp::NodeOptions node_options;
    node_options.automatically_declare_parameters_from_overrides(true);
    node_ = rclcpp::Node::make_shared(node_name, node_options);
    executor_ = std::make_shared<rclcpp::executors::SingleThreadedExecutor>();
    executor_->add_node(node_);
    executor_thread_ = std::thread([this]() { executor_->spin(); });
    setGuideMaxAttempts(getGuideMaxAttemptsFromEnv());
}

CR5Robot::~CR5Robot() {
    if (executor_) {
        executor_->cancel();
    }
    if (executor_thread_.joinable()) {
        executor_thread_.join();
    }
    if (executor_ && node_) {
        executor_->remove_node(node_);
    }
}

bool CR5Robot::init() {
    try {
        std::this_thread::sleep_for(std::chrono::seconds(1));
        move_group_ = std::make_shared<moveit::planning_interface::MoveGroupInterface>(node_, PLANNING_GROUP);

        move_group_->setPlannerId("RRTConnect");
        move_group_->setPlanningTime(8.0);
        move_group_->setNumPlanningAttempts(10);
        move_group_->setMaxVelocityScalingFactor(0.3);
        move_group_->setMaxAccelerationScalingFactor(0.2);
        move_group_->setGoalJointTolerance(0.01);
        move_group_->setGoalPositionTolerance(0.01);
        move_group_->setGoalOrientationTolerance(0.05);

        move_group_->setWorkspace(-1.0, -1.0, 0.02, 1.0, 1.0, 1.5);

        if (attach_probe_model_) {
            attachProbeModel();
        } else {
            RCLCPP_INFO(node_->get_logger(), "已启用干净喷涂场景：不附加测针碰撞模型");
        }

        RCLCPP_INFO(node_->get_logger(), "✓ CR5 初始化完毕");
        return true;
    } catch (const std::exception& e) {
        RCLCPP_ERROR(node_->get_logger(), "初始化失败: %s", e.what());
        return false;
    }
}

bool CR5Robot::isReady() const {
    return static_cast<bool>(move_group_);
}

bool CR5Robot::addBoxObstacleObject(const std::string& object_id,
                                    const geometry_msgs::msg::Pose& box_pose,
                                    double dx,
                                    double dy,
                                    double dz) {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "addBoxObstacleObject 失败：MoveGroup 尚未初始化");
        return false;
    }
    if (object_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "addBoxObstacleObject 失败：object_id 不能为空");
        return false;
    }
    if (dx <= 0.0 || dy <= 0.0 || dz <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "addBoxObstacleObject 失败：尺寸必须大于 0");
        return false;
    }

    moveit_msgs::msg::CollisionObject box;
    box.header.frame_id = "base_link";
    box.id = object_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.BOX;
    primitive.dimensions = {dx, dy, dz};

    box.primitives.push_back(primitive);
    box.primitive_poses.push_back(box_pose);
    box.operation = box.ADD;

    planning_scene_interface_.applyCollisionObject(box);
    RCLCPP_INFO(
        node_->get_logger(),
        "已添加箱体障碍物 %s (尺寸: %.2fm x %.2fm x %.2fm)",
        object_id.c_str(),
        dx,
        dy,
        dz);
    return true;
}

// 新增功能：添加箱体障碍物
void CR5Robot::addBoxObstacle(double x, double y, double z, double dx, double dy, double dz) {
    geometry_msgs::msg::Pose pose;
    pose.position.x = x;
    pose.position.y = y;
    pose.position.z = z;
    pose.orientation.w = 1.0;

    if (!addBoxObstacleObject("test_box", pose, dx, dy, dz)) {
        return;
    }

    has_environment_box_ = true;
    environment_box_x_ = x;
    environment_box_y_ = y;
    environment_box_z_ = z;
    environment_box_dx_ = dx;
    environment_box_dy_ = dy;
    environment_box_dz_ = dz;
}

bool CR5Robot::addMeshObstacle(const std::string& object_id, const std::string& mesh_resource,
                               const geometry_msgs::msg::Pose& mesh_pose, double scale) {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：MoveGroup 尚未初始化");
        return false;
    }
    if (object_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：object_id 不能为空");
        return false;
    }
    if (scale <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：scale 必须大于 0");
        return false;
    }

    // 关键步骤：从资源路径加载 STL 网格并缩放到米制
    std::unique_ptr<shapes::Mesh> mesh(
        shapes::createMeshFromResource(mesh_resource, Eigen::Vector3d(scale, scale, scale)));
    if (!mesh) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：无法加载网格资源 %s", mesh_resource.c_str());
        return false;
    }

    shapes::ShapeMsg shape_msg;
    if (!shapes::constructMsgFromShape(mesh.get(), shape_msg)) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：网格消息转换失败");
        return false;
    }

    const auto* mesh_msg = boost::get<shape_msgs::msg::Mesh>(&shape_msg);
    if (mesh_msg == nullptr) {
        RCLCPP_ERROR(node_->get_logger(), "addMeshObstacle 失败：转换后的消息不是 Mesh");
        return false;
    }

    moveit_msgs::msg::CollisionObject mesh_object;
    mesh_object.header.frame_id = "base_link";
    mesh_object.id = object_id;
    mesh_object.meshes.push_back(*mesh_msg);
    mesh_object.mesh_poses.push_back(mesh_pose);
    mesh_object.operation = mesh_object.ADD;
    planning_scene_interface_.applyCollisionObject(mesh_object);

    auto known_objects = planning_scene_interface_.getKnownObjectNames();
    const bool exists = std::find(known_objects.begin(), known_objects.end(), object_id) != known_objects.end();
    if (!exists) {
        RCLCPP_WARN(node_->get_logger(), "网格对象 %s 已提交，但尚未在已知对象列表中确认", object_id.c_str());
    } else {
        RCLCPP_INFO(node_->get_logger(), "已添加网格障碍物 %s (resource=%s, scale=%.6f)",
                    object_id.c_str(), mesh_resource.c_str(), scale);
    }
    return true;
}

bool CR5Robot::addCylinderObstacle(const std::string& object_id,
                                   const geometry_msgs::msg::Pose& cylinder_pose,
                                   double height_m,
                                   double radius_m) {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "addCylinderObstacle 失败：MoveGroup 尚未初始化");
        return false;
    }
    if (object_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "addCylinderObstacle 失败：object_id 不能为空");
        return false;
    }
    if (height_m <= 0.0 || radius_m <= 0.0) {
        RCLCPP_ERROR(node_->get_logger(), "addCylinderObstacle 失败：高度和半径必须大于 0");
        return false;
    }

    moveit_msgs::msg::CollisionObject cylinder;
    cylinder.header.frame_id = move_group_->getPlanningFrame();
    cylinder.id = object_id;

    shape_msgs::msg::SolidPrimitive primitive;
    primitive.type = primitive.CYLINDER;
    primitive.dimensions = {height_m, radius_m};

    cylinder.primitives.push_back(primitive);
    cylinder.primitive_poses.push_back(cylinder_pose);
    cylinder.operation = cylinder.ADD;

    planning_scene_interface_.applyCollisionObject(cylinder);
    RCLCPP_INFO(node_->get_logger(),
                "已添加圆柱障碍物 %s (height=%.4fm, radius=%.4fm)",
                object_id.c_str(),
                height_m,
                radius_m);
    return true;
}

void CR5Robot::removeCollisionObject(const std::string& object_id) {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "removeCollisionObject 失败：MoveGroup 尚未初始化");
        return;
    }
    if (object_id.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "removeCollisionObject 失败：object_id 不能为空");
        return;
    }

    planning_scene_interface_.removeCollisionObjects({object_id});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));
    RCLCPP_INFO(node_->get_logger(), "已移除碰撞对象 %s", object_id.c_str());
}

void CR5Robot::attachProbeModel() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "attachProbeModel 失败：MoveGroup 尚未初始化");
        return;
    }

    constexpr double kProbeTolerance = 0.0005;

    moveit_msgs::msg::AttachedCollisionObject probe_obj;
    probe_obj.link_name = "Link6";
    probe_obj.object.header.frame_id = "Link6";
    probe_obj.object.id = "rmp60_star_probe";
    probe_obj.touch_links = {"Link6", "Link5", "Link4"};

    shape_msgs::msg::SolidPrimitive body;
    body.type = body.CYLINDER;
    body.dimensions = {my_cr5_control::probe::kProbeBodyHeight, my_cr5_control::probe::kProbeBodyRadius};
    geometry_msgs::msg::Pose body_pose;
    body_pose.orientation.w = 1.0;
    body_pose.position.z = 0.038;

    shape_msgs::msg::SolidPrimitive stem;
    stem.type = stem.CYLINDER;
    stem.dimensions = {my_cr5_control::probe::kProbeStemHeight, my_cr5_control::probe::kProbeStemRadius};
    geometry_msgs::msg::Pose stem_pose;
    stem_pose.orientation.w = 1.0;
    stem_pose.position.z = 0.101;

    shape_msgs::msg::SolidPrimitive star_x;
    star_x.type = star_x.CYLINDER;
    star_x.dimensions = {my_cr5_control::probe::kStarStylusLength, my_cr5_control::probe::kStarStylusTipRadius};
    geometry_msgs::msg::Pose star_x_pose;
    tf2::Quaternion q;
    q.setRPY(0, 1.57, 0);
    star_x_pose.orientation = tf2::toMsg(q);
    star_x_pose.position.z = my_cr5_control::probe::kStarStylusZOffset;

    shape_msgs::msg::SolidPrimitive star_y = star_x;
    geometry_msgs::msg::Pose star_y_pose;
    q.setRPY(1.57, 0, 0);
    star_y_pose.orientation = tf2::toMsg(q);
    star_y_pose.position.z = my_cr5_control::probe::kStarStylusZOffset;

    shape_msgs::msg::SolidPrimitive tip;
    tip.type = tip.CYLINDER;
    tip.dimensions = {my_cr5_control::probe::kProbeTipHeight - kProbeTolerance,
                      my_cr5_control::probe::kProbeTipRadius};
    geometry_msgs::msg::Pose tip_pose;
    tip_pose.orientation.w = 1.0;
    tip_pose.position.z = my_cr5_control::probe::kProbeTipZOffset + (kProbeTolerance / 2.0);

    probe_obj.object.primitives = {body, stem, star_x, star_y, tip};
    probe_obj.object.primitive_poses = {body_pose, stem_pose, star_x_pose, star_y_pose, tip_pose};
    probe_obj.object.operation = probe_obj.object.ADD;

    planning_scene_interface_.applyAttachedCollisionObject(probe_obj);
    RCLCPP_INFO(node_->get_logger(), "测头测针碰撞模型已附加到 Link6");
}

void CR5Robot::setupCalibrationScene() {
    // 测头已在init()中加载，无需重复调用attachProbeModel()

    //  2. 标定球组合体 
    moveit_msgs::msg::CollisionObject sphere_obj;
    sphere_obj.header.frame_id = move_group_->getPlanningFrame();
    sphere_obj.id = "calibration_sphere_assembly";

    double sphere_center_x = 0.5; 
    double sphere_center_y = 0.0;
    double sphere_center_z = 0.45;

    shape_msgs::msg::SolidPrimitive post; post.type = post.CYLINDER;
    post.dimensions = {sphere_center_z, 0.01}; 
    geometry_msgs::msg::Pose post_pose; post_pose.orientation.w = 1.0;
    post_pose.position.x = sphere_center_x;
    post_pose.position.y = sphere_center_y;
    post_pose.position.z = sphere_center_z / 2.0; 

    shape_msgs::msg::SolidPrimitive sphere; sphere.type = sphere.SPHERE;
    sphere.dimensions = {0.0125 - 0.0005}; 
    geometry_msgs::msg::Pose sphere_pose; sphere_pose.orientation.w = 1.0;
    sphere_pose.position.x = sphere_center_x;
    sphere_pose.position.y = sphere_center_y;
    sphere_pose.position.z = sphere_center_z; 

    sphere_obj.primitives = {post, sphere}; sphere_obj.primitive_poses = {post_pose, sphere_pose};
    sphere_obj.operation = sphere_obj.ADD;

    //  3. 地板 
    moveit_msgs::msg::CollisionObject floor_obj;
    floor_obj.header.frame_id = move_group_->getPlanningFrame();
    floor_obj.id = "floor";

    // 尺寸改为 10米 x 10米，厚度 1米，防止任何速度下的穿透
    shape_msgs::msg::SolidPrimitive floor_prim; 
    floor_prim.type = floor_prim.BOX; 
    floor_prim.dimensions = {10.0, 10.0, 1.0}; 

    geometry_msgs::msg::Pose floor_pose; floor_pose.orientation.w = 1.0; 
    floor_pose.position.x = 0.0; floor_pose.position.y = 0.0; 
    // 地板中心在 Z=-0.5，厚度1.0，顶面在 Z=0.0 (紧贴机器人底座)
    floor_pose.position.z = -0.5; 

    floor_obj.primitives.push_back(floor_prim); floor_obj.primitive_poses.push_back(floor_pose);
    floor_obj.operation = floor_obj.ADD;

    // 应用逻辑
    planning_scene_interface_.removeCollisionObjects({"target_cube", "calibration_sphere_assembly", "floor"});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));

    planning_scene_interface_.applyCollisionObjects({sphere_obj, floor_obj});

    // 等待规划场景同步（关键：确保碰撞对象被规划器识别）
    std::this_thread::sleep_for(std::chrono::seconds(3));

    // 验证碰撞对象是否成功添加
    auto known_objects = planning_scene_interface_.getKnownObjectNames();
    RCLCPP_INFO(node_->get_logger(), "规划场景中的碰撞对象数量: %zu", known_objects.size());
    for (const auto& obj : known_objects) {
        RCLCPP_INFO(node_->get_logger(), "  - %s", obj.c_str());
    }

    if (std::find(known_objects.begin(), known_objects.end(), "floor") == known_objects.end()) {
        RCLCPP_ERROR(node_->get_logger(), "⚠ 警告：地板对象未成功添加到规划场景！");
    } else {
        RCLCPP_INFO(node_->get_logger(), "✓ 地板碰撞对象已确认添加");
    }

    if (std::find(known_objects.begin(), known_objects.end(), "calibration_sphere_assembly") == known_objects.end()) {
        RCLCPP_ERROR(node_->get_logger(), "⚠ 警告：标定球对象未成功添加到规划场景！");
    } else {
        RCLCPP_INFO(node_->get_logger(), "✓ 标定球碰撞对象已确认添加");
    }

    RCLCPP_INFO(node_->get_logger(), "仿真环境已加载并验证完成");
}

void CR5Robot::addSimulationEnvironment() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "addSimulationEnvironment 失败：MoveGroup 尚未初始化");
        return;
    }

    // 测头已在init()中加载，无需重复调用attachProbeModel()

    // 添加地板（重要：防止机械臂穿过地面）
    moveit_msgs::msg::CollisionObject floor_obj;
    floor_obj.header.frame_id = move_group_->getPlanningFrame();
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

    planning_scene_interface_.applyCollisionObject(floor_obj);
    RCLCPP_INFO(node_->get_logger(), "✓ 地板碰撞对象已添加");

    planning_scene_interface_.removeCollisionObjects({"ws119_mesh", "test_box"});
    std::this_thread::sleep_for(std::chrono::milliseconds(300));

    const auto v2_profile = my_cr5_control::scene::getV2MeshProfile();
    const geometry_msgs::msg::Pose ws119_pose = my_cr5_control::scene::makeV2MeshPose(v2_profile);
    const Eigen::Vector3d ws119_box_center = my_cr5_control::scene::getV2BoundingBoxCenter(v2_profile);
    const Eigen::Vector3d ws119_box_size = my_cr5_control::scene::getV2BoundingBoxSize(v2_profile);

    // 使用与真实 STL 相同的缩放和位姿，保持代价函数与碰撞环境一致
    has_environment_box_ = true;
    environment_box_x_ = ws119_box_center.x();
    environment_box_y_ = ws119_box_center.y();
    environment_box_z_ = ws119_box_center.z();
    environment_box_dx_ = ws119_box_size.x();
    environment_box_dy_ = ws119_box_size.y();
    environment_box_dz_ = ws119_box_size.z();

    // 关键步骤：优先使用真实 STL 模型，失败时回退到基础箱体
    RCLCPP_INFO(node_->get_logger(), "V2 mesh profile: %s", v2_profile.name.c_str());
    const bool mesh_loaded = addMeshObstacle("ws119_mesh", v2_profile.mesh_resource,
                                             ws119_pose, v2_profile.mesh_scale);
    if (!mesh_loaded) {
        RCLCPP_WARN(node_->get_logger(), "WS119.STL 加载失败，回退为基础箱体障碍物");
        addBoxObstacle(environment_box_x_, environment_box_y_, environment_box_z_,
                       environment_box_dx_, environment_box_dy_, environment_box_dz_);
    }
}

void CR5Robot::removeCalibrationScene() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "removeCalibrationScene 失败：MoveGroup 尚未初始化");
        return;
    }

    planning_scene_interface_.removeCollisionObjects({"calibration_sphere_assembly", "floor"});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    has_calibration_scene_ = false;
    RCLCPP_INFO(node_->get_logger(), "✓ 标定场景已移除（标定球+地板）");
}

void CR5Robot::restoreCalibrationScene() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "restoreCalibrationScene 失败：MoveGroup 尚未初始化");
        return;
    }

    setupCalibrationScene();
    has_calibration_scene_ = true;
    RCLCPP_INFO(node_->get_logger(), "✓ 标定场景已恢复");
}

void CR5Robot::removeBox() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "removeBox 失败：MoveGroup 尚未初始化");
        return;
    }

    planning_scene_interface_.removeCollisionObjects({"test_box", "ws119_mesh"});
    std::this_thread::sleep_for(std::chrono::milliseconds(500));
    RCLCPP_INFO(node_->get_logger(), "✓ 箱体已移除");
}

void CR5Robot::restoreBox() {
    if (!move_group_) {
        RCLCPP_ERROR(node_->get_logger(), "restoreBox 失败：MoveGroup 尚未初始化");
        return;
    }

    if (!has_environment_box_) {
        RCLCPP_WARN(node_->get_logger(), "restoreBox 失败：没有保存的箱体参数");
        return;
    }

    const auto v2_profile = my_cr5_control::scene::getV2MeshProfile();
    const geometry_msgs::msg::Pose ws119_pose = my_cr5_control::scene::makeV2MeshPose(v2_profile);
    const bool mesh_loaded = addMeshObstacle("ws119_mesh", v2_profile.mesh_resource,
                                             ws119_pose, v2_profile.mesh_scale);
    if (!mesh_loaded) {
        RCLCPP_WARN(node_->get_logger(), "WS119.STL 加载失败，使用基础箱体");
        addBoxObstacle(environment_box_x_, environment_box_y_, environment_box_z_,
                      environment_box_dx_, environment_box_dy_, environment_box_dz_);
    }
    RCLCPP_INFO(node_->get_logger(), "✓ 箱体已恢复");
}

bool CR5Robot::planToPoseInternal(const geometry_msgs::msg::Pose& target_pose,
                                  moveit::planning_interface::MoveGroupInterface::Plan& plan) {
    if (!move_group_) {
        return false;
    }

    move_group_->setPoseTarget(target_pose);
    auto result = move_group_->plan(plan);
    move_group_->clearPoseTargets();
    return (result == moveit::core::MoveItErrorCode::SUCCESS);
}

bool CR5Robot::buildStartState(const std::string& start_state_name, moveit::core::RobotState& start_state) const {
    if (!move_group_) {
        return false;
    }

    auto current_state = move_group_->getCurrentState();
    if (!current_state) {
        return false;
    }

    start_state = *current_state;

    if (start_state_name.empty()) {
        start_state.update();
        return true;
    }

    const auto* joint_model_group = start_state.getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        return false;
    }

    start_state.setToDefaultValues(joint_model_group, start_state_name);
    start_state.update();
    return true;
}

geometry_msgs::msg::Pose CR5Robot::getPoseFromState(const moveit::core::RobotState& state) const {
    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;

    const auto* joint_model_group = state.getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        return pose;
    }

    const auto& link_names = joint_model_group->getLinkModelNames();
    if (link_names.empty()) {
        return pose;
    }

    const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(link_names.back());
    const Eigen::Quaterniond quat(transform.rotation());

    pose.position.x = transform.translation().x();
    pose.position.y = transform.translation().y();
    pose.position.z = transform.translation().z();
    pose.orientation.x = quat.x();
    pose.orientation.y = quat.y();
    pose.orientation.z = quat.z();
    pose.orientation.w = quat.w();
    return pose;
}

bool CR5Robot::buildEndStateFromPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                     moveit::core::RobotState& end_state) const {
    const auto& joint_traj = plan.trajectory_.joint_trajectory;
    if (joint_traj.points.empty()) {
        return false;
    }
    const auto& last_point = joint_traj.points.back();
    if (joint_traj.joint_names.size() != last_point.positions.size()) {
        return false;
    }

    for (std::size_t i = 0; i < joint_traj.joint_names.size(); ++i) {
        end_state.setVariablePosition(joint_traj.joint_names[i], last_point.positions[i]);
    }
    end_state.update();
    return true;
}

bool CR5Robot::isTrajectoryCollisionFree(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                         const moveit::core::RobotState& start_state,
                                         const std::string& context) const {
    if (!move_group_ || !node_) {
        return false;
    }

    const auto model = move_group_->getRobotModel();
    if (!model) {
        RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：缺少 robot model", context.c_str());
        return false;
    }

    const auto* joint_model_group = model->getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：找不到规划组 %s",
                     context.c_str(), PLANNING_GROUP.c_str());
        return false;
    }

    const auto& joint_traj = trajectory.joint_trajectory;
    if (joint_traj.joint_names.empty() || joint_traj.points.empty()) {
        RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：轨迹为空", context.c_str());
        return false;
    }

    const auto& model_variables = model->getVariableNames();
    for (const auto& joint_name : joint_traj.joint_names) {
        if (std::find(model_variables.begin(), model_variables.end(), joint_name) ==
            model_variables.end()) {
            RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：未知关节变量 %s",
                         context.c_str(), joint_name.c_str());
            return false;
        }
    }

    auto planning_scene_client =
        node_->create_client<moveit_msgs::srv::GetPlanningScene>("/get_planning_scene");
    if (!planning_scene_client->wait_for_service(std::chrono::seconds(2))) {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s 碰撞复核失败：/get_planning_scene 服务不可用，拒绝执行",
                     context.c_str());
        return false;
    }

    auto request = std::make_shared<moveit_msgs::srv::GetPlanningScene::Request>();
    request->components.components =
        moveit_msgs::msg::PlanningSceneComponents::SCENE_SETTINGS |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE |
        moveit_msgs::msg::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
        moveit_msgs::msg::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
        moveit_msgs::msg::PlanningSceneComponents::TRANSFORMS |
        moveit_msgs::msg::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
        moveit_msgs::msg::PlanningSceneComponents::LINK_PADDING_AND_SCALING;

    auto future = planning_scene_client->async_send_request(request);
    if (future.wait_for(std::chrono::seconds(3)) != std::future_status::ready) {
        RCLCPP_ERROR(node_->get_logger(),
                     "%s 碰撞复核失败：获取 planning scene 超时，拒绝执行",
                     context.c_str());
        return false;
    }

    planning_scene::PlanningScene scene(model);
    const auto response = future.get();
    if (!response || !scene.usePlanningSceneMsg(response->scene)) {
        RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：planning scene 构建失败",
                     context.c_str());
        return false;
    }

    collision_detection::CollisionRequest collision_request;
    collision_request.contacts = true;
    collision_request.max_contacts = 1;
    collision_request.max_contacts_per_pair = 1;

    auto validate_state = [&](const moveit::core::RobotState& candidate_state,
                              const std::string& label) -> bool {
        moveit::core::RobotState checked_state(candidate_state);
        checked_state.update();
        if (!checked_state.satisfiesBounds(joint_model_group, 0.0)) {
            RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：%s 关节越界",
                         context.c_str(), label.c_str());
            return false;
        }

        collision_detection::CollisionResult collision_result;
        scene.checkCollision(collision_request, collision_result, checked_state,
                             scene.getAllowedCollisionMatrix());
        if (collision_result.collision) {
            if (!collision_result.contacts.empty()) {
                const auto& first_contact = collision_result.contacts.begin()->first;
                RCLCPP_ERROR(node_->get_logger(),
                             "%s 碰撞复核失败：%s 检测到碰撞 (%s <-> %s)",
                             context.c_str(), label.c_str(),
                             first_contact.first.c_str(), first_contact.second.c_str());
            } else {
                RCLCPP_ERROR(node_->get_logger(), "%s 碰撞复核失败：%s 检测到碰撞",
                             context.c_str(), label.c_str());
            }
            return false;
        }
        return true;
    };

    if (!validate_state(start_state, "start_state")) {
        return false;
    }

    constexpr double kMaxJointInterpolationStepRad = 0.02;
    std::vector<double> previous_positions;
    previous_positions.reserve(joint_traj.joint_names.size());
    for (const auto& joint_name : joint_traj.joint_names) {
        previous_positions.push_back(start_state.getVariablePosition(joint_name));
    }

    for (std::size_t point_index = 0; point_index < joint_traj.points.size(); ++point_index) {
        const auto& point = joint_traj.points[point_index];
        if (point.positions.size() != joint_traj.joint_names.size()) {
            RCLCPP_ERROR(node_->get_logger(),
                         "%s 碰撞复核失败：轨迹点 %zu 的关节数量不匹配",
                         context.c_str(), point_index);
            return false;
        }

        double max_delta = 0.0;
        for (std::size_t joint_index = 0; joint_index < joint_traj.joint_names.size(); ++joint_index) {
            max_delta = std::max(max_delta,
                                 std::abs(point.positions[joint_index] -
                                          previous_positions[joint_index]));
        }

        const std::size_t interpolation_steps =
            std::max<std::size_t>(1, static_cast<std::size_t>(
                                         std::ceil(max_delta / kMaxJointInterpolationStepRad)));
        for (std::size_t step = 1; step <= interpolation_steps; ++step) {
            const double ratio = static_cast<double>(step) /
                                 static_cast<double>(interpolation_steps);
            moveit::core::RobotState checked_state(start_state);
            for (std::size_t joint_index = 0; joint_index < joint_traj.joint_names.size(); ++joint_index) {
                const double interpolated =
                    previous_positions[joint_index] +
                    ratio * (point.positions[joint_index] - previous_positions[joint_index]);
                checked_state.setVariablePosition(joint_traj.joint_names[joint_index], interpolated);
            }
            if (!validate_state(
                    checked_state,
                    "point_" + std::to_string(point_index) + "_step_" + std::to_string(step))) {
                return false;
            }
        }

        previous_positions.assign(point.positions.begin(), point.positions.end());
    }

    return true;
}

bool CR5Robot::isPlanCollisionFree(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const std::string& context) const {
    return isTrajectoryCollisionFree(plan.trajectory_, start_state, context);
}

bool CR5Robot::executePlanIfCollisionFree(
    const moveit::planning_interface::MoveGroupInterface::Plan& plan,
    const moveit::core::RobotState& start_state,
    const std::string& context) {
    if (!isPlanCollisionFree(plan, start_state, context)) {
        return false;
    }
    return move_group_->execute(plan) == moveit::core::MoveItErrorCode::SUCCESS;
}

std::vector<geometry_msgs::msg::Pose> CR5Robot::generateEllipsoidGuideSamples(
    const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& goal_pose, std::size_t sample_count) const {
    std::vector<geometry_msgs::msg::Pose> guides;
    guides.reserve(sample_count);

    tf2::Vector3 start(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    tf2::Vector3 goal(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    tf2::Vector3 axis = goal - start;
    const double distance = axis.length();
    if (distance < 1e-6) {
        return guides;
    }

    const tf2::Vector3 axis_dir = axis.normalized();
    tf2::Vector3 ref(0.0, 0.0, 1.0);
    if (std::abs(axis_dir.dot(ref)) > 0.95) {
        ref = tf2::Vector3(1.0, 0.0, 0.0);
    }
    const tf2::Vector3 basis_1 = axis_dir.cross(ref).normalized();
    const tf2::Vector3 basis_2 = axis_dir.cross(basis_1).normalized();

    const tf2::Vector3 center = (start + goal) * 0.5;

    // 自适应椭球参数调整 (Adaptive Informed Sampling)
    // 根据场景难度动态调整椭球形状和采样策略
    double semi_major_ratio = 0.60;  // 默认值
    double minor_scale = 0.35;       // 默认值
    double target_bias_prob = 0.35;  // 默认值
    double t_min = 0.20;             // 默认值
    double t_max = 0.95;             // 默认值
    double biased_t_min = 0.70;      // 默认值
    double biased_t_max = 0.98;      // 默认值

    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    const double difficulty = effectiveDifficultyForMode(
        ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode)) {
        // 难度评分: 0.0=easy, 0.5=medium, 1.0=hard
        // Easy场景 (difficulty < 0.3): 收紧椭球，减少探索
        // Medium场景 (0.3 <= difficulty < 0.7): 平衡探索和利用
        // Hard场景 (difficulty >= 0.7): 扩大椭球，增加探索
        if (difficulty < 0.3) {
            // Easy: 收紧椭球，更接近直线路径
            semi_major_ratio = 0.50;
            minor_scale = 0.25;
            target_bias_prob = 0.50;  // 更多目标偏向
            t_min = 0.30;
            t_max = 0.90;
            biased_t_min = 0.75;
            biased_t_max = 0.95;
        } else if (difficulty < 0.7) {
            // Medium: 保持默认参数
            semi_major_ratio = 0.60;
            minor_scale = 0.35;
            target_bias_prob = 0.35;
            t_min = 0.20;
            t_max = 0.95;
            biased_t_min = 0.70;
            biased_t_max = 0.98;
        } else {
            // Hard: 扩大椭球，增加探索范围
            semi_major_ratio = 0.75;
            minor_scale = 0.50;
            target_bias_prob = 0.20;  // 减少目标偏向，更多探索
            t_min = 0.10;
            t_max = 0.98;
            biased_t_min = 0.60;
            biased_t_max = 0.99;
        }
    }

    const double semi_major = std::max(0.10, distance * semi_major_ratio);

    std::mt19937 rng = makeGuideSamplingRng();
    std::uniform_real_distribution<double> prob_dist(0.0, 1.0);
    std::uniform_real_distribution<double> t_dist(t_min, t_max);
    std::uniform_real_distribution<double> biased_t_dist(biased_t_min, biased_t_max);
    constexpr double kPi = 3.14159265358979323846;
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * kPi);
    std::uniform_real_distribution<double> radius_dist(0.0, 1.0);

    const double kTargetBiasProbability = target_bias_prob;
    for (std::size_t i = 0; i < sample_count; ++i) {
        const bool use_goal_bias = prob_dist(rng) < kTargetBiasProbability;
        const double mixture_draw = prob_dist(rng);
        double t = use_goal_bias ? biased_t_dist(rng) : t_dist(rng);
        double radial_scale = std::sqrt(radius_dist(rng));

        if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode) && difficulty >= 0.55) {
            const double flank_ratio =
                difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.45 : 0.30;
            const double late_ratio =
                difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.25 : 0.20;
            if (mixture_draw < flank_ratio) {
                const double flank_t_min =
                    difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.28 : 0.35;
                const double flank_t_max =
                    difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.82 : 0.78;
                t = flank_t_min + (flank_t_max - flank_t_min) * prob_dist(rng);
                radial_scale = 0.45 + 0.55 * std::sqrt(radius_dist(rng));
            } else if (mixture_draw < flank_ratio + late_ratio) {
                const double late_t_min =
                    difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.55 : 0.65;
                const double late_t_max =
                    difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.97 : 0.94;
                t = late_t_min + (late_t_max - late_t_min) * prob_dist(rng);
                radial_scale = 0.20 + 0.80 * std::sqrt(radius_dist(rng));
            }
        }

        const double axis_offset = (t - 0.5) * 2.0 * semi_major;
        const double axis_ratio = std::min(1.0, std::abs(axis_offset) / semi_major);
        const double local_minor = std::max(0.02, minor_scale * distance * std::sqrt(1.0 - axis_ratio * axis_ratio));
        const double r = radial_scale * local_minor;
        const double angle = angle_dist(rng);

        const tf2::Vector3 sampled = center + axis_dir * axis_offset +
                                     basis_1 * (r * std::cos(angle)) +
                                     basis_2 * (r * std::sin(angle));

        geometry_msgs::msg::Pose guide_pose = goal_pose;
        guide_pose.position.x = sampled.x();
        guide_pose.position.y = sampled.y();
        guide_pose.position.z = std::max(0.05, sampled.z());
        guides.push_back(guide_pose);
    }
    return guides;
}

std::vector<geometry_msgs::msg::Pose> CR5Robot::generateRefinedGuideSamples(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const std::vector<GuideCandidate>& seed_candidates,
    std::size_t extra_count) const {
    std::vector<geometry_msgs::msg::Pose> refined_guides;
    if (extra_count == 0 || seed_candidates.empty()) {
        return refined_guides;
    }

    tf2::Vector3 start(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    tf2::Vector3 goal(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    tf2::Vector3 axis = goal - start;
    const double axis_length = axis.length();
    if (axis_length < 1e-6) {
        return refined_guides;
    }

    const tf2::Vector3 axis_dir = axis.normalized();
    tf2::Vector3 ref(0.0, 0.0, 1.0);
    if (std::abs(axis_dir.dot(ref)) > 0.95) {
        ref = tf2::Vector3(1.0, 0.0, 0.0);
    }
    const tf2::Vector3 basis_1 = axis_dir.cross(ref).normalized();
    const tf2::Vector3 basis_2 = axis_dir.cross(basis_1).normalized();
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    const double difficulty = effectiveDifficultyForMode(
        ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    const std::size_t target_seed_count =
        difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 6 :
        (difficulty >= 0.55 ? 5 : paper::kGuideRefinementSeedCount);

    std::vector<GuideCandidate> sorted_seeds = seed_candidates;
    std::stable_sort(sorted_seeds.begin(), sorted_seeds.end(),
                     [](const GuideCandidate& lhs, const GuideCandidate& rhs) {
                         if (lhs.enabled != rhs.enabled) {
                             return lhs.enabled && !rhs.enabled;
                         }
                         if (lhs.ranking_score != rhs.ranking_score) {
                             return lhs.ranking_score < rhs.ranking_score;
                         }
                         return lhs.heuristic_cost < rhs.heuristic_cost;
                     });

    std::vector<GuideCandidate> active_seeds;
    for (const auto& candidate : sorted_seeds) {
        if (candidate.enabled && candidate.ik_feasible) {
            active_seeds.push_back(candidate);
        }
        if (active_seeds.size() >= target_seed_count) {
            break;
        }
    }
    if (active_seeds.empty()) {
        return refined_guides;
    }

    std::mt19937 rng = makeGuideSamplingRng();
    std::uniform_real_distribution<double> unit_dist(0.0, 1.0);
    const double axial_radius_scale =
        difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.16 :
        (difficulty >= 0.55 ? 0.13 : 0.10);
    std::uniform_real_distribution<double> axial_dist(
        -axial_radius_scale * axis_length, 0.10 * axis_length);
    constexpr double kPi = 3.14159265358979323846;
    std::uniform_real_distribution<double> angle_dist(0.0, 2.0 * kPi);
    const double local_radius = std::max(
        0.012,
        (difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.14 :
         (difficulty >= 0.55 ? 0.11 : 0.08)) * axis_length);

    refined_guides.reserve(extra_count);
    for (std::size_t i = 0; i < extra_count; ++i) {
        const auto& seed = active_seeds[i % active_seeds.size()];
        const double radial_unit = std::sqrt(unit_dist(rng));
        const double radial =
            (difficulty >= 0.55 ? (0.25 + 0.75 * radial_unit) : radial_unit) * local_radius;
        const double angle = angle_dist(rng);
        const double axial_offset = axial_dist(rng);
        tf2::Vector3 seed_pos(seed.pose.position.x, seed.pose.position.y, seed.pose.position.z);
        const tf2::Vector3 sampled = seed_pos + axis_dir * axial_offset +
                                     basis_1 * (radial * std::cos(angle)) +
                                     basis_2 * (radial * std::sin(angle));

        geometry_msgs::msg::Pose guide_pose = goal_pose;
        guide_pose.position.x = sampled.x();
        guide_pose.position.y = sampled.y();
        guide_pose.position.z = std::max(0.05, sampled.z());
        refined_guides.push_back(guide_pose);
    }
    return refined_guides;
}

std::vector<geometry_msgs::msg::Pose> CR5Robot::generateEnvironmentAnchorGuideSamples(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose) const {
    std::vector<geometry_msgs::msg::Pose> anchors;
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    const double difficulty = effectiveDifficultyForMode(
        ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    if (!usesEnvironmentAnchors(ablation_mode) ||
        !adaptive_ellipsoid_enabled_ ||
        difficulty < paper::kAdaptiveGuideFirstDifficultyThreshold) {
        return anchors;
    }

    double box_x = 0.0;
    double box_y = 0.0;
    double box_z = 0.0;
    double box_dx = 0.0;
    double box_dy = 0.0;
    double box_dz = 0.0;
    if (!getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
        return anchors;
    }

    const double goal_dx = goal_pose.position.x - box_x;
    const double goal_dy = goal_pose.position.y - box_y;
    const double goal_center_offset = std::hypot(goal_dx, goal_dy);
    const double insertion_radius = 0.35 * std::min(box_dx, box_dy);
    if (goal_center_offset > insertion_radius) {
        return anchors;
    }

    const double box_top_z = box_z + 0.5 * box_dz;
    const double hover_z = std::max({
        goal_pose.position.z + 0.04,
        box_top_z + 0.08 + 0.04 * difficulty,
        start_pose.position.z - 0.12,
    });
    const double recenter_scale =
        difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.20 : 0.35;
    const double half_recenter_scale = 0.5 * (1.0 + recenter_scale);

    auto append_anchor = [&](double x, double y, double z) {
        geometry_msgs::msg::Pose pose = goal_pose;
        pose.position.x = x;
        pose.position.y = y;
        pose.position.z = std::max(0.05, z);
        anchors.push_back(pose);
    };

    append_anchor(box_x, box_y, hover_z);
    append_anchor(box_x + goal_dx * recenter_scale,
                  box_y + goal_dy * recenter_scale,
                  hover_z);
    append_anchor(box_x + goal_dx * half_recenter_scale,
                  box_y + goal_dy * half_recenter_scale,
                  hover_z + 0.015);
    append_anchor(goal_pose.position.x, goal_pose.position.y, hover_z + 0.02);

    if (goal_center_offset > 1e-6) {
        const double tangent_scale =
            std::min(0.03, 0.45 * goal_center_offset);
        const double tangent_x = -goal_dy / goal_center_offset;
        const double tangent_y = goal_dx / goal_center_offset;
        append_anchor(box_x + goal_dx * recenter_scale + tangent_x * tangent_scale,
                      box_y + goal_dy * recenter_scale + tangent_y * tangent_scale,
                      hover_z);
        append_anchor(box_x + goal_dx * recenter_scale - tangent_x * tangent_scale,
                      box_y + goal_dy * recenter_scale - tangent_y * tangent_scale,
                      hover_z);
    }

    return anchors;
}

CR5Robot::GuideCandidate CR5Robot::evaluateGuideCandidate(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    const geometry_msgs::msg::Pose& guide_pose,
    const moveit::core::RobotState& ik_seed_state) const {
    const double direct_cost = computeDirectPathCost(start_pose, goal_pose);
    const double direct_distance = paper::pointDistance(start_pose.position, goal_pose.position);

    tf2::Vector3 start(start_pose.position.x, start_pose.position.y, start_pose.position.z);
    tf2::Vector3 goal(goal_pose.position.x, goal_pose.position.y, goal_pose.position.z);
    tf2::Vector3 axis = goal - start;
    const double axis_length = axis.length();
    const tf2::Vector3 axis_dir = axis_length > 1e-9 ? axis.normalized() : tf2::Vector3(0.0, 0.0, 1.0);

    const geometry_msgs::msg::Point mid_1 = paper::midpoint(start_pose.position, guide_pose.position);
    const geometry_msgs::msg::Point mid_2 = paper::midpoint(guide_pose.position, goal_pose.position);

    const double heuristic_cost = computeImprovedPathCost(start_pose, guide_pose, goal_pose);
    const double start_to_guide_distance = paper::pointDistance(start_pose.position, guide_pose.position);
    const double guide_to_goal_distance = paper::pointDistance(guide_pose.position, goal_pose.position);
    const double guide_penalty = computeObstacleAreaPenalty(guide_pose.position);
    const double mid1_penalty = computeObstacleAreaPenalty(mid_1);
    const double mid2_penalty = computeObstacleAreaPenalty(mid_2);

    tf2::Vector3 guide_vector(
        guide_pose.position.x - start_pose.position.x,
        guide_pose.position.y - start_pose.position.y,
        guide_pose.position.z - start_pose.position.z);
    const double projection = axis_length > 1e-9 ? guide_vector.dot(axis_dir) : 0.0;
    const tf2::Vector3 lateral = guide_vector - axis_dir * projection;

    const double guide_clearance = computeEnvironmentClearance(guide_pose.position);
    const double mid1_clearance = computeEnvironmentClearance(mid_1);
    const double mid2_clearance = computeEnvironmentClearance(mid_2);
    const double min_clearance = std::min({guide_clearance, mid1_clearance, mid2_clearance});

    GuideCandidate candidate;
    candidate.pose = guide_pose;
    candidate.heuristic_cost = heuristic_cost;
    candidate.ranking_score = heuristic_cost;
    candidate.enabled = true;
    candidate.direct_cost = direct_cost;
    candidate.cost_delta_to_direct = heuristic_cost - direct_cost;
    candidate.start_to_guide_distance = start_to_guide_distance;
    candidate.guide_to_goal_distance = guide_to_goal_distance;
    candidate.total_guide_distance = start_to_guide_distance + guide_to_goal_distance;
    candidate.direct_distance = direct_distance;
    candidate.axial_progress = axis_length > 1e-9 ? projection / axis_length : 0.0;
    candidate.lateral_offset = lateral.length();
    candidate.guide_penalty = guide_penalty;
    candidate.mid1_penalty = mid1_penalty;
    candidate.mid2_penalty = mid2_penalty;
    candidate.guide_height = guide_pose.position.z;
    candidate.clearance_margin = std::isfinite(min_clearance) ? min_clearance : 0.0;
    candidate.manipulability_score =
        estimateGuideManipulability(guide_pose, ik_seed_state, &candidate.ik_feasible);

    if (std::isfinite(min_clearance)) {
        candidate.safety_penalty =
            paper::normalizedGapPenalty(candidate.clearance_margin, paper::kGuidePreferredClearanceM) +
            0.25 * (guide_penalty + 0.5 * mid1_penalty + 0.5 * mid2_penalty);
        if (candidate.clearance_margin < paper::kGuideHardClearanceM) {
            candidate.enabled = false;
        }
    }

    const double manipulability_penalty =
        paper::normalizedGapPenalty(candidate.manipulability_score, paper::kGuidePreferredManipulability);
    double adaptive_bonus = 0.0;
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    const double difficulty = effectiveDifficultyForMode(
        ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode)) {
        const double clearance_bonus =
            0.006 * difficulty * std::clamp(candidate.clearance_margin / 0.05, 0.0, 1.0);
        const double preferred_axial =
            difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.55 : 0.65;
        const double axial_bonus =
            0.003 * difficulty *
            std::max(0.0, 1.0 - std::abs(candidate.axial_progress - preferred_axial) / 0.35);
        const double preferred_lateral = 0.008 + 0.022 * difficulty;
        const double lateral_span = 0.012 + 0.025 * difficulty;
        const double lateral_bonus =
            0.004 * difficulty *
            std::max(0.0, 1.0 - std::abs(candidate.lateral_offset - preferred_lateral) /
                                   std::max(0.005, lateral_span));
        adaptive_bonus = clearance_bonus + axial_bonus + lateral_bonus;

        double box_x = 0.0;
        double box_y = 0.0;
        double box_z = 0.0;
        double box_dx = 0.0;
        double box_dy = 0.0;
        double box_dz = 0.0;
        if (usesEnvironmentAnchors(ablation_mode) &&
            difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold &&
            getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
            const double goal_center_offset =
                std::hypot(goal_pose.position.x - box_x, goal_pose.position.y - box_y);
            const double candidate_center_offset =
                std::hypot(guide_pose.position.x - box_x, guide_pose.position.y - box_y);
            const double insertion_radius = 0.35 * std::min(box_dx, box_dy);
            if (goal_center_offset <= insertion_radius) {
                const double recenter_gain =
                    std::max(0.0, goal_center_offset - candidate_center_offset);
                const double recenter_bonus =
                    0.010 * difficulty *
                    std::clamp(recenter_gain /
                                   std::max(0.01, goal_center_offset + 0.02),
                               0.0, 1.0);
                const double hover_bonus =
                    0.007 * difficulty *
                    std::clamp((guide_pose.position.z - goal_pose.position.z) / 0.10, 0.0, 1.0);
                adaptive_bonus += recenter_bonus + hover_bonus;
            }
        }
    }
    candidate.ranking_score =
        heuristic_cost + 0.45 * candidate.safety_penalty + 0.35 * manipulability_penalty -
        adaptive_bonus;

    if (!candidate.ik_feasible ||
        candidate.axial_progress < -0.05 || candidate.axial_progress > 1.10) {
        candidate.enabled = false;
    }

    return candidate;
}

std::vector<CR5Robot::GuideCandidate> CR5Robot::buildGuideCandidates(
    const moveit::core::RobotState& start_state,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    std::size_t sample_count,
    bool apply_ranking) const {
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    const double difficulty = effectiveDifficultyForMode(
        ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    std::size_t effective_sample_count = sample_count;
    if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode)) {
        const double sample_multiplier =
            difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 1.75 :
            (difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold ? 1.25 : 1.0);
        effective_sample_count = std::max<std::size_t>(
            sample_count,
            static_cast<std::size_t>(std::ceil(sample_count * sample_multiplier)));
        effective_sample_count = std::min<std::size_t>(96, effective_sample_count);
    }

    auto guides = generateEllipsoidGuideSamples(start_pose, goal_pose, effective_sample_count);
    if (usesEnvironmentAnchors(ablation_mode)) {
        const auto environment_anchors =
            generateEnvironmentAnchorGuideSamples(start_pose, goal_pose);
        guides.insert(guides.end(), environment_anchors.begin(), environment_anchors.end());
    }

    std::size_t refinement_cap = 0;
    if (usesGuideRefinement(ablation_mode)) {
        refinement_cap =
            (adaptive_ellipsoid_enabled_ && difficulty >= paper::kAdaptiveHardDifficultyThreshold)
                ? 18
                : ((adaptive_ellipsoid_enabled_ &&
                    difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold) ? 12
                                                                                      : (paper::kGuideRefinementSeedCount * paper::kGuideRefinementPerSeed));
    }
    const std::size_t refinement_count =
        std::min(effective_sample_count / 2, refinement_cap);

    std::vector<GuideCandidate> candidates;
    candidates.reserve(guides.size() + refinement_count);
    for (const auto& guide : guides) {
        candidates.push_back(evaluateGuideCandidate(start_pose, goal_pose, guide, start_state));
    }

    const auto refined_guides =
        generateRefinedGuideSamples(start_pose, goal_pose, candidates, refinement_count);
    for (const auto& guide : refined_guides) {
        candidates.push_back(evaluateGuideCandidate(start_pose, goal_pose, guide, start_state));
    }

    std::vector<GuideCandidate> deduplicated;
    deduplicated.reserve(candidates.size());
    constexpr double kDuplicateDistanceM = 0.015;
    for (const auto& candidate : candidates) {
        bool duplicate = false;
        for (const auto& kept : deduplicated) {
            if (paper::pointDistance(candidate.pose.position, kept.pose.position) < kDuplicateDistanceM) {
                duplicate = true;
                break;
            }
        }
        if (!duplicate) {
            deduplicated.push_back(candidate);
        }
    }
    candidates = std::move(deduplicated);

    for (std::size_t index = 0; index < candidates.size(); ++index) {
        candidates[index].candidate_id = static_cast<int>(index);
    }

    if (apply_ranking) {
        applyGuideRanking(start_pose, goal_pose, candidates);
    }

    candidates.erase(
        std::remove_if(candidates.begin(), candidates.end(),
                       [](const GuideCandidate& candidate) {
                           return !candidate.enabled || !candidate.ik_feasible ||
                                  !std::isfinite(candidate.ranking_score);
                       }),
        candidates.end());

    std::stable_sort(candidates.begin(), candidates.end(),
                     [](const GuideCandidate& lhs, const GuideCandidate& rhs) {
                         if (lhs.ranking_score != rhs.ranking_score) {
                             return lhs.ranking_score < rhs.ranking_score;
                         }
                         return lhs.heuristic_cost < rhs.heuristic_cost;
                     });

    return candidates;
}

void CR5Robot::applyGuideRanking(const geometry_msgs::msg::Pose& start_pose,
                                 const geometry_msgs::msg::Pose& goal_pose,
                                 std::vector<GuideCandidate>& candidates) const {
    if (!guide_ranking_function_) {
        return;
    }

    try {
        guide_ranking_function_(start_pose, goal_pose, candidates);
    } catch (const std::exception& e) {
        RCLCPP_WARN(node_->get_logger(),
                    "HeuristicGuided guide ranking '%s' 执行失败，回退默认 heuristic_cost: %s",
                    guide_ranking_name_.c_str(), e.what());
        for (auto& candidate : candidates) {
            candidate.ranking_score = candidate.heuristic_cost;
            candidate.enabled = true;
        }
    } catch (...) {
        RCLCPP_WARN(node_->get_logger(),
                    "HeuristicGuided guide ranking '%s' 执行失败，回退默认 heuristic_cost",
                    guide_ranking_name_.c_str());
        for (auto& candidate : candidates) {
            candidate.ranking_score = candidate.heuristic_cost;
            candidate.enabled = true;
        }
    }
}

bool CR5Robot::getGuideEnvironmentBox(double& x, double& y, double& z,
                                      double& dx, double& dy, double& dz) const {
    if (has_guide_environment_box_hint_) {
        x = guide_environment_box_x_;
        y = guide_environment_box_y_;
        z = guide_environment_box_z_;
        dx = guide_environment_box_dx_;
        dy = guide_environment_box_dy_;
        dz = guide_environment_box_dz_;
        return true;
    }
    if (!has_environment_box_) {
        return false;
    }
    x = environment_box_x_;
    y = environment_box_y_;
    z = environment_box_z_;
    dx = environment_box_dx_;
    dy = environment_box_dy_;
    dz = environment_box_dz_;
    return true;
}

double CR5Robot::computeEnvironmentClearance(const geometry_msgs::msg::Point& point) const {
    double box_x = 0.0;
    double box_y = 0.0;
    double box_z = 0.0;
    double box_dx = 0.0;
    double box_dy = 0.0;
    double box_dz = 0.0;
    if (!getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
        return std::numeric_limits<double>::infinity();
    }

    const double hx = box_dx * 0.5;
    const double hy = box_dy * 0.5;
    const double hz = box_dz * 0.5;
    const double dx = std::abs(point.x - box_x) - hx;
    const double dy = std::abs(point.y - box_y) - hy;
    const double dz = std::abs(point.z - box_z) - hz;

    const double outside_x = std::max(dx, 0.0);
    const double outside_y = std::max(dy, 0.0);
    const double outside_z = std::max(dz, 0.0);
    const double distance_to_box = std::sqrt(outside_x * outside_x + outside_y * outside_y + outside_z * outside_z);

    const bool inside_box = (dx <= 0.0 && dy <= 0.0 && dz <= 0.0);
    if (!inside_box) {
        return distance_to_box;
    }

    const double margin_x = hx - std::abs(point.x - box_x);
    const double margin_y = hy - std::abs(point.y - box_y);
    const double margin_z = hz - std::abs(point.z - box_z);
    return -std::min({margin_x, margin_y, margin_z});
}

double CR5Robot::estimateGuideManipulability(const geometry_msgs::msg::Pose& pose,
                                             const moveit::core::RobotState& ik_seed_state,
                                             bool* ik_feasible) const {
    if (ik_feasible != nullptr) {
        *ik_feasible = false;
    }
    if (!move_group_) {
        return 0.0;
    }

    const auto* joint_model_group = ik_seed_state.getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        return 0.0;
    }

    const auto& link_names = joint_model_group->getLinkModelNames();
    if (link_names.empty()) {
        return 0.0;
    }
    const std::string tip_link = link_names.back();

    auto computeJacobianMinSingularValue = [&](const moveit::core::RobotState& state) -> double {
        Eigen::MatrixXd jacobian;
        const auto* link_model = state.getLinkModel(tip_link);
        if (!link_model) {
            return 0.0;
        }
        state.getJacobian(joint_model_group, link_model, Eigen::Vector3d::Zero(), jacobian);
        if (jacobian.rows() == 0 || jacobian.cols() == 0) {
            return 0.0;
        }
        const Eigen::JacobiSVD<Eigen::MatrixXd> svd(jacobian, Eigen::ComputeThinU | Eigen::ComputeThinV);
        const auto singular_values = svd.singularValues();
        if (singular_values.size() == 0) {
            return 0.0;
        }
        return singular_values(singular_values.size() - 1);
    };

    const bool has_ik_solver =
        static_cast<bool>(joint_model_group->getSolverInstance()) &&
        joint_model_group->canSetStateFromIK(tip_link);
    if (!has_ik_solver) {
        const double radial = std::hypot(pose.position.x, pose.position.y);
        const double height = pose.position.z;
        const bool roughly_reachable =
            radial >= 0.18 && radial <= 0.95 &&
            height >= 0.05 && height <= 1.20;
        if (ik_feasible != nullptr) {
            *ik_feasible = roughly_reachable;
        }
        if (!roughly_reachable) {
            return 0.0;
        }

        const double radial_score = std::clamp(1.0 - std::abs(radial - 0.48) / 0.40, 0.0, 1.0);
        const double height_score = std::clamp(1.0 - std::abs(height - 0.45) / 0.35, 0.0, 1.0);
        const double geometry_score = 0.5 * (radial_score + height_score);
        const double seed_score = computeJacobianMinSingularValue(ik_seed_state);
        return std::max(0.02, seed_score * (0.55 + 0.45 * geometry_score));
    }

    moveit::core::RobotState ik_state(ik_seed_state);
    if (!ik_state.setFromIK(joint_model_group, pose, tip_link, 0.02)) {
        return 0.0;
    }

    if (ik_feasible != nullptr) {
        *ik_feasible = true;
    }
    return computeJacobianMinSingularValue(ik_state);
}

double CR5Robot::computeObstacleAreaPenalty(const geometry_msgs::msg::Point& point) const {
    const double distance_to_box = computeEnvironmentClearance(point);
    if (!std::isfinite(distance_to_box)) {
        return 0.0;
    }

    constexpr double kInfluenceRadius = 0.10;
    if (distance_to_box >= kInfluenceRadius) {
        return 0.0;
    }

    const bool inside_box = distance_to_box < 0.0;
    const double clearance_term = (kInfluenceRadius - distance_to_box) * (kInfluenceRadius - distance_to_box);
    const double inside_term = inside_box ? 2.0 : 0.0;
    return clearance_term + inside_term;
}

double CR5Robot::computeImprovedPathCost(const geometry_msgs::msg::Pose& start_pose,
                                         const geometry_msgs::msg::Pose& guide_pose,
                                         const geometry_msgs::msg::Pose& goal_pose) const {
    const auto distance_3d = [](const geometry_msgs::msg::Point& a, const geometry_msgs::msg::Point& b) {
        const double dx = a.x - b.x;
        const double dy = a.y - b.y;
        const double dz = a.z - b.z;
        return std::sqrt(dx * dx + dy * dy + dz * dz);
    };

    geometry_msgs::msg::Point mid_1;
    mid_1.x = 0.5 * (start_pose.position.x + guide_pose.position.x);
    mid_1.y = 0.5 * (start_pose.position.y + guide_pose.position.y);
    mid_1.z = 0.5 * (start_pose.position.z + guide_pose.position.z);

    geometry_msgs::msg::Point mid_2;
    mid_2.x = 0.5 * (guide_pose.position.x + goal_pose.position.x);
    mid_2.y = 0.5 * (guide_pose.position.y + goal_pose.position.y);
    mid_2.z = 0.5 * (guide_pose.position.z + goal_pose.position.z);

    const double length_cost = distance_3d(start_pose.position, guide_pose.position) +
                               distance_3d(guide_pose.position, goal_pose.position);
    const double area_cost = computeObstacleAreaPenalty(guide_pose.position) +
                             0.5 * computeObstacleAreaPenalty(mid_1) +
                             0.5 * computeObstacleAreaPenalty(mid_2);

    constexpr double kAreaWeight = 2.5;
    return length_cost + (kAreaWeight * area_cost);
}

double CR5Robot::computeDirectPathCost(const geometry_msgs::msg::Pose& start_pose,
                                       const geometry_msgs::msg::Pose& goal_pose) const {
    const double dx = start_pose.position.x - goal_pose.position.x;
    const double dy = start_pose.position.y - goal_pose.position.y;
    const double dz = start_pose.position.z - goal_pose.position.z;
    const double length_cost = std::sqrt(dx * dx + dy * dy + dz * dz);

    geometry_msgs::msg::Point mid;
    mid.x = 0.5 * (start_pose.position.x + goal_pose.position.x);
    mid.y = 0.5 * (start_pose.position.y + goal_pose.position.y);
    mid.z = 0.5 * (start_pose.position.z + goal_pose.position.z);

    constexpr double kAreaWeight = 2.5;
    return length_cost + (kAreaWeight * computeObstacleAreaPenalty(mid));
}

bool CR5Robot::isDeepInsertionGoalForHeuristicRescue(
    const geometry_msgs::msg::Pose& goal_pose) const {
    double box_x = 0.0;
    double box_y = 0.0;
    double box_z = 0.0;
    double box_dx = 0.0;
    double box_dy = 0.0;
    double box_dz = 0.0;
    if (!getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
        return false;
    }

    const double box_top_z = box_z + 0.5 * box_dz;
    const double goal_height_above_top = goal_pose.position.z - box_top_z;
    return goal_height_above_top < paper::kHeuristicRescueMinGoalHeightAboveBoxTopM;
}

bool CR5Robot::isCenteredInsertionGoalForHeuristicRescue(
    const geometry_msgs::msg::Pose& goal_pose) const {
    double box_x = 0.0;
    double box_y = 0.0;
    double box_z = 0.0;
    double box_dx = 0.0;
    double box_dy = 0.0;
    double box_dz = 0.0;
    if (!getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
        return true;
    }

    const double dx = goal_pose.position.x - box_x;
    const double dy = goal_pose.position.y - box_y;
    const double goal_center_offset = std::sqrt(dx * dx + dy * dy);
    return goal_center_offset <= paper::kHeuristicRescueMaxGoalCenterOffsetM;
}

bool CR5Robot::isConservativeHeuristicRescueCandidate(
    const GuideCandidate& candidate,
    const geometry_msgs::msg::Pose& goal_pose,
    double direct_cost) const {
    if (!candidate.enabled || !candidate.ik_feasible ||
        !std::isfinite(candidate.ranking_score) || !std::isfinite(candidate.heuristic_cost)) {
        return false;
    }
    if (isDeepInsertionGoalForHeuristicRescue(goal_pose)) {
        return false;
    }
    if (!isCenteredInsertionGoalForHeuristicRescue(goal_pose)) {
        return false;
    }
    if (candidate.ranking_score >= direct_cost + paper::kHeuristicRescueCostSlack) {
        return false;
    }
    if (candidate.cost_delta_to_direct > 0.01) {
        return false;
    }
    if (candidate.clearance_margin < paper::kHeuristicRescueMinClearanceM) {
        return false;
    }
    if (candidate.manipulability_score < paper::kHeuristicRescueMinManipulability) {
        return false;
    }
    if (candidate.safety_penalty > paper::kHeuristicRescueMaxSafetyPenalty) {
        return false;
    }
    if (candidate.axial_progress < paper::kHeuristicRescueMinAxialProgress ||
        candidate.axial_progress > paper::kHeuristicRescueMaxAxialProgress) {
        return false;
    }
    if (candidate.lateral_offset < paper::kHeuristicRescueMinLateralOffsetM ||
        candidate.lateral_offset > paper::kHeuristicRescueMaxLateralOffsetM) {
        return false;
    }
    return true;
}

bool CR5Robot::isAdaptiveSlowDirectRescueCandidate(
    const GuideCandidate& candidate,
    double adaptive_difficulty,
    double direct_cost) const {
    if (adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) {
        return false;
    }
    if (!candidate.enabled || !candidate.ik_feasible ||
        !std::isfinite(candidate.ranking_score) || !std::isfinite(candidate.heuristic_cost)) {
        return false;
    }

    const double cost_slack =
        adaptive_difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold
            ? paper::kAdaptiveSlowDirectRescueMediumCostSlack
            : paper::kAdaptiveSlowDirectRescueEasyCostSlack;
    if (candidate.ranking_score > direct_cost + cost_slack) {
        return false;
    }
    if (candidate.cost_delta_to_direct > cost_slack) {
        return false;
    }
    if (candidate.clearance_margin < paper::kAdaptiveSlowDirectRescueMinClearanceM) {
        return false;
    }
    if (candidate.manipulability_score < paper::kAdaptiveSlowDirectRescueMinManipulability) {
        return false;
    }
    if (candidate.safety_penalty > paper::kAdaptiveSlowDirectRescueMaxSafetyPenalty) {
        return false;
    }
    if (candidate.axial_progress < paper::kAdaptiveSlowDirectRescueMinAxialProgress ||
        candidate.axial_progress > paper::kAdaptiveSlowDirectRescueMaxAxialProgress) {
        return false;
    }
    if (candidate.lateral_offset < paper::kAdaptiveSlowDirectRescueMinLateralOffsetM ||
        candidate.lateral_offset > paper::kAdaptiveSlowDirectRescueMaxLateralOffsetM) {
        return false;
    }
    return true;
}

void CR5Robot::setSpeed(double scaling) {
    if (move_group_) {
        move_group_->setMaxVelocityScalingFactor(scaling);
        move_group_->setMaxAccelerationScalingFactor(scaling);
    }
}

void CR5Robot::stopMotion() {
    if (!move_group_) {
        return;
    }
    move_group_->stop();
    move_group_->clearPoseTargets();
}

void CR5Robot::setGuideRankingFunction(GuideRankingFunction ranking_function,
                                       const std::string& ranking_name) {
    guide_ranking_function_ = std::move(ranking_function);
    guide_ranking_name_ = ranking_name.empty() ? "custom" : ranking_name;
    RCLCPP_INFO(node_->get_logger(), "HeuristicGuided guide ranking 已切换为: %s",
                guide_ranking_name_.c_str());
}

void CR5Robot::clearGuideRankingFunction() {
    guide_ranking_function_ = {};
    guide_ranking_name_ = "heuristic_cost";
    RCLCPP_INFO(node_->get_logger(), "HeuristicGuided guide ranking 已恢复默认: %s",
                guide_ranking_name_.c_str());
}

void CR5Robot::setGuideDirectCostGateEnabled(bool enabled) {
    guide_direct_cost_gate_enabled_ = enabled;
    RCLCPP_INFO(node_->get_logger(), "HeuristicGuided direct-cost gate 已%s",
                enabled ? "启用" : "关闭");
}

void CR5Robot::setGuideMaxAttempts(std::size_t max_attempts) {
    guide_max_attempts_ = max_attempts;
    if (guide_max_attempts_ == 0) {
        RCLCPP_INFO(node_->get_logger(), "HeuristicGuided guide-route 尝试上限: 不限制");
        return;
    }

    RCLCPP_INFO(node_->get_logger(), "HeuristicGuided guide-route 尝试上限已设置为: %zu",
                guide_max_attempts_);
}

void CR5Robot::setGuideSamplingSeed(std::uint32_t seed) {
    has_guide_sampling_seed_ = true;
    guide_sampling_seed_ = seed;
    guide_sampling_stream_index_ = 0;
    RCLCPP_INFO(node_->get_logger(), "HeuristicGuided guide sampling seed 已固定为: %u", seed);
}

void CR5Robot::clearGuideSamplingSeed() {
    has_guide_sampling_seed_ = false;
    guide_sampling_stream_index_ = 0;
}

std::mt19937 CR5Robot::makeGuideSamplingRng() const {
    if (!has_guide_sampling_seed_) {
        return std::mt19937(std::random_device{}());
    }

    const std::uint64_t stream_id = guide_sampling_stream_index_++;
    std::seed_seq seed_seq{
        guide_sampling_seed_,
        static_cast<std::uint32_t>(stream_id & 0xffffffffu),
        static_cast<std::uint32_t>((stream_id >> 32) & 0xffffffffu)};
    return std::mt19937(seed_seq);
}

void CR5Robot::setGuideEnvironmentBoxHint(double x, double y, double z,
                                          double dx, double dy, double dz) {
    has_guide_environment_box_hint_ = dx > 0.0 && dy > 0.0 && dz > 0.0;
    guide_environment_box_x_ = x;
    guide_environment_box_y_ = y;
    guide_environment_box_z_ = z;
    guide_environment_box_dx_ = dx;
    guide_environment_box_dy_ = dy;
    guide_environment_box_dz_ = dz;
    if (has_guide_environment_box_hint_) {
        RCLCPP_INFO(node_->get_logger(),
                    "HeuristicGuided 环境包围盒 hint 已更新: center=(%.3f, %.3f, %.3f), size=(%.3f, %.3f, %.3f)",
                    x, y, z, dx, dy, dz);
    }
}

void CR5Robot::clearGuideEnvironmentBoxHint() {
    has_guide_environment_box_hint_ = false;
}

std::vector<CR5Robot::GuideCandidate> CR5Robot::sampleGuideCandidates(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& start_state_name,
    std::size_t sample_count,
    bool apply_ranking) {
    if (!move_group_) {
        return {};
    }

    if (start_state_name.empty()) {
        auto current_state = move_group_->getCurrentState();
        if (!current_state) {
            return {};
        }
        const geometry_msgs::msg::Pose start_pose = getPoseFromState(*current_state);
        return buildGuideCandidates(*current_state, start_pose, target_pose, sample_count, apply_ranking);
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        return {};
    }
    const geometry_msgs::msg::Pose start_pose = getPoseFromState(start_state);
    return buildGuideCandidates(start_state, start_pose, target_pose, sample_count, apply_ranking);
}

bool CR5Robot::moveToNamedTarget(const std::string& target_name) {
    if (!move_group_) return false;
    auto current_state = move_group_->getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_ERROR(node_->get_logger(), "moveToNamedTarget 失败：无法获取当前关节状态");
        return false;
    }
    move_group_->setNamedTarget(target_name);
    move_group_->setStartState(*current_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    if (move_group_->plan(plan) != moveit::core::MoveItErrorCode::SUCCESS) {
        move_group_->setStartStateToCurrentState();
        return false;
    }

    const bool executed = executePlanIfCollisionFree(
        plan, *current_state, "moveToNamedTarget(" + target_name + ")");
    move_group_->setStartStateToCurrentState();
    return executed;
}

bool CR5Robot::moveToPoseWithPlanner(const geometry_msgs::msg::Pose& target_pose,
                                     const std::string& planner_id,
                                     double planning_time) {
    if (!move_group_) return false;

    const std::string original_planner = move_group_->getPlannerId();
    const double original_planning_time = move_group_->getPlanningTime();

    move_group_->setPlannerId(planner_id);
    move_group_->setPlanningTime(planning_time);
    auto current_state = move_group_->getCurrentState(1.0);
    if (!current_state) {
        move_group_->setPlannerId(original_planner);
        move_group_->setPlanningTime(original_planning_time);
        RCLCPP_ERROR(node_->get_logger(), "moveToPoseWithPlanner 失败：无法获取当前关节状态");
        return false;
    }
    move_group_->setStartState(*current_state);
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const auto result = move_group_->plan(my_plan);

    move_group_->clearPoseTargets();
    move_group_->setPlannerId(original_planner);
    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        return executePlanIfCollisionFree(
            my_plan, *current_state, "moveToPoseWithPlanner(" + planner_id + ")");
    }
    return false;
}

bool CR5Robot::planToPoseWithPlanner(const geometry_msgs::msg::Pose& target_pose,
                                     const std::string& planner_id,
                                     double planning_time,
                                     const std::string& start_state_name,
                                     PlanningMetrics* metrics) {
    const auto wall_start = std::chrono::steady_clock::now();
    const double planning_budget_ms = std::max(0.0, planning_time) * 1000.0;

    auto finalize_metrics = [&](bool success, double planner_reported_time_ms, int planner_calls) {
        if (metrics == nullptr) {
            return;
        }
        const auto wall_end = std::chrono::steady_clock::now();
        metrics->success = success;
        metrics->wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();
        metrics->planner_reported_time_ms = planner_reported_time_ms;
        metrics->planning_budget_ms = planning_budget_ms;
        metrics->planner_calls = planner_calls;
        metrics->hit_budget_limit = isBudgetHit(metrics->wall_time_ms, metrics->planning_budget_ms);
    };

    if (!move_group_) {
        finalize_metrics(false, 0.0, 0);
        return false;
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        finalize_metrics(false, 0.0, 0);
        return false;
    }

    const std::string original_planner = move_group_->getPlannerId();
    const double original_planning_time = move_group_->getPlanningTime();

    move_group_->setPlannerId(planner_id);
    move_group_->setPlanningTime(planning_time);
    move_group_->setStartState(start_state);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    const auto plan_start = std::chrono::steady_clock::now();
    const bool success = planToPoseInternal(target_pose, my_plan);
    const auto plan_end = std::chrono::steady_clock::now();
    const double plan_wall_ms = elapsedMs(plan_start, plan_end);
    const double planner_reported_time_ms = sanitizeReportedPlanningTimeMs(
        my_plan.planning_time_ * 1000.0, plan_wall_ms, planning_budget_ms);

    move_group_->setPlannerId(original_planner);
    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();

    const bool safe_success =
        success && isPlanCollisionFree(
                       my_plan, start_state,
                       "planToPoseWithPlanner(" + planner_id + ")");
    finalize_metrics(safe_success, planner_reported_time_ms, 1);
    return safe_success;
}

bool CR5Robot::planToPoseWithPlannerTrajectory(
    const geometry_msgs::msg::Pose& target_pose,
    const std::string& planner_id,
    double planning_time,
    const std::string& start_state_name,
    moveit_msgs::msg::RobotTrajectory* trajectory,
    PlanningMetrics* metrics) {
    if (trajectory != nullptr) {
        *trajectory = moveit_msgs::msg::RobotTrajectory();
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const double planning_budget_ms = std::max(0.0, planning_time) * 1000.0;

    auto finalize_metrics = [&](bool success, double planner_reported_time_ms, int planner_calls) {
        if (metrics == nullptr) {
            return;
        }
        const auto wall_end = std::chrono::steady_clock::now();
        metrics->success = success;
        metrics->wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();
        metrics->planner_reported_time_ms = planner_reported_time_ms;
        metrics->planning_budget_ms = planning_budget_ms;
        metrics->planner_calls = planner_calls;
        metrics->hit_budget_limit = isBudgetHit(metrics->wall_time_ms, metrics->planning_budget_ms);
    };

    if (!move_group_) {
        finalize_metrics(false, 0.0, 0);
        return false;
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        finalize_metrics(false, 0.0, 0);
        return false;
    }

    const std::string original_planner = move_group_->getPlannerId();
    const double original_planning_time = move_group_->getPlanningTime();

    move_group_->setPlannerId(planner_id);
    move_group_->setPlanningTime(planning_time);
    move_group_->setStartState(start_state);

    moveit::planning_interface::MoveGroupInterface::Plan plan;
    const auto plan_start = std::chrono::steady_clock::now();
    const bool success = planToPoseInternal(target_pose, plan);
    const auto plan_end = std::chrono::steady_clock::now();
    const double plan_wall_ms = elapsedMs(plan_start, plan_end);
    const double planner_reported_time_ms = sanitizeReportedPlanningTimeMs(
        plan.planning_time_ * 1000.0, plan_wall_ms, planning_budget_ms);
    const bool safe_success =
        success && isPlanCollisionFree(
                       plan, start_state,
                       "planToPoseWithPlannerTrajectory(" + planner_id + ")");
    if (safe_success && trajectory != nullptr) {
        *trajectory = plan.trajectory_;
    }

    move_group_->setPlannerId(original_planner);
    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();

    finalize_metrics(safe_success, planner_reported_time_ms, 1);
    return safe_success;
}

bool CR5Robot::moveToPose(const geometry_msgs::msg::Pose& target_pose) {
    if (!move_group_) return false;
    auto current_state = move_group_->getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_ERROR(node_->get_logger(), "moveToPose 失败：无法获取当前关节状态");
        return false;
    }
    move_group_->setStartState(*current_state);
    move_group_->setPoseTarget(target_pose);

    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    auto result = move_group_->plan(my_plan);
    move_group_->clearPoseTargets();
    move_group_->setStartStateToCurrentState();

    if (result == moveit::core::MoveItErrorCode::SUCCESS) {
        return executePlanIfCollisionFree(my_plan, *current_state, "moveToPose");
    }
    return false;
}

bool CR5Robot::moveToPoseImproved(const geometry_msgs::msg::Pose& target_pose) {
    if (!move_group_) return false;
    const double original_planning_time = move_group_->getPlanningTime();
    auto finish = [&](bool success) {
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        return success;
    };

    // 当前 HeuristicGuided 原型的核心不是“直接生成轨迹”，而是：
    // start/goal -> 采样 guide poses -> 排序 -> 尝试两段连接。
    auto current_state = move_group_->getCurrentState();
    if (!current_state) {
        return finish(false);
    }
    const geometry_msgs::msg::Pose start_pose = getPoseFromState(*current_state);
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    if (ablation_mode == HeuristicAblationMode::DirectOnly) {
        move_group_->setStartStateToCurrentState();
        moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
        const bool direct_success =
            planToPoseInternal(target_pose, direct_plan) &&
            isPlanCollisionFree(direct_plan, *current_state,
                                "moveToPoseImproved.direct_only");
        return finish(direct_success);
    }

    auto candidates = buildGuideCandidates(*current_state, start_pose, target_pose, 24, true);
    const double direct_path_cost = computeDirectPathCost(start_pose, target_pose);
    const double slow_direct_threshold_ms = getGuideSlowDirectThresholdMsFromEnv();
    const double adaptive_difficulty =
        effectiveDifficultyForMode(ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    const bool adaptive_guide_first_enabled =
        (adaptive_ellipsoid_enabled_ && usesAdaptiveGuideFirst(ablation_mode) &&
         adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) ||
        forceGuideFirst(ablation_mode);
    const std::size_t adaptive_guide_first_quota =
        adaptive_guide_first_enabled
            ? (forceGuideFirst(ablation_mode) ? 1u :
               (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 2u : 1u))
            : 0u;
    const double adaptive_near_tie_slack =
        adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.010 :
        (adaptive_difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold ? 0.004 : 0.0015);
    const bool guide_bridge_enabled =
        getGuideBridgeEnabledFromEnv() &&
        adaptive_difficulty >= paper::kGuideBridgeActivationDifficultyThreshold;

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_direct_plan;
    double best_cost = std::numeric_limits<double>::infinity();
    std::size_t guide_attempts = 0;
    double direct_attempt_wall_ms = 0.0;
    bool allow_slow_direct_rescue = false;
    moveit::planning_interface::MoveGroupInterface::Plan direct_plan;

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan_1;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan_2;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_1;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_2;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_3;
    std::vector<int> adaptive_prechecked_candidate_ids;
    int legacy_relaxed_rescue_attempts = 0;
    int adaptive_slow_direct_rescue_attempts = 0;
    const int adaptive_slow_direct_rescue_attempt_limit =
        adaptive_difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold
            ? paper::kAdaptiveSlowDirectRescueMediumAttempts
            : paper::kAdaptiveSlowDirectRescueEasyAttempts;
    auto isAdaptiveNearTieCandidate = [&](const GuideCandidate& candidate) {
        if (!adaptive_guide_first_enabled) {
            return false;
        }
        if (!candidate.enabled || !candidate.ik_feasible ||
            !std::isfinite(candidate.ranking_score)) {
            return false;
        }
        double candidate_slack = adaptive_near_tie_slack;
        double box_x = 0.0;
        double box_y = 0.0;
        double box_z = 0.0;
        double box_dx = 0.0;
        double box_dy = 0.0;
        double box_dz = 0.0;
        if (getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
            const double goal_center_offset =
                std::hypot(target_pose.position.x - box_x, target_pose.position.y - box_y);
            const double candidate_center_offset =
                std::hypot(candidate.pose.position.x - box_x, candidate.pose.position.y - box_y);
            const double insertion_radius = 0.35 * std::min(box_dx, box_dy);
            if (goal_center_offset <= insertion_radius) {
                const bool recentered =
                    candidate_center_offset + 0.005 < goal_center_offset;
                const bool hovered =
                    candidate.pose.position.z > target_pose.position.z + 0.03;
                if (recentered) {
                    candidate_slack += 0.012 * adaptive_difficulty;
                }
                if (hovered) {
                    candidate_slack += 0.010 * adaptive_difficulty;
                }
            }
        }
        if (candidate.ranking_score > direct_path_cost + candidate_slack) {
            return false;
        }
        if (candidate.clearance_margin <
            (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.015 : 0.010)) {
            return false;
        }
        if (candidate.axial_progress < 0.10 || candidate.axial_progress > 0.98) {
            return false;
        }
        if (candidate.lateral_offset >
            (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.10 : 0.08)) {
            return false;
        }
        if (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold &&
            candidate.lateral_offset < 0.006) {
            return false;
        }
        return true;
    };
    auto tryGuideCandidate =
        [&](const GuideCandidate& candidate, double per_attempt_cap_s) -> bool {
            if (guide_max_attempts_ > 0 && guide_attempts >= guide_max_attempts_) {
                return false;
            }

            ++guide_attempts;
            moveit::planning_interface::MoveGroupInterface::Plan plan_1;
            move_group_->setPlanningTime(per_attempt_cap_s);
            move_group_->setStartStateToCurrentState();
            if (!planToPoseInternal(candidate.pose, plan_1)) {
                return false;
            }

            auto planning_start_state = move_group_->getCurrentState();
            if (!planning_start_state) {
                return false;
            }
            if (!isPlanCollisionFree(plan_1, *planning_start_state,
                                     "moveToPoseImproved.guide.segment_1")) {
                return false;
            }
            moveit::core::RobotState end_state(*planning_start_state);
            if (!buildEndStateFromPlan(plan_1, end_state)) {
                return false;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_2;
            move_group_->setPlanningTime(per_attempt_cap_s);
            move_group_->setStartState(end_state);
            if (!planToPoseInternal(target_pose, plan_2)) {
                return false;
            }
            if (!isPlanCollisionFree(plan_2, end_state,
                                     "moveToPoseImproved.guide.segment_2")) {
                return false;
            }

            best_cost = candidate.ranking_score;
            best_plan_1 = plan_1;
            best_plan_2 = plan_2;
            return true;
        };

    if (adaptive_guide_first_quota > 0) {
        std::size_t adaptive_guide_attempts = 0;
        for (const auto& candidate : candidates) {
            if (adaptive_guide_attempts >= adaptive_guide_first_quota) {
                break;
            }
            if (!forceGuideFirst(ablation_mode) && !isAdaptiveNearTieCandidate(candidate)) {
                continue;
            }
            ++adaptive_guide_attempts;
            adaptive_prechecked_candidate_ids.push_back(candidate.candidate_id);
            if (tryGuideCandidate(candidate, paper::kAdaptiveGuideFirstAttemptCapS)) {
                break;
            }
        }
    }

    move_group_->setPlanningTime(original_planning_time);
    if (!best_plan_1) {
        move_group_->setStartStateToCurrentState();
        const auto direct_attempt_start = std::chrono::steady_clock::now();
        if (planToPoseInternal(target_pose, direct_plan) &&
            isPlanCollisionFree(direct_plan, *current_state,
                                "moveToPoseImproved.direct")) {
            direct_attempt_wall_ms =
                elapsedMs(direct_attempt_start, std::chrono::steady_clock::now());
            best_cost = direct_path_cost;
            best_direct_plan = direct_plan;
            allow_slow_direct_rescue =
                (!adaptive_ellipsoid_enabled_ ||
                 !usesAdaptiveDifficulty(ablation_mode) ||
                 adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) &&
                slow_direct_threshold_ms > 0.0 &&
                direct_attempt_wall_ms >= slow_direct_threshold_ms;
        }
    }
    for (const auto& candidate : candidates) {
        if (best_plan_1 && best_plan_2) {
            break;
        }
        if (guide_max_attempts_ > 0 && guide_attempts >= guide_max_attempts_) {
            break;
        }
        if (!adaptive_prechecked_candidate_ids.empty() &&
            std::find(adaptive_prechecked_candidate_ids.begin(),
                      adaptive_prechecked_candidate_ids.end(),
                      candidate.candidate_id) != adaptive_prechecked_candidate_ids.end()) {
            continue;
        }
        bool allow_legacy_relaxed_rescue_candidate = false;
        bool allow_adaptive_slow_direct_rescue_candidate = false;
        if (!guide_ranking_function_ && best_direct_plan && !best_plan_1 &&
            allow_slow_direct_rescue) {
            if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode) &&
                adaptive_slow_direct_rescue_attempts < adaptive_slow_direct_rescue_attempt_limit) {
                allow_adaptive_slow_direct_rescue_candidate =
                    isAdaptiveSlowDirectRescueCandidate(
                        candidate, adaptive_difficulty, best_cost);
            }
            if (!allow_adaptive_slow_direct_rescue_candidate &&
                legacy_relaxed_rescue_attempts < paper::kHeuristicRescueMaxRelaxedAttempts &&
                candidate.ranking_score >= best_cost) {
                allow_legacy_relaxed_rescue_candidate =
                    isConservativeHeuristicRescueCandidate(candidate, target_pose, best_cost);
            }
        }
        const bool allow_relaxed_rescue_candidate =
            allow_legacy_relaxed_rescue_candidate ||
            allow_adaptive_slow_direct_rescue_candidate;
        if (!guide_ranking_function_ && candidate.ranking_score >= best_cost &&
            !allow_relaxed_rescue_candidate) {
            continue;
        }
        if (allow_adaptive_slow_direct_rescue_candidate) {
            ++adaptive_slow_direct_rescue_attempts;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided moveToPose adaptive slow-direct rescue 放行 candidate=%d direct_wall=%.1fms cost_delta=%.4f clearance=%.4f manip=%.4f axial=%.3f lateral=%.3f",
                candidate.candidate_id,
                direct_attempt_wall_ms,
                candidate.cost_delta_to_direct,
                candidate.clearance_margin,
                candidate.manipulability_score,
                candidate.axial_progress,
                candidate.lateral_offset);
        } else if (allow_legacy_relaxed_rescue_candidate) {
            ++legacy_relaxed_rescue_attempts;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided moveToPose 保守 rescue 放行 candidate=%d direct_wall=%.1fms cost_delta=%.4f clearance=%.4f manip=%.4f axial=%.3f lateral=%.3f",
                candidate.candidate_id,
                direct_attempt_wall_ms,
                candidate.cost_delta_to_direct,
                candidate.clearance_margin,
                candidate.manipulability_score,
                candidate.axial_progress,
                candidate.lateral_offset);
        }

        tryGuideCandidate(candidate, original_planning_time);
    }

    if (!best_plan_1 &&
        guide_bridge_enabled &&
        (!best_direct_plan.has_value() || allow_slow_direct_rescue)) {
        const auto bridge_sequences = paper::buildGuideBridgeSequences(
            candidates,
            start_pose,
            target_pose,
            std::isfinite(best_cost) ? best_cost : computeDirectPathCost(start_pose, target_pose),
            adaptive_difficulty,
            getGuideBridgeMaxSequencesFromEnv());
        const double per_attempt_cap_s =
            adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold
                ? paper::kGuideBridgeHardAttemptCapS
                : paper::kGuideBridgeAttemptCapS;

        for (const auto& bridge : bridge_sequences) {
            if (guide_max_attempts_ > 0 && guide_attempts >= guide_max_attempts_) {
                break;
            }
            ++guide_attempts;

            moveit::planning_interface::MoveGroupInterface::Plan plan_1;
            move_group_->setStartStateToCurrentState();
            if (!planToPoseInternal(bridge.first->pose, plan_1)) {
                continue;
            }

            auto start_state = move_group_->getCurrentState();
            if (!start_state) {
                continue;
            }
            if (!isPlanCollisionFree(plan_1, *start_state,
                                     "moveToPoseImproved.bridge.segment_1")) {
                continue;
            }
            moveit::core::RobotState first_end_state(*start_state);
            if (!buildEndStateFromPlan(plan_1, first_end_state)) {
                continue;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_2;
            move_group_->setPlanningTime(per_attempt_cap_s);
            move_group_->setStartState(first_end_state);
            if (!planToPoseInternal(bridge.second->pose, plan_2)) {
                continue;
            }
            if (!isPlanCollisionFree(plan_2, first_end_state,
                                     "moveToPoseImproved.bridge.segment_2")) {
                continue;
            }

            moveit::core::RobotState second_end_state(first_end_state);
            if (!buildEndStateFromPlan(plan_2, second_end_state)) {
                continue;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_3;
            move_group_->setPlanningTime(per_attempt_cap_s);
            move_group_->setStartState(second_end_state);
            if (!planToPoseInternal(target_pose, plan_3)) {
                continue;
            }
            if (!isPlanCollisionFree(plan_3, second_end_state,
                                     "moveToPoseImproved.bridge.segment_3")) {
                continue;
            }

            best_cost = bridge.ranking_score;
            bridge_plan_1 = plan_1;
            bridge_plan_2 = plan_2;
            bridge_plan_3 = plan_3;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided guide-bridge 选中 pair=(%d,%d) score=%.4f progress_gain=%.3f min_clearance=%.4f",
                bridge.first->candidate_id,
                bridge.second->candidate_id,
                bridge.ranking_score,
                bridge.progress_gain,
                bridge.min_clearance);
            break;
        }
    }

    // 候选 guide 只负责引导搜索，最终轨迹仍由底层规划器生成。
    if (best_plan_1 && best_plan_2) {
        if (!executePlanIfCollisionFree(*best_plan_1, *current_state,
                                         "moveToPoseImproved.execute.guide.segment_1")) {
            return finish(false);
        }
        moveit::core::RobotState guide_end_state(*current_state);
        if (!buildEndStateFromPlan(*best_plan_1, guide_end_state)) {
            return finish(false);
        }
        return finish(executePlanIfCollisionFree(
            *best_plan_2, guide_end_state,
            "moveToPoseImproved.execute.guide.segment_2"));
    }

    if (bridge_plan_1 && bridge_plan_2 && bridge_plan_3) {
        if (!executePlanIfCollisionFree(*bridge_plan_1, *current_state,
                                         "moveToPoseImproved.execute.bridge.segment_1")) {
            return finish(false);
        }
        moveit::core::RobotState first_end_state(*current_state);
        if (!buildEndStateFromPlan(*bridge_plan_1, first_end_state)) {
            return finish(false);
        }
        if (!executePlanIfCollisionFree(*bridge_plan_2, first_end_state,
                                         "moveToPoseImproved.execute.bridge.segment_2")) {
            return finish(false);
        }
        moveit::core::RobotState second_end_state(first_end_state);
        if (!buildEndStateFromPlan(*bridge_plan_2, second_end_state)) {
            return finish(false);
        }
        return finish(executePlanIfCollisionFree(
            *bridge_plan_3, second_end_state,
            "moveToPoseImproved.execute.bridge.segment_3"));
    }

    if (best_direct_plan) {
        return finish(executePlanIfCollisionFree(*best_direct_plan, *current_state,
                                                 "moveToPoseImproved.execute.direct"));
    }

    return finish(false);
}

bool CR5Robot::planToPoseViaGuide(const geometry_msgs::msg::Pose& target_pose,
                                  const geometry_msgs::msg::Pose& guide_pose,
                                  const std::string& start_state_name,
                                  double planning_budget_s,
                                  PlanningMetrics* metrics) {
    const auto wall_start = std::chrono::steady_clock::now();
    const double planning_budget_ms = std::max(0.1, planning_budget_s) * 1000.0;
    double planner_reported_time_ms = 0.0;
    int planner_calls = 0;

    auto finalize_metrics = [&](bool success) {
        if (metrics == nullptr) {
            return;
        }
        const auto wall_end = std::chrono::steady_clock::now();
        metrics->success = success;
        metrics->wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();
        metrics->planner_reported_time_ms = planner_reported_time_ms;
        metrics->planning_budget_ms = planning_budget_ms;
        metrics->planner_calls = planner_calls;
        metrics->hit_budget_limit = isBudgetHit(metrics->wall_time_ms, metrics->planning_budget_ms);
    };

    if (!move_group_) {
        finalize_metrics(false);
        return false;
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        finalize_metrics(false);
        return false;
    }

    const double original_planning_time = move_group_->getPlanningTime();
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_budget = std::chrono::duration<double>(std::max(0.1, planning_budget_s));
    constexpr double kPerAttemptCapS = 1.0;

    auto remainingSeconds = [&]() -> double {
        const auto elapsed = std::chrono::steady_clock::now() - planning_start;
        const double remaining = planning_budget.count() -
                                 std::chrono::duration<double>(elapsed).count();
        return remaining;
    };

    auto planWithBudget = [&](const geometry_msgs::msg::Pose& pose,
                              moveit::planning_interface::MoveGroupInterface::Plan& plan,
                              const moveit::core::RobotState& state) -> bool {
        const double remaining = remainingSeconds();
        if (remaining <= 0.05) {
            return false;
        }
        const double attempt_budget_s = std::min(kPerAttemptCapS, remaining);
        const double attempt_budget_ms = attempt_budget_s * 1000.0;
        move_group_->setPlanningTime(attempt_budget_s);
        move_group_->setStartState(state);
        const auto attempt_start = std::chrono::steady_clock::now();
        const bool success = planToPoseInternal(pose, plan);
        const auto attempt_end = std::chrono::steady_clock::now();
        const double attempt_wall_ms = elapsedMs(attempt_start, attempt_end);
        planner_reported_time_ms += sanitizeReportedPlanningTimeMs(
            plan.planning_time_ * 1000.0, attempt_wall_ms, attempt_budget_ms);
        ++planner_calls;
        return success && isPlanCollisionFree(plan, state, "planToPoseViaGuide.segment");
    };

    moveit::planning_interface::MoveGroupInterface::Plan plan_1;
    if (!planWithBudget(guide_pose, plan_1, start_state)) {
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        finalize_metrics(false);
        return false;
    }

    moveit::core::RobotState end_state(start_state);
    if (!buildEndStateFromPlan(plan_1, end_state)) {
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        finalize_metrics(false);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan_2;
    const bool success = planWithBudget(target_pose, plan_2, end_state);

    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();
    finalize_metrics(success);
    return success;
}

bool CR5Robot::planToPoseViaGuideTrajectories(
    const geometry_msgs::msg::Pose& target_pose,
    const geometry_msgs::msg::Pose& guide_pose,
    const std::string& start_state_name,
    double planning_budget_s,
    std::vector<moveit_msgs::msg::RobotTrajectory>* trajectories,
    PlanningMetrics* metrics) {
    if (trajectories != nullptr) {
        trajectories->clear();
    }

    const auto wall_start = std::chrono::steady_clock::now();
    const double planning_budget_ms = std::max(0.1, planning_budget_s) * 1000.0;
    double planner_reported_time_ms = 0.0;
    int planner_calls = 0;

    auto finalize_metrics = [&](bool success) {
        if (metrics == nullptr) {
            return;
        }
        const auto wall_end = std::chrono::steady_clock::now();
        metrics->success = success;
        metrics->wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();
        metrics->planner_reported_time_ms = planner_reported_time_ms;
        metrics->planning_budget_ms = planning_budget_ms;
        metrics->planner_calls = planner_calls;
        metrics->hit_budget_limit = isBudgetHit(metrics->wall_time_ms, metrics->planning_budget_ms);
    };

    if (!move_group_) {
        finalize_metrics(false);
        return false;
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        finalize_metrics(false);
        return false;
    }

    const double original_planning_time = move_group_->getPlanningTime();
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_budget = std::chrono::duration<double>(std::max(0.1, planning_budget_s));
    constexpr double kPerAttemptCapS = 1.0;

    auto remainingSeconds = [&]() -> double {
        const auto elapsed = std::chrono::steady_clock::now() - planning_start;
        return planning_budget.count() - std::chrono::duration<double>(elapsed).count();
    };

    auto planWithBudget = [&](const geometry_msgs::msg::Pose& pose,
                              moveit::planning_interface::MoveGroupInterface::Plan& plan,
                              const moveit::core::RobotState& state) -> bool {
        const double remaining = remainingSeconds();
        if (remaining <= 0.05) {
            return false;
        }
        const double attempt_budget_s = std::min(kPerAttemptCapS, remaining);
        const double attempt_budget_ms = attempt_budget_s * 1000.0;
        move_group_->setPlanningTime(attempt_budget_s);
        move_group_->setStartState(state);
        const auto attempt_start = std::chrono::steady_clock::now();
        const bool success = planToPoseInternal(pose, plan);
        const auto attempt_end = std::chrono::steady_clock::now();
        const double attempt_wall_ms = elapsedMs(attempt_start, attempt_end);
        planner_reported_time_ms += sanitizeReportedPlanningTimeMs(
            plan.planning_time_ * 1000.0, attempt_wall_ms, attempt_budget_ms);
        ++planner_calls;
        return success && isPlanCollisionFree(plan, state,
                                              "planToPoseViaGuideTrajectories.segment");
    };

    moveit::planning_interface::MoveGroupInterface::Plan plan_1;
    if (!planWithBudget(guide_pose, plan_1, start_state)) {
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        finalize_metrics(false);
        return false;
    }

    moveit::core::RobotState end_state(start_state);
    if (!buildEndStateFromPlan(plan_1, end_state)) {
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        finalize_metrics(false);
        return false;
    }

    moveit::planning_interface::MoveGroupInterface::Plan plan_2;
    const bool success = planWithBudget(target_pose, plan_2, end_state);
    if (success && trajectories != nullptr) {
        trajectories->push_back(plan_1.trajectory_);
        trajectories->push_back(plan_2.trajectory_);
    }

    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();
    finalize_metrics(success);
    return success;
}

std::vector<geometry_msgs::msg::Point> CR5Robot::endEffectorPathFromTrajectory(
    const moveit_msgs::msg::RobotTrajectory& trajectory,
    const std::string& link_name) const {
    std::vector<geometry_msgs::msg::Point> points;
    if (!move_group_) {
        return points;
    }
    const auto model = move_group_->getRobotModel();
    if (!model) {
        return points;
    }
    const auto* link_model = model->getLinkModel(link_name);
    if (link_model == nullptr) {
        return points;
    }

    moveit::core::RobotState state(model);
    state.setToDefaultValues();
    const auto& joint_names = trajectory.joint_trajectory.joint_names;
    points.reserve(trajectory.joint_trajectory.points.size());
    for (const auto& waypoint : trajectory.joint_trajectory.points) {
        if (waypoint.positions.size() != joint_names.size()) {
            continue;
        }
        for (std::size_t i = 0; i < joint_names.size(); ++i) {
            state.setVariablePosition(joint_names[i], waypoint.positions[i]);
        }
        state.update();
        const Eigen::Isometry3d& transform = state.getGlobalLinkTransform(link_model);
        geometry_msgs::msg::Point point;
        point.x = transform.translation().x();
        point.y = transform.translation().y();
        point.z = transform.translation().z();
        points.push_back(point);
    }
    return points;
}

bool CR5Robot::planToPoseImproved(const geometry_msgs::msg::Pose& target_pose,
                                  const std::string& start_state_name,
                                  double planning_budget_s,
                                  PlanningMetrics* metrics,
                                  std::size_t guide_sample_count,
                                  const PlanningMetrics* direct_metrics_override) {
    const auto wall_start = std::chrono::steady_clock::now();
    const double planning_budget_ms = std::max(0.1, planning_budget_s) * 1000.0;
    const bool has_direct_metrics_override = direct_metrics_override != nullptr;
    const double direct_override_wall_time_ms =
        has_direct_metrics_override ? direct_metrics_override->wall_time_ms : 0.0;
    double planner_reported_time_ms =
        has_direct_metrics_override ? direct_metrics_override->planner_reported_time_ms : 0.0;
    int planner_calls =
        has_direct_metrics_override ? direct_metrics_override->planner_calls : 0;
    int guide_candidate_count = 0;
    int guide_candidates_attempted = 0;
    bool direct_plan_success = false;
    double direct_attempt_wall_time_ms = 0.0;
    double direct_path_cost = -1.0;
    bool used_direct_plan = false;
    int top_ranked_candidate_id = -1;
    double top_ranked_candidate_heuristic_cost = -1.0;
    double top_ranked_candidate_ranking_score = -1.0;
    double top_ranked_candidate_cost_delta_to_direct = -1.0;
    double top_ranked_candidate_clearance_margin = -1.0;
    double top_ranked_candidate_manipulability_score = -1.0;
    double top_ranked_candidate_axial_progress = -1.0;
    double top_ranked_candidate_lateral_offset = -1.0;
    int selected_candidate_id = -1;
    double selected_candidate_learned_probability = -1.0;
    double selected_candidate_heuristic_cost = -1.0;
    double selected_candidate_ranking_score = -1.0;
    geometry_msgs::msg::Point selected_candidate_point;

    auto finalize_metrics = [&](bool success) {
        if (metrics == nullptr) {
            return;
        }
        const auto wall_end = std::chrono::steady_clock::now();
        metrics->success = success;
        const double extra_wall_time_ms =
            std::chrono::duration_cast<std::chrono::milliseconds>(wall_end - wall_start).count();
        metrics->wall_time_ms = has_direct_metrics_override
            ? (direct_override_wall_time_ms + extra_wall_time_ms)
            : extra_wall_time_ms;
        metrics->planner_reported_time_ms = planner_reported_time_ms;
        metrics->planning_budget_ms = planning_budget_ms;
        metrics->planner_calls = planner_calls;
        metrics->hit_budget_limit = isBudgetHit(metrics->wall_time_ms, metrics->planning_budget_ms);
        metrics->guide_candidate_count = guide_candidate_count;
        metrics->guide_candidates_attempted = guide_candidates_attempted;
        metrics->direct_plan_success = direct_plan_success;
        metrics->direct_attempt_wall_time_ms = direct_attempt_wall_time_ms;
        metrics->direct_path_cost = direct_path_cost;
        metrics->used_direct_plan = used_direct_plan;
        metrics->top_ranked_candidate_id = top_ranked_candidate_id;
        metrics->top_ranked_candidate_heuristic_cost = top_ranked_candidate_heuristic_cost;
        metrics->top_ranked_candidate_ranking_score = top_ranked_candidate_ranking_score;
        metrics->top_ranked_candidate_cost_delta_to_direct = top_ranked_candidate_cost_delta_to_direct;
        metrics->top_ranked_candidate_clearance_margin = top_ranked_candidate_clearance_margin;
        metrics->top_ranked_candidate_manipulability_score =
            top_ranked_candidate_manipulability_score;
        metrics->top_ranked_candidate_axial_progress = top_ranked_candidate_axial_progress;
        metrics->top_ranked_candidate_lateral_offset = top_ranked_candidate_lateral_offset;
        metrics->selected_candidate_id = selected_candidate_id;
        metrics->selected_candidate_learned_probability = selected_candidate_learned_probability;
        metrics->selected_candidate_heuristic_cost = selected_candidate_heuristic_cost;
        metrics->selected_candidate_ranking_score = selected_candidate_ranking_score;
        metrics->selected_candidate_point = selected_candidate_point;
    };

    if (!move_group_) {
        finalize_metrics(false);
        return false;
    }

    moveit::core::RobotState start_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, start_state)) {
        finalize_metrics(false);
        return false;
    }

    const double original_planning_time = move_group_->getPlanningTime();
    const auto planning_start = std::chrono::steady_clock::now();
    const auto planning_budget =
        std::chrono::duration<double>(std::max(0.1, planning_budget_s));
    constexpr double kPerAttemptCapS = 1.0;

    auto remainingSeconds = [&]() -> double {
        const auto elapsed = std::chrono::steady_clock::now() - planning_start;
        const double remaining = planning_budget.count() -
                                 std::chrono::duration<double>(elapsed).count();
        return remaining;
    };

    auto planWithBudget = [&](const geometry_msgs::msg::Pose& pose,
                              moveit::planning_interface::MoveGroupInterface::Plan& plan,
                              const moveit::core::RobotState& state,
                              double per_attempt_cap_s) -> bool {
        const double remaining = remainingSeconds();
        if (remaining <= 0.05) {
            return false;
        }
        const double attempt_budget_s =
            std::min(std::max(0.05, per_attempt_cap_s), remaining);
        const double attempt_budget_ms = attempt_budget_s * 1000.0;
        move_group_->setPlanningTime(attempt_budget_s);
        move_group_->setStartState(state);
        const auto attempt_start = std::chrono::steady_clock::now();
        const bool success = planToPoseInternal(pose, plan);
        const auto attempt_end = std::chrono::steady_clock::now();
        const double attempt_wall_ms = elapsedMs(attempt_start, attempt_end);
        planner_reported_time_ms += sanitizeReportedPlanningTimeMs(
            plan.planning_time_ * 1000.0, attempt_wall_ms, attempt_budget_ms);
        ++planner_calls;
        return success && isPlanCollisionFree(plan, state, "planToPoseImproved.segment");
    };

    // 这个函数就是当前 benchmark 中 HeuristicGuided 的工程接口。
    // 后续 learning-guided 模块应优先接到 guide 候选生成/排序这里，而不是直接输出整条轨迹。
    const geometry_msgs::msg::Pose start_pose =
        start_state_name.empty() ? getCurrentPose() : getPoseFromState(start_state);
    direct_path_cost = computeDirectPathCost(start_pose, target_pose);
    const HeuristicAblationMode ablation_mode = getHeuristicAblationModeFromEnv();
    if (ablation_mode == HeuristicAblationMode::DirectOnly) {
        moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
        const auto direct_attempt_start = std::chrono::steady_clock::now();
        const bool success = planWithBudget(target_pose, direct_plan, start_state, planning_budget_s);
        direct_attempt_wall_time_ms =
            elapsedMs(direct_attempt_start, std::chrono::steady_clock::now());
        direct_plan_success = success;
        used_direct_plan = success;
        move_group_->setPlanningTime(original_planning_time);
        move_group_->setStartStateToCurrentState();
        finalize_metrics(success);
        return success;
    }

    auto candidates = buildGuideCandidates(start_state, start_pose, target_pose, guide_sample_count, true);
    const double slow_direct_threshold_ms = getGuideSlowDirectThresholdMsFromEnv();
    const double adaptive_difficulty =
        effectiveDifficultyForMode(ablation_mode, adaptive_ellipsoid_enabled_, scene_difficulty_score_);
    const bool adaptive_guide_first_enabled =
        !has_direct_metrics_override &&
        ((adaptive_ellipsoid_enabled_ && usesAdaptiveGuideFirst(ablation_mode) &&
          adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) ||
         forceGuideFirst(ablation_mode));
    const std::size_t adaptive_guide_first_quota =
        adaptive_guide_first_enabled
            ? (forceGuideFirst(ablation_mode) ? 1u :
               (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 2u : 1u))
            : 0u;
    const double adaptive_near_tie_slack =
        adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.010 :
        (adaptive_difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold ? 0.004 : 0.0015);
    guide_candidate_count = static_cast<int>(candidates.size());
    if (!candidates.empty()) {
        const auto& candidate = candidates.front();
        top_ranked_candidate_id = candidate.candidate_id;
        top_ranked_candidate_heuristic_cost = candidate.heuristic_cost;
        top_ranked_candidate_ranking_score = candidate.ranking_score;
        top_ranked_candidate_cost_delta_to_direct = candidate.cost_delta_to_direct;
        top_ranked_candidate_clearance_margin = candidate.clearance_margin;
        top_ranked_candidate_manipulability_score = candidate.manipulability_score;
        top_ranked_candidate_axial_progress = candidate.axial_progress;
        top_ranked_candidate_lateral_offset = candidate.lateral_offset;
    }

    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan_1;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> best_plan_2;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_1;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_2;
    std::optional<moveit::planning_interface::MoveGroupInterface::Plan> bridge_plan_3;
    std::vector<int> adaptive_prechecked_candidate_ids;
    bool has_direct_success = false;
    double best_cost = std::numeric_limits<double>::infinity();
    bool allow_slow_direct_rescue = false;
    const bool guide_bridge_enabled =
        getGuideBridgeEnabledFromEnv() &&
        adaptive_difficulty >= paper::kGuideBridgeActivationDifficultyThreshold;
    auto isAdaptiveNearTieCandidate = [&](const GuideCandidate& candidate) {
        if (!adaptive_guide_first_enabled) {
            return false;
        }
        if (!candidate.enabled || !candidate.ik_feasible ||
            !std::isfinite(candidate.ranking_score)) {
            return false;
        }
        double candidate_slack = adaptive_near_tie_slack;
        double box_x = 0.0;
        double box_y = 0.0;
        double box_z = 0.0;
        double box_dx = 0.0;
        double box_dy = 0.0;
        double box_dz = 0.0;
        if (getGuideEnvironmentBox(box_x, box_y, box_z, box_dx, box_dy, box_dz)) {
            const double goal_center_offset =
                std::hypot(target_pose.position.x - box_x, target_pose.position.y - box_y);
            const double candidate_center_offset =
                std::hypot(candidate.pose.position.x - box_x, candidate.pose.position.y - box_y);
            const double insertion_radius = 0.35 * std::min(box_dx, box_dy);
            if (goal_center_offset <= insertion_radius) {
                const bool recentered =
                    candidate_center_offset + 0.005 < goal_center_offset;
                const bool hovered =
                    candidate.pose.position.z > target_pose.position.z + 0.03;
                if (recentered) {
                    candidate_slack += 0.012 * adaptive_difficulty;
                }
                if (hovered) {
                    candidate_slack += 0.010 * adaptive_difficulty;
                }
            }
        }
        if (candidate.ranking_score > direct_path_cost + candidate_slack) {
            return false;
        }
        if (candidate.clearance_margin <
            (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.015 : 0.010)) {
            return false;
        }
        if (candidate.axial_progress < 0.10 || candidate.axial_progress > 0.98) {
            return false;
        }
        if (candidate.lateral_offset >
            (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold ? 0.10 : 0.08)) {
            return false;
        }
        if (adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold &&
            candidate.lateral_offset < 0.006) {
            return false;
        }
        return true;
    };
    auto tryGuideCandidate =
        [&](const GuideCandidate& candidate, double per_attempt_cap_s) -> bool {
            if (remainingSeconds() <= 0.05) {
                return false;
            }
            if (guide_max_attempts_ > 0 &&
                guide_candidates_attempted >= static_cast<int>(guide_max_attempts_)) {
                return false;
            }

            ++guide_candidates_attempted;
            moveit::planning_interface::MoveGroupInterface::Plan plan_1;
            if (!planWithBudget(candidate.pose, plan_1, start_state, per_attempt_cap_s)) {
                return false;
            }

            moveit::core::RobotState end_state(start_state);
            if (!buildEndStateFromPlan(plan_1, end_state)) {
                return false;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_2;
            if (!planWithBudget(target_pose, plan_2, end_state, per_attempt_cap_s)) {
                return false;
            }

            best_cost = candidate.ranking_score;
            best_plan_1 = plan_1;
            best_plan_2 = plan_2;
            selected_candidate_id = candidate.candidate_id;
            selected_candidate_learned_probability = candidate.learned_probability;
            selected_candidate_heuristic_cost = candidate.heuristic_cost;
            selected_candidate_ranking_score = candidate.ranking_score;
            selected_candidate_point = candidate.pose.position;
            return true;
        };

    if (adaptive_guide_first_quota > 0) {
        std::size_t adaptive_guide_attempts = 0;
        for (const auto& candidate : candidates) {
            if (adaptive_guide_attempts >= adaptive_guide_first_quota) {
                break;
            }
            if (!isAdaptiveNearTieCandidate(candidate)) {
                continue;
            }
            ++adaptive_guide_attempts;
            adaptive_prechecked_candidate_ids.push_back(candidate.candidate_id);
            if (tryGuideCandidate(candidate, paper::kAdaptiveGuideFirstAttemptCapS)) {
                break;
            }
        }
    }

    if (!best_plan_1 && has_direct_metrics_override) {
        has_direct_success = direct_metrics_override->success;
        direct_plan_success = has_direct_success;
        if (has_direct_success) {
            best_cost = direct_path_cost;
            direct_attempt_wall_time_ms = direct_metrics_override->wall_time_ms;
            allow_slow_direct_rescue =
                (!adaptive_ellipsoid_enabled_ ||
                 !usesAdaptiveDifficulty(ablation_mode) ||
                 adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) &&
                slow_direct_threshold_ms > 0.0 &&
                direct_attempt_wall_time_ms >= slow_direct_threshold_ms;
        }
    } else if (!best_plan_1) {
        moveit::planning_interface::MoveGroupInterface::Plan direct_plan;
        const auto direct_attempt_start = std::chrono::steady_clock::now();
        has_direct_success = planWithBudget(target_pose, direct_plan, start_state, kPerAttemptCapS);
        direct_attempt_wall_time_ms =
            elapsedMs(direct_attempt_start, std::chrono::steady_clock::now());
        direct_plan_success = has_direct_success;
        if (has_direct_success) {
            best_cost = direct_path_cost;
            has_direct_success = true;
            allow_slow_direct_rescue =
                (!adaptive_ellipsoid_enabled_ ||
                 !usesAdaptiveDifficulty(ablation_mode) ||
                 adaptive_difficulty >= paper::kAdaptiveGuideFirstDifficultyThreshold) &&
                slow_direct_threshold_ms > 0.0 &&
                direct_attempt_wall_time_ms >= slow_direct_threshold_ms;
        }
    }

    int legacy_relaxed_rescue_attempts = 0;
    int adaptive_slow_direct_rescue_attempts = 0;
    const int adaptive_slow_direct_rescue_attempt_limit =
        adaptive_difficulty >= paper::kAdaptiveGuideActivationDifficultyThreshold
            ? paper::kAdaptiveSlowDirectRescueMediumAttempts
            : paper::kAdaptiveSlowDirectRescueEasyAttempts;
    for (const auto& candidate : candidates) {
        if (best_plan_1 && best_plan_2) {
            break;
        }
        if (!adaptive_prechecked_candidate_ids.empty() &&
            std::find(adaptive_prechecked_candidate_ids.begin(),
                      adaptive_prechecked_candidate_ids.end(),
                      candidate.candidate_id) != adaptive_prechecked_candidate_ids.end()) {
            continue;
        }
        if (remainingSeconds() <= 0.05) {
            break;
        }
        if (guide_ranking_function_ && guide_direct_cost_gate_enabled_ &&
            has_direct_success && candidate.heuristic_cost >= best_cost) {
            continue;
        }
        bool allow_legacy_relaxed_rescue_candidate = false;
        bool allow_adaptive_slow_direct_rescue_candidate = false;
        if (!guide_ranking_function_ && has_direct_success && !best_plan_1 &&
            allow_slow_direct_rescue) {
            if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode) &&
                adaptive_slow_direct_rescue_attempts < adaptive_slow_direct_rescue_attempt_limit) {
                allow_adaptive_slow_direct_rescue_candidate =
                    isAdaptiveSlowDirectRescueCandidate(
                        candidate, adaptive_difficulty, best_cost);
            }
            if (!allow_adaptive_slow_direct_rescue_candidate &&
                legacy_relaxed_rescue_attempts < paper::kHeuristicRescueMaxRelaxedAttempts &&
                candidate.ranking_score >= best_cost) {
                allow_legacy_relaxed_rescue_candidate =
                    isConservativeHeuristicRescueCandidate(candidate, target_pose, best_cost);
            }
        }
        const bool allow_relaxed_rescue_candidate =
            allow_legacy_relaxed_rescue_candidate ||
            allow_adaptive_slow_direct_rescue_candidate;
        if (adaptive_ellipsoid_enabled_ && usesAdaptiveDifficulty(ablation_mode) &&
            has_direct_success &&
            !allow_relaxed_rescue_candidate) {
            continue;
        }
        if (!guide_ranking_function_ && candidate.ranking_score >= best_cost &&
            !allow_relaxed_rescue_candidate) {
            continue;
        }

        if (allow_adaptive_slow_direct_rescue_candidate) {
            ++adaptive_slow_direct_rescue_attempts;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided adaptive slow-direct rescue 放行 candidate=%d direct_wall=%.1fms cost_delta=%.4f clearance=%.4f manip=%.4f axial=%.3f lateral=%.3f",
                candidate.candidate_id,
                direct_attempt_wall_time_ms,
                candidate.cost_delta_to_direct,
                candidate.clearance_margin,
                candidate.manipulability_score,
                candidate.axial_progress,
                candidate.lateral_offset);
        } else if (allow_legacy_relaxed_rescue_candidate) {
            ++legacy_relaxed_rescue_attempts;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided 保守 rescue 放行 candidate=%d direct_wall=%.1fms cost_delta=%.4f clearance=%.4f manip=%.4f axial=%.3f lateral=%.3f",
                candidate.candidate_id,
                direct_attempt_wall_time_ms,
                candidate.cost_delta_to_direct,
                candidate.clearance_margin,
                candidate.manipulability_score,
                candidate.axial_progress,
                candidate.lateral_offset);
        }

        tryGuideCandidate(candidate, kPerAttemptCapS);
    }

    if (!best_plan_1 &&
        !bridge_plan_1 &&
        guide_bridge_enabled &&
        (!has_direct_success || allow_slow_direct_rescue)) {
        const auto bridge_sequences = paper::buildGuideBridgeSequences(
            candidates,
            start_pose,
            target_pose,
            std::isfinite(best_cost) ? best_cost : direct_path_cost,
            adaptive_difficulty,
            getGuideBridgeMaxSequencesFromEnv());
        const double per_attempt_cap_s =
            adaptive_difficulty >= paper::kAdaptiveHardDifficultyThreshold
                ? paper::kGuideBridgeHardAttemptCapS
                : paper::kGuideBridgeAttemptCapS;

        for (const auto& bridge : bridge_sequences) {
            if (remainingSeconds() <= 0.05) {
                break;
            }
            if (guide_max_attempts_ > 0 &&
                guide_candidates_attempted >= static_cast<int>(guide_max_attempts_)) {
                break;
            }

            ++guide_candidates_attempted;
            moveit::planning_interface::MoveGroupInterface::Plan plan_1;
            if (!planWithBudget(bridge.first->pose, plan_1, start_state, per_attempt_cap_s)) {
                continue;
            }

            moveit::core::RobotState first_end_state(start_state);
            if (!buildEndStateFromPlan(plan_1, first_end_state)) {
                continue;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_2;
            if (!planWithBudget(bridge.second->pose, plan_2, first_end_state, per_attempt_cap_s)) {
                continue;
            }

            moveit::core::RobotState second_end_state(first_end_state);
            if (!buildEndStateFromPlan(plan_2, second_end_state)) {
                continue;
            }

            moveit::planning_interface::MoveGroupInterface::Plan plan_3;
            if (!planWithBudget(target_pose, plan_3, second_end_state, per_attempt_cap_s)) {
                continue;
            }

            best_cost = bridge.ranking_score;
            bridge_plan_1 = plan_1;
            bridge_plan_2 = plan_2;
            bridge_plan_3 = plan_3;
            selected_candidate_id = bridge.second->candidate_id;
            selected_candidate_learned_probability = std::max(
                bridge.first->learned_probability,
                bridge.second->learned_probability);
            selected_candidate_heuristic_cost =
                0.5 * (bridge.first->heuristic_cost + bridge.second->heuristic_cost);
            selected_candidate_ranking_score = bridge.ranking_score;
            selected_candidate_point = bridge.second->pose.position;
            RCLCPP_INFO(
                node_->get_logger(),
                "HeuristicGuided guide-bridge 选中 pair=(%d,%d) score=%.4f progress_gain=%.3f min_clearance=%.4f",
                bridge.first->candidate_id,
                bridge.second->candidate_id,
                bridge.ranking_score,
                bridge.progress_gain,
                bridge.min_clearance);
            break;
        }
    }

    move_group_->setPlanningTime(original_planning_time);
    move_group_->setStartStateToCurrentState();

    const bool success =
        static_cast<bool>(best_plan_1 && best_plan_2) ||
        static_cast<bool>(bridge_plan_1 && bridge_plan_2 && bridge_plan_3) ||
        has_direct_success;
    used_direct_plan =
        has_direct_success &&
        !static_cast<bool>(best_plan_1 && best_plan_2) &&
        !static_cast<bool>(bridge_plan_1 && bridge_plan_2 && bridge_plan_3);
    finalize_metrics(success);
    return success;
}

geometry_msgs::msg::Pose CR5Robot::getCurrentPose() {
    if (!move_group_) return geometry_msgs::msg::Pose();
    return move_group_->getCurrentPose().pose;
}

bool CR5Robot::computeIKForPose(const geometry_msgs::msg::Pose& target_pose,
                                std::vector<double>* joint_values,
                                const std::string& start_state_name) const {
    if (!move_group_) {
        return false;
    }

    moveit::core::RobotState seed_state(move_group_->getRobotModel());
    if (!buildStartState(start_state_name, seed_state)) {
        const auto* joint_model_group = seed_state.getJointModelGroup(PLANNING_GROUP);
        if (!joint_model_group) {
            return false;
        }
        if (start_state_name.empty()) {
            seed_state.setToDefaultValues();
        } else {
            seed_state.setToDefaultValues(joint_model_group, start_state_name);
        }
        seed_state.update();
    }

    const auto* joint_model_group = seed_state.getJointModelGroup(PLANNING_GROUP);
    if (!joint_model_group) {
        return false;
    }

    const auto& link_names = joint_model_group->getLinkModelNames();
    if (link_names.empty()) {
        return false;
    }

    const std::string tip_link = link_names.back();
    const bool has_ik_solver =
        static_cast<bool>(joint_model_group->getSolverInstance()) &&
        joint_model_group->canSetStateFromIK(tip_link);
    if (!has_ik_solver) {
        const double radial =
            std::hypot(target_pose.position.x, target_pose.position.y);
        const double height = target_pose.position.z;
        const bool roughly_reachable =
            radial >= 0.18 && radial <= 0.95 &&
            height >= 0.05 && height <= 1.20;
        if (!roughly_reachable) {
            return false;
        }
        if (joint_values != nullptr) {
            joint_values->clear();
        }
        return true;
    }

    moveit::core::RobotState ik_state(seed_state);
    const bool success = ik_state.setFromIK(joint_model_group, target_pose, tip_link, 0.05);
    if (!success) {
        return false;
    }

    ik_state.update();
    if (joint_values != nullptr) {
        ik_state.copyJointGroupPositions(joint_model_group, *joint_values);
    }
    return true;
}

// 动态切换规划器
void CR5Robot::setPlanner(const std::string& planner_id) {
    if (!move_group_) return;
    move_group_->setPlannerId(planner_id);
    RCLCPP_INFO(node_->get_logger(), "规划器已切换到: %s", planner_id.c_str());
}

// BIT*规划器
bool CR5Robot::moveToPoseBIT(const geometry_msgs::msg::Pose& target_pose) {
    return moveToPoseWithPlanner(target_pose, "BITstar", 10.0);
}

double CR5Robot::moveLine(const geometry_msgs::msg::Pose& target_pose) {
    if (!move_group_) return 0.0;
    std::vector<geometry_msgs::msg::Pose> waypoints;
    waypoints.push_back(move_group_->getCurrentPose().pose);
    waypoints.push_back(target_pose);

    moveit_msgs::msg::RobotTrajectory trajectory_msg;
    const double fraction = move_group_->computeCartesianPath(waypoints, 0.01, 0.0,
                                                              trajectory_msg, true);

    if (fraction > 0.95) {
        auto current_state = move_group_->getCurrentState(1.0);
        if (!current_state) {
            RCLCPP_ERROR(node_->get_logger(), "直线规划执行失败：无法获取当前关节状态");
            return 0.0;
        }

        robot_trajectory::RobotTrajectory robot_trajectory(
            move_group_->getRobotModel(), PLANNING_GROUP);
        robot_trajectory.setRobotTrajectoryMsg(*current_state, trajectory_msg);

        trajectory_processing::IterativeParabolicTimeParameterization time_parameterization;
        if (!time_parameterization.computeTimeStamps(robot_trajectory, 1.0, 1.0)) {
            RCLCPP_ERROR(node_->get_logger(), "直线规划执行失败：轨迹时间参数化失败");
            return 0.0;
        }

        robot_trajectory.getRobotTrajectoryMsg(trajectory_msg);
        if (!isTrajectoryCollisionFree(trajectory_msg, *current_state, "moveLine")) {
            RCLCPP_ERROR(node_->get_logger(), "直线规划执行失败：轨迹碰撞复核未通过");
            return 0.0;
        }
        const auto exec_result = move_group_->execute(trajectory_msg);
        if (exec_result != moveit::core::MoveItErrorCode::SUCCESS) {
            RCLCPP_ERROR(node_->get_logger(), "直线规划执行失败：execute 返回错误码 %d",
                         exec_result.val);
            return 0.0;
        }
    } else {
        RCLCPP_WARN(node_->get_logger(), "⚠ 直线规划不完整 (%.1f%%) - 放弃执行", fraction * 100.0);
    }
    return fraction;
}

MeasurementResult CR5Robot::measureTipPoint(
    const geometry_msgs::msg::Point& tip_point,
    const geometry_msgs::msg::Quaternion& orientation,
    double approach_distance)
{
    MeasurementResult result;

    const geometry_msgs::msg::Pose target_pose =
        my_cr5_control::tool::buildFlangePoseFromTipPoint(tip_point, orientation);

    geometry_msgs::msg::Pose approach_pose = target_pose;
    approach_pose.position.z += approach_distance;

    // 移动到预备点
    setSpeed(0.25);
    bool reached = false;
    if (moveToPoseImproved(approach_pose)) {
        reached = true;
    } else {
        RCLCPP_WARN(node_->get_logger(), "改进算法失败，尝试标准规划...");
        if (moveToPose(approach_pose)) {
            reached = true;
        }
    }

    if (!reached) {
        RCLCPP_ERROR(node_->get_logger(), "预备点规划失败");
        result.reached_approach = false;
        return result;
    }
    result.reached_approach = true;

    // 直线触碰
    setSpeed(0.05);
    result.touch_fraction = moveLine(target_pose);
    result.final_flange_pose = getCurrentPose();

    if (result.touch_fraction > 0.95) {
        RCLCPP_INFO(node_->get_logger(), "触碰成功 (%.1f%%)", result.touch_fraction * 100.0);
    } else {
        RCLCPP_WARN(node_->get_logger(), "触碰不完整 (%.1f%%)", result.touch_fraction * 100.0);
    }

    // 后退
    setSpeed(0.20);
    double retract_fraction = moveLine(approach_pose);
    if (retract_fraction < 0.95) {
        moveToPose(approach_pose);
    }

    return result;
}

bool CR5Robot::moveJoints(const std::vector<double>& joints) {
    if (!move_group_) return false;
    auto current_state = move_group_->getCurrentState(1.0);
    if (!current_state) {
        RCLCPP_ERROR(node_->get_logger(), "moveJoints 失败：无法获取当前关节状态");
        return false;
    }
    move_group_->setStartState(*current_state);
    move_group_->setJointValueTarget(joints);
    moveit::planning_interface::MoveGroupInterface::Plan my_plan;
    if (move_group_->plan(my_plan) == moveit::core::MoveItErrorCode::SUCCESS) {
        const bool executed = executePlanIfCollisionFree(my_plan, *current_state, "moveJoints");
        move_group_->setStartStateToCurrentState();
        return executed;
    }
    move_group_->setStartStateToCurrentState();
    return false;
}

geometry_msgs::msg::Quaternion CR5Robot::calculateLookAtQuaternion(
    const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Point& target_pos)
{
    //  四元数计算逻辑 
    tf2::Vector3 from(current_pos.x, current_pos.y, current_pos.z);
    tf2::Vector3 to(target_pos.x, target_pos.y, target_pos.z);
    tf2::Vector3 direction = (to - from).normalized();
    tf2::Vector3 z_axis = direction;
    tf2::Vector3 up_temp(0, 0, 1);
    if (std::abs(z_axis.dot(up_temp)) > 0.999) up_temp = tf2::Vector3(1, 0, 0);
    tf2::Vector3 x_axis = up_temp.cross(z_axis).normalized();
    tf2::Vector3 y_axis = z_axis.cross(x_axis).normalized();
    tf2::Matrix3x3 mat(x_axis.x(), y_axis.x(), z_axis.x(),
                       x_axis.y(), y_axis.y(), z_axis.y(),
                       x_axis.z(), y_axis.z(), z_axis.z());
    tf2::Quaternion q;
    mat.getRotation(q);
    return tf2::toMsg(q);
}

// ============================================================================
// 自适应椭球采样接口实现 (Adaptive Informed Sampling)
// ============================================================================

void CR5Robot::enableAdaptiveEllipsoidSampling(bool enabled) {
    if (adaptive_ellipsoid_enabled_ == enabled) {
        return;
    }
    adaptive_ellipsoid_enabled_ = enabled;
    if (enabled) {
        RCLCPP_INFO(node_->get_logger(), "自适应椭球采样已启用");
    } else {
        RCLCPP_INFO(node_->get_logger(), "自适应椭球采样已禁用");
    }
}

void CR5Robot::setSceneDifficultyScore(double difficulty_score) {
    const double clamped = std::max(0.0, std::min(1.0, difficulty_score));
    if (std::abs(scene_difficulty_score_ - clamped) < 1e-6) {
        return;
    }
    scene_difficulty_score_ = clamped;
    RCLCPP_DEBUG(node_->get_logger(), "场景难度评分设置为: %.3f", scene_difficulty_score_);
}

double CR5Robot::getSceneDifficultyScore() const {
    return scene_difficulty_score_;
}
