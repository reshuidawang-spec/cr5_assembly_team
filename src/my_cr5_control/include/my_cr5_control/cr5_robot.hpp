#ifndef CR5_ROBOT_HPP
#define CR5_ROBOT_HPP

#include "my_cr5_control/paper_mainline/guide_types.hpp"

#include <rclcpp/rclcpp.hpp>
#include <rclcpp/executors/single_threaded_executor.hpp>
#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <moveit_msgs/msg/robot_trajectory.hpp>
#include <moveit/robot_state/robot_state.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp> 
#include <moveit_msgs/msg/collision_object.hpp>
#include <functional>
#include <optional>
#include <cstdint>
#include <random>
#include <string>
#include <thread>
#include <vector>

struct MeasurementResult {
    bool reached_approach = false;
    double touch_fraction = 0.0;
    geometry_msgs::msg::Pose final_flange_pose;
};

class CR5Robot {
public:
    using PlanningMetrics = my_cr5_control::paper_mainline::PlanningMetrics;
    using GuideCandidate = my_cr5_control::paper_mainline::GuideCandidate;
    using GuideRankingFunction = my_cr5_control::paper_mainline::GuideRankingFunction;

    CR5Robot(const std::string& node_name = "cr5_robot_client",
             bool attach_probe_model = true);
    ~CR5Robot();

    bool init(); 
    bool isReady() const;
    void setSpeed(double scaling); 
    void stopMotion();

    // 环境配置
    void setupCalibrationScene(); // 保留原有功能
    void addSimulationEnvironment(); // 仿真环境占位接口
    bool addMeshObstacle(const std::string& object_id, const std::string& mesh_resource,
                         const geometry_msgs::msg::Pose& mesh_pose, double scale = 1.0);
    bool addBoxObstacleObject(const std::string& object_id,
                              const geometry_msgs::msg::Pose& box_pose,
                              double dx,
                              double dy,
                              double dz);
    bool addCylinderObstacle(const std::string& object_id,
                             const geometry_msgs::msg::Pose& cylinder_pose,
                             double height_m,
                             double radius_m);
    void addBoxObstacle(double x, double y, double z, double dx, double dy, double dz); // 新增：添加测试箱体

    // 碰撞对象管理
    void removeCollisionObject(const std::string& object_id);
    void removeCalibrationScene(); // 移除标定场景
    void restoreCalibrationScene(); // 恢复标定场景
    void removeBox(); // 移除箱体
    void restoreBox(); // 恢复箱体

    // 运动指令
    bool moveToNamedTarget(const std::string& target_name);
    bool moveToPoseWithPlanner(const geometry_msgs::msg::Pose& target_pose,
                               const std::string& planner_id,
                               double planning_time = 8.0);
    bool planToPoseWithPlanner(const geometry_msgs::msg::Pose& target_pose,
                               const std::string& planner_id,
                               double planning_time = 8.0,
                               const std::string& start_state_name = "",
                               PlanningMetrics* metrics = nullptr);
    bool moveToPose(const geometry_msgs::msg::Pose& target_pose); // 避障路径规划
    bool moveToPoseImproved(const geometry_msgs::msg::Pose& target_pose); // 外部两阶段采样引导原型
    bool planToPoseImproved(const geometry_msgs::msg::Pose& target_pose,
                            const std::string& start_state_name = "",
                            double planning_budget_s = 8.0,
                            PlanningMetrics* metrics = nullptr,
                            std::size_t guide_sample_count = 24,
                            const PlanningMetrics* direct_metrics_override = nullptr); // 对应 HeuristicGuided 接口
    bool moveToPoseBIT(const geometry_msgs::msg::Pose& target_pose); // BIT*规划器
    double moveLine(const geometry_msgs::msg::Pose& target_pose); // 直线规划
    MeasurementResult measureTipPoint(const geometry_msgs::msg::Point& tip_point,
                                      const geometry_msgs::msg::Quaternion& orientation,
                                      double approach_distance = 0.08);
    bool moveJoints(const std::vector<double>& joints);
    geometry_msgs::msg::Pose getCurrentPose();
    bool computeIKForPose(const geometry_msgs::msg::Pose& target_pose,
                          std::vector<double>* joint_values = nullptr,
                          const std::string& start_state_name = "") const;
    geometry_msgs::msg::Quaternion calculateLookAtQuaternion(
        const geometry_msgs::msg::Point& current_pos, const geometry_msgs::msg::Point& target_pos);
    std::vector<GuideCandidate> sampleGuideCandidates(const geometry_msgs::msg::Pose& target_pose,
                                                      const std::string& start_state_name = "",
                                                      std::size_t sample_count = 24,
                                                      bool apply_ranking = false);
    bool planToPoseViaGuide(const geometry_msgs::msg::Pose& target_pose,
                            const geometry_msgs::msg::Pose& guide_pose,
                            const std::string& start_state_name = "",
                            double planning_budget_s = 8.0,
                            PlanningMetrics* metrics = nullptr);
    bool planToPoseWithPlannerTrajectory(const geometry_msgs::msg::Pose& target_pose,
                                         const std::string& planner_id,
                                         double planning_time,
                                         const std::string& start_state_name,
                                         moveit_msgs::msg::RobotTrajectory* trajectory,
                                         PlanningMetrics* metrics = nullptr);
    bool planToPoseViaGuideTrajectories(const geometry_msgs::msg::Pose& target_pose,
                                        const geometry_msgs::msg::Pose& guide_pose,
                                        const std::string& start_state_name,
                                        double planning_budget_s,
                                        std::vector<moveit_msgs::msg::RobotTrajectory>* trajectories,
                                        PlanningMetrics* metrics = nullptr);
    std::vector<geometry_msgs::msg::Point> endEffectorPathFromTrajectory(
        const moveit_msgs::msg::RobotTrajectory& trajectory,
        const std::string& link_name = "Link6") const;
    void setGuideRankingFunction(GuideRankingFunction ranking_function,
                                 const std::string& ranking_name = "custom");
    void clearGuideRankingFunction();
    void setGuideDirectCostGateEnabled(bool enabled);
    void setGuideMaxAttempts(std::size_t max_attempts);
    void setGuideSamplingSeed(std::uint32_t seed);
    void clearGuideSamplingSeed();
    void setGuideEnvironmentBoxHint(double x, double y, double z, double dx, double dy, double dz);
    void clearGuideEnvironmentBoxHint();

    // 自适应椭球采样接口 (Adaptive Informed Sampling)
    void enableAdaptiveEllipsoidSampling(bool enabled);
    void setSceneDifficultyScore(double difficulty_score); // 0.0=easy, 0.5=medium, 1.0=hard
    double getSceneDifficultyScore() const;

    // 规划器配置
    void setPlanner(const std::string& planner_id); // 动态切换规划器

private:
    void attachProbeModel();
    bool buildStartState(const std::string& start_state_name, moveit::core::RobotState& start_state) const;
    geometry_msgs::msg::Pose getPoseFromState(const moveit::core::RobotState& state) const;
    bool planToPoseInternal(const geometry_msgs::msg::Pose& target_pose,
                            moveit::planning_interface::MoveGroupInterface::Plan& plan);
    bool buildEndStateFromPlan(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                               moveit::core::RobotState& end_state) const;
    bool isTrajectoryCollisionFree(const moveit_msgs::msg::RobotTrajectory& trajectory,
                                   const moveit::core::RobotState& start_state,
                                   const std::string& context) const;
    bool isPlanCollisionFree(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                             const moveit::core::RobotState& start_state,
                             const std::string& context) const;
    bool executePlanIfCollisionFree(const moveit::planning_interface::MoveGroupInterface::Plan& plan,
                                    const moveit::core::RobotState& start_state,
                                    const std::string& context);

    std::vector<geometry_msgs::msg::Pose> generateEllipsoidGuideSamples(
        const geometry_msgs::msg::Pose& start_pose, const geometry_msgs::msg::Pose& goal_pose,
        std::size_t sample_count) const;
    std::vector<geometry_msgs::msg::Pose> generateEnvironmentAnchorGuideSamples(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose) const;
    std::vector<geometry_msgs::msg::Pose> generateRefinedGuideSamples(
        const geometry_msgs::msg::Pose& start_pose,
        const geometry_msgs::msg::Pose& goal_pose,
        const std::vector<GuideCandidate>& seed_candidates,
        std::size_t extra_count) const;
    GuideCandidate evaluateGuideCandidate(const geometry_msgs::msg::Pose& start_pose,
                                         const geometry_msgs::msg::Pose& goal_pose,
                                         const geometry_msgs::msg::Pose& guide_pose,
                                         const moveit::core::RobotState& ik_seed_state) const;
    std::vector<GuideCandidate> buildGuideCandidates(const moveit::core::RobotState& start_state,
                                                     const geometry_msgs::msg::Pose& start_pose,
                                                     const geometry_msgs::msg::Pose& goal_pose,
                                                     std::size_t sample_count,
                                                     bool apply_ranking) const;
    std::mt19937 makeGuideSamplingRng() const;
    void applyGuideRanking(const geometry_msgs::msg::Pose& start_pose,
                           const geometry_msgs::msg::Pose& goal_pose,
                           std::vector<GuideCandidate>& candidates) const;
    bool getGuideEnvironmentBox(double& x, double& y, double& z,
                                double& dx, double& dy, double& dz) const;
    double computeEnvironmentClearance(const geometry_msgs::msg::Point& point) const;
    double estimateGuideManipulability(const geometry_msgs::msg::Pose& pose,
                                       const moveit::core::RobotState& ik_seed_state,
                                       bool* ik_feasible) const;
    double computeObstacleAreaPenalty(const geometry_msgs::msg::Point& point) const;
    double computeImprovedPathCost(const geometry_msgs::msg::Pose& start_pose,
                                   const geometry_msgs::msg::Pose& guide_pose,
                                   const geometry_msgs::msg::Pose& goal_pose) const;
    double computeDirectPathCost(const geometry_msgs::msg::Pose& start_pose,
                                 const geometry_msgs::msg::Pose& goal_pose) const;
    bool isDeepInsertionGoalForHeuristicRescue(const geometry_msgs::msg::Pose& goal_pose) const;
    bool isCenteredInsertionGoalForHeuristicRescue(
        const geometry_msgs::msg::Pose& goal_pose) const;
    bool isConservativeHeuristicRescueCandidate(const GuideCandidate& candidate,
                                                const geometry_msgs::msg::Pose& goal_pose,
                                                double direct_cost) const;
    bool isAdaptiveSlowDirectRescueCandidate(const GuideCandidate& candidate,
                                             double adaptive_difficulty,
                                             double direct_cost) const;

    rclcpp::Node::SharedPtr node_;
    rclcpp::executors::SingleThreadedExecutor::SharedPtr executor_;
    std::shared_ptr<moveit::planning_interface::MoveGroupInterface> move_group_;
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface_;
    std::thread executor_thread_;
    bool attach_probe_model_{true};
    bool has_environment_box_{false};
    double environment_box_x_{0.0};
    double environment_box_y_{0.0};
    double environment_box_z_{0.0};
    double environment_box_dx_{0.0};
    double environment_box_dy_{0.0};
    double environment_box_dz_{0.0};
    bool has_calibration_scene_{false};
    bool has_guide_environment_box_hint_{false};
    double guide_environment_box_x_{0.0};
    double guide_environment_box_y_{0.0};
    double guide_environment_box_z_{0.0};
    double guide_environment_box_dx_{0.0};
    double guide_environment_box_dy_{0.0};
    double guide_environment_box_dz_{0.0};
    mutable bool has_guide_sampling_seed_{false};
    mutable std::uint32_t guide_sampling_seed_{0};
    mutable std::uint64_t guide_sampling_stream_index_{0};
    GuideRankingFunction guide_ranking_function_;
    std::string guide_ranking_name_{"heuristic_cost"};
    bool guide_direct_cost_gate_enabled_{false};
    std::size_t guide_max_attempts_{0};

    // 自适应椭球采样参数 (Adaptive Informed Sampling)
    bool adaptive_ellipsoid_enabled_{false};
    double scene_difficulty_score_{0.5}; // 0.0=easy, 0.5=medium, 1.0=hard
};

#endif
