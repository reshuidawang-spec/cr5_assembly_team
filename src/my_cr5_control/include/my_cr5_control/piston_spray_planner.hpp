#ifndef MY_CR5_CONTROL_PISTON_SPRAY_PLANNER_HPP
#define MY_CR5_CONTROL_PISTON_SPRAY_PLANNER_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/vector3.hpp>

#include <string>
#include <vector>

namespace my_cr5_control::piston {

struct PistonSpecMm {
    double diameter_mm{85.0};
    double spray_length_mm{55.0};
};

struct SprayProcessMm {
    double spray_distance_mm{120.0};
    double spray_width_mm{18.0};
    double overlap_mm{6.0};
    double turntable_rpm{90.0};
    double flow_rate_ml_min{125.0};
    double lead_in_mm{5.0};
    double lead_out_mm{5.0};
    double radial_clearance_mm{40.0};
    double approach_speed_mm_s{60.0};
    int sample_count{60};
};

struct WorkpieceFrame {
    geometry_msgs::msg::Point origin;
    geometry_msgs::msg::Vector3 axial_direction;
    geometry_msgs::msg::Vector3 radial_direction;
    double tool_roll_deg{0.0};
};

struct ToolTcpOffsetMm {
    double x_mm{0.0};
    double y_mm{0.0};
    double z_mm{0.0};
};

struct PlanMetrics {
    double piston_radius_mm{0.0};
    double nozzle_radius_mm{0.0};
    double helical_pitch_mm_per_rev{0.0};
    double turntable_rps{0.0};
    double axial_feed_mm_s{0.0};
    double surface_speed_mm_s{0.0};
    double total_revolutions{0.0};
    double spray_time_s{0.0};
    double estimated_graphite_usage_ml{0.0};
    double overlap_ratio{0.0};
};

struct ExecutionStep {
    int step_index{0};
    std::string command;
    std::string pose_name;
    std::string motion_mode{"none"};
    double speed_mm_s{0.0};
    double numeric_value{0.0};
    std::string note;
};

struct KeyPose {
    std::string name;
    double local_x_mm{0.0};
    double local_y_mm{0.0};
    double local_z_mm{0.0};
    geometry_msgs::msg::Pose tcp_pose;
    geometry_msgs::msg::Pose flange_pose;
};

struct PathSample {
    double time_s{0.0};
    double local_x_mm{0.0};
    double local_y_mm{0.0};
    double local_z_mm{0.0};
    double surface_angle_deg{0.0};
    geometry_msgs::msg::Pose tcp_pose;
    geometry_msgs::msg::Pose flange_pose;
    geometry_msgs::msg::Point surface_point_base;
};

struct SprayPlan {
    PistonSpecMm piston;
    SprayProcessMm process;
    WorkpieceFrame frame;
    ToolTcpOffsetMm tool_tcp_offset;
    PlanMetrics metrics;
    std::vector<std::string> notices;
    std::vector<ExecutionStep> execution_steps;
    std::vector<KeyPose> key_poses;
    std::vector<PathSample> path_samples;
};

bool validateInputs(const PistonSpecMm& piston,
                    const SprayProcessMm& process,
                    const WorkpieceFrame& frame,
                    std::vector<std::string>* errors);

bool buildSprayPlan(const PistonSpecMm& piston,
                    const SprayProcessMm& process,
                    const WorkpieceFrame& frame,
                    const ToolTcpOffsetMm& tool_tcp_offset,
                    SprayPlan* plan,
                    std::string* error_message);

geometry_msgs::msg::Point mapLocalPointToBase(const WorkpieceFrame& frame,
                                              double x_mm,
                                              double y_mm,
                                              double z_mm);

geometry_msgs::msg::Quaternion computeSprayOrientation(const WorkpieceFrame& frame,
                                                       bool* ok = nullptr,
                                                       std::string* error_message = nullptr);

geometry_msgs::msg::Pose computeFlangePoseFromTcp(const geometry_msgs::msg::Pose& tcp_pose,
                                                  const ToolTcpOffsetMm& tool_tcp_offset);

geometry_msgs::msg::Pose computeTcpPoseFromFlange(const geometry_msgs::msg::Pose& flange_pose,
                                                  const ToolTcpOffsetMm& tool_tcp_offset);

}  // namespace my_cr5_control::piston

#endif  // MY_CR5_CONTROL_PISTON_SPRAY_PLANNER_HPP
