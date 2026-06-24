#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_TYPES_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_TYPES_HPP

#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>

#include <functional>
#include <vector>

namespace my_cr5_control::paper_mainline {

struct PlanningMetrics {
    bool success{false};
    double wall_time_ms{0.0};
    double planner_reported_time_ms{0.0};
    double planning_budget_ms{0.0};
    int planner_calls{0};
    bool hit_budget_limit{false};
    int guide_candidate_count{0};
    int guide_candidates_attempted{0};
    bool direct_plan_success{false};
    double direct_attempt_wall_time_ms{0.0};
    double direct_path_cost{-1.0};
    bool used_direct_plan{false};
    int top_ranked_candidate_id{-1};
    double top_ranked_candidate_heuristic_cost{-1.0};
    double top_ranked_candidate_ranking_score{-1.0};
    double top_ranked_candidate_cost_delta_to_direct{-1.0};
    double top_ranked_candidate_clearance_margin{-1.0};
    double top_ranked_candidate_manipulability_score{-1.0};
    double top_ranked_candidate_axial_progress{-1.0};
    double top_ranked_candidate_lateral_offset{-1.0};
    int selected_candidate_id{-1};
    double selected_candidate_learned_probability{-1.0};
    double selected_candidate_heuristic_cost{-1.0};
    double selected_candidate_ranking_score{-1.0};
    geometry_msgs::msg::Point selected_candidate_point;
};

struct GuideCandidate {
    int candidate_id{-1};
    geometry_msgs::msg::Pose pose;
    double heuristic_cost{0.0};
    double ranking_score{0.0};
    double learned_probability{-1.0};
    bool enabled{true};
    bool ik_feasible{true};
    double direct_cost{0.0};
    double cost_delta_to_direct{0.0};
    double start_to_guide_distance{0.0};
    double guide_to_goal_distance{0.0};
    double total_guide_distance{0.0};
    double direct_distance{0.0};
    double axial_progress{0.0};
    double lateral_offset{0.0};
    double guide_penalty{0.0};
    double mid1_penalty{0.0};
    double mid2_penalty{0.0};
    double guide_height{0.0};
    double clearance_margin{0.0};
    double manipulability_score{0.0};
    double safety_penalty{0.0};
};

using GuideRankingFunction = std::function<void(
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    std::vector<GuideCandidate>& candidates)>;

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_TYPES_HPP
