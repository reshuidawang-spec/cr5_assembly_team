#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_BRIDGE_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_BRIDGE_HPP

#include "my_cr5_control/paper_mainline/guide_geometry.hpp"
#include "my_cr5_control/paper_mainline/guide_policy_params.hpp"
#include "my_cr5_control/paper_mainline/guide_types.hpp"

#include <geometry_msgs/msg/pose.hpp>

#include <algorithm>
#include <cmath>
#include <limits>
#include <vector>

namespace my_cr5_control::paper_mainline {

struct GuideBridgeSequence {
    const GuideCandidate* first{nullptr};
    const GuideCandidate* second{nullptr};
    double ranking_score{std::numeric_limits<double>::infinity()};
    double transition_distance{0.0};
    double progress_gain{0.0};
    double min_clearance{0.0};
};

inline std::vector<GuideBridgeSequence> buildGuideBridgeSequences(
    const std::vector<GuideCandidate>& candidates,
    const geometry_msgs::msg::Pose& start_pose,
    const geometry_msgs::msg::Pose& goal_pose,
    double direct_path_cost,
    double adaptive_difficulty,
    std::size_t max_sequences) {
    std::vector<GuideBridgeSequence> bridges;
    if (candidates.size() < 2 || max_sequences == 0 ||
        adaptive_difficulty < kGuideBridgeActivationDifficultyThreshold) {
        return bridges;
    }

    const std::size_t seed_count = std::min<std::size_t>(
        candidates.size(),
        adaptive_difficulty >= kAdaptiveHardDifficultyThreshold ? 8u : 6u);
    const double direct_distance =
        pointDistance(start_pose.position, goal_pose.position);
    const double baseline_cost = std::max(direct_path_cost, direct_distance);
    const double max_path_stretch =
        adaptive_difficulty >= kAdaptiveHardDifficultyThreshold ? 1.65 : 1.45;

    for (std::size_t i = 0; i < seed_count; ++i) {
        const auto& first = candidates[i];
        if (!first.enabled || !first.ik_feasible ||
            !std::isfinite(first.ranking_score)) {
            continue;
        }
        if (first.axial_progress < 0.05 || first.axial_progress > 0.92) {
            continue;
        }

        for (std::size_t j = 0; j < seed_count; ++j) {
            if (i == j) {
                continue;
            }

            const auto& second = candidates[j];
            if (!second.enabled || !second.ik_feasible ||
                !std::isfinite(second.ranking_score)) {
                continue;
            }

            const double progress_gain = second.axial_progress - first.axial_progress;
            if (progress_gain < kGuideBridgeMinProgressGain) {
                continue;
            }

            const double transition_distance =
                pointDistance(first.pose.position, second.pose.position);
            if (transition_distance < kGuideBridgeMinWaypointSpacingM ||
                transition_distance > kGuideBridgeMaxTransitionDistanceM) {
                continue;
            }

            const double total_bridge_length =
                pointDistance(start_pose.position, first.pose.position) +
                transition_distance +
                pointDistance(second.pose.position, goal_pose.position);
            if (total_bridge_length >
                std::max(baseline_cost * max_path_stretch, baseline_cost + 0.20)) {
                continue;
            }

            const double min_clearance =
                std::min(first.clearance_margin, second.clearance_margin);
            if (min_clearance < 0.005) {
                continue;
            }

            double score =
                first.ranking_score +
                0.85 * second.ranking_score +
                0.15 * transition_distance;
            score -= 0.010 * adaptive_difficulty *
                     std::clamp(progress_gain / 0.50, 0.0, 1.0);

            if (second.lateral_offset + 0.005 < first.lateral_offset) {
                score -= 0.004 * adaptive_difficulty;
            }
            if (first.pose.position.z > goal_pose.position.z + 0.02) {
                score -= 0.003 * adaptive_difficulty;
            }
            if (second.clearance_margin > first.clearance_margin) {
                score -= 0.003 * adaptive_difficulty;
            }

            bridges.push_back(
                {&first, &second, score, transition_distance, progress_gain, min_clearance});
        }
    }

    std::stable_sort(bridges.begin(), bridges.end(),
                     [](const GuideBridgeSequence& lhs, const GuideBridgeSequence& rhs) {
                         if (lhs.ranking_score != rhs.ranking_score) {
                             return lhs.ranking_score < rhs.ranking_score;
                         }
                         if (lhs.min_clearance != rhs.min_clearance) {
                             return lhs.min_clearance > rhs.min_clearance;
                         }
                         return lhs.progress_gain > rhs.progress_gain;
                     });

    bridges.erase(
        std::unique(bridges.begin(), bridges.end(),
                    [](const GuideBridgeSequence& lhs, const GuideBridgeSequence& rhs) {
                        return lhs.first->candidate_id == rhs.first->candidate_id &&
                               lhs.second->candidate_id == rhs.second->candidate_id;
                    }),
        bridges.end());

    if (bridges.size() > max_sequences) {
        bridges.resize(max_sequences);
    }
    return bridges;
}

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_BRIDGE_HPP
