#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_POLICY_PARAMS_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_POLICY_PARAMS_HPP

#include <cstddef>

namespace my_cr5_control::paper_mainline {

inline constexpr double kGuidePreferredClearanceM = 0.03;
inline constexpr double kGuideHardClearanceM = -0.002;
inline constexpr double kGuidePreferredManipulability = 0.08;
inline constexpr double kGuideSlowDirectCostSlack = 0.05;
inline constexpr double kHeuristicRescueCostSlack = 0.02;
inline constexpr double kHeuristicRescueMinClearanceM = 0.015;
inline constexpr double kHeuristicRescueMinManipulability = 0.05;
inline constexpr double kHeuristicRescueMaxSafetyPenalty = 0.08;
inline constexpr double kHeuristicRescueMinAxialProgress = 0.15;
inline constexpr double kHeuristicRescueMaxAxialProgress = 0.80;
inline constexpr double kHeuristicRescueMinLateralOffsetM = 0.01;
inline constexpr double kHeuristicRescueMaxLateralOffsetM = 0.12;
inline constexpr double kHeuristicRescueMinGoalHeightAboveBoxTopM = 0.06;
inline constexpr double kHeuristicRescueMaxGoalCenterOffsetM = 0.02;
inline constexpr int kHeuristicRescueMaxRelaxedAttempts = 1;
inline constexpr double kAdaptiveSlowDirectRescueEasyCostSlack = 0.012;
inline constexpr double kAdaptiveSlowDirectRescueMediumCostSlack = 0.020;
inline constexpr double kAdaptiveSlowDirectRescueMinClearanceM = 0.010;
inline constexpr double kAdaptiveSlowDirectRescueMinManipulability = 0.05;
inline constexpr double kAdaptiveSlowDirectRescueMaxSafetyPenalty = 0.10;
inline constexpr double kAdaptiveSlowDirectRescueMinAxialProgress = 0.10;
inline constexpr double kAdaptiveSlowDirectRescueMaxAxialProgress = 0.95;
inline constexpr double kAdaptiveSlowDirectRescueMinLateralOffsetM = 0.005;
inline constexpr double kAdaptiveSlowDirectRescueMaxLateralOffsetM = 0.14;
inline constexpr int kAdaptiveSlowDirectRescueEasyAttempts = 1;
inline constexpr int kAdaptiveSlowDirectRescueMediumAttempts = 2;
inline constexpr std::size_t kGuideRefinementSeedCount = 4;
inline constexpr std::size_t kGuideRefinementPerSeed = 2;
inline constexpr double kAdaptiveGuideFirstDifficultyThreshold = 0.70;
inline constexpr double kAdaptiveHardDifficultyThreshold = 0.80;
inline constexpr double kAdaptiveGuideActivationDifficultyThreshold = 0.55;
inline constexpr double kAdaptiveGuideFirstAttemptCapS = 0.35;
inline constexpr double kGuideBridgeActivationDifficultyThreshold = 0.70;
inline constexpr double kGuideBridgeAttemptCapS = 0.45;
inline constexpr double kGuideBridgeHardAttemptCapS = 0.60;
inline constexpr double kGuideBridgeMinProgressGain = 0.08;
inline constexpr double kGuideBridgeMinWaypointSpacingM = 0.03;
inline constexpr double kGuideBridgeMaxTransitionDistanceM = 0.35;
inline constexpr std::size_t kGuideBridgeDefaultMaxSequences = 6;

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_GUIDE_POLICY_PARAMS_HPP
