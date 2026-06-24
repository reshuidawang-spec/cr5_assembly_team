#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_V2_SCENARIO_PROFILE_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_V2_SCENARIO_PROFILE_HPP

#include "my_cr5_control/measurement_point_generator.hpp"

#include <string>
#include <vector>

namespace my_cr5_control::paper_mainline {

struct V2ScenarioSpec {
    std::string name;
    std::string difficulty;
    std::vector<measurement::PointType> point_type_priority;
    bool hero_only{false};
    bool prefer_deepest_point{false};
};

inline const std::vector<V2ScenarioSpec>& canonicalV2ScenarioSpecs() {
    static const std::vector<V2ScenarioSpec> specs = {
        {"Easy_HoleCenter", "easy", {measurement::PointType::HOLE_CENTER,
                                      measurement::PointType::SURFACE}},
        {"Medium_HoleEdge", "medium", {measurement::PointType::HOLE_EDGE,
                                         measurement::PointType::EDGE}},
        {"Hard_DeepInterior", "hard", {measurement::PointType::INTERIOR_DEEP,
                                         measurement::PointType::CORNER}},
        {"Extreme_NarrowPassage", "extreme", {measurement::PointType::NARROW_PASSAGE,
                                                measurement::PointType::INTERIOR_DEEP}},
        {"Extreme_OffsetThroatDeepCavity", "extreme", {measurement::PointType::NARROW_PASSAGE,
                                                        measurement::PointType::INTERIOR_DEEP}, true, true},
    };
    return specs;
}

inline const std::vector<std::string>& canonicalV2DifficultyOrder() {
    static const std::vector<std::string> difficulties = {"easy", "medium", "hard", "extreme"};
    return difficulties;
}

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_V2_SCENARIO_PROFILE_HPP
