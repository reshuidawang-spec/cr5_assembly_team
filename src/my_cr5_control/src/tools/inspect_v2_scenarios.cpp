#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("inspect_v2_scenarios");

    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    RCLCPP_INFO(logger, "V2 mesh profile: %s", profile.name.c_str());

    measurement::MeasurementPointGenerator generator;
    const auto scenarios = generator.generateTestScenarios();
    for (const auto& scenario : scenarios) {
        if (scenario.points.empty()) {
            RCLCPP_INFO(logger, "scenario=%s difficulty=%s points=0",
                        scenario.name.c_str(), scenario.difficulty.c_str());
            continue;
        }

        const auto& point = scenario.points.front();
        RCLCPP_INFO(
            logger,
            "scenario=%s difficulty=%s point_type=%d score=%.3f tip=(%.4f, %.4f, %.4f) desc=%s",
            scenario.name.c_str(),
            scenario.difficulty.c_str(),
            static_cast<int>(point.type),
            point.difficulty_score,
            point.position.x,
            point.position.y,
            point.position.z,
            point.description.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
