#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <iomanip>
#include <rclcpp/rclcpp.hpp>

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("inspect_box_features");

    measurement::MeasurementPointGenerator generator;
    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    const std::string stl_path = my_cr5_control::scene::getV2StlPath(profile);
    RCLCPP_INFO(logger, "读取模型: %s", stl_path.c_str());

    if (!generator.loadSTLModel(stl_path)) {
        RCLCPP_ERROR(logger, "STL 加载失败");
        rclcpp::shutdown();
        return 1;
    }

    const auto geometry = generator.extractBoxFeatures();
    const auto points = generator.generatePoints(geometry, 3);

    RCLCPP_INFO(logger, "bbox center=(%.4f, %.4f, %.4f) size=(%.4f, %.4f, %.4f)",
                geometry.center.x(),
                geometry.center.y(),
                geometry.center.z(),
                geometry.width,
                geometry.height,
                geometry.depth);
    RCLCPP_INFO(logger, "features: holes=%zu cavities=%zu surfaces=%zu edges=%zu corners=%zu points=%zu",
                geometry.holes.size(),
                geometry.cavities.size(),
                geometry.surfaces.size(),
                geometry.edges.size(),
                geometry.corners.size(),
                points.size());

    for (std::size_t i = 0; i < geometry.holes.size(); ++i) {
        const auto& hole = geometry.holes[i];
        RCLCPP_INFO(logger, "hole[%zu]: center=(%.4f, %.4f, %.4f) radius=%.4f depth=%.4f",
                    i,
                    hole.center.x(),
                    hole.center.y(),
                    hole.center.z(),
                    hole.radius,
                    hole.depth);
    }

    for (std::size_t i = 0; i < geometry.cavities.size(); ++i) {
        const auto& cavity = geometry.cavities[i];
        RCLCPP_INFO(logger, "cavity[%zu]: entrance=(%.4f, %.4f, %.4f) bottom=(%.4f, %.4f, %.4f) width=%.4f depth=%.4f narrowness=%.3f",
                    i,
                    cavity.entrance_center.x(),
                    cavity.entrance_center.y(),
                    cavity.entrance_center.z(),
                    cavity.bottom_center.x(),
                    cavity.bottom_center.y(),
                    cavity.bottom_center.z(),
                    cavity.entrance_width,
                    cavity.depth,
                    cavity.narrowness);
    }

    if (!points.empty()) {
        const auto& first = points.front();
        RCLCPP_INFO(logger, "first_point: type=%d pos=(%.4f, %.4f, %.4f) difficulty=%.3f desc=%s",
                    static_cast<int>(first.type),
                    first.position.x,
                    first.position.y,
                    first.position.z,
                    first.difficulty_score,
                    first.description.c_str());
    }

    rclcpp::shutdown();
    return 0;
}
