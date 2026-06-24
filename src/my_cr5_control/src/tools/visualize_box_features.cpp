#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <chrono>
#include <geometry_msgs/msg/point.hpp>
#include <rclcpp/rclcpp.hpp>
#include <visualization_msgs/msg/marker_array.hpp>

namespace {

geometry_msgs::msg::Point toPoint(const Eigen::Vector3d& vector) {
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

visualization_msgs::msg::Marker baseMarker(const std::string& frame_id,
                                           const std::string& ns,
                                           int id,
                                           int type) {
    visualization_msgs::msg::Marker marker;
    marker.header.frame_id = frame_id;
    marker.header.stamp = rclcpp::Clock().now();
    marker.ns = ns;
    marker.id = id;
    marker.type = type;
    marker.action = visualization_msgs::msg::Marker::ADD;
    marker.pose.orientation.w = 1.0;
    marker.lifetime = rclcpp::Duration::from_seconds(0.0);
    return marker;
}

void setColor(visualization_msgs::msg::Marker& marker,
              float r,
              float g,
              float b,
              float a) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

Eigen::Vector3d pointToEigen(const geometry_msgs::msg::Point& point) {
    return Eigen::Vector3d(point.x, point.y, point.z);
}

}  // namespace

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("visualize_box_features");
    auto logger = node->get_logger();

    measurement::MeasurementPointGenerator generator;
    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    const std::string stl_path = my_cr5_control::scene::getV2StlPath(profile);
    if (!generator.loadSTLModel(stl_path)) {
        RCLCPP_ERROR(logger, "STL 加载失败: %s", stl_path.c_str());
        rclcpp::shutdown();
        return 1;
    }

    const auto geometry = generator.extractBoxFeatures();
    const auto points = generator.generatePoints(geometry, 3);

    const auto qos = rclcpp::QoS(1).reliable().transient_local();
    auto publisher = node->create_publisher<visualization_msgs::msg::MarkerArray>(
        "box_feature_markers", qos);

    auto publish_once = [&, geometry, points]() {
        visualization_msgs::msg::MarkerArray marker_array;
        int id = 0;

        visualization_msgs::msg::Marker clear_marker;
        clear_marker.action = visualization_msgs::msg::Marker::DELETEALL;
        marker_array.markers.push_back(clear_marker);

        auto mesh_marker = baseMarker("base_link", "mesh", id++,
                                      visualization_msgs::msg::Marker::MESH_RESOURCE);
        mesh_marker.mesh_resource = profile.mesh_resource;
        mesh_marker.mesh_use_embedded_materials = false;
        mesh_marker.pose = my_cr5_control::scene::makeV2MeshPose(profile);
        mesh_marker.scale.x = profile.mesh_scale;
        mesh_marker.scale.y = profile.mesh_scale;
        mesh_marker.scale.z = profile.mesh_scale;
        setColor(mesh_marker, 0.70f, 0.72f, 0.76f, 0.25f);
        marker_array.markers.push_back(mesh_marker);

        auto bbox_marker = baseMarker("base_link", "bbox", id++,
                                      visualization_msgs::msg::Marker::LINE_LIST);
        bbox_marker.scale.x = 0.0015;
        setColor(bbox_marker, 0.15f, 0.75f, 0.95f, 0.95f);
        const int box_edges[12][2] = {
            {0, 1}, {1, 3}, {3, 2}, {2, 0},
            {4, 5}, {5, 7}, {7, 6}, {6, 4},
            {0, 4}, {1, 5}, {2, 6}, {3, 7},
        };
        for (const auto& edge : box_edges) {
            bbox_marker.points.push_back(toPoint(geometry.corners[edge[0]]));
            bbox_marker.points.push_back(toPoint(geometry.corners[edge[1]]));
        }
        marker_array.markers.push_back(bbox_marker);

        for (std::size_t i = 0; i < geometry.surfaces.size(); ++i) {
            const auto& surface = geometry.surfaces[i];
            auto marker = baseMarker("base_link", "surfaces", id++,
                                     visualization_msgs::msg::Marker::ARROW);
            marker.scale.x = 0.0020;
            marker.scale.y = 0.0040;
            marker.scale.z = 0.0060;
            setColor(marker, 0.20f, 0.90f, 0.30f, 0.90f);
            marker.points.push_back(toPoint(surface.center));
            marker.points.push_back(toPoint(surface.center + surface.normal.normalized() * 0.03));
            marker_array.markers.push_back(marker);
        }

        for (std::size_t i = 0; i < geometry.holes.size(); ++i) {
            const auto& hole = geometry.holes[i];
            auto marker = baseMarker("base_link", "holes", id++,
                                     visualization_msgs::msg::Marker::CYLINDER);
            marker.pose.position = toPoint(Eigen::Vector3d(
                hole.center.x(),
                hole.center.y(),
                hole.center.z() - 0.5 * hole.depth));
            marker.scale.x = 2.0 * hole.radius;
            marker.scale.y = 2.0 * hole.radius;
            marker.scale.z = hole.depth;
            setColor(marker, 0.15f, 0.35f, 0.95f, 0.45f);
            marker_array.markers.push_back(marker);

            auto label = baseMarker("base_link", "hole_labels", id++,
                                    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose.position = toPoint(hole.center + Eigen::Vector3d(0.0, 0.0, 0.025));
            label.scale.z = 0.012;
            label.text = "hole_" + std::to_string(i);
            setColor(label, 0.10f, 0.20f, 0.85f, 1.0f);
            marker_array.markers.push_back(label);
        }

        for (std::size_t i = 0; i < geometry.cavities.size(); ++i) {
            const auto& cavity = geometry.cavities[i];
            auto marker = baseMarker("base_link", "cavities", id++,
                                     visualization_msgs::msg::Marker::CUBE);
            marker.pose.position = toPoint(Eigen::Vector3d(
                cavity.entrance_center.x(),
                cavity.entrance_center.y(),
                cavity.entrance_center.z() - 0.5 * cavity.depth));
            marker.scale.x = cavity.entrance_width;
            marker.scale.y = cavity.entrance_width;
            marker.scale.z = cavity.depth;
            setColor(marker, 0.98f, 0.58f, 0.12f, 0.18f);
            marker_array.markers.push_back(marker);

            auto label = baseMarker("base_link", "cavity_labels", id++,
                                    visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
            label.pose.position = toPoint(cavity.entrance_center + Eigen::Vector3d(0.0, 0.0, 0.02));
            label.scale.z = 0.010;
            label.text = "cavity_" + std::to_string(i);
            setColor(label, 0.82f, 0.35f, 0.05f, 0.95f);
            marker_array.markers.push_back(label);
        }

        auto edge_marker = baseMarker("base_link", "edges", id++,
                                      visualization_msgs::msg::Marker::LINE_LIST);
        edge_marker.scale.x = 0.0010;
        setColor(edge_marker, 0.95f, 0.15f, 0.55f, 0.90f);
        for (const auto& edge : geometry.edges) {
            edge_marker.points.push_back(toPoint(edge.start));
            edge_marker.points.push_back(toPoint(edge.end));
        }
        marker_array.markers.push_back(edge_marker);

        auto corner_marker = baseMarker("base_link", "corners", id++,
                                        visualization_msgs::msg::Marker::SPHERE_LIST);
        corner_marker.scale.x = 0.004;
        corner_marker.scale.y = 0.004;
        corner_marker.scale.z = 0.004;
        setColor(corner_marker, 0.98f, 0.08f, 0.58f, 0.95f);
        for (const auto& corner : geometry.corners) {
            corner_marker.points.push_back(toPoint(corner));
        }
        marker_array.markers.push_back(corner_marker);

        for (std::size_t i = 0; i < points.size(); ++i) {
            const auto& point = points[i];
            auto marker = baseMarker("base_link", "measurement_points", id++,
                                     visualization_msgs::msg::Marker::SPHERE);
            marker.pose.position = point.position;
            marker.scale.x = 0.006;
            marker.scale.y = 0.006;
            marker.scale.z = 0.006;

            switch (point.type) {
                case measurement::PointType::HOLE_CENTER:
                    setColor(marker, 0.05f, 0.75f, 0.95f, 0.95f);
                    break;
                case measurement::PointType::HOLE_EDGE:
                    setColor(marker, 0.10f, 0.85f, 0.40f, 0.95f);
                    break;
                case measurement::PointType::INTERIOR_DEEP:
                    setColor(marker, 0.96f, 0.72f, 0.08f, 0.95f);
                    break;
                case measurement::PointType::NARROW_PASSAGE:
                    setColor(marker, 0.92f, 0.20f, 0.18f, 0.95f);
                    break;
                default:
                    setColor(marker, 0.90f, 0.90f, 0.90f, 0.95f);
                    break;
            }
            marker_array.markers.push_back(marker);

            auto normal_marker = baseMarker("base_link", "point_normals", id++,
                                            visualization_msgs::msg::Marker::ARROW);
            normal_marker.scale.x = 0.0015;
            normal_marker.scale.y = 0.0030;
            normal_marker.scale.z = 0.0045;
            setColor(normal_marker, 0.92f, 0.92f, 0.92f, 0.85f);
            const Eigen::Vector3d point_position = pointToEigen(point.position);
            const Eigen::Vector3d point_normal = pointToEigen(point.normal).normalized();
            normal_marker.points.push_back(point.position);
            normal_marker.points.push_back(toPoint(point_position + point_normal * 0.02));
            marker_array.markers.push_back(normal_marker);

            if (i < 12) {
                auto label = baseMarker("base_link", "point_labels", id++,
                                        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
                label.pose.position = point.position;
                label.pose.position.z += 0.010;
                label.scale.z = 0.008;
                label.text = std::to_string(i);
                setColor(label, 1.0f, 1.0f, 1.0f, 0.95f);
                marker_array.markers.push_back(label);
            }
        }

        publisher->publish(marker_array);
    };

    auto timer = node->create_wall_timer(std::chrono::milliseconds(1000), publish_once);
    static_cast<void>(timer);
    publish_once();

    RCLCPP_INFO(logger, "已发布箱体特征可视化到 /box_feature_markers");
    RCLCPP_INFO(logger, "请在 RViz 中将 Fixed Frame 设为 base_link，并添加 MarkerArray 显示");

    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}
