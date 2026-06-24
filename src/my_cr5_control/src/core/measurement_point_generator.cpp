#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/paper_mainline/v2_scenario_profile.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <algorithm>
#include <cmath>
#include <cstdint>
#include <fstream>
#include <limits>
#include <queue>
#include <rclcpp/rclcpp.hpp>
#include <unordered_map>

namespace measurement {
namespace {

constexpr double kGeometryEpsilon = 1e-9;

struct GridKey {
    int x = 0;
    int y = 0;

    bool operator==(const GridKey& other) const {
        return x == other.x && y == other.y;
    }
};

struct GridKeyHash {
    std::size_t operator()(const GridKey& key) const {
        const std::int64_t packed =
            (static_cast<std::int64_t>(key.x) << 32) ^
            static_cast<std::uint32_t>(key.y);
        return std::hash<std::int64_t>{}(packed);
    }
};

double quantile(std::vector<double> values, double q) {
    if (values.empty()) {
        return 0.0;
    }

    q = std::clamp(q, 0.0, 1.0);
    const std::size_t index = static_cast<std::size_t>(q * static_cast<double>(values.size() - 1));
    std::nth_element(values.begin(), values.begin() + static_cast<std::ptrdiff_t>(index), values.end());
    return values[index];
}

void addHeroOffsetThroatFeature(BoxGeometry& geometry) {
    const auto profile = my_cr5_control::scene::makeHeroOffsetThroatV2MeshProfile();

    BoxGeometry::Hole entrance;
    entrance.center = my_cr5_control::scene::transformV2MeshPoint(
        profile, Eigen::Vector3d(440.0, 440.0, 760.0));
    entrance.axis = Eigen::Vector3d(0.0, 0.0, -1.0);
    entrance.radius = 0.0375;
    entrance.depth = 0.045;
    geometry.holes.insert(geometry.holes.begin(), entrance);

    BoxGeometry::Cavity cavity;
    cavity.entrance_center = my_cr5_control::scene::transformV2MeshPoint(
        profile, Eigen::Vector3d(500.0, 440.0, 570.0));
    cavity.bottom_center = my_cr5_control::scene::transformV2MeshPoint(
        profile, Eigen::Vector3d(500.0, 440.0, 80.0));
    cavity.entrance_width = 0.035;
    cavity.depth = cavity.entrance_center.z() - cavity.bottom_center.z();
    cavity.narrowness = 0.86;
    geometry.cavities.insert(geometry.cavities.begin(), cavity);
}

}  // namespace

MeasurementPointGenerator::MeasurementPointGenerator() {}

bool MeasurementPointGenerator::loadSTLModel(const std::string& stl_path) {
    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    std::ifstream file(stl_path, std::ios::binary);
    if (!file.is_open()) {
        RCLCPP_ERROR(rclcpp::get_logger("MeasurementPointGenerator"),
                     "无法打开STL文件: %s", stl_path.c_str());
        return false;
    }

    char header[80];
    file.read(header, 80);
    static_cast<void>(header);

    uint32_t num_triangles = 0;
    file.read(reinterpret_cast<char*>(&num_triangles), sizeof(uint32_t));

    mesh_.clear();
    mesh_.reserve(num_triangles);

    for (uint32_t i = 0; i < num_triangles; ++i) {
        Triangle tri;

        float nx = 0.0f;
        float ny = 0.0f;
        float nz = 0.0f;
        file.read(reinterpret_cast<char*>(&nx), sizeof(float));
        file.read(reinterpret_cast<char*>(&ny), sizeof(float));
        file.read(reinterpret_cast<char*>(&nz), sizeof(float));
        tri.normal = Eigen::Vector3d(nx, ny, nz);

        float v0x = 0.0f;
        float v0y = 0.0f;
        float v0z = 0.0f;
        float v1x = 0.0f;
        float v1y = 0.0f;
        float v1z = 0.0f;
        float v2x = 0.0f;
        float v2y = 0.0f;
        float v2z = 0.0f;
        file.read(reinterpret_cast<char*>(&v0x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v0y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v0z), sizeof(float));
        file.read(reinterpret_cast<char*>(&v1x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v1y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v1z), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2x), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2y), sizeof(float));
        file.read(reinterpret_cast<char*>(&v2z), sizeof(float));

        tri.v0 = my_cr5_control::scene::transformV2MeshPoint(profile, Eigen::Vector3d(v0x, v0y, v0z));
        tri.v1 = my_cr5_control::scene::transformV2MeshPoint(profile, Eigen::Vector3d(v1x, v1y, v1z));
        tri.v2 = my_cr5_control::scene::transformV2MeshPoint(profile, Eigen::Vector3d(v2x, v2y, v2z));

        uint16_t attribute_byte_count = 0;
        file.read(reinterpret_cast<char*>(&attribute_byte_count), sizeof(uint16_t));
        static_cast<void>(attribute_byte_count);

        mesh_.push_back(tri);
    }

    RCLCPP_INFO(rclcpp::get_logger("MeasurementPointGenerator"),
                "成功加载STL模型: %d 个三角形", static_cast<int>(mesh_.size()));

    return true;
}

BoxGeometry MeasurementPointGenerator::extractBoxFeatures() {
    BoxGeometry geometry;

    if (mesh_.empty()) {
        RCLCPP_ERROR(rclcpp::get_logger("MeasurementPointGenerator"),
                     "网格为空，无法提取特征");
        return geometry;
    }

    Eigen::Vector3d min_corner = mesh_[0].v0;
    Eigen::Vector3d max_corner = mesh_[0].v0;
    for (const auto& tri : mesh_) {
        for (const auto* vertex : {&tri.v0, &tri.v1, &tri.v2}) {
            min_corner = min_corner.cwiseMin(*vertex);
            max_corner = max_corner.cwiseMax(*vertex);
        }
    }

    geometry.min_corner = min_corner;
    geometry.max_corner = max_corner;
    geometry.center = 0.5 * (min_corner + max_corner);
    geometry.width = max_corner.x() - min_corner.x();
    geometry.height = max_corner.y() - min_corner.y();
    geometry.depth = max_corner.z() - min_corner.z();

    RCLCPP_INFO(rclcpp::get_logger("MeasurementPointGenerator"),
                "箱体尺寸: %.3f x %.3f x %.3f m",
                geometry.width, geometry.height, geometry.depth);

    const double x0 = geometry.min_corner.x();
    const double x1 = geometry.max_corner.x();
    const double y0 = geometry.min_corner.y();
    const double y1 = geometry.max_corner.y();
    const double z0 = geometry.min_corner.z();
    const double z1 = geometry.max_corner.z();

    auto add_surface = [&](const Eigen::Vector3d& center,
                           const Eigen::Vector3d& normal,
                           double span_u,
                           double span_v,
                           const std::string& label) {
        BoxGeometry::SurfacePatch patch;
        patch.center = center;
        patch.normal = normal;
        patch.span_u = span_u;
        patch.span_v = span_v;
        patch.area = span_u * span_v;
        patch.label = label;
        geometry.surfaces.push_back(patch);
    };

    add_surface(Eigen::Vector3d(geometry.center.x(), geometry.center.y(), z1),
                Eigen::Vector3d(0.0, 0.0, 1.0),
                geometry.width,
                geometry.height,
                "top");
    add_surface(Eigen::Vector3d(geometry.center.x(), geometry.center.y(), z0),
                Eigen::Vector3d(0.0, 0.0, -1.0),
                geometry.width,
                geometry.height,
                "bottom");
    add_surface(Eigen::Vector3d(x0, geometry.center.y(), geometry.center.z()),
                Eigen::Vector3d(-1.0, 0.0, 0.0),
                geometry.height,
                geometry.depth,
                "left");
    add_surface(Eigen::Vector3d(x1, geometry.center.y(), geometry.center.z()),
                Eigen::Vector3d(1.0, 0.0, 0.0),
                geometry.height,
                geometry.depth,
                "right");
    add_surface(Eigen::Vector3d(geometry.center.x(), y0, geometry.center.z()),
                Eigen::Vector3d(0.0, -1.0, 0.0),
                geometry.width,
                geometry.depth,
                "front");
    add_surface(Eigen::Vector3d(geometry.center.x(), y1, geometry.center.z()),
                Eigen::Vector3d(0.0, 1.0, 0.0),
                geometry.width,
                geometry.depth,
                "back");

    geometry.corners = {
        Eigen::Vector3d(x0, y0, z0), Eigen::Vector3d(x1, y0, z0),
        Eigen::Vector3d(x0, y1, z0), Eigen::Vector3d(x1, y1, z0),
        Eigen::Vector3d(x0, y0, z1), Eigen::Vector3d(x1, y0, z1),
        Eigen::Vector3d(x0, y1, z1), Eigen::Vector3d(x1, y1, z1),
    };

    auto add_edge = [&](int start_idx, int end_idx, const std::string& label) {
        BoxGeometry::EdgeFeature edge;
        edge.start = geometry.corners[start_idx];
        edge.end = geometry.corners[end_idx];
        edge.midpoint = 0.5 * (edge.start + edge.end);
        const Eigen::Vector3d delta = edge.end - edge.start;
        edge.length = delta.norm();
        edge.direction = Eigen::Vector3d::Zero();
        if (edge.length > kGeometryEpsilon) {
            edge.direction = delta / edge.length;
        }
        edge.label = label;
        geometry.edges.push_back(edge);
    };

    add_edge(0, 1, "bottom_front");
    add_edge(2, 3, "bottom_back");
    add_edge(0, 2, "bottom_left");
    add_edge(1, 3, "bottom_right");
    add_edge(4, 5, "top_front");
    add_edge(6, 7, "top_back");
    add_edge(4, 6, "top_left");
    add_edge(5, 7, "top_right");
    add_edge(0, 4, "front_left_vertical");
    add_edge(1, 5, "front_right_vertical");
    add_edge(2, 6, "back_left_vertical");
    add_edge(3, 7, "back_right_vertical");

    const double span_xy = std::min(geometry.width, geometry.height);
    const double min_opening_size = std::max(span_xy * 0.04, 0.006);
    const double depth_threshold = std::max(geometry.depth * 0.05, 0.01);

    const int grid_nx = std::clamp(
        static_cast<int>(std::ceil(geometry.width / std::max(0.0025, span_xy / 120.0))),
        60,
        180);
    const int grid_ny = std::clamp(
        static_cast<int>(std::ceil(geometry.height / std::max(0.0025, span_xy / 120.0))),
        60,
        180);
    const double cell_dx = geometry.width / static_cast<double>(grid_nx);
    const double cell_dy = geometry.height / static_cast<double>(grid_ny);
    const double bin_size = std::max({cell_dx * 4.0, cell_dy * 4.0, 0.01});

    std::unordered_map<GridKey, std::vector<int>, GridKeyHash> triangle_bins;
    triangle_bins.reserve(mesh_.size() / 256 + 16);

    for (int tri_idx = 0; tri_idx < static_cast<int>(mesh_.size()); ++tri_idx) {
        const auto& tri = mesh_[tri_idx];
        const double tri_min_x = std::min({tri.v0.x(), tri.v1.x(), tri.v2.x()});
        const double tri_max_x = std::max({tri.v0.x(), tri.v1.x(), tri.v2.x()});
        const double tri_min_y = std::min({tri.v0.y(), tri.v1.y(), tri.v2.y()});
        const double tri_max_y = std::max({tri.v0.y(), tri.v1.y(), tri.v2.y()});

        const int ix0 = static_cast<int>(std::floor((tri_min_x - x0) / bin_size));
        const int ix1 = static_cast<int>(std::floor((tri_max_x - x0) / bin_size));
        const int iy0 = static_cast<int>(std::floor((tri_min_y - y0) / bin_size));
        const int iy1 = static_cast<int>(std::floor((tri_max_y - y0) / bin_size));

        for (int ix = ix0; ix <= ix1; ++ix) {
            for (int iy = iy0; iy <= iy1; ++iy) {
                triangle_bins[{ix, iy}].push_back(tri_idx);
            }
        }
    }

    auto vertical_ray_hit = [&](double x, double y, const Triangle& tri, double& hit_z) -> bool {
        const double x1_tri = tri.v0.x();
        const double y1_tri = tri.v0.y();
        const double x2_tri = tri.v1.x();
        const double y2_tri = tri.v1.y();
        const double x3_tri = tri.v2.x();
        const double y3_tri = tri.v2.y();

        const double det = (y2_tri - y3_tri) * (x1_tri - x3_tri) +
                           (x3_tri - x2_tri) * (y1_tri - y3_tri);
        if (std::abs(det) < kGeometryEpsilon) {
            return false;
        }

        const double lambda1 = ((y2_tri - y3_tri) * (x - x3_tri) +
                                (x3_tri - x2_tri) * (y - y3_tri)) / det;
        const double lambda2 = ((y3_tri - y1_tri) * (x - x3_tri) +
                                (x1_tri - x3_tri) * (y - y3_tri)) / det;
        const double lambda3 = 1.0 - lambda1 - lambda2;

        if (lambda1 < -kGeometryEpsilon || lambda2 < -kGeometryEpsilon || lambda3 < -kGeometryEpsilon) {
            return false;
        }

        hit_z = lambda1 * tri.v0.z() + lambda2 * tri.v1.z() + lambda3 * tri.v2.z();
        return true;
    };

    const auto flatten = [grid_ny](int ix, int iy) {
        return ix * grid_ny + iy;
    };

    std::vector<double> heights(static_cast<std::size_t>(grid_nx * grid_ny),
                                std::numeric_limits<double>::quiet_NaN());
    std::vector<bool> occupied(static_cast<std::size_t>(grid_nx * grid_ny), false);
    std::vector<double> rim_heights(static_cast<std::size_t>(grid_nx * grid_ny), z1);
    std::vector<int> visit_stamp(mesh_.size(), 0);
    int current_stamp = 1;

    for (int ix = 0; ix < grid_nx; ++ix) {
        const double sample_x = x0 + (static_cast<double>(ix) + 0.5) * cell_dx;
        const int bin_x = static_cast<int>(std::floor((sample_x - x0) / bin_size));

        for (int iy = 0; iy < grid_ny; ++iy) {
            const double sample_y = y0 + (static_cast<double>(iy) + 0.5) * cell_dy;
            const int bin_y = static_cast<int>(std::floor((sample_y - y0) / bin_size));

            double best_hit_z = -std::numeric_limits<double>::infinity();
            bool found_hit = false;
            ++current_stamp;
            if (current_stamp == std::numeric_limits<int>::max()) {
                std::fill(visit_stamp.begin(), visit_stamp.end(), 0);
                current_stamp = 1;
            }

            for (int dx = -1; dx <= 1; ++dx) {
                for (int dy = -1; dy <= 1; ++dy) {
                    const auto it = triangle_bins.find({bin_x + dx, bin_y + dy});
                    if (it == triangle_bins.end()) {
                        continue;
                    }

                    for (const int tri_idx : it->second) {
                        if (visit_stamp[tri_idx] == current_stamp) {
                            continue;
                        }
                        visit_stamp[tri_idx] = current_stamp;

                        double hit_z = 0.0;
                        if (!vertical_ray_hit(sample_x, sample_y, mesh_[tri_idx], hit_z)) {
                            continue;
                        }

                        if (!found_hit || hit_z > best_hit_z) {
                            best_hit_z = hit_z;
                            found_hit = true;
                        }
                    }
                }
            }

            if (found_hit) {
                const int flat_idx = flatten(ix, iy);
                heights[flat_idx] = best_hit_z;
                occupied[flat_idx] = true;
            }
        }
    }

    const int window_radius = std::clamp(
        static_cast<int>(std::round(0.015 / std::max(cell_dx, cell_dy))),
        2,
        5);
    std::vector<bool> depressed(static_cast<std::size_t>(grid_nx * grid_ny), false);

    for (int ix = 0; ix < grid_nx; ++ix) {
        for (int iy = 0; iy < grid_ny; ++iy) {
            const int flat_idx = flatten(ix, iy);
            if (!occupied[flat_idx]) {
                continue;
            }

            double local_rim = heights[flat_idx];
            for (int nx_idx = std::max(0, ix - window_radius);
                 nx_idx <= std::min(grid_nx - 1, ix + window_radius);
                 ++nx_idx) {
                for (int ny_idx = std::max(0, iy - window_radius);
                     ny_idx <= std::min(grid_ny - 1, iy + window_radius);
                     ++ny_idx) {
                    const int neighbor_flat_idx = flatten(nx_idx, ny_idx);
                    if (occupied[neighbor_flat_idx]) {
                        local_rim = std::max(local_rim, heights[neighbor_flat_idx]);
                    }
                }
            }

            rim_heights[flat_idx] = local_rim;
            depressed[flat_idx] = (local_rim - heights[flat_idx]) > depth_threshold;
        }
    }

    std::vector<int> component_id(static_cast<std::size_t>(grid_nx * grid_ny), -1);
    std::vector<std::vector<int>> components;
    components.reserve(64);

    for (int ix = 0; ix < grid_nx; ++ix) {
        for (int iy = 0; iy < grid_ny; ++iy) {
            const int seed_idx = flatten(ix, iy);
            if (!depressed[seed_idx] || component_id[seed_idx] != -1) {
                continue;
            }

            const int current_component = static_cast<int>(components.size());
            components.emplace_back();
            std::queue<int> queue;
            queue.push(seed_idx);
            component_id[seed_idx] = current_component;

            while (!queue.empty()) {
                const int flat_idx = queue.front();
                queue.pop();
                components.back().push_back(flat_idx);

                const int cx_idx = flat_idx / grid_ny;
                const int cy_idx = flat_idx % grid_ny;
                const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};

                for (const auto& offset : offsets) {
                    const int nx_idx = cx_idx + offset[0];
                    const int ny_idx = cy_idx + offset[1];
                    if (nx_idx < 0 || nx_idx >= grid_nx || ny_idx < 0 || ny_idx >= grid_ny) {
                        continue;
                    }

                    const int neighbor_flat_idx = flatten(nx_idx, ny_idx);
                    if (!depressed[neighbor_flat_idx] || component_id[neighbor_flat_idx] != -1) {
                        continue;
                    }

                    component_id[neighbor_flat_idx] = current_component;
                    queue.push(neighbor_flat_idx);
                }
            }
        }
    }

    struct RegionCandidate {
        Eigen::Vector3d center = Eigen::Vector3d::Zero();
        double rim_z = 0.0;
        double bottom_z = 0.0;
        double min_x = 0.0;
        double max_x = 0.0;
        double min_y = 0.0;
        double max_y = 0.0;
        double span_x = 0.0;
        double span_y = 0.0;
        double area = 0.0;
        double depth = 0.0;
        double circularity = 0.0;
        double aspect_ratio = 0.0;
        bool touches_exterior = false;
    };

    std::vector<RegionCandidate> candidates;
    candidates.reserve(components.size());

    for (std::size_t comp_idx = 0; comp_idx < components.size(); ++comp_idx) {
        const auto& component = components[comp_idx];
        if (component.size() < 6U) {
            continue;
        }

        double sum_x = 0.0;
        double sum_y = 0.0;
        double sum_z = 0.0;
        int min_ix = grid_nx - 1;
        int max_ix = 0;
        int min_iy = grid_ny - 1;
        int max_iy = 0;
        bool touches_exterior = false;
        double perimeter = 0.0;
        std::vector<double> border_heights;
        border_heights.reserve(component.size() * 2);

        for (const int flat_idx : component) {
            const int ix = flat_idx / grid_ny;
            const int iy = flat_idx % grid_ny;
            const double sample_x = x0 + (static_cast<double>(ix) + 0.5) * cell_dx;
            const double sample_y = y0 + (static_cast<double>(iy) + 0.5) * cell_dy;

            sum_x += sample_x;
            sum_y += sample_y;
            sum_z += heights[flat_idx];
            min_ix = std::min(min_ix, ix);
            max_ix = std::max(max_ix, ix);
            min_iy = std::min(min_iy, iy);
            max_iy = std::max(max_iy, iy);

            const int offsets[4][2] = {{1, 0}, {-1, 0}, {0, 1}, {0, -1}};
            for (const auto& offset : offsets) {
                const int nx_idx = ix + offset[0];
                const int ny_idx = iy + offset[1];
                const double boundary_length = (offset[0] != 0) ? cell_dy : cell_dx;

                if (nx_idx < 0 || nx_idx >= grid_nx || ny_idx < 0 || ny_idx >= grid_ny) {
                    touches_exterior = true;
                    perimeter += boundary_length;
                    continue;
                }

                const int neighbor_flat_idx = flatten(nx_idx, ny_idx);
                if (!occupied[neighbor_flat_idx]) {
                    touches_exterior = true;
                    perimeter += boundary_length;
                    continue;
                }

                if (component_id[neighbor_flat_idx] != static_cast<int>(comp_idx)) {
                    border_heights.push_back(heights[neighbor_flat_idx]);
                    perimeter += boundary_length;
                }
            }
        }

        const double area = static_cast<double>(component.size()) * cell_dx * cell_dy;
        const double mean_z = sum_z / static_cast<double>(component.size());
        const double rim_z = border_heights.empty()
            ? quantile({z1, mean_z}, 1.0)
            : quantile(border_heights, 0.80);
        const double depth = rim_z - mean_z;
        if (depth < depth_threshold) {
            continue;
        }

        RegionCandidate candidate;
        candidate.center = Eigen::Vector3d(
            sum_x / static_cast<double>(component.size()),
            sum_y / static_cast<double>(component.size()),
            rim_z);
        candidate.rim_z = rim_z;
        candidate.bottom_z = mean_z;
        candidate.min_x = x0 + static_cast<double>(min_ix) * cell_dx;
        candidate.max_x = x0 + static_cast<double>(max_ix + 1) * cell_dx;
        candidate.min_y = y0 + static_cast<double>(min_iy) * cell_dy;
        candidate.max_y = y0 + static_cast<double>(max_iy + 1) * cell_dy;
        candidate.span_x = static_cast<double>(max_ix - min_ix + 1) * cell_dx;
        candidate.span_y = static_cast<double>(max_iy - min_iy + 1) * cell_dy;
        candidate.area = area;
        candidate.depth = depth;
        candidate.aspect_ratio = std::max(candidate.span_x, candidate.span_y) /
                                 std::max(std::min(candidate.span_x, candidate.span_y), kGeometryEpsilon);
        candidate.circularity = perimeter > kGeometryEpsilon
            ? (4.0 * M_PI * area) / (perimeter * perimeter)
            : 0.0;
        candidate.touches_exterior = touches_exterior;
        candidates.push_back(candidate);
    }

    std::vector<RegionCandidate> hole_candidates;
    std::vector<RegionCandidate> cavity_candidates;
    hole_candidates.reserve(candidates.size());
    cavity_candidates.reserve(candidates.size());

    for (const auto& candidate : candidates) {
        const double min_span = std::min(candidate.span_x, candidate.span_y);
        const double max_span = std::max(candidate.span_x, candidate.span_y);
        if (min_span < min_opening_size || max_span < min_opening_size) {
            continue;
        }

        if (candidate.touches_exterior && candidate.depth < depth_threshold * 2.0) {
            continue;
        }

        const bool is_hole =
            !candidate.touches_exterior &&
            candidate.circularity > 0.45 &&
            candidate.aspect_ratio < 1.6 &&
            max_span < span_xy * 0.55;

        if (is_hole) {
            hole_candidates.push_back(candidate);
        } else {
            cavity_candidates.push_back(candidate);
        }
    }

    std::sort(hole_candidates.begin(), hole_candidates.end(),
              [](const RegionCandidate& lhs, const RegionCandidate& rhs) {
                  const double lhs_radius = 0.5 * std::min(lhs.span_x, lhs.span_y);
                  const double rhs_radius = 0.5 * std::min(rhs.span_x, rhs.span_y);
                  if (std::abs(lhs_radius - rhs_radius) > 1e-6) {
                      return lhs_radius > rhs_radius;
                  }
                  return lhs.depth > rhs.depth;
              });

    std::sort(cavity_candidates.begin(), cavity_candidates.end(),
              [](const RegionCandidate& lhs, const RegionCandidate& rhs) {
                  const double lhs_score = lhs.depth * lhs.area;
                  const double rhs_score = rhs.depth * rhs.area;
                  if (std::abs(lhs_score - rhs_score) > 1e-9) {
                      return lhs_score > rhs_score;
                  }
                  return lhs.depth > rhs.depth;
              });

    const auto overlaps_with_gap = [](const RegionCandidate& lhs,
                                      const RegionCandidate& rhs,
                                      double gap) {
        const bool x_overlaps = lhs.min_x <= rhs.max_x + gap && rhs.min_x <= lhs.max_x + gap;
        const bool y_overlaps = lhs.min_y <= rhs.max_y + gap && rhs.min_y <= lhs.max_y + gap;
        return x_overlaps && y_overlaps;
    };

    const auto merge_candidate = [](RegionCandidate& base, const RegionCandidate& other) {
        const double total_area = base.area + other.area;
        if (total_area > kGeometryEpsilon) {
            const Eigen::Vector2d merged_xy =
                (base.center.head<2>() * base.area + other.center.head<2>() * other.area) / total_area;
            base.center.x() = merged_xy.x();
            base.center.y() = merged_xy.y();
        }
        base.area = total_area;
        base.rim_z = std::max(base.rim_z, other.rim_z);
        base.bottom_z = std::min(base.bottom_z, other.bottom_z);
        base.min_x = std::min(base.min_x, other.min_x);
        base.max_x = std::max(base.max_x, other.max_x);
        base.min_y = std::min(base.min_y, other.min_y);
        base.max_y = std::max(base.max_y, other.max_y);
        base.span_x = base.max_x - base.min_x;
        base.span_y = base.max_y - base.min_y;
        base.depth = base.rim_z - base.bottom_z;
        base.aspect_ratio = std::max(base.span_x, base.span_y) /
                            std::max(std::min(base.span_x, base.span_y), kGeometryEpsilon);
        base.center.z() = base.rim_z;
        base.touches_exterior = base.touches_exterior || other.touches_exterior;
    };

    const double cavity_merge_gap = std::max({cell_dx * 2.0, cell_dy * 2.0, 0.008});
    const double rim_similarity_threshold = std::max(depth_threshold, 0.014);
    const double bottom_similarity_threshold = std::max(depth_threshold * 1.5, 0.018);
    std::vector<RegionCandidate> merged_cavity_candidates;
    merged_cavity_candidates.reserve(cavity_candidates.size());

    for (const auto& candidate : cavity_candidates) {
        bool merged = false;
        for (auto& existing : merged_cavity_candidates) {
            if (!overlaps_with_gap(existing, candidate, cavity_merge_gap)) {
                continue;
            }
            if (std::abs(existing.rim_z - candidate.rim_z) > rim_similarity_threshold) {
                continue;
            }
            if (std::abs(existing.bottom_z - candidate.bottom_z) > bottom_similarity_threshold) {
                continue;
            }

            merge_candidate(existing, candidate);
            merged = true;
            break;
        }

        if (!merged) {
            merged_cavity_candidates.push_back(candidate);
        }
    }

    std::vector<RegionCandidate> filtered_cavity_candidates;
    filtered_cavity_candidates.reserve(merged_cavity_candidates.size());
    const double min_cavity_volume_proxy =
        std::max(span_xy * span_xy * depth_threshold * 0.08, 4.0e-5);
    const double shallow_depth_threshold = std::max(geometry.depth * 0.08, 0.018);
    const double shallow_width_threshold = std::max(span_xy * 0.30, 0.06);
    const double broad_cavity_width_threshold = std::max(span_xy * 0.75, 0.12);
    const double broad_cavity_depth_threshold = std::max(geometry.depth * 0.26, 0.065);

    for (const auto& candidate : merged_cavity_candidates) {
        const double volume_proxy = candidate.area * candidate.depth;
        const bool too_small = volume_proxy < min_cavity_volume_proxy;
        const bool too_shallow_and_small =
            candidate.depth < shallow_depth_threshold &&
            std::max(candidate.span_x, candidate.span_y) < shallow_width_threshold;
        const bool too_broad_and_shallow =
            std::max(candidate.span_x, candidate.span_y) > broad_cavity_width_threshold &&
            candidate.depth < broad_cavity_depth_threshold;
        if (too_small || too_shallow_and_small || too_broad_and_shallow) {
            continue;
        }
        filtered_cavity_candidates.push_back(candidate);
    }

    std::sort(filtered_cavity_candidates.begin(), filtered_cavity_candidates.end(),
              [](const RegionCandidate& lhs, const RegionCandidate& rhs) {
                  const double lhs_score = lhs.depth * lhs.area;
                  const double rhs_score = rhs.depth * rhs.area;
                  if (std::abs(lhs_score - rhs_score) > 1e-9) {
                      return lhs_score > rhs_score;
                  }
                  return lhs.depth > rhs.depth;
              });

    if (filtered_cavity_candidates.size() > 6U) {
        filtered_cavity_candidates.resize(6U);
    }

    for (const auto& candidate : hole_candidates) {
        BoxGeometry::Hole hole;
        hole.center = candidate.center;
        hole.axis = Eigen::Vector3d(0.0, 0.0, -1.0);
        hole.radius = std::max(std::sqrt(candidate.area / M_PI),
                               0.5 * std::min(candidate.span_x, candidate.span_y));
        hole.depth = candidate.depth;
        geometry.holes.push_back(hole);
    }

    for (const auto& candidate : filtered_cavity_candidates) {
        BoxGeometry::Cavity cavity;
        cavity.entrance_center = candidate.center;
        cavity.bottom_center = Eigen::Vector3d(
            candidate.center.x(),
            candidate.center.y(),
            candidate.bottom_z);
        cavity.entrance_width = std::max(candidate.span_x, candidate.span_y);
        cavity.depth = candidate.depth;
        cavity.narrowness = std::clamp(
            candidate.depth / std::max(candidate.depth + std::min(candidate.span_x, candidate.span_y),
                                       kGeometryEpsilon),
            0.0,
            1.0);
        geometry.cavities.push_back(cavity);
    }

    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    if (my_cr5_control::scene::isHeroOffsetThroatProfile(profile)) {
        addHeroOffsetThroatFeature(geometry);
    }

    if (geometry.holes.empty() && geometry.cavities.empty()) {
        RCLCPP_WARN(rclcpp::get_logger("MeasurementPointGenerator"),
                    "未检测到明显开口，回退到中心启发式特征");

        BoxGeometry::Hole fallback_hole;
        fallback_hole.center = Eigen::Vector3d(geometry.center.x(), geometry.center.y(), z1);
        fallback_hole.axis = Eigen::Vector3d(0.0, 0.0, -1.0);
        fallback_hole.radius = span_xy * 0.15;
        fallback_hole.depth = geometry.depth * 0.30;
        geometry.holes.push_back(fallback_hole);

        BoxGeometry::Cavity fallback_cavity;
        fallback_cavity.entrance_center = Eigen::Vector3d(geometry.center.x(), geometry.center.y(), z1);
        fallback_cavity.bottom_center = Eigen::Vector3d(
            geometry.center.x(),
            geometry.center.y(),
            std::max(z0, z1 - geometry.depth * 0.45));
        fallback_cavity.entrance_width = span_xy * 0.32;
        fallback_cavity.depth =
            fallback_cavity.entrance_center.z() - fallback_cavity.bottom_center.z();
        fallback_cavity.narrowness = 0.7;
        geometry.cavities.push_back(fallback_cavity);
    }

    RCLCPP_INFO(rclcpp::get_logger("MeasurementPointGenerator"),
                "提取完成: holes=%zu cavities=%zu surfaces=%zu edges=%zu corners=%zu",
                geometry.holes.size(),
                geometry.cavities.size(),
                geometry.surfaces.size(),
                geometry.edges.size(),
                geometry.corners.size());

    return geometry;
}

std::vector<MeasurementPoint> MeasurementPointGenerator::generatePoints(
    const BoxGeometry& geometry,
    int num_points_per_feature) {

    std::vector<MeasurementPoint> points;

    std::vector<BoxGeometry::Hole> wide_holes = geometry.holes;
    std::sort(wide_holes.begin(), wide_holes.end(),
              [](const BoxGeometry::Hole& lhs, const BoxGeometry::Hole& rhs) {
                  if (std::abs(lhs.radius - rhs.radius) > 1e-6) {
                      return lhs.radius > rhs.radius;
                  }
                  return lhs.depth > rhs.depth;
              });

    std::vector<BoxGeometry::Hole> deep_holes = geometry.holes;
    std::sort(deep_holes.begin(), deep_holes.end(),
              [](const BoxGeometry::Hole& lhs, const BoxGeometry::Hole& rhs) {
                  if (std::abs(lhs.depth - rhs.depth) > 1e-6) {
                      return lhs.depth > rhs.depth;
                  }
                  return lhs.radius > rhs.radius;
              });

    std::vector<BoxGeometry::Cavity> narrow_cavities = geometry.cavities;
    std::sort(narrow_cavities.begin(), narrow_cavities.end(),
              [](const BoxGeometry::Cavity& lhs, const BoxGeometry::Cavity& rhs) {
                  if (std::abs(lhs.narrowness - rhs.narrowness) > 1e-6) {
                      return lhs.narrowness > rhs.narrowness;
                  }
                  return lhs.depth > rhs.depth;
              });

    for (const auto& hole : wide_holes) {
        MeasurementPoint p;
        p.position.x = hole.center.x();
        p.position.y = hole.center.y();
        p.position.z = hole.center.z() + 0.03;
        p.normal.x = 0.0;
        p.normal.y = 0.0;
        p.normal.z = -1.0;
        p.type = PointType::HOLE_CENTER;
        p.difficulty_score = 0.3;
        p.description = "真实开口中心上方点（简单）";
        points.push_back(p);
    }

    for (const auto& hole : wide_holes) {
        for (int i = 0; i < num_points_per_feature; ++i) {
            const double angle = 2.0 * M_PI * static_cast<double>(i) /
                                 static_cast<double>(std::max(1, num_points_per_feature));
            MeasurementPoint p;
            p.position.x = hole.center.x() + hole.radius * 0.72 * std::cos(angle);
            p.position.y = hole.center.y() + hole.radius * 0.72 * std::sin(angle);
            p.position.z = hole.center.z() + 0.015;
            p.normal.x = 0.0;
            p.normal.y = 0.0;
            p.normal.z = -1.0;
            p.type = PointType::HOLE_EDGE;
            p.difficulty_score = 0.55;
            p.description = "真实开口边缘偏心点（中等）";
            points.push_back(p);
        }
    }

    for (const auto& hole : deep_holes) {
        for (int i = 0; i < num_points_per_feature; ++i) {
            const double depth_ratio = std::clamp(0.20 + 0.10 * static_cast<double>(i), 0.20, 0.45);
            MeasurementPoint p;
            p.position.x = hole.center.x();
            p.position.y = hole.center.y();
            p.position.z = hole.center.z() - hole.depth * depth_ratio;
            p.normal.x = 0.0;
            p.normal.y = 0.0;
            p.normal.z = -1.0;
            p.type = PointType::INTERIOR_DEEP;
            p.difficulty_score = std::clamp(0.75 + 0.20 * depth_ratio, 0.75, 0.90);
            p.description = "真实开口内部深点（困难）";
            points.push_back(p);
        }
    }

    for (const auto& cavity : narrow_cavities) {
        for (int i = 0; i < num_points_per_feature; ++i) {
            const double lateral_ratio = (i % 2 == 0 ? 1.0 : -1.0);
            const double depth_ratio = std::clamp(0.22 + 0.08 * static_cast<double>(i), 0.22, 0.46);
            MeasurementPoint p;
            p.position.x = cavity.entrance_center.x() + cavity.entrance_width * 0.20 * lateral_ratio;
            p.position.y = cavity.entrance_center.y();
            p.position.z = cavity.entrance_center.z() - cavity.depth * depth_ratio;
            p.normal.x = 0.0;
            p.normal.y = 0.0;
            p.normal.z = -1.0;
            p.type = PointType::NARROW_PASSAGE;
            p.difficulty_score = std::clamp(0.88 + 0.10 * cavity.narrowness, 0.88, 0.98);
            p.description = "真实内腔狭窄通道点（极端困难）";
            points.push_back(p);
        }
    }

    if (points.empty() && !geometry.surfaces.empty()) {
        const auto& top_surface = geometry.surfaces.front();
        MeasurementPoint p;
        p.position.x = top_surface.center.x();
        p.position.y = top_surface.center.y();
        p.position.z = top_surface.center.z() + 0.03;
        p.normal.x = top_surface.normal.x();
        p.normal.y = top_surface.normal.y();
        p.normal.z = top_surface.normal.z();
        p.type = PointType::SURFACE;
        p.difficulty_score = 0.2;
        p.description = "顶面中心回退点";
        points.push_back(p);
    }

    RCLCPP_INFO(rclcpp::get_logger("MeasurementPointGenerator"),
                "生成了 %d 个测点", static_cast<int>(points.size()));

    return points;
}

std::vector<MeasurementPointGenerator::TestScenario>
MeasurementPointGenerator::generateTestScenarios() {
    std::vector<TestScenario> scenarios;

    const auto profile = my_cr5_control::scene::getV2MeshProfile();
    const std::string stl_path = my_cr5_control::scene::getV2StlPath(profile);
    if (!loadSTLModel(stl_path)) {
        RCLCPP_ERROR(rclcpp::get_logger("MeasurementPointGenerator"),
                     "无法加载STL模型，使用默认几何");
    }

    const BoxGeometry geometry = extractBoxFeatures();
    const auto all_points = generatePoints(geometry, 3);

    auto find_first = [&](const std::vector<PointType>& types,
                          bool prefer_deepest_point) -> const MeasurementPoint* {
        for (const auto type : types) {
            const MeasurementPoint* best_point = nullptr;
            for (const auto& point : all_points) {
                if (point.type == type) {
                    if (!prefer_deepest_point) {
                        return &point;
                    }
                    if (best_point == nullptr || point.position.z < best_point->position.z) {
                        best_point = &point;
                    }
                }
            }
            if (best_point != nullptr) {
                return best_point;
            }
        }
        return nullptr;
    };

    for (const auto& spec : my_cr5_control::paper_mainline::canonicalV2ScenarioSpecs()) {
        if (spec.hero_only && !my_cr5_control::scene::isHeroOffsetThroatProfile(profile)) {
            continue;
        }
        const auto* point = find_first(spec.point_type_priority, spec.prefer_deepest_point);
        if (point == nullptr) {
            continue;
        }

        TestScenario scenario;
        scenario.name = spec.name;
        scenario.difficulty = spec.difficulty;
        scenario.points = {*point};
        scenarios.push_back(scenario);
    }

    return scenarios;
}

double MeasurementPointGenerator::calculateDifficultyScore(
    const MeasurementPoint& point,
    const BoxGeometry& geometry) {

    double score = 0.0;

    if (geometry.depth > kGeometryEpsilon) {
        const double depth_ratio = (geometry.max_corner.z() - point.position.z) / geometry.depth;
        score += std::clamp(depth_ratio, 0.0, 1.0) * 0.4;
    }

    const double narrowness = computeNarrowness(
        Eigen::Vector3d(point.position.x, point.position.y, point.position.z),
        geometry
    );
    score += narrowness * 0.4;

    Eigen::Vector3d normal(point.normal.x, point.normal.y, point.normal.z);
    if (normal.norm() > kGeometryEpsilon) {
        normal.normalize();
        const Eigen::Vector3d vertical(0.0, 0.0, 1.0);
        const double alignment = std::clamp(std::abs(normal.dot(vertical)), 0.0, 1.0);
        const double angle = std::acos(alignment);
        score += (angle / M_PI) * 0.2;
    }

    return std::min(1.0, score);
}

double MeasurementPointGenerator::computeNarrowness(
    const Eigen::Vector3d& point,
    const BoxGeometry& geometry) {

    double min_dist = std::numeric_limits<double>::max();
    min_dist = std::min(min_dist, point.x() - geometry.min_corner.x());
    min_dist = std::min(min_dist, geometry.max_corner.x() - point.x());
    min_dist = std::min(min_dist, point.y() - geometry.min_corner.y());
    min_dist = std::min(min_dist, geometry.max_corner.y() - point.y());

    for (const auto& hole : geometry.holes) {
        if (point.z() > hole.center.z() || point.z() < hole.center.z() - hole.depth) {
            continue;
        }

        const double radial_distance =
            (point.head<2>() - hole.center.head<2>()).norm();
        const double hole_clearance = std::max(hole.radius - radial_distance, 0.0);
        const double normalized = 1.0 - std::min(1.0, hole_clearance / std::max(hole.radius, kGeometryEpsilon));
        min_dist = std::min(min_dist, hole_clearance);
        min_dist = std::min(min_dist, hole.radius * (1.0 - normalized));
    }

    for (const auto& cavity : geometry.cavities) {
        if (point.z() > cavity.entrance_center.z() || point.z() < cavity.bottom_center.z()) {
            continue;
        }

        const double half_width = 0.5 * cavity.entrance_width;
        const double lateral_distance = std::abs(point.x() - cavity.entrance_center.x());
        min_dist = std::min(min_dist, std::max(half_width - lateral_distance, 0.0));
    }

    const double max_dimension = std::max({geometry.width, geometry.height, geometry.depth});
    return 1.0 - std::min(1.0, min_dist / std::max(max_dimension * 0.2, kGeometryEpsilon));
}

Eigen::Vector3d MeasurementPointGenerator::computeNormal(
    const Eigen::Vector3d& v0,
    const Eigen::Vector3d& v1,
    const Eigen::Vector3d& v2) {

    Eigen::Vector3d edge1 = v1 - v0;
    Eigen::Vector3d edge2 = v2 - v0;
    return edge1.cross(edge2).normalized();
}

bool MeasurementPointGenerator::isPointInsideBox(
    const Eigen::Vector3d& point,
    const BoxGeometry& geometry) {

    return point.x() >= geometry.min_corner.x() &&
           point.x() <= geometry.max_corner.x() &&
           point.y() >= geometry.min_corner.y() &&
           point.y() <= geometry.max_corner.y() &&
           point.z() >= geometry.min_corner.z() &&
           point.z() <= geometry.max_corner.z();
}

}  // namespace measurement
