#ifndef MY_CR5_CONTROL_SCENE_UTILS_HPP
#define MY_CR5_CONTROL_SCENE_UTILS_HPP

#include "my_cr5_control/probe_params.hpp"
#include "my_cr5_control/env_utils.hpp"

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <Eigen/Dense>
#include <algorithm>
#include <cctype>
#include <filesystem>
#include <string>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace my_cr5_control::scene {

inline constexpr char kWs119MeshResource[] = "package://my_cr5_control/meshes/WS119.STL";
inline constexpr double kWs119MeshScale = 0.00025;
inline constexpr double kWs119DesiredCenterX = 0.45;
inline constexpr double kWs119DesiredCenterY = 0.00;
inline constexpr double kWs119DesiredBottomZ = 0.15;

inline constexpr double kWs119RawMinX = 3.394216299057007;
inline constexpr double kWs119RawMinY = 0.029385089874267578;
inline constexpr double kWs119RawMinZ = 0.0050067901611328125;
inline constexpr double kWs119RawMaxX = 752.5775756835938;
inline constexpr double kWs119RawMaxY = 878.5293579101562;
inline constexpr double kWs119RawMaxZ = 1028.692138671875;

struct V2MeshProfile {
    std::string name;
    std::string mesh_resource;
    std::string mesh_filename;
    double mesh_scale = kWs119MeshScale;
    double desired_center_x = kWs119DesiredCenterX;
    double desired_center_y = kWs119DesiredCenterY;
    double desired_bottom_z = kWs119DesiredBottomZ;
    double raw_min_x = kWs119RawMinX;
    double raw_min_y = kWs119RawMinY;
    double raw_min_z = kWs119RawMinZ;
    double raw_max_x = kWs119RawMaxX;
    double raw_max_y = kWs119RawMaxY;
    double raw_max_z = kWs119RawMaxZ;
};

inline V2MeshProfile makeWs119V2MeshProfile() {
    return {
        "ws119",
        kWs119MeshResource,
        "WS119.STL",
        kWs119MeshScale,
        kWs119DesiredCenterX,
        kWs119DesiredCenterY,
        kWs119DesiredBottomZ,
        kWs119RawMinX,
        kWs119RawMinY,
        kWs119RawMinZ,
        kWs119RawMaxX,
        kWs119RawMaxY,
        kWs119RawMaxZ,
    };
}

inline V2MeshProfile makeHeroOffsetThroatV2MeshProfile() {
    return {
        "hero_offset_throat",
        "package://my_cr5_control/meshes/ws119_v2_hero_offset_throat.stl",
        "ws119_v2_hero_offset_throat.stl",
        kWs119MeshScale,
        0.45,
        0.0,
        0.15,
        0.0,
        0.0,
        0.0,
        880.0,
        880.0,
        760.0,
    };
}

inline std::string normalizeProfileToken(std::string token) {
    std::transform(token.begin(), token.end(), token.begin(), [](unsigned char ch) {
        return static_cast<char>(std::tolower(ch));
    });
    return token;
}

inline V2MeshProfile getV2MeshProfile() {
    const std::string token = normalizeProfileToken(
        my_cr5_control::env::getString("MY_CR5_CONTROL_V2_MESH_PROFILE", "ws119"));
    if (token == "hero" || token == "hero_offset_throat" || token == "offset_throat") {
        return makeHeroOffsetThroatV2MeshProfile();
    }
    return makeWs119V2MeshProfile();
}

inline bool isHeroOffsetThroatProfile(const V2MeshProfile& profile) {
    return profile.name == "hero_offset_throat";
}

inline geometry_msgs::msg::Pose makeV2MeshPose(const V2MeshProfile& profile) {
    const double raw_center_x = 0.5 * (profile.raw_min_x + profile.raw_max_x);
    const double raw_center_y = 0.5 * (profile.raw_min_y + profile.raw_max_y);

    geometry_msgs::msg::Pose pose;
    pose.orientation.w = 1.0;
    pose.position.x = profile.desired_center_x - (raw_center_x * profile.mesh_scale);
    pose.position.y = profile.desired_center_y - (raw_center_y * profile.mesh_scale);
    pose.position.z = profile.desired_bottom_z - (profile.raw_min_z * profile.mesh_scale);
    return pose;
}

inline Eigen::Vector3d transformV2MeshPoint(const V2MeshProfile& profile,
                                            const Eigen::Vector3d& mesh_point) {
    const geometry_msgs::msg::Pose pose = makeV2MeshPose(profile);
    return Eigen::Vector3d(
        pose.position.x + mesh_point.x() * profile.mesh_scale,
        pose.position.y + mesh_point.y() * profile.mesh_scale,
        pose.position.z + mesh_point.z() * profile.mesh_scale);
}

inline Eigen::Vector3d getV2BoundingBoxCenter(const V2MeshProfile& profile) {
    return transformV2MeshPoint(profile, Eigen::Vector3d(
        0.5 * (profile.raw_min_x + profile.raw_max_x),
        0.5 * (profile.raw_min_y + profile.raw_max_y),
        0.5 * (profile.raw_min_z + profile.raw_max_z)));
}

inline Eigen::Vector3d getV2BoundingBoxSize(const V2MeshProfile& profile) {
    return Eigen::Vector3d(
        (profile.raw_max_x - profile.raw_min_x) * profile.mesh_scale,
        (profile.raw_max_y - profile.raw_min_y) * profile.mesh_scale,
        (profile.raw_max_z - profile.raw_min_z) * profile.mesh_scale);
}

inline std::string getV2StlPath(const V2MeshProfile& profile) {
    const auto share_dir = ament_index_cpp::get_package_share_directory("my_cr5_control");
    return (std::filesystem::path(share_dir) / "meshes" / profile.mesh_filename).string();
}

inline geometry_msgs::msg::Pose makeWs119MeshPose() {
    return makeV2MeshPose(makeWs119V2MeshProfile());
}

inline Eigen::Vector3d transformWs119MeshPoint(const Eigen::Vector3d& mesh_point) {
    return transformV2MeshPoint(makeWs119V2MeshProfile(), mesh_point);
}

inline Eigen::Vector3d getWs119BoundingBoxCenter() {
    return getV2BoundingBoxCenter(makeWs119V2MeshProfile());
}

inline Eigen::Vector3d getWs119BoundingBoxSize() {
    return getV2BoundingBoxSize(makeWs119V2MeshProfile());
}

inline std::string getWs119StlPath() {
    return getV2StlPath(makeWs119V2MeshProfile());
}

}  // namespace my_cr5_control::scene

namespace my_cr5_control::tool {

inline geometry_msgs::msg::Pose buildFlangePoseFromTipPoint(
    const geometry_msgs::msg::Point& tip_point,
    const geometry_msgs::msg::Quaternion& flange_orientation) {
    tf2::Quaternion flange_quat;
    tf2::fromMsg(flange_orientation, flange_quat);

    const tf2::Vector3 tip_offset_world =
        tf2::quatRotate(flange_quat, tf2::Vector3(0.0, 0.0, probe::kProbeLength));

    geometry_msgs::msg::Pose flange_pose;
    flange_pose.orientation = flange_orientation;
    flange_pose.position.x = tip_point.x - tip_offset_world.x();
    flange_pose.position.y = tip_point.y - tip_offset_world.y();
    flange_pose.position.z = tip_point.z - tip_offset_world.z();
    return flange_pose;
}

}  // namespace my_cr5_control::tool

#endif  // MY_CR5_CONTROL_SCENE_UTILS_HPP
