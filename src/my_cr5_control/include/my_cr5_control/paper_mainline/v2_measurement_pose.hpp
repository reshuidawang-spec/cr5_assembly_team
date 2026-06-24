#ifndef MY_CR5_CONTROL_PAPER_MAINLINE_V2_MEASUREMENT_POSE_HPP
#define MY_CR5_CONTROL_PAPER_MAINLINE_V2_MEASUREMENT_POSE_HPP

#include "my_cr5_control/measurement_point_generator.hpp"
#include "my_cr5_control/scene_utils.hpp"

#include <Eigen/Geometry>
#include <cmath>
#include <geometry_msgs/msg/pose.hpp>
#include <geometry_msgs/msg/quaternion.hpp>

namespace my_cr5_control::paper_mainline {

inline geometry_msgs::msg::Pose buildV2MeasurementFlangePose(
    const measurement::MeasurementPoint& point) {
    Eigen::Vector3d normal(point.normal.x, point.normal.y, point.normal.z);
    Eigen::Vector3d z_axis = normal.normalized();

    Eigen::Vector3d lateral_ref(0.0, 1.0, 0.0);
    if (std::abs(z_axis.dot(lateral_ref)) > 0.99) {
        lateral_ref = Eigen::Vector3d(1.0, 0.0, 0.0);
    }

    const Eigen::Vector3d x_axis = lateral_ref.cross(z_axis).normalized();
    const Eigen::Vector3d y_axis = z_axis.cross(x_axis).normalized();

    Eigen::Matrix3d rotation;
    rotation.col(0) = x_axis;
    rotation.col(1) = y_axis;
    rotation.col(2) = z_axis;

    const Eigen::Quaterniond quat(rotation);
    geometry_msgs::msg::Quaternion flange_orientation;
    flange_orientation.x = quat.x();
    flange_orientation.y = quat.y();
    flange_orientation.z = quat.z();
    flange_orientation.w = quat.w();

    return my_cr5_control::tool::buildFlangePoseFromTipPoint(point.position, flange_orientation);
}

}  // namespace my_cr5_control::paper_mainline

#endif  // MY_CR5_CONTROL_PAPER_MAINLINE_V2_MEASUREMENT_POSE_HPP
