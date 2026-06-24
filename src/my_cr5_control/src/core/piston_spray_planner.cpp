#include "my_cr5_control/piston_spray_planner.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <string>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>

namespace my_cr5_control::piston {
namespace {

constexpr double kMmToM = 0.001;
constexpr double kParallelThreshold = 1e-6;
constexpr double kPi = 3.14159265358979323846;

tf2::Vector3 toTf(const geometry_msgs::msg::Vector3& v) {
    return tf2::Vector3(v.x, v.y, v.z);
}

tf2::Vector3 toTf(const geometry_msgs::msg::Point& p) {
    return tf2::Vector3(p.x, p.y, p.z);
}

geometry_msgs::msg::Point toPointMsg(const tf2::Vector3& v) {
    geometry_msgs::msg::Point point;
    point.x = v.x();
    point.y = v.y();
    point.z = v.z();
    return point;
}

bool buildFrameBasis(const WorkpieceFrame& frame,
                     tf2::Vector3* x_axis,
                     tf2::Vector3* y_axis,
                     tf2::Vector3* z_axis,
                     std::string* error_message) {
    if (x_axis == nullptr || y_axis == nullptr || z_axis == nullptr) {
        if (error_message != nullptr) {
            *error_message = "Frame basis output pointers must not be null.";
        }
        return false;
    }

    tf2::Vector3 z = toTf(frame.axial_direction);
    tf2::Vector3 x = toTf(frame.radial_direction);

    if (z.length2() < kParallelThreshold) {
        if (error_message != nullptr) {
            *error_message = "Axial direction must not be a zero vector.";
        }
        return false;
    }
    if (x.length2() < kParallelThreshold) {
        if (error_message != nullptr) {
            *error_message = "Radial direction must not be a zero vector.";
        }
        return false;
    }

    z.normalize();
    x -= z * z.dot(x);
    if (x.length2() < kParallelThreshold) {
        if (error_message != nullptr) {
            *error_message =
                "Radial direction is parallel to the piston axis; choose a non-parallel vector.";
        }
        return false;
    }
    x.normalize();

    tf2::Vector3 y = z.cross(x);
    if (y.length2() < kParallelThreshold) {
        if (error_message != nullptr) {
            *error_message = "Unable to construct a right-handed workpiece frame.";
        }
        return false;
    }
    y.normalize();

    x = y.cross(z);
    x.normalize();

    *x_axis = x;
    *y_axis = y;
    *z_axis = z;
    return true;
}

geometry_msgs::msg::Pose makePose(const geometry_msgs::msg::Point& position,
                                  const geometry_msgs::msg::Quaternion& orientation) {
    geometry_msgs::msg::Pose pose;
    pose.position = position;
    pose.orientation = orientation;
    return pose;
}

void appendHighLevelNotices(const PlanMetrics& metrics, std::vector<std::string>* notices) {
    if (notices == nullptr) {
        return;
    }

    if (metrics.overlap_ratio > 0.75) {
        notices->push_back("重合量偏大，轴向进给会较保守。");
    }
    if (metrics.total_revolutions < 1.0) {
        notices->push_back("喷涂长度不足一整圈，建议复核工艺覆盖范围。");
    }
    if (metrics.spray_time_s > 120.0) {
        notices->push_back("单次喷涂时间较长，建议检查供料稳定性。");
    }
    if (metrics.axial_feed_mm_s > 80.0) {
        notices->push_back("轴向进给速度偏高，建议确认滑台动态性能。");
    }
}

void addKeyPose(const std::string& name,
                double local_x_mm,
                double local_y_mm,
                double local_z_mm,
                const WorkpieceFrame& frame,
                const ToolTcpOffsetMm& tool_tcp_offset,
                std::vector<KeyPose>* key_poses) {
    if (key_poses == nullptr) {
        return;
    }

    const geometry_msgs::msg::Point tcp_point =
        mapLocalPointToBase(frame, local_x_mm, local_y_mm, local_z_mm);
    const geometry_msgs::msg::Quaternion orientation = computeSprayOrientation(frame);
    const geometry_msgs::msg::Pose tcp_pose = makePose(tcp_point, orientation);

    KeyPose key_pose;
    key_pose.name = name;
    key_pose.local_x_mm = local_x_mm;
    key_pose.local_y_mm = local_y_mm;
    key_pose.local_z_mm = local_z_mm;
    key_pose.tcp_pose = tcp_pose;
    key_pose.flange_pose = computeFlangePoseFromTcp(tcp_pose, tool_tcp_offset);
    key_poses->push_back(key_pose);
}

}  // namespace

bool validateInputs(const PistonSpecMm& piston,
                    const SprayProcessMm& process,
                    const WorkpieceFrame& frame,
                    std::vector<std::string>* errors) {
    if (errors != nullptr) {
        errors->clear();
    }

    auto push_error = [errors](const std::string& message) {
        if (errors != nullptr) {
            errors->push_back(message);
        }
    };

    if (piston.diameter_mm <= 0.0) {
        push_error("活塞直径必须大于 0 mm。");
    }
    if (piston.spray_length_mm <= 0.0) {
        push_error("喷涂长度必须大于 0 mm。");
    }

    if (process.spray_distance_mm <= 0.0) {
        push_error("喷涂距离必须大于 0 mm。");
    }
    if (process.spray_width_mm <= 0.0) {
        push_error("喷幅必须大于 0 mm。");
    }
    if (process.overlap_mm < 0.0) {
        push_error("重合量必须大于等于 0 mm。");
    }
    if (process.overlap_mm >= process.spray_width_mm) {
        push_error("重合量必须小于喷幅。");
    }
    if (process.turntable_rpm <= 0.0) {
        push_error("转台转速必须大于 0 rpm。");
    }
    if (process.flow_rate_ml_min <= 0.0) {
        push_error("喷涂流量必须大于 0 ml/min。");
    }
    if (process.lead_in_mm < 0.0 || process.lead_out_mm < 0.0) {
        push_error("导入/导出长度必须大于等于 0 mm。");
    }
    if (process.radial_clearance_mm < 0.0) {
        push_error("安全径向退让必须大于等于 0 mm。");
    }
    if (process.approach_speed_mm_s <= 0.0) {
        push_error("接近速度必须大于 0 mm/s。");
    }
    if (process.sample_count < 2) {
        push_error("采样点数量至少为 2。");
    }

    std::string frame_error;
    tf2::Vector3 x_axis;
    tf2::Vector3 y_axis;
    tf2::Vector3 z_axis;
    if (!buildFrameBasis(frame, &x_axis, &y_axis, &z_axis, &frame_error)) {
        push_error(frame_error);
    }

    return errors == nullptr || errors->empty();
}

geometry_msgs::msg::Point mapLocalPointToBase(const WorkpieceFrame& frame,
                                              double x_mm,
                                              double y_mm,
                                              double z_mm) {
    tf2::Vector3 x_axis;
    tf2::Vector3 y_axis;
    tf2::Vector3 z_axis;
    std::string error_message;
    if (!buildFrameBasis(frame, &x_axis, &y_axis, &z_axis, &error_message)) {
        return frame.origin;
    }

    const tf2::Vector3 origin = toTf(frame.origin);
    const tf2::Vector3 mapped =
        origin +
        x_axis * (x_mm * kMmToM) +
        y_axis * (y_mm * kMmToM) +
        z_axis * (z_mm * kMmToM);
    return toPointMsg(mapped);
}

geometry_msgs::msg::Quaternion computeSprayOrientation(const WorkpieceFrame& frame,
                                                       bool* ok,
                                                       std::string* error_message) {
    tf2::Vector3 x_axis;
    tf2::Vector3 y_axis;
    tf2::Vector3 z_axis;
    const bool basis_ok = buildFrameBasis(frame, &x_axis, &y_axis, &z_axis, error_message);
    if (ok != nullptr) {
        *ok = basis_ok;
    }

    geometry_msgs::msg::Quaternion orientation;
    orientation.w = 1.0;
    if (!basis_ok) {
        return orientation;
    }

    const tf2::Vector3 tool_x = z_axis;
    const tf2::Vector3 tool_z = -x_axis;
    const tf2::Vector3 tool_y = tool_z.cross(tool_x).normalized();

    tf2::Matrix3x3 basis_matrix(
        tool_x.x(), tool_y.x(), tool_z.x(),
        tool_x.y(), tool_y.y(), tool_z.y(),
        tool_x.z(), tool_y.z(), tool_z.z());

    tf2::Quaternion base_quaternion;
    basis_matrix.getRotation(base_quaternion);

    tf2::Quaternion roll_quaternion;
    roll_quaternion.setRotation(tf2::Vector3(0.0, 0.0, 1.0),
                                frame.tool_roll_deg * kPi / 180.0);

    const tf2::Quaternion final_quaternion = base_quaternion * roll_quaternion;
    orientation = tf2::toMsg(final_quaternion.normalized());
    return orientation;
}

geometry_msgs::msg::Pose computeFlangePoseFromTcp(const geometry_msgs::msg::Pose& tcp_pose,
                                                  const ToolTcpOffsetMm& tool_tcp_offset) {
    tf2::Quaternion orientation;
    tf2::fromMsg(tcp_pose.orientation, orientation);

    const tf2::Vector3 offset_mm(
        tool_tcp_offset.x_mm * kMmToM,
        tool_tcp_offset.y_mm * kMmToM,
        tool_tcp_offset.z_mm * kMmToM);
    const tf2::Vector3 world_offset = tf2::quatRotate(orientation, offset_mm);
    const tf2::Vector3 tcp_position = toTf(tcp_pose.position);
    const tf2::Vector3 flange_position = tcp_position - world_offset;

    geometry_msgs::msg::Pose flange_pose = tcp_pose;
    flange_pose.position = toPointMsg(flange_position);
    return flange_pose;
}

geometry_msgs::msg::Pose computeTcpPoseFromFlange(const geometry_msgs::msg::Pose& flange_pose,
                                                  const ToolTcpOffsetMm& tool_tcp_offset) {
    tf2::Quaternion orientation;
    tf2::fromMsg(flange_pose.orientation, orientation);

    const tf2::Vector3 offset_mm(
        tool_tcp_offset.x_mm * kMmToM,
        tool_tcp_offset.y_mm * kMmToM,
        tool_tcp_offset.z_mm * kMmToM);
    const tf2::Vector3 world_offset = tf2::quatRotate(orientation, offset_mm);
    const tf2::Vector3 flange_position = toTf(flange_pose.position);
    const tf2::Vector3 tcp_position = flange_position + world_offset;

    geometry_msgs::msg::Pose tcp_pose = flange_pose;
    tcp_pose.position = toPointMsg(tcp_position);
    return tcp_pose;
}

bool buildSprayPlan(const PistonSpecMm& piston,
                    const SprayProcessMm& process,
                    const WorkpieceFrame& frame,
                    const ToolTcpOffsetMm& tool_tcp_offset,
                    SprayPlan* plan,
                    std::string* error_message) {
    if (plan == nullptr) {
        if (error_message != nullptr) {
            *error_message = "Spray plan output pointer must not be null.";
        }
        return false;
    }

    std::vector<std::string> validation_errors;
    if (!validateInputs(piston, process, frame, &validation_errors)) {
        if (error_message != nullptr) {
            error_message->clear();
            for (std::size_t i = 0; i < validation_errors.size(); ++i) {
                if (i > 0) {
                    *error_message += "\n";
                }
                *error_message += validation_errors[i];
            }
        }
        return false;
    }

    SprayPlan result;
    result.piston = piston;
    result.process = process;
    result.frame = frame;
    result.tool_tcp_offset = tool_tcp_offset;

    result.metrics.piston_radius_mm = 0.5 * piston.diameter_mm;
    result.metrics.nozzle_radius_mm =
        result.metrics.piston_radius_mm + process.spray_distance_mm;
    result.metrics.helical_pitch_mm_per_rev = process.spray_width_mm - process.overlap_mm;
    result.metrics.turntable_rps = process.turntable_rpm / 60.0;
    result.metrics.axial_feed_mm_s =
        result.metrics.helical_pitch_mm_per_rev * result.metrics.turntable_rps;
    result.metrics.surface_speed_mm_s =
        kPi * piston.diameter_mm * result.metrics.turntable_rps;
    result.metrics.total_revolutions =
        piston.spray_length_mm / result.metrics.helical_pitch_mm_per_rev;
    result.metrics.spray_time_s = piston.spray_length_mm / result.metrics.axial_feed_mm_s;
    result.metrics.estimated_graphite_usage_ml =
        process.flow_rate_ml_min * result.metrics.spray_time_s / 60.0;
    result.metrics.overlap_ratio = process.overlap_mm / process.spray_width_mm;

    appendHighLevelNotices(result.metrics, &result.notices);

    addKeyPose("safe_start",
               result.metrics.nozzle_radius_mm + process.radial_clearance_mm,
               0.0,
               -process.lead_in_mm,
               frame,
               tool_tcp_offset,
               &result.key_poses);
    addKeyPose("process_start",
               result.metrics.nozzle_radius_mm,
               0.0,
               -process.lead_in_mm,
               frame,
               tool_tcp_offset,
               &result.key_poses);
    addKeyPose("coat_entry",
               result.metrics.nozzle_radius_mm,
               0.0,
               0.0,
               frame,
               tool_tcp_offset,
               &result.key_poses);
    addKeyPose("coat_exit",
               result.metrics.nozzle_radius_mm,
               0.0,
               piston.spray_length_mm,
               frame,
               tool_tcp_offset,
               &result.key_poses);
    addKeyPose("safe_exit",
               result.metrics.nozzle_radius_mm + process.radial_clearance_mm,
               0.0,
               piston.spray_length_mm + process.lead_out_mm,
               frame,
               tool_tcp_offset,
               &result.key_poses);

    result.execution_steps = {
        {1, "MOVE_NOZZLE", "safe_start", "planned", process.approach_speed_mm_s, 0.0, "移动到安全起点。"},
        {2, "MOVE_NOZZLE", "process_start", "planned", process.approach_speed_mm_s * 0.5, 0.0, "接近目标喷涂距离。"},
        {3, "SET_TURNTABLE_RPM", "", "none", 0.0, process.turntable_rpm, "设置转台转速。"},
        {4, "START_TURNTABLE", "", "none", 0.0, 0.0, "启动转台。"},
        {5, "OPEN_SPRAY", "", "none", 0.0, process.flow_rate_ml_min, "打开喷头并设置流量。"},
        {6, "MOVE_NOZZLE", "coat_entry", "cartesian", result.metrics.axial_feed_mm_s, 0.0, "导入到喷涂起点。"},
        {7, "MOVE_NOZZLE", "coat_exit", "cartesian", result.metrics.axial_feed_mm_s, 0.0, "执行主喷涂行程。"},
        {8, "CLOSE_SPRAY", "", "none", 0.0, 0.0, "关闭喷头。"},
        {9, "STOP_TURNTABLE", "", "none", 0.0, 0.0, "停止转台。"},
        {10, "MOVE_NOZZLE", "safe_exit", "planned", process.approach_speed_mm_s, 0.0, "退回安全终点。"},
    };

    const geometry_msgs::msg::Quaternion orientation = computeSprayOrientation(frame);
    for (int index = 0; index < process.sample_count; ++index) {
        const double progress =
            static_cast<double>(index) / static_cast<double>(process.sample_count - 1);
        const double time_s = result.metrics.spray_time_s * progress;
        const double local_z_mm = piston.spray_length_mm * progress;
        const double surface_angle_deg = result.metrics.total_revolutions * 360.0 * progress;
        const double surface_angle_rad = surface_angle_deg * kPi / 180.0;

        PathSample sample;
        sample.time_s = time_s;
        sample.local_x_mm = result.metrics.nozzle_radius_mm;
        sample.local_y_mm = 0.0;
        sample.local_z_mm = local_z_mm;
        sample.surface_angle_deg = std::fmod(surface_angle_deg, 360.0);
        sample.tcp_pose = makePose(
            mapLocalPointToBase(frame, sample.local_x_mm, sample.local_y_mm, sample.local_z_mm),
            orientation);
        sample.flange_pose = computeFlangePoseFromTcp(sample.tcp_pose, tool_tcp_offset);
        sample.surface_point_base = mapLocalPointToBase(
            frame,
            result.metrics.piston_radius_mm * std::cos(surface_angle_rad),
            result.metrics.piston_radius_mm * std::sin(surface_angle_rad),
            local_z_mm);
        result.path_samples.push_back(sample);
    }

    *plan = result;
    return true;
}

}  // namespace my_cr5_control::piston
