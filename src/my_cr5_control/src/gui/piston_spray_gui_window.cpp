#include "my_cr5_control/piston_spray_gui_window.hpp"

#include <QDateTime>
#include <QDir>
#include <QFile>
#include <QFileDialog>
#include <QFileInfo>
#include <QFrame>
#include <QFormLayout>
#include <QGroupBox>
#include <QGridLayout>
#include <QHBoxLayout>
#include <QHeaderView>
#include <QJsonArray>
#include <QJsonDocument>
#include <QJsonObject>
#include <QJsonValue>
#include <QLabel>
#include <QMessageBox>
#include <QPainter>
#include <QPainterPath>
#include <QPushButton>
#include <QScrollArea>
#include <QSplitter>
#include <QStandardPaths>
#include <QTabWidget>
#include <QVBoxLayout>

#include <algorithm>
#include <cmath>
#include <stdexcept>

#include <tf2/LinearMath/Matrix3x3.h>
#include <tf2/LinearMath/Quaternion.h>
#include <tf2/LinearMath/Vector3.h>
#include <tf2_geometry_msgs/tf2_geometry_msgs.hpp>
#include <visualization_msgs/msg/marker.hpp>

namespace {

using my_cr5_control::piston::CR5MotionInterface;
using my_cr5_control::piston::ExecutionEvent;
using my_cr5_control::piston::ExecutionState;
using my_cr5_control::piston::MockMotionInterface;
using my_cr5_control::piston::MockPeripheralInterface;
using my_cr5_control::piston::PistonSpecMm;
using my_cr5_control::piston::SprayProcessMm;
using my_cr5_control::piston::SprayExecutionEngine;
using my_cr5_control::piston::SprayPlan;
using my_cr5_control::piston::WorkpieceFrame;
constexpr double kPi = 3.14159265358979323846;
constexpr double kMmToM = 0.001;
constexpr double kAxisVectorThreshold = 1e-9;
constexpr int kTeachStateVersion = 1;
constexpr const char* kRvizPistonObjectId = "piston_visual_cylinder";
constexpr const char* kWorkpieceMarkerTopic = "piston_spray/workpiece_markers";
constexpr const char* kWorkpieceMarkerFrameId = "base_link";
constexpr const char* kWorkpieceMarkerNamespace = "piston_workpiece";
constexpr int kWorkpieceMarkerCount = 10;

QDoubleSpinBox* createDoubleSpin(double min,
                                 double max,
                                 double value,
                                 int decimals,
                                 double step) {
    auto* spin = new QDoubleSpinBox();
    spin->setRange(min, max);
    spin->setDecimals(decimals);
    spin->setValue(value);
    spin->setSingleStep(step);
    spin->setKeyboardTracking(false);
    return spin;
}

QJsonObject pointToJson(const geometry_msgs::msg::Point& point) {
    QJsonObject object;
    object["x"] = point.x;
    object["y"] = point.y;
    object["z"] = point.z;
    return object;
}

QJsonObject vectorToJson(const geometry_msgs::msg::Vector3& vector) {
    QJsonObject object;
    object["x"] = vector.x;
    object["y"] = vector.y;
    object["z"] = vector.z;
    return object;
}

QJsonObject quaternionToJson(const geometry_msgs::msg::Quaternion& q) {
    QJsonObject object;
    object["x"] = q.x;
    object["y"] = q.y;
    object["z"] = q.z;
    object["w"] = q.w;
    return object;
}

QJsonObject poseToJson(const geometry_msgs::msg::Pose& pose) {
    QJsonObject object;
    object["position_m"] = pointToJson(pose.position);
    object["orientation_xyzw"] = quaternionToJson(pose.orientation);
    return object;
}

bool pointFromJson(const QJsonObject& object, geometry_msgs::msg::Point* point) {
    if (point == nullptr) {
        return false;
    }
    if (!object.contains("x") || !object.contains("y") || !object.contains("z")) {
        return false;
    }
    if (!object.value("x").isDouble() ||
        !object.value("y").isDouble() ||
        !object.value("z").isDouble()) {
        return false;
    }
    point->x = object.value("x").toDouble();
    point->y = object.value("y").toDouble();
    point->z = object.value("z").toDouble();
    return true;
}

bool quaternionFromJson(const QJsonObject& object, geometry_msgs::msg::Quaternion* q) {
    if (q == nullptr) {
        return false;
    }
    if (!object.contains("x") ||
        !object.contains("y") ||
        !object.contains("z") ||
        !object.contains("w")) {
        return false;
    }
    if (!object.value("x").isDouble() ||
        !object.value("y").isDouble() ||
        !object.value("z").isDouble() ||
        !object.value("w").isDouble()) {
        return false;
    }
    q->x = object.value("x").toDouble();
    q->y = object.value("y").toDouble();
    q->z = object.value("z").toDouble();
    q->w = object.value("w").toDouble();
    return true;
}

bool poseFromJson(const QJsonObject& object, geometry_msgs::msg::Pose* pose) {
    if (pose == nullptr) {
        return false;
    }
    if (!object.contains("position_m") || !object.contains("orientation_xyzw")) {
        return false;
    }
    geometry_msgs::msg::Point point;
    geometry_msgs::msg::Quaternion q;
    if (!pointFromJson(object.value("position_m").toObject(), &point) ||
        !quaternionFromJson(object.value("orientation_xyzw").toObject(), &q)) {
        return false;
    }
    pose->position = point;
    pose->orientation = q;
    return true;
}

QJsonObject localPointToJson(double x_mm, double y_mm, double z_mm) {
    QJsonObject object;
    object["x_mm"] = x_mm;
    object["y_mm"] = y_mm;
    object["z_mm"] = z_mm;
    return object;
}

void setSpinFromJson(QDoubleSpinBox* spin,
                     const QJsonObject& object,
                     const QString& key) {
    if (spin == nullptr || !object.contains(key) || !object.value(key).isDouble()) {
        return;
    }
    spin->setValue(object.value(key).toDouble());
}

void setVectorSpinsFromJson(QDoubleSpinBox* x_spin,
                            QDoubleSpinBox* y_spin,
                            QDoubleSpinBox* z_spin,
                            const QJsonObject& object) {
    setSpinFromJson(x_spin, object, "x");
    setSpinFromJson(y_spin, object, "y");
    setSpinFromJson(z_spin, object, "z");
}

QString levelColor(const QString& level) {
    if (level == "SUCCESS") {
        return "#2E7D32";
    }
    if (level == "WARN") {
        return "#EF6C00";
    }
    if (level == "ERROR") {
        return "#C62828";
    }
    return "#1565C0";
}

QString executionStateColor(ExecutionState state) {
    switch (state) {
        case ExecutionState::Ready:
            return "#1565C0";
        case ExecutionState::Running:
            return "#2E7D32";
        case ExecutionState::Paused:
            return "#EF6C00";
        case ExecutionState::Completed:
            return "#00695C";
        case ExecutionState::Stopped:
            return "#546E7A";
        case ExecutionState::EmergencyStopped:
            return "#C62828";
        case ExecutionState::Fault:
            return "#B71C1C";
        case ExecutionState::Idle:
        default:
            return "#455A64";
    }
}

QString executionStateLabel(ExecutionState state) {
    switch (state) {
        case ExecutionState::Idle:
            return "空闲";
        case ExecutionState::Ready:
            return "已就绪";
        case ExecutionState::Running:
            return "执行中";
        case ExecutionState::Paused:
            return "已暂停";
        case ExecutionState::Completed:
            return "已完成";
        case ExecutionState::Stopped:
            return "已停止";
        case ExecutionState::EmergencyStopped:
            return "急停锁定";
        case ExecutionState::Fault:
            return "故障";
    }
    return "未知";
}

void quaternionToEulerDeg(const geometry_msgs::msg::Quaternion& q,
                          double* roll_deg,
                          double* pitch_deg,
                          double* yaw_deg) {
    tf2::Quaternion tf_q;
    tf2::fromMsg(q, tf_q);
    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf2::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
    if (roll_deg != nullptr) {
        *roll_deg = roll * 180.0 / kPi;
    }
    if (pitch_deg != nullptr) {
        *pitch_deg = pitch * 180.0 / kPi;
    }
    if (yaw_deg != nullptr) {
        *yaw_deg = yaw * 180.0 / kPi;
    }
}

tf2::Vector3 toTf(const geometry_msgs::msg::Point& point) {
    return tf2::Vector3(point.x, point.y, point.z);
}

tf2::Vector3 toTf(const geometry_msgs::msg::Vector3& vector) {
    return tf2::Vector3(vector.x, vector.y, vector.z);
}

geometry_msgs::msg::Point toPointMsg(const tf2::Vector3& vector) {
    geometry_msgs::msg::Point point;
    point.x = vector.x();
    point.y = vector.y();
    point.z = vector.z();
    return point;
}

bool buildWorkpieceAxes(const WorkpieceFrame& frame,
                        tf2::Vector3* radial_axis,
                        tf2::Vector3* side_axis,
                        tf2::Vector3* axial_axis,
                        QString* error_message = nullptr) {
    if (radial_axis == nullptr || side_axis == nullptr || axial_axis == nullptr) {
        if (error_message != nullptr) {
            *error_message = "工件坐标系输出指针为空。";
        }
        return false;
    }

    tf2::Vector3 x = toTf(frame.radial_direction);
    tf2::Vector3 z = toTf(frame.axial_direction);
    if (x.length2() < kAxisVectorThreshold) {
        if (error_message != nullptr) {
            *error_message = "工件径向方向不能为零向量。";
        }
        return false;
    }
    if (z.length2() < kAxisVectorThreshold) {
        if (error_message != nullptr) {
            *error_message = "工件轴向方向不能为零向量。";
        }
        return false;
    }

    z.normalize();
    x -= z * z.dot(x);
    if (x.length2() < kAxisVectorThreshold) {
        if (error_message != nullptr) {
            *error_message = "工件径向方向不能与轴向方向平行。";
        }
        return false;
    }
    x.normalize();

    tf2::Vector3 y = z.cross(x);
    if (y.length2() < kAxisVectorThreshold) {
        if (error_message != nullptr) {
            *error_message = "工件坐标系无法构成右手系。";
        }
        return false;
    }
    y.normalize();
    x = y.cross(z).normalized();

    *radial_axis = x;
    *side_axis = y;
    *axial_axis = z;
    return true;
}

geometry_msgs::msg::Quaternion eulerDegToQuaternion(double roll_deg,
                                                    double pitch_deg,
                                                    double yaw_deg) {
    tf2::Quaternion q;
    q.setRPY(roll_deg * kPi / 180.0,
             pitch_deg * kPi / 180.0,
             yaw_deg * kPi / 180.0);
    return tf2::toMsg(q.normalized());
}

geometry_msgs::msg::Pose applyPoseOffset(const geometry_msgs::msg::Pose& base_pose,
                                         double dx_mm,
                                         double dy_mm,
                                         double dz_mm,
                                         double droll_deg,
                                         double dpitch_deg,
                                         double dyaw_deg) {
    geometry_msgs::msg::Pose adjusted_pose = base_pose;
    adjusted_pose.position.x += dx_mm * kMmToM;
    adjusted_pose.position.y += dy_mm * kMmToM;
    adjusted_pose.position.z += dz_mm * kMmToM;

    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    quaternionToEulerDeg(base_pose.orientation, &roll_deg, &pitch_deg, &yaw_deg);
    adjusted_pose.orientation =
        eulerDegToQuaternion(roll_deg + droll_deg,
                             pitch_deg + dpitch_deg,
                             yaw_deg + dyaw_deg);
    return adjusted_pose;
}

QString poseSummaryText(const geometry_msgs::msg::Pose& pose) {
    double roll_deg = 0.0;
    double pitch_deg = 0.0;
    double yaw_deg = 0.0;
    quaternionToEulerDeg(pose.orientation, &roll_deg, &pitch_deg, &yaw_deg);
    return QString("X=%1 m, Y=%2 m, Z=%3 m | R=%4 deg, P=%5 deg, Y=%6 deg")
        .arg(pose.position.x, 0, 'f', 4)
        .arg(pose.position.y, 0, 'f', 4)
        .arg(pose.position.z, 0, 'f', 4)
        .arg(roll_deg, 0, 'f', 2)
        .arg(pitch_deg, 0, 'f', 2)
        .arg(yaw_deg, 0, 'f', 2);
}

geometry_msgs::msg::Pose buildPistonCylinderPoseFromFrame(const WorkpieceFrame& frame,
                                                          double piston_length_mm) {
    tf2::Vector3 radial_axis;
    tf2::Vector3 side_axis;
    tf2::Vector3 axial_axis;
    if (!buildWorkpieceAxes(frame, &radial_axis, &side_axis, &axial_axis)) {
        geometry_msgs::msg::Pose fallback_pose;
        fallback_pose.position = frame.origin;
        fallback_pose.orientation.w = 1.0;
        return fallback_pose;
    }

    tf2::Matrix3x3 cylinder_basis(
        radial_axis.x(), side_axis.x(), axial_axis.x(),
        radial_axis.y(), side_axis.y(), axial_axis.y(),
        radial_axis.z(), side_axis.z(), axial_axis.z());
    tf2::Quaternion cylinder_q;
    cylinder_basis.getRotation(cylinder_q);

    const tf2::Vector3 center =
        toTf(frame.origin) + axial_axis * (0.5 * piston_length_mm * kMmToM);
    geometry_msgs::msg::Pose cylinder_pose;
    cylinder_pose.position.x = center.x();
    cylinder_pose.position.y = center.y();
    cylinder_pose.position.z = center.z();
    cylinder_pose.orientation = tf2::toMsg(cylinder_q.normalized());
    return cylinder_pose;
}

geometry_msgs::msg::Point buildSprayReferencePointFromFrame(const WorkpieceFrame& frame,
                                                            double radial_offset_mm) {
    return my_cr5_control::piston::mapLocalPointToBase(frame, radial_offset_mm, 0.0, 0.0);
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

void setMarkerColor(visualization_msgs::msg::Marker& marker,
                    float r,
                    float g,
                    float b,
                    float a) {
    marker.color.r = r;
    marker.color.g = g;
    marker.color.b = b;
    marker.color.a = a;
}

QString teachStatePath() {
    const QString config_root =
        QStandardPaths::writableLocation(QStandardPaths::GenericConfigLocation);
    if (!config_root.isEmpty()) {
        return config_root + "/cr5_piston_spray_gui/teach_state.json";
    }
    return QDir::homePath() + "/.config/cr5_piston_spray_gui/teach_state.json";
}

QString legacyTeachStatePath() {
    const QString config_root =
        QStandardPaths::writableLocation(QStandardPaths::AppConfigLocation);
    if (!config_root.isEmpty()) {
        return config_root + "/teach_state.json";
    }
    return QDir::homePath() + "/.config/cr5_piston_spray_gui/teach_state.json";
}

QPointF scaledPoint(const QPointF& point,
                    const QRectF& content_rect,
                    const QRectF& data_rect) {
    const double width = std::max(1.0, data_rect.width());
    const double height = std::max(1.0, data_rect.height());
    const double x =
        content_rect.left() + (point.x() - data_rect.left()) / width * content_rect.width();
    const double y =
        content_rect.bottom() - (point.y() - data_rect.top()) / height * content_rect.height();
    return QPointF(x, y);
}

QRectF expandBounds(const QRectF& rect, double pad_ratio = 0.1) {
    const double pad_x = std::max(1e-6, rect.width() * pad_ratio);
    const double pad_y = std::max(1e-6, rect.height() * pad_ratio);
    return QRectF(rect.left() - pad_x,
                  rect.top() - pad_y,
                  rect.width() + 2.0 * pad_x,
                  rect.height() + 2.0 * pad_y);
}

QRectF buildBounds(const QVector<QPointF>& points) {
    if (points.isEmpty()) {
        return QRectF(-1.0, -1.0, 2.0, 2.0);
    }
    double min_x = points.first().x();
    double max_x = points.first().x();
    double min_y = points.first().y();
    double max_y = points.first().y();
    for (const QPointF& point : points) {
        min_x = std::min(min_x, point.x());
        max_x = std::max(max_x, point.x());
        min_y = std::min(min_y, point.y());
        max_y = std::max(max_y, point.y());
    }
    if (std::abs(max_x - min_x) < 1e-6) {
        max_x += 1.0;
        min_x -= 1.0;
    }
    if (std::abs(max_y - min_y) < 1e-6) {
        max_y += 1.0;
        min_y -= 1.0;
    }
    return QRectF(min_x, min_y, max_x - min_x, max_y - min_y);
}

class PistonSprayPreviewWidgetLocal : public QWidget {
public:
    explicit PistonSprayPreviewWidgetLocal(QWidget* parent = nullptr)
        : QWidget(parent) {
        setMinimumHeight(360);
        setAutoFillBackground(true);
    }

    void setPlan(const std::optional<SprayPlan>& plan) {
        plan_ = plan;
        update();
    }

protected:
    void paintEvent(QPaintEvent* event) override {
        QWidget::paintEvent(event);

        QPainter painter(this);
        painter.setRenderHint(QPainter::Antialiasing, true);
        painter.fillRect(rect(), QColor("#FAFAFA"));

        const int gap = 12;
        QRect top = rect().adjusted(12, 12, -12, -12);
        QRect local_rect(top.left(), top.top(), top.width() / 2 - gap / 2, top.height() / 2 - gap / 2);
        QRect xy_rect(top.left() + top.width() / 2 + gap / 2,
                      top.top(),
                      top.width() / 2 - gap / 2,
                      top.height() / 2 - gap / 2);
        QRect xz_rect(top.left(),
                      top.top() + top.height() / 2 + gap / 2,
                      top.width(),
                      top.height() / 2 - gap / 2);

        drawPanel(painter, local_rect, "局部工艺视图");
        drawPanel(painter, xy_rect, "基坐标 XY 轨迹");
        drawPanel(painter, xz_rect, "基坐标 XZ 轨迹");

        if (!plan_.has_value()) {
            painter.setPen(QPen(QColor("#546E7A")));
            painter.drawText(local_rect, Qt::AlignCenter, "点击“生成喷涂计划”后显示预览");
            painter.drawText(xy_rect, Qt::AlignCenter, "当前无轨迹");
            painter.drawText(xz_rect, Qt::AlignCenter, "当前无轨迹");
            return;
        }

        drawLocalPanel(painter, local_rect, *plan_);
        drawBaseProjectionPanel(painter, xy_rect, *plan_, true);
        drawBaseProjectionPanel(painter, xz_rect, *plan_, false);
    }

private:
    void drawPanel(QPainter& painter, const QRect& rect, const QString& title) const {
        painter.save();
        painter.setPen(QPen(QColor("#CFD8DC"), 1.0));
        painter.setBrush(QColor("#FFFFFF"));
        painter.drawRoundedRect(rect, 8, 8);
        painter.setPen(QPen(QColor("#263238"), 1.0));
        painter.drawText(rect.adjusted(10, 8, -10, -8), Qt::AlignTop | Qt::AlignLeft, title);
        painter.restore();
    }

    void drawLocalPanel(QPainter& painter, const QRect& panel, const SprayPlan& plan) const {
        QRectF content = panel.adjusted(18, 34, -18, -18);
        QVector<QPointF> nozzle_points;
        for (const auto& sample : plan.path_samples) {
            nozzle_points.push_back(QPointF(sample.local_z_mm, sample.local_x_mm));
        }
        nozzle_points.push_back(QPointF(-plan.process.lead_in_mm,
                                        plan.metrics.nozzle_radius_mm + plan.process.radial_clearance_mm));
        nozzle_points.push_back(QPointF(plan.piston.spray_length_mm + plan.process.lead_out_mm,
                                        plan.metrics.nozzle_radius_mm + plan.process.radial_clearance_mm));

        QRectF bounds = expandBounds(buildBounds(nozzle_points), 0.15);
        bounds.setTop(std::min(bounds.top(), -plan.metrics.piston_radius_mm * 1.1));
        bounds.setBottom(std::max(bounds.bottom(), plan.metrics.nozzle_radius_mm + plan.process.radial_clearance_mm * 1.2));

        painter.save();
        painter.setClipRect(content);

        const QRectF piston_rect_data(
            0.0,
            -plan.metrics.piston_radius_mm,
            plan.piston.spray_length_mm,
            plan.metrics.piston_radius_mm * 2.0);
        const QPointF piston_top_left = scaledPoint(piston_rect_data.topLeft(), content, bounds);
        const QPointF piston_bottom_right = scaledPoint(piston_rect_data.bottomRight(), content, bounds);
        QRectF piston_rect(piston_top_left, piston_bottom_right);
        piston_rect = piston_rect.normalized();
        painter.setPen(Qt::NoPen);
        painter.setBrush(QColor("#CFD8DC"));
        painter.drawRect(piston_rect);

        painter.setPen(QPen(QColor("#90A4AE"), 1.0, Qt::DashLine));
        const QPointF axis_start = scaledPoint(QPointF(bounds.left(), 0.0), content, bounds);
        const QPointF axis_end = scaledPoint(QPointF(bounds.right(), 0.0), content, bounds);
        painter.drawLine(axis_start, axis_end);

        QPainterPath path;
        bool first = true;
        for (const auto& sample : plan.path_samples) {
            const QPointF mapped = scaledPoint(
                QPointF(sample.local_z_mm, sample.local_x_mm), content, bounds);
            if (first) {
                path.moveTo(mapped);
                first = false;
            } else {
                path.lineTo(mapped);
            }
        }
        painter.setPen(QPen(QColor("#1565C0"), 2.0));
        painter.drawPath(path);

        for (const auto& key_pose : plan.key_poses) {
            const QPointF point =
                scaledPoint(QPointF(key_pose.local_z_mm, key_pose.local_x_mm), content, bounds);
            painter.setBrush(QColor("#D84315"));
            painter.setPen(Qt::NoPen);
            painter.drawEllipse(point, 4.0, 4.0);
            painter.setPen(QPen(QColor("#37474F"), 1.0));
            painter.drawText(point + QPointF(6.0, -6.0), QString::fromStdString(key_pose.name));
        }
        painter.restore();
    }

    void drawBaseProjectionPanel(QPainter& painter,
                                 const QRect& panel,
                                 const SprayPlan& plan,
                                 bool xy_projection) const {
        QRectF content = panel.adjusted(18, 34, -18, -18);
        QVector<QPointF> points;
        for (const auto& sample : plan.path_samples) {
            if (xy_projection) {
                points.push_back(QPointF(sample.tcp_pose.position.x, sample.tcp_pose.position.y));
            } else {
                points.push_back(QPointF(sample.tcp_pose.position.x, sample.tcp_pose.position.z));
            }
        }

        if (xy_projection) {
            points.push_back(QPointF(plan.frame.origin.x, plan.frame.origin.y));
        } else {
            points.push_back(QPointF(plan.frame.origin.x, plan.frame.origin.z));
        }

        const QRectF bounds = expandBounds(buildBounds(points), 0.18);

        painter.save();
        painter.setClipRect(content);

        QPainterPath path;
        bool first = true;
        for (const auto& sample : plan.path_samples) {
            const QPointF raw_point = xy_projection
                ? QPointF(sample.tcp_pose.position.x, sample.tcp_pose.position.y)
                : QPointF(sample.tcp_pose.position.x, sample.tcp_pose.position.z);
            const QPointF mapped = scaledPoint(raw_point, content, bounds);
            if (first) {
                path.moveTo(mapped);
                first = false;
            } else {
                path.lineTo(mapped);
            }
        }
        painter.setPen(QPen(QColor("#00897B"), 2.0));
        painter.drawPath(path);

        for (const auto& key_pose : plan.key_poses) {
            const QPointF raw_point = xy_projection
                ? QPointF(key_pose.tcp_pose.position.x, key_pose.tcp_pose.position.y)
                : QPointF(key_pose.tcp_pose.position.x, key_pose.tcp_pose.position.z);
            const QPointF mapped = scaledPoint(raw_point, content, bounds);
            painter.setPen(Qt::NoPen);
            painter.setBrush(QColor("#8E24AA"));
            painter.drawEllipse(mapped, 4.0, 4.0);
        }

        const QPointF origin_raw = xy_projection
            ? QPointF(plan.frame.origin.x, plan.frame.origin.y)
            : QPointF(plan.frame.origin.x, plan.frame.origin.z);
        const QPointF origin = scaledPoint(origin_raw, content, bounds);
        painter.setBrush(QColor("#263238"));
        painter.drawEllipse(origin, 4.5, 4.5);

        painter.restore();
    }

    std::optional<SprayPlan> plan_;
};

}  // namespace

PistonSprayGuiWindow::PistonSprayGuiWindow(QWidget* parent)
    : QMainWindow(parent) {
    setupUi();
    suppress_teach_state_save_ = true;
    resetToDefaults();
    suppress_teach_state_save_ = false;
    loadTeachState();
    updateWorkpieceFramePreview();
}

PistonSprayGuiWindow::~PistonSprayGuiWindow() {
    if (execution_engine_) {
        execution_engine_->emergencyStop();
        execution_engine_.reset();
    }
    clearWorkpieceFrameMarkers();
    workpiece_marker_pub_.reset();
    workpiece_marker_node_.reset();
    if (rviz_scene_robot_ && rviz_scene_robot_->isReady()) {
        rviz_scene_robot_->removeCollisionObject(kRvizPistonObjectId);
    }
    rviz_scene_robot_.reset();
    manual_motion_interface_.reset();
    motion_interface_.reset();
    peripheral_interface_.reset();
}

void PistonSprayGuiWindow::setupUi() {
    setWindowTitle("CR5 活塞喷涂基础软件");
    resize(1480, 920);

    auto* central = new QWidget(this);
    setCentralWidget(central);

    auto* main_layout = new QHBoxLayout(central);
    main_layout->setContentsMargins(10, 10, 10, 10);

    auto* horizontal_splitter = new QSplitter(Qt::Horizontal, this);
    horizontal_splitter->addWidget(createControlPanel());

    auto* right_splitter = new QSplitter(Qt::Vertical, horizontal_splitter);
    preview_widget_ = new PistonSprayPreviewWidgetLocal(right_splitter);
    right_splitter->addWidget(preview_widget_);

    auto* tab_widget = new QTabWidget(right_splitter);

    auto* pose_tab = new QWidget();
    auto* pose_layout = new QVBoxLayout(pose_tab);
    pose_table_ = new QTableWidget(0, 11, pose_tab);
    pose_table_->setHorizontalHeaderLabels({
        "名称", "Local X(mm)", "Local Y(mm)", "Local Z(mm)",
        "TCP X(m)", "TCP Y(m)", "TCP Z(m)",
        "Roll(deg)", "Pitch(deg)", "Yaw(deg)",
        "Flange XYZ(m)"
    });
    pose_table_->horizontalHeader()->setSectionResizeMode(QHeaderView::ResizeToContents);
    pose_table_->horizontalHeader()->setStretchLastSection(true);
    pose_layout->addWidget(pose_table_);
    tab_widget->addTab(pose_tab, "关键位姿");

    auto* log_tab = new QWidget();
    auto* log_layout = new QVBoxLayout(log_tab);
    log_text_ = new QTextEdit(log_tab);
    log_text_->setReadOnly(true);
    log_text_->setStyleSheet(
        "background:#111827; color:#E5E7EB; font-family:'Courier New'; font-size:10pt;");
    log_layout->addWidget(log_text_);
    tab_widget->addTab(log_tab, "运行日志");

    right_splitter->addWidget(tab_widget);
    right_splitter->setStretchFactor(0, 3);
    right_splitter->setStretchFactor(1, 2);

    horizontal_splitter->addWidget(right_splitter);
    horizontal_splitter->setStretchFactor(0, 0);
    horizontal_splitter->setStretchFactor(1, 1);
    horizontal_splitter->setSizes({420, 1000});
    main_layout->addWidget(horizontal_splitter);
}

QWidget* PistonSprayGuiWindow::createControlPanel() {
    auto* container = new QWidget();
    auto* container_layout = new QVBoxLayout(container);
    container_layout->setContentsMargins(0, 0, 0, 0);

    auto* scroll_area = new QScrollArea(container);
    scroll_area->setWidgetResizable(true);
    scroll_area->setFrameShape(QFrame::NoFrame);

    auto* scroll_widget = new QWidget();
    auto* layout = new QVBoxLayout(scroll_widget);

    auto* job_group = new QGroupBox("任务");
    auto* job_layout = new QFormLayout(job_group);
    job_name_edit_ = new QLineEdit(job_group);
    job_layout->addRow("任务名称", job_name_edit_);
    layout->addWidget(job_group);

    auto* process_group = new QGroupBox("喷涂工艺参数");
    auto* process_layout = new QFormLayout(process_group);
    piston_diameter_spin_ = createDoubleSpin(1.0, 500.0, 85.0, 2, 1.0);
    spray_length_spin_ = createDoubleSpin(1.0, 500.0, 55.0, 2, 1.0);
    spray_distance_spin_ = createDoubleSpin(1.0, 500.0, 120.0, 2, 1.0);
    spray_width_spin_ = createDoubleSpin(0.1, 200.0, 18.0, 2, 0.5);
    overlap_spin_ = createDoubleSpin(0.0, 199.0, 6.0, 2, 0.5);
    turntable_rpm_spin_ = createDoubleSpin(0.1, 500.0, 90.0, 2, 1.0);
    flow_rate_spin_ = createDoubleSpin(0.1, 2000.0, 125.0, 2, 1.0);
    lead_in_spin_ = createDoubleSpin(0.0, 200.0, 5.0, 2, 0.5);
    lead_out_spin_ = createDoubleSpin(0.0, 200.0, 5.0, 2, 0.5);
    radial_clearance_spin_ = createDoubleSpin(0.0, 300.0, 40.0, 2, 1.0);
    approach_speed_spin_ = createDoubleSpin(0.1, 500.0, 60.0, 2, 1.0);
    sample_count_spin_ = new QSpinBox(process_group);
    sample_count_spin_->setRange(10, 1000);
    sample_count_spin_->setValue(60);
    process_layout->addRow("活塞直径 (mm)", piston_diameter_spin_);
    process_layout->addRow("喷涂长度 (mm)", spray_length_spin_);
    process_layout->addRow("喷涂距离 (mm)", spray_distance_spin_);
    process_layout->addRow("喷幅 (mm)", spray_width_spin_);
    process_layout->addRow("重合量 (mm)", overlap_spin_);
    process_layout->addRow("转台转速 (rpm)", turntable_rpm_spin_);
    process_layout->addRow("流量 (ml/min)", flow_rate_spin_);
    process_layout->addRow("导入长度 (mm)", lead_in_spin_);
    process_layout->addRow("导出长度 (mm)", lead_out_spin_);
    process_layout->addRow("径向安全退让 (mm)", radial_clearance_spin_);
    process_layout->addRow("接近速度 (mm/s)", approach_speed_spin_);
    process_layout->addRow("采样点数", sample_count_spin_);
    layout->addWidget(process_group);

    auto* frame_group = new QGroupBox("工件坐标系与装夹");
    auto* frame_layout = new QFormLayout(frame_group);
    frame_origin_x_spin_ = createDoubleSpin(-2.0, 2.0, 0.45, 4, 0.01);
    frame_origin_y_spin_ = createDoubleSpin(-2.0, 2.0, 0.00, 4, 0.01);
    frame_origin_z_spin_ = createDoubleSpin(-2.0, 2.0, 0.20, 4, 0.01);
    axial_x_spin_ = createDoubleSpin(-1.0, 1.0, 0.0, 4, 0.1);
    axial_y_spin_ = createDoubleSpin(-1.0, 1.0, 0.0, 4, 0.1);
    axial_z_spin_ = createDoubleSpin(-1.0, 1.0, 1.0, 4, 0.1);
    radial_x_spin_ = createDoubleSpin(-1.0, 1.0, 1.0, 4, 0.1);
    radial_y_spin_ = createDoubleSpin(-1.0, 1.0, 0.0, 4, 0.1);
    radial_z_spin_ = createDoubleSpin(-1.0, 1.0, 0.0, 4, 0.1);
    tool_roll_spin_ = createDoubleSpin(-180.0, 180.0, 0.0, 2, 1.0);
    auto* origin_row = new QWidget(frame_group);
    auto* origin_layout = new QHBoxLayout(origin_row);
    origin_layout->setContentsMargins(0, 0, 0, 0);
    origin_layout->addWidget(frame_origin_x_spin_);
    origin_layout->addWidget(frame_origin_y_spin_);
    origin_layout->addWidget(frame_origin_z_spin_);
    auto* axial_row = new QWidget(frame_group);
    auto* axial_layout = new QHBoxLayout(axial_row);
    axial_layout->setContentsMargins(0, 0, 0, 0);
    axial_layout->addWidget(axial_x_spin_);
    axial_layout->addWidget(axial_y_spin_);
    axial_layout->addWidget(axial_z_spin_);
    auto* radial_row = new QWidget(frame_group);
    auto* radial_layout = new QHBoxLayout(radial_row);
    radial_layout->setContentsMargins(0, 0, 0, 0);
    radial_layout->addWidget(radial_x_spin_);
    radial_layout->addWidget(radial_y_spin_);
    radial_layout->addWidget(radial_z_spin_);
    frame_layout->addRow("喷涂起点轴心 origin (m)", origin_row);
    frame_layout->addRow("活塞轴向方向", axial_row);
    frame_layout->addRow("起始径向方向", radial_row);
    frame_layout->addRow("喷头绕喷射轴滚转 (deg)", tool_roll_spin_);
    workpiece_frame_preview_label_ = new QLabel("基座下工件坐标系预览: 尚未计算", frame_group);
    workpiece_frame_preview_label_->setWordWrap(true);
    workpiece_frame_preview_label_->setStyleSheet("color:#334155;");
    frame_layout->addRow("当前预览", workpiece_frame_preview_label_);
    layout->addWidget(frame_group);

    auto* tool_group = new QGroupBox("工具 TCP 偏移");
    auto* tool_layout = new QFormLayout(tool_group);
    tcp_offset_x_spin_ = createDoubleSpin(-500.0, 500.0, 0.0, 2, 1.0);
    tcp_offset_y_spin_ = createDoubleSpin(-500.0, 500.0, 0.0, 2, 1.0);
    tcp_offset_z_spin_ = createDoubleSpin(-500.0, 500.0, 0.0, 2, 1.0);
    auto* tcp_row = new QWidget(tool_group);
    auto* tcp_layout = new QHBoxLayout(tcp_row);
    tcp_layout->setContentsMargins(0, 0, 0, 0);
    tcp_layout->addWidget(tcp_offset_x_spin_);
    tcp_layout->addWidget(tcp_offset_y_spin_);
    tcp_layout->addWidget(tcp_offset_z_spin_);
    tool_layout->addRow("TCP 相对法兰偏移 (mm)", tcp_row);
    auto* tcp_note = new QLabel(
        "含义：真实喷涂点 TCP 相对法兰坐标原点的 XYZ 偏移。软件会用它在 TCP 与法兰位姿之间换算。",
        tool_group);
    tcp_note->setWordWrap(true);
    tcp_note->setStyleSheet("color:#475569;");
    tool_layout->addRow("", tcp_note);
    layout->addWidget(tool_group);

    auto* teach_group = new QGroupBox("末端示教与微调");
    auto* teach_layout = new QVBoxLayout(teach_group);

    auto* teach_note = new QLabel("用于保存当前末端 TCP 位姿，并在不接转台的情况下做小范围微调。", teach_group);
    teach_note->setWordWrap(true);
    teach_note->setStyleSheet("color:#475569;");
    teach_layout->addWidget(teach_note);

    teach_live_pose_label_ = new QLabel("当前TCP: 尚未读取", teach_group);
    teach_live_pose_label_->setWordWrap(true);
    teach_layout->addWidget(teach_live_pose_label_);

    teach_saved_pose_label_ = new QLabel("示教基准: 尚未保存", teach_group);
    teach_saved_pose_label_->setWordWrap(true);
    teach_layout->addWidget(teach_saved_pose_label_);

    teach_target_pose_label_ = new QLabel("微调目标: 请先保存当前末端位姿", teach_group);
    teach_target_pose_label_->setWordWrap(true);
    teach_target_pose_label_->setStyleSheet("color:#0F766E; font-weight:bold;");
    teach_layout->addWidget(teach_target_pose_label_);

    auto* teach_offset_grid = new QGridLayout();
    teach_offset_grid->addWidget(new QLabel("dX (mm)", teach_group), 0, 0);
    teach_delta_x_spin_ = createDoubleSpin(-100.0, 100.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_x_spin_, 0, 1);
    teach_offset_grid->addWidget(new QLabel("dY (mm)", teach_group), 0, 2);
    teach_delta_y_spin_ = createDoubleSpin(-100.0, 100.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_y_spin_, 0, 3);
    teach_offset_grid->addWidget(new QLabel("dZ (mm)", teach_group), 0, 4);
    teach_delta_z_spin_ = createDoubleSpin(-100.0, 100.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_z_spin_, 0, 5);

    teach_offset_grid->addWidget(new QLabel("dRoll (deg)", teach_group), 1, 0);
    teach_delta_roll_spin_ = createDoubleSpin(-180.0, 180.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_roll_spin_, 1, 1);
    teach_offset_grid->addWidget(new QLabel("dPitch (deg)", teach_group), 1, 2);
    teach_delta_pitch_spin_ = createDoubleSpin(-180.0, 180.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_pitch_spin_, 1, 3);
    teach_offset_grid->addWidget(new QLabel("dYaw (deg)", teach_group), 1, 4);
    teach_delta_yaw_spin_ = createDoubleSpin(-180.0, 180.0, 0.0, 2, 0.5);
    teach_offset_grid->addWidget(teach_delta_yaw_spin_, 1, 5);
    teach_layout->addLayout(teach_offset_grid);

    auto* teach_motion_row = new QWidget(teach_group);
    auto* teach_motion_layout = new QHBoxLayout(teach_motion_row);
    teach_motion_layout->setContentsMargins(0, 0, 0, 0);
    teach_motion_mode_combo_ = new QComboBox(teach_motion_row);
    teach_motion_mode_combo_->addItem("直线微调");
    teach_motion_mode_combo_->addItem("规划微调");
    teach_move_speed_spin_ = createDoubleSpin(1.0, 200.0, 20.0, 1, 1.0);
    teach_motion_layout->addWidget(new QLabel("移动方式", teach_motion_row));
    teach_motion_layout->addWidget(teach_motion_mode_combo_);
    teach_motion_layout->addWidget(new QLabel("速度 (mm/s)", teach_motion_row));
    teach_motion_layout->addWidget(teach_move_speed_spin_);
    teach_layout->addWidget(teach_motion_row);

    capture_teach_pose_btn_ = new QPushButton("保存当前末端位姿", teach_group);
    capture_teach_pose_btn_->setStyleSheet(
        "background:#0369A1; color:white; font-weight:bold; padding:8px;");
    connect(capture_teach_pose_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onCaptureTeachPose);
    teach_layout->addWidget(capture_teach_pose_btn_);

    move_adjusted_teach_pose_btn_ = new QPushButton("移动到微调目标", teach_group);
    move_adjusted_teach_pose_btn_->setStyleSheet(
        "background:#15803D; color:white; font-weight:bold; padding:8px;");
    connect(move_adjusted_teach_pose_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onMoveToAdjustedTeachPose);
    teach_layout->addWidget(move_adjusted_teach_pose_btn_);

    reset_teach_offset_btn_ = new QPushButton("清零微调参数", teach_group);
    reset_teach_offset_btn_->setStyleSheet(
        "background:#475569; color:white; font-weight:bold; padding:8px;");
    connect(reset_teach_offset_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onResetTeachOffsets);
    teach_layout->addWidget(reset_teach_offset_btn_);

    auto update_teach_target = [this](double) { updateAdjustedTeachPose(); };
    auto save_teach_state = [this](double) {
        if (!suppress_teach_state_save_) {
            saveTeachState();
        }
    };
    connect(teach_delta_x_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(teach_delta_y_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(teach_delta_z_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(teach_delta_roll_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(teach_delta_pitch_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(teach_delta_yaw_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_teach_target);
    connect(tcp_offset_x_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            save_teach_state);
    connect(tcp_offset_y_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            save_teach_state);
    connect(tcp_offset_z_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            save_teach_state);
    connect(teach_move_speed_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            save_teach_state);
    connect(teach_motion_mode_combo_,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                if (!suppress_teach_state_save_) {
                    saveTeachState();
                }
            });

    auto update_workpiece_preview = [this](double) { updateWorkpieceFramePreview(); };
    connect(frame_origin_x_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(frame_origin_y_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(frame_origin_z_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(axial_x_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(axial_y_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(axial_z_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(radial_x_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(radial_y_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(radial_z_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(tool_roll_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(piston_diameter_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    connect(spray_length_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    layout->addWidget(teach_group);

    auto* rviz_piston_group = new QGroupBox("RViz 活塞圆柱");
    auto* rviz_piston_layout = new QVBoxLayout(rviz_piston_group);
    auto* rviz_note = new QLabel(
        "工件坐标系以机器人底座 base_link 为参照。先发布工件坐标系，再按该坐标系生成活塞圆柱。",
        rviz_piston_group);
    rviz_note->setWordWrap(true);
    rviz_note->setStyleSheet("color:#475569;");
    rviz_piston_layout->addWidget(rviz_note);

    publish_workpiece_frame_btn_ = new QPushButton("生成/刷新工件坐标系", rviz_piston_group);
    publish_workpiece_frame_btn_->setStyleSheet(
        "background:#1D4ED8; color:white; font-weight:bold; padding:8px;");
    connect(publish_workpiece_frame_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onPublishWorkpieceFrame);
    rviz_piston_layout->addWidget(publish_workpiece_frame_btn_);

    clear_workpiece_frame_btn_ = new QPushButton("清除工件坐标系", rviz_piston_group);
    clear_workpiece_frame_btn_->setStyleSheet(
        "background:#475569; color:white; font-weight:bold; padding:8px;");
    connect(clear_workpiece_frame_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onClearWorkpieceFrame);
    rviz_piston_layout->addWidget(clear_workpiece_frame_btn_);

    workpiece_frame_status_label_ = new QLabel(
        "工件坐标系: 未发布，RViz 中请添加 MarkerArray 话题 /piston_spray/workpiece_markers",
        rviz_piston_group);
    workpiece_frame_status_label_->setWordWrap(true);
    rviz_piston_layout->addWidget(workpiece_frame_status_label_);

    rviz_piston_distance_spin_ = createDoubleSpin(0.0, 500.0, 120.0, 2, 1.0);
    connect(rviz_piston_distance_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            save_teach_state);
    connect(rviz_piston_distance_spin_,
            qOverload<double>(&QDoubleSpinBox::valueChanged),
            this,
            update_workpiece_preview);
    auto* rviz_form = new QFormLayout();
    rviz_form->addRow("法兰末端到活塞表面距离 (mm)", rviz_piston_distance_spin_);
    rviz_piston_layout->addLayout(rviz_form);

    generate_rviz_piston_btn_ = new QPushButton("生成 RViz 活塞圆柱", rviz_piston_group);
    generate_rviz_piston_btn_->setStyleSheet(
        "background:#7C2D12; color:white; font-weight:bold; padding:8px;");
    connect(generate_rviz_piston_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onGenerateRvizPiston);
    rviz_piston_layout->addWidget(generate_rviz_piston_btn_);

    clear_rviz_piston_btn_ = new QPushButton("清除 RViz 活塞圆柱", rviz_piston_group);
    clear_rviz_piston_btn_->setStyleSheet(
        "background:#334155; color:white; font-weight:bold; padding:8px;");
    connect(clear_rviz_piston_btn_,
            &QPushButton::clicked,
            this,
            &PistonSprayGuiWindow::onClearRvizPiston);
    rviz_piston_layout->addWidget(clear_rviz_piston_btn_);

    rviz_piston_status_label_ = new QLabel("状态: 未生成圆柱", rviz_piston_group);
    rviz_piston_status_label_->setWordWrap(true);
    rviz_piston_layout->addWidget(rviz_piston_status_label_);
    layout->addWidget(rviz_piston_group);

    auto* action_group = new QGroupBox("操作");
    auto* action_layout = new QVBoxLayout(action_group);
    auto* generate_btn = new QPushButton("生成喷涂计划", action_group);
    generate_btn->setStyleSheet(
        "background:#0F766E; color:white; font-weight:bold; padding:10px;");
    connect(generate_btn, &QPushButton::clicked, this, &PistonSprayGuiWindow::onGeneratePlan);
    action_layout->addWidget(generate_btn);

    auto* load_btn = new QPushButton("导入任务 JSON", action_group);
    load_btn->setStyleSheet(
        "background:#1D4ED8; color:white; font-weight:bold; padding:8px;");
    connect(load_btn, &QPushButton::clicked, this, &PistonSprayGuiWindow::onLoadJobJson);
    action_layout->addWidget(load_btn);

    auto* export_btn = new QPushButton("导出计划 JSON", action_group);
    export_btn->setStyleSheet(
        "background:#7C3AED; color:white; font-weight:bold; padding:8px;");
    connect(export_btn, &QPushButton::clicked, this, &PistonSprayGuiWindow::onExportPlanJson);
    action_layout->addWidget(export_btn);

    auto* reset_btn = new QPushButton("恢复默认值", action_group);
    reset_btn->setStyleSheet(
        "background:#475569; color:white; font-weight:bold; padding:8px;");
    connect(reset_btn, &QPushButton::clicked, this, &PistonSprayGuiWindow::resetToDefaults);
    action_layout->addWidget(reset_btn);
    layout->addWidget(action_group);

    auto* summary_group = new QGroupBox("摘要");
    auto* summary_layout = new QVBoxLayout(summary_group);
    summary_text_ = new QTextEdit(summary_group);
    summary_text_->setReadOnly(true);
    summary_text_->setMinimumHeight(180);
    summary_text_->setStyleSheet("background:#F8FAFC;");
    summary_layout->addWidget(summary_text_);
    layout->addWidget(summary_group);

    auto* execution_group = new QGroupBox("执行控制");
    auto* execution_layout = new QVBoxLayout(execution_group);
    execution_backend_combo_ = new QComboBox(execution_group);
    execution_backend_combo_->addItem("仿真执行");
    execution_backend_combo_->addItem("CR5位姿执行（转台/喷头模拟）");
    execution_layout->addWidget(execution_backend_combo_);

    execution_backend_label_ = new QLabel("后端: 仿真执行", execution_group);
    execution_backend_label_->setWordWrap(true);
    execution_layout->addWidget(execution_backend_label_);

    execution_state_label_ = new QLabel("执行状态: 空闲", execution_group);
    execution_state_label_->setStyleSheet("color:#455A64; font-weight:bold;");
    execution_layout->addWidget(execution_state_label_);

    start_execution_btn_ = new QPushButton("开始执行", execution_group);
    start_execution_btn_->setStyleSheet(
        "background:#047857; color:white; font-weight:bold; padding:8px;");
    connect(start_execution_btn_, &QPushButton::clicked, this, &PistonSprayGuiWindow::onStartExecution);
    execution_layout->addWidget(start_execution_btn_);

    pause_resume_btn_ = new QPushButton("暂停", execution_group);
    pause_resume_btn_->setStyleSheet(
        "background:#D97706; color:white; font-weight:bold; padding:8px;");
    connect(pause_resume_btn_, &QPushButton::clicked, this, &PistonSprayGuiWindow::onPauseResumeExecution);
    execution_layout->addWidget(pause_resume_btn_);

    stop_execution_btn_ = new QPushButton("停止", execution_group);
    stop_execution_btn_->setStyleSheet(
        "background:#475569; color:white; font-weight:bold; padding:8px;");
    connect(stop_execution_btn_, &QPushButton::clicked, this, &PistonSprayGuiWindow::onStopExecution);
    execution_layout->addWidget(stop_execution_btn_);

    emergency_stop_btn_ = new QPushButton("急停", execution_group);
    emergency_stop_btn_->setStyleSheet(
        "background:#B91C1C; color:white; font-weight:bold; padding:10px;");
    connect(emergency_stop_btn_, &QPushButton::clicked, this, &PistonSprayGuiWindow::onEmergencyStopExecution);
    execution_layout->addWidget(emergency_stop_btn_);

    connect(execution_backend_combo_,
            qOverload<int>(&QComboBox::currentIndexChanged),
            this,
            [this](int) {
                execution_backend_index_ = -1;
                execution_engine_.reset();
                motion_interface_.reset();
                peripheral_interface_.reset();
                updateExecutionUi();
            });
    layout->addWidget(execution_group);

    layout->addStretch();
    scroll_area->setWidget(scroll_widget);
    container_layout->addWidget(scroll_area);
    return container;
}

void PistonSprayGuiWindow::appendLog(const QString& message, const QString& level) {
    const QString timestamp = QDateTime::currentDateTime().toString("hh:mm:ss");
    const QString color = levelColor(level);
    const QString html =
        QString("<span style='color:#9CA3AF;'>[%1]</span> "
                "<span style='color:%2; font-weight:bold;'>[%3]</span> "
                "<span>%4</span>")
            .arg(timestamp, color, level, message.toHtmlEscaped());
    log_text_->append(html);
}

void PistonSprayGuiWindow::resetToDefaults() {
    const bool previous_suppression = suppress_teach_state_save_;
    suppress_teach_state_save_ = true;
    if (execution_engine_) {
        execution_engine_->stop(nullptr);
    }

    job_name_edit_->setText("cr5_piston_basic_gui");
    piston_diameter_spin_->setValue(85.0);
    spray_length_spin_->setValue(55.0);
    spray_distance_spin_->setValue(120.0);
    spray_width_spin_->setValue(18.0);
    overlap_spin_->setValue(6.0);
    turntable_rpm_spin_->setValue(90.0);
    flow_rate_spin_->setValue(125.0);
    lead_in_spin_->setValue(5.0);
    lead_out_spin_->setValue(5.0);
    radial_clearance_spin_->setValue(40.0);
    approach_speed_spin_->setValue(60.0);
    sample_count_spin_->setValue(60);

    frame_origin_x_spin_->setValue(0.45);
    frame_origin_y_spin_->setValue(0.00);
    frame_origin_z_spin_->setValue(0.20);
    axial_x_spin_->setValue(0.0);
    axial_y_spin_->setValue(0.0);
    axial_z_spin_->setValue(1.0);
    radial_x_spin_->setValue(1.0);
    radial_y_spin_->setValue(0.0);
    radial_z_spin_->setValue(0.0);
    tool_roll_spin_->setValue(0.0);

    tcp_offset_x_spin_->setValue(0.0);
    tcp_offset_y_spin_->setValue(0.0);
    tcp_offset_z_spin_->setValue(0.0);
    teach_delta_x_spin_->setValue(0.0);
    teach_delta_y_spin_->setValue(0.0);
    teach_delta_z_spin_->setValue(0.0);
    teach_delta_roll_spin_->setValue(0.0);
    teach_delta_pitch_spin_->setValue(0.0);
    teach_delta_yaw_spin_->setValue(0.0);
    teach_move_speed_spin_->setValue(20.0);
    teach_motion_mode_combo_->setCurrentIndex(0);
    rviz_piston_distance_spin_->setValue(120.0);

    current_plan_.reset();
    current_tcp_pose_.reset();
    saved_teach_tcp_pose_.reset();
    adjusted_teach_tcp_pose_.reset();
    summary_text_->setPlainText("填写参数后点击“生成喷涂计划”。");
    pose_table_->setRowCount(0);
    static_cast<PistonSprayPreviewWidgetLocal*>(preview_widget_)->setPlan(current_plan_);
    execution_backend_combo_->setCurrentIndex(0);
    execution_backend_index_ = -1;
    execution_engine_.reset();
    manual_motion_interface_.reset();
    motion_interface_.reset();
    peripheral_interface_.reset();
    if (rviz_scene_robot_ && rviz_scene_robot_->isReady()) {
        rviz_scene_robot_->removeCollisionObject(kRvizPistonObjectId);
    }
    clearWorkpieceFrameMarkers();
    rviz_scene_robot_.reset();
    workpiece_frame_status_label_->setText(
        "工件坐标系: 未发布，RViz 中请添加 MarkerArray 话题 /piston_spray/workpiece_markers");
    rviz_piston_status_label_->setText("状态: 未生成圆柱");
    updateExecutionUi();
    updateWorkpieceFramePreview();
    updateAdjustedTeachPose();
    suppress_teach_state_save_ = previous_suppression;
    if (!suppress_teach_state_save_) {
        saveTeachState();
    }
    appendLog("已恢复默认喷涂参数。", "INFO");
}

bool PistonSprayGuiWindow::collectInputs(my_cr5_control::piston::PistonSpecMm* piston,
                                         my_cr5_control::piston::SprayProcessMm* process,
                                         my_cr5_control::piston::WorkpieceFrame* frame,
                                         my_cr5_control::piston::ToolTcpOffsetMm* tool_offset) const {
    if (piston == nullptr || process == nullptr || frame == nullptr || tool_offset == nullptr) {
        return false;
    }

    piston->diameter_mm = piston_diameter_spin_->value();
    piston->spray_length_mm = spray_length_spin_->value();

    process->spray_distance_mm = spray_distance_spin_->value();
    process->spray_width_mm = spray_width_spin_->value();
    process->overlap_mm = overlap_spin_->value();
    process->turntable_rpm = turntable_rpm_spin_->value();
    process->flow_rate_ml_min = flow_rate_spin_->value();
    process->lead_in_mm = lead_in_spin_->value();
    process->lead_out_mm = lead_out_spin_->value();
    process->radial_clearance_mm = radial_clearance_spin_->value();
    process->approach_speed_mm_s = approach_speed_spin_->value();
    process->sample_count = sample_count_spin_->value();

    frame->origin.x = frame_origin_x_spin_->value();
    frame->origin.y = frame_origin_y_spin_->value();
    frame->origin.z = frame_origin_z_spin_->value();
    frame->axial_direction.x = axial_x_spin_->value();
    frame->axial_direction.y = axial_y_spin_->value();
    frame->axial_direction.z = axial_z_spin_->value();
    frame->radial_direction.x = radial_x_spin_->value();
    frame->radial_direction.y = radial_y_spin_->value();
    frame->radial_direction.z = radial_z_spin_->value();
    frame->tool_roll_deg = tool_roll_spin_->value();

    tool_offset->x_mm = tcp_offset_x_spin_->value();
    tool_offset->y_mm = tcp_offset_y_spin_->value();
    tool_offset->z_mm = tcp_offset_z_spin_->value();
    return true;
}

QString PistonSprayGuiWindow::jobName() const {
    const QString raw = job_name_edit_->text().trimmed();
    return raw.isEmpty() ? QStringLiteral("cr5_piston_job") : raw;
}

QString PistonSprayGuiWindow::defaultExportPath() const {
    return QString("/home/zhu/dobot_ws/piston/output/%1_plan.json").arg(jobName());
}

void PistonSprayGuiWindow::onGeneratePlan() {
    my_cr5_control::piston::PistonSpecMm piston;
    my_cr5_control::piston::SprayProcessMm process;
    my_cr5_control::piston::WorkpieceFrame frame;
    my_cr5_control::piston::ToolTcpOffsetMm tool_offset;
    if (!collectInputs(&piston, &process, &frame, &tool_offset)) {
        QMessageBox::critical(this, "错误", "参数收集失败。");
        return;
    }

    SprayPlan plan;
    std::string error_message;
    if (!my_cr5_control::piston::buildSprayPlan(
            piston, process, frame, tool_offset, &plan, &error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "参数错误", QString::fromStdString(error_message));
        return;
    }

    current_plan_ = plan;
    static_cast<PistonSprayPreviewWidgetLocal*>(preview_widget_)->setPlan(current_plan_);
    updateSummary();
    updatePoseTable();

    appendLog(
        QString("喷涂计划已生成: job=%1, 轴向进给=%.3f mm/s, 喷涂时间=%.3f s, 总圈数=%.3f")
            .arg(jobName())
            .arg(plan.metrics.axial_feed_mm_s)
            .arg(plan.metrics.spray_time_s)
            .arg(plan.metrics.total_revolutions),
        "SUCCESS");
}

void PistonSprayGuiWindow::onLoadJobJson() {
    const QString path = QFileDialog::getOpenFileName(
        this,
        "选择任务 JSON",
        "/home/zhu/dobot_ws/piston/examples",
        "JSON (*.json)");
    if (path.isEmpty()) {
        return;
    }

    QFile file(path);
    if (!file.open(QIODevice::ReadOnly)) {
        QMessageBox::critical(this, "读取失败", "无法打开所选 JSON 文件。");
        return;
    }

    try {
        applyLoadedJson(file.readAll());
    } catch (const std::exception& exc) {
        const QString message = QString::fromUtf8(exc.what());
        QMessageBox::critical(this, "读取失败", message);
        appendLog(message, "ERROR");
        return;
    }
    appendLog(QString("已导入任务 JSON: %1").arg(path), "INFO");
}

void PistonSprayGuiWindow::applyLoadedJson(const QByteArray& content) {
    QJsonParseError parse_error;
    const QJsonDocument document = QJsonDocument::fromJson(content, &parse_error);
    if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
        throw std::runtime_error(
            QString("JSON 解析失败: %1").arg(parse_error.errorString()).toStdString());
    }

    const QJsonObject root = document.object();
    if (root.contains("job_name") && root.value("job_name").isString()) {
        job_name_edit_->setText(root.value("job_name").toString());
    }

    const QJsonObject piston = root.value("piston").toObject();
    setSpinFromJson(piston_diameter_spin_, piston, "diameter_mm");
    setSpinFromJson(spray_length_spin_, piston, "spray_length_mm");

    const QJsonObject process = root.value("process").toObject();
    setSpinFromJson(spray_distance_spin_, process, "spray_distance_mm");
    setSpinFromJson(spray_width_spin_, process, "spray_width_mm");
    setSpinFromJson(overlap_spin_, process, "overlap_mm");
    setSpinFromJson(turntable_rpm_spin_, process, "turntable_rpm");
    setSpinFromJson(flow_rate_spin_, process, "flow_rate_ml_min");
    setSpinFromJson(lead_in_spin_, process, "lead_in_mm");
    setSpinFromJson(lead_out_spin_, process, "lead_out_mm");
    setSpinFromJson(radial_clearance_spin_, process, "radial_clearance_mm");
    setSpinFromJson(approach_speed_spin_, process, "approach_speed_mm_s");
    if (process.contains("sample_count") && process.value("sample_count").isDouble()) {
        sample_count_spin_->setValue(process.value("sample_count").toInt());
    }

    const QJsonObject frame = root.value("frame").toObject();
    if (!frame.isEmpty()) {
        setVectorSpinsFromJson(frame_origin_x_spin_,
                               frame_origin_y_spin_,
                               frame_origin_z_spin_,
                               frame.value("origin_m").toObject());
        setVectorSpinsFromJson(axial_x_spin_,
                               axial_y_spin_,
                               axial_z_spin_,
                               frame.value("axial_direction").toObject());
        setVectorSpinsFromJson(radial_x_spin_,
                               radial_y_spin_,
                               radial_z_spin_,
                               frame.value("radial_direction").toObject());
        setSpinFromJson(tool_roll_spin_, frame, "tool_roll_deg");
    }

    const QJsonObject tool_offset = root.value("tool_tcp_offset_mm").toObject();
    if (!tool_offset.isEmpty()) {
        setVectorSpinsFromJson(tcp_offset_x_spin_,
                               tcp_offset_y_spin_,
                               tcp_offset_z_spin_,
                               tool_offset);
    }

    updateWorkpieceFramePreview();
}

void PistonSprayGuiWindow::updateSummary() {
    if (!current_plan_.has_value()) {
        summary_text_->setPlainText("当前无喷涂计划。");
        return;
    }

    const auto& plan = *current_plan_;
    QStringList lines;
    lines
        << QString("任务: %1").arg(jobName())
        << QString("螺旋节距: %1 mm/rev").arg(plan.metrics.helical_pitch_mm_per_rev, 0, 'f', 3)
        << QString("轴向进给: %1 mm/s").arg(plan.metrics.axial_feed_mm_s, 0, 'f', 3)
        << QString("表面线速度: %1 mm/s").arg(plan.metrics.surface_speed_mm_s, 0, 'f', 3)
        << QString("喷涂时间: %1 s").arg(plan.metrics.spray_time_s, 0, 'f', 3)
        << QString("总圈数: %1").arg(plan.metrics.total_revolutions, 0, 'f', 3)
        << QString("石墨估算用量: %1 ml").arg(plan.metrics.estimated_graphite_usage_ml, 0, 'f', 3)
        << QString("局部喷头半径: %1 mm").arg(plan.metrics.nozzle_radius_mm, 0, 'f', 3)
        << QString("关键位姿数: %1 | 轨迹采样数: %2")
               .arg(plan.key_poses.size())
               .arg(plan.path_samples.size());

    tf2::Vector3 radial_axis;
    tf2::Vector3 side_axis;
    tf2::Vector3 axial_axis;
    if (buildWorkpieceAxes(plan.frame, &radial_axis, &side_axis, &axial_axis)) {
        lines
            << ""
            << "工件坐标系(base_link):"
            << QString("  origin = (%1, %2, %3) m")
                   .arg(plan.frame.origin.x, 0, 'f', 4)
                   .arg(plan.frame.origin.y, 0, 'f', 4)
                   .arg(plan.frame.origin.z, 0, 'f', 4)
            << QString("  X(radial) = (%1, %2, %3)")
                   .arg(radial_axis.x(), 0, 'f', 3)
                   .arg(radial_axis.y(), 0, 'f', 3)
                   .arg(radial_axis.z(), 0, 'f', 3)
            << QString("  Y(side) = (%1, %2, %3)")
                   .arg(side_axis.x(), 0, 'f', 3)
                   .arg(side_axis.y(), 0, 'f', 3)
                   .arg(side_axis.z(), 0, 'f', 3)
            << QString("  Z(axial) = (%1, %2, %3)")
                   .arg(axial_axis.x(), 0, 'f', 3)
                   .arg(axial_axis.y(), 0, 'f', 3)
                   .arg(axial_axis.z(), 0, 'f', 3);
    }

    if (!plan.notices.empty()) {
        lines << "" << "提示:";
        for (const auto& notice : plan.notices) {
            lines << QString("  - %1").arg(QString::fromStdString(notice));
        }
    }

    lines << "" << "基础执行流程:";
    for (const auto& step : plan.execution_steps) {
        QString line = QString("  %1. %2")
                           .arg(step.step_index)
                           .arg(QString::fromStdString(step.command));
        if (!step.pose_name.empty()) {
            line += QString(" -> %1").arg(QString::fromStdString(step.pose_name));
        }
        if (step.speed_mm_s > 0.0) {
            line += QString(" @ %1 mm/s").arg(step.speed_mm_s, 0, 'f', 2);
        }
        if (step.numeric_value > 0.0) {
            line += QString(" | value=%1").arg(step.numeric_value, 0, 'f', 2);
        }
        if (!step.motion_mode.empty()) {
            line += QString(" | mode=%1").arg(QString::fromStdString(step.motion_mode));
        }
        line += QString(" | %1").arg(QString::fromStdString(step.note));
        lines << line;
    }

    summary_text_->setPlainText(lines.join("\n"));
}

void PistonSprayGuiWindow::updatePoseTable() {
    pose_table_->setRowCount(0);
    if (!current_plan_.has_value()) {
        return;
    }

    const auto& plan = *current_plan_;
    pose_table_->setRowCount(static_cast<int>(plan.key_poses.size()));
    for (int row = 0; row < static_cast<int>(plan.key_poses.size()); ++row) {
        const auto& key_pose = plan.key_poses[static_cast<std::size_t>(row)];
        double roll_deg = 0.0;
        double pitch_deg = 0.0;
        double yaw_deg = 0.0;
        quaternionToEulerDeg(key_pose.tcp_pose.orientation, &roll_deg, &pitch_deg, &yaw_deg);

        const QString flange_xyz = QString("[%1, %2, %3]")
            .arg(key_pose.flange_pose.position.x, 0, 'f', 4)
            .arg(key_pose.flange_pose.position.y, 0, 'f', 4)
            .arg(key_pose.flange_pose.position.z, 0, 'f', 4);

        const QStringList values = {
            QString::fromStdString(key_pose.name),
            QString::number(key_pose.local_x_mm, 'f', 3),
            QString::number(key_pose.local_y_mm, 'f', 3),
            QString::number(key_pose.local_z_mm, 'f', 3),
            QString::number(key_pose.tcp_pose.position.x, 'f', 4),
            QString::number(key_pose.tcp_pose.position.y, 'f', 4),
            QString::number(key_pose.tcp_pose.position.z, 'f', 4),
            QString::number(roll_deg, 'f', 2),
            QString::number(pitch_deg, 'f', 2),
            QString::number(yaw_deg, 'f', 2),
            flange_xyz,
        };

        for (int col = 0; col < values.size(); ++col) {
            pose_table_->setItem(row, col, new QTableWidgetItem(values[col]));
        }
    }
}

bool PistonSprayGuiWindow::ensureManualTeachBackend(std::string* error_message) {
    const ExecutionState state =
        execution_engine_ ? execution_engine_->state() : ExecutionState::Idle;
    if (state == ExecutionState::Running || state == ExecutionState::Paused) {
        if (error_message != nullptr) {
            *error_message = "执行链路运行中，不能同时进行手动示教。";
        }
        return false;
    }

    if (!manual_motion_interface_) {
        manual_motion_interface_ =
            std::make_shared<CR5MotionInterface>("piston_spray_gui_manual_teach", false);
    }

    if (!manual_motion_interface_->ensureReady(error_message)) {
        return false;
    }
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool PistonSprayGuiWindow::ensureRvizSceneRobot(std::string* error_message) {
    const ExecutionState state =
        execution_engine_ ? execution_engine_->state() : ExecutionState::Idle;
    if (state == ExecutionState::Running || state == ExecutionState::Paused) {
        if (error_message != nullptr) {
            *error_message = "执行链路运行中，不能同时刷新 RViz 活塞模型。";
        }
        return false;
    }

    if (!rviz_scene_robot_) {
        rviz_scene_robot_ = std::make_shared<CR5Robot>("piston_spray_gui_rviz_scene", false);
        if (!rviz_scene_robot_->init()) {
            rviz_scene_robot_.reset();
            if (error_message != nullptr) {
                *error_message = "RViz 场景机器人初始化失败，请确认 MoveIt / RViz 已启动。";
            }
            return false;
        }
    }

    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool PistonSprayGuiWindow::ensureWorkpieceMarkerPublisher(std::string* error_message) {
    if (workpiece_marker_pub_) {
        if (error_message != nullptr) {
            error_message->clear();
        }
        return true;
    }

    try {
        if (!workpiece_marker_node_) {
            const std::string node_name =
                "piston_spray_gui_workpiece_markers_" +
                std::to_string(QDateTime::currentMSecsSinceEpoch());
            workpiece_marker_node_ = rclcpp::Node::make_shared(node_name);
        }
        const auto qos = rclcpp::QoS(1).reliable().transient_local();
        workpiece_marker_pub_ =
            workpiece_marker_node_->create_publisher<visualization_msgs::msg::MarkerArray>(
                kWorkpieceMarkerTopic,
                qos);
    } catch (const std::exception& exc) {
        if (error_message != nullptr) {
            *error_message = QString("工件坐标系 Marker 发布器初始化失败: %1")
                                 .arg(exc.what())
                                 .toStdString();
        }
        return false;
    }

    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

bool PistonSprayGuiWindow::publishWorkpieceFrameMarkers(const PistonSpecMm& piston,
                                                        const WorkpieceFrame& frame,
                                                        double reference_distance_mm,
                                                        std::string* error_message) {
    if (!ensureWorkpieceMarkerPublisher(error_message)) {
        return false;
    }

    tf2::Vector3 radial_axis;
    tf2::Vector3 side_axis;
    tf2::Vector3 axial_axis;
    QString frame_error;
    if (!buildWorkpieceAxes(frame, &radial_axis, &side_axis, &axial_axis, &frame_error)) {
        if (error_message != nullptr) {
            *error_message = frame_error.toStdString();
        }
        return false;
    }

    const double axis_xy_length_m =
        std::max(0.08, std::max(0.5 * piston.diameter_mm * kMmToM * 1.6, 0.5 * reference_distance_mm * kMmToM));
    const double axis_z_length_m =
        std::max(0.12, piston.spray_length_mm * kMmToM * 1.05);
    const geometry_msgs::msg::Point origin = frame.origin;
    const geometry_msgs::msg::Point x_tip = toPointMsg(toTf(origin) + radial_axis * axis_xy_length_m);
    const geometry_msgs::msg::Point y_tip = toPointMsg(toTf(origin) + side_axis * axis_xy_length_m * 0.85);
    const geometry_msgs::msg::Point z_tip = toPointMsg(toTf(origin) + axial_axis * axis_z_length_m);
    const geometry_msgs::msg::Point reference_tcp =
        buildSprayReferencePointFromFrame(frame, 0.5 * piston.diameter_mm + reference_distance_mm);

    visualization_msgs::msg::MarkerArray marker_array;

    auto origin_marker = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        0,
        visualization_msgs::msg::Marker::SPHERE);
    origin_marker.pose.position = origin;
    origin_marker.scale.x = 0.018;
    origin_marker.scale.y = 0.018;
    origin_marker.scale.z = 0.018;
    setMarkerColor(origin_marker, 0.10f, 0.12f, 0.16f, 0.95f);
    marker_array.markers.push_back(origin_marker);

    auto x_marker = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        1,
        visualization_msgs::msg::Marker::ARROW);
    x_marker.scale.x = 0.006;
    x_marker.scale.y = 0.012;
    x_marker.scale.z = 0.016;
    x_marker.points = {origin, x_tip};
    setMarkerColor(x_marker, 0.92f, 0.18f, 0.18f, 0.95f);
    marker_array.markers.push_back(x_marker);

    auto y_marker = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        2,
        visualization_msgs::msg::Marker::ARROW);
    y_marker.scale.x = 0.006;
    y_marker.scale.y = 0.012;
    y_marker.scale.z = 0.016;
    y_marker.points = {origin, y_tip};
    setMarkerColor(y_marker, 0.10f, 0.72f, 0.22f, 0.95f);
    marker_array.markers.push_back(y_marker);

    auto z_marker = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        3,
        visualization_msgs::msg::Marker::ARROW);
    z_marker.scale.x = 0.006;
    z_marker.scale.y = 0.012;
    z_marker.scale.z = 0.016;
    z_marker.points = {origin, z_tip};
    setMarkerColor(z_marker, 0.15f, 0.36f, 0.92f, 0.95f);
    marker_array.markers.push_back(z_marker);

    auto origin_label = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        4,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    origin_label.pose.position = toPointMsg(toTf(origin) + tf2::Vector3(0.0, 0.0, 0.03));
    origin_label.scale.z = 0.022;
    origin_label.text =
        QString("Workpiece O (%1, %2, %3)")
            .arg(origin.x, 0, 'f', 3)
            .arg(origin.y, 0, 'f', 3)
            .arg(origin.z, 0, 'f', 3)
            .toStdString();
    setMarkerColor(origin_label, 0.12f, 0.18f, 0.28f, 0.95f);
    marker_array.markers.push_back(origin_label);

    auto x_label = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        5,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    x_label.pose.position = toPointMsg(toTf(x_tip) + tf2::Vector3(0.0, 0.0, 0.02));
    x_label.scale.z = 0.020;
    x_label.text = "X / radial";
    setMarkerColor(x_label, 0.92f, 0.18f, 0.18f, 0.95f);
    marker_array.markers.push_back(x_label);

    auto y_label = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        6,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    y_label.pose.position = toPointMsg(toTf(y_tip) + tf2::Vector3(0.0, 0.0, 0.02));
    y_label.scale.z = 0.020;
    y_label.text = "Y / side";
    setMarkerColor(y_label, 0.10f, 0.72f, 0.22f, 0.95f);
    marker_array.markers.push_back(y_label);

    auto z_label = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        7,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    z_label.pose.position = toPointMsg(toTf(z_tip) + tf2::Vector3(0.0, 0.0, 0.02));
    z_label.scale.z = 0.020;
    z_label.text = "Z / axial";
    setMarkerColor(z_label, 0.15f, 0.36f, 0.92f, 0.95f);
    marker_array.markers.push_back(z_label);

    auto reference_marker = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        8,
        visualization_msgs::msg::Marker::SPHERE);
    reference_marker.pose.position = reference_tcp;
    reference_marker.scale.x = 0.014;
    reference_marker.scale.y = 0.014;
    reference_marker.scale.z = 0.014;
    setMarkerColor(reference_marker, 0.95f, 0.58f, 0.10f, 0.95f);
    marker_array.markers.push_back(reference_marker);

    auto reference_label = baseMarker(
        kWorkpieceMarkerFrameId,
        kWorkpieceMarkerNamespace,
        9,
        visualization_msgs::msg::Marker::TEXT_VIEW_FACING);
    reference_label.pose.position =
        toPointMsg(toTf(reference_tcp) + tf2::Vector3(0.0, 0.0, 0.025));
    reference_label.scale.z = 0.018;
    reference_label.text =
        QString("SprayRef d=%1 mm").arg(reference_distance_mm, 0, 'f', 1).toStdString();
    setMarkerColor(reference_label, 0.95f, 0.58f, 0.10f, 0.95f);
    marker_array.markers.push_back(reference_label);

    workpiece_marker_pub_->publish(marker_array);
    if (error_message != nullptr) {
        error_message->clear();
    }
    return true;
}

void PistonSprayGuiWindow::clearWorkpieceFrameMarkers() {
    if (!workpiece_marker_pub_) {
        return;
    }

    visualization_msgs::msg::MarkerArray marker_array;
    for (int id = 0; id < kWorkpieceMarkerCount; ++id) {
        auto marker = baseMarker(
            kWorkpieceMarkerFrameId,
            kWorkpieceMarkerNamespace,
            id,
            visualization_msgs::msg::Marker::SPHERE);
        marker.action = visualization_msgs::msg::Marker::DELETE;
        marker_array.markers.push_back(marker);
    }
    workpiece_marker_pub_->publish(marker_array);
}

void PistonSprayGuiWindow::updateManualTeachUi() {
    if (teach_live_pose_label_ == nullptr ||
        teach_saved_pose_label_ == nullptr ||
        teach_target_pose_label_ == nullptr) {
        return;
    }

    if (current_tcp_pose_.has_value()) {
        teach_live_pose_label_->setText(
            QString("当前TCP: %1").arg(poseSummaryText(*current_tcp_pose_)));
    } else {
        teach_live_pose_label_->setText("当前TCP: 尚未读取");
    }

    if (saved_teach_tcp_pose_.has_value()) {
        teach_saved_pose_label_->setText(
            QString("示教基准: %1").arg(poseSummaryText(*saved_teach_tcp_pose_)));
    } else {
        teach_saved_pose_label_->setText("示教基准: 尚未保存");
    }

    if (adjusted_teach_tcp_pose_.has_value()) {
        teach_target_pose_label_->setText(
            QString("微调目标: %1").arg(poseSummaryText(*adjusted_teach_tcp_pose_)));
    } else {
        teach_target_pose_label_->setText("微调目标: 请先保存当前末端位姿");
    }

    const ExecutionState state =
        execution_engine_ ? execution_engine_->state() : ExecutionState::Idle;
    const bool execution_busy =
        state == ExecutionState::Running || state == ExecutionState::Paused;
    const bool has_saved_pose = saved_teach_tcp_pose_.has_value();

    capture_teach_pose_btn_->setEnabled(!execution_busy);
    teach_delta_x_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_delta_y_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_delta_z_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_delta_roll_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_delta_pitch_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_delta_yaw_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_move_speed_spin_->setEnabled(has_saved_pose && !execution_busy);
    teach_motion_mode_combo_->setEnabled(has_saved_pose && !execution_busy);
    move_adjusted_teach_pose_btn_->setEnabled(
        adjusted_teach_tcp_pose_.has_value() && !execution_busy);
    reset_teach_offset_btn_->setEnabled(has_saved_pose && !execution_busy);
    publish_workpiece_frame_btn_->setEnabled(!execution_busy);
    clear_workpiece_frame_btn_->setEnabled(!execution_busy);
    generate_rviz_piston_btn_->setEnabled(!execution_busy);
    clear_rviz_piston_btn_->setEnabled(!execution_busy);
}

void PistonSprayGuiWindow::updateWorkpieceFramePreview() {
    if (workpiece_frame_preview_label_ == nullptr) {
        return;
    }

    WorkpieceFrame frame;
    frame.origin.x = frame_origin_x_spin_->value();
    frame.origin.y = frame_origin_y_spin_->value();
    frame.origin.z = frame_origin_z_spin_->value();
    frame.axial_direction.x = axial_x_spin_->value();
    frame.axial_direction.y = axial_y_spin_->value();
    frame.axial_direction.z = axial_z_spin_->value();
    frame.radial_direction.x = radial_x_spin_->value();
    frame.radial_direction.y = radial_y_spin_->value();
    frame.radial_direction.z = radial_z_spin_->value();
    frame.tool_roll_deg = tool_roll_spin_->value();

    tf2::Vector3 radial_axis;
    tf2::Vector3 side_axis;
    tf2::Vector3 axial_axis;
    QString error_message;
    if (!buildWorkpieceAxes(frame, &radial_axis, &side_axis, &axial_axis, &error_message)) {
        workpiece_frame_preview_label_->setText(
            QString("基座下工件坐标系预览: 参数无效，%1").arg(error_message));
        return;
    }

    const geometry_msgs::msg::Pose cylinder_pose =
        buildPistonCylinderPoseFromFrame(frame, spray_length_spin_->value());
    const geometry_msgs::msg::Point reference_tcp = buildSprayReferencePointFromFrame(
        frame,
        0.5 * piston_diameter_spin_->value() + rviz_piston_distance_spin_->value());

    workpiece_frame_preview_label_->setText(
        QString(
            "base_link 下 O=(%1,%2,%3) m | X=(%4,%5,%6) | Y=(%7,%8,%9) | Z=(%10,%11,%12) | "
            "圆柱中心=(%13,%14,%15) m | 喷涂参考点=(%16,%17,%18) m")
            .arg(frame.origin.x, 0, 'f', 4)
            .arg(frame.origin.y, 0, 'f', 4)
            .arg(frame.origin.z, 0, 'f', 4)
            .arg(radial_axis.x(), 0, 'f', 3)
            .arg(radial_axis.y(), 0, 'f', 3)
            .arg(radial_axis.z(), 0, 'f', 3)
            .arg(side_axis.x(), 0, 'f', 3)
            .arg(side_axis.y(), 0, 'f', 3)
            .arg(side_axis.z(), 0, 'f', 3)
            .arg(axial_axis.x(), 0, 'f', 3)
            .arg(axial_axis.y(), 0, 'f', 3)
            .arg(axial_axis.z(), 0, 'f', 3)
            .arg(cylinder_pose.position.x, 0, 'f', 4)
            .arg(cylinder_pose.position.y, 0, 'f', 4)
            .arg(cylinder_pose.position.z, 0, 'f', 4)
            .arg(reference_tcp.x, 0, 'f', 4)
            .arg(reference_tcp.y, 0, 'f', 4)
            .arg(reference_tcp.z, 0, 'f', 4));
}

void PistonSprayGuiWindow::updateAdjustedTeachPose() {
    if (!saved_teach_tcp_pose_.has_value()) {
        adjusted_teach_tcp_pose_.reset();
        updateManualTeachUi();
        if (!suppress_teach_state_save_) {
            saveTeachState();
        }
        return;
    }

    adjusted_teach_tcp_pose_ = applyPoseOffset(
        *saved_teach_tcp_pose_,
        teach_delta_x_spin_->value(),
        teach_delta_y_spin_->value(),
        teach_delta_z_spin_->value(),
        teach_delta_roll_spin_->value(),
        teach_delta_pitch_spin_->value(),
        teach_delta_yaw_spin_->value());
    updateManualTeachUi();
    if (!suppress_teach_state_save_) {
        saveTeachState();
    }
}

void PistonSprayGuiWindow::saveTeachState() const {
    const QString path = teachStatePath();
    QDir().mkpath(QFileInfo(path).absolutePath());

    QJsonObject root;
    root["version"] = kTeachStateVersion;
    root["job_name"] = jobName();

    QJsonObject tool_offset;
    tool_offset["x"] = tcp_offset_x_spin_->value();
    tool_offset["y"] = tcp_offset_y_spin_->value();
    tool_offset["z"] = tcp_offset_z_spin_->value();
    root["tool_tcp_offset_mm"] = tool_offset;

    if (saved_teach_tcp_pose_.has_value()) {
        root["saved_teach_tcp_pose"] = poseToJson(*saved_teach_tcp_pose_);
    }
    if (adjusted_teach_tcp_pose_.has_value()) {
        root["adjusted_teach_tcp_pose"] = poseToJson(*adjusted_teach_tcp_pose_);
    }

    QJsonObject offsets;
    offsets["dx_mm"] = teach_delta_x_spin_->value();
    offsets["dy_mm"] = teach_delta_y_spin_->value();
    offsets["dz_mm"] = teach_delta_z_spin_->value();
    offsets["droll_deg"] = teach_delta_roll_spin_->value();
    offsets["dpitch_deg"] = teach_delta_pitch_spin_->value();
    offsets["dyaw_deg"] = teach_delta_yaw_spin_->value();
    root["teach_offsets"] = offsets;

    root["teach_move_speed_mm_s"] = teach_move_speed_spin_->value();
    root["teach_motion_mode_index"] = teach_motion_mode_combo_->currentIndex();
    root["rviz_piston_distance_mm"] = rviz_piston_distance_spin_->value();

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        return;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
}

void PistonSprayGuiWindow::loadTeachState() {
    const QString primary_path = teachStatePath();
    const QString legacy_path = legacyTeachStatePath();

    QString load_path;
    QFile file(primary_path);
    if (file.exists()) {
        load_path = primary_path;
    } else if (legacy_path != primary_path) {
        QFile legacy_file(legacy_path);
        if (legacy_file.exists()) {
            load_path = legacy_path;
            file.setFileName(load_path);
        }
    }

    if (load_path.isEmpty()) {
        updateManualTeachUi();
        return;
    }
    if (!file.open(QIODevice::ReadOnly)) {
        appendLog(QString("示教持久化文件读取失败: %1").arg(load_path), "WARN");
        return;
    }

    QJsonParseError parse_error;
    const QJsonDocument document = QJsonDocument::fromJson(file.readAll(), &parse_error);
    file.close();
    if (parse_error.error != QJsonParseError::NoError || !document.isObject()) {
        appendLog(QString("示教持久化文件解析失败: %1").arg(parse_error.errorString()), "WARN");
        return;
    }

    suppress_teach_state_save_ = true;

    const QJsonObject root = document.object();
    const QJsonObject tool_offset = root.value("tool_tcp_offset_mm").toObject();
    setVectorSpinsFromJson(tcp_offset_x_spin_,
                           tcp_offset_y_spin_,
                           tcp_offset_z_spin_,
                           tool_offset);

    const QJsonObject offsets = root.value("teach_offsets").toObject();
    setSpinFromJson(teach_delta_x_spin_, offsets, "dx_mm");
    setSpinFromJson(teach_delta_y_spin_, offsets, "dy_mm");
    setSpinFromJson(teach_delta_z_spin_, offsets, "dz_mm");
    setSpinFromJson(teach_delta_roll_spin_, offsets, "droll_deg");
    setSpinFromJson(teach_delta_pitch_spin_, offsets, "dpitch_deg");
    setSpinFromJson(teach_delta_yaw_spin_, offsets, "dyaw_deg");
    setSpinFromJson(teach_move_speed_spin_, root, "teach_move_speed_mm_s");
    setSpinFromJson(rviz_piston_distance_spin_, root, "rviz_piston_distance_mm");
    if (root.contains("teach_motion_mode_index") &&
        root.value("teach_motion_mode_index").isDouble()) {
        teach_motion_mode_combo_->setCurrentIndex(root.value("teach_motion_mode_index").toInt());
    }

    geometry_msgs::msg::Pose pose;
    if (poseFromJson(root.value("saved_teach_tcp_pose").toObject(), &pose)) {
        saved_teach_tcp_pose_ = pose;
    }
    if (poseFromJson(root.value("adjusted_teach_tcp_pose").toObject(), &pose)) {
        adjusted_teach_tcp_pose_ = pose;
    }

    suppress_teach_state_save_ = false;
    updateAdjustedTeachPose();
    appendLog(QString("已恢复持久化示教点: %1").arg(load_path), "INFO");
}

void PistonSprayGuiWindow::onCaptureTeachPose() {
    std::string error_message;
    if (!ensureManualTeachBackend(&error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "示教失败", QString::fromStdString(error_message));
        return;
    }

    my_cr5_control::piston::ToolTcpOffsetMm tool_offset;
    tool_offset.x_mm = tcp_offset_x_spin_->value();
    tool_offset.y_mm = tcp_offset_y_spin_->value();
    tool_offset.z_mm = tcp_offset_z_spin_->value();

    const geometry_msgs::msg::Pose flange_pose = manual_motion_interface_->currentPose();
    current_tcp_pose_ =
        my_cr5_control::piston::computeTcpPoseFromFlange(flange_pose, tool_offset);
    saved_teach_tcp_pose_ = current_tcp_pose_;
    updateAdjustedTeachPose();

    appendLog(
        QString("已保存当前末端 TCP 位姿: %1").arg(poseSummaryText(*saved_teach_tcp_pose_)),
        "SUCCESS");
}

void PistonSprayGuiWindow::onMoveToAdjustedTeachPose() {
    if (!adjusted_teach_tcp_pose_.has_value()) {
        QMessageBox::warning(this, "尚未示教", "请先保存当前末端位姿。");
        return;
    }

    std::string error_message;
    if (!ensureManualTeachBackend(&error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "移动失败", QString::fromStdString(error_message));
        return;
    }

    my_cr5_control::piston::ToolTcpOffsetMm tool_offset;
    tool_offset.x_mm = tcp_offset_x_spin_->value();
    tool_offset.y_mm = tcp_offset_y_spin_->value();
    tool_offset.z_mm = tcp_offset_z_spin_->value();

    const geometry_msgs::msg::Pose flange_target =
        my_cr5_control::piston::computeFlangePoseFromTcp(*adjusted_teach_tcp_pose_, tool_offset);
    const bool linear_motion = teach_motion_mode_combo_->currentIndex() == 0;
    if (!manual_motion_interface_->moveToPose(
            flange_target,
            teach_move_speed_spin_->value(),
            linear_motion,
            &error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "移动失败", QString::fromStdString(error_message));
        return;
    }

    const geometry_msgs::msg::Pose current_flange_pose = manual_motion_interface_->currentPose();
    current_tcp_pose_ =
        my_cr5_control::piston::computeTcpPoseFromFlange(current_flange_pose, tool_offset);
    updateManualTeachUi();
    appendLog(
        QString("末端已移动到微调目标: %1").arg(poseSummaryText(*current_tcp_pose_)),
        "SUCCESS");
}

void PistonSprayGuiWindow::onResetTeachOffsets() {
    teach_delta_x_spin_->setValue(0.0);
    teach_delta_y_spin_->setValue(0.0);
    teach_delta_z_spin_->setValue(0.0);
    teach_delta_roll_spin_->setValue(0.0);
    teach_delta_pitch_spin_->setValue(0.0);
    teach_delta_yaw_spin_->setValue(0.0);
    updateAdjustedTeachPose();
}

void PistonSprayGuiWindow::onPublishWorkpieceFrame() {
    PistonSpecMm piston;
    SprayProcessMm process;
    WorkpieceFrame frame;
    my_cr5_control::piston::ToolTcpOffsetMm tool_offset;
    if (!collectInputs(&piston, &process, &frame, &tool_offset)) {
        QMessageBox::warning(this, "参数错误", "工件坐标系参数读取失败。");
        return;
    }

    std::string error_message;
    if (!publishWorkpieceFrameMarkers(
            piston,
            frame,
            rviz_piston_distance_spin_->value(),
            &error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "发布失败", QString::fromStdString(error_message));
        workpiece_frame_status_label_->setText("工件坐标系: 发布失败");
        return;
    }

    const geometry_msgs::msg::Pose cylinder_pose =
        buildPistonCylinderPoseFromFrame(frame, piston.spray_length_mm);
    workpiece_frame_status_label_->setText(
        QString("工件坐标系: 已发布 | origin=(%1,%2,%3) m | 圆柱中心=(%4,%5,%6) m")
            .arg(frame.origin.x, 0, 'f', 4)
            .arg(frame.origin.y, 0, 'f', 4)
            .arg(frame.origin.z, 0, 'f', 4)
            .arg(cylinder_pose.position.x, 0, 'f', 4)
            .arg(cylinder_pose.position.y, 0, 'f', 4)
            .arg(cylinder_pose.position.z, 0, 'f', 4));
    appendLog(
        QString("已发布工件坐标系到 /%1: origin=(%2,%3,%4) m, axial=(%5,%6,%7)")
            .arg(kWorkpieceMarkerTopic)
            .arg(frame.origin.x, 0, 'f', 4)
            .arg(frame.origin.y, 0, 'f', 4)
            .arg(frame.origin.z, 0, 'f', 4)
            .arg(frame.axial_direction.x, 0, 'f', 3)
            .arg(frame.axial_direction.y, 0, 'f', 3)
            .arg(frame.axial_direction.z, 0, 'f', 3),
        "SUCCESS");
}

void PistonSprayGuiWindow::onClearWorkpieceFrame() {
    clearWorkpieceFrameMarkers();
    workpiece_frame_status_label_->setText(
        "工件坐标系: 已清除，RViz 中请添加 MarkerArray 话题 /piston_spray/workpiece_markers");
    appendLog("已清除 RViz 工件坐标系标记。", "INFO");
}

void PistonSprayGuiWindow::onGenerateRvizPiston() {
    PistonSpecMm piston;
    SprayProcessMm process;
    WorkpieceFrame frame;
    my_cr5_control::piston::ToolTcpOffsetMm tool_offset;
    if (!collectInputs(&piston, &process, &frame, &tool_offset)) {
        QMessageBox::warning(this, "参数错误", "工件坐标系参数读取失败。");
        return;
    }

    tf2::Vector3 radial_axis;
    tf2::Vector3 side_axis;
    tf2::Vector3 axial_axis;
    QString frame_error;
    if (!buildWorkpieceAxes(frame, &radial_axis, &side_axis, &axial_axis, &frame_error)) {
        appendLog(frame_error, "ERROR");
        QMessageBox::warning(this, "参数错误", frame_error);
        rviz_piston_status_label_->setText("状态: 工件坐标系参数无效");
        return;
    }

    std::string error_message;
    if (!ensureRvizSceneRobot(&error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "生成失败", QString::fromStdString(error_message));
        rviz_piston_status_label_->setText("状态: 生成失败");
        return;
    }

    const double diameter_mm = piston_diameter_spin_->value();
    const double length_mm = spray_length_spin_->value();
    const double distance_mm = rviz_piston_distance_spin_->value();
    if (diameter_mm <= 0.0 || length_mm <= 0.0) {
        QMessageBox::warning(this, "参数错误", "活塞直径和长度必须大于 0。");
        return;
    }
    const geometry_msgs::msg::Pose cylinder_pose =
        buildPistonCylinderPoseFromFrame(frame, length_mm);
    if (!rviz_scene_robot_->addCylinderObstacle(
            kRvizPistonObjectId,
            cylinder_pose,
            length_mm * kMmToM,
            diameter_mm * 0.5 * kMmToM)) {
        const QString message = "圆柱碰撞体发布失败，请检查 RViz / MoveIt 场景。";
        appendLog(message, "ERROR");
        QMessageBox::warning(this, "生成失败", message);
        rviz_piston_status_label_->setText("状态: 发布失败");
        return;
    }

    std::string marker_error;
    if (!publishWorkpieceFrameMarkers(piston, frame, distance_mm, &marker_error)) {
        appendLog(QString("活塞圆柱已生成，但工件坐标系标记发布失败: %1")
                      .arg(QString::fromStdString(marker_error)),
                  "WARN");
        workpiece_frame_status_label_->setText("工件坐标系: 发布失败");
    } else {
        workpiece_frame_status_label_->setText(
            QString("工件坐标系: 已发布 | origin=(%1,%2,%3) m")
                .arg(frame.origin.x, 0, 'f', 4)
                .arg(frame.origin.y, 0, 'f', 4)
                .arg(frame.origin.z, 0, 'f', 4));
    }

    rviz_piston_status_label_->setText(
        QString("状态: 已生成圆柱 | 直径=%1 mm, 长度=%2 mm, origin=(%3,%4,%5) m")
            .arg(diameter_mm, 0, 'f', 1)
            .arg(length_mm, 0, 'f', 1)
            .arg(frame.origin.x, 0, 'f', 4)
            .arg(frame.origin.y, 0, 'f', 4)
            .arg(frame.origin.z, 0, 'f', 4));
    appendLog(
        QString("已在 RViz 生成活塞圆柱: 直径=%1 mm, 长度=%2 mm, 工件 origin=(%3,%4,%5) m, 轴向=(%6,%7,%8), 喷涂参考距离=%9 mm")
            .arg(diameter_mm, 0, 'f', 1)
            .arg(length_mm, 0, 'f', 1)
            .arg(frame.origin.x, 0, 'f', 4)
            .arg(frame.origin.y, 0, 'f', 4)
            .arg(frame.origin.z, 0, 'f', 4)
            .arg(axial_axis.x(), 0, 'f', 2)
            .arg(axial_axis.y(), 0, 'f', 2)
            .arg(axial_axis.z(), 0, 'f', 2)
            .arg(distance_mm, 0, 'f', 1),
        "SUCCESS");
}

void PistonSprayGuiWindow::onClearRvizPiston() {
    if (!rviz_scene_robot_) {
        rviz_piston_status_label_->setText("状态: 未生成圆柱");
        return;
    }

    rviz_scene_robot_->removeCollisionObject(kRvizPistonObjectId);
    rviz_piston_status_label_->setText("状态: 已清除圆柱");
    appendLog("已从 RViz 规划场景清除活塞圆柱。", "INFO");
}

void PistonSprayGuiWindow::ensureExecutionBackend() {
    const int backend_index = execution_backend_combo_->currentIndex();
    if (execution_engine_ && execution_backend_index_ == backend_index) {
        return;
    }

    execution_backend_index_ = backend_index;
    if (backend_index == 1) {
        motion_interface_ = std::make_shared<CR5MotionInterface>("piston_spray_gui_executor", false);
        peripheral_interface_ = std::make_shared<MockPeripheralInterface>();
    } else {
        motion_interface_ = std::make_shared<MockMotionInterface>();
        peripheral_interface_ = std::make_shared<MockPeripheralInterface>();
    }
    execution_engine_ =
        std::make_shared<SprayExecutionEngine>(motion_interface_, peripheral_interface_);
}

void PistonSprayGuiWindow::updateExecutionUi() {
    ExecutionState state = ExecutionState::Idle;
    if (execution_engine_) {
        state = execution_engine_->state();
    } else if (current_plan_.has_value()) {
        state = ExecutionState::Ready;
    }

    const bool has_plan = current_plan_.has_value();
    const bool running = state == ExecutionState::Running;
    const bool paused = state == ExecutionState::Paused;
    const bool ready_like =
        state == ExecutionState::Idle ||
        state == ExecutionState::Ready ||
        state == ExecutionState::Completed ||
        state == ExecutionState::Stopped ||
        state == ExecutionState::Fault ||
        state == ExecutionState::EmergencyStopped;

    execution_backend_label_->setText(
        execution_backend_combo_->currentIndex() == 1
            ? "后端: CR5位姿执行，转台/喷头仍为模拟接口"
            : "后端: 仿真执行");
    execution_state_label_->setText(
        QString("执行状态: %1").arg(executionStateLabel(state)));
    execution_state_label_->setStyleSheet(
        QString("color:%1; font-weight:bold;").arg(executionStateColor(state)));

    start_execution_btn_->setEnabled(has_plan && ready_like);
    pause_resume_btn_->setEnabled(running || paused);
    pause_resume_btn_->setText(paused ? "继续" : "暂停");
    stop_execution_btn_->setEnabled(has_plan && (running || paused || state == ExecutionState::Ready));
    emergency_stop_btn_->setEnabled(has_plan && state != ExecutionState::Completed);
    updateManualTeachUi();
}

void PistonSprayGuiWindow::handleExecutionEvent(const ExecutionEvent& event) {
    const QString level = (event.state == ExecutionState::Fault ||
                           event.state == ExecutionState::EmergencyStopped)
        ? "ERROR"
        : ((event.state == ExecutionState::Paused || event.state == ExecutionState::Stopped)
            ? "WARN"
            : "INFO");

    appendLog(
        QString("[执行 %1/%2] %3")
            .arg(event.current_step_index)
            .arg(event.total_step_count)
            .arg(QString::fromStdString(event.message)),
        level);
    updateExecutionUi();
}

void PistonSprayGuiWindow::onStartExecution() {
    if (!current_plan_.has_value()) {
        QMessageBox::warning(this, "尚未生成", "请先生成喷涂计划。");
        return;
    }

    ensureExecutionBackend();

    std::string error_message;
    if (!execution_engine_->loadPlan(*current_plan_, &error_message)) {
        QMessageBox::warning(this, "加载失败", QString::fromStdString(error_message));
        appendLog(QString::fromStdString(error_message), "ERROR");
        return;
    }

    const bool started = execution_engine_->start(
        [this](const ExecutionEvent& event) {
            QMetaObject::invokeMethod(
                this,
                [this, event]() { handleExecutionEvent(event); },
                Qt::QueuedConnection);
        },
        &error_message);
    if (!started) {
        QMessageBox::warning(this, "执行失败", QString::fromStdString(error_message));
        appendLog(QString::fromStdString(error_message), "ERROR");
        updateExecutionUi();
        return;
    }

    appendLog(
        execution_backend_combo_->currentIndex() == 1
            ? "已启动 CR5 位姿执行链路，当前转台/喷头仍为模拟接口。"
            : "已启动仿真执行链路。",
        "SUCCESS");
    updateExecutionUi();
}

void PistonSprayGuiWindow::onPauseResumeExecution() {
    if (!execution_engine_) {
        return;
    }

    std::string error_message;
    const ExecutionState state = execution_engine_->state();
    const bool ok = (state == ExecutionState::Paused)
        ? execution_engine_->resume(&error_message)
        : execution_engine_->pause(&error_message);
    if (!ok) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "操作失败", QString::fromStdString(error_message));
        return;
    }
    updateExecutionUi();
}

void PistonSprayGuiWindow::onStopExecution() {
    if (!execution_engine_) {
        return;
    }

    std::string error_message;
    if (!execution_engine_->stop(&error_message)) {
        appendLog(QString::fromStdString(error_message), "ERROR");
        QMessageBox::warning(this, "停止失败", QString::fromStdString(error_message));
        return;
    }
    appendLog("已发送停止请求。", "WARN");
    updateExecutionUi();
}

void PistonSprayGuiWindow::onEmergencyStopExecution() {
    if (!execution_engine_) {
        ensureExecutionBackend();
    }
    execution_engine_->emergencyStop();
    appendLog("已触发急停。", "ERROR");
    updateExecutionUi();
}

void PistonSprayGuiWindow::onExportPlanJson() {
    if (!current_plan_.has_value()) {
        QMessageBox::warning(this, "尚未生成", "请先生成喷涂计划。");
        return;
    }

    const QString path = QFileDialog::getSaveFileName(
        this,
        "导出喷涂计划 JSON",
        defaultExportPath(),
        "JSON (*.json)");
    if (path.isEmpty()) {
        return;
    }

    const auto& plan = *current_plan_;

    QJsonObject root;
    root["job_name"] = jobName();

    QJsonObject piston;
    piston["diameter_mm"] = plan.piston.diameter_mm;
    piston["spray_length_mm"] = plan.piston.spray_length_mm;
    root["piston"] = piston;

    QJsonObject process;
    process["spray_distance_mm"] = plan.process.spray_distance_mm;
    process["spray_width_mm"] = plan.process.spray_width_mm;
    process["overlap_mm"] = plan.process.overlap_mm;
    process["turntable_rpm"] = plan.process.turntable_rpm;
    process["flow_rate_ml_min"] = plan.process.flow_rate_ml_min;
    process["lead_in_mm"] = plan.process.lead_in_mm;
    process["lead_out_mm"] = plan.process.lead_out_mm;
    process["radial_clearance_mm"] = plan.process.radial_clearance_mm;
    process["approach_speed_mm_s"] = plan.process.approach_speed_mm_s;
    process["sample_count"] = plan.process.sample_count;
    root["process"] = process;

    QJsonObject frame;
    frame["origin_m"] = pointToJson(plan.frame.origin);
    frame["axial_direction"] = vectorToJson(plan.frame.axial_direction);
    frame["radial_direction"] = vectorToJson(plan.frame.radial_direction);
    frame["tool_roll_deg"] = plan.frame.tool_roll_deg;
    root["frame"] = frame;

    QJsonObject offset;
    offset["x"] = plan.tool_tcp_offset.x_mm;
    offset["y"] = plan.tool_tcp_offset.y_mm;
    offset["z"] = plan.tool_tcp_offset.z_mm;
    root["tool_tcp_offset_mm"] = offset;

    QJsonObject derived;
    derived["piston_radius_mm"] = plan.metrics.piston_radius_mm;
    derived["nozzle_radius_mm"] = plan.metrics.nozzle_radius_mm;
    derived["helical_pitch_mm_per_rev"] = plan.metrics.helical_pitch_mm_per_rev;
    derived["turntable_rps"] = plan.metrics.turntable_rps;
    derived["axial_feed_mm_s"] = plan.metrics.axial_feed_mm_s;
    derived["surface_speed_mm_s"] = plan.metrics.surface_speed_mm_s;
    derived["total_revolutions"] = plan.metrics.total_revolutions;
    derived["spray_time_s"] = plan.metrics.spray_time_s;
    derived["estimated_graphite_usage_ml"] = plan.metrics.estimated_graphite_usage_ml;
    derived["overlap_ratio"] = plan.metrics.overlap_ratio;
    root["derived"] = derived;

    QJsonArray notices;
    for (const auto& notice : plan.notices) {
        notices.append(QString::fromStdString(notice));
    }
    root["notices"] = notices;

    QJsonArray steps;
    for (const auto& step : plan.execution_steps) {
        QJsonObject step_object;
        step_object["step"] = step.step_index;
        step_object["command"] = QString::fromStdString(step.command);
        if (!step.pose_name.empty()) {
            step_object["pose_name"] = QString::fromStdString(step.pose_name);
        }
        step_object["motion_mode"] = QString::fromStdString(step.motion_mode);
        step_object["speed_mm_s"] = step.speed_mm_s;
        step_object["numeric_value"] = step.numeric_value;
        step_object["note"] = QString::fromStdString(step.note);
        steps.append(step_object);
    }
    root["execution_steps"] = steps;

    QJsonArray key_poses;
    for (const auto& key_pose : plan.key_poses) {
        QJsonObject pose_object;
        pose_object["name"] = QString::fromStdString(key_pose.name);
        pose_object["local_point_mm"] =
            localPointToJson(key_pose.local_x_mm, key_pose.local_y_mm, key_pose.local_z_mm);
        pose_object["tcp_pose_base"] = poseToJson(key_pose.tcp_pose);
        pose_object["flange_pose_base"] = poseToJson(key_pose.flange_pose);
        key_poses.append(pose_object);
    }
    root["key_poses"] = key_poses;

    QJsonArray samples;
    for (const auto& sample : plan.path_samples) {
        QJsonObject sample_object;
        sample_object["time_s"] = sample.time_s;
        sample_object["local_point_mm"] =
            localPointToJson(sample.local_x_mm, sample.local_y_mm, sample.local_z_mm);
        sample_object["surface_angle_deg"] = sample.surface_angle_deg;
        sample_object["tcp_pose_base"] = poseToJson(sample.tcp_pose);
        sample_object["flange_pose_base"] = poseToJson(sample.flange_pose);
        sample_object["surface_point_base_m"] = pointToJson(sample.surface_point_base);
        samples.append(sample_object);
    }
    root["path_samples"] = samples;

    QFile file(path);
    if (!file.open(QIODevice::WriteOnly | QIODevice::Truncate)) {
        QMessageBox::critical(this, "导出失败", "无法写入目标 JSON 文件。");
        return;
    }
    file.write(QJsonDocument(root).toJson(QJsonDocument::Indented));
    file.close();

    appendLog(QString("已导出喷涂计划 JSON: %1").arg(path), "SUCCESS");
}
