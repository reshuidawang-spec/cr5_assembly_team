#ifndef MEASUREMENT_POINT_GENERATOR_HPP
#define MEASUREMENT_POINT_GENERATOR_HPP

#include <vector>
#include <string>
#include <geometry_msgs/msg/point.hpp>
#include <geometry_msgs/msg/pose.hpp>
#include <Eigen/Dense>

namespace measurement {

// 测点类型
enum class PointType {
    HOLE_CENTER,      // 孔中心
    HOLE_EDGE,        // 孔边缘
    SURFACE,          // 表面点
    EDGE,             // 边缘点
    CORNER,           // 角点
    INTERIOR_DEEP,    // 深孔内部
    NARROW_PASSAGE    // 狭窄通道
};

// 测点结构
struct MeasurementPoint {
    geometry_msgs::msg::Point position;
    geometry_msgs::msg::Point normal;  // 表面法向量
    PointType type;
    double difficulty_score;  // 难度评分（0-1）
    std::string description;
};

// 箱体几何特征
struct BoxGeometry {
    Eigen::Vector3d min_corner;
    Eigen::Vector3d max_corner;
    Eigen::Vector3d center;
    double width = 0.0;
    double height = 0.0;
    double depth = 0.0;

    // 孔特征
    struct Hole {
        Eigen::Vector3d center;
        Eigen::Vector3d axis;  // 孔轴方向
        double radius;
        double depth;
    };
    std::vector<Hole> holes;

    // 内腔特征
    struct Cavity {
        Eigen::Vector3d entrance_center;
        Eigen::Vector3d bottom_center;
        double entrance_width;
        double depth;
        double narrowness;  // 狭窄度（0-1）
    };
    std::vector<Cavity> cavities;

    // 外包络面特征
    struct SurfacePatch {
        Eigen::Vector3d center;
        Eigen::Vector3d normal;
        double span_u = 0.0;
        double span_v = 0.0;
        double area = 0.0;
        std::string label;
    };
    std::vector<SurfacePatch> surfaces;

    struct EdgeFeature {
        Eigen::Vector3d start;
        Eigen::Vector3d end;
        Eigen::Vector3d midpoint;
        Eigen::Vector3d direction;
        double length = 0.0;
        std::string label;
    };
    std::vector<EdgeFeature> edges;

    std::vector<Eigen::Vector3d> corners;
};

class MeasurementPointGenerator {
public:
    MeasurementPointGenerator();

    // 从STL文件加载箱体模型
    bool loadSTLModel(const std::string& stl_path);

    // 提取箱体几何特征
    BoxGeometry extractBoxFeatures();

    // 生成测点
    std::vector<MeasurementPoint> generatePoints(
        const BoxGeometry& geometry,
        int num_points_per_feature = 3);

    // 生成分层测试场景（简单、中等、困难）
    struct TestScenario {
        std::string name;
        std::string difficulty;
        std::vector<MeasurementPoint> points;
    };
    std::vector<TestScenario> generateTestScenarios();

    // 计算测点难度评分
    double calculateDifficultyScore(const MeasurementPoint& point,
                                   const BoxGeometry& geometry);

    // 可视化测点（发布到RViz）
    void visualizePoints(const std::vector<MeasurementPoint>& points);

private:
    // STL网格数据
    struct Triangle {
        Eigen::Vector3d v0, v1, v2;
        Eigen::Vector3d normal;
    };
    std::vector<Triangle> mesh_;

    // 辅助函数
    Eigen::Vector3d computeNormal(const Eigen::Vector3d& v0,
                                  const Eigen::Vector3d& v1,
                                  const Eigen::Vector3d& v2);

    bool isPointInsideBox(const Eigen::Vector3d& point,
                         const BoxGeometry& geometry);

    double computeNarrowness(const Eigen::Vector3d& point,
                            const BoxGeometry& geometry);
};

} // namespace measurement

#endif // MEASUREMENT_POINT_GENERATOR_HPP
