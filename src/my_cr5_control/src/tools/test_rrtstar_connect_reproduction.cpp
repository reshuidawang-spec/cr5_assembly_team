#include <rclcpp/rclcpp.hpp>

#include <algorithm>
#include <array>
#include <cmath>
#include <cstddef>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <limits>
#include <optional>
#include <random>
#include <sstream>
#include <string>
#include <utility>
#include <vector>

namespace {

// ============================================================================
// 基础常量和数据结构
// ============================================================================

constexpr double kPi = 3.14159265358979323846;

// 二维向量结构，用于表示平面上的点或方向
struct Vec2 {
    double x{0.0};  // X坐标
    double y{0.0};  // Y坐标
};

// 向量加法运算符
Vec2 operator+(const Vec2& lhs, const Vec2& rhs) {
    return {lhs.x + rhs.x, lhs.y + rhs.y};
}

// 向量减法运算符
Vec2 operator-(const Vec2& lhs, const Vec2& rhs) {
    return {lhs.x - rhs.x, lhs.y - rhs.y};
}

// 向量数乘运算符（向量乘以标量）
Vec2 operator*(const Vec2& value, double scale) {
    return {value.x * scale, value.y * scale};
}

// 向量数除运算符（向量除以标量）
Vec2 operator/(const Vec2& value, double scale) {
    return {value.x / scale, value.y / scale};
}

// 计算两个向量的点积（内积）
double dot(const Vec2& lhs, const Vec2& rhs) {
    return lhs.x * rhs.x + lhs.y * rhs.y;
}

// 计算向量的模长（长度）
double norm(const Vec2& value) {
    return std::sqrt(dot(value, value));
}

// 计算两点之间的欧氏距离
double distance(const Vec2& lhs, const Vec2& rhs) {
    return norm(lhs - rhs);
}

// 将向量归一化为单位向量（长度为1）
Vec2 normalize(const Vec2& value) {
    const double length = norm(value);
    if (length < 1e-9) {  // 避免除以零
        return {0.0, 0.0};
    }
    return value / length;
}

// 将向量旋转指定角度（逆时针为正）
// 使用旋转矩阵: [cos -sin; sin cos]
Vec2 rotate(const Vec2& value, double angle) {
    const double c = std::cos(angle);
    const double s = std::sin(angle);
    return {c * value.x - s * value.y, s * value.x + c * value.y};
}

// 计算路径的总长度（所有线段长度之和）
double pathLength(const std::vector<Vec2>& path) {
    double total = 0.0;
    for (std::size_t i = 1; i < path.size(); ++i) {
        total += distance(path[i - 1], path[i]);
    }
    return total;
}

// 矩形障碍物结构
struct Rect {
    double x_min{0.0};  // 矩形左边界
    double y_min{0.0};  // 矩形下边界
    double x_max{0.0};  // 矩形右边界
    double y_max{0.0};  // 矩形上边界
};

// RRT树的节点结构
struct Node {
    Vec2 point;        // 节点在空间中的位置
    int parent{-1};    // 父节点索引（-1表示根节点）
    double cost{0.0};  // 从根节点到该节点的累计代价
};

// RRT树结构（用于双向RRT*-Connect算法）
struct Tree {
    std::vector<Node> nodes;      // 树中的所有节点
    bool root_is_start{false};    // 标记：根节点是起点(true)还是终点(false)
};

// 路径规划问题定义
struct Problem {
    Vec2 bounds_min{0.0, 0.0};    // 规划空间的最小边界
    Vec2 bounds_max{1.0, 1.0};    // 规划空间的最大边界
    Vec2 start{0.10, 0.10};       // 起点位置
    Vec2 goal{0.92, 0.88};        // 终点位置
    std::vector<Rect> obstacles;  // 障碍物列表
};

// 规划器配置参数（所有参数都经过调优）
struct PlannerConfig {
    // 迭代和连接参数
    int max_iterations{2400};              // 最大迭代次数
    int max_connect_steps{24};             // 连接尝试的最大步数
    int collision_samples_per_meter{50};   // 碰撞检测时每米采样点数
    int area_grid_resolution{12};          // 障碍物面积估计的网格分辨率
    int bezier_samples_per_segment{10};    // Bezier曲线每段的采样点数

    // 步长参数（控制树的生长速度）
    double base_step{0.070};               // 基础步长
    double min_step{0.018};                // 最小步长（靠近障碍物时）
    double max_step{0.095};                // 最大步长（远离障碍物时）

    // 距离阈值
    double near_radius{0.135};             // RRT*重连线的邻域半径
    double connect_tolerance{0.040};       // 判定两树连接成功的距离阈值
    double duplicate_tolerance{0.010};     // 判定节点重复的距离阈值

    // 采样策略参数（论文Eq.(6)相关）
    double p_max{0.90};                    // 目标偏向概率的最大值
    double p_min{0.40};                    // 目标偏向概率的最小值
    double inertia_w1{0.90};               // 惯性权重上界
    double inertia_w2{0.40};               // 惯性权重下界

    // 人工势场（APF）参数（论文Eq.(10)-(15)）
    double safe_distance{0.12};            // 安全距离（障碍物影响范围）
    double attractive_gain{1.10};          // 吸引力增益系数
    double repulsive_gain{0.007};          // 排斥力增益系数

    // 代价函数参数（论文Eq.(9)）
    double obstacle_area_weight{2.50};     // 障碍物面积惩罚权重
    double obstacle_intersection_radius{0.10};  // 障碍物交叠估计半径
};

// 采样状态统计（用于自适应采样策略）
struct SamplingState {
    int random_points_generated{0};        // 已生成的随机采样点总数
    int successful_growth_points{0};       // 成功添加到树中的点数
    double last_success_ratio{1.0};        // 最近的成功率
    double last_wldiw{0.0};                // 最近的线性递减惯性权重
    double last_waiw{0.0};                 // 最近的自适应惯性权重
    double last_target_bias{0.0};          // 最近的目标偏向概率
};

// 论文算法与工程代码的映射关系
struct ProjectMappingEntry {
    const char* paper_component;           // 论文中的算法组件
    const char* this_test_file;            // 本测试文件中的对应函数
    const char* current_project_status;    // 当前工程项目中的实现状态
};

// 障碍物信息（用于人工势场计算）
struct ObstacleInfo {
    double signed_distance{std::numeric_limits<double>::infinity()};  // 有符号距离（正=外部，负=内部）
    Vec2 outward_gradient{0.0, 0.0};                                  // 向外的梯度方向（排斥力方向）
};

// 自适应步长计算结果
struct AdaptiveStepResult {
    double step{0.0};                      // 计算得到的步长
    double attractive_norm{0.0};           // 吸引力的模长
    double repulsive_norm{0.0};            // 排斥力的模长
    double clearance{std::numeric_limits<double>::infinity()};  // 到最近障碍物的距离
    Vec2 direction{0.0, 0.0};              // 最终的生长方向
};

// 树扩展操作的结果
struct ExtendResult {
    bool added{false};      // 是否成功添加了新节点
    int node_index{-1};     // 新添加节点的索引（-1表示未添加）
};

// 树连接操作的结果
struct ConnectResult {
    bool connected{false};      // 是否成功连接到目标
    int other_tree_index{-1};   // 另一棵树中连接点的索引
    int connect_steps{0};       // 连接过程中的步数
};

// 路径规划的最终结果
struct PlanResult {
    bool success{false};                   // 规划是否成功
    int iterations{0};                     // 实际使用的迭代次数
    int start_tree_nodes{0};               // 起点树的节点数
    int goal_tree_nodes{0};                // 终点树的节点数
    std::vector<Vec2> raw_path;            // 原始路径（未平滑）
    std::vector<Vec2> smoothed_path;       // 平滑后的路径
    SamplingState sampling_state;          // 采样统计信息
    Tree start_tree_snapshot;              // 起点树快照（用于可视化）
    Tree goal_tree_snapshot;               // 终点树快照（用于可视化）
};

// ============================================================================
// 碰撞检测和几何计算函数
// ============================================================================

// 检查点是否在规划空间的边界内
bool isInsideBounds(const Vec2& point, const Problem& problem) {
    return point.x >= problem.bounds_min.x && point.x <= problem.bounds_max.x &&
           point.y >= problem.bounds_min.y && point.y <= problem.bounds_max.y;
}

// 检查点是否在矩形内部
bool isInsideRect(const Vec2& point, const Rect& rect) {
    return point.x >= rect.x_min && point.x <= rect.x_max &&
           point.y >= rect.y_min && point.y <= rect.y_max;
}

// 检查点是否与障碍物碰撞（包括边界检查）
bool collides(const Vec2& point, const Problem& problem) {
    // 首先检查是否超出边界
    if (!isInsideBounds(point, problem)) {
        return true;
    }
    // 然后检查是否与任何障碍物相交
    for (const auto& obstacle : problem.obstacles) {
        if (isInsideRect(point, obstacle)) {
            return true;
        }
    }
    return false;
}

// 计算点到矩形的最近点（投影到矩形边界）
Vec2 closestPointOnRect(const Vec2& point, const Rect& rect) {
    return {
        std::clamp(point.x, rect.x_min, rect.x_max),  // X坐标限制在矩形范围内
        std::clamp(point.y, rect.y_min, rect.y_max)   // Y坐标限制在矩形范围内
    };
}

// 计算点到矩形的有符号距离和梯度（用于人工势场）
// 返回值：正数=点在矩形外部，负数=点在矩形内部
ObstacleInfo signedDistanceToRect(const Vec2& point, const Rect& rect) {
    ObstacleInfo info;
    const bool inside = isInsideRect(point, rect);

    // 情况1：点在矩形外部
    if (!inside) {
        const Vec2 closest = closestPointOnRect(point, rect);
        const Vec2 delta = point - closest;
        const double dist = norm(delta);
        info.signed_distance = dist;  // 正距离
        // 梯度指向远离矩形的方向
        info.outward_gradient = dist < 1e-9 ? Vec2{1.0, 0.0} : delta / dist;
        return info;
    }

    // 情况2：点在矩形内部，计算到各边的距离
    const double to_left = point.x - rect.x_min;      // 到左边的距离
    const double to_right = rect.x_max - point.x;     // 到右边的距离
    const double to_bottom = point.y - rect.y_min;    // 到下边的距离
    const double to_top = rect.y_max - point.y;       // 到上边的距离

    // 找到最近的边（最小穿透深度）
    const double min_penetration = std::min({to_left, to_right, to_bottom, to_top});
    info.signed_distance = -min_penetration;  // 负距离表示在内部

    // 根据最近的边设置梯度方向（指向外部）
    if (min_penetration == to_left) {
        info.outward_gradient = {-1.0, 0.0};  // 向左
    } else if (min_penetration == to_right) {
        info.outward_gradient = {1.0, 0.0};   // 向右
    } else if (min_penetration == to_bottom) {
        info.outward_gradient = {0.0, -1.0};  // 向下
    } else {
        info.outward_gradient = {0.0, 1.0};   // 向上
    }
    return info;
}

// 找到距离给定点最近的障碍物信息
ObstacleInfo nearestObstacleInfo(const Vec2& point, const Problem& problem) {
    ObstacleInfo best;
    for (const auto& obstacle : problem.obstacles) {
        const ObstacleInfo current = signedDistanceToRect(point, obstacle);
        if (current.signed_distance < best.signed_distance) {
            best = current;
        }
    }
    return best;
}

// 检查线段是否与障碍物碰撞（通过密集采样检测）
bool segmentCollides(const Vec2& a, const Vec2& b, const Problem& problem, int samples_per_meter) {
    const double segment_length = distance(a, b);
    // 根据线段长度计算采样点数（至少2个点）
    const int samples = std::max(2, static_cast<int>(std::ceil(segment_length * samples_per_meter)));

    // 在线段上均匀采样并检查每个点
    for (int i = 0; i <= samples; ++i) {
        const double t = static_cast<double>(i) / static_cast<double>(samples);
        const Vec2 point{a.x + (b.x - a.x) * t, a.y + (b.y - a.y) * t};
        if (collides(point, problem)) {
            return true;
        }
    }
    return false;
}

// 估计圆形区域与障碍物的交叠面积（用于代价函数）
// 使用网格采样方法近似计算
double estimateObstacleIntersectionArea(const Vec2& center,
                                        double radius,
                                        const Problem& problem,
                                        int grid_resolution) {
    const double side = 2.0 * radius;  // 正方形边长
    const double cell = side / static_cast<double>(grid_resolution);  // 网格单元大小
    double area = 0.0;

    // 遍历网格中的每个单元
    for (int ix = 0; ix < grid_resolution; ++ix) {
        for (int iy = 0; iy < grid_resolution; ++iy) {
            // 计算网格单元中心点坐标
            const double x = center.x - radius + (static_cast<double>(ix) + 0.5) * cell;
            const double y = center.y - radius + (static_cast<double>(iy) + 0.5) * cell;
            const Vec2 sample{x, y};

            // 只考虑圆形区域内的点
            if (distance(sample, center) > radius) {
                continue;
            }

            // 检查该点是否在任何障碍物内
            for (const auto& obstacle : problem.obstacles) {
                if (isInsideRect(sample, obstacle)) {
                    area += cell * cell;  // 累加单元面积
                    break;
                }
            }
        }
    }

    return area;
}

// 计算边的代价（论文Eq.(9)）
// 代价 = 欧氏距离 + 障碍物面积惩罚
double edgeCost(const Vec2& parent,
                const Vec2& child,
                const Problem& problem,
                const PlannerConfig& config) {
    // Eq. (9): C(X) = C(parent) + d(parent, X) + w_area * Intersection(parent, X)
    // 论文文字里 w_area 既像半径又像权重。这里拆成两个工程参数：
    // 1) obstacle_intersection_radius: 局部障碍交叠估计半径
    // 2) obstacle_area_weight: 在总代价里的惩罚权重
    const double intersection_area = estimateObstacleIntersectionArea(
        parent, config.obstacle_intersection_radius, problem, config.area_grid_resolution);
    return distance(parent, child) + config.obstacle_area_weight * intersection_area;
}

// ============================================================================
// 自适应采样策略（论文Eq.(4)-(6)）
// ============================================================================

// 计算采样成功率
double successRatio(const SamplingState& sampling_state) {
    if (sampling_state.random_points_generated <= 0) {
        return 1.0;
    }
    return static_cast<double>(sampling_state.successful_growth_points) /
           static_cast<double>(sampling_state.random_points_generated);
}

// 计算线性递减惯性权重（LDIW: Linearly Decreasing Inertia Weight）
// 随着迭代次数增加，权重从w1线性递减到w2
double computeWldiw(int iteration, const PlannerConfig& config) {
    return (config.inertia_w1 - config.inertia_w2) *
               (static_cast<double>(config.max_iterations - iteration) /
                static_cast<double>(config.max_iterations)) +
           config.inertia_w2;
}

// 计算自适应惯性权重（AIWS: Adaptive Inertia Weight Strategy）
// 根据采样成功率动态调整权重
double computeWaiw(const SamplingState& sampling_state, const PlannerConfig& config) {
    return ((config.inertia_w1 - config.inertia_w2) * successRatio(sampling_state)) +
           config.inertia_w2;
}

// 计算目标偏向概率（论文Eq.(6)）
// 成功率越高、迭代越接近末期，目标偏向概率越大
double computeTargetBias(int iteration,
                         const SamplingState& sampling_state,
                         const PlannerConfig& config) {
    // Eq. (6) 的 OCR 排版较乱，这里按论文文字含义做规范化：
    // 成长成功率越高、迭代越接近末期，Pa 越大；在拥挤场景下 Pa 会被压低。
    const double ps = std::max(0.05, successRatio(sampling_state));  // 避免除以零
    const double remaining =
        static_cast<double>(config.max_iterations - iteration) / static_cast<double>(config.max_iterations);
    const double raw = 1.0 -
                       ((config.p_max - config.p_min) / ps +
                        (config.p_min / ps) * remaining);
    return std::clamp(raw, 0.0, 1.0);  // 限制在[0,1]范围内
}

// 全局均匀采样（在整个规划空间内随机采样）
Vec2 sampleGlobal(const Problem& problem, std::mt19937& rng) {
    std::uniform_real_distribution<double> x_dist(problem.bounds_min.x, problem.bounds_max.x);
    std::uniform_real_distribution<double> y_dist(problem.bounds_min.y, problem.bounds_max.y);
    return {x_dist(rng), y_dist(rng)};
}

// 椭球约束采样（论文Eq.(2)）
// 在以起点和终点为焦点的椭球内采样，提高采样效率
Vec2 sampleEllipsoid(const Problem& problem, std::mt19937& rng) {
    const Vec2 axis = problem.goal - problem.start;
    const double d = std::max(distance(problem.start, problem.goal), 1e-6);  // 焦距
    const Vec2 center = (problem.start + problem.goal) * 0.5;  // 椭球中心
    const double angle = std::atan2(axis.y, axis.x);  // 椭球旋转角度
    std::uniform_real_distribution<double> unit(0.0, 1.0);
    std::uniform_real_distribution<double> theta_dist(0.0, 2.0 * kPi);

    // 尝试多次采样，确保点在规划空间内
    for (int attempt = 0; attempt < 64; ++attempt) {
        const double r = std::sqrt(unit(rng));  // 极坐标半径
        const double theta = theta_dist(rng);   // 极坐标角度
        // Eq. (2): full major/minor axis = 3d / 0.8d, so semi-axis = 1.5d / 0.4d.
        // 长轴半径=1.5d，短轴半径=0.4d
        const Vec2 local{1.5 * d * r * std::cos(theta), 0.4 * d * r * std::sin(theta)};
        const Vec2 sample = center + rotate(local, angle);  // 旋转到正确方向
        if (isInsideBounds(sample, problem)) {
            return sample;
        }
    }

    // 如果多次尝试失败，回退到全局采样
    return sampleGlobal(problem, rng);
}

// 目标导向的自适应采样（综合多种采样策略）
// 根据当前迭代状态和成功率，动态选择采样策略
Vec2 sampleTarget(int iteration,
                  const Problem& problem,
                  const PlannerConfig& config,
                  SamplingState& sampling_state,
                  std::mt19937& rng) {
    std::uniform_real_distribution<double> unit(0.0, 1.0);

    // 更新采样统计信息
    sampling_state.last_success_ratio = successRatio(sampling_state);
    sampling_state.last_wldiw = computeWldiw(iteration, config);
    sampling_state.last_waiw = computeWaiw(sampling_state, config);
    sampling_state.last_target_bias = computeTargetBias(iteration, sampling_state, config);

    // 策略1：以一定概率直接采样目标点（加速收敛）
    if (unit(rng) < sampling_state.last_target_bias) {
        return problem.goal;
    }

    // 策略2：在椭球内采样（启发式搜索）
    // 椭球概率由两种惯性权重的平均值决定
    const double ellipse_probability =
        std::clamp(0.5 * (sampling_state.last_wldiw + sampling_state.last_waiw), 0.15, 0.95);
    if (unit(rng) < ellipse_probability) {
        return sampleEllipsoid(problem, rng);
    }

    // 策略3：全局均匀采样（保持探索能力）
    return sampleGlobal(problem, rng);
}

// ============================================================================
// 自适应步长计算（基于人工势场APF，论文Eq.(10)-(15)）
// ============================================================================

// 计算自适应步长和生长方向
// 结合目标吸引力和障碍物排斥力，动态调整步长
AdaptiveStepResult computeAdaptiveStep(const Vec2& x_near,
                                       const Vec2& x_target,
                                       const Vec2& attractor,
                                       const Problem& problem,
                                       const PlannerConfig& config) {
    AdaptiveStepResult result;
    const Vec2 to_target = normalize(x_target - x_near);  // 指向采样点的方向
    const Vec2 goal_dir = normalize(attractor - x_near);  // 指向目标的方向
    const double d_goal = distance(x_near, attractor);    // 到目标的距离

    // Eq. (10) - (15): APF 风格的自适应步长。

    // 步骤1：计算吸引力（指向目标）
    const Vec2 attractive_force = goal_dir * (config.attractive_gain * d_goal);
    result.attractive_norm = norm(attractive_force);

    // 步骤2：计算排斥力（远离障碍物）
    const ObstacleInfo obstacle = nearestObstacleInfo(x_near, problem);
    result.clearance = obstacle.signed_distance;

    Vec2 repulsive_force{0.0, 0.0};
    if (obstacle.signed_distance <= config.safe_distance) {
        // 只有在安全距离内才施加排斥力
        const double d = std::max(std::abs(obstacle.signed_distance), 1e-3);  // 避免除以零
        // 排斥力大小与距离的平方成反比
        const double repulsive_magnitude =
            config.repulsive_gain * (1.0 / d - 1.0 / config.safe_distance) / (d * d);
        repulsive_force = obstacle.outward_gradient * repulsive_magnitude;
    }
    result.repulsive_norm = norm(repulsive_force);

    // 步骤3：融合方向（采样方向 + APF方向）
    Vec2 blended_direction = to_target;
    const Vec2 apf_direction = normalize(attractive_force + repulsive_force);
    if (norm(apf_direction) > 1e-9) {
        // 将APF方向以35%的权重混合到采样方向中
        blended_direction = normalize(to_target + apf_direction * 0.35);
    }
    if (norm(blended_direction) < 1e-9) {
        blended_direction = goal_dir;  // 回退到目标方向
    }
    result.direction = blended_direction;

    // 步骤4：根据障碍物距离调整步长
    if (obstacle.signed_distance > config.safe_distance) {
        // 远离障碍物：使用较大步长
        result.step = std::min(config.max_step, config.base_step * 1.30);
        return result;
    }

    // Eq. (15) 原文在 PDF 中有排版歧义。这里保留其核心项的同时，
    // 按论文文字描述做成"越靠近障碍，步长越小"的规范化版本。
    // 靠近障碍物：步长随距离减小
    const double log_term = std::log1p(
        result.attractive_norm *
        (std::sqrt(std::max(0.0, 2.0 * result.repulsive_norm /
                                     std::max(config.repulsive_gain, 1e-9))) +
         1.0 / config.safe_distance));
    result.step = std::clamp(
        config.base_step / (1.0 + log_term),
        config.min_step,
        config.max_step);
    return result;
}

// ============================================================================
// RRT*树操作函数
// ============================================================================

// 找到树中距离给定采样点最近的节点
int nearestNodeIndex(const Tree& tree, const Vec2& sample) {
    int best_index = 0;
    double best_distance = std::numeric_limits<double>::infinity();
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        const double current_distance = distance(tree.nodes[i].point, sample);
        if (current_distance < best_distance) {
            best_distance = current_distance;
            best_index = static_cast<int>(i);
        }
    }
    return best_index;
}

// 找到树中在给定半径内的所有节点（用于RRT*重连线）
std::vector<int> nearNodeIndices(const Tree& tree, const Vec2& point, double radius) {
    std::vector<int> near;
    for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
        if (distance(tree.nodes[i].point, point) <= radius) {
            near.push_back(static_cast<int>(i));
        }
    }
    return near;
}

// 检查新节点是否与树中已有节点重复（避免冗余节点）
bool isDuplicateNode(const Tree& tree, const Vec2& point, double tolerance) {
    for (const auto& node : tree.nodes) {
        if (distance(node.point, point) < tolerance) {
            return true;
        }
    }
    return false;
}

// 向目标方向扩展树（RRT*的核心操作）
// 包含：最近邻搜索、自适应步长、碰撞检测、父节点选择、重连线优化
ExtendResult extendTreeToward(Tree& tree,
                              const Vec2& target,
                              const Problem& problem,
                              const PlannerConfig& config) {
    ExtendResult result;

    // 步骤1：找到树中距离目标最近的节点
    const int nearest_index = nearestNodeIndex(tree, target);

    // 步骤2：确定吸引点（起点树吸引向终点，终点树吸引向起点）
    const Vec2 attractor = tree.root_is_start ? problem.goal : problem.start;

    // 步骤3：计算自适应步长和生长方向
    const AdaptiveStepResult adaptive_step =
        computeAdaptiveStep(tree.nodes[nearest_index].point, target, attractor, problem, config);

    // 步骤4：生成新节点位置
    const Vec2 new_point = tree.nodes[nearest_index].point + adaptive_step.direction * adaptive_step.step;

    // 步骤5：碰撞检测（点和线段）
    if (!isInsideBounds(new_point, problem) || collides(new_point, problem)) {
        return result;  // 新点碰撞，扩展失败
    }
    if (segmentCollides(tree.nodes[nearest_index].point, new_point, problem,
                        config.collision_samples_per_meter)) {
        return result;  // 连接线段碰撞，扩展失败
    }

    // 步骤6：检查节点重复
    if (isDuplicateNode(tree, new_point, config.duplicate_tolerance)) {
        return result;  // 节点重复，扩展失败
    }

    // 步骤7：RRT*父节点选择（在邻域内寻找代价最小的父节点）
    int best_parent = nearest_index;
    double best_cost = tree.nodes[nearest_index].cost +
                       edgeCost(tree.nodes[nearest_index].point, new_point, problem, config);

    const auto near = nearNodeIndices(tree, new_point, config.near_radius);
    for (const int index : near) {
        // 检查连接是否无碰撞
        if (segmentCollides(tree.nodes[index].point, new_point, problem,
                            config.collision_samples_per_meter)) {
            continue;
        }
        // 计算通过该节点的总代价
        const double candidate_cost =
            tree.nodes[index].cost + edgeCost(tree.nodes[index].point, new_point, problem, config);
        if (candidate_cost < best_cost) {
            best_cost = candidate_cost;
            best_parent = index;
        }
    }

    // 步骤8：添加新节点到树中
    tree.nodes.push_back({new_point, best_parent, best_cost});
    result.added = true;
    result.node_index = static_cast<int>(tree.nodes.size() - 1);

    // 步骤9：RRT*重连线（尝试将邻域内的节点重连到新节点以降低代价）
    for (const int index : near) {
        if (index == best_parent) {
            continue;  // 跳过父节点
        }
        if (segmentCollides(new_point, tree.nodes[index].point, problem,
                            config.collision_samples_per_meter)) {
            continue;  // 连接碰撞，跳过
        }
        // 计算通过新节点的代价
        const double rewired_cost =
            best_cost + edgeCost(new_point, tree.nodes[index].point, problem, config);
        if (rewired_cost + 1e-9 < tree.nodes[index].cost) {
            // 重连线可以降低代价，更新父节点
            tree.nodes[index].parent = result.node_index;
            tree.nodes[index].cost = rewired_cost;
        }
    }

    return result;
}

// 尝试将树连接到目标点（RRT-Connect的核心操作）
// 通过多步扩展，尝试快速接近目标
ConnectResult connectTreeToward(Tree& tree,
                                const Vec2& target,
                                const Problem& problem,
                                const PlannerConfig& config) {
    ConnectResult result;
    double previous_distance = std::numeric_limits<double>::infinity();

    // 最多尝试max_connect_steps步
    for (int step = 0; step < config.max_connect_steps; ++step) {
        // 检查是否还在接近目标（如果距离不再减小，停止尝试）
        const int before_nearest = nearestNodeIndex(tree, target);
        const double before_distance = distance(tree.nodes[before_nearest].point, target);
        if (before_distance >= previous_distance - 1e-9) {
            break;  // 距离不再减小，停止
        }
        previous_distance = before_distance;

        // 向目标扩展一步
        const ExtendResult extend = extendTreeToward(tree, target, problem, config);
        if (!extend.added) {
            break;  // 扩展失败（碰撞或其他原因），停止
        }

        result.connect_steps = step + 1;
        result.other_tree_index = extend.node_index;

        // 检查是否已经足够接近目标（在容差范围内）
        const double after_distance = distance(tree.nodes[extend.node_index].point, target);
        if (after_distance <= config.connect_tolerance &&
            !segmentCollides(tree.nodes[extend.node_index].point, target, problem,
                             config.collision_samples_per_meter)) {
            result.connected = true;  // 连接成功！
            return result;
        }
    }

    return result;  // 未能在最大步数内连接
}

// ============================================================================
// 路径提取和处理函数
// ============================================================================

// 从树中提取从指定节点到根节点的路径
std::vector<Vec2> traceToRoot(const Tree& tree, int node_index) {
    std::vector<Vec2> path;
    int current = node_index;
    // 沿着父节点指针回溯到根节点
    while (current >= 0) {
        path.push_back(tree.nodes[current].point);
        current = tree.nodes[current].parent;
    }
    // 反转路径（从根到叶）
    std::reverse(path.begin(), path.end());
    return path;
}

// 合并两棵树的路径（双向RRT*-Connect的最终步骤）
std::vector<Vec2> mergePaths(const Tree& tree_a,
                             int index_a,
                             const Tree& tree_b,
                             int index_b) {
    const Tree* start_tree = nullptr;
    int start_index = -1;
    const Tree* goal_tree = nullptr;
    int goal_index = -1;

    // 确定哪棵树是起点树，哪棵是终点树
    if (tree_a.root_is_start) {
        start_tree = &tree_a;
        start_index = index_a;
        goal_tree = &tree_b;
        goal_index = index_b;
    } else {
        start_tree = &tree_b;
        start_index = index_b;
        goal_tree = &tree_a;
        goal_index = index_a;
    }

    // 提取两段路径
    std::vector<Vec2> start_path = traceToRoot(*start_tree, start_index);
    std::vector<Vec2> goal_path = traceToRoot(*goal_tree, goal_index);
    std::reverse(goal_path.begin(), goal_path.end());  // 反转终点路径

    // 合并路径（起点→连接点→终点）
    std::vector<Vec2> merged = start_path;
    for (const auto& point : goal_path) {
        // 避免重复点
        if (merged.empty() || distance(merged.back(), point) > 1e-9) {
            merged.push_back(point);
        }
    }
    return merged;
}

// 将当前双树状态标准化为“起点树 + 终点树”快照，便于结果导出和可视化。
void assignTreeSnapshots(PlanResult& result, const Tree& tree_a, const Tree& tree_b) {
    if (tree_a.root_is_start) {
        result.start_tree_snapshot = tree_a;
        result.goal_tree_snapshot = tree_b;
    } else {
        result.start_tree_snapshot = tree_b;
        result.goal_tree_snapshot = tree_a;
    }
}

// 检查折线路径是否与障碍物碰撞
bool polylineCollides(const std::vector<Vec2>& path,
                      const Problem& problem,
                      int samples_per_meter) {
    for (std::size_t i = 1; i < path.size(); ++i) {
        if (segmentCollides(path[i - 1], path[i], problem, samples_per_meter)) {
            return true;
        }
    }
    return false;
}

// ============================================================================
// 路径平滑（使用三次Bezier曲线，论文Eq.(18)）
// ============================================================================

// 计算三次Bezier曲线上的点
// B(t) = (1-t)³P0 + 3(1-t)²tP1 + 3(1-t)t²P2 + t³P3
Vec2 cubicBezier(const Vec2& p0,
                 const Vec2& p1,
                 const Vec2& p2,
                 const Vec2& p3,
                 double t) {
    const double u = 1.0 - t;
    const double b0 = u * u * u;        // (1-t)³
    const double b1 = 3.0 * t * u * u;  // 3(1-t)²t
    const double b2 = 3.0 * t * t * u;  // 3(1-t)t²
    const double b3 = t * t * t;        // t³
    return {b0 * p0.x + b1 * p1.x + b2 * p2.x + b3 * p3.x,
            b0 * p0.y + b1 * p1.y + b2 * p2.y + b3 * p3.y};
}

// 使用分段三次Bezier曲线平滑路径
// 每4个连续点生成一段Bezier曲线
std::vector<Vec2> smoothPathWithBezier(const std::vector<Vec2>& raw_path,
                                       const Problem& problem,
                                       const PlannerConfig& config) {
    // 路径太短，无需平滑
    if (raw_path.size() < 4) {
        return raw_path;
    }

    std::vector<Vec2> smoothed{raw_path.front()};  // 保留起点
    std::size_t segment_start = 0;

    // 每次处理4个点，生成一段Bezier曲线
    while (segment_start + 3 < raw_path.size()) {
        // 在Bezier曲线上采样点
        std::vector<Vec2> candidate_segment;
        candidate_segment.reserve(static_cast<std::size_t>(config.bezier_samples_per_segment) + 1);
        for (int i = 0; i <= config.bezier_samples_per_segment; ++i) {
            const double t =
                static_cast<double>(i) / static_cast<double>(config.bezier_samples_per_segment);
            candidate_segment.push_back(
                cubicBezier(raw_path[segment_start],
                            raw_path[segment_start + 1],
                            raw_path[segment_start + 2],
                            raw_path[segment_start + 3],
                            t));
        }

        // 检查平滑后的曲线是否无碰撞
        if (!polylineCollides(candidate_segment, problem, config.collision_samples_per_meter)) {
            // 无碰撞，使用平滑曲线
            for (std::size_t i = 1; i < candidate_segment.size(); ++i) {
                smoothed.push_back(candidate_segment[i]);
            }
        } else {
            // 有碰撞，保留原始折线
            for (std::size_t i = segment_start + 1; i <= segment_start + 3; ++i) {
                smoothed.push_back(raw_path[i]);
            }
        }
        segment_start += 3;  // 移动到下一段（重叠1个点）
    }

    // 添加剩余的点
    for (std::size_t i = segment_start + 1; i < raw_path.size(); ++i) {
        if (distance(smoothed.back(), raw_path[i]) > 1e-9) {
            smoothed.push_back(raw_path[i]);
        }
    }
    return smoothed;
}

// ============================================================================
// 主规划算法：双向RRT*-Connect（论文核心算法）
// ============================================================================

// 双向RRT*-Connect路径规划算法
// 结合了RRT*的渐进最优性和RRT-Connect的快速连接能力
PlanResult planBidirectionalRrtStarConnect(const Problem& problem,
                                           const PlannerConfig& config,
                                           std::mt19937& rng) {
    PlanResult result;

    // 初始化两棵树：起点树和终点树
    Tree start_tree{{Node{problem.start, -1, 0.0}}, true};   // 根节点是起点
    Tree goal_tree{{Node{problem.goal, -1, 0.0}}, false};    // 根节点是终点
    SamplingState sampling_state;

    // 主循环：交替扩展两棵树
    for (int iteration = 0; iteration < config.max_iterations; ++iteration) {
        // 步骤1：生成采样点（使用自适应采样策略）
        sampling_state.random_points_generated += 1;
        const Vec2 sample = sampleTarget(iteration, problem, config, sampling_state, rng);

        // 步骤2：向采样点扩展当前树（start_tree）
        ExtendResult grow = extendTreeToward(start_tree, sample, problem, config);
        if (!grow.added) {
            // 扩展失败，交换两棵树，下次迭代扩展另一棵树
            std::swap(start_tree, goal_tree);
            continue;
        }
        sampling_state.successful_growth_points += 1;  // 记录成功扩展

        // 步骤3：尝试将另一棵树（goal_tree）连接到新节点
        const Vec2 new_point = start_tree.nodes[grow.node_index].point;
        const ConnectResult connect = connectTreeToward(goal_tree, new_point, problem, config);

        // 步骤4：检查是否成功连接两棵树
        if (connect.connected) {
            // 成功！提取并平滑路径
            result.success = true;
            result.iterations = iteration + 1;
            result.raw_path = mergePaths(start_tree, grow.node_index, goal_tree, connect.other_tree_index);
            result.smoothed_path = smoothPathWithBezier(result.raw_path, problem, config);
            assignTreeSnapshots(result, start_tree, goal_tree);

            // 记录树的大小
            result.start_tree_nodes = start_tree.root_is_start ?
                                      static_cast<int>(start_tree.nodes.size()) :
                                      static_cast<int>(goal_tree.nodes.size());
            result.goal_tree_nodes = start_tree.root_is_start ?
                                     static_cast<int>(goal_tree.nodes.size()) :
                                     static_cast<int>(start_tree.nodes.size());
            result.sampling_state = sampling_state;
            return result;
        }

        // 步骤5：交换两棵树，下次迭代扩展另一棵树（保持平衡生长）
        std::swap(start_tree, goal_tree);
    }

    // 达到最大迭代次数仍未找到路径
    result.success = false;
    result.iterations = config.max_iterations;
    result.start_tree_nodes = start_tree.root_is_start ?
                              static_cast<int>(start_tree.nodes.size()) :
                              static_cast<int>(goal_tree.nodes.size());
    result.goal_tree_nodes = start_tree.root_is_start ?
                             static_cast<int>(goal_tree.nodes.size()) :
                             static_cast<int>(start_tree.nodes.size());
    result.sampling_state = sampling_state;
    assignTreeSnapshots(result, start_tree, goal_tree);
    return result;
}

// ============================================================================
// 辅助函数：问题构建、文件输出、格式化
// ============================================================================

// 构建默认测试问题（包含4个矩形障碍物）
Problem buildDefaultProblem() {
    Problem problem;
    problem.obstacles = {
        {0.24, 0.18, 0.40, 0.74},  // 障碍物1
        {0.50, 0.00, 0.62, 0.46},  // 障碍物2
        {0.50, 0.58, 0.62, 1.00},  // 障碍物3
        {0.74, 0.30, 0.82, 0.78}   // 障碍物4
    };
    return problem;
}

// 构建论文算法与工程代码的映射关系表
std::vector<ProjectMappingEntry> buildProjectMapping() {
    return {
        {
            "椭球约束采样 / target-biased sampling",
            "sampleEllipsoid() + sampleTarget()",
            "现有工程已有近似实现: CR5Robot::generateEllipsoidGuideSamples()"
        },
        {
            "改进代价函数 Eq.(9)",
            "edgeCost() + estimateObstacleIntersectionArea()",
            "现有工程已有近似实现: computeObstacleAreaPenalty() + computeImprovedPathCost()"
        },
        {
            "APF 自适应步长 Eq.(10)-(15)",
            "computeAdaptiveStep()",
            "现有工程尚未形成完整 RRT*-Connect 节点扩展器"
        },
        {
            "双向 RRT*-Connect 树生长",
            "planBidirectionalRrtStarConnect()",
            "现有工程 HeuristicGuided 当前是外部两阶段 guide-candidate 原型，不是完整双树规划器"
        },
        {
            "分段三次 Bezier 平滑 Eq.(18)",
            "smoothPathWithBezier()",
            "现有工程暂无对应通用平滑模块"
        }
    };
}

// 将路径数据写入CSV文件（用于可视化）
void writePathCsv(const std::filesystem::path& path,
                  const std::vector<Vec2>& raw_path,
                  const std::vector<Vec2>& smoothed_path) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << "path_type,index,x,y\n";  // CSV表头
    // 写入原始路径
    for (std::size_t i = 0; i < raw_path.size(); ++i) {
        out << "raw," << i << "," << std::fixed << std::setprecision(6)
            << raw_path[i].x << "," << raw_path[i].y << "\n";
    }
    // 写入平滑路径
    for (std::size_t i = 0; i < smoothed_path.size(); ++i) {
        out << "smoothed," << i << "," << std::fixed << std::setprecision(6)
            << smoothed_path[i].x << "," << smoothed_path[i].y << "\n";
    }
}

// 将两棵树的节点和父子关系写入CSV文件（用于Python可视化）。
void writeTreeCsv(const std::filesystem::path& path,
                  const Tree& start_tree,
                  const Tree& goal_tree) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    out << "tree_type,node_index,parent_index,x,y,cost\n";

    const auto write_tree = [&out](const Tree& tree, const char* tree_type) {
        for (std::size_t i = 0; i < tree.nodes.size(); ++i) {
            const auto& node = tree.nodes[i];
            out << tree_type << "," << i << "," << node.parent << ","
                << std::fixed << std::setprecision(6)
                << node.point.x << "," << node.point.y << "," << node.cost << "\n";
        }
    };

    write_tree(start_tree, "start");
    write_tree(goal_tree, "goal");
}

// 写入规划结果摘要（包含论文信息和性能指标）
void writeSummary(const std::filesystem::path& path,
                  const PlanResult& result,
                  const Problem& problem,
                  const PlannerConfig& config) {
    std::filesystem::create_directories(path.parent_path());
    std::ofstream out(path);
    // 论文信息
    out << "paper_title=A novel RRT*-Connect algorithm for path planning on robotic arm collision avoidance\n";
    out << "paper_journal=Scientific Reports\n";
    out << "paper_year=2025\n";
    out << "equations=Eq.(2),(4)-(9),(10)-(15),(18)\n";
    // 规划结果
    out << "success=" << (result.success ? 1 : 0) << "\n";
    out << "iterations=" << result.iterations << "\n";
    out << "start_tree_nodes=" << result.start_tree_nodes << "\n";
    out << "goal_tree_nodes=" << result.goal_tree_nodes << "\n";
    out << std::fixed << std::setprecision(6);
    out << "raw_path_length=" << pathLength(result.raw_path) << "\n";
    out << "smoothed_path_length=" << pathLength(result.smoothed_path) << "\n";
    // 采样统计
    out << "sampling_success_ratio=" << successRatio(result.sampling_state) << "\n";
    out << "last_wldiw=" << result.sampling_state.last_wldiw << "\n";
    out << "last_waiw=" << result.sampling_state.last_waiw << "\n";
    out << "last_target_bias=" << result.sampling_state.last_target_bias << "\n";
    // 问题配置
    out << "obstacle_count=" << problem.obstacles.size() << "\n";
    out << "base_step=" << config.base_step << "\n";
    out << "safe_distance=" << config.safe_distance << "\n";
    out << "raw_path_collision_free="
        << (!polylineCollides(result.raw_path, problem, config.collision_samples_per_meter) ? 1 : 0)
        << "\n";
    out << "smoothed_path_collision_free="
        << (!polylineCollides(result.smoothed_path, problem, config.collision_samples_per_meter) ? 1 : 0)
        << "\n";
}

// 格式化点坐标为字符串（用于日志输出）
std::string formatPoint(const Vec2& point) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << "(" << point.x << ", " << point.y << ")";
    return stream.str();
}

}  // namespace

// ============================================================================
// 主函数：运行论文算法复现测试
// ============================================================================

int main(int argc, char** argv) {
    rclcpp::init(argc, argv);
    auto logger = rclcpp::get_logger("test_rrtstar_connect_reproduction");

    /*
     * TEST 文件用途：
     * 1. 读取论文《A novel RRT*-Connect algorithm for path planning on robotic arm collision avoidance》
     *    后，将核心数学逻辑映射成可执行代码。
     * 2. 先在二维平面做最小复现，验证”采样-扩展-代价-平滑”的基础算法链路。
     * 3. 再标注它与当前工程 HeuristicGuided 的接口对应关系，避免算法说明停留在论文层面。
     */

    // 步骤1：构建测试问题和配置
    const Problem problem = buildDefaultProblem();
    const PlannerConfig config;
    std::mt19937 rng(42);  // 固定随机种子，确保结果可复现

    // 步骤2：输出测试信息
    RCLCPP_INFO(logger, "开始运行论文复现 TEST: improved RRT*-Connect 2D prototype");
    RCLCPP_INFO(logger, "场景: start=%s goal=%s obstacle_count=%zu",
                formatPoint(problem.start).c_str(),
                formatPoint(problem.goal).c_str(),
                problem.obstacles.size());

    // 步骤3：运行路径规划算法
    const PlanResult result = planBidirectionalRrtStarConnect(problem, config, rng);
    if (!result.success) {
        RCLCPP_ERROR(logger, "规划失败: 在 %d 次迭代内未连通", result.iterations);
        rclcpp::shutdown();
        return 1;
    }
    const bool raw_path_collision_free =
        !polylineCollides(result.raw_path, problem, config.collision_samples_per_meter);
    const bool smoothed_path_collision_free =
        !polylineCollides(result.smoothed_path, problem, config.collision_samples_per_meter);
    if (!raw_path_collision_free || !smoothed_path_collision_free) {
        RCLCPP_ERROR(logger,
                     "规划结果碰撞复核失败: raw=%s smoothed=%s",
                     raw_path_collision_free ? "free" : "collision",
                     smoothed_path_collision_free ? "free" : "collision");
        rclcpp::shutdown();
        return 2;
    }

    // 步骤4：保存结果到文件
    std::filesystem::path output_root;
#ifdef MY_CR5_CONTROL_SOURCE_DIR
    output_root =
        std::filesystem::path(MY_CR5_CONTROL_SOURCE_DIR) / "test_results" / "operations" /
        "rrtstar_connect_reproduction";
#else
    output_root =
        std::filesystem::path("test_results") / "operations" / "rrtstar_connect_reproduction";
#endif
    writePathCsv(output_root / "path_points.csv", result.raw_path, result.smoothed_path);
    writeTreeCsv(output_root / "tree_nodes.csv",
                 result.start_tree_snapshot,
                 result.goal_tree_snapshot);
    writeSummary(output_root / "summary.txt", result, problem, config);

    // 步骤5：输出规划结果统计
    RCLCPP_INFO(logger, "规划成功: iterations=%d start_tree_nodes=%d goal_tree_nodes=%d",
                result.iterations, result.start_tree_nodes, result.goal_tree_nodes);
    RCLCPP_INFO(logger, "原始路径点数=%zu 路径长度=%.4f",
                result.raw_path.size(), pathLength(result.raw_path));
    RCLCPP_INFO(logger, "Bezier 平滑后点数=%zu 路径长度=%.4f",
                result.smoothed_path.size(), pathLength(result.smoothed_path));
    RCLCPP_INFO(logger, "采样统计: success_ratio=%.3f last_wldiw=%.3f last_waiw=%.3f last_Pa=%.3f",
                successRatio(result.sampling_state),
                result.sampling_state.last_wldiw,
                result.sampling_state.last_waiw,
                result.sampling_state.last_target_bias);
    RCLCPP_INFO(logger, "输出文件: %s", output_root.c_str());

    // 步骤6：输出论文算法与工程代码的映射关系
    for (const auto& mapping : buildProjectMapping()) {
        RCLCPP_INFO(logger, "工程映射 | %s | TEST=%s | 当前工程=%s",
                    mapping.paper_component,
                    mapping.this_test_file,
                    mapping.current_project_status);
    }

    rclcpp::shutdown();
    return 0;
}
