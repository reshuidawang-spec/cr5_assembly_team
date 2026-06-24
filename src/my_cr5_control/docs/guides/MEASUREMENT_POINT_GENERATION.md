# 测点生成策略文档

## 问题背景

### 当前研究范围的局限
**现状**：假设测点已知（手动示教），只研究路径规划
**问题**：实际测量中，测点从哪里来？

### 完整的测量流程
```
CAD模型 → 特征提取 → 测点生成 → 路径规划 → 执行测量
          ↑_______↑      ↑_______↑
          缺失部分        当前研究
```

## 解决方案：混合方案

### 核心思路
- **主要贡献**：Learning-Guided路径规划（深入研究，80%精力）
- **次要贡献**：基于几何的测点生成（使用现有方法，20%精力）

### 为什么不深入研究测点生成？
1. **研究范围控制**：两个深度创新点会导致研究周期过长（6-8个月）
2. **学术聚焦**：路径规划的Learning-Guided方法是核心创新
3. **现有技术充足**：测点生成有成熟的几何方法，不需要深度学习

## 测点生成方法（简单但有效）

### 方法1：基于曲率的自适应采样

**原理**：在曲率大的区域密集采样，平坦区域稀疏采样

**实现**：
```cpp
std::vector<Point> generateCurvatureBasedSamples(const Mesh& mesh) {
    std::vector<Point> samples;

    for (const auto& vertex : mesh.vertices) {
        // 计算局部曲率（高斯曲率或平均曲率）
        double curvature = computeGaussianCurvature(vertex);

        // 曲率越大，采样密度越高
        double sampling_density = 1.0 + 10.0 * curvature;

        if (shouldSample(sampling_density)) {
            samples.push_back(vertex.position);
        }
    }

    return samples;
}
```

**适用场景**：
- 复杂曲面测量
- 自由曲面检测
- 叶片、模具测量

### 方法2：特征点提取

**原理**：识别几何特征（孔、边缘、角点），在特征位置采样

**实现**：
```cpp
struct GeometricFeature {
    enum Type { HOLE, EDGE, CORNER, PLANE };
    Type type;
    Point center;
    Vector normal;
    double size;
};

std::vector<Point> generateFeatureBasedSamples(const CADModel& model) {
    std::vector<Point> samples;

    // 1. 提取孔特征
    auto holes = extractHoles(model);
    for (const auto& hole : holes) {
        // 孔中心 + 圆周上的点
        samples.push_back(hole.center);
        samples.push_back(hole.center + Vector(hole.radius, 0, 0));
        samples.push_back(hole.center + Vector(0, hole.radius, 0));
    }

    // 2. 提取边缘特征
    auto edges = extractEdges(model);
    for (const auto& edge : edges) {
        // 边缘中点
        samples.push_back(edge.midpoint());
    }

    // 3. 提取角点特征
    auto corners = extractCorners(model);
    for (const auto& corner : corners) {
        samples.push_back(corner.position);
    }

    return samples;
}
```

**适用场景**：
- 箱体测量（我们的场景）
- 机械零件检测
- 装配件测量

### 方法3：均匀网格采样

**原理**：在零件表面均匀分布测点

**实现**：
```cpp
std::vector<Point> generateUniformGridSamples(
    const BoundingBox& bbox,
    double grid_spacing) {

    std::vector<Point> samples;

    for (double x = bbox.min.x; x <= bbox.max.x; x += grid_spacing) {
        for (double y = bbox.min.y; y <= bbox.max.y; y += grid_spacing) {
            for (double z = bbox.min.z; z <= bbox.max.z; z += grid_spacing) {
                Point p(x, y, z);

                // 检查点是否在零件表面附近
                if (isNearSurface(p, bbox.mesh)) {
                    samples.push_back(p);
                }
            }
        }
    }

    return samples;
}
```

**适用场景**：
- 简单几何体
- 快速检测
- 基准测试

## 论文中的处理方式

### 章节结构调整

**3. Problem Formulation**
```
3.1 Measurement Point Generation (1页)
    - 问题定义：给定CAD模型，生成测点集合
    - 方法：基于特征的采样（简要描述）
    - 实现：使用方法2（特征点提取）
    - 说明：这不是本文重点，使用现有几何方法

3.2 Path Planning Problem (2-3页) ← 重点
    - 问题定义：给定测点集合，规划访问路径
    - 约束：碰撞、关节限位、奇异点
    - 优化目标：时间最优 + 平滑度
    - 详细建模
```

**4. Methodology**
```
4.1 Feature-Based Sampling (0.5页)
    - 算法伪代码
    - 参数设置
    - 简单验证

4.2 Learning-Guided BIT* (4-5页) ← 重点
    - 特征设计
    - 网络架构
    - 采样策略
    - 集成方法
```

**5. Experiments**
```
5.1 Measurement Point Quality (0.5页)
    - 测点覆盖率
    - 特征检测准确率
    - 简单对比（均匀采样 vs 特征采样）

5.2 Path Planning Performance (4-5页) ← 重点
    - 基准对比
    - 消融实验
    - 泛化性测试
    - 真机验证
```

### 创新点声明

**主要贡献**（论文摘要和引言中强调）：
1. **Learning-Guided Sampling Framework** ← 核心创新
2. **Geometry-Aware Feature Design** ← 核心创新
3. **Integration with BIT*** ← 技术贡献
4. **Complete Measurement System** ← 应用贡献（包括测点生成）

**次要贡献**（简要提及）：
- Feature-based measurement point generation（使用现有方法）

## 实施计划

### 阶段0.5：测点生成器实现（1周）

**任务**：
1. 实现方法2（特征点提取）
2. 针对箱体场景优化
3. 生成测试数据

**输出**：
- `MeasurementPointGenerator`类
- 支持箱体的孔、边缘、角点提取
- 生成100+个测试场景

### 阶段1-5：保持不变

继续按照原计划进行Learning-Guided BIT*的研究。

## 对论文发表的影响

### 优势
✅ **完整性**：覆盖了测量的完整流程
✅ **实用性**：可以直接应用到实际测量
✅ **聚焦性**：主要创新点仍然是路径规划
✅ **可接受性**：测点生成使用成熟方法，审稿人不会质疑

### 风险
⚠️ **篇幅**：论文可能略长，需要控制测点生成部分的篇幅
⚠️ **创新性**：需要明确主要创新点是路径规划，不是测点生成

### 应对策略
1. **在摘要和引言中明确**：主要贡献是Learning-Guided路径规划
2. **测点生成部分简洁**：1-1.5页，不展开细节
3. **实验部分侧重**：路径规划实验占80%，测点生成实验占20%

## 具体实现示例（针对箱体）

### 箱体特征提取

```cpp
class BoxFeatureExtractor {
public:
    struct BoxFeatures {
        std::vector<Point> hole_centers;      // 孔中心
        std::vector<Point> edge_midpoints;    // 边缘中点
        std::vector<Point> corner_points;     // 角点
        std::vector<Point> surface_samples;   // 表面采样点
    };

    BoxFeatures extract(const BoxGeometry& box) {
        BoxFeatures features;

        // 1. 提取孔特征（如果有）
        for (const auto& hole : box.holes) {
            features.hole_centers.push_back(hole.center);

            // 孔周围的采样点
            for (int i = 0; i < 4; ++i) {
                double angle = i * M_PI / 2.0;
                Point p = hole.center + Point(
                    hole.radius * cos(angle),
                    hole.radius * sin(angle),
                    0.0
                );
                features.surface_samples.push_back(p);
            }
        }

        // 2. 提取边缘中点
        for (const auto& edge : box.edges) {
            features.edge_midpoints.push_back(edge.midpoint());
        }

        // 3. 提取角点
        features.corner_points = box.corners;

        // 4. 表面均匀采样
        double spacing = 0.05;  // 5cm间距
        for (const auto& face : box.faces) {
            auto samples = uniformSampleOnFace(face, spacing);
            features.surface_samples.insert(
                features.surface_samples.end(),
                samples.begin(),
                samples.end()
            );
        }

        return features;
    }
};
```

### 使用示例

```cpp
// 1. 加载箱体模型
BoxGeometry box = loadBoxFromCAD("box_model.stl");

// 2. 提取特征并生成测点
BoxFeatureExtractor extractor;
auto features = extractor.extract(box);

// 3. 合并所有测点
std::vector<Point> measurement_points;
measurement_points.insert(measurement_points.end(),
                         features.hole_centers.begin(),
                         features.hole_centers.end());
measurement_points.insert(measurement_points.end(),
                         features.edge_midpoints.begin(),
                         features.edge_midpoints.end());
measurement_points.insert(measurement_points.end(),
                         features.surface_samples.begin(),
                         features.surface_samples.end());

// 4. 使用Learning-Guided BIT*规划路径
auto path = planMeasurementPath(measurement_points);

// 5. 执行测量
executeMeasurement(path);
```

## 总结

### 最终方案
- **测点生成**：使用简单的几何方法（特征提取）
- **路径规划**：深入研究Learning-Guided BIT*
- **论文定位**：主要贡献是路径规划，测点生成是辅助

### 时间分配
- 测点生成实现：1周
- 路径规划研究：11-13周
- 总计：12-14周（不变）

### 学术价值
- 完整的测量系统（提高实用性）
- 聚焦的创新点（保证深度）
- 平衡的研究范围（适合二区期刊）
