# Box Feature Visualization Guide

## 1. 目的

这个入口用于核对 `WS119.STL` 的自动特征提取结果是否合理。

它会在 RViz 中同时发布：

- 原始 `WS119` mesh
- 外包络 bounding box
- 检测到的 `hole`
- 检测到的 `cavity`
- 外包络 `edges / corners / surface normals`
- 自动生成的 measurement points

## 2. 运行方式

先编译并加载环境：

```bash
cd ~/dobot_ws
colcon build --packages-select my_cr5_control
source install/setup.bash
```

运行可视化节点：

```bash
ros2 run my_cr5_control visualize_box_features_node
```

## 3. RViz 设置

打开 `rviz2`，然后：

1. `Fixed Frame` 设为 `base_link`
2. 添加一个 `MarkerArray` 显示
3. Topic 选择 `/box_feature_markers`

## 4. 颜色约定

- 半透明灰色 mesh：`WS119.STL`
- 青色线框：外包络 bounding box
- 蓝色半透明柱体：检测到的 `hole`
- 橙色半透明方块：检测到的 `cavity`
- 洋红线 / 点：外包络 `edges / corners`
- 绿色箭头：外包络 `surface normals`
- 测点颜色：
  - 蓝青：`HOLE_CENTER`
  - 绿色：`HOLE_EDGE`
  - 黄色：`INTERIOR_DEEP`
  - 红色：`NARROW_PASSAGE`

## 5. 当前用途

这个入口当前主要用来回答两个问题：

1. 自动提取出来的 hole / cavity 是否真的对着 STL 的真实结构
2. 自动生成的测点是否落在“有测量意义且可规划”的位置

## 6. 当前已知限制

- 当前 cavity 检测已经加入 merge / filter 规则，但本质上仍然是基于顶向高度图和下陷连通域，不是 CAD 拓扑级语义识别
- 当前更适合做“工程核对”和“论文实验前的数据 sanity check”，还不是最终 CAD 语义级识别
- 下一步应基于 RViz 核对结果，继续确认保留下来的 cavity 是否都具有真实测量意义
