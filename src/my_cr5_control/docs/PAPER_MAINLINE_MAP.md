# Paper Mainline Map

## 1. 作用

这份文档是当前工程里对“论文主线”边界的唯一收敛说明。

如果某个文件名、旧命令、日志字符串和这里冲突，以这里为准。

## 2. 当前论文主线到底是什么

当前论文主线不是某一个 demo 节点，也不是某一个 launch 文件。

它由 4 层共同组成：

1. 问题层
   - `机器人接触测量场景下的受限路径规划`
2. 方法层
   - `difficulty-adaptive informed guide sampling`
3. 工程承载层
   - `HeuristicGuided two-stage guidance interface`
   - 入口：`CR5Robot::planToPoseImproved(...)`
4. 证据层
   - `simple` 可控箱体 benchmark
   - `v2` 基于 `WS119.STL` 的真实几何 benchmark

因此，当前论文主线应始终被理解为：

- `HeuristicGuided + difficulty-adaptive informed guide sampling + simple + v2/WS119`

而不是：

- 某个 synthetic demo
- learned guidance 支线
- RRT*-Connect reproduction

## 3. 主线代码地图

### 3.1 方法主入口

- [include/my_cr5_control/cr5_robot.hpp](../include/my_cr5_control/cr5_robot.hpp)
- [src/core/cr5_robot.cpp](../src/core/cr5_robot.cpp)

这里定义：

- direct planning
- guide candidate 生成
- candidate ranking
- guide-first / direct-preservation gate
- 两阶段规划执行

主线共享类型已经开始从 `CR5Robot` 中抽出：

- [include/my_cr5_control/paper_mainline/guide_types.hpp](../include/my_cr5_control/paper_mainline/guide_types.hpp)
- [include/my_cr5_control/paper_mainline/guide_policy_params.hpp](../include/my_cr5_control/paper_mainline/guide_policy_params.hpp)
- [include/my_cr5_control/paper_mainline/guide_geometry.hpp](../include/my_cr5_control/paper_mainline/guide_geometry.hpp)
- [include/my_cr5_control/paper_mainline/guide_bridge.hpp](../include/my_cr5_control/paper_mainline/guide_bridge.hpp)

这里固定保存 `PlanningMetrics`、`GuideCandidate` 和 `GuideRankingFunction`。
`guide_policy_params.hpp` 固定保存当前论文主线的 guide/gate 阈值常量。
`guide_geometry.hpp` 保存 guide 相关纯几何工具，`guide_bridge.hpp` 保存两 guide
bridge 候选组合规则。
`CR5Robot` 暂时保留同名兼容别名，后续 sampler / gate / benchmark 代码应优先依赖
`my_cr5_control::paper_mainline` 下的类型和参数。

### 3.2 `simple` 可控箱体 benchmark

- [src/benchmarks/planner_comparison_simple.cpp](../src/benchmarks/planner_comparison_simple.cpp)
- [src/tools/heuristic_guided_visual_debug.cpp](../src/tools/heuristic_guided_visual_debug.cpp)

它负责：

- 可控箱体几何
- `easy / medium / hard / extreme` 场景分层
- 论文里对 selective activation 行为的解释

### 3.3 `v2` / `WS119.STL` 主线

- [src/meshes/WS119.STL](../src/meshes/WS119.STL)
- [include/my_cr5_control/scene_utils.hpp](../include/my_cr5_control/scene_utils.hpp)
- [include/my_cr5_control/paper_mainline/v2_measurement_pose.hpp](../include/my_cr5_control/paper_mainline/v2_measurement_pose.hpp)
- [include/my_cr5_control/paper_mainline/v2_scenario_profile.hpp](../include/my_cr5_control/paper_mainline/v2_scenario_profile.hpp)
- [src/core/measurement_point_generator.cpp](../src/core/measurement_point_generator.cpp)
- [src/benchmarks/planner_comparison_v2.cpp](../src/benchmarks/planner_comparison_v2.cpp)
- [docs/guides/V2_HERO_SCENE_DESIGN.md](./guides/V2_HERO_SCENE_DESIGN.md)

它负责：

- 真实 STL 几何加载
- world pose / scale 对齐
- measurement point 法向到测针法兰位姿的统一转换
- hole / cavity / measurement point 自动提取
- canonical V2 场景 profile 与测点类型 fallback 顺序
- `Easy_HoleCenter / Medium_HoleEdge / Hard_DeepInterior / Extreme_NarrowPassage`
- `v2 hero scene` 的长期展示约束与防混淆命名

### 3.4 正式结果与论文工作区

- [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
- [paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
- [paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md)
- [paper_workspace/README.md](../paper_workspace/README.md)

## 4. 最容易混淆、但不属于论文主线的东西

### 4.1 synthetic guidance demo

- [src/tools/synthetic_guidance_demo.cpp](../src/tools/synthetic_guidance_demo.cpp)
- [launch/synthetic_guidance_demo.launch.py](../launch/synthetic_guidance_demo.launch.py)

这条线是：

- synthetic 场景
- RViz 演示 / 调试入口
- 非 canonical benchmark

它不是当前论文主线。

兼容说明：

- 旧名字 `cr5_paper_mainline_demo_node`
- 旧 launch `paper_mainline_demo.launch.py`

目前仅保留为兼容别名，不应再被用来指代论文主线。

### 4.1A `v2 hero scene` 的边界

- `v2 hero scene` 是 `v2/WS119` 主线的可视化展示场景设计
- 它服务于：
  - `Hard_DeepInterior`
  - `Extreme_NarrowPassage`
  - active guidance 的可解释展示
- 它不是：
  - synthetic demo
  - `paper_mainline_demo`
  - 任意历史 RViz 演示别名

统一参考：

- [docs/guides/V2_HERO_SCENE_DESIGN.md](./guides/V2_HERO_SCENE_DESIGN.md)

### 4.2 learned guidance 支线

- [src/benchmarks/guide_ranking_simple_experiment.cpp](../src/benchmarks/guide_ranking_simple_experiment.cpp)
- [scripts/models/train_guide_ranking_model.py](../scripts/models/train_guide_ranking_model.py)

这是可执行扩展和消融线，不是当前主论文主结果。

### 4.3 旧复现 / GUI / 业务节点

- `test_rrtstar_connect_reproduction`
- `cr5_experimental_task_space_rrtstar_demo`
- `src/gui/`
- `piston_spray_*`

这些都属于工程资产，但不参与当前论文主线叙事。

## 5. 当前推荐阅读顺序

1. [README.md](../README.md)
2. [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
3. [docs/PAPER_MAINLINE_MAP.md](./PAPER_MAINLINE_MAP.md)
4. [docs/guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md](./guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md)
5. [paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
6. [paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md)

## 6. 今后命名规则

从现在开始：

- 只有 `simple + v2/WS119 + HeuristicGuided B+` 可以被叫做“论文主线”
- synthetic 场景文件必须使用 `synthetic` / `legacy_demo` 一类命名
- learned ranking 文件不得再被写成“paper mainline”
- 如果某个新入口只是演示，不得再使用 `paper_mainline_*` 前缀
