# my_cr5_control

CR5 机械臂在 MoveIt2/OMPL 上的控制、benchmark 和路径规划算法实验工程。

当前已经冻结的论文主线为：

- `Difficulty-Adaptive Informed Guide Sampling for Constrained Motion Planning in Robotic Contact Measurement`
- 工程承载接口：`HeuristicGuided two-stage guidance interface`
- 几何验证主线：
  - `simple` 可控箱体 benchmark
  - `v2` 基于 `WS119.STL` 的 benchmark

## 主要内容

- `src/core/`
  - `CR5Robot` 核心封装，包含标准规划接口和 `HeuristicGuided` 两阶段引导规划。
- `src/benchmarks/`
  - `simple`、`v2` benchmark 和随机任务数据采集节点。
- `scripts/`
  - benchmark 导出、绘图、数据维护和模型训练脚本。
- `docs/`
  - 命令、指南、分析报告和论文路线文档。
- `paper_artifacts/`
  - 当前论文正式结果归档与表格输入。
- `test_results/`
  - benchmark、数据集、导出表和模型产物。

## 构建

在工作空间根目录执行：

```bash
colcon build --packages-select my_cr5_control
```

## 文档入口

建议先看：

1. [docs/README.md](./docs/README.md)
2. [PROJECT_MEMORY.md](./PROJECT_MEMORY.md)
3. [docs/PAPER_MAINLINE_MAP.md](./docs/PAPER_MAINLINE_MAP.md)
4. [docs/COMMANDS.md](./docs/COMMANDS.md)
5. [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](./paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
6. [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](./paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)

## 常用运行入口

- benchmark launch:

```bash
ros2 launch my_cr5_control planner_benchmark.launch.py benchmark:=simple repeats:=10
```

- simple benchmark 节点：

```bash
ros2 run my_cr5_control planner_comparison_simple_node
```

- 说明：
  - `simple` / `v2` benchmark 现在默认按论文主线启用 adaptive ellipsoid。
  - 如需做非主线消融，可显式设置 `MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID=0`。

- guide ranking 数据采集 / 消融：

```bash
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

## 当前工程状态

- 当前主方法已经固定为：`difficulty-adaptive informed guide sampling`
- 当前论文主线不要再被理解成某个单独 demo 节点：
  - 统一以 [docs/PAPER_MAINLINE_MAP.md](./docs/PAPER_MAINLINE_MAP.md) 为准
- 如果是 `v2/WS119` 的 RViz 展示场景设计：
  - 统一以 [docs/guides/V2_HERO_SCENE_DESIGN.md](./docs/guides/V2_HERO_SCENE_DESIGN.md) 为准
- 当前正式 paper-facing 结果归档位于：
  - [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](./paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
  - [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](./paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)
- 当前完整数据仓位于：
  - [test_results/README.md](./test_results/README.md)
