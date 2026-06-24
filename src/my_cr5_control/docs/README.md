# 工程文档总览

## 1. 这份文档的作用

这个工程已经同时包含：

- 机器人控制与规划代码
- benchmark 与随机任务数据
- 论文路线与写作骨架
- 实验分析报告
- 正式论文归档材料

当前最大的问题不是文档缺失，而是入口分散。  
这份索引文档的目标就是给整个工程提供一个统一入口。

当前根目录已经新增两个收敛入口：

- `paper_workspace/`
  - 当前论文主线文档、正式结果和参考材料集中区
- `project_archive/`
  - 历史实验、旧分析和旧结果归档区

## 2. 当前推荐阅读顺序

如果你是第一次进入当前工作区，建议按下面顺序阅读：

1. 根目录 [README.md](../README.md)
2. 根目录 [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
3. [docs/PAPER_MAINLINE_MAP.md](./PAPER_MAINLINE_MAP.md)
4. [docs/COMMANDS.md](./COMMANDS.md)
5. [docs/PROJECT_LAYOUT.md](./PROJECT_LAYOUT.md)
6. [paper_workspace/README.md](../paper_workspace/README.md)
7. [docs/guides/README.md](./guides/README.md)
8. [docs/roadmap/README.md](./roadmap/README.md)
9. [docs/analysis/README.md](./analysis/README.md)

## 3. 按目标找文档

### 3.1 想先跑工程

优先看：

- [README.md](../README.md)
- [docs/COMMANDS.md](./COMMANDS.md)
- [docs/PROJECT_LAYOUT.md](./PROJECT_LAYOUT.md)

### 3.2 想理解当前算法主线

优先看：

- [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
- [docs/PAPER_MAINLINE_MAP.md](./PAPER_MAINLINE_MAP.md)
- [docs/guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md](./guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md)
- [docs/guides/V2_HERO_SCENE_DESIGN.md](./guides/V2_HERO_SCENE_DESIGN.md)
- [paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)

### 3.3 想写论文

优先看：

- [paper_workspace/README.md](../paper_workspace/README.md)
- [paper_workspace/docs/Q2_PAPER_ROADMAP.md](../paper_workspace/docs/Q2_PAPER_ROADMAP.md)
- [paper_workspace/docs/Q2_PAPER_METHOD_RESULTS_SKELETON.md](../paper_workspace/docs/Q2_PAPER_METHOD_RESULTS_SKELETON.md)
- [paper_workspace/docs/Q2_PAPER_CHINESE_DRAFT_V1.md](../paper_workspace/docs/Q2_PAPER_CHINESE_DRAFT_V1.md)
- [paper_workspace/docs/PAPER_REVIEW_RESOLUTION_LOG.md](../paper_workspace/docs/PAPER_REVIEW_RESOLUTION_LOG.md)
- [paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
- [paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md)

### 3.4 想查正式结果与表格输入

优先看：

- [paper_workspace/README.md](../paper_workspace/README.md)
- [paper_artifacts/README.md](../paper_artifacts/README.md)
- [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
- [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)
- [test_results/README.md](../test_results/README.md)

### 3.5 想补 benchmark / 数据分析

优先看：

- [docs/guides/BENCHMARK_DATASET_GUIDE.md](./guides/BENCHMARK_DATASET_GUIDE.md)
- [docs/guides/RANDOM_TASK_DATASET_GUIDE.md](./guides/RANDOM_TASK_DATASET_GUIDE.md)
- [docs/guides/DATA_STORAGE_GUIDE.md](./guides/DATA_STORAGE_GUIDE.md)
- [docs/analysis/README.md](./analysis/README.md)

## 4. 文档目录分工

### 4.1 根目录文档

- [README.md](../README.md)
  - 工程总入口
- [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
  - 当前研究主线、阶段结论、数据口径和维护约定

### 4.2 `docs/guides/`

- 长期有效的使用指南、接口说明和数据流程说明
- 适合工程实现、复现实验和维护工作

### 4.3 `docs/analysis/`

- 已完成实验的分析报告
- 当前最重要的是正式 benchmark 和 post-fix rerun 分析

### 4.4 `docs/roadmap/`

- 论文路线、方法写作骨架和中文初稿骨架
- 当前写论文时应优先使用这一组文档

### 4.5 `paper_artifacts/`

- 面向论文主表、主图和正式引用的归档材料
- 当前根目录 `paper_artifacts/` 是指向 `paper_workspace/formal_results/` 的兼容入口
- 这是从 `test_results/` 和 `docs/analysis/` 中抽出的“论文用正式子集”

### 4.6 `test_results/`

- 当前保留的是仍与主线 benchmark 直接相关的结果
- 大量历史实验已归档到 `project_archive/test_results/`
- 它不是“论文正式入口”

### 4.7 `project_archive/`

- 历史实验、旧分析、旧模型和临时清理目录归档区
- 不参与当前论文主线叙事，主要用于回溯

## 5. 当前最重要的 9 个文档

如果只看最关键的文档，建议固定为这 9 个：

1. [README.md](../README.md)
2. [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
3. [docs/COMMANDS.md](./COMMANDS.md)
4. [docs/guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md](./guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md)
5. [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
6. [docs/roadmap/Q2_PAPER_ROADMAP.md](./roadmap/Q2_PAPER_ROADMAP.md)
7. [docs/roadmap/Q2_PAPER_METHOD_RESULTS_SKELETON.md](./roadmap/Q2_PAPER_METHOD_RESULTS_SKELETON.md)
8. [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)
9. [docs/guides/V2_HERO_SCENE_DESIGN.md](./guides/V2_HERO_SCENE_DESIGN.md)

## 6. 当前文档维护规则

1. 新增可执行命令时，同步更新 [docs/COMMANDS.md](./COMMANDS.md)
2. 新增重要研究结论时，同步更新 [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
3. 新增实验分析时，放入 `docs/analysis/`
4. 新增长期使用说明时，放入 `docs/guides/`
5. 新增论文路线或写作骨架时，放入 `docs/roadmap/`
6. 新增论文正式图表和结果归档时，放入 `paper_artifacts/`
7. 不再把新的说明性文档直接堆到仓库根目录
8. 历史实验和旧分析优先归档到 `project_archive/`
