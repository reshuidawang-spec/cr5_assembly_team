# Data Storage Guide

## 1. 目标

高价值数据不能只“存在 `test_results/` 里”，必须区分：

- 哪些是正式 benchmark
- 哪些是训练候选数据
- 哪些只是 smoke / 过渡结果
- 哪些文件后续写论文和作图时应作为唯一引用源

## 2. 当前固定存放原则

当前统一规则：

1. 所有原始结果 CSV 保留在 `test_results/`
2. 时间戳命名的原始结果文件视为不可覆盖原件
3. `test_results/` 采用分层结构：
   - `benchmarks/simple/raw`
   - `benchmarks/v2/raw`
   - `exports`
   - `models`
   - `logs`
4. 说明文档统一放在 `docs/` 下：
   - 指南类文档放在 `docs/guides/`
   - 分析报告放在 `docs/analysis/`
5. 所有“值得保留”的正式论文结果必须在 `paper_workspace/formal_results/` 下有 README 或 ANALYSIS 入口。
6. 历史 manifest 已归档到 `project_archive/test_results/dataset_manifest.csv`；若重新启用数据集 manifest，应在 `test_results/` 下重建并同步更新本文档。

## 3. 当前建议保留的核心数据

优先级最高的数据：

- `paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/`
- `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/`
- `paper_workspace/qualitative_results/p1_3_v2_hero_20260511/`
- `test_results/benchmarks/simple/`
- `test_results/benchmarks/v2/`

这些文件是后续：

- 做正式图表
- 写论文实验部分
- 做复现实验

时的主数据源。

## 4. 不同类型数据的角色

### 4.1 正式 benchmark

作用：

- 经典算法对比
- 论文主表与基础结论

当前参考：

- `paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md`
- `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md`
- `test_results/benchmarks/simple/`
- `test_results/benchmarks/v2/`

### 4.2 统一导出训练表

作用：

- 跨 `simple + v2` 的统一分析输入
- 后续模型训练的基线表

当前参考：

- 历史导出表已归档到 `project_archive/test_results/exports/`
- 当前论文正式表格优先从 `paper_workspace/formal_results/` 查找

### 4.3 随机任务训练数据

作用：

- 训练 `hit_budget_limit`
- 训练 `fast_solve_lt_1s`
- 提取 hard/extreme 任务标签

当前参考：

- `project_archive/test_results/datasets/simple_random/`

### 4.4 模型训练产物

作用：

- 保存第一版 baseline 训练 run
- 固化权重、评估摘要和测试集预测
- 为后续 planner 引导验证提供可追溯入口

当前参考：

- `project_archive/test_results/models/simple_random_baseline/`

### 4.5 Learned Guidance 候选数据与在线消融

作用：

- 保存 `guide candidate ranking` 的训练输入
- 保存 learned guidance 首轮线上对照结果
- 固化“接口已打通但模型是否真正有效”的证据

当前参考：

- `project_archive/test_results/datasets/guide_ranking_simple/`
- `project_archive/test_results/models/`
- `project_archive/test_results/benchmarks/simple_guidance/`

## 5. 后续归档要求

今后每生成一批“正式可用”的结果，至少同步做三件事：

1. 在 `paper_workspace/formal_results/` 创建带 README 或 ANALYSIS 的归档目录
2. 如果值得保留，补对应分析文档
3. 在 `PROJECT_MEMORY.md` 记录它为什么重要

## 6. 当前不建议删除的文件

即使不是最终主数据，也先不要删：

- smoke 数据
- 旧版 benchmark
- 旧版 random dataset

原因：

- 它们可以帮助回溯参数变化
- 可以定位数据 schema 演化
- 可以在论文审稿阶段回答“你们之前是否有失败版本”

## 7. 当前已固化的索引入口

当前统一从下面两个入口找高价值数据：

- [paper_workspace/formal_results/README.md](../../paper_workspace/formal_results/README.md)
- [PROJECT_MEMORY.md](../../PROJECT_MEMORY.md)
- [project_archive/test_results/dataset_manifest.csv](../../project_archive/test_results/dataset_manifest.csv)
