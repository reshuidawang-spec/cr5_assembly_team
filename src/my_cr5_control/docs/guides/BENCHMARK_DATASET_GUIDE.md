# Benchmark Dataset Guide

## 1. 现在新增了什么

`simple` 和 `v2` 的明细结果 CSV 现在都包含下面这些字段：

- `墙钟时间(ms)`
- `MoveIt规划时间(ms)`
- `预算上限(ms)`
- `规划调用次数`
- `触发预算上限`
- `快速求解(<1s)`

摘要 CSV 现在新增：

- `中位时间(ms)`
- `P25(ms)`
- `P75(ms)`
- `预算命中数`
- `预算命中率(%)`
- `快速求解数`
- `快速求解率(%)`
- `平均MoveIt规划时间(ms)`
- `平均规划调用次数`

## 2. 新的累计文件

为了避免和旧 schema 混在一起，新的累计文件改为：

- `test_results/benchmarks/simple/aggregates/planner_comparison_simple_plot_data_metrics.csv`
- `test_results/benchmarks/simple/aggregates/planner_comparison_simple_plot_summary_metrics.csv`
- `test_results/benchmarks/v2/aggregates/planner_comparison_v2_plot_data_metrics.csv`
- `test_results/benchmarks/v2/aggregates/planner_comparison_v2_plot_summary_metrics.csv`

## 3. 导出统一数据集

把最新的 `simple + v2` 结果合并成一张标准化数据表：

```bash
python3 scripts/benchmarks/export_benchmark_dataset.py
```

默认输出：

```bash
test_results/exports/benchmark_training_dataset.csv
```

历史版本默认输入曾优先从 `test_results/dataset_manifest.csv` 读取：

- `simple_formal_latest`
- `v2_formal_latest`

当前 `test_results/dataset_manifest.csv` 已归档到
`project_archive/test_results/dataset_manifest.csv`。论文正式表格优先使用
`paper_workspace/formal_results/` 下的 formal rerun / ablation 归档；只有在重建数据集导出流程时，才重新生成 `test_results/dataset_manifest.csv`。

如果要手动指定输入文件：

```bash
python3 scripts/benchmarks/export_benchmark_dataset.py \
  --inputs \
  test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_results.csv \
  test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv
```

## 3.1 当前 canonical 导出快照

截至 `2026-03-17`，当前统一导出表对应：

- `simple`: `test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_results.csv`
- `v2`: `test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv`
- 导出文件：`test_results/exports/benchmark_training_dataset.csv`

当前这张表的规模是：

- 总行数：`700`
- `simple`: `420`
- `v2`: `280`
- 7 个 planner 各 `100` 行

## 3.2 一键自动采集

如果你要从头自动跑 `simple + v2`，并在结束后直接导出统一数据集：

```bash
source ~/dobot_ws/install/setup.bash
python3 scripts/benchmarks/collect_benchmark_dataset.py
```

这个脚本会：

1. 检查当前是否已有 `move_group`
2. 没有就自动启动 `cr5_moveit demo.launch.py`
3. 顺序运行 `simple` 和 `v2`
4. 自动导出统一数据集
5. 如果是它自己启动的 demo，会在结束后自动关闭

常用参数：

```bash
python3 scripts/benchmarks/collect_benchmark_dataset.py --simple-repeats 5 --v2-repeats 5
python3 scripts/benchmarks/collect_benchmark_dataset.py --benchmarks simple --simple-repeats 10
python3 scripts/benchmarks/collect_benchmark_dataset.py --export-only
```

## 4. 统一数据集字段

导出的数据集统一成下面这些字段：

- `benchmark_family`
- `benchmark_version`
- `timestamp`
- `repeat_index`
- `planner`
- `planning_mode`
- `planner_id`
- `scene_name`
- `difficulty`
- `point_description`
- `difficulty_score`
- `tip_x`, `tip_y`, `tip_z`
- `flange_x`, `flange_y`, `flange_z`
- `success`
- `wall_time_ms`
- `moveit_time_ms`
- `planning_budget_ms`
- `planner_calls`
- `hit_budget_limit`
- `fast_solve_lt_1s`
- `budget_headroom_ms`

## 5. 推荐用法

现在最推荐的工作流是：

1. 跑 `simple` 或 `v2`
2. 直接查看时间戳结果文件
3. 用 `export_benchmark_dataset.py` 或 `collect_benchmark_dataset.py` 合并成统一数据表
4. 后续做图、做统计、做训练集划分都基于这张统一表

## 6. 和当前模型脚本的关系

要特别注意：

- `benchmark_training_dataset.csv` 当前主要服务于：
  - 论文主表统计
  - benchmark 对比绘图
  - 跨 `simple + v2` 的统一分析输入
- 现有 `scripts/models/` 下的训练与评估脚本，默认仍然围绕：
  - `test_results/exports/simple_random_training_table.csv`
  - `simple_random` 随机任务标签

这意味着当前状态是：

- benchmark 导出链路已经刷新并可直接用于论文
- 学习链路并没有直接切换到 `benchmark_training_dataset.csv`
- 如果后续要做 benchmark 级学习实验，应单独做一条准备脚本，而不是直接把 `simple_random` 脚本硬改成共用
