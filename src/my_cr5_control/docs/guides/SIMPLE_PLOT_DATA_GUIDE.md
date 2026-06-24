# Simple Benchmark 绘图数据说明

## 1. 目标

`planner_comparison_simple_node` 现在除了输出带时间戳的单次结果文件，还会自动把数据累计到统一的绘图数据文件中，方便后续直接做柱状图、箱线图和成功率图。

## 2. 主数据文件

统一绘图明细文件：

- `test_results/benchmarks/simple/aggregates/planner_comparison_simple_plot_data_metrics.csv`

统一绘图摘要文件：

- `test_results/benchmarks/simple/aggregates/planner_comparison_simple_plot_summary_metrics.csv`

每次运行仍然会保留原始时间戳文件：

- `test_results/benchmarks/simple/raw/<timestamp>_planner_comparison_simple_results.csv`
- `test_results/benchmarks/simple/raw/<timestamp>_planner_comparison_simple_summary.csv`

## 3. 字段说明

### 3.1 明细文件 `planner_comparison_simple_plot_data_metrics.csv`

每一行表示一次 `规划器 x 场景 x 重复序号` 的实验结果，主要字段：

- `基准版本`
- `实验时间戳`
- `总重复次数`
- `重复序号`
- `规划器`
- `模式`
- `规划器ID`
- `场景名称`
- `难度`
- `测点描述`
- `难度评分`
- `尖端X/Y/Z`
- `法兰X/Y/Z`
- `成功`
- `墙钟时间(ms)`
- `MoveIt规划时间(ms)`
- `预算上限(ms)`
- `规划调用次数`
- `触发预算上限`
- `快速求解(<1s)`

这个文件最适合画：

- 箱线图
- 小提琴图
- 单场景 planner 时间分布图
- 成功/失败散点图

### 3.2 摘要文件 `planner_comparison_simple_plot_summary_metrics.csv`

每一行表示一次 `实验时间戳 x 规划器` 的汇总结果，主要字段：

- `基准版本`
- `实验时间戳`
- `总重复次数`
- `规划器`
- `模式`
- `规划器ID`
- `成功率(%)`
- `成功样本数`
- `总样本数`
- `平均时间(ms)`
- `中位时间(ms)`
- `P25(ms)`
- `P75(ms)`
- `预算命中数`
- `预算命中率(%)`
- `快速求解数`
- `快速求解率(%)`
- `平均MoveIt规划时间(ms)`
- `平均规划调用次数`
- `简单场景(ms)`
- `中等场景(ms)`
- `困难场景(ms)`
- `极端场景(ms)`
- `详细结果文件`

这个文件最适合画：

- planner 总体成功率柱状图
- planner 总体均值/中位数对比图
- 不同难度平均时间对比图
- timeout-like heatmap

## 4. 当前 simple 基准版本

当前代码使用的版本标记是：

- `simple_v20260311_6scene_metrics`

后面如果你继续改场景定义，建议保留新的版本号。做图时先按 `基准版本` 过滤，避免把不同场景版本的数据混在一起。

## 5. 运行方式

默认跑 1 次：

```bash
ros2 run my_cr5_control planner_comparison_simple_node
```

跑 10 次重复实验：

```bash
MY_CR5_CONTROL_SIMPLE_REPEATS=10 ros2 run my_cr5_control planner_comparison_simple_node
```

## 6. 画图建议

如果你后面用 Python 画图，优先直接读取：

- 明细分布图：`planner_comparison_simple_plot_data_metrics.csv`
- 汇总柱状图：`planner_comparison_simple_plot_summary_metrics.csv`

仓库内已经提供可直接出图脚本：

```bash
python3 scripts/benchmarks/plot_simple_benchmark.py \
  --benchmark-version simple_v20260311_6scene_metrics \
  --timestamp 20260311_124449_790
```

默认输出目录：

- `test_results/benchmarks/simple/plots/<基准版本>/<时间戳>/`

脚本会自动生成：

- `overall_stats.csv`
- `per_scene_stats.csv`
- `overall_success_rate.png`
- `overall_time_mean_vs_median.png`
- `per_scene_median_time.png`
- `per_scene_timeout_heatmap.png`
- `per_scene_boxplots.png`

第一层过滤建议：

1. 先过滤 `基准版本 == simple_v20260311_6scene_metrics`
2. 再按需要过滤某个 `实验时间戳` 或者合并多个时间戳
3. 画图时把 `重复序号` 当作独立样本
