# Simple Benchmark 正式运行分析

## 1. 本次运行

- 实验时间戳：`20260311_101849_082`
- 基准版本：`simple_v20260311_6scene`
- 重复次数：`10`
- 场景数：`6`
- 规划器数：`7`
- 总样本数：`420`

原始结果文件：

- `test_results/benchmarks/simple/raw/20260311_101849_082_planner_comparison_simple_results.csv`
- `test_results/benchmarks/simple/raw/20260311_101849_082_planner_comparison_simple_summary.csv`

统一绘图文件：

- `test_results/benchmarks/simple/legacy/planner_comparison_simple_plot_data.csv`
- `test_results/benchmarks/simple/legacy/planner_comparison_simple_plot_summary.csv`

## 2. 总体结论

这轮 `simple` 基准已经可以用于论文前期 baseline 分析，但目前更适合做：

- 时间分布对比
- 稳定性对比
- 场景敏感性对比

还不适合直接做：

- 成功率主结论

原因很直接：

- 除 `RRTstar` 在 `Medium_SideSurface` 上出现 `1` 次失败外，其余 planner 全部 `60/60` 成功
- 当前主要差异来自“是否经常卡到 10 秒预算附近”，而不是“能不能求出来”

## 3. 总体表现

### 3.1 成功率

- `RRTConnect`: `100.0%` (`60/60`)
- `RRTstar`: `98.3%` (`59/60`)
- `LBTRRT`: `100.0%` (`60/60`)
- `FMT`: `100.0%` (`60/60`)
- `BFMT`: `100.0%` (`60/60`)
- `PRMstar`: `100.0%` (`60/60`)
- `HeuristicGuided`: `100.0%` (`60/60`)

唯一失败样本：

- `RRTstar`, `repeat=2`, `Medium_SideSurface`, `10029.0 ms`

### 3.2 整体时间统计

按平均时间：

- `HeuristicGuided`: `383.2 ms`
- `LBTRRT`: `2881.4 ms`
- `FMT`: `3876.3 ms`
- `PRMstar`: `4043.3 ms`
- `BFMT`: `4206.7 ms`
- `RRTConnect`: `4208.8 ms`
- `RRTstar`: `10017.5 ms`

按中位数：

- `LBTRRT`: `65.5 ms`
- `FMT`: `67.0 ms`
- `BFMT`: `70.0 ms`
- `RRTConnect`: `72.0 ms`
- `PRMstar`: `73.0 ms`
- `HeuristicGuided`: `75.5 ms`
- `RRTstar`: `10017.0 ms`

## 4. 最重要的统计解释

这批结果是明显的双峰分布：

- 一部分样本在 `几十毫秒`
- 另一部分样本在 `约 10000 ms`

所以：

- `均值` 会被 10 秒级样本严重拉高
- `中位数` 更能反映“典型求解速度”
- `10 秒级卡顿比例` 更能反映 planner 的稳定性

总体 `10 秒级` 卡顿比例：

- `RRTstar`: `60/60`
- `RRTConnect`: `25/60`
- `BFMT`: `25/60`
- `PRMstar`: `24/60`
- `FMT`: `23/60`
- `LBTRRT`: `17/60`
- `HeuristicGuided`: `0/60`

总体 `<1s` 快速求解比例：

- `LBTRRT`: `43/60`
- `HeuristicGuided`: `40/60`
- `FMT`: `37/60`
- `PRMstar`: `36/60`
- `RRTConnect`: `35/60`
- `BFMT`: `35/60`
- `RRTstar`: `0/60`

## 5. 分场景观察

### 5.1 `Medium_SideSurface`

中位数：

- `FMT`: `72.0 ms`
- `LBTRRT`: `79.0 ms`
- `HeuristicGuided`: `99.0 ms`
- `RRTConnect`: `10020.5 ms`
- `PRMstar`: `10022.0 ms`
- `RRTstar`: `10021.5 ms`
- `BFMT`: `5041.0 ms`

结论：

- 这个场景对 `FMT / LBTRRT / HeuristicGuided` 友好
- 对 `RRTConnect / PRMstar / RRTstar` 非常不友好

### 5.2 `MediumPlus_RightUpperAngled`

中位数：

- `LBTRRT`: `63.0 ms`
- `FMT`: `70.0 ms`
- `HeuristicGuided`: `83.5 ms`
- `RRTConnect`: `68.0 ms`，但不稳定
- `BFMT`: `10013.5 ms`
- `PRMstar`: `10016.5 ms`
- `RRTstar`: `10016.0 ms`

结论：

- 这个场景对 planner 的区分度比 `Medium` 更好
- `LBTRRT / FMT / HeuristicGuided` 更稳
- `BFMT / PRMstar / RRTstar` 更容易卡到上限

### 5.3 `Hard_HoleShallow`

中位数：

- `BFMT`: `55.5 ms`
- `LBTRRT`: `59.5 ms`
- `PRMstar`: `60.0 ms`
- `RRTConnect`: `61.5 ms`
- `HeuristicGuided`: `61.5 ms`
- `FMT`: `5040.0 ms`
- `RRTstar`: `10019.5 ms`

结论：

- `Hard` 不是所有算法都同难
- `FMT / RRTstar` 在这个点上明显弱于其他方法

### 5.4 `HardPlus_HoleEdgeOffset`

中位数：

- `LBTRRT`: `56.0 ms`
- `PRMstar`: `60.5 ms`
- `HeuristicGuided`: `65.5 ms`
- `RRTConnect`: `69.5 ms`
- `BFMT`: `71.0 ms`
- `FMT`: `5041.0 ms`
- `RRTstar`: `10017.5 ms`

结论：

- `Hard+` 已经成功拉开时间差异
- `LBTRRT / PRMstar / HeuristicGuided` 在该点表现更稳
- `FMT / RRTstar` 依然容易卡住

### 5.5 `Extreme_HoleDeep`

中位数：

- `RRTConnect`: `50.0 ms`
- `BFMT`: `59.0 ms`
- `FMT`: `65.0 ms`
- `HeuristicGuided`: `68.5 ms`
- `LBTRRT`: `70.0 ms`
- `PRMstar`: `5035.5 ms`
- `RRTstar`: `10015.0 ms`

结论：

- 极端场景下 `RRTConnect` 反而很强
- `PRMstar / RRTstar` 不适合这个点

## 6. 对论文最有价值的结论

当前 `simple` 实验已经支持下面这些说法：

1. 不同 planner 在受限接触测量任务中存在明显的稳定性差异，而不是简单的“都能成功”。
2. 这种差异主要体现为“是否频繁卡到规划时间上限”，因此只看平均时间不够。
3. `HeuristicGuided` 的最大优势不是绝对最快中位数，而是“没有 10 秒级卡顿”。
4. `LBTRRT` 是目前最强的经典 baseline 候选之一，应当保留到后续主表。
5. `RRTstar` 在当前任务设置下不适合作为主力基线，只适合作为历史对照。

## 7. 下一步建议

最建议优先做的是：

1. 用 `planner_comparison_simple_plot_data.csv` 画每个场景的箱线图
2. 用 `planner_comparison_simple_plot_summary.csv` 画总体成功率和平均时间柱状图
3. 在论文主文中优先报告 `中位数 + IQR + 成功率`
4. 把 `LBTRRT / FMT / RRTConnect / HeuristicGuided` 作为 simple 阶段重点基线
5. 再把同样的数据采集思路迁移到 `v2`

## 8. 一个必须注意的问题

当前直接 `ros2 run` 时仍然有：

- `No kinematics plugins defined`

这次没有阻止结果生成，但它仍然是环境层面的不规范点。进入论文正式实验前，建议把这一项补齐，避免审稿或复现实验时留下隐患。
