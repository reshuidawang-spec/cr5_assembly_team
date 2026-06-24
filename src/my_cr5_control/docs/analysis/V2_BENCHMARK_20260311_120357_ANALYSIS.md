# V2 Benchmark 正式运行分析

## 1. 本次运行

- 实验时间戳：`20260311_120357_091`
- 基准版本：`v2_v20260311_reachable_scenes`
- 重复次数：`10`
- 场景数：`4`
- 规划器数：`7`
- 总样本数：`280`

原始结果文件：

- `test_results/benchmarks/v2/raw/20260311_120357_091_planner_comparison_v2_results.csv`
- `test_results/benchmarks/v2/raw/20260311_120357_091_planner_comparison_v2_summary.csv`

统一绘图文件：

- `test_results/benchmarks/v2/legacy/planner_comparison_v2_plot_data.csv`
- `test_results/benchmarks/v2/legacy/planner_comparison_v2_plot_summary.csv`

## 2. 总体结论

这轮 `v2` 已经从“场景定义错误导致几乎不可达”修复成“可用于正式 baseline 分析”的状态。

当前最重要的结论是：

- `v2` 不再是空结果或伪失败场景
- `7` 个规划器在 `4` 个 STL 场景上全部 `100%` 成功
- 真正拉开差异的指标已经变成 `时间分布` 和 `是否频繁卡到 10 秒预算`

所以这轮 `v2` 的价值和 `simple` 一样，不在于再讨论“能不能解”，而在于：

- 谁更稳定
- 谁更容易触发 10 秒级卡顿
- 谁在真实几何场景里更适合作为论文主 baseline

## 3. 总体表现

### 3.1 成功率

- `RRTConnect`: `100.0%` (`40/40`)
- `RRTstar`: `100.0%` (`40/40`)
- `LBTRRT`: `100.0%` (`40/40`)
- `FMT`: `100.0%` (`40/40`)
- `BFMT`: `100.0%` (`40/40`)
- `PRMstar`: `100.0%` (`40/40`)
- `HeuristicGuided`: `100.0%` (`40/40`)

### 3.2 按平均时间

- `HeuristicGuided`: `296.3 ms`
- `PRMstar`: `2795.9 ms`
- `BFMT`: `3045.2 ms`
- `LBTRRT`: `3294.0 ms`
- `FMT`: `3295.0 ms`
- `RRTConnect`: `3790.6 ms`
- `RRTstar`: `10019.8 ms`

### 3.3 按中位数

- `HeuristicGuided`: `59.0 ms`
- `PRMstar`: `60.0 ms`
- `FMT`: `64.0 ms`
- `RRTConnect`: `64.0 ms`
- `BFMT`: `65.0 ms`
- `LBTRRT`: `65.5 ms`
- `RRTstar`: `10018.0 ms`

## 4. 最重要的统计解释

这轮 `v2` 和 `simple` 一样，仍然是明显双峰分布：

- 一部分样本在 `几十毫秒`
- 一部分样本在 `约 10000 ms`

因此：

- `均值` 仍然会被预算上限样本严重拉高
- `中位数` 更接近 planner 的典型求解速度
- `>=9s` 的卡顿比例是非常关键的稳定性指标

总体 `>=9s` 卡顿比例：

- `RRTstar`: `40/40`
- `RRTConnect`: `15/40`
- `FMT`: `13/40`
- `LBTRRT`: `13/40`
- `BFMT`: `12/40`
- `PRMstar`: `11/40`
- `HeuristicGuided`: `0/40`

总体 `<1s` 快速求解比例：

- `HeuristicGuided`: `30/40`
- `PRMstar`: `29/40`
- `BFMT`: `28/40`
- `FMT`: `27/40`
- `LBTRRT`: `27/40`
- `RRTConnect`: `25/40`
- `RRTstar`: `0/40`

这里最关键的一句结论是：

- `HeuristicGuided` 的优势不是“每个场景都压倒性最小中位数”，而是它在 `v2` 上依旧没有出现 `10 秒级卡顿`

## 5. 分场景观察

### 5.1 `Easy_HoleCenter`

按中位数：

- `PRMstar`: `49.5 ms`
- `FMT`: `54.5 ms`
- `RRTConnect`: `55.5 ms`
- `HeuristicGuided`: `60.0 ms`
- `LBTRRT`: `64.0 ms`
- `BFMT`: `67.0 ms`
- `RRTstar`: `10018.0 ms`

结论：

- `Easy` 场景里最强经典方法是 `PRMstar`
- `HeuristicGuided` 不是最快中位数，但依旧稳定且无长尾

### 5.2 `Medium_HoleEdge`

按中位数：

- `HeuristicGuided`: `52.5 ms`
- `FMT`: `61.5 ms`
- `LBTRRT`: `63.0 ms`
- `PRMstar`: `64.0 ms`
- `BFMT`: `66.0 ms`
- `RRTConnect`: `5036.5 ms`
- `RRTstar`: `10018.0 ms`

结论：

- 中等难度场景已经能有效区分 planner
- `HeuristicGuided` 在这个场景上是明显最优
- `RRTConnect` 在 `v2` 的中等场景上稳定性偏弱

### 5.3 `Hard_DeepInterior`

按中位数：

- `LBTRRT`: `61.5 ms`
- `HeuristicGuided`: `63.5 ms`
- `PRMstar`: `68.5 ms`
- `BFMT`: `69.5 ms`
- `FMT`: `5042.0 ms`
- `RRTConnect`: `5048.5 ms`
- `RRTstar`: `10017.5 ms`

结论：

- `Hard` 场景里 `LBTRRT` 是最强经典 baseline
- `HeuristicGuided` 的中位数几乎追平 `LBTRRT`，但平均时间更低
- `RRTConnect / FMT / RRTstar` 在这个点上明显更容易进入长尾

### 5.4 `Extreme_NarrowPassage`

按中位数：

- `HeuristicGuided`: `58.0 ms`
- `BFMT`: `61.0 ms`
- `RRTConnect`: `61.0 ms`
- `FMT`: `61.0 ms`
- `PRMstar`: `5037.5 ms`
- `LBTRRT`: `10014.5 ms`
- `RRTstar`: `10017.5 ms`

结论：

- 极端场景下 `HeuristicGuided` 的优势最明显
- `BFMT / RRTConnect / FMT` 有快解能力，但波动比 `HeuristicGuided` 大
- `LBTRRT / RRTstar` 在这个点上不适合作为主打方法

## 6. 对论文最有价值的结论

当前 `v2` 实验已经支持下面这些说法：

1. STL 场景修正后，真实几何 benchmark 可以稳定生成可重复结果。
2. 在 `100%` 成功率条件下，planner 差异主要体现在 `长尾卡顿`，而不是单纯成功/失败。
3. `HeuristicGuided` 在 `simple` 和 `v2` 上都表现出一致的稳定性优势，这一点比单次最快时间更有论文价值。
4. `RRTstar` 在当前任务设置下依旧更适合做历史对照，而不适合作为主力 baseline。
5. 经典 baseline 不应只留一个，当前最值得保留的是 `PRMstar / LBTRRT / FMT`，再加 `RRTConnect` 作为工业速度基线。

## 7. 现在最该做什么

最推荐的下一步不是继续手改 benchmark 场景，而是进入学习方法前的最后准备阶段：

1. 固定 `simple + v2` 当前版本，不再频繁改场景定义。
2. 在 CSV 中补充 `首条可行解时间` 和 `是否触发预算上限` 两个字段。
3. 基于当前 `simple + v2` 基准开始自动数据采集，形成第一版训练集。
4. 以 `PRMstar / LBTRRT / FMT / RRTConnect / HeuristicGuided` 作为论文主实验候选集合。
5. 开始做第一版轻量学习模型，不要直接进入深度集成 OMPL。

## 8. 一个必须注意的问题

虽然这轮结果已经可用，但目前仍然存在一个方法学层面的注意点：

- `v2` 当前的测点依然是“基于 STL 包围盒和开口启发式”生成的

这意味着它已经适合做论文 baseline 和学习数据采集，但如果后面要把 `v2` 写成“真实几何特征提取算法”，还需要单独补一版：

- 更接近真实孔径/内腔的局部几何分析
- 每个测点的 RViz 可视化截图
- 更清楚的物理解释
