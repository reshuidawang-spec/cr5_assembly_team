# Random Task Dataset Guide

## 1. 第一版节点

第一版随机任务数据采集节点：

```bash
ros2 run my_cr5_control random_task_dataset_simple_node
```

对应源码：

- `src/benchmarks/random_task_dataset_simple.cpp`

## 2. 当前支持的任务族

第一版 `simple` 随机任务会从下面 6 类里随机采样：

- `top_open`
- `front_side`
- `right_upper_angled`
- `hole_shallow`
- `hole_edge`
- `hole_deep`

这 6 类任务已经能自然产生：

- 快速成功样本
- 10 秒级长尾样本
- 明确失败样本

所以它已经适合做 Stage 2 的第一版训练数据采集。

## 3. 环境变量

### 3.1 任务数

默认 `60` 个任务：

```bash
export MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS=60
```

### 3.2 随机种子

如果要复现某次采样：

```bash
export MY_CR5_CONTROL_RANDOM_SIMPLE_SEED=12345
```

### 3.3 规划器集合

默认 planner 集合是：

- `FMT`
- `LBTRRT`
- `RRTConnect`
- `HeuristicGuided`

如果要手动指定：

```bash
export MY_CR5_CONTROL_RANDOM_SIMPLE_PLANNERS=FMT,LBTRRT,RRTConnect,HeuristicGuided
```

## 4. 推荐运行方式

终端 1：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

终端 2：

```bash
source ~/dobot_ws/install/setup.bash
export MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS=100
ros2 run my_cr5_control random_task_dataset_simple_node
```

## 5. 输出文件

节点会输出两类文件：

- `test_results/datasets/simple_random/raw/*_simple_random_task_dataset_results.csv`
- `test_results/datasets/simple_random/raw/*_simple_random_task_dataset_summary.csv`

明细结果包含：

- 任务族
- 任务几何特征
- 难度标签
- planner 成功/失败
- `wall_time_ms`
- `moveit_time_ms`
- `hit_budget_limit`
- `fast_solve_lt_1s`

推荐绘图命令：

```bash
python3 scripts/datasets/plot_simple_random_dataset.py
```

默认输出目录：

- `test_results/datasets/simple_random/plots/<timestamp>/`

## 6. 当前 smoke 结果

第一轮 smoke 文件：

- `test_results/datasets/simple_random/raw/20260311_135953_957_simple_random_task_dataset_results.csv`
- `test_results/datasets/simple_random/raw/20260311_135953_957_simple_random_task_dataset_summary.csv`

这轮 `12` 个随机任务、`4` 个 planner，共 `48` 条样本，已经出现：

- `42` 条成功
- `6` 条失败

说明第一版采集器已经能够产生可用于训练的正负样本混合数据。
