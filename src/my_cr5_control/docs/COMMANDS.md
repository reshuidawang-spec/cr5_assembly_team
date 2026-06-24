# Terminal Commands

## 1. 维护约定

- 这个文档是当前工程“用户可在终端直接执行”的统一命令入口。
- 以后每新增一个可执行命令，必须在同一次改动里同步更新本文件。
- 这里的“可执行命令”包括：
  - `ros2 run ...`
  - `ros2 launch ...`
  - `python3 scripts/<category>/<tool>.py`
  - 新增环境变量驱动的标准工作流

## 2. 基础环境

工作区编译：

```bash
cd ~/dobot_ws
colcon build --packages-select my_cr5_control
```

重编 modern baseline 所需的 MoveIt OMPL overlay：

```bash
cd ~/dobot_ws
colcon build --packages-select moveit_planners_ompl cr5_moveit
```

说明：

- 当前 overlay 源码位于：
  - `/home/zhu/dobot_ws/src/_overlay_moveit2/moveit2-humble/moveit_planners/ompl`

加载环境：

```bash
source ~/dobot_ws/install/setup.bash
```

如果只想重新加载当前工作区环境：

```bash
source install/setup.bash
```

## 3. MoveIt 依赖

启动外部 `cr5_moveit` demo：

```bash
ros2 launch cr5_moveit demo.launch.py
```

## 4. ROS 2 节点

启动基础规划节点：

```bash
ros2 run my_cr5_control cr5_planner_node
```

启动更稳妥的 CR5 MoveIt demo（自动补激活 `cr5_group_controller`）：

```bash
ros2 launch my_cr5_control cr5_moveit_stable_demo.launch.py
```

运行 synthetic guidance legacy demo（非论文主线，只是 synthetic 场景演示）：

```bash
ros2 run my_cr5_control synthetic_guidance_demo_node
```

运行论文主线的 `simple` 箱体 RViz 可视化：

```bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py \
  benchmark:=simple \
  scene:=Hard_HoleShallow
```

运行论文主线的 `v2 / WS119` RViz 可视化：

```bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py \
  benchmark:=v2 \
  scene:=Hard_DeepInterior
```

运行 HeuristicGuided 场景可视化诊断（默认 `simple / Medium_SideSurface`，会在 RViz 中显示起点、目标点、top guide 候选与 direct/heuristic 诊断摘要）：

```bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py
```

运行指定 simple 长尾场景的可视化诊断：

```bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py \
  benchmark:=simple \
  scene:=Medium_SideSurface
```

运行指定 v2 场景的可视化诊断：

```bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py \
  benchmark:=v2 \
  scene:=Easy_HoleCenter
```

直接运行可视化诊断节点（若已手动启动 `cr5_moveit demo.launch.py`）：

```bash
MY_CR5_CONTROL_DEBUG_BENCHMARK=simple \
MY_CR5_CONTROL_DEBUG_SCENE=Medium_SideSurface \
ros2 run my_cr5_control heuristic_guided_visual_debug_node
```

运行 experimental task-space RRT* 机械臂测试（自写采样/扩展/重连 + 地板 + 简单障碍物）：

```bash
ros2 run my_cr5_control cr5_experimental_task_space_rrtstar_demo_node
```

推荐方式：一键启动稳定版 MoveIt + controller 激活 + experimental task-space RRT* 测试：

```bash
ros2 launch my_cr5_control experimental_task_space_rrtstar_demo.launch.py
```

启动箱体 RViz 示教测试：

```bash
ros2 run my_cr5_control cr5_box_tester_node
```

启动 Qt GUI：

```bash
ros2 run my_cr5_control cr5_gui_node
```

启动活塞喷涂基础 GUI：

```bash
ros2 run my_cr5_control piston_spray_gui_node
```

一键启动活塞喷涂真机 GUI：

```bash
ros2 launch my_cr5_control piston_spray_real_robot.launch.py
```

运行旧版通用规划器对比节点：

```bash
ros2 run my_cr5_control planner_comparison_node
```

运行 simple benchmark：

```bash
ros2 run my_cr5_control planner_comparison_simple_node
```

运行 simple benchmark 并指定重复次数：

```bash
MY_CR5_CONTROL_SIMPLE_REPEATS=10 ros2 run my_cr5_control planner_comparison_simple_node
```

运行 simple benchmark 并手动指定 planner 集合：

```bash
MY_CR5_CONTROL_SIMPLE_PLANNERS=RRTConnect,PRMstar,HeuristicGuided \
ros2 run my_cr5_control planner_comparison_simple_node
```

运行 simple benchmark 并只筛选指定场景子集：

```bash
MY_CR5_CONTROL_SIMPLE_SCENES=Hard_HoleShallow,HardPlus_HoleEdgeOffset,Extreme_HoleDeep \
ros2 run my_cr5_control planner_comparison_simple_node
```

运行 v2 benchmark：

```bash
ros2 run my_cr5_control planner_comparison_v2_node
```

运行 v2 benchmark 并指定重复次数：

```bash
MY_CR5_CONTROL_V2_REPEATS=10 ros2 run my_cr5_control planner_comparison_v2_node
```

运行 v2 benchmark 并手动指定 planner 集合：

```bash
MY_CR5_CONTROL_V2_PLANNERS=RRTConnect,PRMstar,HeuristicGuided \
ros2 run my_cr5_control planner_comparison_v2_node
```

运行 v2 benchmark 并只筛选指定场景子集：

```bash
MY_CR5_CONTROL_V2_SCENES=Easy_HoleCenter,Hard_DeepInterior,Extreme_NarrowPassage \
ros2 run my_cr5_control planner_comparison_v2_node
```

说明：

- `planner_comparison_simple_node` 优先读取 `MY_CR5_CONTROL_SIMPLE_PLANNERS`。
- `planner_comparison_v2_node` 优先读取 `MY_CR5_CONTROL_V2_PLANNERS`。
- `planner_comparison_simple_node` 优先读取 `MY_CR5_CONTROL_SIMPLE_SCENES`。
- `planner_comparison_v2_node` 优先读取 `MY_CR5_CONTROL_V2_SCENES`。
- 两者都会回退读取共享变量 `MY_CR5_CONTROL_BENCHMARK_PLANNERS`。
- 两者也都会回退读取共享变量 `MY_CR5_CONTROL_BENCHMARK_SCENES`。
- 当前默认 planner 集合保持不变。
- `BITstar / InformedRRTstar / ABITstar / AITstar / EITstar` 已可通过环境变量名字被识别，但是否真能运行不只取决于项目 YAML。
- 当前工作区通过 overlay `moveit_planners_ompl` 扩展了 `BITstar / InformedRRTstar` 的运行时注册。
- 如果只回到 `/opt/ros/humble` 自带二进制环境，单改 `cr5_moveit/config/ompl_planning.yaml` 仍不足以启用它们。
- 开跑 modern baseline 前，先执行 `python3 scripts/benchmarks/test_bitstar.py`。
- `2026-03-19` smoke 结果：
  - `BITstar` 已完成运行时注册，也能完成单次/短程规划请求。
  - 但在干净隔离域 `ROS_DOMAIN_ID=88` 下执行
    `ros2 launch my_cr5_control planner_benchmark.launch.py benchmark:=simple repeats:=10 planners:=BITstar`
    时，`move_group` 会在第 1 轮 `Extreme_HoleDeep` 场景内复现 `SIGSEGV`，堆栈落在 `ompl::geometric::BITstar::publishSolution()`。
  - `InformedRRTstar` 经过 overlay 中的 goal warm-up 修复后，已不再触发先前的 `PathLengthDirectInfSampler` 初始化报错，并且可以真实完成规划请求。
  - 但当前 `InformedRRTstar simple` smoke 仍只有 `83.3% (5/6)` 成功率，且 `6/6` 命中 `10 s` 预算。
  - 因此当前 modern planner 结论应统一为：
    - `test_bitstar.py` 的 `READY` 只代表“配置 + 运行时注册完成”
    - `BITstar / InformedRRTstar` 目前都不建议直接纳入正式主表

运行 simple 随机任务采集：

```bash
ros2 run my_cr5_control random_task_dataset_simple_node
```

运行 simple 随机任务采集并指定任务数：

```bash
MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS=300 ros2 run my_cr5_control random_task_dataset_simple_node
```

运行 simple 随机任务采集并固定种子：

```bash
MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS=300 \
MY_CR5_CONTROL_RANDOM_SIMPLE_SEED=3551515123 \
ros2 run my_cr5_control random_task_dataset_simple_node
```

运行 simple 随机任务采集并手动指定 planner 集合：

```bash
MY_CR5_CONTROL_RANDOM_SIMPLE_PLANNERS=FMT,LBTRRT,RRTConnect,HeuristicGuided \
ros2 run my_cr5_control random_task_dataset_simple_node
```

运行 simple guide candidate 数据采集：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=collect_dataset \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

用 launch 在隔离 domain 中运行 guide experiment：

```bash
ROS_DOMAIN_ID=88 ros2 launch my_cr5_control guide_experiment.launch.py
```

运行 simple guide candidate 数据采集并指定重复次数、候选数和预算：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=collect_dataset \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=12 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

运行 simple guide candidate 数据采集并固定候选采样流：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=collect_dataset \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=12 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

运行 learned guidance 对照消融：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/<timestamp>/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

用 launch 在隔离 domain 中运行 fixed-seed learned guidance 公平对照消融：

```bash
ROS_DOMAIN_ID=88 ros2 launch my_cr5_control guide_experiment.launch.py \
  mode:=ablation \
  repeats:=3 \
  sample_count:=12 \
  budget_s:=4.0 \
  scenes:=Easy_TopCenter,Hard_HoleShallow \
  sample_seed:=20260318 \
  model_path:=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
  top_k:=1 \
  threshold:=0.55 \
  selection_mode:=top_prob \
  direct_gate_mode:=off
```

说明：

- `guide_experiment.launch.py` 现在走 `cr5_moveit/demo.launch.py` 启动链，因此会带上 fake controller 与 `joint_states`。
- 实验节点结束后 launch 会自动退出，结果写到 `test_results/benchmarks/simple_guidance/raw/`。

用 launch 在隔离 domain 中运行 benchmark 并筛选 hard subset：

```bash
ROS_DOMAIN_ID=88 ros2 launch my_cr5_control planner_benchmark.launch.py \
  benchmark:=simple \
  repeats:=3 \
  planners:=HeuristicGuided \
  scenes:=Hard_HoleShallow,HardPlus_HoleEdgeOffset,Extreme_HoleDeep
```

说明：

- `planner_benchmark.launch.py` 现在支持 `scenes:=...`，会透传到 `MY_CR5_CONTROL_BENCHMARK_SCENES`。
- benchmark 节点结束后 launch 会自动退出，不再需要手动中断。

分析 `HeuristicGuided` raw benchmark，筛查哪些场景值得做 rescue / gate 细化：

```bash
python3 scripts/benchmarks/analyze_heuristic_rescue_candidates.py \
  --baseline-results test_results/benchmarks/simple/raw/<baseline>_planner_comparison_simple_results.csv \
  --compare-results test_results/benchmarks/simple/raw/<compare>_planner_comparison_simple_results.csv \
  --slow-threshold-ms 800
```

说明：

- 这个脚本会输出：
  - `scene_summary.csv`
  - `scene_compare.csv`
  - `recommended_subset.txt`
- 默认分析 `HeuristicGuided + heuristic_guided`。
- 默认输出目录：
  - `test_results/exports/heuristic_rescue_analysis/<timestamp>/`

分析默认 `HeuristicGuided` 当前到底是“active guidance”还是“direct-first + dormant near-tie gate”：

```bash
python3 scripts/benchmarks/analyze_heuristic_gate_activity.py \
  --results test_results/benchmarks/simple/raw/<timestamp>_planner_comparison_simple_results.csv \
  --direct-slow-threshold-ms 800 \
  --near-tie-delta 0.005
```

说明：

- 这个脚本会输出：
  - `scene_summary.csv`
  - `overall_summary.json`
- 关键看：
  - `guide_attempt_rate`
  - `direct_fallback_rate`
  - `mean_delta_h`
  - `scene_classification`
- 默认输出目录：
  - `test_results/exports/heuristic_gate_activity/<timestamp>/`

运行 fixed-seed learned guidance 公平对照消融：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=3 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

运行 direct-gated learned guidance 对照消融：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=3 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

运行共享 direct baseline 且只在“未撞预算的 direct”上拦 guide 的对照消融：

```bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=HardPlus_HoleEdgeOffset,Extreme_HoleDeep \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct_no_budget_hit \
MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE=1 \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

说明：

- 当前 learned ablation 已不是“全量候选按线性 logit 直接排序”。
- 当前默认 online policy 是：`top-k gating + heuristic fallback`。
- 推荐先从 `top_k=1`、`threshold=0.55` 开始。
- 如果当前 shell 里可能残留旧的 MoveIt / benchmark 进程，优先改用 `guide_experiment.launch.py` 并设置独立 `ROS_DOMAIN_ID`。
- 如果要做 heuristic vs learned 的公平对照，建议固定：
  - `MY_CR5_CONTROL_GUIDE_SCENES`
  - `MY_CR5_CONTROL_GUIDE_SAMPLE_SEED`
- 固定 `MY_CR5_CONTROL_GUIDE_SAMPLE_SEED` 后：
  - `collect_dataset` 会复用同一候选生成流
  - `ablation` 会让同一 `repeat + scene` 下 heuristic 与 learned 看到相同 guide candidate 流
- 当前单候选选择默认是：
  - `MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob`
- 如果要让 learned guidance 在 direct 已明显更优时直接回退，可额外指定：
  - `MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct`
- 如果只想在“direct 成功且未撞预算”时才拦 guide，推荐指定：
  - `MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct_no_budget_hit`
- 如果要让 heuristic / learned 共享同一条 scenario-level direct baseline，可额外指定：
  - `MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE=1`
- 如果要实验 retained set 内部二次排序，可额外指定：
  - `MY_CR5_CONTROL_GUIDE_RETAINED_ORDER=heuristic`
- 如果要实验单候选 hybrid 选择，可额外指定：
  - `MY_CR5_CONTROL_GUIDE_SELECTION_MODE=hybrid`
  - `MY_CR5_CONTROL_GUIDE_HYBRID_ALPHA=2.0`

检查 `WS119` 箱体/STL 特征提取结果：

```bash
ros2 run my_cr5_control inspect_box_features_node
```

在 RViz 中可视化 `WS119` 的 mesh / hole / cavity / measurement points：

```bash
ros2 run my_cr5_control visualize_box_features_node
```

运行论文 `RRT*-Connect` 二维复现 TEST：

```bash
ros2 run my_cr5_control test_rrtstar_connect_reproduction_node
```

基于 `path_points.csv` 生成一个简单的二维动画 GIF；如果同目录下存在 `tree_nodes.csv`，会自动叠加起点树和终点树：

```bash
python3 scripts/operations/animate_rrtstar_connect.py --show-summary
```

## 5. Launch 入口

启动 `my_cr5_control` 自带规划 launch：

```bash
ros2 launch my_cr5_control run_cr5_planner.launch.py
```

启动活塞喷涂真机联机 launch：

```bash
ros2 launch my_cr5_control piston_spray_real_robot.launch.py
```

一条命令启动 `cr5_moveit demo + benchmark`：

```bash
ros2 launch my_cr5_control planner_benchmark.launch.py benchmark:=simple repeats:=10
```

运行 v2 benchmark：

```bash
ros2 launch my_cr5_control planner_benchmark.launch.py benchmark:=v2 repeats:=10
```

自定义 benchmark 启动延迟：

```bash
ros2 launch my_cr5_control planner_benchmark.launch.py \
  benchmark:=simple \
  repeats:=10 \
  start_delay:=5.0
```

通过 launch 统一指定 benchmark planner 集合：

```bash
ros2 launch my_cr5_control planner_benchmark.launch.py \
  benchmark:=simple \
  repeats:=10 \
  planners:=RRTConnect,PRMstar,HeuristicGuided
```

## 6. Python 脚本

自动顺序运行 `simple + v2` benchmark 并导出统一训练表：

```bash
python3 scripts/benchmarks/collect_benchmark_dataset.py
```

只跑 simple：

```bash
python3 scripts/benchmarks/collect_benchmark_dataset.py --benchmarks simple --simple-repeats 10
```

自定义 simple / v2 重复次数：

```bash
python3 scripts/benchmarks/collect_benchmark_dataset.py --simple-repeats 5 --v2-repeats 5
```

只做导出，不重跑 benchmark：

```bash
python3 scripts/benchmarks/collect_benchmark_dataset.py --export-only
```

将最新 simple random 结果整理成训练表：

```bash
python3 scripts/models/prepare_training_table.py
```

生成 task-level oracle planner 数据集：

```bash
python3 scripts/models/prepare_oracle_planner_dataset.py
```

训练第一版 baseline 分类模型：

```bash
python3 scripts/models/train_baseline_model.py
```

只在经典 planner 子集上训练 baseline：

```bash
python3 scripts/models/train_baseline_model.py \
  --planners FMT LBTRRT RRTConnect
```

只训练预算命中预测模型：

```bash
python3 scripts/models/train_baseline_model.py --targets hit_budget_limit
```

训练第一版 guide ranking 线性模型：

```bash
python3 scripts/models/train_guide_ranking_model.py --target candidate_viable
```

训练 direct-bad 场景下的 guide rescue 模型：

```bash
python3 scripts/models/train_guide_ranking_model.py --target candidate_direct_rescue
```

从最近一轮 attempted-guide trace 生成 triggered guide 子集：

```bash
python3 scripts/models/prepare_triggered_guide_dataset.py
```

手动指定 attempted trace 与源数据集，生成 triggered guide 子集：

```bash
python3 scripts/models/prepare_triggered_guide_dataset.py \
  --attempted-csvs test_results/exports/guide_ablation_trace/<timestamp>/attempted_only.csv \
  --source-dataset test_results/datasets/guide_ranking_simple/raw/<timestamp>_guide_ranking_simple_dataset_results.csv
```

直接从多轮 ablation results 回建更大的 triggered guide 子集：

```bash
python3 scripts/models/prepare_triggered_guide_dataset.py \
  --results-csvs \
  test_results/benchmarks/simple_guidance/raw/<timestamp1>_learned_guidance_simple_ablation_results.csv \
  test_results/benchmarks/simple_guidance/raw/<timestamp2>_learned_guidance_simple_ablation_results.csv
```

在 triggered 子集上训练 budget-rescue guide 模型：

```bash
python3 scripts/models/train_guide_ranking_model.py \
  --dataset test_results/datasets/guide_ranking_simple/filtered/<timestamp>_guide_ranking_simple_triggered_dataset_results.csv \
  --target candidate_budget_rescue
```

在 triggered 子集上训练带 direct rescue 特征的 budget-rescue 模型：

```bash
python3 scripts/models/train_guide_ranking_model.py \
  --dataset test_results/datasets/guide_ranking_simple/filtered/<timestamp>_guide_ranking_simple_triggered_dataset_results.csv \
  --target candidate_budget_rescue \
  --feature-profile rescue
```

手动指定数据集训练 guide ranking 模型：

```bash
python3 scripts/models/train_guide_ranking_model.py \
  --dataset test_results/datasets/guide_ranking_simple/raw/<timestamp>_guide_ranking_simple_dataset_results.csv \
  --target candidate_fast
```

分析最新一轮 guide ablation trace：

```bash
python3 scripts/models/analyze_guide_ablation_trace.py
```

分析指定 ablation 结果并导出 attempted-guide 子集：

```bash
python3 scripts/models/analyze_guide_ablation_trace.py \
  --results test_results/benchmarks/simple_guidance/raw/<timestamp>_learned_guidance_simple_ablation_results.csv
```

说明：

- `train_guide_ranking_model.py` 在未显式传 `--dataset` 时，当前会默认跳过小于 `100` 行的 guide dataset，避免误吃可复现性验证用的小样本。
- `prepare_triggered_guide_dataset.py` 除了读 `attempted_only.csv`，现在也支持直接读取一批 ablation `results.csv` 并自动提取 attempted rows。
- 当前可直接训练的 guide 目标包括：
  - `candidate_viable`
  - `candidate_fast`
  - `candidate_preferred`
  - `candidate_direct_rescue`
  - `candidate_budget_rescue`
- `train_guide_ranking_model.py` 现在支持：
  - `--feature-profile geometric`
  - `--feature-profile rescue`
- `rescue` profile 会在几何候选特征之外，再加入 direct baseline 状态：
  - `difficulty_score_raw`
  - `direct_success_flag`
  - `direct_hit_budget_flag`
  - `direct_bad_flag`
  - `direct_wall_time_ratio`
  - `direct_moveit_time_ratio`

评估最新一次 baseline 训练 run：

```bash
python3 scripts/models/evaluate_baseline_model.py
```

评估指定 run：

```bash
python3 scripts/models/evaluate_baseline_model.py \
  --run-dir test_results/models/simple_random_baseline/<timestamp>
```

运行离线 planner 选择实验：

```bash
python3 scripts/models/evaluate_planner_selection.py
```

只在经典 baseline 子集里评估 planner 选择：

```bash
python3 scripts/models/evaluate_planner_selection.py \
  --run-dir test_results/models/simple_random_baseline/<timestamp> \
  --planners FMT LBTRRT RRTConnect
```

训练直接预测 oracle planner 的模型：

```bash
python3 scripts/models/train_oracle_planner_model.py \
  --dataset test_results/exports/simple_random_oracle_planner_dataset_FMT_LBTRRT_RRTConnect.csv
```

评估 oracle planner 模型：

```bash
python3 scripts/models/evaluate_oracle_planner_model.py \
  --run-dir test_results/models/simple_random_oracle_planner/<timestamp>
```

导出最新 `simple + v2` 为统一数据集：

```bash
python3 scripts/benchmarks/export_benchmark_dataset.py
```

手动指定输入结果文件再导出：

```bash
python3 scripts/benchmarks/export_benchmark_dataset.py \
  --inputs \
  test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_results.csv \
  test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv
```

绘制 simple benchmark 图表：

```bash
python3 scripts/benchmarks/plot_simple_benchmark.py \
  --benchmark-version simple_v20260311_6scene_metrics \
  --timestamp 20260311_124449_790
```

绘制 v2 benchmark 图表：

```bash
python3 scripts/benchmarks/plot_simple_benchmark.py \
  --plot-data test_results/benchmarks/v2/aggregates/planner_comparison_v2_plot_data_metrics.csv \
  --plot-summary test_results/benchmarks/v2/aggregates/planner_comparison_v2_plot_summary_metrics.csv \
  --benchmark-version v2_v20260311_reachable_scenes_metrics \
  --timestamp 20260317_142203_372 \
  --output-dir test_results/benchmarks/v2/plots/v2_v20260311_reachable_scenes_metrics/20260317_142203_372
```

绘制最新 simple random dataset 图表：

```bash
python3 scripts/datasets/plot_simple_random_dataset.py
```

手动指定 random dataset 结果文件出图：

```bash
python3 scripts/datasets/plot_simple_random_dataset.py \
  --results test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv
```

分析最新 simple random dataset：

```bash
python3 scripts/datasets/analyze_simple_random_dataset.py
```

手动分析指定 random dataset：

```bash
python3 scripts/datasets/analyze_simple_random_dataset.py \
  --results test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv
```

修复历史 simple random dataset 的 `MoveIt规划时间(ms)` 脏值并重建摘要：

```bash
python3 scripts/maintenance/repair_simple_random_dataset_metrics.py \
  test_results/datasets/simple_random/raw/20260311_140426_647_simple_random_task_dataset_results.csv \
  test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv
```

重建高价值数据 manifest：

```bash
python3 scripts/maintenance/build_dataset_manifest.py
```

快速检查当前 MoveIt / OMPL 规划器环境：

```bash
python3 scripts/benchmarks/test_bitstar.py
```

如果要把 modern baseline 检查作为硬条件：

```bash
python3 scripts/benchmarks/test_bitstar.py --require-modern
```

## 7. 常用环境变量

- `MY_CR5_CONTROL_RESULTS_DIR`
  - 覆盖默认结果输出目录。

- `MY_CR5_CONTROL_SIMPLE_REPEATS`
  - 控制 `planner_comparison_simple_node` 的重复次数。

- `MY_CR5_CONTROL_V2_REPEATS`
  - 控制 `planner_comparison_v2_node` 的重复次数。

- `MY_CR5_CONTROL_BENCHMARK_PLANNERS`
  - 为 `planner_comparison_simple_node`、`planner_comparison_v2_node` 和旧版 `planner_comparison_node` 提供共享 planner 集合。

- `MY_CR5_CONTROL_SIMPLE_PLANNERS`
  - 覆盖 simple benchmark 使用的 planner 集合。

- `MY_CR5_CONTROL_V2_PLANNERS`
  - 覆盖 v2 benchmark 使用的 planner 集合。

- `MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS`
  - 控制 `random_task_dataset_simple_node` 生成的任务数。

- `MY_CR5_CONTROL_RANDOM_SIMPLE_SEED`
  - 固定 simple 随机任务采样种子。

- `MY_CR5_CONTROL_RANDOM_SIMPLE_PLANNERS`
  - 指定 simple 随机任务采集使用的 planner 集合。

- `MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE`
  - 控制 `guide_ranking_simple_experiment_node` 运行 `collect_dataset` 或 `ablation` 模式。

- `MY_CR5_CONTROL_GUIDE_REPEATS`
  - 控制 guide ranking 实验的重复次数。

- `MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT`
  - 控制每个场景采样的 guide candidate 数量。
  - `collect_dataset` 与 `ablation` 现在都会读取这个值。
  - `ablation` 未显式设置时默认仍使用 `24`，以兼容历史 HeuristicGuided 行为。

- `MY_CR5_CONTROL_GUIDE_BUDGET_S`
  - 控制 guide ranking 实验的单次规划预算秒数。

- `MY_CR5_CONTROL_GUIDE_SCENES`
  - 控制 guide ranking 实验只运行指定的场景子集，多个场景用逗号分隔。

- `MY_CR5_CONTROL_GUIDE_SAMPLE_SEED`
  - 固定 guide candidate 采样流。
  - 在 `collect_dataset` 模式下可复现实验候选几何特征。
  - 在 `ablation` 模式下可让同一 `repeat + scene` 的 heuristic 与 learned 复用同一候选流，降低比较条件偏差。

- `MY_CR5_CONTROL_GUIDE_MODEL_PATH`
  - 在 `ablation` 模式下指定在线加载的线性 ranker 模型路径。

- `MY_CR5_CONTROL_GUIDE_MODEL_TOP_K`
  - 控制 learned ablation 中允许进入在线尝试队列的高置信 candidate 数量。

- `MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD`
  - 控制 learned ablation 中 candidate 被保留所需的最小预测概率阈值。

- `MY_CR5_CONTROL_GUIDE_SELECTION_MODE`
  - 控制通过 gating 后如何选择 learned candidate。
  - 当前支持：
    - `top_prob`
    - `heuristic_gate`
    - `hybrid`
  - 当前稳定默认值是：
    - `top_prob`

- `MY_CR5_CONTROL_GUIDE_HYBRID_ALPHA`
  - 当 `MY_CR5_CONTROL_GUIDE_SELECTION_MODE=hybrid` 时，控制 learned probability 对 heuristic cost 的权重。

- `MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE`
  - 控制 learned guidance 是否在 direct plan 已成功且 candidate 不优于 direct 时直接回退。
  - 当前支持：
    - `off`
    - `beat_direct`
    - `beat_direct_no_budget_hit`
  - 推荐先用：
    - `beat_direct_no_budget_hit`

- `MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE`
  - 控制 `ablation` 是否让 heuristic / learned 共享同一条 scenario-level direct baseline。
  - 设为 `1` 后：
    - heuristic / learned 不会再各自重新跑一条随机 direct 作为内部基线
    - 结果更适合做固定条件对比

- `MY_CR5_CONTROL_GUIDE_RETAINED_ORDER`
  - 控制 retained set 内部二次排序方式。
  - 当前支持：
    - `heuristic`
    - `learned`
    - `hybrid`
  - 这是实验性开关，只有 `top_k > 1` 时才真正生效。

- `MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS`
  - 控制一次 `HeuristicGuided` 规划里最多允许尝试多少条 guide route。
  - 默认 `0`
    - 表示不限制
  - 当前用于保守调参，避免 hard case 上 guide 长尾失控。

- `MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS`
  - 实验性开关。
  - 当 direct 已成功，但 direct 本身已经慢到超过该阈值时，允许 heuristic 放宽 direct-cost gate，尝试少量 guide rescue。
  - 默认关闭。
  - 当前不建议作为论文主线默认值直接启用。

- `MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID`
  - 控制 `HeuristicGuided` 的自适应椭球采样是否启用。
  - 论文主线默认启用；未设置时按 `enabled` 处理。
  - 设为 `0/false/off/no` 时显式关闭。
  - 当前已接入：
    - `planner_comparison_simple_node`
    - `planner_comparison_v2_node`
    - `random_task_dataset_simple_node`
    - `guide_ranking_simple_experiment_node`
  - 显式关闭时，仍保持固定参数椭球采样。

- `MY_CR5_CONTROL_ADAPTIVE_ELLIPSOID_FIXED_DIFFICULTY`
  - 可选开关。
  - 为自适应椭球采样指定固定难度评分，范围 `0.0 ~ 1.0`。
  - 未设置时，默认复用各场景 / 任务自带的 `difficulty_score`。

- `MY_CR5_CONTROL_HEURISTIC_GUIDE_BRIDGE`
  - 控制 `HeuristicGuided` 的多级 `guide-bridge` 扩展是否启用。
  - 设为 `1/true/on/yes` 时启用。
  - 启用后，在高难度场景下如果单个 guide 或 direct plan 不理想，会额外尝试：
    - `start -> guide_1 -> guide_2 -> goal`
  - 默认关闭。
  - 当前定位是：
    - `RRT*-Connect` 思路向机械臂工程接口的保守接入
    - 不替代当前论文主线默认配置

- `MY_CR5_CONTROL_HEURISTIC_GUIDE_BRIDGE_MAX_SEQUENCES`
  - 控制最多尝试多少条 `guide-pair bridge` 序列。
  - 默认 `6`
  - 合法范围：`1 ~ 24`

## 8. 推荐工作流

跑正式 simple benchmark：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_SIMPLE_REPEATS=10 ros2 run my_cr5_control planner_comparison_simple_node
```

跑正式 v2 benchmark：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_V2_REPEATS=10 ros2 run my_cr5_control planner_comparison_v2_node
```

跑 simple 随机任务训练数据：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_RANDOM_SIMPLE_TASKS=300 ros2 run my_cr5_control random_task_dataset_simple_node
```

跑第一版 learned guide ranking 闭环：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=collect_dataset \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=12 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

```bash
python3 scripts/models/train_guide_ranking_model.py --target candidate_viable
```

```bash
python3 scripts/models/train_guide_ranking_model.py --target candidate_direct_rescue
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/<timestamp>/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

跑固定候选流的公平对照 ablation：

```bash
source ~/dobot_ws/install/setup.bash
ros2 launch cr5_moveit demo.launch.py
```

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=3 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

```bash
python3 scripts/models/analyze_guide_ablation_trace.py \
  --results test_results/benchmarks/simple_guidance/raw/<timestamp>_learned_guidance_simple_ablation_results.csv
```

跑 direct-gated 的固定候选流公平对照 ablation：

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=3 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```

跑共享 direct baseline 的 harder-subset 对照：

```bash
source ~/dobot_ws/install/setup.bash
MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation \
MY_CR5_CONTROL_GUIDE_REPEATS=2 \
MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT=24 \
MY_CR5_CONTROL_GUIDE_BUDGET_S=4.0 \
MY_CR5_CONTROL_GUIDE_SCENES=HardPlus_HoleEdgeOffset,Extreme_HoleDeep \
MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318 \
MY_CR5_CONTROL_GUIDE_MODEL_TOP_K=1 \
MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD=0.55 \
MY_CR5_CONTROL_GUIDE_SELECTION_MODE=top_prob \
MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct_no_budget_hit \
MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE=1 \
MY_CR5_CONTROL_GUIDE_MODEL_PATH=test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined/linear_model.csv \
ros2 run my_cr5_control guide_ranking_simple_experiment_node
```
