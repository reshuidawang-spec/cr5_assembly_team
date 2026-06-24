# HeuristicGuided Sampling Interface

## 1. 这份文档解决什么问题

这份文档用来固定一个容易写偏的边界：

- 我们研究的是：`机械臂接触测量场景下的受限路径规划`
- 我们采用的方法是：`learning-guided sampling`

因此：

- `路径规划` 是主任务
- `采样引导` 是方法层
- `HeuristicGuided` 是当前工程中承载这条方法线的接口

## 2. 当前工程映射

当前 `HeuristicGuided` 在工程里的含义不是：

- 端到端轨迹生成
- 自定义 OMPL sampler 插件
- 直接替代底层规划器

当前它实际对应的是：

- 一个外部两阶段采样引导原型
- benchmark 中 `planning_mode = heuristic_guided`
- 入口函数：`CR5Robot::planToPoseImproved(...)`
- 可替换 ranking 入口：
  - `CR5Robot::setGuideRankingFunction(...)`
  - `CR5Robot::clearGuideRankingFunction()`

相关代码位置：

- `include/my_cr5_control/cr5_robot.hpp`
- `src/core/cr5_robot.cpp`
- `src/benchmarks/planner_comparison_simple.cpp`
- `src/benchmarks/planner_comparison_v2.cpp`
- `src/benchmarks/random_task_dataset_simple.cpp`

## 3. 当前接口语义

当前应把 `HeuristicGuided` 理解成下面这条链路：

1. 输入：
   - `start_state`
   - `target_pose`
   - `planning_budget`
2. 生成一批 `guide pose` 候选
3. 对候选做启发式代价排序
4. 尝试 `start -> guide -> goal` 两段规划
5. 若两段方案不优或失败，则回退 direct plan

当前还新增了一条可选扩展线：

- 在高难度场景下，可选启用 `guide-bridge`
- 即额外尝试：
  - `start -> guide_1 -> guide_2 -> goal`
- 这条扩展用于把 `RRT*-Connect` 类“多中间节点连接”的思想，保守地接入现有机械臂规划接口
- 默认关闭，不覆盖当前论文主线默认结果

所以它本质上是：

- `采样/中间引导点选择接口`

而不是：

- `轨迹直接生成接口`

## 4. 当前实现细节

当前默认 guide 候选来自：

- `generateEllipsoidGuideSamples(...)`

当前默认候选排序依据来自：

- `computeImprovedPathCost(...)`
- `computeDirectPathCost(...)`

当前默认行为仍然是：

- 不注入 custom ranker 时，仍由工程内置 heuristic baseline 决定排序
- 当前 baseline 已不是纯 `heuristic_cost`
- 而是：
  - `heuristic_cost`
  - `clearance-aware` 安全惩罚
  - `IK / manipulability` 惩罚
  - `top-seed local refinement` 后的候选重排

当前实现特征：

- 带目标偏置
- 带椭球分布采样
- 带障碍区域惩罚
- 带 `IK / manipulability` 可达性过滤
- 带 `clearance-aware` 安全评分
- 带 `top-seed local refinement` 局部增密重采样
- 可选带 `guide-pair bridge` 多级连接尝试
- 输出仍然交给底层 MoveIt/OMPL 做路径生成
- 当前 `IK / manipulability` 评估必须注意运行时环境：
  - 若 `cr5_group` 有 kinematics solver，则使用真实 IK + Jacobian 评估
  - 若当前 MoveIt 环境没有 solver，则回退到：
    - `seed-state Jacobian`
    - `rough geometric reachability`
  - 不能把“当前环境缺少 solver”误当成“candidate 本身不可行”

`2026-03-17` 新增的这批增强，工程意义是：

- 不直接替换底层 planner
- 先把 `constraint-aware sampling / safety filter / local refinement` 这些先进模块吸收进当前 heuristic baseline
- 让当前 `HeuristicGuided` 先具备更像论文方法雏形的结构

## 5. 后续学习模块应该接在哪里

后续 learning-guided 模块优先替换下面两个位置：

1. `guide pose` 生成分布
2. 候选 guide 的排序/筛选函数

当前工程里第 2 个位置已经具备了明确注入点：

- 通过 `setGuideRankingFunction(...)` 注入自定义 candidate ranking 逻辑
- ranker 可以：
  - 修改 `ranking_score`
  - 禁用部分 candidate
  - 保持底层规划调用逻辑不变

`2026-03-17` 当前已经落地第一版最小闭环：

- 数据采集：
  - `ros2 run my_cr5_control guide_ranking_simple_experiment_node`
  - `MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=collect_dataset`
- 离线训练：
  - `python3 scripts/models/train_guide_ranking_model.py`
- 在线回接：
  - `MY_CR5_CONTROL_GUIDE_EXPERIMENT_MODE=ablation`
  - `MY_CR5_CONTROL_GUIDE_MODEL_PATH=<...>/linear_model.csv`

这说明：

- `guide candidate ranking` 已经不再只是概念接口
- 学习模型现在可以真实接回 `HeuristicGuided`
- 但当前首版线性模型还不能宣称有效

当前首轮 simple 消融结果是：

- `heuristic_guided`: 成功率 `100%`，预算命中 `0/12`
- `learned_guided`: 成功率 `91.7%`，预算命中 `12/12`

`2026-03-17` 第二轮 refined 重跑后，一度确认了：

- `candidate_viable_refined` online ablation：
  - `learned_guided` 仍然 `12/12` 命中预算上限
- `candidate_preferred_refined` online ablation：
  - `learned_guided` 仍然 `12/12` 命中预算上限

所以当时问题收敛为：

- 不是接口没打通
- 不是候选几何/安全特征取不到
- 而是 `线性分类分数 -> online 排序` 这一步策略不对

`2026-03-18` 已进一步把 online policy 改成：

- `top-k gating + heuristic fallback`

即：

- 模型只决定“哪些少量 candidate 值得尝试”
- 不再把全量 candidate 都拖进在线尝试队列
- 若模型不自信，则回退到 `heuristic best`

当前最新 simple ablation 结果说明：

- `candidate_viable_refined + top-k gating`
  - 成功率 `100.0%`
  - 预算命中 `0/12`
  - 平均 `1226.1 ms`
- `candidate_preferred_refined + top-k gating`
  - 成功率 `91.7%`
  - 预算命中 `0/12`
  - 平均 `1501.2 ms`

这意味着当前 learned guidance 的状态已经从：

- `系统性撞满预算`

变成：

- `预算稳定性明显恢复，但速度仍落后 heuristic baseline`

因此现阶段应把这条线表述为：

- `online learned guidance interface 已打通`
- `raw-logit full sort 已被证明不对`
- `top-k gating 已经修复预算塌陷`
- `当前下一步不再是“能不能跑通”，而是“能不能在不失稳的前提下把速度做上去”`

可加入的信号包括：

- 几何特征
- 运动学特征
- 历史预算命中风险
- fast/slow solve 先验

短期内不建议做的事：

- 直接让网络输出整条轨迹
- 一开始就强改 OMPL 内部 sampler
- 把 benchmark 主线改成“纯采样算法论文”

## 6. 对论文写作的约束

后续写论文时统一使用下面这套说法：

- 问题层：`constrained path planning for robotic contact measurement`
- 方法层：`learning-guided sampling`
- 工程承载：`HeuristicGuided two-stage guidance interface`

最稳的统一句式是：

> We study constrained path planning for robotic contact measurement and improve it through a learning-guided sampling interface that ranks intermediate guide poses before final motion planning.

## 7. 当前结论

当前工程和论文应对齐为：

- 主线不是“测量”对“采样”二选一
- 而是“测量场景下的路径规划”作为问题
- “采样引导”作为算法创新抓手
- `HeuristicGuided` 作为两者之间的工程接口
- 当前 learned guidance 已经接回该接口，但首版模型还没有形成可用于论文主结论的正向收益
