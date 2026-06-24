# Project Memory

## 0E. 2026-05-11 轨迹安全门与碰撞复核约束

- 用户指出机械臂运行时可能穿过地面或障碍物；代码审查确认该风险在工程层面属实：
  - 多个 `move_group_->execute(...)` 路径此前依赖 MoveIt 返回 `SUCCESS`，缺少统一二次碰撞复核。
  - `moveToNamedTarget()` 曾直接调用 `move()`，会绕过自定义安全检查。
  - `moveLine()` 虽调用 Cartesian path，但执行前缺少统一硬校验。
- 当前已经在 `CR5Robot` 内新增统一轨迹安全门：
  - 执行前或对外报告规划成功前，必须从 `/get_planning_scene` 读取当前 planning scene。
  - 校验内容包括 world collision objects、attached probe、allowed collision matrix、关节限位和轨迹格式。
  - 轨迹检查覆盖原始 waypoint，并以 `0.02 rad` 关节步长做插值复核。
  - planning scene 不可用、轨迹为空、关节越界、碰撞或格式异常时一律 fail closed，拒绝执行。
- 后续维护规则：
  - 新增任何会执行轨迹的接口，必须通过 `executePlanIfCollisionFree(...)` 或等价安全门。
  - 新增任何 benchmark / visualization 需要返回轨迹成功状态时，必须先通过 `isPlanCollisionFree(...)` 或 `isTrajectoryCollisionFree(...)`。
  - 论文实验结果若因安全门拒绝不安全轨迹而发生数值变化，应以安全修复后的结果为准。

## 0D. 2026-05-11 正式论文证据优先级覆盖说明

- 若本文档中较早日期的 `post-fix`、`2026-03-24` 或 `10-repeat` 结果与本节冲突，一律以后续 `2026-05-03` / `2026-05-11` formal evidence 为准。
- 当前论文主表应优先引用：
  - unified formal stable-core rerun:
    `paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md`
  - formal ablation:
    `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md`
- 当前主结果：
  - `simple`: `HeuristicGuided` mean `389.5 ms`，success `100.0%`，budget-hit `0.0%`，相对 `RRTConnect` mean reduction `89.0%`
  - `v2/WS119`: `HeuristicGuided` mean `358.4 ms`，success `100.0%`，budget-hit `0.0%`，相对 `RRTConnect` mean reduction `89.4%`
- 当前已解决到 `MANUSCRIPT_READY`：
  - `P0.1` unified formal rerun
  - `P0.2` formal ablation
  - `P0.3` contribution boundary rewrite
  - `P1.1` statistical credibility
  - `P1.2` benchmark objectivity
  - `P1.3` qualitative visual validation
  - `P2.1` method reproducibility
  - `P2.2` related work positioning

## 0C. 2026-05-03 论文审稿风险清单与整改优先级

- 当前已经新增导师视角的论文审稿风险清单：
  - `paper_workspace/docs/PAPER_REVIEW_RISK_REGISTER.md`
- 当前已经新增论文问题解决记录：
  - `paper_workspace/docs/PAPER_REVIEW_RESOLUTION_LOG.md`
- 这个文档固定保存：
  - 当前论文是否满足 Q2/Q3 叙事的判断
  - 最可能被审稿人质疑的问题
  - P0/P1/P2 优先级整改路线
  - 后续需要逐条回答的问题清单
- `PAPER_REVIEW_RESOLUTION_LOG.md` 固定用于记录每个问题最终如何被解决：
  - 实际动作
  - 证据路径
  - 是否写入论文
  - 残留风险
- 后续每解决一个审稿风险问题，必须同步更新 `PAPER_REVIEW_RESOLUTION_LOG.md`。
- 当前导师级判断固定为：
  - 三区：已有合格骨架
  - 二区：需要补强公平重跑、消融实验、真实/半真实验证和贡献边界表述
- 当前最高优先级固定为：
  1. `P0.1` unified formal rerun：baseline 与 HeuristicGuided 同环境、同版本、同配置重跑
  2. `P0.2` ablation suite：拆分 difficulty-adaptive、anchors、ellipsoid enrichment、selective activation、ranking 的贡献
  3. `P0.3` 贡献边界重写：本文是 task-structured guidance layer，不是新 backend planner
- `P0.1` 当前已经推进到 `EVIDENCE_READY`：
  - stable-core unified formal rerun:
    `paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/`
  - analysis:
    `paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md`
  - 结论：
    - `HeuristicGuided` 在 `simple` 与 `v2` 上均为 `100.0%` success、`0.0%` budget-hit
    - 相比本次统一重跑下最佳稳定 baseline `RRTConnect`，mean time 降低约 `89%`
    - `FMT` 在全 planner attempt 中复现 `move_group` 崩溃，`FMT/BFMT` 暂作为 runtime-unstable baseline 单独说明
- 当前下一优先级应转向：
  1. `P0.2` ablation suite
  2. `P0.3` 贡献边界重写
- 2026-05-11 更新：
  - `P0.2` 已完成 full-scene formal ablation，并归档：
    `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md`
  - `P0.3` 已推进到 `MANUSCRIPT_READY`：
    中文草稿已明确本文是 contact-measurement-oriented `task-structured guidance layer`，不是新的 MoveIt/OMPL backend planner；difficulty score 是 geometry-prior driven，不是 online learned estimator；不主张新的渐近最优性或完备性证明。
  - `P1.1` 已推进到 `MANUSCRIPT_READY`：
    已新增 `paper_workspace/docs/P1_1_STATISTICAL_CREDIBILITY_APPENDIX.md`，并在中文草稿补充 P90、max 和 mean 95% CI。下一优先级转向 `P1.2 Benchmark Objectivity`。
  - `P1.2` 已推进到 `MANUSCRIPT_READY`：
    已新增 `paper_workspace/docs/P1_2_BENCHMARK_OBJECTIVITY_NOTE.md`，并在中文草稿写明 `simple` 是 controllable benchmark、`v2/WS119` 是 canonical STL-based realistic geometry benchmark、`v2 hero scene` 仅用于 RViz qualitative visualization 且不进入正式主表。下一优先级转向 `P1.3 Real or Semi-Real Validation`。
  - `P1.3` 已推进到 `MANUSCRIPT_READY`：
    已归档 `paper_workspace/qualitative_results/p1_3_v2_hero_20260511/README.md`、headless visual-debug 日志和 RViz screenshot；direct 命中 `10024.0 ms` budget，HeuristicGuided 生成 `56` 个 guide candidates 并以 `123.0 ms` 成功规划。尚缺录屏/真机执行，但半真实 qualitative trajectory evidence 已可写入论文。
  - `P2.1` 已推进到 `MANUSCRIPT_READY`：
    已新增 `paper_workspace/docs/P2_1_METHOD_REPRODUCIBILITY_NOTE.md`，并在中文草稿 Section 5.6 增加主方法实现参数表，参数均映射到当前代码位置。
  - `P2.2` 已推进到 `MANUSCRIPT_READY`：
    已新增 `paper_workspace/docs/P2_2_RELATED_WORK_POSITIONING_NOTE.md`，重写 related work 对位逻辑，并补充 `references.bib` 中缺失的近五年关键条目。
- 后续新对话如继续推进论文，先读取：
  - `docs/PAPER_MAINLINE_MAP.md`
  - `paper_workspace/docs/PAPER_REVIEW_RISK_REGISTER.md`
  - `paper_workspace/docs/PAPER_REVIEW_RESOLUTION_LOG.md`

## 0A. 2026-04-21 主线边界澄清与命名重构

- 当前论文主线必须严格理解为：
  - `HeuristicGuided two-stage guidance interface`
  - `difficulty-adaptive informed guide sampling`
  - `simple` 可控箱体 benchmark
  - `v2` 基于 `WS119.STL` 的真实几何 benchmark
- 也就是说，当前论文主线不是某一个 demo 节点，而是：
  - `方法 + 两套 benchmark + formal rerun 结果`
- 当前最容易导致误读的旧名字已经明确降级：
  - `cr5_paper_mainline_demo_node`
  - `paper_mainline_demo.launch.py`
- 这两者只对应：
  - synthetic 场景演示
  - legacy compatibility alias
  - 不再代表论文主线
- 当前新的统一边界文档固定为：
  - `docs/PAPER_MAINLINE_MAP.md`
- 今后若文件名、日志名、旧命令与主线定义冲突：
  - 以本节和 `docs/PAPER_MAINLINE_MAP.md` 为准

## 0B. 2026-05-02 v2 hero scene 防混淆锚点

- 当前已经为 `v2/WS119` 主线单独固定展示场景设计说明：
  - `docs/guides/V2_HERO_SCENE_DESIGN.md`
- 这个文档的作用是：
  - 固定 `v2 hero scene` 的几何目标
  - 固定它服务于 `Hard_DeepInterior / Extreme_NarrowPassage` 的展示逻辑
  - 避免下次新对话把它与 `synthetic_guidance_demo` 或 `paper_mainline_demo` 混淆
- 从现在开始：
  - `v2 hero scene` 只指 `v2/WS119` 主线的 RViz 展示场景
  - 不得再用 `paper_mainline_demo` 一类历史命名来指代这个场景
- 如果下次新对话要继续做 `v2` 展示场景、开窗 mesh、答辩截图或 RViz 讲解：
  - 先看 `docs/PAPER_MAINLINE_MAP.md`
  - 再看 `docs/guides/V2_HERO_SCENE_DESIGN.md`

## 0. 2026-03-23 主线冻结结论

- 当前投稿目标固定为：`高质量二区`
- 当前论文定位固定为：
  - `Difficulty-Adaptive Informed Guide Sampling for Constrained Motion Planning in Robotic Contact Measurement`
- 当前方法主线固定为：
  - `HeuristicGuided two-stage guidance interface`
  - `difficulty-adaptive informed guide sampling (B+)`
- 当前工程主攻方向固定为：
  1. 强化 `buildGuideCandidates(...) / generateEllipsoidGuideSamples(...)` 的候选分布
  2. 用场景/任务难度驱动 informed guide sampling，而不是继续纠缠全局 slow-direct rescue
  3. 重点验证：
     - `guide_attempt_rate`
     - `direct_fallback_rate`
     - `budget-hit rate`
     - `P75 / 长尾时间`
- `learned guidance` 当前统一降级为：
  - `executable extension`
  - `ablation / negative evidence line`
  - 不是当前论文主方法
- `BITstar / InformedRRTstar` 等 modern baseline 当前只作为参考对照方向：
  - 不作为当前主线实现依赖
  - 不绑定当前论文成败
- 若本文档旧段落与本节冲突：
  - 以本节和后续更新日期更晚的主线结论为准

### 0.1 2026-03-23 post-fix smoke 结论

- 当前 B+ 主线已经完成一次 `post-fix full-scene smoke` 验证：
  - `simple`: `20260323_110723_103`
  - `v2`: `20260323_110900_192`
- 两次 smoke 给出的结论一致：
  - `hard / extreme` 场景已经进入 `active_guidance`
  - `easy / medium` 场景保持 `direct-only`
- 当前行为结论固定为：
  - `guide-first` 只在困难场景承担主触发角色
  - `direct` 一旦成功，默认不再继续尝试常规 guide 候选
  - 如果后续要处理 `Easy / Medium` 的 `~1s` 长尾，应走 `slow-direct rescue` 增强线，而不是重新放松常规 guide gate
- 因此当前主线已经从：
  - “如何让 adaptive ellipsoid 真正接通”
- 切换为：
  - “如何基于已确认的 hard-scene active guidance 形态，完成 formal rerun / ablation / paper tables”

### 0.2 2026-03-24 post-fix formal rerun 结论（历史记录，已被 0D 覆盖）

- 历史 `HeuristicGuided` 的正式后修复结果曾完成：
  - `simple`: `20260324_081713_711`
  - `v2`: `20260324_081904_937`
- 这两组结果现在仅作为历史记录，不再作为当前论文主表优先引用数字：
  - `simple`: 成功率 `100%`，平均 `425.5 ms`，中位 `240.5 ms`，预算命中 `0/60`，快速求解 `80.0%`
  - `v2`: 成功率 `100%`，平均 `302.0 ms`，中位 `125.5 ms`，预算命中 `0/40`，快速求解 `85.0%`
- 与旧的主表数字相比：
  - `simple`: `845.8 ms -> 425.5 ms`
  - `v2`: `377.1 ms -> 302.0 ms`
- 当前 formal rerun 的行为结论与 smoke 一致：
  - `hard / extreme = active_guidance`
  - `easy / medium = direct-only`
  - 长尾目前主要来自 `direct` 在 easy / medium 上的偶发慢解，而不是 guide 误激活
- 当前论文主表已切换为 `2026-05-03` stable-core unified formal rerun，详见 0D 与 `PAPER_REVIEW_RESOLUTION_LOG.md`。

### 0.3 2026-03-24 正式归档目录

- 当前已经新增正式论文归档目录：
  - `paper_artifacts/q2_formal_20260324/`
- 该目录当前集中保留：
  - 经典 baseline canonical benchmark
  - `HeuristicGuided` post-fix formal rerun
  - 对应 gate activity 摘要
  - 当前图表输入 CSV
  - 当前论文主表可直接使用的 curated table inputs
  - 关键分析文档
- 当前建议把这个目录视为：
  - 论文主表/主图/结果解释的统一入口
- 目录说明文件：
  - `paper_artifacts/q2_formal_20260324/README.md`
- 当前最直接可用的表格输入：
  - `paper_artifacts/q2_formal_20260324/table_inputs/simple_main_table.csv`
  - `paper_artifacts/q2_formal_20260324/table_inputs/v2_main_table.csv`
  - `paper_artifacts/q2_formal_20260324/table_inputs/cross_benchmark_budget_hit.csv`
  - `paper_artifacts/q2_formal_20260324/table_inputs/heuristicguided_scene_gate_profile.csv`
- 重要说明：
  - 本次整理采用“正式归档副本”方式
- 原始文件仍保留在 `test_results/` 和 `docs/` 下
- 这样做是为了避免打断已有脚本、manifest 和历史引用

### 0.4 2026-03-24 文档入口整理

- 当前已经补齐统一文档入口：
  - `docs/README.md`
- 当前文档层次固定为：
  - 根入口：
    - `README.md`
    - `PROJECT_MEMORY.md`
  - 常规工程文档：
    - `docs/guides/`
    - `docs/analysis/`
    - `docs/roadmap/`
  - 原始结果仓：
    - `test_results/`
  - 论文正式归档：
    - `paper_artifacts/`
- 当前使用规则固定为：
  - 查运行命令：看 `docs/COMMANDS.md`
  - 查主线结论：看 `PROJECT_MEMORY.md`
  - 查论文写作：看 `docs/roadmap/`
  - 查论文正式表格输入：看 `paper_artifacts/`
  - 查原始结果和历史数据：看 `test_results/`

### 0.5 2026-03-31 论文数学化补强

- 当前中文论文草稿
  - `docs/roadmap/Q2_PAPER_CHINESE_DRAFT_V1.md`
  - 已进一步补齐方法部分的数学表达
- 本次补强重点固定为：
  - 明确 `difficulty score` 的来源，而不是把 \(d\) 写成未定义黑箱
  - 明确候选集合由 `ellipsoid + anchors + refinement + dedup` 组成
  - 明确 pose / position 记号区分：
    - `s=(s_p,R_s)`
    - `g=(g_p,R_g)`
    - `u=(u_p,R_g)`
  - 明确 `guide/direct cost`、`safety penalty`、`adaptive bonus` 与 `selective activation` 的公式
- 当前论文写作规则进一步固定为：
  - 若正文提到 `difficulty-adaptive`
  - 就必须同时说明 \(d\) 在当前工程中如何由 benchmark / geometry 规则实例化
  - 不再允许把当前实现表述成“在线 learned difficulty estimator 已完成”

### 0.6 2026-03-31 工程清洁化整理

- 当前根目录已新增两个统一入口：
  - `paper_workspace/`
  - `project_archive/`
- `paper_workspace/` 当前集中保存：
  - 论文主线文档副本
  - 正式 paper-facing 结果归档
  - 当前参考 PDF
- 根目录 `paper_artifacts/` 已改为指向 `paper_workspace/formal_results/` 的兼容入口
- `project_archive/` 当前集中保存：
  - learned-guidance 历史分析
  - simple-random 历史数据与导出
  - 旧模型训练产物
  - heuristic rescue 旧导出
  - reproduction 旧结果
  - `review_cleanup/` 临时清理目录
- 当前工作规则固定为：
  - 写论文、找正式结果：优先进 `paper_workspace/`
  - 查当前 benchmark 源结果：看 `test_results/`
  - 查旧实验、旧尝试：看 `project_archive/`

### 0.7 2026-04-09 LaTeX 文稿主工程落地

- 当前已经新增论文 LaTeX 主工程：
  - `paper_workspace/manuscript/latex/`
- 当前主入口固定为：
  - `paper_workspace/manuscript/latex/main.tex`
- 当前主工程已完成：
  - 章节骨架落地
  - 方法与问题定义的数学化英文草稿
  - 主结果表接入
  - 4 张正式 paper-facing 图接入
  - 本地 `latexmk` 编译验证通过
- 当前编译产物输出位置：
  - `paper_workspace/manuscript/latex/build/main.pdf`
- 因此当前论文工作流进一步固定为：
  - 中文推演和长篇笔记继续放在 `paper_workspace/docs/`
  - 正式英文稿以 `paper_workspace/manuscript/latex/` 为准
- 当前文稿还已新增：
  - `sections/related_work.tex`
  - `references.bib`
  - `biblatex + biber` 引文链
- 当前 `Method + Experiments` 已进一步 formalize：
  - 明确 `d=0.55 / 0.70 / 0.80` 三个关键阈值
  - 明确 adaptive ellipsoid、anchor、ranking、guide-first precheck 的公式化表述
  - 明确 benchmark protocol：
    - `simple = 6 scenes x 10 repeats`
    - `v2 = 4 scenes x 10 repeats`
    - `budget = 10 s`
  - 文稿当前可无 warning 编译为：
    - `paper_workspace/manuscript/latex/build/main.pdf`

## 1. 总目标

- 最终目标不是做一个普通机械臂控制工程，而是支撑一篇 `一区 / 二区` 论文。
- 论文方向固定为：`机器人接触测量场景中的受限运动规划与路径规划算法创新`。
- 当前论文定位固定为：`Difficulty-Adaptive Informed Guide Sampling for Constrained Motion Planning in Robotic Contact Measurement`
- 主创新对象固定为：`HeuristicGuided` 外层引导接口上的 `difficulty-adaptive informed guide sampling`
- `learned guidance` 保留为可执行扩展，不再作为当前主论文主线
- 当前不要把论文绑定到某个具体经典 planner 变体，经典算法主要作为 baseline 和对照组。

## 2. 双线管理视图

今后统一从两条主线看项目进度：

1. `论文进行`
2. `工程完成度`

这样做的目的不是形式化管理，而是避免把“代码已经能跑”误判成“论文已经成型”。

## 3. 论文进行

### 3.1 论文主线

当前论文叙事应固定为：

1. 工业接触测量任务不是普通抓取或自由空间运动，而是带有狭窄空间、姿态约束、深孔、侧面和边缘接近等限制的受限运动规划问题。
2. 经典采样规划器在这类任务中存在成功率不稳定、时间长尾明显、预算命中率高的问题。
3. 因此先建立一个可信的接触测量规划基准，而不是直接空谈“新 planner”。
4. 在当前 `HeuristicGuided` 两阶段接口上，引入 `difficulty-adaptive informed guide sampling`，让 guide 候选分布随场景难度而变化。
5. 最终证明：这种难度感知的 informed guide sampling 可以降低复杂测量任务中的预算命中率和长尾风险，并减少当前 `direct-first / dormant guide` 问题。

这里额外固定一个边界，避免后面写偏：

- 主问题：`机械臂接触测量场景下的受限路径规划`
- 主方法：`difficulty-adaptive informed guide sampling`
- 工程承载：`HeuristicGuided two-stage guidance interface`
- `learned guidance`：`扩展线 / 对照线 / 可选增强`
- 两者不是同义词
- 当前论文不能写成“只做采样算法”，也不能写成“只有测量应用没有算法抓手”
- 最稳妥的统一表述是：
  - `用难度感知的 informed guide sampling，解决机械臂接触测量中的受限路径规划问题`

### 3.2 论文主体骨架

当前建议的论文主体结构：

1. `Introduction`
   - 为什么接触测量是一个典型的受限规划问题
   - 为什么经典 planner 在该场景下仍存在效率和稳定性缺口
2. `Problem Definition`
   - 接触测量任务定义
   - 约束形式
   - 评价指标
3. `Benchmark and Dataset`
   - `simple` 与 `v2` 场景
   - 正式 benchmark
   - hard subset 与随机任务数据
4. `Difficulty-Adaptive Informed Guide Sampling`
   - 难度定义
   - 自适应 guide proposal / ellipsoid policy
   - 两阶段规划接口中的候选生成与回退逻辑
5. `Experiments`
   - 与经典 baseline 对比
   - 长尾与预算命中分析
   - 困难任务族分析
   - adaptive vs non-adaptive 消融
   - 可选 learned extension 对照
6. `Conclusion`

### 3.3 论文进度判断

当前论文进度可以分为四层：

- `A. 问题定义与场景合理性`
  - 状态：`已完成基础版`
- `B. 可信 benchmark 与训练数据`
  - 状态：`已完成`
- `C. HeuristicGuided + difficulty-adaptive informed guide sampling`
  - 状态：`已开始，当前是主方法推进方向`
- `D. learned guidance 扩展线`
  - 状态：`已可执行，但尚未形成主结果级正收益`
- `E. 论文级实验、消融、写作`
  - 状态：`已开始，当前已进入正文撰写`

### 3.4 论文已完成部分

已经完成并且足以写进论文前半部分的内容：

- 已明确论文定位：`Difficulty-Adaptive Informed Guide Sampling for Constrained Motion Planning in Robotic Contact Measurement`
- 已建立 `simple` 和 `v2` 两套接触测量 benchmark 场景
- 已完成正式 benchmark 数据采集
- 已完成统一指标体系：
  - `wall_time_ms`
  - `moveit_time_ms`
  - `hit_budget_limit`
  - `fast_solve_lt_1s`
  - 中位数 / P25 / P75
- 已完成 `simple random` 随机任务采集器
- 已形成 smoke / 100-task / 300-task 三批随机数据
- 已完成 300-task 数据分析，已经足以支撑“为什么要做难度感知 guide sampling”这部分动机
- 已完成 `HeuristicGuided` 的 `2026-03-24` post-fix formal rerun，并冻结为当前论文正式引用结果
- 已完成中文论文初稿主干：
  - `docs/roadmap/Q2_PAPER_CHINESE_DRAFT_V1.md`
  - 当前已覆盖：摘要、引言、相关工作、问题定义、benchmark、方法、实验结果、行为分析、讨论、结论

### 3.5 论文未完成部分

当前还不能说论文主体完成，因为下面这些还没落地：

- 组件级正式消融：
  - no adaptive difficulty
  - no anchor
  - no selective activation
- 论文主图和表格的正式出图与统一编号
- 英文稿转写与摘要/引言英文打磨
- 参考文献系统整理与高区相关工作补引
- 可选 learned extension 是否保留为附加结果
- 可选真机统计是否需要补入投稿版正文或附录

### 3.6 当前论文阶段结论

当前最重要的判断：

- 论文前半段“可信 benchmark + 可信训练数据”已经基本完成。
- 论文真正的创新后半段已经不再定义为“先做 learning-guided 闭环”。
- 当前更关键的一跳是：`从稳定但 direct-first 的 HeuristicGuided，走向真正会改变 candidate distribution 的 difficulty-adaptive informed guide sampling`。
- 这一步当前已完成到“可验证形态”：
  - 不再是纯 `direct-first / dormant guide`
  - 而是 `hard-scene active guidance + easy/medium direct preservation`

### 3.7 当前论文写作入口

- `2026-03-17` 已新增论文直接写作骨架：
  - `docs/roadmap/Q2_PAPER_METHOD_RESULTS_SKELETON.md`
- `2026-03-24` 已新增中文论文初稿主文：
  - `docs/roadmap/Q2_PAPER_CHINESE_DRAFT_V1.md`
- 这份文档当前负责：
  - `Method` 章节结构
  - `Results` 章节结构
  - 主表/主图清单
  - 当前可直接写入论文的核心句式
- 中文初稿当前负责：
  - 作为可直接继续精修的论文正文底稿
  - 固定当前 paper-facing 叙事与正式结果数字
- 从现在开始：
  - `Q2_PAPER_ROADMAP.md` 负责路线和决策
  - `Q2_PAPER_METHOD_RESULTS_SKELETON.md` 负责真正的论文中段写作骨架
  - `Q2_PAPER_CHINESE_DRAFT_V1.md` 负责当前完整正文的持续精修
- 后续任何新实验都应优先回答：
  - 它最终落到哪一张表、哪一张图、哪一段结果解释里

### 3.8 当前投稿目标判断：高质量二区 vs 可冲一区

当前最稳妥的项目策略是：

- `投稿目标` 先按 `高质量二区保底` 管理
- `方法设计与实验标准` 按 `冲一区` 的要求推进

原因：

- 当前已经具备：
  - 清晰的问题定义
  - 可信 benchmark
  - canonical 数据
  - `HeuristicGuided` 的工程主线
- 但当前还不具备：
  - 完整消融
  - 更充分的跨几何泛化证明
  - 真机闭环统计结果

当前两档目标的差别应明确写死：

- `高质量二区` 需要的核心条件：
  - benchmark 与场景定义可信
  - 方法主线清楚
  - 与经典 baseline 有稳定对比
  - 至少有一条明确的方法收益主结论
  - 写作、图表和实验组织扎实
- `可认真冲一区` 额外需要的核心条件：
  - 在当前已确认的 `difficulty-selective active guidance` 基础上，补齐更强的组件级消融与未见几何泛化
  - adaptive sampling 在线上形成稳定正收益
  - 在 `simple + v2 + 未见几何` 上都能证明收益
  - 有完整消融：
    - no guidance
    - heuristic guidance
    - adaptive informed guide sampling
    - 可选 learned extension
    - 难度特征 / 几何特征 / 组合特征
  - 最好再补：
    - 真机验证
    - 方法可解释性分析

当前判断结论：

- 按今天的状态看：
  - `高质量二区` 是现实且稳妥的目标
  - `一区` 还不能直接当成当前完成度结论
- 但如果下面这条主线闭环成立：
  - `difficulty-adaptive informed guide sampling -> true guide activation -> cross-geometry gains`
  - 那就具备从“强二区稿”上调到“可冲一区稿”的资格

因此后续不要把讨论方式写成：

- “我们到底投一区还是二区”

而应该写成：

- “当前按高质量二区保底推进，同时以一区标准检查方法闭环是否成立”

### 3.9 决定能否上调到一区的硬门槛

后续只有满足下面这些条件，才应正式把目标从“高质量二区”上调成“冲一区”：

1. `difficulty-adaptive informed guide sampling` 在当前 `HeuristicGuided` 主线中持续稳定产生正收益
2. 至少一项核心指标稳定优于当前默认 `direct-first` 版本
3. 收益不只在单一场景成立，而是至少跨：
   - `simple`
   - `v2`
   - 一组未见几何
4. 消融结果能说明：
   - 提升来自难度感知的 adaptive sampling，而不是偶然参数波动
5. 论文主结论能从：
   - “我们工程做得很完整”
   - 升级为
   - “我们提出的 difficulty-adaptive guidance framework 在受限测量场景中稳定降低规划长尾风险”

如果这 5 条里有 2 到 3 条做不到，就不要勉强按一区定位写作，避免论文整体姿态过高但证据不够。

## 4. 当前工程完成度

这里的“完成度”不是指代码量，而是指距离“支撑论文闭环”还有多远。

### 4.1 工程完成度快照

- `基础机器人与规划实验平台`：`90%`
- `simple / v2 正式 benchmark`：`95%`
- `random simple 数据采集`：`95%`
- `数据清洗、分析、绘图、归档`：`95%`
- `统一训练数据导出`：`90%`
- `第一版训练特征工程`：`55%`
- `difficulty-adaptive informed guide sampling`：`45%`
- `learned guidance 在线扩展 / 消融`：`60%`
- `论文级消融与泛化实验工具链`：`25%`

### 4.2 当前工程整体判断

- 如果只看“benchmark / dataset 工程”，当前完成度已经很高。
- 如果看“论文最终系统工程”，当前整体完成度更接近 `60% ~ 65%`。
- 原因不是基础差，而是最关键的“自适应 guide sampling 闭环”还没有形成正式证据。

### 4.3 已完成工程模块

已经落地的模块：

- `CR5Robot` 核心规划与指标记录
- 基础规划节点
- 箱体测试节点
- Qt GUI
- `planner_comparison_simple_node`
- `planner_comparison_v2_node`
- `random_task_dataset_simple_node`
- benchmark 统一导出脚本
- simple / v2 benchmark 绘图脚本
- simple random dataset 分析与绘图脚本
- simple random 训练表准备脚本
- task-level oracle planner 数据集生成脚本
- baseline 模型训练脚本
- baseline 模型评估脚本
- 离线 planner 选择实验脚本
- direct oracle planner 训练与评估脚本
- guide candidate 数据采集节点
- guide ranking 线性模型训练脚本
- learned guidance 对照消融节点
- dataset manifest
- 数据归档规则与文档体系

### 4.4 未完成工程模块

还需要补的核心模块：

- 更稳定的场景/任务难度定义与校准
- 更完整的 adaptive informed guide sampling 策略：
  - 椭球尺度
  - target bias
  - 采样密度
  - local refinement 配额
- `baseline vs adaptive` 的正式对比实验
- `simple / v2 / hard subset` 的长尾消融
- 可选 learned guidance 扩展是否值得保留的最终判断

这里的“planner 引导接口”当前已明确限定为：

- 不是端到端轨迹生成接口
- 不是先去改 OMPL 底层 sampler 插件
- 而是优先落到 `HeuristicGuided` 的外部采样引导接口：
  - `start_state / target_pose / budget`
  - `guide pose` 候选生成
  - 候选排序或筛选
  - 两段规划连接

## 5. 当前算法结论（历史段落，已被 0D 覆盖）

以下内容基于 `2026-03-24` 的 post-fix formal rerun 与既有 canonical baseline，已经被 `2026-05-03` stable-core unified formal rerun 覆盖：

- `HeuristicGuided` 仍然是当前主方法，也是唯一在两类 benchmark 上都保持极低预算命中率的方法
  - `simple`: 成功率 `100%`，平均 `425.5 ms`，预算命中 `0/60`
  - `v2`: 成功率 `100%`，平均 `302.0 ms`，预算命中 `0/40`
- `RRTstar` 继续只保留为历史对照，不作为主改进对象
- 经典 baseline 已不能再概括成“单一最佳”
  - `simple` 上更值得保留的是 `LBTRRT / FMT / PRMstar`
  - 新 `v2` 上更值得保留的是 `BFMT / RRTConnect / PRMstar`
- 因此论文主表中的经典对照组应固定保留：
  - `BFMT`
  - `LBTRRT`
  - `FMT`
  - `PRMstar`
  - `RRTConnect`
- 如果只看当前 `simple_random` 的 classical-only 学习实验，`FMT` 仍然是需要重点对照的固定经典基线

当前推荐主表角色优先级：

1. `HeuristicGuided`
2. `BFMT`
3. `LBTRRT`
4. `FMT`
5. `PRMstar`
6. `RRTConnect`
7. `RRTstar`

## 6. Stage 2 数据采集现状

### 6.1 第一版范围

第一版先做：

- `simple` 场景随机任务采集
- 输出统一训练 CSV
- 标签优先包括：
  - `success`
  - `wall_time_ms`
  - `moveit_time_ms`
  - `hit_budget_limit`
  - `fast_solve_lt_1s`

第一版暂不做：

- 深度 OMPL sampler 集成
- 真正论文最终版的 STL 几何特征提取
- 端到端轨迹学习

### 6.2 当前结果

`simple` 随机任务第一轮正式采集已经完成：

- 文件：`test_results/datasets/simple_random/raw/20260311_140426_647_simple_random_task_dataset_results.csv`
- 摘要：`test_results/datasets/simple_random/raw/20260311_140426_647_simple_random_task_dataset_summary.csv`
- 分析：`docs/analysis/SIMPLE_RANDOM_DATASET_20260311_140426_ANALYSIS.md`

关键记忆点：

- 总样本：`400`
- 成功：`380`
- 失败：`20`
- 最主要难例任务族：`hole_deep`
- `HeuristicGuided` 预算命中最少
- `FMT / LBTRRT` 仍然是最值得保留的经典对照

`simple` 随机任务第二轮 300-task 正式采集也已完成：

- 文件：`test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_results.csv`
- 摘要：`test_results/datasets/simple_random/raw/20260311_142950_774_simple_random_task_dataset_summary.csv`
- 分析：`docs/analysis/SIMPLE_RANDOM_DATASET_20260311_142950_774_ANALYSIS.md`

关键记忆点：

- 总样本：`1200`
- 成功：`1143`
- 失败：`57`
- 总体成功率：`95.2%`
- 最主要难例任务族仍然是：`hole_deep`
- 任务族成功率：
  - `hole_deep`: `70.7%`
  - 其余任务族基本全成功，只有 `front_side` 出现 `2` 个失败
- 各规划器表现：
  - `FMT`: 成功率 `95.0%`，中位数 `98.5 ms`，预算命中 `124/300`
  - `LBTRRT`: 成功率 `95.0%`，中位数 `82.0 ms`，预算命中 `112/300`
  - `RRTConnect`: 成功率 `95.3%`，中位数 `88.0 ms`，预算命中 `116/300`
  - `HeuristicGuided`: 成功率 `95.7%`，中位数 `82.0 ms`，预算命中 `13/300`

当前最稳固结论：

- `HeuristicGuided` 的核心优势是显著降低预算命中率
- `FMT / LBTRRT / RRTConnect` 在成功率上接近，但长尾明显更重
- 下一步应优先学习 `hit_budget_limit` 和 `fast_solve_lt_1s`，而不是只学 `success`

### 6.3 当前是否继续扩数

当前判断：

- `300-task simple` 已足够支撑第一版轻量模型训练
- 现在不必立刻扩到 `500-task`
- 只有在下面两种情况下再考虑继续扩数：
  - 第一版模型对 `hole_deep` 的泛化明显不稳定
  - 需要为 `v2` 迁移前再补更多 hard/extreme 标签

## 7. 当前最明确下一步

下一步固定为：

1. 固定 `HeuristicGuided` 为论文主方法承载接口，不再把 planner 选择 / learned ranking 当成当前主攻线
2. 继续完善 `difficulty-adaptive informed guide sampling`：
   - 难度评分
   - 椭球参数
   - target bias
   - guide sample budget
3. 在 `simple` 和 `v2` 上做 `HeuristicGuided-only` 的 baseline vs adaptive 正式对比
4. 用新增诊断列验证 adaptive sampling 是否真的改变：
   - `guide_attempt_rate`
   - `direct_fallback_rate`
   - `budget-hit rate`
   - `P75 / 长尾时间`
5. 只有当 adaptive sampling 形成明确正收益后，才决定是否继续接入轻量 learned re-ranking 作为扩展线

历史补充状态（保留归档，不构成当前主线执行顺序）：

- `2026-03-17` 已补第一版 baseline 训练脚本框架：
  - `scripts/models/prepare_training_table.py`
  - `scripts/models/train_baseline_model.py`
  - `scripts/models/evaluate_baseline_model.py`
- 已完成第一轮 baseline run：
  - 训练表：`test_results/exports/simple_random_training_table.csv`
  - 模型 run：`test_results/models/simple_random_baseline/20260317_100446/`
  - `hit_budget_limit`：`AUC 0.725`，`F1 0.566`，`balanced_accuracy 0.684`
  - `fast_solve_lt_1s`：`AUC 0.640`，`F1 0.640`，`balanced_accuracy 0.594`
- 已完成第一轮离线 planner 选择实验：
  - run：`test_results/models/simple_random_baseline/20260317_113000_selection_refresh/`
  - 全 planner 集合下，预测策略退化为始终选择 `HeuristicGuided`，与当前 benchmark 结论一致
  - 经典 planner 子集 `FMT / LBTRRT / RRTConnect` 下，预测策略退化为始终选择 `RRTConnect`
  - 该经典子集实验中：
    - 最佳固定基线：`FMT`
    - `FMT` 预算命中率：`31.7%`
    - 当前预测选择策略预算命中率：`43.3%`
  - 说明当前第一版特征足以识别“`HeuristicGuided` 整体更稳”，但还不足以支持经典 planner 内部的有效逐任务选择
- 已完成第二轮增强特征 + classical-only 训练实验：
  - run：`test_results/models/simple_random_baseline/20260317_114500_classical_interactions_v2/`
  - 新增内容：
    - 几何派生特征
    - `planner x task` 交互特征
    - classical-only 训练
  - 经典 planner 子集实验结果：
    - 最佳固定基线 `FMT`：预算命中率 `31.7%`
    - 当前增强特征预测策略：预算命中率 `46.7%`
  - 说明：
    - 当前问题已经不只是“特征不够”
    - 还包括“用逐 planner 风险分数直接做三选一”这一目标形式本身不合适
- 已完成第一轮 direct oracle planner 实验：
  - oracle 数据集：`test_results/exports/simple_random_oracle_planner_dataset_FMT_LBTRRT_RRTConnect.csv`
  - model run：`test_results/models/simple_random_oracle_planner/20260317_120000_direct_oracle/`
  - multiclass 指标：
    - accuracy：`31.7%`
    - macro F1：`0.317`
    - balanced accuracy：`0.338`
  - 选择结果：
    - 预测 oracle planner：预算命中率 `48.3%`
    - 最佳固定基线 `FMT`：预算命中率 `31.7%`
  - 说明：
    - 将目标直接改成 `oracle planner` 之后，当前线性模型仍未超过固定 `FMT`
    - 因此下一步问题已经进一步收敛为：`需要比当前线性基线更适合的模型形式或分层决策结构`
- 当前结论：
  - 第一版几何 + planner 条件特征已经能学到“预算风险”信号
  - `hit_budget_limit` 比 `fast_solve_lt_1s` 更适合作为第一优先目标
  - 下一步不该继续整理目录，而应转向更直接的 planner 选择目标形式
  - “离线 planner 选择实验”这一步已经做完，但结果表明：当前模型还不具备支撑经典 planner 内部排序的能力
  - “direct oracle planner” 这一步也已经做完，但当前 softmax 线性模型同样没有超过固定 `FMT`

`2026-03-17` 晚间更新：

- 已完成第一版 `guide candidate ranking -> linear model -> online guidance ablation` 最小闭环
- 新增工程入口：
  - `guide_ranking_simple_experiment_node`
  - `scripts/models/train_guide_ranking_model.py`
- 当前最重要事实：
  - learning 模型已经可以真实接回 `HeuristicGuided`
  - 但首版 `candidate_viable` 线性 ranker 是负结果
- 因此当前后面最该做的两件事变为：
  1. 扩充 `guide candidate` 数据规模并重新筛目标/特征，让 learned ranker 至少不劣于当前 heuristic ranking
  2. 在得到稳定 ranker 后，立刻重跑 `simple -> v2 -> 未见几何` 的在线消融，判断是否具备从高质量二区上调到冲一区的资格

## 8. 数据质量修复记忆点

- `2026-03-11` 已修复 `CR5Robot` 中失败规划路径对 `MoveIt规划时间(ms)` 的脏值写入问题
- 根因：失败时直接读取 `MoveGroupInterface::Plan::planning_time_`，该值在失败路径上不可信
- 处理方式：
  - 源码中对 reported planning time 做了 sane-check，异常时回退到该次真实 wall time
- 已用脚本修复历史 `simple random` 数据：
  - `20260311_140426_647_simple_random_task_dataset_results.csv`
  - `20260311_142950_774_simple_random_task_dataset_results.csv`
- 当前这两批随机数据现在可直接用于统计和后续图表

## 9. 数据归档入口

- 高价值数据归档规则已写入：`docs/guides/DATA_STORAGE_GUIDE.md`
- 当前机器可读索引文件：`test_results/dataset_manifest.csv`
- 当前 manifest 已登记：
  - `simple` 正式 benchmark 最新版
  - `v2` 正式 benchmark 最新版
  - `benchmark_training_dataset.csv`
  - `simple random` smoke / 100-task / 300-task
- 今后新的正式数据必须同步更新 manifest，避免结果散落后难以追踪

## 10. 命令文档维护

- 当前用户终端命令统一入口：`docs/COMMANDS.md`
- 当前目录结构说明文档：`docs/PROJECT_LAYOUT.md`
- 以后每新增一个用户可直接执行的终端命令，必须在同一次提交里同步更新 `docs/COMMANDS.md`
- 这里的“可执行命令”包括：
  - `ros2 run ...`
  - `ros2 launch ...`
  - `python3 scripts/<group>/...`
  - 新增环境变量驱动的标准工作流

## 11. 协作约定

- 今后讨论“下一步做什么”时，默认至少同时给出后面最该做的 `两件事`
- 这两件事应按优先级排序，并给出一句话说明为什么先做它们
- 目的不是一次铺很长计划，而是让论文推进和工程推进始终保持可评估、可取舍

## 12. 箱体特征提取现状

- `2026-03-17` 已将 `MeasurementPointGenerator` 从“包围盒中心假设孔/腔体”升级为“基于 STL 网格高度图的开口/腔体提取”
- 当前实现位于：
  - `include/my_cr5_control/measurement_point_generator.hpp`
  - `src/core/measurement_point_generator.cpp`
- 当前提取逻辑：
  - 先从 `WS119.STL` 加载统一世界坐标网格
  - 计算包围盒作为外包络
  - 通过顶向高度图 + 连通域下陷检测提取真实开口/内腔候选
  - 同时补齐 `surfaces / edges / corners` 外包络基础特征
- 当前新增检查命令：
  - `ros2 run my_cr5_control inspect_box_features_node`
  - `ros2 run my_cr5_control visualize_box_features_node`
  - 已同步登记到 `docs/COMMANDS.md`
- 当前 RViz 核对说明文档：
  - `docs/guides/BOX_FEATURE_VISUALIZATION.md`
- 当前一次实测输出：
  - `holes=1`
  - `cavities=2`
  - `surfaces=6`
  - `edges=12`
  - `corners=8`
  - `points=13`
- 当前已补上的稳定化规则：
  - 对近邻且深度相近的 cavity 候选做 merge
  - 过滤过浅、过小、体积代理分数过低的碎片 cavity
  - 过滤“过宽但不够深”的整体顶面凹区，避免把大范围包络下陷误当作局部测量腔体
  - 当前 `WS119` 的 cavity 检测已从 `11` 个收敛到 `2` 个
- 已修复 `generateTestScenarios()` 的点型优先级问题：
  - `Extreme_NarrowPassage` 现在会优先选 `NARROW_PASSAGE`
  - 不会再被更早出现的 `INTERIOR_DEEP` 点错误抢占
- 这说明工程上已经具备“从真实 STL 自动提取测量相关几何特征”的基础能力，不再只是手工构造中心孔假设
- 但这还不是最终论文级几何语义识别，后续仍需要：
  - 做 RViz 可视化核对提取结果是否与真实结构一致
  - 继续判断当前保留下来的 `3` 个 cavity 是否都具有真实测量意义

## 13. 最新正式 v2 benchmark

- `2026-03-17` 已完成当前 `2-cavity` STL 特征提取版本的正式 `v2` benchmark
- 当前正式结果文件：
  - `test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv`
  - `test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_summary.csv`
- 当前正式分析文档：
  - `docs/analysis/V2_BENCHMARK_20260317_142203_372_ANALYSIS.md`
- 这版结果当前应视为：
  - `v2 classical-baseline canonical`
- 注意：
  - 更早的 `20260317_140546_426` 单轮 run 发生在 `Extreme_NarrowPassage` 点型优先级 bug 修复前
  - `20260317_140921_704` 发生在“3-cavity 版本”提取器上
  - `20260317_141715_861` 是 `2-cavity` 提取器的单轮 sanity run
  - 当前经典 baseline 讨论仍可继续以 `20260317_142203_372` 为准
  - 但 `HeuristicGuided` 的论文最终引用值已更新到 `20260324_081904_937`
- 这次正式 run 已确认：
  - `Extreme_NarrowPassage` 使用的是 `真实内腔狭窄通道点（极端困难）`
  - 新的 STL 特征提取链路已经稳定支撑 `10` 次重复正式 benchmark
- 当前正式结果摘要：
  - `HeuristicGuided` 历史 canonical: 成功率 `100%`，平均 `377.1 ms`，中位 `75.0 ms`，预算命中 `0/40`
  - `HeuristicGuided` post-fix formal rerun (`20260324_081904_937`): 成功率 `100%`，平均 `302.0 ms`，中位 `125.5 ms`，预算命中 `0/40`
  - `BFMT`: 成功率 `100%`，平均 `2552.2 ms`，中位 `66.0 ms`，预算命中 `10/40`
  - `RRTConnect`: 成功率 `100%`，平均 `2803.2 ms`，中位 `71.5 ms`，预算命中 `11/40`
  - `PRMstar`: 成功率 `100%`，平均 `3050.9 ms`，中位 `68.5 ms`，预算命中 `12/40`
  - `LBTRRT`: 成功率 `100%`，平均 `4296.2 ms`，中位 `79.0 ms`，预算命中 `17/40`
  - `FMT`: 成功率 `100%`，平均 `4541.4 ms`，中位 `81.5 ms`，预算命中 `18/40`
  - `RRTstar`: 成功率 `100%`，平均 `10014.2 ms`，预算命中 `40/40`
- 当前关键判断：
  - 当前 `2-cavity` 版 `v2` 比旧版更能暴露经典 planner 的长尾风险
  - `HeuristicGuided` 虽然均值上升，但仍然保持 `0%` 预算命中率
  - 在经典 baseline 里，`BFMT` 是当前最值得保留的新 `v2` 参考点
  - `FMT / LBTRRT` 在新 `v2` 上的长尾显著加重，这对论文主结论是加分项而不是噪声

## 14. Canonical Benchmark Export 与模型侧现状

- `2026-03-17` 已重新刷新统一 benchmark 导出：
  - `test_results/exports/benchmark_training_dataset.csv`
- 当前 canonical 输入源固定为：
  - `simple`: `test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_results.csv`
  - `v2`: `test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_results.csv`
- 注意：
  - 这份统一导出当前仍主要对应历史全量 benchmark
  - 论文主表里 `HeuristicGuided` 一行不应再直接引用这里的旧值
  - 当前应改用：
    - `simple`: `test_results/benchmarks/simple/raw/20260324_081713_711_planner_comparison_simple_results.csv`
    - `v2`: `test_results/benchmarks/v2/raw/20260324_081904_937_planner_comparison_v2_results.csv`
- 当前导出表规模：
  - 总行数 `700`
  - `simple` `420`
  - `v2` `280`
  - 7 个 planner 各 `100` 条
  - 总成功 `689/700`
- 当前 planner 级 budget 命中率摘要：
  - `HeuristicGuided`: `1.0%`
  - `BFMT`: `32.0%`
  - `LBTRRT`: `31.0%`
  - `PRMstar`: `31.0%`
  - `FMT`: `38.0%`
  - `RRTConnect`: `40.0%`
  - `RRTstar`: `100.0%`
- 这张导出表当前应视为：
  - 论文主表/图表输入
  - benchmark 级统一分析输入
  - 不是当前学习脚本的默认训练输入
- 当前 `scripts/models/` 下的训练和评估链路仍然主要围绕：
  - `simple_random_training_table.csv`
  - `simple_random` 随机任务标签
- 因此当前事实应明确写死：
  - benchmark 导出链路已经刷新完成
  - 模型链路尚未直接切换到 `benchmark_training_dataset.csv`
  - 若后续要做 benchmark 级模型实验，应单独准备对应的数据准备与评估脚本

## 15. HeuristicGuided 采样引导接口约束

- `HeuristicGuided` 当前在 benchmark 与随机任务代码里的工程映射是：
  - `planning_mode = heuristic_guided`
  - 非 OMPL planner id 插件路径
  - 直接调用 `CR5Robot::planToPoseImproved(...)`
- 当前这条线的意义要固定：
  - 它是“外部两阶段采样引导原型”
  - 不是“最终论文版自定义 OMPL sampler”
  - 也不是“端到端轨迹网络”
- 当前实现主链路：
  - 输入：起点状态、目标法兰位姿、规划预算
  - 中间：生成椭球/目标偏置 `guide poses`
  - 中间：基于启发式代价排序候选
  - 输出：`start -> guide -> goal` 两段规划，或 direct fallback
- 这条接口是后续 learning 模块最优先替换的位置：
  - 可以替换 `guide pose` 生成分布
  - 可以替换候选排序函数
  - 可以加入几何/运动学特征打分
  - 但短期内不要把目标改成“网络直接输出整条轨迹”
- 当前接口说明文档：
  - `docs/guides/HEURISTIC_GUIDED_SAMPLING_INTERFACE.md`
- `2026-03-17` 已完成第一步工程抽象：
  - `CR5Robot` 已新增可替换 guide-ranking 接口
  - 入口：
    - `setGuideRankingFunction(...)`
    - `clearGuideRankingFunction()`
  - 默认行为当前已从“纯 `heuristic_cost` 排序”升级为：
    - `heuristic_cost + safety/manipulability-aware ranking`
  - 这意味着后续第一版学习模型现在可以优先接到：
    - `guide candidate ranking`
  - 而不需要立刻改动：
    - 底层 MoveIt / OMPL
    - benchmark 调用链
- `2026-03-17` 已补第一轮低风险算法增强，用来吸收“先进算法模块”而不重写底层规划器：
  - `guide environment box hint`
    - 让 simple 场景下的外部组合箱体也能参与 guide clearance 估计
  - `IK / manipulability-aware filtering`
    - guide 候选会先做 IK 可达性和 Jacobian 最小奇异值评估
  - `local guide densification`
    - 会围绕最有希望的 top seed 再做一轮局部增密采样
  - 当前意义：
    - 这更接近 `constraint-aware sampling + safety filter + local refinement`
    - 是比“直接上大模型”更稳的一步

## 16. Learned Guidance 首轮闭环结果

- `2026-03-17` 已完成第一版在线 learned guide ranking 闭环：
  - candidate 数据：
    - `test_results/datasets/guide_ranking_simple/raw/20260317_160531_045_guide_ranking_simple_dataset_results.csv`
  - 模型：
    - `test_results/models/guide_ranking_simple/20260317_160902_candidate_viable/linear_model.csv`
  - 消融：
    - `test_results/benchmarks/simple_guidance/raw/20260317_160940_152_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260317_160940_152_learned_guidance_simple_ablation_summary.csv`
- 数据与模型快照：
  - candidate 数据总行数：`144`
  - `guided_success`：`111/144`
  - `candidate_viable`：`111/144`
  - `candidate_fast`：`51/144`
  - `candidate_preferred`：`68/144`
  - 当前选用目标：`candidate_viable`
  - 测试集指标：
    - accuracy：`0.417`
    - F1：`0.571`
    - ROC AUC：`0.171`
- 当前首轮 simple 在线消融结论：
  - `no_guidance`：
    - 成功率 `100%`
    - 平均 `1727.0 ms`
    - 中位 `168.0 ms`
    - 预算命中 `5/12`
  - `heuristic_guided`：
    - 成功率 `100%`
    - 平均 `567.8 ms`
    - 中位 `709.5 ms`
    - 预算命中 `0/12`
  - `learned_guided`：
    - 成功率 `91.7%`
    - 平均 `4057.9 ms`
    - 中位 `4027.5 ms`
    - 预算命中 `12/12`
- 当前工程意义：
  - 好消息不是“模型已经有效”
  - 而是“模型在线接回 HeuristicGuided 的工程链路已经打通”
  - 当前负结果说明：
    - 小样本 candidate 数据还不足以支持稳定泛化
    - 当前 `candidate_viable` 目标和线性 ranker 组合不适合作为论文主结果
- 当前论文表述边界：
  - 可以说：
    - `we established the first online learned-guide-ranking closure`
  - 不能说：
    - `learned guidance already outperforms heuristic guidance`
- 当前对应分析文档：
  - `docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_160940_152_ANALYSIS.md`

## 17. HeuristicGuided 启发式增强现状

- `2026-03-17` 已对默认 `HeuristicGuided` 候选生成链路加入三类增强：
  - `clearance-aware` 候选安全评分
  - `manipulability-aware` IK/奇异性过滤
  - `top-seed local refinement` 局部增密重采样
- 代码位置：
  - `include/my_cr5_control/cr5_robot.hpp`
  - `src/core/cr5_robot.cpp`
  - `src/benchmarks/planner_comparison_simple.cpp`
  - `src/benchmarks/random_task_dataset_simple.cpp`
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`
- 当前工程判断：
  - 这次不是 learned model 改进
  - 而是把先进 constrained-planning 思路先模块化吸收进当前 heuristic baseline
  - 目的是先抬高当前 `HeuristicGuided` 的基础强度，再为下一轮 learned ranking 提供更干净的候选分布
- `2026-03-17` 首次重跑时暴露了一个真实运行时约束：
  - 当前 MoveIt 环境没有为 `cr5_group` 实例化 kinematics plugin
  - 日志会出现：`No kinematics solver instantiated for group 'cr5_group'`
  - 因此 `manipulability-aware` 逻辑不能硬依赖在线 IK 求解器
- 已做稳定化修复：
  - 若存在 solver，则使用真实 IK + Jacobian 最小奇异值估计
  - 若不存在 solver，则回退到 `seed Jacobian + 粗几何可达性` 估计
  - 缺少 solver 时不再把候选整体误判为不可用或导致 `HeuristicGuided` 退化
- 当前验证状态：
  - `colcon build --packages-select my_cr5_control` 已通过
  - 已完成 `simple` quick rerun：
    - `test_results/benchmarks/simple/raw/20260317_170614_014_planner_comparison_simple_results.csv`
    - `test_results/benchmarks/simple/raw/20260317_170614_014_planner_comparison_simple_summary.csv`
  - quick rerun 关键摘要：
    - `HeuristicGuided`：成功率 `100.0%`，平均 `678.4 ms`，中位 `73.5 ms`，预算命中 `0/12`
    - `RRTConnect`：成功率 `100.0%`，平均 `3679.0 ms`，预算命中 `4/12`
    - `LBTRRT`：成功率 `100.0%`，平均 `2117.3 ms`，预算命中 `2/12`
    - `FMT`：成功率 `100.0%`，平均 `3398.1 ms`，预算命中 `4/12`
- 当前判断：
  - `no-IK fallback` 是当前环境下必须保留的稳定化措施
  - 修复后启发式增强版 `HeuristicGuided` 已恢复到可继续作为 learned guidance 对照基线的状态

## 18. Learned Guidance 第二轮 refined 重跑结果

- `2026-03-17` 已完成第二轮 guide-ranking 数据重采样、重训练与 online ablation：
  - 新 candidate 数据：
    - `test_results/datasets/guide_ranking_simple/raw/20260317_171238_252_guide_ranking_simple_dataset_results.csv`
  - 新模型目录：
    - `test_results/models/guide_ranking_simple/20260317_171600_candidate_viable_refined`
    - `test_results/models/guide_ranking_simple/20260317_171600_candidate_fast_refined`
    - `test_results/models/guide_ranking_simple/20260317_171600_candidate_preferred_refined`
  - 新 online ablation：
    - `test_results/benchmarks/simple_guidance/raw/20260317_171817_260_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260317_171817_260_learned_guidance_simple_ablation_summary.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260317_171943_687_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260317_171943_687_learned_guidance_simple_ablation_summary.csv`
  - 当前 refined 分析文档：
    - `docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md`
- 新 candidate 数据快照：
  - 总行数：`305`
  - `guided_success`：`263/305`
  - `candidate_viable`：`263/305`
  - `candidate_fast`：`135/305`
  - `candidate_preferred`：`137/305`
  - `ik_feasible`：`305/305`
  - 平均 `clearance_margin`：`0.122982`
  - 平均 `manipulability_score`：`0.141921`
- 第二轮离线模型测试集指标：
  - `candidate_viable_refined`：
    - accuracy `0.690`
    - F1 `0.788`
    - ROC AUC `0.709`
  - `candidate_fast_refined`：
    - accuracy `0.493`
    - F1 `0.550`
    - ROC AUC `0.541`
  - `candidate_preferred_refined`：
    - accuracy `0.648`
    - F1 `0.627`
    - ROC AUC `0.738`
- 第二轮 simple 在线消融结论：
  - `candidate_viable_refined`：
    - `no_guidance`：成功率 `100.0%`，平均 `1499.1 ms`，预算命中 `3/12`
    - `heuristic_guided`：成功率 `91.7%`，平均 `715.8 ms`，预算命中 `1/12`
    - `learned_guided`：成功率 `91.7%`，平均 `4020.9 ms`，预算命中 `12/12`
  - `candidate_preferred_refined`：
    - `no_guidance`：成功率 `100.0%`，平均 `1801.8 ms`，预算命中 `5/12`
    - `heuristic_guided`：成功率 `91.7%`，平均 `720.2 ms`，预算命中 `1/12`
    - `learned_guided`：成功率 `91.7%`，平均 `4025.3 ms`，预算命中 `12/12`
  - 两个 refined model 的 online 负结果具有一致性：
    - `learned_guided` 在 `easy / medium / hard / extreme` 四个难度层都命中预算上限
    - `extreme` 难度仍然掉到 `1/2` 成功
    - 这不是局部失败，而是当前 online 排序目标整体不对
- 当前明确结论：
  - 第二轮 refined 数据和离线指标比首轮明显更好
  - 但 online learned guidance 仍然系统性塌到 `4 s` 预算上限
  - 当前 bottleneck 已不是“接口没打通”或“特征取不到”
  - 而是：
    - `线性分类 logit 直接排序` 这个 online ranking 策略不成立
    - 需要改成更像排序问题而不是继续机械重训同一套线性分类器

## 19. Learned Guidance 第三轮：online ranking policy 修复

- `2026-03-18` 已把 online learned guidance 从：
  - `线性分类 logit 对全量 candidate 直接排序`
- 改为：
  - `top-k gating + heuristic fallback`
- 当前实现位置：
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`
- 当前默认策略：
  - 只保留 `top_k=1` 个模型高置信 candidate 进入在线尝试队列
  - 默认置信阈值：`0.55`
  - 若模型没有高置信候选，则回退到 `heuristic best`
- 当前新增环境变量：
  - `MY_CR5_CONTROL_GUIDE_MODEL_TOP_K`
  - `MY_CR5_CONTROL_GUIDE_MODEL_SCORE_THRESHOLD`
- 这次修复的核心目的不是立刻追求更快，而是先解决：
  - learned guidance 把几乎所有候选都拖进规划尝试，导致稳定撞满预算

- `candidate_viable_refined` 新 online ablation：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_085048_146_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_085048_146_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance`：成功率 `100.0%`，平均 `877.2 ms`，预算命中 `2/12`
    - `heuristic_guided`：成功率 `83.3%`，平均 `964.1 ms`，预算命中 `2/12`
    - `learned_guided`：成功率 `100.0%`，平均 `1226.1 ms`，预算命中 `0/12`
  - 关键变化：
    - learned guidance 已不再出现 `12/12` 预算命中
    - 当前更像“稳定但偏慢”，而不是“直接失效”

- `candidate_preferred_refined` 新 online ablation：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_085205_318_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_085205_318_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance`：成功率 `91.7%`，平均 `1481.5 ms`，预算命中 `4/12`
    - `heuristic_guided`：成功率 `91.7%`，平均 `755.4 ms`，预算命中 `1/12`
    - `learned_guided`：成功率 `91.7%`，平均 `1501.2 ms`，预算命中 `0/12`
  - 关键变化：
    - 即使是较弱的 `candidate_preferred_refined`，新策略也把预算塌陷从 `12/12` 降到了 `0/12`

- 当前第三轮结论要写死：
  - `ranking policy fix` 已经生效
  - 当前最主要的正向变化是：
    - `online learned guidance no longer collapses to budget hits`
  - 但当前还不能说 learned guidance 已全面优于 heuristic guidance
  - 因为当前代价是：
    - 中位时间仍偏高
    - `fast solve` 比例明显偏低
- 当前最合理的判断：
  - 第二轮的问题确实主要出在 `full-candidate logit sort`
  - 现在接口和策略都已进入“可继续优化”的状态
  - 下一步优化目标应从“先修塌陷”切换到：
    - `在保持低 budget-hit 的同时，把 mean / median time 拉回 heuristic 量级`

## 20. retained-set 二次排序首次在线尝试

- `2026-03-18` 已额外测试 retained set 内部二次排序实验开关：
  - 新增环境变量：
    - `MY_CR5_CONTROL_GUIDE_RETAINED_ORDER`
  - 当前支持：
    - `heuristic`
    - `learned`
    - `hybrid`
- 首次在线尝试配置：
  - `top_k=2`
  - `retained_order=heuristic`
  - 模型：
    - `candidate_viable_refined`
- 对应结果：
  - `test_results/benchmarks/simple_guidance/raw/20260318_090156_876_learned_guidance_simple_ablation_results.csv`
  - `test_results/benchmarks/simple_guidance/raw/20260318_090156_876_learned_guidance_simple_ablation_summary.csv`
- 这次摘要：
  - `no_guidance`：成功率 `91.7%`，平均 `2112.3 ms`，预算命中 `5/12`
  - `heuristic_guided`：成功率 `100.0%`，平均 `375.4 ms`，预算命中 `0/12`
  - `learned_guided`：成功率 `100.0%`，平均 `2502.5 ms`，中位 `3081.5 ms`，预算命中 `1/12`
- 当前明确结论：
  - `top_k=2 + retained heuristic re-rank` 这条在线策略当前是负结果
  - 它没有把时间拉回 heuristic 量级，反而显著变慢
  - 并且把之前 `0/12` 的 budget-hit 又带回了 `1/12`
- 因此当前工程决策应固定为：
  - 保留 `MY_CR5_CONTROL_GUIDE_RETAINED_ORDER` 作为实验开关
  - 但默认策略仍维持：
    - `top_k=1`
    - `threshold=0.55`
    - `top-k gating + heuristic fallback`
  - 不把 `k2 heuristic re-rank` 升级成默认线上策略

## 21. 单候选 selector 策略抽象

- `2026-03-18` 已把 learned 单候选选择规则抽成独立实验开关：
  - `MY_CR5_CONTROL_GUIDE_SELECTION_MODE`
  - `MY_CR5_CONTROL_GUIDE_HYBRID_ALPHA`
- 当前支持三类单候选选择：
  - `top_prob`
    - 直接选预测概率最高的 learned candidate
  - `heuristic_gate`
    - 在通过概率阈值的 candidate 里选 heuristic 最优
  - `hybrid`
    - 用 `normalized heuristic cost - alpha * probability` 做折中选优
- 当前工程结论：
  - 这一步的意义不是再加一个“模型分数”
  - 而是把 single-candidate selection 从硬编码改成可对比、可消融、可重复实验的算法接口

- `hybrid` 在线尝试：
  - 配置：
    - `top_k=1`
    - `threshold=0.55`
    - `selection_mode=hybrid`
    - `hybrid_alpha=2.0`
    - 模型：`candidate_viable_refined`
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_092127_381_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_092127_381_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `heuristic_guided`：成功率 `100.0%`，平均 `592.1 ms`，预算命中 `0/12`
    - `learned_guided`：成功率 `91.7%`，平均 `1208.5 ms`，中位 `1127.0 ms`，预算命中 `0/12`
- 当前结论：
  - `hybrid` 没有把 learned guidance 速度拉回 heuristic 量级
  - 并且相较当前稳定版 `top_prob`，还带来了成功率回落
  - 因此当前默认选择模式仍固定为：
    - `selection_mode=top_prob`
  - `hybrid` 作为实验性接口保留，但暂不作为下一轮主推方案

## 22. 固定候选流的公平对照机制

- `2026-03-18` 已补上 `HeuristicGuided` 的确定性 guide sampling seed 支持
- 代码位置：
  - `include/my_cr5_control/cr5_robot.hpp`
  - `src/core/cr5_robot.cpp`
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`
- 当前新增环境变量：
  - `MY_CR5_CONTROL_GUIDE_SAMPLE_SEED`
- 当前机制要点：
  - `collect_dataset` 下可固定 guide candidate 采样流
  - `ablation` 下同一 `repeat + scene` 会先导出相同的 scenario seed
  - heuristic 与 learned 会在各自 `planToPoseImproved(...)` 前重置到同一 seed
  - 这保证两者看到同一批 candidate，而不是被不同随机采样流干扰

- `collect_dataset` 可复现性核对：
  - 对比文件：
    - `test_results/datasets/guide_ranking_simple/raw/20260318_093121_557_guide_ranking_simple_dataset_results.csv`
    - `test_results/datasets/guide_ranking_simple/raw/20260318_093148_517_guide_ranking_simple_dataset_results.csv`
  - 固定条件：
    - `MY_CR5_CONTROL_GUIDE_SCENES=Easy_TopCenter,Hard_HoleShallow`
    - `MY_CR5_CONTROL_GUIDE_SAMPLE_SEED=20260318`
  - 核对结论：
    - 候选生成与几何特征列 `0-36` 两次完全一致
    - 差异只出现在在线规划结果标签列：
      - `direct/guided wall time`
      - `hit_budget`
      - `candidate_fast`
      - `candidate_preferred`
  - 当前判断：
    - 固定 seed 已足够支撑“同一候选流”的公平对照
    - 规划耗时本身仍受在线求解波动影响，这是预期现象

- 固定条件 ablation：
  - 配置：
    - 场景：`Easy_TopCenter,Hard_HoleShallow`
    - `repeats=3`
    - `budget=4.0s`
    - `sample_seed=20260318`
    - `top_k=1`
    - `threshold=0.55`
    - `selection_mode=top_prob`
    - 模型：`candidate_viable_refined`
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_093457_054_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_093457_054_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance`：成功率 `100.0%`，平均 `2700.8 ms`，预算命中 `4/6`
    - `heuristic_guided`：成功率 `100.0%`，平均 `213.3 ms`，预算命中 `0/6`
    - `learned_guided`：成功率 `100.0%`，平均 `959.5 ms`，中位 `1118.0 ms`，预算命中 `0/6`
- 当前结论要固定：
  - 固定候选流后，heuristic vs learned 的比较条件已经更干净
  - 当前 learned guidance 的正向点仍然是：
    - 不再出现 budget-hit 塌陷
  - 但在同一 candidate stream 下：
    - learned 依旧明显慢于 heuristic
  - 一次离线排查已看到当前慢点来源：
    - `candidate_viable_refined` 的 `top_prob` 候选会偏向“高可行但偏慢”的 guide
    - 例如 `Hard_HoleShallow` 上，模型 top-prob 候选约 `1074 ms`
    - 同批 candidate 内存在约 `121 ms` 的 `candidate_fast=1` 候选，但预测概率更低
  - 因此下一步优化重点不是继续堆实验开关，而是：
    - 让 learned 排序在保留 `0 budget-hit` 的同时，减少不必要的慢候选进入在线尝试

## 23. guide ablation trace 与 direct-fallback 新发现

- `2026-03-18` 已补上 ablation trace 输出：
  - 代码位置：
    - `include/my_cr5_control/cr5_robot.hpp`
    - `src/core/cr5_robot.cpp`
    - `src/benchmarks/guide_ranking_simple_experiment.cpp`
  - 当前 `PlanningMetrics` 新增：
    - `guide_candidate_count`
    - `guide_candidates_attempted`
    - `used_direct_plan`
    - `selected_candidate_id`
    - `selected_candidate_learned_probability`
    - `selected_candidate_heuristic_cost`
    - `selected_candidate_ranking_score`
    - `selected_candidate_point`
  - 当前 ablation CSV 会直接写出：
    - guide 采样数
    - 实际保留候选数
    - 实际尝试次数
    - 是否退回 direct plan
    - 最终成功 candidate 的 trace

- 同一天顺手修正了一个接口一致性问题：
  - `MY_CR5_CONTROL_GUIDE_SAMPLE_COUNT` 之前只影响 `collect_dataset`
  - 现在 `ablation` 也会读取这个值
  - 但为兼容历史 HeuristicGuided 行为：
    - `ablation` 未显式设置时默认仍是 `24`

- `candidate_fast_refined` 固定条件对照：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_094327_566_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_094327_566_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `heuristic_guided`：成功率 `100.0%`，平均 `375.7 ms`，预算命中 `0/6`
    - `learned_guided`：成功率 `100.0%`，平均 `1277.7 ms`，中位 `1116.0 ms`，预算命中 `0/6`
  - 当前结论：
    - 单纯把目标切到 `candidate_fast_refined` 也没有直接把 online learned guidance 拉回 heuristic 量级

- 新 trace 版 `candidate_viable_refined` 固定条件对照：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_095059_676_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_095059_676_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `heuristic_guided`：成功率 `100.0%`，平均 `214.5 ms`，预算命中 `0/6`
    - `learned_guided`：成功率 `100.0%`，平均 `1127.2 ms`，中位 `651.0 ms`，预算命中 `0/6`
  - trace 直接揭示的新事实：
    - 在固定子集 `Easy_TopCenter + Hard_HoleShallow` 上
    - `heuristic_guided` 的 `guide_candidates_attempted = 0`，且 `used_direct_plan = 是`，共 `6/6`
    - 也就是当前 heuristic 版本在这组场景里实际没有使用 guide，而是全部退回 direct plan
    - `learned_guided` 则是 `guide_candidate_count = 1`、`guide_candidates_attempted = 1`、`used_direct_plan = 否`，共 `6/6`
    - 也就是当前 learned top-k=1 在这组场景里会稳定强制尝试一个 learned-selected guide
- 当前必须写死的判断：
  - 这组 fixed-seed 对照不只是“heuristic 比 learned 快”
  - 还包含了一个策略层差异：
    - heuristic 当前常常直接 fallback 到 direct
    - learned 当前常常强制走 guided route
  - 因此下一轮真正优先优化的对象已经从“再换一个训练 target”部分转移为：
    - `direct-fallback policy / direct-gate policy`
  - 如果这个 gate 不理顺，后续很多 learned vs heuristic 对比都会混入策略条件不一致

## 24. direct-gate 策略修复

- `2026-03-18` 已把 learned guidance 的 direct-cost gate 接到核心规划接口：
  - 代码位置：
    - `include/my_cr5_control/cr5_robot.hpp`
    - `src/core/cr5_robot.cpp`
    - `src/benchmarks/guide_ranking_simple_experiment.cpp`
  - 当前新增环境变量：
    - `MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE`
  - 当前支持：
    - `off`
    - `beat_direct`

- 这个 gate 的目的要写死：
  - 当 direct plan 已成功，且 learned-selected candidate 在几何代价上不优于 direct
  - learned guidance 不再被强制拖进 guided route
  - 而是直接回退到 direct plan
- 这一步的意义不是提速调参，而是先修复：
  - heuristic 常常 direct fallback
  - learned 常常 forced guided
  - 导致对比条件不对齐

- easy + hard_shallow 固定子集 direct-gate 结果：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_102735_175_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_102735_175_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `heuristic_guided`：成功率 `100.0%`，平均 `542.5 ms`，预算命中 `0/6`
    - `learned_guided + beat_direct`：成功率 `100.0%`，平均 `220.0 ms`，中位 `66.0 ms`，预算命中 `0/6`
  - trace 结论：
    - `learned_guided + beat_direct` 在该子集上也是 `6/6` 直接回退 direct
    - 且 `guide_candidates_attempted = 0`
    - 说明 forced-guided 路径已被成功切断

- harder 子集 `HardPlus_HoleEdgeOffset + Extreme_HoleDeep` 的 direct-gate 结果：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_102846_077_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_102846_077_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance`：成功率 `75.0%`，平均 `279.8 ms`
    - `heuristic_guided`：成功率 `50.0%`，平均 `2126.0 ms`，预算命中 `2/4`
    - `learned_guided + beat_direct`：成功率 `100.0%`，平均 `730.2 ms`，预算命中 `0/4`
  - 但这里必须明确写上保留判断：
    - trace 显示 `learned_guided + beat_direct` 在这组样本里依旧是 `4/4` 直接回退 direct
    - 所以这组结果不能被解释成“learned guidance 已真正学会更好的 guide”
    - 更准确的解释是：
      - direct-gate 先把 forced-guided 负担切掉了
      - 而 direct plan 本身的随机波动主导了最后的 success / time

- 当前阶段结论：
  - `direct-gate` 是正确方向，已经解决了 learned 被错误强制 guide 的核心策略问题
  - 但它也暴露出新的实验瓶颈：
    - 当 heuristic 与 learned 都 fallback direct 时
    - 差异会强烈受 direct planner 自身随机波动影响
- 因此下一步真正高优先级不再是继续换一个 target 名字，而是：
  - 要么做 `scenario-level direct result reuse`
  - 要么补 `direct planner reproducibility control`
  - 要么选一组“guide 实际会被触发”的 harder subset 做下一轮对比

## 25. 共享 direct baseline 与预算敏感 gate

- `2026-03-18` 已把 `scenario-level direct baseline reuse` 真正接入 `planToPoseImproved(...)`
  - 不再只是事后覆盖结果 CSV
  - 当前 heuristic / learned 在 ablation 中可以直接复用 `no_guidance` 的 direct baseline
  - 新增环境变量：
    - `MY_CR5_CONTROL_GUIDE_REUSE_DIRECT_BASELINE`
- 当前额外扩展了 direct gate 模式：
  - `MY_CR5_CONTROL_GUIDE_DIRECT_GATE_MODE=beat_direct_no_budget_hit`
  - 含义：
    - 只有当 direct 成功且没有撞预算时，才拦住 learned 去试 guide
    - 如果 direct 只是“4 秒成功但撞满预算”，允许 learned 继续尝试 guide

- 代码位置：
  - `include/my_cr5_control/cr5_robot.hpp`
  - `src/core/cr5_robot.cpp`
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`

- easy + hard_shallow 共享 direct baseline 结果：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_105133_746_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_105133_746_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance / heuristic_guided / learned_guided` 三者已对齐到同一 direct baseline
    - 均为 `100.0%`，平均 `1373.3 ms`，预算命中 `2/6`
  - 当前解释：
    - 这说明当前在这个子集上，guide 根本没有真正被触发
    - 之前的差异主要来自各自重复跑 direct 的随机波动

- harder 子集在共享 baseline 下的 `beat_direct` 结果：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_105546_387_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_105546_387_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - 三种模式全部约 `4s`
    - `guide_candidates_attempted = 0`
  - 当前解释：
    - 只要 gate 条件还是“direct 成功就拦”
    - 那么哪怕 direct 已撞预算，guide 依旧不会被触发
    - 这条规则太保守

- harder 子集在共享 baseline 下的 `beat_direct_no_budget_hit` 结果：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_105728_382_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_105728_382_learned_guidance_simple_ablation_summary.csv`
  - 摘要：
    - `no_guidance`：成功率 `100.0%`，平均 `2991.8 ms`，预算命中 `2/4`
    - `heuristic_guided`：成功率 `100.0%`，平均 `2994.0 ms`，预算命中 `2/4`
    - `learned_guided`：成功率 `100.0%`，平均 `3289.5 ms`，预算命中 `2/4`
  - 但这次最重要的不是均值，而是 trace：
    - `HardPlus_HoleEdgeOffset` 第 1 次：
      - `learned_guided` 已实际尝试 `1` 个 guide，且 `used_direct_plan = 否`
      - 说明“预算敏感 gate”已经能在 direct 撞预算时真正放开 guide
    - `Extreme_HoleDeep` 第 2 次：
      - `learned_guided` 也尝试了 `1` 个 guide，随后回退 direct
  - 当前结论：
    - 现在实验链路终于进入了“该 fallback 时 fallback、该放 guide 时放 guide”的状态
    - 这比之前“learned 永远被 forced-guided”或“guide 永远被 direct 成功拦住”都更合理

- 当前阶段最关键判断：
  - 我们已经把 online learned guidance 的实验口径修到足够可信：
    - 固定 candidate stream
    - 共享 direct baseline
    - direct-gate 可控
    - trace 可见
  - 下一步不应再优先修实验框架，而应把重点转到：
    - 在“direct 撞预算时被真正放开的这些 guide-attempt 场景”上，
    - 训练更贴近 `beat_direct_no_budget_hit` 目标的标签 / 排序模型

## 26. direct-rescue 目标与 trace 分析脚本

- `2026-03-18` 已修正 `train_guide_ranking_model.py` 的默认数据集选择：
  - 之前默认总是拿最新 raw dataset
  - 但最新文件已经变成 `17` 行的可复现性验证子集
  - 当前未显式传 `--dataset` 时，会跳过小于 `100` 行的 guide dataset
  - 现在默认会回到：
    - `test_results/datasets/guide_ranking_simple/raw/20260317_171238_252_guide_ranking_simple_dataset_results.csv`

- 同一天已新增派生训练目标：
  - `candidate_direct_rescue`
  - 定义：
    - 当 `direct` 失败或撞预算
    - 且该 guide `guided_success=1` 且 `guided_hit_budget=0`
    - 则记为正样本
  - 当前目的：
    - 让模型学“什么时候 direct 已经不够好，值得放 guide”
    - 比单纯的 `candidate_viable` 更贴近 `beat_direct_no_budget_hit` 的 online gate 逻辑

- 第一版 `candidate_direct_rescue` 训练结果：
  - run dir：
    - `test_results/models/guide_ranking_simple/20260318_114349`
  - 数据：
    - train `234` 行，正样本 `105`
    - test `71` 行，正样本 `31`
  - 指标：
    - test accuracy `0.704`
    - test F1 `0.656`
    - test AUC `0.738`
  - 当前判断：
    - 离线质量至少达到“可继续在线试验”的水平
    - 也避免了继续误用 17 行验证集训练模型

- 已新增 trace 分析脚本：
  - `scripts/models/analyze_guide_ablation_trace.py`
  - 作用：
    - 读取 ablation result csv
    - 汇总每个 mode 的：
      - attempted guide 次数
      - direct fallback 次数
      - budget hit 次数
      - mean wall time
    - 导出：
      - `trace_summary.csv`
      - `attempted_only.csv`
      - `trace_metadata.json`
- 已在关键 run 上执行：
  - 输入：
    - `test_results/benchmarks/simple_guidance/raw/20260318_105728_382_learned_guidance_simple_ablation_results.csv`
  - 导出目录：
    - `test_results/exports/guide_ablation_trace/20260318_114550`
  - 关键结论：
    - `learned_guided`：`attempted=2`
    - `heuristic_guided`：`attempted=0`
  - 这份导出当前就是“被真正放开 guide 的样本池”入口

- `candidate_direct_rescue` 在线试跑：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_114421_904_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_114421_904_learned_guidance_simple_ablation_summary.csv`
  - 配置：
    - `beat_direct_no_budget_hit`
    - `reuse_direct_baseline=1`
    - harder subset：`HardPlus_HoleEdgeOffset,Extreme_HoleDeep`
  - 这次摘要：
    - `no_guidance / heuristic_guided / learned_guided` 三者都 `100%`
    - 平均时间约 `254 ~ 257 ms`
    - `learned_guided` 的 `guide_candidates_attempted = 0`
  - 当前判断：
    - 这次 run 没踩到“direct 撞预算从而放 guide”的那一类 scenario
    - 所以它不能说明新模型已经在线变强
    - 但至少说明：
      - 新目标不会在 direct 已稳定时强行恶化当前策略

## 27. triggered-guide 子集与 budget-rescue 目标

- `2026-03-18` 已新增脚本：
  - `scripts/models/prepare_triggered_guide_dataset.py`
  - 作用：
    - 从 `guide_ablation_trace/*/attempted_only.csv` 提取“在线真的触发过 guide attempt”的场景池
    - 再从 canonical raw guide dataset 中反查并导出对应训练子集
  - 当前默认行为：
    - 默认读取最新一份 `attempted_only.csv`
    - 默认过滤 `mode=learned_guided`
    - 默认按 `场景名称` 回查
    - 默认输出到：
      - `test_results/datasets/guide_ranking_simple/filtered/`

- 同一天已新增更窄的派生标签：
  - `candidate_budget_rescue`
  - 定义：
    - `direct_hit_budget=1`
    - 且 `guided_success=1`
    - 且 `guided_hit_budget=0`
  - 当前定位：
    - 比 `candidate_direct_rescue` 更贴近 `beat_direct_no_budget_hit`
    - 只学习“direct 撞预算时，哪个 guide 真能把它救回预算内”

- 已基于当前 attempted-guide 池生成第一版 triggered 子集：
  - 来源 trace：
    - `test_results/exports/guide_ablation_trace/20260318_114550/attempted_only.csv`
  - 来源 raw dataset：
    - `test_results/datasets/guide_ranking_simple/raw/20260317_171238_252_guide_ranking_simple_dataset_results.csv`
  - 输出文件：
    - `test_results/datasets/guide_ranking_simple/filtered/20260318_115736_guide_ranking_simple_triggered_dataset_results.csv`
    - `test_results/datasets/guide_ranking_simple/filtered/20260318_115736_guide_ranking_simple_triggered_dataset_metadata.json`
  - 当前子集统计：
    - 总行数：`106`
    - 场景数：`2`
    - scene uid 数：`6`
    - `candidate_budget_rescue` 正样本：`31`
  - 当前子集覆盖：
    - `HardPlus_HoleEdgeOffset`
    - `Extreme_HoleDeep`

- 这条子集链路的意义固定为：
  - 不再让模型被大量“direct 已明显够好”的 easy scene 稀释
  - 优先在“guide 真有机会被放开”的局部场景上学排序
  - 为下一步在线 ablation 提供更聚焦的训练入口

- 已在这版 triggered 子集上训练第一版 `candidate_budget_rescue` 模型：
  - run dir：
    - `test_results/models/guide_ranking_simple/20260318_115745`
  - 测试指标：
    - accuracy `0.400`
    - F1 `0.087`
    - AUC `0.230`
  - 当前判断：
    - 这不是可直接上线的模型
    - 问题不是训练没跑通，而是当前 triggered 子集太小，且 `scene_uid` 间标签分布差异非常大
    - 例如：
      - `HardPlus_HoleEdgeOffset` 的不同 repeat 在正样本率上可从 `0/18` 到 `18/18`
      - `Extreme_HoleDeep` 的不同 repeat 也存在 `0/17` 到 `7/18` 的明显波动
    - 因此当前更合理的下一步不是直接上线这版模型，而是：
      - 继续扩大 attempted-guide trace 池
      - 或补能解释 repeat 间差异的特征

## 28. 扩大 triggered 池 + rescue 特征 + 新一轮 online 结果

- `2026-03-18` 已扩展 `prepare_triggered_guide_dataset.py`：
  - 除了读取 `attempted_only.csv`
  - 现在也支持直接输入多份 ablation `results.csv`
  - 脚本会自动提取其中 `guide_candidates_attempted > 0` 的 attempted rows
  - 这样后续不需要先手工逐个导 trace 再回建 triggered 子集

- 同一天已扩展 `train_guide_ranking_model.py`：
  - 新增 `--feature-profile`
    - `geometric`
    - `rescue`
  - `rescue` profile 在原始几何候选特征之外，还会加入：
    - `difficulty_score_raw`
    - `direct_success_flag`
    - `direct_hit_budget_flag`
    - `direct_bad_flag`
    - `direct_wall_time_ratio`
    - `direct_moveit_time_ratio`
  - 当前判断：
    - 这些 direct baseline 特征是必要的
    - 因为 `candidate_budget_rescue` 本身就是在回答：
      - `direct` 已经坏到什么程度时，guide 值得被放开

- 同时已修正一个训练脚本细节问题：
  - `train_guide_ranking_model.py` 默认输出目录之前只按秒级时间戳命名
  - 并行起两次训练会发生 run 目录覆盖
  - 当前已改成：
    - 若时间戳目录已存在，则自动分配带后缀的新目录

- 已基于两份 learned-attempt run 扩大 triggered 子集：
  - 来源 results：
    - `test_results/benchmarks/simple_guidance/raw/20260318_095059_676_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_105728_382_learned_guidance_simple_ablation_results.csv`
  - 输出：
    - `test_results/datasets/guide_ranking_simple/filtered/20260318_120756_guide_ranking_simple_triggered_dataset_results.csv`
    - `test_results/datasets/guide_ranking_simple/filtered/20260318_120756_guide_ranking_simple_triggered_dataset_metadata.json`
  - 当前统计：
    - 总行数：`199`
    - 场景数：`4`
    - `candidate_budget_rescue` 正样本：`93`
  - 当前覆盖场景：
    - `Easy_TopCenter`
    - `Hard_HoleShallow`
    - `HardPlus_HoleEdgeOffset`
    - `Extreme_HoleDeep`

- 在这版 `199` 行 triggered 子集上训练 `candidate_budget_rescue`：
  - `geometric` profile：
    - run dir：`test_results/models/guide_ranking_simple/20260318_120854`
    - test accuracy `0.312`
    - test F1 `0.476`
    - test AUC `0.285`
  - `rescue` profile：
    - run dir：`test_results/models/guide_ranking_simple/20260318_120848`
    - test accuracy `0.646`
    - test F1 `0.785`
    - test AUC `0.915`
  - 关键结论：
    - 仅靠几何候选特征不足以学会 `budget_rescue`
    - 把 direct baseline 状态纳入特征后，离线质量出现明显跃升

- 为了让这版 rescue 模型可以真正进入 online ablation：
  - 已在 `src/benchmarks/guide_ranking_simple_experiment.cpp` 中补上 learned scorer 对 direct rescue 特征的支持
  - 当前 benchmark-side learned scorer 已可识别：
    - `difficulty_score_raw`
    - `direct_success_flag`
    - `direct_hit_budget_flag`
    - `direct_bad_flag`
    - `direct_wall_time_ratio`
    - `direct_moveit_time_ratio`
  - 注意：
    - 这里先只打通了 benchmark-side online scorer
    - 还没有把这套 richer feature context 上升为通用 `CR5Robot` ranking 接口

- 第一轮 online ablation（`sample_count=24`）：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_121248_268_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_121248_268_learned_guidance_simple_ablation_summary.csv`
  - trace：
    - `test_results/exports/guide_ablation_trace/20260318_121326`
  - 配置：
    - `reuse_direct_baseline=1`
    - `beat_direct_no_budget_hit`
    - scenes：`Easy_TopCenter,Hard_HoleShallow,HardPlus_HoleEdgeOffset,Extreme_HoleDeep`
    - model：`20260318_120848`
  - 摘要：
    - `no_guidance`：成功率 `91.7%`，平均 `1285.9 ms`，预算命中 `3`
    - `heuristic_guided`：成功率 `91.7%`，平均 `1631.2 ms`，预算命中 `4`
    - `learned_guided`：成功率 `100.0%`，平均 `1595.2 ms`，预算命中 `3`
  - trace 结论：
    - `heuristic_guided` attempted `1`
    - `learned_guided` attempted `4`
    - learned 确实更积极地在“允许放 guide”的场景上触发了 rescue
  - 关键逐场景收益：
    - `Extreme_HoleDeep / repeat 1`
      - `direct` 失败 `2422 ms`
      - `heuristic` 失败 `6542 ms`
      - `learned` 成功 `2867 ms`
    - 这是当前最重要的一条正信号：
      - learned rescue 已经不只是离线分数提升
      - 它在线上确实救回了一个 heuristic / direct 都没解决好的 case
  - 同时也出现了当前副作用：
    - learned 在若干 “direct 已成功但撞预算” 的 case 上也会放 guide
    - 例如：
      - `HardPlus_HoleEdgeOffset / repeat 1`
      - `HardPlus_HoleEdgeOffset / repeat 3`
      - `Hard_HoleShallow / repeat 3`
    - 这些 case 没把预算命中救回来，反而拉高了 mean time

- 第二轮 online ablation（`sample_count=12`，与 canonical guide dataset 分布对齐）：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260318_121425_909_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260318_121425_909_learned_guidance_simple_ablation_summary.csv`
  - trace：
    - `test_results/exports/guide_ablation_trace/20260318_121458`
  - 摘要：
    - `no_guidance`：成功率 `100.0%`，平均 `1124.3 ms`
    - `heuristic_guided`：成功率 `100.0%`，平均 `1127.1 ms`
    - `learned_guided`：成功率 `100.0%`，平均 `1314.6 ms`
  - 当前判断：
    - `sample_count` 与训练分布对齐后，并没有自动消除 learned 的额外尝试
    - 这说明问题不只是 train/test candidate count mismatch
    - 更深层的问题仍然是：
      - learned 还不够会区分“值得救的 budget-hit”与“救了也大概率救不回来的 budget-hit”

- 到这一步的总体结论应固定为：
  - `正确的方向已经很清楚`
    - `triggered subset + budget_rescue target + direct rescue features`
    - 这是当前 learning-guided 主线里真正有效的算法抓手
  - `但 online policy 还没有收敛`
    - 已经看到明确正收益 case
    - 也看到若干无效放 guide 的 case
  - 因此下一步应聚焦：
    - 扩大 hard attempted pool
    - 补更细的 direct 压力 / 可救性特征
    - 或继续收窄 target，把“预算命中但可救回”的定义再做细化

## 29. guide model schema 已统一到共享配置

- `2026-03-18` 已新增共享 schema 目录：
  - `config/guide_model_schema/feature_profiles.csv`
  - `config/guide_model_schema/targets.csv`

- 当前统一收口的内容：
  - `feature profile`
    - `geometric`
    - `rescue`
  - `target schema`
    - `candidate_viable`
    - `candidate_fast`
    - `candidate_preferred`
    - `candidate_direct_rescue`
    - `candidate_budget_rescue`

- Python 侧现在不再手写 profile / target 常量：
  - `scripts/models/train_guide_ranking_model.py`
  - `scripts/models/prepare_triggered_guide_dataset.py`
  - 它们统一通过：
    - `scripts/models/guide_model_schema.py`
    - 读取共享 schema

- C++ online 侧也已接入共享 schema 校验：
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`
  - 当前在加载 `linear_model.csv` 时会检查：
    - model target 是否出现在共享 `targets.csv`
    - model feature 是否出现在共享 `feature_profiles.csv`
  - 工程意义：
    - 即使以后 Python 训练脚本改了 feature 名
    - 只要 C++ online scorer 没同步支持
    - runtime 会直接报错，而不是悄悄带着错误模型继续跑 ablation

- 同时已补安装一致性：
  - `CMakeLists.txt` 现在会安装：
    - `config/`
    - `scripts/models/guide_model_schema.py`
    - `prepare_triggered_guide_dataset.py`
    - `analyze_guide_ablation_trace.py`

- 当前判断：
  - 这次改动没有直接提升算法指标
  - 但它修掉了一个很危险的工程风险：
    - `offline 训练特征` 与 `online 打分特征` 双份漂移
  - 后续所有 guide ranking 实验，默认都应建立在这份共享 schema 上

## 30. HeuristicGuided 数学边界与底层 planner 决策

- `2026-03-19` 需要把一个容易被自己写偏的边界固定下来：
  - 当前 `HeuristicGuided` 不是“简单调用某个库里的 heuristic planner”
  - 也不是“我们已经提出了一个新的底层 OMPL planner”
  - 它是：
    - 自定义 `guide pose` 采样
    - 自定义候选特征计算
    - 自定义 heuristic 排序/过滤
    - 自定义 `start -> guide -> goal` 两段式调度
    - 最后把每一段规划请求交给底层 MoveIt / OMPL

- 当前数学实现主干应统一理解为：
  - 候选生成：
    - `generateEllipsoidGuideSamples(...)`
    - `generateRefinedGuideSamples(...)`
  - 候选基础代价：
    - `computeImprovedPathCost(...)`
    - 形式上是：
      - `路径长度`
      - 加 `guide / 两段中点` 的障碍区域惩罚
  - 候选增强项：
    - `clearance-aware` safety penalty
    - `IK / manipulability-aware` penalty
  - 最终 ranking：
    - `heuristic_cost + 0.45 * safety_penalty + 0.35 * manipulability_penalty`

## 31. Learned Guidance 隔离域链路修复与最新 clean ablation

- `2026-03-19` 已把 learned-guidance 的隔离域 launch 链路修到“可复现执行”：
  - `src/benchmarks/guide_ranking_simple_experiment.cpp`
    - CSV 解析现在会统一清洗 `BOM / CRLF / 首尾空白`
    - 直接修复了：
      - `linear_model.csv` 为 `CRLF` 时
      - `model.target_name` 带隐藏 `\r`
      - 从而被 runtime 误判为“不在共享 schema 中”的问题
  - `launch/guide_experiment.launch.py`
    - 已从 `move_group.launch.py` 切到 `cr5_moveit/demo.launch.py`
    - 原因：
      - 仅起 `move_group` 时没有 fake controller / `joint_states`
      - ablation 会退化成 `Failed to fetch current robot state`
      - 并产出伪失败结果
    - 现在还会在 `guide_ranking_simple_experiment_node` 退出后自动 `Shutdown`
      - 后续隔离域实验不再需要靠外部 `timeout` 硬切

- 最小 smoke 已确认：
  - `20260319_160358_840`
  - `candidate_budget_rescue (20260318_120848)`
  - 结果说明：
    - schema 校验已通过
    - learned policy 已真正进入 online 执行
    - `guide_candidate_count / attempted / selected_candidate_*` 字段都已正常写出

- 第一轮 clean subset ablation：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260319_160821_976_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260319_160821_976_learned_guidance_simple_ablation_summary.csv`
  - 配置：
    - model：`20260318_120848`
    - target：`candidate_budget_rescue`
    - scenes：
      - `Easy_TopCenter`
      - `Hard_HoleShallow`
      - `HardPlus_HoleEdgeOffset`
      - `Extreme_HoleDeep`
    - repeats：`3`
    - `sample_count=12`
    - `reuse_direct_baseline=1`
    - fixed `sample_seed=20260318`
  - 摘要：
    - `no_guidance`：`100%`，均值 `64.3 ms`
    - `heuristic_guided`：`100%`，均值 `71.8 ms`
    - `learned_guided`：`100%`，均值 `998.2 ms`
  - trace 结论：
    - `learned_guided attempted = 12/12`
    - `heuristic_guided attempted = 0/12`
  - 工程判断：
    - 当前 `candidate_budget_rescue` 在线 gate 明显过激
    - 在这组 clean subset 上几乎把所有 case 都拖进了 guided route
    - 它已不是“接口未打通”
    - 而是“模型目标 / gate 选择性不够”

- 第二轮 clean subset ablation：
  - 结果：
    - `test_results/benchmarks/simple_guidance/raw/20260319_161601_847_learned_guidance_simple_ablation_results.csv`
    - `test_results/benchmarks/simple_guidance/raw/20260319_161601_847_learned_guidance_simple_ablation_summary.csv`
  - 配置：
    - model：`20260318_114349`
    - target：`candidate_direct_rescue`
    - 其余配置与上一轮相同
  - 摘要：
    - 同一 run 内：
      - `no_guidance`：均值 `1061.8 ms`
      - `heuristic_guided`：均值 `1068.4 ms`
      - `learned_guided`：均值 `1835.8 ms`
    - `budget_hit` 三者都 `3/12`
    - `learned_guided attempted = 12/12`
  - 当前判断：
    - 换成 `candidate_direct_rescue` 后，online 行为仍然不够保守
    - 它没有减少 budget hit
    - 也没有在当前 clean subset 上形成速度收益

- 到这一步的结论必须固定：
  - learned guidance 这条工程链已经能稳定跑通
  - 当前问题不再是 launch / schema / runtime integration
  - 当前两个最值得尝试的在线模型：
    - `candidate_budget_rescue`
    - `candidate_direct_rescue`
    - 都表现出“false-positive guide trigger 过多”的同类问题
  - 因此它们现在还不能进入论文主结果主表

- 下一步应收口到：
  - 继续扩大 hard attempted pool
  - 把 target 从“能救”进一步收紧到“值得救且大概率不拖慢”
  - 或在 online gate 上加入更强的 direct-side 抑制
    - 例如 direct 风险阈值校准
    - 或显式的“不优于 direct 就不放 guide”判据

- 额外工程注记：
  - `guide_experiment.launch.py` 自动收尾后，`move_group` 在 teardown 阶段仍可能出现退出期段错误
  - 但当前实验结果文件都已在此之前写出
  - 因此这是“launch 收尾质量”问题，不影响本轮结论有效性
  - 代码主位置：
    - `src/core/cr5_robot.cpp`
    - `include/my_cr5_control/cr5_robot.hpp`

- 当前库调用边界也要写死：
  - 底层真正执行规划的是：
    - `move_group_->plan(plan)`
  - 当前默认 planner id 是：
    - `RRTConnect`
  - 也就是说：
    - `HeuristicGuided` 的创新层在外部 guidance layer
    - 不在底层求解器替换本身

- 当前工程决策：
  - 短期内不建议为了“看起来更新”而立刻替换默认底层 backend
  - 原因：
    - 当前最强信号来自 guidance layer 对 `budget-hit` 和长尾的抑制
    - 若在 learned guidance 尚未稳定前同时更换底层 planner
    - 会把方法增益与 backend 变更混在一起
  - 因此默认策略保持：
    - `HeuristicGuided guidance layer + RRTConnect backend`

- 但论文侧需要补 stronger modern baseline，而不是只保留经典 planner：
  - 建议新增或重点补充：
    - `BIT*`
    - `Informed RRT*`
  - 同时保留：
    - `PRMstar`
    - `RRTConnect`
  - 当前含义：
    - 不是马上把生产/benchmark 默认 backend 切掉
    - 而是让论文对照组更能回答“modern sampling / informed search 是否已经覆盖了我们的收益”

- 当前近三年的文献支撑可按下面这条逻辑使用：
  - 对当前外层 guidance / learned sampling 主线：
    - Johnson, Qureshi, Yip,
      `Learning Sampling Dictionaries for Efficient and Generalizable Robot Motion Planning with Transformers`,
      `IEEE Robotics and Automation Letters`, `2023`
    - 结论意义：
      - 近年高水平工作仍然在“学习引导 classical SMP”这条线推进
      - 并不是都在替换底层规划器
  - 对受限/约束规划问题本身：
    - Ni, Qureshi,
      `Physics-informed Neural Motion Planning on Constraint Manifolds`,
      `ICRA`, `2024`
    - 结论意义：
      - 受限/约束规划仍然是前沿问题
      - 我们的任务设定没有过时
  - 对 roadmap 家族近年仍有高水平延展：
    - Zheng et al.,
      `CS-BRM: A Probabilistic RoadMap for Consistent Belief Space Planning With Reachability Guarantees`,
      `IEEE Transactions on Robotics`, `2024`
    - 结论意义：
      - `PRM / roadmap` 不是“老掉牙就不能写”
      - 但要把它放在方法脉络里，而不是当作我们自己的创新

- 因此当前最稳的论文与工程统一表述应固定为：
  - 我们的方法创新集中在：
    - `learning-guided / heuristic-guided sampling interface`
  - 当前底层 planner：
    - 是稳定 backend
  - 论文下一步需要做的不是“先急着换底层”
  - 而是：
    - 固定当前 backend
    - 补 modern baseline
    - 证明 guidance layer 自身的独立增益

- `2026-03-19` 新增运行时结论：
  - 本机 `/opt/ros/humble` 的 `ompl` 头文件虽然存在：
  - `BITstar`
  - `InformedRRTstar`

## 32. 论文主线冻结到 HeuristicGuided + heuristic 工程下一步

- `2026-03-19` 当前主线决策已经固定：
  - 论文主结果先冻结在：
    - `HeuristicGuided two-stage guidance interface`
  - `learned guidance`
    - 降级为扩展线 / 诊断线
    - 只有在在线上形成稳定正收益后，才允许重新升回主方法

- 这样定主线的原因不是保守，而是证据强弱已经分层：
  - `HeuristicGuided`
    - 已有 canonical benchmark
    - 已有清楚的工程边界
    - 已有可写进主表的 success / time / budget-hit 结果
  - `learned guidance`
    - 现在已经能跑
    - 但仍然会系统性 false-positive guide trigger
    - 当前不能支撑“主方法增益”叙事

- 本轮已执行的 heuristic 主线工程：
  - 在 `src/benchmarks/planner_comparison_simple.cpp`
  - 在 `src/benchmarks/planner_comparison_v2.cpp`
  - 已补写出：
    - `guide候选数`
    - `guide尝试数`
    - `使用direct回退`
  - 这样后续调 `HeuristicGuided` 时，不再只能看总时间和成功率

- 同时已在 `src/core/cr5_robot.cpp` 增加 heuristic 调参开关：
  - `MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS`
    - 限制一次 `HeuristicGuided` 调用里最多放开多少条 guide route
    - 默认 `0`
    - 表示不限制
  - `MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS`
    - 实验性开关
    - 当 direct 已成功，但 direct 本身已经慢到超过该阈值时，允许 heuristic 放宽一点 direct-cost gate 去尝试 guide rescue
    - 默认关闭

- 这两个新开关的当前验证结果：
  - baseline smoke：
    - `test_results/benchmarks/simple/raw/20260319_164811_286_planner_comparison_simple_results.csv`
    - `test_results/benchmarks/simple/raw/20260319_164811_286_planner_comparison_simple_summary.csv`
  - 配置：
    - `simple`
    - `HeuristicGuided only`
    - `repeats=3`
    - slow-direct rescue 未开启
  - 结果：
    - 成功率 `18/18`
    - 平均时间 `364.4 ms`
    - 平均 guide 尝试 `0.0`
    - `direct fallback = 18/18`
  - 当前解释：
    - 在这组 clean smoke 上
    - 当前 `HeuristicGuided` 基本表现为：
      - `direct-first + failure rescue`
    - 而不是“经常主动放 guide 的 planner”

- 已做一轮 slow-direct rescue 原型试跑：
  - 结果：
    - `test_results/benchmarks/simple/raw/20260319_165438_421_planner_comparison_simple_results.csv`
    - `test_results/benchmarks/simple/raw/20260319_165438_421_planner_comparison_simple_summary.csv`
  - 试验口径等价于：
    - `MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS=3`
    - `MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS=800`
  - 结果：
    - 只触发了 `2/18` 条 guide attempt
    - 但把：
      - `Medium_SideSurface`
      - `HardPlus_HoleEdgeOffset`
    - 拉出了明显长尾
      - 其中最坏达到 `3101 ms`
    - 这轮均值从 `364.4 ms` 升到 `373.3 ms`
  - 当前判断：
    - `slow-direct rescue` 不能作为默认主线策略
    - 它更像一个实验性开关
    - 只有在 hard subset 上进一步校准后，才值得继续

- 已验证默认主线仍然稳：
  - `test_results/benchmarks/simple/raw/20260319_170038_320_planner_comparison_simple_results.csv`
  - `repeats=1`
  - 未开启 slow-direct rescue
  - 结果：
    - `guide尝试数 = 0`
    - `direct fallback = 6/6`
  - 说明：
    - 当前默认主线行为已恢复为保守 heuristic rescue

- 到这一步，heuristic 主线的工程结论应固定为：
  - 主论文线继续写 `HeuristicGuided`
  - learned 暂不争主位
  - heuristic 下一步不要追求“盲目让更多 case 放 guide”
  - 下一步要做的是：
    - 找到哪些 hard subset 下 direct 长尾真的值得 rescue
    - 再局部细化 heuristic gate
    - 而不是在全场景上粗暴打开 slow-direct trigger
    - `ABITstar`
    - `AITstar`
    - `EITstar`
  - 但当前二进制 `moveit_ompl_interface` 并没有把 `geometric::BITstar` 暴露成可运行 planner allocator
  - 实测里只把 `BITstar / InformedRRTstar` 写入 `cr5_moveit/config/ompl_planning.yaml` 并重新安装 `cr5_moveit` 后，
    真实规划仍会报：
    - `Unknown planner: 'geometric::BITstar'`
  - 这说明：
    - `YAML 注册完成 != MoveIt 运行时真正支持`
    - 当前 Humble 二进制环境下，modern baseline 不能靠改配置文件直接接入

- 因此短期策略再收紧一层：
  - 默认实验线继续保持：
    - `HeuristicGuided guidance layer + RRTConnect backend`
  - `BIT* / Informed RRT*` 仍可作为论文中“应补强 baseline”的目标
  - 但若要把它们变成当前环境下真正可运行的主表对照，

    下一步必须是：
    - 重编译/补丁化 `moveit_planners_ompl` 或 `moveit_ompl_interface`
    - 而不是只改 `ompl_planning.yaml`

- `2026-03-19` 后续执行结果：
  - 已在工作区引入 `moveit_planners_ompl` overlay，并在其
    `planning_context_manager.cpp` 中补注册：
    - `geometric::BITstar`
    - `geometric::InformedRRTstar`
  - 重新编译 overlay 后，MoveIt 运行时已不再报：
    - `Unknown planner: 'geometric::BITstar'`
  - 实际 smoke test 结果分化明显：
    - `BITstar`
      - 已被 MoveIt 正确实例化
      - 可在 `planner_comparison_simple_node` 中完成真实规划请求
      - 但第一次长跑尝试 `20260319_153346_298` 在中途中断，只留下 `34` 条完整样本：
        - 成功 `34 / 34`
        - budget hit `13 / 34`
        - mean wall `5104.3 ms`
        - median wall `324.5 ms`
      - 该批数据不能进入正式汇总，因为 run 未正常结束
      - 随后在干净隔离域中执行：
        - `ROS_DOMAIN_ID=88 ros2 launch my_cr5_control planner_benchmark.launch.py benchmark:=simple repeats:=10 planners:=BITstar`
      - 可复现更强结论：
        - `move_group` 在第 1 轮 `Extreme_HoleDeep` 场景内崩溃
        - 错误为 `SIGSEGV`
        - 堆栈落在：
          - `ompl::geometric::BITstar::solve(...)`
          - `ompl::geometric::BITstar::publishSolution()`
          - `ompl::geometric::BITstar::Vertex::state() const`
      - 因此当前对 `BITstar` 的最准确定位应改成：
        - 已完成配置与运行时接入
        - 可处理部分短程请求
        - 但在当前 MoveIt2 Humble + OMPL 1.7 组合下，不具备正式 benchmark 稳定性
    - `InformedRRTstar`
      - 已完成运行时注册
      - 初始版本会在当前 MoveIt goal construction / GoalLazySamples 链路下稳定触发
        `PathLengthDirectInfSampler: There must be at least 1 start and 1 goal state`
      - 后续已在 overlay `model_based_planning_context.cpp` 中增加
        `geometric::InformedRRTstar` 的求解前 lazy-goal warm-up
      - 修复后真实 smoke `20260319_152557_979` 已不再触发上述初始化异常
      - 但当前 `simple` 单轮 smoke 指标仍为：
        - 成功率 `83.3% (5/6)`
        - 平均时间 `10033.2 ms`
        - budget hit `6/6`
      - 结论应改成：
        - 已可运行
        - 但当前性能仍明显弱于可进入主表的 stronger baseline 标准

- 因此当前更准确的 modern baseline 结论应改写为：
  - 当前已完成配置与运行时注册的 modern planners：
    - `BITstar`
    - `InformedRRTstar`
  - 当前不应直接进入正式主表的：
    - `BITstar`
    - `InformedRRTstar`
  - 当前论文主表仍应保持：
    - `HeuristicGuided`
    - `BFMT`
    - `LBTRRT`
    - `FMT`
    - `PRMstar`
    - `RRTConnect`
    - `RRTstar`
  - 当前工程下一步优先级应重新上调为：
    - 回到 `learned guidance` 闭环与正式消融
    - 而不是继续消耗时间在不稳定的 modern baseline 接入上

## 33. improved RRT*-Connect 论文复现 TEST 已完成，但不进入当前主线

- `2026-03-21` 已在工程内完成一条独立旁路线：
  - 读取 `RRT nature Q1.pdf`
  - 新增：
    - `src/tools/test_rrtstar_connect_reproduction.cpp`
    - `docs/analysis/TEST_RRTSTAR_CONNECT_REPRODUCTION.md`
  - 新增可执行：
    - `test_rrtstar_connect_reproduction_node`

- 这条 TEST 线当前已经覆盖：
  - 论文 `A novel RRT*-Connect algorithm for path planning on robotic arm collision avoidance`
  - 二维最小可运行复现
  - 公式映射到代码：
    - 椭球采样
    - 自适应目标偏置
    - 改进代价函数
    - APF 自适应步长
    - 双向 `RRT*-Connect`
    - 分段三次 `Bezier` 平滑
  - 已完成编译与 smoke test

- 但这条线当前必须明确降级为：
  - `参考原型 / 论文算法复现资产`
  - 而不是当前工程主线

- 原因要写死：
  - 当前论文主线已经冻结在：
    - `HeuristicGuided two-stage guidance interface`
  - 当前最需要补的是：
    - `heuristic gate` 的局部细化
    - hard subset 下的 rescue 条件
    - `HeuristicGuided` 主线实验与论文主表闭环
  - 现在如果把 improved `RRT*-Connect` 强行并入主链路，会把：
    - 外层 guidance 论文主线
    - 与底层 planner 替换问题
    - 混在一起

- 因此当前固定决策：
  - `TEST` 保留
  - 不删除
  - 不接入当前 benchmark 主表
  - 不作为当前 `HeuristicGuided` 的默认实现

- 后续只有在下面三种需求出现时，才重新启用这条线：
  1. 需要把论文中的 constrained-planning 数学模块进一步吸收到当前 heuristic baseline
  2. 需要把 `APF step / obstacle-area cost / Bezier smoothing` 逐项迁移到 `CR5Robot`
  3. 需要在论文写作中补“原始算法复现 / 方法脉络对照 / appendix prototype”

- 当前工程主线不因这条 TEST 改变：
  - 下一步仍然优先做：
    - `hard subset` 下 direct 长尾是否值得 rescue 的筛查
    - 再基于筛查结果局部细化 `HeuristicGuided` gate
  - 这条优先级判断高于前文阶段性出现的：
    - “先回到 learned guidance 闭环”的临时建议
  - 从现在开始，以本条和 `## 32` 的主线冻结结论为准

## 34. hard subset rescue 筛查工具链已补齐，当前结论不支持保留全局 slow-direct rescue

- `2026-03-21` 已补齐为 heuristic 主线服务的两项基础设施：
  - benchmark 场景子集过滤
    - `planner_comparison_simple_node` 支持：
      - `MY_CR5_CONTROL_SIMPLE_SCENES`
      - `MY_CR5_CONTROL_BENCHMARK_SCENES`
    - `planner_comparison_v2_node` 支持：
      - `MY_CR5_CONTROL_V2_SCENES`
      - `MY_CR5_CONTROL_BENCHMARK_SCENES`
  - benchmark launch 自动退出与场景透传
    - `launch/planner_benchmark.launch.py`
    - 新增：
      - `scenes:=...`
    - benchmark 节点结束后 launch 自动 shutdown

- 同时新增专门的筛查脚本：
  - `scripts/benchmarks/analyze_heuristic_rescue_candidates.py`
  - 作用：
    - 从 raw benchmark csv 里按场景汇总：
      - `direct fallback rate`
      - `guide attempt rate`
      - `p75 / p90 / max wall time`
      - `budget-hit`
    - 自动给出：
      - `recommended_subset`
      - baseline / compare 场景级差分

- 先对历史全场景对照做了一次筛查：
  - baseline：
    - `20260319_164811_286`
  - slow-direct rescue：
    - `20260319_165438_421`
  - 结论：
    - 全场景里 even `Easy_TopCenter` 也会因 direct 随机长尾被误判成“值得 rescue”
    - 说明不应继续用全场景结果直接决定 heuristic gate
    - 下一步必须改用 `hard subset` 重新筛查

- 随后已跑 `hard subset = Hard_HoleShallow, HardPlus_HoleEdgeOffset, Extreme_HoleDeep`
  - baseline：
    - `ROS_DOMAIN_ID=88`
    - `20260321_091037_103`
    - 结果：
      - 成功率 `100%`
      - 平均 `278.1 ms`
      - `guide尝试 = 0`
      - `direct fallback = 100%`
  - slow-direct rescue：
    - `ROS_DOMAIN_ID=89`
    - `MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS=3`
    - `MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS=800`
    - `20260321_091103_400`
    - 结果：
      - 成功率 `100%`
      - 平均 `744.9 ms`
      - 平均 `guide尝试 = 0.3`
      - `direct fallback = 66.7%`

- 这组 hard subset 的场景级差分结论要固定：
  - `Extreme_HoleDeep`
    - slow-direct rescue 明显变差
    - `delta_mean_ms = +1675.3`
    - `delta_p75_ms = +2517.5`
    - 当前应视为：
      - `rescue_hurt`
  - `Hard_HoleShallow`
    - 放开 guide 后略慢
    - 当前没有正收益证据
  - `HardPlus_HoleEdgeOffset`
    - 本次 compare 表面更快
    - 但 `guide_attempt_rate` 没有上升
    - 说明收益来自 direct 随机波动，而不是 rescue 本身

- 当前最重要的工程结论：
  - 全局 `slow-direct rescue` 仍然不应保留为默认策略
  - `Extreme_HoleDeep` 不应作为下一轮 heuristic rescue 的主攻点
  - 如果还要继续局部细化 heuristic gate，下一轮只值得盯：
    - `Hard_HoleShallow`
    - `HardPlus_HoleEdgeOffset`
  - 但在继续改 gate 之前，必须承认：
    - 当前 direct planner 随机波动仍然足够大
    - 很容易把“随机快慢”误读成“guide rescue 有效”

- 因此从这一步开始，heuristic 主线的下一条更具体执行顺序应固定为：
  1. 继续只在 `hard subset` 上做 heuristic gate 调整
  2. 不再用全场景直接评估 slow-direct rescue
  3. 默认禁止把 `Extreme_HoleDeep` 纳入 slow-direct 放宽触发候选
  4. 若后续再做 rescue 规则试验，必须同时输出 scene-level compare，而不是只看总均值

## 35. heuristic rescue 新诊断与 centered-insertion gate

- `2026-03-21` 在 `hard subset` 上继续推进时，先补了 pre-gate 数学诊断列：
  - `include/my_cr5_control/cr5_robot.hpp`
  - `src/core/cr5_robot.cpp`
  - `src/benchmarks/planner_comparison_simple.cpp`
  - `src/benchmarks/planner_comparison_v2.cpp`
  - 新写入 csv 的核心列包括：
    - `direct规划成功`
    - `direct尝试时间(ms)`
    - `direct路径代价`
    - `top guide heuristic_cost`
    - `top guide ranking_score`
    - `top guide cost_delta_to_direct`
    - `top guide clearance / manipulability / axial_progress / lateral_offset`

- 先跑了新的 hard-subset baseline：
  - `20260321_093502_932`
  - 配置：
    - `HeuristicGuided only`
    - 未开启 slow-direct rescue
  - 关键结论要固定：
    - 三个 hard scene 里 `guide尝试` 仍然都是 `0`
    - 但 top guide 并不是“远差于 direct”
    - 它们对 `direct_cost` 的差值通常只有：
      - `Hard_HoleShallow`: 均值约 `+0.0067`
      - `HardPlus_HoleEdgeOffset`: 均值约 `+0.0027`
      - `Extreme_HoleDeep`: 均值约 `+0.0032`
    - 同时 `top guide ranking_score ≈ top guide heuristic_cost`
    - 说明这批 hard scene 当前的主矛盾不是 ranking penalty，而是：
      - 候选大多只是 `near-tie but slightly worse than direct`

- 随后用旧的保守 rescue gate 重新跑诊断版 compare：
  - `20260321_093606_150`
  - 配置：
    - `MY_CR5_CONTROL_HEURISTIC_MAX_GUIDE_ATTEMPTS=3`
    - `MY_CR5_CONTROL_HEURISTIC_SLOW_DIRECT_THRESHOLD_MS=800`
  - 场景级结果：
    - `Hard_HoleShallow`
      - `rescue_helped`
      - `delta_mean_ms = -284.3`
      - `delta_p75_ms = -427.0`
      - `delta_attempt_rate = +0.33`
    - `HardPlus_HoleEdgeOffset`
      - `rescue_hurt`
      - `delta_mean_ms = +1361.3`
      - `delta_p75_ms = +2032.0`
      - `delta_attempt_rate = +0.67`
    - `Extreme_HoleDeep`
      - `no_clear_change`
      - `delta_attempt_rate = 0.00`
  - 这一步给出的最重要新判断是：
    - slow-direct rescue 不是“对所有 hard scene 都有价值”
    - 它只可能在 `centered shallow insertion` 一类目标上有局部价值
    - 对 `off-center edge-offset` 目标，放 guide 的风险明显更大

- 基于这条结论，已把保守 rescue gate 再收紧一层：
  - `include/my_cr5_control/cr5_robot.hpp`
  - `src/core/cr5_robot.cpp`
  - 新增：
    - `isCenteredInsertionGoalForHeuristicRescue(...)`
    - `kHeuristicRescueMaxGoalCenterOffsetM = 0.02`
  - 新规则：
    - slow-direct rescue 只允许接近箱体中心轴的插入目标进入候选
    - `HardPlus_HoleEdgeOffset` 这种偏心目标默认排除

- 新 gate 的验证 run：
  - `20260321_093844_336`
  - 结果：
    - `Hard_HoleShallow`
      - 仍然会触发局部 rescue
      - `rescue_helped`
      - `delta_mean_ms = -284.0`
      - `delta_p75_ms = -420.0`
      - `delta_attempt_rate = +0.33`
    - `HardPlus_HoleEdgeOffset`
      - guide trigger 已压到 `0`
      - `no_clear_change`
    - `Extreme_HoleDeep`
      - guide trigger 仍为 `0`
      - `no_clear_change`

- 但这一步不能过早下正结论，所以随后又补了一轮 `10 repeats` 验证：
  - baseline：
    - `20260321_094155_219`
  - centered rescue：
    - `20260321_094233_418`
  - 更可靠的场景级结论应以这一轮为准：
    - `Hard_HoleShallow`
      - `rescue_hurt`
      - `delta_mean_ms = +229.6`
      - `delta_p75_ms = +60.2`
      - `delta_attempt_rate = +0.30`
    - `HardPlus_HoleEdgeOffset`
      - `no_clear_change`
      - `delta_mean_ms = -104.2`
      - `delta_p75_ms = -3.0`
      - `delta_attempt_rate = 0.00`
  - 这说明：
    - `centered goal gate` 确实更擅长压掉 `HardPlus_HoleEdgeOffset` 这种错误触发
    - 但它还不足以证明 `Hard_HoleShallow` 上存在稳定正收益
    - 当前 centered rescue 仍然只能算：
      - 更干净的诊断版实验线
      - 不是稳定可保留的默认策略

- 这一步之后，heuristic rescue 主线结论应更新为：
  - 全局 slow-direct rescue 仍然不能写成默认通用策略
  - `centered shallow insertion` 仍然是唯一看起来有几何合理性的局部方向
  - 但当前还没有足够证据把它升级成稳定有效策略
  - `HardPlus_HoleEdgeOffset` 当前应作为：
    - rescue 禁区示例
  - `Extreme_HoleDeep` 当前继续保持：
    - 默认不放宽

- 另外还顺手修了接口一致性：
  - `moveToPoseImproved(...)` 已从旧的 `kGuideSlowDirectCostSlack` 放宽逻辑
  - 对齐到与 `planToPoseImproved(...)` 同一套保守 rescue gate
  - 这样 benchmark 路径与执行路径不再继续分叉

## 36. 当前版本 HeuristicGuided 的 canonical 诊断结论：稳定，但几乎完全是 direct-first

- `2026-03-21` 曾尝试直接重跑一轮 `simple 10-repeat + 全 planner canonical benchmark`
  - 命令：
    - `benchmark:=simple repeats:=10 planners:=RRTConnect,RRTstar,LBTRRT,FMT,BFMT,PRMstar,HeuristicGuided`
  - 结果：
    - 超过 `2h` 仍未完成
    - 中途再次遇到 `move_group` 段错误
  - 这一轮没有产出可用 canonical 主表
  - 因此当前更合理的工程策略固定为：
    - 经典 planner 的论文主表对比，继续沿用已经稳定写入主线的历史 canonical run
    - 当前代码版本需要补的新证据，优先用 `HeuristicGuided-only` 10-repeat + 新诊断列完成

- 目前论文主表仍沿用的稳定 canonical 对比基线：
  - `simple`
    - `test_results/benchmarks/simple/raw/20260311_124449_790_planner_comparison_simple_summary.csv`
  - `v2`
    - `test_results/benchmarks/v2/raw/20260317_142203_372_planner_comparison_v2_summary.csv`

- 随后已成功补跑当前代码版本下的 `HeuristicGuided-only` 全场景 `10 repeats`：
  - simple：
    - `20260321_115713_880`
    - summary：
      - `test_results/benchmarks/simple/raw/20260321_115713_880_planner_comparison_simple_summary.csv`
    - 结果：
      - 成功率 `100.0%`
      - 平均 `516.0 ms`
      - 中位 `208.0 ms`
      - `P75 = 1033.2 ms`
      - 预算命中 `0/60`
      - 平均 `guide候选数 = 28.5`
      - 平均 `guide尝试数 = 0.0`
      - `direct回退率 = 100%`
  - v2：
    - `20260321_115837_516`
    - summary：
      - `test_results/benchmarks/v2/raw/20260321_115837_516_planner_comparison_v2_summary.csv`
    - 结果：
      - 成功率 `100.0%`
      - 平均 `305.9 ms`
      - 中位 `67.0 ms`
      - `P75 = 354.8 ms`
      - 预算命中 `0/40`
      - 平均 `guide候选数 = 29.1`
      - 平均 `guide尝试数 = 0.0`
      - `direct回退率 = 100%`

- 为了避免后续每次手工看 csv，已新增 gate 行为分析脚本：
  - `scripts/benchmarks/analyze_heuristic_gate_activity.py`
  - 已写入：
    - `docs/COMMANDS.md`
  - 已加入安装：
    - `CMakeLists.txt`

- 用这个脚本对上面两轮新 run 的正式导出如下：
  - simple：
    - `test_results/exports/heuristic_gate_activity/20260321_115713_880/scene_summary.csv`
    - `test_results/exports/heuristic_gate_activity/20260321_115713_880/overall_summary.json`
  - v2：
    - `test_results/exports/heuristic_gate_activity/20260321_115837_516/scene_summary.csv`
    - `test_results/exports/heuristic_gate_activity/20260321_115837_516/overall_summary.json`

- 这一步之后，关于当前默认 `HeuristicGuided` 的工程事实要改写为：
  - 它依然是稳定方法：
    - 两套 benchmark 都保持 `100%` 成功率
    - 两套 benchmark 都保持 `0%` 预算命中
  - 但当前默认实现几乎完全表现为：
    - `direct-first`
    - `guide-never-fired`
  - 证据不是主观判断，而是：
    - `guide_attempt_rate = 0.00`
    - `direct_fallback_rate = 1.00`
    - 所有场景都由脚本判成：
      - `dormant_near_tie`
      - 或 `dormant_near_tie_long_tail`

- 更细一点的数学解释也要固定：
  - 在 simple 当前 run 中，按场景统计的 `mean_delta_h = top_guide_heuristic_cost - direct_cost`
    - `Easy_TopCenter`: `+0.0017`
    - `Medium_SideSurface`: `+0.0042`
    - `MediumPlus_RightUpperAngled`: `+0.0037`
    - `Hard_HoleShallow`: `+0.0029`
    - `HardPlus_HoleEdgeOffset`: `+0.0027`
    - `Extreme_HoleDeep`: `+0.0032`
  - 在 v2 当前 run 中：
    - `Easy_HoleCenter`: `+0.0024`
    - `Medium_HoleEdge`: `+0.0017`
    - `Hard_DeepInterior`: `+0.0010`
    - `Extreme_NarrowPassage`: `+0.0018`
  - 这说明：
    - 当前 top guide 往往不是“特别差”
    - 而是“和 direct 很接近，但略差一点”
    - 因而在当前保守 gate 下，guide route 被系统性压成 0 次尝试

- 所以当前 heuristic 主线的下一条工程结论应明确为：
  - 不要继续花时间在 `slow-direct rescue` 上
  - 当前最值得做的下一步是：
    - 直接优化 `buildGuideCandidates(...)`
    - 目标不是“放松 gate”
    - 而是让 top guide 从 `near-tie but slightly worse` 进化到：
      - `occasionally better than direct`
  - 换句话说：
    - 当前 heuristic 的主矛盾已经不是 trigger 规则
    - 而是候选分布本身还不够强

## 37. 2026-03-23 第一轮 B+ 主线对照：adaptive ellipsoid 已接通，但当前没有真正改变 guide 行为

- 已按当前 B+ 主线完成第一轮正式对照：
  - `simple baseline`
    - `test_results/benchmarks/simple/raw/20260323_101854_752_planner_comparison_simple_results.csv`
    - `test_results/benchmarks/simple/raw/20260323_101854_752_planner_comparison_simple_summary.csv`
  - `simple adaptive`
    - `test_results/benchmarks/simple/raw/20260323_102053_308_planner_comparison_simple_results.csv`
    - `test_results/benchmarks/simple/raw/20260323_102053_308_planner_comparison_simple_summary.csv`
  - `v2 baseline`
    - `test_results/benchmarks/v2/raw/20260323_102251_366_planner_comparison_v2_results.csv`
    - `test_results/benchmarks/v2/raw/20260323_102251_366_planner_comparison_v2_summary.csv`
  - `v2 adaptive`
    - `test_results/benchmarks/v2/raw/20260323_102402_685_planner_comparison_v2_results.csv`
    - `test_results/benchmarks/v2/raw/20260323_102402_685_planner_comparison_v2_summary.csv`

- 四轮 benchmark 都补做了 gate activity 分析：
  - `test_results/exports/heuristic_gate_activity/20260323_101854_752/`
  - `test_results/exports/heuristic_gate_activity/20260323_102053_308/`
  - `test_results/exports/heuristic_gate_activity/20260323_102251_366/`
  - `test_results/exports/heuristic_gate_activity/20260323_102402_685/`

- 这轮对照必须固定下来的核心事实：
  - `simple baseline`
    - 成功率 `100%`
    - 平均 `387.7 ms`
    - `P75 = 1023.0 ms`
    - `guide_attempt_rate = 0.00`
    - `direct_fallback_rate = 1.00`
  - `simple adaptive`
    - 成功率 `100%`
    - 平均 `449.9 ms`
    - `P75 = 1029.8 ms`
    - `guide_attempt_rate = 0.00`
    - `direct_fallback_rate = 1.00`
  - `v2 baseline`
    - 成功率 `100%`
    - 平均 `378.5 ms`
    - `P75 = 1023.2 ms`
    - `guide_attempt_rate = 0.00`
    - `direct_fallback_rate = 1.00`
  - `v2 adaptive`
    - 成功率 `100%`
    - 平均 `380.1 ms`
    - `P75 = 1024.2 ms`
    - `guide_attempt_rate = 0.00`
    - `direct_fallback_rate = 1.00`

- 因此这一步的正式工程结论应写死：
  - 当前 adaptive ellipsoid 开关虽然已经真实接入 benchmark 运行链路
  - 但它还没有触到当前主矛盾
  - 也就是：
    - 没有把 `guide_attempt_rate` 从 `0` 拉起来
    - 没有降低 `direct_fallback_rate`
    - 没有改善长尾
  - `simple` 上当前 adaptive 版本甚至使均值从 `387.7 ms` 上升到 `449.9 ms`
  - `v2` 上当前 adaptive 版本基本等价于 baseline

- 对这条结果的正确解释不是：
  - `路线 B 已失败`
  - 而是：
    - 当前实现仍然太弱
    - 仅靠按 difficulty 调整椭球尺度 / 偏置，还不足以把 top guide 从 `near-tie but slightly worse` 推到 `occasionally better than direct`

- 所以下一条执行顺序要进一步收紧为：
  1. 不再重复当前这一版 `baseline vs adaptive` 口径
  2. 下一轮 B+ 改动必须直接增强 candidate distribution，而不是只改一个椭球大小
  3. 优先考虑一起联动：
     - `semi_major / minor_scale`
     - `target_bias`
     - `guide_sample_count`
     - `top-seed local refinement` 配额
     - `hard subset` 定向增密
  4. 下一轮是否值得继续，唯一硬标准仍然是：
     - `guide_attempt_rate` 必须真正上升
     - `direct_fallback_rate` 必须真正下降
     - 不能只看均值小波动

- 额外记录一个与方法无关但会反复出现的工程现象：
  - 这四轮 launch 在 benchmark 节点结束后，`move_group` 都会在 shutdown 阶段再次出现段错误
  - 当前崩溃发生在 benchmark 正常写盘之后
  - 因此暂不影响本轮结果可信性
  - 但如果后续要做批量自动化重复实验，需要单独处理这个 launch 收尾稳定性问题

## 38. 2026-03-23：B+ 第二轮实现已从“纯参数改动”推进到“真实 active guidance”

- 这轮代码改动已经不再只是按 difficulty 改一个椭球比例，而是同时改了三层：
  - `candidate distribution`
  - `difficulty-adaptive ranking bonus`
  - `hard-scene guide-first trigger`

- 当前核心实现位于：
  - `src/core/cr5_robot.cpp`

- 本轮新增的关键行为：
  - 困难场景下会自动放大 `guide candidate` 预算，并增加 refinement 配额
  - 椭球采样从单一分布改成了混合采样：
    - baseline-like 主分布
    - hard-scene flank band
    - late-approach band
  - `ranking_score` 中加入了 difficulty-conditioned 几何奖励：
    - clearance
    - axial progress
    - moderate lateral offset
  - `planToPoseImproved(...)` 在 hard scene 中新增有限的 `guide-first` 预尝试：
    - 仅对 near-tie 候选开放
    - 每段使用更短的 attempt cap
    - 失败后再回退 direct

- 这轮最重要的结论是：
  - `adaptive` 不再是“开关接好了但行为不变”
  - 它已经能在困难场景里真正触发 guide route

- 小规模验证结果已经证明这一点：

  - `simple hard subset`
    - 结果文件：
      - `test_results/benchmarks/simple/raw/20260323_104007_115_planner_comparison_simple_results.csv`
      - `test_results/benchmarks/simple/raw/20260323_104007_115_planner_comparison_simple_summary.csv`
    - gate 分析：
      - `test_results/exports/heuristic_gate_activity/20260323_104007_115/`
    - 结论：
      - overall `guide_attempt_rate = 0.50`
      - overall `direct_fallback_rate = 0.50`
      - `Hard_HoleShallow`
        - `guide_attempts = 1`
        - `used_direct = false`
        - `wall_time = 137 ms`
      - `Extreme_HoleDeep`
        - `guide_attempts = 1`
        - `used_direct = false`
        - `wall_time = 102 ms`
      - `HardPlus_HoleEdgeOffset`
        - top guide 仍明显差于 direct
        - 目前保持 direct fallback
      - `Medium_SideSurface`
        - 目前仍属于 dormant near-tie

  - `v2 hard subset`
    - 结果文件：
      - `test_results/benchmarks/v2/raw/20260323_104138_145_planner_comparison_v2_results.csv`
      - `test_results/benchmarks/v2/raw/20260323_104138_145_planner_comparison_v2_summary.csv`
    - gate 分析：
      - `test_results/exports/heuristic_gate_activity/20260323_104138_145/`
    - 结论：
      - overall `guide_attempt_rate = 0.50`
      - overall `direct_fallback_rate = 0.50`
      - `Hard_DeepInterior`
        - `guide_attempts = 1`
        - `used_direct = false`
        - `wall_time = 424 ms`
      - `Extreme_NarrowPassage`
        - top guide 仍明显差于 direct
        - 目前保持 direct fallback

- 所以当前主线判断需要更新：
  - 之前那版结论是：
    - adaptive 已接入，但仍完全 dormant
  - 现在应改成：
    - B+ 已经出现真实 active guidance
    - 但激活仍局部，只覆盖一部分 hard scene
    - 还没有形成“全场景稳定优于 direct”的结果

- 这意味着路线 B+ 现在仍值得继续，但接下来的重点要非常明确：
  1. 不是再证明“adaptive 会不会动”
  2. 而是继续扩大 active guidance 覆盖面
  3. 优先突破两类剩余场景：
     - `guide_clearly_worse_than_direct`
       - 例如 `HardPlus_HoleEdgeOffset`
       - 说明候选分布还没打到有效区域
     - `dormant_near_tie`
       - 例如 `Medium_SideSurface`
       - 说明 gate / bonus / sample budget 还可以继续推

- 当前最合理的下一步实验顺序：
  1. 继续只跑 hard subset，不回到全量 10-repeat
  2. 先针对 `HardPlus_HoleEdgeOffset` / `Extreme_NarrowPassage` 做 candidate distribution 定向增强
  3. 再复测 `guide_attempt_rate` 与 `direct_fallback_rate`
  4. 只有当 hard subset 稳定后，再回到完整 simple + v2 对照

## 39. 2026-03-23：环境锚点 + guide-first 让顽固 hard scene 全部转入 active guidance

- 本轮在 `B+` 上新增的不是单纯参数微调，而是：
  - environment-aware anchor guides
  - candidate-specific near-tie slack
  - 针对 insertion-like goal 的 recenter / hover bonus

- 关键代码位置：
  - `src/core/cr5_robot.cpp`
    - `generateEnvironmentAnchorGuideSamples(...)`
    - `evaluateGuideCandidate(...)`
    - `planToPoseImproved(...)`

- 本轮最重要的工程结论：
  - 之前仍未激活的两个顽固场景：
    - `HardPlus_HoleEdgeOffset`
    - `Extreme_NarrowPassage`
  - 现在都已在 clean sequential rerun 中稳定变成：
    - `guide_attempts = 1`
    - `used_direct = false`

- 单场景 clean rerun：

  - `HardPlus_HoleEdgeOffset`
    - results:
      - `test_results/benchmarks/simple/raw/20260323_105708_688_planner_comparison_simple_results.csv`
    - gate:
      - `test_results/exports/heuristic_gate_activity/20260323_105708_688/`
    - 结论：
      - `guide_attempt_rate = 1.00`
      - `direct_fallback_rate = 0.00`
      - `wall_time = 126 ms`
      - `top guide cost_delta_to_direct = 0.0006`

  - `Extreme_NarrowPassage`
    - results:
      - `test_results/benchmarks/v2/raw/20260323_105736_402_planner_comparison_v2_results.csv`
    - gate:
      - `test_results/exports/heuristic_gate_activity/20260323_105736_402/`
    - 结论：
      - `guide_attempt_rate = 1.00`
      - `direct_fallback_rate = 0.00`
      - `wall_time = 461 ms`
      - `top guide cost_delta_to_direct = 0.0011`

- 随后又做了 clean hard-subset 汇总验证：

  - `simple hard subset`
    - results:
      - `test_results/benchmarks/simple/raw/20260323_105837_925_planner_comparison_simple_results.csv`
    - gate:
      - `test_results/exports/heuristic_gate_activity/20260323_105837_925/`
    - overall:
      - `guide_attempt_rate = 1.00`
      - `direct_fallback_rate = 0.00`
    - per-scene:
      - `Hard_HoleShallow`: `422 ms`, `delta_h = 0.0004`
      - `HardPlus_HoleEdgeOffset`: `152 ms`, `delta_h = 0.0042`
      - `Extreme_HoleDeep`: `749 ms`, `delta_h = 0.0089`

  - `v2 hard subset`
    - results:
      - `test_results/benchmarks/v2/raw/20260323_105904_483_planner_comparison_v2_results.csv`
    - gate:
      - `test_results/exports/heuristic_gate_activity/20260323_105904_483/`
    - overall:
      - `guide_attempt_rate = 1.00`
      - `direct_fallback_rate = 0.00`
    - per-scene:
      - `Hard_DeepInterior`: `755 ms`, `delta_h = 0.0047`
      - `Extreme_NarrowPassage`: `429 ms`, `delta_h = 0.0011`

- 这使得当前 B+ 主线判断应再次更新：
  - 不是“只有部分 hard scene 被激活”
  - 而是：
    - clean hard subset 上已经全部转入 `active guidance`
    - 当前方法已经完成从 dormant guide 到 real guide routing 的关键跨越

- 这也意味着下一步应正式升级为：
  1. 回到 `simple + v2` 完整场景集
  2. 做 post-fix 的完整对照 rerun
  3. 重点看：
     - `guide_attempt_rate`
     - `direct_fallback_rate`
     - `wall_time` 长尾
     - 是否在 medium/easy scene 引入了不必要的过激 guide 触发

- 仍需保留一个工程注意事项：
  - `move_group` 在 launch 收尾阶段的段错误仍然存在
  - 但本轮所有 benchmark 结果都在写盘完成后才发生崩溃
  - 因此当前结果仍可用于方法判断

## 40. P0.2 HeuristicGuided 组件消融开关与轻量验证

- 已为论文 P0.2 补上 `HeuristicGuided` 组件级消融入口：
  - `MY_CR5_CONTROL_HEURISTIC_ABLATION_MODE=full`
  - `direct_only`
  - `fixed_guide`
  - `no_anchors`
  - `no_adaptive_difficulty`
  - `always_guide`
- 已扩展统一 runner：
  - `scripts/benchmarks/run_unified_formal_rerun.py`
  - 新增 `--ablation-modes paper`
  - ablation run 自动以 `HeuristicGuided` 为 planner，并在 manifest / README / log file 中记录 ablation mode
- 编译验证：
  - `colcon build --packages-select my_cr5_control --cmake-args -DCMAKE_BUILD_TYPE=RelWithDebInfo`
- 轻量结果归档：
  - smoke:
    - `paper_workspace/formal_results/q2_ablation_smoke_20260503_234524/`
  - simple representative:
    - `paper_workspace/formal_results/q2_ablation_light_20260503_234657/`
  - v2 representative:
    - `paper_workspace/formal_results/q2_ablation_light_v2_complete_20260503_235305/`
  - combined analysis:
    - `paper_workspace/formal_results/q2_ablation_light_P0_2_ANALYSIS.md`
- 当前结论：
  - 这是 P0.2 的工具链与轻量证据，不是最终主表。
  - `direct_only` 明确表现为负对照。
  - `no_adaptive_difficulty` 在 simple/v2 代表场景均慢于 `full`，支持 difficulty-adaptive 的论文主张。
  - `always_guide` 可验证 selective activation 的 tradeoff。
  - `no_anchors` / `fixed_guide` 需要全场景正式消融后再写强结论。

## 41. P0.2 正式全场景消融完成

- 正式 session：
  - main:
    - `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/`
  - direct-only completion:
    - `paper_workspace/formal_results/q2_ablation_formal_direct_only_completion_20260511_111548/`
  - analysis:
    - `paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md`
- 配置：
  - `simple + v2/WS119`
  - all canonical scenes
  - `30` repeats
  - modes:
    - `full`
    - `direct_only`
    - `fixed_guide`
    - `no_anchors`
    - `no_adaptive_difficulty`
    - `always_guide`
- `direct_only` 在主 session 中因 `300s` unit timeout 中止：
  - simple 到第 `13/30` 轮
  - v2 到第 `19/30` 轮
  - 后续用 `--unit-timeout 1200` 单独补跑完成
- 核心结论：
  - `direct_only` 是有效负对照，尤其 simple 出现 `27.8%` budget-hit 和明显 P75 long tail。
  - `no_adaptive_difficulty` / `fixed_guide` 的 P75 在 simple/v2 都明显差于 `full`，支持 difficulty-adaptive 和 adaptive guide shaping 的长尾抑制主张。
  - `always_guide` 几乎消除 direct fallback，但中位数变差，支持 selective activation 的必要性。
  - `no_anchors` 结果混合；论文中不能声称 anchors 普适正贡献，只能写成 scene-dependent environment-aware component。
- P0.2 状态：
  - 已从 `IN_PROGRESS` 更新为 `EVIDENCE_READY`。
