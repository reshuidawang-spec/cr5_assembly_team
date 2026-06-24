# Project Layout

## 1. 目标

这份文档用于说明当前工程的目录分工，解决两个问题：

1. 第一次进入仓库时，知道“先看哪里”
2. 后续新增代码、数据和文档时，知道“该放哪里”

当前工程已经进入“代码 + benchmark + 论文材料并行维护”的状态，因此目录结构需要按用途理解，而不是只按文件类型理解。

## 2. 当前推荐阅读顺序

第一次进入仓库时，建议固定按下面顺序阅读：

1. [README.md](../README.md)
2. [docs/README.md](./README.md)
3. [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
4. [docs/COMMANDS.md](./COMMANDS.md)
5. [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
6. [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)

## 3. 顶层目录说明

- `src/`
  - C++ 源码主目录。
  - `src/core/`：核心机器人封装与规划逻辑。
  - `src/operations/`：基础操作、测试和示教节点。
  - `src/gui/`：GUI 相关代码。
  - `src/benchmarks/`：benchmark 与随机任务采集节点。

- `include/`
  - 公共头文件。

- `launch/`
  - `ros2 launch` 入口。

- `scripts/`
  - Python 工具链。
  - `scripts/benchmarks/`：benchmark 采集、导出、绘图、检查。
  - `scripts/datasets/`：random dataset 分析与绘图。
  - `scripts/models/`：模型训练、评估与导出。
  - `scripts/maintenance/`：manifest 与历史数据维护。

- `docs/`
  - 常规工程文档主目录。
  - `docs/guides/`：长期有效的使用说明。
  - `docs/analysis/`：实验分析报告。
  - `docs/roadmap/`：论文路线与写作骨架。

- `test_results/`
  - 原始结果、导出表、模型、日志和 manifest 的完整数据仓。

- `paper_artifacts/`
  - 面向论文主表、主图和正式引用的归档材料。

- `PROJECT_MEMORY.md`
  - 当前研究主线、冻结结论和维护约定。

## 4. 文档体系

当前工程文档按下面 4 层理解：

### 4.1 根入口

- [README.md](../README.md)
- [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)

### 4.2 普通工程文档

- [docs/README.md](./README.md)
- [docs/COMMANDS.md](./COMMANDS.md)
- [docs/guides/README.md](./guides/README.md)
- [docs/analysis/README.md](./analysis/README.md)
- [docs/roadmap/README.md](./roadmap/README.md)

### 4.3 原始数据文档

- [test_results/README.md](../test_results/README.md)

### 4.4 论文正式归档文档

- [paper_artifacts/README.md](../paper_artifacts/README.md)
- [paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../paper_artifacts/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
- [paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md](../paper_artifacts/q2_ablation_formal_20260511_104104/ANALYSIS.md)

## 5. 后续新增文件放哪里

新增长期指南：

- 放到 `docs/guides/`

新增实验分析：

- 放到 `docs/analysis/`

新增论文路线、章节骨架、写作母版：

- 放到 `docs/roadmap/`

新增原始结果、导出表、模型和日志：

- 放到 `test_results/`

新增论文可直接引用的归档表格和图表输入：

- 放到 `paper_artifacts/`

新增命令工作流：

- 同步更新 `docs/COMMANDS.md`

## 6. 当前根目录保留项

根目录应主要承载：

- 工程入口文件
- 构建文件
- 核心源码目录
- 文档入口
- 数据目录

`build/`、`install/`、`log/` 这类目录属于生成产物，不应作为主要阅读入口。

## 7. `test_results/` 与 `paper_artifacts/` 的区别

这两个目录很容易混淆，当前必须明确：

- `test_results/`
  - 完整原始数据仓
  - 适合追溯、复算、训练和导出
- `paper_artifacts/`
  - 论文正式材料的归档副本
  - 适合主表、主图和结果解释

简单说：

- 查完整历史，去 `test_results/`
- 写论文主表，先去 `paper_artifacts/`

## 8. 维护规则

1. 新增可执行命令时，同步更新 [docs/COMMANDS.md](./COMMANDS.md)
2. 新增正式可用数据时，在 `paper_workspace/formal_results/` 下建立 README 或 ANALYSIS 入口
3. 新增重要研究结论时，同步更新 [PROJECT_MEMORY.md](../PROJECT_MEMORY.md)
4. 新增 paper-facing 归档时，同步更新 [paper_artifacts/README.md](../paper_artifacts/README.md)
5. 不再把新的说明性文档直接堆到仓库根目录
