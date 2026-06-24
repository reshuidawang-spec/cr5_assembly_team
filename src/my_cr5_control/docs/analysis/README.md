# Analysis Index

## 1. 目录作用

`docs/analysis/` 保存已经完成的实验分析报告。  
这些文档主要回答“某次 benchmark / ablation / 数据采集得出了什么结论”。

## 2. 当前最重要的分析

### 当前论文主线必须优先看的

1. [q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md](../../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
   - 当前论文主表的统一 stable-core formal rerun 证据
2. [q2_ablation_formal_20260511_104104/ANALYSIS.md](../../paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md)
   - 当前论文方法贡献拆分的 formal ablation 证据
3. [HEURISTIC_GUIDED_POSTFIX_FORMAL_RERUN_20260324_ANALYSIS.md](./HEURISTIC_GUIDED_POSTFIX_FORMAL_RERUN_20260324_ANALYSIS.md)
   - 历史 post-fix rerun 结论，已被 2026-05 formal evidence 覆盖
4. [SIMPLE_BENCHMARK_20260311_101849_ANALYSIS.md](./SIMPLE_BENCHMARK_20260311_101849_ANALYSIS.md)
   - `simple` canonical benchmark 分析
5. [V2_BENCHMARK_20260317_142203_372_ANALYSIS.md](./V2_BENCHMARK_20260317_142203_372_ANALYSIS.md)
   - `v2` canonical benchmark 分析

## 3. 文档分类

### 3.1 当前主线正式分析

- [2026-05-03 stable-core unified formal rerun](../../paper_workspace/formal_results/q2_unified_formal_stable_core_20260503_135832/ANALYSIS.md)
- [2026-05-11 formal ablation](../../paper_workspace/formal_results/q2_ablation_formal_20260511_104104/ANALYSIS.md)
- [2026-03-24 post-fix formal rerun 历史记录](./HEURISTIC_GUIDED_POSTFIX_FORMAL_RERUN_20260324_ANALYSIS.md)

### 3.2 benchmark 历史分析

- [SIMPLE_BENCHMARK_20260311_101849_ANALYSIS.md](./SIMPLE_BENCHMARK_20260311_101849_ANALYSIS.md)
- [V2_BENCHMARK_20260311_120357_ANALYSIS.md](./V2_BENCHMARK_20260311_120357_ANALYSIS.md)
- [V2_BENCHMARK_20260317_142203_372_ANALYSIS.md](./V2_BENCHMARK_20260317_142203_372_ANALYSIS.md)

### 3.3 random dataset 历史分析

- [SIMPLE_RANDOM_DATASET_20260311_140426_ANALYSIS.md](../../project_archive/docs/analysis/SIMPLE_RANDOM_DATASET_20260311_140426_ANALYSIS.md)
- [SIMPLE_RANDOM_DATASET_20260311_142950_774_ANALYSIS.md](../../project_archive/docs/analysis/SIMPLE_RANDOM_DATASET_20260311_142950_774_ANALYSIS.md)

### 3.4 learned guidance / ablation 历史分析

- [LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_160940_152_ANALYSIS.md](../../project_archive/docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_160940_152_ANALYSIS.md)
- [LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md](../../project_archive/docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260317_171817_260_171943_687_ANALYSIS.md)
- [LEARNED_GUIDANCE_SIMPLE_ABLATION_20260318_085048_146_085205_318_ANALYSIS.md](../../project_archive/docs/analysis/LEARNED_GUIDANCE_SIMPLE_ABLATION_20260318_085048_146_085205_318_ANALYSIS.md)

### 3.5 其他实验记录

- [TEST_RRTSTAR_CONNECT_REPRODUCTION.md](../../project_archive/docs/analysis/TEST_RRTSTAR_CONNECT_REPRODUCTION.md)

## 4. 使用约定

1. 当前写论文时，优先使用 2026-05 formal evidence
2. 老分析不删除，但默认视为历史参考
3. 如果某份分析已经被正式归档到 `paper_workspace/formal_results/`，论文引用时优先看归档副本的说明
