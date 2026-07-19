# 4号调度模块 GitHub 上传包说明

这个文件夹是精简后的上传包，适合合并到 GitHub 仓库对应目录中。

## 上传原则

不要用整个文件夹直接覆盖仓库，只把里面的同名目录合并到 GitHub 仓库对应位置。

本包只包含 4 号调度相关内容：

- 五机械臂调度配置；
- 装配元件与工步模型；
- 调度算法与实验脚本；
- 单元测试；
- 核心结果表；
- 可视化时间轴；
- 汇报说明文档。

## 主要内容

### 代码和配置

- `configs/`
- `interfaces/types.py`
- `scheduler/`
- `mock/`
- `scripts/`
- `tests/test_scheduler_v2.py`

### 说明文档

- `docs/ASSEMBLY_PROCESS_OPTIMIZATION.md`
- `docs/FIVE_CR5A_SCHEDULER_ALIGNMENT.md`
- `docs/FOUR_SCHEME_FAULT_COMPARISON.md`
- `docs/4号调度模块方案说明.md`

### 核心结果

- `data/assembly_process_v2/component_sequence.csv`
- `data/assembly_process_v2/step_timeline.csv`
- `data/assembly_process_v2/line_balance_summary.json`
- `data/assembly_process_v2/line_balance_recommendations.json`
- `data/scheme_fault_comparison/scenario_summary.csv`
- `data/scheme_fault_comparison/scheme_comparison.csv`
- `output/visualizations/assembly_step_timeline.html`

## 没有放进来的内容

没有放入大量详细调度过程表，例如 `schedules/` 下的 28 个 CSV。那些文件适合本地留档，不建议全部上传到 GitHub，避免仓库过重、过乱。

## 已验证

本地测试结果：

```text
python -m unittest tests.test_scheduler_v2
13/13 通过
```

四方案对比已生成：

```text
最基本串行
基础并行 FIFO
单一关键路径优先
综合优化调度
```

结论：综合优化调度能明显提前急单完成时间，并且比单一规则更适合五机械臂装配场景。
