# 调度算法测试模块

本目录只说明本次新增的动态调度测试功能。原系统 APP、场景文件、Lua 场景脚本和机械臂运动规划保持原仓库逻辑。

## 1. 测试入口

动态订单输入窗口：

```bash
python scripts/dynamic_order_window.py
```

CoppeliaSim 演示脚本：

```bash
python scripts/run_coppelia_order_demo.py
```

Ubuntu 辅助脚本：

```bash
bash scripts/start_coppelia_ubuntu.sh
bash scripts/run_dynamic_order_window_ubuntu.sh
```

## 2. 本次涉及的代码

- `scheduler/dynamic_order_sequence.py`
- `scheduler/task_generator.py`
- `scheduler/experiment.py`
- `scheduler/scheduler.py`
- `configs/product_types.yaml`
- `configs/scheduler.yaml`
- `configs/assembly_components.yaml`
- `scripts/dynamic_order_window.py`
- `scripts/run_coppelia_order_demo.py`
- `tests/test_scheduler_v2.py`

## 3. 不修改的内容

- `app/main_app.py`
- `scenes/`
- Lua 场景脚本
- 机械臂运动规划
- 原仓库已有说明文档

## 4. 相关说明

- Ubuntu 运行步骤：[RUN_UBUNTU.md](RUN_UBUNTU.md)
- 调度算法思路：[SCHEDULING_LOGIC.md](SCHEDULING_LOGIC.md)
- 窗口与仿真接口：[INTERFACE.md](INTERFACE.md)
- 示例订单：[demo_order_example.json](demo_order_example.json)
