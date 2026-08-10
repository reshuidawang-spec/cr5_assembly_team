# 调度算法测试模块

本目录只说明本次新增的动态调度测试功能。原系统 APP、场景文件、Lua 场景脚本和机械臂运动规划保持原仓库逻辑。

## 1. 测试入口

Ubuntu 下动态订单输入窗口：

```bash
bash scripts/run_dynamic_order_window_ubuntu.sh
```

单独启动 CoppeliaSim 场景：

```bash
bash scripts/start_coppelia_ubuntu.sh
```

当前演示脚本已经按最新 `docs/五臂协同.md` 对齐：调度算法仍负责订单排序与急单重排，仿真运动入口改为
`robot_control/coordinated_engine.py` → `scripts/coordinated_front.py` → `data/captured_paths/`
预录轨迹回放。也就是说，窗口里的每个产品单元按调度顺序触发一轮五臂协同装配，不再默认调用旧的
`robot_control/r1_motion.py ~ r5_motion.py` 串行控制器。

如需不经过窗口、直接运行演示脚本：

```bash
python3 scripts/run_coppelia_order_demo.py
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
- `robot_control/coordinated_engine.py`
- `robot_control/replay_controller.py`
- `scripts/coordinated_front.py`
- `data/captured_paths/`
- `tests/test_scheduler_v2.py`

## 3. 不修改的内容

- `app/main_app.py`
- `scenes/`
- Lua 场景脚本
- 机械臂运动规划
- 原仓库已有说明文档

## 4. 相关说明

- 最终演示使用说明：[FINAL_USAGE.md](FINAL_USAGE.md)
- Ubuntu 运行步骤：[RUN_UBUNTU.md](RUN_UBUNTU.md)
- 调度算法思路：[SCHEDULING_LOGIC.md](SCHEDULING_LOGIC.md)
- 窗口与仿真接口：[INTERFACE.md](INTERFACE.md)
- 示例订单：[demo_order_example.json](demo_order_example.json)
- A/B 换型插单流程：[AB_CHANGEOVER_FLOW.md](AB_CHANGEOVER_FLOW.md)
