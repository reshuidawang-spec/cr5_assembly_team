# R1 正式执行器

`RobotExecutor` 是团队接口 `IRobotExecutor` 的真实 CoppeliaSim 实现。
本文件说明已经验证的 R1 箱体和端子动作；R2 PCB 动作见
`robot_control/R2_EXECUTOR.md`。R3/R5 见各自执行文档；未知动作仍明确返回
`failed`。

## 前置条件

打开并重新加载保存场景：

```text
/home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
```

首次执行箱体或完整循环前，场景必须停止、R1 六关节为零，箱体和端子位于
供料位。不要保存运行后的工件或关节状态。

## 命令行使用

完整 R1 循环：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/run_r1_task.py R1_COMPLETE_CYCLE
```

按总调度顺序分成两个 Task：

```bash
python3 robot_control/run_r1_task.py R1_BOX_PLACED
# R2 PCB、R3 模块等中间任务使用同一个运行场景
python3 robot_control/run_r1_task.py R1_TERMINAL_PLACED
```

成功后执行器保留当前运行场景并释放 stepping 控制，使后续机械臂可以继续；
不能在两个分段 Task 之间停止仿真，否则 CoppeliaSim 会恢复初始场景。

## Python 接口

```python
from interfaces.types import Task
from robot_control.robot_executor import RobotExecutor

task = Task(
    task_id="T-R1-BOX",
    order_id="ORDER-1",
    product_type="A",
    process="assemble",
    target_area="assembly_area",
    target_point="R1_BOX_PLACED",
    available_robots=["R1"],
)
result = RobotExecutor().execute_task(task)
```

支持的标准动作是 `R1_BOX_PLACED`、`R1_TERMINAL_PLACED` 和便捷组合动作
`R1_COMPLETE_CYCLE`。动作名可放在 `Task.target_point`、`process` 或
`task_id`；推荐放在 `target_point`。

## 安全与状态

- 8 个 Git APP/TCP target 在执行前逐一核对，不会被修改。
- 场景文件 SHA-256 与计划不一致时拒绝执行，要求重新完整 preflight。
- 运行中检查机械臂/夹爪/工件与环境碰撞、自碰撞和 R1 工作区墙。
- 开放空间峰值 `50 deg/s`，精细下降不超过 `24 deg/s`，使用 minimum-jerk 启停。
- 装配共享区由互斥锁保护；私有供料区不需要共享锁。
- 成功返回 `finished`；碰撞、超时、错误初态、错误机械臂或未知任务返回 `failed`。
- 当前抓取是视觉 attach，不是物理摩擦夹持验收。

2026-07-17 实测：组合 Task 和箱体/端子分段 Task 均成功；最终回零最大误差
`0.001642 deg`，箱体位置 `(-1.150002,0.199900,0.215915)`，端子位置
`(-1.089629,0.129919,0.301536)`。
