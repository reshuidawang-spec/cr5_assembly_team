# R2 PCB 正式执行器

R2 PCB 安装已经接入团队既有接口：

```text
RobotExecutor implements IRobotExecutor
execute_task(Task) -> TaskResult
R2_PCB_PLACED -> R2
```

## 执行顺序

重新加载并保持仓库中的保存场景为停止状态：

```text
scenes/five_cr5a_cell.ttt
```

然后从仓库根目录依次执行：

```bash
cd /你的克隆目录/cr5_assembly_team
python3 robot_control/run_r1_task.py R1_BOX_PLACED
python3 robot_control/run_r2_task.py R2_PCB_PLACED
```

两条命令之间不要停止仿真或重新加载场景。R2 要求使用 R1 实际放到装配位
的 `Box_Blank`，并要求 R1 已退出装配共享区、停在
`R1_TERMINAL_PICK_APP`。R2 成功安装实际 `PCB_Supply` 后返回零位，仿真
继续运行，可供后续 R3 或 R1 端子任务接续。

## Python 接口

```python
from interfaces.types import Task
from robot_control.robot_executor import RobotExecutor

task = Task(
    task_id="T-R2-PCB",
    order_id="ORDER-1",
    product_type="A",
    process="assemble",
    target_area="assembly_area",
    target_point="R2_PCB_PLACED",
    available_robots=["R2"],
)
result = RobotExecutor().execute_task(task)
```

动作名可以放在 `Task.target_point`、`process` 或 `task_id`，推荐使用
`target_point`。错误机械臂分配、未知动作或任何前置条件失败都会返回
`TaskResult.status == "failed"`。

## 前置检查与安全边界

- 场景文件名、大小和 SHA-256 必须匹配已经验证的 R1/R2 场景。
- 四个 Git `R2_PCB_*_APP/TCP` 的位置和姿态必须保持不变。
- R1 必须位于箱体任务末态，R2 必须位于六关节零位。
- 实际箱体必须位于 R1 装配位置，实际 PCB 必须位于 R2 私有供料位置，
  且两个工件都必须由 `/FiveCR5A_Cell/Parts` 持有。
- 运行时检查 R2/PCB 与环境和 R1 的碰撞、R2 自碰撞、PCB 与 R2 碰撞，
  并执行 R2 隐形工作区墙检查。
- 同一 `RobotExecutor` 实例中的 R1 与 R2 共用同一个
  `assembly_shared` 互斥锁；两个 CLI 命令必须严格顺序执行。
- 开放空间峰值为 `50 deg/s`，精细下降上限为 `24 deg/s`，轨迹使用
  minimum-jerk 启停。

当前保存场景没有 R2 吸盘几何。正式动作仍是经过碰撞验证的视觉吸附：
运行时创建沿 Link6 工具轴前伸 `100 mm` 的虚拟 TCP，姿态为
`(195,0,90)` 度，PCB 视觉偏移为 `52 mm`。这不代表物理吸盘或真实负载
抓取已经验收，也不会修改保存场景或 Git target。
