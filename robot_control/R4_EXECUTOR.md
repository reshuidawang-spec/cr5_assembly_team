# R4 螺钉锁付正式执行器

R4 螺钉锁付已经接入团队既有接口：

```text
RobotExecutor implements IRobotExecutor
execute_task(Task) -> TaskResult
R4_SCREW_DONE -> R4
```

## 当前验收边界

保存场景没有 R4 螺丝刀模型。经用户确认，正式动作仅在运行时创建一把
`100 mm` 可视螺丝刀，使用 `(180,0,-135)` 度竖直姿态。Git 定义的
`R4_SCREW_APP/TCP/PRESS` 位置和 `{0,0,0}` 姿态保持不变。

该动作表示 APP 停留、下降到 TCP、下压到 PRESS、可见旋转、撤回和 R4
回零。它通过环境、自碰撞、工具到机械臂和 R4 工作区检查，但不代表真实
扭矩、接触力或螺钉物理锁付验收。

## 命令行使用

打开以下保存场景并保持停止态：

```text
/home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
```

然后从仓库根目录执行：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/run_r4_task.py R4_SCREW_DONE
```

R4 必须位于六关节零位，检测产品必须位于
`(0.15,0.05,0.216)` 且由 `/FiveCR5A_Cell/Parts` 持有。运行时工具、临时
target 和命令脚本会在成功或失败后删除，不会写入 `.ttt`。

## Python 接口

```python
from interfaces.types import Task
from robot_control.robot_executor import RobotExecutor

task = Task(
    task_id="T-R4-SCREW",
    order_id="ORDER-1",
    product_type="A",
    process="screw",
    target_area="inspection_screw_area",
    target_point="R4_SCREW_DONE",
    available_robots=["R4"],
)
result = RobotExecutor().execute_task(task)
```

错误机械臂分配、场景指纹变化、target 变化、错误关节/产品初态、碰撞、
工作区越界或工具清理失败都会返回 `TaskResult.status == "failed"`。

## 重复验收

用户确认运行时 `100 mm` 可视螺丝刀、`(180,0,-135)` 度竖直姿态、下压、
速度和两圈可见旋转效果可接受。前两次正式执行分别为 `22.9 s` 和 `21.7 s`。

剩余 8 次使用可复现运行器执行：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 robot_control/repeat_r4_acceptance.py \
  --runs 8 \
  --prior-successes 2 \
  --output data/logs/r4_repeat_acceptance_2026-07-18.json
```

运行器每次先停止并重新加载保存场景，核对指纹、R4 六关节零位、检测产品
位置/父节点和 Runtime 清理，再调用正式 `R4_SCREW_DONE -> R4`。每次完成后
独立检查仿真时间、回零误差、Runtime 对象、环境/自碰撞及检测产品状态。

2026-07-18 的结构化日志记录：

```text
prior successes           = 2
new runs                  = 8/8 finished
total acceptance          = 10/10
new-run wall time range   = 21.187-22.415 s
new-run mean/median       = 21.939/22.027 s
10-run mean               = approximately 22.011 s
R4 max home error         = 0.000632 deg
runtime tool/script       = none, every run
R4 environment collision = false, every run
R4 self collision        = false, every run
simulation time           = continues advancing, every run
postflight failures       = none
```

检测产品保持在 `(0.15,0.05,0.216)`、父节点为 `/Parts`，30 个产品形状
可见。正式代码的停止态 preflight 另覆盖 364 个路径状态和 33 个旋转状态。
R4 接入时测试为 33 项；加入新场景审计后为 38 项。当前五臂协同版
完整测试为 45 项，全部通过。
R4 已完成正式视觉重复验收 `10/10`；该结果仍不表示物理扭矩、接触力或
真实螺钉锁紧验收。
