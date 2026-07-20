# 五机械臂基础协同运行

`FiveArmCoordinator` 使用一个长期 `SimBridge` 连接和同一个 `RobotExecutor`
完成一件产品的七段基础视觉流程：

```text
R1_BOX_PLACED
-> R2_PCB_PLACED
-> R3_MODULE_PLACED
-> R1_TERMINAL_PLACED
-> R3_PRODUCT_TO_INSPECTION
-> R4_SCREW_DONE
-> R5_SORT_GOOD_DONE / R5_SORT_DEFECT_DONE
```

任务之间不停止仿真、不重载场景、不重建主执行连接。R1/R2/R3 共用装配区锁，
R3/R4/R5 共用检测区锁。`FiveArmCoordinator` 保留为固定顺序执行和路径回归
入口；正式软件集成入口已经接入 `scheduler.Scheduler`、
`OrderParser` 和 `IntegratedRobotExecutor`。

## 运行前提

1. 使用仓库中的 `scenes/five_cr5a_cell.ttt`。
2. 打开场景后保持仿真停止，不要保存上一次运行末态。
3. CoppeliaSim ZMQ Remote API 监听 `127.0.0.1:23000`。
4. 场景文件指纹必须与已验证基线一致；所有 Git target 必须未变。

推荐启动 CoppeliaSim：

```bash
source /opt/ros/humble/setup.bash
/home/vboxuser/CoppeliaSim/coppeliaSim.sh \
  /path/to/cr5_assembly_team/scenes/five_cr5a_cell.ttt
```

## 一键运行

Good 产品：

```bash
cd /path/to/cr5_assembly_team
python3 robot_control/run_five_arm_cycle.py \
  --quality good \
  --output data/logs/five_arm_good_cycle.json
```

Defect 产品：

```bash
python3 robot_control/run_five_arm_cycle.py \
  --quality defect \
  --output data/logs/five_arm_defect_cycle.json
```

输出 JSON 记录每个 Task 的 `TaskResult`、墙钟时间、仿真时间和相邻任务交接
空档。任一任务失败后立即停止后续工艺，不会把未执行动作报告为成功。

## 真实调度与 GUI 入口

当前场景只有一套 A 型工件且没有自动补料/场景复位，因此 REAL 模式保守限制为
一个 A 型、数量 1 的订单。Good 和 Defect 由 `Order.expected_quality=OK/NG`
选择，不生成另一条未执行分支。

```bash
python3 run_demo.py --real

python3 run_demo.py \
  --real \
  --headless \
  --quality good \
  --output data/logs/scheduled_good.json
```

默认运动参数保持 `50 deg/s`、APP `0.8 s`。用户已经取消 defect
`60 deg/s`、APP `0.4 s` 加速回归，本次集成没有改默认值。

`IntegratedRobotExecutor` 在 R4 前发送 `camera_good/camera_defect`，并从独立
ZMQ 观察连接读取对应机械臂六关节。关节相对任务下发基线变化达到 `0.02 deg`
时，`TaskResult.metrics` 记录：

```text
motion_timing.dispatch_to_first_motion_wall_s
motion_timing.task_call_to_first_motion_wall_s
motion_timing.first_motion_simulation_time_s
handoff_to_first_motion_simulation_s
task_end_simulation_time_s
```

原 `handoff_delay_simulation_s` 只表示相邻 Task 函数调用时间，不能继续当成
真实机械臂交接延迟。

## 完整实跑证据

2026-07-20 从干净停止场景通过正式 Scheduler/GUI 执行适配层运行默认参数
Good：

```text
log                       = data/logs/scheduled_good_first_motion_2026-07-20.json
command                   = run_demo.py --real --headless --quality good
command speed             = 50 deg/s
APP hold                  = 0.8 s
status                    = finished
seven scheduled tasks     = 7/7 finished
wall duration             = 283.763 s
monitor-armed-to-motion   = 1.733-3.348 s
scheduler-to-motion wall  = 2.182-3.800 s
first R1 command-to-motion= 3.529 s
real simulation handoff   = 1.300-2.300 s
R5 release yaw            = -89.998160 deg
R5 track parallel error   = 0.001840 deg
R5 grasp transform error  = 1.11e-16
max postflight home error = 0.004661 deg (R5)
all runtime objects       = none
R1-R5 final collision     = none
```

这次运行证明调度、相机分支、GUI 使用的执行适配器和七个真实动作已经贯通；
也证明当前在线 preflight/IK 准备仍使真实交接超过 `0.5 s` 仿真时间目标。
后续应在 READY 阶段预加载路径和监测连接，不能通过暂停仿真或改写指标隐藏空档。

2026-07-20 从干净停止场景运行新 yaw 对齐 Good 路线：

```text
log                       = data/logs/five_arm_good_yaw_aligned_60_2026-07-20.json
command speed             = 60 deg/s
APP hold                  = 0.4 s
status                    = finished
seven formal tasks        = 7/7 finished
wall duration             = 230.954 s
R5 wall duration          = 45.936 s
R5 release yaw            = -89.998158 deg
R5 track parallel error   = 0.001842 deg
R5 grasp transform error  = 1.67e-16
max postflight home error = 0.008217 deg (R5)
all runtime objects       = none
R1-R5 final collision     = none
```

以下 2026-07-19 日志保留为历史路线和 defect 基线。

`data/logs/five_arm_good_cycle_2026-07-19.json` 记录了从干净停止场景开始的
一次完整成功运行：

```text
status                     = finished
seven formal tasks         = 7/7 finished
wall duration              = 319.515 s
Task-call gap              = 0.0 s for normal robot handoffs
camera transition gap      = 0.05 s before R4
real first-motion handoff  = not measured in this historical log
all robot states           = idle/home
all runtime objects        = none
R1-R5 final collision      = none
```

`data/logs/five_arm_defect_cycle_2026-07-19.json` 记录了另一次从干净场景
开始的 defect 完整运行：

```text
status                  = finished
seven formal tasks      = 7/7 finished
wall duration           = 296.252 s
handoff simulation gap  = 0.0-0.05 s
all robot states        = idle/home
all runtime objects     = none
defect product          = carried by R5 to the defect conveyor
```

本流程是视觉 attach/吸附/工具动作验收，不是物理抓取、物理扭矩或
物理负载验收。R5 good 已改为固定产品到 TCP 变换的同步路线；其
`joint1=-121 deg` 预转、运行时 XY 偏移、精确目标带面接触和 yaw 对齐
证据见 `robot_control/R5_EXECUTOR.md`。
