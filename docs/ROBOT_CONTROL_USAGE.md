# CR5 五机械臂运动控制使用说明

> 3号成员交付 — 2026-07-20

## 文件总览

```
robot_control/
  r1_motion.py  r2_motion.py  r3_motion.py  r4_motion.py  r5_motion.py  ← 各臂控制器
  robot_executor.py          ← IRobotExecutor 实现，Task→动作映射
  runtime_cartesian.py       ← 共享运行时：simIK、碰撞检查、SmoothRunner
  five_arm_coordinator.py    ← 五臂固定顺序协调器
  integrated_executor.py     ← 调度/GUI 适配层，含首动监测
  motion_timing.py           ← 独立 ZMQ 连接首动观察器
  run_r[1-5]_task.py         ← 单臂 CLI
  run_five_arm_cycle.py      ← 一键五臂 CLI
  repeat_r4_acceptance.py    ← R4 10/10 重复验收
  plans/r1_complete_cycle_plan.json  ← R1 验证计划缓存
  R[1-5]_EXECUTOR.md         ← 各臂执行器说明
  FIVE_ARM_COORDINATOR.md    ← 协调器说明

sim_bridge/
  coppelia_client.py         ← SimBridge(ISimBridge) ZMQ 五臂通信
  scene_objects.py           ← 场景对象路径/工作区/私有区定义
  audit_five_cr5a_scene.py   ← 只读场景审计工具

scenes/
  five_cr5a_cell.ttt         ← 五臂 CoppeliaSim 场景（2.8 MB）

configs/
  five_cr5a_scene_audit_baseline.json  ← 场景指纹与 target 基线

tests/                        ← 62 项自动化测试

scripts/
  run_five_arm_cycle.sh       ← 通用五臂固定流程入口
  run_real_scheduler.sh       ← 通用真实 Scheduler headless 入口
  run_robot_task.sh           ← 通用单 Task 入口
```

## 前提条件

1. **CoppeliaSim 4.9+** 已安装，ZMQ Remote API 可用
2. 场景文件 `scenes/five_cr5a_cell.ttt` 指纹正确：
   - 大小: `2875907` 字节
   - SHA-256: `0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657`
3. ZMQ 端口 `23000` 未被占用
4. Python 3 环境，依赖 `coppeliasim-zmqremoteapi-client`

## 团队统一启动方式

所有成员都使用同一套命令，不要把 `/home/vboxuser/桌面/...` 或
`/path/to/...` 当成固定路径写死。先进入自己电脑上的仓库根目录，也就是能看到
`README.md`、`robot_control/`、`scenes/` 的目录：

```bash
cd /你的克隆目录/cr5_assembly_team
```

从这里开始，场景路径统一写成 `$(pwd)/scenes/five_cr5a_cell.ttt`，这样每个成员
只需要进入自己的仓库根目录，不需要手写仓库绝对路径。

### 启动 CoppeliaSim

```bash
source /opt/ros/humble/setup.bash
/home/vboxuser/CoppeliaSim/coppeliaSim.sh \
  "$(pwd)/scenes/five_cr5a_cell.ttt"
```

这条命令会打开：

```text
<当前仓库>/scenes/five_cr5a_cell.ttt
```

因此仓库放在 `/home/vboxuser/桌面`、`~/code` 或其他路径都可以，成员不需要修改
场景路径。`$(pwd)` 会自动替换成当前仓库目录。

如果某台电脑的 CoppeliaSim 不在 `/home/vboxuser/CoppeliaSim`，只替换第一段
`coppeliaSim.sh` 的位置，场景参数仍保持不变：

```bash
/实际安装目录/coppeliaSim.sh \
  "$(pwd)/scenes/five_cr5a_cell.ttt"
```

打开场景后保持仿真停止，不要手动按 Start，不要保存上一次运行末态。运动脚本
会自己启动仿真、接管 stepping、执行后释放或停止。

### 运行五臂完整工艺

另开一个终端，仍然进入同一个仓库根目录：

```bash
cd /你的克隆目录/cr5_assembly_team

# Good 产品（默认 50 deg/s，APP 保持 0.8 s）
bash scripts/run_five_arm_cycle.sh good \
  --output data/logs/five_arm_good.json

# Defect 产品
bash scripts/run_five_arm_cycle.sh defect \
  --output data/logs/five_arm_defect.json
```

七段工艺：`R1 箱体 → R2 PCB → R3 模块 → R1 端子 → R3 产品转运 → R4 螺丝 → R5 分拣`

### 调度/GUI 模式

```bash
# 真实模式无界面
bash scripts/run_real_scheduler.sh good \
  --speed-deg-s 50 --hold-seconds 0.8 \
  --output data/logs/scheduled_good.json

# 真实模式带 GUI
python3 run_demo.py --real
```

## 单臂命令

每个 Task 对应一个正式 CLI。推荐使用统一入口，由脚本按 Task 名自动选择
`run_r1_task.py` 到 `run_r5_task.py`：

```bash
bash scripts/run_robot_task.sh R1_BOX_PLACED
bash scripts/run_robot_task.sh R1_TERMINAL_PLACED
bash scripts/run_robot_task.sh R2_PCB_PLACED
bash scripts/run_robot_task.sh R3_MODULE_PLACED
bash scripts/run_robot_task.sh R3_PRODUCT_TO_INSPECTION
bash scripts/run_robot_task.sh R4_SCREW_DONE
bash scripts/run_robot_task.sh R5_SORT_GOOD_DONE
bash scripts/run_robot_task.sh R5_SORT_DEFECT_DONE
```

**注意**：单臂命令不是都能从干净场景直接运行。`R1_TERMINAL_PLACED` 必须接在
`R1_BOX_PLACED` 成功后运行；`R2_PCB_PLACED` 必须接在真实 `R1_BOX_PLACED`
后；R3/R4/R5 也需要前序产品状态。单臂命令之间不得停止或重载仿真。

## 速度参数

| 参数 | 默认值 | 已验证范围 |
|------|--------|-----------|
| 开放空间速度 | 50 deg/s | Good: 50-60 deg/s；Defect: 50 deg/s |
| 精细下降上限 | 24 deg/s | 不可超过 |
| APP 保持时间 | 0.8 s | Good: 0.4-0.8 s；Defect: 0.8 s |

```bash
# Good 加速回归（已验证通过；Defect 60/0.4 尚未运行）
bash scripts/run_five_arm_cycle.sh good \
  --speed-deg-s 60 \
  --hold-seconds 0.4 \
  --output data/logs/five_arm_good_yaw_aligned_60.json
```

`run_demo.py --real` 也支持 `--speed-deg-s` 和 `--hold-seconds`，但当前
Scheduler/GUI 真实集成的结构化实跑证据使用默认 `50 deg/s`、APP `0.8 s`。
Defect `60 deg/s`、APP `0.4 s` 回归尚未完成；验收前不要把旧 Defect 日志当作
新速度验收。

## 输出 JSON

运行日志记录每个 Task 的：

| 字段 | 含义 |
|------|------|
| `status` | finished / failed |
| `wall_duration_s` | 墙钟耗时 |
| `simulation_duration_s` | 仿真时间 |
| `handoff_delay_simulation_s` | 函数调用级交接空档 |
| `motion_timing.task_call_to_first_motion_wall_s` | 命令下发到首动墙钟 |
| `handoff_to_first_motion_simulation_s` | **真实首动仿真交接延迟** |

## 集成接口

其他成员通过标准接口调用，不直接操作控制器：

```python
from robot_control.robot_executor import RobotExecutor
from sim_bridge.coppelia_client import SimBridge

bridge = SimBridge("127.0.0.1", 23000)
executor = RobotExecutor(sim_bridge=bridge, speed_deg_s=50.0, hold_seconds=0.8)

# 准备阶段（停止态预计算路径 + 进入 READY）
ready = executor.prepare_cycle(quality="good")

# 执行 Task
from interfaces.types import Task
task = Task(
    task_id="DEMO-01",
    order_id="ORDER-1",
    product_type="A",
    process="assemble",
    target_area="assembly_area",
    target_point="R1_BOX_PLACED",
    available_robots=["R1"],
)
result = executor.execute_task(task)
print(result.status, result.message)
```

## 场景审计

新场景到达时运行只读审计：

```bash
cd /你的克隆目录/cr5_assembly_team
python3 sim_bridge/audit_five_cr5a_scene.py \
  --scene scenes/five_cr5a_cell.ttt \
  --baseline configs/five_cr5a_scene_audit_baseline.json
```

## 测试

```bash
cd /你的克隆目录/cr5_assembly_team
python3 -m unittest discover -s tests -v
# Ran 62 tests — OK
```

## 常见启动错误

- 把 `/path/to/cr5_assembly_team` 原样复制执行：这是占位符，会找不到场景。
  现在统一先 `cd` 到自己的仓库根目录，再使用 `$(pwd)/scenes/five_cr5a_cell.ttt`。
- 命令里把 `scenes/` 和 `five_cr5a_cell.ttt` 分成两行或中间多了空格：文件路径会
  变成错误参数。建议按文档保留引号：`"$(pwd)/scenes/five_cr5a_cell.ttt"`。
- CoppeliaSim 没显示五臂场景：先确认命令中的第二个参数是当前仓库下的
  `scenes/five_cr5a_cell.ttt`，不是旧的 `compact_cell.ttt`。
- 找不到 CoppeliaSim：把命令开头的 `/home/vboxuser/CoppeliaSim/coppeliaSim.sh`
  替换成那台电脑真实的 `coppeliaSim.sh` 路径。
- 运动脚本连不上 `23000`：确认 CoppeliaSim 已打开场景，ZMQ Remote API 已启用，
  且没有另一个旧 CoppeliaSim 进程占用端口。

## 限制与边界

- **视觉验收**：夹持/吸附/螺丝刀均为运行时视觉表示，不等于物理抓取或扭矩验收
- **受保护 target**：Git HOME/APP/TCP 坐标和姿态不得修改
- **场景文件**：不得保存运行瞬态到 `five_cr5a_cell.ttt`
- **R2 暂停**：因 PCB 供料区与 R2 基座重叠等待场景组修复
- **Defect 加速**：60 deg/s defect 回归尚未完成
- **单产品**：当前只支持一件 A 型产品，无自动补料/场景复位
- **R4 不预定位**：screw APP 在检测区，与 R3 产品转运路径冲突

## 参考文档

- `docs/Five_CR5A_Cell_Control_Interface.md` — 五臂场景控制接口
- `docs/INTERFACES.md` — 团队接口契约
- `docs/SCENE_CHANGE_REQUEST_2026-07-18.md` — 场景组修改清单
- `robot_control/R[1-5]_EXECUTOR.md` — 各臂详细说明
- `robot_control/FIVE_ARM_COORDINATOR.md` — 协调器说明
