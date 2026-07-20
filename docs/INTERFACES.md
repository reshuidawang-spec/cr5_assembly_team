# 接口规范

本文件用于统一场景、动作、调度、软件集成和数据看板之间的数据格式，避免各模块互相等待或最后无法整合。

核心原则：

- 场景同学负责点位和对象命名；
- 机械臂控制同学只接收标准任务并返回状态；
- 调度同学只生成任务，不直接写机械臂底层运动；
- 软件集成同学只通过接口调用各模块，不私自定义另一套格式；
- 未完成的真实模块先用 Mock 模块替代。

### 当前五臂场景的强制覆盖规则

本文件后面的 `P_FEED_01`、`/CompactCell` 和四臂示例是早期接口草案。
当前五臂实现以 `docs/Five_CR5A_Cell_Control_Interface.md` 为场景命名权威，
必须使用 `/FiveCR5A_Cell`、`/R1` 到 `/R5` 和既有 APP/TCP 名称。

软件模块入口保持不变：

```text
sim_bridge.coppelia_client.SimBridge implements ISimBridge
robot_control.robot_executor.RobotExecutor implements IRobotExecutor
execute_task(Task) -> TaskResult
get_robot_states() -> list[RobotState]
```

真实软件集成入口现为：

```text
scheduler.order_parser.OrderParser implements IOrderParser
scheduler.scheduler.Scheduler implements IScheduler
robot_control.integrated_executor.IntegratedRobotExecutor implements IRobotExecutor
run_demo.py --real [--headless] [--quality good|defect]
```

当前保存场景只能完成一个 A 型单件订单。REAL 模式会拒绝多订单、B/C 型、
`quantity != 1` 和运行中急单插入；这些限制是场景无补料/复位能力的真实边界，
不能用 Mock 成功代替。

当前真实执行器已经支持：

| `Task.target_point` / `process` / `task_id` | 机械臂 | 结果 |
|---|---|---|
| `R1_BOX_PLACED` | R1 | 箱体抓放后退出装配共享区，停在端子 APP |
| `R1_TERMINAL_PLACED` | R1 | 从箱体任务末态继续端子抓放并回零 |
| `R1_COMPLETE_CYCLE` | R1 | 上述两个动作连续执行 |
| `R2_PCB_PLACED` | R2 | 接续 R1 箱体任务，将实际 PCB 安装到实际箱体并回零 |
| `R3_MODULE_PLACED` | R3 | 接续 PCB 末态，安装实际模块并回零 |
| `R3_PRODUCT_TO_INSPECTION` | R3 | 将执行器产品模板转移到检测区并回零 |
| `R4_SCREW_DONE` | R4 | 使用运行时可视螺丝刀完成 APP/TCP/PRESS、旋转、撤回并回零 |
| `R5_SORT_GOOD_DONE` | R5 | 固定产品到 TCP，同步进入 good 带面、yaw 对齐并回零 |
| `R5_SORT_DEFECT_DONE` | R5 | 携视觉产品到 defect 带面并回零 |

未知任务和错误机械臂分配必须返回
`TaskResult.status == "failed"`，不得使用 sleep 或 Mock 假完成。
R1/R2/R3 的私有供料区应互不重叠；同一正式执行器实例调度它们进入装配区
时必须持有同一个装配共享区互斥锁。R3/R4/R5 进入检测/锁付区时必须使用
检测共享区互斥锁；当前正式 R3/R4/R5 控制器已经接入该锁。

固定顺序的五臂基础协同入口为：

```text
robot_control.five_arm_coordinator.FiveArmCoordinator
robot_control/run_five_arm_cycle.py
```

该协调器使用单一长期 `SimBridge/RobotExecutor` 实例，不在相邻工序间停止或
重连仿真。它是机械臂执行层的基础工艺协同，不取代订单调度模块。

R4 保存场景没有螺丝刀模型。运行时使用 `100 mm` 可视工具和
`(180,0,-135)` 度竖直姿态；三个 Git APP/TCP/PRESS target 均不修改。
这只表示视觉旋转/下压，不代表物理扭矩或真实螺钉接触验收。

---

## 1. 订单输入格式

文件建议路径：`data/orders/demo_orders.json`

```json
[
  {
    "order_id": "A_OK",
    "product_type": "A",
    "priority": 1,
    "quantity": 1,
    "due_time": 120,
    "expected_quality": "OK"
  },
  {
    "order_id": "B_NG",
    "product_type": "B",
    "priority": 2,
    "quantity": 1,
    "due_time": 160,
    "expected_quality": "NG"
  }
]
```

| 字段 | 类型 | 含义 |
|---|---|---|
| `order_id` | string | 订单编号 |
| `product_type` | string | 产品类型，如 A/B/C 型柜 |
| `priority` | int | 优先级，数值越大越紧急 |
| `quantity` | int | 数量 |
| `due_time` | float | 期望完成时间，可选 |
| `expected_quality` | string | 当前真实演示的检测结果，`OK` 或 `NG`，可选，默认 `OK` |

说明：`expected_quality` 用于稳定触发 R4 相机信号和 R5 唯一分支，避免答辩
现场随机选择。真实视觉检测接入后，该字段可改为期望值/测试配置，实际分支应
使用相机返回结果。

---

## 2. 产品工艺配置

文件建议路径：`configs/product_types.yaml`

基础工艺链：

```yaml
A:
  processes: [feed, assemble, screw, inspect, sort]
  component_count: 3
  screw_count: 6
  inspect_points: 2

B:
  processes: [feed, assemble, screw, inspect, sort]
  component_count: 5
  screw_count: 10
  inspect_points: 3

C:
  processes: [feed, assemble, screw, inspect, sort]
  component_count: 6
  screw_count: 12
  inspect_points: 4
```

拓展工艺链：

```yaml
rework_enabled: false
rework_processes: [move_to_rework, disassemble]
```

说明：

- `sort` 表示 R4 根据 R3 检测结果执行良品/不良品分拣；
- `rework_enabled` 为 `true` 时，调度系统可以为不良品追加返修拆解任务；
- 两个月内优先实现 `sort`，返修拆解作为增强功能。

---

## 3. 场景点位接口

文件建议路径：`configs/points.yaml`

场景搭建同学必须保证 CoppeliaSim 中的关键点位和这里保持一致。

```yaml
P_FEED_01:
  area: feed_area
  robot: R1
  action: feed
  description: 上料抓取点

P_ASSEMBLY_01:
  area: assembly_area
  robot: R2
  action: assemble
  description: 元件装配点 1

P_SCREW_01:
  area: screw_area
  robot: R3
  action: screw
  description: 螺丝锁付点 1

P_INSPECT_01:
  area: inspect_area
  robot: R3
  action: inspect
  description: 检测点 1

P_GOOD_01:
  area: good_area
  robot: R4
  action: sort_good
  description: 良品下料点

P_DEFECT_01:
  area: defect_area
  robot: R4
  action: sort_defect
  description: 不良品下料点

P_REWORK_01:
  area: rework_area
  robot: R4
  action: rework
  description: 不良品返修拆解点，可选
```

---

## 4. 任务格式

调度模块内部任务对象建议格式：

```json
{
  "task_id": "T001",
  "order_id": "A_OK",
  "product_type": "A",
  "process": "feed",
  "target_area": "feed_area",
  "target_point": "P_FEED_01",
  "available_robots": ["R1"],
  "duration": 8,
  "predecessors": [],
  "priority": 1,
  "status": "pending"
}
```

检测任务示例：

```json
{
  "task_id": "T004",
  "order_id": "A_OK",
  "product_type": "A",
  "process": "inspect",
  "target_area": "inspect_area",
  "target_point": "P_INSPECT_01",
  "available_robots": ["R3"],
  "duration": 6,
  "predecessors": ["T003"],
  "priority": 1,
  "status": "pending",
  "quality_required": true
}
```

分拣任务不建议在订单生成时同时生成 `sort_good` 和 `sort_defect` 两个任务，而应在 R3 检测完成后根据检测结果动态生成一个。

分拣任务示例：

```json
{
  "task_id": "T005",
  "order_id": "A_OK",
  "product_type": "A",
  "process": "sort_good",
  "target_area": "good_area",
  "target_point": "P_GOOD_01",
  "available_robots": ["R4"],
  "duration": 8,
  "predecessors": ["T004"],
  "priority": 1,
  "status": "pending",
  "quality_result_dependency": "T004"
}
```

---

## 5. 调度输出格式

调度算法给机械臂动作模块的输出建议格式：

```json
{
  "timestamp": 12.0,
  "robot_id": "R2",
  "task_id": "T002",
  "action": "assemble",
  "target_area": "assembly_area",
  "target_point": "P_ASSEMBLY_01",
  "estimated_duration": 15,
  "payload": {
    "order_id": "A_OK",
    "product_type": "A"
  }
}
```

R4 分拣任务输出示例：

```json
{
  "timestamp": 41.0,
  "robot_id": "R4",
  "task_id": "T005",
  "action": "sort_defect",
  "target_area": "defect_area",
  "target_point": "P_DEFECT_01",
  "estimated_duration": 8,
  "payload": {
    "order_id": "B_NG",
    "quality_result": "NG",
    "source_task": "T004"
  }
}
```

---

## 6. 机械臂执行接口

机械臂控制模块建议向软件集成模块暴露统一函数：

```python
def execute_task(task_command: dict) -> dict:
    """执行调度模块下发的任务，返回统一状态。"""
```

R3 检测任务返回示例：

```json
{
  "task_id": "T004",
  "robot_id": "R3",
  "status": "finished",
  "start_time": 35.0,
  "end_time": 41.0,
  "quality_result": "NG",
  "message": "missing screw detected"
}
```

R4 分拣任务返回示例：

```json
{
  "task_id": "T005",
  "robot_id": "R4",
  "status": "finished",
  "start_time": 41.0,
  "end_time": 49.0,
  "quality_result": "NG",
  "message": "defect product moved to defect area"
}
```

当前真实 `TaskResult` 还包含可选 `metrics` 字段。首动监测示例：

```json
{
  "motion_timing": {
    "motion_detected": true,
    "threshold_deg": 0.02,
    "dispatch_to_first_motion_wall_s": 1.76,
    "task_call_to_first_motion_wall_s": 2.18,
    "first_motion_simulation_time_s": 29.0,
    "monitor_error": ""
  },
  "handoff_to_first_motion_simulation_s": 1.30,
  "task_end_simulation_time_s": 58.20
}
```

`handoff_to_first_motion_simulation_s` 以当前 Task 首次真实关节变化减去上一 Task
结束仿真时刻；它不同于旧协调器的 Task 函数调用间隔。

其中 `quality_result` 建议取值：

| 值 | 含义 |
|---|---|
| `OK` | 合格 |
| `NG` | 不合格 |
| `UNKNOWN` | 检测失败或结果未知 |

---

## 7. R3 检测到 R4 分拣的状态流

```text
R3 执行 inspect
    ↓
返回 quality_result
    ↓
quality_result == OK
    └── 调度模块动态生成 sort_good 任务，目标点 P_GOOD_01

quality_result == NG
    └── 调度模块动态生成 sort_defect 任务，目标点 P_DEFECT_01

quality_result == UNKNOWN
    └── 可选：repeat_inspect / manual_check
```

要求：同一件产品检测后只能生成一个 R4 分拣任务，避免同时生成良品和不良品两个分支。

---

## 8. 区域锁格式

共享区域建议使用区域锁机制，先实现稳定演示，再逐步升级到更复杂的避碰算法。

```yaml
areas:
  feed_area:
    lock_required: false
  assembly_area:
    lock_required: true
  screw_area:
    lock_required: true
  inspect_area:
    lock_required: true
  good_area:
    lock_required: false
  defect_area:
    lock_required: false
  rework_area:
    lock_required: true
  shared_transfer_area:
    lock_required: true
```

区域锁事件示例：

```json
{
  "robot_id": "R1",
  "area_id": "shared_transfer_area",
  "event": "request_lock",
  "timestamp": 18.5
}
```

---

## 9. 日志输出格式

文件建议路径：`data/logs/task_log.csv`

```csv
task_id,order_id,robot_id,process,start_time,end_time,duration,wait_time,status,quality_result,target_area
T001,A_OK,R1,feed,0,8,8,0,finished,,feed_area
T002,A_OK,R2,assemble,8,23,15,2,finished,,assembly_area
T003,A_OK,R3,screw,23,35,12,1,finished,,screw_area
T004,A_OK,R3,inspect,35,41,6,0,finished,OK,inspect_area
T005,A_OK,R4,sort_good,41,49,8,0,finished,OK,good_area
T006,B_NG,R3,inspect,50,56,6,0,finished,NG,inspect_area
T007,B_NG,R4,sort_defect,56,64,8,0,finished,NG,defect_area
```

---

## 10. 评价指标

```text
makespan = max(all_task_end_time) - min(all_task_start_time)
utilization(robot_i) = robot_i_busy_time / makespan
average_waiting_time = sum(task_wait_time) / task_count
conflict_count = 共享区域锁申请失败或等待事件次数
sorting_response_time = R4分拣完成时间 - R3检测完成时间
sorting_accuracy = 正确分拣数量 / 分拣任务数量
rework_response_time = 不良检测完成时刻到不良品进入返修/不良品区的时间
```

---

## 11. 推荐目录

```text
data/
├── orders/
│   └── demo_orders.json
├── logs/
│   └── task_log.csv
└── results/
    ├── makespan_compare.png
    ├── robot_utilization.png
    ├── waiting_time_compare.png
    └── quality_sorting_summary.png

configs/
├── robots.yaml
├── stations.yaml
├── points.yaml
├── product_types.yaml
└── scheduler.yaml
```
