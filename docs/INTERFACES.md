# 接口规范

本文件用于统一场景、动作、调度、软件集成和数据看板之间的数据格式，避免各模块互相等待或最后无法整合。

核心原则：

- 场景同学负责点位和对象命名；
- 机械臂控制同学只接收标准任务并返回状态；
- 调度同学只生成任务，不直接写机械臂底层运动；
- 软件集成同学只通过接口调用各模块，不私自定义另一套格式；
- 未完成的真实模块先用 Mock 模块替代。

---

## 1. 订单输入格式

文件建议路径：`data/orders/demo_orders.json`

```json
[
  {
    "order_id": "A001",
    "product_type": "A",
    "priority": 1,
    "quantity": 1,
    "due_time": 120
  },
  {
    "order_id": "B001",
    "product_type": "B",
    "priority": 2,
    "quantity": 1,
    "due_time": 160
  },
  {
    "order_id": "C_URGENT",
    "product_type": "C",
    "priority": 5,
    "quantity": 1,
    "due_time": 90
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
  "order_id": "A001",
  "product_type": "A",
  "process": "feed",
  "target_area": "feed_area",
  "target_point": "P_FEED_01",
  "available_robots": ["R1"],
  "duration": 8,
  "predecessors": [],
  "priority": 1,
  "status": "pending",
  "quality_required": false
}
```

检测任务示例：

```json
{
  "task_id": "T004",
  "order_id": "A001",
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

分拣任务示例：

```json
{
  "task_id": "T005",
  "order_id": "A001",
  "product_type": "A",
  "process": "sort",
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
  "task_id": "T004",
  "action": "pick_and_place",
  "target_area": "assembly_area",
  "target_point": "P_ASSEMBLY_02",
  "estimated_duration": 15,
  "payload": {
    "order_id": "A001",
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
    "order_id": "A001",
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

Mock 返回示例：

```json
{
  "task_id": "T005",
  "robot_id": "R4",
  "status": "finished",
  "start_time": 41.0,
  "end_time": 49.0,
  "quality_result": null,
  "message": "defect product moved to defect area"
}
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

其中 `quality_result` 建议取值：

| 值 | 含义 |
|---|---|
| `OK` | 合格 |
| `NG` | 不合格 |
| `UNKNOWN` | 检测失败或结果未知 |

---

## 7. 区域锁格式

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

## 8. 日志输出格式

文件建议路径：`data/logs/task_log.csv`

```csv
task_id,order_id,robot_id,process,start_time,end_time,duration,wait_time,status,quality_result,target_area
T001,A001,R1,feed,0,8,8,0,finished,,feed_area
T002,A001,R2,assemble,8,23,15,2,finished,,assembly_area
T003,A001,R3,screw,23,35,12,1,finished,,screw_area
T004,A001,R3,inspect,35,41,6,0,finished,NG,inspect_area
T005,A001,R4,sort,41,49,8,0,finished,NG,defect_area
```

---

## 9. 评价指标

```text
makespan = max(all_task_end_time) - min(all_task_start_time)
utilization(robot_i) = robot_i_busy_time / makespan
average_waiting_time = sum(task_wait_time) / task_count
conflict_count = 共享区域锁申请失败或等待事件次数
sorting_accuracy = 正确分拣数量 / 分拣任务数量
rework_response_time = 不良检测完成时刻到不良品进入返修/不良品区的时间
```

---

## 10. 推荐目录

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
