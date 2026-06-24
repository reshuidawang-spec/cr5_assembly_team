# 接口规范

本文件用于统一 2、3、4、5 号同学之间的数据格式，避免仿真、动作、调度、看板互相等待。

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

```yaml
A:
  processes: [feed, assemble, screw, inspect]
  component_count: 3
  screw_count: 6

B:
  processes: [feed, assemble, screw, inspect]
  component_count: 5
  screw_count: 10

C:
  processes: [feed, assemble, screw, inspect]
  component_count: 6
  screw_count: 12
```

---

## 3. 任务格式

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
  "status": "pending"
}
```

---

## 4. 调度输出格式

调度算法给机械臂动作模块的输出建议格式：

```json
{
  "timestamp": 12.0,
  "robot_id": "R2",
  "task_id": "T004",
  "action": "pick_and_place",
  "target_area": "assembly_area",
  "target_point": "P_ASSEMBLY_02",
  "estimated_duration": 15
}
```

---

## 5. 区域锁格式

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
    lock_required: false
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

## 6. 日志输出格式

文件建议路径：`data/logs/task_log.csv`

```csv
task_id,order_id,robot_id,process,start_time,end_time,duration,wait_time,status
T001,A001,R1,feed,0,8,8,0,finished
T002,A001,R2,assemble,8,23,15,2,finished
T003,A001,R3,screw,23,35,12,1,finished
T004,A001,R3,inspect,35,41,6,0,finished
```

---

## 7. 评价指标

```text
makespan = max(all_task_end_time) - min(all_task_start_time)
utilization(robot_i) = robot_i_busy_time / makespan
average_waiting_time = sum(task_wait_time) / task_count
conflict_count = 共享区域锁申请失败或等待事件次数
```

---

## 8. 推荐目录

```text
data/
├── orders/
│   └── demo_orders.json
├── logs/
│   └── task_log.csv
└── results/
    ├── makespan_compare.png
    ├── robot_utilization.png
    └── waiting_time_compare.png

configs/
├── robots.yaml
├── stations.yaml
├── product_types.yaml
└── scheduler.yaml
```
