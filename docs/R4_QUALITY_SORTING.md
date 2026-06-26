# R4 良品/不良品分拣与返修拓展方案

## 1. 加入 R4 的目的

在原有 R1、R2、R3 三机械臂基础上，增加 R4 作为质量分拣与返修拓展机械臂，可以让产线从“完成装配”进一步扩展到“检测后闭环处理”。

R4 的加入能够体现：

- 检测结果驱动的自主决策；
- 良品与不良品自动分流；
- 不良品返修区管理；
- 产线异常处理能力；
- 多机械臂协同效能优化。

R4 属于增强模块。优先实现良品/不良品分拣，返修拆解作为时间充足后的拓展功能。

---

## 2. R4 工作空间

建议在 CoppeliaSim 场景中增加三个区域：

| 区域 | 名称 | 作用 |
|---|---|---|
| 良品区 | `good_area` | 存放检测合格产品 |
| 不良品区 | `defect_area` | 存放检测不合格产品 |
| 返修区 | `rework_area` | 可选，用于不良品再拆解或返修预处理 |

推荐点位：

| 点位 | 含义 | 执行机械臂 |
|---|---|---|
| `P_GOOD_01` | 良品下料点 | R4 |
| `P_DEFECT_01` | 不良品下料点 | R4 |
| `P_REWORK_01` | 返修拆解点 | R4 |

---

## 3. R4 任务生成逻辑

R3 完成检测后，需要返回 `quality_result`：

```json
{
  "task_id": "T004",
  "robot_id": "R3",
  "status": "finished",
  "quality_result": "NG",
  "message": "missing screw detected"
}
```

调度模块根据 `quality_result` 生成 R4 任务：

```text
quality_result == OK → 生成 sort_good 任务 → R4 移动到 P_GOOD_01
quality_result == NG → 生成 sort_defect 任务 → R4 移动到 P_DEFECT_01
quality_result == UNKNOWN → 生成 manual_check 或 repeat_inspect 任务
```

---

## 4. 分拣任务示例

良品分拣：

```json
{
  "task_id": "T005",
  "order_id": "A001",
  "robot_id": "R4",
  "action": "sort_good",
  "target_point": "P_GOOD_01",
  "target_area": "good_area",
  "predecessors": ["T004"],
  "quality_result": "OK"
}
```

不良品分拣：

```json
{
  "task_id": "T005",
  "order_id": "A001",
  "robot_id": "R4",
  "action": "sort_defect",
  "target_point": "P_DEFECT_01",
  "target_area": "defect_area",
  "predecessors": ["T004"],
  "quality_result": "NG"
}
```

返修拆解，可选：

```json
{
  "task_id": "T006",
  "order_id": "A001",
  "robot_id": "R4",
  "action": "rework",
  "target_point": "P_REWORK_01",
  "target_area": "rework_area",
  "predecessors": ["T005"],
  "quality_result": "NG"
}
```

---

## 5. 演示方案

### 基础演示

```text
R3 检测合格
    ↓
调度系统生成 R4 sort_good 任务
    ↓
R4 抓取产品并放入良品区
```

### 不良品演示

```text
R3 检测不合格
    ↓
调度系统生成 R4 sort_defect 任务
    ↓
R4 抓取产品并放入不良品区
```

### 拓展演示

```text
R3 检测不合格
    ↓
R4 放入不良品区
    ↓
若 R4 空闲且 rework_enabled = true
    ↓
R4 将不良品转移到返修区
    ↓
执行拆解动作或返修预处理动作
```

---

## 6. 评价指标

| 指标 | 含义 |
|---|---|
| 分拣响应时间 | R3 检测完成到 R4 完成分拣的时间 |
| 分拣准确率 | R4 正确放入良品/不良品区的比例 |
| 不良品处理率 | 检测为不良的产品中进入不良品区或返修区的比例 |
| R4 利用率 | R4 忙碌时间 / 总任务完成时间 |
| 返修响应时间 | 不良品进入返修区所需时间 |

---

## 7. 实施优先级

### 必做

- R3 检测任务返回 `quality_result`；
- 调度模块根据 `OK/NG` 生成 R4 分拣任务；
- R4 完成良品/不良品分流；
- 软件界面显示检测结果和分拣状态。

### 选做

- R4 将不良品送入返修区；
- R4 对不良品进行拆解动作；
- 统计返修响应时间；
- 加入重复检测或人工复核逻辑。

---

## 8. 风险控制

为避免工作量过大，R4 返修拆解不应作为前期主线。

推荐开发顺序：

```text
第一阶段：R4 到点分拣，良品/不良品区域分流
第二阶段：R4 抓取并移动产品模型
第三阶段：R4 不良品转入返修区
第四阶段：R4 拆解不良品，作为增强展示
```

如果时间不足，保留第一阶段和第二阶段即可满足质量闭环展示需求。
