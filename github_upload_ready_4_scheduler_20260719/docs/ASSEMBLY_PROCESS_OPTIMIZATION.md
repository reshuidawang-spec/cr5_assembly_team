# 4号调度增强：元件细化、工步仿真与装配线平衡

## 1. 是否需要建模

需要建模，但这里主要不是建立三维几何模型，而是建立调度侧的“装配过程模型”。

三维场景里已经有箱体、PCB、控制模块、端子排、检测平台、机械臂目标点等对象。4号调度需要补的是：

```text
装配元件 -> 依赖关系 -> 安装顺序 -> 工序 -> 工步 -> 执行资源 -> 时间 -> 瓶颈分析
```

也就是说，调度模型要回答：

1. 低压配电柜由哪些元件组成；
2. 哪些元件必须先装，哪些可以作为同层细节；
3. 每个工序由哪些更小的工步构成；
4. 每个工步由哪台机械臂、相机或检测平台参与；
5. 运行后哪台机械臂最忙，是否需要装配线平衡优化。

## 2. 元件细化思路

原来调度侧只看到 4 个粗粒度元件：

```text
箱体、PCB板、控制模块、端子排
```

现在按照五 CR5A 场景对象进一步细化为：

| 层级 | 元件/特征 | 场景对象 | 作用 |
|---|---|---|---|
| 1 | 箱体壳 | `Assembly_ControlBox_Product_Shell` | 装配基础 |
| 2 | PCB板 | `Assembly_ControlBox_Product_PCB` | 电路板主体 |
| 3 | PCB定位孔 | `Assembly_ControlBox_Product_PCB_Hole_1~4` | 表达安装定位检查 |
| 3 | PCB板载元件 | `Assembly_ControlBox_Product_PCB_Main_Chip/Connector/Capacitor` | 表达芯片、连接器、电容等检测对象 |
| 4 | 控制模块本体 | `Assembly_ControlBox_Product_Control_Module_Body` | R3 安装的模块主体 |
| 5 | 控制模块标签 | `Assembly_ControlBox_Product_Control_Module_Label` | 表达识别和检测细节 |
| 6 | 端子排本体 | `Assembly_ControlBox_Product_Terminal_Block_Body` | R1 安装的端子排主体 |
| 7 | 端子排接线槽 | `Assembly_ControlBox_Product_Terminal_Block_Slot_1~4` | 表达接线槽细节 |
| 8 | 端子排主螺钉头 | `Assembly_ControlBox_Product_Terminal_Block_Main_ScrewHead` | R4 后续锁付的作用目标 |

注意：PCB定位孔、PCB板载元件、控制模块标签、端子排接线槽、端子排主螺钉头不一定都生成独立机械臂搬运动作。它们是细化后的装配/检测对象，用来让调度结果更接近真实装配过程。

## 3. 安装顺序

层次化拓扑排序后，当前顺序为：

```text
箱体壳
 -> PCB板
 -> PCB定位孔 / PCB板载元件
 -> 控制模块本体
 -> 控制模块标签 / 端子排本体
 -> 端子排接线槽 / 端子排主螺钉头
 -> 转移完整装配体到检测区
 -> 固定相机检测
 -> R4 螺钉锁付
 -> R5 合格品/缺陷品分拣
```

这个顺序仍然保持五臂场景主流程不变：

```text
R1 箱体上料
 -> R2 PCB安装
 -> R3 控制模块安装
 -> R1 端子排安装
 -> R3 转移检测区
 -> CAMERA 检测
 -> R4 锁付
 -> R5 分拣
```

区别是：现在每个粗工序下面有了更细的装配对象和工步解释。

## 4. 什么是层次化拓扑排序

拓扑排序就是：如果 B 依赖 A，那么 A 必须排在 B 前面。

层次化拓扑排序是在这个基础上再分层：

- 第 1 层：没有前置依赖的对象；
- 第 2 层：只依赖第 1 层的对象；
- 第 3 层：依赖前面层级完成的对象；
- 同一层内的对象没有严格先后依赖，可以看作同层细节或并行候选。

例如 PCB定位孔和 PCB板载元件都依赖 PCB板，但它们彼此之间没有严格前后关系，所以可以处在同一层。

## 5. 目标函数优化

原来的综合评分主要考虑：

```text
订单优先级 + 交期紧急程度 + 等待时间 + 剩余关键路径
```

现在增加了两个轻量优化项：

```text
瓶颈资源惩罚 + 共享区域冲突惩罚
```

当前配置为：

```text
瓶颈资源：R3
冲突敏感区域：inspection_platform_area
普通订单会受到轻微惩罚，急单不受惩罚
```

这样做的意思是：如果普通订单之间竞争，调度器会稍微避免把任务继续压到 R3 或检测平台上；但如果是急单，仍然按急单优先。

## 6. 工步级时间仿真

运行：

```bash
python scripts/run_assembly_process_simulation.py
```

会输出：

| 文件 | 内容 |
|---|---|
| `data/assembly_process_v2/component_sequence.csv` | 细化元件和操作的层次化顺序 |
| `data/assembly_process_v2/step_timeline.csv` | 每个工序拆成工步后的时间表 |
| `data/assembly_process_v2/line_balance_summary.json` | 装配线平衡指标 |
| `data/assembly_process_v2/line_balance_recommendations.json` | 根据瓶颈生成的优化建议 |
| `output/visualizations/assembly_step_timeline.html` | 类似序列编辑器的工步时间轴 |

## 7. 装配线平衡结果

当前仿真结果显示：

```text
平衡率约为 61.3%
瓶颈资源为 R3
```

这说明 R3 同时承担控制模块安装和完整装配体转移，是当前负载最高的资源。

优化方向：

1. 普通订单优先让 R1/R2 完成箱体、PCB、端子排等准备动作；
2. 避免多个订单同时卡在 R3 前后；
3. 检测平台是 CAMERA、R4、R5 的共享区域，需要减少检测、锁付、分拣之间的等待；
4. 急单不套用瓶颈惩罚，保证急单响应速度。

## 8. 和之前做法的区别

之前的做法更像：

```text
订单 -> 大工序 -> 分配机械臂 -> 看完成时间
```

现在变成：

```text
订单 -> 装配元件模型 -> 层次化安装顺序 -> 工序 -> 工步 -> 时间轴 -> 产线平衡
```

所以这次优化不是单纯“把算法名字写复杂”，而是让调度有了更真实的装配对象、依赖关系和结果解释。
