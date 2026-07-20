# 五台 CR5A 小型电控箱协同装配仿真场景说明与控制接口文档

> 适用场景：CoppeliaSim + ROS2 + 五台 DOBOT CR5A 机械臂协同装配小型电控箱  
> 当前目标：让其他小组成员能够基于本场景继续实现机械臂运动控制、路径规划、抓取释放和多机械臂协同调度。

---

## 1. 项目总体思路

本场景用于模拟一个“小型电控箱自动装配单元”。场景中包含五台 CR5A 机械臂、两个圆形减震工作台、供料区、装配区、检测区、锁螺钉区、视觉相机、合格品传送带和缺陷品传送带。

整体流程为：

```text
供料区
  ↓
R1 抓取箱体并放到装配区
  ↓
R2 安装 PCB 板
  ↓
R3 安装控制模块
  ↓
R1 安装端子排
  ↓
R3 将完整装配体搬运到检测区
  ↓
固定相机检测
  ↓
R4 对端子螺钉进行锁付
  ↓
R5 根据检测结果把产品放到合格品或缺陷品传送带
```

当前场景的重点不是完整实现路径规划，而是提供一个清晰、可调用、可扩展的仿真接口，使负责运动控制的成员可以根据目标点、末端点和 ROS2 指令继续实现实际控制。

---

## 2. 当前场景已经能实现什么效果

当前场景主要由两个脚本组成：

```text
Main_Cell_Generator
ROS2_All_Robot_Bridge
```

其中：

### 2.1 Main_Cell_Generator 能实现的功能

主场景脚本负责生成和维护仿真环境，当前能够实现：

1. 自动生成 `/FiveCR5A_Cell` 主场景树。
2. 自动生成地面、两个圆形减震工作台、R1 到 R5 的机械臂安装基座。
3. 自动生成供料区：
   - 箱体供料区
   - 端子排供料区
   - PCB 供料区
   - 控制模块供料区
4. 自动生成装配区和检测/锁付区。
5. 自动生成固定立柱相机和检测视野区域。
6. 自动生成合格品传送带和缺陷品传送带。
7. 自动生成供料零件：
   - `Box_Blank`
   - `PCB_Supply`
   - `Control_Module_Supply`
   - `Terminal_Block_Supply`
8. 自动生成装配区和检测区的产品模板：
   - `Assembly_ControlBox_Product`
   - `Inspection_ControlBox_Product`
9. 自动把已有的 `/R1` 到 `/R5` 机械臂摆放到对应基座位置。
10. 自动冻结机械臂动力学，避免 URDF 或导入模型在仿真启动后乱飞。
11. 自动生成 R1 到 R5 的末端点：
    - `R1_gripper_tip`
    - `R2_gripper_tip`
    - `R3_gripper_tip`
    - `R4_tool_tip`
    - `R5_gripper_tip`
12. 自动生成所有工序目标点 Dummy。
13. 根据内部 signal 显示或隐藏装配阶段产品。
14. 根据检测结果改变相机检测区域颜色。
15. 根据分拣结果启动合格品或缺陷品传送带上的产品运动。

### 2.2 ROS2_All_Robot_Bridge 能实现的功能

ROS2 桥接脚本负责把 ROS2 指令转换为 CoppeliaSim 内部 signal，当前能够实现：

1. 订阅统一主场景指令。
2. 订阅 R1 到 R5 的独立机械臂指令。
3. 订阅一个兼容总入口 `/compact_cell/cmd`。
4. 将 ROS2 指令转换为内部 signal。
5. 触发主场景显示装配阶段、检测阶段、传送带阶段。
6. 向 `/compact_cell/status` 发布当前执行状态。

---

## 3. 当前场景不能直接实现什么

需要明确，当前场景还没有自动实现以下内容：

1. 不自动计算机械臂逆运动学。
2. 不自动规划 R1 到 R5 的避障路径。
3. 不自动完成真实夹爪物理抓取。
4. 不自动控制每个关节沿复杂轨迹运动到目标点。
5. 不自动判断多机械臂之间的碰撞和等待关系。

这些部分需要由负责控制的成员继续实现。当前场景已经提供了：

```text
机械臂对象
末端 tip 点
工序目标点
产品对象路径
ROS2 指令接口
CoppeliaSim 内部 signal 接口
```

其他成员需要基于这些接口实现：

```text
读取目标点
规划运动
控制机械臂关节或末端
绑定工件
释放工件
发布动作完成指令
```

---

## 4. 场景脚本结构

建议最终保留两个核心 Dummy 脚本：

```text
Main_Cell_Generator
ROS2_All_Robot_Bridge
```

### 4.1 Main_Cell_Generator

该脚本负责场景本体，建议挂在一个独立 Dummy 上。

主要职责：

```text
场景生成
机械臂摆放
tip 点生成
target 点生成
产品状态管理
传送带运动
相机检测状态显示
机械臂动力学冻结
```

第一次从零生成场景时，开关建议为：

```lua
local REBUILD_SCENE_ON_START = true
local RESET_TIPS_ON_START = true
local RESET_TARGETS_ON_START = true
```

第一次成功生成后，建议改成：

```lua
local REBUILD_SCENE_ON_START = false
local RESET_TIPS_ON_START = false
local RESET_TARGETS_ON_START = false
```

这样后续手动调整场景、tip 或目标点时，不会每次启动仿真都被重置。

### 4.2 ROS2_All_Robot_Bridge

该脚本负责 ROS2 通信，不直接控制机械臂路径。

主要职责：

```text
订阅 ROS2 topic
解析 R1~R5 指令
把指令转换成 CoppeliaSim 内部 signal
发布状态到 /compact_cell/status
```

---

## 5. 场景对象路径规范

所有场景对象建议统一挂在：

```text
/FiveCR5A_Cell
```

其下建议结构为：

```text
/FiveCR5A_Cell
  ├─ Ground_Group
  ├─ Tables
  ├─ RobotBases
  ├─ Areas
  ├─ Parts
  ├─ Conveyors
  ├─ Sensors
  └─ Targets
```

### 5.1 零件对象路径

| 对象 | 路径 | 说明 |
|---|---|---|
| 箱体毛坯 | `/FiveCR5A_Cell/Parts/Box_Blank` | R1 抓取的箱体 |
| PCB 供料 | `/FiveCR5A_Cell/Parts/PCB_Supply` | R2 抓取的 PCB |
| 控制模块供料 | `/FiveCR5A_Cell/Parts/Control_Module_Supply` | R3 抓取的控制模块 |
| 端子排供料 | `/FiveCR5A_Cell/Parts/Terminal_Block_Supply` | R1 抓取的端子排 |
| 装配区产品 | `/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product` | 装配阶段产品模板 |
| 检测区产品 | `/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product` | 检测/锁付/分拣阶段产品模板 |

### 5.2 区域对象路径

| 区域 | 路径 |
|---|---|
| 箱体供料区 | `/FiveCR5A_Cell/Areas/Box_Supply_Area` |
| 端子排供料区 | `/FiveCR5A_Cell/Areas/Terminal_Supply_Area` |
| PCB 供料区 | `/FiveCR5A_Cell/Areas/PCB_Supply_Area` |
| 控制模块供料区 | `/FiveCR5A_Cell/Areas/Module_Supply_Area` |
| 装配区 | `/FiveCR5A_Cell/Areas/Assembly_Area` |
| 装配夹具 | `/FiveCR5A_Cell/Areas/Assembly_Fixture` |
| 检测/锁付区 | `/FiveCR5A_Cell/Areas/Inspection_Screw_Area` |
| 检测平台 | `/FiveCR5A_Cell/Areas/Inspection_Platform` |

### 5.3 传送带对象路径

| 传送带 | 路径 |
|---|---|
| 合格品传送带 | `/FiveCR5A_Cell/Conveyors/Good_Conveyor` |
| 缺陷品传送带 | `/FiveCR5A_Cell/Conveyors/Defect_Conveyor` |

---

## 6. 机械臂分工

| 机械臂 | 任务 |
|---|---|
| R1 | 箱体上料、端子排安装 |
| R2 | PCB 安装 |
| R3 | 控制模块安装、完整装配体转移到检测区 |
| R4 | 单个端子螺钉锁付 |
| R5 | 合格品/缺陷品分拣 |

---

## 7. 末端 tip 点说明

每台机械臂都需要有一个稳定的末端点，用于路径规划、抓取绑定和释放判断。

| 机械臂 | tip 名称 | 用途 |
|---|---|---|
| R1 | `R1_gripper_tip` | 箱体、端子排抓取点 |
| R2 | `R2_gripper_tip` | PCB 抓取点 |
| R3 | `R3_gripper_tip` | 控制模块和完整装配体抓取点 |
| R4 | `R4_tool_tip` | 螺丝刀工具末端 |
| R5 | `R5_gripper_tip` | 检测区产品分拣抓取点 |

这些 tip 点应挂在对应机械臂的 `Link6_visual` 下。例如：

```text
/R1/.../Link6_visual/R1_gripper_tip
/R2/.../Link6_visual/R2_gripper_tip
/R3/.../Link6_visual/R3_gripper_tip
/R4/.../Link6_visual/R4_tool_tip
/R5/.../Link6_visual/R5_gripper_tip
```

如果控制成员要让机械臂末端运动到某个目标点，本质上就是让：

```text
R*_gripper_tip 或 R4_tool_tip
```

运动到对应的目标 Dummy 位姿。

---

## 8. 工序目标点规划

目标点统一放在：

```text
/FiveCR5A_Cell/Targets
```

目标点命名规则：

```text
APP = Approach，接近点，位于目标上方，用于安全接近
TCP = 实际抓取、放置或操作点
```

控制逻辑建议为：

```text
先运动到 APP
再下到 TCP
执行 attach / detach / screw
再返回 APP
```

### 8.1 R1 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R1_HOME_REF | `/FiveCR5A_Cell/Targets/R1_Targets/R1_HOME_REF` | R1 参考初始位 |
| R1_BOX_PICK_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_APP` | 箱体抓取接近点 |
| R1_BOX_PICK_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_TCP` | 箱体抓取点 |
| R1_BOX_PLACE_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PLACE_APP` | 箱体放置接近点 |
| R1_BOX_PLACE_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PLACE_TCP` | 箱体放置点 |
| R1_TERMINAL_PICK_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PICK_APP` | 端子排抓取接近点 |
| R1_TERMINAL_PICK_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PICK_TCP` | 端子排抓取点 |
| R1_TERMINAL_PLACE_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PLACE_APP` | 端子排安装接近点 |
| R1_TERMINAL_PLACE_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PLACE_TCP` | 端子排安装点 |

### 8.2 R2 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R2_HOME_REF | `/FiveCR5A_Cell/Targets/R2_Targets/R2_HOME_REF` | R2 参考初始位 |
| R2_PCB_PICK_APP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PICK_APP` | PCB 抓取接近点 |
| R2_PCB_PICK_TCP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PICK_TCP` | PCB 抓取点 |
| R2_PCB_PLACE_APP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PLACE_APP` | PCB 安装接近点 |
| R2_PCB_PLACE_TCP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PLACE_TCP` | PCB 安装点 |

### 8.3 R3 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R3_HOME_REF | `/FiveCR5A_Cell/Targets/R3_Targets/R3_HOME_REF` | R3 参考初始位 |
| R3_MODULE_PICK_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PICK_APP` | 控制模块抓取接近点 |
| R3_MODULE_PICK_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PICK_TCP` | 控制模块抓取点 |
| R3_MODULE_PLACE_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PLACE_APP` | 控制模块安装接近点 |
| R3_MODULE_PLACE_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PLACE_TCP` | 控制模块安装点 |
| R3_PRODUCT_PICK_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PICK_APP` | 装配体抓取接近点 |
| R3_PRODUCT_PICK_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PICK_TCP` | 装配体抓取点 |
| R3_PRODUCT_PLACE_INSPECTION_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PLACE_INSPECTION_APP` | 检测区放置接近点 |
| R3_PRODUCT_PLACE_INSPECTION_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PLACE_INSPECTION_TCP` | 检测区放置点 |

### 8.4 R4 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R4_HOME_REF | `/FiveCR5A_Cell/Targets/R4_Targets/R4_HOME_REF` | R4 参考初始位 |
| R4_SCREW_APP | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_APP` | 螺钉接近点 |
| R4_SCREW_TCP | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_TCP` | 螺钉接触点 |
| R4_SCREW_PRESS | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_PRESS` | 模拟下压锁付点 |

### 8.5 R5 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R5_HOME_REF | `/FiveCR5A_Cell/Targets/R5_Targets/R5_HOME_REF` | R5 参考初始位 |
| R5_PRODUCT_PICK_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_PRODUCT_PICK_APP` | 检测区产品抓取接近点 |
| R5_PRODUCT_PICK_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_PRODUCT_PICK_TCP` | 检测区产品抓取点 |
| R5_GOOD_PLACE_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_GOOD_PLACE_APP` | 合格品放置接近点 |
| R5_GOOD_PLACE_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_GOOD_PLACE_TCP` | 合格品放置点 |
| R5_DEFECT_PLACE_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_DEFECT_PLACE_APP` | 缺陷品放置接近点 |
| R5_DEFECT_PLACE_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_DEFECT_PLACE_TCP` | 缺陷品放置点 |

### 8.6 相机检测目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| CAMERA_INSPECTION_CENTER | `/FiveCR5A_Cell/Targets/Sensor_Targets/CAMERA_INSPECTION_CENTER` | 检测中心参考点 |

---

## 9. ROS2 topic 接口

ROS2 桥接脚本订阅以下 topic：

| topic | 消息类型 | 说明 |
|---|---|---|
| `/compact_cell/main_cmd` | `std_msgs/msg/String` | 主场景命令 |
| `/compact_cell/r1_cmd` | `std_msgs/msg/String` | R1 命令 |
| `/compact_cell/r2_cmd` | `std_msgs/msg/String` | R2 命令 |
| `/compact_cell/r3_cmd` | `std_msgs/msg/String` | R3 命令 |
| `/compact_cell/r4_cmd` | `std_msgs/msg/String` | R4 命令 |
| `/compact_cell/r5_cmd` | `std_msgs/msg/String` | R5 命令 |
| `/compact_cell/cmd` | `std_msgs/msg/String` | 总入口，自动根据前缀分发 |
| `/compact_cell/status` | `std_msgs/msg/String` | 状态发布 |

推荐调试时可以全部使用总入口：

```text
/compact_cell/cmd
```

正式分工开发时，每个机械臂成员可以使用自己的 topic：

```text
/compact_cell/r1_cmd
/compact_cell/r2_cmd
/compact_cell/r3_cmd
/compact_cell/r4_cmd
/compact_cell/r5_cmd
```

---

## 10. ROS2 命令定义

### 10.1 主场景命令

| 命令 | 作用 |
|---|---|
| `RESET_CELL` | 重置场景状态 |
| `SHOW_ASSEMBLY_SHELL` | 装配区显示箱体 |
| `SHOW_ASSEMBLY_PCB` | 装配区显示 PCB |
| `SHOW_ASSEMBLY_MODULE` | 装配区显示控制模块 |
| `SHOW_ASSEMBLY_FULL` | 装配区显示完整装配体 |
| `SHOW_INSPECTION_FULL` | 检测区显示完整装配体 |
| `CAMERA_GOOD` | 相机检测区域变为合格状态 |
| `CAMERA_DEFECT` | 相机检测区域变为缺陷状态 |
| `CONVEYOR_GOOD` | 启动合格品传送带产品运动 |
| `CONVEYOR_DEFECT` | 启动缺陷品传送带产品运动 |

### 10.2 R1 命令

| 命令 | 作用 |
|---|---|
| `R1_READY` | R1 就绪 |
| `R1_BOX_PLACED` | R1 已把箱体放到装配区 |
| `R1_TERMINAL_PLACED` | R1 已把端子排安装到装配区 |

### 10.3 R2 命令

| 命令 | 作用 |
|---|---|
| `R2_READY` | R2 就绪 |
| `R2_PCB_PLACED` | R2 已把 PCB 安装到箱体内 |

### 10.4 R3 命令

| 命令 | 作用 |
|---|---|
| `R3_READY` | R3 就绪 |
| `R3_MODULE_PLACED` | R3 已把控制模块安装到箱体内 |
| `R3_PRODUCT_TO_INSPECTION` | R3 已把完整装配体放到检测区 |

### 10.5 R4 命令

| 命令 | 作用 |
|---|---|
| `R4_READY` | R4 就绪 |
| `R4_SCREW_DONE` | R4 已完成螺钉锁付 |

### 10.6 R5 命令

| 命令 | 作用 |
|---|---|
| `R5_READY` | R5 就绪 |
| `R5_SORT_GOOD_DONE` | R5 已把产品放到合格品传送带 |
| `R5_SORT_DEFECT_DONE` | R5 已把产品放到缺陷品传送带 |

---

## 11. CoppeliaSim 内部 signal 接口

ROS2 桥接脚本会将 ROS2 命令转换成 CoppeliaSim 内部 signal。

### 11.1 产品状态 signal

signal 名称：

```lua
cell_product_state
```

可用值：

| signal 值 | 场景效果 |
|---|---|
| `reset` | 重置场景 |
| `assembly_shell` | 装配区显示箱体 |
| `assembly_pcb` | 装配区显示箱体 + PCB |
| `assembly_module` | 装配区显示箱体 + PCB + 控制模块 |
| `assembly_full` | 装配区显示完整产品 |
| `inspection_full` | 检测区显示完整产品 |
| `camera_good` | 相机区域变为合格颜色 |
| `camera_defect` | 相机区域变为缺陷颜色 |

调用方式：

```lua
sim.setStringSignal('cell_product_state', 'assembly_shell')
```

### 11.2 传送带状态 signal

signal 名称：

```lua
cell_conveyor_state
```

可用值：

| signal 值 | 场景效果 |
|---|---|
| `good` | 合格品传送带开始运动 |
| `defect` | 缺陷品传送带开始运动 |

调用方式：

```lua
sim.setStringSignal('cell_conveyor_state', 'good')
```

### 11.3 螺钉状态 signal

signal 名称：

```lua
cell_screw_state
```

可用值：

| signal 值 | 场景效果 |
|---|---|
| `done` | 表示锁付完成，当前主要用于状态记录，可后续扩展视觉效果 |

### 11.4 工件视觉所有权 signal

signal 名称：

```lua
cell_visual_owner
```

可用值：

| signal 值 | 所有权与场景行为 |
|---|---|
| `template` | Generator 按 `cell_product_state` 显示阶段产品模板，供 Mock/手动展示使用 |
| `executor` | 真实运动执行器管理 `Box_Blank` 等实际工件的 attach、detach 和可见性；Generator 仍处理状态，但跳过 `assembly_shell`、`assembly_pcb`、`assembly_module`、`assembly_full`、`inspection_full` 模板显示 |

正式 `SimBridge.connect()` 会设置：

```lua
sim.setStringSignal('cell_visual_owner', 'executor')
```

这样 `/compact_cell/status` 仍可返回 `DONE:*`，同时不会在实际搬运工件之外
再生成一套重叠产品模板。未设置该 signal 时保持原 Generator 行为，兼容
Mock 和手动 `SHOW_*` 命令。

---

## 12. 典型完整流程

下面是推荐的完整控制逻辑。

### 12.1 初始化

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

场景恢复到初始状态：

```text
供料区有箱体、PCB、控制模块、端子排
装配区为空
检测区为空
传送带为空
```

### 12.2 R1 箱体上料

控制成员需要实现：

```text
R1_gripper_tip → R1_BOX_PICK_APP
R1_gripper_tip → R1_BOX_PICK_TCP
attach Box_Blank 到 R1_gripper_tip
R1_gripper_tip → R1_BOX_PICK_APP
R1_gripper_tip → R1_BOX_PLACE_APP
R1_gripper_tip → R1_BOX_PLACE_TCP
detach Box_Blank 到装配区
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

或：

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

### 12.3 R2 安装 PCB

控制成员需要实现：

```text
R2_gripper_tip → R2_PCB_PICK_APP
R2_gripper_tip → R2_PCB_PICK_TCP
attach PCB_Supply 到 R2_gripper_tip
R2_gripper_tip → R2_PCB_PICK_APP
R2_gripper_tip → R2_PCB_PLACE_APP
R2_gripper_tip → R2_PCB_PLACE_TCP
detach PCB 到装配区箱体内部
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r2_cmd std_msgs/msg/String "{data: 'R2_PCB_PLACED'}" --once
```

### 12.4 R3 安装控制模块

控制成员需要实现：

```text
R3_gripper_tip → R3_MODULE_PICK_APP
R3_gripper_tip → R3_MODULE_PICK_TCP
attach Control_Module_Supply 到 R3_gripper_tip
R3_gripper_tip → R3_MODULE_PICK_APP
R3_gripper_tip → R3_MODULE_PLACE_APP
R3_gripper_tip → R3_MODULE_PLACE_TCP
detach 控制模块到 PCB 上
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_MODULE_PLACED'}" --once
```

### 12.5 R1 安装端子排

控制成员需要实现：

```text
R1_gripper_tip → R1_TERMINAL_PICK_APP
R1_gripper_tip → R1_TERMINAL_PICK_TCP
attach Terminal_Block_Supply 到 R1_gripper_tip
R1_gripper_tip → R1_TERMINAL_PICK_APP
R1_gripper_tip → R1_TERMINAL_PLACE_APP
R1_gripper_tip → R1_TERMINAL_PLACE_TCP
detach 端子排到箱体内
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_TERMINAL_PLACED'}" --once
```

### 12.6 R3 转移完整装配体到检测区

控制成员需要实现：

```text
R3_gripper_tip → R3_PRODUCT_PICK_APP
R3_gripper_tip → R3_PRODUCT_PICK_TCP
attach Assembly_ControlBox_Product 到 R3_gripper_tip
R3_gripper_tip → R3_PRODUCT_PICK_APP
R3_gripper_tip → R3_PRODUCT_PLACE_INSPECTION_APP
R3_gripper_tip → R3_PRODUCT_PLACE_INSPECTION_TCP
detach 产品到检测区
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_PRODUCT_TO_INSPECTION'}" --once
```

### 12.7 相机检测

检测结果为合格：

```bash
ros2 topic pub /compact_cell/main_cmd std_msgs/msg/String "{data: 'CAMERA_GOOD'}" --once
```

检测结果为缺陷：

```bash
ros2 topic pub /compact_cell/main_cmd std_msgs/msg/String "{data: 'CAMERA_DEFECT'}" --once
```

### 12.8 R4 锁螺钉

控制成员需要实现：

```text
R4_tool_tip → R4_SCREW_APP
R4_tool_tip → R4_SCREW_TCP
R4_tool_tip → R4_SCREW_PRESS
模拟旋转/下压
R4_tool_tip → R4_SCREW_APP
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r4_cmd std_msgs/msg/String "{data: 'R4_SCREW_DONE'}" --once
```

### 12.9 R5 分拣

合格品流程：

```text
R5_gripper_tip → R5_PRODUCT_PICK_APP
R5_gripper_tip → R5_PRODUCT_PICK_TCP
attach Inspection_ControlBox_Product 到 R5_gripper_tip
R5_gripper_tip → R5_PRODUCT_PICK_APP
R5_gripper_tip → R5_GOOD_PLACE_APP
R5_gripper_tip → R5_GOOD_PLACE_TCP
detach 产品到合格品传送带入口
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_GOOD_DONE'}" --once
```

缺陷品流程：

```text
R5_gripper_tip → R5_PRODUCT_PICK_APP
R5_gripper_tip → R5_PRODUCT_PICK_TCP
attach Inspection_ControlBox_Product 到 R5_gripper_tip
R5_gripper_tip → R5_PRODUCT_PICK_APP
R5_gripper_tip → R5_DEFECT_PLACE_APP
R5_gripper_tip → R5_DEFECT_PLACE_TCP
detach 产品到缺陷品传送带入口
```

动作完成后发布：

```bash
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_DEFECT_DONE'}" --once
```

---

## 13. 控制成员需要如何读取目标点

在 CoppeliaSim Lua 中读取目标点位姿：

```lua
local target = sim.getObject('/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_TCP')
local pos = sim.getObjectPosition(target, -1)
local ori = sim.getObjectOrientation(target, -1)
```

读取 R1 末端 tip：

```lua
local tip = sim.getObject('/R1/.../R1_gripper_tip')
```

如果不确定完整路径，可以在 `/R1` 树下递归查找名称为 `R1_gripper_tip` 的对象。

---

## 14. 抓取与释放的推荐实现方式

当前建议使用运动学绑定，不做真实夹爪接触力学。

### 14.1 抓取

```lua
local part = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank')
local tip = sim.getObject('/R1/.../R1_gripper_tip')

sim.setObjectParent(part, tip, true)
```

这里第三个参数 `true` 表示保持当前世界位姿，只改变父子关系。绑定后，工件会跟随机械臂末端运动。

### 14.2 释放

```lua
local part = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank')
local partsRoot = sim.getObject('/FiveCR5A_Cell/Parts')

sim.setObjectParent(part, partsRoot, true)
sim.setObjectPosition(part, -1, {-1.15, 0.20, 0.216})
```

释放后，再发布对应完成命令。例如：

```lua
sim.setStringSignal('cell_product_state', 'assembly_shell')
```

或由 ROS2 发送：

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

---

## 15. 控制成员推荐开发步骤

### 15.1 第一步：只验证 ROS2 通信

运行 CoppeliaSim 后，在终端输入：

```bash
ros2 topic echo /compact_cell/status
```

另开一个终端发送：

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

正常情况下，应该能看到状态返回。

### 15.2 第二步：只验证场景状态变化

依次发送：

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_SHELL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_PCB'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_MODULE'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_FULL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_INSPECTION_FULL'}" --once
```

观察装配区和检测区是否按阶段显示产品。

### 15.3 第三步：验证单个机械臂控制

每个机械臂成员先实现：

```text
读取 joints
读取 tip
读取 target
控制某一个关节点动
打印当前关节角
```

不要一开始就写完整自动路径。

### 15.4 第四步：实现 APP → TCP → APP 的简单运动

先不用复杂避障，每个动作按三个点做：

```text
APP
TCP
APP
```

### 15.5 第五步：加入 attach / detach

确认机械臂末端可以带着工件运动。

### 15.6 第六步：加入 ROS2 动作完成反馈

动作完成后发布对应命令，使主场景进入下一阶段。

---

## 16. 各成员分工建议

| 成员 | 负责内容 |
|---|---|
| 场景搭建成员 | 维护 `Main_Cell_Generator`、目标点、对象路径、布局 |
| ROS2 接口成员 | 维护 `ROS2_All_Robot_Bridge`、topic、命令格式 |
| R1 控制成员 | 箱体上料、端子排安装 |
| R2 控制成员 | PCB 安装 |
| R3 控制成员 | 控制模块安装、产品转移 |
| R4 控制成员 | 螺钉锁付动作 |
| R5 控制成员 | 检测后分拣 |
| 调度成员 | 控制 R1~R5 的执行顺序和互锁逻辑 |

---

## 17. 推荐的总调度顺序

总调度程序可以按如下逻辑执行：

```text
RESET_CELL
等待 READY

启动 R1 箱体上料
等待 R1_BOX_PLACED

启动 R2 PCB 安装
等待 R2_PCB_PLACED

启动 R3 控制模块安装
等待 R3_MODULE_PLACED

启动 R1 端子排安装
等待 R1_TERMINAL_PLACED

启动 R3 产品转移
等待 R3_PRODUCT_TO_INSPECTION

启动相机检测
若合格：CAMERA_GOOD
若缺陷：CAMERA_DEFECT

启动 R4 锁螺钉
等待 R4_SCREW_DONE

若合格：启动 R5 合格品分拣
等待 R5_SORT_GOOD_DONE

若缺陷：启动 R5 缺陷品分拣
等待 R5_SORT_DEFECT_DONE
```

---

## 18. 调试命令汇总

### 18.1 查看状态

```bash
ros2 topic echo /compact_cell/status
```

### 18.2 查看 topic

```bash
ros2 topic list
```

### 18.3 重置场景

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

### 18.4 显示装配阶段

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_SHELL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_PCB'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_MODULE'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_FULL'}" --once
```

### 18.5 显示检测区完整产品

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_INSPECTION_FULL'}" --once
```

### 18.6 相机检测结果

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_GOOD'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_DEFECT'}" --once
```

### 18.7 传送带

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CONVEYOR_GOOD'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CONVEYOR_DEFECT'}" --once
```

### 18.8 R1 到 R5 完成信号

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
ros2 topic pub /compact_cell/r2_cmd std_msgs/msg/String "{data: 'R2_PCB_PLACED'}" --once
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_MODULE_PLACED'}" --once
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_TERMINAL_PLACED'}" --once
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_PRODUCT_TO_INSPECTION'}" --once
ros2 topic pub /compact_cell/r4_cmd std_msgs/msg/String "{data: 'R4_SCREW_DONE'}" --once
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_GOOD_DONE'}" --once
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_DEFECT_DONE'}" --once
```

---

## 19. 启动注意事项

必须从已经 source ROS2 的终端启动 CoppeliaSim。不要直接双击桌面图标启动，否则 `simROS2` 可能加载失败。

推荐命令：

```bash
source /opt/ros/humble/setup.bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

如果有自己的 ROS2 工作空间：

```bash
source ~/dobot_ws/install/setup.bash
```

也可以在启动 CoppeliaSim 前一起 source。

---

## 20. 当前实现的定位

本场景目前适合作为：

```text
多机械臂装配任务的仿真场景基础
ROS2 通信接口验证平台
机械臂路径规划小组的目标点接口
装配流程可视化平台
多机械臂协同调度测试平台
```

它不是最终完整控制系统。最终系统还需要补充：

```text
每台机械臂的运动控制脚本
目标点到关节轨迹的求解
避障规划
夹取与释放逻辑
多机械臂互锁调度
视觉检测结果输入
异常处理机制
```

---

## 21. 交付给控制成员的核心接口

控制成员最需要关注以下五类接口：

```text
1. 机械臂根路径：/R1 ~ /R5
2. 末端点：R1_gripper_tip ~ R5_gripper_tip，R4_tool_tip
3. 目标点：/FiveCR5A_Cell/Targets/...
4. 工件对象：/FiveCR5A_Cell/Parts/...
5. ROS2 命令：/compact_cell/..._cmd
```

控制成员只要能够做到：

```text
读取目标点
控制 tip 到达目标点
绑定或释放工件
发送动作完成命令
```

就可以基于本场景实现完整的五机械臂协同装配流程。

---

## 22. 当前仓库的真实五臂执行入口

前述 Lua 桥接本身仍只负责 signal 和状态，但当前仓库已在 Python 执行层
实现 R1-R5 基础视觉动作和固定顺序协同：

```text
sim_bridge.coppelia_client.SimBridge
robot_control.robot_executor.RobotExecutor
robot_control.five_arm_coordinator.FiveArmCoordinator
```

打开本文档对应的 `five_cr5a_cell.ttt` 并保持停止后，运行：

```bash
python3 robot_control/run_five_arm_cycle.py --quality good
python3 robot_control/run_five_arm_cycle.py --quality defect
```

两个命令都使用同一个长期 CoppeliaSim 连接，不在相邻机械臂工序间停止
或重载仿真。运行约束、当前场景的视觉偏移和 R5 good 边界见
`robot_control/FIVE_ARM_COORDINATOR.md`。
