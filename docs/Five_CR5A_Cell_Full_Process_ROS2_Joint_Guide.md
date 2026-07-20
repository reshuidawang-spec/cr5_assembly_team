# 五台 CR5A 小型电控箱协同装配仿真场景完整搭建与使用流程

> 适用环境：Ubuntu 22.04 + ROS2 Humble + CoppeliaSim Edu 4.10 + 五台 DOBOT CR5/CR5A 机械臂  
> 当前目标：完成一个可与 ROS2 互通的五机械臂协同装配仿真场景，并提供末端工具控制、工件绑定释放、产品阶段显示、APP/TCP 工艺点位和关节测试控制接口。  
> 说明：本文档记录从零搭建、脚本使用、对象命名、ROS2 测试、关节控制和后续交付的完整过程。

---

## 1. 当前场景最终能实现什么

当前场景已经实现：

```text
1. 五台机械臂 R1 ~ R5 的协同装配场景
2. 60% 缩放后的小型电控箱、PCB、控制模块、端子排
3. 两个圆形减震工作台
4. 供料区、装配区、检测/锁付区、合格品传送带、缺陷品传送带
5. 固定立柱相机和检测区域
6. R1/R3/R5 宽口夹爪
7. R2 吸盘
8. R4 电动螺丝刀
9. 夹爪开合、吸盘吸附、螺丝刀旋转
10. 工件 attach / detach 绑定释放
11. 产品按装配阶段显示和隐藏
12. CoppeliaSim 与 ROS2 topic 互通
13. 通过 ROS2 控制末端工具动作
14. 通过 ROS2 控制机械臂关节点动
15. 生成 R1 ~ R5 的 APP/TCP 工艺目标点
```

当前还没有完整实现：

```text
1. 自动逆运动学求解
2. 自动避障路径规划
3. 多机械臂自动调度
4. 机械臂自动按照 APP → TCP → APP 运动完成整套任务
```

也就是说，本场景已经完成“场景搭建 + ROS2 接口 + 工具动作 + 关节点动 + 工艺点位接口”。  
后续路径规划成员可以基于本场景继续实现完整自动运动控制。

---

## 2. 总体工艺流程

完整装配流程如下：

```text
RESET_CELL
  ↓
R1 抓取箱体并放到装配区
  ↓
R2 吸取 PCB 并放入箱体
  ↓
R3 抓取控制模块并安装到 PCB 上
  ↓
R1 抓取端子排并安装到箱体内
  ↓
R3 抓取完整装配体并搬运到检测区
  ↓
固定相机检测
  ↓
R4 对端子排螺钉进行锁付
  ↓
R5 根据检测结果把产品放到合格品或缺陷品传送带
```

机械臂分工：

| 机械臂 | 任务 | 末端工具 |
|---|---|---|
| R1 | 箱体上料、端子排安装 | 宽口可调夹爪 |
| R2 | PCB 安装 | 吸盘 |
| R3 | 控制模块安装、完整装配体转移到检测区 | 宽口夹爪 |
| R4 | 单个端子螺钉锁付 | 电动螺丝刀 |
| R5 | 合格品/缺陷品分拣 | 宽口夹爪 |

---

## 3. 总共需要几个脚本

最终一共使用 **7 个 Lua 脚本**。

分为两类：

### 3.1 生成类脚本：运行一次后禁用

这些脚本只负责生成场景、末端工具或目标点。运行成功后必须禁用，否则每次仿真启动都可能重复生成对象或覆盖手动调整。

| 序号 | 脚本名 | 作用 | 运行状态 |
|---|---|---|---|
| 1 | `Step01_Create_Clean_Cell_60_GreyTable_RobotColor.lua` | 生成 60% 缩放总场景 | 运行一次后禁用 |
| 2 | `Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua` | 创建并安装 R1/R3/R5 夹爪、R2 吸盘、R4 螺丝刀 | 运行一次后禁用 |
| 3 | `Step03_Create_Process_Targets_60.lua` | 创建 APP/TCP 工艺目标点 | 运行一次后禁用 |

### 3.2 运行类脚本：仿真时一直启用

这些脚本负责运行时控制，应长期保持启用。

| 序号 | 脚本名 | 作用 | 运行状态 |
|---|---|---|---|
| 4 | `Product_Stage_Controller_60.lua` | 控制产品阶段显示/隐藏 | 一直启用 |
| 5 | `Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua` | 控制夹爪、吸盘、螺丝刀、工件绑定释放 | 一直启用 |
| 6 | `ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua` | ROS2 与 CoppeliaSim 工艺命令互通 | 一直启用 |
| 7 | `ROS2_Joint_Jog_Controller_R1_R5.lua` | ROS2 控制 R1~R5 关节点动 | 一直启用 |

---

## 4. CoppeliaSim 启动方式

### 4.1 不能直接双击启动

如果直接双击 CoppeliaSim，可能会出现：

```text
plugin simROS2: Cannot load library ...
libnav_msgs__rosidl_typesupport_cpp.so: cannot open shared object file
```

这不是脚本问题，而是因为 CoppeliaSim 没有继承 ROS2 环境变量。

### 4.2 正确启动方式

打开 Ubuntu 终端，输入：

```bash
source /opt/ros/humble/setup.bash
```

如果使用自己的工作空间，再输入：

```bash
source ~/dobot_ws/install/setup.bash
```

然后启动 CoppeliaSim：

```bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

以后所有需要 simROS2 的场景，都必须这样启动。

### 4.3 如果仍然缺库

如果仍然提示缺 `nav_msgs`，安装：

```bash
sudo apt update
sudo apt install ros-humble-nav-msgs
```

也可以顺手安装常用消息包：

```bash
sudo apt install ros-humble-std-msgs ros-humble-geometry-msgs ros-humble-sensor-msgs ros-humble-tf2-msgs ros-humble-nav-msgs
```

---

## 5. Step01：创建 60% 缩放装配场景

### 5.1 准备机械臂对象

场景中需要有五台机械臂，根对象名称必须为：

```text
R1
R2
R3
R4
R5
```

如果机械臂名称不是这几个，需要先重命名。

### 5.2 添加 Step01 脚本

新建 Dummy：

```text
Step01_Create_Clean_Cell_60
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
Step01_Create_Clean_Cell_60_GreyTable_RobotColor.lua
```

点击仿真运行一次。

运行成功后，脚本会创建：

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

运行成功后，禁用或删除 Step01 脚本。

### 5.3 60% 缩放工件尺寸

| 工件 | 缩放后尺寸 |
|---|---|
| 电控箱底壳 | 0.21 × 0.15 × 0.072 m |
| PCB | 0.144 × 0.096 × 0.0048 m |
| 控制模块 | 0.054 × 0.039 × 0.021 m |
| 端子排 | 0.096 × 0.021 × 0.021 m |

---

## 6. 场景主要对象名称与路径

### 6.1 主场景根路径

```text
/FiveCR5A_Cell
```

### 6.2 工件对象

| 对象 | 路径 | 说明 |
|---|---|---|
| 箱体供料 | `/FiveCR5A_Cell/Parts/Box_Blank` | R1 抓取 |
| PCB 供料 | `/FiveCR5A_Cell/Parts/PCB_Supply` | R2 抓取 |
| 控制模块供料 | `/FiveCR5A_Cell/Parts/Control_Module_Supply` | R3 抓取 |
| 端子排供料 | `/FiveCR5A_Cell/Parts/Terminal_Block_Supply` | R1 抓取 |
| 装配区产品模板 | `/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product` | 装配阶段显示 |
| 检测区产品模板 | `/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product` | 检测、锁付、分拣阶段显示 |

### 6.3 区域对象

| 区域 | 路径 |
|---|---|
| 箱体供料区 | `/FiveCR5A_Cell/Areas/Box_Supply_Area` |
| PCB 供料区 | `/FiveCR5A_Cell/Areas/PCB_Supply_Area` |
| 控制模块供料区 | `/FiveCR5A_Cell/Areas/Module_Supply_Area` |
| 端子排供料区 | `/FiveCR5A_Cell/Areas/Terminal_Supply_Area` |
| 装配区 | `/FiveCR5A_Cell/Areas/Assembly_Area` |
| 装配夹具 | `/FiveCR5A_Cell/Areas/Assembly_Fixture` |
| 检测/锁付区 | `/FiveCR5A_Cell/Areas/Inspection_Screw_Area` |
| 检测平台 | `/FiveCR5A_Cell/Areas/Inspection_Platform` |

### 6.4 传送带对象

| 对象 | 路径 |
|---|---|
| 合格品传送带 | `/FiveCR5A_Cell/Conveyors/Good_Conveyor` |
| 缺陷品传送带 | `/FiveCR5A_Cell/Conveyors/Defect_Conveyor` |

### 6.5 相机对象

| 对象 | 路径 |
|---|---|
| 固定视觉相机工位 | `/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station` |
| 相机视野区域 | `/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station/Camera_View_Area` |

---

## 7. Step02A：创建并安装末端工具

### 7.1 添加末端工具创建脚本

新建 Dummy：

```text
Create_EndEffectors
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
```

运行一次。

运行后会生成并安装：

| 机械臂 | 工具根对象 | 类型 |
|---|---|---|
| R1 | `R1T` | 宽口可调夹爪 |
| R2 | `R2T` | 吸盘 |
| R3 | `R3T` | 宽口夹爪 |
| R4 | `R4T` | 电动螺丝刀 |
| R5 | `R5T` | 宽口夹爪 |

运行成功后，禁用或删除该脚本。

### 7.2 末端 tip 名称

| 机械臂 | tip 名称 | 用途 |
|---|---|---|
| R1 | `R1_gripper_tip` | 箱体、端子排抓取点 |
| R2 | `R2_vacuum_tip` | PCB 吸取点 |
| R3 | `R3_gripper_tip` | 控制模块、完整装配体抓取点 |
| R4 | `R4_tool_tip` | 螺丝刀末端点 |
| R5 | `R5_gripper_tip` | 检测区产品分拣抓取点 |

### 7.3 关键工具子对象

R1：

```text
R1T
  ├─ R1T_left_finger_link
  ├─ R1T_right_finger_link
  └─ R1T_tool_tcp
       └─ R1_gripper_tip
```

R2：

```text
R2T
  └─ R2T_tool_tcp
       └─ R2_vacuum_tip
```

R3：

```text
R3T
  ├─ R3T_left_finger_link
  ├─ R3T_right_finger_link
  └─ R3T_tool_tcp
       └─ R3_gripper_tip
```

R4：

```text
R4T
  └─ R4T_screw_spin_link
       └─ R4T_tool_tcp
            └─ R4_tool_tip
```

R5：

```text
R5T
  ├─ R5T_left_finger_link
  ├─ R5T_right_finger_link
  └─ R5T_tool_tcp
       └─ R5_gripper_tip
```

---

## 8. Step02B：末端工具动作控制脚本

### 8.1 添加控制脚本

新建 Dummy：

```text
Step02B_Tool_Action_Controller
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua
```

该脚本需要一直保持启用。

### 8.2 工具控制原理

当前抓取方式不是物理摩擦夹取，而是：

```text
夹爪视觉开合
  +
工件 attach 到工具 TCP
  +
释放时 detach 回 Parts 根节点
```

这种方式适合工艺流程展示、ROS2 接口验证和路径规划测试。

---

## 9. Step02C：产品阶段显示脚本

### 9.1 添加脚本

新建 Dummy：

```text
Product_Stage_Controller_60
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
Product_Stage_Controller_60.lua
```

该脚本需要一直保持启用。

### 9.2 产品阶段 signal

signal 名称：

```lua
cell_product_state
```

可用值：

| 值 | 效果 |
|---|---|
| `reset` | 重置产品状态 |
| `assembly_shell` | 装配区显示箱体 |
| `assembly_pcb` | 装配区显示箱体 + PCB |
| `assembly_module` | 装配区显示箱体 + PCB + 控制模块 |
| `assembly_full` | 装配区显示完整产品 |
| `inspection_full` | 检测区显示完整产品 |
| `camera_good` | 相机检测区域变为合格颜色 |
| `camera_defect` | 相机检测区域变为缺陷颜色 |

本地 Lua 测试：

```lua
sim.setStringSignal('cell_product_state','reset')
sim.setStringSignal('cell_product_state','assembly_shell')
sim.setStringSignal('cell_product_state','assembly_pcb')
sim.setStringSignal('cell_product_state','assembly_module')
sim.setStringSignal('cell_product_state','assembly_full')
sim.setStringSignal('cell_product_state','inspection_full')
```

### 9.3 传送带 signal

signal 名称：

```lua
cell_conveyor_state
```

可用值：

| 值 | 效果 |
|---|---|
| `good` | 合格品传送带动作 |
| `defect` | 缺陷品传送带动作 |

测试：

```lua
sim.setStringSignal('cell_conveyor_state','good')
sim.setStringSignal('cell_conveyor_state','defect')
```

---

## 10. Step02D：ROS2 工艺命令桥接脚本

### 10.1 添加脚本

新建 Dummy：

```text
ROS2_CompactCell_Bridge
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua
```

该脚本需要一直保持启用。

### 10.2 ROS2 topic

该脚本负责创建：

| topic | 类型 | 作用 |
|---|---|---|
| `/compact_cell/cmd` | `std_msgs/msg/String` | 总入口 |
| `/compact_cell/main_cmd` | `std_msgs/msg/String` | 主场景命令 |
| `/compact_cell/tool_cmd` | `std_msgs/msg/String` | 工具动作命令 |
| `/compact_cell/r1_cmd` | `std_msgs/msg/String` | R1 命令 |
| `/compact_cell/r2_cmd` | `std_msgs/msg/String` | R2 命令 |
| `/compact_cell/r3_cmd` | `std_msgs/msg/String` | R3 命令 |
| `/compact_cell/r4_cmd` | `std_msgs/msg/String` | R4 命令 |
| `/compact_cell/r5_cmd` | `std_msgs/msg/String` | R5 命令 |
| `/compact_cell/status` | `std_msgs/msg/String` | 状态反馈 |

查看 topic：

```bash
ros2 topic list
```

监听状态：

```bash
ros2 topic echo /compact_cell/status
```

---

## 11. Step02E：ROS2 关节点动控制脚本

### 11.1 添加脚本

新建 Dummy：

```text
ROS2_Joint_Jog_Controller
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
ROS2_Joint_Jog_Controller_R1_R5.lua
```

该脚本需要一直保持启用。

### 11.2 作用

该脚本用于通过 ROS2 控制 R1~R5 的 J1~J6 关节运动。

它不是路径规划，只是关节点动测试工具。

可用于验证：

```text
1. ROS2 能控制机械臂关节
2. 脚本能找到 R1~R5 的六个关节
3. 机械臂关节能被 setJointTargetPosition 或 setJointPosition 控制
```

### 11.3 ROS2 topic

| topic | 类型 | 作用 |
|---|---|---|
| `/compact_cell/joint_cmd` | `std_msgs/msg/String` | 关节控制命令 |
| `/compact_cell/joint_status` | `std_msgs/msg/String` | 关节控制状态反馈 |

查看：

```bash
ros2 topic list
```

正常应多出：

```text
/compact_cell/joint_cmd
/compact_cell/joint_status
```

监听状态：

```bash
ros2 topic echo /compact_cell/joint_status
```

---

## 12. Step03：创建 APP/TCP 工艺目标点

### 12.1 添加脚本

新建 Dummy：

```text
Step03_Create_Process_Targets_60
```

添加：

```text
Non-threaded child script
```

粘贴脚本：

```text
Step03_Create_Process_Targets_60.lua
```

运行一次。

运行成功后，禁用或删除该脚本。

### 12.2 目标点目录

所有目标点位于：

```text
/FiveCR5A_Cell/Targets
```

分组如下：

```text
/FiveCR5A_Cell/Targets/R1_Targets
/FiveCR5A_Cell/Targets/R2_Targets
/FiveCR5A_Cell/Targets/R3_Targets
/FiveCR5A_Cell/Targets/R4_Targets
/FiveCR5A_Cell/Targets/R5_Targets
/FiveCR5A_Cell/Targets/Sensor_Targets
```

### 12.3 APP/TCP 定义

```text
APP = Approach，接近点，位于目标上方
TCP = 实际抓取、释放或操作点
PRESS = R4 螺丝刀下压点
```

推荐控制逻辑：

```text
机械臂 tip → APP
机械臂 tip → TCP
执行 attach / detach / screw
机械臂 tip → APP
```

---

## 13. R1 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R1_HOME_REF | `/FiveCR5A_Cell/Targets/R1_Targets/R1_HOME_REF` | R1 初始参考点 |
| R1_BOX_PICK_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_APP` | 箱体抓取接近点 |
| R1_BOX_PICK_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_TCP` | 箱体抓取点 |
| R1_BOX_PLACE_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PLACE_APP` | 箱体放置接近点 |
| R1_BOX_PLACE_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PLACE_TCP` | 箱体放置点 |
| R1_TERMINAL_PICK_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PICK_APP` | 端子排抓取接近点 |
| R1_TERMINAL_PICK_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PICK_TCP` | 端子排抓取点 |
| R1_TERMINAL_PLACE_APP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PLACE_APP` | 端子排安装接近点 |
| R1_TERMINAL_PLACE_TCP | `/FiveCR5A_Cell/Targets/R1_Targets/R1_TERMINAL_PLACE_TCP` | 端子排安装点 |

---

## 14. R2 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R2_HOME_REF | `/FiveCR5A_Cell/Targets/R2_Targets/R2_HOME_REF` | R2 初始参考点 |
| R2_PCB_PICK_APP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PICK_APP` | PCB 抓取接近点 |
| R2_PCB_PICK_TCP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PICK_TCP` | PCB 抓取点 |
| R2_PCB_PLACE_APP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PLACE_APP` | PCB 安装接近点 |
| R2_PCB_PLACE_TCP | `/FiveCR5A_Cell/Targets/R2_Targets/R2_PCB_PLACE_TCP` | PCB 安装点 |

---

## 15. R3 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R3_HOME_REF | `/FiveCR5A_Cell/Targets/R3_Targets/R3_HOME_REF` | R3 初始参考点 |
| R3_MODULE_PICK_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PICK_APP` | 控制模块抓取接近点 |
| R3_MODULE_PICK_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PICK_TCP` | 控制模块抓取点 |
| R3_MODULE_PLACE_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PLACE_APP` | 控制模块安装接近点 |
| R3_MODULE_PLACE_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_MODULE_PLACE_TCP` | 控制模块安装点 |
| R3_PRODUCT_PICK_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PICK_APP` | 完整装配体抓取接近点 |
| R3_PRODUCT_PICK_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PICK_TCP` | 完整装配体抓取点 |
| R3_PRODUCT_PLACE_INSPECTION_APP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PLACE_INSPECTION_APP` | 检测区放置接近点 |
| R3_PRODUCT_PLACE_INSPECTION_TCP | `/FiveCR5A_Cell/Targets/R3_Targets/R3_PRODUCT_PLACE_INSPECTION_TCP` | 检测区放置点 |

---

## 16. R4 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R4_HOME_REF | `/FiveCR5A_Cell/Targets/R4_Targets/R4_HOME_REF` | R4 初始参考点 |
| R4_SCREW_APP | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_APP` | 螺丝刀接近点 |
| R4_SCREW_TCP | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_TCP` | 螺丝刀接触点 |
| R4_SCREW_PRESS | `/FiveCR5A_Cell/Targets/R4_Targets/R4_SCREW_PRESS` | 螺丝刀下压锁付点 |

---

## 17. R5 目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| R5_HOME_REF | `/FiveCR5A_Cell/Targets/R5_Targets/R5_HOME_REF` | R5 初始参考点 |
| R5_PRODUCT_PICK_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_PRODUCT_PICK_APP` | 检测区产品抓取接近点 |
| R5_PRODUCT_PICK_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_PRODUCT_PICK_TCP` | 检测区产品抓取点 |
| R5_GOOD_PLACE_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_GOOD_PLACE_APP` | 合格品放置接近点 |
| R5_GOOD_PLACE_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_GOOD_PLACE_TCP` | 合格品放置点 |
| R5_DEFECT_PLACE_APP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_DEFECT_PLACE_APP` | 缺陷品放置接近点 |
| R5_DEFECT_PLACE_TCP | `/FiveCR5A_Cell/Targets/R5_Targets/R5_DEFECT_PLACE_TCP` | 缺陷品放置点 |

---

## 18. 相机目标点

| 目标点 | 路径 | 用途 |
|---|---|---|
| CAMERA_INSPECTION_CENTER | `/FiveCR5A_Cell/Targets/Sensor_Targets/CAMERA_INSPECTION_CENTER` | 相机检测中心参考点 |

---

## 19. ROS2 工具动作测试命令

先打开一个终端监听状态：

```bash
source /opt/ros/humble/setup.bash
ros2 topic echo /compact_cell/status
```

另开一个终端发布命令。

### 19.1 重置场景

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

### 19.2 R1 夹爪测试

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_OPEN'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_CLOSE_BOX'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_CLOSE_TERMINAL'}" --once
```

### 19.3 R1 抓箱体和端子排

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_ATTACH_BOX'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_RELEASE_BOX_ASSEMBLY'}" --once
```

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_ATTACH_TERMINAL'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_RELEASE_TERMINAL_ASSEMBLY'}" --once
```

### 19.4 R2 吸 PCB

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R2_ATTACH_PCB'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R2_RELEASE_PCB_ASSEMBLY'}" --once
```

### 19.5 R3 安装模块和转移产品

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_ATTACH_MODULE'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_RELEASE_MODULE_ASSEMBLY'}" --once
```

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_ATTACH_ASSEMBLY_PRODUCT'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_RELEASE_PRODUCT_INSPECTION'}" --once
```

### 19.6 R4 拧螺丝

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_INSPECTION_FULL'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R4_SCREW_START'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R4_SCREW_STOP'}" --once
```

### 19.7 R5 分拣

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_ATTACH_INSPECTION_PRODUCT'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_RELEASE_GOOD'}" --once
```

缺陷品：

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_RELEASE_DEFECT'}" --once
```

---

## 20. ROS2 关节控制测试命令

### 20.1 查看 joint topic

```bash
ros2 topic list
```

应看到：

```text
/compact_cell/joint_cmd
/compact_cell/joint_status
```

监听关节状态：

```bash
ros2 topic echo /compact_cell/joint_status
```

### 20.2 单关节点动

R1 的 J1 正向转 10 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 +10'}" --once
```

R1 的 J1 反向转 10 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 -10'}" --once
```

R3 的 J6 正向转 20 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R3 J6 +20'}" --once
```

### 20.3 设置绝对角度

R1 的 J1 转到 30 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 =30'}" --once
```

R2 的 J4 转到 -45 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R2 J4 =-45'}" --once
```

### 20.4 六关节一次性设置

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 SET 0 20 -30 0 45 0'}" --once
```

含义：

```text
R1:
J1 = 0°
J2 = 20°
J3 = -30°
J4 = 0°
J5 = 45°
J6 = 0°
```

### 20.5 回零

R1 回零：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 HOME'}" --once
```

全部机械臂回零：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'ALL HOME'}" --once
```

---

## 21. 推荐完整测试顺序

搭好场景后，按以下顺序测试：

```text
1. 从 ROS2 终端启动 CoppeliaSim
2. 打开场景
3. 点击开始仿真
4. ros2 topic list
5. 确认出现 /compact_cell/status
6. 确认出现 /compact_cell/tool_cmd
7. 确认出现 /compact_cell/joint_cmd
8. 测 RESET_CELL
9. 测 R1_GRIPPER_OPEN
10. 测 R1_GRIPPER_CLOSE_BOX
11. 测 R1_ATTACH_BOX
12. 测 R1_RELEASE_BOX_ASSEMBLY
13. 测 R2_ATTACH_PCB
14. 测 R4_SCREW_START
15. 测 R5_RELEASE_GOOD
16. 测 R1 J1 +10
17. 测 R1 J1 -10
18. 保存稳定版本
```

---

## 22. 最终运行时脚本状态

最终场景中应启用：

```text
Product_Stage_Controller_60.lua
Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua
ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua
ROS2_Joint_Jog_Controller_R1_R5.lua
```

最终场景中应禁用：

```text
Step01_Create_Clean_Cell_60_GreyTable_RobotColor.lua
Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
Step03_Create_Process_Targets_60.lua
```

---

## 23. 保存场景

建议保存为：

```text
Five_CR5A_Cell_Stage03_ROS2_Joint_OK.ttt
```

或中文名：

```text
五台CR5A电控箱装配场景_ROS2关节控制完成版.ttt
```

保存前确认：

```text
1. 生成类脚本已禁用
2. 运行类脚本已启用
3. 目标点没有被重复生成
4. 夹爪没有重复对象
5. ROS2 topic 能正常出现
6. 工具动作能执行
7. 关节点动能执行
```

---

## 24. 后续给路径规划成员的接口

后续路径规划成员需要基于以下接口开发：

```text
机械臂对象：
/R1
/R2
/R3
/R4
/R5

末端点：
R1_gripper_tip
R2_vacuum_tip
R3_gripper_tip
R4_tool_tip
R5_gripper_tip

目标点：
/FiveCR5A_Cell/Targets/R1_Targets/...
/FiveCR5A_Cell/Targets/R2_Targets/...
/FiveCR5A_Cell/Targets/R3_Targets/...
/FiveCR5A_Cell/Targets/R4_Targets/...
/FiveCR5A_Cell/Targets/R5_Targets/...

工件对象：
/FiveCR5A_Cell/Parts/Box_Blank
/FiveCR5A_Cell/Parts/PCB_Supply
/FiveCR5A_Cell/Parts/Control_Module_Supply
/FiveCR5A_Cell/Parts/Terminal_Block_Supply
/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product
/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product

ROS2 工具命令：
/compact_cell/tool_cmd

ROS2 关节命令：
/compact_cell/joint_cmd
```

推荐运动逻辑：

```text
读取 APP 点
读取 TCP 点
控制机械臂关节运动，使 tip 到 APP
控制机械臂关节运动，使 tip 到 TCP
发送 attach / detach / screw 命令
控制机械臂回 APP
发送动作完成信号
```

---

## 25. 常见问题

### 25.1 simROS2 报 nav_msgs 缺失

报错：

```text
libnav_msgs__rosidl_typesupport_cpp.so: cannot open shared object file
```

解决：

```bash
sudo apt install ros-humble-nav-msgs
source /opt/ros/humble/setup.bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

最常见原因是没有从 ROS2 终端启动 CoppeliaSim。

### 25.2 ros2 topic list 只看到 /compact_cell/status

说明 publisher 成功，但 subscriber 可能没成功。  
应使用：

```text
ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua
```

因为 CoppeliaSim 的 simROS2 订阅回调需要全局函数名。

### 25.3 ROS2 发布命令一直 Waiting

如果出现：

```text
Waiting for at least 1 matching subscription(s)...
```

检查：

```text
1. CoppeliaSim 是否从 source 过 ROS2 的终端启动
2. 是否点击了开始仿真
3. ROS2 桥接脚本是否启用
4. ros2 topic info /compact_cell/tool_cmd 是否显示 Subscription count: 1
```

### 25.4 夹爪张开后视觉上断开

使用最终版本：

```text
Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
```

该版本已把 R1/R3/R5 改为一体式长夹指结构。

### 25.5 关节不动

检查：

```text
1. ROS2_Joint_Jog_Controller_R1_R5.lua 是否启用
2. CoppeliaSim 是否正在仿真
3. /compact_cell/joint_cmd 是否有订阅
4. 机械臂关节名称是否被脚本识别
5. 控制台是否打印 R1 J1 -> joint1 等信息
```

如果关节名称识别错，需要根据实际机械臂模型的关节名称调整脚本中的关节查找逻辑。

---

## 26. 当前阶段结论

当前场景已经完成：

```text
场景搭建
末端工具
产品阶段控制
ROS2 通信
工具动作控制
工件绑定释放
APP/TCP 工艺目标点
机械臂关节点动控制
```

后续还需要开发的是：

```text
逆运动学
自动路径规划
APP/TCP 自动运动
避障
多机械臂调度
完整自动流程
```

因此，本版本可以作为：

```text
1. 多机械臂装配仿真基础场景
2. ROS2 通信测试平台
3. 机械臂路径规划小组的目标点接口
4. 装配流程可视化平台
5. 多机械臂协同调度开发基础
```
