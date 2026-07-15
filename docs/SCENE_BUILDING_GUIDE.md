# 五台 CR5A 协同装配场景搭建完整指南

> **适用对象**：负责场景搭建、机械臂控制、ROS2 通信和调度的全体小组成员  
> **场景名称**：Five CR5A Electrical Control Box Assembly Cell  
> **仿真平台**：CoppeliaSim Edu V4.10 + ROS2 Humble  
> **机械臂型号**：DOBOT CR5A × 5

---

## 目录

1. [场景总览](#1-场景总览)
2. [仓库文件结构](#2-仓库文件结构)
3. [核心脚本说明](#3-核心脚本说明)
4. [场景对象路径规范](#4-场景对象路径规范)
5. [机械臂分工与末端点](#5-机械臂分工与末端点)
6. [工序目标点完整参考](#6-工序目标点完整参考)
7. [ROS2 通信接口](#7-ros2-通信接口)
8. [CoppeliaSim 内部 Signal 接口](#8-coppeliasim-内部-signal-接口)
9. [环境搭建与启动流程](#9-环境搭建与启动流程)
10. [典型完整工艺流程](#10-典型完整工艺流程)
11. [各成员分工与开发步骤](#11-各成员分工与开发步骤)
12. [调试命令汇总](#12-调试命令汇总)
13. [控制成员开发指南](#13-控制成员开发指南)
14. [常见问题排查](#14-常见问题排查)

---

## 1. 场景总览

### 1.1 场景用途

本场景模拟一个"小型电控箱自动装配单元"，五台 CR5A 机械臂分布在两个圆形减震工作台周围，通过供料区、装配区、检测/锁付区和双传送带，完成从零件到成品的完整装配流程。

### 1.2 整体流程

```text
供料区（箱体、PCB、控制模块、端子排）
         ↓
R1 抓取箱体 → 放到装配区
         ↓
R2 安装 PCB 板到箱体内
         ↓
R3 安装控制模块到 PCB 上方
         ↓
R1 安装端子排到箱体内
         ↓
R3 将完整装配体搬运到检测区
         ↓
固定相机检测（合格 / 缺陷）
         ↓
R4 对端子螺钉进行锁付
         ↓
R5 根据检测结果分拣：
   ├─ 合格 → 合格品传送带
   └─ 缺陷 → 缺陷品传送带
```

### 1.3 场景布局示意

```text
                    相机立柱
                       │
    ┌──────────────────┼──────────────────┐
    │                                      │
    │   R1(箱体+端子)    R3(模块+转移)       │
    │       │                │              │
    │   ┌───┴────┐     ┌────┴───┐          │
    │   │ 左工作台 │     │右工作台  │  R4     │
    │   │ 装配区  │     │检测/锁付│(锁螺钉)  │
    │   └────────┘     └────────┘          │
    │       R2(PCB)          │              │
    │                     R5(分拣)           │
    │                        │              │
    │              ┌─────────┴─────────┐    │
    │              │                   │    │
    │         合格品传送带        缺陷品传送带 │
    └──────────────────────────────────────┘
```

---

## 2. 仓库文件结构

```text
cr5_assembly_team/
├── README.md                              # 项目总览
├── scenes/                                # 🎯 场景文件（场景搭建核心交付物）
│   ├── compact_cell.ttt                   # CoppeliaSim 场景文件（含机械臂+场景）
│   ├── main_cell_generator.lua            # 主场景生成脚本
│   └── ros2_all_robot_bridge.lua          # ROS2 通信桥接脚本
├── docs/                                  # 📖 文档
│   ├── SCENE_BUILDING_GUIDE.md            # 本文档 — 场景搭建完整指南
│   ├── Five_CR5A_Cell_Control_Interface.md # 五臂控制接口详细文档
│   ├── INTERFACES.md                      # 系统接口规范（通用）
│   ├── PROJECT_PLAN.md                    # 项目计划
│   ├── WORKSPACE_DESIGN.md                # 工作空间设计方案
│   ├── SETUP_GUIDE.md                     # 团队环境搭建指南
│   ├── TEAM_WORKFLOW.md                   # 团队协作流程
│   └── R4_QUALITY_SORTING.md             # R4 质量分拣说明
├── configs/                               # ⚙️ 配置文件
│   ├── robots.yaml                        # 机械臂配置
│   ├── points.yaml                        # 点位配置
│   ├── product_types.yaml                 # 产品类型配置
│   └── scheduler.yaml                     # 调度器配置
├── sim_bridge/                            # 🔗 CoppeliaSim 通信桥接（Python）
├── robot_control/                         # 🦾 机械臂运动控制
├── scheduler/                             # 📋 任务调度器
├── interfaces/                            # 📡 模块间接口定义
├── mock/                                  # 🧪 Mock 模块（开发测试用）
├── app/                                   # 🖥️ 数据看板
├── data/                                  # 📊 数据文件
│   └── orders/demo_orders.json            # 演示订单
├── src/                                   # 📦 ROS2 源码（DOBOT 驱动）
└── models/                                # 🧊 3D 模型文件
```

### 场景搭建核心交付物

| 文件 | 位置 | 说明 |
|---|---|---|
| `main_cell_generator.lua` | `scenes/` | 场景本体脚本，挂在 CoppeliaSim Dummy 上 |
| `ros2_all_robot_bridge.lua` | `scenes/` | ROS2 通信脚本，挂在 CoppeliaSim Dummy 上 |
| `Five_CR5A_Cell_Control_Interface.md` | `docs/` | 五臂系统的控制接口完整参考 |

---

## 3. 核心脚本说明

### 3.1 Main_Cell_Generator（主场景生成器）

**挂载位置**：场景中任意独立 Dummy 对象的 child script  
**文件**：`scenes/main_cell_generator.lua`

**功能清单**：

| # | 功能 |
|---|---|
| 1 | 自动生成 `/FiveCR5A_Cell` 主场景树 |
| 2 | 自动生成地面、两个圆形减震工作台、R1~R5 机械臂安装基座 |
| 3 | 自动生成供料区（箱体、端子排、PCB、控制模块）|
| 4 | 自动生成装配区和检测/锁付区 |
| 5 | 自动生成固定立柱相机和检测视野区域 |
| 6 | 自动生成合格品/缺陷品两条传送带 |
| 7 | 自动生成供料零件和产品模板 |
| 8 | 自动摆放 R1~R5 机械臂到对应基座位置 |
| 9 | 自动冻结机械臂动力学 |
| 10 | 自动生成 R1~R5 末端 tip 点 |
| 11 | 自动生成所有工序目标点 Dummy |
| 12 | 根据内部 signal 管理产品装配阶段显示/隐藏 |
| 13 | 根据检测结果改变相机检测区域颜色 |
| 14 | 根据分拣结果驱动传送带产品运动 |

**关键开关（首次运行后建议修改）**：

```lua
-- 首次从零生成场景时设为 true，生成成功后改为 false
local REBUILD_SCENE_ON_START = false
local RESET_TIPS_ON_START = false
local RESET_TARGETS_ON_START = false

-- 始终保持开启
local AUTO_PLACE_ROBOTS = true
local FREEZE_ROBOT_DYNAMICS_ON_START = true
```

### 3.2 ROS2_All_Robot_Bridge（ROS2 通信桥接器）

**挂载位置**：场景中任意独立 Dummy 对象的 child script  
**文件**：`scenes/ros2_all_robot_bridge.lua`

**功能清单**：

| # | 功能 |
|---|---|
| 1 | 订阅统一主场景指令 `/compact_cell/main_cmd` |
| 2 | 订阅 R1~R5 独立指令 `/compact_cell/r1_cmd` ~ `/compact_cell/r5_cmd` |
| 3 | 订阅兼容总入口 `/compact_cell/cmd`（自动按前缀分发）|
| 4 | 将 ROS2 命令转换为 CoppeliaSim 内部 signal |
| 5 | 向 `/compact_cell/status` 发布当前执行状态 |

---

## 4. 场景对象路径规范

所有场景对象统一挂在 `/FiveCR5A_Cell` 下：

```text
/FiveCR5A_Cell
  ├─ Ground_Group          # 地面
  ├─ Tables                # 两个圆形减震工作台
  ├─ RobotBases            # R1~R5 安装基座
  ├─ Areas                 # 供料区、装配区、检测区
  ├─ Parts                 # 零件和产品模板
  ├─ Conveyors             # 合格品/缺陷品传送带
  ├─ Sensors               # 固定相机及检测视野
  └─ Targets               # 所有工序目标点（APP + TCP）
```

### 4.1 零件对象路径

| 对象 | 路径 | 用途 |
|---|---|---|
| 箱体毛坯 | `/FiveCR5A_Cell/Parts/Box_Blank` | R1 抓取上料 |
| PCB 供料 | `/FiveCR5A_Cell/Parts/PCB_Supply` | R2 抓取安装 |
| 控制模块供料 | `/FiveCR5A_Cell/Parts/Control_Module_Supply` | R3 抓取安装 |
| 端子排供料 | `/FiveCR5A_Cell/Parts/Terminal_Block_Supply` | R1 抓取安装 |
| 装配区产品 | `/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product` | 装配阶段产品模板 |
| 检测区产品 | `/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product` | 检测/锁付/分拣阶段产品模板 |

### 4.2 区域对象路径

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

### 4.3 传送带对象路径

| 传送带 | 路径 |
|---|---|
| 合格品传送带 | `/FiveCR5A_Cell/Conveyors/Good_Conveyor` |
| 缺陷品传送带 | `/FiveCR5A_Cell/Conveyors/Defect_Conveyor` |

---

## 5. 机械臂分工与末端点

### 5.1 机械臂分工

| 机械臂 | 任务 | 使用工具 |
|---|---|---|
| R1 | 箱体上料 + 端子排安装 | 夹爪 |
| R2 | PCB 安装 | 夹爪 |
| R3 | 控制模块安装 + 完整装配体转移到检测区 | 夹爪 |
| R4 | 端子螺钉锁付 | 螺丝刀 |
| R5 | 合格品/缺陷品分拣 | 夹爪 |

### 5.2 末端 tip 点

tip 点挂在各机械臂 `Link6_visual` 下：

| 机械臂 | tip 名称 | tip 路径（示例） |
|---|---|---|
| R1 | `R1_gripper_tip` | `/R1/.../Link6_visual/R1_gripper_tip` |
| R2 | `R2_gripper_tip` | `/R2/.../Link6_visual/R2_gripper_tip` |
| R3 | `R3_gripper_tip` | `/R3/.../Link6_visual/R3_gripper_tip` |
| R4 | `R4_tool_tip` | `/R4/.../Link6_visual/R4_tool_tip` |
| R5 | `R5_gripper_tip` | `/R5/.../Link6_visual/R5_gripper_tip` |

---

## 6. 工序目标点完整参考

所有目标点位于 `/FiveCR5A_Cell/Targets/` 下，分为 5 组 + 相机组。

**命名规则**：
- `APP`（Approach）= 接近点，位于目标上方，用于安全接近
- `TCP` = 实际抓取、放置或操作点

**控制逻辑**：`APP → TCP → 执行动作 → APP`

### 6.1 R1 目标点（箱体上料 + 端子排安装）

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| R1_HOME_REF | `.../R1_Targets/R1_HOME_REF` | (-1.47, 0.67, 0.80) | R1 参考初始位 |
| R1_BOX_PICK_APP | `.../R1_Targets/R1_BOX_PICK_APP` | (-1.80, 0.35, 0.55) | 箱体抓取接近点 |
| R1_BOX_PICK_TCP | `.../R1_Targets/R1_BOX_PICK_TCP` | (-1.80, 0.35, 0.30) | 箱体抓取点 |
| R1_BOX_PLACE_APP | `.../R1_Targets/R1_BOX_PLACE_APP` | (-1.15, 0.20, 0.55) | 箱体放置接近点 |
| R1_BOX_PLACE_TCP | `.../R1_Targets/R1_BOX_PLACE_TCP` | (-1.15, 0.20, 0.30) | 箱体放置点 |
| R1_TERMINAL_PICK_APP | `.../R1_Targets/R1_TERMINAL_PICK_APP` | (-1.90, 0.10, 0.45) | 端子排抓取接近点 |
| R1_TERMINAL_PICK_TCP | `.../R1_Targets/R1_TERMINAL_PICK_TCP` | (-1.90, 0.10, 0.24) | 端子排抓取点 |
| R1_TERMINAL_PLACE_APP | `.../R1_Targets/R1_TERMINAL_PLACE_APP` | (-1.09, 0.13, 0.50) | 端子排安装接近点 |
| R1_TERMINAL_PLACE_TCP | `.../R1_Targets/R1_TERMINAL_PLACE_TCP` | (-1.09, 0.13, 0.34) | 端子排安装点 |

### 6.2 R2 目标点（PCB 安装）

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| R2_HOME_REF | `.../R2_Targets/R2_HOME_REF` | (-1.55, -0.15, 0.80) | R2 参考初始位 |
| R2_PCB_PICK_APP | `.../R2_Targets/R2_PCB_PICK_APP` | (-1.28, -0.28, 0.45) | PCB 抓取接近点 |
| R2_PCB_PICK_TCP | `.../R2_Targets/R2_PCB_PICK_TCP` | (-1.28, -0.28, 0.22) | PCB 抓取点 |
| R2_PCB_PLACE_APP | `.../R2_Targets/R2_PCB_PLACE_APP` | (-1.15, 0.20, 0.50) | PCB 安装接近点 |
| R2_PCB_PLACE_TCP | `.../R2_Targets/R2_PCB_PLACE_TCP` | (-1.15, 0.20, 0.29) | PCB 安装点 |

### 6.3 R3 目标点（控制模块安装 + 产品转移）

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| R3_HOME_REF | `.../R3_Targets/R3_HOME_REF` | (-0.55, 0.28, 0.80) | R3 参考初始位 |
| R3_MODULE_PICK_APP | `.../R3_Targets/R3_MODULE_PICK_APP` | (-0.85, -0.05, 0.45) | 控制模块抓取接近点 |
| R3_MODULE_PICK_TCP | `.../R3_Targets/R3_MODULE_PICK_TCP` | (-0.85, -0.05, 0.24) | 控制模块抓取点 |
| R3_MODULE_PLACE_APP | `.../R3_Targets/R3_MODULE_PLACE_APP` | (-1.12, 0.22, 0.50) | 控制模块安装接近点 |
| R3_MODULE_PLACE_TCP | `.../R3_Targets/R3_MODULE_PLACE_TCP` | (-1.12, 0.22, 0.34) | 控制模块安装点 |
| R3_PRODUCT_PICK_APP | `.../R3_Targets/R3_PRODUCT_PICK_APP` | (-1.15, 0.20, 0.60) | 装配体抓取接近点 |
| R3_PRODUCT_PICK_TCP | `.../R3_Targets/R3_PRODUCT_PICK_TCP` | (-1.15, 0.20, 0.34) | 装配体抓取点 |
| R3_PRODUCT_PLACE_INSPECTION_APP | `.../R3_Targets/R3_PRODUCT_PLACE_INSPECTION_APP` | (0.15, 0.05, 0.60) | 检测区放置接近点 |
| R3_PRODUCT_PLACE_INSPECTION_TCP | `.../R3_Targets/R3_PRODUCT_PLACE_INSPECTION_TCP` | (0.15, 0.05, 0.34) | 检测区放置点 |

### 6.4 R4 目标点（螺钉锁付）

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| R4_HOME_REF | `.../R4_Targets/R4_HOME_REF` | (0.58, 0.25, 0.80) | R4 参考初始位 |
| R4_SCREW_APP | `.../R4_Targets/R4_SCREW_APP` | (0.21, -0.02, 0.55) | 螺钉接近点 |
| R4_SCREW_TCP | `.../R4_Targets/R4_SCREW_TCP` | (0.21, -0.02, 0.36) | 螺钉接触点 |
| R4_SCREW_PRESS | `.../R4_Targets/R4_SCREW_PRESS` | (0.21, -0.02, 0.33) | 模拟下压锁付点 |

### 6.5 R5 目标点（分拣）

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| R5_HOME_REF | `.../R5_Targets/R5_HOME_REF` | (0.15, -0.50, 0.80) | R5 参考初始位 |
| R5_PRODUCT_PICK_APP | `.../R5_Targets/R5_PRODUCT_PICK_APP` | (0.15, 0.05, 0.60) | 检测区产品抓取接近点 |
| R5_PRODUCT_PICK_TCP | `.../R5_Targets/R5_PRODUCT_PICK_TCP` | (0.15, 0.05, 0.34) | 检测区产品抓取点 |
| R5_GOOD_PLACE_APP | `.../R5_Targets/R5_GOOD_PLACE_APP` | (0.65, -1.10, 0.62) | 合格品放置接近点 |
| R5_GOOD_PLACE_TCP | `.../R5_Targets/R5_GOOD_PLACE_TCP` | (0.65, -1.10, 0.42) | 合格品放置点 |
| R5_DEFECT_PLACE_APP | `.../R5_Targets/R5_DEFECT_PLACE_APP` | (-0.35, -1.12, 0.62) | 缺陷品放置接近点 |
| R5_DEFECT_PLACE_TCP | `.../R5_Targets/R5_DEFECT_PLACE_TCP` | (-0.35, -1.12, 0.42) | 缺陷品放置点 |

### 6.6 相机检测目标点

| 目标点 | 路径 | 位置 (x, y, z) | 用途 |
|---|---|---|---|
| CAMERA_INSPECTION_CENTER | `.../Sensor_Targets/CAMERA_INSPECTION_CENTER` | (0.15, 0.05, 0.55) | 检测中心参考点 |

---

## 7. ROS2 通信接口

### 7.1 Topic 列表

| Topic | 消息类型 | 说明 |
|---|---|---|
| `/compact_cell/main_cmd` | `std_msgs/String` | 主场景命令 |
| `/compact_cell/r1_cmd` | `std_msgs/String` | R1 机械臂命令 |
| `/compact_cell/r2_cmd` | `std_msgs/String` | R2 机械臂命令 |
| `/compact_cell/r3_cmd` | `std_msgs/String` | R3 机械臂命令 |
| `/compact_cell/r4_cmd` | `std_msgs/String` | R4 机械臂命令 |
| `/compact_cell/r5_cmd` | `std_msgs/String` | R5 机械臂命令 |
| `/compact_cell/cmd` | `std_msgs/String` | 总入口，自动按前缀分发 |
| `/compact_cell/status` | `std_msgs/String` | 状态发布 |

### 7.2 命令定义

#### 主场景命令

| 命令 | 作用 |
|---|---|
| `RESET_CELL` | 重置场景状态 |
| `SHOW_ASSEMBLY_SHELL` | 装配区显示箱体 |
| `SHOW_ASSEMBLY_PCB` | 装配区显示箱体+PCB |
| `SHOW_ASSEMBLY_MODULE` | 装配区显示箱体+PCB+控制模块 |
| `SHOW_ASSEMBLY_FULL` | 装配区显示完整装配体 |
| `SHOW_INSPECTION_FULL` | 检测区显示完整装配体 |
| `CAMERA_GOOD` | 相机检测区域变为合格（绿色）|
| `CAMERA_DEFECT` | 相机检测区域变为缺陷（红色）|
| `CONVEYOR_GOOD` | 启动合格品传送带 |
| `CONVEYOR_DEFECT` | 启动缺陷品传送带 |

#### R1 命令

| 命令 | 作用 |
|---|---|
| `R1_READY` | R1 就绪 |
| `R1_BOX_PLACED` | R1 已把箱体放到装配区 |
| `R1_TERMINAL_PLACED` | R1 已把端子排安装到装配区 |

#### R2 命令

| 命令 | 作用 |
|---|---|
| `R2_READY` | R2 就绪 |
| `R2_PCB_PLACED` | R2 已把 PCB 安装到箱体内 |

#### R3 命令

| 命令 | 作用 |
|---|---|
| `R3_READY` | R3 就绪 |
| `R3_MODULE_PLACED` | R3 已把控制模块安装到箱体内 |
| `R3_PRODUCT_TO_INSPECTION` | R3 已把完整装配体放到检测区 |

#### R4 命令

| 命令 | 作用 |
|---|---|
| `R4_READY` | R4 就绪 |
| `R4_SCREW_DONE` | R4 已完成螺钉锁付 |

#### R5 命令

| 命令 | 作用 |
|---|---|
| `R5_READY` | R5 就绪 |
| `R5_SORT_GOOD_DONE` | R5 已把产品放到合格品传送带 |
| `R5_SORT_DEFECT_DONE` | R5 已把产品放到缺陷品传送带 |

### 7.3 总入口使用

所有命令都可以通过 `/compact_cell/cmd` 统一发送，脚本会自动按前缀（`R1_`、`R2_`...）分发到对应处理函数：

```bash
# 通过总入口发送（推荐调试方式）
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

---

## 8. CoppeliaSim 内部 Signal 接口

ROS2 桥接脚本将 ROS2 命令转换为以下内部 signal。

### 8.1 产品状态 signal：`cell_product_state`

```lua
sim.setStringSignal('cell_product_state', '<value>')
```

| 值 | 场景效果 |
|---|---|
| `reset` | 重置场景 |
| `assembly_shell` | 装配区显示箱体 |
| `assembly_pcb` | 装配区显示箱体 + PCB |
| `assembly_module` | 装配区显示箱体 + PCB + 控制模块 |
| `assembly_full` | 装配区显示完整产品 |
| `inspection_full` | 检测区显示完整产品（装配区清空）|
| `camera_good` | 相机区域变绿色 |
| `camera_defect` | 相机区域变红色 |

### 8.2 传送带状态 signal：`cell_conveyor_state`

```lua
sim.setStringSignal('cell_conveyor_state', '<value>')
```

| 值 | 场景效果 |
|---|---|
| `good` | 合格品传送带开始运动 |
| `defect` | 缺陷品传送带开始运动 |

### 8.3 螺钉状态 signal：`cell_screw_state`

```lua
sim.setStringSignal('cell_screw_state', 'done')
```

### 8.4 机械臂 ROS 指令 signal（R1~R5 独立）

桥接脚本会为每台机械臂设置对应的 signal，可供各机械臂控制脚本读取：

```lua
-- R1 收到的 ROS 命令
sim.getStringSignal('r1_ros_cmd')
sim.getStringSignal('r2_ros_cmd')
sim.getStringSignal('r3_ros_cmd')
sim.getStringSignal('r4_ros_cmd')
sim.getStringSignal('r5_ros_cmd')
```

---

## 9. 环境搭建与启动流程

### 9.1 前提条件

- Ubuntu 22.04
- ROS2 Humble
- CoppeliaSim Edu V4.10
- DOBOT ROS2 驱动包（仓库 `src/DOBOT_6Axis_ROS2_V4/`）

### 9.2 首次启动步骤

```bash
# 1. Source ROS2 环境
source /opt/ros/humble/setup.bash

# 2. （可选）Source 自己的 ROS2 工作空间
source ~/dobot_ws/install/setup.bash

# 3. 进入 CoppeliaSim 目录并启动
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh

# 4. 在 CoppeliaSim 中：
#    - 打开场景文件 scenes/compact_cell.ttt
#    - 或新建场景，手动添加两个 Dummy 对象
#    - 将 main_cell_generator.lua 挂在 Dummy 1 的 child script
#    - 将 ros2_all_robot_bridge.lua 挂在 Dummy 2 的 child script
#    - 首次运行确保开关：REBUILD_SCENE_ON_START = true
#    - 启动仿真
```

> ⚠️ **重要**：必须从已 source ROS2 环境的终端启动 CoppeliaSim，否则 `simROS2` 插件无法加载。

### 9.3 首次运行后的配置

首次成功生成场景后，修改 `main_cell_generator.lua` 中的开关：

```lua
local REBUILD_SCENE_ON_START = false
local RESET_TIPS_ON_START = false
local RESET_TARGETS_ON_START = false
```

这样后续手动调整场景时不会每次都被重置。

### 9.4 首次验证步骤

```bash
# 终端 1：查看状态
ros2 topic echo /compact_cell/status

# 终端 2：发送命令
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
# 正常情况应看到状态返回
```

---

## 10. 典型完整工艺流程

### 10.1 初始化

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

场景恢复到初始状态：供料区有零件，装配区/检测区/传送带为空。

### 10.2 R1 箱体上料

```text
R1_gripper_tip → R1_BOX_PICK_APP → R1_BOX_PICK_TCP
  → attach Box_Blank → R1_BOX_PICK_APP
  → R1_BOX_PLACE_APP → R1_BOX_PLACE_TCP
  → detach Box_Blank 到装配区
```

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

### 10.3 R2 安装 PCB

```text
R2_gripper_tip → R2_PCB_PICK_APP → R2_PCB_PICK_TCP
  → attach PCB_Supply → R2_PCB_PICK_APP
  → R2_PCB_PLACE_APP → R2_PCB_PLACE_TCP
  → detach PCB 到装配区箱体内
```

```bash
ros2 topic pub /compact_cell/r2_cmd std_msgs/msg/String "{data: 'R2_PCB_PLACED'}" --once
```

### 10.4 R3 安装控制模块

```text
R3_gripper_tip → R3_MODULE_PICK_APP → R3_MODULE_PICK_TCP
  → attach Control_Module_Supply → R3_MODULE_PICK_APP
  → R3_MODULE_PLACE_APP → R3_MODULE_PLACE_TCP
  → detach 控制模块到 PCB 上方
```

```bash
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_MODULE_PLACED'}" --once
```

### 10.5 R1 安装端子排

```text
R1_gripper_tip → R1_TERMINAL_PICK_APP → R1_TERMINAL_PICK_TCP
  → attach Terminal_Block_Supply → R1_TERMINAL_PICK_APP
  → R1_TERMINAL_PLACE_APP → R1_TERMINAL_PLACE_TCP
  → detach 端子排到箱体内
```

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_TERMINAL_PLACED'}" --once
```

### 10.6 R3 转移完整装配体到检测区

```text
R3_gripper_tip → R3_PRODUCT_PICK_APP → R3_PRODUCT_PICK_TCP
  → attach Assembly_ControlBox_Product → R3_PRODUCT_PICK_APP
  → R3_PRODUCT_PLACE_INSPECTION_APP → R3_PRODUCT_PLACE_INSPECTION_TCP
  → detach 产品到检测区
```

```bash
ros2 topic pub /compact_cell/r3_cmd std_msgs/msg/String "{data: 'R3_PRODUCT_TO_INSPECTION'}" --once
```

### 10.7 相机检测

```bash
# 合格
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_GOOD'}" --once
# 或缺陷
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_DEFECT'}" --once
```

### 10.8 R4 锁螺钉

```text
R4_tool_tip → R4_SCREW_APP → R4_SCREW_TCP → R4_SCREW_PRESS
  → 模拟旋转/下压 → R4_SCREW_APP
```

```bash
ros2 topic pub /compact_cell/r4_cmd std_msgs/msg/String "{data: 'R4_SCREW_DONE'}" --once
```

### 10.9 R5 分拣

**合格品流程**：

```text
R5_gripper_tip → R5_PRODUCT_PICK_APP → R5_PRODUCT_PICK_TCP
  → attach Inspection_ControlBox_Product → R5_PRODUCT_PICK_APP
  → R5_GOOD_PLACE_APP → R5_GOOD_PLACE_TCP
  → detach 产品到合格品传送带入口
```

```bash
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_GOOD_DONE'}" --once
```

**缺陷品流程**：

```text
R5_gripper_tip → R5_PRODUCT_PICK_APP → R5_PRODUCT_PICK_TCP
  → attach → R5_PRODUCT_PICK_APP
  → R5_DEFECT_PLACE_APP → R5_DEFECT_PLACE_TCP
  → detach 产品到缺陷品传送带入口
```

```bash
ros2 topic pub /compact_cell/r5_cmd std_msgs/msg/String "{data: 'R5_SORT_DEFECT_DONE'}" --once
```

### 10.10 推荐总调度顺序

```text
RESET_CELL
  → R1 箱体上料 → R1_BOX_PLACED
  → R2 PCB 安装 → R2_PCB_PLACED
  → R3 控制模块安装 → R3_MODULE_PLACED
  → R1 端子排安装 → R1_TERMINAL_PLACED
  → R3 产品转移 → R3_PRODUCT_TO_INSPECTION
  → 相机检测 → CAMERA_GOOD / CAMERA_DEFECT
  → R4 锁螺钉 → R4_SCREW_DONE
  → R5 分拣（根据检测结果）→ R5_SORT_GOOD_DONE / R5_SORT_DEFECT_DONE
```

---

## 11. 各成员分工与开发步骤

### 11.1 分工表

| 角色 | 负责内容 | 涉及文件 |
|---|---|---|
| 场景搭建 (你) | `Main_Cell_Generator`、目标点、对象路径、布局 | `scenes/main_cell_generator.lua`、`scenes/compact_cell.ttt` |
| ROS2 接口 | `ROS2_All_Robot_Bridge`、topic、命令格式 | `scenes/ros2_all_robot_bridge.lua` |
| R1 控制 | 箱体上料、端子排安装 | 新建 R1 控制脚本 |
| R2 控制 | PCB 安装 | 新建 R2 控制脚本 |
| R3 控制 | 控制模块安装、产品转移 | 新建 R3 控制脚本 |
| R4 控制 | 螺钉锁付动作 | 新建 R4 控制脚本 |
| R5 控制 | 检测后分拣 | 新建 R5 控制脚本 |
| 调度 | R1~R5 执行顺序和互锁逻辑 | Python 调度模块 |
| 软件集成 | 看板、订单输入、日志 | `app/`、`interfaces/` |

### 11.2 控制成员推荐开发步骤

1. **验证 ROS2 通信**：`ros2 topic echo /compact_cell/status`，发送 `RESET_CELL`
2. **验证场景状态变化**：依次发送 `SHOW_ASSEMBLY_SHELL` 等观察场景变化
3. **读取单个关节**：读取 joints、tip、target，控制某一个关节点动
4. **实现 APP→TCP→APP 简单运动**：三个点的基本动作
5. **加入 attach/detach**：确认工件可以跟随末端运动
6. **加入 ROS2 动作完成反馈**：动作完成后发布对应命令

---

## 12. 调试命令汇总

### 12.1 查看状态

```bash
ros2 topic echo /compact_cell/status
ros2 topic list
```

### 12.2 重置场景

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

### 12.3 显示装配阶段

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_SHELL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_PCB'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_MODULE'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_ASSEMBLY_FULL'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_INSPECTION_FULL'}" --once
```

### 12.4 相机和传送带

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_GOOD'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CAMERA_DEFECT'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CONVEYOR_GOOD'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'CONVEYOR_DEFECT'}" --once
```

### 12.5 R1~R5 完成信号

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

## 13. 控制成员开发指南

### 13.1 读取目标点位姿

```lua
local target = sim.getObject('/FiveCR5A_Cell/Targets/R1_Targets/R1_BOX_PICK_TCP')
local pos = sim.getObjectPosition(target, -1)     -- 世界坐标
local ori = sim.getObjectOrientation(target, -1)  -- 世界姿态(Euler)
```

### 13.2 查找末端 tip

```lua
-- 方法 1：通过已知路径
local tip = sim.getObject('/R1/.../Link6_visual/R1_gripper_tip')

-- 方法 2：在 /R1 树下递归查找
-- （参考 main_cell_generator.lua 中的 findInTreeByName 函数）
```

### 13.3 抓取（运动学绑定）

```lua
local part = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank')
local tip = sim.getObject('/R1/.../R1_gripper_tip')
sim.setObjectParent(part, tip, true)  -- true=保持世界位姿
```

### 13.4 释放

```lua
local part = sim.getObject('/FiveCR5A_Cell/Parts/Box_Blank')
local partsRoot = sim.getObject('/FiveCR5A_Cell/Parts')
sim.setObjectParent(part, partsRoot, true)
sim.setObjectPosition(part, -1, {-1.15, 0.20, 0.216})  -- 放置位置
```

### 13.5 发送内部 signal

```lua
sim.setStringSignal('cell_product_state', 'assembly_shell')
```

或由 ROS2 发送：

```bash
ros2 topic pub /compact_cell/r1_cmd std_msgs/msg/String "{data: 'R1_BOX_PLACED'}" --once
```

---

## 14. 常见问题排查

| 问题 | 可能原因 | 解决方法 |
|---|---|---|
| `simROS2` 加载失败 | 未从 ROS2 终端启动 | `source /opt/ros/humble/setup.bash` 后从终端启动 CoppeliaSim |
| 机械臂启动后乱飞 | 动力学未冻结 | 确保 `FREEZE_ROBOT_DYNAMICS_ON_START = true` |
| tip 点找不到 | 首次未生成或路径不对 | 设置 `RESET_TIPS_ON_START = true` 重新生成 |
| 目标点位置不对 | 手动移动过但开关未开 | 设置 `RESET_TARGETS_ON_START = true` 重置 |
| ROS2 topic 无响应 | 桥接脚本未运行 | 检查 ROS2_All_Robot_Bridge 脚本是否正确挂载 |
| 传送带产品不移动 | 未发送正确的 conveyor signal | 检查 `cell_conveyor_state` signal |
| 装配区不显示产品 | product state signal 未触发 | 按顺序发送 `SHOW_ASSEMBLY_*` 命令 |

---

## 附录 A：场景参数速查

| 参数 | 值 |
|---|---|
| 工作台高度 | 0.12 m |
| 工作台半径 | 0.86 m |
| 装配区中心 | (-1.15, 0.20) |
| 检测区中心 | (0.15, 0.05) |
| 传送带总长 | ~1.25 m（合格品）/ ~1.20 m（缺陷品）|
| 传送带速度 | 0.18 m/s |
| 箱体尺寸 | 0.35 × 0.25 × 0.12 m |
| PCB 尺寸 | 0.24 × 0.16 × 0.008 m |

## 附录 B：颜色定义

| 对象 | RGB |
|---|---|
| 地面 | (0.78, 0.78, 0.78) |
| 工作台 | (0.60, 0.72, 0.38) |
| 供料/装配/检测区 | (0.88, 0.84, 0.68) |
| 箱体 | (0.62, 0.62, 0.62) |
| PCB 板 | (0.00, 0.45, 0.18) |
| 控制模块 | (0.08, 0.12, 0.18) |
| 端子排 | (0.92, 0.92, 0.88) |
| 相机检测区 | (0.75, 0.92, 1.00) 正常 / (0.20, 0.90, 0.25) 合格 / (0.95, 0.15, 0.10) 缺陷 |
| 目标点 Dummy | (1.00, 0.72, 0.20) |

---

> **文档维护**：场景搭建负责人  
> **最后更新**：2026-07-15  
> **对应版本**：Five CR5A Cell v1.0
