# 五台 CR5A 电控箱装配场景 — 快速上手指南

> 5 分钟了解场景结构、搭建流程和关键接口。

---

## 1. 场景是什么

一个在 CoppeliaSim 中运行的**五台 CR5A 机械臂协同装配仿真场景**，通过 ROS2 与外部 Python 控制程序通信。

```
/FiveCR5A_Cell          ← 场景根节点
  ├─ Ground_Group       地面
  ├─ Tables             两个圆形减震工作台
  ├─ RobotBases         五个机械臂基座
  ├─ Areas              供料区/装配区/检测区
  ├─ Parts              工件（箱体/PCB/控制模块/端子排）
  ├─ Conveyors          合格品传送带 + 缺陷品传送带
  ├─ Sensors            固定立柱相机
  └─ Targets            APP/TCP 目标点（约35个）
```

## 2. 五台机械臂分工

| 机械臂 | 干什么 | 用什么工具 |
|--------|--------|-----------|
| R1 | 抓箱体放到装配区 + 抓端子排安装 | 宽口夹爪 `R1T` |
| R2 | 吸 PCB 放入箱体 | 吸盘 `R2T` |
| R3 | 安装控制模块 + 搬完整产品到检测区 | 宽口夹爪 `R3T` |
| R4 | 锁端子排螺钉 | 电动螺丝刀 `R4T` |
| R5 | 检测后分拣（合格→合格品传送带，缺陷→缺陷品传送带） | 宽口夹爪 `R5T` |

完整流程：R1 箱体 → R2 PCB → R3 模块 → R1 端子排 → R3 搬运 → 相机检测 → R4 锁付 → R5 分拣

## 3. 场景搭建三步骤

### 第一步：准备机械臂
场景中需要已有五台机械臂，根对象名称必须是 `R1` `R2` `R3` `R4` `R5`。

### 第二步：依次运行三个生成脚本（用后禁用）

| 顺序 | 脚本 | 做什么 |
|------|------|--------|
| 1 | `Step01_Create_Clean_Cell_60_GreyTable_RobotColor.lua` | 生成地面、工作台、供料区、装配区、检测区、传送带、相机、60%缩放的工件 |
| 2 | `Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua` | 创建并安装 R1/R3/R5 夹爪、R2 吸盘、R4 螺丝刀 |
| 3 | `Step03_Create_Process_Targets_60.lua` | 创建 R1~R5 的 APP/TCP 工艺目标点 |

每个脚本：新建 Dummy → 添加 Non-threaded child script → 粘贴代码 → 运行一次 → 禁用。

### 第三步：启用四个运行时脚本（一直开启）

| 脚本 | 负责什么 |
|------|---------|
| `Product_Stage_Controller_60.lua` | 按装配阶段显示/隐藏工件 |
| `Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua` | 夹爪开合、吸盘吸附、螺丝刀旋转、工件绑定释放 |
| `ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua` | ROS2 工艺命令桥接（工具动作、场景命令） |
| `ROS2_Joint_Jog_Controller_R1_R5.lua` | ROS2 关节运动控制 |

## 4. ROS2 通信接口速查

### 关键 Topic

| Topic | 用途 | 示例命令 |
|-------|------|---------|
| `/compact_cell/cmd` | 场景总命令 | `RESET_CELL` |
| `/compact_cell/tool_cmd` | 工具动作 | `R1_GRIPPER_OPEN`、`R4_SCREW_START` |
| `/compact_cell/joint_cmd` | 关节控制 | `R1 J1 +10`、`R1 SET 0 20 -30 0 45 0`、`ALL HOME` |
| `/compact_cell/status` | 状态反馈 | 监听即可 |

### 快速测试

```bash
# 重置场景
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once

# R1 夹爪打开
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_OPEN'}" --once

# R1 J1 关节转 10 度
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 +10'}" --once

# 全部回零
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'ALL HOME'}" --once
```

## 5. 关键对象路径速查

### 工件
| 路径 | 用途 |
|------|------|
| `/FiveCR5A_Cell/Parts/Box_Blank` | 箱体供料 |
| `/FiveCR5A_Cell/Parts/PCB_Supply` | PCB 供料 |
| `/FiveCR5A_Cell/Parts/Control_Module_Supply` | 控制模块供料 |
| `/FiveCR5A_Cell/Parts/Terminal_Block_Supply` | 端子排供料 |
| `/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product` | 装配区产品模板 |
| `/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product` | 检测区产品模板 |

### 末端 Tip（路径规划用）
| Tip | 所属机械臂 |
|-----|-----------|
| `R1_gripper_tip` | R1 |
| `R2_vacuum_tip` | R2 |
| `R3_gripper_tip` | R3 |
| `R4_tool_tip` | R4 |
| `R5_gripper_tip` | R5 |

### 目标点（APP/TCP）
| 机械臂 | 目标点路径 |
|--------|-----------|
| R1 | `/FiveCR5A_Cell/Targets/R1_Targets/` |
| R2 | `/FiveCR5A_Cell/Targets/R2_Targets/` |
| R3 | `/FiveCR5A_Cell/Targets/R3_Targets/` |
| R4 | `/FiveCR5A_Cell/Targets/R4_Targets/` |
| R5 | `/FiveCR5A_Cell/Targets/R5_Targets/` |

## 6. 启动方式

**不能直接双击 CoppeliaSim！** 必须从 ROS2 终端启动：

```bash
source /opt/ros/humble/setup.bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

## 7. 场景当前能力

### ✅ 已实现
- 五臂场景 + 工件 + 末端工具 + 传送带 + 相机
- ROS2 双向通信
- 工具动作（夹爪/吸盘/螺丝刀）
- 工件绑定/释放
- 产品装配阶段显示
- 关节运动控制（手动点动 + 绝对设定 + 回零）
- APP/TCP 工艺目标点

### ❌ 待开发（3号负责）
- 逆运动学自动求解
- 自动路径规划
- APP→TCP→APP 自动运动
- 避障
- 多臂调度

## 8. 详细文档

| 文档 | 说明 |
|------|------|
| [完整搭建指南](Five_CR5A_Cell_Full_Process_ROS2_Joint_Guide.md) | 分步搭建 + 全部 ROS2 命令 |
| [场景对象参考](SCENE_OBJECTS_REFERENCE.md) | 每个模型的路径/尺寸/颜色 |
| [控制接口文档](Five_CR5A_Cell_Control_Interface.md) | ROS2 topic/signal 完整定义 |
| [调度方案说明](4号调度模块方案说明.md) | 4号调度算法 |
