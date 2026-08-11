# 五台 CR5A 电控箱装配场景 — 快速上手指南

> 5 分钟了解场景结构、搭建流程和关键接口。

---

## 1. 场景是什么

一个在 CoppeliaSim 中运行的**五台 CR5A 机械臂协同装配仿真场景**。默认 GUI 通过 ZMQ Remote API 控制场景模型，不连接真实机械臂。

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

完整流程：R1 箱体 → R2 PCB → R1 端子排 → R3 模块 → R3 搬运 → 相机检测 → R4 锁付并回 HOME → R5 分拣

## 3. 场景搭建三步骤

### 第一步：准备机械臂
场景中需要已有五台机械臂，根对象名称必须是 `R1` `R2` `R3` `R4` `R5`。

### 第二步：依次运行三个生成脚本（用后禁用）

| 顺序 | 脚本 | 做什么 |
|------|------|--------|
| 1 | `Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua` | 生成地面、工作台、供料区、装配区、检测区、传送带、相机、60%缩放的工件 |
| 2 | `Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua` | 创建并安装 R1/R3/R5 夹爪、R2 吸盘、R4 螺丝刀 |
| 3 | `Step03_Create_Process_Targets_60_CloserTables.lua` | 创建 R1~R5 的 APP/TCP 工艺目标点 |

每个脚本：新建 Dummy → 添加 Non-threaded child script → 粘贴代码 → 运行一次 → 禁用。

### 第三步：启用四个运行时脚本（一直开启）

| 脚本 | 负责什么 |
|------|---------|
| `Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua` | 按装配阶段显示/隐藏工件 |
| `Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua` | 夹爪开合、吸盘吸附、螺丝刀旋转、工件绑定释放 |
| `ROS2_CompactCell_Bridge_V3_ColorCycle.lua` | ROS2 工艺命令桥接（工具动作、场景命令） |
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

## 6. 一体化软件启动方式

推荐直接启动软件：

```bash
cd /home/zhu/cr5_assembly_team
python3 run_demo.py
```

软件主窗口包含 `仿真执行` 和 `调度分析` 两个页签。单笔订单填写后可直接点击 START；多笔订单逐笔点击 ADD，最后点击 START。软件会自动启动或连接 CoppeliaSim、加载最新版场景并运行 R1～R5 五臂流水线，同时更新订单进度、任务队列、资源状态、检测结果、日志和实际 KPI。

点击 START 后，底部状态应依次显示 `CONNECTING COPPELIASIM`、`PREPARING SIMULATION`、`EXECUTING` 和 `ALL ORDERS COMPLETE`。连接异常会在 5 秒内返回明确错误，不会无限卡住。修改代码后必须关闭旧的软件窗口并重新运行，已启动的 Python 进程不会自动加载新代码。

当前 GUI 每批支持 1～20 个预置产品单元。第一件由 R3 转移并清空装配夹具后，系统生成下一套物料，R1/R2/R3 开始下一件；上一件同时继续由 R4/R5 锁付和分拣。

`3A + 1B` 急单操作：选择 A 型、数量输入 3，点击 START；状态进入 `EXECUTING` 后选择 B 型并点击 `URGENT`。急单数量固定为 1。系统会完成当前在制 A，暂停放行后续 A，等检测工位清空及五臂进入安全等待位后生产 B，随后恢复剩余 A。每批运行中只接受一台 B 急单，普通订单、第二台 B 或其他型号会被拒绝。

不良品设置位于订单输入区的 `NG A UNIT`：`0` 表示全部良品，`1`、`2`、`3` 分别将对应的 A 型产品设为 NG。例如数量为 3 且 `NG A UNIT=2` 时，A2 在检测和锁付后由 R5 沿不良品轨迹送到不良品区域 `(-0.15, -1.12, 0.270)`；其他 A 和 B 急单仍进入良品区。软件准备场景时会自动隐藏全部示教点，但不会删除点位或影响轨迹。全部完成后点击 RESET 再开始下一批。完全离线、无机械臂运动的界面调试使用：

```bash
python3 run_demo.py --mock
```

需要比较多订单调度方案时，在 `仿真执行` 页用 ADD 累计多笔订单，再切换到 `调度分析` 页点击“分析当前订单”。分析页会显示 Baseline 与 Proposed 指标及推荐任务时间线，但不会驱动机械臂。EXPORT 默认生成包含订单、执行结果、实际 KPI 和调度分析的统一 JSON。

`--scene-replay` 也只回放工艺状态，不执行关节轨迹；`--real` 会明确拒绝运行，整个工程不会连接物理机械臂。

### 手动启动 CoppeliaSim

正常使用无需手动启动。确有需要时运行：

```bash
/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/coppeliaSim.sh \
  /home/zhu/cr5_assembly_team/scenes/compact_cell.ttt
```

不要在终端中直接输入 `.ttt` 文件路径；它是场景数据，不是可执行程序。

## 7. 场景当前能力

### ✅ 已实现
- 五臂场景 + 工件 + 末端工具 + 传送带 + 相机
- ROS2 双向通信
- 工具动作（夹爪/吸盘/螺丝刀）
- 工件绑定/释放
- 产品装配阶段显示
- 关节运动控制（手动点动 + 绝对设定 + 回零）
- APP/TCP 工艺目标点
- GUI 内启动/连接 CoppeliaSim
- GUI 订单录入、动态调度、状态回传和场景阶段联动
- R1～R5 五套场景绑定运动计划
- 调度器驱动完整 8 工序运动闭环
- 运行时关节、工作空间和碰撞检查
- 运行中安全插入一台 B 型急单（已验证 `3A + 1B`）

### ❌ 待开发
- 新场景/新点位的逆运动学自动求解与在线路径生成
- 任意数量、任意型号的通用在线订单插入
- 任意物理机械臂连接或控制

## 8. 详细文档

| 文档 | 说明 |
|------|------|
| [完整搭建指南](Five_CR5A_Cell_Full_Process_ROS2_Joint_Guide.md) | 分步搭建 + 全部 ROS2 命令 |
| [场景对象参考](SCENE_OBJECTS_REFERENCE.md) | 每个模型的路径/尺寸/颜色 |
| [控制接口文档](Five_CR5A_Cell_Control_Interface.md) | ROS2 topic/signal 完整定义 |
| [调度方案说明](4号调度模块方案说明.md) | 4号调度算法 |
