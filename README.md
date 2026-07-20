# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库  
> 当前主线统一为：**五台 DOBOT CR5A + 小型电控箱装配 + 固定相机检测 + R4 锁付 + R5 良品/缺陷品分拣**。

---

## 1. 当前项目状态

仓库目前已经包含两部分可以独立使用、正在继续集成的资产：

| 部分 | 当前内容 | 状态 |
|---|---|---|
| 五臂 CoppeliaSim 场景 | 五台 CR5A、供料区、装配夹具、检测/锁付平台、固定相机、双传送带、末端工具、APP/TCP 目标点、ROS2 命令桥接与关节点动 | 场景资产与 Lua 脚本已提交 |
| Python 调度与演示程序 | 订单解析、五臂细粒度工序链、区域互斥、动态调度、故障实验、Mock 执行器和界面 | Mock/离线仿真可运行，真实 ROS2/CoppeliaSim 执行层仍需接通 |

当前不能将“场景已搭建”理解为整套系统已经自动运行。以下能力仍属于后续集成任务：

- 自动逆运动学与轨迹规划；
- 多机械臂实时避碰；
- Python 调度器向 ROS2/CoppeliaSim 下发真实任务；
- 真实抓取、吸附、锁付和视觉检测结果回传；
- 完整订单驱动的自动连续运行。

---

## 2. 统一工艺流程

```text
R1 抓取箱体并放入装配夹具
    ↓
R2 吸取 PCB 并装入箱体
    ↓
R3 安装控制模块
    ↓
R1 安装端子排
    ↓
R3 将完整装配体转移到检测/锁付平台
    ↓
固定相机检测并产生 OK / NG 结果
    ↓
R4 完成端子螺钉锁付
    ↓
R5 根据先前检测结果分拣到合格品或缺陷品传送带
```

### 机械臂分工

| 资源 | 任务 | 末端工具 |
|---|---|---|
| R1 | 箱体上料、端子排安装 | 宽口夹爪 |
| R2 | PCB 安装 | 吸盘 |
| R3 | 控制模块安装、装配体转移 | 宽口夹爪 |
| CAMERA | 固定视觉检测，返回 OK/NG | 固定相机 |
| R4 | 端子螺钉锁付 | 电动螺丝刀 |
| R5 | 合格品/缺陷品分拣 | 宽口夹爪 |

调度逻辑中，检测结果会先被记录；只有 R4 锁付完成后，系统才动态生成一条 R5 分拣任务。这样可以避免同一产品同时生成良品和缺陷品两条分拣任务。

---

## 3. 已提交的核心内容

### 3.1 CoppeliaSim 场景与脚本

`scenes/` 目录包含：

- `Step01_Create_Clean_Cell_60_GreyTable_RobotColor.lua`：生成五臂装配单元；
- `Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua`：创建夹爪、吸盘和螺丝刀；
- `Step03_Create_Process_Targets_60.lua`：创建 APP/TCP 工艺目标点；
- `Product_Stage_Controller_60.lua`：控制产品装配阶段显示；
- `Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua`：控制末端工具与工件绑定/释放；
- `ROS2_CompactCell_Bridge_V2_GlobalCallbacks.lua`：工艺命令 ROS2 桥接；
- `ROS2_Joint_Jog_Controller_R1_R5.lua`：R1～R5 关节点动；
- `compact_cell1ttt.ttt`：当前场景文件。

完整搭建和使用方法见：

- [五台 CR5A 场景完整流程](docs/Five_CR5A_Cell_Full_Process_ROS2_Joint_Guide.md)
- [快速开始](docs/QUICK_START.md)

### 3.2 调度模块

当前五臂调度配置和代码主要位于：

```text
configs/
├── assembly_components.yaml
├── points.yaml
├── product_types.yaml
├── robots.yaml
└── scheduler.yaml

scheduler/
├── assembly_process.py
├── scheduler.py
├── task_generator.py
└── experiment.py
```

调度模块已经按五臂场景细化任务、机械臂资源和共享区域。说明文档见：

- [五臂场景调度对齐说明](docs/FIVE_CR5A_SCHEDULER_ALIGNMENT.md)
- [装配流程优化说明](docs/ASSEMBLY_PROCESS_OPTIMIZATION.md)
- [四种方案与故障对比](docs/FOUR_SCHEME_FAULT_COMPARISON.md)
- [4号调度模块方案说明](docs/4号调度模块方案说明.md)

### 3.3 Mock 演示与测试

`mock/` 用于在真实 ROS2 和 CoppeliaSim 执行层尚未接通时验证订单、任务状态和调度闭环。

```bash
python3 run_demo.py
python3 run_demo.py --headless
```

调度测试：

```bash
python3 -m pytest tests/test_scheduler_v2.py
```

装配流程和故障实验脚本位于 `scripts/`。

---

## 4. ROS2 与 CoppeliaSim 启动

推荐环境：

- Ubuntu 22.04；
- ROS2 Humble；
- CoppeliaSim Edu 4.10；
- Python 3。

CoppeliaSim 必须从已经加载 ROS2 环境的终端启动：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash

cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

若直接双击启动，`simROS2` 可能因缺少 ROS2 动态库而加载失败。

---

## 5. 团队统一规则

### 5.1 当前唯一主线

所有新代码、点位和文档均以“五台 CR5A 小型电控箱场景”为主线：

- R4 负责锁付；
- R5 负责质量分拣；
- 固定相机负责检测；
- 场景目标点以 `scenes/Step03_Create_Process_Targets_60.lua` 和 `configs/points.yaml` 为准；
- 调度任务名称、区域名称和机械臂名称应与配置文件保持一致。

### 5.2 历史四臂文档

以下文件保留为早期方案和设计参考，不代表当前场景的最终分工：

- `docs/PROJECT_PLAN.md`；
- `docs/R4_QUALITY_SORTING.md`；
- `docs/WORKSPACE_DESIGN.md`。

其中涉及“四臂、R4 分拣、低压配电柜”的内容属于历史方案。开发和联调时应优先参考本 README、五臂完整流程文档和五臂调度对齐说明。

### 5.3 分支与合并

- 每个成员在独立分支提交；
- PR 必须说明修改内容、接口影响和验证方法；
- 不直接覆盖他人负责模块；
- 合并前确认脚本文件名、对象路径、ROS2 topic、任务名称和配置名称一致；
- 冲突 PR 不直接强制合并，应将仍有效的内容迁移到最新 `main` 后再关闭旧 PR。

---

## 6. 推荐下一步

1. 使用 `compact_cell1ttt.ttt` 验证七个 Lua 脚本和对象命名；
2. 固化 R1～R5 的关节名、TCP 名和目标点名；
3. 实现 Python 调度任务到 ROS2 topic 的适配层；
4. 接收 CoppeliaSim 完成信号并推进任务状态；
5. 加入 MoveIt2 或自定义轨迹规划与共享区域避碰；
6. 完成订单输入到五臂连续装配、检测、锁付和分拣的端到端演示。

---

## 7. 仓库说明

仓库中 `github_upload_ready_4_scheduler_20260719/` 是阶段性交付副本，正式开发应优先修改仓库根目录下的同名模块，避免两套代码继续分叉。

ROS2 的 `build/`、`install/`、`log/`，以及 Python 缓存、IDE 临时文件不应提交。
