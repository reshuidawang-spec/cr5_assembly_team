# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库  
> 面向低压配电柜、新能源箱变等电气装备柔性制造场景，构建“**四机械臂 + 多工序 + 动态订单 + 检测分拣闭环**”的自主调度、协同避碰与效能优化系统。

### 当前五臂场景覆盖说明

上述“四机械臂”内容是早期方案。当前 CoppeliaSim 主线以
`docs/Five_CR5A_Cell_Control_Interface.md` 为准，使用 `/R1` 到 `/R5`：

```text
R1 箱体/端子 -> R2 PCB -> R3 模块/产品转移
-> R4 视觉锁付 -> R5 good/defect 分拣
```

五臂基础协同已接入真实 `RobotExecutor/SimBridge`。所有成员先进入自己 clone 的
仓库根目录，再用 `$(pwd)` 打开当前仓库中的场景，不需要手写仓库绝对路径：

```bash
cd /你的克隆目录/cr5_assembly_team
source /opt/ros/humble/setup.bash
/home/vboxuser/CoppeliaSim/coppeliaSim.sh \
  "$(pwd)/scenes/five_cr5a_cell.ttt"
```

打开场景并保持停止后，可运行：

```bash
bash scripts/run_five_arm_cycle.sh good
# 或
bash scripts/run_five_arm_cycle.sh defect
```

正式订单调度和 GUI 也已接入真实模块：

```bash
# 五臂 REAL GUI
python3 run_demo.py --real

# 单个 A 型 Good 订单，默认 50 deg/s、APP 0.8 s
bash scripts/run_real_scheduler.sh good \
  --output data/logs/scheduled_good.json
```

当前 REAL 模式只支持干净场景中的一个 A 型单件订单；B/C、多订单、自动补料、
动态急单和跨机械臂故障重分配尚未实现场景级验证。GUI 已显示 R1-R5、
Good/Defect 选择、真实任务状态和首动延迟。

详细启动方式、单臂 Task 命令、前置条件、视觉验收边界和实跑证据见
[`docs/ROBOT_CONTROL_USAGE.md`](docs/ROBOT_CONTROL_USAGE.md) 与
[`robot_control/FIVE_ARM_COORDINATOR.md`](robot_control/FIVE_ARM_COORDINATOR.md)。

---

## 1. 项目定位

本项目面向多品种、小批量、快换型的电气装备柔性装配产线，针对传统机械臂依赖固定程序、人工示教换型时间长、多机械臂共享空间作业易冲突、设备利用率不足等问题，设计一套多机械臂自主调度与协同效能优化系统。

系统以动态订单为输入，将产线任务分解为上料定位、元件装配、螺丝锁付、视觉检测、良品/不良品分拣等工序，并根据机械臂状态、工序优先级、区域占用情况、检测结果和任务耗时进行动态任务分配与共享空间避碰，最终通过 CoppeliaSim 仿真与实验数据验证调度策略对产线节拍、设备利用率和质量闭环处理能力的提升效果。

最终作品形态不是单独的仿真场景或零散脚本，而是一套可演示的软件系统：

```text
软件端输入订单
    ↓
订单拆解与任务调度
    ↓
机械臂动作执行接口
    ↓
CoppeliaSim 仿真通信
    ↓
R1 上料 → R2 装配 → R3 锁付/检测 → R4 分拣/可选返修
    ↓
状态反馈与数据看板
```

---

## 2. 核心场景

### 低压配电柜多工艺柔性装配产线

项目以低压配电柜装配为典型场景，构建包含 **4 台 CR5 机械臂**、多个工位和 4 类以上工艺任务的柔性产线仿真系统。工作空间布局、零部件清单和采购建议详见 [docs/WORKSPACE_DESIGN.md](docs/WORKSPACE_DESIGN.md)。

| 机械臂 | 主要职责 | 典型动作 |
|---|---|---|
| R1 上料定位机械臂 | 柜体底板/安装板搬运与定位 | 抓取、搬运、放置、回零 |
| R2 元件装配机械臂 | 电气元件抓取与安装 | 取料、定位、装配 |
| R3 锁付检测机械臂 | 螺丝锁付与质量检测 | 移动到锁付点、检测点扫描，返回 OK/NG |
| R4 分拣返修机械臂 | 良品/不良品分拣，空闲时可选返修拆解 | 根据 R3 检测结果分拣至良品区/不良品区 |

R1-R3 构成基础装配闭环，R4 构成检测结果驱动的质量分拣闭环。R4 的返修拆解为拓展功能，优先实现良品/不良品分流。

支持的演示场景：

- A/B/C 三种产品订单混流输入；
- 正常订单调度与任务队列生成；
- 急单插入后的动态优先级调整；
- 共享区域冲突检测与区域锁避碰；
- R3 检测结果驱动 R4 良品/不良品分拣；
- R4 对不良品进行可选返修拆解；
- 机械臂故障或任务失败后的重分配；
- 固定顺序调度与动态调度对比实验。

---

## 3. 系统架构

```text
软件平台 / 上位机界面
    ├── 订单输入
    ├── 任务队列显示
    ├── 四台机械臂状态显示
    ├── CoppeliaSim 连接状态
    └── 数据看板
        ↓
动态订单输入
        ↓
订单解析与工序任务分解
        ↓
多机械臂任务调度器
    ├── 动态优先级策略
    ├── 机械臂状态评估
    ├── 共享区域锁机制
    ├── 急单/故障重调度
    └── 检测结果驱动的质量分拣策略
        ↓
机械臂运动执行层
    ├── R1 上料定位
    ├── R2 元件装配
    ├── R3 锁付检测
    └── R4 良品/不良品分拣与可选返修
        ↓
CoppeliaSim 仿真场景
        ↓
日志记录与数据看板
        ↓
效能评价：总完成时间、利用率、等待时间、冲突次数、分拣响应时间
```

---

## 4. 仓库结构

```text
cr5_assembly_team/
├── README.md                         # 项目总说明
├── requirements.txt                  # Python 依赖
├── .gitignore                        # ROS2 / Python / IDE 忽略规则
├── run_demo.py                       # 一键启动入口
│
├── app/                              # 软件集成模块（5号）
│   ├── main_app.py                   # 主界面（tkinter GUI）
│   └── dashboard.py                  # 数据看板与图表
│
├── interfaces/                       # 抽象接口定义（所有成员遵循）
│   ├── types.py                      # 共享数据类型（Order, Task, TaskResult, RobotState）
│   ├── order_interface.py            # IOrderParser 接口
│   ├── scheduler_interface.py        # IScheduler 接口
│   ├── robot_interface.py            # IRobotExecutor 接口
│   └── sim_interface.py              # ISimBridge 接口
│
├── mock/                             # Mock 假实现（开发阶段使用）
│   ├── mock_order_parser.py
│   ├── mock_scheduler.py
│   ├── mock_robot_executor.py
│   └── mock_sim_bridge.py
│
├── scheduler/                        # 订单解析与调度模块（4号实现）
├── robot_control/                    # 机械臂控制模块（3号实现）
├── sim_bridge/                       # 仿真通信模块（2号/3号实现）
│
├── configs/                          # 配置文件
│   ├── points.yaml                   # 场景点位表，含 R4 分拣点位
│   ├── robots.yaml                   # 机械臂配置，含 R1-R4
│   ├── product_types.yaml            # 产品工艺配置
│   └── scheduler.yaml                # 调度参数
│
├── data/                             # 数据文件
│   ├── orders/demo_orders.json       # 示例订单
│   ├── logs/                         # 运行日志
│   └── results/                      # 实验结果
│
├── docs/                             # 方案文档
│   ├── PROJECT_PLAN.md
│   ├── INTERFACES.md
│   ├── WORKSPACE_DESIGN.md
│   ├── R4_QUALITY_SORTING.md
│   └── TEAM_WORKFLOW.md
│
└── src/
    └── DOBOT_6Axis_ROS2_V4/          # DOBOT 官方 ROS2 驱动
```

> 注：`build/`、`install/`、`log/` 等 ROS2 编译产物不应提交到仓库。

### 4.1 模块分工与接口

| 模块 | 负责同学 | 实现接口 | 当前状态 |
|------|---------|---------|---------|
| 软件集成 (app/) | 5号 | 维护 main_app.py，调用各模块 | GUI 可用 |
| 订单调度 (scheduler/) | 4号 | IOrderParser, IScheduler | Mock 占位 |
| 机械臂控制 (robot_control/) | 3号 | IRobotExecutor | R1-R5 基础视觉动作与一键协同已接入真实 Coppelia 执行 |
| 仿真通信 (sim_bridge/) | 2号/3号 | ISimBridge | 五臂 ZMQ、对象、关节、夹爪和场景控制已实现 |
| 场景搭建 | 2号 | configs/points.yaml | 点位表已定义，需落到 CoppeliaSim |

### 4.2 开发流程

1. **先跑通 Mock 闭环**：`python3 run_demo.py` 启动 GUI，用假数据验证完整链路；
2. **按接口开发**：每个同学在对应目录实现 `interfaces/` 中定义的抽象接口；
3. **替换 Mock**：在 `app/main_app.py` 的 `set_modules()` 中注入真实实现；
4. **每周集成**：所有模块合并到主分支，验证 `run_demo.py` 可运行；
5. **R4 分拣优先级**：先完成 OK/NG 分流，再考虑返修拆解。

---

## 5. 系统要求

| 项目 | 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| 机械臂 | DOBOT CR5，主线统一为 R1-R4 四台 CR5 仿真验证 |
| 运动规划 | MoveIt2 |
| 仿真平台 | CoppeliaSim 为主，RViz / Gazebo 可辅助验证 |
| 软件平台 | Python + tkinter / PyQt / Streamlit 均可，优先保证稳定闭环 |
| 编程语言 | Python / C++ |

---

## 6. 快速开始

### 6.1 安装 ROS2 Humble

推荐使用鱼香 ROS 一键安装：

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

安装完成后验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 6.2 安装编译工具与依赖

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-rosdep \
  python3-vcstool \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit-visual-tools \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher
```

首次使用 rosdep 时执行：

```bash
sudo rosdep init
rosdep update
```

### 6.3 克隆并编译仓库

```bash
git clone https://github.com/reshuidawang-spec/cr5_assembly_team.git ~/cr5_assembly_team
cd ~/cr5_assembly_team

pip3 install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 6.4 运行 Mock 闭环

```bash
python3 run_demo.py
# 或无界面模式
python3 run_demo.py --headless
```

### 6.5 RViz 验证

```bash
ros2 launch cr5_moveit demo.launch.py
```

正常情况下应看到 CR5 三维模型与 MoveIt2 运动规划面板。

---

## 7. 真实机械臂连接

DOBOT 控制器默认 IP 通常为：`192.168.1.6`。电脑需设置为同网段静态 IP，例如 `192.168.1.100`。

```bash
ping 192.168.1.6
```

启动真机通信：

```bash
export IP_address=192.168.1.6
export DOBOT_TYPE=CR5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

另开终端启动 MoveIt：

```bash
source ~/cr5_assembly_team/install/setup.bash
ros2 launch cr5_moveit dobot_moveit.launch.py
```

---

## 8. 团队分工

| 成员 | 模块 | 交付物 |
|---|---|---|
| 1 号 | 项目负责人 | 技术路线、报告、PPT、进度统筹、最终答辩逻辑 |
| 2 号 | CoppeliaSim 场景 | 上下料区、装配区、螺丝点、检测区、良品区、不良品区、返修区、点位表 |
| 3 号 | 机械臂动作与工具 | 运动控制、夹爪、螺丝锁付、检测动作、R4 分拣动作接口 |
| 4 号 | 订单拆解与调度 | 订单解析、任务队列、动态调度、R3 检测结果处理、R4 分拣任务生成、异常重分配 |
| 5 号 | 软件集成与看板 | 软件界面、模块集成、CoppeliaSim 通信入口、状态显示、数据图表 |

开发原则：先统一接口，再并行开发；先跑通最小闭环，再逐步美化场景；先用 Mock 模块保证联动，再替换为真实 CoppeliaSim 控制。

---

## 9. 实验评价指标

| 指标 | 含义 |
|---|---|
| Makespan | 全部订单完成总时间 |
| Utilization | 机械臂利用率 |
| Waiting Time | 机械臂或任务等待时间 |
| Conflict Count | 共享区域冲突次数 |
| Reconfiguration Time | 订单切换或急单响应时间 |
| Sorting Response Time | R3 检测完成到 R4 完成分拣的时间 |
| Sorting Accuracy | R4 良品/不良品分拣准确率 |
| Rework Response Time | 不良品进入返修区的响应时间，可选 |

建议实验对比：

| 方案 | 说明 |
|---|---|
| Baseline | 固定顺序、固定机械臂分工、人工或固定下料 |
| Proposed | 动态优先级调度 + 区域锁避碰 + 检测结果驱动分拣 + 急单/故障重调度 |

---

## 10. 开发规范

### 分支策略

- `main`：稳定分支，仅保留可展示版本；
- `feature/simulation`：仿真场景；
- `feature/motion`：机械臂动作与路径；
- `feature/scheduler`：任务调度算法；
- `feature/software`：软件集成与数据看板；
- `feature/docs`：文档、PPT 与答辩材料。

### 提交信息格式

```text
<type>: <description>
```

常用类型：`feat` / `fix` / `docs` / `refactor` / `test` / `chore`。

示例：

```text
feat: add dynamic task scheduler
feat: add R4 quality sorting task
fix: correct CR5 home pose config
docs: update competition project plan
```

---

## 11. 常见问题

**1. `ros2: command not found`**

```bash
source /opt/ros/humble/setup.bash
source ~/cr5_assembly_team/install/setup.bash
```

**2. `colcon build` 缺少 Python 模块**

```bash
pip3 install -r requirements.txt
```

**3. RViz 不显示模型**  
手动添加 `RobotModel` 显示项，Topic 设置为 `/robot_description`。

**4. 无法连接真机**  
检查网线、IP 设置、`IP_address` 和 `DOBOT_TYPE` 环境变量。

**5. 消息类型编译错误**

```bash
colcon build --packages-up-to dobot_msgs_v4
colcon build
```
