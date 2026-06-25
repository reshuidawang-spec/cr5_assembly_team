# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库  
> 面向低压配电柜、新能源箱变等电气装备柔性制造场景，构建“三机械臂 + 多工序 + 动态订单”的自主调度、协同避碰与效能优化系统。

---

## 1. 项目定位

本项目面向多品种、小批量、快换型的电气装备柔性装配产线，针对传统机械臂依赖固定程序、人工示教换型时间长、多机械臂共享空间作业易冲突、设备利用率不足等问题，设计一套多机械臂自主调度与协同效能优化系统。

系统以动态订单为输入，将产线任务分解为上料定位、元件装配、螺丝锁付、视觉检测等工序，并根据机械臂状态、工序优先级、区域占用情况和任务耗时进行动态任务分配与共享空间避碰，最终通过仿真与实验数据验证调度策略对产线节拍和设备利用率的提升效果。

---

## 2. 核心场景

### 低压配电柜多工艺柔性装配产线

项目以低压配电柜装配为典型场景，构建包含 3 台 CR5 机械臂、多个工位和 3 类以上工艺任务的柔性产线仿真系统。工作空间布局、零部件清单和采购建议详见 [docs/WORKSPACE_DESIGN.md](docs/WORKSPACE_DESIGN.md)。

| 机械臂 | 主要职责 | 典型动作 |
|---|---|---|
| R1 上料定位机械臂 | 柜体底板/安装板搬运与定位 | 抓取、搬运、放置、回零 |
| R2 元件装配机械臂 | 电气元件抓取与安装 | 取料、定位、装配 |
| R3 锁付检测机械臂 | 螺丝锁付与质量检测 | 移动到锁付点、检测点扫描 |

支持的演示场景：

- A/B/C 三种产品订单混流输入；
- 正常订单调度与任务队列生成；
- 急单插入后的动态优先级调整；
- 共享区域冲突检测与区域锁避碰；
- 机械臂故障或任务失败后的重分配；
- 固定顺序调度与动态调度对比实验。

---

## 3. 系统架构

```text
动态订单输入
    ↓
订单解析与工序任务分解
    ↓
多机械臂任务调度器
    ├── 动态优先级策略
    ├── 机械臂状态评估
    ├── 共享区域锁机制
    └── 急单/故障重调度
    ↓
机械臂运动执行层
    ├── R1 上料定位
    ├── R2 元件装配
    └── R3 锁付检测
    ↓
日志记录与数据看板
    ↓
效能评价：总完成时间、利用率、等待时间、冲突次数
```

---

## 4. 仓库结构

```text
cr5_assembly_team/
├── README.md                         # 项目总说明
├── requirements.txt                  # Python 算法与数据分析依赖
├── .gitignore                        # ROS2 / Python / IDE 忽略规则
├── docs/                             # 方案、接口、分工与答辩材料
│   ├── PROJECT_PLAN.md               # 两个月并行开发计划
│   ├── INTERFACES.md                 # 订单、任务、调度、日志接口规范
│   ├── WORKSPACE_DESIGN.md           # 工作空间布局、零部件清单、采购建议
│   └── TEAM_WORKFLOW.md              # 团队协作规范与分支策略
├── src/
│   └── DOBOT_6Axis_ROS2_V4/          # DOBOT 官方六轴机械臂 ROS2 驱动
└── data/                             # 建议存放订单样例、日志与实验结果
```

> 注：`build/`、`install/`、`log/` 等 ROS2 编译产物不应提交到仓库。

---

## 5. 系统要求

| 项目 | 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| 机械臂 | DOBOT CR5，可扩展至 CR3/CR7/CR10/CR12/CR16 |
| 运动规划 | MoveIt2 |
| 仿真平台 | RViz / Gazebo / CoppeliaSim，可按成员任务并行推进 |
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

### 6.4 RViz 验证

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
| 1 号 | 项目负责人 | 技术路线、报告、PPT、进度统筹 |
| 2 号 | 仿真场景 | 极简场景、三机械臂工位、演示视频 |
| 3 号 | 动作与路径 | 动作模板、运动调用、共享区域避碰 |
| 4 号 | 调度算法 | 订单解析、任务队列、动态调度、异常重分配 |
| 5 号 | 数据展示 | 实验日志、对比图表、看板、材料美化 |

开发原则：先统一接口，再并行开发；先跑通极简闭环，再逐步美化场景。

---

## 9. 实验评价指标

| 指标 | 含义 |
|---|---|
| Makespan | 全部订单完成总时间 |
| Utilization | 机械臂利用率 |
| Waiting Time | 机械臂或任务等待时间 |
| Conflict Count | 共享区域冲突次数 |
| Reconfiguration Time | 订单切换或急单响应时间 |

建议实验对比：

| 方案 | 说明 |
|---|---|
| Baseline | 固定顺序、固定机械臂分工 |
| Proposed | 动态优先级调度 + 区域锁避碰 + 急单重调度 |

---

## 10. 开发规范

### 分支策略

- `main`：稳定分支，仅保留可展示版本；
- `feature/simulation`：仿真场景；
- `feature/motion`：机械臂动作与路径；
- `feature/scheduler`：任务调度算法；
- `feature/dashboard`：数据统计与可视化；
- `feature/docs`：文档、PPT 与答辩材料。

### 提交信息格式

```text
<type>: <description>
```

常用类型：

- `feat`：新增功能；
- `fix`：修复问题；
- `docs`：文档修改；
- `refactor`：代码结构调整；
- `test`：测试或实验脚本；
- `chore`：配置、依赖、整理类修改。

示例：

```text
feat: add dynamic task scheduler
fix: correct CR5 home pose config
docs: update competition project plan
```

---

## 11. 常见问题

**1. `ros2: command not found`**  
执行：

```bash
source /opt/ros/humble/setup.bash
source ~/cr5_assembly_team/install/setup.bash
```

**2. `colcon build` 缺少 Python 模块**  
执行：

```bash
pip3 install -r requirements.txt
```

**3. RViz 不显示模型**  
手动添加 `RobotModel` 显示项，Topic 设置为 `/robot_description`。

**4. 无法连接真机**  
检查网线、IP 设置、`IP_address` 和 `DOBOT_TYPE` 环境变量。

**5. 消息类型编译错误**  
优先编译消息包：

```bash
colcon build --packages-up-to dobot_msgs_v4
colcon build
```
