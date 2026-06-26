# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库  
> 面向低压配电柜、新能源箱变等电气装备柔性制造场景，构建“软件平台 + CoppeliaSim 仿真产线 + 多机械臂任务调度”的自主调度、协同避碰与效能优化系统。

---

## 1. 项目定位

本项目面向多品种、小批量、快换型的电气装备柔性装配产线，针对传统机械臂依赖固定程序、人工示教换型时间长、多机械臂共享空间作业易冲突、设备利用率不足等问题，设计一套多机械臂自主调度与协同效能优化系统。

系统以动态订单为输入，将产线任务分解为上料定位、元件装配、螺丝锁付、视觉检测、良品/不良品分拣等工序，并根据机械臂状态、工序优先级、区域占用情况和任务耗时进行动态任务分配与共享空间避碰，最终通过 CoppeliaSim 仿真与实验数据验证调度策略对产线节拍、设备利用率和异常响应能力的提升效果。

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
多机械臂执行上料、装配、锁付、检测、分拣
    ↓
状态反馈与数据看板
```

---

## 2. 核心场景

### 低压配电柜多工艺柔性装配产线

项目以低压配电柜装配为典型场景，构建包含 4 台 CR5 机械臂、多个工位和 4 类以上工艺任务的柔性产线仿真系统。

| 机械臂 | 主要职责 | 典型动作 |
|---|---|---|
| R1 上料定位机械臂 | 柜体底板/安装板搬运与定位 | 抓取、搬运、放置、回零 |
| R2 元件装配机械臂 | 电气元件抓取与安装 | 取料、定位、装配 |
| R3 锁付检测机械臂 | 螺丝锁付与质量检测 | 移动到锁付点、检测点扫描、输出检测结果 |
| R4 分拣返修机械臂 | 根据检测结果进行良品/不良品分流 | 良品下料、不良品转入返修区、可选拆解返修 |

R4 的加入用于增强作品的工程完整性：当 R3 检测结果为合格时，R4 将工件转移至良品区；当 R3 检测结果为不合格时，R4 将工件转移至不良品区。若时间充足，可进一步实现 R4 对不良品进行再拆解或返修预处理。

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
    ├── 机械臂状态显示
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
效能评价：总完成时间、利用率、等待时间、冲突次数、分拣准确率、返修响应时间
```

---

## 4. 模块划分

项目采用“先接口、后实现；先最小闭环、后逐步替换真实模块”的开发方式。

| 模块 | 负责人 | 核心输入 | 核心输出 |
|---|---|---|---|
| 场景搭建模块 | 2号 | 工作空间布局需求 | CoppeliaSim 场景、点位表、对象名称 |
| 机械臂控制模块 | 3号 | 任务指令 | 机械臂动作结果、执行状态 |
| 订单拆解与调度模块 | 4号 | 订单、机械臂状态、检测结果 | 任务队列、调度指令、重调度结果 |
| 软件集成模块 | 5号 | 用户输入、模块反馈 | 软件界面、任务可视化、数据看板 |
| 项目管理与材料模块 | 1号 | 各模块结果 | 技术报告、PPT、演示视频、答辩材料 |

模块化原则：每个模块可以独立开发，但必须遵守统一接口。未完成的模块先用 Mock 函数替代，保证主流程持续可运行。

---

## 5. 仓库结构

```text
cr5_assembly_team/
├── README.md                         # 项目总说明
├── requirements.txt                  # Python 算法与数据分析依赖
├── .gitignore                        # ROS2 / Python / IDE 忽略规则
├── docs/                             # 方案、接口、分工与答辩材料
│   ├── PROJECT_PLAN.md               # 两个月并行开发计划
│   ├── INTERFACES.md                 # 订单、任务、调度、日志接口规范
│   ├── MODULE_INTEGRATION.md         # 成品倒推与模块集成方案
│   └── R4_QUALITY_SORTING.md         # R4 分拣与返修拓展方案
├── configs/                          # 建议存放点位、产品、机械臂配置
├── data/                             # 建议存放订单样例、日志与实验结果
└── src/
    ├── DOBOT_6Axis_ROS2_V4/          # DOBOT 官方六轴机械臂 ROS2 驱动
    ├── my_cr5_control/               # 自定义运动规划与控制研究
    └── cr5_rmp60_measurement/        # 测针测量与标定工具
```

> 注：`build/`、`install/`、`log/` 等 ROS2 编译产物不应提交到仓库。

---

## 6. 系统要求

| 项目 | 说明 |
|---|---|
| 操作系统 | Ubuntu 22.04 LTS |
| ROS2 | Humble Hawksbill |
| 机械臂 | DOBOT CR5，可扩展至 CR3/CR7/CR10/CR12/CR16 |
| 运动规划 | MoveIt2 |
| 仿真平台 | CoppeliaSim 为主，RViz / Gazebo 可辅助验证 |
| 软件平台 | Python + PyQt5 / PySide6 / Streamlit 均可，优先保证稳定闭环 |
| 编程语言 | Python / C++ |

---

## 7. 快速开始

### 7.1 安装 ROS2 Humble

推荐使用鱼香 ROS 一键安装：

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

安装完成后验证：

```bash
source /opt/ros/humble/setup.bash
ros2 --version
```

### 7.2 安装编译工具与依赖

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

### 7.3 克隆并编译仓库

```bash
git clone https://github.com/reshuidawang-spec/cr5_assembly_team.git ~/cr5_assembly_team
cd ~/cr5_assembly_team

pip3 install -r requirements.txt
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
source install/setup.bash
```

### 7.4 RViz 验证

```bash
ros2 launch cr5_moveit demo.launch.py
```

正常情况下应看到 CR5 三维模型与 MoveIt2 运动规划面板。

---

## 8. 真实机械臂连接

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

## 9. 团队分工

| 成员 | 模块 | 交付物 |
|---|---|---|
| 1 号 | 项目负责人 | 技术路线、报告、PPT、进度统筹、最终答辩逻辑 |
| 2 号 | CoppeliaSim 场景 | 上下料区、装配区、螺丝点、检测区、分拣区、返修区、点位表 |
| 3 号 | 机械臂动作与工具 | 运动控制、夹爪、螺丝锁付、检测动作、R4 分拣动作接口 |
| 4 号 | 订单拆解与调度 | 订单解析、任务队列、动态调度、质量分拣任务生成、异常重分配 |
| 5 号 | 软件集成与看板 | 软件界面、模块集成、CoppeliaSim 通信入口、状态显示、数据图表 |

开发原则：先统一接口，再并行开发；先跑通最小闭环，再逐步美化场景；先用 Mock 模块保证联动，再替换为真实 CoppeliaSim 控制。

---

## 10. 实验评价指标

| 指标 | 含义 |
|---|---|
| Makespan | 全部订单完成总时间 |
| Utilization | 机械臂利用率 |
| Waiting Time | 机械臂或任务等待时间 |
| Conflict Count | 共享区域冲突次数 |
| Reconfiguration Time | 订单切换或急单响应时间 |
| Sorting Accuracy | R4 良品/不良品分拣准确率 |
| Rework Response Time | 不良品进入返修区的响应时间 |

建议实验对比：

| 方案 | 说明 |
|---|---|
| Baseline | 固定顺序、固定机械臂分工、人工分拣逻辑 |
| Proposed | 动态优先级调度 + 区域锁避碰 + 检测结果驱动分拣 + 急单/故障重调度 |

---

## 11. 开发规范

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
feat: add R4 quality sorting task
fix: correct CR5 home pose config
docs: update competition project plan
```

---

## 12. 常见问题

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
