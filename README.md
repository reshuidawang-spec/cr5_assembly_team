# CR5 Assembly Team — ROS2 工作空间

面向低压配电柜柔性装配产线的多机械臂自主调度与协同避碰系统。本仓库是五人团队的统一 ROS2 工作空间，包含 DOBOT CR5 六轴机械臂的完整驱动、MoveIt2 运动规划配置，以及自定义运动规划与测量工具。

## 团队角色

| 角色 | 职责 |
|------|------|
| 1 号：项目负责人 | 总体方案设计、技术报告、PPT 答辩、进度统筹 |
| 2 号：仿真场景 | CoppeliaSim / Gazebo 场景搭建、三台机械臂建模 |
| 3 号：机械臂动作 | 抓取/放置/锁付/检测动作流程、避碰逻辑 |
| 4 号：调度算法 | 订单解析、任务队列、动态优先级调度 |
| 5 号：数据看板 | 实验数据统计、可视化、视频剪辑、材料美化 |

## 系统要求

| 项目 | 说明 |
|------|------|
| 操作系统 | Ubuntu 22.04 LTS (Jammy) |
| ROS2 | Humble Hawksbill |
| 机械臂 | DOBOT CR5（也支持 CR3/CR7/CR10/CR12/CR16） |
| 通信方式 | TCP/IP（网线直连或局域网） |

## 快速开始

### 1. 安装 ROS2 Humble

推荐鱼香 ROS 一键安装：

```bash
wget http://fishros.com/install -O fishros && bash fishros
```

选「1」一键安装 → Humble (Ubuntu 22.04) → 桌面版。

```bash
source /opt/ros/humble/setup.bash
ros2 --version   # 验证安装
```

### 2. 安装编译工具

```bash
sudo apt update
sudo apt install -y python3-colcon-common-extensions python3-rosdep python3-vcstool
sudo rosdep init && rosdep update
```

### 3. 安装 ROS2 功能包

```bash
sudo apt install -y \
  ros-humble-moveit \
  ros-humble-ros2-control \
  ros-humble-ros2-controllers \
  ros-humble-moveit-visual-tools \
  ros-humble-xacro \
  ros-humble-joint-state-publisher \
  ros-humble-joint-state-publisher-gui \
  ros-humble-robot-state-publisher \
  ros-humble-warehouse-ros-mongo \
  ros-humble-srdfdom
```

可选：Gazebo 仿真支持

```bash
sudo apt install -y \
  ros-humble-gazebo-ros-pkgs \
  ros-humble-gazebo-ros2-control \
  ros-humble-joint-trajectory-controller
```

### 4. 克隆本仓库并编译

```bash
git clone https://github.com/reshuidawang-spec/cr5_assembly_team.git ~/cr5_assembly_team
cd ~/cr5_assembly_team

pip3 install pyyaml numpy
rosdep install --from-paths src --ignore-src -r -y
colcon build --symlink-install
```

### 5. 配置环境变量

```bash
echo 'source /opt/ros/humble/setup.bash' >> ~/.bashrc
echo 'source ~/cr5_assembly_team/install/setup.bash' >> ~/.bashrc
source ~/.bashrc
```

### 6. RViz 验证（无需真机）

```bash
ros2 launch cr5_moveit demo.launch.py
```

应看到 CR5 3D 模型和 MoveIt 运动规划面板。

## 连接真实机械臂

### 网络设置

DOBOT 控制器默认 IP：`192.168.1.6`。将电脑设为同网段静态 IP（如 `192.168.1.100`）。

```bash
ping 192.168.1.6   # 确认连通
```

### 启动真机通信

```bash
export IP_address=192.168.1.6
export DOBOT_TYPE=CR5
ros2 launch cr_robot_ros2 dobot_bringup_ros2.launch.py
```

### 真机 + MoveIt + RViz

另开终端：

```bash
source ~/cr5_assembly_team/install/setup.bash
ros2 launch cr5_moveit dobot_moveit.launch.py
```

## 工作空间结构

```
cr5_assembly_team/
├── src/
│   ├── DOBOT_6Axis_ROS2_V4/       # DOBOT 官方六轴机械臂 ROS2 驱动
│   │   ├── cra_description/       # URDF/XACRO 模型 (CR3/CR5/CR7/CR10/CR12/CR16/CR30H/Nova2/Nova5)
│   │   ├── dobot_msgs_v4/         # 自定义 ROS2 消息和动作接口
│   │   ├── dobot_bringup_v4/      # 真机通信驱动节点 (cr_robot_ros2)
│   │   ├── cr5_moveit/            # CR5 MoveIt2 配置与启动
│   │   ├── dobot_moveit/          # 通用 MoveIt 辅助节点
│   │   ├── dobot_rviz/            # RViz 可视化启动
│   │   ├── dobot_demo/            # 示例脚本
│   │   ├── dobot_gazebo/          # Gazebo 仿真启动
│   │   └── servo_action/          # 伺服控制动作接口
│   ├── my_cr5_control/            # 自定义运动规划研究 (Heuristic-Guided Motion Planning)
│   └── cr5_rmp60_measurement/     # 探针测量与标定工具
├── .gitignore
└── README.md
```

## 团队开发规范

### 分支策略

- `main` — 稳定分支，只接受经过 review 的 PR
- `dev/<name>` — 个人开发分支
- `feature/<功能名>` — 功能分支

### 提交规范

- 提交信息使用英文，格式：`<type>: <description>`
- 类型：`feat` / `fix` / `docs` / `refactor` / `test`
- 不要提交 `build/`、`install/`、`log/`、`__pycache__/`、`.pyc`

### 添加新包

```bash
cd ~/cr5_assembly_team/src
ros2 pkg create --build-type ament_cmake my_new_package
# 或
ros2 pkg create --build-type ament_python my_new_package
```

在 `CMakeLists.txt` 或 `package.xml` 中声明依赖，然后：

```bash
cd ~/cr5_assembly_team
colcon build --symlink-install --packages-select my_new_package
```

## 常见问题

**`ros2: command not found`** — 执行 `source /opt/ros/humble/setup.bash && source ~/cr5_assembly_team/install/setup.bash`

**`colcon build` 报 missing module** — `pip3 install <missing-package>`

**RViz 不显示模型** — 手动添加 `RobotModel` 显示项，Topic 设为 `/robot_description`

**无法连接真机** — 检查网线、IP 设置、`IP_address` 和 `DOBOT_TYPE` 环境变量

**消息类型编译错误** — 先编译 `dobot_msgs_v4`：`colcon build --packages-up-to dobot_msgs_v4 && colcon build`
