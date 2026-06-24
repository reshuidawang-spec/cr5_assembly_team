# CR5 机器人Qt5图形界面使用说明

## 功能概述

这是一个基于Qt5的CR5机器人控制图形界面，提供以下功能：

1. **箱体示教测量**：通过手动输入坐标进行3点测量
2. **TCP标定**：一键启动自动标定流程
3. **关键测试功能集成**：可直接从 GUI 启动 `simple benchmark`、`v2 benchmark` 和 `simple random dataset`
4. **绘图与数据维护功能**：可直接从 GUI 触发 benchmark 绘图、random dataset 绘图、benchmark 导出和 manifest 刷新
5. **机器人状态监控**：实时显示机器人位姿信息
6. **停止与状态清理**：支持停止当前任务，并一键清除当前 GUI 状态
7. **日志显示**：彩色日志窗口显示运行状态

## 编译步骤

### 1. 安装Qt5依赖（如果尚未安装）

```bash
sudo apt update
sudo apt install qtbase5-dev libqt5widgets5 libqt5core5a
```

### 2. 编译ROS包

```bash
cd ~/dobot_ws
colcon build --packages-select my_cr5_control
source install/setup.bash
```

## 运行方式

### 启动GUI节点

```bash
ros2 run my_cr5_control cr5_gui_node
```

## 使用流程

### 箱体测量流程

1. **输入坐标**：
   - 在界面中为P1、P2、P3三个点分别输入XYZ坐标（单位：米）
   - 例如：X=0.500, Y=0.000, Z=0.300

2. **记录点**：
   - 点击每个点对应的"记录"按钮
   - 状态栏会显示"✓ 已记录"

3. **开始测量**：
   - 当所有3个点都记录后，"开始箱体测量"按钮会激活
   - 点击按钮开始自动测量流程
   - 机器人会依次移动到每个点进行触碰测量

4. **查看结果**：
   - 测量完成后，日志会保存到 `box_measurement_log.csv`
   - 日志窗口会显示详细的执行过程

### TCP标定流程

1. 确保标定球已正确放置在工作空间
2. 点击"开始TCP标定"按钮
3. 确认对话框后，系统会自动执行5点标定流程
4. 标定结果会保存到日志文件

### 紧急停止

- 如果需要中断正在执行的任务，点击"紧急停止"按钮
- GUI 会向当前机器人任务发送停止请求，并终止正在运行的外部 benchmark / 绘图任务

### 测试与绘图流程

GUI 现在新增了“测试与绘图”面板，支持以下常用操作：

1. **运行 Simple Benchmark**
   - 使用当前界面中的 benchmark 重复次数启动 `planner_comparison_simple_node`
2. **运行 V2 Benchmark**
   - 使用当前界面中的 benchmark 重复次数启动 `planner_comparison_v2_node`
3. **采集 Simple 随机任务**
   - 使用当前界面中的随机任务数启动 `random_task_dataset_simple_node`
4. **绘制 Simple Benchmark 图**
   - 调用 `scripts/benchmarks/plot_simple_benchmark.py`
5. **绘制 Simple Random 图**
   - 调用 `scripts/datasets/plot_simple_random_dataset.py`
6. **导出 Benchmark 训练表**
   - 调用 `scripts/benchmarks/export_benchmark_dataset.py`
7. **刷新 Dataset Manifest**
   - 调用 `scripts/maintenance/build_dataset_manifest.py`

这些任务都会将输出流实时写入右侧日志面板。

### 清除当前所有状态

- 点击“清除当前所有状态”会执行以下操作：
  - 停止当前外部任务
  - 向当前机器人任务发送停止请求
  - 清空示教点与日志
  - 恢复 GUI 到默认空闲状态
  - 恢复箱体 / 标定场景按钮的默认显示状态

## 界面说明

### 示教点坐标输入区
- 3行输入框，每行对应一个测量点
- 输入XYZ坐标后点击"记录"按钮
- 状态列显示是否已记录

### 控制面板
- **清除示教点**：清空已记录的坐标
- **开始箱体测量**：执行3点测量流程
- **开始TCP标定**：执行自动标定
- **紧急停止**：中断当前任务
- **清除当前所有状态**：恢复 GUI 到默认空闲状态

### 测试与绘图面板
- **Benchmark重复次数**：用于 `simple` 和 `v2` benchmark
- **随机任务数**：用于 `simple random dataset`
- **运行 Simple Benchmark**：启动规则场景 benchmark
- **运行 V2 Benchmark**：启动 STL 场景 benchmark
- **采集 Simple 随机任务**：启动随机任务数据采集
- **绘制 Simple Benchmark 图**：生成 simple benchmark 图表
- **绘制 Simple Random 图**：生成 simple random 图表
- **导出 Benchmark 训练表**：刷新统一 benchmark 导出表
- **刷新 Dataset Manifest**：重建数据 manifest

### 机器人状态区
- 显示连接状态
- 显示运行状态（空闲/执行中）
- 实时显示当前位姿

### 运行日志区
- 彩色日志显示
- 自动滚动到最新消息
- 不同级别用不同颜色标识：
  - 绿色：成功
  - 蓝色：信息
  - 橙色：警告
  - 红色：错误

## 注意事项

1. **坐标范围**：Z坐标应在0.05-1.0米范围内
2. **安全距离**：系统会自动添加8cm的预备距离
3. **速度控制**：
   - 移动到预备点：25%速度
   - 触碰动作：5%速度
   - 后退动作：20%速度
4. **日志文件**：测量结果保存在当前目录的CSV文件中
5. **外部任务并发**：GUI 同一时刻只允许执行一个机器人任务或一个外部 benchmark / 绘图任务

## 故障排除

### 编译错误

如果遇到Qt相关的编译错误：
```bash
sudo apt install qt5-qmake qtbase5-dev-tools
```

### 运行时错误

如果提示找不到Qt库：
```bash
export LD_LIBRARY_PATH=/usr/lib/x86_64-linux-gnu:$LD_LIBRARY_PATH
```

### 机器人连接失败

1. 检查MoveIt配置是否正确加载
2. 确认cr5_moveit包已正确安装
3. 查看日志窗口的错误信息

## 技术架构

- **前端**：Qt5 Widgets
- **后端**：ROS 2 + MoveIt
- **核心库**：CR5Robot类（复用现有代码）
- **通信**：Qt信号槽 + ROS回调
- **多线程**：测量任务在独立线程执行，避免界面卡顿
- **外部任务执行**：benchmark 与绘图功能通过 `QProcess` 调用现有 ROS 节点与 Python 脚本

## 文件说明

- `cr5_gui_window.hpp/cpp`：主窗口类实现
- `cr5_gui_main.cpp`：程序入口
- `box_measurement_log.csv`：测量结果日志
