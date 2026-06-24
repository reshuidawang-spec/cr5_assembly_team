# 活塞喷涂基础 GUI 说明

## 1. 目标

`piston_spray_gui_node` 是当前活塞喷涂工程的第一阶段上位机原型。

它只实现基础能力：

- 工艺参数录入
- 工件坐标系设置
- 喷涂轨迹生成
- 基坐标真实位姿映射
- Qt 界面可视化
- 计划 JSON 导出
- 末端 TCP 示教保存
- 基于保存位姿的 XYZ / RPY 微调
- 基于工件坐标系的 RViz 活塞圆柱生成
- 喷涂专用干净场景初始化（默认不附加测针碰撞模型）

暂不实现：

- PLC 通讯
- 喷头开关 IO
- 流量传感器闭环
- 激光位移闭环
- CCD 均匀度识别
- 自动优化

## 2. 启动

离线基础版：

```bash
cd ~/dobot_ws
colcon build --packages-select my_cr5_control --symlink-install
source install/setup.bash
ros2 run my_cr5_control piston_spray_gui_node
```

真实机械臂一键联机版：

```bash
cd ~/dobot_ws
colcon build --packages-select \
  dobot_msgs_v4 \
  cr_robot_ros2 \
  dobot_rviz \
  cra_description \
  cr5_moveit \
  dobot_moveit \
  my_cr5_control \
  --symlink-install
source install/setup.bash
ros2 launch my_cr5_control piston_spray_real_robot.launch.py
```

这个 launch 会自动带起：

- `cr_robot_ros2`
- `dobot_moveit/action_move_server`
- `dobot_moveit/joint_states`
- `cr5_moveit` 的 `robot_state_publisher / move_group / RViz`
- `piston_spray_gui_node`

## 3. 核心输入

界面左侧分为四组：

- 喷涂工艺参数
- 工件坐标系与装夹
- 工具 TCP 偏移
- 操作

其中最重要的是工件坐标系：

- `origin`
  - 活塞喷涂起点处的轴心点，单位 m
- `axial_direction`
  - 活塞轴向方向
- `radial_direction`
  - 喷涂起始时，从活塞轴线指向喷头的方向

程序会据此建立工件局部坐标系，并把局部轨迹映射到机器人基坐标。

当前界面会实时显示：

- `base_link` 下的工件坐标系原点
- 归一化后的 `X(radial) / Y(side) / Z(axial)`
- 按当前参数推算的活塞圆柱中心
- 喷涂参考点

`工具 TCP 偏移` 的含义：

- 它表示真实喷涂点 TCP 相对机器人法兰坐标原点的 XYZ 偏移，单位 mm
- 如果喷头不是直接装在法兰原点，而是通过接头、支架、延长段伸出去，就必须填写这里
- 软件会用它在 `tcp_pose_base` 和 `flange_pose_base` 之间做换算

## 4. 位姿定义

程序内部采用如下局部定义：

- 工件局部 `Z`
  - 沿活塞喷涂方向
- 工件局部 `X`
  - 从活塞轴线指向喷头
- 喷头喷射方向
  - 指向局部 `-X`

导出时同时给出两套位姿：

- `tcp_pose_base`
  - 喷头 TCP 在机器人基坐标系中的目标位姿
- `flange_pose_base`
  - 根据 TCP 偏移反算的法兰位姿

## 5. 可视化

右侧可视化包含三块：

- 局部工艺视图
- 基坐标 XY 轨迹
- 基坐标 XZ 轨迹

下方表格会列出：

- `safe_start`
- `process_start`
- `coat_entry`
- `coat_exit`
- `safe_exit`

这些都是后续接 MoveIt 或 PLC 时最基础的关键点。

## 6. 末端示教与微调

界面左侧新增了“末端示教与微调”区域，适合在不接转台的情况下做真机示教。

基本流程：

1. 手动把 CR5 末端移动到目标空间点
2. 点击“保存当前末端位姿”
3. 在 `dX / dY / dZ / dRoll / dPitch / dYaw` 中输入细微修正量
4. 选择“直线微调”或“规划微调”
5. 点击“移动到微调目标”

说明：

- 软件保存的是当前 TCP 位姿，不是转台状态
- 目标位姿会结合当前 `TCP 相对法兰偏移` 自动换算成法兰目标
- 如果要以新的位置继续微调，再次点击“保存当前末端位姿”即可

## 7. RViz 活塞圆柱生成

界面左侧新增了“RViz 活塞圆柱”区域。

使用方式：

1. 启动 MoveIt / RViz 场景
2. 在 RViz 中添加一个 `MarkerArray` 显示，话题设为 `/piston_spray/workpiece_markers`
3. 在界面中填写或确认：
   - `origin`
   - `axial_direction`
   - `radial_direction`
   - `活塞直径`
   - `喷涂长度`
   - `法兰末端到活塞表面距离`
4. 点击“生成/刷新工件坐标系”
5. 点击“生成 RViz 活塞圆柱”

当前版本行为：

- 工件坐标系以机器人底座 `base_link` 为参照
- 活塞喷涂 GUI 默认使用干净场景，不自动附加测头测针碰撞模型
- 修改 `origin / axial_direction / radial_direction` 会直接改变工件生成位置和朝向
- RViz 会发布工件 `X/Y/Z` 三轴、原点标签和喷涂参考点
- 圆柱使用上方 `活塞直径` 作为直径
- 圆柱使用上方 `喷涂长度` 作为长度
- 圆柱中心按工件坐标系 `origin + axial * length/2` 计算
- 圆柱轴线跟随界面中的 `活塞轴向方向`
- 圆柱不会跟随机械臂当前法兰朝向去横放
- “法兰末端到活塞表面距离” 当前主要用于显示喷涂参考点
- 当前是简化模型，只用于位置、朝向和基础避障验证

## 8. 示教点持久化

当前版本会自动持久化保存：

- 已保存的 TCP 示教点
- 微调量 `dX / dY / dZ / dRoll / dPitch / dYaw`
- 微调速度和移动方式
- TCP 偏移
- RViz 圆柱距离参数

软件下次启动时会自动恢复这些内容。

默认持久化文件位置：

- `~/.config/cr5_piston_spray_gui/teach_state.json`
