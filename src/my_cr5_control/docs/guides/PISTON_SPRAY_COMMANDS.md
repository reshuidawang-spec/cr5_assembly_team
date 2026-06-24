# 活塞喷涂可执行命令

## 1. 工作区编译

离线 GUI 只编译 `my_cr5_control` 即可：

```bash
cd ~/dobot_ws
colcon build --packages-select my_cr5_control --symlink-install
```

真实机械臂联机建议一起编译：

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
```

## 2. 加载环境

```bash
cd ~/dobot_ws
source install/setup.bash
```

## 3. 从工作区直接启动喷涂 GUI

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 run my_cr5_control piston_spray_gui_node
```

## 4. 从工作区一键启动真实机械臂喷涂 GUI

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 launch my_cr5_control piston_spray_real_robot.launch.py
```

如果机械臂 IP 不是默认值：

```bash
cd ~/dobot_ws
source install/setup.bash
ros2 launch my_cr5_control piston_spray_real_robot.launch.py robot_ip:=192.168.5.1
```

## 5. 无界面环境烟测启动

```bash
cd ~/dobot_ws
source install/setup.bash
QT_QPA_PLATFORM=offscreen ros2 run my_cr5_control piston_spray_gui_node
```

## 6. 运行基础 Python 示例规划器

```bash
cd ~/dobot_ws
python3 piston/basic_spray_planner.py
```

## 7. 从交付目录直接启动基础版

```bash
cd ~/dobot_ws/piston_industrial_delivery
./app/run_piston_spray_gui.sh
```

## 8. 从交付目录直接启动真机版

```bash
cd ~/dobot_ws/piston_industrial_delivery
./app/run_piston_spray_real_robot.sh
```

如果需要指定 IP：

```bash
cd ~/dobot_ws/piston_industrial_delivery
PISTON_SPRAY_ROBOT_IP=192.168.5.1 ./app/run_piston_spray_real_robot.sh
```

## 9. 安装桌面启动按钮

```bash
cd ~/dobot_ws/piston_industrial_delivery
./app/install_desktop_button.sh
```

## 10. 刷新交付目录

当源码、可执行程序或示例文件更新后，执行：

```bash
cd ~/dobot_ws/piston_industrial_delivery
./tools/refresh_delivery_bundle_from_workspace.sh
```

如果只想刷新运行时副本：

```bash
cd ~/dobot_ws/piston_industrial_delivery
./tools/refresh_runtime_bundle_from_workspace.sh
```

## 11. 在目标工作区重建交付源码

```bash
cd ~/dobot_ws/piston_industrial_delivery
./tools/rebuild_target_workspace.sh /path/to/target_ws
```

## 12. 常用说明

- 当前喷涂 GUI 支持：
  - 参数录入
  - 轨迹可视化
  - 基础执行状态机
  - 末端 TCP 示教保存与微调
- 真机一键启动会自动带起：
  - `cr_robot_ros2`
  - `dobot_moveit/action_move_server`
  - `dobot_moveit/joint_states`
  - `cr5_moveit` 的 `robot_state_publisher / move_group / RViz`
- 当前版本不连接转台真实控制接口。
