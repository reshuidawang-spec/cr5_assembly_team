# CR5 + RMP60 工具模组说明

本文档说明 CR5 末端工具模组的 URDF/xacro 结构，以及后期更换测头、测针、夹具尺寸时应该修改哪些参数。

## 文件结构

| 文件 | 作用 |
|------|------|
| `urdf/rmp60_probe_module.xacro` | 可复用的测头/测针工具宏。 |
| `urdf/cr5_with_rmp60.urdf.xacro` | 将工具宏安装到 CR5 `Link6` 末端。 |
| `urdf/cr5_moveit_with_rmp60.urdf.xacro` | MoveIt demo 使用的 CR5 + 工具模型，包含 ros2_control。 |
| `config/moveit/cr5_rmp60.srdf` | 将 MoveIt `cr5_group` 的 tip 设置为 `rmp60_tip`。 |
| `launch/view_cr5_rmp60_model.launch.py` | 启动 robot_state_publisher 和 RViz，可视化完整模型。 |
| `launch/cr5_rmp60_moveit_demo.launch.py` | 启动以 `rmp60_tip` 为末端的 MoveIt/RViz demo。 |
| `config/rviz/cr5_rmp60_model.rviz` | RViz 配置，显示 RobotModel 和 TF。 |

## 当前默认几何

默认单位是米：

| 参数 | 默认值 | 含义 |
|------|--------|------|
| `adapter_length` | `0.0494` | 法兰/转接段有效轴向长度，按图中 `5 + 40 + 4.4 = 49.4mm`。 |
| `adapter_radius` | `0.030` | 法兰/转接段显示半径。 |
| `probe_body_length` | `0.076` | RMP60 本体近似长度。 |
| `probe_body_radius` | `0.0315` | RMP60 本体近似半径。 |
| `stylus_length` | `0.075` | 十字测针各方向球中心距，当前按 75mm。 |
| `stylus_radius` | `0.0015` | 测针杆半径，仅用于显示/碰撞近似。 |
| `stylus_ball_radius` | `0.001` | 测针球头半径；“小球 2.0”按直径 2.0mm 处理。 |
| `tool_axis_sign` | `1` | 工具沿 `Link6` 的 `+Z` 方向伸出。 |
| `mount_xyz` | `0 0 0` | 工具相对 `Link6` 的安装平移。 |
| `mount_rpy` | `0 0 0` | 工具相对 `Link6` 的安装旋转。 |

当前 `Link6 -> rmp60_tip` 总长度为：

```text
adapter_length + probe_body_length + stylus_length = 0.2004 m
```

默认 TF 平移为 `Link6 -> rmp60_tip = [0, 0, +0.2004]`，即测头和测针从法兰外侧伸出。
其中机器人法兰到十字测针分支原点为 `49.4 + 76.0 = 125.4 mm`，再加轴向测球中心距 `75.0 mm` 得到轴向 tip 总长 `200.4 mm`。

十字测针当前还额外提供可视化和标定用 frame：

| Frame | 含义 |
|------|------|
| `rmp60_tip` | 兼容现有 MoveIt 配置的轴向 tip。 |
| `rmp60_tip_z` | 轴向测针球心 frame。 |
| `rmp60_tip_x_pos` | 横向 `+X` 测球中心。 |
| `rmp60_tip_x_neg` | 横向 `-X` 测球中心。 |
| `rmp60_tip_y_pos` | 横向 `+Y` 测球中心。 |
| `rmp60_tip_y_neg` | 横向 `-Y` 测球中心。 |

横向十字测针目前只作为可视化和 TF frame，不加入碰撞几何，避免在现有 MoveIt SRDF 未同步禁碰前产生误报。

## 替换测针尺寸

如果只更换测针球中心距，例如 75mm 改成 100mm：

```bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py stylus_length:=0.100
```

如果球头半径从 3mm 改成 4mm：

```bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py stylus_ball_radius:=0.004
```

## 替换测头型号

如果更换测头本体，只需要改本体长度和半径，例如：

```bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py probe_body_length:=0.090 probe_body_radius:=0.035
```

如果新测头安装方向不是沿 `Link6 +Z`，优先修改：

```bash
tool_axis_sign:=-1
```

或通过 `mount_rpy` 设置安装旋转。

如果后续确认 `Link6` 坐标系和实际法兰安装面存在偏移，通过 `mount_xyz` 修正，例如沿 `Link6 +Z` 方向把工具整体外移 5mm：

```bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py mount_xyz:='0 0 0.005'
```

## 验证工具长度

生成 URDF 并检查结构：

```bash
source /opt/ros/humble/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
xacro urdf/cr5_with_rmp60.urdf.xacro > /tmp/cr5_with_rmp60.urdf
check_urdf /tmp/cr5_with_rmp60.urdf
```

启动 RViz 后检查末端 tip 相对 `Link6` 的 TF：

```bash
source /opt/ros/humble/setup.bash
ros2 run tf2_ros tf2_echo Link6 rmp60_tip
```

默认应看到接近：

```text
Translation: [0.000, 0.000, 0.200]
```

## 当前边界

- 这是几何可视化和近似碰撞模型，不是最终标定结果。
- RMP60 本体尺寸目前用近似圆柱表示；后续可以替换为真实 mesh。
- 十字测针横向杆已做可视化和 TF frame；横向碰撞模型和横向真实执行安全门仍未完成。
- 项目内 MoveIt demo 已使用 `rmp60_tip` 作为规划 tip；厂家原始 `cr5_moveit` demo 仍使用 `Link6`。
- IK 检查脚本会自动根据接近方向生成 `rmp60_tip` 姿态；需要覆盖时可使用 `--manual-orientation`。
