# CR5 + RMP60 仿真开发说明

当前没有真机可用时，软件开发转到 RViz/MoveIt 仿真。

## 已有模型资源

工作空间已有 CR5 相关模型和 MoveIt 配置：

| 资源 | 路径 |
|------|------|
| CR5 MoveIt 包 | `/home/zhu/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr5_moveit` |
| CR5 MoveIt URDF xacro | `/home/zhu/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/cr5_robot.urdf.xacro` |
| CR5 SRDF | `/home/zhu/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cr5_moveit/config/cr5_robot.srdf` |
| CR5 描述模型 | `/home/zhu/dobot_ws/src/DOBOT_6Axis_ROS2_V4/cra_description/urdf/cr5_robot.xacro` |
| RViz URDF | `/home/zhu/dobot_ws/src/DOBOT_6Axis_ROS2_V4/dobot_rviz/urdf/cr5_robot.urdf` |
| 现有 MoveIt demo | `/home/zhu/dobot_ws/src/my_cr5_control/launch/cr5_moveit_stable_demo.launch.py` |

## 启动 RViz / MoveIt

优先使用带 RMP60 tip 的项目 demo。这个 demo 将 `cr5_group` 的 MoveIt tip 设置为 `rmp60_tip`：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/cr5_rmp60_moveit_demo.launch.py
```

如果只需要原始 CR5 模型，不带 RMP60 工具，使用已有稳定 demo：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 launch my_cr5_control cr5_moveit_stable_demo.launch.py
```

## 测量方向建模

真实测量不会只有竖直向下或水平触碰。后续软件应把测量动作抽象为：

```text
contact_point: 期望接触点 [x,y,z]
approach_vector: 测头接近方向 [dx,dy,dz]，单位向量
standoff_mm: 接触前安全距离
travel_mm: 沿 approach_vector 的下探/前探距离
```

轨迹点定义：

```text
safe_pose_position = contact_point - approach_vector * standoff_mm
target_pose_position = contact_point + approach_vector * travel_mm
tip_orientation = 让 rmp60_tip 本地 +Z 方向对齐 approach_vector 的四元数
```

说明：
- `approach_vector` 指向“机器人运动接近工件”的方向。
- 竖直向下触碰时，`approach_vector = [0, 0, -1]`。
- 从 X- 往 X+ 触碰时，`approach_vector = [1, 0, 0]`。

## 当前仿真开发任务

1. 生成任意方向测量 pose 序列。
2. 在 RViz 中显示这些 pose / marker。
3. 接入 MoveIt，验证这些 pose 是否可达。
4. 再把可达 pose 转换为真实机器人测量命令。

## RViz Marker 可视化

发布一个斜向接近测量动作：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/visualize_measurement_poses.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5 --frame-id base_link
```

在 RViz 中添加：

```text
Add -> By topic -> /rmp60_measurement_markers -> MarkerArray
```

Marker 含义：
- 黄色球：期望接触点。
- 蓝色方块：接触前安全点。
- 红色方块：经过接触点后的目标点。
- 绿色箭头：safe 到 target 的接近运动方向。

默认输入单位是 mm，脚本发布到 RViz 时会转换为 m。若 RViz 看不到 Marker，优先检查 `Fixed Frame` 是否为 `base_link`，或把脚本的 `--frame-id` 改成当前 RViz 使用的固定坐标系。

项目内也提供了预置 MarkerArray 的 RViz 配置：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
rviz2 -d /home/zhu/dobot_ws/src/cr5_rmp60_measurement/config/rviz/rmp60_measurement_markers.rviz
```

## MoveIt IK 可达性检查

启动 MoveIt demo 后，可以检查 safe/contact/target 三个位置是否能求出 IK：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/check_measurement_reachability.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5
```

当前检查默认以 `rmp60_tip` 作为 IK link，目标点就是测针球头位置。姿态会根据 `approach_vector` 自动生成：`rmp60_tip` 本地 `+Z` 方向对齐接近方向。若要手动指定四元数，可加 `--manual-orientation --qx ... --qy ... --qz ... --qw ...`。

## MoveIt 路径规划显示

启动 RMP60 MoveIt demo 后，可以规划并显示从当前状态到 safe/contact/target 的三段路径：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/plan_measurement_path.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5
```

脚本会向 `/display_planned_path` 发布规划轨迹。RViz 中使用 MotionPlanning/Planned Path 显示该话题；测量 safe/contact/target marker 仍由 `/rmp60_measurement_markers` 显示。

当前 demo 使用 `config/moveit/rmp60_initial_positions.yaml` 初始化 fake ros2_control，避免全零腕部位姿下 RMP60 工具与 `Link5` 的自碰撞误报。SRDF 中也关闭了 `Link5` 与 RMP60 固定工具几何之间的自碰撞检查。

## 直线探测段规划

OMPL 分段规划可以验证姿态和目标点可达，但真实触碰时 safe 到 target 应尽量保持直线。使用 Cartesian path 检查探测段：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/plan_probe_cartesian_path.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5
```

脚本先对 `safe_position` 求 IK，再用 `/compute_cartesian_path` 生成 `safe_position -> contact -> target_position`。输出中的 `fraction=1.000` 表示直线探测段完整生成。

## 真实执行前预检查

任意方向测量路径进入真实执行前，先使用预检查脚本汇总验证：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/preflight_measurement_plan.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5
```

检查内容：
- `Link6 -> rmp60_tip` TF 是否为外伸方向，默认应接近 `[0, 0, +0.2004]`。
- safe/contact/target 三个末端位姿是否 IK 可达。
- 当前状态到 safe 是否能规划。
- safe/contact/target 直线探测段是否 `fraction=1.000`。

该脚本只调用 MoveIt/TF/RViz 仿真接口，不调用 Dobot 真实运动服务。

## 多角度批量计划

当一个工件需要从多个方向触碰时，先用 CSV 写出每个接触点和接近方向，再生成统一 JSON：

```bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/generate_measurement_plan_from_csv.py --input data/multi_angle_points_example.csv --output data/multi_angle_measurement_plan.json
```

CSV 必填列为 `x,y,z,dx,dy,dz`，其中 `dx,dy,dz` 是测针向接触点前进的方向。可选列 `name,standoff_mm,travel_mm` 用于给每个点单独命名和覆盖距离。

可视化：

```bash
./scripts/visualize_measurement_poses.py --input data/multi_angle_measurement_plan.json
```

批量预检查：

```bash
./scripts/preflight_measurement_plan.py --input data/multi_angle_measurement_plan.json --no-display
```

当前示例包含竖直、两个斜向和一个侧向测量点；在 MoveIt demo 后端运行时已验证全部预检查 `PASS`。

## 任意方向执行 dry-run

真实执行脚本目前只做 dry-run 脚手架：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/execute_measurement_plan.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5
```

它会复用预检查逻辑，打印未来真实执行状态机，并把计划写入 `data/measurement_plan_dry_run.json`。当前版本显式禁用 `--execute`，避免在真实任意方向执行逻辑未完成前误运动。

## Tip 到法兰姿态转换

MoveIt 中的测量目标是 `rmp60_tip`，真实 CR5 `MovL` 使用法兰位姿 `x/y/z/rx/ry/rz`。当前提供离线评审工具：

```bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/review_flange_pose_conversion.py --contact 360 429 180 --approach 1 0 -1 --standoff-mm 20 --travel-mm 5 --json-output data/flange_pose_conversion_review.json
```

它会根据 `Link6 -> rmp60_tip = [0,0,+0.2004]` 反算候选法兰位置，并输出 ROS RPY 候选角。输出中的 `reconstruction_error_mm=0` 表示由候选法兰位姿重建 tip 位姿无误；`tool_axis_vs_approach_angle_deg=0` 表示测针轴与接近方向一致。

边界：Dobot 控制器的 `rx/ry/rz` 解释仍需真机 `GetPose` 对照确认，确认前不能把该候选姿态用于真实执行。

## CR5 姿态约定验证

任意方向测量最终要把 `rmp60_tip` 目标姿态转换成 CR5 的法兰 `MovL(x,y,z,rx,ry,rz)`。目前 `rx/ry/rz` 使用 ROS RPY 候选约定，必须在真机上做只读对照。

离线验证示例：

```bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/validate_cr5_pose_convention.py --pose 0 0 0 135 0 90 --expected-axis 1 0 -1 --json-output data/cr5_pose_convention_review.json
```

真机可用后，先把机械臂停在一个方向容易观察的安全姿态，不移动机器人，只读取 `GetPose`：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/validate_cr5_pose_convention.py --expected-axis DX DY DZ --json-output data/cr5_pose_convention_real.json
```

`DX DY DZ` 填写实际观察到的工具本地 `+Z` 方向。输出里 `axis_error` 最小的候选才可以作为真实运动转换依据。

## CR5 + RMP60 工具模型

项目内新增了可替换的测头工具模组：

```text
urdf/rmp60_probe_module.xacro
urdf/cr5_with_rmp60.urdf.xacro
urdf/cr5_moveit_with_rmp60.urdf.xacro
config/moveit/cr5_rmp60.srdf
```

启动完整模型可视化：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py
```

默认几何：
- 法兰/转接段有效轴向长度：49.4mm
- RMP60 本体近似长度：76mm
- 十字测针各方向球中心距：75mm
- 小球直径：2.0mm
- 工具沿 `Link6 +Z` 方向伸出

更换测针长度示例：

```bash
ros2 launch /home/zhu/dobot_ws/src/cr5_rmp60_measurement/launch/view_cr5_rmp60_model.launch.py stylus_length:=0.100
```

详细维护说明见 `TOOL_MODULE.md`。

## 批量测量计划报告

当拿到一个包含多个测量方向的 JSON 计划后，可以一次性跑完所有 MoveIt 预检查并生成结构化报告，用于在真机执行前筛选不可达或规划失败的点位：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/report_measurement_plan.py --input data/multi_angle_measurement_plan.json
```

也可以输出到单独目录，目录会自动创建：

```bash
./scripts/report_measurement_plan.py \
  --input data/multi_angle_measurement_plan.json \
  --json-output data/reports/measurement_plan_report.json \
  --csv-output data/reports/measurement_plan_report.csv
```

报告产出两个文件：

| 文件 | 用途 |
|------|------|
| `data/measurement_plan_report.json` | 每个点的 IK、规划、笛卡尔段和法兰转换的完整结果 |
| `data/measurement_plan_report.csv` | 每个点的 PASS/FAIL 摘要，可直接用电子表格打开筛选 |

典型工作流：

```text
CSV 点位 → generate_measurement_plan_from_csv.py → JSON
                                                         ↓
                                              report_measurement_plan.py
                                                         ↓
                                              JSON + CSV 报告
                                                         ↓
                                        筛掉 FAIL → 留存 PASS → 等真机执行
```

空计划会被拒绝，避免出现 `0 poses, overall PASS` 的误判。

报告中的 `FAIL` 条目通常意味着：
- 该方向超出机械臂可达空间 → 调整 `contact` 位置或 `approach` 方向
- 路径规划失败 → 增大 `--planning-time` 或 `--attempts`
- 笛卡尔直线段不完整（`fraction < 1.0`）→ 检查 detour 或碰撞，调整 `standoff`/`travel`
- 工具 TF 长度/方向与预期不符 → 检查 xacro 参数是否匹配真实硬件
