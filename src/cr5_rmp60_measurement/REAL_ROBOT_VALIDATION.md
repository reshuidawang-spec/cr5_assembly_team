# CR5 + RMP60 真机恢复验证清单

本文档用于机械臂重新可用后，按安全顺序恢复验证。不要跳过前置步骤直接执行任意方向真实运动。

当前工程总览见 `PROJECT_OVERVIEW.md`。

## 当前状态

- 仿真侧已完成 CR5 + RMP60 工具模型、任意方向 pose 生成、MoveIt 预检查、批量报告。
- `data/measurement_plan_report.json` 和 `data/measurement_plan_report.csv` 已生成，当前多角度示例为 4/4 `PASS`。
- 任意方向真实执行仍禁用；`execute_measurement_plan.py --execute` 会拒绝。
- 真实执行前必须确认 Dobot `rx/ry/rz` 姿态约定和 DI1 触发后的 Stop 响应。
- 竖直测针已于 2026-05-29 安装完成，并已通过 Z- 5mm 单次触碰、固定起点 3 次重复性、3 点短线和 2x2 小网格恢复验证；旧竖直真实测量流程仍需显式现场确认。
- 当前已真实验证十字测针 `y_neg` 分支沿基坐标 `X-` 横向触碰；标准姿态在 `config/measurement_poses.yaml`。
- 当前 `speed=1` 的 X- 横向 Stop 过冲均值约 `1.51mm`，后续横向测量至少预留 `2mm` 余量。

## 当前推荐真机验证顺序

1. 确认 DI1/FeedInfo 和 `GetPose/DI` 服务读数正常。
2. 手动触发/释放测头，确认 DI1 跟随变化。
3. 用竖直测针做低速 Stop 响应验收。
4. 做固定起点 3 次竖直重复性测试。
5. 再恢复短线、小网格和横向批量流程。

竖直测针已修复或更换后，必须从下面第 1 节重新开始。2026-05-29 已完成第 2、3、4 节核心恢复验证，并已完成一次 3 点短线和 2x2 小网格验收；后续正式采集仍需按工件和路径逐次确认。

## 0. 启动和确认

启动 ROS2 驱动后，先确认 DI1 和 FeedInfo 正常：

```bash
source /opt/ros/humble/setup.bash
source ~/dobot_ws/install/setup.bash
cd /home/zhu/dobot_ws/src/cr5_rmp60_measurement
./scripts/probe_di_monitor.py
```

验收：
- 未触发时显示 DI1 未触发。
- 手动触发测头/接收器时显示 DI1 触发。
- 释放后 DI1 回到未触发。

如果这一步不稳定，停止后续所有运动测试。

## 1. 姿态约定只读验证

把机械臂停在一个安全、容易观察工具轴方向的姿态。不要移动机器人，只读取 `GetPose` 并比较工具本地 `+Z` 在基坐标系中的实际方向：

```bash
./scripts/validate_cr5_pose_convention.py \
  --expected-axis DX DY DZ \
  --json-output data/cr5_pose_convention_real.json
```

示例：如果实际观察工具轴大致朝基坐标 `X+` 和 `Z-`：

```bash
./scripts/validate_cr5_pose_convention.py \
  --expected-axis 1 0 -1 \
  --json-output data/cr5_pose_convention_real.json
```

验收：
- `xyz` / ROS RPY 候选约定应是误差最小或与真实约定一致。
- 若不是 `xyz`，不得使用当前候选法兰 `MovL(rx,ry,rz)` 进行任意方向真实执行。

2026-05-22 当前结果：
- 用户观察当前 RMP60 本体中心轴大致朝基坐标 `Z-`。
- 已运行 `--expected-axis 0 0 -1` 并保存到 `data/cr5_pose_convention_real.json`。
- `xyz` 候选误差约 `4.8deg`，但与 `yxz`、`XYZ`、`YXZ` 并列最小。
- 该姿态接近竖直，不能唯一确定欧拉角约定；仍需在明显非竖直姿态下再验证一次。

2026-05-22 第二次结果：
- 用户随后观察当前测头主轴大约朝基坐标 `X-`。
- 已运行 `--expected-axis -1 0 0` 并保存到 `data/cr5_pose_convention_real_xneg.json`。
- 控制器读取位姿与上一轮相同；最小候选 `xzy/ZXY` 误差约 `33.6deg`，`xyz` 误差约 `85.8deg`。
- 该结果与 `Z-` 观察冲突，不能作为姿态约定确认依据。需要先明确观察的是工具本体轴、横杆轴还是接触方向，再用明显非竖直姿态验证。

## 2. 软件 Stop 链路低风险验证

先使用极短下探距离和最低速度。确保人员手在急停附近，测头附近留有足够空间。

```bash
./scripts/probe_touch.py \
  --execute \
  --approach-mm 1 \
  --retract-mm 8 \
  --speed 1 \
  --timeout 10 \
  --output data/real_stop_validation.csv
```

验收：
- 触发 DI1 后脚本立即发送 Stop。
- 机器人停止后执行回退。
- CSV 中记录触发瞬间 `flange_*` 和 Stop 后 `stop_flange_*`。
- 如果出现 FeedInfo 变 stale、DI1 无变化、Stop 不及时或回退方向不正确，停止后续测试。

## 3. Stop 过冲重复验证

通过多次短下探观察 Stop 后过冲量：

```bash
./scripts/repeat_probe_test.py \
  --execute \
  --cycles 3 \
  --approach-mm 1 \
  --retract-mm 8 \
  --speed 1 \
  --timeout 10 \
  --output data/real_stop_repeat_validation.csv
```

分析结果：

```bash
./scripts/analyze_probe_repeatability.py \
  --input data/real_stop_repeat_validation.csv
```

验收：
- 每次触发后都能回到安全位姿。
- `stop_flange_*` 相对触发瞬间的过冲量稳定且在机械安全余量内。
- DI1 释放后能回到未触发。

2026-05-29 固定起点 3 次重复结果：

```text
fixed safe start:
[-344.7692, 148.4787, 355.4291, -179.3460, -1.4400, 124.6769]

triggered cycles: 3/3
flange_z mean: 352.3769 mm
flange_z sample std: 0.0266 mm
flange_z range: 352.3613 .. 352.4076 mm
final pose: [-344.7690, 148.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_probe_repeat_20260529.csv
```

## 4. 线测量真实验收

姿态参数确认后，再测一条短线。先使用 3 个点，确认无误后再增加到 5 个点：

```bash
./scripts/measure_line.py \
  --execute \
  --x1 360 --y1 429 \
  --x2 362 --y2 429 \
  --points 3 \
  --safe-z 325 \
  --approach-mm 3 \
  --speed 1 \
  --rx RX --ry RY --rz RZ \
  --output data/line_real_validation.csv
```

验收：
- 每个点都先到 safe pose，再低速触碰。
- 每个点触发或失败都有输出。
- 每点结束后回到 safe pose。

2026-05-29 3 点短线结果：

```text
line: X=-344.7692, Y=148.4787..150.4787 mm
safe_z: 355.4291 mm
approach: Z- 5 mm
points: 3/3 triggered
final pose: [-344.7690, 150.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_line_validation_20260529.csv
```

## 5. 网格测量真实验收

线测量通过后，再做小范围 3x3 网格：

```bash
./scripts/measure_grid.py \
  --execute \
  --x-min 360 --x-max 362 \
  --y-min 429 --y-max 431 \
  --rows 3 --cols 3 \
  --safe-z 325 \
  --approach-mm 3 \
  --speed 1 \
  --rx RX --ry RY --rz RZ \
  --output data/grid_real_validation.csv
```

验收：
- 9 个点都明确成功或失败。
- 任一点失败时，默认停止；若使用 `--continue-on-error`，必须人工确认失败点不影响后续运动安全。

2026-05-29 2x2 小网格结果：

```text
grid: X=-344.7692..-344.2692 mm, Y=150.4787..151.4787 mm
safe_z: 355.4291 mm
approach: Z- 5 mm
retract: Z+ 8 mm
points: 4/4 triggered
flange_z mean: 352.3802 mm
flange_z sample std: 0.0489 mm
final pose: [-344.2690, 151.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_grid_validation_20260529.csv
```

## 6. 任意方向真实执行解锁条件

只有以下条件全部满足，才允许设计任意方向真实执行开关：

- DI1 监控稳定，触发和释放都可靠。
- Dobot `rx/ry/rz` 姿态约定已通过真机只读验证。
- 短下探 Stop 响应和过冲已记录并评估。
- 线测量和 3x3 网格真实验收通过。
- 当前测头、法兰、测针长度参数与真实硬件一致，或误差已被记录。
- `data/measurement_plan_report.csv` 中待执行点全部为 `PASS`。

在这些条件满足前，`execute_measurement_plan.py` 的真实执行保持禁用。
