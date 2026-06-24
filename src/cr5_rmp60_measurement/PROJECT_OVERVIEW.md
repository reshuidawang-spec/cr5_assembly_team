# CR5 + RMP60 工程总览

本文档是当前工程空间的入口。`COMMANDS.md` 保存可执行命令，`ROADMAP.md` 保存开发路线，`PROJECT_MEMORY.md` 保存历史记录，`CROSS_PROBE_CALIBRATION_STATUS.md` 保存十字测针多姿态标定的当前结论和下一步流程。本文件只保留最重要的当前事实、风险和下一步。

## 当前目标

CR5 机械臂搭载 Renishaw RMP60 测头，实现接触式测量。竖直 Z 向测针已于 2026-05-29 安装完成，并已完成一次 Z- 5mm 触碰和固定起点 3 次重复性恢复验证；硬工件平面 3x3/5x5 采集也已完成阶段性验证。

当前核心目标已经转向十字测针工具几何标定：先对同一颗物理红宝石球，例如当前 `y_neg` 分支，使用 20mm 标准球做多姿态绝对标定，求解标准球球心 `C_s` 和该分支红宝石球在法兰坐标系下的真实 offset `p`。固定姿态四方向低过冲相对标定已经完成，但它不是完整绝对 TCP/分支 offset，不能直接覆盖最终几何配置。半自动法向采集脚本已经实现，下一步是操作者确认安全近球位置后，用脚本根据当前初值计算两球心连线方向并短距离低速触碰。

十字测针多姿态标定的最新结论见：

```text
CROSS_PROBE_CALIBRATION_STATUS.md
```

## 当前硬件状态

- CR5 控制柜 IP：`192.168.5.1`。
- RMP60 接收器已接 CR5 控制柜 `DI1`。
- ROS2 驱动可读 `/dobot_bringup_ros2/srv/DI` 和 `/dobot_bringup_ros2/msg/FeedInfo`。
- 竖直 Z 向测针已重新安装，旧竖直真实测量流程仍保持受保护状态；单次 Z- 5mm、固定起点 3 次重复性、3 点短线、2x2 小网格和硬工件 3x3/5x5 采集已通过阶段性验证。
- 当前使用十字型测针，已真实验证标定球四个水平接近方向；工件面采集仍必须按具体装夹状态重新注册批准面。

## 已验证的真实结果

### DI1 和停止链路

- DI1 未触发时可读为 `{0}`。
- 横向触碰触发后，脚本能通过 FeedInfo 检测 DI1 并发送 `Stop()`。
- 当前停止链路是软件 Stop，不是硬实时安全回路；现场人员仍必须守急停。

### 十字测针 X- 横向触碰

已验证配置：

```text
branch = y_neg
approach = [-1, 0, 0]
standard_pose = x_neg_y_neg_verified
rx=-175.2180, ry=-0.4330, rz=123.3110
speed = 1
distance = 5mm
```

标准姿态配置位置：

```text
config/measurement_poses.yaml
```

十字测针名义几何配置位置：

```text
config/cross_probe_geometry.yaml
```

固定起点 3 次重复触碰结果：

```text
fixed safe start:
[-245.1260, 136.8420, 219.4730, -175.2180, -0.4330, 123.3110]

trigger_flange_x mean: -248.5175 mm
trigger_flange_x sample std: 0.0392 mm
trigger_flange_x range: -248.5403 .. -248.4722 mm

stop_overtravel_along_approach mean: 1.5133 mm
stop_overtravel_along_approach sample std: 0.0341 mm
stop_overtravel_along_approach range: 1.4790 .. 1.5471 mm
```

抬高工件和机械臂后的单次触碰结果：

```text
raised safe start:
[-247.5230, 96.1100, 289.1730, -175.2180, -0.4330, 123.3110]

trigger flange:
[-250.2176, 96.1058, 289.1763, -175.2169, -0.4333, 123.3123]

stop flange:
[-251.2031, 96.1100, 289.1730, -175.2180, -0.4330, 123.3110]

stop_overtravel_along_approach: 0.9855 mm
final retracted pose:
[-245.5230, 96.1100, 289.1730, -175.2180, -0.4330, 123.3110]
```

当前结论：旧的单段 `5mm` 触碰在 `speed=1` 下已观察到约 `1.44..2.47mm` Stop 过冲。2026-06-01 已把触碰入口改为默认 `--probe-step-mm 0.5` 分段探测，并将分段到达判定收紧到 `0.03mm`。四方向低过冲复测结果：X- `0.0421..0.0436mm`，Y+ `0.2641..0.3070mm`，X+ `0.4114..0.4257mm`，Y- `-0.0082..0.4735mm`。使用该低过冲数据重新标定后，四方向相对球心一致性 RMS 从旧数据 `0.0551mm` 降到 `0.0134mm`。正式测量规划建议至少保留 `0.8..1.0mm` 方向性停止余量，直到在真实工件面上复核。

Stop 过冲的来源是完整停止链路的累计延迟：测针机构触发、RMP60/接收器输出 DI、控制器 FeedInfo 刷新、ROS 脚本检测 DI1、发送 `Stop()`、机械臂执行减速停止。当前记录的 `stop_overtravel_along_approach` 是触发反馈位姿到最终停止位姿之间的实际距离，不是几何模型补偿项；因此既要统计它，也要在路径和工件间隙中预留它。

## 当前最重要的事情

1. **当前先做十字测针单分支绝对标定**

   当前不要急着进入正式工件形位公差评价。先选定一颗物理红宝石球，例如 `y_neg`，用同一颗球触碰同一个 20mm 标准球，联合求解标准球球心和该分支 offset。`data/cross_probe_sphere_calibration_low_overtravel_20260601.json` 只能作为固定姿态相对标定参考，不是最终绝对工具几何。

2. **不要把横向分支的纯 `z-` 当作常规有效点**

   `data/260603.1` 追加 sample 6 的 `z-` 点在 `y_neg` 拟合中残差约 `9.01mm`，明显不符合当前分支有效接触。对十字测针横向分支，纯 `z-` 很可能碰到竖直针、测杆、十字中心或非目标部位。后续除非现场明确确认目标红宝石球会先接触标准球，否则不要把 `z-` 纳入该分支标定。

3. **下一步是半自动法向采集，不是完全无人值守**

   当前系统能读法兰位姿、姿态、DI1 和 FeedInfo，也有初步 `C_s / p` 拟合结果，但系统还不知道标准球支架、其他测针分支、RMP60 本体和工作台边缘的完整碰撞关系。合理流程是操作者把目标球尖放到安全近球位置并确认净空，程序计算 `approach = normalize(C_s - C_p)`，执行短距离低速触碰、记录、回退和重新拟合。

4. **真实测量必须约束姿态**

   十字测针不是各向同性工具。真实测量不能只给 `x/y/z`，必须同时约束 `rx/ry/rz`，并且方向、分支、姿态要成组配置。
   姿态约束的目标不是简单要求“机械臂末端垂直于地面”，而是保证当前参与触碰的测球分支、接近方向、被测面法向和避障空间一致。测竖直面时，RMP60 本体中心轴保持近似竖直、十字横杆水平通常是一个方便且可复现的标准姿态；但最终应以“哪个分支触碰、沿哪个基坐标方向接近、其它分支和本体是否避让工件”为准。

5. **竖直测针已安装，但先重新验收**

   2026-05-29 竖直测针已安装，但这只改变硬件可用性，不等于恢复全部测量资格。`probe_touch.py`、`repeat_probe_test.py`、`measure_line.py`、`measure_grid.py` 仍需要显式 `--ack-vertical-stylus-ok`。当前已完成 DI1/FeedInfo 只读确认、Z- 5mm 单次触碰、固定起点 3 次重复性、3 点短线和 2x2 小网格恢复验证。
   2026-05-27 曾出现 ROS 服务 `GetPose/DI` 返回 `res=0` 但 `robot_return` 为空、驱动日志 `tcpDoCmd failed` 的情况；恢复真机运动前必须确认服务读数和 FeedInfo 都正常。

6. **标定球四方向已验收，但工件面仍要单独注册**

   20mm 标定球 `X- / X+ / Y+ / Y-` 已建立标准姿态并完成固定起点重复触碰。这只证明标定球场景下四方向链路可用；正式工件的每个被测面仍必须重新教导安全起点、批准面、移动范围和回退空间，不能直接套用标定球安全坐标。

7. **`rx/ry/rz` 姿态约定仍需真机只读确认**

   代码当前按 ROS RPY 候选约定处理几何换算。已在当前姿态下做过两次只读验证：按 `Z-` 观察时 `xyz` 候选误差约 `4.8deg` 并列最小；按 `X-` 观察时最小误差仍约 `33.6deg`，且控制器位姿未变化。两次观察结果冲突，说明当前还没有可靠确认 Dobot `rx/ry/rz` 约定。正式多面测量前，需要重新明确观察的是哪根轴，并在明显非竖直姿态下再验证。

8. **`data/cross_probe_contacts.csv` 是混合实验记录**

   该文件包含多次验证和后续尝试数据。做正式分析时，必须按时间段、固定起点和现场状态筛选，不要把所有行直接混在一起做精度结论。

9. **当前几何参数是名义值，不是最终标定值**

   十字测针球心距、RMP60 本体长度、法兰转接长度都来自图纸或估算。当前可用于流程验证，不可作为最终测量精度保证。

10. **先标定工具几何，再重构工件**

   不能把十字测针四个方向触发数据直接用于未知盒子的最终重构。若工具几何和工件几何同时未知，误差会互相吸收，难以判断是测针 offset、姿态约定、Stop 过冲还是工件自身形状导致偏差。下一阶段应先采集可用于标定的横向触碰数据，拟合连接法兰、RMP60、十字测针各分支相对于法兰的真实几何，再用标定后的工具模型去重构平面和立方体。

## 当前应该做什么

当前优先级是把十字测针横向测量从“人工猜方向触碰标准球”推进到“单分支多姿态绝对标定 + 半自动法向采集”。

1. **继续 `y_neg` 分支，不混分支**

   当前先固定同一颗物理红宝石球。每次采集前确认 GUI/CSV 中的 `branch=y_neg` 与现场实际触碰球一致。不要把 `x_neg`、`y_neg` 或口头“1号球”的历史命名混在一起做同一个分支拟合。

2. **以当前 5 点结果作为临时初值**

   当前阶段性参考文件：

   ```text
   data/260603.1/yneg_new_5pts_absolute_fit_20260603.json
   ```

   该结果 RMS `1.434982mm`，只能作为半自动法向采集初值，不能作为最终标定参数。`yneg_current_7pts_absolute_fit_20260603.json` 包含明显异常 `z-` 点，不应作为几何参数来源。

3. **使用半自动法向采集脚本**

   独立脚本已实现：

   ```text
   scripts/semi_auto_normal_probe.py
   ```

   当前状态机：

   ```text
   读取当前法兰位姿
   -> 根据临时 C_s / p 计算当前红宝石球心 C_p
   -> approach = normalize(C_s - C_p)
   -> 操作者确认短距离路径安全
   -> 低速短距离 MovL 搜索
   -> DI1 触发 Stop
   -> 反向回退
   -> 写 CSV
   -> 重新拟合并输出该点残差
   ```

   该流程仍必须人工确认安全，不做无人值守全自动。脚本默认 dry-run；真实短距离运动必须显式加 `--execute --ack-safe-normal-probe`，且 `--search-mm` 限制为 `<= 1.0mm`。

4. **用环规做交叉验证**

   单分支 RMS 进入亚毫米级后，再用 100mm 环规验证圆心、半径和圆度一致性。环规用于验证标定补偿，不替代标准球多姿态标定。

5. **教导并注册当前工件安装状态**

   人工将测球移动到目标竖直面有效区域前方并确认定位、搜索和回退路径安全。使用 `teach_cross_probe_face.py` 保存为 `taught`，复核后显式批准为 `approved`。工件、夹具或测头安装关系变化后，必须创建新的 `setup_id` 或重新教导。

2. **保持横向触碰记录完整**

   `cross_probe_touch.py` 已在每次触发时记录关节角、FeedInfo 法兰位姿、FeedInfo 新鲜度、Stop 后位姿、分支、接近方向、标准姿态、速度和沿接近方向的 Stop 过冲。关节角用于 FK 复核，FeedInfo 法兰位姿作为当前主数据源。后续真实采集必须带 `session_id / setup_id / workpiece_id / face_id`，不要混用不同安装状态的数据。

3. **建立四个横向方向的标准姿态和注册面**

   在当前已验证 `X-` 的基础上，为 `X+ / Y- / Y+` 分别建立 `branch + approach + rx/ry/rz` 成组配置。每个方向先 dry-run，再做短距离低速真实触碰，最后做固定安全起点 3 次重复测试，统计触发重复性和 Stop 过冲。

4. **先做十字测针工具几何标定**

   标定目标是拟合每个分支测球中心相对法兰的真实 offset，而不是直接拟合盒子尺寸。当前名义模型为：

   ```text
   ball_center_base = flange_position_base + R_flange_base * branch_offset_flange
   surface_point = ball_center_base + approach_unit * ball_radius
   ```

   已有离线标定脚手架 `calibrate_cross_probe_geometry.py`，后续应使用四方向触发数据和已知约束面估计真实 `branch_offset_flange`。

4. **重新确认抬高后的固定起点再做重复触碰**

   2026-05-25 只读复查时，当前法兰位姿为 `[-236.5531,90.8024,301.5564,-177.7902,2.2433,153.3941]`，不等于 2026-05-23 已成功触碰的抬高后测量起点。脚本已支持显式 `--fixed-safe-pose`，但真实执行前必须现场确认从当前位置移动到固定起点、标准姿态旋转扫掠、`X- 5mm` 接近以及回退路径全部无碰撞。

5. **当前 Z 高度已验证路径安全，但尚未得到接触点**

   用户确认当前高度安全后，已在法兰 `Z=301.556mm` 建立新固定起点并执行一次 `X- 5mm` 真实动作。机械臂到达 `X=-252.5230mm` 未触发 DI1，随后正常回退并返回固定起点。因此当前结果只能证明这段路径可安全执行，不能证明该测线落在工件侧面内；继续加深接近距离或做重复采集前，应先确认测球在当前 `Y/Z` 处仍对准待测面。

6. **真实触碰必须绑定已批准的工件面记录**

   系统原先只知道法兰位姿和 DI1，不知道当前测线是否穿过工件面。现在新增 `config/workpiece_setups.yaml`：工件或夹具位置变化后，操作人员需要先将测球放到目标面有效区域前方，记录并批准 `setup_id / face_id / safe_start_pose / 允许搜索行程 / 回退距离 / 速度上限`。2026-05-25 已创建第一个批准面 `box_raised_setup_20260525_xneg_aligned / x_neg_face_aligned_01`，并从该面完成一次成功 `X- 5mm` 触碰。

注册面首次真实触碰结果：

```text
safe start:
[-241.3136, 86.7837, 305.4958, -176.9814, 3.0476, 149.7910]

trigger flange:
[-242.7125, 86.7765, 305.5025, -176.9799, 3.0474, 149.7921]

stop flange:
[-243.5539, 86.7838, 305.4960, -176.9812, 3.0478, 149.7910]

stop_overtravel_along_approach: 0.8414 mm
trigger_feed_age_ms: 0.030
final retracted pose:
[-239.3140, 86.7840, 305.4960, -176.9810, 3.0480, 149.7910]
```

第一次网格进入时，原教导安全起点在定位阶段提前触发，因此该记录已暂停。将起点沿 `X+` 外移 `2mm` 后，新活动面 `x_neg_face_aligned_02_clearance_rebased` 已完成单次验证和二维采集：

```text
samples: 6 (Y offsets 0/-5/+5 mm, Z offsets 0/-5 mm)
surface point model: nominal cross-probe geometry
plane centroid: [-277.7998, 13.1071, 181.8920] mm
plane normal: [-0.999467, -0.028384, -0.016129]
plane RMS residual: 0.0183 mm
plane max residual: 0.0262 mm
Stop overtravel mean: 1.1013 mm
Stop overtravel range: 0.8982 .. 1.6759 mm
```

当前残差可说明该小区域的接触采集稳定且可拟合成面，但真实十字测针 offset 尚未用标准面标定，因此质心和绝对面位置不能作为最终工件尺寸结果。

2026-05-25 代码审查后已补强以下约束：

- 接触记录改为完成回退后再写盘，避免写文件异常使测头停留在接触位置。
- 注册面网格定位前设置批准速度，且每次转移前检查 DI1 已释放。
- 网格样点由注册面的切向基向量生成，并在真实触碰核心入口重新校验面内边界。
- `operator_approved_validation` 只允许单次触碰入口，不允许重复或网格入口；当前活动面再次进行批量采集前，需要完成 `verified` 姿态验收。
- 标定平面约束已修复非单位法向量处理；单面及立方体侧面重构会拒绝共线/重复点，并支持标定后的接触点输入。

## 后续应该做什么

1. 确认 Dobot `rx/ry/rz` 到旋转矩阵的约定，必要时用关节角和 URDF/MoveIt FK 交叉验证。
2. 用标准块或已知平面采集标定数据，拟合连接法兰、RMP60、本体中心轴和十字测针分支的真实几何。
3. 将标定结果写入 `config/cross_probe_geometry.yaml` 或新建标定结果文件，保留名义值和实测值来源。
4. 用标定后的工具模型将触发法兰位姿转换为真实接触点。
5. 对单个竖直面做平面拟合，检查残差、重复性和法向。
6. 扩展到多个面，重构盒子的平面、夹角和尺寸。
7. 最后再考虑把任意方向真实执行从 dry-run 脚手架升级为正式执行入口。

## 2026-05-29 竖直测针恢复验收

固定安全起点：

```text
[-344.7692, 148.4787, 355.4291, -179.3460, -1.4400, 124.6769]
```

单次 Z- 5mm 恢复验证：

```text
trigger flange: [-344.7738, 148.4700, 352.2652, -179.3445, -1.4411, 124.6784]
stop flange:    [-344.7690, 148.4790, 350.8224, -179.3460, -1.4400, 124.6770]
Stop overtravel along Z-: 1.4428 mm
final DI1: {0}
csv: data/vertical_probe_revalidation_20260529.csv
```

固定起点 3 次重复触碰结果：

```text
triggered cycles: 3/3
flange_z mean: 352.3769 mm
flange_z sample std: 0.0266 mm
flange_z range: 352.3613 .. 352.4076 mm
tip_z_est mean: 151.9769 mm
tip_z_est sample std: 0.0266 mm
final pose: [-344.7690, 148.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_probe_repeat_20260529.csv
```

本次暴露并修复了旧竖直脚本的等待问题：`probe_touch.py` 原先在发出回退 `MovL` 后立即检查 DI1，可能在回退途中误判未释放。现在单次、重复、单点和多点竖直流程都会等待 safe/retract pose 实际到位后再继续判断。

竖直 3 点短线验收：

```text
line: X=-344.7692, Y=148.4787..150.4787 mm, safe_z=355.4291 mm
points: 3/3 triggered
trigger flange_z: 352.3596 / 352.3142 / 352.2738 mm
tip_z_est: 151.9596 / 151.9142 / 151.8738 mm
final pose: [-344.7690, 150.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_line_validation_20260529.csv
```

竖直 2x2 小网格验收：

```text
grid: X=-344.7692..-344.2692 mm, Y=150.4787..151.4787 mm, safe_z=355.4291 mm
points: 4/4 triggered
trigger flange_z: 352.4165 / 352.3604 / 352.4234 / 352.3203 mm
flange_z mean: 352.3802 mm
flange_z sample std: 0.0489 mm
final pose: [-344.2690, 151.4790, 355.4290, -179.3460, -1.4400, 124.6770]
final DI1: {0}
csv: data/vertical_grid_validation_20260529.csv
```

竖直单面拟合流程已打通：

```text
script: scripts/reconstruct_vertical_plane.py
input: data/vertical_grid_validation_20260529.csv
output: data/vertical_grid_plane_reconstruction_20260529.json
residual csv: data/vertical_grid_plane_residuals_20260529.csv
least-squares residual span: 0.0235 mm
```

该结果只证明当前 `0.5 x 1.0mm` 小范围数据可完成采集和拟合闭环，不能作为最终平面度结论。正式形位评价至少使用 `3x3`，更推荐 `5x5` 或更多点，并且测区需要覆盖被测特征有效区域。

首组 `3x3` 上表面采集已完成：

```text
grid: X=-345.2692..-343.2692 mm, Y=149.4787..153.4787 mm, safe_z=355.4291 mm
points: 9/9 triggered
csv: data/vertical_surface_3x3_20260529.csv
plane json: data/vertical_surface_3x3_plane_20260529.json
residual csv: data/vertical_surface_3x3_residuals_20260529.csv
normal: [-0.023363, -0.004419, 0.999717]
rms_residual_mm: 0.064714
max_abs_residual_mm: 0.106104
flatness_estimate_ls_mm: 0.185489
final DI1: {0}
```

该 `3x3` 数据已经可以做初步平面度评价。正式判断前建议重复一次同一区域 `3x3` 或扩大到 `5x5`，用重复性区分真实表面形状和测量触发波动。

同一区域 `3x3` 重复采集已完成：

```text
repeat csv: data/vertical_surface_3x3_repeat_20260529.csv
repeat plane json: data/vertical_surface_3x3_repeat_plane_20260529.json
repeat residual csv: data/vertical_surface_3x3_repeat_residuals_20260529.csv
repeat rms_residual_mm: 0.034250
repeat max_abs_residual_mm: 0.066934
repeat flatness_estimate_ls_mm: 0.122751
same-point tip_z delta mean: +0.011089 mm
same-point tip_z delta sample std: 0.089617 mm
same-point tip_z delta range: -0.143100 .. +0.145300 mm
normal angle delta: 1.231975 deg
```

这说明测量链路可重复采集同一区域，但同点高度仍有约 `0.09mm` 样本标准差的波动。当前结果适合做初步趋势判断；用于正式形位公差结论前，需要继续量化并降低重复性误差。

更换硬工件后完成同一区域 `3x3`：

```text
center csv: data/vertical_surface_hard_center_20260529.csv
hard csv: data/vertical_surface_hard_3x3_20260529.csv
hard plane json: data/vertical_surface_hard_3x3_plane_20260529.json
hard residual csv: data/vertical_surface_hard_3x3_residuals_20260529.csv
points: 9/9 triggered
normal: [-0.023393, 0.018567, 0.999554]
rms_residual_mm: 0.025231
max_abs_residual_mm: 0.048975
flatness_estimate_ls_mm: 0.089231
hard minus soft-repeat tip_z mean: +0.338922 mm
final DI1: {0}
```

硬工件的残差和平面度估计明显小于软工件两组，说明软表面压陷或触发不稳定很可能是此前重复性波动的重要来源。

硬工件同一区域 `5x5` 加密采样已完成：

```text
hard 5x5 csv: data/vertical_surface_hard_5x5_20260529.csv
hard 5x5 plane json: data/vertical_surface_hard_5x5_plane_20260529.json
hard 5x5 residual csv: data/vertical_surface_hard_5x5_residuals_20260529.csv
points: 25/25 triggered
normal: [-0.008647, 0.009831, 0.999914]
rms_residual_mm: 0.043600
max_abs_residual_mm: 0.108018
flatness_estimate_ls_mm: 0.191242
top residuals: grid_r1c2=-0.108018, grid_r3c5=-0.092281, grid_r2c4=+0.083224
final DI1: {0}
```

5x5 结果比硬工件 3x3 的 `0.089231mm` 更大，主要因为加密采样暴露了局部高低点。当前结果更适合作为待复测的误差地图；正式结论前应复测高残差点或重复整组 5x5。

用户当前选择跳过 5x5 高残差点复测，直接规划下一测量面。由于下一步使用十字形测针，不能沿用竖直测针上表面流程；下一面优先规划为 `y_neg` 分支沿基坐标 `X-` 触碰的竖直侧面：

```text
setup_id: hard_workpiece_cross_setup_20260529
workpiece_id: hard_workpiece
face_id: hard_x_neg_face_01
standard_pose: x_neg_y_neg_verified
branch: y_neg
approach: [-1, 0, 0]
single validation: X- 5mm, retract 2mm, speed 1
initial grid after validation: Y=-5/0/+5mm, Z=-5/0/+5mm
```

此前 `box_raised_setup_20260525_xneg_aligned` 属于旧工件/旧安装状态，不可直接作为新硬工件侧面记录。新侧面必须重新人工对准、教导安全起点并批准后才能真实触碰。

新硬工件 X- 侧面单点验证已完成：

```text
standard_pose: hard_x_neg_y_neg_operator_approved_20260529
setup_id: hard_workpiece_cross_setup_20260529
face_id: hard_x_neg_face_01
safe_start_pose: [-227.4934, 118.6944, 346.3719, -179.1428, 1.1229, 90.7873]
trigger flange: [-230.2924, 118.6911, 346.3675, -179.1437, 1.1230, 90.7877]
stop flange: [-231.8833, 118.6941, 346.3720, -179.1430, 1.1230, 90.7870]
stop_overtravel_along_approach_mm: 1.5909
final pose: [-225.4930, 118.6940, 346.3720, -179.1430, 1.1230, 90.7870]
final DI1: {0}
csv: data/hard_x_neg_face_single_20260529.csv
```

单点验证后，先回到批准安全起点，再执行固定起点 3 次重复；重复通过后才考虑侧面网格。

新硬工件 X- 侧面固定起点 3 次重复已完成：

```text
repeat csv: data/hard_x_neg_face_repeat_20260529.csv
cycles: 3/3 triggered
trigger_flange_x: -230.1840 / -230.1837 / -230.2564 mm
trigger_flange_x mean: -230.2080 mm
trigger_flange_x sample std: 0.0419 mm
stop_overtravel mean: 1.7123 mm
stop_overtravel sample std: 0.0419 mm
stop_overtravel range: 1.6639 .. 1.7366 mm
final pose: [-227.4930, 118.6940, 346.3720, -179.1430, 1.1230, 90.7870]
final DI1: {0}
```

`hard_x_neg_y_neg_operator_approved_20260529` 已升级为 `verified`，允许后续重复和批准面采集入口。当前 `hard_x_neg_face_01` 尚未写入侧面 `Y/Z` 采集 bounds，因此网格采集仍需先由现场确认有效移动范围。

新硬工件 X- 侧面 `3x3` 网格 dry-run 已完成：

```text
grid: Y=-5/0/+5 mm, Z=-5/0/+5 mm
entry clearance pose: [-225.4934, 118.6944, 346.3719, -179.1428, 1.1229, 90.7873]
safe X: -227.4934 mm
target X: -232.4934 mm
clearance X: -225.4934 mm
output if executed: data/hard_x_neg_face_grid_3x3_20260529.csv
```

真实执行前必须确认当前机器人在入口净空位姿附近，并确认 `Y=113.6944/118.6944/123.6944mm`、`Z=341.3719/346.3719/351.3719mm` 的 9 个点都落在有效侧面内。

新硬工件 X- 侧面 `3x3` 真实网格采集和名义几何重构已完成：

```text
grid csv: data/hard_x_neg_face_grid_3x3_20260529.csv
plane json: data/hard_x_neg_face_plane_20260529.json
points: 9/9 triggered
normal: [-0.996308, 0.063115, -0.058191]
rms_residual_mm: 0.1611
max_abs_residual_mm: 0.2376
stop_overtravel_mean_mm: 1.3444
stop_overtravel_range_mm: 1.0451 .. 1.6616
final pose: [-225.4930, 123.6940, 351.3720, -179.1430, 1.1230, 90.7870]
final DI1: {0}
```

该侧面重构使用名义十字测针几何。它证明横向侧面采集和重构链路可用，但在十字测针各分支 offset 完成实测标定前，不把该平面绝对位置、尺寸或形位公差作为最终结论。

## 当前软件模块

核心真实触碰：

- `scripts/cross_probe_touch.py`：单次十字测针横向触碰，默认使用标准姿态，带 DI1 预检、FeedInfo 新鲜度检查、分段探测、Stop 和回退；触发记录已包含关节角、FeedInfo 元数据、标准姿态、速度、`probe_step_mm` 和 Stop 过冲等后续标定字段。
- `scripts/repeat_cross_probe_test.py`：固定安全起点重复横向触碰，统计 Stop 过冲；支持用 `--fixed-safe-pose` 显式固化已确认起点，默认继承 `--probe-step-mm 0.5` 分段探测。
- `scripts/measure_registered_face_grid.py`：仅在批准面有效范围内执行二维触碰采样，面内转移使用注册的回退净空路径。
- `scripts/reconstruct_registered_face.py`：按 `setup_id / face_id` 拟合单个注册面，并输出残差和 Stop 过冲统计。
- `scripts/measure_cube_side_line.py`：盒子侧面多点横向触碰脚本；注册面路径尚未接入前，真实执行已禁用。
- `scripts/reconstruct_vertical_plane.py`：从竖直测针 CSV 拟合单个平面，输出最小二乘残差和平面度估计。
- `scripts/reconstruct_cube_from_contacts.py`：从竖直/横向触点 CSV 拟合平面，属于后处理工具。

几何和姿态：

- `config/cross_probe_geometry.yaml`：十字测针名义几何。
- `config/measurement_poses.yaml`：真实测量标准姿态；`status: verified` 允许批准路径的批量执行，`operator_approved_validation` 仅允许单次受限验证入口，`status: planned` 只允许 dry-run。
- `config/workpiece_setups.yaml`：工件安装状态和人工教导面；只有 `status: approved` 的面才可用于真实触碰。
- `scripts/teach_cross_probe_face.py`：读取人工放置后的安全起点并创建或批准面记录。
- `scripts/workpiece_registration.py`：在真实动作前校验 setup、面、分支、姿态、运动限制和实际起点。
- `scripts/cross_probe_model.py`：法兰位姿到十字测球中心和估计表面点的换算。
- `scripts/calibrate_cross_probe_geometry.py`：离线十字测针分支 offset 标定脚手架，支持已知接触点/平面约束 CSV，也支持生成合成数据做闭环测试。
- `scripts/calibrate_cross_probe_from_sphere.py`：使用 20mm 标定球四方向触碰数据做十字测针相对分支 offset 标定；当前输出用于四方向一致性修正，不直接覆盖名义几何。
- `scripts/plan_cross_probe_sphere_pose.py`：多姿态标准球触碰的离线规划器；用已标定分支球心偏置和标准球心反推法兰安全位/触发位/搜索终点，并检查测球球心路径、分支轴线夹角和 planned YAML 片段。
- `scripts/convert_cross_probe_contacts.py`：使用标定 JSON 将横向触碰 CSV 转换为标定后的测球中心和真实接触点 CSV。
- `scripts/validate_cr5_pose_convention.py`：只读验证 CR5 `rx/ry/rz` 欧拉角约定。

仿真和预检查：

- `urdf/rmp60_probe_module.xacro`：可替换工具模组。
- `launch/cr5_rmp60_moveit_demo.launch.py`：CR5 + RMP60 MoveIt demo。
- `scripts/preflight_measurement_plan.py`：任意方向真实执行前的 MoveIt/TF 预检查。
- `scripts/execute_measurement_plan.py`：任意方向真实执行 dry-run 脚手架，当前真实执行仍禁用。

旧竖直流程：

- `scripts/probe_touch.py`：竖直触碰核心，默认 `--probe-step-mm 0.5` 分段下探。
- `scripts/repeat_probe_test.py`
- `scripts/measure_point.py`
- `scripts/measure_points_csv.py`
- `scripts/measure_line.py`
- `scripts/measure_grid.py`

这些脚本仍有参考价值。竖直测针已通过单点、3 次重复、短线和 2x2 小网格恢复验证；正式竖直批量采集前仍不要跳过 `--ack-vertical-stylus-ok` 和现场路径确认。

## 下一步顺序

1. 竖直测针基础链路已恢复：`GetPose`、DI1、FeedInfo 正常，Z- 5mm 单次触碰、固定起点 3 次重复触碰、3 点短线和 2x2 小网格均通过。
2. 硬工件同一区域 `5x5` 已完成；用户当前选择先不复测高残差点，直接进入十字测针侧面规划。
3. 新硬工件 X- 侧面已经完成单点 `X- 5mm` 验证和固定起点 3 次重复，触发 X 样本标准差 `0.0419mm`。
4. 20mm 标定球 `X- / X+ / Y+ / Y-` 四个水平方向均已完成单点和固定起点 3 次重复触碰。
5. 已生成第一版四方向相对标定 `data/cross_probe_sphere_calibration_20260601.json`；结果提示旧名义支臂长度/分支原点定义很可能不符合实物，正式覆盖几何前还需要做多姿态或更低过冲的复测。
6. 已完成代码侧低过冲优化和固定姿态标准球相对标定闭环：标定球四方向低过冲复测最大过冲约 `0.4735mm`，新相对标定文件为 `data/cross_probe_sphere_calibration_low_overtravel_20260601.json`，整体 RMS `0.013357mm`。
7. 多姿态标准球标定不能直接复用固定姿态的 base X/Y 搜索路径；当前使用 `scripts/semi_auto_normal_probe.py` 根据临时 `C_s / p` 和当前法兰位姿自动计算两球心法向，先 dry-run 复核方向，再做 `0.2mm` 起步的短距离真实采集。
8. 侧面 `Y/Z=-5/0/+5mm` 小范围已完成真实 `3x3` 采集和名义几何平面重构；相对标定 JSON 已可用于转换旧侧面数据，但是否要复测应先看低过冲验证结果。

## 文档维护要求

每次真实运动后至少更新：

- `PROJECT_MEMORY.md`：追加事实记录。
- `COMMANDS.md`：如果命令或参数变化，更新可执行命令。
- `ROADMAP.md`：更新完成项和下一步。
- 本文件：如果硬件状态、安全结论、标准姿态或下一步顺序发生变化，必须同步更新。
