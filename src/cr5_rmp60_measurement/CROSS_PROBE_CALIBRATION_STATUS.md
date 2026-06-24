# 十字测针标定状态与下一步流程

更新时间：2026-06-06

本文档整理当前关于 CR5 + RMP60 + 十字测针标定的讨论结论、数据状态和后续流程。当前目标不是马上评价工件形位公差，而是先建立可靠的工具几何标定链路。

## 1. 当前核心目标

最终要解决的问题：

```text
DI1 触发瞬间法兰位姿
-> 某一颗红宝石球在基坐标系下的真实球心
-> 真实接触点
-> 标准球/环规/工件平面/立方体重构
-> 形位公差评价
```

对十字测针来说，每一颗横向红宝石球都应看成一个独立分支 offset。不能把一个分支的 TCP 或补偿量直接用于另一个分支。

当前先做 `y_neg` 分支，也就是同一颗物理红宝石球的多姿态标准球标定。

## 2. 标定模型

对每一次标准球触碰，触发瞬间满足：

```text
C_s = F + R * p + a * (R_s + R_p)
```

含义：

- `C_s`：20mm 标准球球心，未知量。
- `F`：触发瞬间法兰位置，由机械臂 FeedInfo 记录。
- `R`：触发瞬间法兰姿态对应的旋转矩阵。
- `p`：当前分支红宝石球心在法兰坐标系下的 offset，未知量。
- `a`：接近方向单位向量。
- `R_s`：标准球半径，当前为 `10mm`。
- `R_p`：红宝石球半径，来自 `config/cross_probe_geometry.yaml` 或命令行参数。

未知量一共 6 个：

```text
C_s = [x, y, z]
p   = [x, y, z]
```

每次触碰提供 3 个方程。理论上至少需要 2 个明显不同姿态，实际需要更多点，因为存在触发误差、Stop 过冲、机械臂姿态误差和接近方向误差。

当前使用脚本：

```bash
./scripts/calibrate_branch_sphere_absolute.py
```

## 3. 球心如何得到

标准球球心和红宝石球心不是一开始就准确知道的。

第一阶段：

1. 操作者人工把目标红宝石球移动到标准球附近。
2. 用 GUI 或点动脚本低速触碰。
3. 记录 DI1 触发瞬间法兰位姿、姿态、接近方向和分支标签。
4. 用最小二乘同时反算标准球球心 `C_s` 和分支 offset `p`。

第二阶段：

1. 用初步拟合出的 `C_s` 和 `p`，实时计算当前姿态下红宝石球心：

   ```text
   C_p = F_current + R_current * p
   ```

2. 根据两球心连线自动生成下一次接近方向：

   ```text
   approach = normalize(C_s - C_p)
   ```

3. 程序沿该方向做短距离、低速触碰。
4. 触发后记录并重新拟合。

这就是后续“半自动法向采集”的基础。

## 4. 姿态和接触原则

当前最重要原则：

```text
当前标定哪一颗红宝石球，就必须确保只有这颗球先接触标准球。
```

对十字测针横向分支，不能简单照搬普通竖直测针的“顶部极点、赤道点、上半球斜向点”流程。普通竖直测针可以较安全地做 `z-` 顶部触碰；但横向十字测针做纯 `z-` 时，很容易让竖直测针、测杆、十字中心或非目标部位先碰标准球。

当前实验已经验证：

- `y_neg` 分支 sample 6 的 `z-` 点残差约 `9.01mm`。
- 该点不应视为有效 `y_neg` 红宝石球触碰。
- 后续 `y_neg` 分支标定不推荐采纯 `z-`。

推荐姿态策略：

- 固定同一颗物理红宝石球。
- RZ 有明显变化。
- RX/RY 可少量变化，但不要进入危险姿态。
- 接近方向以横向或复合横向为主。
- 如需增加 Z 约束，使用带小 Z 分量的复合方向，并由操作者现场确认只有目标球会先接触。

## 5. 当前数据状态

### 固定姿态四方向相对标定

推荐参考：

```text
data/cross_probe_sphere_calibration_low_overtravel_20260601.json
```

性质：

- 固定姿态四方向相对一致性标定。
- RMS `0.013357mm`。
- 不能直接作为完整绝对 TCP / 分支 offset。
- 不能直接覆盖 `config/cross_probe_geometry.yaml`。

### 2026-06-03 GUI 多姿态数据

原始采集：

```text
data/gui_ball1_sphere_contacts_20260603.csv
```

注意：

- 前 5 条为 `x_neg`。
- 后续为 `y_neg`。
- 不能把 `x_neg` 和 `y_neg` 混合作为同一分支求解。

### `260603.1` 当前 y_neg 数据

当前批次相关文件：

```text
data/260603.1/1.csv
data/260603.1/1_latest_with_header.csv
data/260603.1/yneg_current_7pts_for_fit.csv
data/260603.1/yneg_current_6pts_no_z_for_fit.csv
data/260603.1/yneg_new_5pts_for_fit.csv
```

当前最有参考意义的是旧 5 点阶段性结果：

```text
data/260603.1/yneg_new_5pts_absolute_fit_20260603.json
```

结果：

```text
rows: 5
rank: 6
condition: 530.481951
sphere_center_mm: [-402.9934, 126.9687, 86.9531]
local_ball_offset_mm: [18.7113, -31.9482, 178.1800]
RMS: 1.434982mm
max residual: 2.234877mm
```

这不是最终标定，只是阶段性参考。主要问题是接近方向全部在 XY 平面，`approach_z=0`，导致球心 Z 和 offset Z 约束不足。

追加 sample 6-7 后的 7 点结果：

```text
data/260603.1/yneg_current_7pts_absolute_fit_20260603.json
```

结果：

```text
rows: 7
rank: 6
condition: 221.178500
RMS: 4.458202mm
max residual: 9.011658mm
```

结论：

- 7 点结果不能作为标定参数。
- sample 6 `z-` 为明显异常点。
- 删除 `z-` 后 6 点 RMS 仍为 `2.334887mm`，说明 sample 7 也不够一致。

## 6. 当前结论

1. 当前数据还不够做最终标定。
2. 当前数据足够证明多姿态联合求解流程可以跑通。
3. `z-` 纯下压不适合作为 `y_neg` 横向球尖的常规采点方式。
4. 继续人工猜 `x- / y- / vec` 的效率较低，容易采到不满足两球法向接触的数据。
5. 下一步应从人工点动升级到半自动法向采集。

## 7. 推荐下一步：距离粗拟合到半自动法向采集

目标：

```text
人工粗采先得到 C_s / p 初值；
操作者再把目标球尖摆到安全近球位置；
程序根据当前 C_s / p 自动计算两球心连线方向并短距离触碰。
```

启动粗拟合模型不依赖人工点动方向：

```text
||F_i + R_i*p - C_s|| = R_s + R_p
```

当前独立入口：

```bash
./scripts/calibrate_branch_sphere_distance_only.py \
  --input data/gui_ball1_sphere_contacts_20260603.csv \
  --branch y_neg \
  --physical-ball-id 1 \
  --json-output data/yneg_distance_only_coarse_fit.json \
  --residual-output data/yneg_distance_only_coarse_fit_residuals.csv
```

该结果只作为半自动法向采集初值，不作为最终工具几何。重新采集时建议同一颗红宝石球采 10 到 15 个独立触发点；若 `degrees_of_freedom <= 0`，残差可能被插值为 0，不能说明拟合可靠。

流程：

1. 操作者选择分支，例如 `y_neg`。
2. 使用 GUI/点动脚本人工粗采同一颗红宝石球触碰同一个 20mm 标准球的数据。
3. 运行 `scripts/calibrate_branch_sphere_distance_only.py` 得到临时 `C_s / p`。
4. 操作者把 `y_neg` 红宝石球移动到标准球附近。
5. 操作者确认：
   - 目标仍是同一颗物理红宝石球；
   - 路径附近不会让测杆、中心结构、竖直针或其他分支先碰到标准球；
   - 当前点允许短距离低速前进和回退。
6. 程序读取当前法兰位姿。
7. 程序用阶段性 `C_s` 和 `p` 计算当前红宝石球心。
8. 程序计算接近方向：

   ```text
   approach = normalize(C_s - C_p)
   ```

9. 程序执行短距离、低速 MovL 搜索，例如 `0.2..1.0mm`。
10. DI1 触发后立即 Stop。
11. 程序反向回退。
12. 程序写 CSV，重新拟合并输出该点残差。
13. 残差过大时标记为异常，不自动并入最终标定。

当前独立入口：

```bash
./scripts/semi_auto_normal_probe.py \
  --fit-json data/yneg_distance_only_coarse_fit.json \
  --branch y_neg \
  --search-mm 0.2 \
  --retract-mm 1 \
  --speed 1 \
  --output data/semi_auto_yneg_normal_contacts.csv
```

默认 dry-run，只打印当前 `C_p`、计算出的 `approach` 和目标法兰位姿。真实短距离运动必须显式加 `--execute --ack-safe-normal-probe`，且 `--search-mm` 被限制在 `<= 1.0mm`。

安全边界：

- 不做无人值守全自动。
- 不自动大范围移动到新姿态。
- 不自动执行纯 `z-`。
- 不在操作者未确认净空时执行真实运动。

## 8. 后续科研输出方向

专利、EI 和 SCI 论文可以围绕以下贡献组织：

- 机器人搭载十字测针的分支球心多姿态联合标定方法。
- 标准球约束下的法兰位姿、红宝石球 offset、标准件球心联合求解。
- 基于初始粗标定的两球心法向半自动采集策略。
- DI1 触发延迟与 Stop 过冲数据的建模和剔点/补偿策略。
- 用环规、平面、立方体进行补偿前后几何重构验证。

## 9. 近期执行清单

1. 先继续 `y_neg` 分支。
2. 不使用当前 `z-` 异常点。
3. 重新粗采 10 到 15 个同一颗物理球的触发点。
4. 使用 `scripts/calibrate_branch_sphere_distance_only.py` 得到半自动法向初值。
5. 使用 `scripts/semi_auto_normal_probe.py` 先做 dry-run 复核法向，再由操作者确认是否允许真实短距离采集。
6. 每次只执行短距离低速探测。
7. 每采一个点立即重新拟合并检查残差。
8. 当单分支 RMS 稳定进入亚毫米级后，再用环规做交叉验证。
