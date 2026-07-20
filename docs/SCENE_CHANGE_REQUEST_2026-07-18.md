# 五机械臂场景集中修改建议

日期：2026-07-18
当前验证场景：`scenes/five_cr5a_cell.ttt`
当前场景指纹：

```text
size   = 2875907
sha256 = 0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657
```

以下建议来自当前场景中的真实 IK、工件对齐和碰撞检查。场景成员可以根据
整体布局做小幅调整，但请在交付新场景时提供最终坐标和修改说明。控制组不会
在收到新场景前把这些建议值写入正式 target。

## 1. R2 PCB 供料区与基座重叠

当前问题：

```text
/FiveCR5A_Cell/Areas/PCB_Supply_Area
/FiveCR5A_Cell/RobotBases/R2_Base
```

两者包围盒重叠。当前路径组按用户要求没有移动任何一方，也没有通过碰撞
排除掩盖该问题。

修改建议：

- 优先保持 R2 机械臂真实安装基座不动，调整 PCB 私有供料区布局；
- 如果移动供料区，必须同步移动 `PCB_Supply`、供料板和
  `R2_PCB_PICK_APP/TCP`，不能只移动装饰底板；
- 保持 R2 私有区与 R1 私有供料区不重叠，并保留到装配共享区的安全通道；
- 修改后检查 R2 零位、PICK APP/TCP、PCB 和供料板均无环境碰撞；
- 请提供最终 R2 基座、供料区、PCB 和两个 PICK target 的世界坐标。

## 2. R3 控制模块与 PCB 元件重叠

当前模板模块中心约为：

```text
(-1.120, 0.220, 0.3015)
```

实际 R2 PCB 放置后，该位置使控制模块先与 PCB 板重叠，继续稍微抬高后仍会
与 PCB 主芯片重叠。单纯把模块提高约 18 mm 虽然能消除碰撞，但模块会高出
箱壁，不适合作为最终场景方案。

当前模型中较稳妥的平面候选约为：

```text
建议模块中心：(-1.105, 0.185, 0.3035)
相对原位置：  X +15 mm，Y -35 mm，Z 约 +2 mm
```

该候选在当前实际箱体和 PCB 几何下不碰 PCB 芯片、箱壁或箱内立柱。最终
坐标仍请场景成员根据视觉布局确认。

修改时必须同步：

- `Assembly_ControlBox_Product` 中的控制模块；
- `Inspection_ControlBox_Product` 中的控制模块；
- `R3_MODULE_PLACE_APP` 的 XY；
- `R3_MODULE_PLACE_TCP` 的 XY；
- 完整产品显示阶段使用的控制模块位置。

APP 应继续位于 TCP 正上方。TCP 的 Z 必须结合最终 R3 工具长度重新标定，
不要直接把模块中心 Z 当成 TCP Z。

## 3. R5 合格品分拣不可安全搬运

当前 R5 没有夹爪或吸盘模型，`R5_gripper_tip` 位于 Link6 原点。使用临时
顶部吸附时，缺陷品分支存在无碰撞候选，但合格品分支的两个可达姿态都会在
携物抬升第 53 个状态发生：

```text
Inspection_ControlBox_Product_Shell_Front_Wall
    与
R5/Link2_respondable
```

碰撞。已经扫描合格带入口附近
`x=[0.45,0.75]`、`y=[-1.30,-1.05]`，没有找到能同时满足安全抓取姿态、
合格 APP/TCP 可达和端点无碰撞的简单平移方案。

修改建议：

- 为 R5 增加真实顶部吸盘或夹爪，并标定真实 TCP；
- 联合检查 R5 基座朝向、合格传送带入口和检测区抓取点，不要只改一个 APP；
- 优先让同一个安全抓取姿态覆盖检测区 PICK 和合格品 PLACE；
- 保持完整产品世界姿态稳定，避免为了可达让产品在搬运中大幅翻转；
- 修改后必须携带完整 30 形状产品检查产品到 R5、产品到相机/平台/传送带、
  R5 自碰撞和机器人间碰撞。

当前缺陷品静态候选仅作为参考，不是场景最终值：

```text
runtime TCP        = 100 mm
orientation        = (195,-45,0) deg
transfer waypoint  = (-0.15,-0.15,0.65)
```

## 4. R5 传送带放置高度相差 26 mm

当前值：

```text
R5_PRODUCT_PICK_TCP z = 0.340
产品抓取前根节点 z    = 0.216
R5_*_PLACE_TCP z       = 0.420
刚性保持抓取后的根节点 = 0.296
Generator 带面产品根节点 = 0.270
```

因此当前放置 target 会让产品悬在带面上方约 26 mm。若保持现有抓取相对
高度，两个 PLACE TCP 的参考 Z 应约为：

```text
0.270 + (0.340 - 0.216) = 0.394 m
```

若继续保持 APP 到 TCP 的 200 mm 竖直下降，对应 APP Z 可参考约
`0.594 m`。这些是几何参考值，最终仍需结合真实 R5 工具、带面接触和完整
产品碰撞重新验证。

必须同时检查：

- `R5_GOOD_PLACE_APP/TCP`；
- `R5_DEFECT_PLACE_APP/TCP`；
- `goodStart`、`defectStart` 和 `PRODUCT_ON_BELT_Z`；
- 产品放置后是否与黑色带面接触但不穿入框架；
- 传送带启动后产品是否沿正确方向运动。

## 5. 相机视野可视体不应参与物理碰撞

`Camera_View_Area` 按设计覆盖检测区完整产品，是相机检测范围的可视辅助体，
不是实体障碍物。当前它会与检测产品直接重叠并触发碰撞。

建议将该对象设为不可碰撞的可视辅助形状，同时保留：

- 固定相机机身；
- 相机支架；
- 检测平台；
- 检测区实体结构。

控制代码当前只排除 `Camera_View_Area`，没有排除任何实体相机或支架。

## 6. 新场景交付要求

请不要保存机械臂运行终点、临时 Runtime TCP、工件 attach 状态或调试
visibility 到 `.ttt`。交付前建议确认：

- 仿真停止；
- R1-R5 六关节均为零；
- 所有实际供料工件由 `/FiveCR5A_Cell/Parts` 持有；
- 无 `R*_Runtime_*`、Runtime Command Bridge 或临时 target；
- R1-R5、两个脚本 Dummy、35 个机器人 target 和相机 target 均存在；
- Generator/ROS2 Bridge 没有重复实例；
- 提供新文件大小、SHA-256 和所有修改对象的前后坐标。

新场景到达后，控制组会重新执行场景审计、target 差异、IK、路径、碰撞、
工作区和正式任务回归，不会直接复用旧场景路径缓存。

打开待验收场景并保持停止态后，可在主仓库运行：

```bash
python3 sim_bridge/audit_five_cr5a_scene.py \
  --scene /absolute/path/to/new_scene.ttt \
  --output /tmp/five_cr5a_new_scene_audit.json \
  --summary-only
```

工具只查询当前打开的场景，不会加载、启动、停止或保存场景。退出码 `0`
表示结构和已知几何问题满足“可以开始重新做路径验证”，退出码 `1` 表示仍有
阻塞项。即使退出码为 `0`，只要指纹或 target 有变化，仍必须重新做完整路径
和碰撞回归。
