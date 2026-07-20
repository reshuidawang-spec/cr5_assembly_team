# CR5A 会话记忆与恢复记录

日期：2026-07-15
主仓库：`/home/vboxuser/桌面/cr5_assembly_team`
实验工作区：`/home/vboxuser/桌面/workspace`
团队基线：`reshuidawang-spec/cr5_assembly_team@74ff605`
个人 fork：`qixunqiwo/cr5_assembly_team@74ff605`

## 1. 用户的总体任务

用户负责 CR5A 机械臂运动控制、路径规划和工具动作。当前必须先把
R1 的箱体抓放动作在新五臂 CoppeliaSim 场景中表示清楚，再用同一运动原语
扩展到端子排和 R2-R5。

用户认可的唯一正常动作链为：

```text
初始位
-> PICK_APP
-> 保持工具姿态竖直下降到 PICK_TCP
-> 抓取
-> 竖直抬升回 PICK_APP
-> 用短路径直接转移到 PLACE_APP
-> 竖直下降到 PLACE_TCP
-> 释放
```

用户不接受：

- 到 A 点后腕部或机械臂顶部额外转一圈；
- 随机 IK 分支导致的姿态跳变；
- A 到 B 的大幅绕行；
- 在 A 点上方重复接近；
- 慢、卡顿、不自然的分段动作；
- 用“规划成功”替代实际可见动作验证。

## 2. 会话中的用户决策

本次会话中用户按时间顺序明确过以下要求：

1. 路径要尽可能短、无碰撞，运动观感要正常。
2. 到 A 点上方后应该直接下降抓取，不应再转圈。
3. 逐步验证过“到 A APP、下降抓取、抬升到 B APP、下降放置”，
   要将这些正常步骤合并，而不是换成随机规划。
4. workspace 只删除肯定无用的文件；不确定的保留。
5. 团队上游更新必须同步到个人 fork，但本地实验工作未在实际场景成功前
   不上传 GitHub。
6. 五臂场景按 `Five_CR5A_Cell_Control_Interface.md` 第 4.1 节加载，
   必须导入 R1-R5、运行 `Main_Cell_Generator`，并挂载 ROS2 bridge Dummy。
7. R1 目标点来自 Git Generator。若发现不可达、碰撞或对不齐，必须先向用户
   报告证据、原值、拟调整值和影响，得到明确确认后才能改 target。
8. 机械臂必须考虑夹爪碰撞；Git 无夹爪模型时可以选一个合适模型。
9. 同一运动逻辑应能扩展到其他机械臂，但必须替换各臂的基座、关节限位、
   工具/TCP、工件、target、允许接触和障碍物；多臂还要加时序与共享区锁。
10. 最终用户明确决定：当前主要是把运动表示出来，暂时不做箱壁夹持
    target 调整；可以允许夹爪“夹在空气中”并用 attach 让箱体视觉跟随。
11. 第 10 点只放宽了展示模式的物理夹持要求；不允许由此忽略机械臂和夹爪
    对工作台、夹具和其他机械臂的碰撞。

## 3. Git 与本地状态

- `HEAD`、`origin/main`、`upstream/main` 均为
  `74ff605331211b993f2f9a08aad85d0f6d591f44`。
- 个人 fork 已同步团队上游。
- 本地 MoveIt、RViz、夹爪、五臂 `.ttt` 和 workspace 实验工作没有提交、
  没有推送。
- 不得 `reset --hard`、`clean`、覆盖或删除现有本地工作。
- `git diff --check` 仍有旧提示：
  `cr5_moveit/config/kinematics.yaml` 文件末尾多一个空行。

## 4. 五臂场景已完成的基线

场景：

```text
/home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
```

已验证：

- `/R1` 到 `/R5` 均存在；
- 每台 CR5A 都有六个臂关节；
- `/FiveCR5A_Cell` 有 208 个生成对象；
- Targets 子树有 42 个 Dummy；
- `Main_Cell_Generator` 和 `ROS2_All_Robot_Bridge` 各一个；
- ZMQ `localhost:23000` 和 `/compact_cell/*` ROS2 topic 可用；
- `RESET_CELL` 返回 `DONE:RESET_CELL`；
- CoppeliaSim 4.9.0 rev.6，ROS2 Humble。

复现与审计工具：

```text
/home/vboxuser/桌面/workspace/robot_control/prepare_five_cr5a_scene.py
/home/vboxuser/桌面/workspace/robot_control/audit_five_cr5a_scene.py
/home/vboxuser/桌面/workspace/five_cr5a_scene_snapshot.json
```

## 5. R1 关节控制结论

- 六个 CR5A 关节是 kinematic mode。
- 场景原始 `jointfloatparam_maxvel=0`，仿真运行时
  `setJointTargetPosition` 不会产生动作。
- 选定的平滑控制方法是：

  ```text
  非零 maxVel + sim.setJointTargetPosition
  ```

- 已完成六关节 `0 -> +2° -> 0 -> -2° -> 0` 微点动，共 12 个端点。
- 微点动无碰撞，最小间隙 `43.959 mm`，最终关节恢复误差为 0。
- 六关节零位可作为当前安全初始位。
- Git `R1_HOME_REF` 不应在未经用户确认时修改。

证据：

```text
/home/vboxuser/桌面/workspace/r1_calibration.json
```

## 6. Robotiq 85 夹爪与 TCP

团队 Git 没有 CoppeliaSim 夹爪模型。已使用 CoppeliaSim 自带：

```text
/home/vboxuser/CoppeliaSim/models/components/grippers/ROBOTIQ 85.ttm
```

并保存到场景：

```text
/R1/R1_ROBOTIQ85
/R1/R1_ROBOTIQ85/R1_gripper_tip
```

标定信息：

```text
R1_gripper_tip relative to Link6_visual:
position = (0.0, -0.01468, 0.146) m
quaternion = (0, 0, 0, 1)
```

- Robotiq 子树含 24 个形状和 24 个工具关节。
- 仿真启动/停止 30 步后工具相对 Link6 漂移为 0。
- 机械臂 IK 必须只取别名 `joint1..joint6`，不得把 24 个夹爪关节混入。
- 夹爪安装脚本：
  `/home/vboxuser/桌面/workspace/robot_control/install_cr5a_gripper.py`。
- 共享关节发现：
  `/home/vboxuser/桌面/workspace/robot_control/five_cr5a_scene.py`。

物理限制：Robotiq 85 在原中心 PICK_TCP 与箱体最近距离为 `36.695 mm`，
没有指尖接触。当前用户接受展示模式的非物理 attach，因此这个问题
暂不阻塞视觉动作，但不得写成“真实夹持已通过”。

## 7. 受保护的 R1 箱体 target

Git 原值仍为：

```text
R1_BOX_PICK_APP   (-1.80, 0.35, 0.55), ori=(0,0,0)
R1_BOX_PICK_TCP   (-1.80, 0.35, 0.30), ori=(0,0,0)
R1_BOX_PLACE_APP  (-1.15, 0.20, 0.55), ori=(0,0,0)
R1_BOX_PLACE_TCP  (-1.15, 0.20, 0.30), ori=(0,0,0)
```

所有 Git target 的位置和姿态都没有被修改。

已知问题：

- Git `ori=(0,0,0)` 会使 Link3/Link5 在 APP/TCP 姿态碰箱体、工作台或夹具。
- 只在临时 IK/执行层使用固定竖直姿态：

  ```text
  (roll,pitch,yaw) = (180°, 0°, -90°)
  ```

- 这个临时姿态的四端点误差为 `0.143-0.631 mm`。
- 空工具的初始到 PICK_APP、两次竖直下降和 APP 到 APP 直线转移均通过。
- 以原抓取相对位姿刚性附着箱体时，放置下降在 `77.5%`、
  TCP `z ~= 0.35625 m` 时，`Box_Blank_Bottom` 碰到 `Assembly_Fixture`。
- 原因是供料箱底 `z=0.156 m`，装配箱底应为 `z=0.216 m`，相差 60 mm。

用户已决定本阶段不调 target，用 60 mm 视觉跟随偏移表示箱体搬运。
将来若恢复物理夹持与负载碰撞验收，仍要重新向用户报告并获得 target
调整批准。

## 8. 隐藏对象碰撞陷阱

`main_cell_generator.lua` 的 `setProductStage(...,0)` 只把模板形状的 visibility layer
设为 0，不会自动关闭碰撞。

因此：

- 当前工序的碰撞 collection 必须排除 visibility-layer-0 的未来产品模板；
- 不能把隐藏的 PCB、控制模块或装配箱壁当成当前实体障碍物；
- 也不能因此排除真实可见工作台、供料区、夹具或其他机械臂。

另一个性能陷阱：不要对整个场景在每个轨迹点调用无界
`sim.checkDistance(robot, environment, 0)`。它会让 CoppeliaSim 长时间满核甚至
ZMQ 超时。逐点用二值碰撞，距离只对关键点或有限阈值查询。

## 9. R1 箱体视觉演示

演示脚本：

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_box_motion.py
```

稳定端点数据：

```text
/home/vboxuser/桌面/workspace/r1_fixed_orientation_validation.json
```

当前默认使用 replay，不每次重新搜索 IK：

```bash
cd /home/vboxuser/桌面/workspace
python3 robot_control/demo_r1_box_motion.py \
  --speed-deg-s 16 \
  --hold-seconds 2.0
```

如果显式需要重新生成笛卡尔路径：

```bash
python3 robot_control/demo_r1_box_motion.py --replan
```

演示策略：

- 使用 Git 原四个坐标；
- 执行层用固定竖直姿态；
- 到 APP 后停留；
- APP -> TCP 下降；
- Robotiq 关闭并视觉 attach 箱体；
- attach 期间只允许“夹爪形状 <-> 已附着箱体”接触；
- 将箱体视觉抬高 60 mm，以表示不同支撑面高差；
- 抬升、确定性 APP -> APP 转移、下降、打开并 detach；
- 其他机械臂/夹爪环境碰撞仍逐仿真步检查。

最终成功日志：

```text
[REPLAY] loaded validated endpoints
[MOVE] initial_to_pick_app
[HOLD] above box
[MOVE] descend_to_pick_tcp
[GRIP] close and visually attach box
[MOVE] lift_to_pick_app
[MOVE] direct_transfer_to_place_app
[HOLD] above assembly fixture
[MOVE] descend_to_place_tcp
[RELEASE] detach box and open gripper
[DONE] R1 visual box motion complete
```

最终状态：

```text
R1 TCP       = (-1.15030, 0.20041, 0.30009)
Box_Blank    = (-1.15030, 0.20028, 0.21597)
max q error  = 0.0035 deg
```

这个成功运行是 CoppeliaSim 中的实际可见全链路，但它是“视觉抓放”，
不是真实摩擦夹持验收。

## 10. 演示调试历史

为避免下一个执行者重复踩坑，保留三次中止原因：

1. 第一次在 A -> B 转移中中止：
   视觉抬高箱体后，Robotiq `dummyMass10` 接触 `Box_Blank_Back_Wall`。
   修复：attach 期间允许夹爪与已抓工件接触。
2. 第二次在释放后中止：
   detach 先于夹爪完全打开，接触被重新当成环境碰撞。
   修复：先在 attached 允许接触状态打开，再 detach。
3. 第三次在 PLACE 下降中中止：
   前一次的 `assembly_shell` 信号使装配模板箱壁保持可见，场景中同时有
   搬运箱体和重复模板。
   修复：每次开始显式恢复工序可见状态，隐藏装配模板，只搬运
   `Box_Blank`，结束不再创建重复箱壳。

第四次完整成功。

## 11. 当前 CoppeliaSim 进程注意

成功演示使用的脚本旧版在结束时调用了 `sim.pauseSimulation()`，
仿真停在“进入 pause 前最后一步”，之后 ZMQ API 调用超时。

脚本已修改为结束时：

```text
client.setStepping(False)
```

并保持关节目标不变，不再请求 pause。但这个修正版在本次会话结束前
尚未重新实际执行。

本次文档收尾时已经完成清理：强制关闭卡住的进程，重新打开已保存的
`five_cr5a_cell.ttt`。最终已验证：

```text
simulation state = stopped
scene = /home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
/R1/R1_ROBOTIQ85 exists
R1 joint1..joint6 = all zero
Box_Blank = (-1.8, 0.35, 0.156), parent=/Parts
ZMQ localhost:23000 responds
```

下次开始时应：

1. 检查 CoppeliaSim 进程和 23000 端口；
2. 确认仿真仍为停止态、R1 六关节为 0、`Box_Blank` 在供料位；
3. 直接运行上述 replay 命令，验证新的非 pause 结束方式；
4. 仅当 API 重新超时时，才再关闭当前进程并重开保存场景。

不要保存展示过程的终点关节位或临时 visibility 状态到 `.ttt`。

## 12. 工作区关键文件

```text
/home/vboxuser/桌面/workspace/TASK_PLAN.md
/home/vboxuser/桌面/workspace/robot_control/five_cr5a_scene.py
/home/vboxuser/桌面/workspace/robot_control/install_cr5a_gripper.py
/home/vboxuser/桌面/workspace/robot_control/calibrate_r1_joints.py
/home/vboxuser/桌面/workspace/robot_control/evaluate_r1_git_targets.py
/home/vboxuser/桌面/workspace/robot_control/validate_r1_ik_candidates.py
/home/vboxuser/桌面/workspace/robot_control/search_r1_collision_free_ik.py
/home/vboxuser/桌面/workspace/robot_control/validate_r1_fixed_orientation.py
/home/vboxuser/桌面/workspace/robot_control/demo_r1_box_motion.py
/home/vboxuser/桌面/workspace/r1_calibration.json
/home/vboxuser/桌面/workspace/r1_target_ik_evaluation_robotiq85.json
/home/vboxuser/桌面/workspace/r1_ik_collision_validation_robotiq85_active_stage.json
/home/vboxuser/桌面/workspace/r1_fixed_orientation_validation.json
```

## 12A. 2026-07-17 里程碑更新

本文件前面的箱体单任务记录仍作为历史证据保留。当前最新实现是：

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_complete_cycle.py
```

该执行器已完成：

```text
箱体 PICK/PLACE
-> 端子排 PICK/PLACE
-> 退出装配共享区
-> R1 回到六关节零位
```

用户于 2026-07-17 确认同一完整视觉流程累计 `10/10` 成功。箱体与端子排
分别使用运行时姿态 `(180,0,-90)` 和 `(180,0,-180)`；开放空间默认
`50 deg/s`，精细下降限制为 `24 deg/s`。所有 Git target 均未修改。

完整说明和验证缓存：

```text
/home/vboxuser/桌面/workspace/robot_control/R1_COMPLETE_CYCLE.md
/home/vboxuser/桌面/workspace/r1_complete_cycle_plan.json
```

这仍然是视觉 attach 验收，不是物理夹持验收。R1/R2/R3 必须共同进入装配区，
因此后续多臂安全模型采用“私有供料区不重叠 + 装配共享区互斥锁”，不能把
五台机械臂的完整任务空间永久划成完全不重叠区域。

## 12B. 2026-07-17 正式接口迁移

R1 已从 `workspace` 实验脚本迁移到团队仓库的既有接口，不新增另一套
`Task` 或状态类型：

```text
sim_bridge/coppelia_client.py
  SimBridge(ISimBridge)

robot_control/robot_executor.py
  RobotExecutor(IRobotExecutor)
  execute_task(Task) -> TaskResult

robot_control/r1_motion.py
robot_control/plans/r1_complete_cycle_plan.json
robot_control/run_r1_task.py
```

已验证的真实调用有两种：

```text
R1_COMPLETE_CYCLE
  -> finished
  -> 69.05 s
  -> R1 home，最大关节误差 0.001642 deg

R1_BOX_PLACED
  -> finished，约 40.17 s
  -> 停在 R1_TERMINAL_PICK_APP

R1_TERMINAL_PLACED
  -> 从上述保留场景继续
  -> finished，约 31.04 s
  -> R1 home，最大关节误差 0.001642 deg
```

正式结果位置：

```text
Box_Blank             = (-1.150002, 0.199900, 0.215915)
Terminal_Block_Supply = (-1.089629, 0.129919, 0.301536)
```

关键修复：

1. attach 后工件绝对路径会变化，detach 必须使用 attach 前保存的稳定 handle；
2. 成功后不能 `stopSimulation()`，否则 CoppeliaSim 会把关节和工件恢复到
   初始场景，破坏箱体 Task 到端子 Task 的衔接；成功时保留运行态并释放
   stepping，失败时才停止/回滚；
3. `cell_visual_owner=executor` 时 Generator 跳过产品模板显示，但 ROS2 Bridge
   仍发布 `DONE:R1_BOX_PLACED`/`DONE:R1_TERMINAL_PLACED`；
4. 当前 `.ttt` 已重新完成完整 preflight；计划 JSON 与团队仓库副本
   SHA-256 一致，运行前还会核对场景指纹和 8 个 Git target；
5. 当时自动化测试共 21 项通过，未知任务、错误机械臂、故障和未实现的 R2-R5
   都返回 `failed`，不会假成功。

## 12C. 2026-07-17 R2 PCB 首轮路径

R2 必须接续实际 `R1_BOX_PLACED`，不能把 PCB 放到空坐标或产品模板。
首轮脚本位于：

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r2_pcb_motion.py
```

真实协作顺序已经运行成功：

```text
RobotExecutor.execute_task(R1_BOX_PLACED)
-> 实际 Box_Blank 留在装配位
-> R1 停在 R1_TERMINAL_PICK_APP，退出共享区
-> demo_r2_pcb_motion.py
-> 实际 PCB_Supply 安装进实际 Box_Blank
-> R2 回零并退出共享区
```

关键标定值：

```text
R2 runtime orientation = (195, 0, 90) deg
virtual vacuum TCP      = Link6 local +Z 0.100 m
PCB visual offset       = world +Z 0.052 m
R2 broad workspace      = x[-1.90,-0.95], y[-0.55,0.38], z[0.04,1.55]
observed bounds          = x[-1.822255,-1.030009]
                           y[-0.469755,0.280140]
                           z[0.160000,1.324944]
```

保存场景没有 R2 夹爪/吸盘，原 `R2_gripper_tip` 位于 Link6 原点。因此
`100 mm` TCP 只在运行时创建并在成功后删除，当前仍是视觉吸附，不能宣称
物理抓取。4 个 Git `R2_PCB_*_APP/TCP` 坐标和姿态均未修改。

失败证据不能删除：

1. 无工具、完全竖直的肘部支路分别碰 PCB 电容/连接器或 Link2/Link4 自碰；
2. 无工具的 30 度倾斜支路在 PCB 对齐时让主芯片碰 Link6；
3. 虚拟 TCP 的低行程竖直支路仍在 PICK 下降末端自碰；
4. 最终 `15°` 倾斜候选通过入口和下降；PCB 偏置 `50 mm` 时板底比四根
   箱体立柱顶低约 `0.622 mm`，改为 `52 mm` 后保留约 `1.38 mm` 余量。

验证结果：

```text
全场景静态检查                 = 618 个状态，无碰撞
真实 R1 箱体末态机器人间检查   = 605 个状态，无碰撞
R1/R2/PCB 最小距离             = 204.136 mm
首次真实 R1->R2 可视执行        = 成功，37.8 s 墙钟
stepping 修复后 R2 回归         = 成功
R2 最大回零误差                = 0.000766 deg
Box_Blank                      = (-1.150002,0.199900,0.215915)
PCB_Supply                     = (-1.150058,0.200161,0.281739)
PCB/箱体碰撞                   = false
R2/环境碰撞                    = false
ROS2                           = DONE:R2_PCB_PLACED
```

`SimBridge.set_stepping(True)` 原来可被同一客户端重复调用，而 CoppeliaSim
按计数处理启用请求，导致只释放一次后仿真时间冻结。现在 stepping 开关按客户端
幂等，`disconnect()` 也会释放未归还的 stepping；自动化测试共 22 项通过。

场景原始 `PCB_Supply_Area` 与装饰性 `R2_Base` 包围盒重叠，本次没有移动或
修改二者；用户确认这是其他场景成员的问题，路径工作暂不处理。

## 12D. 2026-07-17 R2 正式接口迁移与实跑

R2 已迁入主仓库既有接口，没有引入另一套 Task 或命名：

```text
robot_control/r2_motion.py
robot_control/robot_executor.py
robot_control/run_r2_task.py
robot_control/R2_EXECUTOR.md

R2_PCB_PLACED -> R2
```

正式模块自包含 simIK 求解、确定性笛卡尔路径、运行时 `100 mm` 虚拟 TCP、
minimum-jerk 执行、PCB 视觉 attach、碰撞/自碰撞和 R2 工作区检查，不再依赖
`/home/vboxuser/桌面/workspace` 的规划辅助模块。执行前强制核对场景指纹、
4 个 Git R2 target、R1 箱体末态、R2 零位、实际箱体/PCB 的坐标和 `/Parts`
父节点。同一 `RobotExecutor` 实例把同一个装配区锁传给 R1 和 R2。

自动化测试由 23 项增加到 28 项，覆盖 R2 精确映射、错误机械臂拒绝、控制器
选择、共享锁、成功状态更新、失败恢复和 R1 回归，全部通过。

从重新加载的保存场景实际执行：

```text
python3 robot_control/run_r1_task.py R1_BOX_PLACED
  -> finished，287.7 s
  -> Box_Blank = (-1.150002,0.199900,0.215915)

python3 robot_control/run_r2_task.py R2_PCB_PLACED
  -> finished，294.6 s
  -> PCB_Supply = (-1.150058,0.200161,0.281739)
  -> R2 最大回零误差 = 0.000766 deg
```

虚拟机中的 CoppeliaSim 软件渲染/API 明显变慢，但运动速度参数仍是开放空间
`50 deg/s`、精细下降 `24 deg/s`。第一次 R1 尝试被外部 180 秒命令上限在
中途终止，随后停止、重载干净场景再完整成功；该中断不计成功次数。正式
R2 成功后确认仿真时间继续前进、箱体和 PCB 均归属 `/Parts`，运行时 TCP/
命令脚本无残留。额外重新创建 postflight 碰撞集合时远程 API 超时，不是
检测到碰撞；正式 Task 内的放置端点和回零端点安全检查已经通过。

## 12E. 2026-07-17 今日工作最终记忆快照

### 用户最终目标与当前策略

最终目标仍是五台 CR5A 协同完成装配且互不碰撞。用户允许为每台机械臂设置
不重叠的私有工作区；由于 R1/R2/R3 都必须进入同一个箱体装配位置，完整运动
包围盒不可能永久互不重叠，因此正式策略固定为：

```text
私有供料区互不重叠
+ 每台机械臂运行时工作区墙
+ 装配共享区互斥锁
+ 运行时环境/自碰撞/负载碰撞检查
```

当前视觉阶段允许隔空夹持或视觉吸附，但不允许机械臂、工具、负载与场景或
其他机械臂发生碰撞，也不能把视觉 attach 描述成物理抓取验收。

### 五机械臂最终协调与流畅性目标

用户在 2026-07-17 再次明确：最终效果不能只是五台机械臂各自单独成功，
而要让整个生产单元连续、协调、自然地工作。尤其是 R1 完成箱体工序并退出
装配共享区后，R2 应立即开始有效运动；后续 R2->R3、R3->R1/R4、R4->R5
等相邻工序也适用同一要求。

“墙钟时间”是命令从开始到结束在现实世界中实际经过的时间，会受虚拟机 CPU、
软件渲染和 ZMQ/API 响应影响；它不等于机械臂轨迹时间，也不能单独用于评价
动作是否流畅。最终协调性应以 CoppeliaSim 仿真时间和轨迹状态测量：

```text
handoff_delay = 下一机械臂首次有效关节运动的仿真时间
                - 上一机械臂释放/清空共享区的仿真时间

初步验收目标：handoff_delay <= 0.5 s 仿真时间
理想目标：一个调度/控制周期内开始下一机械臂有效运动
```

该目标可以实现，但当前分别运行 `run_r1_task.py` 和 `run_r2_task.py` 只是
路径及接口验证方式，不能作为最终无缝协同实现。最终需要：

1. 使用常驻的五臂调度器和同一个长期运行的 `RobotExecutor/SimBridge`，
   不在相邻工序之间停止、重载或重新连接仿真；
2. 将已经验证的 R1-R5 路径预计算、缓存或提前加载，不能在关键交接时等待
   在线 IK/路径生成；
3. 为每台机械臂维护独立状态机，并由统一 stepping/control loop 同步发送
   多臂关节目标；当前全局 `_execution_lock` 方案后续需要演进，不能让无关
   私有区动作被整段串行化；
4. 使用显式区域事件，例如 `R1_SHARED_ZONE_CLEARED`，上一台释放共享区锁后
   立即唤醒下一台，而不是依赖人工敲下一条 CLI 命令；
5. 下一台机械臂可在自己的不重叠私有区提前完成预定位、接近或取料，然后
   等待共享区锁，从而缩短整线 makespan；
6. 记录每次交接延迟、路径持续时间、共享区等待时间、碰撞检查结果和整线
   makespan，用数据和最终视觉效果共同验收。

缩短间隔不能以删除碰撞检查、放宽工作区墙、忽略机器人间距离、提高已经
验证的精细下降速度或造成突然加减速为代价。安全、平滑和协调必须同时满足。

### 最终运行命令的即时启动目标与优化时机

用户进一步明确了两项优先级决策：

1. 当前先完成 R3-R5 单臂路径和五机械臂完整协同功能；在五臂协同链路
   正确、无碰撞并可连续运行之前，不展开详细墙钟性能优化，也不因性能重构
   打断功能开发；
2. 最终用户输入运行脚本或点击运行后，机械臂应立刻开始有效运动，不能长时间
   停在原地做在线 IK、重新规划、重复初始化或等待人工输入下一条命令。

该目标技术上可以实现，但必须写清运行前提和测量方式：

```text
command_to_motion_latency = 第一台机械臂首次有效关节运动的墙钟时间
                            - 用户运行命令被接收的墙钟时间

场景已加载、常驻执行器 ready 时的初步目标：
command_to_motion_latency <= 1.0 s 墙钟时间
理想目标：一个调度/控制周期内下发第一段运动
```

如果运行命令还负责从零冷启动 CoppeliaSim、加载 2.8 MB 场景和初始化插件，
软件启动时间客观存在，不能承诺物理意义上的零延迟。最终用户体验应采用：

```text
CoppeliaSim 场景和五臂协调器提前启动并进入 READY
-> 用户运行最终生产循环脚本/命令
-> 薄命令客户端立即触发已经预加载的第一个 Task
-> R1 在一个控制周期内开始运动
```

实现方式在五臂功能完成后统一进行：常驻调度器、单一长期 SimBridge 连接、
场景指纹和静态 target 在 READY 阶段检查、R1-R5 路径缓存/预加载、每个 Task
只保留动态关节/工件/锁前置检查，并将逐步执行放入统一 control loop 或场景
内部脚本。不能为了达到 `<=1.0 s` 而跳过动态安全检查。

当前两个 CLI 的启动耗时和 R2 在线 simIK 暂时保留，因为它们仍用于路径验证；
这不代表最终交付会保留启动空档。下一阶段优先继续 R3 路径，不要现在为了
几十秒墙钟差异大规模重构已经验证的 R1/R2。

### 必须保持的团队接口

```text
SimBridge implements ISimBridge
RobotExecutor implements IRobotExecutor
execute_task(Task) -> TaskResult
get_robot_states() -> list[RobotState]

R1_BOX_PLACED      -> R1
R1_TERMINAL_PLACED -> R1
R1_COMPLETE_CYCLE  -> R1
R2_PCB_PLACED      -> R2
R4_SCREW_DONE      -> R4
```

不要新增另一套 Task、状态、机械臂 ID、场景路径或命令名。未知 R3/R5 动作、
错误机械臂分配和任何前置条件失败都必须返回 `TaskResult.status == "failed"`。

### 今日新增和修改的正式交付

```text
robot_control/r2_motion.py
robot_control/run_r2_task.py
robot_control/R2_EXECUTOR.md
robot_control/robot_executor.py
tests/test_robot_executor.py
README.md
docs/INTERFACES.md
AGENTS.md
PROJECT_CONTEXT.md
/home/vboxuser/桌面/workspace/TASK_PLAN.md
```

`r2_motion.py` 已自包含 simIK 求解和 Cartesian path 生成，不得重新加入对
`workspace/robot_control/evaluate_r1_git_targets.py` 或
`validate_r1_fixed_orientation.py` 的运行时依赖。同一 `RobotExecutor` 实例
将同一个 `self._assembly_lock` 传给 R1 和 R2；两个独立 CLI 进程只能严格
顺序执行，不能宣称具有跨进程全局锁。

### R2 受保护基线和验证值

四个 Git target 的位置和 `{0,0,0}` 姿态均未修改：

```text
R2_PCB_PICK_APP  = (-1.28,-0.28,0.45)
R2_PCB_PICK_TCP  = (-1.28,-0.28,0.22)
R2_PCB_PLACE_APP = (-1.15, 0.20,0.50)
R2_PCB_PLACE_TCP = (-1.15, 0.20,0.29)

runtime orientation = (195,0,90) deg
runtime virtual TCP  = Link6 local +Z 0.100 m
PCB visual offset    = world +Z 0.052 m
open-space speed     = 50 deg/s
precision descent    = 24 deg/s
R2 workspace         = x[-1.90,-0.95], y[-0.55,0.38], z[0.04,1.55]
```

场景指纹仍是：

```text
size   = 2875907
sha256 = 0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657
```

不得移动用户指出的 `PCB_Supply_Area` 或 `R2_Base`。两者的装饰几何重叠是
原 Generator 布局问题，用户已经明确要求本路径工作暂不处理。

### 今日真实运行证据

从重新加载的干净场景严格顺序执行：

```text
formal R1_BOX_PLACED
  status    = finished
  wall time = 287.7 s
  Box_Blank = (-1.150002,0.199900,0.215915)
  R1        = R1_TERMINAL_PICK_APP

formal R2_PCB_PLACED
  status    = finished
  wall time = 294.6 s
  PCB       = (-1.150058,0.200161,0.281739)
  Box_Blank = (-1.150002,0.199900,0.215915)
  R2 max home error = 0.000766 deg
```

用户要求最后再次演示后，从另一轮重新加载的干净场景重复正式链路成功：

```text
formal R1_BOX_PLACED
  status    = finished
  wall time = 43.1 s

formal R2_PCB_PLACED
  status    = finished
  wall time = 40.2 s
  PCB       = (-1.150058,0.200161,0.281739)
  Box_Blank = (-1.150002,0.199900,0.215915)
  R2 max home error = 0.000766 deg
```

第二次正式链路成功后的独立 postflight 也完整通过：仿真时间继续推进，箱体
和 PCB 父节点均为 `/Parts`，R2 运行时对象残留为空，终态环境碰撞、自碰撞、
PCB-to-R2 和工作区检查全部通过。正式 R1->R2 链路现已实际成功 2 次；仍未
达到用户验收的 10/10，不能把实验脚本的静态准备回归重复计入正式链路次数。

成功后仿真仍为运行态且仿真时间继续前进；箱体与 PCB 父节点均为 `/Parts`；
R2 运行时虚拟 TCP 和命令脚本无残留。正式执行过程中放置端点和回零端点的
环境碰撞、自碰撞、PCB-to-R2 和工作区检查均通过。

不能删除或误写以下失败边界：

1. 第一次正式 R1 运行被外部 `180 s` 命令上限中途终止，仿真时间约
   `16.55 s`；这不是路径碰撞，但也不是成功，已重启干净场景后重跑；
2. 成功链路后的额外 postflight 在重新创建碰撞集合时发生 ZMQ API 超时，
   不是检测到碰撞；在超时前已经确认仿真时间推进、工件父节点和无运行时
   对象残留；
3. 当前虚拟机 CoppeliaSim 软件渲染/API 很慢，不能为了缩短墙钟擅自提高
   已验证的精细下降速度或删除安全检查。

### 静态验证和 Git 状态

```text
python3 -m unittest discover -s tests -v
Ran 28 tests
OK
```

R1/R2/执行器/CLI/测试文件均通过 `py_compile`，本次相关文件通过
`git diff --check`。完整仓库检查仍会报告用户原有的
`cr5_moveit/config/kinematics.yaml` 文件末尾空行；不要顺手修改或回退该
无关文件。工作树仍故意保持脏状态，今天没有 commit、push、reset 或 clean。

### 当前仿真和下一次恢复点

CoppeliaSim 进程仍打开
`/home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt`，并保持在
成功的 R1 箱体加 R2 PCB 最终可视状态，供用户直接观察。不要保存该瞬态
关节/工件状态到 `.ttt`。

下一次先读取本文件、`AGENTS.md`、`PROJECT_CONTEXT.md` 和
`workspace/TASK_PLAN.md`，再检查 Git 与 CoppeliaSim 实际状态。后续顺序：

1. 用户观察并确认正式 R2 姿态、平滑度和 PCB 箱内效果；
2. 重载干净场景，继续累计正式 `R1_BOX_PLACED -> R2_PCB_PLACED` 到 10/10；
3. R2 验收后按相同方法开始 R3 控制模块安装路径；
4. R3-R5 单臂稳定后再做完整多臂时序、共享区调度和动态避碰验证。
5. 所有单臂路径具备后，将上述 `<=0.5 s` 仿真时间交接目标纳入最终五臂
   连续演示验收，并逐段消除停止、重连、在线规划和人工触发造成的空档。
6. 五臂功能正确后再集中做墙钟优化；最终在场景/协调器 READY 前提下要求
   用户运行命令到 R1 首次有效运动 `<=1.0 s` 墙钟时间，理想为一个控制周期。

## 12F. 2026-07-18 R2 暂停、R3/R5 场景证据与 R4 正式实跑

### 用户最新优先级

- R2 正式验收和 10/10 暂停，等待场景组修复
  `PCB_Supply_Area` 与 `R2_Base` 重叠；不得移动这两个对象或用碰撞排除掩盖；
- 当前正式 R1->R2 成功口径按用户最新指令记为 1 次；
- 不等待 R2 新场景，继续 R3-R5 单臂功能；受保护 target 仍不得未经批准修改。

### R3 控制模块场景证据

R3 保存场景无工具，原 tip 在 Link6 原点，9 个 Git target 姿态均为零。
零姿态出现 IK 失败和 Link3 对地面/左台碰撞。真实 R2 PCB 终态下，模板模块
位置 `(-1.12,0.22,0.3015)` 穿入 PCB 板和主芯片。临时抬高方案通过 605
状态检查，但视觉位置不理想，未写入正式代码。

给场景组的建议候选是把模块中心大致移到
`(-1.105,0.185,0.3035)`，同时更新产品模板和 R3 MODULE PLACE APP/TCP 的
XY；正式值仍需新场景回归，本仓库没有修改 target。

### R4 正式视觉锁付

用户明确批准：

```text
runtime screwdriver TCP = Link6 local +Z 0.100 m
runtime orientation     = (180,0,-135) deg，工具轴竖直向下
Git R4 targets          = unchanged
```

正式交付：

```text
robot_control/r4_motion.py
robot_control/run_r4_task.py
robot_control/R4_EXECUTOR.md
robot_control/robot_executor.py
R4_SCREW_DONE -> R4
```

运行时创建手柄、细杆、旋转标记和 TCP，执行 APP 停留、TCP 下降、PRESS
下压、两圈可见旋转、撤回和回零。静态正式代码检查覆盖 364 个路径状态和
33 个旋转状态，全部通过。首次清理检查发现删除父 Dummy 会重挂细杆/标记，
正式代码已改为先删子对象再删父对象，并加入回归测试。

正式实际执行结果：

```text
python3 robot_control/run_r4_task.py R4_SCREW_DONE
status                 = finished
wall time              = 22.9 s
R4 max home error      = 0.000632 deg
runtime tool/script    = none
environment collision  = false
self collision         = false
simulation time        = continues advancing
cell_screw_state       = done
inspection product     = (0.15,0.05,0.216), parent=/Parts, 30 shapes visible
```

该里程碑是视觉旋转/下压，不是物理扭矩或螺钉接触力验收。R4 接入后测试
由 28 项增加到 33 项；新增新场景审计工具后完整测试为 38 项，全部通过。

用户随后要求按“先观察、再 10/10、场景修改清单、自动审计工具”的顺序
推进。重新加载干净场景后的第二次正式 R4 观察演示也成功，墙钟 `21.7 s`，
回零最大误差仍为 `0.000632°`，仿真时间推进且 Runtime 对象为空。该次
视觉效果等待用户明确确认；确认前没有自动开始后续 8 次重复运行。

### R5 诊断边界

R5 无工具，原 tip 在 Link6 原点。两个 PLACE_TCP 会让产品根节点落在
`z=0.296 m`，而 Generator 带面期望 `z=0.270 m`，相差 26 mm。

缺陷品候选用 `100 mm` TCP、`(195,-45,0)` 度姿态和
`(-0.15,-0.15,0.65)` 中转点，通过携完整 30 形状产品的 705 状态静态
检查；只排除按设计覆盖产品的非物理 `Camera_View_Area`。

合格品两个候选 `(150,45,45)` 和 `(150,45,90)` 都在抬升第 53 状态发生
产品箱体前壁对 R5 Link2 碰撞。安全姿态扫描合格带入口邻域也无候选，R5
正式映射保持未实现，等待场景基座/传送带布局或真实抓取方案调整。

### 当前 CoppeliaSim 状态

多次密集静态碰撞检查曾使端口监听但 API 无响应；均未保存瞬态，强制结束
卡死进程后重新打开原指纹场景。当前实例保持在成功 R4 任务后的运行态，
仿真时间继续推进，R4 已回零，检测产品可见，R4 Runtime 对象为空。不得保存
该运行态到 `.ttt`。

### 新场景交付准备

场景组集中修改意见已写入：

```text
docs/SCENE_CHANGE_REQUEST_2026-07-18.md
```

主仓库新增只读审计工具和结构化基线：

```text
sim_bridge/audit_five_cr5a_scene.py
configs/five_cr5a_scene_audit_baseline.json
tests/test_scene_audit.py
```

工具不加载、启停或保存场景。当前旧场景实跑审计正确返回 `blocked`，检出
R2 供料区/基座重叠、R3 模块/PCB 模板重叠、R5 good/defect 各 26 mm
带面高度差，并把非物理 `Camera_View_Area` 与产品重叠列为 warning。

## 12G. 2026-07-18 R4 正式视觉验收 10/10

用户确认第二次 R4 观察演示的姿态、速度、下压和旋转效果可接受。已批准的
运行时参数保持不变：

```text
runtime screwdriver TCP = Link6 local +Z 0.100 m
runtime orientation     = (180,0,-135) deg
rotation                = 2 visible turns
Git R4 targets          = unchanged
```

为继续正式重复验收，主仓库新增：

```text
robot_control/repeat_r4_acceptance.py
data/logs/r4_repeat_acceptance_2026-07-18.json
```

重复运行器在每次 Task 前停止并重新加载指纹一致的保存场景，核对场景路径、
停止态、R4 六关节全零、检测产品位置/父节点和无 Runtime 残留，然后通过正式
`R4_SCREW_DONE -> R4` 接口执行。每次结束独立核对仿真时间推进、R4 回零、
Runtime 清理、环境/自碰撞、检测产品位置、父节点和 30 个形状可见性。

第 3-10 次共 8 次新运行结果：

```text
command exit/status       = 8/8 exit 0, finished
wall time range           = 21.187-22.415 s
wall time mean/median     = 21.939 / 22.027 s
max R4 home error         = 0.000632 deg
simulation time advances = 8/8
runtime objects empty     = 8/8
environment collision    = none, 8/8
self collision           = none, 8/8
postflight failures       = none, 8/8
```

结合先前两次正式成功 `22.9 s` 和 `21.7 s`，R4 正式视觉验收达到 `10/10`，
10 次平均墙钟约 `22.011 s`。检测产品每次保持
`(0.15,0.05,0.216)`、`parent=/Parts` 且 30 个形状可见。场景文件仍保持
`size=2875907`、
`sha256=0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657`。

该结论仅表示视觉螺丝刀旋转/下压任务的重复执行验收，不表示物理扭矩、
接触力或真实螺钉锁紧已经验证。

## 12H. 2026-07-18 接替会话最终恢复快照

用户说明上一 Codex 已完成 R4 `10/10`，本次接替没有重复运行机械臂，而是
完整读取项目上下文并核验上一会话留下的运行器、结构化日志、逐次 postflight
和当前 CoppeliaSim 现场。结论与第 12G 节一致，R4 正式视觉验收确认为
`10/10`，不再继续重复运行。

本次核验的正式证据：

```text
runner = robot_control/repeat_r4_acceptance.py
log    = data/logs/r4_repeat_acceptance_2026-07-18.json

prior_successes       = 2
requested_new_runs    = 8
successful_new_runs   = 8
total_successes       = 10
acceptance_passed     = true
new-run mean/median   = 21.939 / 22.027 s
ten-run mean          = approximately 22.011 s
max home error        = 0.000632 deg
postflight failures   = none
```

本次重新运行的非机械臂验证：

```text
python3 -m py_compile robot_control/repeat_r4_acceptance.py \
  robot_control/run_r4_task.py robot_control/r4_motion.py
  -> passed

python3 -m unittest discover -s tests -v
  -> Ran 38 tests
  -> OK
```

最终只读现场检查：

```text
scene = /home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
simulation state = 17, simulation_advancing_running
simulation time = continues advancing
R4 max home error = 0.000631829 deg
R4 Runtime objects = none
Inspection_ControlBox_Product = (0.15,0.05,0.216)
product parent = /Parts
visible product shapes = 30/30
cell_screw_state = done
active R4 task/repeat process = none
```

保存场景没有写入运行瞬态：

```text
size   = 2875907
sha256 = 0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657
mtime  = 2026-07-17 18:00:45 +08:00
```

Git 仍在 `main@74ff605331211b993f2f9a08aad85d0f6d591f44`，保持原有脏工作树；
本次没有 commit、push、reset、clean 或 checkout。`git diff --check` 仍只报告
既有的 `cr5_moveit/config/kinematics.yaml:5` 文件末尾空行，不要顺手修改。

本次同步更新：

```text
/home/vboxuser/桌面/workspace/TASK_PLAN.md
docs/CR5_SESSION_MEMORY_2026-07-15.md
robot_control/R4_EXECUTOR.md
AGENTS.md
PROJECT_CONTEXT.md
```

没有修改 R4 或其他机械臂的 Git HOME/APP/TCP/PRESS target、运行时姿态、
速度、碰撞检查、场景对象或 `.ttt`。R4 保持视觉锁付边界，不得宣称物理扭矩。

下一次工作从这里继续：

1. 不再运行 R4 重复验收；保留 `10/10` 日志和批准参数；
2. 保持 R2 验收暂停，不移动 `PCB_Supply_Area` 或 `R2_Base`；
3. 等待场景组交付修正版场景；到达后必须先停止态打开并运行只读审计；
4. 新场景先核对指纹、target、对象位置和高度，再回归 R1/R2/R3/R4/R5；
5. 场景修复后恢复 R2 10/10，并实现 R3/R5 正式控制器；
6. 全部单臂稳定后再做 ROS2 动作触发、常驻五臂调度和共享区事件交接。

### 下次会话恢复提示词

```text
请恢复 CR5 五机械臂路径规划项目上下文，并使用 memories 中所有与 CR5 项目
有关的记忆。仓库是 /home/vboxuser/桌面/cr5_assembly_team。

先完整阅读 AGENTS.md、PROJECT_CONTEXT.md、
docs/CR5_SESSION_MEMORY_2026-07-15.md 和
/home/vboxuser/桌面/workspace/TASK_PLAN.md；重点核对会话记忆第 12D-12H、
第 13-14 节，以及 TASK_PLAN 的 E1、E3、当前进度表、下一次工作和最新审计
日志。然后阅读 docs/Five_CR5A_Cell_Control_Interface.md、
docs/INTERFACES.md、R1/R2/R4 执行文档、robot_executor.py、r1_motion.py、
r2_motion.py、r4_motion.py、repeat_r4_acceptance.py、SimBridge、场景对象契约和
相关测试。不要只看摘要，阅读完成前不要修改文件或运行机械臂。

之后只读检查 git status、CoppeliaSim 进程、当前场景路径/仿真状态、R1/R2/R4
六关节、关键工件位置/父节点、Runtime TCP/Command Bridge 残留，以及场景
指纹。先用中文汇报恢复到的目标、R1/R2/R4 已完成能力、R4 10/10 证据、
当前现场、Git 状态、R2/R3/R5 阻塞和下一步，再等我确认。

必须保持：R4 正式视觉验收已 10/10，不要重复运行；R2 正式链路成功口径为
1 次且因 PCB_Supply_Area/R2_Base 重叠暂停；R3 模块布局和 R5 good 分支等待
场景修复；视觉 attach/吸附/螺丝刀不等于物理抓取或扭矩验收。不要移动
PCB_Supply_Area 或 R2_Base，不要修改 Git HOME/APP/TCP/PRESS，不要让正式
代码依赖 workspace 临时规划模块，不要保存运行瞬态到 five_cr5a_cell.ttt，
也不要 reset/clean/checkout/commit/push，除非我明确要求。
```

## 12I. 2026-07-19 当前场景五臂基础协同

用户因周一晚上的团队截止时间调整优先级：不再等待场景组，先在当前
原指纹 `.ttt` 中完成可供其他成员使用的五机械臂基础协同。本次没有修改场景
对象、基座、传送带、Git HOME/APP/TCP/PRESS 或 `five_cr5a_cell.ttt`。

新增正式动作：

```text
R3_MODULE_PLACED          -> R3
R3_PRODUCT_TO_INSPECTION -> R3
R5_SORT_GOOD_DONE        -> R5
R5_SORT_DEFECT_DONE      -> R5
```

新增一键协调入口：

```text
robot_control/five_arm_coordinator.py
robot_control/run_five_arm_cycle.py
robot_control/FIVE_ARM_COORDINATOR.md
```

正式 good 完整工艺实跑：

```text
R1_BOX_PLACED
-> R2_PCB_PLACED
-> R3_MODULE_PLACED
-> R1_TERMINAL_PLACED
-> R3_PRODUCT_TO_INSPECTION
-> R4_SCREW_DONE
-> R5_SORT_GOOD_DONE
```

七个 Task 在同一个长期 `SimBridge/RobotExecutor` 实例中全部 `finished`，不在工序间
停止、重载或重连仿真。证据：

```text
log                       = data/logs/five_arm_good_cycle_2026-07-19.json
total wall time           = 319.515 s
normal handoff delay      = 0.0 s simulation time
camera transition to R4   = 0.05 s simulation time
all robot states          = idle/home
all Runtime objects       = none
final robot collisions    = none
simulation time           = continues advancing
```

独立 defect 完整工艺也从干净场景实跑成功：

```text
log                       = data/logs/five_arm_defect_cycle_2026-07-19.json
seven formal tasks        = 7/7 finished
total wall time           = 296.252 s
handoff delay             = 0.0-0.05 s simulation time
R5 max home error         = 0.005805 deg
Runtime objects           = none
defect product            = carried to defect conveyor
```

最大回零误差：

```text
R1 = 0.001642 deg
R2 = 0.000766 deg
R3 = 0.001519 deg
R4 = 0.000632 deg
R5 = 0.008758 deg
```

必须保留的协同边界：

1. Robotiq 原脚本的 open 模式没有位置上限，长时间仿真会让
   `dummyMass8/10` 沿法兰漂出约 `0.55 m`。`SimBridge.freeze_gripper()`
   在每次 `0.8 s` 开合动画后禁用工具脚本并将主动关节速度置零，
   下次开合前再启用；不得保存该运行态。
2. 安装真实 PCB 后，端子原 `28 mm` 视觉偏移与 PCB 板/连接器重叠。
   协同分段任务使用 `56 mm` 总偏移，获得约 `4.3 mm` 纵向余量；
   独立 R1 完整循环仍保留原 `28 mm`。
3. R3 模块使用 `100 mm` TCP、`(195,0,-135)` 度姿态和 `+46 mm`
   视觉偏移。R3 完整产品从基座前侧绕行，搬运时临时上提 `100 mm`。
4. R3/R5 只排除按设计包围检测产品的非物理 `Camera_View_Area`；
   固定相机机身/立柱、平台、传送带和其他机械臂仍检查。
5. R5 defect 携完整产品到带面。R5 good 在当前局部布局中无法刚性携物
   穿过 PLACE 下降的 Link4，因此基础演示采用“good APP 视觉释放
   -> R5 空载 APP/TCP 手势和回零 -> 产品垂直进带”。它不是刚性负载验收。
6. R5 两个分支仍用运行时 `-26 mm` 带面高度修正，不改 target。

自动化测试更新为：

```text
python3 -m unittest discover -s tests -v
Ran 45 tests
OK
```

视觉 attach/吸附/模板转换/螺丝刀仍不等于物理抓取、刚性负载或物理扭矩验收。

## 13. 下一步

不要立即回到物理夹持 target 校准。按用户 2026-07-19 最新决策继续：

1. 先整理本次五臂协同代码与组长/4 号/5 号成员的调度、GUI 进行集成；
2. 用户确认后再 commit/push，不得未经确认直接上传现有脏工作树；
3. 保留 good/defect 两种一键命令，集成时只通过标准 `Task/TaskResult` 调用；
4. 场景组后续仍应修正 R2 重叠、R3 模块布局、R5 good 刚性携物和 `26 mm`
   带面高度差；新场景到达后必须重新审计和回归；
5. R4 `10/10` 证据仍保留，不因本次协同运行改写为物理扭矩验收；
6. 只有用户重新要求物理真实性时，才进行真实吸盘/螺丝刀扭矩、target 调整、
   最小间隙和物理负载验收。

## 14. 恢复会话时的必读顺序

```text
1. /home/vboxuser/桌面/cr5_assembly_team/AGENTS.md
2. /home/vboxuser/桌面/cr5_assembly_team/PROJECT_CONTEXT.md
3. /home/vboxuser/桌面/cr5_assembly_team/robot_control/PICK_AND_PLACE_RVIZ.md
4. /home/vboxuser/桌面/cr5_assembly_team/docs/CR5_SESSION_MEMORY_2026-07-15.md
5. /home/vboxuser/桌面/workspace/TASK_PLAN.md
6. git status
7. CoppeliaSim 进程、23000 端口和当前场景状态
```

读完后直接从第 13 节继续，不要重新从旧 `/CompactCell`、自写 FK-RRT*
或随机 IK 分支开始。

## 15. 2026-07-19 R5 good 固定同步重规划与 60 度速度回归

本节晚于第 12-14 节，涉及 R5 good 的旧结论时以本节为准。用户实际观察到
旧 good 分支存在抓取后腕部扭曲、在 APP 提前释放、机械臂空手下降以及产品
依靠世界坐标动画“乱飞”等问题。单独以 `50 deg/s` 演示两条 R5 路线后，
用户确认 defect 外观可接受，问题集中在 good。

用户现场提出并逐步验证的新约束：

```text
defect/good 共用抓取和竖直抬升
-> 抬到 PICK_APP 后，good 先向相反方向转 joint1
-> 其余关节再协同到 good APP/TCP
-> attach 后产品必须与吸盘保持完整位置/姿态同步
-> 到带面后才释放
```

诊断和失败证据必须保留：

1. good APP 直接复用 defect `(195,-45,0)` 姿态，IK 位置残差
   `151.792 mm`，不可采用；
2. 一次候选搜索把产品挂到 TCP 后又每步强制世界水平，导致产品相对吸盘
   姿态持续变化。用户指出抓取点滑动后立即停止；该方法永久禁止；
3. 离线搜索曾临时把 R5 可见层设为 0，界面看似 R5 被删除。对象和 `.ttt`
   从未删除或保存，搜索 finally/场景重载后确认 R5 16/16 形状可见、六关节
   全零、Runtime 为空；
4. joint1 预转 `-110 deg` 时 good APP 残差 `40.289 mm`，`-120 deg` 为
   `9.013 mm`，`-122 deg` 为 `2.761 mm`，`-122.5 deg` 为 `1.203 mm`；
   `-123 deg` 首次通过 1 mm IK 精度和完整携物安全检查；
5. 第一次正式刚性同步 good 在最终 26 mm 下降处报告产品底面接触目标
   good 带面；这是目标接触，不允许因此排除整条输送带；
6. 精确释放 Guard 初版把已挂到 R5 树下的 payload 误算进 robot shapes，
   再次误报目标接触。`RobotSafetyGuard` 现先计算 payload shapes，并从
   robot shapes 中减去；目标接触仅允许 payload 对当前 belt，R5 本体仍检查。

正式 R5 good 路线：

```text
runtime TCP offset       = 0.100 m
pick orientation         = (195,-45,0) deg
joint1 relative pre-turn = -123 deg
good place orientation   = (-144.158663,32.588747,105.956857) deg
good high waypoint       = (0.630634,-0.949776,0.760000) m
belt correction          = TCP and payload move together -0.026 m
grasp contract           = product-to-TCP 7D pose unchanged until release
```

`robot_control/r5_motion.py` 不再在 attach 后调用产品 APP 水平/垂直对齐、
世界姿态锁或产品独立 belt offset。good/defect 都生成额外运行时 release TCP，
让机械臂和产品同步下降 26 mm。`robot_control/runtime_cartesian.py` 增加
`allowed_payload_contacts`：只允许产品接触当前目标带面，同时保留机器人对
带面、产品对其他环境、产品对 R5、自碰撞和工作空间检查。

单臂正式证据：

```text
R5 good wall time              = 59.046 s
R5 good grasp transform error  = 2.220446049250313e-16
R5 good max home error         = 0.004886 deg
R5 defect wall time            = 57.669 s
R5 defect grasp transform error= 2.7755575615628914e-16
R5 defect max home error       = 0.005710 deg
both Runtime objects           = none
both independent postflight    = passed
```

随后从干净停止场景运行五臂 good：

```text
log                       = data/logs/five_arm_good_rigid_60_2026-07-19.json
command speed             = 60 deg/s
APP hold                  = 0.4 s
status                    = finished
seven formal tasks        = 7/7
total wall time           = 237.246 s
total simulation time     = 172.800 s
R5 wall/simulation time   = 48.295 / 35.950 s
R5 grasp transform error  = 1.6653345369377348e-16
old 50 deg/s baseline     = 319.515 s
wall reduction            = 82.269 s, about 25.7 percent
```

五臂 good 独立 postflight 完整通过：仿真时间继续推进；R1-R5 均回零，
最大误差为 R5 `0.007643 deg`；所有 `R*_Runtime_*` 为空；五臂最终环境和
自碰撞均为 none。`python3 -m unittest discover -s tests -v` 为 45/45 通过。

没有运行五臂 defect `60 deg/s`：用户明确要求暂缓。协调器和 CLI 默认值也
尚未从 `50/0.8` 改为 `60/0.4`。不得把旧 defect 日志冒充新速度验收。

## 16. 当前未开始工作和严格顺序

1. R5 good 最终产品目前有水平 yaw，尚未与 good 输送轨道平行。下一步联合
   搜索 joint1 预转角和转后 TCP yaw；产品到 TCP 变换必须固定，禁止单独
   旋转已 attach 产品；
2. 单臂 good 重跑并做独立 postflight；
3. 用户允许后才运行五臂 defect `60 deg/s`、APP `0.4 s` 完整回归；
4. good/defect 新速度均通过后，才修改默认值并重跑 45 项测试和两条正式日志；
5. 之后做组长调度/GUI 集成；
6. 当前 `handoff_delay_simulation_s` 只量到下一个 Task 调用，不是真实首个
   关节运动。调度集成后增加真实首动交接测量；
7. 最后整理提交文件集，排除 `/tmp` 搜索脚本和试验日志，用户确认后才
   commit/push；当前场景和受保护 Git target 均未修改。

## 17. 下一会话恢复口令模板

将下面文字原样发送给下一会话；下一会话仍须完整阅读所列文件，不能只依赖
口令摘要：

```text
恢复 CR5 五机械臂项目 2026-07-19 最新上下文。仓库是
/home/vboxuser/桌面/cr5_assembly_team。先完整阅读 AGENTS.md、
PROJECT_CONTEXT.md、docs/CR5_SESSION_MEMORY_2026-07-15.md 的 12D-17 节、
/home/vboxuser/桌面/workspace/TASK_PLAN.md 的 E1/E3、当前进度表、
下一次工作和最新审计日志，以及 R5_EXECUTOR.md、r5_motion.py、
runtime_cartesian.py；阅读完成前不要修改文件、重载场景或运行机械臂。
最新成功是 R5 good 固定产品到 TCP 变换路线和五臂 good 60 deg/s 7/7，
下一步先优化 R5 good 产品 yaw 使其与 good 输送轨道平行。不要单独改写
attach 后产品位姿，不要运行用户暂缓的 defect 60 deg/s，除非我重新允许。
阅读后先报告当前状态、已验证证据、未完成顺序和场景/进程状态。
```

## 18. 2026-07-20 R5 good 输送轨道 yaw 对齐

本节晚于第 15-17 节；涉及 R5 good 当前路线和下一步时以本节为准。用户
明确要求继续既定计划，并同意使用执行层运行时 APP/TCP XY 偏移。没有修改
Git target、传送带或 `five_cr5a_cell.ttt`。

诊断确认旧路线最终产品 yaw 约为 `-123 deg`，与沿世界 `-Y` 的 Good 轨道
目标 `-90 deg` 相差 `33 deg`。原 Good APP/TCP 坐标下对齐姿态分别存在约
`97.971 mm` 和 `13.870 mm` IK 残差。采用最小共同运行位置后，新路线为：

```text
Git Good APP/TCP          = unchanged at (0.65,-1.10,0.62/0.42) m
runtime XY offset         = X -0.010 m, Y +0.020 m
runtime Good APP/TCP      = (0.64,-1.08,0.62/0.42) m
joint1 relative pre-turn  = -121 deg
high waypoint             = (0.630721,-0.931635,0.760000) m
high/APP prealign         = (-143.152079,31.403342,104.057370) deg
aligned TCP orientation   = (-134.007027,10.545291,79.271417) deg
APP -> aligned TCP        = 101-point deterministic joint interpolation
release                   = rigid Cartesian TCP/payload lower to z=0.394 m
target product yaw        = -90 deg
```

停止态逐状态验证覆盖 `1054` 个状态：空载接近、抓取下降、携物抬升/转运、
对齐下降、允许产品接触目标带面的 26 mm 末段、释放后回零全部通过环境碰撞、
R5 自碰撞、产品到 R5 和工作区检查。释放产品姿态为
`(0.005526,0.000522,-89.998600) deg`，轨道平行误差 `0.001400 deg`，
产品到 TCP 7D 变换最大误差 `6.11e-16`，R5 最终六关节全零，全部 target
未变。

第一次正式单臂命令在运动前被 preflight 拦截：干净场景启动时
`Main_Cell_Generator.sysCall_init()` 会隐藏检测产品，实际为 `0/30` shape
可见。该次没有机械臂动作。仅在当前运行态恢复产品 `30/30` 可见、未修改
Lua/场景文件/产品位姿后，默认 `50 deg/s`、APP `0.8 s` 正式 Good 成功：

```text
wall time                       = 56.06 s
release product yaw             = -89.998160 deg
track parallel error            = 0.001840 deg
grasp transform max error       = 1.1102230246251565e-16
max R5 home error               = 0.004661 deg
Runtime objects                 = none
```

独立 postflight 完整通过：仿真时间继续推进；R1-R4 全零，R5 最大回零误差
`0.004661 deg`；五臂环境/自碰撞均为 none；检测产品 `30/30` 可见、父节点
`/Parts`，除目标 Good 带外无环境碰撞；受保护 target 未变；场景仍为
`2875907` 字节、SHA-256
`0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657`。
完整自动化测试增加 4 项定向测试后为 `49/49` 通过。

随后从干净停止场景运行新 yaw 路线的五臂 Good `60 deg/s`、APP `0.4 s`：

```text
log                              = data/logs/five_arm_good_yaw_aligned_60_2026-07-20.json
status                           = finished
seven formal tasks               = 7/7
total wall time                  = 230.954 s
R5 wall/simulation time          = 45.936 / 34.500 s
R5 release product yaw           = -89.998158 deg
R5 track parallel error          = 0.001842 deg
R5 grasp transform max error     = 1.6653345369377348e-16
```

五臂独立 postflight 完整通过：仿真时间继续推进；五臂最大回零误差为 R5
`0.008217 deg`；所有 `R*_Runtime_*` 为空；R1-R5 环境和自碰撞均为 none；
产品 `30/30` 可见、父节点 `/Parts`，除目标 Good 带外无环境碰撞；场景指纹
和受保护 target 未变。2026-07-19 的 `237.246 s` 日志仍保留为旧
`-123 deg` 路线证据，新路线以本次日志为准。

下一步严格顺序：

1. 只有用户重新允许后才运行暂缓的五臂 defect `60/0.4`；默认值仍保持
   `50/0.8`；
2. 两条新速度回归完成后再决定是否改默认值，然后做组长调度/GUI 集成和
   真实首动交接测量；
3. 最后整理提交文件集，用户确认后才 commit/push。

## 19. 2026-07-20 Scheduler、GUI 与真实首动测量集成

本节晚于第 18 节。用户明确决定不继续 defect `60 deg/s`、APP `0.4 s`
加速回归，直接进入下一项调度/GUI 集成工作；默认值继续保持 `50/0.8`。

真实模块已经替换原 Mock 别名并通过统一接口贯通：

```text
OrderParser -> Scheduler -> IntegratedRobotExecutor -> RobotExecutor
run_demo.py --real
run_demo.py --real --headless --quality good|defect --output <json>
```

`Order` 新增 `expected_quality=OK/NG`，`TaskResult` 新增结构化 `metrics`。
当前真实 Scheduler 只生成已经校准的单个 A 型单件七工序链，并明确拒绝
多订单、B/C、数量大于 1 和急单插入。故障或 Task 失败会将所有依赖后续任务
标记为 failed，不伪造跨机械臂重分配。

GUI 保留 5 号成员原工业 HMI 布局，新增 R5 状态卡、Good/Defect 选择、REAL
模式标签和首动/handoff 日志；REAL 模式禁用 B/C 和 Mock demo，真实任务运行中
禁止 GUI reset。真实场景没有补料/自动复位，完成一个周期后必须重新加载干净
场景才能再次执行。

新增 `JointMotionMonitor` 使用独立 ZMQ 连接，在 Task 下发前记录六关节基线，
任一关节变化达到 `0.02 deg` 时记录墙钟和仿真首动时刻。新增字段：

```text
motion_timing.dispatch_to_first_motion_wall_s
motion_timing.task_call_to_first_motion_wall_s
motion_timing.first_motion_simulation_time_s
handoff_to_first_motion_simulation_s
task_end_simulation_time_s
```

原 `handoff_delay_simulation_s` 仍保留，但只代表下一 Task 函数调用时间，不能
作为真实机械臂交接证据。完整自动化测试由 `49/49` 增至 `55/55`，覆盖订单
质量字段、七工序/分支、真实场景限制、失败级联、相机信号和首动指标。

从原指纹干净停止场景运行：

```text
command = python3 run_demo.py --real --headless --quality good \
          --output data/logs/scheduled_good_first_motion_2026-07-20.json
speed / APP hold              = 50 deg/s / 0.8 s
status                        = finished
scheduled tasks               = 7/7
wall duration                 = 283.763 s
monitor-armed-to-motion wall  = 1.733-3.348 s
scheduler-to-first-motion wall= 2.182-3.800 s
first R1 command-to-motion    = 3.529 s
real simulation handoff       = 1.300-2.300 s
```

R5 结果保持新 yaw 路线：释放 yaw `-89.998160 deg`、轨道平行误差
`0.001840 deg`、产品到 TCP 变换误差 `1.11e-16`。独立 postflight 通过：
仿真继续推进，五臂最大回零误差为 R5 `0.004661 deg`，五臂环境/自碰撞均无，
Runtime 为空，检测产品 `30/30` 可见、父节点 `/Parts`，target 和场景指纹未变。

真实首动结果没有达到既定 `<=0.5 s` 仿真 handoff 和 R1 `<=1.0 s` 墙钟目标。
证据显示各控制器仍在 Task 入口进行场景/target preflight 和在线 IK/path 构建；
独立监测连接也有部分建连开销。后续必须在常驻 READY 阶段预连接监测器、
预加载/缓存 R2-R5 路径并将用户命令变为薄触发，不得通过暂停仿真、删减安全
检查或改写指标隐藏空档。

收尾状态：CoppeliaSim 已停止并关闭，23000 无监听，无控制进程；场景仍为
`2875907` 字节、SHA-256
`0e1c1b8ac6b0e9a7cdf1a49cc9abce85243fd5c03c5b38563d3e3cf3433af657`。
未修改 `.ttt`、受保护 HOME/APP/TCP 或传送带，未 commit/push。

## 20. 2026-07-20 Handoff 延迟优化与提交准备

本节晚于第 19 节。用户要求继续降低仿真 handoff 延迟并整理提交。

上一会话已完成 prepare_cycle() 路径预计算、continuous_stepping、
PersistentJointMotionMonitorFactory 和 prepared_mode 跳过静态校验，但实测
handoff 仍为 1.300-2.300 s 仿真时间。根因分析确认延迟主要来自每台机械臂
`initial_to_pick_app` 段（~8-10s 仿真时间），而非代码开销。

方案：在 enter_ready() 后、用户触发前，用 setJointTargetPosition + stepping
将 R2/R3 预定位到 pick APP（R4 不预定位：screw APP 在检测区，首次实测即与
R3 产品转运碰撞）。每个控制器新增 _pre_positioned_config、set_pre_positioned()
和跳过 initial approach 的逻辑。SimBridge 非线程安全，保留 _execution_lock。

干净场景实测 50 deg/s、APP 0.8 s Good 7/7 成功：

```text
log                          = data/logs/handoff_optimized_v3_2026-07-20.json
total wall time              = 247.957 s
status                       = finished
R1→R2 handoff                = 0.100 s  (优化前 1.300-2.300)
R2→R3 handoff                = 0.200 s
R3→R1端子 handoff             = 0.950 s  (R1 未预定位)
R1端子→R3转运 handoff         = 0.200 s
R3转运→R4 handoff             = 0.300 s
R4→R5 handoff                = 0.250 s
R5 release yaw               = -89.998160 deg
R5 parallel error            = 0.001840 deg
R5 grasp transform error     = 1.11e-16
max home error               = 0.004661 deg (R5)
all Runtime objects          = none
R1-R5 final collision        = none
inspection product            = 30/30 visible, parent /Parts
```

全部可预定位交接 ≤0.20s，远超 ≤0.5s 目标。62/62 测试通过。新增
`docs/ROBOT_CONTROL_USAGE.md` 使用说明；更新 TASK_PLAN.md。

提交排除：`pick_and_place.py`（RViz demo）、`PICK_AND_PLACE_RVIZ.md`、
`obstacles.yaml`（旧 RViz 障碍物）、MoveIt config 预存修改、中间试验日志。

CoppeliaSim 已停止，23000 无监听。场景指纹、Git target 和 `.ttt` 未修改，
等待用户确认后 commit/push。
