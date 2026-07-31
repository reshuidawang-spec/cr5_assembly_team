# CR5 RViz 初步抓放路径规划说明

日期：2026-07-15
代码文件：`robot_control/pick_and_place.py`

## 必读恢复记录

本文档同时是 RViz 参考和当前 CoppeliaSim 动作入口。继续工作前必须同时阅读：

```text
/home/vboxuser/桌面/cr5_assembly_team/docs/CR5_SESSION_MEMORY_2026-07-15.md
/home/vboxuser/桌面/cr5_assembly_team/PROJECT_CONTEXT.md
/home/vboxuser/桌面/workspace/TASK_PLAN.md
```

用户最新决定是：当前先将运动清楚表示出来，可以使用隔空夹取和
箱体视觉 attach；暂不修改 Git 箱体 target，暂不将物理夹持作为视觉演示的
阻塞条件。机械臂和夹爪对环境的碰撞仍必须检查。

正式团队接口已迁移到：

```text
robot_control/robot_executor.py
robot_control/r1_motion.py
robot_control/plans/r1_complete_cycle_plan.json
robot_control/run_r1_task.py
```

`RobotExecutor` 实现既有 `IRobotExecutor`，接收 `Task` 并返回
`TaskResult`。完整使用方法见 `robot_control/R1_EXECUTOR.md`；本文件后面的
MoveIt/RViz 内容继续作为历史参考，不是五臂正式入口。

## 2026-07-17 R1 完整视觉循环

当前最新入口已经从单独箱体 replay 更新为：

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_complete_cycle.py
```

运行命令：

```bash
cd /home/vboxuser/桌面/workspace
python3 robot_control/demo_r1_complete_cycle.py
```

该脚本完成箱体抓放、端子排抓放、退出装配共享区和 R1 回零。用户已确认
同一完整视觉流程累计 `10/10` 成功。箱体执行姿态为 `(180,0,-90)`，端子排
执行姿态为 `(180,0,-180)`；开放空间默认 `50 deg/s`，精细下降不超过
`24 deg/s`。全部 Git target 保持不变。

重复运行前必须停止仿真并重新加载保存场景。完整说明见：

```text
/home/vboxuser/桌面/workspace/robot_control/R1_COMPLETE_CYCLE.md
```

## CoppeliaSim R1 箱体视觉演示（历史单任务基线）

场景：

```text
/home/vboxuser/桌面/cr5_assembly_team/scenes/five_cr5a_cell.ttt
```

演示脚本位于 workspace，不在本目录：

```text
/home/vboxuser/桌面/workspace/robot_control/demo_r1_box_motion.py
```

重启并打开上述场景后，执行：

```bash
cd /home/vboxuser/桌面/workspace
python3 robot_control/demo_r1_box_motion.py \
  --speed-deg-s 16 \
  --hold-seconds 2.0
```

默认是已验证端点 replay，不再为每次展示重新耗时搜索 IK。只有确实需要
重新生成笛卡尔路径时才使用：

```bash
python3 robot_control/demo_r1_box_motion.py --replan
```

演示完整执行：

```text
初始零位
-> R1_BOX_PICK_APP
-> 下降 R1_BOX_PICK_TCP
-> Robotiq 关闭 + 箱体视觉 attach
-> 抬升 R1_BOX_PICK_APP
-> 直接转移 R1_BOX_PLACE_APP
-> 下降 R1_BOX_PLACE_TCP
-> Robotiq 打开 + detach
```

完整成功日志：

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

实际终点：

```text
R1 TCP       = (-1.15030, 0.20041, 0.30009)
Box_Blank    = (-1.15030, 0.20028, 0.21597)
max q error  = 0.0035 deg
```

重要限制：

1. Git 四个箱体 target 的位置和 `{0,0,0}` 姿态都没有修改。
2. 执行层临时使用固定竖直姿态 `(180°,0°,-90°)`。
3. Robotiq 实际与中心抓取箱体相距 `36.695 mm`，所以当前是视觉 attach。
4. 用 60 mm 视觉偏移表示供料台和装配夹具的高度差。
5. attach 期间只允许夹爪形状接触已附着箱体；其他环境碰撞仍中止。
6. 成功演示当时的旧版脚本在结尾请求 pause，使 Coppelia 卡在 pause 过渡；
   脚本已改为 `client.setStepping(False)`，但修正版还未重跑。
7. 卡住的进程已关闭并重新打开保存场景；当前仿真停止、API 响应、
   R1 六关节全零、箱体在供料位。下次可直接验证修正后的结束方式。

箱体单任务 replay 已被上面的完整 R1 循环覆盖，保留本节只用于历史对照。

## 今日工作总结

当前默认演示严格复用逐步验证成功的动作：到 A 点正上方停留，保持姿态竖直下降抓取，一次性抬升并直线移动到 B 点正上方，再竖直下降放下。该版本只使用 RViz / MoveIt 仿真。

主要调整如下：

1. 放弃了全程分段 RRT* 执行的默认方式。
   旧版本每一段都单独用 RRT* 规划，A 到 B 容易在关节空间里绕一大圈，视觉上不直观。

2. 默认改为确定性 Cartesian 动作链。
   旧脚本随机采样 A 点 IK，可能选到等价但会让腕部转一圈的关节分支。现在若当前工具轴朝下，就直接从当前状态开始；冷启动姿态不合适时，只先用 OMPL 到固定 HOME，之后不再随机采样 IK：

   ```text
   initial_home
   -> approach_A
   -> A 上方停留
   -> descend_A
   -> close gripper
   -> lift_A_and_transfer_A_to_B（同一个两-waypoint 请求）
   -> B 上方停留
   -> descend_B
   -> open gripper
   -> 在 B 点打开夹爪并停止
   ```

3. HOME 仅作为冷启动姿态修正。
   当前姿态已经适合抓取时不会额外回 HOME；工具轴倾斜超过 `30°` 时才先到固定 HOME。

4. 去除抓取前转圈和 A 到 B 绕行。
   HOME 到 A 上方、A 点下降、抬升加 A 到 B、B 点下降均使用固定姿态 `compute_cartesian_path`。旧随机入口仅在显式传入 `--sampled-approach` 时启用。

5. 修正“抓取动作奇怪”的问题。
   脚本会在 A 点正上方停留，再下降到 A 点短停，然后闭合夹爪；B 点也同理，先在 B 上方停留，再下降放下。

6. 优化执行卡顿和速度。
   对 MoveIt 输出的轨迹做执行前处理：
   - 少量平滑；
   - 按关节空间弧长重采样；
   - 自动补速度；
   - `lift_A` 和 `transfer_A_to_B` 在一次 Cartesian 请求中生成；
   - 平滑后的实际控制点再次逐点做碰撞检查；
   - 搬运阶段将物块作为附着碰撞体检查；
   - 默认参数改为演示速度优先。

7. 验证结果。
   最新实际执行日志显示完整成功：

   ```text
   Executing initial_home
   Holding at initial point for 0.3s
   Executing approach_A
   Holding above A for 1.0s
   Executing descend_A
   Closing gripper at A
   Executing lift_A_and_transfer_A_to_B
   Holding above B for 1.0s
   Executing descend_B
   Opening gripper at B
   Direct Cartesian pick-and-place complete; block released at B
   Pick-and-place finished successfully
   ```

   冷启动实测四段 Cartesian 的 `fraction` 均为 `1.000`，默认停在 B 放置点。

## 运行前准备

打开一个终端，启动 RViz / MoveIt：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team

source /opt/ros/humble/setup.bash
source ./install/setup.bash

ros2 launch cr5_moveit demo.launch.py
```

等待 RViz 打开，并确认左侧 MotionPlanning 面板和 CR5 模型正常显示。

## 运行抓放演示

另开一个终端，执行：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team

source /opt/ros/humble/setup.bash
source ./install/setup.bash

python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62
```

默认 A 点和 B 点为：

```text
A = (0.40, -0.25, 0.50)
B = (0.35,  0.30, 0.50)
```

其中坐标表示 `gripper_base` 在 `dummy_link` 坐标系下的位置。

## 测试不同 A/B 点

可以通过 `--pick` 和 `--place` 指定点位：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --pick 0.42,-0.18,0.50 \
  --place 0.30,0.25,0.50
```

只验证规划、不实际执行：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-only \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --pick 0.42,-0.18,0.50 \
  --place 0.30,0.25,0.50
```

## 常用参数

更快一些：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --joint-speed 0.80 \
  --min-point-time 0.04 \
  --max-joint-step 0.10
```

更稳、更顺一些：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --joint-speed 0.55 \
  --min-point-time 0.06 \
  --max-joint-step 0.07
```

不先回初始点：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --skip-initial-home
```

仅用于对比旧随机入口模式：

```bash
python3 ./robot_control/pick_and_place.py \
  --plan-time 3 \
  --attempts 2 \
  --hover-z 0.62 \
  --sampled-approach \
  --entry-planners RRTConnect,RRTstar \
  --entry-samples 3 \
  --optimize-approach
```

## 参数说明

| 参数 | 作用 | 默认值 |
| --- | --- | --- |
| `--plan-time` | 单次 OMPL 规划时间 | `3` 推荐 |
| `--attempts` | 单次规划尝试次数 | `2` 推荐 |
| `--hover-z` | A/B 上方悬停高度 | `0.62` |
| `--hold-seconds` | A/B 上方停留时间 | `1.0` |
| `--settle-seconds` | A/B 点下降后的短停时间 | `0.25` |
| `--initial-hold-seconds` | 初始点停留时间 | `0.35` |
| `--joint-speed` | 执行轨迹的关节速度尺度 | `0.60` |
| `--min-point-time` | 轨迹点之间最小时间 | `0.05` |
| `--smooth-passes` | 执行前平滑次数 | `1` |
| `--max-joint-step` | 重采样最大关节步长 | `0.08` |
| `--max-tool-tilt-deg` | 允许直接抓取的最大工具轴倾角 | `30.0` |
| `--sampled-approach` | 启用旧随机 IK 入口作对比 | 默认关闭 |
| `--retreat-after-place` | 放置后竖直撤离 B | 默认关闭 |
| `--skip-initial-home` | 当前姿态不合适时也禁止回 HOME | 默认关闭 |
| `--plan-only` | 只规划验证，不执行 | 默认执行 |

## 排错

如果出现：

```text
can't open ... No such file or directory
```

通常是路径写错。请先进入仓库目录再执行：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 ./robot_control/pick_and_place.py
```

注意路径是中文 `桌面`，不是 `Desktop`。

如果提示 MoveIt 服务不可用，说明没有启动 RViz / MoveIt，先运行：

```bash
ros2 launch cr5_moveit demo.launch.py
```

如果某组 A/B 点规划失败，可以：

1. 提高悬停高度：

   ```bash
   --hover-z 0.66
   ```

2. 用 `--plan-only` 确认四个 Cartesian 段的 `fraction=1.000`。

3. 提高 `hover-z` 后仍失败，再换一个略近的 A/B 点测试。不要先启用旧随机入口。

## 当前限制

1. 这是 RViz / MoveIt 初步仿真，不是真实机械臂控制。
2. 小物块通过 RViz Marker 模拟跟随夹爪，不做真实物理抓取。
3. 默认 A 到 B 搬运段是固定姿态 Cartesian 直线；任一段不能完整达到 `fraction=1.000` 时不执行。
4. RViz/FakeSystem 的刷新和控制器模拟本身会带来少量视觉不连续，这和真实控制器的运动效果不完全等价。
