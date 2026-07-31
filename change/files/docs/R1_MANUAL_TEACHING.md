# R1 手动示教操作说明

这份文档用于手动教 R1 走一条安全路径。你的任务是在 CoppeliaSim 画面里观察机械臂姿态，我用脚本记录关节角，之后把这些 waypoint 接进正式 R1 路径。

## 当前原则

不要保存 `.ttt` 场景。

不要移动这些目标点或零件：

```text
/FiveCR5A_Cell/Targets
/FiveCR5A_Cell/Parts
R1_BOX_PICK_APP
R1_BOX_PICK_TCP
R1_BOX_PLACE_APP
R1_BOX_PLACE_TCP
R1_TERMINAL_PICK_APP
R1_TERMINAL_PICK_TCP
R1_TERMINAL_PLACE_APP
R1_TERMINAL_PLACE_TCP
```

只移动 R1 的六个关节：

```text
joint1 joint2 joint3 joint4 joint5 joint6
```

## 打开场景

在终端运行：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
/home/vboxuser/CoppeliaSim/coppeliaSim.sh \
  /home/vboxuser/桌面/cr5_assembly_team/scenes/compact_cell1ttt.ttt
```

打开后先保持仿真停止状态，不要点播放。

如果已经打开了，就不用重复打开。

## 看当前 R1 角度

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 scripts/r1_teach.py show
```

输出示例：

```text
R1 current joints:
  deg: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
  rad: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
```

一般你看 `deg` 就够了。

## 检查当前姿态是否碰撞

```bash
python3 scripts/r1_teach.py check
```

如果安全，会看到：

```text
collision check: OK
```

如果有碰撞，会看到：

```text
collision check: COLLISION / INVALID - ...
```

这种姿态不要记录成正式 waypoint。

## 推荐：按键移动模式

进入按键模式：

```bash
cd /home/vboxuser/桌面/cr5_assembly_team
python3 scripts/r1_teach.py drive
```

默认每按一次移动 5 度。

按键含义：

```text
1 2 3 4 5 6  = joint1 到 joint6 正方向转 5 度
q w e r t y  = joint1 到 joint6 反方向转 5 度
a            = 自动记录当前姿态
c            = 检查当前姿态是否碰撞
s            = 显示当前六个关节角度
u            = 撤销上一步移动
0            = 回到初始姿态 [0,0,0,0,0,0]
+            = 每次移动角度加 1 度
-            = 每次移动角度减 1 度
h            = 显示帮助
x 或 Esc     = 退出按键模式
```

注意：这里 `q` 不是退出。`q` 是 joint1 反方向转 5 度。退出用 `x` 或 `Esc`。

进入后你可以这样操作：

```text
按 1  -> joint1 +5 度
按 q  -> joint1 -5 度
按 2  -> joint2 +5 度
按 w  -> joint2 -5 度
按 a  -> 记录当前姿态
按 x  -> 退出
```

按键模式每次移动后都会自动做碰撞检查。

如果安全，会显示：

```text
collision check: OK
```

如果有碰撞，会显示碰撞信息，并且默认自动退回上一个姿态：

```text
collision check: COLLISION / INVALID - ...
collision detected; reverted to previous pose
```

这时你换另一个方向或换另一个关节继续试。

如果你看到的是 `R1 workspace violation`，这不是碰撞，而是超过了代码里给 R1 设定的工作区边界。探索路径时可以临时忽略工作区，但仍然检查真实碰撞：

```bash
python3 scripts/r1_teach.py drive --ignore-workspace
```

这个参数只影响示教脚本，不会修改正式 R1 工作区配置。

按 `a` 记录时，脚本会自动生成 waypoint 名字：

```text
initial_escape_1
initial_escape_2
initial_escape_3
...
```

记录文件仍然是：

```text
data/manual_waypoints/r1_teach_waypoints.json
```

## 小幅移动关节

下面是非交互命令模式。按键模式更方便；这些命令主要用于精确设置或回退。

每次建议只动 2 到 5 度。

joint2 往负方向动 5 度：

```bash
python3 scripts/r1_teach.py jog --joint 2 --deg -5
```

joint3 往正方向动 5 度：

```bash
python3 scripts/r1_teach.py jog --joint 3 --deg 5
```

joint4 往负方向动 5 度：

```bash
python3 scripts/r1_teach.py jog --joint 4 --deg -5
```

`jog` 之后脚本会自动做一次碰撞检查。

安全时会显示：

```text
collision check: OK
```

碰撞时会显示：

```text
collision check: COLLISION / INVALID - ...
```

## 碰撞后怎么退回

如果刚才这一步碰撞了，反向移动同一个关节即可。

比如你刚运行了：

```bash
python3 scripts/r1_teach.py jog --joint 2 --deg -5
```

结果碰撞，那就退回：

```bash
python3 scripts/r1_teach.py jog --joint 2 --deg 5
```

也可以一键回到初始姿态：

```bash
python3 scripts/r1_teach.py set --deg 0 0 0 0 0 0
```

## 直接设置六个关节角

如果你已经有一组角度，可以直接设置：

```bash
python3 scripts/r1_teach.py set --deg 0 -30 45 -80 90 20
```

设置后也会自动做碰撞检查。

## 记录一个安全姿态

当画面看起来合理，并且脚本显示：

```text
collision check: OK
```

就记录这个姿态：

```bash
python3 scripts/r1_teach.py record initial_escape_1
```

继续移动，继续记录：

```bash
python3 scripts/r1_teach.py record initial_escape_2
python3 scripts/r1_teach.py record initial_escape_3
```

记录文件会写到：

```text
data/manual_waypoints/r1_teach_waypoints.json
```

`record` 默认会先做碰撞检查。碰撞姿态不会被记录。

如果当前姿态只是工作区越界、没有真实碰撞，可以这样记录：

```bash
python3 scripts/r1_teach.py record initial_escape_1 --ignore-workspace
```

## 查看已记录 waypoint

```bash
python3 scripts/r1_teach.py list
```

## 这次我们要找什么路径

现在 R1 失败在第一段：

```text
initial_to_box_pick_app
```

也就是：

```text
初始姿态 -> 箱子抓取上方
```

你要帮我找的是几个中间安全姿态：

```text
初始姿态
-> initial_escape_1
-> initial_escape_2
-> initial_escape_3
-> 接近 R1_BOX_PICK_APP
```

不用一次找到完美路径。先记录 2 到 4 个看起来安全、脚本也显示 OK 的姿态就行。

## 碰撞会不会自己有反应

不要依赖画面反应。

原因是：我们现在是在仿真停止状态下直接改关节角，CoppeliaSim 通常不会像真实机械臂一样自动弹开、报警或停机。模型之间就算穿模，画面上也可能不明显。

所以判断标准是脚本输出，而不是肉眼：

```text
collision check: OK
```

才可以继续或记录。

```text
collision check: COLLISION / INVALID
```

就退回，不要记录。

正式程序运行时也会做碰撞检查。如果路径中间发生碰撞，程序会失败并报出碰撞对象，例如：

```text
self collision during initial_to_box_pick_app:
['/R1/R1T_top_rail', '/R1/Link2_respondable']
```

所以最终仍然以这个命令是否通过为准：

```bash
python3 robot_control/run_r1_task.py R1_BOX_PLACED
```
