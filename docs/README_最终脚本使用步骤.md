# 五台 CR5A 小型电控箱协同装配仿真最终脚本包

## 0. 最终脚本总数

从零搭建最终版一共需要 7 个主脚本：

1. Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua
2. Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
3. Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua
4. Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua
5. ROS2_CompactCell_Bridge_V3_ColorCycle.lua
6. ROS2_Joint_Jog_Controller_R1_R5.lua
7. Step03_Create_Process_Targets_60_CloserTables.lua

另外有 1 个可选脚本：

8. OneClick_Adjust_Tables_Closer.lua

它只用于修复当前已经搭好的旧场景，把右侧工作台调近但不重叠。
如果你从零使用新版 Step01 搭建，则不需要使用这个可选脚本。

---

# 一、从零搭建最终版流程

## Step 0：从 ROS2 终端启动 CoppeliaSim

不要直接双击 CoppeliaSim。需要在 Ubuntu 终端输入：

```bash
source /opt/ros/humble/setup.bash
```

如果有自己的工作空间，再输入：

```bash
source ~/dobot_ws/install/setup.bash
```

然后启动：

```bash
cd /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
./coppeliaSim.sh
```

---

## Step 1：准备五台机械臂

场景中需要有五台机械臂，根对象名称必须是：

```text
R1
R2
R3
R4
R5
```

如果名称不是这样，先在场景树中重命名。

---

## Step 2：运行 Step01，创建场景

使用脚本：

```text
Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua
```

作用：

```text
1. 创建两个圆形工作台，且距离更近但不重叠
2. 创建供料区、装配区、检测区、传送带、固定相机
3. 创建 60% 缩放后的箱体、PCB、控制模块、端子排
4. 自动摆放 R1~R5
5. 创建 /FiveCR5A_Cell 主场景结构
```

操作：

```text
1. 新建 Dummy：Step01_Create_Clean_Cell_60
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴 Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua
5. 运行一次
6. 运行成功后禁用或删除这个脚本
```

---

## Step 3：运行末端工具生成脚本

使用脚本：

```text
Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
```

作用：

```text
1. 给 R1 安装宽口可调夹爪
2. 给 R2 安装吸盘
3. 给 R3 安装宽口夹爪
4. 给 R4 安装电动螺丝刀
5. 给 R5 安装宽口夹爪
6. 创建各机械臂 tip 点
```

操作：

```text
1. 新建 Dummy：Create_EndEffectors
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 运行一次
6. 确认 R1/R3/R5 夹爪、R2 吸盘、R4 螺丝刀已经出现
7. 成功后禁用或删除这个脚本
```

生成的工具名称：

```text
R1T：R1 宽口夹爪
R2T：R2 吸盘
R3T：R3 宽口夹爪
R4T：R4 螺丝刀
R5T：R5 宽口夹爪
```

生成的 tip 点：

```text
R1_gripper_tip
R2_vacuum_tip
R3_gripper_tip
R4_tool_tip
R5_gripper_tip
```

---

## Step 4：添加运行脚本 1，末端工具控制

使用脚本：

```text
Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua
```

作用：

```text
1. 控制 R1/R3/R5 夹爪开合
2. 控制 R2 吸盘
3. 控制 R4 螺丝刀旋转
4. 控制工件 attach / detach
```

操作：

```text
1. 新建 Dummy：Step02B_Tool_Action_Controller
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 保持启用，不要禁用
```

---

## Step 5：添加运行脚本 2，产品阶段与三色循环

使用脚本：

```text
Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua
```

作用：

```text
1. 控制产品按阶段显示
2. 每次 RESET_CELL 后，电柜元件自动切换颜色
3. 三套颜色循环，方便演示多个电柜连续装配
```

三色循环：

```text
第 1 套：蓝色系
第 2 套：橙色系
第 3 套：紫绿色系
然后继续循环
```

操作：

```text
1. 新建 Dummy：Product_Stage_Controller_60
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 保持启用，不要禁用
```

---

## Step 6：添加运行脚本 3，ROS2 工艺命令桥接

使用脚本：

```text
ROS2_CompactCell_Bridge_V3_ColorCycle.lua
```

作用：

```text
1. 创建 ROS2 topic
2. 接收 ROS2 的字符串命令
3. 转换为 CoppeliaSim 内部 signal
4. 支持 RESET_CELL、工具动作、颜色切换等命令
```

操作：

```text
1. 新建 Dummy：ROS2_CompactCell_Bridge
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 保持启用，不要禁用
```

---

## Step 7：添加运行脚本 4，ROS2 关节点动控制

使用脚本：

```text
ROS2_Joint_Jog_Controller_R1_R5.lua
```

作用：

```text
1. 通过 ROS2 控制 R1~R5 的 J1~J6 关节
2. 可单关节点动
3. 可六关节一次性设置角度
4. 可回零
```

操作：

```text
1. 新建 Dummy：ROS2_Joint_Jog_Controller
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 保持启用，不要禁用
```

---

## Step 8：运行 Step03，创建 APP/TCP 目标点

使用脚本：

```text
Step03_Create_Process_Targets_60_CloserTables.lua
```

作用：

```text
1. 创建 R1~R5 的 APP/TCP 工艺目标点
2. 创建 R4_SCREW_APP / TCP / PRESS
3. 创建 CAMERA_INSPECTION_CENTER
4. 目标点与近距离工作台布局匹配
```

操作：

```text
1. 新建 Dummy：Step03_Create_Process_Targets_60
2. 添加 Non-threaded child script
3. 删除默认模板
4. 粘贴脚本
5. 运行一次
6. 成功后禁用或删除这个脚本
```

---

# 二、最终脚本启用状态

## 需要一直启用的 4 个脚本

```text
Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua
Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua
ROS2_CompactCell_Bridge_V3_ColorCycle.lua
ROS2_Joint_Jog_Controller_R1_R5.lua
```

## 运行一次后禁用的 3 个脚本

```text
Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua
Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua
Step03_Create_Process_Targets_60_CloserTables.lua
```

## 可选修复脚本

```text
OneClick_Adjust_Tables_Closer.lua
```

只用于当前已经搭好的旧场景。如果从零用新版 Step01，则不需要它。

---

# 三、ROS2 测试命令

## 1. 查看 topic

```bash
ros2 topic list
```

正常应看到：

```text
/compact_cell/cmd
/compact_cell/tool_cmd
/compact_cell/status
/compact_cell/joint_cmd
/compact_cell/joint_status
```

---

## 2. 监听状态

```bash
ros2 topic echo /compact_cell/status
```

另开终端监听关节状态：

```bash
ros2 topic echo /compact_cell/joint_status
```

---

## 3. 重置场景并自动换色

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'RESET_CELL'}" --once
```

每执行一次 `RESET_CELL`，电柜元件会切换到下一套颜色。

---

## 4. 手动切换颜色

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'COLOR_NEXT'}" --once
```

指定颜色：

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'COLOR_1'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'COLOR_2'}" --once
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'COLOR_3'}" --once
```

---

## 5. R1 夹爪测试

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_OPEN'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_CLOSE_BOX'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_GRIPPER_CLOSE_TERMINAL'}" --once
```

---

## 6. R1 抓箱体

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_ATTACH_BOX'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R1_RELEASE_BOX_ASSEMBLY'}" --once
```

---

## 7. R2 吸 PCB

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R2_ATTACH_PCB'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R2_RELEASE_PCB_ASSEMBLY'}" --once
```

---

## 8. R3 安装模块与搬运装配体

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_ATTACH_MODULE'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_RELEASE_MODULE_ASSEMBLY'}" --once
```

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_ATTACH_ASSEMBLY_PRODUCT'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R3_RELEASE_PRODUCT_INSPECTION'}" --once
```

---

## 9. R4 螺丝刀

```bash
ros2 topic pub /compact_cell/cmd std_msgs/msg/String "{data: 'SHOW_INSPECTION_FULL'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R4_SCREW_START'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R4_SCREW_STOP'}" --once
```

---

## 10. R5 分拣

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_ATTACH_INSPECTION_PRODUCT'}" --once
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_RELEASE_GOOD'}" --once
```

缺陷品：

```bash
ros2 topic pub /compact_cell/tool_cmd std_msgs/msg/String "{data: 'R5_RELEASE_DEFECT'}" --once
```

---

## 11. 关节点动

R1 的 J1 正向转 10 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 +10'}" --once
```

R1 的 J1 反向转 10 度：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 J1 -10'}" --once
```

R1 六关节一次性设置：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 SET 0 20 -30 0 45 0'}" --once
```

R1 回零：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'R1 HOME'}" --once
```

全部机械臂回零：

```bash
ros2 topic pub /compact_cell/joint_cmd std_msgs/msg/String "{data: 'ALL HOME'}" --once
```

---

# 四、目标点目录

所有目标点位于：

```text
/FiveCR5A_Cell/Targets
```

分组：

```text
/FiveCR5A_Cell/Targets/R1_Targets
/FiveCR5A_Cell/Targets/R2_Targets
/FiveCR5A_Cell/Targets/R3_Targets
/FiveCR5A_Cell/Targets/R4_Targets
/FiveCR5A_Cell/Targets/R5_Targets
/FiveCR5A_Cell/Targets/Sensor_Targets
```

---

# 五、保存最终场景

建议保存为：

```text
Five_CR5A_Cell_Final_CloserTables_ColorCycle_ROS2Joint.ttt
```

或中文名：

```text
五台CR5A电控箱装配场景_近距离工作台_三色循环_ROS2关节控制版.ttt
```

保存前确认：

```text
1. 两个工作台不重叠，距离也不过远
2. 颜色循环正常
3. 生成类脚本已禁用
4. 运行类脚本已启用
5. ROS2 topic 正常出现
6. 工具动作正常
7. 关节点动正常
```
