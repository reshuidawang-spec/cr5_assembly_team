# AGV 柔性换型 — Cart + A/B 产品搭建指南

在已有 CR5A 五臂场景基础上，添加 AGV 小车和 A/B 产品型号切换的搭建步骤。

## 新增功能概述

- **AGV 小车调度**：CartA/CartB 在供料位和等待位之间移动，由 ROS2 `/compact_cell/cmd` 控制
- **A/B 产品换型**：通过 `PRODUCT_A`/`PRODUCT_B` 命令切换生产型号，B 型号为蓝色配色
- **合格品传送带移位**：Good_Conveyor 左移 0.5m，R5_GOOD_PLACE 点位同步更新

## 架构

```
ROS2 Bridge (V3)
  ├─ /compact_cell/cmd →
  │   ├─ cell_product_state → Product Stage Controller (A/B切换 + 阶段显示)
  │   └─ cart_order        → Cart Order Controller (CartA/CartB 移动)
  ├─ /compact_cell/tool_cmd → Tool Action Controller
  └─ /compact_cell/status  ← 状态反馈
```

## 搭建步骤（在已有场景上添加）

### 1. 创建 B 型号产品

```bash
# 在 CoppeliaSim 中：
1. 新建 Dummy → Non-threaded child script
2. 粘贴 scenes/Product_B_Create.lua
3. ▶️ 运行仿真 → 看到 "B型号产品创建完成"
4. ⏹️ 停止仿真 → 禁用该脚本
```

B 产品在 `/FiveCR5A_Cell/PartsB/` 下：
- `Box_Blank_B` — 红色箱体（装配时开口朝上）
- `PCB_Supply_B` — 红橙色 PCB
- `Control_Module_Supply_B` — 深蓝色模块
- `Terminal_Block_Supply_B` — 橙黄色端子排
- `Assembly_ControlBox_Product_B` — 完整装配体
- `Inspection_ControlBox_Product_B` — 检测产品

### 2. 导入 AGV 小车

```bash
1. Modules → Importers → URDF importer...
2. 选择 models/FTF_AGV_fixed_v4/FTF_AGV_fixed_v4.urdf
3. 导入 2 次 → 改名 CartA、CartB
4. 拖动到场景中的大致位置
```

### 3. 创建小车目标点

```bash
1. 新建 Dummy → Non-threaded child script
2. 粘贴 scenes/Cart_Targets_Setup.lua
3. ▶️ 运行仿真
4. ⏹️ 停止 → 禁用
```

### 4. 挂载小车调度控制器（长期运行）

```bash
1. 新建 Dummy → Non-threaded child script
2. 粘贴 scenes/Cart_Order_Controller.lua
3. 保持启用
```

### 5. 更新 ROS2 桥接和产品控制器

确保 `/FiveCR5A_Cell` 上的 customization script 是最新版的：
- `scenes/ROS2_CompactCell_Bridge_V3_ColorCycle.lua` — 包含 PRODUCT_A/B 和 CART 命令
- `scenes/Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua` — 包含 A/B 显示切换

### 6. 验证

```bash
# ROS2 命令测试
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'PRODUCT_B'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'PRODUCT_A'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_A_SUPPLY'"
ros2 topic pub --once /compact_cell/cmd std_msgs/msg/String "data: 'CART_RESET'"
```

## 传送带和 R5 点位变更

| 对象 | 旧坐标 | 新坐标 | 说明 |
|------|--------|--------|------|
| Good_Conveyor (中心) | X=0.98 | **X=0.48** | 左移 0.5m |
| goodStart | (0.98, -1.06) | **(0.48, -1.06)** | 良品起点 |
| goodEnd | (0.98, -2.20) | **(0.48, -2.20)** | 良品终点 |
| R5_GOOD_PLACE | (0.85, -1.10) | **(0.35, -1.10)** | R5 放置位 |

## 新增脚本清单

| 脚本 | 类型 | 用途 |
|------|------|------|
| `Product_B_Create.lua` | 生成类（一次） | 从A复制B产品并差异化着色 |
| `Cart_Targets_Setup.lua` | 生成类（一次） | 创建4个小车目标点Dummy |
| `Cart_Order_Controller.lua` | 长期运行 | 信号驱动的小车移动控制 |

## 新增 ROS2 命令

| 命令 | Topic | 效果 |
|------|-------|------|
| `PRODUCT_A` | `/compact_cell/cmd` | 切换 A 型号 + 显示 A 供料件 |
| `PRODUCT_B` | `/compact_cell/cmd` | 切换 B 型号 + 显示 B 供料件 |
| `CART_A_SUPPLY` | `/compact_cell/cmd` | CartA→供料位, CartB→等待位 |
| `CART_B_SUPPLY` | `/compact_cell/cmd` | CartB→供料位, CartA→等待位 |
| `CART_RESET` | `/compact_cell/cmd` | 两车都回等待位 |
