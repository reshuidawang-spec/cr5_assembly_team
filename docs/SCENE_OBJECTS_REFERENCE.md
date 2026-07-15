# 场景对象与零件完整参考表

> 场景根路径：`/FiveCR5A_Cell`  
> 场景脚本：`scenes/main_cell_generator.lua`  
> 版本：Five CR5A Cell v1.0

---

## 1. 场景树总览

```text
/FiveCR5A_Cell
  ├─ Ground_Group
  │   └─ Ground
  ├─ Tables
  │   ├─ Damping_Table_Left
  │   ├─ Damping_Table_Right
  │   ├─ Left_RubberPad_1 ~ 8
  │   └─ Right_RubberPad_1 ~ 8
  ├─ RobotBases
  │   ├─ R1_Base
  │   ├─ R2_Base
  │   ├─ R3_Base
  │   ├─ R4_Base
  │   └─ R5_Base
  ├─ Areas
  │   ├─ Box_Supply_Area
  │   ├─ Terminal_Supply_Area
  │   ├─ PCB_Supply_Area
  │   ├─ Module_Supply_Area
  │   ├─ Assembly_Area
  │   ├─ Assembly_Fixture
  │   ├─ Inspection_Screw_Area
  │   └─ Inspection_Platform
  ├─ Parts
  │   ├─ Box_Blank
  │   │   ├─ Box_Blank_Bottom / Left_Wall / Right_Wall / Front_Wall / Back_Wall
  │   │   ├─ Box_Blank_Post_1 ~ 4
  │   │   └─ Box_Blank_CableGland_1 ~ 2
  │   ├─ PCB_Supply
  │   │   ├─ PCB_Supply_Board
  │   │   ├─ PCB_Supply_Hole_1 ~ 4
  │   │   ├─ PCB_Supply_Main_Chip / Small_Chip
  │   │   ├─ PCB_Supply_Connector_1 ~ 2
  │   │   └─ PCB_Supply_Capacitor_1 ~ 2
  │   ├─ Control_Module_Supply
  │   │   ├─ Control_Module_Supply_Body
  │   │   └─ Control_Module_Supply_Label
  │   ├─ Terminal_Block_Supply
  │   │   ├─ Terminal_Block_Supply_Body
  │   │   ├─ Terminal_Block_Supply_Slot_1 ~ 4
  │   │   └─ Terminal_Block_Supply_Main_ScrewHead
  │   ├─ Assembly_ControlBox_Product  ←─ 装配区产品模板
  │   │   ├─ Assembly_ControlBox_Product_Shell
  │   │   │   ├─ *_Shell_Bottom / Left_Wall / Right_Wall / Front_Wall / Back_Wall
  │   │   │   ├─ *_Shell_Post_1 ~ 4
  │   │   │   └─ *_Shell_CableGland_1 ~ 2
  │   │   ├─ Assembly_ControlBox_Product_PCB
  │   │   │   ├─ *_PCB_Board
  │   │   │   ├─ *_PCB_Hole_1 ~ 4 / Main_Chip / Small_Chip
  │   │   │   ├─ *_PCB_Connector_1 ~ 2
  │   │   │   └─ *_PCB_Capacitor_1 ~ 2
  │   │   ├─ Assembly_ControlBox_Product_Control_Module
  │   │   │   ├─ *_Control_Module_Body
  │   │   │   └─ *_Control_Module_Label
  │   │   └─ Assembly_ControlBox_Product_Terminal_Block
  │   │       ├─ *_Terminal_Block_Body
  │   │       ├─ *_Terminal_Block_Slot_1 ~ 4
  │   │       └─ *_Terminal_Block_Main_ScrewHead
  │   └─ Inspection_ControlBox_Product  ←─ 检测区产品模板（结构同上）
  │       └─ ... (同上，前缀为 Inspection_ControlBox_Product_)
  ├─ Conveyors
  │   ├─ Good_Conveyor
  │   │   ├─ Good_Conveyor_Frame / Belt_Black / Leg_1 ~ 4
  │   │   └─ (传送带上运行的产品)
  │   └─ Defect_Conveyor
  │       └─ Defect_Conveyor_Frame / Belt_Black / Leg_1 ~ 4
  ├─ Sensors
  │   └─ Fixed_Vision_Camera_Station
  │       ├─ Camera_Column_Base / Camera_Column
  │       ├─ Camera_Bracket_X / Camera_Bracket_Y
  │       ├─ Fixed_Camera_Body / Fixed_Camera_Lens
  │       └─ Camera_View_Area
  └─ Targets
      ├─ R1_Targets/ (R1_HOME_REF 等 9 个)
      ├─ R2_Targets/ (R2_HOME_REF 等 5 个)
      ├─ R3_Targets/ (R3_HOME_REF 等 9 个)
      ├─ R4_Targets/ (R4_HOME_REF 等 4 个)
      ├─ R5_Targets/ (R5_HOME_REF 等 7 个)
      └─ Sensor_Targets/ (CAMERA_INSPECTION_CENTER)
```

---

## 2. 基础结构对象（Ground、Tables、Bases）

### 2.1 地面组 `Ground_Group`

| 对象名 | 完整路径 | 类型 | 尺寸 (L×W×H) m | 位置 (x,y,z) | 颜色 |
|---|---|---|---|---|---|
| Ground | `/FiveCR5A_Cell/Ground_Group/Ground` | Cuboid | 5.6 × 4.3 × 0.02 | (-0.55, -0.30, -0.01) | (0.78, 0.78, 0.78) 灰 |

### 2.2 工作台 `Tables`

| 对象名 | 完整路径 | 类型 | 尺寸 | 位置 | 颜色 |
|---|---|---|---|---|---|
| Damping_Table_Left | `/FiveCR5A_Cell/Tables/Damping_Table_Left` | Cylinder | R=0.86, H=0.12 | (-1.20, 0.25, 0.06) | (0.60, 0.72, 0.38) 绿 |
| Damping_Table_Right | `/FiveCR5A_Cell/Tables/Damping_Table_Right` | Cylinder | R=0.86, H=0.12 | (0.55, -0.10, 0.06) | (0.60, 0.72, 0.38) 绿 |
| Left_RubberPad_1~8 | `/FiveCR5A_Cell/Tables/Left_RubberPad_N` | Cylinder | R=0.045, H=0.06 | 沿左工作台圆周均匀分布 (R=0.68) | (0.02, 0.02, 0.02) 黑 |
| Right_RubberPad_1~8 | `/FiveCR5A_Cell/Tables/Right_RubberPad_N` | Cylinder | R=0.045, H=0.06 | 沿右工作台圆周均匀分布 (R=0.68) | (0.02, 0.02, 0.02) 黑 |

### 2.3 机械臂基座 `RobotBases`

| 对象名 | 完整路径 | 类型 | 尺寸 | 位置 | 颜色 |
|---|---|---|---|---|---|
| R1_Base | `/FiveCR5A_Cell/RobotBases/R1_Base` | Cylinder | R=0.18, H=0.10 | (-1.47, 0.67, 0.17) | (0.55, 0.55, 0.55) 金属 |
| R2_Base | `/FiveCR5A_Cell/RobotBases/R2_Base` | Cylinder | R=0.18, H=0.10 | (-1.55, -0.15, 0.17) | (0.55, 0.55, 0.55) 金属 |
| R3_Base | `/FiveCR5A_Cell/RobotBases/R3_Base` | Cylinder | R=0.18, H=0.10 | (-0.55, 0.28, 0.17) | (0.55, 0.55, 0.55) 金属 |
| R4_Base | `/FiveCR5A_Cell/RobotBases/R4_Base` | Cylinder | R=0.18, H=0.10 | (0.58, 0.25, 0.17) | (0.55, 0.55, 0.55) 金属 |
| R5_Base | `/FiveCR5A_Cell/RobotBases/R5_Base` | Cylinder | R=0.18, H=0.10 | (0.15, -0.50, 0.17) | (0.55, 0.55, 0.55) 金属 |

---

## 3. 功能区域对象 `Areas`

所有区域高度统一：`H=0.035`，Z=工作台面 + H/2 = 0.1375。

| 对象名 | 完整路径 | 类型 | 尺寸 (L×W×H) | 中心位置 (x,y) | 颜色 | 用途 |
|---|---|---|---|---|---|---|
| Box_Supply_Area | `.../Areas/Box_Supply_Area` | Cuboid | 0.32×0.22×0.035 | (-1.80, 0.35) | (0.88, 0.84, 0.68) 米黄 | 箱体供料位 |
| Terminal_Supply_Area | `.../Areas/Terminal_Supply_Area` | Cuboid | 0.24×0.16×0.035 | (-1.90, 0.10) | (0.88, 0.84, 0.68) 米黄 | 端子排供料位 |
| PCB_Supply_Area | `.../Areas/PCB_Supply_Area` | Cuboid | 0.36×0.24×0.035 | (-1.28, -0.28) | (0.88, 0.84, 0.68) 米黄 | PCB 供料位 |
| Module_Supply_Area | `.../Areas/Module_Supply_Area` | Cuboid | 0.32×0.20×0.035 | (-0.85, -0.05) | (0.88, 0.84, 0.68) 米黄 | 控制模块供料位 |
| Assembly_Area | `.../Areas/Assembly_Area` | Cuboid | 0.50×0.35×0.035 | (-1.15, 0.20) | (0.90, 0.82, 0.55) 暖黄 | 装配区底板 |
| Assembly_Fixture | `.../Areas/Assembly_Fixture` | Cuboid | 0.42×0.28×0.060 | (-1.15, 0.20) | (0.55, 0.55, 0.55) 金属 | 装配夹具 |
| Inspection_Screw_Area | `.../Areas/Inspection_Screw_Area` | Cuboid | 0.52×0.34×0.035 | (0.15, 0.05) | (0.90, 0.82, 0.55) 暖黄 | 检测/锁付区底板 |
| Inspection_Platform | `.../Areas/Inspection_Platform` | Cuboid | 0.42×0.28×0.060 | (0.15, 0.05) | (0.55, 0.55, 0.55) 金属 | 检测平台 |

---

## 4. 供料零件 `Parts`（独立零件）

### 4.1 箱体毛坯 `Box_Blank`

**路径**：`/FiveCR5A_Cell/Parts/Box_Blank`  
**整体尺寸**：0.35 × 0.25 × 0.12 m（壁厚 0.012 m）  
**位置**：箱体供料区 (-1.80, 0.35)  

| 子对象 | 完整路径示例 | 类型 | 尺寸 (L×W×H) m | 颜色 |
|---|---|---|---|---|
| (Group) | `.../Parts/Box_Blank` | Dummy | — | — |
| Box_Blank_Bottom | `.../Box_Blank/Box_Blank_Bottom` | Cuboid | 0.35×0.25×0.012 | (0.62, 0.62, 0.62) 灰 |
| Box_Blank_Left_Wall | `.../Box_Blank/Box_Blank_Left_Wall` | Cuboid | 0.012×0.25×0.12 | (0.62, 0.62, 0.62) 灰 |
| Box_Blank_Right_Wall | `.../Box_Blank/Box_Blank_Right_Wall` | Cuboid | 0.012×0.25×0.12 | (0.62, 0.62, 0.62) 灰 |
| Box_Blank_Front_Wall | `.../Box_Blank/Box_Blank_Front_Wall` | Cuboid | 0.35×0.012×0.12 | (0.62, 0.62, 0.62) 灰 |
| Box_Blank_Back_Wall | `.../Box_Blank/Box_Blank_Back_Wall` | Cuboid | 0.35×0.012×0.12 | (0.62, 0.62, 0.62) 灰 |
| Box_Blank_Post_1~4 | `.../Box_Blank/Box_Blank_Post_N` | Cylinder | R=0.010, H=0.050 | (0.55, 0.55, 0.55) 金属 |
| Box_Blank_CableGland_1~2 | `.../Box_Blank/Box_Blank_CableGland_N` | Cylinder | R=0.018, H=0.035 | (0.02, 0.02, 0.02) 黑 |

**四角立柱位置**：(x±0.125, y±0.080)  
**电缆接头位置**：(x±0.08, y-0.137) — 前面板下方

### 4.2 PCB 供料 `PCB_Supply`

**路径**：`/FiveCR5A_Cell/Parts/PCB_Supply`  
**整体尺寸**：0.24 × 0.16 × 0.008 m  
**位置**：PCB 供料区 (-1.28, -0.28)

| 子对象 | 类型 | 尺寸 m | 颜色 |
|---|---|---|---|
| (Group) | Dummy | — | — |
| PCB_Supply_Board | Cuboid | 0.24×0.16×0.008 | (0.00, 0.45, 0.18) 绿 |
| PCB_Supply_Hole_1~4 | Cylinder | R=0.007, H=0.004 | (0.95, 0.70, 0.15) 金 |
| PCB_Supply_Main_Chip | Cuboid | 0.045×0.045×0.012 | (0.02, 0.02, 0.02) 黑 |
| PCB_Supply_Small_Chip | Cuboid | 0.030×0.025×0.010 | (0.02, 0.02, 0.02) 黑 |
| PCB_Supply_Connector_1~2 | Cuboid | 0.060×0.020×0.020 | (0.92, 0.92, 0.88) 白 |
| PCB_Supply_Capacitor_1~2 | Cylinder | R=0.008, H=0.025 | (0.05, 0.20, 0.80) 蓝 |

**安装孔位置**：(x±0.105, y±0.065)  
**主芯片位置**：(x-0.03, y)

### 4.3 控制模块供料 `Control_Module_Supply`

**路径**：`/FiveCR5A_Cell/Parts/Control_Module_Supply`  
**整体尺寸**：0.09 × 0.065 × 0.035 m  
**位置**：控制模块供料区 (-0.85, -0.05)

| 子对象 | 类型 | 尺寸 m | 颜色 |
|---|---|---|---|
| (Group) | Dummy | — | — |
| Control_Module_Supply_Body | Cuboid | 0.09×0.065×0.035 | (0.08, 0.12, 0.18) 深蓝灰 |
| Control_Module_Supply_Label | Cuboid | 0.002×0.035×0.020 | (0.90, 0.90, 0.82) 浅白 |

### 4.4 端子排供料 `Terminal_Block_Supply`

**路径**：`/FiveCR5A_Cell/Parts/Terminal_Block_Supply`  
**整体尺寸**：0.16 × 0.035 × 0.035 m  
**位置**：端子排供料区 (-1.90, 0.10)

| 子对象 | 类型 | 尺寸 m | 颜色 |
|---|---|---|---|
| (Group) | Dummy | — | — |
| Terminal_Block_Supply_Body | Cuboid | 0.16×0.035×0.035 | (0.92, 0.92, 0.88) 白 |
| Terminal_Block_Supply_Slot_1~4 | Cuboid | 0.020×0.004×0.014 | (0.02, 0.02, 0.02) 黑 |
| Terminal_Block_Supply_Main_ScrewHead | Cylinder | R=0.008, H=0.005 | (0.55, 0.55, 0.55) 金属 |

**4 个槽位**：从 x-0.060 到 x+0.060，间距 0.040

---

## 5. 产品模板 `Parts`（装配体）

### 5.1 装配区产品 `Assembly_ControlBox_Product`

**路径**：`/FiveCR5A_Cell/Parts/Assembly_ControlBox_Product`  
**位置**：装配区 (-1.15, 0.20)

产品由 4 个阶段子组件构成，通过 `cell_product_state` signal 控制显示：

| 阶段 | signal 值 | 可见子组件 |
|---|---|---|
| 0 | （初始）| 全部不可见 |
| 1 | `assembly_shell` | `_Shell`（箱体壳）|
| 2 | `assembly_pcb` | `_Shell` + `_PCB` |
| 3 | `assembly_module` | `_Shell` + `_PCB` + `_Control_Module` |
| 4 | `assembly_full` | `_Shell` + `_PCB` + `_Control_Module` + `_Terminal_Block` |

子组件命名前缀：`Assembly_ControlBox_Product_`

| 子组件 | 在父 Group 中的名称 | 内容（与供料零件结构一致）|
|---|---|---|
| 箱体壳 | `Assembly_ControlBox_Product_Shell` | Bottom + 4 Wall + 4 Post + 2 CableGland |
| PCB 板 | `Assembly_ControlBox_Product_PCB` | Board + 4 Hole + 2 Chip + 2 Connector + 2 Capacitor |
| 控制模块 | `Assembly_ControlBox_Product_Control_Module` | Body + Label |
| 端子排 | `Assembly_ControlBox_Product_Terminal_Block` | Body + 4 Slot + Main_ScrewHead |

**各组件 Z 高度**（相对箱体底面）：

| 组件 | Z 偏移 |
|---|---|
| 箱体底面 | 0 m |
| PCB | +0.064 m |
| 控制模块 | +0.0855 m |
| 端子排 | +0.0855 m |

### 5.2 检测区产品 `Inspection_ControlBox_Product`

**路径**：`/FiveCR5A_Cell/Parts/Inspection_ControlBox_Product`  
**位置**：检测区 (0.15, 0.05)  
**结构**：与 `Assembly_ControlBox_Product` 完全相同，前缀为 `Inspection_ControlBox_Product_`。  
**用途**：R3 将装配体转移到检测区后，此对象可见；R5 分拣时抓取此对象放到传送带。

---

## 6. 传送带对象 `Conveyors`

### 6.1 合格品传送带 `Good_Conveyor`

**路径**：`/FiveCR5A_Cell/Conveyors/Good_Conveyor`  
**方向**：Y 轴（纵向）  
**长度**：1.25 m，**宽度**：0.36 m

| 子对象 | 类型 | 尺寸 m | 颜色 |
|---|---|---|---|
| Good_Conveyor_Frame | Cuboid | 0.46×1.25×0.12 | (0.55, 0.55, 0.55) 金属 |
| Good_Conveyor_Belt_Black | Cuboid | 0.36×1.17×0.030 | (0.005, 0.005, 0.005) 黑 |
| Good_Conveyor_Leg_1~4 | Cuboid | 0.035×0.035×变长 | (0.55, 0.55, 0.55) 金属 |

**产品起点**：(0.65, -1.10, 0.27)  
**产品终点**：(0.65, -2.20, 0.27)  
**传送速度**：0.18 m/s

### 6.2 缺陷品传送带 `Defect_Conveyor`

**路径**：`/FiveCR5A_Cell/Conveyors/Defect_Conveyor`  
**方向**：X 轴（横向）  
**长度**：1.20 m，**宽度**：0.36 m

| 子对象 | 类型 | 尺寸 m | 颜色 |
|---|---|---|---|
| Defect_Conveyor_Frame | Cuboid | 1.20×0.46×0.12 | (0.55, 0.55, 0.55) 金属 |
| Defect_Conveyor_Belt_Black | Cuboid | 1.12×0.36×0.030 | (0.005, 0.005, 0.005) 黑 |
| Defect_Conveyor_Leg_1~4 | Cuboid | 0.035×0.035×变长 | (0.55, 0.55, 0.55) 金属 |

**产品起点**：(-0.35, -1.12, 0.27)  
**产品终点**：(-1.45, -1.12, 0.27)  
**传送速度**：0.18 m/s

---

## 7. 视觉传感器 `Sensors`

**路径**：`/FiveCR5A_Cell/Sensors/Fixed_Vision_Camera_Station`

| 对象名 | 类型 | 尺寸 m | 位置 (x,y,z) | 颜色 |
|---|---|---|---|---|
| Camera_Column_Base | Cylinder | R=0.055, H=0.08 | (-0.10, 0.55, 0.08) | (0.55, 0.55, 0.55) 金属 |
| Camera_Column | Cylinder | R=0.022, H=0.80 | (-0.10, 0.55, 0.48) | (0.55, 0.55, 0.55) 金属 |
| Camera_Bracket_X | Cuboid | 0.30×0.035×0.035 | (0.03, 0.55, 0.86) | (0.55, 0.55, 0.55) 金属 |
| Camera_Bracket_Y | Cuboid | 0.035×0.45×0.035 | (0.15, 0.33, 0.86) | (0.55, 0.55, 0.55) 金属 |
| Fixed_Camera_Body | Cuboid | 0.08×0.06×0.06 | (0.15, 0.15, 0.82) | (0.02, 0.02, 0.02) 黑 |
| Fixed_Camera_Lens | Cuboid | 0.035×0.035×0.035 | (0.15, 0.15, 0.775) | (0.005, 0.005, 0.005) 黑 |
| Camera_View_Area | Cuboid | 0.42×0.30×0.005 | (0.15, 0.05, ~0.218) | (0.75, 0.92, 1.00) 默认蓝 |

**Camera_View_Area 颜色随检测结果变化**：

| 状态 | RGB | signal |
|---|---|---|
| 默认（未检测）| (0.75, 0.92, 1.00) 浅蓝 | — |
| 合格 | (0.20, 0.90, 0.25) 绿 | `cell_product_state = camera_good` |
| 缺陷 | (0.95, 0.15, 0.10) 红 | `cell_product_state = camera_defect` |

---

## 8. 目标点 `Targets`

所有目标点均为 Dummy 对象，尺寸 0.025，颜色 (1.00, 0.72, 0.20) 黄。

### 8.1 R1 目标点（箱体 + 端子排）

**路径前缀**：`/FiveCR5A_Cell/Targets/R1_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| R1_HOME_REF | (-1.47, 0.67, 0.80) | R1 初始位 |
| R1_BOX_PICK_APP | (-1.80, 0.35, 0.55) | 箱体抓取接近 |
| R1_BOX_PICK_TCP | (-1.80, 0.35, 0.30) | 箱体抓取 |
| R1_BOX_PLACE_APP | (-1.15, 0.20, 0.55) | 箱体放置接近 |
| R1_BOX_PLACE_TCP | (-1.15, 0.20, 0.30) | 箱体放置 |
| R1_TERMINAL_PICK_APP | (-1.90, 0.10, 0.45) | 端子排抓取接近 |
| R1_TERMINAL_PICK_TCP | (-1.90, 0.10, 0.24) | 端子排抓取 |
| R1_TERMINAL_PLACE_APP | (-1.09, 0.13, 0.50) | 端子排安装接近 |
| R1_TERMINAL_PLACE_TCP | (-1.09, 0.13, 0.34) | 端子排安装 |

### 8.2 R2 目标点（PCB）

**路径前缀**：`/FiveCR5A_Cell/Targets/R2_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| R2_HOME_REF | (-1.55, -0.15, 0.80) | R2 初始位 |
| R2_PCB_PICK_APP | (-1.28, -0.28, 0.45) | PCB 抓取接近 |
| R2_PCB_PICK_TCP | (-1.28, -0.28, 0.22) | PCB 抓取 |
| R2_PCB_PLACE_APP | (-1.15, 0.20, 0.50) | PCB 安装接近 |
| R2_PCB_PLACE_TCP | (-1.15, 0.20, 0.29) | PCB 安装 |

### 8.3 R3 目标点（控制模块 + 产品转移）

**路径前缀**：`/FiveCR5A_Cell/Targets/R3_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| R3_HOME_REF | (-0.55, 0.28, 0.80) | R3 初始位 |
| R3_MODULE_PICK_APP | (-0.85, -0.05, 0.45) | 控制模块抓取接近 |
| R3_MODULE_PICK_TCP | (-0.85, -0.05, 0.24) | 控制模块抓取 |
| R3_MODULE_PLACE_APP | (-1.12, 0.22, 0.50) | 控制模块安装接近 |
| R3_MODULE_PLACE_TCP | (-1.12, 0.22, 0.34) | 控制模块安装 |
| R3_PRODUCT_PICK_APP | (-1.15, 0.20, 0.60) | 装配体抓取接近 |
| R3_PRODUCT_PICK_TCP | (-1.15, 0.20, 0.34) | 装配体抓取 |
| R3_PRODUCT_PLACE_INSPECTION_APP | (0.15, 0.05, 0.60) | 检测区放置接近 |
| R3_PRODUCT_PLACE_INSPECTION_TCP | (0.15, 0.05, 0.34) | 检测区放置 |

### 8.4 R4 目标点（螺钉锁付）

**路径前缀**：`/FiveCR5A_Cell/Targets/R4_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| R4_HOME_REF | (0.58, 0.25, 0.80) | R4 初始位 |
| R4_SCREW_APP | (0.21, -0.02, 0.55) | 螺钉接近 |
| R4_SCREW_TCP | (0.21, -0.02, 0.36) | 螺钉接触 |
| R4_SCREW_PRESS | (0.21, -0.02, 0.33) | 下压锁付 |

### 8.5 R5 目标点（分拣）

**路径前缀**：`/FiveCR5A_Cell/Targets/R5_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| R5_HOME_REF | (0.15, -0.50, 0.80) | R5 初始位 |
| R5_PRODUCT_PICK_APP | (0.15, 0.05, 0.60) | 检测区产品抓取接近 |
| R5_PRODUCT_PICK_TCP | (0.15, 0.05, 0.34) | 检测区产品抓取 |
| R5_GOOD_PLACE_APP | (0.65, -1.10, 0.62) | 合格品放置接近 |
| R5_GOOD_PLACE_TCP | (0.65, -1.10, 0.42) | 合格品放置 |
| R5_DEFECT_PLACE_APP | (-0.35, -1.12, 0.62) | 缺陷品放置接近 |
| R5_DEFECT_PLACE_TCP | (-0.35, -1.12, 0.42) | 缺陷品放置 |

### 8.6 相机检测目标点

**路径前缀**：`/FiveCR5A_Cell/Targets/Sensor_Targets/`

| 目标点 | 位置 (x, y, z) | 用途 |
|---|---|---|
| CAMERA_INSPECTION_CENTER | (0.15, 0.05, 0.55) | 检测中心参考 |

---

## 9. 机械臂末端 Tip 点

Tip 点挂在各机械臂 `Link6_visual`（或 `Link6`）下，为 Dummy 对象，尺寸 0.025。

| 机械臂 | Tip 名称 | 在机械臂内的挂载位置 |
|---|---|---|
| R1 | `R1_gripper_tip` | `/R1/.../Link6_visual/R1_gripper_tip` |
| R2 | `R2_gripper_tip` | `/R2/.../Link6_visual/R2_gripper_tip` |
| R3 | `R3_gripper_tip` | `/R3/.../Link6_visual/R3_gripper_tip` |
| R4 | `R4_tool_tip` | `/R4/.../Link6_visual/R4_tool_tip` |
| R5 | `R5_gripper_tip` | `/R5/.../Link6_visual/R5_gripper_tip` |

---

## 10. 全局颜色常量参考

| 颜色名 | RGB | 用途 |
|---|---|---|
| COLOR_GROUND | (0.78, 0.78, 0.78) | 地面 |
| COLOR_TABLE | (0.60, 0.72, 0.38) | 工作台（绿）|
| COLOR_METAL | (0.55, 0.55, 0.55) | 金属件 |
| COLOR_DARK | (0.02, 0.02, 0.02) | 深色件（橡胶垫等）|
| COLOR_BLACK | (0.005, 0.005, 0.005) | 黑色（传送带面、相机镜头）|
| COLOR_AREA | (0.88, 0.84, 0.68) | 供料区底板（米黄）|
| COLOR_BOX | (0.62, 0.62, 0.62) | 箱体外壳（灰）|
| COLOR_PCB | (0.00, 0.45, 0.18) | PCB 板（绿）|
| COLOR_MODULE | (0.08, 0.12, 0.18) | 控制模块（深蓝灰）|
| COLOR_TERMINAL | (0.92, 0.92, 0.88) | 端子排（白）|
| COLOR_CAMERA_VIEW | (0.75, 0.92, 1.00) | 检测视野（浅蓝）|
| COLOR_TARGET | (1.00, 0.72, 0.20) | 目标点 Dummy（黄）|

---

## 11. 关键尺寸常量

| 常量 | 值 | 含义 |
|---|---|---|
| TABLE_TOP_Z | 0.12 m | 工作台面高度 |
| AREA_H | 0.035 m | 区域底板厚度 |
| AREA_Z | 0.1375 m | 区域底板中心 Z |
| AREA_TOP_Z | 0.155 m | 区域底板顶面 Z |
| FIXTURE_H | 0.060 m | 夹具高度 |
| FIXTURE_Z | 0.215 m | 夹具中心 Z |
| FIXTURE_TOP_Z | 0.275 m | 夹具顶面 Z |
| ASSEMBLY_BOX_Z | 0.276 m | 装配区箱体底面 Z |
| INSPECTION_BOX_Z | 0.276 m | 检测区箱体底面 Z |
| SUPPLY_TOP_Z | 0.156 m | 供料零件底面 Z |
| PCB_Z_OFFSET | 0.064 m | PCB 距箱体底面高度 |
| MODULE_Z_OFFSET | 0.0855 m | 控制模块距箱体底面高度 |
| TERMINAL_Z_OFFSET | 0.0855 m | 端子排距箱体底面高度 |
| PRODUCT_ON_BELT_Z | 0.270 m | 传送带上产品 Z |
| ROBOT_Z_OFFSET | 0.02 m | 机械臂距基座 Z 偏移 |
| CONVEYOR_SPEED | 0.18 m/s | 传送带速度 |

---

> **文档维护**：场景搭建负责人  
> **对应脚本**：`scenes/main_cell_generator.lua`  
> **最后更新**：2026-07-15
