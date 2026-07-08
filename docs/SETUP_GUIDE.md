# CR5 Assembly Team 环境搭建指南

> 所有成员按本文档操作，10 分钟跑通场景。

---

## 1. 获取仓库

```bash
git clone https://github.com/reshuidawang-spec/cr5_assembly_team.git ~/cr5_assembly_team
```

---

## 2. 安装 CoppeliaSim（如果需要）

参考仓库内 `docs/` 目录，或打开以下链接下载 Edu V4.10.0：

```
https://www.coppeliarobotics.com/downloads
```

选择 **Ubuntu 22.04** 版本，解压到 `/opt/coppeliasim/`。

```bash
# 解压
sudo tar -xf CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04.tar.xz -C /opt/
sudo ln -s /opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04 /opt/coppeliasim

# 启动
/opt/coppeliasim/coppeliaSim.sh
```

---

## 3. 打开场景

1. 启动 CoppeliaSim
2. **File → Open scene...**
3. 选 `~/cr5_assembly_team/scenes/compact_cell.ttt`
4. 点 ▶️ 启动仿真

场景自动运行：
- 箱体在工作台上
- 传送带运送成品/缺陷品
- 所有点位（P_*）已就位

---

## 4. 导入机械臂模型

场景中的环境（工作台、传送带、点位）已生成，机械臂模型需手动导入：

1. **Modules → Importers → URDF importer...**
2. 选 `~/cr5_assembly_team/models/cr5_robot_fixed.urdf`（4 次）
3. 分别改名 R1、R2、R3、R4
4. 放到场景里对应的基座上

| 机械臂 | 基座位置 | 
|--------|---------|
| R1 | 左侧圆台上方 (-1.75, 0.95) |
| R2 | 左侧圆台下方 (-1.75, -0.30) |
| R3 | 右侧圆台上方 (1.05, 0.40) |
| R4 | 右侧圆台下方 (1.30, -0.65) |

### 4.1 配置 R3 末端相机

R3 的末端会自动挂载视觉相机，需要将 R3 的末端关节改名为 `tip`：

1. 场景树里展开 **R3**
2. 找到最末端的 `Link6`
3. 右键 → **Rename** → 输入 `tip`
4. 下次启动仿真时相机自动挂到 `tip` 下面

---

## 5. 文件结构

```
cr5_assembly_team/
├── scenes/
│   └── compact_cell.ttt          ← 场景文件
├── models/
│   ├── cr5_robot_fixed.urdf      ← CR5 模型
│   ├── cr12_robot_fixed.urdf     ← CR12 模型
│   ├── cr5_meshes/               ← CR5 网格
│   └── cr12_meshes/              ← CR12 网格
├── sim_bridge/
│   ├── build_scene.lua           ← 场景生成脚本
│   └── scene_objects.py          ← 点位映射
├── configs/
│   ├── points.yaml               ← 13个标准点位
│   ├── robots.yaml               ← 机械臂配置
│   └── scheduler.yaml            ← 调度参数
├── interfaces/                   ← 接口定义
├── scheduler/                    ← 调度模块
├── robot_control/                ← 控制模块
└── app/                          ← GUI 集成
```

---

## 6. 常见问题

**Q: 场景打开后看不到环境？**
点 ▶️ 启动仿真，`build_scene.lua` 的脚本会在仿真开始后生成环境。

**Q: 机械臂模型导入失败？**
确认 URDF 文件路径没有中文，且 mesh 文件在同一目录下。

**Q: 怎么获取最新版本？**
```bash
cd ~/cr5_assembly_team
git pull origin main
```
