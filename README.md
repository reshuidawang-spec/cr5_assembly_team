# 工序自适，群臂协同
## 面向多工艺柔性产线的多机械臂自主调度与效能优化系统

> CR5 Assembly Team — 江科大学生参赛项目仓库  
> 当前主线统一为：**五台 DOBOT CR5A + 小型电控箱装配 + 固定相机检测 + R4 锁付 + R5 良品/缺陷品分拣**。

---

## 1. 当前项目状态

仓库当前已经形成一条可直接运行的纯仿真闭环：

| 部分 | 当前内容 | 状态 |
|---|---|---|
| 五臂 CoppeliaSim 场景 | 五台 CR5A、供料区、装配夹具、检测/锁付平台、固定相机、双传送带、末端工具、工件和已验证关节路径 | 当前场景与五套运动计划已绑定并通过整轮验收 |
| Python 调度与演示程序 | 订单解析、五臂细粒度工序链、区域互斥、动态调度、统一编排器、CoppeliaSim 执行器和界面 | START 可自动启动场景，完成 8 道任务并回传订单、任务、资源、质量和 KPI 状态 |

当前执行边界如下：

- 自动逆运动学与轨迹规划；
- 任意新场景或新点位的在线路径生成；
- 除“运行中的 A 型批次插入一台 B 型急单”外的任意在线订单变更；
- 任何物理机械臂连接或控制。

当前主流程使用当前场景对应的离线捕获路径、运行时碰撞检查和确定性步进，只控制 CoppeliaSim 内的模型，不连接真实机械臂。每批可预先加入 1～20 个产品单元；系统会为每个成品复制独立场景对象，并在上一单进入 R4/R5 后段后，让 R1/R2/R3 立即开始下一单，实现前后段重叠流水。

---

## 2. 统一工艺流程

```text
R1 抓取箱体并放入装配夹具
    ↓
R2 吸取 PCB 并装入箱体
    ↓
R1 安装端子排
    ↓
R3 安装控制模块
    ↓
R3 将完整装配体转移到检测/锁付平台
    ↓
固定相机检测并产生 OK / NG 结果
    ↓
R4 完成端子螺钉锁付
    ↓
R5 根据先前检测结果分拣到合格品或缺陷品传送带
```

### 机械臂分工

| 资源 | 任务 | 末端工具 |
|---|---|---|
| R1 | 箱体上料、端子排安装 | 宽口夹爪 |
| R2 | PCB 安装 | 吸盘 |
| R3 | 控制模块安装、装配体转移 | 宽口夹爪 |
| CAMERA | 固定视觉检测，返回 OK/NG | 固定相机 |
| R4 | 端子螺钉锁付 | 电动螺丝刀 |
| R5 | 合格品/缺陷品分拣 | 宽口夹爪 |

调度逻辑中，检测结果会先被记录；只有 R4 锁付完成后，系统才动态生成一条 R5 分拣任务。这样可以避免同一产品同时生成良品和缺陷品两条分拣任务。

---

## 3. 已提交的核心内容

### 3.1 CoppeliaSim 场景与脚本

`scenes/` 目录包含：

- `Step01_Create_Clean_Cell_60_CloserTables_ColorReady.lua`：生成五臂装配单元；
- `Create_Direct_Visible_EndEffectors_R1R3R5Wide_ConnectedJaw_R4fixed.lua`：创建夹爪、吸盘和螺丝刀；
- `Step03_Create_Process_Targets_60_CloserTables.lua`：创建 APP/TCP 工艺目标点；
- `Product_Stage_Controller_60_ColorCycle_V4_CloserTables.lua`：控制产品装配阶段显示；
- `Step02B_Tool_Action_Controller_V6_R1R3R5ConnectedJaw.lua`：控制末端工具与工件绑定/释放；
- `ROS2_CompactCell_Bridge_V3_ColorCycle.lua`：工艺命令 ROS2 桥接；
- `ROS2_Joint_Jog_Controller_R1_R5.lua`：R1～R5 关节点动；
- `compact_cell.ttt`：当前五臂并行协同流程使用的最新版场景文件；
- `configs/scene_contract.yaml`：场景哈希、对象数量和内嵌脚本版本基线。

完整搭建和使用方法见：

- [五台 CR5A 场景完整流程](docs/Five_CR5A_Cell_Full_Process_ROS2_Joint_Guide.md)
- [快速开始](docs/QUICK_START.md)

### 3.2 调度模块

当前五臂调度配置和代码主要位于：

```text
configs/
├── assembly_components.yaml
├── points.yaml
├── product_types.yaml
├── robots.yaml
├── scene_contract.yaml
└── scheduler.yaml

scheduler/
├── assembly_process.py
├── scheduler.py
├── task_generator.py
└── experiment.py
```

调度模块已经按五臂场景细化任务、机械臂资源和共享区域。说明文档见：

- [五臂场景调度对齐说明](docs/FIVE_CR5A_SCHEDULER_ALIGNMENT.md)
- [装配流程优化说明](docs/ASSEMBLY_PROCESS_OPTIMIZATION.md)
- [四种方案与故障对比](docs/FOUR_SCHEME_FAULT_COMPARISON.md)
- [4号调度模块方案说明](docs/4号调度模块方案说明.md)

### 3.3 一体化 GUI、仿真运动与测试

默认 GUI 是“订单 + 调度 + 五臂 CoppeliaSim 执行 + 数据看板”的一体化软件。主窗口包含两个页签：`仿真执行` 自动启动/连接当前场景，校验场景指纹和 R1～R5 五套路径，并实时展示订单进度、任务状态、六个资源状态、质量结果、日志与实际 KPI；`调度分析` 对当前订单队列运行 Baseline 串行方案和 Proposed 动态并行方案，展示总完成时间、平均等待、加权延期、冲突、并行效率及推荐任务时间线。两页共用正式订单解析器、产品配置和调度实验模型。`orchestration/` 中的统一编排器负责“生成任务 → 调度 → 恰好一次派发 → 完成回传 → 动态生成锁付/分拣任务”，并保存执行结果供 KPI 和统一导出使用。

B 型采用预集成 PCB/模块工艺：与 A 型相比跳过 R2 独立 PCB 安装，R2 在 B 型前段加工期间保持安全等待；R3 执行一体化模块安装，R4执行更长的加强锁付。该差异同时反映在任务 DAG、五臂实际运动和界面任务名称中，不增加新的示教点或轨迹。

指定不良品完成R5放置后，会从不良品传送带右侧入口沿X轴平滑输送约0.45米并停在带面中部，以直观表现检测分流后的物流过程。

R5在抓取整柜和中间转运时使用更密集的2°关节插值，保持原示教点及安全等待/放料路线不变，减少抓取下降和长距离转运中的可见顿挫。

R5抓取整柜后会先反向执行取件下降段，使末端从检测台上方明确抬升；随后退回R5安全等待位，再反向复用良品或不良品的回程轨迹进入放料位。放料后的空载回程与进场轨迹完全对称，从而避免中段下沉观感，同时不新增未经验证的示教圆弧。

正常完成后软件保留 CoppeliaSim 最终场景供观察；连接、准备或执行失败时会停止仿真并关闭由当前软件启动的 CoppeliaSim。界面运行记录和协调子进程的完整输出保存在 `log/runtime/`，用于追溯具体订单、工序、机械臂和异常原因。

```bash
python3 run_demo.py                 # 一体化 GUI；自动启动/连接 CoppeliaSim 并执行五臂运动
python3 run_demo.py --mock          # 完全离线 GUI
python3 run_demo.py --headless      # 无界面调度闭环 + Mock 执行
python3 run_demo.py --scene-check   # 连接已打开的场景并只读检查契约
python3 run_demo.py --scene-replay  # Mock 动作 + 真实场景状态信号
python3 run_demo.py --real          # 明确拒绝物理机械臂连接
```

仿真执行顺序：填写订单号、产品类型、数量、优先级和交期；单笔订单可直接点击 START，多笔订单先逐笔点击 ADD，最后点击 START。若 CoppeliaSim 未运行，软件会根据 `configs/runtime.yaml` 自动加载最新版场景。R1/R2/R3 完成一件产品并由 R3 清空装配夹具后，系统立即生成下一套箱体与物料；上一件产品同时由 CAMERA、R4、R5 完成检测、锁付和分拣。每批最多 20 个预置产品单元。

当前在线急单验证模式为 `3A + 1B`：先输入一笔 A 型订单并将数量设为 3，点击 START；状态变为 `EXECUTING` 后选择 B 型并点击 `URGENT`。软件固定插入一台 B，锁存急单后停止放行下一台 A，等待当前在制品完成、产线清空和五臂进入安全等待位，再生产 B；B 完成后恢复剩余 A。运行中只接受一台、数量为 1 的 B 型急单。

订单输入区的 `NG A UNIT` 用于指定三台 A 中的不良品：`0` 为全部良品，`1～3` 对应 A1～A3。NG 产品在 R4 锁付后由 R5 送往不良品区域，其余产品进入良品区。场景准备阶段会隐藏全部示教目标点，但保留其运动引用。

多订单分析顺序：在 `仿真执行` 页用 ADD 加入多笔订单，然后切换到 `调度分析` 页并点击“分析当前订单”。分析页只做离散事件调度，不驱动 CoppeliaSim；仿真执行页则会按同一批订单驱动多产品流水场景。底部 EXPORT 默认导出统一 JSON，其中同时包含订单、执行任务、任务结果、实际 KPI 和最近一次调度分析；CSV 用于导出逐任务执行明细。

默认 GUI 会发送关节轨迹给 CoppeliaSim 模型；`--mock` 和 `--scene-replay` 不会执行机械臂轨迹。`configs/motion_validation.yaml` 明确将仿真运动打开、物理机械臂运动关闭；`--real` 不存在绕过方式。

完整仿真验收（需有图形桌面）可运行：

```bash
python3 scripts/simulation_cycle_smoke.py --quality OK --timeout 900
python3 scripts/gui_integration_smoke.py --orders 3 --urgent-b
```

调度测试：

```bash
python3 -m unittest discover -s tests -q
```

装配流程和故障实验脚本位于 `scripts/`。

---

## 4. CoppeliaSim 启动

推荐环境：

- Ubuntu 22.04；
- CoppeliaSim Edu 4.10；
- Python 3。

正常情况下只需运行 `python3 run_demo.py`，GUI 会自动启动 CoppeliaSim。需要手动打开时，可以使用：

```bash
/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/coppeliaSim.sh \
  /home/zhu/cr5_assembly_team/scenes/compact_cell.ttt
```

`.ttt` 是 CoppeliaSim 场景文件，不能作为 shell 命令直接执行。默认 GUI 的运动路径通过 CoppeliaSim ZMQ Remote API 控制仿真模型，不经过物理机械臂驱动。

---

## 5. 团队统一规则

### 5.1 当前唯一主线

所有新代码、点位和文档均以“五台 CR5A 小型电控箱场景”为主线：

- R4 负责锁付；
- R5 负责质量分拣；
- 固定相机负责检测；
- 场景目标点以 `scenes/Step03_Create_Process_Targets_60_CloserTables.lua` 和 `configs/points.yaml` 为准；
- 调度任务名称、区域名称和机械臂名称应与配置文件保持一致。

### 5.2 历史四臂文档

以下文件保留为早期方案和设计参考，不代表当前场景的最终分工：

- `docs/PROJECT_PLAN.md`；
- `docs/R4_QUALITY_SORTING.md`；
- `docs/WORKSPACE_DESIGN.md`。

其中涉及“四臂、R4 分拣、低压配电柜”的内容属于历史方案。开发和联调时应优先参考本 README、五臂完整流程文档和五臂调度对齐说明。

### 5.3 分支与合并

- 每个成员在独立分支提交；
- PR 必须说明修改内容、接口影响和验证方法；
- 不直接覆盖他人负责模块；
- 合并前确认脚本文件名、对象路径、ROS2 topic、任务名称和配置名称一致；
- 冲突 PR 不直接强制合并，应将仍有效的内容迁移到最新 `main` 后再关闭旧 PR。

---

## 6. 推荐下一步

1. 以 `configs/scene_contract.yaml` 中的当前场景哈希为基线，重新规划 R1～R5 的 APP/TCP/Home 运动；
2. 对每条轨迹执行自碰撞、机器人间碰撞、夹具/工件间隙和共享区域互斥验证；
3. 将通过验证的轨迹清单写入 `configs/motion_validation.yaml`，再接入真实运动执行器；
4. 接收真实运动、夹具和视觉完成结果并推进统一编排器状态；
5. 完成订单输入到五臂连续装配、检测、锁付和分拣的端到端实机演示。

---

## 7. 仓库说明

动态调度测试相关说明集中放在 `scheduling_algorithm_test/`；正式开发仍以仓库根目录下的同名模块为准，避免维护多套副本。`scripts/dynamic_order_window.py` 和 `scripts/run_coppelia_order_demo.py` 是旧的独立演示入口，不参与正式一体化流程。

ROS2 的 `build/`、`install/`、`log/`，以及 Python 缓存、IDE 临时文件不应提交。
