# 调度算法动态测试使用说明

本说明只对应本项目新增的“动态订单调度测试 + CoppeliaSim 联动演示”部分。原系统 APP、原场景文件、Lua 场景脚本和库内机械臂轨迹不需要重新说明。

## 1. Ubuntu 启动方式

默认运行环境为 Ubuntu。首次使用时，在仓库根目录执行：

```bash
python3 -m venv .venv
source .venv/bin/activate
pip install -r requirements.txt
```

设置 CoppeliaSim 安装路径：

```bash
export COPPELIASIM_ROOT=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04
```

启动调度测试窗口：

```bash
bash scripts/run_dynamic_order_window_ubuntu.sh
```

窗口中输入 A/B/C 订单数量、订单型号、急单信息和评价参数后，点击“开始仿真”。程序会自动调用：

```text
scripts/dynamic_order_window.py
  -> scripts/run_coppelia_order_demo.py
  -> robot_control/coordinated_engine.py
  -> scripts/coordinated_front.py
  -> data/captured_paths/
```

其中 `data/captured_paths/` 是库内五机械臂协同轨迹，最终演示时机械臂动作采用该轨迹回放。

如果需要单独先打开 CoppeliaSim 场景，也可以运行：

```bash
bash scripts/start_coppelia_ubuntu.sh
```

## 2. 当前演示流程

1. 在窗口输入初始订单，例如先生产 A 型。
2. 点击“开始仿真”后，CoppeliaSim 加载五机械臂场景。
3. 对应型号小车前移，显示物料切换/送料过程。
4. 小车送出的物料落到 R1 上料位，R1 从该位置抓取。
5. 五台机械臂按库内轨迹完成装配、检测、锁付和分拣。
6. 中途可以在窗口继续插入订单或急单，调度算法重新排序。
7. 若流程为 A-B-A，则完成指定数量 A 后插入 B，完成 B 后继续执行剩余 A。
8. 演示结束后，窗口生成评价结果，包括完成时间、急单完成时间、成功率、良品/次品数量等。

## 3. 颜色规则

颜色只表示当前物料型号，不再表示工序阶段。

- A 型小车为橘色，A 型箱体、PCB、模块、端子、成品全部为橘色。
- B 型小车为蓝色，B 型箱体、PCB、模块、端子、成品全部为蓝色。
- 急单在窗口/订单标记中用红色提示，但真实零件仍然跟随它所属型号的小车颜色。

这样可以避免演示过程中出现物体突然串色或变色的问题。

## 4. 调度算法思路

调度算法不改变单个产品内部工序，只改变订单执行顺序。每个订单仍然遵守固定流程：

```text
上料 -> PCB安装 -> 模块安装 -> 端子安装 -> 转运检测 -> 相机检测 -> 锁付/分拣
```

排序时综合考虑：

- 订单优先级；
- 交期紧迫程度；
- 当前产线型号与目标订单型号是否一致；
- 换型代价；
- 急单插入后的重新排序；
- 订单等待时间。

因此，窗口再次输入新订单或急单时，系统会根据当前未完成订单重新计算执行队列，但不会打乱任意单件产品内部工序。

## 5. 评价输出

演示结束后会给出一组流程评价指标，主要用于观察调度是否合理：

- 总完成时间；
- 订单完成数量；
- 急单完成时间；
- 换型次数；
- 良品数量；
- 次品数量；
- 人为设定成功率/缺陷率；
- 冲突或等待情况。

这些指标用于判断急单插入后产线是否能完成 A-B-A 换型生产，以及调度顺序是否符合评分逻辑。

## 6. 需要上传的核心文件

如果只上传本次调度测试功能，至少保留以下内容：

```text
configs/
data/captured_paths/
docs/五臂协同.md
docs/AGV_FLEXIBLE_CELL_GUIDE.md
interfaces/
robot_control/coordinated_engine.py
robot_control/motion_control.py
robot_control/runtime_cartesian.py
scheduler/
scheduling_algorithm_test/
scripts/dynamic_order_window.py
scripts/run_coppelia_order_demo.py
scripts/coordinated_front.py
scripts/run_dynamic_order_window_ubuntu.sh
scripts/start_coppelia_ubuntu.sh
sim_bridge/
tests/test_scheduler_v2.py
requirements.txt
```

原始场景和原系统 APP 如果仓库中已有，可继续使用原仓库版本；本测试功能主要通过脚本和调度逻辑与场景通信。
