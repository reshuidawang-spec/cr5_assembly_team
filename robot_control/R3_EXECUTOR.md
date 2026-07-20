# R3 视觉装配与产品转移执行器

R3 已接入团队现有接口：

```text
R3_MODULE_PLACED         -> R3
R3_PRODUCT_TO_INSPECTION -> R3
```

当前保存场景没有 R3 吸盘。两个动作都在运行时创建沿 Link6
工具轴前伸 `100 mm` 的虚拟 TCP，使用 `(195,0,-135)` 度姿态，
不修改 9 个 Git R3 target。

## 动作边界

`R3_MODULE_PLACED` 接续实际 R2 PCB 末态。当前模块和 PCB 布局重叠，
因此模块在视觉吸附后使用世界 `+Z 46 mm` 偏移。任务完成取料、
安装、共享区退出和 R3 回零。

`R3_PRODUCT_TO_INSPECTION` 在 R1 端子安装后执行。执行器隐藏独立工件，
显示并搬运完整装配模板，经基座前侧高位路径移到检测区。搬运时
产品视觉上提 `100 mm`，到达检测 TCP 后降回原高。R3 回零后才切换到
`Inspection_ControlBox_Product`。

`Camera_View_Area` 是按设计包围检测产品的非物理视野体，产品转移只排除
该单一 shape。相机机身、立柱、检测台、其他机械臂和全部工作台仍在碰撞集中。

## 命令

R3 模块必须在 R1 箱体和 R2 PCB 完成后运行；R3 产品转移必须再接续
R1 端子任务。推荐直接使用一键协调器。单步调试命令为：

```bash
python3 robot_control/run_r3_task.py R3_MODULE_PLACED
python3 robot_control/run_r3_task.py R3_PRODUCT_TO_INSPECTION
```

这两个动作是视觉吸附和模板转换，不是物理吸盘、真实负载或物理装配验收。
