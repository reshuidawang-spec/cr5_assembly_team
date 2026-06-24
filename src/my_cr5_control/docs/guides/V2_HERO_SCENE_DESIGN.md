# V2 Hero Scene Design

## 1. 作用

这份文档用于固定一个不会与其他 demo 混淆的 `v2` 主线展示场景设计。

目标不是单纯“做更难的障碍”，而是做一个在 RViz 中能够直观看出：

- 为什么 `direct` 会出现长尾或不稳定
- 为什么 `HeuristicGuided` 选择某个中间 guide 是合理的
- 为什么 `hard / extreme` 场景应触发 active guidance

如果以后要做新的 `v2` 展示 mesh、开窗模型、答辩可视化或 hero benchmark scene，以本文件为准。

## 2. 该场景必须服务于什么主线

当前论文主线固定为：

- `HeuristicGuided two-stage guidance interface`
- `difficulty-adaptive informed guide sampling`
- `v2` 基于 `WS119.STL` 的真实几何 benchmark

因此这个 hero scene 的作用不是替代 `v2` benchmark，而是：

- 作为 `v2` 主线的高可解释展示场景
- 强化 `Hard_DeepInterior / Extreme_NarrowPassage` 这类几何条件下的可视化证据

## 3. 设计目标

理想场景应同时满足 4 个条件：

1. 人眼能理解“先对准入口，再下探”比直接硬插更合理
2. `direct` 规划仍然可行，但更容易出现长尾
3. 存在少量几何上合理的中间 guide pose
4. 被选中的 guide route 在 RViz 中能一眼看懂

这比单纯做成“越难越好”的迷宫更重要。

## 4. 推荐的 hero scene 结构

推荐使用：

- 单顶入口
- 深内部目标
- 入口下方过渡腔
- 偏置的二次收缩喉口

可以把它理解成：

- 外部从上方进入
- 先进入一个可重对准的过渡区
- 再通过一个横向略偏的窄喉口进入深腔
- 目标位于深腔内部而不是入口正下方

这种结构最适合展示：

- `direct` 看起来路径更短，但进入姿态不稳定
- top guide 候选会自然落在“入口上方 / 过渡腔上方 / 喉口前重对准位”
- `HeuristicGuided` 的两阶段路径是“可解释的”

## 5. 几何建议

建议先用下列量级，而不是一开始就做极端难例：

- 外箱尺寸：`220 x 220 x 180 mm`
- 顶部入口宽度：`70-80 mm`
- 过渡腔深度：`35-50 mm`
- 二次喉口宽度：`45-55 mm`
- 深腔总深度：`90-120 mm`
- 深腔内目标横向偏置：`10-20 mm`
- 目标距深腔底部高度：`20-35 mm`

约束原则：

- 顶部入口必须肉眼可见
- 喉口必须明显比入口更窄
- 目标不要与顶入口严格同轴
- 但也不要偏到需要“绕迷宫”才能到达

## 6. 为什么这种结构适合当前代码

当前 `v2` 的难度映射本身就是围绕下面几类点构造的：

- `Easy_HoleCenter`
- `Medium_HoleEdge`
- `Hard_DeepInterior`
- `Extreme_NarrowPassage`

参见：

- [measurement_point_generator.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/core/measurement_point_generator.cpp#L824)
- [measurement_point_generator.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/core/measurement_point_generator.cpp#L928)

当前 `HeuristicGuided` 在困难场景里会优先寻找满足这些性质的 candidate：

- 更接近入口中心的 recentered pose
- 略高于目标的 hovered pose
- 合理的 axial progress
- 受控的 lateral offset
- 足够 clearance

参见：

- [cr5_robot.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/core/cr5_robot.cpp#L2556)
- [cr5_robot.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/core/cr5_robot.cpp#L2582)
- [cr5_robot.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/core/cr5_robot.cpp#L2632)

因此，最适合当前方法的场景不是复杂迷宫，而是：

- 有明确入口语义
- 有可重对准区域
- 有深内部或窄通道约束

## 7. RViz 中应看到什么

hero scene 的 RViz 展示应能清楚看到：

1. 起点和目标点
2. direct line 穿过入口但没有体现合理预对准
3. top guide candidates 聚集在入口上方或喉口前
4. selected guide 明显形成“先对准，再下探”的折线
5. 文字摘要里 direct wall time 明显高于 heuristic wall time

当前可视化调试节点已经支持这些元素：

- start / goal marker
- direct marker
- top guides
- selected guide
- direct vs heuristic 诊断摘要

参见：

- [heuristic_guided_visual_debug.cpp](/home/zhu/dobot_ws/src/my_cr5_control/src/tools/heuristic_guided_visual_debug.cpp#L538)

## 8. 不建议的场景

以下设计会弱化展示效果：

- 开口太大，导致 `direct` 也很稳
- 完全同轴的深孔，导致“引导合理性”不明显
- 完全封死或超窄迷宫，导致问题退化为不可达
- 大量无关障碍堆叠，视觉复杂但不服务于入口重对准语义

## 9. 建议的展示组合

建议不是只做一个 hero scene，而是做 3 个层次：

1. `Easy`：
   - 大开口、浅目标、同轴
   - 用来证明方法不会无意义触发 guidance
2. `Hard`：
   - 深内部目标、入口明确、轻微偏置
   - 用来证明 active guidance 开始变得有意义
3. `Extreme Hero`：
   - 单顶入口 + 偏置深腔 + 二次收缩喉口
   - 用来做答辩、截图和 RViz 演示

## 10. 防混淆约定

从现在开始，关于这个展示场景统一遵守下面命名：

- 统一称为：`v2 hero scene`
- 如果落成具体 mesh，文件名应包含：`ws119_v2_hero`
- 如果是半剖开展示版，文件名应包含：`ws119_v2_hero_cutaway`
- 不得使用：
  - `paper_mainline_demo`
  - `synthetic_guidance_demo`
  - `mainline_demo`

原因是：

- `paper mainline` 指的是方法 + benchmark 证据链
- 不是一个单独的可视化场景文件

## 10A. 已落地的 hero scene

当前已落地的第一版 hero scene 是：

- mesh profile：`hero_offset_throat`
- benchmark scene：`Extreme_OffsetThroatDeepCavity`
- 完整碰撞 mesh：`src/meshes/ws119_v2_hero_offset_throat.stl`
- 半剖展示 mesh：`src/meshes/ws119_v2_hero_offset_throat_cutaway.stl`

快速检查场景生成：

```bash
cd /home/zhu/dobot_ws
source install/setup.bash
MY_CR5_CONTROL_V2_MESH_PROFILE=hero_offset_throat \
  ros2 run my_cr5_control inspect_v2_scenarios_node
```

RViz 展示命令：

```bash
cd /home/zhu/dobot_ws
source install/setup.bash
ros2 launch my_cr5_control heuristic_guided_visual_debug.launch.py \
  benchmark:=v2 \
  scene:=Extreme_OffsetThroatDeepCavity \
  mesh_profile:=hero_offset_throat \
  sample_count:=24 \
  top_guides:=10 \
  hold_s:=60 \
  guide_seed:=119 \
  execute_motion:=false \
  use_rviz:=true \
  keep_alive:=true \
  shutdown_on_exit:=false \
  start_delay:=5.0
```

RViz 中的颜色约定：

- 红色轨迹：`RRTConnect` direct plan
- 青色轨迹：`HeuristicGuided` 两阶段轨迹
- 蓝/青色测头：与碰撞模型尺寸一致的可视化测头 marker
- 黄色十字测针：星形测针位置
- 绿色/黄色候选点：ranked guide candidates

## 11. 下次新对话的最小上下文

如果以后新开对话，只要先明确下面 3 句话，就能避免混淆：

1. 当前展示对象是 `v2/WS119` 主线，不是 synthetic demo
2. 目标是设计一个能体现 `Hard_DeepInterior / Extreme_NarrowPassage` 优势的 `v2 hero scene`
3. 以本文件和 [docs/PAPER_MAINLINE_MAP.md](/home/zhu/dobot_ws/src/my_cr5_control/docs/PAPER_MAINLINE_MAP.md) 为准
