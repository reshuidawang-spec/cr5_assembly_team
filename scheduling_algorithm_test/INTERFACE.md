# 窗口与仿真接口

动态订单窗口和 CoppeliaSim 演示脚本通过 JSON 文件通信。

## 1. 接口文件

```text
output/dynamic_order_window_orders.json
```

窗口负责写入该文件；仿真脚本以监听模式读取该文件。

## 2. 写入时机

以下操作会重写 JSON：

- 新增普通订单；
- 插入急单；
- 修改算法权重并重排；
- 载入演示订单；
- 清空订单；
- 点击“开始仿真演示”。

## 3. JSON 字段

```json
{
  "updated_at": 0,
  "scoring": {
    "priority_weight": 0.45,
    "due_weight": 0.30,
    "lateness_risk_weight": 0.18,
    "waiting_weight": 0.15,
    "post_inspection_clearance_bonus": 0.40,
    "screw_clearance_bonus": 0.35,
    "urgent_due_boost": 0.20
  },
  "quality_policy": {
    "defects_per_100": 2
  },
  "material_switch": {
    "changeover_seconds": 3,
    "enabled_types": ["A", "B"]
  },
  "orders": [
    {
      "order_id": "A001",
      "product_type": "A",
      "quantity": 1,
      "priority": 1,
      "arrival_time": 0,
      "due_time": 260,
      "quality": "AUTO"
    }
  ]
}
```

## 4. 仿真脚本命令

```bash
python scripts/run_coppelia_order_demo.py \
  --orders-json output/dynamic_order_window_orders.json \
  --watch-orders \
  --speed 2
```

## 5. 颜色规则

- A：黄色；
- B：绿色；
- C：蓝色；
- 急单：红色；
- 已完成订单：灰色。

窗口表格中：

- 未开始：灰色；
- 进行中：黄色；
- 已完成：绿色。

为了避免串色，装配区和检测区壳体按工位分区刷新颜色。

## 6. A/B 换型与评价输出

窗口会把物料换型和质量策略同步给仿真脚本：

- `quality_policy.defects_per_100`：AUTO 检测时每 100 台设置几台次品；
- `material_switch.changeover_seconds`：A/B 型号切换时等待几秒；
- `material_switch.enabled_types`：当前演示参与换型的型号。

仿真脚本结束后会生成：

```text
output/dynamic_order_evaluation.json
```

其中包含：

- 总产品数；
- 良品数量；
- 次品数量；
- 成功率；
- 总完工时间；
- 冲突数；
- 检测后等待清台时间；
- A/B 型号完成数量。
