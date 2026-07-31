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

