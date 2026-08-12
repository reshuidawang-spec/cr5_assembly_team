"""Validated order input for the production scheduler.

The GUI, command-line entry points and scheduling-analysis page all use this
parser so an order has the same meaning everywhere in the application.
"""

from __future__ import annotations

import json
import math
from pathlib import Path
from typing import List

from interfaces.order_interface import IOrderParser
from interfaces.types import Order


class OrderParser(IOrderParser):
    """Parse and retain validated A/B/C product orders."""

    PRODUCT_TYPES = {"A", "B", "C"}

    def __init__(self):
        self._orders: List[Order] = []

    def parse_file(self, filepath: str) -> List[Order]:
        data = json.loads(Path(filepath).read_text(encoding="utf-8"))
        if isinstance(data, dict) and "orders" in data:
            data = data["orders"]
        if not isinstance(data, list):
            raise ValueError("订单文件必须是 JSON 数组或包含 orders 数组的对象")

        orders = [self.parse_dict(item) for item in data]
        self._ensure_unique(orders, existing=self._orders)
        self._orders.extend(orders)
        return list(orders)

    def parse_dict(self, data: dict) -> Order:
        if not isinstance(data, dict):
            raise ValueError("每个订单必须是 JSON 对象")

        order_id = str(data.get("order_id", "")).strip()
        product_type = str(data.get("product_type", "")).strip().upper()
        if not order_id:
            raise ValueError("order_id 不能为空")
        if product_type not in self.PRODUCT_TYPES:
            raise ValueError(f"不支持的 product_type: {product_type or '<空>'}")

        priority = self._integer(data.get("priority", 1), "priority")
        quantity = self._integer(data.get("quantity", 1), "quantity")
        if not 1 <= priority <= 10:
            raise ValueError("priority 必须在 1 到 10 之间")
        if quantity < 1:
            raise ValueError("quantity 必须大于 0")

        due_time = self._nonnegative_number(data.get("due_time", 0.0), "due_time")
        arrival_time = self._nonnegative_number(
            data.get("arrival_time", 0.0), "arrival_time"
        )
        return Order(
            order_id=order_id,
            product_type=product_type,
            priority=priority,
            quantity=quantity,
            due_time=due_time,
            arrival_time=arrival_time,
        )

    def add_order(self, order: Order) -> None:
        validated = self.parse_dict(order.to_dict())
        self._ensure_unique([validated], existing=self._orders)
        self._orders.append(validated)

    def get_orders(self) -> List[Order]:
        return list(self._orders)

    def clear(self) -> None:
        self._orders.clear()

    @staticmethod
    def _integer(value: object, field: str) -> int:
        if isinstance(value, bool):
            raise ValueError(f"{field} 必须是整数")
        try:
            converted = int(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field} 必须是整数") from exc
        try:
            numeric = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field} 必须是整数") from exc
        if not math.isfinite(numeric) or numeric != converted:
            raise ValueError(f"{field} 必须是整数")
        return converted

    @staticmethod
    def _nonnegative_number(value: object, field: str) -> float:
        if isinstance(value, bool):
            raise ValueError(f"{field} 必须是非负数")
        try:
            converted = float(value)
        except (TypeError, ValueError) as exc:
            raise ValueError(f"{field} 必须是非负数") from exc
        if not math.isfinite(converted) or converted < 0:
            raise ValueError(f"{field} 必须是非负数")
        return converted

    @staticmethod
    def _ensure_unique(orders: List[Order], existing: List[Order]) -> None:
        seen = {order.order_id for order in existing}
        for order in orders:
            if order.order_id in seen:
                raise ValueError(f"订单编号重复: {order.order_id}")
            seen.add(order.order_id)


__all__ = ["OrderParser"]
