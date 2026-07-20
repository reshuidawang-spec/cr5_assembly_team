"""Validated order parser used by the real scheduler integration."""

from __future__ import annotations

import json
from pathlib import Path
from typing import List

from interfaces.order_interface import IOrderParser
from interfaces.types import Order


class OrderParser(IOrderParser):
    def __init__(self):
        self._orders: List[Order] = []

    def parse_file(self, filepath: str) -> List[Order]:
        data = json.loads(Path(filepath).read_text(encoding="utf-8"))
        if not isinstance(data, list):
            raise ValueError("order file must contain a JSON list")
        orders = [self.parse_dict(item) for item in data]
        self._orders.extend(orders)
        return orders

    def parse_dict(self, data: dict) -> Order:
        if not isinstance(data, dict):
            raise ValueError("each order must be a JSON object")
        order = Order.from_dict(data)
        if not order.order_id.strip():
            raise ValueError("order_id must not be empty")
        if order.quantity <= 0:
            raise ValueError("order quantity must be positive")
        if order.priority <= 0:
            raise ValueError("order priority must be positive")
        quality = order.expected_quality.upper()
        if quality not in {"OK", "NG"}:
            raise ValueError("expected_quality must be OK or NG")
        order.expected_quality = quality
        return order

    def add_order(self, order: Order) -> None:
        validated = self.parse_dict(order.to_dict())
        self._orders.append(validated)

    def get_orders(self) -> List[Order]:
        return list(self._orders)

    def clear(self) -> None:
        self._orders.clear()


__all__ = ["OrderParser"]
