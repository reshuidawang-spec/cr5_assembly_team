"""Mock 订单解析器 —— 4号同学替换为真实实现"""

import json
from typing import List
from interfaces.order_interface import IOrderParser
from interfaces.types import Order


class MockOrderParser(IOrderParser):
    """模拟订单解析：内存中维护订单列表"""

    def __init__(self):
        self._orders: List[Order] = []

    def parse_file(self, filepath: str) -> List[Order]:
        with open(filepath, "r", encoding="utf-8") as f:
            data = json.load(f)
        orders = [self.parse_dict(item) for item in data]
        self._orders.extend(orders)
        return orders

    def parse_dict(self, data: dict) -> Order:
        return Order.from_dict(data)

    def add_order(self, order: Order) -> None:
        self._orders.append(order)

    def get_orders(self) -> List[Order]:
        return list(self._orders)

    def clear(self) -> None:
        self._orders.clear()
