"""订单解析模块接口 —— 4号同学实现"""

from abc import ABC, abstractmethod
from typing import List
from .types import Order


class IOrderParser(ABC):
    """订单解析接口

    职责：读取订单输入（文件 / GUI / 命令行），解析为标准 Order 对象列表。

    4号同学需要实现：
      - parse_file(filepath) → 从 JSON 文件读取订单
      - parse_dict(data)    → 从字典解析单个订单
      - get_orders()        → 返回当前所有待处理订单
    """

    @abstractmethod
    def parse_file(self, filepath: str) -> List[Order]:
        """从 JSON 文件解析订单列表"""
        ...

    @abstractmethod
    def parse_dict(self, data: dict) -> Order:
        """从字典解析单个订单"""
        ...

    @abstractmethod
    def add_order(self, order: Order) -> None:
        """添加新订单到队列"""
        ...

    @abstractmethod
    def get_orders(self) -> List[Order]:
        """返回当前所有订单"""
        ...

    @abstractmethod
    def clear(self) -> None:
        """清空订单"""
        ...
