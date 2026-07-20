"""共享数据类型定义 —— 所有模块统一使用这些数据结构。"""

from dataclasses import dataclass, field
from typing import Any, List, Optional
from enum import Enum


class TaskStatus(str, Enum):
    PENDING = "pending"
    RUNNING = "running"
    FINISHED = "finished"
    FAILED = "failed"
    WAITING = "waiting"


class RobotStatus(str, Enum):
    IDLE = "idle"
    BUSY = "busy"
    FAULT = "fault"


class QualityResult(str, Enum):
    OK = "OK"
    NG = "NG"


class ProcessType(str, Enum):
    FEED = "feed"
    ASSEMBLE = "assemble"
    SCREW = "screw"
    INSPECT = "inspect"
    SORT_GOOD = "sort_good"
    SORT_DEFECT = "sort_defect"
    UNLOAD = "unload"
    REWORK = "rework"


@dataclass
class Order:
    """订单数据结构"""
    order_id: str
    product_type: str          # A / B / C
    priority: int              # 越大越紧急
    quantity: int = 1
    due_time: float = 0.0
    expected_quality: str = "OK"

    def to_dict(self) -> dict:
        return {
            "order_id": self.order_id,
            "product_type": self.product_type,
            "priority": self.priority,
            "quantity": self.quantity,
            "due_time": self.due_time,
            "expected_quality": self.expected_quality,
        }

    @classmethod
    def from_dict(cls, d: dict) -> "Order":
        return cls(
            order_id=d["order_id"],
            product_type=d["product_type"],
            priority=d.get("priority", 1),
            quantity=d.get("quantity", 1),
            due_time=d.get("due_time", 0.0),
            expected_quality=str(d.get("expected_quality", "OK")).upper(),
        )


@dataclass
class Task:
    """任务数据结构 —— 调度模块输出、机械臂控制模块输入"""
    task_id: str
    order_id: str
    product_type: str
    process: str                # feed / assemble / screw / inspect / sort_good / sort_defect / rework
    target_area: str
    target_point: str
    available_robots: List[str] = field(default_factory=list)
    duration: float = 5.0
    predecessors: List[str] = field(default_factory=list)
    priority: int = 1
    status: str = "pending"

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "order_id": self.order_id,
            "product_type": self.product_type,
            "process": self.process,
            "target_area": self.target_area,
            "target_point": self.target_point,
            "available_robots": self.available_robots,
            "duration": self.duration,
            "predecessors": self.predecessors,
            "priority": self.priority,
            "status": self.status,
        }


@dataclass
class TaskResult:
    """任务执行结果 —— 机械臂控制模块输出"""
    task_id: str
    robot_id: str
    status: str                 # finished / failed / running
    start_time: float = 0.0
    end_time: float = 0.0
    message: str = ""
    quality_result: str = ""    # OK / NG (仅 inspection 任务有效)
    metrics: dict[str, Any] = field(default_factory=dict)

    def to_dict(self) -> dict:
        return {
            "task_id": self.task_id,
            "robot_id": self.robot_id,
            "status": self.status,
            "start_time": self.start_time,
            "end_time": self.end_time,
            "message": self.message,
            "quality_result": self.quality_result,
            "metrics": self.metrics,
        }


@dataclass
class RobotState:
    """机械臂状态"""
    robot_id: str
    status: str = "idle"        # idle / busy / fault
    current_task: Optional[str] = None
    position: str = "home"
    utilization: float = 0.0
    completed_tasks: int = 0

    def to_dict(self) -> dict:
        return {
            "robot_id": self.robot_id,
            "status": self.status,
            "current_task": self.current_task,
            "position": self.position,
            "utilization": self.utilization,
            "completed_tasks": self.completed_tasks,
        }


@dataclass
class SystemSnapshot:
    """系统状态快照 —— 用于 GUI 刷新和数据记录"""
    timestamp: float = 0.0
    orders: List[Order] = field(default_factory=list)
    tasks: List[Task] = field(default_factory=list)
    robots: List[RobotState] = field(default_factory=list)
    logs: List[str] = field(default_factory=list)
    makespan: float = 0.0
    conflict_count: int = 0
