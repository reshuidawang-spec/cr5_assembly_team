#!/usr/bin/env python3
"""
RMP60 测头 DI1 信号监控节点

读取 dobot_bringup_ros2 发布的 FeedInfo 消息，
从中提取 digital_input_bits，监控 DI1（bit 0）状态。

用法:
    source ~/dobot_ws/install/setup.bash
    python3 probe_di_monitor.py

DI1 映射: digital_input_bits 的 bit 0
"""

import rclpy
from rclpy.node import Node
from rclpy.executors import ExternalShutdownException
from std_msgs.msg import String
import json


class ProbeDIMonitor(Node):
    def __init__(self):
        super().__init__("probe_di_monitor")
        self.sub = self.create_subscription(
            String,
            "/dobot_bringup_ros2/msg/FeedInfo",
            self._feed_info_callback,
            10,
        )
        self._last_state = None
        self._msg_count = 0
        self.get_logger().info("RMP60测头 DI 监控已启动，等待 FeedInfo ...")
        # 每 2 秒输出一次心跳
        self._timer = self.create_timer(2.0, self._heartbeat)

    def _feed_info_callback(self, msg):
        self._msg_count += 1
        try:
            data = json.loads(msg.data)
        except json.JSONDecodeError:
            return

        bits = data.get("digital_input_bits", 0)
        di1 = (bits & 0x01) != 0  # DI1 = bit 0

        if di1 != self._last_state:
            state_str = "触发" if di1 else "未触发"
            self.get_logger().info(f"DI1 状态变化: {state_str}")
            self._last_state = di1

    def _heartbeat(self):
        if self._msg_count > 0:
            state_str = "触发" if self._last_state else "未触发"
            self.get_logger().info(f"运行正常 (收到{self._msg_count}条消息, DI1={state_str})")
        else:
            self.get_logger().warning("尚未收到 FeedInfo，请检查 dobot_bringup_ros2 是否启动")


def main():
    """Main."""
    rclpy.init()
    node = ProbeDIMonitor()
    try:
        rclpy.spin(node)
    except (KeyboardInterrupt, ExternalShutdownException):
        pass
    finally:
        node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()


if __name__ == "__main__":
    main()
