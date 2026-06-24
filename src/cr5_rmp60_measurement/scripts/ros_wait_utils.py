#!/usr/bin/env python3
"""Small ROS2 waiting helpers used by planning scripts."""
import time

import rclpy


def wait_for_future(node, future, timeout_sec, description):
    """Spin the ROS2 node until a Future completes or timeout expires."""
    deadline = time.monotonic() + float(timeout_sec)
    while rclpy.ok() and time.monotonic() < deadline:
        rclpy.spin_once(node, timeout_sec=0.05)
        if future.done():
            return future.result()
    if future.done():
        return future.result()
    raise TimeoutError(f"timed out waiting for {description} after {timeout_sec:.3f}s")
