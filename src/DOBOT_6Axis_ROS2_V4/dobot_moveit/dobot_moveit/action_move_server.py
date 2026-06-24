#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
@author FTX
@date 2025 / 03 / 03
"""

import math
import os
import time
from typing import List

import rclpy
from control_msgs.action import FollowJointTrajectory
from dobot_msgs_v4.srv import ClearError, EnableRobot, MovJ, ServoJ
from rclpy.action import ActionServer
from rclpy.node import Node
from trajectory_msgs.msg import JointTrajectory

class FollowJointTrajectoryServer(Node):

    def __init__(self):
        super().__init__('dobot_group_controller')
        name = os.getenv("DOBOT_TYPE", "cr5")
        # 创建FollowJointTrajectory动作服务器
        self._action_server = ActionServer(
            self,
            FollowJointTrajectory,
            f'/{name}_group_controller/follow_joint_trajectory',
            self.execute_callback,
        )
        self.get_logger().info(
            f"FollowJointTrajectory Action Server is ready on /{name}_group_controller/follow_joint_trajectory"
        )
        self.enable_robot_client = self.create_client(
            EnableRobot, '/dobot_bringup_ros2/srv/EnableRobot'
        )
        self.clear_error_client = self.create_client(
            ClearError, '/dobot_bringup_ros2/srv/ClearError'
        )
        self.servoj_client = self.create_client(ServoJ, '/dobot_bringup_ros2/srv/ServoJ')
        self.movj_client = self.create_client(MovJ, '/dobot_bringup_ros2/srv/MovJ')
        while not self.enable_robot_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.clear_error_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.servoj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')
        while not self.movj_client.wait_for_service(timeout_sec=1.0):
            self.get_logger().info('service not available, waiting again...')

    async def execute_callback(self, goal_handle):
        self.get_logger().info("Received a new trajectory goal!")
        trajectory = goal_handle.request.trajectory

        result = FollowJointTrajectory.Result()
        if not trajectory.points:
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.INVALID_GOAL
            result.error_string = "Empty trajectory."
            return result

        if not await self._prepare_robot():
            goal_handle.abort()
            result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
            result.error_string = "Failed to clear/enable robot before executing trajectory."
            return result

        stream_ok = await self._execute_with_servoj(trajectory)
        if not stream_ok:
            self.get_logger().warning(
                "ServoJ stream failed. Falling back to final-point MovJ in joint mode."
            )
            fallback_ok = await self._execute_final_point_with_movj(trajectory)
            if not fallback_ok:
                goal_handle.abort()
                result.error_code = FollowJointTrajectory.Result.PATH_TOLERANCE_VIOLATED
                result.error_string = "ServoJ and MovJ fallback both failed."
                return result

        goal_handle.succeed()
        result.error_code = FollowJointTrajectory.Result.SUCCESSFUL
        result.error_string = "Trajectory executed."
        return result

    async def _prepare_robot(self) -> bool:
        clear_req = ClearError.Request()
        enable_req = EnableRobot.Request()
        clear_res = await self.clear_error_client.call_async(clear_req)
        if clear_res is None or clear_res.res != 0:
            self.get_logger().error(f"ClearError failed, res={None if clear_res is None else clear_res.res}")
            return False
        enable_res = await self.enable_robot_client.call_async(enable_req)
        if enable_res is None or enable_res.res != 0:
            self.get_logger().error(f"EnableRobot failed, res={None if enable_res is None else enable_res.res}")
            return False
        return True

    async def _execute_with_servoj(self, trajectory: JointTrajectory) -> bool:
        last_time_s = 0.0
        for idx, point in enumerate(trajectory.points):
            joints_deg = self._point_to_deg(point.positions)
            if len(joints_deg) < 6:
                self.get_logger().error(f"Point {idx} has insufficient joints: {len(joints_deg)}")
                return False

            current_time_s = float(point.time_from_start.sec) + float(point.time_from_start.nanosec) * 1e-9
            segment_s = max(0.03, min(0.25, current_time_s - last_time_s))
            last_time_s = current_time_s

            req = ServoJ.Request()
            req.a = joints_deg[0]
            req.b = joints_deg[1]
            req.c = joints_deg[2]
            req.d = joints_deg[3]
            req.e = joints_deg[4]
            req.f = joints_deg[5]
            req.param_value = [f"t={segment_s:.3f}"]

            res = await self.servoj_client.call_async(req)
            if res is None or res.res != 0:
                self.get_logger().error(
                    f"ServoJ failed at point {idx}, res={None if res is None else res.res}, joints={joints_deg}"
                )
                return False

            # rclpy action execute callback does not guarantee an asyncio event loop.
            # Use plain sleep to pace ServoJ streaming.
            time.sleep(segment_s)

        return True

    async def _execute_final_point_with_movj(self, trajectory: JointTrajectory) -> bool:
        final_point = trajectory.points[-1]
        joints_deg = self._point_to_deg(final_point.positions)
        if len(joints_deg) < 6:
            self.get_logger().error(f"Final point has insufficient joints: {len(joints_deg)}")
            return False

        req = MovJ.Request()
        req.mode = True  # joint mode
        req.a = joints_deg[0]
        req.b = joints_deg[1]
        req.c = joints_deg[2]
        req.d = joints_deg[3]
        req.e = joints_deg[4]
        req.f = joints_deg[5]
        req.param_value = ["v=20", "a=20"]

        res = await self.movj_client.call_async(req)
        if res is None or res.res != 0:
            self.get_logger().error(
                f"MovJ fallback failed, res={None if res is None else res.res}, robot_return={'' if res is None else res.robot_return}"
            )
            return False
        return True

    @staticmethod
    def _point_to_deg(rad_positions: List[float]) -> List[float]:
        return [float(p) * 180.0 / math.pi for p in rad_positions]

def main(args=None):
    rclpy.init(args=args)
    follow_joint_trajectory_server = FollowJointTrajectoryServer()
    rclpy.spin(follow_joint_trajectory_server)
    follow_joint_trajectory_server.destroy_node()
    rclpy.shutdown()

if __name__ == '__main__':
    main()
