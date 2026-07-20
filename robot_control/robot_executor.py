"""Team-facing real robot executor for the five-CR5A simulation cell."""

from __future__ import annotations

import math
import threading
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from interfaces.robot_interface import IRobotExecutor
from interfaces.types import RobotState, RobotStatus, Task, TaskResult, TaskStatus
from robot_control.r1_motion import (
    PLAN_PATH,
    R1_ACTIONS,
    R1_BOX_PLACED,
    R1_COMPLETE_CYCLE,
    R1_TERMINAL_PLACED,
    R1MotionController,
    load_r1_plan,
)
from robot_control.r2_motion import (
    R2_ACTIONS,
    R2_PCB_PLACED,
    R2MotionController,
)
from robot_control.r3_motion import (
    R3_ACTIONS,
    R3_MODULE_PLACED,
    R3_PRODUCT_TO_INSPECTION,
    R3MotionController,
)
from robot_control.r4_motion import (
    R4_ACTIONS,
    R4_SCREW_DONE,
    R4MotionController,
)
from robot_control.r5_motion import (
    R5_ACTIONS,
    R5_SORT_DEFECT_DONE,
    R5_SORT_GOOD_DONE,
    R5MotionController,
)
from sim_bridge.coppelia_client import SimBridge
from sim_bridge.scene_objects import ROBOT_IDS, normalize_robot_id


SUPPORTED_ACTIONS = R1_ACTIONS | R2_ACTIONS | R3_ACTIONS | R4_ACTIONS | R5_ACTIONS
ACTION_ROBOTS = {
    **{action: "R1" for action in R1_ACTIONS},
    **{action: "R2" for action in R2_ACTIONS},
    **{action: "R3" for action in R3_ACTIONS},
    **{action: "R4" for action in R4_ACTIONS},
    **{action: "R5" for action in R5_ACTIONS},
}


class RobotExecutor(IRobotExecutor):
    """Execute validated tasks without changing the shared interface contract.

    The current five-arm visual process actions are implemented for R1-R5.
    Unsupported robot/task combinations return ``failed``; they are never
    reported as successful placeholders.
    """

    def __init__(
        self,
        sim_bridge: Optional[SimBridge] = None,
        plan_path: Path = PLAN_PATH,
        speed_deg_s: float = 50.0,
        hold_seconds: float = 0.8,
        motion_controller_factory: Callable[..., R1MotionController] = (
            R1MotionController
        ),
        r2_motion_controller_factory: Callable[..., R2MotionController] = (
            R2MotionController
        ),
        r3_motion_controller_factory: Callable[..., R3MotionController] = (
            R3MotionController
        ),
        r4_motion_controller_factory: Callable[..., R4MotionController] = (
            R4MotionController
        ),
        r5_motion_controller_factory: Callable[..., R5MotionController] = (
            R5MotionController
        ),
    ):
        self._bridge = sim_bridge or SimBridge()
        self._plan_path = Path(plan_path)
        self._speed_deg_s = float(speed_deg_s)
        self._hold_seconds = float(hold_seconds)
        self._motion_controller_factory = motion_controller_factory
        self._r2_motion_controller_factory = r2_motion_controller_factory
        self._r3_motion_controller_factory = r3_motion_controller_factory
        self._r4_motion_controller_factory = r4_motion_controller_factory
        self._r5_motion_controller_factory = r5_motion_controller_factory
        self._state_lock = threading.RLock()
        self._execution_lock = threading.Lock()
        self._assembly_lock = threading.Lock()
        self._inspection_lock = threading.Lock()
        self._controllers: Dict[str, Any] = {}
        self._ready = False
        self._last_error = ""
        self._robots: Dict[str, RobotState] = {
            robot_id: RobotState(robot_id=robot_id)
            for robot_id in ROBOT_IDS
        }

    @property
    def last_error(self) -> str:
        return self._last_error

    @staticmethod
    def _resolve_action(task: Task) -> Optional[str]:
        for candidate in (task.target_point, task.process, task.task_id):
            normalized = str(candidate).strip().upper()
            if normalized in SUPPORTED_ACTIONS:
                return normalized
        return None

    @staticmethod
    def _task_robot(task: Task, action: Optional[str]) -> str:
        if task.available_robots:
            return normalize_robot_id(task.available_robots[0])
        if action in ACTION_ROBOTS:
            return ACTION_ROBOTS[action]
        raise ValueError("task has no available robot")

    @staticmethod
    def _result(
        task: Task,
        robot_id: str,
        status: str,
        start_time: float,
        message: str,
    ) -> TaskResult:
        return TaskResult(
            task_id=task.task_id,
            robot_id=robot_id,
            status=status,
            start_time=start_time,
            end_time=time.time(),
            message=message,
        )

    def _connect(self) -> None:
        if self._bridge.is_connected():
            return
        if not self._bridge.connect():
            raise RuntimeError(
                self._bridge.last_error or "cannot connect to CoppeliaSim"
            )

    def _controller_for(self, robot_id: str) -> Any:
        controller = self._controllers.get(robot_id)
        if controller is not None:
            return controller
        if robot_id == "R1":
            controller = self._motion_controller_factory(
                self._bridge,
                plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R2":
            controller = self._r2_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R3":
            controller = self._r3_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                assembly_lock=self._assembly_lock,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R4":
            controller = self._r4_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        elif robot_id == "R5":
            controller = self._r5_motion_controller_factory(
                self._bridge,
                r1_plan_path=self._plan_path,
                inspection_lock=self._inspection_lock,
                speed_deg_s=self._speed_deg_s,
                hold_seconds=self._hold_seconds,
            )
        else:
            raise ValueError(f"unsupported robot controller: {robot_id}")
        self._controllers[robot_id] = controller
        return controller

    @staticmethod
    def _quality_action(quality: str) -> str:
        normalized = str(quality).strip().lower()
        if normalized in {"good", "ok"}:
            return R5_SORT_GOOD_DONE
        if normalized in {"defect", "ng"}:
            return R5_SORT_DEFECT_DONE
        raise ValueError("quality must be good/OK or defect/NG")

    def _preposition_robots(
        self,
        r2: Any,
        r3: Any,
        r4: Any,
    ) -> float:
        """Step R2-R4 to their pick-APP configs after simulation is running.

        Must be called **after** ``enter_ready()`` because
        ``sim.setJointPosition`` in stopped mode is discarded by
        ``startSimulation``.  We use ``setJointTargetPosition`` with the
        already-held stepping to converge each robot without advancing
        simulation time for the subsequent task loop.

        Returns the additional simulation time consumed.
        """
        sim = getattr(self._bridge, "sim", None)
        if sim is None:
            return 0.0  # mock / fake bridge used in tests
        entries = (
            # (robot_id, controller, action_key, segment_name)
            # R4 is intentionally excluded: its screw APP is inside the
            # inspection zone and would collide with R3's product transfer.
            ("R2", r2, None, "initial_to_pick_app"),
            ("R3", r3, R3_MODULE_PLACED, "initial_to_pick_app"),
        )
        total_sim_time = 0.0
        for robot_id, controller, action_key, segment_name in entries:
            prepared = getattr(controller, "_prepared_paths", None)
            if prepared is None:
                continue
            # R3 stores a dict of actions → paths; R2/R4 store paths directly.
            paths = (
                prepared.get(action_key, {})
                if action_key is not None
                else prepared
            )
            segment = paths.get(segment_name) if isinstance(paths, dict) else None
            if not segment:
                continue
            config = segment[-1]
            joints = self._bridge.get_robot_joint_handles(robot_id)

            # Enable motion for kinematic joints that have maxVel == 0.
            max_vel = math.radians(60.0)
            original_velocities: list[float] = []
            for joint in joints:
                original_velocities.append(
                    sim.getObjectFloatParam(joint, sim.jointfloatparam_maxvel)
                )
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, max_vel
                )
            for idx, joint in enumerate(joints):
                sim.setJointTargetPosition(joint, float(config[idx]))

            # Converge within the already-held stepping loop.
            sim_before = float(sim.getSimulationTime())
            for _ in range(300):
                if not self._bridge.step():
                    raise RuntimeError(
                        self._bridge.last_error
                        or f"{robot_id} pre-position step failed"
                    )
                current = [
                    float(sim.getJointPosition(joint)) for joint in joints
                ]
                errors = [
                    abs(current[i] - config[i]) for i in range(len(config))
                ]
                if max(errors) <= math.radians(0.12):
                    break
            else:
                raise RuntimeError(
                    f"{robot_id} did not converge to pre-position target"
                )
            sim_after = float(sim.getSimulationTime())
            total_sim_time += sim_after - sim_before

            # Restore original maxVel so later execute() can set its own.
            for joint, original in zip(joints, original_velocities):
                sim.setObjectFloatParam(
                    joint, sim.jointfloatparam_maxvel, original
                )

            setter = getattr(controller, "set_pre_positioned", None)
            if setter is not None:
                setter(action_key, list(config))

        return total_sim_time

    def prepare_cycle(
        self, quality: str = "good", preload_both_r5: bool = False
    ) -> dict[str, Any]:
        """Precompute deterministic paths, then enter the resident READY state."""
        selected_r5_action = self._quality_action(quality)
        started = time.monotonic()
        self._ready = False
        evidence: list[dict[str, Any]] = []
        with self._execution_lock:
            self._connect()
            r1 = self._controller_for("R1")
            r2 = self._controller_for("R2")
            r3 = self._controller_for("R3")
            r4 = self._controller_for("R4")
            r5 = self._controller_for("R5")

            evidence.append(r1.prepare(R1_BOX_PLACED))
            evidence.append(r2.prepare(R2_PCB_PLACED))
            evidence.append(r3.prepare(R3_MODULE_PLACED))
            evidence.append(r3.prepare(R3_PRODUCT_TO_INSPECTION))
            evidence.append(r4.prepare(R4_SCREW_DONE))
            r5_actions = [selected_r5_action]
            if preload_both_r5:
                r5_actions = [R5_SORT_GOOD_DONE, R5_SORT_DEFECT_DONE]
            for action in r5_actions:
                evidence.append(r5.prepare(action))

            for controller in (r1, r2, r3, r4):
                controller.set_continuous_stepping(True)
            r5.set_continuous_stepping(False)

            ready_state = r1.enter_ready()

            # Pre-position R2-R4 to their pick APPs now that the simulation
            # is running with stepping held.  This lets each subsequent
            # execute() skip the initial-approach segment.
            preposition_sim_s = self._preposition_robots(r2, r3, r4)
            ready_state["preposition_simulation_time_s"] = preposition_sim_s

            self._ready = True

        path_points = sum(
            sum(int(count) for count in record.get("path_points", {}).values())
            for record in evidence
        )
        return {
            "ready": True,
            "quality_action": selected_r5_action,
            "preloaded_both_r5": bool(preload_both_r5),
            "controllers": evidence,
            "path_points_total": path_points,
            "ready_state": ready_state,
            "prepare_wall_s": time.monotonic() - started,
        }

    def execute_task(self, task: Task) -> TaskResult:
        start_time = time.time()
        action = self._resolve_action(task)
        try:
            robot_id = self._task_robot(task, action)
        except (KeyError, ValueError) as exc:
            self._last_error = str(exc)
            fallback = task.available_robots[0] if task.available_robots else ""
            return self._result(
                task,
                fallback,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )

        if action is None:
            self._last_error = (
                f"unsupported task {task.task_id}: expected one of "
                f"{sorted(SUPPORTED_ACTIONS)} in target_point/process/task_id"
            )
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )
        assigned_robot = ACTION_ROBOTS[action]
        if robot_id != assigned_robot:
            self._last_error = (
                f"{action} is assigned to {assigned_robot}, not {robot_id}"
            )
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )

        with self._state_lock:
            state = self._robots[robot_id]
            if state.status == RobotStatus.FAULT.value:
                self._last_error = f"{robot_id} is in fault state"
                return self._result(
                    task,
                    robot_id,
                    TaskStatus.FAILED.value,
                    start_time,
                    self._last_error,
                )
            if state.status == RobotStatus.BUSY.value:
                self._last_error = f"{robot_id} is already busy"
                return self._result(
                    task,
                    robot_id,
                    TaskStatus.FAILED.value,
                    start_time,
                    self._last_error,
                )
            state.status = RobotStatus.BUSY.value
            state.current_task = task.task_id

        try:
            with self._execution_lock:
                self._connect()
                controller = self._controller_for(robot_id)
                details = controller.execute(action)
            with self._state_lock:
                state = self._robots[robot_id]
                state.position = (
                    "R1_TERMINAL_PICK_APP"
                    if action == R1_BOX_PLACED
                    else "home"
                )
                state.completed_tasks += 1
            self._last_error = ""
            grasp_note = (
                "visual suction; physical grasp not validated"
                if robot_id in {"R2", "R3", "R5"}
                else (
                    "runtime visual screwdriver; physical torque not validated"
                    if robot_id == "R4"
                    else "visual attach; physical grasp not validated"
                )
            )
            return self._result(
                task,
                robot_id,
                TaskStatus.FINISHED.value,
                start_time,
                f"{action} completed ({grasp_note}); {details}",
            )
        except Exception as exc:
            self._last_error = str(exc)
            return self._result(
                task,
                robot_id,
                TaskStatus.FAILED.value,
                start_time,
                self._last_error,
            )
        finally:
            with self._state_lock:
                state = self._robots[robot_id]
                if state.status != RobotStatus.FAULT.value:
                    state.status = RobotStatus.IDLE.value
                state.current_task = None

    def execute_task_async(
        self, task: Task, callback: Callable[[TaskResult], None]
    ) -> None:
        def _run() -> None:
            callback(self.execute_task(task))

        threading.Thread(
            target=_run,
            name=f"robot-task-{task.task_id}",
            daemon=True,
        ).start()

    def move_to_point(self, robot_id: str, point_name: str) -> bool:
        """Accept only an idempotent request for the robot's current endpoint.

        Arbitrary point-to-point motion has not yet received path-level
        collision validation, so this method must not fabricate success.
        """
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError as exc:
            self._last_error = str(exc)
            return False
        with self._state_lock:
            current = self._robots[robot_id].position
        aliases = {
            "HOME": "home",
            **{f"{known_robot}_HOME_REF": "home" for known_robot in ROBOT_IDS},
        }
        requested = aliases.get(point_name.strip().upper(), point_name)
        if current == requested:
            self._last_error = ""
            return True
        self._last_error = (
            f"no independently validated path from {current} to {point_name} "
            f"for {robot_id}; use execute_task"
        )
        return False

    def gripper_open(self, robot_id: str) -> bool:
        return self._set_gripper(robot_id, True)

    def gripper_close(self, robot_id: str) -> bool:
        return self._set_gripper(robot_id, False)

    def _set_gripper(self, robot_id: str, opened: bool) -> bool:
        try:
            robot_id = normalize_robot_id(robot_id)
            self._connect()
            result = self._bridge.set_gripper(robot_id, opened)
            if not result:
                self._last_error = self._bridge.last_error
            return result
        except (KeyError, RuntimeError) as exc:
            self._last_error = str(exc)
            return False

    def screw_execute(self, robot_id: str, point_name: str) -> bool:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError as exc:
            self._last_error = str(exc)
            return False
        if robot_id != "R4" or point_name.strip().upper() not in {
            "R4_SCREW_TCP",
            "R4_SCREW_PRESS",
            R4_SCREW_DONE,
        }:
            self._last_error = (
                "R4 screw execution requires R4_SCREW_TCP, "
                "R4_SCREW_PRESS, or R4_SCREW_DONE"
            )
            return False
        task = Task(
            task_id=f"R4-SCREW-{time.time_ns()}",
            order_id="R4-DIRECT",
            product_type="A",
            process="screw",
            target_area="inspection_screw_area",
            target_point=R4_SCREW_DONE,
            available_robots=["R4"],
        )
        return self.execute_task(task).status == TaskStatus.FINISHED.value

    def robot_home(self, robot_id: str) -> bool:
        return self.move_to_point(robot_id, f"{robot_id.strip().upper()}_HOME_REF")

    def get_robot_states(self) -> List[RobotState]:
        with self._state_lock:
            return [
                RobotState(
                    robot_id=state.robot_id,
                    status=state.status,
                    current_task=state.current_task,
                    position=state.position,
                    utilization=state.utilization,
                    completed_tasks=state.completed_tasks,
                )
                for state in self._robots.values()
            ]

    def set_robot_fault(self, robot_id: str) -> None:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError:
            return
        with self._state_lock:
            self._robots[robot_id].status = RobotStatus.FAULT.value

    def clear_robot_fault(self, robot_id: str) -> None:
        try:
            robot_id = normalize_robot_id(robot_id)
        except KeyError:
            return
        with self._state_lock:
            state = self._robots[robot_id]
            state.status = RobotStatus.IDLE.value
            state.current_task = None


__all__ = [
    "RobotExecutor",
    "R1_BOX_PLACED",
    "R1_TERMINAL_PLACED",
    "R1_COMPLETE_CYCLE",
    "R2_PCB_PLACED",
    "R3_MODULE_PLACED",
    "R3_PRODUCT_TO_INSPECTION",
    "R4_SCREW_DONE",
    "R5_SORT_GOOD_DONE",
    "R5_SORT_DEFECT_DONE",
    "load_r1_plan",
]
