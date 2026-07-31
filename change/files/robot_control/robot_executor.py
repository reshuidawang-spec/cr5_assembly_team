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
    R3_PRODUCT_TRANSFER_CLEARANCE,
    R3_PRODUCT_TO_INSPECTION,
    R3MotionController,
)
from robot_control.r4_motion import (
    R4_ACTIONS,
    R4_SCREW_DONE,
    R4_WAIT_POINT,
    R4MotionController,
)
from robot_control.r5_motion import (
    R5_ACTIONS,
    R5_SORT_DEFECT_DONE,
    R5_SORT_GOOD_DONE,
    R5_WAIT_POINT,
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
        hold_seconds: float = 1.0,
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
        self._hold_seconds = min(1.0, max(0.0, float(hold_seconds)))
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
        host = getattr(self._bridge, "host", None)
        port = getattr(self._bridge, "port", None)
        connected = (
            self._bridge.connect(host, port)
            if host is not None and port is not None
            else self._bridge.connect()
        )
        if not connected:
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

    @staticmethod
    def _grasp_note(robot_id: str) -> str:
        if robot_id == "R2":
            return "visual suction; physical grasp not validated"
        if robot_id in {"R3", "R5"}:
            return "visual gripper; physical grasp not validated"
        if robot_id == "R4":
            return "runtime visual screwdriver; physical torque not validated"
        return "visual attach; physical grasp not validated"

    def _preposition_robots(
        self,
        r2: Any,
        r3: Any,
        r4: Any,
        r5: Any,
        r5_action: str,
        preposition_front_half: bool = False,
    ) -> float:
        """Optionally step front-half robots to their pick-APP configs.

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
        entries = []
        if preposition_front_half:
            # (robot_id, controller, action_key, segment_name)
            # R2 is intentionally excluded for the coordinated front half:
            # waiting at R2_PCB_PICK_APP blocks R1's box transfer. R2 now
            # starts from zero, picks the PCB, moves to a near safe wait, and
            # only enters the assembly area after R1 clears the interference.
            entries.append(
                ("R3", r3, R3_MODULE_PLACED, "initial_to_pick_app")
            )
            entries.extend(
                [
                    ("R4", r4, None, "home_to_wait"),
                    ("R5", r5, r5_action, "home_to_wait"),
                ]
            )
        total_sim_time = 0.0
        for robot_id, controller, action_key, segment_name in entries:
            prepared = getattr(controller, "_prepared_paths", None)
            if prepared is None:
                continue
            # R3/R5 store action -> paths; R2/R4 store paths directly.
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
        self,
        quality: str = "good",
        preload_both_r5: bool = False,
        preposition_front_half: bool = False,
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

            # Optionally pre-position front-half robots now that the
            # simulation is running with stepping held. The coordinated
            # front-half runner leaves this disabled so R3 cannot visibly
            # move before the PCB-release handoff.
            preposition_sim_s = self._preposition_robots(
                r2,
                r3,
                r4,
                r5,
                selected_r5_action,
                preposition_front_half=preposition_front_half,
            )
            ready_state["preposition_simulation_time_s"] = preposition_sim_s
            ready_state["front_half_prepositioned"] = bool(
                preposition_front_half
            )

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

    @staticmethod
    def _coordinated_task(
        action: str,
        robot_id: str,
        index: int,
        order_id: str,
        process: str = "assemble",
        area: str = "assembly_area",
    ) -> Task:
        return Task(
            task_id=f"{order_id}-COORD-{index:02d}-{action}",
            order_id=order_id,
            product_type="A",
            process=process,
            target_area=area,
            target_point=action,
            available_robots=[robot_id],
        )

    def execute_coordinated_front_half(
        self,
        quality: str = "good",
        order_id: str = "FIVE-ARM-DEMO",
    ) -> dict[str, Any]:
        """Run the taught R1/R2/R3 front half with explicit handoff gates."""
        from robot_control.coordinated_front_half import CoordinatedFrontHalfRunner

        selected_r5_action = self._quality_action(quality)

        task_entries = [
            (R1_BOX_PLACED, "R1", "assemble", "assembly_area"),
            (R2_PCB_PLACED, "R2", "assemble", "assembly_area"),
            (R3_MODULE_PLACED, "R3", "assemble", "assembly_area"),
            (R1_TERMINAL_PLACED, "R1", "assemble", "assembly_area"),
            (
                R3_PRODUCT_TO_INSPECTION,
                "R3",
                "transfer",
                "inspection_screw_area",
            ),
        ]

        with self._execution_lock:
            self._connect()
            tasks = {
                action: self._coordinated_task(
                    action,
                    robot_id,
                    index,
                    order_id,
                    process,
                    area,
                )
                for index, (action, robot_id, process, area) in enumerate(
                    task_entries, start=1
                )
            }
            start_wall = time.time()
            start_sim = float(self._bridge.sim.getSimulationTime())
            for _, robot_id, _, _ in task_entries:
                with self._state_lock:
                    state = self._robots[robot_id]
                    state.status = RobotStatus.BUSY.value
                    state.current_task = "COORDINATED_FRONT_HALF"
            try:
                runner = CoordinatedFrontHalfRunner(
                    self._bridge,
                    speed_deg_s=self._speed_deg_s,
                    hold_seconds=self._hold_seconds,
                )
                details = runner.execute()
                prepositioned = details.get("prepositioned_configs", {})
                r4_config = prepositioned.get("R4", {}).get("config")
                if r4_config is not None:
                    r4 = self._controller_for("R4")
                    setter = getattr(r4, "set_pre_positioned", None)
                    if setter is not None:
                        setter(R4_SCREW_DONE, list(r4_config))
                    with self._state_lock:
                        self._robots["R4"].position = R4_WAIT_POINT
                r5_config = prepositioned.get("R5", {}).get("config")
                if r5_config is not None:
                    r5 = self._controller_for("R5")
                    setter = getattr(r5, "set_pre_positioned", None)
                    if setter is not None:
                        setter(selected_r5_action, list(r5_config))
                    with self._state_lock:
                        self._robots["R5"].position = R5_WAIT_POINT
            except Exception as exc:
                end_wall = time.time()
                try:
                    end_sim = float(self._bridge.sim.getSimulationTime())
                except Exception:
                    end_sim = start_sim
                cleanup_error = ""
                try:
                    self._bridge.stop_simulation()
                except Exception as cleanup_exc:
                    cleanup_error = str(cleanup_exc)
                result = TaskResult(
                    task_id=f"{order_id}-COORDINATED_FRONT_HALF",
                    robot_id="R1/R2/R3/R4/R5",
                    status=TaskStatus.FAILED.value,
                    start_time=start_wall,
                    end_time=end_wall,
                    message=(
                        str(exc)
                        if not cleanup_error
                        else f"{exc}; cleanup_error={cleanup_error}"
                    ),
                )
                for _, robot_id, _, _ in task_entries:
                    with self._state_lock:
                        state = self._robots[robot_id]
                        if state.status != RobotStatus.FAULT.value:
                            state.status = RobotStatus.IDLE.value
                        state.current_task = None
                record = {
                    "task": {
                        "task_id": result.task_id,
                        "order_id": order_id,
                        "product_type": "A",
                        "process": "coordinated_front_half",
                        "target_area": "assembly_area",
                        "target_point": "COORDINATED_FRONT_HALF",
                        "available_robots": ["R1", "R2", "R3", "R4", "R5"],
                    },
                    "coordinated_front_half": True,
                    "single_step_runner": True,
                    "start_wall_epoch_s": start_wall,
                    "end_wall_epoch_s": end_wall,
                    "start_simulation_time_s": start_sim,
                    "end_simulation_time_s": end_sim,
                    "wall_duration_s": end_wall - start_wall,
                    "simulation_duration_s": max(0.0, end_sim - start_sim),
                    "result": result.to_dict(),
                }
                return {
                    "status": "failed",
                    "tasks": [record],
                    "failed_action": "COORDINATED_FRONT_HALF",
                    "errors": {"COORDINATED_FRONT_HALF": str(exc)},
                }

            end_wall = time.time()
            end_sim = float(self._bridge.sim.getSimulationTime())
            records = []
            for action, robot_id, process, area in task_entries:
                task = tasks[action]
                result = TaskResult(
                    task_id=task.task_id,
                    robot_id=robot_id,
                    status=TaskStatus.FINISHED.value,
                    start_time=start_wall,
                    end_time=end_wall,
                    message=(
                        f"{action} completed in single-step coordinated "
                        f"front half; {details}"
                    ),
                )
                records.append(
                    {
                        "task": task.to_dict(),
                        "coordinated_front_half": True,
                        "single_step_runner": True,
                        "start_wall_epoch_s": start_wall,
                        "end_wall_epoch_s": end_wall,
                        "start_simulation_time_s": start_sim,
                        "end_simulation_time_s": end_sim,
                        "wall_duration_s": end_wall - start_wall,
                        "simulation_duration_s": max(0.0, end_sim - start_sim),
                        "motion_timing": {
                            "robot_id": robot_id,
                            "motion_detected": True,
                            "monitor_error": "single-step coordinated direct run",
                        },
                        "result": result.to_dict(),
                    }
                )
                with self._state_lock:
                    state = self._robots[robot_id]
                    state.completed_tasks += 1
                    state.status = RobotStatus.IDLE.value
                    state.current_task = None
                    if action == R2_PCB_PLACED:
                        state.position = "R2_PCB_PICK_APP"
                    elif action == R3_MODULE_PLACED:
                        state.position = (
                            "R3_TEMP_CLEAR_FOR_R1_TERMINAL_BEFORE_PRODUCT_PICK_APP"
                        )
                    elif action == R3_PRODUCT_TO_INSPECTION:
                        state.position = R3_PRODUCT_TRANSFER_CLEARANCE
                    else:
                        state.position = "home"
            return {
                "status": "finished",
                "tasks": records,
                "failed_action": None,
                "errors": {},
                "details": details,
            }

        task_entries = [
            (R1_BOX_PLACED, "R1"),
            (R2_PCB_PLACED, "R2"),
            (R3_MODULE_PLACED, "R3"),
            (R1_TERMINAL_PLACED, "R1"),
            (
                R3_PRODUCT_TO_INSPECTION,
                "R3",
                "transfer",
                "inspection_screw_area",
            ),
        ]
        tasks: dict[str, Task] = {}
        for index, entry in enumerate(task_entries, start=1):
            action, robot_id = entry[0], entry[1]
            process = entry[2] if len(entry) > 2 else "assemble"
            area = entry[3] if len(entry) > 3 else "assembly_area"
            tasks[action] = self._coordinated_task(
                action, robot_id, index, order_id, process, area
            )

        with self._execution_lock:
            self._connect()
            r1 = self._controller_for("R1")
            r2 = self._controller_for("R2")
            r3 = self._controller_for("R3")
            for controller in (r1, r2, r3):
                setter = getattr(controller, "set_continuous_stepping", None)
                if setter is not None:
                    setter(True)
            for controller in (r2, r3):
                setter = getattr(controller, "set_coordinated_mode", None)
                if setter is not None:
                    setter(True)

            pcb_done = threading.Event()
            module_done = threading.Event()
            records: dict[str, dict[str, Any]] = {}
            errors: dict[str, str] = {}
            result_lock = threading.Lock()

            def wait_for_pcb() -> None:
                pcb_done.wait()
                error = errors.get(R2_PCB_PLACED)
                if error:
                    raise RuntimeError(
                        f"R3 module placement blocked because R2 failed: {error}"
                    )

            def wait_for_module() -> None:
                module_done.wait()
                error = errors.get(R3_MODULE_PLACED)
                if error:
                    raise RuntimeError(
                        f"R1 terminal placement blocked because R3 failed: {error}"
                    )

            r3.set_assembly_entry_wait(R3_MODULE_PLACED, wait_for_pcb)
            r1.set_assembly_entry_wait(R1_TERMINAL_PLACED, wait_for_module)

            def run_action(action: str, controller: Any) -> None:
                task = tasks[action]
                robot_id = task.available_robots[0]
                start_wall = time.time()
                start_sim = float(self._bridge.sim.getSimulationTime())
                with self._state_lock:
                    state = self._robots[robot_id]
                    state.status = RobotStatus.BUSY.value
                    state.current_task = task.task_id
                status = TaskStatus.FINISHED.value
                message = ""
                details: dict[str, Any] | None = None
                try:
                    details = controller.execute(action)
                    message = (
                        f"{action} completed ({self._grasp_note(robot_id)}); "
                        f"{details}"
                    )
                except Exception as exc:
                    status = TaskStatus.FAILED.value
                    message = str(exc)
                    with result_lock:
                        errors[action] = message
                finally:
                    end_wall = time.time()
                    end_sim = float(self._bridge.sim.getSimulationTime())
                    result = TaskResult(
                        task_id=task.task_id,
                        robot_id=robot_id,
                        status=status,
                        start_time=start_wall,
                        end_time=end_wall,
                        message=message,
                    )
                    with self._state_lock:
                        state = self._robots[robot_id]
                        if state.status != RobotStatus.FAULT.value:
                            state.status = RobotStatus.IDLE.value
                        state.current_task = None
                        if status == TaskStatus.FINISHED.value:
                            state.completed_tasks += 1
                            if action == R1_BOX_PLACED:
                                state.position = "R1_TERMINAL_PICK_APP"
                            elif action == R2_PCB_PLACED:
                                state.position = "R2_PCB_PICK_APP"
                            elif action == R3_MODULE_PLACED:
                                state.position = (
                                    "R3_TEMP_CLEAR_FOR_R1_TERMINAL_BEFORE_PRODUCT_PICK_APP"
                                )
                            elif action == R3_PRODUCT_TO_INSPECTION:
                                state.position = R3_PRODUCT_TRANSFER_CLEARANCE
                            else:
                                state.position = "home"
                    record = {
                        "task": task.to_dict(),
                        "coordinated_front_half": True,
                        "start_wall_epoch_s": start_wall,
                        "end_wall_epoch_s": end_wall,
                        "start_simulation_time_s": start_sim,
                        "end_simulation_time_s": end_sim,
                        "wall_duration_s": end_wall - start_wall,
                        "simulation_duration_s": max(0.0, end_sim - start_sim),
                        "motion_timing": {
                            "robot_id": robot_id,
                            "motion_detected": status == TaskStatus.FINISHED.value,
                            "monitor_error": "coordinated front-half direct run",
                        },
                        "result": result.to_dict(),
                    }
                    if details is not None:
                        record["controller_details"] = details
                    with result_lock:
                        records[action] = record
                    if action == R2_PCB_PLACED:
                        pcb_done.set()
                    if action == R3_MODULE_PLACED:
                        module_done.set()

            def start_thread(action: str, controller: Any) -> threading.Thread:
                thread = threading.Thread(
                    target=run_action,
                    args=(action, controller),
                    name=f"coordinated-{action}",
                    daemon=True,
                )
                thread.start()
                return thread

            try:
                r1_box = start_thread(R1_BOX_PLACED, r1)
                r2_pcb = start_thread(R2_PCB_PLACED, r2)
                r3_module = start_thread(R3_MODULE_PLACED, r3)

                r1_box.join()
                r1_terminal: threading.Thread | None = None
                if R1_BOX_PLACED not in errors:
                    r1_terminal = start_thread(R1_TERMINAL_PLACED, r1)

                r2_pcb.join()
                r3_module.join()
                if r1_terminal is not None:
                    r1_terminal.join()

                if not errors:
                    run_action(R3_PRODUCT_TO_INSPECTION, r3)
            finally:
                pcb_done.set()
                module_done.set()
                r3.set_assembly_entry_wait(R3_MODULE_PLACED, None)
                r1.set_assembly_entry_wait(R1_TERMINAL_PLACED, None)
                for controller in (r2, r3):
                    setter = getattr(controller, "set_coordinated_mode", None)
                    if setter is not None:
                        setter(False)

            ordered_records = [
                records[action]
                for action in (
                    R1_BOX_PLACED,
                    R2_PCB_PLACED,
                    R3_MODULE_PLACED,
                    R1_TERMINAL_PLACED,
                    R3_PRODUCT_TO_INSPECTION,
                )
                if action in records
            ]
            failed_action = next(
                (
                    action
                    for action in (
                        R1_BOX_PLACED,
                        R2_PCB_PLACED,
                        R3_MODULE_PLACED,
                        R1_TERMINAL_PLACED,
                        R3_PRODUCT_TO_INSPECTION,
                    )
                    if action in errors
                ),
                None,
            )
            return {
                "status": "failed" if failed_action else "finished",
                "tasks": ordered_records,
                "failed_action": failed_action,
                "errors": dict(errors),
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
                if action == R1_BOX_PLACED:
                    state.position = "R1_TERMINAL_PICK_APP"
                elif action == R4_SCREW_DONE:
                    state.position = R4_WAIT_POINT
                elif action == R3_PRODUCT_TO_INSPECTION:
                    state.position = R3_PRODUCT_TRANSFER_CLEARANCE
                else:
                    state.position = "home"
                state.completed_tasks += 1
            self._last_error = ""
            return self._result(
                task,
                robot_id,
                TaskStatus.FINISHED.value,
                start_time,
                f"{action} completed ({self._grasp_note(robot_id)}); {details}",
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
