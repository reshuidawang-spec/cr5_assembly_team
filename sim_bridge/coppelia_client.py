"""Safe CoppeliaSim ZMQ bridge for the current five-CR5A scene."""

from __future__ import annotations

import hashlib
import importlib
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from interfaces.sim_interface import ISimBridge
from scheduler.config_loader import load_yaml
from sim_bridge.scene_objects import (
    ARM_JOINT_ALIASES,
    POINTS,
    PROCESS_SIGNAL_UPDATES,
    QUALITY_COMMANDS,
    ROBOT_IDS,
    ROBOT_ROOTS,
    SCENE_ROOT,
    TOOL_COMMANDS,
    get_tip_alias,
    normalize_robot_id,
    resolve_object_path,
)


ClientFactory = Callable[..., Any]
REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONTRACT = REPO_ROOT / "configs" / "scene_contract.yaml"


def _remote_api_client_class() -> type:
    """Load the installed or CoppeliaSim-bundled ZMQ client."""
    module_name = "coppeliasim_zmqremoteapi_client"
    try:
        return importlib.import_module(module_name).RemoteAPIClient
    except ImportError:
        pass

    roots = []
    configured_root = os.environ.get("COPPELIASIM_ROOT")
    if configured_root:
        roots.append(Path(configured_root).expanduser())
    roots.extend(
        [
            Path("/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04"),
            Path("/opt/CoppeliaSim"),
            Path("/opt/coppeliasim"),
            Path.home() / "CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04",
            Path.home() / "CoppeliaSim",
        ]
    )
    for root in roots:
        client_path = root / "programming/zmqRemoteApi/clients/python/src"
        if not client_path.is_dir():
            continue
        path_text = str(client_path)
        if path_text not in sys.path:
            sys.path.insert(0, path_text)
        try:
            return importlib.import_module(module_name).RemoteAPIClient
        except ImportError:
            continue
    raise RuntimeError(
        "CoppeliaSim ZMQ client is unavailable; install "
        "coppeliasim-zmqremoteapi-client or set COPPELIASIM_ROOT"
    )


def _fingerprint(path: Path) -> dict[str, Any]:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return {"size": path.stat().st_size, "sha256": digest.hexdigest()}


class SimBridge(ISimBridge):
    """Low-level scene communication with a strict scene-contract gate.

    This class deliberately refuses Cartesian teleportation and unvalidated
    inverse kinematics. Joint motion is available only as a low-level primitive;
    high-level executors must independently validate their paths.
    """

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 23000,
        client_factory: Optional[ClientFactory] = None,
        contract_path: Path | str = DEFAULT_CONTRACT,
        validate_contract: bool = True,
        request_timeout: float = 5.0,
    ):
        self.host = host
        self.port = int(port)
        self.contract_path = Path(contract_path)
        self.validate_contract_on_connect = bool(validate_contract)
        self.request_timeout = max(float(request_timeout), 0.5)
        self._client_factory = client_factory
        self._client: Any = None
        self._sim: Any = None
        self._connected = False
        self._stepping = False
        self._joint_cache: dict[str, list[int]] = {}
        self._last_error = ""
        self._contract_report: dict[str, Any] = {}

    @property
    def last_error(self) -> str:
        return self._last_error

    @property
    def sim(self) -> Any:
        self._require_connected()
        return self._sim

    @property
    def stepping_enabled(self) -> bool:
        return self._stepping

    @property
    def contract_report(self) -> dict[str, Any]:
        return dict(self._contract_report)

    def connect(self, host: str = "127.0.0.1", port: int = 23000) -> bool:
        self.host = host
        self.port = int(port)
        self._last_error = ""
        try:
            factory = self._client_factory or _remote_api_client_class()
            self._client = factory(host=self.host, port=self.port)
            self._configure_client_timeout()
            self._sim = self._client.require("sim")
            self._sim.getSimulationState()
            self._sim.getObject(SCENE_ROOT)
            self._connected = True
            self._joint_cache.clear()
            if self.validate_contract_on_connect:
                self._contract_report = self.validate_scene_contract()
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self.disconnect()
            return False

    def disconnect(self) -> None:
        if self._stepping and self._client is not None:
            try:
                self._client.setStepping(False)
            except Exception:
                pass
        self._joint_cache.clear()
        self._stepping = False
        self._connected = False
        self._sim = None
        if self._client is not None:
            socket = getattr(self._client, "socket", None)
            context = getattr(self._client, "context", None)
            if socket is not None:
                try:
                    socket.close(linger=0)
                except Exception:
                    pass
            if context is not None:
                try:
                    context.term()
                except Exception:
                    pass
        self._client = None

    def _configure_client_timeout(self) -> None:
        """Bound Remote API waits so a stale endpoint cannot freeze the GUI."""
        if self._client is None:
            return
        self._client.timeout = self.request_timeout
        socket = getattr(self._client, "socket", None)
        if socket is None:
            return
        try:
            import zmq

            timeout_ms = int(self.request_timeout * 1000)
            socket.setsockopt(zmq.RCVTIMEO, timeout_ms)
            socket.setsockopt(zmq.SNDTIMEO, timeout_ms)
            socket.setsockopt(zmq.LINGER, 0)
        except (ImportError, AttributeError):
            pass

    def is_connected(self) -> bool:
        if not self._connected or self._sim is None:
            return False
        try:
            self._sim.getSimulationState()
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self._connected = False
            return False

    def _require_connected(self) -> None:
        if not self._connected or self._sim is None:
            raise RuntimeError("SimBridge is not connected")

    def _robot_root(self, robot_id: str) -> int:
        normalized = normalize_robot_id(robot_id)
        return int(self._sim.getObject(ROBOT_ROOTS[normalized]))

    def _find_unique_alias(
        self,
        root: int,
        alias: str,
        object_type: Optional[int] = None,
    ) -> int:
        selected_type = (
            self._sim.handle_all if object_type is None else object_type
        )
        matches = [
            handle
            for handle in self._sim.getObjectsInTree(root, selected_type, 0)
            if self._sim.getObjectAlias(handle) == alias
        ]
        if len(matches) != 1:
            raise RuntimeError(f"expected one {alias}, found {len(matches)}")
        return int(matches[0])

    def _arm_joints(self, robot_id: str) -> list[int]:
        normalized = normalize_robot_id(robot_id)
        cached = self._joint_cache.get(normalized)
        if cached:
            return list(cached)
        root = self._robot_root(normalized)
        by_alias = {
            self._sim.getObjectAlias(handle): int(handle)
            for handle in self._sim.getObjectsInTree(
                root, self._sim.object_joint_type, 0
            )
            if self._sim.getObjectAlias(handle) in ARM_JOINT_ALIASES
        }
        if set(by_alias) != set(ARM_JOINT_ALIASES):
            raise RuntimeError(
                f"{normalized} arm joints are incomplete: {sorted(by_alias)}"
            )
        joints = [by_alias[alias] for alias in ARM_JOINT_ALIASES]
        self._joint_cache[normalized] = joints
        return list(joints)

    def validate_scene_contract(self) -> dict[str, Any]:
        """Validate the open scene without moving any object."""
        self._require_connected()
        contract = load_yaml(self.contract_path)
        expected_scene = contract["scene"]
        live_path = Path(
            self._sim.getStringParam(
                self._sim.stringparam_scene_path_and_name
            )
        )
        if not live_path.is_file():
            raise RuntimeError(f"open scene file is unavailable: {live_path}")
        actual_fingerprint = _fingerprint(live_path)
        expected_fingerprint = {
            "size": int(expected_scene["size"]),
            "sha256": str(expected_scene["sha256"]),
        }
        if actual_fingerprint != expected_fingerprint:
            raise RuntimeError(
                "scene fingerprint mismatch: "
                f"expected {expected_fingerprint}, got {actual_fingerprint}"
            )

        self._sim.getObject(SCENE_ROOT)
        for path in POINTS.values():
            self._sim.getObject(path)
        robots = {}
        for robot_id in ROBOT_IDS:
            joints = self._arm_joints(robot_id)
            root = self._robot_root(robot_id)
            tip = self._find_unique_alias(root, get_tip_alias(robot_id))
            robots[robot_id] = {
                "joint_count": len(joints),
                "tip": self._sim.getObjectAlias(tip),
            }
        return {
            "scene_path": str(live_path),
            **actual_fingerprint,
            "root": SCENE_ROOT,
            "target_count": len(POINTS),
            "robots": robots,
        }

    def get_object_handle(self, name: str) -> int:
        self._require_connected()
        return int(self._sim.getObject(resolve_object_path(name)))

    def get_object_handles(self, names: List[str]) -> Dict[str, int]:
        return {name: self.get_object_handle(name) for name in names}

    def get_robot_joint_handles(self, robot_id: str) -> list[int]:
        self._require_connected()
        return self._arm_joints(robot_id)

    def get_robot_joint_positions(self, robot_id: str) -> list[float]:
        self._require_connected()
        return [
            float(self._sim.getJointPosition(handle))
            for handle in self._arm_joints(robot_id)
        ]

    def move_robot_joints(
        self, robot_id: str, joint_angles: List[float]
    ) -> bool:
        self._require_connected()
        if len(joint_angles) != 6 or not all(
            math.isfinite(float(value)) for value in joint_angles
        ):
            self._last_error = "joint_angles must contain six finite radians"
            return False
        try:
            state = self._sim.getSimulationState()
            setter = (
                self._sim.setJointPosition
                if state == self._sim.simulation_stopped
                else self._sim.setJointTargetPosition
            )
            for handle, angle in zip(
                self._arm_joints(robot_id), joint_angles
            ):
                setter(handle, float(angle))
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def move_robot_pose(
        self,
        robot_id: str,
        x: float,
        y: float,
        z: float,
        roll: float = 0,
        pitch: float = 0,
        yaw: float = 0,
    ) -> bool:
        normalize_robot_id(robot_id)
        self._last_error = (
            "Cartesian motion requires a collision-checked executor validated "
            "for the current scene fingerprint"
        )
        return False

    def get_robot_pose(self, robot_id: str) -> Optional[Dict]:
        self._require_connected()
        try:
            root = self._robot_root(robot_id)
            tip = self._find_unique_alias(root, get_tip_alias(robot_id))
            position = self._sim.getObjectPosition(tip, -1)
            orientation = self._sim.getObjectOrientation(tip, -1)
            return {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "roll": float(orientation[0]),
                "pitch": float(orientation[1]),
                "yaw": float(orientation[2]),
                "quaternion": [
                    float(value)
                    for value in self._sim.getObjectQuaternion(tip, -1)
                ],
                "tip": get_tip_alias(robot_id),
            }
        except Exception as exc:
            self._last_error = str(exc)
            return None

    def get_target_pose(self, target_name: str) -> Dict[str, list[float]]:
        target = self.get_object_handle(target_name)
        return {
            "position": [
                float(value)
                for value in self._sim.getObjectPosition(target, -1)
            ],
            "orientation": [
                float(value)
                for value in self._sim.getObjectOrientation(target, -1)
            ],
            "quaternion": [
                float(value)
                for value in self._sim.getObjectQuaternion(target, -1)
            ],
        }

    def set_string_signal(self, name: str, value: str) -> None:
        self._require_connected()
        self._sim.setStringSignal(str(name), str(value))

    def get_string_signal(self, name: str) -> Optional[str]:
        self._require_connected()
        return self._sim.getStringSignal(str(name))

    def clear_string_signal(self, name: str) -> None:
        self._require_connected()
        self._sim.clearStringSignal(str(name))

    def send_process_command(self, command: str) -> None:
        normalized = str(command).strip().upper()
        try:
            signal, value = PROCESS_SIGNAL_UPDATES[normalized]
        except KeyError as exc:
            raise ValueError(f"unsupported process command: {command}") from exc
        self.set_string_signal(signal, value)
        if self._stepping:
            self.step()

    def send_quality_result(self, quality: str) -> None:
        normalized = str(quality).strip().upper()
        try:
            command = QUALITY_COMMANDS[normalized]
        except KeyError as exc:
            raise ValueError(f"unsupported quality result: {quality}") from exc
        self.send_process_command(command)

    def send_tool_command(self, command: str) -> None:
        normalized = str(command).strip().upper()
        if normalized not in TOOL_COMMANDS:
            raise ValueError(f"unsupported tool command: {command}")
        if self._sim.getSimulationState() == self._sim.simulation_stopped:
            raise RuntimeError("tool commands require a running simulation")
        self.set_string_signal("tool_cmd", normalized)
        if self._stepping:
            self.step()

    def set_gripper(self, robot_id: str, open: bool) -> bool:
        self._require_connected()
        normalized = normalize_robot_id(robot_id)
        if normalized == "R2":
            command = "R2_VACUUM_OFF" if open else "R2_VACUUM_ON"
        elif normalized in {"R1", "R3", "R5"}:
            command = f"{normalized}_GRIPPER_{'OPEN' if open else 'CLOSE'}"
        else:
            self._last_error = f"{normalized} has no gripper"
            return False
        try:
            self.send_tool_command(command)
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def set_gripper_gap(self, robot_id: str, gap_m: float) -> bool:
        """Set the visible parallel-jaw opening for a specific workpiece."""
        self._require_connected()
        normalized = normalize_robot_id(robot_id)
        if normalized not in {"R1", "R3", "R5"}:
            self._last_error = f"{normalized} has no parallel gripper"
            return False
        gap = float(gap_m)
        if not 0.0 <= gap <= 0.18:
            self._last_error = f"invalid {normalized} gripper gap: {gap}"
            return False
        try:
            robot = self._robot_root(normalized)
            tool = self._find_unique_alias(robot, f"{normalized}T")
            left = self._find_unique_alias(
                robot, f"{normalized}T_left_finger_link"
            )
            right = self._find_unique_alias(
                robot, f"{normalized}T_right_finger_link"
            )
            target_y = (gap + 0.020) / 2.0
            left_position = list(self._sim.getObjectPosition(left, tool))
            right_position = list(self._sim.getObjectPosition(right, tool))
            left_position[1] = target_y
            right_position[1] = -target_y
            self._sim.setObjectPosition(left, tool, left_position)
            self._sim.setObjectPosition(right, tool, right_position)
            self._last_error = ""
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def freeze_gripper(self, robot_id: str) -> bool:
        """Native scene tools are position-controlled; nothing to freeze."""
        normalize_robot_id(robot_id)
        return True

    def unfreeze_gripper(self, robot_id: str) -> bool:
        """Native scene tools are position-controlled; nothing to unfreeze."""
        normalize_robot_id(robot_id)
        return True

    def set_visual_owner(self, owner: str = "executor") -> None:
        if owner not in {"executor", "template"}:
            raise ValueError(f"unsupported visual owner: {owner}")
        self.set_string_signal("cell_visual_owner", owner)

    def get_visual_owner(self) -> Optional[str]:
        return self.get_string_signal("cell_visual_owner")

    def set_object_parent(
        self, object_name: str, parent_name: str, keep_in_place: bool = True
    ) -> None:
        self._require_connected()
        child = self.get_object_handle(object_name)
        parent = self.get_object_handle(parent_name)
        self._sim.setObjectParent(child, parent, bool(keep_in_place))

    def attach_object(self, object_name: str, robot_id: str) -> None:
        self._require_connected()
        normalized = normalize_robot_id(robot_id)
        robot = self._robot_root(normalized)
        tip = self._find_unique_alias(robot, get_tip_alias(normalized))
        child = self.get_object_handle(object_name)
        world_pose = [
            float(value) for value in self._sim.getObjectPose(child, -1)
        ]
        self._sim.setObjectParent(child, tip, True)
        # Parenting is only a rigid attachment operation.  Never let it snap
        # a workpiece to the TCP: the robot must already be at the grasp pose.
        self._sim.setObjectPose(child, -1, world_pose)
        after = [float(value) for value in self._sim.getObjectPose(child, -1)]
        position_error = max(
            abs(after[index] - world_pose[index]) for index in range(3)
        )
        if position_error > 1e-6:
            raise RuntimeError(
                f"{normalized} attachment moved {object_name} by "
                f"{position_error * 1000.0:.3f} mm"
            )

    def detach_object(
        self, object_name: str | int, parent_name: str = "PARTS_ROOT"
    ) -> None:
        self._require_connected()
        if parent_name == "PARTS_ROOT":
            parent = self._sim.getObject(f"{SCENE_ROOT}/Parts")
        else:
            parent = self.get_object_handle(parent_name)
        child = (
            int(object_name)
            if isinstance(object_name, int) and not isinstance(object_name, bool)
            else self.get_object_handle(str(object_name))
        )
        self._sim.setObjectParent(child, parent, True)

    def start_simulation(self) -> bool:
        self._require_connected()
        last_error = ""
        for attempt in range(6):
            try:
                # All repository motion controllers advance one deterministic
                # CoppeliaSim step at a time.  Enabling stepping here makes the
                # bridge contract consistent for READY, scene replay and motion.
                self.set_stepping(True)
                if self._sim.getSimulationState() == self._sim.simulation_stopped:
                    self._sim.startSimulation()
                self._last_error = ""
                return True
            except Exception as exc:
                last_error = str(exc)
                if "temporarily unavailable" not in last_error.lower():
                    break
                time.sleep(0.5 + attempt * 0.25)
        self._last_error = last_error
        return False

    def set_stepping(self, enabled: bool) -> None:
        self._require_connected()
        enabled = bool(enabled)
        if self._stepping == enabled:
            return
        self._client.setStepping(enabled)
        self._stepping = enabled

    def stop_simulation(self) -> bool:
        self._require_connected()
        try:
            if self._sim.getSimulationState() != self._sim.simulation_stopped:
                self._sim.stopSimulation()
                deadline = time.monotonic() + 10.0
                while (
                    self._sim.getSimulationState()
                    != self._sim.simulation_stopped
                    and time.monotonic() < deadline
                ):
                    if self._stepping:
                        self._client.step()
                    time.sleep(0.05)
            stopped = (
                self._sim.getSimulationState()
                == self._sim.simulation_stopped
            )
            if self._stepping:
                self.set_stepping(False)
            if not stopped:
                self._last_error = "simulation did not stop within 10 seconds"
            return stopped
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def step(self) -> bool:
        self._require_connected()
        if not self._stepping:
            self._last_error = "stepping mode is not enabled"
            return False
        try:
            self._client.step()
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def scene_path(self) -> str:
        self._require_connected()
        configured = os.environ.get("CR5_SCENE_PATH")
        if configured:
            return str(Path(configured))
        if os.environ.get("CR5_SKIP_SCENE_FINGERPRINT") == "1":
            fallback = REPO_ROOT / "scenes" / "compact_cell1ttt.ttt"
            if fallback.exists():
                return str(fallback)
        try:
            path = self._sim.getStringParam(
                self._sim.stringparam_scene_path_and_name
            )
            if path:
                return str(path)
        except Exception:
            pass
        fallback = REPO_ROOT / "scenes" / "compact_cell1ttt.ttt"
        if fallback.exists():
            return str(fallback)
        return str(REPO_ROOT / "scenes" / "compact_cell.ttt")

    def __enter__(self) -> "SimBridge":
        if not self.connect(self.host, self.port):
            raise RuntimeError(self.last_error or "failed to connect")
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.disconnect()
