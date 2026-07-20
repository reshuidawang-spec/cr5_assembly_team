"""Real CoppeliaSim ZMQ implementation of :class:`ISimBridge`.

The public class name remains ``SimBridge`` as required by the integration
contract. High-level collision-free Cartesian planning belongs to
``IRobotExecutor``; this module deliberately exposes only verified low-level
scene communication and does not claim that an unimplemented pose move worked.
"""

from __future__ import annotations

import importlib
import math
import os
import sys
import time
from pathlib import Path
from typing import Any, Callable, Dict, List, Optional

from interfaces.sim_interface import ISimBridge
from sim_bridge.scene_objects import (
    ARM_JOINT_ALIASES,
    PARTS,
    ROBOT_ROOTS,
    ROBOT_TOOL_ROOTS,
    SCENE_ROOT,
    get_tip_alias,
    normalize_robot_id,
    resolve_object_path,
)


ClientFactory = Callable[..., Any]


def _remote_api_client_class() -> type:
    """Load the pip package or CoppeliaSim's bundled Python client."""
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
            Path.home() / "CoppeliaSim",
            Path("/opt/CoppeliaSim"),
            Path("/opt/coppeliasim"),
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


class SimBridge(ISimBridge):
    """Five-CR5A scene communication over CoppeliaSim ZMQ Remote API."""

    def __init__(
        self,
        host: str = "127.0.0.1",
        port: int = 23000,
        client_factory: Optional[ClientFactory] = None,
    ):
        self.host = host
        self.port = int(port)
        self._client_factory = client_factory
        self._client: Any = None
        self._sim: Any = None
        self._connected = False
        self._stepping = False
        self._joint_cache: dict[str, list[int]] = {}
        self._last_error = ""

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

    def connect(self, host: str = "127.0.0.1", port: int = 23000) -> bool:
        self.host = host
        self.port = int(port)
        self._last_error = ""
        try:
            factory = self._client_factory or _remote_api_client_class()
            self._client = factory(host=self.host, port=self.port)
            self._sim = self._client.require("sim")
            self._sim.getSimulationState()
            self._sim.getObject(SCENE_ROOT)
            self._connected = True
            self._joint_cache.clear()
            self.set_visual_owner("executor")
            return True
        except Exception as exc:
            self._last_error = str(exc)
            self._connected = False
            self._client = None
            self._sim = None
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
        self._client = None

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
        robot_id = normalize_robot_id(robot_id)
        return self._sim.getObject(ROBOT_ROOTS[robot_id])

    def _find_unique_alias(
        self,
        root: int,
        alias: str,
        object_type: Optional[int] = None,
    ) -> int:
        object_type = self._sim.handle_all if object_type is None else object_type
        matches = [
            handle
            for handle in self._sim.getObjectsInTree(root, object_type, 0)
            if self._sim.getObjectAlias(handle) == alias
        ]
        if len(matches) != 1:
            root_path = self._sim.getObjectAlias(root, 1)
            raise RuntimeError(
                f"expected one {alias} below {root_path}, found {len(matches)}"
            )
        return matches[0]

    def _arm_joints(self, robot_id: str) -> list[int]:
        robot_id = normalize_robot_id(robot_id)
        cached = self._joint_cache.get(robot_id)
        if cached:
            try:
                aliases = [self._sim.getObjectAlias(handle) for handle in cached]
                if aliases == list(ARM_JOINT_ALIASES):
                    return list(cached)
            except Exception:
                self._joint_cache.pop(robot_id, None)

        robot = self._robot_root(robot_id)
        by_alias = {
            self._sim.getObjectAlias(handle): handle
            for handle in self._sim.getObjectsInTree(
                robot, self._sim.object_joint_type, 0
            )
            if self._sim.getObjectAlias(handle) in ARM_JOINT_ALIASES
        }
        if set(by_alias) != set(ARM_JOINT_ALIASES):
            raise RuntimeError(
                f"{robot_id} arm joints are incomplete: {sorted(by_alias)}"
            )
        joints = [by_alias[alias] for alias in ARM_JOINT_ALIASES]
        self._joint_cache[robot_id] = joints
        return list(joints)

    def get_object_handle(self, name: str) -> int:
        self._require_connected()
        return int(self._sim.getObject(resolve_object_path(name)))

    def get_object_handles(self, names: List[str]) -> Dict[str, int]:
        return {name: self.get_object_handle(name) for name in names}

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
            joints = self._arm_joints(robot_id)
            state = self._sim.getSimulationState()
            setter = (
                self._sim.setJointPosition
                if state == self._sim.simulation_stopped
                else self._sim.setJointTargetPosition
            )
            for joint, angle in zip(joints, joint_angles):
                setter(joint, float(angle))
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def get_robot_joint_positions(self, robot_id: str) -> list[float]:
        self._require_connected()
        return [
            float(self._sim.getJointPosition(joint))
            for joint in self._arm_joints(robot_id)
        ]

    def get_robot_joint_handles(self, robot_id: str) -> list[int]:
        """Return the recursively discovered six arm joints in axis order."""
        self._require_connected()
        return self._arm_joints(robot_id)

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
            "move_robot_pose requires the collision-checked RobotExecutor; "
            "SimBridge will not teleport or run unvalidated IK"
        )
        return False

    def get_robot_pose(self, robot_id: str) -> Optional[Dict]:
        self._require_connected()
        try:
            robot = self._robot_root(robot_id)
            tip = self._find_unique_alias(robot, get_tip_alias(robot_id))
            position = self._sim.getObjectPosition(tip, -1)
            orientation = self._sim.getObjectOrientation(tip, -1)
            quaternion = self._sim.getObjectQuaternion(tip, -1)
            return {
                "x": float(position[0]),
                "y": float(position[1]),
                "z": float(position[2]),
                "roll": float(orientation[0]),
                "pitch": float(orientation[1]),
                "yaw": float(orientation[2]),
                "quaternion": [float(value) for value in quaternion],
                "tip": get_tip_alias(robot_id),
            }
        except Exception as exc:
            self._last_error = str(exc)
            return None

    def get_target_pose(self, target_name: str) -> Dict[str, list[float]]:
        target = self.get_object_handle(target_name)
        return {
            "position": [
                float(value) for value in self._sim.getObjectPosition(target, -1)
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

    def set_gripper(self, robot_id: str, open: bool) -> bool:
        self._require_connected()
        robot_id = normalize_robot_id(robot_id)
        tool_path = ROBOT_TOOL_ROOTS.get(robot_id)
        if tool_path is None:
            self._last_error = f"no calibrated gripper is registered for {robot_id}"
            return False
        try:
            if self._sim.getSimulationState() == self._sim.simulation_stopped:
                self._last_error = "gripper scripts require a running simulation"
                return False
            tool = self._sim.getObject(tool_path)
            scripts = self._sim.getObjectsInTree(
                tool, self._sim.object_script_type, 0
            )
            if len(scripts) != 1:
                raise RuntimeError(
                    f"expected one gripper script below {tool_path}, "
                    f"found {len(scripts)}"
                )
            # The stock Robotiq script applies velocity continuously and has
            # no open-position limit.  Coordinated runs freeze it after each
            # short animation, so re-enable it explicitly for the next
            # command.
            was_enabled = bool(
                self._sim.getObjectInt32Param(
                    scripts[0], self._sim.scriptintparam_enabled
                )
            )
            self._sim.setObjectInt32Param(
                scripts[0], self._sim.scriptintparam_enabled, 1
            )
            if not was_enabled and self._stepping and not self.step():
                raise RuntimeError(
                    self._last_error or "cannot initialize re-enabled gripper script"
                )
            function_name = "openClicked" if open else "closeClicked"
            mode = 1 if open else 2
            self._sim.callScriptFunction(function_name, scripts[0], 0, mode)
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def freeze_gripper(self, robot_id: str) -> bool:
        """Stop the stock gripper's unbounded velocity after visual motion."""
        self._require_connected()
        robot_id = normalize_robot_id(robot_id)
        tool_path = ROBOT_TOOL_ROOTS.get(robot_id)
        if tool_path is None:
            self._last_error = f"no calibrated gripper is registered for {robot_id}"
            return False
        try:
            tool = self._sim.getObject(tool_path)
            scripts = self._sim.getObjectsInTree(
                tool, self._sim.object_script_type, 0
            )
            if len(scripts) != 1:
                raise RuntimeError(
                    f"expected one gripper script below {tool_path}, "
                    f"found {len(scripts)}"
                )
            active = [
                handle
                for handle in self._sim.getObjectsInTree(
                    tool, self._sim.object_joint_type, 0
                )
                if self._sim.getObjectAlias(handle) in {"active1", "active2"}
            ]
            if len(active) != 2:
                raise RuntimeError(
                    f"expected active1/active2 below {tool_path}, found {len(active)}"
                )
            self._sim.setObjectInt32Param(
                scripts[0], self._sim.scriptintparam_enabled, 0
            )
            for joint in active:
                self._sim.setJointTargetVelocity(joint, 0.0)
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def start_simulation(self) -> bool:
        self._require_connected()
        try:
            self.set_stepping(True)
            if self._sim.getSimulationState() == self._sim.simulation_stopped:
                self._sim.startSimulation()
            return True
        except Exception as exc:
            self._last_error = str(exc)
            return False

    def set_stepping(self, enabled: bool) -> None:
        """Let an executor take or release deterministic stepping control."""
        self._require_connected()
        enabled = bool(enabled)
        # CoppeliaSim counts stepping-enable requests. Repeating True and
        # releasing it only once leaves a running simulation permanently
        # waiting for another step, so this operation must be idempotent per
        # client.
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
                    time.sleep(0.05)
            stopped = (
                self._sim.getSimulationState() == self._sim.simulation_stopped
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

    def set_string_signal(self, name: str, value: str) -> None:
        self._require_connected()
        self._sim.setStringSignal(name, value)

    def get_string_signal(self, name: str) -> Optional[str]:
        self._require_connected()
        return self._sim.getStringSignal(name)

    def clear_string_signal(self, name: str) -> None:
        self._require_connected()
        self._sim.clearStringSignal(name)

    def set_visual_owner(self, owner: str = "executor") -> None:
        if owner not in {"executor", "template"}:
            raise ValueError(f"unsupported visual owner: {owner}")
        self.set_string_signal("cell_visual_owner", owner)

    def get_visual_owner(self) -> Optional[str]:
        return self.get_string_signal("cell_visual_owner")

    def set_object_parent(
        self, object_name: str, parent_name: str, keep_in_place: bool = True
    ) -> None:
        child = self.get_object_handle(object_name)
        parent = self.get_object_handle(parent_name)
        self._sim.setObjectParent(child, parent, bool(keep_in_place))

    def attach_object(self, object_name: str, robot_id: str) -> None:
        robot_id = normalize_robot_id(robot_id)
        robot = self._robot_root(robot_id)
        tip = self._find_unique_alias(robot, get_tip_alias(robot_id))
        child = self.get_object_handle(object_name)
        self._sim.setObjectParent(child, tip, True)

    def detach_object(
        self, object_name: str | int, parent_name: str = "PARTS_ROOT"
    ) -> None:
        # Physical supply objects remain real scene objects below /Parts.
        # A caller must explicitly request another owner when that is intended.
        if parent_name == "PARTS_ROOT":
            parent = self._sim.getObject(f"{SCENE_ROOT}/Parts")
        else:
            parent = self.get_object_handle(parent_name)
        # Once attached, a part's original absolute path no longer resolves.
        # Motion code therefore retains and passes its stable scene handle.
        child = (
            int(object_name)
            if isinstance(object_name, int) and not isinstance(object_name, bool)
            else self.get_object_handle(str(object_name))
        )
        self._sim.setObjectParent(child, parent, True)

    def scene_path(self) -> str:
        self._require_connected()
        return str(
            self._sim.getStringParam(self._sim.stringparam_scene_path_and_name)
        )

    def __enter__(self) -> "SimBridge":
        if not self.connect(self.host, self.port):
            raise RuntimeError(self.last_error or "failed to connect to CoppeliaSim")
        return self

    def __exit__(self, exc_type: Any, exc: Any, traceback: Any) -> None:
        self.disconnect()
