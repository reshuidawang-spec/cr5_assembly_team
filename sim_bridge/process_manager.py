"""Launch and discover the project CoppeliaSim scene from the GUI."""

from __future__ import annotations

import os
import signal
import socket
import subprocess
import time
from pathlib import Path
from typing import Callable, Optional

from scheduler.config_loader import load_yaml


REPO_ROOT = Path(__file__).resolve().parents[1]
DEFAULT_CONFIG = REPO_ROOT / "configs" / "runtime.yaml"


class CoppeliaProcessManager:
    """Own only CoppeliaSim processes launched by the current application."""

    def __init__(
        self,
        config_path: str | Path = DEFAULT_CONFIG,
        popen_factory: Callable = subprocess.Popen,
    ):
        config = load_yaml(config_path).get("coppeliasim", {})
        executable = Path(
            config.get(
                "executable",
                "/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04/"
                "coppeliaSim.sh",
            )
        ).expanduser()
        scene = Path(
            config.get("scene", "scenes/compact_cell.ttt")
        ).expanduser()
        if not scene.is_absolute():
            scene = REPO_ROOT / scene

        self.executable = executable
        self.scene = scene
        self.setup_scripts = [
            Path(item).expanduser()
            for item in config.get("setup_scripts", [])
        ]
        self.host = str(config.get("host", "127.0.0.1"))
        self.port = int(config.get("port", 23000))
        self.startup_timeout = float(config.get("startup_timeout", 30))
        self._popen_factory = popen_factory
        self._process: Optional[subprocess.Popen] = None
        self._process_group_id: Optional[int] = None

    @property
    def process(self) -> Optional[subprocess.Popen]:
        return self._process

    def validate_installation(self) -> None:
        if not self.executable.is_file():
            raise RuntimeError(
                f"CoppeliaSim executable not found: {self.executable}"
            )
        if not self.scene.is_file():
            raise RuntimeError(f"scene file not found: {self.scene}")
        missing_setup = [
            str(path) for path in self.setup_scripts if not path.is_file()
        ]
        if missing_setup:
            raise RuntimeError(
                "environment setup script not found: "
                + ", ".join(missing_setup)
            )

    def is_owned_process_running(self) -> bool:
        if self._process is not None and self._process.poll() is None:
            return True
        if self._process_group_id is None:
            return False
        try:
            os.killpg(self._process_group_id, 0)
            return True
        except (ProcessLookupError, PermissionError):
            return False

    def endpoint_reachable(
        self,
        host: Optional[str] = None,
        port: Optional[int] = None,
        timeout: float = 0.25,
    ) -> bool:
        try:
            with socket.create_connection(
                (host or self.host, int(port or self.port)),
                timeout=timeout,
            ):
                return True
        except OSError:
            return False

    def launch(self) -> subprocess.Popen:
        if self.is_owned_process_running():
            return self._process
        self.validate_installation()
        command = [str(self.executable), str(self.scene)]
        if self.setup_scripts:
            arguments = [
                *(str(path) for path in self.setup_scripts),
                str(self.executable),
                str(self.scene),
            ]
            executable_index = len(self.setup_scripts) + 1
            scene_index = executable_index + 1
            script = "set -e\n" + "\n".join(
                f'source "${index}"'
                for index in range(1, executable_index)
            )
            script += f'\nexec "${executable_index}" "${scene_index}"'
            command = [
                "/bin/bash",
                "-c",
                script,
                "cr5-coppelia-launch",
                *arguments,
            ]
        self._process = self._popen_factory(
            command,
            cwd=str(self.executable.parent),
            start_new_session=True,
        )
        self._process_group_id = getattr(self._process, "pid", None)
        return self._process

    def terminate_owned_process(self) -> None:
        """Terminate only the process launched by this manager."""
        if not self.is_owned_process_running():
            return
        group_id = self._process_group_id
        if group_id is not None:
            try:
                os.killpg(group_id, signal.SIGTERM)
            except ProcessLookupError:
                pass
            deadline = time.monotonic() + 5.0
            while time.monotonic() < deadline:
                try:
                    os.killpg(group_id, 0)
                except ProcessLookupError:
                    break
                time.sleep(0.05)
            else:
                try:
                    os.killpg(group_id, signal.SIGKILL)
                except ProcessLookupError:
                    pass
        elif self._process is not None:
            self._process.terminate()
        self._process = None
        self._process_group_id = None
