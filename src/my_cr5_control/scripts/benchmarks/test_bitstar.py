#!/usr/bin/env python3
"""检查当前 MoveIt OMPL 配置是否已注册现代 benchmark planners。"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path


REQUIRED_CLASSICAL = [
    "RRTConnect",
    "RRTstar",
    "LBTRRT",
    "FMT",
    "BFMT",
    "PRMstar",
]

RECOMMENDED_MODERN = [
    "BITstar",
    "InformedRRTstar",
    "ABITstar",
    "AITstar",
    "EITstar",
]


def repo_root() -> Path:
    """Repo root."""
    return Path(__file__).resolve().parents[2]


def default_config_path() -> Path:
    """Default config path."""
    workspace_src = repo_root().parent
    return workspace_src / "DOBOT_6Axis_ROS2_V4" / "cr5_moveit" / "config" / "ompl_planning.yaml"


def default_moveit_catalog_path() -> Path:
    """Default moveit catalog path."""
    return Path("/opt/ros/humble/share/moveit_configs_utils/default_configs/ompl_defaults.yaml")


def locate_config(explicit_path: str | None) -> Path:
    """Locate config."""
    if explicit_path:
        return Path(explicit_path).expanduser().resolve()
    return default_config_path().resolve()


def locate_moveit_catalog(explicit_path: str | None) -> Path:
    """Locate moveit catalog."""
    if explicit_path:
        return Path(explicit_path).expanduser().resolve()
    return default_moveit_catalog_path().resolve()


def current_moveit_pkg_prefix() -> tuple[Path | None, str]:
    """Current moveit pkg prefix."""
    ros2_path = shutil.which("ros2")
    if ros2_path is None:
        return None, "未找到 ros2 命令，无法解析当前 moveit_planners_ompl 前缀。"

    result = subprocess.run(
        ["ros2", "pkg", "prefix", "moveit_planners_ompl"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return None, f"ros2 pkg prefix moveit_planners_ompl 执行失败: {result.stderr.strip() or 'unknown error'}"

    return Path(result.stdout.strip()).resolve(), "已解析当前 moveit_planners_ompl 前缀。"


def parse_ompl_config(path: Path) -> tuple[list[str], dict[str, list[str]]]:
    """Parse ompl config."""
    registered: list[str] = []
    groups: dict[str, list[str]] = {}

    current_top_level: str | None = None
    in_root_planner_configs = False
    in_group_planner_configs = False

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        indent = len(line) - len(line.lstrip(" "))

        if indent == 0:
            in_root_planner_configs = stripped == "planner_configs:"
            in_group_planner_configs = False
            if stripped.endswith(":") and stripped != "planner_configs:":
                current_top_level = stripped[:-1]
                groups.setdefault(current_top_level, [])
            else:
                current_top_level = None
            continue

        if in_root_planner_configs and indent == 2 and stripped.endswith(":"):
            registered.append(stripped[:-1])
            continue

        if current_top_level is not None and indent == 2 and stripped == "planner_configs:":
            in_group_planner_configs = True
            continue

        if in_group_planner_configs and current_top_level is not None and indent >= 4 and stripped.startswith("- "):
            groups[current_top_level].append(stripped[2:].strip())

    return registered, groups


def parse_root_planner_ids(path: Path) -> list[str]:
    """Parse root planner ids."""
    registered: list[str] = []
    in_root_planner_configs = False

    for raw_line in path.read_text(encoding="utf-8").splitlines():
        line = raw_line.rstrip()
        stripped = line.strip()
        if not stripped or stripped.startswith("#"):
            continue

        indent = len(line) - len(line.lstrip(" "))
        if indent == 0:
            in_root_planner_configs = stripped == "planner_configs:"
            continue

        if in_root_planner_configs and indent == 2 and stripped.endswith(":"):
            registered.append(stripped[:-1])

    return registered


def check_moveit_ompl_installed() -> tuple[bool, str]:
    """Check moveit ompl installed."""
    ros2_path = shutil.which("ros2")
    if ros2_path is None:
        return False, "未找到 ros2 命令，跳过运行时安装检查。"

    result = subprocess.run(
        ["ros2", "pkg", "list"],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return False, f"ros2 pkg list 执行失败: {result.stderr.strip() or 'unknown error'}"
    if "moveit_planners_ompl" in result.stdout.splitlines():
        return True, "检测到 moveit_planners_ompl。"
    return False, "未在 ros2 pkg list 中检测到 moveit_planners_ompl。"


def format_list(items: list[str]) -> str:
    """Format list."""
    return ", ".join(items) if items else "(none)"


def detect_registered_planners_from_library(path: Path, planner_ids: list[str]) -> tuple[list[str], str]:
    """Detect registered planners from library."""
    if not path.exists():
        return [], "当前 moveit_ompl_interface 动态库不存在。"

    strings_path = shutil.which("strings")
    if strings_path is None:
        return [], "未找到 strings 命令，无法做动态库符号检查。"

    result = subprocess.run(
        [strings_path, str(path)],
        capture_output=True,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return [], f"strings 执行失败: {result.stderr.strip() or 'unknown error'}"

    detected = [planner_id for planner_id in planner_ids if f"geometric::{planner_id}" in result.stdout]
    return detected, "已完成动态库符号检查。"


def main() -> int:
    """Main."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument(
        "--config",
        help="Path to ompl_planning.yaml. Defaults to sibling cr5_moveit config in the workspace.",
    )
    parser.add_argument(
        "--moveit-catalog",
        help="Optional path to MoveIt OMPL defaults catalog. Used as a conservative runtime-support hint.",
    )
    parser.add_argument(
        "--group",
        default="cr5_group",
        help="Planning group to inspect inside ompl_planning.yaml.",
    )
    parser.add_argument(
        "--require-modern",
        action="store_true",
        help="Return non-zero if BITstar and InformedRRTstar are not both registered for the target group.",
    )
    args = parser.parse_args()

    config_path = locate_config(args.config)
    moveit_catalog_path = locate_moveit_catalog(args.moveit_catalog)

    print("=" * 68)
    print("MoveIt / OMPL planner availability check")
    print("=" * 68)
    print(f"Config path: {config_path}")
    print(f"MoveIt catalog path: {moveit_catalog_path}")

    installed, install_message = check_moveit_ompl_installed()
    print(f"Runtime package check: {'OK' if installed else 'WARN'} - {install_message}")

    if not config_path.exists():
        print("Config status: ERROR - ompl_planning.yaml 不存在")
        return 2

    registered, groups = parse_ompl_config(config_path)
    group_planners = groups.get(args.group, [])
    moveit_catalog = []
    if moveit_catalog_path.exists():
        moveit_catalog = parse_root_planner_ids(moveit_catalog_path)
    pkg_prefix, prefix_message = current_moveit_pkg_prefix()
    runtime_library = None
    runtime_registered = []
    runtime_message = "未执行动态库符号检查。"
    if pkg_prefix is not None:
        runtime_library = pkg_prefix / "lib" / "libmoveit_ompl_interface.so"
        runtime_registered, runtime_message = detect_registered_planners_from_library(
            runtime_library,
            REQUIRED_CLASSICAL + RECOMMENDED_MODERN,
        )

    print(f"Registered planner IDs: {format_list(registered)}")
    print(f"Group `{args.group}` planners: {format_list(group_planners)}")
    print(f"MoveIt default planner catalog: {format_list(moveit_catalog)}")
    print(f"Current runtime prefix: {pkg_prefix if pkg_prefix is not None else '(unknown)'}")
    print(f"Current runtime library: {runtime_library if runtime_library is not None else '(unknown)'}")
    print(f"Runtime prefix check: {prefix_message}")
    print(f"Runtime symbol check: {runtime_message}")
    print(f"Runtime-registered planners: {format_list(runtime_registered)}")

    missing_classical_registered = [name for name in REQUIRED_CLASSICAL if name not in registered]
    missing_classical_group = [name for name in REQUIRED_CLASSICAL if name not in group_planners]
    missing_modern_registered = [name for name in RECOMMENDED_MODERN if name not in registered]
    missing_modern_group = [name for name in RECOMMENDED_MODERN if name not in group_planners]
    missing_modern_catalog = [name for name in RECOMMENDED_MODERN if name not in moveit_catalog]
    missing_modern_runtime = [name for name in RECOMMENDED_MODERN if name not in runtime_registered]

    print(f"Missing classical planners in registry: {format_list(missing_classical_registered)}")
    print(f"Missing classical planners in group: {format_list(missing_classical_group)}")
    print(f"Missing modern planners in registry: {format_list(missing_modern_registered)}")
    print(f"Missing modern planners in group: {format_list(missing_modern_group)}")
    print(f"Missing modern planners in MoveIt catalog: {format_list(missing_modern_catalog)}")
    print(f"Missing modern planners in runtime library: {format_list(missing_modern_runtime)}")

    config_ready = (
        "BITstar" in registered
        and "InformedRRTstar" in registered
        and "BITstar" in group_planners
        and "InformedRRTstar" in group_planners
    )
    runtime_registration_ready = (
        "BITstar" in runtime_registered
        and "InformedRRTstar" in runtime_registered
    )
    modern_ready = config_ready and runtime_registration_ready

    print(
        "Canonical benchmark readiness: "
        + ("READY" if not missing_classical_registered and not missing_classical_group else "INCOMPLETE")
    )
    print("Modern baseline config readiness: " + ("READY" if config_ready else "NOT READY"))
    print("Modern baseline runtime registration: " + ("READY" if runtime_registration_ready else "NOT READY"))
    print("Modern baseline readiness: " + ("READY" if modern_ready else "NOT READY"))
    print("Note: readiness here checks config + runtime registration, not scenario-level planning quality.")

    if modern_ready:
        print("Conclusion: 当前环境已完成 BITstar / InformedRRTstar 的配置与运行时注册。")
        print("Caution: 这不代表场景级稳定性已验证，正式 benchmark 前仍需单独做 smoke / crash 检查。")
    else:
        print("Conclusion: 当前环境仍应保持经典 baseline + HeuristicGuided 为默认集。")
        if not config_ready:
            print("Reason: cr5_moveit/ompl_planning.yaml 尚未注册 BITstar / InformedRRTstar。")
        if not runtime_registration_ready:
            print("Reason: 当前实际加载的 moveit_ompl_interface 动态库未注册 BITstar / InformedRRTstar。")

    if args.require_modern and not modern_ready:
        return 3
    return 0


if __name__ == "__main__":
    sys.exit(main())
