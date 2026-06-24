#!/usr/bin/env python3
"""Unified formal rerun — re-runs all benchmark experiments under controlled, reproducible conditions and exports formal results for paper submission."""

import argparse
import json
import os
import shutil
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_ARCHIVE_ROOT = REPO_ROOT / "paper_workspace" / "formal_results"
DEFAULT_PLANNERS = "RRTConnect,RRTstar,LBTRRT,FMT,BFMT,PRMstar,HeuristicGuided"
DEFAULT_ABLATION_MODES = "full,direct_only,fixed_guide,no_anchors,no_adaptive_difficulty,always_guide"

BENCHMARKS = {
    "simple": {
        "executable": "planner_comparison_simple_node",
        "repeat_env": "MY_CR5_CONTROL_SIMPLE_REPEATS",
        "result_glob": "benchmarks/simple/raw/*_planner_comparison_simple_results.csv",
        "summary_glob": "benchmarks/simple/raw/*_planner_comparison_simple_summary.csv",
        "aggregate_globs": [
            "benchmarks/simple/aggregates/planner_comparison_simple_plot_data_metrics.csv",
            "benchmarks/simple/aggregates/planner_comparison_simple_plot_summary_metrics.csv",
        ],
    },
    "v2": {
        "executable": "planner_comparison_v2_node",
        "repeat_env": "MY_CR5_CONTROL_V2_REPEATS",
        "result_glob": "benchmarks/v2/raw/*_planner_comparison_v2_results.csv",
        "summary_glob": "benchmarks/v2/raw/*_planner_comparison_v2_summary.csv",
        "aggregate_globs": [
            "benchmarks/v2/aggregates/planner_comparison_v2_plot_data_metrics.csv",
            "benchmarks/v2/aggregates/planner_comparison_v2_plot_summary_metrics.csv",
        ],
    },
}


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Run a paper-facing unified formal rerun for simple/v2 benchmarks."
    )
    parser.add_argument(
        "--session-name",
        default="q2_unified_formal",
        help="Archive session name prefix.",
    )
    parser.add_argument(
        "--archive-root",
        default=str(DEFAULT_ARCHIVE_ROOT),
        help="Root directory for paper formal-result archives.",
    )
    parser.add_argument(
        "--benchmarks",
        default="simple,v2",
        help="Comma-separated benchmark list: simple,v2.",
    )
    parser.add_argument(
        "--planners",
        default=DEFAULT_PLANNERS,
        help="Comma-separated planner list shared by all selected benchmarks.",
    )
    parser.add_argument(
        "--simple-repeats",
        type=int,
        default=30,
        help="Repeat count for the simple benchmark.",
    )
    parser.add_argument(
        "--v2-repeats",
        type=int,
        default=30,
        help="Repeat count for the v2 benchmark.",
    )
    parser.add_argument(
        "--scenes",
        default="",
        help="Optional comma-separated scene filter shared by selected benchmarks.",
    )
    parser.add_argument(
        "--v2-mesh-profile",
        default="ws119",
        help="V2 mesh profile for formal rerun. Default keeps canonical WS119 benchmark.",
    )
    parser.add_argument(
        "--start-demo",
        action="store_true",
        help="Start cr5_moveit demo before running benchmark nodes.",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=45.0,
        help="Seconds to wait for move_group readiness when --start-demo is used.",
    )
    parser.add_argument(
        "--unit-timeout",
        type=float,
        default=0.0,
        help="Optional seconds before a benchmark unit is killed and marked failed.",
    )
    parser.add_argument(
        "--dry-run",
        action="store_true",
        help="Create archive metadata and print commands without running benchmarks.",
    )
    parser.add_argument(
        "--isolate-planners",
        action="store_true",
        help="Run each planner in a separate benchmark invocation.",
    )
    parser.add_argument(
        "--restart-demo-per-run",
        action="store_true",
        help="Restart cr5_moveit demo for every benchmark/planner unit.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Record failed benchmark/planner units and continue with remaining units.",
    )
    parser.add_argument(
        "--ablation-modes",
        default="",
        help=(
            "Optional comma-separated HeuristicGuided ablation modes. "
            f"Use 'paper' for {DEFAULT_ABLATION_MODES}."
        ),
    )
    return parser.parse_args()


def timestamp():
    """Timestamp."""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def split_csv(raw):
    """Split csv."""
    return [item.strip() for item in raw.split(",") if item.strip()]


def run_capture(command, env):
    """Run capture."""
    return subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )


def ros2_lines(command, env):
    """Ros2 lines."""
    result = run_capture(command, env)
    if result.returncode != 0:
        return []
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def move_group_ready(env):
    """Move group ready."""
    nodes = set(ros2_lines(["ros2", "node", "list"], env))
    services = set(ros2_lines(["ros2", "service", "list"], env))
    return "/move_group" in nodes and "/compute_ik" in services and "/check_state_validity" in services


def wait_for_move_group(env, timeout_s):
    """Wait for move group."""
    deadline = time.time() + timeout_s
    while time.time() < deadline:
        if move_group_ready(env):
            return True
        time.sleep(1.0)
    return False


def start_demo(env, log_path):
    """Start demo."""
    log_handle = open(log_path, "w", encoding="utf-8")
    process = subprocess.Popen(
        ["ros2", "launch", "cr5_moveit", "demo.launch.py", "use_rviz:=false"],
        cwd=REPO_ROOT,
        env=env,
        stdout=log_handle,
        stderr=subprocess.STDOUT,
        text=True,
        start_new_session=True,
    )
    return process, log_handle


def stop_demo(process, log_handle):
    """Stop demo."""
    try:
        if process.poll() is None:
            os.killpg(process.pid, signal.SIGINT)
            try:
                process.wait(timeout=10)
            except subprocess.TimeoutExpired:
                os.killpg(process.pid, signal.SIGKILL)
                process.wait(timeout=5)
    finally:
        log_handle.close()


def stop_process_group(process):
    """Stop process group."""
    if process.poll() is not None:
        return
    os.killpg(process.pid, signal.SIGINT)
    try:
        process.wait(timeout=10)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGTERM)
    try:
        process.wait(timeout=5)
        return
    except subprocess.TimeoutExpired:
        pass
    os.killpg(process.pid, signal.SIGKILL)
    process.wait(timeout=5)


def kill_orphaned_executable(executable):
    """Kill orphaned executable."""
    subprocess.run(
        ["pkill", "-TERM", "-f", f"/{executable}$"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )
    time.sleep(1.0)
    subprocess.run(
        ["pkill", "-KILL", "-f", f"/{executable}$"],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def newest_matches(root, pattern):
    """Newest matches."""
    return set(root.glob(pattern))


def detect_new_file(root, before, pattern):
    """Detect new file."""
    after = newest_matches(root, pattern)
    created = sorted(after - before)
    if created:
        return created[-1]
    matches = sorted(after)
    return matches[-1] if matches else None


def relative_to_repo(path):
    """Convert an absolute path to a path relative to the repository root."""
    try:
        return str(path.relative_to(REPO_ROOT))
    except ValueError:
        return str(path)


def write_json(path, data):
    """Write json."""
    path.write_text(json.dumps(data, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")


def selected_benchmarks(raw):
    """Selected benchmarks."""
    names = split_csv(raw)
    unknown = [name for name in names if name not in BENCHMARKS]
    if unknown:
        raise RuntimeError(f"Unsupported benchmark(s): {', '.join(unknown)}")
    return names


def selected_ablation_modes(raw):
    """Selected ablation modes."""
    token = raw.strip()
    if not token:
        return []
    if token == "paper":
        return split_csv(DEFAULT_ABLATION_MODES)
    return split_csv(token)


def session_dir(args, stamp):
    """Session dir."""
    return Path(args.archive_root) / f"{args.session_name}_{stamp}"


def build_run_units(args, benchmarks, ablation_modes):
    """Build run units."""
    if ablation_modes:
        return [
            (benchmark, "HeuristicGuided", ablation_mode)
            for benchmark in benchmarks
            for ablation_mode in ablation_modes
        ]
    if args.isolate_planners:
        return [
            (benchmark, planner, None)
            for benchmark in benchmarks
            for planner in split_csv(args.planners)
        ]
    return [(benchmark, None, None) for benchmark in benchmarks]


def build_benchmark_env(base_env, args, benchmark, results_root, planner_override=None, ablation_mode=None):
    """Build benchmark env."""
    env = base_env.copy()
    env["MY_CR5_CONTROL_RESULTS_DIR"] = str(results_root)
    env["MY_CR5_CONTROL_BENCHMARK_PLANNERS"] = planner_override or args.planners
    env["MY_CR5_CONTROL_V2_MESH_PROFILE"] = args.v2_mesh_profile
    if ablation_mode:
        env["MY_CR5_CONTROL_HEURISTIC_ABLATION_MODE"] = ablation_mode
    if args.scenes.strip():
        env["MY_CR5_CONTROL_BENCHMARK_SCENES"] = args.scenes.strip()
    repeats = args.simple_repeats if benchmark == "simple" else args.v2_repeats
    env[BENCHMARKS[benchmark]["repeat_env"]] = str(repeats)
    return env


def benchmark_command(benchmark):
    """Benchmark command."""
    return ["ros2", "run", "my_cr5_control", BENCHMARKS[benchmark]["executable"]]


def run_benchmark(benchmark, args, session, base_env, planner_override=None, ablation_mode=None):
    """Run benchmark."""
    config = BENCHMARKS[benchmark]
    results_root = session / "results"
    logs_dir = session / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    results_root.mkdir(parents=True, exist_ok=True)

    env = build_benchmark_env(base_env, args, benchmark, results_root, planner_override, ablation_mode)
    before_results = newest_matches(results_root, config["result_glob"])
    before_summary = newest_matches(results_root, config["summary_glob"])
    log_parts = [benchmark]
    if planner_override:
        log_parts.append(planner_override)
    if ablation_mode:
        log_parts.append(ablation_mode)
    log_suffix = "_".join(log_parts)
    log_path = logs_dir / f"{log_suffix}.log"

    command = benchmark_command(benchmark)
    with open(log_path, "w", encoding="utf-8") as log_handle:
        process = subprocess.Popen(
            command,
            cwd=REPO_ROOT,
            env=env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            start_new_session=True,
        )
        try:
            process.wait(timeout=args.unit_timeout if args.unit_timeout > 0.0 else None)
        except subprocess.TimeoutExpired as exc:
            stop_process_group(process)
            kill_orphaned_executable(config["executable"])
            raise RuntimeError(
                f"{benchmark} timed out after {args.unit_timeout:.1f}s"
            ) from exc

    result_file = detect_new_file(results_root, before_results, config["result_glob"])
    summary_file = detect_new_file(results_root, before_summary, config["summary_glob"])
    if process.returncode != 0:
        raise RuntimeError(f"{benchmark} failed with exit code {process.returncode}; see {log_path}")
    if result_file is None or summary_file is None:
        raise RuntimeError(f"{benchmark} finished without detected result files; see {log_path}")

    aggregate_files = []
    for pattern in config["aggregate_globs"]:
        aggregate_files.extend(sorted(results_root.glob(pattern)))

    return {
        "benchmark": benchmark,
        "planner_override": planner_override or "",
        "ablation_mode": ablation_mode or "",
        "command": " ".join(command),
        "repeats": args.simple_repeats if benchmark == "simple" else args.v2_repeats,
        "result_file": relative_to_repo(result_file),
        "summary_file": relative_to_repo(summary_file),
        "log_file": relative_to_repo(log_path),
        "aggregate_files": [relative_to_repo(path) for path in aggregate_files],
    }


def git_identity():
    """Git identity."""
    result = run_capture(["git", "rev-parse", "--short", "HEAD"], os.environ.copy())
    head = result.stdout.strip() if result.returncode == 0 else "unknown"
    status = run_capture(["git", "status", "--short"], os.environ.copy())
    return {
        "head": head,
        "dirty_status": status.stdout.splitlines() if status.returncode == 0 else [],
    }


def create_readme(session, manifest):
    """Create readme."""
    planner_lines = "\n".join(f"- `{planner}`" for planner in split_csv(manifest["planners"]))
    benchmark_lines = []
    for run in manifest["runs"]:
        if "result_file" in run:
            planner_text = f" planner={run['planner_override']}" if run.get("planner_override") else ""
            ablation_text = f" ablation={run['ablation_mode']}" if run.get("ablation_mode") else ""
            benchmark_lines.append(
                f"- `{run['benchmark']}`{planner_text}{ablation_text} repeats={run['repeats']}\n"
                f"  - results: `{run['result_file']}`\n"
                f"  - summary: `{run['summary_file']}`\n"
                f"  - log: `{run['log_file']}`"
            )
        elif run.get("status") == "failed":
            planner_text = f" planner={run['planner_override']}" if run.get("planner_override") else ""
            ablation_text = f" ablation={run['ablation_mode']}" if run.get("ablation_mode") else ""
            benchmark_lines.append(
                f"- `{run['benchmark']}`{planner_text}{ablation_text} failed\n"
                f"  - error: `{run['error']}`\n"
                f"  - log: `{run.get('log_file', 'see logs directory')}`"
            )
        else:
            benchmark_lines.append(
                f"- `{run['benchmark']}` repeats={run['repeats']}\n"
                f"  - command: `{run['command']}`\n"
                f"  - dry-run environment recorded in `manifest.json`"
            )

    readme = f"""# Unified Formal Rerun

Created: {manifest['created_at']}

Purpose: P0.1 fair rerun for the paper review risk register.

## Configuration

- benchmarks: `{','.join(manifest['benchmarks'])}`
- planners:
{planner_lines}
- simple repeats: `{manifest['simple_repeats']}`
- v2 repeats: `{manifest['v2_repeats']}`
- v2 mesh profile: `{manifest['v2_mesh_profile']}`
- scenes filter: `{manifest['scenes'] or 'none'}`
- ablation modes: `{','.join(manifest['ablation_modes']) if manifest['ablation_modes'] else 'none'}`
- isolate planners: `{manifest['isolate_planners']}`
- restart demo per run: `{manifest['restart_demo_per_run']}`
- unit timeout: `{manifest['unit_timeout_s']} s`
- results root: `{manifest['results_root']}`

## Outputs

{chr(10).join(benchmark_lines) if benchmark_lines else '- Dry run only; no benchmark outputs yet.'}

## Notes

- All selected planners in a benchmark are run by the same benchmark node invocation.
- `MY_CR5_CONTROL_RESULTS_DIR` is isolated to this archive session.
- This directory is intended to replace mixed historical result bundles once the rerun completes.
"""
    (session / "README.md").write_text(readme, encoding="utf-8")


def main():
    """Main."""
    args = parse_args()
    stamp = timestamp()
    benchmarks = selected_benchmarks(args.benchmarks)
    ablation_modes = selected_ablation_modes(args.ablation_modes)
    if not benchmarks:
        raise RuntimeError("No benchmarks selected.")

    session = session_dir(args, stamp)
    session.mkdir(parents=True, exist_ok=False)
    (session / "logs").mkdir(parents=True, exist_ok=True)
    (session / "results").mkdir(parents=True, exist_ok=True)

    manifest = {
        "created_at": stamp,
        "session_dir": relative_to_repo(session),
        "results_root": relative_to_repo(session / "results"),
        "benchmarks": benchmarks,
        "planners": args.planners,
        "simple_repeats": args.simple_repeats,
        "v2_repeats": args.v2_repeats,
        "scenes": args.scenes,
        "v2_mesh_profile": args.v2_mesh_profile,
        "ablation_modes": ablation_modes,
        "dry_run": args.dry_run,
        "isolate_planners": args.isolate_planners,
        "restart_demo_per_run": args.restart_demo_per_run,
        "continue_on_failure": args.continue_on_failure,
        "unit_timeout_s": args.unit_timeout,
        "git": git_identity(),
        "runs": [],
    }

    base_env = os.environ.copy()
    base_env["MY_CR5_CONTROL_RESULTS_DIR"] = str(session / "results")
    demo_process = None
    demo_log_handle = None

    def start_demo_if_needed(label):
        """Start demo if needed."""
        nonlocal demo_process, demo_log_handle
        if not args.start_demo:
            if not move_group_ready(base_env):
                raise RuntimeError(
                    "move_group is not ready. Start MoveIt first or rerun with --start-demo."
                )
            return
        if demo_process is not None and demo_process.poll() is None and move_group_ready(base_env):
            return
        demo_log = session / "logs" / f"cr5_moveit_demo_{label}.log"
        demo_process, demo_log_handle = start_demo(base_env, demo_log)
        if not wait_for_move_group(base_env, args.startup_timeout):
            raise RuntimeError(f"move_group not ready within {args.startup_timeout:.1f}s; see {demo_log}")

    def stop_demo_if_running():
        """Stop demo if running."""
        nonlocal demo_process, demo_log_handle
        if demo_process is not None and demo_log_handle is not None:
            stop_demo(demo_process, demo_log_handle)
        demo_process = None
        demo_log_handle = None

    try:
        run_units = build_run_units(args, benchmarks, ablation_modes)
        if args.dry_run:
            for benchmark, planner, ablation_mode in run_units:
                env = build_benchmark_env(
                    base_env, args, benchmark, session / "results", planner, ablation_mode)
                manifest["runs"].append(
                    {
                        "benchmark": benchmark,
                        "planner_override": planner or "",
                        "ablation_mode": ablation_mode or "",
                        "command": " ".join(benchmark_command(benchmark)),
                        "repeats": args.simple_repeats if benchmark == "simple" else args.v2_repeats,
                        "env": {
                            "MY_CR5_CONTROL_RESULTS_DIR": env["MY_CR5_CONTROL_RESULTS_DIR"],
                            "MY_CR5_CONTROL_BENCHMARK_PLANNERS": env["MY_CR5_CONTROL_BENCHMARK_PLANNERS"],
                            "MY_CR5_CONTROL_BENCHMARK_SCENES": env.get("MY_CR5_CONTROL_BENCHMARK_SCENES", ""),
                            "MY_CR5_CONTROL_V2_MESH_PROFILE": env["MY_CR5_CONTROL_V2_MESH_PROFILE"],
                            "MY_CR5_CONTROL_HEURISTIC_ABLATION_MODE": env.get(
                                "MY_CR5_CONTROL_HEURISTIC_ABLATION_MODE", ""
                            ),
                            BENCHMARKS[benchmark]["repeat_env"]: env[BENCHMARKS[benchmark]["repeat_env"]],
                        },
                    }
                )
            write_json(session / "manifest.json", manifest)
            create_readme(session, manifest)
            print(f"Dry run archive created: {session}")
            return 0

        for benchmark, planner, ablation_mode in run_units:
            label_parts = [benchmark]
            if planner:
                label_parts.append(planner)
            if ablation_mode:
                label_parts.append(ablation_mode)
            label = "_".join(label_parts)
            try:
                if args.restart_demo_per_run:
                    stop_demo_if_running()
                start_demo_if_needed(label)
                print(f"Running {label} unified formal rerun...")
                run = run_benchmark(benchmark, args, session, base_env, planner, ablation_mode)
                manifest["runs"].append(run)
                print(f"  result: {run['result_file']}")
                print(f"  summary: {run['summary_file']}")
            except Exception as exc:
                failed_run = {
                    "benchmark": benchmark,
                    "planner_override": planner or "",
                    "ablation_mode": ablation_mode or "",
                    "status": "failed",
                    "error": str(exc),
                    "log_file": relative_to_repo(session / "logs" / f"{label}.log"),
                }
                manifest["runs"].append(failed_run)
                if not args.continue_on_failure:
                    write_json(session / "manifest.json", manifest)
                    create_readme(session, manifest)
                    raise
                print(f"  failed: {label}: {exc}")
            finally:
                write_json(session / "manifest.json", manifest)
                create_readme(session, manifest)
                if args.restart_demo_per_run:
                    stop_demo_if_running()

        write_json(session / "manifest.json", manifest)
        create_readme(session, manifest)
        print(f"Unified formal rerun archive: {session}")
        return 0
    except Exception:
        write_json(session / "manifest.json", manifest)
        create_readme(session, manifest)
        raise
    finally:
        stop_demo_if_running()


if __name__ == "__main__":
    sys.exit(main())
