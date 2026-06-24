#!/usr/bin/env python3
"""Benchmark dataset collector — end-to-end pipeline: launches MoveIt, runs simple/V2 planner benchmarks, and exports a unified training dataset CSV."""

import argparse
import os
import signal
import subprocess
import sys
import time
from datetime import datetime
from pathlib import Path


REPO_ROOT = Path(__file__).resolve().parents[2]
RESULTS_ROOT = REPO_ROOT / "test_results"
LOG_ROOT = RESULTS_ROOT / "logs" / "collection"

BENCHMARK_CONFIG = {
    "simple": {
        "executable": "planner_comparison_simple_node",
        "env_var": "MY_CR5_CONTROL_SIMPLE_REPEATS",
        "result_pattern": "benchmarks/simple/raw/20*_planner_comparison_simple_results.csv",
        "summary_pattern": "benchmarks/simple/raw/20*_planner_comparison_simple_summary.csv",
    },
    "v2": {
        "executable": "planner_comparison_v2_node",
        "env_var": "MY_CR5_CONTROL_V2_REPEATS",
        "result_pattern": "benchmarks/v2/raw/20*_planner_comparison_v2_results.csv",
        "summary_pattern": "benchmarks/v2/raw/20*_planner_comparison_v2_summary.csv",
    },
}


def parse_args():
    """Parse args."""
    parser = argparse.ArgumentParser(
        description="Run simple/v2 benchmarks end-to-end and export a unified dataset."
    )
    parser.add_argument(
        "--benchmarks",
        default="simple,v2",
        help="Comma-separated list from: simple,v2",
    )
    parser.add_argument(
        "--simple-repeats",
        type=int,
        default=10,
        help="Repeat count for the simple benchmark.",
    )
    parser.add_argument(
        "--v2-repeats",
        type=int,
        default=10,
        help="Repeat count for the v2 benchmark.",
    )
    parser.add_argument(
        "--dataset-output",
        default=str(RESULTS_ROOT / "exports" / "benchmark_training_dataset.csv"),
        help="Output csv for the normalized dataset.",
    )
    parser.add_argument(
        "--export-only",
        action="store_true",
        help="Skip benchmark execution and only export the unified dataset.",
    )
    parser.add_argument(
        "--skip-export",
        action="store_true",
        help="Skip the final dataset export step.",
    )
    parser.add_argument(
        "--startup-timeout",
        type=float,
        default=45.0,
        help="Seconds to wait for MoveIt services when starting a demo instance.",
    )
    return parser.parse_args()


def session_stamp():
    """Return a timestamp string for naming output files."""
    return datetime.now().strftime("%Y%m%d_%H%M%S")


def ensure_logs_dir():
    """Ensure logs dir."""
    LOG_ROOT.mkdir(parents=True, exist_ok=True)
    return LOG_ROOT


def ros2_lines(command, env):
    """Ros2 lines."""
    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return []
    return [line.strip() for line in result.stdout.splitlines() if line.strip()]


def move_group_ready(env):
    """Move group ready."""
    nodes = set(ros2_lines(["ros2", "node", "list"], env))
    services = set(ros2_lines(["ros2", "service", "list"], env))
    required_nodes = {"/move_group"}
    required_services = {"/compute_ik", "/check_state_validity"}
    return required_nodes.issubset(nodes) and required_services.issubset(services)


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
        ["ros2", "launch", "cr5_moveit", "demo.launch.py"],
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


def newest_files(pattern):
    """Newest files."""
    return set(RESULTS_ROOT.glob(pattern))


def detect_new_file(before, pattern):
    """Detect new file."""
    after = newest_files(pattern)
    new_files = sorted(after - before)
    if new_files:
        return new_files[-1]
    matches = sorted(after)
    return matches[-1] if matches else None


def run_benchmark(benchmark, repeats, env, stamp):
    """Run benchmark."""
    config = BENCHMARK_CONFIG[benchmark]
    bench_env = env.copy()
    bench_env[config["env_var"]] = str(repeats)

    result_before = newest_files(config["result_pattern"])
    summary_before = newest_files(config["summary_pattern"])
    log_path = ensure_logs_dir() / f"{stamp}_{benchmark}.log"

    with open(log_path, "w", encoding="utf-8") as log_handle:
        result = subprocess.run(
            ["ros2", "run", "my_cr5_control", config["executable"]],
            cwd=REPO_ROOT,
            env=bench_env,
            stdout=log_handle,
            stderr=subprocess.STDOUT,
            text=True,
            check=False,
        )

    result_file = detect_new_file(result_before, config["result_pattern"])
    summary_file = detect_new_file(summary_before, config["summary_pattern"])

    if result.returncode != 0:
        raise RuntimeError(
            f"{benchmark} benchmark failed with exit code {result.returncode}. "
            f"See log: {log_path}"
        )

    if result_file is None or summary_file is None:
        raise RuntimeError(
            f"{benchmark} benchmark finished but result files were not detected. "
            f"See log: {log_path}"
        )

    return {
        "benchmark": benchmark,
        "repeats": repeats,
        "result_file": result_file,
        "summary_file": summary_file,
        "log_file": log_path,
    }


def export_dataset(env, output_path, input_files):
    """Export dataset."""
    command = [
        sys.executable,
        str(REPO_ROOT / "scripts" / "benchmarks" / "export_benchmark_dataset.py"),
        "--output",
        str(output_path),
    ]
    if input_files:
        command.extend(["--inputs", *[str(path) for path in input_files]])

    result = subprocess.run(
        command,
        cwd=REPO_ROOT,
        env=env,
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        raise RuntimeError(
            f"Dataset export failed.\nSTDOUT:\n{result.stdout}\nSTDERR:\n{result.stderr}"
        )
    return result.stdout.strip()


def selected_benchmarks(raw):
    """Selected benchmarks."""
    items = [item.strip() for item in raw.split(",") if item.strip()]
    if not items:
        return []
    unknown = [item for item in items if item not in BENCHMARK_CONFIG]
    if unknown:
        raise RuntimeError(f"Unsupported benchmarks: {', '.join(unknown)}")
    return items


def main():
    """Main."""
    args = parse_args()
    env = os.environ.copy()
    stamp = session_stamp()
    dataset_output = Path(args.dataset_output)
    benchmarks = selected_benchmarks(args.benchmarks)

    if args.export_only:
        message = export_dataset(env, dataset_output, [])
        print(message)
        return 0

    started_demo = False
    demo_process = None
    demo_log_handle = None
    demo_log_path = ensure_logs_dir() / f"{stamp}_cr5_moveit_demo.log"

    try:
        if move_group_ready(env):
            print("Reusing existing move_group instance.")
        else:
            print(f"Starting cr5_moveit demo, log: {demo_log_path}")
            demo_process, demo_log_handle = start_demo(env, demo_log_path)
            started_demo = True
            if not wait_for_move_group(env, args.startup_timeout):
                raise RuntimeError(
                    f"MoveIt demo did not become ready within {args.startup_timeout:.1f}s. "
                    f"See log: {demo_log_path}"
                )
            print("MoveIt demo is ready.")

        run_records = []
        for benchmark in benchmarks:
            repeats = args.simple_repeats if benchmark == "simple" else args.v2_repeats
            print(f"Running {benchmark} benchmark with repeats={repeats}")
            record = run_benchmark(benchmark, repeats, env, stamp)
            run_records.append(record)
            print(f"  result: {record['result_file']}")
            print(f"  summary: {record['summary_file']}")
            print(f"  log: {record['log_file']}")

        if not args.skip_export:
            export_message = export_dataset(
                env,
                dataset_output,
                [record["result_file"] for record in run_records],
            )
            print(export_message)
            print(f"Dataset: {dataset_output}")

        return 0
    finally:
        if started_demo and demo_process is not None and demo_log_handle is not None:
            print("Stopping cr5_moveit demo.")
            stop_demo(demo_process, demo_log_handle)


if __name__ == "__main__":
    sys.exit(main())
