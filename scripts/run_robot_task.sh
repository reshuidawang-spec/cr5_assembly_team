#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/run_robot_task.sh TASK_NAME [task options]

Examples:
  bash scripts/run_robot_task.sh R1_BOX_PLACED
  bash scripts/run_robot_task.sh R2_PCB_PLACED
  bash scripts/run_robot_task.sh R5_SORT_GOOD_DONE --speed-deg-s 50

The CoppeliaSim scene must already be open through scripts/start_five_cr5a_scene.sh.
Single tasks require their real upstream scene state; they are not all valid from
a clean scene.
EOF
}

if [[ $# -lt 1 || "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit $([[ $# -lt 1 ]] && echo 2 || echo 0)
fi

TASK="$1"
shift

case "$TASK" in
  R1_*) RUNNER="robot_control/run_r1_task.py" ;;
  R2_*) RUNNER="robot_control/run_r2_task.py" ;;
  R3_*) RUNNER="robot_control/run_r3_task.py" ;;
  R4_*) RUNNER="robot_control/run_r4_task.py" ;;
  R5_*) RUNNER="robot_control/run_r5_task.py" ;;
  *)
    echo "Unsupported task name: $TASK" >&2
    usage >&2
    exit 2
    ;;
esac

exec python3 "$RUNNER" "$TASK" "$@"
