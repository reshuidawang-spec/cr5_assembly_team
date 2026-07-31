#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
cd "$REPO_ROOT"

QUALITY="good"
if [[ "${1:-}" == "good" || "${1:-}" == "defect" ]]; then
  QUALITY="$1"
  shift
fi

exec python3 robot_control/run_five_arm_cycle.py --quality "$QUALITY" "$@"
