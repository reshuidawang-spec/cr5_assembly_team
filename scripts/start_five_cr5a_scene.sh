#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(CDPATH= cd -- "$(dirname -- "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(CDPATH= cd -- "$SCRIPT_DIR/.." && pwd)"
SCENE_PATH="$REPO_ROOT/scenes/five_cr5a_cell.ttt"

usage() {
  cat <<'EOF'
Usage:
  bash scripts/start_five_cr5a_scene.sh [extra CoppeliaSim args]

Starts the checked-in five-CR5A CoppeliaSim scene from any clone location.

Optional environment variables:
  COPPELIASIM_SH=/absolute/path/to/coppeliaSim.sh
  COPPELIASIM_ROOT=/absolute/path/to/CoppeliaSim
  ROS_SETUP=/opt/ros/humble/setup.bash
  SOURCE_ROS=0    # skip sourcing ROS setup
EOF
}

if [[ "${1:-}" == "-h" || "${1:-}" == "--help" ]]; then
  usage
  exit 0
fi

if [[ ! -f "$SCENE_PATH" ]]; then
  echo "Scene file not found: $SCENE_PATH" >&2
  echo "Run this script from a clone that contains scenes/five_cr5a_cell.ttt." >&2
  exit 1
fi

if [[ "${SOURCE_ROS:-1}" != "0" ]]; then
  ROS_SETUP_PATH="${ROS_SETUP:-/opt/ros/humble/setup.bash}"
  if [[ -f "$ROS_SETUP_PATH" ]]; then
    # shellcheck source=/dev/null
    source "$ROS_SETUP_PATH"
  else
    echo "Warning: ROS setup not found at $ROS_SETUP_PATH; continuing without it." >&2
  fi
fi

declare -a CANDIDATES=()
if [[ -n "${COPPELIASIM_SH:-}" ]]; then
  CANDIDATES+=("$COPPELIASIM_SH")
fi
if [[ -n "${COPPELIASIM_ROOT:-}" ]]; then
  CANDIDATES+=("$COPPELIASIM_ROOT/coppeliaSim.sh")
fi
CANDIDATES+=(
  "$HOME/CoppeliaSim/coppeliaSim.sh"
  "/home/vboxuser/CoppeliaSim/coppeliaSim.sh"
  "/opt/CoppeliaSim/coppeliaSim.sh"
  "/opt/coppeliasim/coppeliaSim.sh"
)
if command -v coppeliaSim.sh >/dev/null 2>&1; then
  CANDIDATES+=("$(command -v coppeliaSim.sh)")
fi
for path in /opt/CoppeliaSim*/coppeliaSim.sh "$HOME"/CoppeliaSim*/coppeliaSim.sh; do
  [[ -e "$path" ]] && CANDIDATES+=("$path")
done

COPPELIASIM=""
for candidate in "${CANDIDATES[@]}"; do
  if [[ -x "$candidate" ]]; then
    COPPELIASIM="$candidate"
    break
  fi
done

if [[ -z "$COPPELIASIM" ]]; then
  cat >&2 <<EOF
Could not find CoppeliaSim.

Set one of these and run again:
  COPPELIASIM_SH=/absolute/path/to/coppeliaSim.sh bash scripts/start_five_cr5a_scene.sh
  COPPELIASIM_ROOT=/absolute/path/to/CoppeliaSim bash scripts/start_five_cr5a_scene.sh
EOF
  exit 1
fi

echo "Repository: $REPO_ROOT"
echo "Scene:      $SCENE_PATH"
echo "Coppelia:   $COPPELIASIM"
echo "Open the scene and keep simulation stopped until a robot_control command starts it."

exec "$COPPELIASIM" "$SCENE_PATH" "$@"
