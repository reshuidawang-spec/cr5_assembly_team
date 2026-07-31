#!/usr/bin/env bash
set -euo pipefail

REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
COPPELIASIM_ROOT="${COPPELIASIM_ROOT:-/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04}"
SCENE_PATH="${CR5_SCENE_PATH:-${REPO_ROOT}/scenes/compact_cell1ttt.ttt}"

if [[ ! -x "${COPPELIASIM_ROOT}/coppeliaSim.sh" ]]; then
  echo "CoppeliaSim 启动脚本不存在或不可执行: ${COPPELIASIM_ROOT}/coppeliaSim.sh" >&2
  echo "请设置 COPPELIASIM_ROOT，例如:" >&2
  echo "  export COPPELIASIM_ROOT=/opt/CoppeliaSim_Edu_V4_10_0_rev0_Ubuntu22_04" >&2
  exit 1
fi

if [[ ! -f "${SCENE_PATH}" ]]; then
  echo "场景文件不存在: ${SCENE_PATH}" >&2
  echo "请设置 CR5_SCENE_PATH，例如:" >&2
  echo "  export CR5_SCENE_PATH=/home/you/cr5_scene/compact_cell1ttt.ttt" >&2
  exit 1
fi

cd "${COPPELIASIM_ROOT}"
exec ./coppeliaSim.sh "${SCENE_PATH}"
