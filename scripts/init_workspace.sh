#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
cd "$ROOT/src"
if [[ ! -e CMakeLists.txt ]]; then
  catkin_init_workspace
fi
printf 'Workspace initialized at %s\n' "$ROOT"
