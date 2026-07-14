#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source /opt/ros/noetic/setup.bash
"$ROOT/scripts/init_workspace.sh"
cd "$ROOT"
catkin_make
printf '\nBuild complete. Run: source %s/devel/setup.bash\n' "$ROOT"
