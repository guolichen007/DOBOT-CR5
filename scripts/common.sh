#!/usr/bin/env bash
set -euo pipefail
REPO_ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$REPO_ROOT/config/lab.env"
source /opt/ros/noetic/setup.bash
if [[ -f "$REPO_ROOT/devel/setup.bash" ]]; then
  source "$REPO_ROOT/devel/setup.bash"
fi
