#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
export DOBOT_TYPE
exec roslaunch dobot_moveit moveit.launch
