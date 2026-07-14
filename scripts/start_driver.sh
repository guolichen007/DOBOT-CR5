#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
: "${ROBOT_IP:?ROBOT_IP is not set}"
export DOBOT_TYPE
exec roslaunch dobot_bringup bringup.launch robot_ip:="$ROBOT_IP"
