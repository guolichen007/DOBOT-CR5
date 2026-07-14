#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
rosservice call /dobot_bringup/srv/ClearError "{}"
rosservice call /dobot_bringup/srv/SpeedFactor "{ratio: 5}"
rosservice call /dobot_bringup/srv/EnableRobot "{args: []}"
echo "ClearError, SpeedFactor=5 and EnableRobot requests were sent. Run preflight again."
