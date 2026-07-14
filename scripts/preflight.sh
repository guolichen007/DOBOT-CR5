#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
export DOBOT_TYPE
exec roslaunch a4_spray_demo quick_check.launch eef_link:="$A4_EEF_LINK"
