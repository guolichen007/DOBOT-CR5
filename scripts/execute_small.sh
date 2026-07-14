#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
cat <<'WARN'
REAL ROBOT MOTION REQUESTED.
Prerequisites: physical E-stop attended, area clear, Tool_end verified, controller SpeedFactor=5.
WARN
read -r -p "Type CR5_A4_EXECUTE to continue: " TOKEN
[[ "$TOKEN" == "CR5_A4_EXECUTE" ]] || { echo "Cancelled."; exit 3; }
exec roslaunch a4_spray_demo a4_raster_demo.launch \
  config:="$(rospack find a4_spray_demo)/config/small_test.yaml" \
  execute:=true confirmation:=CR5_A4_EXECUTE eef_link:="$A4_EEF_LINK"
