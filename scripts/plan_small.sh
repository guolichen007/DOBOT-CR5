#!/usr/bin/env bash
set -euo pipefail
source "$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)/common.sh"
exec roslaunch a4_spray_demo a4_raster_demo.launch \
  config:="$(rospack find a4_spray_demo)/config/small_test.yaml" \
  execute:=false eef_link:="$A4_EEF_LINK"
