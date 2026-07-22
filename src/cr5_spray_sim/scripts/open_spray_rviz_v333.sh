#!/usr/bin/env bash
# ===========================================================================
# V3.3.3 Open RViz with Spray Configuration
#
# 用法:
#   方式1 (独立会话):
#     bash open_spray_rviz_v333.sh
#
#   方式2 (连接已有会话):
#     source use_spray_session_v33.sh
#     rviz -d $(rospack find cr5_spray_sim)/config/cr5_spray_v333.rviz
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
RViz_CONFIG="${PKG_DIR}/config/cr5_spray_v333.rviz"

if [[ ! -f "$RViz_CONFIG" ]]; then
  echo "ERROR: RViz config not found: $RViz_CONFIG" >&2
  exit 1
fi

# 检查 ROS master
if ! rostopic list >/dev/null 2>&1; then
  echo "ERROR: No ROS master. Source the session first:" >&2
  echo "  source ${PKG_DIR}/scripts/use_spray_session_v33.sh" >&2
  exit 1
fi

echo "Opening RViz with spray configuration..."
echo "  Config: ${RViz_CONFIG}"
echo ""
echo "  Displays:"
echo "    - RobotModel (CR5)"
echo "    - TF (world→Link6→spray_nozzle_frame)"
echo "    - /spray_demo/spray_marker"
echo "    - /spray_demo/paint_patches"
echo "    - /spray_demo/hit_point"
echo "    - /spray_demo/hit_normal"

rviz -d "$RViz_CONFIG" "$@"
