#!/usr/bin/env bash
# ===========================================================================
# V4: 在当前活跃的 CR5 Spray V3.3.7 会话中执行任意命令.
#
# 用法:
#   bash run_in_simulation_session.sh rosrun image_view image_view image:=/cam_front_left/camera/color/image_raw
#   bash run_in_simulation_session.sh rosrun cr5_spray_sim validate_calibration_target_visibility.py --output artifacts/calib_v4/visibility
# ===========================================================================
set -euo pipefail

ENV_FILE="/tmp/cr5_spray_simulation.env"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: No active CR5 Spray V3.3.7 session" >&2
  echo "Start first: bash run_simulation.sh --gui --object=<type>" >&2
  exit 1
fi

if [[ $# -eq 0 ]]; then
  echo "Usage: bash run_in_simulation_session.sh <command> [args...]"
  echo ""
  echo "Examples:"
  echo "  bash run_in_simulation_session.sh rostopic list"
  echo "  bash run_in_simulation_session.sh rosrun image_view image_view image:=/cam_front_left/camera/color/image_raw"
  echo "  bash run_in_simulation_session.sh rosrun tf tf_echo world calibration_target_frame"
  exit 1
fi

source "$ENV_FILE"
source "$CR5_SPRAY_WS/devel/setup.bash" 2>/dev/null

echo "[run_in_simulation_session] ROS_MASTER_URI=$ROS_MASTER_URI"
echo "[run_in_simulation_session] executing: $*"
echo ""

exec "$@"
