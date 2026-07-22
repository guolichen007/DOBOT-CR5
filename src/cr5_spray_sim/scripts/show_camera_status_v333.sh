#!/usr/bin/env bash
# ===========================================================================
# V3.3.3 Camera Status Diagnostic
# 显示 /clock、各相机 color/depth 帧率和 publisher 数量
#
# 用法:
#   source ~/cr5_ros1_ws/src/cr5_spray_sim/scripts/use_spray_session_v33.sh
#   bash show_camera_status_v333.sh
# ===========================================================================
set -euo pipefail

echo "=============================================="
echo "  Camera Status Diagnostic"
echo "=============================================="

# /clock
echo ""
echo "--- /clock ---"
CLOCK_RESULT=$(timeout 3 rostopic hz /clock 2>&1 || true)
echo "$CLOCK_RESULT" | tail -3

# 各相机
for cam in cam_front_left cam_front_right cam_rear; do
  echo ""
  echo "--- ${cam} ---"

  # Color
  COLOR_TOPIC=$(rostopic list 2>/dev/null | grep "${cam}.*color/image_raw" | head -1 || echo "")
  if [[ -n "$COLOR_TOPIC" ]]; then
    echo "  Color: ${COLOR_TOPIC}"
    timeout 3 rostopic hz "$COLOR_TOPIC" 2>&1 | tail -3 || echo "    (no data)"
    echo "  Publishers:"
    rostopic info "$COLOR_TOPIC" 2>/dev/null | grep -E "Publishers|Subscribers" || echo "    none"
  else
    echo "  Color: NOT FOUND"
  fi

  # Depth
  DEPTH_TOPIC=$(rostopic list 2>/dev/null | grep "${cam}.*depth/image_raw" | head -1 || echo "")
  if [[ -n "$DEPTH_TOPIC" ]]; then
    echo "  Depth: ${DEPTH_TOPIC}"
    timeout 3 rostopic hz "$DEPTH_TOPIC" 2>&1 | tail -3 || echo "    (no data)"
  else
    echo "  Depth: NOT FOUND"
  fi

  # CameraInfo
  CI_TOPIC=$(rostopic list 2>/dev/null | grep "${cam}.*color/camera_info" | head -1 || echo "")
  if [[ -n "$CI_TOPIC" ]]; then
    echo "  CameraInfo: ${CI_TOPIC}"
    timeout 3 rostopic echo -n1 "$CI_TOPIC" 2>/dev/null | grep -E '^(width|height|K):' || echo "    (no message)"
  fi
done

echo ""
echo "=============================================="
echo "  To view images:"
echo "    rqt_image_view /cam_front_left/camera/color/image_raw"
echo "    rqt_image_view /cam_front_right/camera/color/image_raw"
echo "    rqt_image_view /cam_rear/camera/color/image_raw"
echo "=============================================="
