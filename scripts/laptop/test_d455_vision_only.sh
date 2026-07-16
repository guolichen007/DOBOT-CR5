#!/usr/bin/env bash
set -euo pipefail
MODE="${1:-help}"
WS="${WS:-$HOME/cr5_ros1_ws}"

source /opt/ros/noetic/setup.bash
[ -f "$WS/devel/setup.bash" ] && source "$WS/devel/setup.bash"

case "$MODE" in
  precheck)
    git -C "$WS" branch --show-current
    git -C "$WS" rev-parse HEAD
    git -C "$WS" status --short --branch
    lsusb
    lsusb -t
    command -v rs-enumerate-devices || true
    rs-enumerate-devices 2>/dev/null || true
    rospack find realsense2_camera 2>/dev/null || true
    rospack find cr5_book_spray_demo 2>/dev/null || true
    ping -c 2 192.168.110.214 || true
    ;;
  camera)
    if pgrep -af 'realsense2_camera|rs_camera.launch|realsense-viewer' | grep -v "$0"; then
      echo "[ERROR] A RealSense process may already be running." >&2
      exit 1
    fi
    exec roslaunch realsense2_camera rs_camera.launch       align_depth:=true enable_color:=true enable_depth:=true
    ;;
  topics)
    rostopic list | grep '^/camera/' | sort || true
    echo "Then run:"
    echo "  rostopic hz /camera/color/image_raw"
    echo "  rostopic hz /camera/aligned_depth_to_color/image_raw"
    echo "  rostopic echo -n 1 /camera/color/camera_info"
    ;;
  vision)
    exec roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false
    ;;
  *)
    echo "Usage: $0 {precheck|camera|topics|vision}"
    ;;
esac
