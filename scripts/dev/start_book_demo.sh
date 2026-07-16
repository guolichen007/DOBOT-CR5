#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_book_demo.sh - 启动书本识别
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  启动书本识别"
echo "=========================================="

# 检查相机是否运行
if ! pgrep -f "realsense2_camera_node" &>/dev/null; then
    echo "[WARN] RealSense 相机未运行"
    echo "请先启动: start_camera"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查相机话题
echo "检查相机话题..."
for TOPIC in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw /camera/color/camera_info; do
    if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
        echo "[PASS] $TOPIC"
    else
        echo "[FAIL] $TOPIC 不存在"
        echo "请确保相机已启动"
        exit 1
    fi
done

# 加载环境
source "$SCRIPTS_DEV_DIR/env.sh" 2>/dev/null || true

echo
echo "启动书本识别..."
echo "参数: start_camera:=false"
echo

exec roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false
