#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_camera.sh - 启动 D455 相机
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  启动 D455 相机"
echo "=========================================="

# 检查是否已在运行
if pgrep -f "realsense2_camera_node" &>/dev/null; then
    echo "[WARN] RealSense 相机已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 检查 USB 设备
if ! lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
    echo "[WARN] D455 USB 设备未检测到"
    echo "请检查 USB 连接"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 加载环境
source "$SCRIPTS_DEV_DIR/env.sh" 2>/dev/null || true

echo "启动 D455 相机..."
echo "参数: align_depth:=true"
echo

exec roslaunch cr5_book_spray_demo d455_camera.launch
