#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_all.sh - 一键启动所有组件
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  一键启动所有组件"
echo "=========================================="
echo
echo "启动顺序:"
echo "  1. CR5 Driver"
echo "  2. MoveIt"
echo "  3. D455 相机"
echo "  4. 书本识别"
echo

# 检查是否已有进程运行
for PROC in dobot_bringup move_group realsense2_camera_node; do
    if pgrep -f "$PROC" &>/dev/null; then
        echo "[WARN] $PROC 已在运行"
        echo "如需重启，请先执行: stop_all"
        exit 1
    fi
done

# 加载环境
source "$SCRIPTS_DEV_DIR/env.sh" 2>/dev/null || true

# 创建日志目录
LOG_DIR="$HOME/cr5_test_logs"
mkdir -p "$LOG_DIR"
STAMP="$(date +%Y%m%d_%H%M%S)"

echo "=========================================="
echo "  步骤 1/4: 启动 CR5 Driver"
echo "=========================================="

if ! ping -c 1 -W 1 192.168.110.214 &>/dev/null; then
    echo "[WARN] CR5 控制柜不可达，跳过 Driver 启动"
    SKIP_DRIVER=true
else
    roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214 &>"$LOG_DIR/driver_${STAMP}.log" &
    DRIVER_PID=$!
    echo "[INFO] CR5 Driver PID: $DRIVER_PID"
    sleep 3
fi

echo
echo "=========================================="
echo "  步骤 2/4: 启动 MoveIt"
echo "=========================================="

if [ "${SKIP_DRIVER:-false}" = true ]; then
    echo "[WARN] 跳过 MoveIt 启动（Driver 未启动）"
else
    roslaunch dobot_moveit moveit.launch &>"$LOG_DIR/moveit_${STAMP}.log" &
    MOVEIT_PID=$!
    echo "[INFO] MoveIt PID: $MOVEIT_PID"
    sleep 3
fi

echo
echo "=========================================="
echo "  步骤 3/4: 启动 D455 相机"
echo "=========================================="

if ! lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
    echo "[WARN] D455 USB 设备未检测到，跳过相机启动"
    SKIP_CAMERA=true
else
    roslaunch cr5_book_spray_demo d455_camera.launch &>"$LOG_DIR/camera_${STAMP}.log" &
    CAMERA_PID=$!
    echo "[INFO] D455 相机 PID: $CAMERA_PID"
    sleep 5
fi

echo
echo "=========================================="
echo "  步骤 4/4: 启动书本识别"
echo "=========================================="

if [ "${SKIP_CAMERA:-false}" = true ]; then
    echo "[WARN] 跳过书本识别启动（相机未启动）"
else
    # 检查相机话题
    if rostopic list 2>/dev/null | grep -q "/camera/color/image_raw"; then
        roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false &>"$LOG_DIR/book_demo_${STAMP}.log" &
        BOOK_PID=$!
        echo "[INFO] 书本识别 PID: $BOOK_PID"
    else
        echo "[WARN] 相机话题不存在，跳过书本识别启动"
    fi
fi

echo
echo "=========================================="
echo "  启动完成"
echo "=========================================="
echo
echo "启动的组件:"
[ -n "${DRIVER_PID:-}" ] && echo "  CR5 Driver: PID $DRIVER_PID"
[ -n "${MOVEIT_PID:-}" ] && echo "  MoveIt: PID $MOVEIT_PID"
[ -n "${CAMERA_PID:-}" ] && echo "  D455 相机: PID $CAMERA_PID"
[ -n "${BOOK_PID:-}" ] && echo "  书本识别: PID $BOOK_PID"
echo
echo "日志目录: $LOG_DIR"
echo
echo "停止所有: stop_all"
echo
