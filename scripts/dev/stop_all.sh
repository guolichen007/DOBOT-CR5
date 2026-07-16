#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# stop_all.sh - 停止所有 CR5 相关进程
# ============================================================

echo "=========================================="
echo "  停止所有 CR5 相关进程"
echo "=========================================="

STOPPED=0

# 停止书本识别
if pgrep -f "book_pose_estimator\|book_spray_planner\|vision_only.launch" &>/dev/null; then
    echo "停止书本识别..."
    pkill -f "book_pose_estimator" 2>/dev/null || true
    pkill -f "book_spray_planner" 2>/dev/null || true
    pkill -f "vision_only.launch" 2>/dev/null || true
    ((STOPPED++))
fi

# 停止 RealSense
if pgrep -f "realsense2_camera_node\|rs_camera.launch\|d455_camera.launch" &>/dev/null; then
    echo "停止 RealSense 相机..."
    pkill -f "realsense2_camera_node" 2>/dev/null || true
    pkill -f "rs_camera.launch" 2>/dev/null || true
    pkill -f "d455_camera.launch" 2>/dev/null || true
    ((STOPPED++))
fi

# 停止 MoveIt
if pgrep -f "move_group\|moveit.launch" &>/dev/null; then
    echo "停止 MoveIt..."
    pkill -f "move_group" 2>/dev/null || true
    pkill -f "moveit.launch" 2>/dev/null || true
    ((STOPPED++))
fi

# 停止 CR5 Driver
if pgrep -f "dobot_bringup\|bringup.launch" &>/dev/null; then
    echo "停止 CR5 Driver..."
    pkill -f "dobot_bringup" 2>/dev/null || true
    pkill -f "bringup.launch" 2>/dev/null || true
    ((STOPPED++))
fi

# 停止 roscore（可选）
if pgrep -f "roscore" &>/dev/null; then
    read -p "是否停止 roscore？(y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        echo "停止 roscore..."
        pkill -f "roscore" 2>/dev/null || true
        ((STOPPED++))
    fi
fi

echo
if [ "$STOPPED" -gt 0 ]; then
    echo "[INFO] 已停止 $STOPPED 组进程"
else
    echo "[INFO] 没有需要停止的进程"
fi

# 等待进程完全退出
sleep 2

# 检查是否还有残留
echo
echo "--- 残留进程检查 ---"
for PROC in dobot_bringup move_group realsense2_camera_node book_pose_estimator; do
    if pgrep -f "$PROC" &>/dev/null; then
        echo "[WARN] $PROC 仍在运行"
    fi
done

echo
echo "停止完成"
