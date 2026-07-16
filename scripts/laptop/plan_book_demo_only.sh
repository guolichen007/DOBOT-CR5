#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# plan_book_demo_only.sh
# MoveIt plan-only 测试（禁止实体执行）
# ============================================================

WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

# ============================================================
# 环境加载函数
# ============================================================
load_ros_environment() {
    source /opt/ros/noetic/setup.bash

    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        echo "[ERROR] RealSense workspace is not built:"
        echo "        $REALSENSE_WS"
        echo
        echo "Run:"
        echo "  bash $WS/scripts/laptop/setup_realsense_ros1.sh"
        exit 1
    fi

    source "$REALSENSE_WS/devel/setup.bash"

    if [ ! -f "$WS/devel/local_setup.bash" ]; then
        echo "[ERROR] CR5 workspace is not built:"
        echo "        $WS"
        exit 1
    fi

    source "$WS/devel/local_setup.bash"
}

load_ros_environment

echo "=========================================="
echo "  PLAN-ONLY MODE"
echo "=========================================="
echo "allow_execution=false"
echo "path_mode=single_stroke"
echo "orientation_mode=keep_current"
echo "eef_link=Tool_end"
echo ""
echo "禁止："
echo "  - execute_path"
echo "  - confirm_execute"
echo "  - 机械臂使能"
echo "  - 喷阀/气源"
echo "=========================================="

exec roslaunch cr5_book_spray_demo planner_only.launch \
    allow_execution:=false \
    path_mode:=single_stroke \
    orientation_mode:=keep_current \
    eef_link:=Tool_end
