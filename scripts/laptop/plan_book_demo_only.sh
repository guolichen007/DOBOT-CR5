#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# plan_book_demo_only.sh
# MoveIt plan-only 测试（禁止实体执行）
# ============================================================

WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

fail() { echo "[ERROR] $*" >&2; exit 1; }

# ============================================================
# 环境加载函数（使用 --extend 叠加）
# ============================================================
load_ros_environment() {
    # 1. 加载 ROS 基础环境
    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        fail "ROS Noetic 未安装"
    fi
    source /opt/ros/noetic/setup.bash

    # 2. 加载 RealSense 工作空间
    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        fail "RealSense 工作空间未编译: $REALSENSE_WS
请运行: bash $WS/scripts/laptop/setup_realsense_ros1.sh"
    fi
    source "$REALSENSE_WS/devel/setup.bash"

    # 3. 加载 CR5 工作空间（使用 --extend 叠加）
    if [ ! -f "$WS/devel/setup.bash" ]; then
        fail "CR5 工作空间未编译: $WS
请运行: bash $WS/scripts/laptop/pull_build_book_demo.sh"
    fi
    source "$WS/devel/setup.bash" --extend
}

# ============================================================
# 加载环境
# ============================================================
load_ros_environment

# 验证 ROS 包
rospack find cr5_book_spray_demo &>/dev/null || fail "cr5_book_spray_demo 未找到"
rospack find realsense2_camera &>/dev/null || fail "realsense2_camera 未找到"

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
