#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# plan_book_demo_only.sh
# MoveIt plan-only 测试（禁止实体执行）
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$(cd "$SCRIPT_DIR/../dev" && pwd)"

# 复用 common.sh
source "$DEV_DIR/common.sh"

# 加载环境
load_cr5_environment

# 验证 ROS 包
verify_ros_package cr5_book_spray_demo
verify_ros_package realsense2_camera

echo "=========================================="
echo "  PLAN-ONLY MODE"
echo "=========================================="
echo "allow_execution=false"
echo "path_mode=single_stroke"
echo "orientation_mode=keep_current"
echo "eef_link=Tool_end"
echo
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
