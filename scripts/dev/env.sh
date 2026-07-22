#!/bin/bash
# ============================================================
# env.sh - CR5 开发环境加载
# 使用方法: source scripts/dev/env.sh
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 加载公共函数
source "$SCRIPT_DIR/common.sh"

# 加载 ROS 环境
if ! load_cr5_environment; then
    echo "[ERROR] 环境加载失败" >&2
    return 1 2>/dev/null || exit 1
fi

# 验证 ROS 包
echo "验证 ROS 包..."
ALL_PASS=true

for PKG in realsense2_camera realsense2_description cr5_book_spray_demo dobot_bringup dobot_moveit; do
    if rospack find "$PKG" &>/dev/null; then
        echo "[PASS] $PKG"
    else
        echo "[WARN] $PKG 未找到"
        ALL_PASS=false
    fi
done

# 快捷命令
alias doctor="$SCRIPT_DIR/doctor.sh"
alias build="$SCRIPT_DIR/build.sh"
alias start_driver="$SCRIPT_DIR/start_driver.sh"
alias start_moveit="$SCRIPT_DIR/start_moveit.sh"
alias start_camera="$SCRIPT_DIR/start_camera.sh"
alias start_book_demo="$SCRIPT_DIR/start_book_demo.sh"
alias start_vision="$SCRIPT_DIR/start_vision.sh"
alias start_all="$SCRIPT_DIR/start_all.sh"
alias stop_all="$SCRIPT_DIR/stop_all.sh"
alias clean_logs="$SCRIPT_DIR/clean_logs.sh"
alias robot_status="$SCRIPT_DIR/robot_status.sh"
alias enable_robot_safe="$SCRIPT_DIR/enable_robot_safe.sh"
alias disable_robot_safe="$SCRIPT_DIR/disable_robot_safe.sh"

echo
echo "================================="
echo " CR5 Environment Loaded"
echo "================================="
echo
echo "Workspace: $CR5_WS"
echo "RealSense: $REALSENSE_WS"
echo
echo "ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
    [ -n "$p" ] && echo "  $p"
done
echo
echo "可用命令:"
echo "  doctor            - 全面诊断"
echo "  build             - 编译项目"
echo "  robot_status      - 机器人状态（只读）"
echo "  enable_robot_safe - 安全使能机器人"
echo "  disable_robot_safe- 安全下使能机器人"
echo "  start_driver      - 启动 CR5 Driver"
echo "  start_moveit      - 启动 MoveIt"
echo "  start_camera      - 启动 D455 相机"
echo "  start_book_demo   - 启动书本识别"
echo "  start_vision      - 一键启动视觉调试"
echo "  start_all         - 启动所有（需要参数）"
echo "  stop_all          - 停止所有"
echo "  clean_logs        - 清理日志"
echo
