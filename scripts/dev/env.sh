#!/bin/bash
# ============================================================
# env.sh - CR5 开发环境加载
# 使用方法: source scripts/dev/env.sh
# ============================================================

# 项目根目录
export CR5_WS="${CR5_WS:-$HOME/cr5_ros1_ws}"
export REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

# 加载 ROS 基础环境
source /opt/ros/noetic/setup.bash

# 加载 RealSense 工作空间
if [ -f "$REALSENSE_WS/devel/setup.bash" ]; then
    source "$REALSENSE_WS/devel/setup.bash"
else
    echo "[WARN] RealSense 工作空间未编译: $REALSENSE_WS"
fi

# 加载 CR5 工作空间（使用 --extend 叠加）
if [ -f "$CR5_WS/devel/setup.bash" ]; then
    source "$CR5_WS/devel/setup.bash" --extend
else
    echo "[WARN] CR5 工作空间未编译: $CR5_WS"
fi

# 网络配置
unset ROS_IP 2>/dev/null || true
unset ROS_HOSTNAME 2>/dev/null || true
export ROS_MASTER_URI=http://127.0.0.1:11311

# 导出工具函数
export SCRIPTS_DEV_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

# 快捷命令
alias doctor="$SCRIPTS_DEV_DIR/doctor.sh"
alias build="$SCRIPTS_DEV_DIR/build.sh"
alias start_driver="$SCRIPTS_DEV_DIR/start_driver.sh"
alias start_moveit="$SCRIPTS_DEV_DIR/start_moveit.sh"
alias start_camera="$SCRIPTS_DEV_DIR/start_camera.sh"
alias start_book_demo="$SCRIPTS_DEV_DIR/start_book_demo.sh"
alias start_all="$SCRIPTS_DEV_DIR/start_all.sh"
alias stop_all="$SCRIPTS_DEV_DIR/stop_all.sh"
alias clean_logs="$SCRIPTS_DEV_DIR/clean_logs.sh"

echo
echo "================================="
echo " CR5 Environment Loaded"
echo "================================="
echo
echo "Workspace: $CR5_WS"
echo "RealSense: $REALSENSE_WS"
echo
echo "可用命令:"
echo "  doctor          - 全面诊断"
echo "  build           - 编译项目"
echo "  start_driver    - 启动 CR5 Driver"
echo "  start_moveit    - 启动 MoveIt"
echo "  start_camera    - 启动 D455 相机"
echo "  start_book_demo - 启动书本识别"
echo "  start_all       - 一键启动所有"
echo "  stop_all        - 停止所有"
echo "  clean_logs      - 清理日志"
echo
