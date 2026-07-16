#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# build.sh - 编译 CR5 工作空间
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"
LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/build_${STAMP}.log"

mkdir -p "$LOG_DIR"

echo "=========================================="
echo "  编译 CR5 工作空间"
echo "=========================================="
echo "工作空间: $WS"
echo "日志文件: $LOG_FILE"
echo

# 加载 ROS 环境
source /opt/ros/noetic/setup.bash

cd "$WS"

# 编译
echo "开始编译..."
if catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 2>&1 | tee "$LOG_FILE"; then
    echo
    echo "[SUCCESS] 编译成功"
    echo "日志: $LOG_FILE"
else
    echo
    echo "[FAIL] 编译失败"
    echo "日志: $LOG_FILE"
    exit 1
fi
