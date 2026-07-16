#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# clean_logs.sh - 清理日志文件
# ============================================================

LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  清理日志文件"
echo "=========================================="

# 统计当前日志
if [ -d "$LOG_DIR" ]; then
    LOG_COUNT="$(find "$LOG_DIR" -type f | wc -l)"
    LOG_SIZE="$(du -sh "$LOG_DIR" 2>/dev/null | cut -f1)"
    echo "日志目录: $LOG_DIR"
    echo "文件数量: $LOG_COUNT"
    echo "占用空间: $LOG_SIZE"
else
    echo "日志目录不存在: $LOG_DIR"
fi

# 清理选项
echo
echo "清理选项:"
echo "  1) 清理 7 天前的日志"
echo "  2) 清理 30 天前的日志"
echo "  3) 清理所有日志"
echo "  4) 退出"
echo

read -p "请选择 (1-4): " CHOICE

case "$CHOICE" in
    1)
        echo "清理 7 天前的日志..."
        find "$LOG_DIR" -type f -mtime +7 -delete 2>/dev/null || true
        ;;
    2)
        echo "清理 30 天前的日志..."
        find "$LOG_DIR" -type f -mtime +30 -delete 2>/dev/null || true
        ;;
    3)
        read -p "确定清理所有日志？(y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$LOG_DIR"/*
            echo "已清理所有日志"
        else
            echo "取消"
        fi
        ;;
    4)
        echo "退出"
        exit 0
        ;;
    *)
        echo "无效选择"
        exit 1
        ;;
esac

# 清理 ROS 日志
ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -d "$ROS_LOG_DIR" ]; then
    ROS_LOG_COUNT="$(find "$ROS_LOG_DIR" -type f | wc -l)"
    ROS_LOG_SIZE="$(du -sh "$ROS_LOG_DIR" 2>/dev/null | cut -f1)"
    echo
    echo "ROS 日志目录: $ROS_LOG_DIR"
    echo "文件数量: $ROS_LOG_COUNT"
    echo "占用空间: $ROS_LOG_SIZE"

    read -p "是否清理 ROS 日志？(y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -rf "$ROS_LOG_DIR"/*
        echo "已清理 ROS 日志"
    fi
fi

# 清理 catkin 构建日志
BUILD_LOG="$WS/build/CMakeFiles/*.log"
if ls $BUILD_LOG &>/dev/null; then
    echo
    read -p "是否清理 catkin 构建日志？(y/N) " -n 1 -r
    echo
    if [[ $REPLY =~ ^[Yy]$ ]]; then
        rm -f $BUILD_LOG
        echo "已清理 catkin 构建日志"
    fi
fi

echo
echo "清理完成"
