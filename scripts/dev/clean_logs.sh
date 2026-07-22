#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# clean_logs.sh - 清理日志文件
# 用法: clean_logs [--older-than DAYS|--all|--show]
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

MODE="${1:---show}"
LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"

# 安全检查：确保 LOG_DIR 不是危险路径
if [ -z "$LOG_DIR" ] || [ "$LOG_DIR" = "/" ] || [ "$LOG_DIR" = "$HOME" ]; then
    echo "[ERROR] LOG_DIR 路径不安全: $LOG_DIR"
    exit 1
fi

echo "=========================================="
echo "  清理日志文件"
echo "=========================================="
echo "日志目录: $LOG_DIR"

# 统计
if [ -d "$LOG_DIR" ]; then
    LOG_COUNT="$(find "$LOG_DIR" -type f 2>/dev/null | wc -l)"
    LOG_SIZE="$(du -sh "$LOG_DIR" 2>/dev/null | cut -f1)"
    echo "文件数量: $LOG_COUNT"
    echo "占用空间: $LOG_SIZE"
else
    echo "[INFO] 日志目录不存在"
    exit 0
fi

case "$MODE" in
    --show)
        echo
        echo "最近的日志:"
        find "$LOG_DIR" -type f -mtime -7 -printf "  %T+ %p\n" 2>/dev/null | sort -r | head -10
        echo
        echo "用法:"
        echo "  clean_logs --older-than 7   # 清理 7 天前的日志"
        echo "  clean_logs --older-than 30  # 清理 30 天前的日志"
        echo "  clean_logs --all            # 清理所有日志"
        ;;

    --older-than)
        DAYS="${2:-7}"
        echo
        echo "清理 $DAYS 天前的日志..."
        DELETE_COUNT="$(find "$LOG_DIR" -type f -mtime +"$DAYS" 2>/dev/null | wc -l)"
        echo "将删除 $DELETE_COUNT 个文件"

        read -p "确认删除？(y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            find "$LOG_DIR" -type f -mtime +"$DAYS" -delete
            echo "已删除 $DELETE_COUNT 个文件"
        else
            echo "取消"
        fi
        ;;

    --all)
        echo
        echo "清理所有日志..."
        read -p "确定清理所有日志？(y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$LOG_DIR"/*
            echo "已清理所有日志"
        else
            echo "取消"
        fi
        ;;

    *)
        echo "[ERROR] 未知模式: $MODE"
        echo "用法: clean_logs [--older-than DAYS|--all|--show]"
        exit 1
        ;;
esac

# 清理 ROS 日志
ROS_LOG_DIR="${ROS_LOG_DIR:-$HOME/.ros/log}"
if [ -d "$ROS_LOG_DIR" ]; then
    ROS_LOG_COUNT="$(find "$ROS_LOG_DIR" -type f 2>/dev/null | wc -l)"
    ROS_LOG_SIZE="$(du -sh "$ROS_LOG_DIR" 2>/dev/null | cut -f1)"
    echo
    echo "ROS 日志: $ROS_LOG_DIR ($ROS_LOG_COUNT 个文件, $ROS_LOG_SIZE)"

    if [ "$MODE" != "--show" ]; then
        read -p "是否清理 ROS 日志？(y/N) " -n 1 -r
        echo
        if [[ $REPLY =~ ^[Yy]$ ]]; then
            rm -rf "$ROS_LOG_DIR"/*
            echo "已清理 ROS 日志"
        fi
    fi
fi

echo
echo "清理完成"
