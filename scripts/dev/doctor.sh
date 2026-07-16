#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# doctor.sh - CR5 系统全面诊断
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() { echo "[PASS] $*"; ((PASS_COUNT++)); }
warn() { echo "[WARN] $*"; ((WARN_COUNT++)); }
fail() { echo "[FAIL] $*"; ((FAIL_COUNT++)); }

echo "=========================================="
echo "  CR5 系统诊断"
echo "=========================================="
echo "时间: $(date)"
echo

# 1. Git 状态
echo "--- Git 状态 ---"
if [ -d "$WS/.git" ]; then
    BRANCH="$(git -C "$WS" branch --show-current 2>/dev/null || echo 'unknown')"
    SHA="$(git -C "$WS" rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
    STATUS="$(git -C "$WS" status --short 2>/dev/null || echo 'unknown')"
    pass "Git 仓库: $WS"
    echo "  分支: $BRANCH"
    echo "  SHA: $SHA"
    echo "  状态: ${STATUS:-clean}"
else
    fail "Git 仓库不存在: $WS"
fi

# 2. ROS 环境
echo
echo "--- ROS 环境 ---"
if [ -n "${ROS_DISTRO:-}" ]; then
    pass "ROS_DISTRO: $ROS_DISTRO"
else
    fail "ROS_DISTRO 未设置"
fi

if [ -n "${ROS_PACKAGE_PATH:-}" ]; then
    pass "ROS_PACKAGE_PATH 已设置"
    echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done
else
    fail "ROS_PACKAGE_PATH 未设置"
fi

# 3. ROS 包
echo
echo "--- ROS 包 ---"
for PKG in cr5_book_spray_demo dobot_bringup dobot_moveit realsense2_camera; do
    if rospack find "$PKG" &>/dev/null; then
        pass "$PKG: $(rospack find "$PKG")"
    else
        fail "$PKG 未找到"
    fi
done

# 4. 网络连接
echo
echo "--- 网络连接 ---"
if ping -c 1 -W 1 192.168.110.214 &>/dev/null; then
    pass "CR5 控制柜可达 (192.168.110.214)"
else
    warn "CR5 控制柜不可达 (192.168.110.214)"
fi

# 5. USB 设备
echo
echo "--- USB 设备 ---"
if lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
    pass "D455 USB 设备检测到"
else
    warn "D455 USB 设备未检测到"
fi

# 6. ROS Master
echo
echo "--- ROS Master ---"
if rostopic list &>/dev/null; then
    pass "ROS Master 可达"
    TOPIC_COUNT="$(rostopic list 2>/dev/null | wc -l)"
    echo "  话题数量: $TOPIC_COUNT"
else
    warn "ROS Master 不可达（可能未启动）"
fi

# 7. 进程检查
echo
echo "--- 运行中的进程 ---"
for PROC in roslaunch rosrun move_group realsense2_camera_node dobot_bringup; do
    if pgrep -f "$PROC" &>/dev/null; then
        pass "$PROC 正在运行"
    else
        echo "[INFO] $PROC 未运行"
    fi
done

# 8. 磁盘空间
echo
echo "--- 磁盘空间 ---"
DISK_USAGE="$(df -h "$WS" | tail -1 | awk '{print $5}' | tr -d '%')"
if [ "$DISK_USAGE" -lt 80 ]; then
    pass "磁盘使用: ${DISK_USAGE}%"
else
    warn "磁盘使用: ${DISK_USAGE}%（建议清理）"
fi

# 9. 日志目录
echo
echo "--- 日志目录 ---"
LOG_DIR="$HOME/cr5_test_logs"
if [ -d "$LOG_DIR" ]; then
    LOG_COUNT="$(find "$LOG_DIR" -type f | wc -l)"
    pass "日志目录: $LOG_DIR ($LOG_COUNT 个文件)"
else
    echo "[INFO] 日志目录不存在: $LOG_DIR"
fi

# 总结
echo
echo "=========================================="
echo "  诊断总结"
echo "=========================================="
echo "PASS: $PASS_COUNT"
echo "WARN: $WARN_COUNT"
echo "FAIL: $FAIL_COUNT"

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo
    echo "[RESULT] 存在失败项，请检查"
    exit 1
elif [ "$WARN_COUNT" -gt 0 ]; then
    echo
    echo "[RESULT] 存在警告，但可以继续"
    exit 0
else
    echo
    echo "[RESULT] 所有检查通过"
    exit 0
fi
