#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# enable_robot_safe.sh - 安全使能机器人
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  安全使能 CR5 机器人"
echo "=========================================="

# 加载环境
load_cr5_environment

# 1. 检查 Driver
if ! rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    echo "[ERROR] CR5 Driver 未运行"
    echo "  请先启动: start_driver"
    exit 1
fi

# 2. 检查服务
if ! rosservice list 2>/dev/null | grep -q "/dobot_bringup/srv/EnableRobot"; then
    echo "[ERROR] EnableRobot 服务不可用"
    exit 1
fi

# 3. 检查错误
echo "检查机器人错误状态..."
ERROR_ID="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetErrorID()'" 2>/dev/null || echo "")"
echo "GetErrorID: $ERROR_ID"

if echo "$ERROR_ID" | grep -qv "0\b"; then
    echo "[ERROR] 机器人存在错误，请先清除错误"
    exit 1
fi

# 4. 显示当前状态
echo
echo "当前机器人状态:"
ROBOT_MODE="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'RobotMode()'" 2>/dev/null || echo "")"
echo "RobotMode: $ROBOT_MODE"

# 5. 安全提示
echo
echo "=========================================="
echo "  安全提示"
echo "=========================================="
echo "使能前请确认："
echo "  1. 机器人工作区内无人"
echo "  2. 急停按钮可触及"
echo "  3. 机械臂不会碰撞障碍物"
echo

# 6. 要求确认
read -p "请输入确认词 ENABLE_CR5 以继续: " CONFIRM
if [ "$CONFIRM" != "ENABLE_CR5" ]; then
    echo "取消使能"
    exit 1
fi

# 7. 调用使能服务
echo
echo "调用 EnableRobot 服务..."
RESULT="$(rosservice call /dobot_bringup/srv/EnableRobot "args: []" 2>/dev/null || echo "服务调用失败")"
echo "结果: $RESULT"

# 8. 等待使能完成
echo
echo "等待使能完成..."
TIMEOUT=20
ELAPSED=0
SUCCESS=false

while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
    FEED_INFO="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
    ROBOT_STATUS="$(rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "")"

    if echo "$FEED_INFO" | grep -q "EnableStatus: 1" && \
       echo "$ROBOT_STATUS" | grep -q "is_enable: True"; then
        SUCCESS=true
        break
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done
echo

if [ "$SUCCESS" = true ]; then
    echo "[SUCCESS] 机器人已使能"
    echo
    echo "验证状态:"
    ROBOT_MODE="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'RobotMode()'" 2>/dev/null || echo "")"
    echo "RobotMode: $ROBOT_MODE"
    rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null | grep -E "is_enable|RobotMode"
    rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null | grep -E "EnableStatus|ErrorStatus"
else
    echo "[FAIL] 使能超时 (${TIMEOUT}s)"
    echo "请检查机器人状态"
    exit 1
fi
