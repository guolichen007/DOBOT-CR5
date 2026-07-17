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

# 3. 检查错误状态（fail-closed）
echo "检查机器人错误状态..."
ERROR_ID="$(get_robot_status_field ErrorID)"
if [ "$ERROR_ID" = "ERROR" ]; then
    echo "[ERROR] 无法读取错误状态"
    echo "  请手动检查机器人状态"
    exit 1
fi

echo "GetErrorID: $ERROR_ID"

# 精确检查错误码是否为 0
if [ "$ERROR_ID" != "0" ]; then
    echo "[ERROR] 机器人存在错误 (ErrorID=$ERROR_ID)，请先清除错误"
    exit 1
fi

# 4. 显示当前状态
echo
echo "当前机器人状态:"
ROBOT_MODE="$(get_robot_status_field RobotMode)"
echo "RobotMode: $ROBOT_MODE"

if [ "$ROBOT_MODE" = "ERROR" ]; then
    echo "[ERROR] 无法读取 RobotMode"
    exit 1
fi

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

# 8. 等待使能完成（四项全部验证）
echo
echo "等待使能完成..."
TIMEOUT=20
ELAPSED=0
SUCCESS=false

while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
    ROBOT_MODE="$(get_robot_status_field RobotMode 2>/dev/null || echo "ERROR")"
    IS_ENABLE="$(get_robot_status_field is_enable 2>/dev/null || echo "ERROR")"
    ENABLE_STATUS="$(get_robot_status_field EnableStatus 2>/dev/null || echo "ERROR")"
    ERROR_STATUS="$(get_robot_status_field ErrorStatus 2>/dev/null || echo "ERROR")"

    # 四项全部验证
    if [ "$ROBOT_MODE" = "5" ] && [ "$IS_ENABLE" = "True" ] && \
       [ "$ENABLE_STATUS" = "1" ] && [ "$ERROR_STATUS" = "0" ]; then
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
    echo "  RobotMode: $ROBOT_MODE"
    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"
else
    echo "[FAIL] 使能超时 (${TIMEOUT}s)"
    echo "当前状态:"
    echo "  RobotMode: $ROBOT_MODE"
    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"
    echo "请检查机器人状态"
    exit 1
fi
