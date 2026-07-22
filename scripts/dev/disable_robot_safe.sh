#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# disable_robot_safe.sh - 安全下使能机器人
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  安全下使能 CR5 机器人"
echo "=========================================="

# 加载环境
load_cr5_environment

# 1. 检查 Driver
if ! rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    echo "[ERROR] CR5 Driver 未运行"
    exit 1
fi

# 2. 检查 RunQueuedCmd（fail-closed）
echo "检查运动队列..."
RUN_QUEUED="$(get_robot_status_field RunQueuedCmd)"

if [ "$RUN_QUEUED" = "ERROR" ]; then
    echo "[ERROR] 无法读取 RunQueuedCmd"
    echo "  请手动检查机器人状态"
    exit 1
fi

echo "RunQueuedCmd: $RUN_QUEUED"

if [ "$RUN_QUEUED" -ne 0 ]; then
    echo "[ERROR] 运动队列不为空 (RunQueuedCmd=$RUN_QUEUED)"
    echo "  请等待运动完成或手动停止"
    exit 1
fi

# 3. 检查当前使能状态
IS_ENABLE="$(get_robot_status_field is_enable)"
ENABLE_STATUS="$(get_robot_status_field EnableStatus)"

if [ "$IS_ENABLE" = "ERROR" ] || [ "$ENABLE_STATUS" = "ERROR" ]; then
    echo "[ERROR] 无法读取使能状态"
    echo "  请手动检查机器人状态"
    exit 1
fi

echo
echo "当前机器人状态:"
echo "  is_enable: $IS_ENABLE"
echo "  EnableStatus: $ENABLE_STATUS"

if [ "$IS_ENABLE" = "False" ] && [ "$ENABLE_STATUS" = "0" ]; then
    echo
    echo "[INFO] 机器人已处于下使能状态"
    exit 0
fi

# 4. 要求确认
echo
read -p "确认下使能机器人？(y/N) " -n 1 -r
echo
if [[ ! $REPLY =~ ^[Yy]$ ]]; then
    echo "取消下使能"
    exit 1
fi

# 5. 调用下使能服务
echo
echo "调用 DisableRobot 服务..."
RESULT="$(rosservice call /dobot_bringup/srv/DisableRobot "{}" 2>/dev/null || echo "服务调用失败")"
echo "结果: $RESULT"

# 6. 等待下使能完成（fail-closed）
echo
echo "等待下使能完成..."
TIMEOUT=20
ELAPSED=0
SUCCESS=false

while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
    IS_ENABLE="$(get_robot_status_field is_enable 2>/dev/null || echo "ERROR")"
    ENABLE_STATUS="$(get_robot_status_field EnableStatus 2>/dev/null || echo "ERROR")"
    ERROR_STATUS="$(get_robot_status_field ErrorStatus 2>/dev/null || echo "ERROR")"

    # 检查读取失败
    if [ "$IS_ENABLE" = "ERROR" ] || [ "$ENABLE_STATUS" = "ERROR" ] || [ "$ERROR_STATUS" = "ERROR" ]; then
        echo
        echo "[ERROR] 无法读取机器人状态"
        exit 1
    fi

    # 验证下使能状态
    if [ "$IS_ENABLE" = "False" ] && [ "$ENABLE_STATUS" = "0" ] && [ "$ERROR_STATUS" = "0" ]; then
        SUCCESS=true
        break
    fi

    sleep 1
    ELAPSED=$((ELAPSED + 1))
    echo -n "."
done
echo

if [ "$SUCCESS" = true ]; then
    echo "[SUCCESS] 机器人已下使能"
    echo
    echo "注意：下使能时可能有抱闸收敛，机械臂可能会有轻微移动"
    echo
    echo "验证状态:"
    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"

    # 采样关节角（供人工检查漂移）
    echo
    echo "采样关节角（供人工检查漂移）..."
    ANGLE1="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "读取失败")"
    sleep 1
    ANGLE2="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "读取失败")"
    sleep 1
    ANGLE3="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "读取失败")"

    echo "采样 1: $ANGLE1"
    echo "采样 2: $ANGLE2"
    echo "采样 3: $ANGLE3"
    echo
    echo "[INFO] 请人工检查关节角是否有明显漂移"
else
    echo "[FAIL] 下使能超时 (${TIMEOUT}s)"
    echo "当前状态:"
    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"
    echo "请检查机器人状态"
    exit 1
fi
