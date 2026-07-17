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

# 2. 检查 RunQueuedCmd
echo "检查运动队列..."
FEED_INFO="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
RUN_QUEUED="$(echo "$FEED_INFO" | grep -oP "RunQueuedCmd: \K\d+" || echo "0")"

if [ "$RUN_QUEUED" -ne 0 ]; then
    echo "[ERROR] 运动队列不为空 (RunQueuedCmd=$RUN_QUEUED)"
    echo "  请等待运动完成或手动停止"
    exit 1
fi

# 3. 显示当前状态
echo
echo "当前机器人状态:"
rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null | grep -E "is_enable|RobotMode"
rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null | grep -E "EnableStatus|ErrorStatus|RunQueuedCmd"

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

# 6. 等待下使能完成
echo
echo "等待下使能完成..."
TIMEOUT=20
ELAPSED=0
SUCCESS=false

while [ "$ELAPSED" -lt "$TIMEOUT" ]; do
    FEED_INFO="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
    ROBOT_STATUS="$(rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "")"

    if echo "$FEED_INFO" | grep -q "EnableStatus: 0" && \
       echo "$ROBOT_STATUS" | grep -q "is_enable: False"; then
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
    rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null | grep -E "is_enable|RobotMode"
    rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null | grep -E "EnableStatus|ErrorStatus"

    # 采样关节角检查是否漂移
    echo
    echo "采样关节角检查漂移..."
    ANGLE1="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "")"
    sleep 1
    ANGLE2="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "")"
    sleep 1
    ANGLE3="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "")"

    echo "采样 1: $ANGLE1"
    echo "采样 2: $ANGLE2"
    echo "采样 3: $ANGLE3"
else
    echo "[FAIL] 下使能超时 (${TIMEOUT}s)"
    echo "请检查机器人状态"
    exit 1
fi
