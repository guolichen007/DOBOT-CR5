#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# stop_all.sh - 停止所有 CR5 相关进程
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  停止所有 CR5 相关进程"
echo "=========================================="

# 加载环境
load_cr5_environment 2>/dev/null || true

STOPPED=0

# 1. 停止书本识别
echo
echo "--- 停止书本识别 ---"
if rostopic list 2>/dev/null | grep -q "/book_demo/estimator/debug_image"; then
    echo "停止书本识别节点..."
    rosnode kill /book_demo/estimator 2>/dev/null || true
    rosnode kill /book_demo/planner 2>/dev/null || true
    STOPPED=$((STOPPED + 1))
else
    echo "[INFO] 书本识别未运行"
fi

# 2. 停止 RealSense
echo
echo "--- 停止 RealSense 相机 ---"
if rosnode list 2>/dev/null | grep -q "/camera/realsense2_camera"; then
    echo "停止 RealSense 节点..."
    rosnode kill /camera/realsense2_camera 2>/dev/null || true
    rosnode kill /camera/realsense2_camera_manager 2>/dev/null || true
    STOPPED=$((STOPPED + 1))
else
    echo "[INFO] RealSense 相机未运行"
fi

# 3. 停止 MoveIt
echo
echo "--- 停止 MoveIt ---"
if rosnode list 2>/dev/null | grep -q "/move_group"; then
    echo "停止 MoveIt 节点..."
    rosnode kill /move_group 2>/dev/null || true
    STOPPED=$((STOPPED + 1))
else
    echo "[INFO] MoveIt 未运行"
fi

# 4. 停止 Driver（需要安全检查）
echo
echo "--- 停止 CR5 Driver ---"
if rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    # 检查机器人是否使能
    echo "检查机器人状态..."
    FEED_INFO="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
    ROBOT_STATUS="$(rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "")"

    ENABLE_STATUS="$(echo "$FEED_INFO" | grep -oP "EnableStatus: \K\d+" || echo "0")"
    IS_ENABLE="$(echo "$ROBOT_STATUS" | grep -oP "is_enable: \K\w+" || echo "False")"
    RUN_QUEUED="$(echo "$FEED_INFO" | grep -oP "RunQueuedCmd: \K\d+" || echo "0")"

    echo "EnableStatus: $ENABLE_STATUS"
    echo "is_enable: $IS_ENABLE"
    echo "RunQueuedCmd: $RUN_QUEUED"

    if [ "$ENABLE_STATUS" = "1" ] || [ "$IS_ENABLE" = "True" ]; then
        echo
        echo "[ERROR] 机器人仍处于使能状态"
        echo "  请先执行: disable_robot_safe"
        echo "  然后再执行: stop_all"
        exit 1
    fi

    if [ "$RUN_QUEUED" -ne 0 ]; then
        echo
        echo "[ERROR] 运动队列不为空 (RunQueuedCmd=$RUN_QUEUED)"
        echo "  请等待运动完成或手动停止"
        exit 1
    fi

    echo "停止 CR5 Driver..."
    rosnode kill /cr5_robot 2>/dev/null || true
    STOPPED=$((STOPPED + 1))
else
    echo "[INFO] CR5 Driver 未运行"
fi

# 5. 清理 PID 文件
echo
echo "--- 清理 PID 文件 ---"
if [ -d "$RUN_DIR" ]; then
    rm -f "$RUN_DIR"/*.pid
    echo "已清理 PID 文件"
fi

# 6. 等待进程退出
echo
echo "等待进程退出..."
sleep 2

# 7. 检查残留
echo
echo "--- 残留进程检查 ---"
for NODE in /cr5_robot /move_group /camera/realsense2_camera /book_demo/estimator; do
    if rosnode list 2>/dev/null | grep -q "^${NODE}$"; then
        echo "[WARN] $NODE 仍在运行"
    fi
done

echo
echo "=========================================="
echo "  停止完成"
echo "=========================================="
echo
echo "已停止 $STOPPED 组进程"

if rosnode list 2>/dev/null | grep -q "/rosout"; then
    echo
    echo "注意: roscore 仍在运行"
    echo "如需停止: pkill -f roscore"
fi
