#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_moveit.sh - 启动 MoveIt
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  启动 MoveIt"
echo "=========================================="

# 1. 加载环境
load_ros_environment

# 2. 验证包
verify_ros_package dobot_moveit

# 3. 检查 Driver
if ! rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    echo "[ERROR] CR5 Driver 未运行"
    echo "  请先启动: start_driver"
    exit 1
fi

# 4. 检查 /joint_states
echo "检查 /joint_states..."
if ! rostopic list 2>/dev/null | grep -q "^/joint_states$"; then
    echo "[ERROR] /joint_states 话题不存在"
    exit 1
fi

# 等待至少 3 帧
echo "等待 /joint_states 数据..."
JOINT_COUNT=0
for i in {1..10}; do
    if timeout 2 rostopic echo -n 1 /joint_states &>/dev/null; then
        JOINT_COUNT=$((JOINT_COUNT + 1))
        if [ "$JOINT_COUNT" -ge 3 ]; then
            echo "[PASS] /joint_states 数据正常"
            break
        fi
    fi
    sleep 0.5
done

if [ "$JOINT_COUNT" -lt 3 ]; then
    echo "[WARN] /joint_states 数据不足"
fi

# 5. 输出当前关节角
echo
echo "当前关节角:"
timeout 2 rostopic echo -n 1 /joint_states 2>/dev/null | grep -A10 "position:" | head -12 || echo "  无法读取"

# 6. 检查是否已在运行
if rosnode list 2>/dev/null | grep -q "/move_group"; then
    echo "[WARN] MoveIt 已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 7. 启动 MoveIt
echo
echo "启动 MoveIt..."
roslaunch dobot_moveit moveit.launch &
MOVEIT_PID=$!
echo "$MOVEIT_PID" > "$RUN_DIR/moveit.pid"

# 8. 等待节点就绪
echo "等待 MoveIt 就绪..."
wait_for_ros_node /move_group 30

# 9. 检查 FollowJointTrajectory Action
echo "检查 FollowJointTrajectory Action..."
sleep 2
if rostopic list 2>/dev/null | grep -q "follow_joint_trajectory"; then
    echo "[PASS] FollowJointTrajectory Action 存在"
else
    echo "[WARN] FollowJointTrajectory Action 未检测到"
fi

echo
echo "=========================================="
echo "  MoveIt 启动完成"
echo "=========================================="
echo
echo "重要提示:"
echo "  Velocity Scaling = 0.03～0.05"
echo "  Accel. Scaling = 0.03～0.05"
echo "  Start State = Current"
echo "  先 Plan，审核后再 Execute"
echo
echo "下一步:"
echo "  start_camera      - 启动 D455 相机"
echo "  start_book_demo   - 启动书本识别"
echo
