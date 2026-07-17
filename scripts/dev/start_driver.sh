#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_driver.sh - 启动 CR5 Driver
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  启动 CR5 Driver"
echo "=========================================="

# 1. 加载环境
load_cr5_environment

# 2. 验证包
verify_ros_package dobot_bringup

# 3. 检查端口
echo "检查控制柜端口..."
for PORT in 29999 30003 30004; do
    if timeout 2 bash -c "echo >/dev/tcp/192.168.110.214/$PORT" 2>/dev/null; then
        echo "[PASS] 端口 $PORT 可达"
    else
        echo "[WARN] 端口 $PORT 不可达"
    fi
done

# 4. 检查是否已在运行
if rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    echo "[WARN] CR5 Driver 已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 5. 启动 Driver
echo
echo "启动 dobot_bringup..."
echo "robot_ip: 192.168.110.214"
echo

# 后台启动并保存 PID
roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214 &
DRIVER_PID=$!
echo "$DRIVER_PID" > "$RUN_DIR/driver.pid"

# 6. 等待节点就绪
echo "等待 CR5 Driver 就绪..."
wait_for_ros_node /cr5_robot 30
wait_for_topic_publisher /joint_states 30

# 7. 等待服务就绪
wait_for_service /dobot_bringup/srv/EnableRobot 10
wait_for_service /dobot_bringup/srv/DisableRobot 10

# 8. 输出状态
echo
echo "CR5 Driver 已启动"
echo "PID: $DRIVER_PID"
echo

# 只读状态
robot_readonly_status

echo
echo "=========================================="
echo "  Driver 启动完成"
echo "=========================================="
echo
echo "下一步:"
echo "  robot_status      - 查看机器人状态"
echo "  enable_robot_safe - 安全使能机器人"
echo "  start_moveit      - 启动 MoveIt"
echo
