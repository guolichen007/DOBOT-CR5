#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_all.sh - 启动所有组件
# 用法: start_all {vision|planning}
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

MODE="${1:-}"

echo "=========================================="
echo "  启动所有组件"
echo "=========================================="

# 显示帮助
show_help() {
    echo
    echo "用法: start_all {vision|planning}"
    echo
    echo "模式:"
    echo "  vision    - 纯视觉模式（不启动 MoveIt，不使能机器人）"
    echo "  planning  - 规划模式（启动 MoveIt，不自动使能）"
    echo
    echo "示例:"
    echo "  start_all vision"
    echo "  start_all planning"
    echo
}

if [ -z "$MODE" ]; then
    show_help
    exit 0
fi

# 加载环境
load_cr5_environment

case "$MODE" in
    vision)
        echo
        echo "模式: 纯视觉"
        echo "启动: Camera + Book Demo"
        echo "不启动: MoveIt"
        echo "不使能: 机器人"
        echo

        # 调用 start_vision
        exec "$SCRIPT_DIR/start_vision.sh"
        ;;

    planning)
        echo
        echo "模式: 规划"
        echo "启动: Driver + MoveIt + Camera"
        echo "不启动: Book Demo"
        echo "不自动使能: 机器人"
        echo

        # 1. 启动 Driver
        echo "--- 步骤 1/3: 启动 CR5 Driver ---"
        if rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
            echo "[INFO] CR5 Driver 已在运行"
        else
            "$SCRIPT_DIR/start_driver.sh"
        fi

        # 2. 启动 MoveIt
        echo
        echo "--- 步骤 2/3: 启动 MoveIt ---"
        if rosnode list 2>/dev/null | grep -q "/move_group"; then
            echo "[INFO] MoveIt 已在运行"
        else
            "$SCRIPT_DIR/start_moveit.sh"
        fi

        # 3. 启动相机
        echo
        echo "--- 步骤 3/3: 启动 D455 相机 ---"
        if rosnode list 2>/dev/null | grep -q "/camera/realsense2_camera"; then
            echo "[INFO] D455 相机已在运行"
        else
            "$SCRIPT_DIR/start_camera.sh"
        fi

        echo
        echo "=========================================="
        echo "  规划模式启动完成"
        echo "=========================================="
        echo
        echo "下一步:"
        echo "  robot_status       - 查看机器人状态"
        echo "  enable_robot_safe  - 安全使能机器人"
        echo "  start_book_demo    - 启动书本识别"
        echo
        echo "重要提示:"
        echo "  Velocity Scaling = 0.03～0.05"
        echo "  Accel. Scaling = 0.03～0.05"
        echo "  Start State = Current"
        echo "  先 Plan，审核后再 Execute"
        echo
        ;;

    *)
        echo "[ERROR] 未知模式: $MODE"
        show_help
        exit 1
        ;;
esac
