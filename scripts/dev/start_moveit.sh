#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_moveit.sh - 启动 MoveIt
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  启动 MoveIt"
echo "=========================================="

# 检查 CR5 Driver 是否运行
if ! pgrep -f "dobot_bringup" &>/dev/null; then
    echo "[WARN] CR5 Driver 未运行"
    echo "请先启动: start_driver"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 检查是否已在运行
if pgrep -f "move_group" &>/dev/null; then
    echo "[WARN] MoveIt 已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 加载环境
source "$SCRIPTS_DEV_DIR/env.sh" 2>/dev/null || true

echo "启动 MoveIt..."
echo

exec roslaunch dobot_moveit moveit.launch
