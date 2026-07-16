#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_driver.sh - 启动 CR5 Driver
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS="${CR5_WS:-$HOME/cr5_ros1_ws}"

echo "=========================================="
echo "  启动 CR5 Driver"
echo "=========================================="

# 检查是否已在运行
if pgrep -f "dobot_bringup" &>/dev/null; then
    echo "[WARN] CR5 Driver 已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 检查网络
if ! ping -c 1 -W 1 192.168.110.214 &>/dev/null; then
    echo "[WARN] CR5 控制柜不可达 (192.168.110.214)"
    echo "请检查网络连接"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 加载环境
source "$SCRIPTS_DEV_DIR/env.sh" 2>/dev/null || true

echo "启动 dobot_bringup..."
echo "robot_ip: 192.168.110.214"
echo

exec roslaunch dobot_bringup bringup.launch robot_ip:=192.168.110.214
