#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# robot_status.sh - 机器人只读状态查询
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  CR5 机器人状态"
echo "=========================================="

# 加载环境
load_cr5_environment

# 检查 Driver
if ! rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    echo "[WARN] CR5 Driver 未运行"
    echo "  请先启动: start_driver"
    exit 1
fi

# 只读状态
robot_readonly_status

echo
echo "=========================================="
echo "  状态查询完成"
echo "=========================================="
