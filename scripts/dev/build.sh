#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# build.sh - 编译 CR5 工作空间
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/build_${STAMP}.log"

echo "=========================================="
echo "  编译 CR5 工作空间"
echo "=========================================="

# 加载构建环境（不要求 CR5 devel 已存在）
load_cr5_build_environment

# 显示 Git 信息
echo
echo "--- Git 信息 ---"
echo "分支: $(git -C "$CR5_WS" branch --show-current 2>/dev/null || echo 'unknown')"
echo "SHA: $(git -C "$CR5_WS" rev-parse HEAD 2>/dev/null || echo 'unknown')"

# 编译
echo
echo "开始编译..."
echo "日志文件: $LOG_FILE"
echo

cd "$CR5_WS"

if catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 2>&1 | tee "$LOG_FILE"; then
    echo
    echo "[SUCCESS] 编译成功"
else
    echo
    echo "[FAIL] 编译失败"
    echo "日志: $LOG_FILE"
    exit 1
fi

# 验证包（此时需要加载完整运行时环境）
echo
echo "--- 验证 ROS 包 ---"
source "$CR5_WS/devel/setup.bash" --extend

for PKG in cr5_book_spray_demo dobot_bringup dobot_moveit; do
    if rospack find "$PKG" &>/dev/null; then
        echo "[PASS] $PKG"
    else
        echo "[WARN] $PKG 未找到"
    fi
done

echo
echo "编译完成"
echo "日志: $LOG_FILE"
