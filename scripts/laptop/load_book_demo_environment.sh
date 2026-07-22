#!/usr/bin/env bash
# ============================================================
# load_book_demo_environment.sh - 公共 ROS 环境加载函数
# 复用 scripts/dev/common.sh
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$(cd "$SCRIPT_DIR/../dev" && pwd)"

# 复用 common.sh
source "$DEV_DIR/common.sh"

# 加载环境
load_cr5_environment

# 验证 ROS 包
verify_ros_packages() {
    local ALL_PASS=true

    echo ""
    echo "--- ROS 包验证 ---"

    for PKG in realsense2_camera realsense2_description cr5_book_spray_demo; do
        local PKG_PATH
        PKG_PATH="$(rospack find "$PKG" 2>/dev/null || true)"
        if [ -n "$PKG_PATH" ]; then
            echo "[PASS] $PKG: $PKG_PATH"
        else
            echo "[FAIL] $PKG 未找到"
            ALL_PASS=false
        fi
    done

    echo ""
    echo "[INFO] ROS_PACKAGE_PATH:"
    echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done

    if [ "$ALL_PASS" = false ]; then
        echo "[ERROR] ROS 包验证失败"
        return 1
    fi
}
