#!/usr/bin/env bash
# ============================================================
# load_book_demo_environment.sh
# 公共 ROS 环境加载函数
# 使用方法: source scripts/laptop/load_book_demo_environment.sh
# ============================================================

WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

fail() { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN] $*" >&2; }
info() { echo "[INFO] $*"; }

# ============================================================
# 加载 ROS 环境（正确叠加方式）
# ============================================================
load_ros_environment() {
    # 1. 加载 ROS 基础环境
    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        fail "ROS Noetic 未安装"
    fi
    source /opt/ros/noetic/setup.bash

    # 2. 加载 RealSense 工作空间
    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        fail "RealSense 工作空间未编译: $REALSENSE_WS
请运行: bash $WS/scripts/laptop/setup_realsense_ros1.sh"
    fi
    source "$REALSENSE_WS/devel/setup.bash"

    # 3. 加载 CR5 工作空间（使用 --extend 叠加）
    if [ ! -f "$WS/devel/setup.bash" ]; then
        fail "CR5 工作空间未编译: $WS
请运行: bash $WS/scripts/laptop/pull_build_book_demo.sh"
    fi
    source "$WS/devel/setup.bash" --extend
}

# ============================================================
# 验证 ROS 包可用性
# ============================================================
verify_ros_packages() {
    local ALL_PASS=true

    echo ""
    echo "--- ROS 包验证 ---"

    # 检查 realsense2_camera
    local CAMERA_PATH
    CAMERA_PATH="$(rospack find realsense2_camera 2>/dev/null || true)"
    if [ -n "$CAMERA_PATH" ]; then
        echo "[PASS] realsense2_camera: $CAMERA_PATH"
    else
        echo "[FAIL] realsense2_camera 未找到"
        ALL_PASS=false
    fi

    # 检查 realsense2_description
    local DESC_PATH
    DESC_PATH="$(rospack find realsense2_description 2>/dev/null || true)"
    if [ -n "$DESC_PATH" ]; then
        echo "[PASS] realsense2_description: $DESC_PATH"
    else
        echo "[FAIL] realsense2_description 未找到"
        ALL_PASS=false
    fi

    # 检查 cr5_book_spray_demo
    local DEMO_PATH
    DEMO_PATH="$(rospack find cr5_book_spray_demo 2>/dev/null || true)"
    if [ -n "$DEMO_PATH" ]; then
        echo "[PASS] cr5_book_spray_demo: $DEMO_PATH"
    else
        echo "[FAIL] cr5_book_spray_demo 未找到"
        ALL_PASS=false
    fi

    # 检查 ROS_PACKAGE_PATH
    echo ""
    echo "--- ROS_PACKAGE_PATH ---"
    echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done

    if [ "$ALL_PASS" = false ]; then
        fail "ROS 包验证失败"
    fi
}
