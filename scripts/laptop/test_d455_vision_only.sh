#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# test_d455_vision_only.sh
# D455 纯视觉测试脚本
# ============================================================

MODE="${1:-help}"
WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"

fail() { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN] $*" >&2; }
info() { echo "[INFO] $*"; }

# ============================================================
# 环境加载函数（使用 --extend 叠加）
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
    echo "[INFO] ROS_PACKAGE_PATH:"
    echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done

    if [ "$ALL_PASS" = false ]; then
        fail "ROS 包验证失败"
    fi
}

# ============================================================
# environment 模式
# ============================================================
do_environment() {
    echo "=========================================="
    echo "  ROS 环境诊断"
    echo "=========================================="

    echo ""
    echo "--- 环境变量 ---"
    echo "WS=$WS"
    echo "REALSENSE_WS=$REALSENSE_WS"
    echo "ROS_DISTRO=${ROS_DISTRO:-<未设置>}"
    echo ""

    echo "--- 环境文件检查 ---"
    [ -f "/opt/ros/noetic/setup.bash" ] && echo "[PASS] /opt/ros/noetic/setup.bash" || echo "[FAIL] /opt/ros/noetic/setup.bash"
    [ -f "$REALSENSE_WS/devel/setup.bash" ] && echo "[PASS] $REALSENSE_WS/devel/setup.bash" || echo "[FAIL] $REALSENSE_WS/devel/setup.bash"
    [ -f "$WS/devel/setup.bash" ] && echo "[PASS] $WS/devel/setup.bash" || echo "[FAIL] $WS/devel/setup.bash"

    echo ""
    echo "--- 加载环境 ---"
    load_ros_environment

    echo ""
    echo "--- 验证 ROS 包 ---"
    verify_ros_packages

    echo ""
    echo "--- rospack profile ---"
    rospack profile

    echo ""
    echo "--- CMAKE_PREFIX_PATH ---"
    echo "$CMAKE_PREFIX_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done

    echo ""
    echo "--- ROS_PACKAGE_PATH ---"
    echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
        [ -n "$p" ] && echo "  $p"
    done
}

# ============================================================
# precheck 模式
# ============================================================
do_precheck() {
    echo "=========================================="
    echo "  D455 Precheck"
    echo "=========================================="

    local PASS_COUNT=0
    local WARN_COUNT=0
    local FAIL_COUNT=0

    # 1. Git 状态
    echo ""
    echo "--- Git ---"
    local BRANCH="$(git -C "$WS" branch --show-current 2>/dev/null || echo 'unknown')"
    local SHA="$(git -C "$WS" rev-parse HEAD 2>/dev/null || echo 'unknown')"
    local STATUS="$(git -C "$WS" status --short 2>/dev/null || echo 'unknown')"

    echo "  分支: $BRANCH"
    echo "  SHA: $SHA"
    echo "  状态: ${STATUS:-clean}"

    if [ "$BRANCH" = "feature/book-vision-spray-demo-v1" ]; then
        echo "[PASS] Git 分支正确"
        ((PASS_COUNT++))
    else
        echo "[WARN] Git 分支不是 feature/book-vision-spray-demo-v1"
        ((WARN_COUNT++))
    fi

    # 2. USB 设备检测
    echo ""
    echo "--- USB ---"
    local D455_FOUND=false
    local D455_USB_SPEED="unknown"

    if lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
        echo "[PASS] D455 USB 设备检测到"
        D455_FOUND=true
        ((PASS_COUNT++))
    else
        echo "[FAIL] D455 USB 设备未检测到"
        ((FAIL_COUNT++))
    fi

    # 3. USB 速度检测
    if [ "$D455_FOUND" = true ]; then
        local D455_BUS_DEV="$(lsusb | grep -i "8086:0b5c\|RealSense Depth Camera 455" | head -1 | awk '{print $2}' | tr -d ':')"
        if [ -n "$D455_BUS_DEV" ]; then
            D455_USB_SPEED="$(lsusb -t 2>/dev/null | grep -B5 "Dev.*$D455_BUS_DEV" | grep -oP '\d+M' | head -1 || echo 'unknown')"
        fi

        if [ "$D455_USB_SPEED" = "5000M" ] || [ "$D455_USB_SPEED" = "10000M" ]; then
            echo "[PASS] USB 速度: $D455_USB_SPEED"
            ((PASS_COUNT++))
        elif [ "$D455_USB_SPEED" = "480M" ]; then
            echo "[FAIL] USB 速度只有 $D455_USB_SPEED，需要 USB 3.x"
            ((FAIL_COUNT++))
        else
            echo "[WARN] 无法确定 USB 速度: $D455_USB_SPEED"
            ((WARN_COUNT++))
        fi
    fi

    # 4. Librealsense
    echo ""
    echo "--- Librealsense ---"
    if command -v rs-enumerate-devices &>/dev/null; then
        echo "[PASS] rs-enumerate-devices 可用"
        ((PASS_COUNT++))

        if rs-enumerate-devices 2>/dev/null | grep -q "Intel RealSense D455"; then
            echo "[PASS] rs-enumerate-devices 检测到 D455"
            ((PASS_COUNT++))
        else
            echo "[WARN] rs-enumerate-devices 未检测到 D455（可能需要 sudo）"
            ((WARN_COUNT++))
        fi
    else
        echo "[FAIL] rs-enumerate-devices 不可用"
        ((FAIL_COUNT++))
    fi

    # 5. ROS 包
    echo ""
    echo "--- ROS Packages ---"
    load_ros_environment 2>/dev/null || true

    if rospack find realsense2_camera &>/dev/null; then
        echo "[PASS] realsense2_camera: $(rospack find realsense2_camera)"
        ((PASS_COUNT++))
    else
        echo "[FAIL] realsense2_camera 未找到"
        ((FAIL_COUNT++))
    fi

    if rospack find realsense2_description &>/dev/null; then
        echo "[PASS] realsense2_description: $(rospack find realsense2_description)"
        ((PASS_COUNT++))
    else
        echo "[WARN] realsense2_description 未找到"
        ((WARN_COUNT++))
    fi

    if rospack find cr5_book_spray_demo &>/dev/null; then
        echo "[PASS] cr5_book_spray_demo: $(rospack find cr5_book_spray_demo)"
        ((PASS_COUNT++))
    else
        echo "[FAIL] cr5_book_spray_demo 未找到"
        ((FAIL_COUNT++))
    fi

    # 6. 控制柜网络（独立检查，不影响 D455 判定）
    echo ""
    echo "--- CR5 Network ---"
    if ping -c 2 -W 2 192.168.110.214 &>/dev/null; then
        echo "[PASS] CR5 controller reachable (192.168.110.214)"
    else
        echo "[WARN] CR5 controller not reachable (192.168.110.214)"
        echo "       这不影响 D455 视觉测试"
        ((WARN_COUNT++))
    fi

    # 7. 总结
    echo ""
    echo "=========================================="
    echo "  Precheck 总结"
    echo "=========================================="
    echo "PASS: $PASS_COUNT"
    echo "WARN: $WARN_COUNT"
    echo "FAIL: $FAIL_COUNT"
    echo ""

    echo "D455_USB=$([ "$D455_FOUND" = true ] && echo PASS || echo FAIL)"
    echo "D455_USB_SPEED=$D455_USB_SPEED"
    echo "LIBREALSENSE=$(command -v rs-enumerate-devices &>/dev/null && echo PASS || echo FAIL)"
    echo "REALSENSE_ROS_PACKAGE=$(rospack find realsense2_camera &>/dev/null && echo PASS || echo FAIL)"
    echo "BOOK_DEMO_PACKAGE=$(rospack find cr5_book_spray_demo &>/dev/null && echo PASS || echo FAIL)"
    echo "CR5_NETWORK=$(ping -c 1 -W 1 192.168.110.214 &>/dev/null && echo PASS || echo WARN)"

    if [ "$FAIL_COUNT" -gt 0 ]; then
        echo ""
        echo "[FAIL] 存在失败项，请检查后重试"
        return 1
    fi
}

# ============================================================
# camera 模式
# ============================================================
do_camera() {
    echo "=========================================="
    echo "  启动 D455 相机"
    echo "=========================================="

    load_ros_environment

    # 检查 realsense2_camera 包
    rospack find realsense2_camera &>/dev/null || fail "realsense2_camera 未找到，请先运行: bash $WS/scripts/laptop/setup_realsense_ros1.sh"

    # 检查是否已有进程占用相机
    if pgrep -af 'realsense2_camera_node|rs_camera.launch|realsense-viewer' | grep -v "$0" | grep -v "grep"; then
        fail "已有 RealSense 进程运行，请先关闭"
    fi

    info "启动 rs_camera.launch (align_depth:=true)"
    exec roslaunch realsense2_camera rs_camera.launch align_depth:=true
}

# ============================================================
# topics 模式
# ============================================================
do_topics() {
    echo "=========================================="
    echo "  D455 话题检查"
    echo "=========================================="

    load_ros_environment

    echo ""
    echo "相机话题："
    rostopic list | grep '^/camera/' | sort || true

    echo ""
    echo "--- 话题存在性检查 ---"
    local TOPICS_OK=true

    for TOPIC in \
        /camera/color/image_raw \
        /camera/aligned_depth_to_color/image_raw \
        /camera/color/camera_info \
        /camera/aligned_depth_to_color/camera_info; do
        if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
            echo "[PASS] $TOPIC"
        else
            echo "[FAIL] $TOPIC 不存在"
            TOPICS_OK=false
        fi
    done

    if [ "$TOPICS_OK" = false ]; then
        fail "必要话题不存在，请检查相机是否启动"
    fi

    echo ""
    echo "--- CameraInfo 检查 ---"
    echo "Color CameraInfo:"
    timeout 3 rostopic echo -n 1 /camera/color/camera_info 2>/dev/null | grep -E "width|height|frame_id" || echo "[WARN] 无法读取"
    echo ""
    echo "Aligned Depth CameraInfo:"
    timeout 3 rostopic echo -n 1 /camera/aligned_depth_to_color/camera_info 2>/dev/null | grep -E "width|height|frame_id" || echo "[WARN] 无法读取"

    echo ""
    echo "--- Image Header 检查 ---"
    echo "Color Image Header:"
    timeout 3 rostopic echo -n 1 /camera/color/image_raw/header 2>/dev/null | grep -E "frame_id|stamp" || echo "[WARN] 无法读取"
    echo ""
    echo "Aligned Depth Image Header:"
    timeout 3 rostopic echo -n 1 /camera/aligned_depth_to_color/image_raw/header 2>/dev/null | grep -E "frame_id|stamp" || echo "[WARN] 无法读取"

    echo ""
    echo "请在其他终端执行频率检查："
    echo "  rostopic hz /camera/color/image_raw"
    echo "  rostopic hz /camera/aligned_depth_to_color/image_raw"
}

# ============================================================
# vision 模式
# ============================================================
do_vision() {
    echo "=========================================="
    echo "  启动书本视觉识别"
    echo "=========================================="

    load_ros_environment

    # 验证 ROS 包
    rospack find cr5_book_spray_demo &>/dev/null || fail "cr5_book_spray_demo 未找到"

    # 验证相机话题存在
    echo ""
    echo "--- 相机话题验证 ---"
    local TOPICS_OK=true

    for TOPIC in \
        /camera/color/image_raw \
        /camera/aligned_depth_to_color/image_raw \
        /camera/color/camera_info; do
        if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
            echo "[PASS] $TOPIC"
        else
            echo "[FAIL] $TOPIC 不存在"
            TOPICS_OK=false
        fi
    done

    if [ "$TOPICS_OK" = false ]; then
        fail "相机话题不存在，请先启动相机:
  bash $WS/scripts/laptop/test_d455_vision_only.sh camera"
    fi

    # 检查 robot-to-camera TF
    echo ""
    echo "--- TF 检查 ---"
    if timeout 3 rosrun tf tf_echo base_link camera_color_optical_frame &>/dev/null; then
        echo "[PASS] base_link -> camera_color_optical_frame TF 存在"
    else
        echo "[WARN] robot-to-camera TF unavailable"
        echo "       允许查看彩色/深度/调试图"
        echo "       禁止锁定为机器人基座目标"
        echo "       禁止 MoveIt plan-only"
    fi

    info "启动 vision_only.launch (start_camera:=false)"
    exec roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false
}

# ============================================================
# 主入口
# ============================================================
case "$MODE" in
    environment)
        do_environment
        ;;
    precheck)
        do_precheck
        ;;
    camera)
        do_camera
        ;;
    topics)
        do_topics
        ;;
    vision)
        do_vision
        ;;
    *)
        echo "Usage: $0 {environment|precheck|camera|topics|vision}"
        echo ""
        echo "Commands:"
        echo "  environment - 显示 ROS 环境诊断信息"
        echo "  precheck    - 检查 D455 USB、驱动和 ROS 包状态"
        echo "  camera      - 启动 RealSense 相机"
        echo "  topics      - 显示相机话题"
        echo "  vision      - 启动书本视觉识别（不启动相机）"
        ;;
esac
