#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_vision.sh - 纯视觉一键启动
# 不启动 MoveIt，不使能机器人
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  纯视觉一键启动"
echo "=========================================="
echo
echo "启动顺序:"
echo "  1. 检查机器人状态"
echo "  2. D455 相机"
echo "  3. TF 检查"
echo "  4. 书本识别"
echo
echo "不启动: MoveIt"
echo "不使能: 机器人"
echo

# 1. 加载环境
load_cr5_environment

# 2. 检查机器人是否使能（fail-closed）
echo "--- 检查机器人状态 ---"
if rosnode list 2>/dev/null | grep -q "/cr5_robot"; then
    ENABLE_STATUS="$(get_robot_status_field EnableStatus)"

    if [ "$ENABLE_STATUS" = "ERROR" ]; then
        echo "[ERROR] 无法读取 EnableStatus"
        echo "  状态未知时不得继续视觉调试"
        echo "  请手动检查机器人状态"
        exit 1
    fi

    if [ "$ENABLE_STATUS" = "1" ]; then
        echo "[ERROR] 机器人已使能"
        echo "  视觉调试时机器人应处于下使能状态"
        echo "  请先执行: disable_robot_safe"
        exit 1
    fi

    echo "[PASS] 机器人未使能"
else
    echo "[INFO] CR5 Driver 未运行（camera-frame-only 模式）"
fi

# 3. 启动 D455 相机
echo
echo "--- 启动 D455 相机 ---"

# 检查是否已有相机节点
CAMERA_RUNNING=false
if rosnode list 2>/dev/null | grep -q "/camera/realsense2_camera"; then
    CAMERA_RUNNING=true
    echo "[INFO] 相机已在运行"
fi

if [ "$CAMERA_RUNNING" = false ]; then
    roslaunch cr5_book_spray_demo d455_camera.launch &
    CAMERA_PID=$!
    echo "$CAMERA_PID" > "$RUN_DIR/camera.pid"

    echo "等待相机话题..."
    wait_for_topic_publisher /camera/color/image_raw 30
    wait_for_topic_publisher /camera/aligned_depth_to_color/image_raw 30

    # 等待实际数据
    echo "等待相机数据..."
    wait_for_topic_data /camera/color/image_raw 10
    wait_for_topic_data /camera/aligned_depth_to_color/image_raw 10
fi

# 4. 检查 TF（使用可靠的单次检查）
echo
echo "--- 检查 TF ---"
TF_AVAILABLE=false
if wait_for_tf_once base_link camera_color_optical_frame 5; then
    TF_AVAILABLE=true
else
    echo "[WARN] robot-to-camera TF unavailable"
    echo "  当前状态: base-frame 目标锁定不可用"
    echo "  允许: 二维和 camera-frame 视觉"
fi

# 5. 启动书本识别
echo
echo "--- 启动书本识别 ---"

# 检查是否已在运行
BOOK_RUNNING=false
if rostopic list 2>/dev/null | grep -q "/book_demo/estimator/debug_image"; then
    BOOK_RUNNING=true
    echo "[INFO] 书本识别已在运行"
fi

if [ "$BOOK_RUNNING" = false ]; then
    roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false &
    BOOK_PID=$!
    echo "$BOOK_PID" > "$RUN_DIR/book_demo.pid"

    echo "等待书本识别节点..."
    wait_for_topic_publisher /book_demo/estimator/debug_image 30
fi

echo
echo "=========================================="
echo "  纯视觉启动完成"
echo "=========================================="
echo
echo "查看输出:"
echo "  rqt_image_view /book_demo/estimator/debug_image"
echo "  rostopic echo /book_demo/estimator/book_pose"
echo "  rostopic echo /book_demo/estimator/plane_rmse"
echo
echo "锁定目标:"
echo "  rosservice call /book_demo/estimator/lock_target '{}'"
echo
if [ "$TF_AVAILABLE" = false ]; then
    echo "[WARN] base-frame 目标锁定不可用"
    echo "  当前只能使用 camera-frame 视觉"
    echo "  如需 base-frame 锁定，请先修复 TF"
fi
echo
echo "当前阶段: allow_execution=false"
echo "禁止: 机械臂运动、喷阀、气源"
echo
