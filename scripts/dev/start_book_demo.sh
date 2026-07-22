#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_book_demo.sh - 启动书本识别
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  启动书本识别"
echo "=========================================="

# 1. 加载环境
load_cr5_environment

# 2. 验证包
verify_ros_package cr5_book_spray_demo

# 3. 检查相机话题（使用可靠的 publisher 检查）
echo "检查相机话题..."
CAMERA_TOPICS_OK=true

for TOPIC in \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /camera/aligned_depth_to_color/image_raw \
    /camera/aligned_depth_to_color/camera_info; do
    # 检查话题是否存在
    if ! rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
        echo "[FAIL] $TOPIC 不存在"
        CAMERA_TOPICS_OK=false
        continue
    fi

    # 检查是否有真正的 publisher
    local_info="$(rostopic info "$TOPIC" 2>/dev/null || echo "")"
    if echo "$local_info" | grep -q "^Publishers:" && \
       ! echo "$local_info" | grep -q "Publishers: None"; then
        echo "[PASS] $TOPIC (有 publisher)"
    else
        echo "[WARN] $TOPIC (无 publisher)"
        CAMERA_TOPICS_OK=false
    fi
done

if [ "$CAMERA_TOPICS_OK" = false ]; then
    echo
    echo "[ERROR] 相机话题不完整或无 publisher"
    echo "  请先启动: start_camera"
    exit 1
fi

# 4. 等待至少一帧数据（彩色和深度可以检查实际消息）
echo
echo "等待相机数据..."
if ! wait_for_topic_data /camera/color/image_raw 5; then
    echo "[ERROR] 未收到 color image"
    exit 1
fi

if ! wait_for_topic_data /camera/aligned_depth_to_color/image_raw 5; then
    echo "[ERROR] 未收到 aligned depth image"
    exit 1
fi

# 5. 启动书本识别
echo
echo "启动书本识别..."
echo "参数: start_camera:=false"
echo

roslaunch cr5_book_spray_demo vision_only.launch start_camera:=false &
BOOK_PID=$!
echo "$BOOK_PID" > "$RUN_DIR/book_demo.pid"

# 6. 等待节点和话题 publisher
echo "等待书本识别节点..."
wait_for_topic_publisher /book_demo/estimator/debug_image 30
wait_for_topic_publisher /book_demo/estimator/valid 10
# book_pose 和 plane_rmse 可能需要检测成功后才有数据，只检查 publisher
wait_for_topic_publisher /book_demo/estimator/book_pose 10
wait_for_topic_publisher /book_demo/estimator/plane_rmse 10

# 等待服务
wait_for_service /book_demo/estimator/lock_target 10
wait_for_service /book_demo/estimator/clear_target 10

echo
echo "=========================================="
echo "  书本识别启动完成"
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
echo "当前阶段: allow_execution=false"
echo "禁止: 机械臂运动、喷阀、气源"
echo
