#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# start_camera.sh - 启动 D455 相机
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

echo "=========================================="
echo "  启动 D455 相机"
echo "=========================================="

# 1. 加载环境
load_cr5_environment

# 2. 验证包
verify_ros_package realsense2_camera
verify_ros_package cr5_book_spray_demo

# 3. 检查 USB 设备
echo "检查 D455 USB 设备..."
if lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
    echo "[PASS] D455 USB 设备检测到"
else
    echo "[WARN] D455 USB 设备未检测到"
    echo "请检查 USB 连接"
    read -p "是否继续？(y/N) " -n 1 -r
    echo
    if [[ ! $REPLY =~ ^[Yy]$ ]]; then
        exit 1
    fi
fi

# 4. 检查是否已有相机节点运行（使用 rosnode 而不是 pgrep）
echo "检查相机节点..."
CAMERA_RUNNING=false

if rosnode list 2>/dev/null | grep -q "/camera/realsense2_camera"; then
    CAMERA_RUNNING=true
fi

if [ "$CAMERA_RUNNING" = true ]; then
    echo "[WARN] RealSense 相机节点已在运行"
    echo "如需重启，请先执行: stop_all"
    exit 1
fi

# 5. 启动相机
echo
echo "启动 D455 相机..."
echo "参数: align_depth:=true, enable_sync:=true"
echo

roslaunch cr5_book_spray_demo d455_camera.launch &
CAMERA_PID=$!
echo "$CAMERA_PID" > "$RUN_DIR/camera.pid"

# 6. 等待话题 publisher 就绪
echo "等待相机话题 publisher..."
for TOPIC in \
    /camera/color/image_raw \
    /camera/color/camera_info \
    /camera/aligned_depth_to_color/image_raw \
    /camera/aligned_depth_to_color/camera_info; do
    wait_for_topic_publisher "$TOPIC" 30
done

# 7. 等待实际数据
echo
echo "等待相机数据..."
wait_for_topic_data /camera/color/image_raw 10
wait_for_topic_data /camera/aligned_depth_to_color/image_raw 10

# 8. 运行话题检查
echo
echo "运行话题一致性检查..."
if [ -f "$CR5_WS/scripts/laptop/check_d455_topics.py" ]; then
    timeout 30 python3 "$CR5_WS/scripts/laptop/check_d455_topics.py" 2>/dev/null || echo "[WARN] 话题检查超时或失败"
else
    echo "[INFO] check_d455_topics.py 不存在"
fi

# 9. 输出频率
echo
echo "采样频率..."
for TOPIC in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw; do
    HZ="$(timeout 5 rostopic hz "$TOPIC" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A")"
    echo "  $TOPIC: $HZ Hz"
done

echo
echo "=========================================="
echo "  D455 相机启动完成"
echo "=========================================="
echo
echo "下一步:"
echo "  start_book_demo   - 启动书本识别"
echo "  rqt_image_view    - 查看调试图像"
echo
