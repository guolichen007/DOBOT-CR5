#!/usr/bin/env bash
# ===========================================================================
# V4 Calibration Texture Smoke Test.
#
# 在完整场景之前独立验证:
#   - Gazebo 能否加载 DAE mesh + OGRE material
#   - 贴图是否正确渲染 (非纯白/灰)
#   - ChArUco Front 面板 Marker 100~123 能否检出
#
# 通过后输出 CALIBRATION_TEXTURE_SMOKE_PASS.
# 失败则输出 CALIBRATION_TEXTURE_SMOKE_FAIL 并退出 1.
#
# 用法:
#   bash run_calibration_texture_smoke.sh [--gui]
# ===========================================================================
set -euo pipefail

GUI=false
if [[ "${1:-}" == "--gui" ]]; then
    GUI=true
fi

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
PKG_DIR="$SCRIPT_DIR/.."

# Workspace 环境
source /opt/ros/noetic/setup.bash
if [[ -f "$WS_DIR/devel/setup.bash" ]]; then
    source "$WS_DIR/devel/setup.bash"
fi

# Gazebo model path
export GAZEBO_MODEL_PATH="$PKG_DIR/models:${GAZEBO_MODEL_PATH:-}"

# 验证关键文件
SMOKE_WORLD="$PKG_DIR/worlds/calibration_texture_smoke.world"
DAE_MESH="$PKG_DIR/models/calibration_target_v1/meshes/panel_unit.dae"
MAT_SCRIPT="$PKG_DIR/models/calibration_target_v1/materials/scripts/calibration_target_v1.material"
FRONT_TEX="$PKG_DIR/models/calibration_target_v1/materials/textures/charuco_front_v1.png"

for f in "$SMOKE_WORLD" "$DAE_MESH" "$MAT_SCRIPT" "$FRONT_TEX"; do
    if [[ ! -f "$f" ]]; then
        echo "ERROR: Smoke test resource missing: $f"
        exit 12
    fi
done

echo "========================================="
echo "Calibration Texture Smoke Test V4"
echo "========================================="
echo "World:    $SMOKE_WORLD"
echo "DAE:      $DAE_MESH"
echo "Material: $MAT_SCRIPT"
echo "Texture:  $FRONT_TEX"
echo ""

# 随机端口
ROS_PORT=$((11311 + RANDOM % 1000))
GZ_PORT=$((11345 + RANDOM % 1000))
export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
export GAZEBO_MASTER_URI="http://localhost:${GZ_PORT}"

echo "[$(date +%H:%M:%S)] Starting roscore (port $ROS_PORT)..."
roscore -p "$ROS_PORT" &
ROSCORE_PID=$!
sleep 3

if ! kill -0 "$ROSCORE_PID" 2>/dev/null; then
    echo "ERROR: roscore failed"
    exit 1
fi

cleanup() {
    echo "[$(date +%H:%M:%S)] cleanup..."
    kill "$ROSCORE_PID" 2>/dev/null || true
    wait "$ROSCORE_PID" 2>/dev/null || true
    pkill -f "calibration_texture_smoke" 2>/dev/null || true
}
trap cleanup EXIT

echo "[$(date +%H:%M:%S)] Starting Gazebo with smoke world..."

GZ_EXTRA=""
if $GUI; then
    GZ_EXTRA="-g"
fi

# 使用 roslaunch gazebo_ros empty_world 启动 (确保 ROS 话题桥接正确)
roslaunch gazebo_ros empty_world.launch \
    paused:=false \
    gui:=$GUI \
    world_name:="$SMOKE_WORLD" \
    extra_gazebo_args:="-p $GZ_PORT" \
    > /tmp/smoke_gazebo.log 2>&1 &
GZ_PID=$!

# 等待 camera 话题
echo "[$(date +%H:%M:%S)] Waiting for smoke camera topic..."
MAX_WAIT=60
WAITED=0
CAM_TOPIC="/smoke_camera/camera/image_raw"
while [[ $WAITED -lt $MAX_WAIT ]]; do
    if rostopic list 2>/dev/null | grep -q "$CAM_TOPIC"; then
        echo "[$(date +%H:%M:%S)] Camera topic ready (${WAITED}s)"
        break
    fi
    sleep 2
    WAITED=$((WAITED + 2))
done

if [[ $WAITED -ge $MAX_WAIT ]]; then
    echo "ERROR: Camera topic $CAM_TOPIC not found after ${MAX_WAIT}s"
    echo "Gazebo log tail:"
    tail -30 /tmp/smoke_gazebo.log 2>/dev/null || true
    echo "CALIBRATION_TEXTURE_SMOKE_FAIL"
    exit 1
fi

# 等待几帧
sleep 3

# 捕获图像
CAPTURE_DIR="/tmp/smoke_test_output"
mkdir -p "$CAPTURE_DIR"

echo "[$(date +%H:%M:%S)] Capturing image..."
# 用 Python 脚本捕获 + 检测
python3 - "$CAPTURE_DIR" <<'PYEOF'
import sys, os, json
import cv2, numpy as np
import rospy
from sensor_msgs.msg import Image
from cv_bridge import CvBridge
from cv2 import aruco

output_dir = sys.argv[1]
rospy.init_node("smoke_test_capture", anonymous=True, log_level=rospy.WARN)

# 等待图像
try:
    msg = rospy.wait_for_message("/smoke_camera/camera/image_raw", Image, timeout=15.0)
except:
    print("ERROR: Image capture timeout", file=sys.stderr)
    sys.exit(1)

bridge = CvBridge()
bgr = bridge.imgmsg_to_cv2(msg, "bgr8")
gray = cv2.cvtColor(bgr, cv2.COLOR_BGR2GRAY)

# 保存原图
cv2.imwrite(os.path.join(output_dir, "smoke_raw.png"), bgr)
print(f"Image saved: {bgr.shape}")

# 检查黑色像素比例 (贴图应该有黑/白区域)
total_px = gray.size
black_px = int((gray < 50).sum())
white_px = int((gray > 200).sum())
black_pct = black_px / total_px * 100
white_pct = white_px / total_px * 100
print(f"Black pixels: {black_pct:.1f}%  White pixels: {white_pct:.1f}%")

if black_pct < 5 or white_pct < 5:
    print(f"FAIL: Texture appears blank. Black={black_pct:.1f}% White={white_pct:.1f}%")
    sys.exit(1)

# ChArUco detection
dict_obj = aruco.getPredefinedDictionary(aruco.DICT_5X5_1000)
params = aruco.DetectorParameters_create()
params.cornerRefinementMethod = aruco.CORNER_REFINE_SUBPIX

corners, ids, rejected = aruco.detectMarkers(gray, dict_obj, parameters=params)

if ids is None:
    print("FAIL: No ArUco markers detected")
    sys.exit(1)

ids_flat = [int(i) for i in ids.flatten()]
front_ids = [i for i in ids_flat if 100 <= i <= 123]
print(f"Detected markers: {ids_flat}")
print(f"Front ChArUco markers (100-123): {front_ids} ({len(front_ids)}/24)")

if len(front_ids) < 12:
    print(f"FAIL: Only {len(front_ids)} front markers detected (need >= 12)")
    sys.exit(1)

# 绘制 annotated 图
annotated = aruco.drawDetectedMarkers(bgr.copy(), corners, ids)
cv2.imwrite(os.path.join(output_dir, "smoke_annotated.png"), annotated)

result = {
    "pass": True,
    "black_pct": round(black_pct, 1),
    "white_pct": round(white_pct, 1),
    "total_markers": len(ids_flat),
    "front_markers": front_ids,
    "front_marker_count": len(front_ids),
    "image_shape": list(bgr.shape),
}
with open(os.path.join(output_dir, "smoke_result.json"), "w") as f:
    json.dump(result, f, indent=2)

print("CALIBRATION_TEXTURE_SMOKE_PASS")
PYEOF

SMOKE_RET=$?

kill "$GZ_PID" 2>/dev/null || true

if [[ $SMOKE_RET -ne 0 ]]; then
    echo ""
    echo "CALIBRATION_TEXTURE_SMOKE_FAIL"
    echo "Fix texture/mesh/material before running full CR5 scene."
    exit 1
fi

echo ""
echo "CALIBRATION_TEXTURE_SMOKE_PASS"
echo "Smoke test artifacts: $CAPTURE_DIR"
exit 0
