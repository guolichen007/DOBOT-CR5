#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# doctor.sh - CR5 系统全面诊断
# 用法: doctor [--offline|--runtime|--vision]
# ============================================================

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
source "$SCRIPT_DIR/common.sh"

MODE="${1:---offline}"

PASS_COUNT=0
WARN_COUNT=0
FAIL_COUNT=0

pass() { echo "[PASS] $*"; PASS_COUNT=$((PASS_COUNT + 1)); }
warn() { echo "[WARN] $*"; WARN_COUNT=$((WARN_COUNT + 1)); }
fail() { echo "[FAIL] $*"; FAIL_COUNT=$((FAIL_COUNT + 1)); }

echo "=========================================="
echo "  CR5 系统诊断 ($MODE)"
echo "=========================================="
echo "时间: $(date)"
echo

# ============================================================
# 离线检查（不要求硬件在线）
# ============================================================
do_offline() {
    # 1. Git 状态
    echo "--- Git 状态 ---"
    if [ -d "$CR5_WS/.git" ]; then
        BRANCH="$(git -C "$CR5_WS" branch --show-current 2>/dev/null || echo 'unknown')"
        SHA="$(git -C "$CR5_WS" rev-parse --short HEAD 2>/dev/null || echo 'unknown')"
        STATUS="$(git -C "$CR5_WS" status --short 2>/dev/null || echo 'unknown')"
        pass "Git 仓库: $CR5_WS"
        echo "  分支: $BRANCH"
        echo "  SHA: $SHA"
        echo "  状态: ${STATUS:-clean}"
    else
        fail "Git 仓库不存在: $CR5_WS"
    fi

    # 2. ROS 环境
    echo
    echo "--- ROS 环境 ---"
    if [ -n "${ROS_DISTRO:-}" ]; then
        pass "ROS_DISTRO: $ROS_DISTRO"
    else
        fail "ROS_DISTRO 未设置"
    fi

    # 3. ROS 包
    echo
    echo "--- ROS 包 ---"
    for PKG in cr5_book_spray_demo dobot_bringup dobot_moveit realsense2_camera realsense2_description; do
        if rospack find "$PKG" &>/dev/null; then
            pass "$PKG: $(rospack find "$PKG")"
        else
            fail "$PKG 未找到"
        fi
    done

    # 4. 编译检查
    echo
    echo "--- 编译检查 ---"
    if [ -f "$CR5_WS/devel/setup.bash" ]; then
        pass "CR5 工作空间已编译"
    else
        fail "CR5 工作空间未编译"
    fi

    if [ -f "$REALSENSE_WS/devel/setup.bash" ]; then
        pass "RealSense 工作空间已编译"
    else
        warn "RealSense 工作空间未编译"
    fi

    # 5. 磁盘空间
    echo
    echo "--- 磁盘空间 ---"
    DISK_USAGE="$(df -h "$CR5_WS" | tail -1 | awk '{print $5}' | tr -d '%')"
    if [ "$DISK_USAGE" -lt 80 ]; then
        pass "磁盘使用: ${DISK_USAGE}%"
    else
        warn "磁盘使用: ${DISK_USAGE}%（建议清理）"
    fi
}

# ============================================================
# 运行时检查（检查 ROS 节点和服务）
# ============================================================
do_runtime() {
    # 1. ROS Master
    echo
    echo "--- ROS Master ---"
    if rostopic list &>/dev/null; then
        pass "ROS Master 可达"
    else
        fail "ROS Master 不可达"
        return 1
    fi

    # 2. 网络连接
    echo
    echo "--- 网络连接 ---"
    if ping -c 1 -W 1 192.168.110.214 &>/dev/null; then
        pass "CR5 控制柜可达 (192.168.110.214)"
    else
        warn "CR5 控制柜不可达 (192.168.110.214)"
    fi

    # 3. 端口检查
    echo
    echo "--- 端口检查 ---"
    for PORT in 29999 30003 30004; do
        if timeout 2 bash -c "echo >/dev/tcp/192.168.110.214/$PORT" 2>/dev/null; then
            pass "端口 $PORT 可达"
        else
            warn "端口 $PORT 不可达"
        fi
    done

    # 4. ROS 节点
    echo
    echo "--- ROS 节点 ---"
    for NODE in /cr5_robot /move_group; do
        if rosnode list 2>/dev/null | grep -q "^${NODE}$"; then
            pass "节点 $NODE 运行中"
        else
            echo "[INFO] 节点 $NODE 未运行"
        fi
    done

    # 5. 话题检查
    echo
    echo "--- 话题检查 ---"
    for TOPIC in /joint_states /dobot_bringup/msg/RobotStatus /dobot_bringup/msg/FeedInfo; do
        if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
            pass "话题 $TOPIC 存在"
        else
            echo "[INFO] 话题 $TOPIC 不存在"
        fi
    done

    # 6. 服务检查
    echo
    echo "--- 服务检查 ---"
    for SERVICE in /dobot_bringup/srv/EnableRobot /dobot_bringup/srv/DisableRobot; do
        if rosservice list 2>/dev/null | grep -q "^${SERVICE}$"; then
            pass "服务 $SERVICE 存在"
        else
            echo "[INFO] 服务 $SERVICE 不存在"
        fi
    done

    # 7. Action 检查
    echo
    echo "--- Action 检查 ---"
    if rostopic list 2>/dev/null | grep -q "follow_joint_trajectory"; then
        pass "FollowJointTrajectory Action 存在"
    else
        echo "[INFO] FollowJointTrajectory Action 不存在"
    fi

    # 8. 机器人状态（只读）
    echo
    echo "--- 机器人状态 ---"
    robot_readonly_status 2>/dev/null || echo "[INFO] 无法获取机器人状态"
}

# ============================================================
# 视觉检查（D455 相关）
# ============================================================
do_vision() {
    # 1. USB 设备
    echo
    echo "--- USB 设备 ---"
    if lsusb 2>/dev/null | grep -qi "8086:0b5c\|RealSense Depth Camera 455"; then
        pass "D455 USB 设备检测到"
    else
        fail "D455 USB 设备未检测到"
    fi

    # 2. USB 速度
    if lsusb -t 2>/dev/null | grep -q "5000M"; then
        pass "USB 3.x 速度检测"
    else
        warn "USB 速度可能不是 3.x"
    fi

    # 3. Librealsense
    echo
    echo "--- Librealsense ---"
    if command -v rs-enumerate-devices &>/dev/null; then
        pass "rs-enumerate-devices 可用"
    else
        fail "rs-enumerate-devices 不可用"
    fi

    # 4. ROS 包
    echo
    echo "--- RealSense ROS 包 ---"
    for PKG in realsense2_camera realsense2_description; do
        if rospack find "$PKG" &>/dev/null; then
            pass "$PKG: $(rospack find "$PKG")"
        else
            fail "$PKG 未找到"
        fi
    done

    # 5. 运行时库
    echo
    echo "--- 运行时库 ---"
    CAMERA_SO="$REALSENSE_WS/devel/lib/librealsense2_camera.so"
    if [ -f "$CAMERA_SO" ]; then
        if ldd "$CAMERA_SO" 2>/dev/null | grep -q "/usr/local/lib/librealsense2.so"; then
            pass "librealsense2 链接正确"
        else
            warn "librealsense2 链接可能不正确"
        fi
    else
        echo "[INFO] $CAMERA_SO 不存在"
    fi

    # 6. 相机话题
    echo
    echo "--- 相机话题 ---"
    for TOPIC in /camera/color/image_raw /camera/color/camera_info /camera/aligned_depth_to_color/image_raw /camera/aligned_depth_to_color/camera_info; do
        if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
            pass "话题 $TOPIC 存在"
        else
            echo "[INFO] 话题 $TOPIC 不存在"
        fi
    done

    # 7. 话题频率
    echo
    echo "--- 话题频率 ---"
    for TOPIC in /camera/color/image_raw /camera/aligned_depth_to_color/image_raw; do
        if rostopic list 2>/dev/null | grep -q "^${TOPIC}$"; then
            HZ="$(timeout 5 rostopic hz "$TOPIC" 2>/dev/null | grep "average rate" | awk '{print $3}' || echo "N/A")
            echo "  $TOPIC: $HZ Hz"
        fi
    done

    # 8. TF 检查
    echo
    echo "--- TF 检查 ---"
    if timeout 3 rosrun tf tf_echo base_link camera_color_optical_frame &>/dev/null; then
        pass "base_link -> camera_color_optical_frame TF 存在"
    else
        warn "base_link -> camera_color_optical_frame TF 不存在"
    fi

    # 9. 书本识别包
    echo
    echo "--- 书本识别包 ---"
    if rospack find cr5_book_spray_demo &>/dev/null; then
        pass "cr5_book_spray_demo: $(rospack find cr5_book_spray_demo)"
    else
        fail "cr5_book_spray_demo 未找到"
    fi
}

# ============================================================
# 主入口
# ============================================================
case "$MODE" in
    --offline)
        do_offline
        ;;
    --runtime)
        do_offline
        do_runtime
        ;;
    --vision)
        do_offline
        do_vision
        ;;
    *)
        echo "用法: $0 [--offline|--runtime|--vision]"
        echo
        echo "模式:"
        echo "  --offline  离线检查（默认）"
        echo "  --runtime  运行时检查"
        echo "  --vision   视觉检查"
        exit 1
        ;;
esac

# 总结
echo
echo "=========================================="
echo "  诊断总结"
echo "=========================================="
echo "PASS: $PASS_COUNT"
echo "WARN: $WARN_COUNT"
echo "FAIL: $FAIL_COUNT"

# 输出 Git 信息
echo
echo "--- Git 信息 ---"
echo "分支: $(git -C "$CR5_WS" branch --show-current 2>/dev/null || echo 'unknown')"
echo "SHA: $(git -C "$CR5_WS" rev-parse HEAD 2>/dev/null || echo 'unknown')"

if [ "$FAIL_COUNT" -gt 0 ]; then
    echo
    echo "[RESULT] 存在失败项，请检查"
    exit 1
elif [ "$WARN_COUNT" -gt 0 ]; then
    echo
    echo "[RESULT] 存在警告，但可以继续"
    exit 0
else
    echo
    echo "[RESULT] 所有检查通过"
    exit 0
fi
