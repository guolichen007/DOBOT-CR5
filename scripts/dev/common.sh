#!/usr/bin/env bash
# ============================================================
# common.sh - CR5 开发工具链公共函数
# ============================================================

# 全局变量
CR5_WS="${CR5_WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"
LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
RUN_DIR="${RUN_DIR:-$HOME/cr5_test_logs/run}"

# ============================================================
# 环境加载
# ============================================================
load_cr5_environment() {
    # 1. 加载 ROS 基础环境
    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        echo "[ERROR] ROS Noetic 未安装" >&2
        return 1
    fi
    source /opt/ros/noetic/setup.bash

    # 2. 加载 RealSense 工作空间
    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        echo "[ERROR] RealSense 工作空间未编译: $REALSENSE_WS" >&2
        echo "  请运行: bash $CR5_WS/scripts/laptop/setup_realsense_ros1.sh" >&2
        return 1
    fi
    source "$REALSENSE_WS/devel/setup.bash"

    # 3. 加载 CR5 工作空间（使用 --extend 叠加）
    if [ ! -f "$CR5_WS/devel/setup.bash" ]; then
        echo "[ERROR] CR5 工作空间未编译: $CR5_WS" >&2
        return 1
    fi
    source "$CR5_WS/devel/setup.bash" --extend

    # 4. 网络配置
    unset ROS_IP 2>/dev/null || true
    unset ROS_HOSTNAME 2>/dev/null || true
    export ROS_MASTER_URI=http://127.0.0.1:11311

    # 5. 创建目录
    mkdir -p "$LOG_DIR" "$RUN_DIR"

    return 0
}

# ============================================================
# 文件验证
# ============================================================
verify_required_file() {
    local file="$1"
    local desc="${2:-$file}"
    if [ ! -f "$file" ]; then
        echo "[ERROR] $desc 不存在: $file" >&2
        return 1
    fi
    return 0
}

# ============================================================
# ROS 包验证
# ============================================================
verify_ros_package() {
    local pkg="$1"
    if ! rospack find "$pkg" &>/dev/null; then
        echo "[ERROR] ROS 包未找到: $pkg" >&2
        return 1
    fi
    return 0
}

# ============================================================
# ROS Master 检查
# ============================================================
ensure_ros_master() {
    if ! rostopic list &>/dev/null; then
        echo "[ERROR] ROS Master 不可达" >&2
        echo "  请启动 roscore 或检查 ROS_MASTER_URI" >&2
        return 1
    fi
    return 0
}

# ============================================================
# 等待 ROS 节点
# ============================================================
wait_for_ros_node() {
    local node="$1"
    local timeout="${2:-30}"
    local interval="${3:-1}"
    local elapsed=0

    echo -n "等待节点 $node "
    while [ "$elapsed" -lt "$timeout" ]; do
        if rosnode list 2>/dev/null | grep -q "^${node}$"; then
            echo " ✓"
            return 0
        fi
        echo -n "."
        sleep "$interval"
        elapsed=$((elapsed + interval))
    done
    echo " ✗ (超时 ${timeout}s)"
    return 1
}

# ============================================================
# 等待话题 publisher
# ============================================================
wait_for_topic_publisher() {
    local topic="$1"
    local timeout="${2:-30}"
    local interval="${3:-1}"
    local elapsed=0

    echo -n "等待话题 $topic "
    while [ "$elapsed" -lt "$timeout" ]; do
        local pub_count
        pub_count="$(rostopic info "$topic" 2>/dev/null | grep -c "^Publishers:" || echo 0)"
        if [ "$pub_count" -gt 0 ]; then
            echo " ✓"
            return 0
        fi
        echo -n "."
        sleep "$interval"
        elapsed=$((elapsed + interval))
    done
    echo " ✗ (超时 ${timeout}s)"
    return 1
}

# ============================================================
# 等待服务
# ============================================================
wait_for_service() {
    local service="$1"
    local timeout="${2:-30}"
    local interval="${3:-1}"
    local elapsed=0

    echo -n "等待服务 $service "
    while [ "$elapsed" -lt "$timeout" ]; do
        if rosservice list 2>/dev/null | grep -q "^${service}$"; then
            echo " ✓"
            return 0
        fi
        echo -n "."
        sleep "$interval"
        elapsed=$((elapsed + interval))
    done
    echo " ✗ (超时 ${timeout}s)"
    return 1
}

# ============================================================
# 机器人只读状态
# ============================================================
robot_readonly_status() {
    echo "--- 机器人状态 ---"

    # RobotMode
    local robot_mode
    robot_mode="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'RobotMode()'" 2>/dev/null || echo "服务调用失败")"
    echo "RobotMode: $robot_mode"

    # GetErrorID
    local error_id
    error_id="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetErrorID()'" 2>/dev/null || echo "服务调用失败")"
    echo "GetErrorID: $error_id"

    # GetAngle
    echo "GetAngle:"
    rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "  服务调用失败"

    # GetPose
    echo "GetPose:"
    rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetPose()'" 2>/dev/null || echo "  服务调用失败"

    # RobotStatus
    echo "RobotStatus:"
    rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "  话题不可用"

    # FeedInfo
    echo "FeedInfo:"
    rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "  话题不可用"

    # /joint_states
    echo "joint_states (最新帧):"
    rostopic echo -n 1 /joint_states 2>/dev/null | head -20 || echo "  话题不可用"
}

# ============================================================
# 检查机器人是否使能
# ============================================================
is_robot_enabled() {
    local feed_info
    feed_info="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
    if echo "$feed_info" | grep -q "EnableStatus: 1"; then
        return 0  # 已使能
    else
        return 1  # 未使能
    fi
}

# ============================================================
# 安全计数器（避免 set -e 问题）
# ============================================================
safe_increment() {
    local var_name="$1"
    eval "$var_name=\$(( $var_name + 1 ))"
}
