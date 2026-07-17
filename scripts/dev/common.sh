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
# 环境加载 - 构建环境（不要求 CR5 devel 已存在）
# ============================================================
load_cr5_build_environment() {
    # 1. 加载 ROS 基础环境
    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        echo "[ERROR] ROS Noetic 未安装" >&2
        return 1
    fi
    source /opt/ros/noetic/setup.bash

    # 2. 加载 RealSense 工作空间（如果存在）
    if [ -f "$REALSENSE_WS/devel/setup.bash" ]; then
        source "$REALSENSE_WS/devel/setup.bash"
    fi

    # 3. 网络配置
    unset ROS_IP 2>/dev/null || true
    unset ROS_HOSTNAME 2>/dev/null || true
    export ROS_MASTER_URI=http://127.0.0.1:11311

    # 4. 创建目录
    mkdir -p "$LOG_DIR" "$RUN_DIR"

    return 0
}

# ============================================================
# 环境加载 - 运行时环境（要求 CR5 devel 已存在）
# ============================================================
load_cr5_environment() {
    # 1. 加载构建环境
    load_cr5_build_environment

    # 2. 加载 CR5 工作空间（使用 --extend 叠加）
    if [ ! -f "$CR5_WS/devel/setup.bash" ]; then
        echo "[ERROR] CR5 工作空间未编译: $CR5_WS" >&2
        echo "  请运行: bash $CR5_WS/scripts/dev/build.sh" >&2
        return 1
    fi
    source "$CR5_WS/devel/setup.bash" --extend

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
# 等待话题 publisher（真正检查有发布者）
# ============================================================
wait_for_topic_publisher() {
    local topic="$1"
    local timeout="${2:-30}"
    local interval="${3:-1}"
    local elapsed=0

    echo -n "等待话题 $topic "
    while [ "$elapsed" -lt "$timeout" ]; do
        # 检查话题是否存在且有非 None 的 publisher
        local info
        info="$(rostopic info "$topic" 2>/dev/null || echo "")"
        if echo "$info" | grep -q "^Publishers:" && \
           ! echo "$info" | grep -q "Publishers: None"; then
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
# 等待话题有实际数据
# ============================================================
wait_for_topic_data() {
    local topic="$1"
    local timeout="${2:-10}"

    echo -n "等待话题数据 $topic "
    if timeout "$timeout" rostopic echo -n 1 "$topic" &>/dev/null; then
        echo " ✓"
        return 0
    else
        echo " ✗ (超时 ${timeout}s)"
        return 1
    fi
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
# 等待 TF（单次检查，收到第一条有效 TF 后退出）
# ============================================================
wait_for_tf_once() {
    local parent="$1"
    local child="$2"
    local timeout_sec="${3:-5}"

    echo -n "检查 TF $parent -> $child "
    if timeout "$timeout_sec" bash -c \
        "rosrun tf tf_echo '$parent' '$child' 2>&1 | grep -m1 -q 'Translation:'"; then
        echo " ✓"
        return 0
    else
        echo " ✗ (超时 ${timeout_sec}s)"
        return 1
    fi
}

# ============================================================
# 机器人只读状态（fail-closed：读取失败则返回错误）
# ============================================================
robot_readonly_status() {
    echo "--- 机器人状态 ---"
    local READ_OK=true

    # RobotMode
    local robot_mode
    robot_mode="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'RobotMode()'" 2>/dev/null || echo "")"
    if [ -z "$robot_mode" ]; then
        echo "[ERROR] 无法读取 RobotMode"
        READ_OK=false
    else
        echo "RobotMode: $robot_mode"
    fi

    # GetErrorID
    local error_id
    error_id="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetErrorID()'" 2>/dev/null || echo "")"
    if [ -z "$error_id" ]; then
        echo "[ERROR] 无法读取 GetErrorID"
        READ_OK=false
    else
        echo "GetErrorID: $error_id"
    fi

    # GetAngle
    echo "GetAngle:"
    local angle
    angle="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetAngle()'" 2>/dev/null || echo "")"
    if [ -z "$angle" ]; then
        echo "[ERROR] 无法读取 GetAngle"
        READ_OK=false
    else
        echo "  $angle"
    fi

    # GetPose
    echo "GetPose:"
    local pose
    pose="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetPose()'" 2>/dev/null || echo "")"
    if [ -z "$pose" ]; then
        echo "[ERROR] 无法读取 GetPose"
        READ_OK=false
    else
        echo "  $pose"
    fi

    # RobotStatus
    echo "RobotStatus:"
    local robot_status
    robot_status="$(rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "")"
    if [ -z "$robot_status" ]; then
        echo "[ERROR] 无法读取 RobotStatus"
        READ_OK=false
    else
        echo "$robot_status" | head -10
    fi

    # FeedInfo
    echo "FeedInfo:"
    local feed_info
    feed_info="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
    if [ -z "$feed_info" ]; then
        echo "[ERROR] 无法读取 FeedInfo"
        READ_OK=false
    else
        echo "$feed_info" | head -10
    fi

    # /joint_states
    echo "joint_states (最新帧):"
    local joint_states
    joint_states="$(rostopic echo -n 1 /joint_states 2>/dev/null || echo "")"
    if [ -z "$joint_states" ]; then
        echo "[ERROR] 无法读取 /joint_states"
        READ_OK=false
    else
        echo "$joint_states" | head -20
    fi

    if [ "$READ_OK" = false ]; then
        return 1
    fi
    return 0
}

# ============================================================
# 获取机器人状态字段（fail-closed）
# ============================================================
get_robot_status_field() {
    local field="$1"

    case "$field" in
        RobotMode)
            local result
            result="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'RobotMode()'" 2>/dev/null || echo "")"
            if [ -z "$result" ]; then
                echo "ERROR"
                return 1
            fi
            # 提取数字
            echo "$result" | grep -oP '\d+' | head -1
            ;;
        ErrorID)
            local result
            result="$(rosservice call /dobot_bringup/srv/TcpDashboard "command: 'GetErrorID()'" 2>/dev/null || echo "")"
            if [ -z "$result" ]; then
                echo "ERROR"
                return 1
            fi
            echo "$result" | grep -oP '\d+' | head -1
            ;;
        EnableStatus)
            local feed_info
            feed_info="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
            if [ -z "$feed_info" ]; then
                echo "ERROR"
                return 1
            fi
            echo "$feed_info" | grep -oP "EnableStatus: \K\d+" || echo "ERROR"
            ;;
        is_enable)
            local robot_status
            robot_status="$(rostopic echo -n 1 /dobot_bringup/msg/RobotStatus 2>/dev/null || echo "")"
            if [ -z "$robot_status" ]; then
                echo "ERROR"
                return 1
            fi
            echo "$robot_status" | grep -oP "is_enable: \K\w+" || echo "ERROR"
            ;;
        ErrorStatus)
            local feed_info
            feed_info="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
            if [ -z "$feed_info" ]; then
                echo "ERROR"
                return 1
            fi
            echo "$feed_info" | grep -oP "ErrorStatus: \K\d+" || echo "ERROR"
            ;;
        RunQueuedCmd)
            local feed_info
            feed_info="$(rostopic echo -n 1 /dobot_bringup/msg/FeedInfo 2>/dev/null || echo "")"
            if [ -z "$feed_info" ]; then
                echo "ERROR"
                return 1
            fi
            echo "$feed_info" | grep -oP "RunQueuedCmd: \K\d+" || echo "ERROR"
            ;;
        *)
            echo "[ERROR] 未知字段: $field" >&2
            return 1
            ;;
    esac
}

# ============================================================
# 检查机器人是否使能（fail-closed）
# ============================================================
is_robot_enabled() {
    local enable_status
    enable_status="$(get_robot_status_field EnableStatus)"

    if [ "$enable_status" = "ERROR" ]; then
        echo "[ERROR] 无法读取 EnableStatus" >&2
        return 2  # 状态未知，返回错误
    fi

    if [ "$enable_status" = "1" ]; then
        return 0  # 已使能
    else
        return 1  # 未使能
    fi
}

# ============================================================
# 验证机器人使能状态（四项全部检查）
# ============================================================
verify_robot_enabled() {
    echo "验证机器人使能状态..."

    local ROBOT_MODE="$(get_robot_status_field RobotMode)"
    local IS_ENABLE="$(get_robot_status_field is_enable)"
    local ENABLE_STATUS="$(get_robot_status_field EnableStatus)"
    local ERROR_STATUS="$(get_robot_status_field ErrorStatus)"

    echo "  RobotMode: $ROBOT_MODE"
    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"

    # 检查是否读取失败
    if [ "$ROBOT_MODE" = "ERROR" ] || [ "$IS_ENABLE" = "ERROR" ] || \
       [ "$ENABLE_STATUS" = "ERROR" ] || [ "$ERROR_STATUS" = "ERROR" ]; then
        echo "[ERROR] 无法读取机器人状态"
        return 1
    fi

    # 四项全部验证
    if [ "$ROBOT_MODE" = "5" ] && [ "$IS_ENABLE" = "True" ] && \
       [ "$ENABLE_STATUS" = "1" ] && [ "$ERROR_STATUS" = "0" ]; then
        echo "[PASS] 机器人已使能，状态正常"
        return 0
    else
        echo "[FAIL] 机器人状态不符合预期"
        return 1
    fi
}

# ============================================================
# 验证机器人下使能状态
# ============================================================
verify_robot_disabled() {
    echo "验证机器人下使能状态..."

    local IS_ENABLE="$(get_robot_status_field is_enable)"
    local ENABLE_STATUS="$(get_robot_status_field EnableStatus)"
    local ERROR_STATUS="$(get_robot_status_field ErrorStatus)"
    local RUN_QUEUED="$(get_robot_status_field RunQueuedCmd)"

    echo "  is_enable: $IS_ENABLE"
    echo "  EnableStatus: $ENABLE_STATUS"
    echo "  ErrorStatus: $ERROR_STATUS"
    echo "  RunQueuedCmd: $RUN_QUEUED"

    # 检查是否读取失败
    if [ "$IS_ENABLE" = "ERROR" ] || [ "$ENABLE_STATUS" = "ERROR" ] || \
       [ "$ERROR_STATUS" = "ERROR" ] || [ "$RUN_QUEUED" = "ERROR" ]; then
        echo "[ERROR] 无法读取机器人状态"
        return 1
    fi

    # 验证下使能状态
    if [ "$IS_ENABLE" = "False" ] && [ "$ENABLE_STATUS" = "0" ] && \
       [ "$ERROR_STATUS" = "0" ] && [ "$RUN_QUEUED" = "0" ]; then
        echo "[PASS] 机器人已下使能，状态正常"
        return 0
    else
        echo "[FAIL] 机器人状态不符合预期"
        return 1
    fi
}

# ============================================================
# 安全计数器（避免 set -e 问题）
# ============================================================
safe_increment() {
    local var_name="$1"
    eval "$var_name=\$(( $var_name + 1 ))"
}
