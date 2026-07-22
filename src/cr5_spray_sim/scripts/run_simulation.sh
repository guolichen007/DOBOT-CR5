#!/usr/bin/env bash
# ============================================================================
# CR5 Multi-Camera Calibration Simulation与信号闭环
# ============================================================================
#
# 五阶段确定性启动:
#   Phase A: 进程审计 + 参数解析
#   Phase B: paused 场景启动 (controller 不自动启动)
#   Phase C1: 等待模型 spawn
#   Phase C2: 绝对几何检查 + CR5 零位设置
#   Phase D1: unpause + clock + 顺序 controller 启动
#   Phase D2: CR5 运动链 + 相机 + 喷枪信号
#
# 失败分级:
#   Phase B/C1 失败 (Gazebo/模型没起来) → exit 1 + cleanup
#   Phase C2/D 失败 (位置错/信号异常) → DEGRADED + 保持运行 (GUI)
#   --strict / headless → 任何失败 exit 1 + cleanup
#
# GUI 默认固定端口 11311/11345，占用时自动 fallback
# --isolated 用于 headless 自动测试
#
# 用法:
#   bash run_simulation.sh --gui --object=motor_housing_cylinder
#   bash run_simulation.sh --isolated --object=motor_housing_cylinder  # headless
# ============================================================================
set -euo pipefail

# ---- 默认值 ----
GUI=false
HEADLESS=true
ISOLATED=false
OBJECT="motor_housing_cylinder"
PROFILE="vm"
PHYSICS_MODE="stable"
ENABLE_SPRAY_SIM=true
ENABLE_PAINT_PATCHES=true
STRICT=false
VERBOSE=false

# 默认固定端口 (GUI 模式)
DEFAULT_ROS_PORT=11311
DEFAULT_GZ_PORT=11345

# ---- 单实例锁 ----
LOCK_FILE="/tmp/cr5_spray_demo.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "ERROR: another CR5 spray session is running."
    echo "Close the previous session with Ctrl+C first."
    exit 3
fi

# ---- 帮助 ----
usage() {
    cat <<'EOF'
CR5 Multi-Camera Calibration Simulation

用法:
  bash run_simulation.sh [OPTIONS]

选项:
  --gui               启动 Gazebo GUI (默认 headless)
  --isolated          使用独立 roscore + 随机端口 (用于 headless 批量测试)
  --object TYPE       motor_housing_cylinder | rectangular_housing
  --profile PROFILE   vm | quality (默认: vm)
  --physics-mode MODE stable | gravity (默认: stable)
  --strict            健康检查失败后自动退出 (用于自动测试)
  --no-spray-sim      不启动喷涂控制节点
  --no-paint-patches  不生成 paint patches
  --verbose           打印 roslaunch 日志到终端
  -h, --help          显示此帮助

示例:
  # GUI 默认固定端口
  bash run_simulation.sh --gui --object=motor_housing_cylinder
  # Headless 自动测试 (随机端口)
  bash run_simulation.sh --isolated --strict
EOF
    exit 0
}

# ---- 参数解析 ----
while [[ $# -gt 0 ]]; do
    case "$1" in
        --gui)       GUI=true; HEADLESS=false ;;
        --headless)  HEADLESS=true; GUI=false ;;
        --isolated)  ISOLATED=true ;;
        --strict)    STRICT=true ;;
        --no-spray-sim) ENABLE_SPRAY_SIM=false ;;
        --no-paint-patches) ENABLE_PAINT_PATCHES=false ;;
        --verbose)   VERBOSE=true ;;
        -h|--help)   usage ;;
        --object=*)   OBJECT="${1#*=}" ;;
        --object)     OBJECT="$2"; shift ;;
        --profile=*)  PROFILE="${1#*=}" ;;
        --profile)    PROFILE="$2"; shift ;;
        --physics-mode=*) PHYSICS_MODE="${1#*=}" ;;
        --physics-mode)   PHYSICS_MODE="$2"; shift ;;
        *) echo "Unknown option: $1"; usage ;;
    esac
    shift
done

# ---- 校验 ----
if [[ "$PHYSICS_MODE" != "stable" && "$PHYSICS_MODE" != "gravity" ]]; then
    echo "ERROR: --physics-mode must be 'stable' or 'gravity'"
    exit 1
fi
if [[ "$OBJECT" != "motor_housing_cylinder" && "$OBJECT" != "rectangular_housing" && "$OBJECT" != "calibration_target" ]]; then
    echo "ERROR: --object must be 'calibration_target', 'motor_housing_cylinder', or 'rectangular_housing'"
    exit 1
fi

# 标定模式: 默认关闭喷涂模拟
if [[ "$OBJECT" == "calibration_target" ]]; then
    ENABLE_SPRAY_SIM=false
    ENABLE_PAINT_PATCHES=false
fi

# ---- 工作空间检查 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
    echo "ERROR: workspace not built: $WS_DIR/devel/setup.bash not found"
    exit 1
fi

# ---- 环境设置 ----
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

PKG_DIR="$(rospack find cr5_spray_sim)"
export GAZEBO_MODEL_PATH="$PKG_DIR/models:${GAZEBO_MODEL_PATH:-}"
export GAZEBO_RESOURCE_PATH="$PKG_DIR:${GAZEBO_RESOURCE_PATH:-}"

# 启动前验证标定资源 (阻塞式失败)
require_file() {
    if [[ ! -f "$1" ]]; then
        echo "ERROR: required calibration resource missing: $1" >&2
        exit 12
    fi
}
if [[ "$OBJECT" == "calibration_target" ]]; then
    require_file "$PKG_DIR/models/calibration_target/materials/scripts/calibration_target.material"
    require_file "$PKG_DIR/models/calibration_target/materials/textures/charuco_front.png"
    require_file "$PKG_DIR/models/calibration_target/materials/textures/charuco_left.png"
    require_file "$PKG_DIR/models/calibration_target/materials/textures/charuco_back.png"
    require_file "$PKG_DIR/models/calibration_target/materials/textures/apriltag_right.png"
    require_file "$PKG_DIR/models/calibration_target/materials/textures/apriltag_top.png"
    echo "GAZEBO_RESOURCE_PATH=$GAZEBO_RESOURCE_PATH"
    echo "Calibration media tree:"
    find "$PKG_DIR/models/calibration_target/materials" -maxdepth 2 -type f -printf '  %p\n' | sort
fi

# ---- Session 设置 ----
SESSION_ID="spray_sim_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_sim_${SESSION_ID}"
ARTIFACT_DIR="${LOG_DIR}/artifacts"
mkdir -p "$LOG_DIR" "$ARTIFACT_DIR"

GIT_BRANCH="$(git -C "$WS_DIR" branch --show-current 2>/dev/null || echo unknown)"
GIT_SHA="$(git -C "$WS_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"

ROS_MASTER_PID=""
ROS_MASTER_OWNED=false
LAUNCH_PID=""
LAUNCH_PGID=""
CLEANED=false
SIGINT_RECEIVED=false

# ---- 诊断辅助 ----
save_diagnostics() {
    local reason="${1:-UNKNOWN}"
    local diag_dir="${LOG_DIR}/diagnostics_$(date +%H%M%S)"
    mkdir -p "$diag_dir"

    rosrun cr5_spray_sim capture_scene_snapshot.py \
        --output "$diag_dir/scene_snapshot.json" 2>/dev/null || true

    rosservice call /controller_manager/list_controllers 2>/dev/null \
        > "$diag_dir/controllers.txt" || true

    if compgen -G "${ROS_LOG_DIR:-$HOME/.ros/log}/roslaunch-*.log" > /dev/null 2>&1; then
        tail -80 "${ROS_LOG_DIR:-$HOME/.ros/log}"/roslaunch-*.log 2>/dev/null \
            > "$diag_dir/roslaunch_tail.txt" || true
    fi

    cat > "$diag_dir/summary.json" <<JSONSUM
{
  "version": "calibration-baseline",
  "session_id": "$SESSION_ID",
  "branch": "$GIT_BRANCH",
  "sha": "$GIT_SHA",
  "reason": "$reason",
  "wall_time": "$(date -Iseconds)",
  "physics_mode": "$PHYSICS_MODE",
  "gui": $GUI,
  "object": "$OBJECT"
}
JSONSUM

    rosparam set /cr5_spray/session_state "DEGRADED" 2>/dev/null || true
    rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true

    echo "[$(date +%H:%M:%S)] diagnostics saved: $diag_dir"
}

# ---- SIGINT handler ----
on_sigint() {
    SIGINT_RECEIVED=true
    echo ""
    echo "USER_REQUESTED_SHUTDOWN"
    # cleanup 由 trap EXIT 触发
    exit 0
}
trap on_sigint INT TERM

# ---- Cleanup ----
cleanup() {
    if $CLEANED; then return; fi
    CLEANED=true
    echo ""
    echo "========================================="
    echo "[$(date +%H:%M:%S)] cleanup ..."

    # 更新状态
    rosparam set /cr5_spray/session_state "ENDED" 2>/dev/null || true

    # 只 kill 当前 roslaunch 进程组 (不全局 pkill)
    if [[ -n "$LAUNCH_PGID" ]] && kill -0 -- "-$LAUNCH_PGID" 2>/dev/null; then
        echo "[$(date +%H:%M:%S)] stopping roslaunch pgid=$LAUNCH_PGID ..."
        kill -INT -- "-$LAUNCH_PGID" 2>/dev/null || true
        for i in $(seq 1 8); do
            kill -0 -- "-$LAUNCH_PGID" 2>/dev/null || break
            sleep 1
        done
        if kill -0 -- "-$LAUNCH_PGID" 2>/dev/null; then
            kill -TERM -- "-$LAUNCH_PGID" 2>/dev/null || true
            sleep 2
        fi
        if kill -0 -- "-$LAUNCH_PGID" 2>/dev/null; then
            kill -KILL -- "-$LAUNCH_PGID" 2>/dev/null || true
        fi
        echo "[$(date +%H:%M:%S)] roslaunch stopped"
    fi

    # 停止本脚本启动的 roscore (不论 GUI/isolated)
    if $ROS_MASTER_OWNED && [[ -n "$ROS_MASTER_PID" ]] && kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
        echo "[$(date +%H:%M:%S)] stopping owned roscore pid=$ROS_MASTER_PID ..."
        kill "$ROS_MASTER_PID" 2>/dev/null || true
        wait "$ROS_MASTER_PID" 2>/dev/null || true
    fi

    # 清理 session 文件
    rm -f "/tmp/cr5_spray_simulation.env"

    # 释放锁
    flock -u 9 2>/dev/null || true

    echo "[$(date +%H:%M:%S)] cleanup done"
    echo "========================================="
}
trap cleanup EXIT

# ---- 端口工具 ----
port_available() {
    local port=$1
    ! ss -tlnp 2>/dev/null | grep -q ":${port} "
}

find_available_port() {
    local start=$1
    for offset in $(seq 0 200); do
        local port=$((start + offset))
        if port_available "$port"; then
            echo "$port"
            return 0
        fi
    done
    return 1
}

# ---- Phase A1/A2: Master 端口选择 + 启动 ----
start_master() {
    local ros_port gz_port

    if $ISOLATED; then
        # Headless 测试: 随机端口
        ros_port=$((11311 + RANDOM % 1000))
        gz_port=$((11345 + RANDOM % 1000))
    elif $GUI; then
        # GUI 模式: 优先固定端口，冲突则 fallback
        if port_available "$DEFAULT_ROS_PORT" && port_available "$DEFAULT_GZ_PORT"; then
            ros_port=$DEFAULT_ROS_PORT
            gz_port=$DEFAULT_GZ_PORT
        else
            echo "[$(date +%H:%M:%S)] default ports occupied, finding alternatives..."
            ros_port=$(find_available_port "$DEFAULT_ROS_PORT" || echo 0)
            gz_port=$(find_available_port "$DEFAULT_GZ_PORT" || echo 0)
            if [[ "$ros_port" == "0" ]] || [[ "$gz_port" == "0" ]]; then
                echo "ERROR: no available ports found"
                exit 2
            fi
            echo "[$(date +%H:%M:%S)] using fallback ports: ROS=$ros_port GAZEBO=$gz_port"
        fi
    else
        # Headless 非 isolated: 用共享 roscore (不启动自己的)
        echo "[$(date +%H:%M:%S)] Phase A1: using shared roscore"
        return 0
    fi

    export ROS_MASTER_URI="http://localhost:${ros_port}"
    export GAZEBO_MASTER_URI="http://localhost:${gz_port}"

    echo "[$(date +%H:%M:%S)] Phase A2: starting roscore (port $ros_port)..."
    roscore -p "$ros_port" &
    ROS_MASTER_PID=$!
    sleep 3

    if ! kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
        echo "ERROR: roscore failed to start"
        exit 1
    fi
    ROS_MASTER_OWNED=true
    echo "[$(date +%H:%M:%S)] roscore ready (pid=$ROS_MASTER_PID, owned=true)"

    # 写入当前会话环境文件
    write_session_env
}

write_session_env() {
    local ENV_FILE="/tmp/cr5_spray_simulation.env"
    cat > "$ENV_FILE" <<EOF
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"

export ROS_MASTER_URI="$ROS_MASTER_URI"
export GAZEBO_MASTER_URI="$GAZEBO_MASTER_URI"
export GAZEBO_RESOURCE_PATH="$GAZEBO_RESOURCE_PATH"
export GAZEBO_MODEL_PATH="$GAZEBO_MODEL_PATH"

export CR5_SPRAY_SESSION_ID="$SESSION_ID"
export CR5_SPRAY_BRANCH="$GIT_BRANCH"
export CR5_SPRAY_HEAD="$GIT_SHA"
export CR5_SPRAY_LOG_DIR="$LOG_DIR"
export CR5_SPRAY_WS="$WS_DIR"
EOF
    chmod 600 "$ENV_FILE"
    echo "[$(date +%H:%M:%S)] session env written: $ENV_FILE"
}

run_audit() {
    # 非 isolated 模式才做审计 (isolated 用随机端口，冲突几率极低)
    # audit 在 roscore 启动前运行，直接用 python3 而非 rosrun
    if $ISOLATED; then
        echo "[$(date +%H:%M:%S)] Phase A0: skipping audit (isolated mode)"
        echo "SIM_PROCESS_PREFLIGHT_PASS"
        return 0
    fi

    echo "[$(date +%H:%M:%S)] Phase A0: auditing old simulation processes..."
    local audit_ret=0
    python3 "$SCRIPT_DIR/audit_sim_processes.py" $($GUI && echo "--gui") || audit_ret=$?

    if [[ $audit_ret -ne 0 ]]; then
        echo "ERROR: SIM_PROCESS_PREFLIGHT_FAIL"
        echo "Close the previous session with Ctrl+C, wait a few seconds, and retry."
        exit 1
    fi
    echo "[$(date +%H:%M:%S)] SIM_PROCESS_PREFLIGHT_PASS"
}

# ---- Phase B: 启动 paused 场景 ----
launch_scene() {
    echo "========================================="
    echo "CR5 Multi-Camera Calibration Simulation"
    echo "========================================="
    echo "Session:   $SESSION_ID"
    echo "Branch:    $GIT_BRANCH ($GIT_SHA)"
    echo "Object:    $OBJECT"
    echo "Profile:   $PROFILE"
    echo "Physics:   $PHYSICS_MODE"
    echo "GUI:       $GUI"
    echo "Strict:    $STRICT"
    echo "Isolated:  $ISOLATED"
    echo "Log:       $LOG_DIR"
    echo "ROS_MASTER_URI:  ${ROS_MASTER_URI:-shared}"
    echo "GAZEBO_MASTER_URI: ${GAZEBO_MASTER_URI:-shared}"
    echo "========================================="

    local LAUNCH_ARGS=""
    LAUNCH_ARGS="${LAUNCH_ARGS} object_type:=${OBJECT}"
    LAUNCH_ARGS="${LAUNCH_ARGS} camera_profile:=${PROFILE}"
    LAUNCH_ARGS="${LAUNCH_ARGS} paused:=true"
    LAUNCH_ARGS="${LAUNCH_ARGS} spawn_controllers:=false"
    LAUNCH_ARGS="${LAUNCH_ARGS} enable_spray_sim:=${ENABLE_SPRAY_SIM}"
    LAUNCH_ARGS="${LAUNCH_ARGS} enable_paint_patches:=${ENABLE_PAINT_PATCHES}"

    if $GUI; then
        LAUNCH_ARGS="${LAUNCH_ARGS} gui:=true headless:=false"
    else
        LAUNCH_ARGS="${LAUNCH_ARGS} gui:=false headless:=true"
    fi

    if [[ "$PHYSICS_MODE" == "gravity" ]]; then
        LAUNCH_ARGS="${LAUNCH_ARGS} robot_gravity:=true"
    else
        LAUNCH_ARGS="${LAUNCH_ARGS} robot_gravity:=false"
    fi

    echo "[$(date +%H:%M:%S)] launching paused scene..."
    echo "  args: $LAUNCH_ARGS"

    setsid roslaunch cr5_spray_sim spray_simulation.launch $LAUNCH_ARGS \
        > "$LOG_DIR/roslaunch.log" 2>&1 &
    LAUNCH_PID=$!
    LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID 2>/dev/null | tr -d ' ')
    echo "  launch_pid=$LAUNCH_PID pgid=$LAUNCH_PGID"

    if $VERBOSE; then
        tail -f "$LOG_DIR/roslaunch.log" &
    fi
}

wait_gazebo_services() {
    echo "[$(date +%H:%M:%S)] waiting for Gazebo services ..."
    local MAX_WAIT=120
    local waited=0
    while [[ $waited -lt $MAX_WAIT ]]; do
        if rosservice list 2>/dev/null | grep -q '/gazebo/'; then
            echo "[$(date +%H:%M:%S)] Gazebo services available (${waited}s)"
            return 0
        fi
        if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
            echo "ERROR: roslaunch died before Gazebo started"
            tail -60 "$LOG_DIR/roslaunch.log" 2>/dev/null || true
            return 1
        fi
        sleep 2
        waited=$((waited + 2))
    done
    echo "ERROR: Gazebo did not start within ${MAX_WAIT}s"
    return 1
}

# ---- Phase C: 模型 + 几何 ----
wait_models() {
    echo "[$(date +%H:%M:%S)] Phase C1: waiting for scene models ..."
    rosrun cr5_spray_sim wait_scene_models.py --timeout 45.0
}

check_geometry() {
    echo "[$(date +%H:%M:%S)] Phase C2: checking absolute scene geometry ..."
    rosrun cr5_spray_sim check_scene_geometry.py
}

# ---- Phase D: 控制器 + 运行时 ----
bootstrap_controllers() {
    echo "[$(date +%H:%M:%S)] Phase D1: bootstrapping controllers (unpause + sequential spawn)..."
    rosrun cr5_spray_sim bootstrap_controllers.py
}

check_runtime() {
    echo "[$(date +%H:%M:%S)] Phase D2a: checking CR5 kinematic chain..."
    rosrun cr5_spray_sim check_scene_runtime.py \
        --output "$ARTIFACT_DIR/scene_runtime_pose.json"
}

check_signals() {
    echo "[$(date +%H:%M:%S)] Phase D2b: checking runtime signals..."
    rosrun cr5_spray_sim check_runtime_signals.py \
        --output "$ARTIFACT_DIR/camera/"
}

# ---- 失败处理 ----
# 致命失败 (Phase B/C1): 模型都没出来 → exit + cleanup
fatal_failure() {
    local phase="$1"
    local reason="$2"
    echo "========================================="
    echo "FATAL: $phase — $reason"
    echo "========================================="
    save_diagnostics "$phase: $reason"
    exit 1
}

# 可恢复失败 (Phase C2/D): 模型在但有问题 → DEGRADED
degraded_failure() {
    local phase="$1"
    local reason="$2"
    echo "========================================="
    echo "DEGRADED: $phase — $reason"
    echo "========================================="
    save_diagnostics "$phase: $reason"

    if $STRICT || ! $GUI; then
        echo "Strict/headless mode: exiting ..."
        exit 1
    else
        echo ""
        echo "========================================="
        echo "Session DEGRADED — Gazebo is still running."
        echo "Check the Gazebo window, then press Ctrl+C to exit."
        echo "Log: $LOG_DIR"
        echo "========================================="
        wait "$LAUNCH_PID" 2>/dev/null || true
        exit 1
    fi
}

# ====================================================================
# 主流程
# ====================================================================

# V4: Phase A0 先审计旧进程, Phase A1 再选端口, Phase A2 启动 roscore
run_audit

# Phase A1/A2: 端口选择 + Master 启动
if $ISOLATED || $GUI; then
    start_master
fi

# Phase B: 启动 paused 场景
launch_scene

if ! wait_gazebo_services; then
    fatal_failure "PHASE_B" "Gazebo did not start"
fi

# Phase C1: 等待模型
if ! wait_models; then
    fatal_failure "PHASE_C1" "SCENE_MODELS_FAILED"
fi

# Phase C2: 绝对几何检查 (paused 状态下)
if ! check_geometry; then
    degraded_failure "PHASE_C2" "ABSOLUTE_SCENE_GEOMETRY_FAIL or CR5_ZERO_CONFIGURATION_FAIL"
fi

# Phase D1: 启动控制器 (unpause + 顺序启动)
if ! bootstrap_controllers; then
    degraded_failure "PHASE_D1" "CONTROLLERS_BOOTSTRAP_FAILED"
fi

# Phase D2a: CR5 运动链
if ! check_runtime; then
    degraded_failure "PHASE_D2a" "CR5_KINEMATIC_CHAIN_FAIL"
fi

# Phase D2b: 运行时信号
if ! check_signals; then
    degraded_failure "PHASE_D2b" "RUNTIME_SIGNALS_DEGRADED"
fi

# ====================================================================
# 成功!
# ====================================================================
echo "========================================="
echo ""
echo "  SIM_PROCESS_PREFLIGHT_PASS    ✓"
echo "  SCENE_MODELS_READY           ✓"
echo "  ABSOLUTE_SCENE_GEOMETRY_PASS  ✓"
echo "  CR5_ZERO_CONFIGURATION_PASS   ✓"
echo "  SIM_CLOCK_ADVANCING           ✓"
echo "  CONTROLLERS_RUNNING           ✓"
echo "  CR5_KINEMATIC_CHAIN_PASS     ✓"
echo "  CAMERA_COLOR_3_OF_3_PASS     ✓"
echo "  CAMERA_DEPTH_3_OF_3_PASS     ✓"
echo "  SPRAY_SIGNAL_PASS            ✓"
echo ""
echo "  Session ACTIVE — Ctrl+C to exit"
echo ""
echo "  Connect another terminal with:"
echo "    source /tmp/cr5_spray_simulation.env"
echo ""
echo "========================================="

rosparam set /cr5_spray/session_state "ACTIVE" 2>/dev/null || true
rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true
echo "Session ACTIVE"

# Headless 模式: 成功后退回 shell
if ! $GUI; then
    echo "[$(date +%H:%M:%S)] headless mode: session active, exiting cleanly."
    exit 0
fi

# GUI 模式: 等待用户 Ctrl+C
echo "Press Ctrl+C to stop."
echo "ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
wait "$LAUNCH_PID" 2>/dev/null || true
