#!/usr/bin/env bash
# ============================================================================
# CR5 Spray Demo V3.3.6 — 最小稳定运行版启动器
# ============================================================================
#
# 四阶段简化启动:
#   Phase A: 单实例检查 + 参数解析
#   Phase B: 启动 unpaused 场景 (gravity=false, controllers 直接启动)
#   Phase C: 等待模型 + 控制器就绪
#   Phase D: 验证姿态 + 运行时信号
#
# GUI 模式: 检查失败 → DEGRADED，Gazebo 保持打开，不自动退出。
# --strict / headless: 检查失败 → exit 1 + cleanup.
#
# 用法:
#   bash run_scene_v33_spray.sh [--gui] [--isolated] [--object TYPE] [--profile PROFILE]
#                               [--physics-mode stable|gravity] [--strict] [--verbose]
#                               [--no-spray-sim] [--no-paint-patches]
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

# ---- 单实例锁 ----
LOCK_FILE="/tmp/cr5_spray_demo.lock"
exec 9>"$LOCK_FILE"
if ! flock -n 9; then
    echo "ERROR: another CR5 spray session is running."
    echo "Close the previous session with Ctrl+C first, or remove $LOCK_FILE if stale."
    exit 3
fi

# ---- 帮助 ----
usage() {
    cat <<'EOF'
CR5 Spray Demo V3.3.6 — 最小稳定运行版

用法:
  bash run_scene_v33_spray.sh [OPTIONS]

选项:
  --gui              启动 Gazebo GUI (默认 headless)
  --isolated          使用独立 roscore + gazebo master
  --object TYPE       motor_housing_cylinder | rectangular_housing (默认: motor_housing_cylinder)
  --profile PROFILE   vm | quality (默认: vm)
  --physics-mode MODE stable | gravity (默认: stable, CR5 运动链无重力)
  --strict            健康检查失败后自动退出 (用于自动测试)
  --no-spray-sim      不启动喷涂控制节点
  --no-paint-patches  不生成 paint patches
  --verbose           打印 roslaunch 日志到终端
  -h, --help          显示此帮助

示例:
  bash run_scene_v33_spray.sh --gui --isolated
  bash run_scene_v33_spray.sh --gui --isolated --strict              # 失败自动退出
  bash run_scene_v33_spray.sh --isolated --strict                    # headless 自动测试
EOF
    exit 0
}

# ---- 参数解析 (支持 --key=value 和 --key value 两种格式) ----
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
    echo "ERROR: --physics-mode must be 'stable' or 'gravity', got '$PHYSICS_MODE'"
    exit 1
fi
if [[ "$OBJECT" != "motor_housing_cylinder" && "$OBJECT" != "rectangular_housing" ]]; then
    echo "ERROR: --object must be 'motor_housing_cylinder' or 'rectangular_housing'"
    exit 1
fi

# ---- 工作空间检查 ----
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
WS_DIR="$(cd "$SCRIPT_DIR/../../.." && pwd)"
if [[ ! -f "$WS_DIR/devel/setup.bash" ]]; then
    echo "ERROR: workspace not built: $WS_DIR/devel/setup.bash not found"
    echo "Run: cd $WS_DIR && catkin_make"
    exit 1
fi

# ---- 环境设置 ----
source /opt/ros/noetic/setup.bash
source "$WS_DIR/devel/setup.bash"
export GAZEBO_MODEL_PATH="$WS_DIR/src/cr5_spray_sim/models:${GAZEBO_MODEL_PATH:-}"

# ---- Session 设置 ----
SESSION_ID="v336_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_v336_${SESSION_ID}"
mkdir -p "$LOG_DIR"

GIT_BRANCH="$(git -C "$WS_DIR" branch --show-current 2>/dev/null || echo unknown)"
GIT_SHA="$(git -C "$WS_DIR" rev-parse --short HEAD 2>/dev/null || echo unknown)"

ROS_MASTER_PID=""
LAUNCH_PID=""
LAUNCH_PGID=""
CLEANED=false

# ---- 诊断辅助函数 ----
save_diagnostics() {
    local reason="${1:-UNKNOWN}"
    local diag_dir="${LOG_DIR}/diagnostics_$(date +%H%M%S)"
    mkdir -p "$diag_dir"

    # 模型快照
    rosrun cr5_spray_sim capture_scene_snapshot_v335.py \
        --output "$diag_dir/scene_snapshot.json" 2>/dev/null || true

    # 控制器状态
    rosservice call /controller_manager/list_controllers 2>/dev/null \
        > "$diag_dir/controllers.txt" || true

    # roslaunch 日志
    if compgen -G "${ROS_LOG_DIR:-$HOME/.ros/log}/roslaunch-*.log" > /dev/null 2>&1; then
        tail -80 "${ROS_LOG_DIR:-$HOME/.ros/log}"/roslaunch-*.log 2>/dev/null \
            > "$diag_dir/roslaunch_tail.txt" || true
    fi

    # 诊断摘要
    cat > "$diag_dir/summary.json" <<JSONSUM
{
  "version": "V3.3.6",
  "session_id": "$SESSION_ID",
  "branch": "$GIT_BRANCH",
  "sha": "$GIT_SHA",
  "reason": "$reason",
  "wall_time": "$(date -Iseconds)",
  "physics_mode": "$PHYSICS_MODE"
}
JSONSUM

    # 设置 session state
    rosparam set /cr5_spray/session_state "DEGRADED" 2>/dev/null || true
    rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true

    echo "[$(date +%H:%M:%S)] diagnostics saved: $diag_dir"
}

# ---- Cleanup ----
cleanup() {
    if $CLEANED; then return; fi
    CLEANED=true
    echo ""
    echo "========================================="
    echo "[$(date +%H:%M:%S)] cleanup ..."

    # 更新状态
    rosparam set /cr5_spray/session_state "ENDED" 2>/dev/null || true

    # 停止 gzclient
    if $GUI; then
        pkill -f "gzclient" 2>/dev/null || true
        echo "[$(date +%H:%M:%S)] gzclient stopped"
    fi

    # 停止 rqt
    pkill -f "rqt" 2>/dev/null || true

    # 停止 roslaunch 进程组
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

    # 停止独立 roscore
    if $ISOLATED && [[ -n "$ROS_MASTER_PID" ]] && kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
        echo "[$(date +%H:%M:%S)] stopping roscore pid=$ROS_MASTER_PID ..."
        kill "$ROS_MASTER_PID" 2>/dev/null || true
        wait "$ROS_MASTER_PID" 2>/dev/null || true
    fi

    # 清理 env 文件
    if $ISOLATED; then
        rm -f "/tmp/cr5_spray_v336_env" "/tmp/cr5_spray_v336_env_pending"
    fi

    # 释放锁
    flock -u 9 2>/dev/null || true

    echo "[$(date +%H:%M:%S)] cleanup done"
    echo "========================================="
}

trap cleanup INT TERM EXIT

# ---- Phase A: 独立 master 启动 ----
start_isolated_master() {
    local ROS_PORT=$((11311 + RANDOM % 1000))
    local GZ_PORT=$((11345 + RANDOM % 1000))

    export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
    export GAZEBO_MASTER_URI="http://localhost:${GZ_PORT}"

    roscore -p "$ROS_PORT" &
    ROS_MASTER_PID=$!
    sleep 3

    if ! kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
        echo "ERROR: roscore failed to start"
        exit 1
    fi

    # 写入 pending env (activate 时转为 current)
    cat > "/tmp/cr5_spray_v336_env_pending" <<ENV
SESSION_ID=$SESSION_ID
ROS_MASTER_URI=$ROS_MASTER_URI
GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI
LAUNCH_PID=__PENDING__
LAUNCH_PGID=__PENDING__
BRANCH=$GIT_BRANCH
SHA=$GIT_SHA
PHYSICS_MODE=$PHYSICS_MODE
ENV
}

activate_session() {
    if $ISOLATED; then
        sed "s/LAUNCH_PID=__PENDING__/LAUNCH_PID=$LAUNCH_PID/; \
             s/LAUNCH_PGID=__PENDING__/LAUNCH_PGID=$LAUNCH_PGID/" \
            "/tmp/cr5_spray_v336_env_pending" > "/tmp/cr5_spray_v336_env"
        rm -f "/tmp/cr5_spray_v336_env_pending"
    fi
    rosparam set /cr5_spray/session_state "ACTIVE" 2>/dev/null || true
    rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true
    echo "Session ACTIVE"
}

# ---- Phase B: 启动场景 ----
launch_scene() {
    echo "========================================="
    echo "CR5 Spray Demo V3.3.6 — 最小稳定运行版"
    echo "========================================="
    echo "Session:   $SESSION_ID"
    echo "Branch:    $GIT_BRANCH ($GIT_SHA)"
    echo "Object:    $OBJECT"
    echo "Profile:   $PROFILE"
    echo "Physics:   $PHYSICS_MODE"
    echo "GUI:       $GUI"
    echo "Strict:    $STRICT"
    echo "Log:       $LOG_DIR"
    echo "========================================="

    # 构建 launch 参数
    local LAUNCH_ARGS=""
    LAUNCH_ARGS="${LAUNCH_ARGS} object_type:=${OBJECT}"
    LAUNCH_ARGS="${LAUNCH_ARGS} camera_profile:=${PROFILE}"
    LAUNCH_ARGS="${LAUNCH_ARGS} paused:=false"
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

    echo "[$(date +%H:%M:%S)] launching scene..."
    echo "  args: $LAUNCH_ARGS"

    setsid roslaunch cr5_spray_sim scene_v33_spray.launch $LAUNCH_ARGS \
        > "$LOG_DIR/roslaunch.log" 2>&1 &
    LAUNCH_PID=$!
    LAUNCH_PGID=$(ps -o pgid= -p $LAUNCH_PID 2>/dev/null | tr -d ' ')
    echo "  launch_pid=$LAUNCH_PID pgid=$LAUNCH_PGID"

    if $VERBOSE; then
        tail -f "$LOG_DIR/roslaunch.log" &
    fi
}

# ---- Phase C: 等待就绪 ----
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

wait_models() {
    echo "[$(date +%H:%M:%S)] Phase C1: waiting for scene models ..."
    rosrun cr5_spray_sim wait_scene_models_v336.py --timeout 45.0
    local ret=$?
    if [[ $ret -ne 0 ]]; then
        echo "ERROR: SCENE_MODELS_FAILED"
        return 1
    fi
    return 0
}

wait_controllers() {
    echo "[$(date +%H:%M:%S)] Phase C2: waiting for controllers running ..."
    rosrun cr5_spray_sim wait_controllers_running_v336.py --timeout 45.0
    local ret=$?
    if [[ $ret -ne 0 ]]; then
        echo "ERROR: CONTROLLERS_FAILED"
        return 1
    fi
    return 0
}

# ---- Phase D: 验证 ----
check_clock() {
    echo "[$(date +%H:%M:%S)] Phase D1: checking clock ..."
    rosrun cr5_spray_sim check_clock_v336.py
    local ret=$?
    if [[ $ret -ne 0 ]]; then
        echo "ERROR: CLOCK_NOT_ADVANCING"
        return 1
    fi
    return 0
}

check_poses() {
    echo "[$(date +%H:%M:%S)] Phase D2: checking model poses ..."
    rosrun cr5_spray_sim check_model_poses_v336.py
    local ret=$?
    if [[ $ret -ne 0 ]]; then
        echo "ERROR: MODEL_POSES_UNSTABLE"
        return 1
    fi
    return 0
}

check_signals() {
    echo "[$(date +%H:%M:%S)] Phase D3: checking runtime signals ..."
    rosrun cr5_spray_sim check_runtime_signals_v336.py
    local ret=$?
    if [[ $ret -ne 0 ]]; then
        echo "ERROR: RUNTIME_SIGNALS_DEGRADED"
        return 1
    fi
    return 0
}

# ---- 处理失败 ----
handle_failure() {
    local phase="$1"
    local reason="$2"
    echo "========================================="
    echo "FAILURE: $phase — $reason"
    echo "========================================="

    save_diagnostics "$phase: $reason"

    if $STRICT || ! $GUI; then
        # 严格模式或 headless → 自动退出
        echo "Strict/headless mode: exiting ..."
        exit 1
    else
        # GUI 模式 → DEGRADED，保持运行等待用户 Ctrl+C
        echo ""
        echo "========================================="
        echo "Session DEGRADED — Gazebo is still running."
        echo "Check the Gazebo window, then press Ctrl+C to exit."
        echo "Log: $LOG_DIR"
        echo "========================================="
        # 等待 launch 进程 — 用户 Ctrl+C 触发 cleanup
        wait "$LAUNCH_PID" 2>/dev/null || true
        exit 1
    fi
}

# ====================================================================
# 主流程
# ====================================================================

# Phase A: 独立 master (如果需要)
if $ISOLATED; then
    echo "[$(date +%H:%M:%S)] Phase A: starting isolated roscore ..."
    start_isolated_master
else
    echo "[$(date +%H:%M:%S)] Phase A: using shared roscore"
fi

# Phase B: 启动场景
launch_scene

# 等待 Gazebo 服务
if ! wait_gazebo_services; then
    handle_failure "PHASE_B" "Gazebo did not start"
fi

# Phase C: 模型 + 控制器就绪
if ! wait_models; then
    handle_failure "PHASE_C1" "SCENE_MODELS_FAILED"
fi

if ! wait_controllers; then
    handle_failure "PHASE_C2" "CONTROLLERS_FAILED"
fi

# Phase D: 验证
echo "========================================="

if ! check_clock; then
    handle_failure "PHASE_D1" "CLOCK_NOT_ADVANCING"
fi

if ! check_poses; then
    handle_failure "PHASE_D2" "MODEL_POSES_UNSTABLE"
fi

if ! check_signals; then
    handle_failure "PHASE_D3" "RUNTIME_SIGNALS_DEGRADED"
fi

# ====================================================================
# 成功!
# ====================================================================
echo "========================================="
echo ""
echo "  SCENE_MODELS_READY     ✓"
echo "  CONTROLLERS_RUNNING    ✓"
echo "  SIM_CLOCK_ADVANCING    ✓"
echo "  MODEL_POSES_STABLE     ✓"
echo "  RUNTIME_SIGNALS_READY  ✓"
echo ""
echo "  Session ACTIVE — Ctrl+C to exit"
echo ""
echo "========================================="

activate_session

# Headless 模式: 成功后退回 shell
if ! $GUI; then
    echo "[$(date +%H:%M:%S)] headless mode: session active, exiting cleanly."
    exit 0
fi

# GUI 模式: 等待用户 Ctrl+C
echo "Press Ctrl+C to stop."
wait "$LAUNCH_PID" 2>/dev/null || true
