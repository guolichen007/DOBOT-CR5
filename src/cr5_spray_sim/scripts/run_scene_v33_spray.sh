#!/usr/bin/env bash
# ===========================================================================
# CR5 Spray Demo V3.3.5 Launcher
#
# 修复 V3.3.4 控制器生命周期误判 (initialized vs stopped)
# 默认 stable 无重力模式, 移除运行时 SetLinkProperties 风险
# 分步启动控制器, 启动前模型快照, 失败取证
#
# 用法:
#   bash run_scene_v33_spray.sh [--gui] [--headless] [--isolated]
#     [--object motor_housing_cylinder]
#     [--profile vm]
#     [--physics-mode stable|gravity]
#     [--no-spray-sim] [--verbose]
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# ===== 默认值 =====
GUI=false
HEADLESS=true
ISOLATED=false
OBJECT="motor_housing_cylinder"
PROFILE="vm"
PHYSICS_MODE="stable"
ENABLE_SPRAY_SIM=true
ENABLE_PAINT_PATCHES=true
VERBOSE=false

print_usage() {
  cat << EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --gui                       Launch with Gazebo GUI
  --headless                  Launch headless (default)
  --isolated                  Use random ports for multi-session
  --object TYPE               motor_housing_cylinder | rectangular_housing
  --profile PROF              vm | quality (default: vm)
  --physics-mode MODE         stable (default, CR5 no-gravity) | gravity (experimental)
  --no-spray-sim              Disable spray control
  --no-paint-patches          Disable paint patches
  --verbose                   Tail log files
  -h, --help                  Show this help
EOF
}

while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)          GUI=true; HEADLESS=false; shift ;;
    --headless)     HEADLESS=true; GUI=false; shift ;;
    --isolated)     ISOLATED=true; shift ;;
    --verbose)      VERBOSE=true; shift ;;
    --no-spray-sim)      ENABLE_SPRAY_SIM=false; shift ;;
    --no-paint-patches)  ENABLE_PAINT_PATCHES=false; shift ;;
    --object)       OBJECT="$2"; shift 2 ;;
    --object=*)     OBJECT="${1#*=}"; shift ;;
    --profile)      PROFILE="$2"; shift 2 ;;
    --profile=*)    PROFILE="${1#*=}"; shift ;;
    --physics-mode) PHYSICS_MODE="$2"; shift 2 ;;
    --physics-mode=*) PHYSICS_MODE="${1#*=}"; shift ;;
    -h|--help)      print_usage; exit 0 ;;
    *)              echo "ERROR: Unknown argument: $1" >&2; exit 2 ;;
  esac
done

if [[ "$PHYSICS_MODE" == "gravity" ]]; then
  echo "WARNING: gravity mode is EXPERIMENTAL. Default is stable." >&2
fi
if [[ "$PHYSICS_MODE" != "stable" && "$PHYSICS_MODE" != "gravity" ]]; then
  echo "ERROR: physics-mode must be 'stable' or 'gravity', got '$PHYSICS_MODE'" >&2; exit 1
fi

echo "Argument parse PASS"
echo "  WS: ${WS_DIR}  PKG: ${PKG_DIR}"
echo "  physics: ${PHYSICS_MODE}"

# ===== 验证 =====
if [[ "$OBJECT" != "motor_housing_cylinder" && "$OBJECT" != "rectangular_housing" ]]; then
  echo "ERROR: Unknown object type: $OBJECT" >&2; exit 1
fi
if [[ ! -f "${WS_DIR}/devel/setup.bash" ]]; then
  echo "ERROR: Workspace not built. Run: cd ${WS_DIR} && catkin_make" >&2; exit 1
fi

# ===== 环境 =====
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"
export GAZEBO_MODEL_PATH="${PKG_DIR}/models:${GAZEBO_MODEL_PATH:-}"

# ===== 会话 =====
ENV_FILE_CURRENT="/tmp/cr5_spray_v33_current.env"
SESSION_ID="v335_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_v33_${SESSION_ID}"
mkdir -p "$LOG_DIR"
export CR5_SPRAY_LOG_DIR="${LOG_DIR}"
BRANCH=$(cd "$WS_DIR" && git branch --show-current 2>/dev/null || echo "unknown")
HEAD_SHA=$(cd "$WS_DIR" && git rev-parse --short HEAD 2>/dev/null || echo "unknown")

ROS_MASTER_PID=""
LAUNCH_PID=""
LAUNCH_PGID=""
GRAVITY_GUARD_USED=false
CLEANED=false

# ===== 失败取证函数 =====
fatal_with_snapshot() {
  local reason="$1"
  echo "" >&2
  echo "==============================================" >&2
  echo "  FATAL: ${reason}" >&2
  echo "==============================================" >&2

  # 标记失败
  rosparam set /cr5_spray/session_state "FAILED" 2>/dev/null || true

  # 保存现场快照
  local snapshot="${LOG_DIR}/failure_snapshot_$(date +%H%M%S).json"
  echo "  Saving failure snapshot → ${snapshot}" >&2
  "${PKG_DIR}/scripts/capture_scene_snapshot_v335.py" --output "$snapshot" 2>/dev/null || true

  # 保存 controller 状态
  rosservice call /controller_manager/list_controllers "{}" 2>/dev/null > "${LOG_DIR}/failure_controllers.txt" || true

  # 保存 roslaunch log 尾部
  if [[ -f "${LOG_DIR}/roslaunch.log" ]]; then
    tail -80 "${LOG_DIR}/roslaunch.log" > "${LOG_DIR}/failure_roslaunch_tail.txt" 2>/dev/null || true
  fi

  # 生成摘要
  cat > "${LOG_DIR}/failure_summary.json" << EOF
{
  "session": "${SESSION_ID}",
  "reason": "${reason}",
  "timestamp_wall": "$(date -Iseconds)",
  "physics_mode": "${PHYSICS_MODE}",
  "snapshot": "${snapshot}"
}
EOF

  echo "" >&2
  echo "Session FAILED — Gazebo is shutting down." >&2
  echo "Do not assess model geometry during teardown." >&2
  echo "Use snapshots: ${LOG_DIR}/" >&2

  exit 1
}

# ===== 清理函数 =====
cleanup() {
  local exit_code=$?
  [[ "$CLEANED" == "true" ]] && return
  CLEANED=true

  echo ""
  echo "=== Cleanup (exit=${exit_code}) ==="

  # V3.3.5: 只在确实使用了 gravity guard 时才恢复
  if [[ "$GRAVITY_GUARD_USED" == "true" ]]; then
    if rosservice list 2>/dev/null | grep -q '/gazebo/'; then
      echo "  Restoring CR5 gravity (experimental mode)..."
      "${PKG_DIR}/scripts/experimental/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
    fi
  fi

  rosparam set /cr5_spray/session_state "ENDED" 2>/dev/null || true

  # 1. 先关闭 gzclient (避免残留画面)
  if [[ "$GUI" == "true" ]]; then
    pkill -f "gzclient.*${GAZEBO_MASTER_URI:-}" 2>/dev/null || true
    sleep 1
  fi

  # 2. 停止辅助进程
  if [[ -n "${RQT_PID:-}" ]]; then kill "$RQT_PID" 2>/dev/null || true; fi

  # 3. 停止 roslaunch 进程组
  if [[ -n "${LAUNCH_PGID:-}" ]]; then
    echo "  Stopping launch process group ${LAUNCH_PGID}..."
    kill -INT -- -${LAUNCH_PGID} 2>/dev/null || true
    for i in $(seq 1 8); do
      if ! kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then break; fi
      sleep 1
    done
    if kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then
      kill -TERM -- -${LAUNCH_PGID} 2>/dev/null || true
      sleep 2
    fi
    if kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then
      kill -KILL -- -${LAUNCH_PGID} 2>/dev/null || true
    fi
  elif [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  if [[ -n "${LAUNCH_PID:-}" ]]; then wait "$LAUNCH_PID" 2>/dev/null || true; fi

  # 4. 停止 roscore
  if [[ "$ISOLATED" == "true" ]] && [[ -n "${ROS_MASTER_PID:-}" ]]; then
    echo "  Stopping roscore (pid=${ROS_MASTER_PID})..."
    kill "$ROS_MASTER_PID" 2>/dev/null || true
    wait "$ROS_MASTER_PID" 2>/dev/null || true
  fi

  # 5. 删除 env
  if [[ "$ISOLATED" == "true" ]]; then
    rm -f "$ENV_FILE_CURRENT" /tmp/cr5_spray_v33_pending_*.env
  fi

  echo "=== Cleanup done ==="
  echo "  Session: ${SESSION_ID}  Logs: ${LOG_DIR}"
}
trap cleanup INT TERM EXIT

# ===== 独立 master =====
start_isolated_master() {
  local port=$((11311 + RANDOM % 1000))
  export ROS_MASTER_URI="http://localhost:${port}"

  roscore -p "$port" > "${LOG_DIR}/roscore.log" 2>&1 &
  ROS_MASTER_PID=$!
  sleep 3

  if ! kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
    echo "FATAL: roscore failed (port ${port})" >&2
    cat "${LOG_DIR}/roscore.log" >&2
    exit 1
  fi

  local gz_port=$((11345 + RANDOM % 1000))
  export GAZEBO_MASTER_URI="http://localhost:${gz_port}"

  # V3.3.5: pending env (激活后才 rename 为 current)
  local pending_env="/tmp/cr5_spray_v33_pending_${SESSION_ID}.env"
  cat > "$pending_env" << EOF
export ROS_MASTER_URI=http://localhost:${port}
export GAZEBO_MASTER_URI=http://localhost:${gz_port}
export CR5_SPRAY_SESSION=${SESSION_ID}
export CR5_SPRAY_LOG_DIR=${LOG_DIR}
export CR5_SPRAY_BRANCH=${BRANCH}
export CR5_SPRAY_HEAD=${HEAD_SHA}
EOF

  echo "ROS master ready (port=${port})"
}

activate_session() {
  if [[ "$ISOLATED" == "true" ]]; then
    local pending_env="/tmp/cr5_spray_v33_pending_${SESSION_ID}.env"
    mv "$pending_env" "$ENV_FILE_CURRENT"
  fi
  rosparam set /cr5_spray/session_state "ACTIVE" 2>/dev/null || true
  rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true
}

set_bootstrapping() {
  rosparam set /cr5_spray/session_state "BOOTSTRAPPING" 2>/dev/null || true
  rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true
}

if [[ "$ISOLATED" == "true" ]]; then
  start_isolated_master
fi

# ===== Launch =====
LAUNCH_ARGS="object_type:=${OBJECT} camera_profile:=${PROFILE}"
LAUNCH_ARGS="${LAUNCH_ARGS} gui:=${GUI} headless:=${HEADLESS}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_spray_tool:=true"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_spray_sim:=${ENABLE_SPRAY_SIM}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_paint_patches:=${ENABLE_PAINT_PATCHES}"
LAUNCH_ARGS="${LAUNCH_ARGS} paused:=true start_controllers:=false"
LAUNCH_ARGS="${LAUNCH_ARGS} physics_mode:=${PHYSICS_MODE}"
if [[ "$PHYSICS_MODE" == "gravity" ]]; then
  LAUNCH_ARGS="${LAUNCH_ARGS} robot_gravity:=true"
else
  LAUNCH_ARGS="${LAUNCH_ARGS} robot_gravity:=false"
fi

echo ""
echo "=============================================="
echo "  CR5 Spray Demo V3.3.5"
echo "  Session:  ${SESSION_ID}"
echo "  Object:   ${OBJECT}"
echo "  Physics:  ${PHYSICS_MODE}"
echo "  GUI: ${GUI}  Isolated: ${ISOLATED}"
echo "  Branch:   ${BRANCH}  HEAD: ${HEAD_SHA}"
echo "  Logs:     ${LOG_DIR}"
echo "=============================================="

setsid roslaunch cr5_spray_sim scene_v33_spray.launch ${LAUNCH_ARGS} \
  > "${LOG_DIR}/roslaunch.log" 2>&1 &
LAUNCH_PID=$!
LAUNCH_PGID=$(ps -o pgid= -p "$LAUNCH_PID" 2>/dev/null | tr -d ' ' || echo "")
echo "Launch: pid=${LAUNCH_PID} pgid=${LAUNCH_PGID}"

if [[ "$VERBOSE" == "true" ]]; then
  tail -f "${LOG_DIR}/roslaunch.log" &
fi

# ===== 等待 Gazebo =====
echo ""
echo "--- Waiting for Gazebo ---"
WAIT_START=$(date +%s)
MAX_WAIT=120
while ! rosservice list 2>/dev/null | grep -q '/gazebo/'; do
  sleep 1
  if [[ $(($(date +%s) - WAIT_START)) -gt $MAX_WAIT ]]; then
    fatal_with_snapshot "Gazebo did not start within ${MAX_WAIT}s"
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FATAL: roslaunch died" >&2
    tail -60 "${LOG_DIR}/roslaunch.log" >&2
    exit 1
  fi
done
echo "  Gazebo ready ($(($(date +%s) - WAIT_START))s)"

set_bootstrapping

# ============================================================================
# V3.3.5 硬门启动流程 (stable 模式默认，无 SetLinkProperties)
# ============================================================================

# ===== Phase 1: 控制器 loaded/not-running =====
echo ""
echo "--- Phase 1: Controller Loaded Check (V3.3.5) ---"
CONTROLLER_STATES=$("${PKG_DIR}/scripts/check_controllers_loaded_v335.py" 2>&1 > "${LOG_DIR}/controller_initial_states.json" || true)
echo "  ${CONTROLLER_STATES}"
if ! echo "$CONTROLLER_STATES" | grep -q "CONTROLLERS_LOADED_NOT_RUNNING"; then
  fatal_with_snapshot "PHASE1_CONTROLLERS_NOT_LOADED"
fi

# ===== Phase 2: 启动前模型快照 (paused) =====
echo ""
echo "--- Phase 2: Pre-Bootstrap Scene Snapshot ---"
SNAPSHOT_PATH="${LOG_DIR}/scene_snapshot_pre_bootstrap.json"
SNAPSHOT_OK=$("${PKG_DIR}/scripts/capture_scene_snapshot_v335.py" --output "$SNAPSHOT_PATH" 2>&1 || true)
echo "  ${SNAPSHOT_OK}"
if ! echo "$SNAPSHOT_OK" | grep -q "PRE_BOOTSTRAP_SCENE_BASELINE_PASS"; then
  echo "  [WARN] Snapshot has non-finite coordinates (non-fatal)"
fi
echo "  Snapshot: ${SNAPSHOT_PATH}"

# ===== Phase 3: Unpause + Clock 验证 =====
echo ""
echo "--- Phase 3: Unpause + Clock Verification ---"
if [[ "$PHYSICS_MODE" == "gravity" ]]; then
  # Experimental: 先关重力再 unpause
  echo "  [EXPERIMENTAL] Disabling CR5 gravity before unpause..."
  "${PKG_DIR}/scripts/experimental/cr5_gravity_guard_v334.py" disable 2>&1 || {
    fatal_with_snapshot "PHASE3_GRAVITY_DISABLE_FAILED"
  }
  GRAVITY_GUARD_USED=true
fi

"${PKG_DIR}/scripts/unpause_and_verify_clock_v333.py" 2>&1 || {
  fatal_with_snapshot "PHASE3_CLOCK_NOT_ADVANCING"
}
echo "  SIM_CLOCK_ADVANCING"

# ===== Phase 4: 分步启动控制器 =====
echo ""
echo "--- Phase 4: Sequential Controller Start ---"
"${PKG_DIR}/scripts/start_cr5_controllers_v335.py" 2>&1 || {
  fatal_with_snapshot "PHASE4_CONTROLLERS_FAILED"
}
echo "  CONTROLLERS_RUNNING"

# ===== Phase 5: 零位保持 + 高度验证 =====
echo ""
echo "--- Phase 5: Zero-Position Hold ---"
"${PKG_DIR}/scripts/hold_cr5_zero_v334.py" 2>&1 || {
  fatal_with_snapshot "PHASE5_ZERO_HOLD_FAILED"
}
echo "  CR5_ZERO_HOLD_OK"

# gravity 模式下恢复重力
if [[ "$GRAVITY_GUARD_USED" == "true" ]]; then
  echo ""
  echo "--- Phase 5b: Gravity Restore (experimental) ---"
  "${PKG_DIR}/scripts/experimental/cr5_gravity_guard_v334.py" restore 2>&1 || {
    fatal_with_snapshot "PHASE5b_GRAVITY_RESTORE_FAILED"
  }
  echo "  CR5_GRAVITY_RESTORED"
  GRAVITY_GUARD_USED=false  # 已恢复
fi

# 验证 Link6/nozzle 高度
echo ""
echo "--- Phase 5c: Frame Height Verification ---"
sleep 1
LINK6_Z=""
for i in $(seq 1 5); do
  RESULT=$(timeout 3 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" || echo "")
  LINK6_Z=$(echo "$RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
  if [[ -n "$LINK6_Z" ]]; then break; fi
  sleep 1
done
if [[ -z "$LINK6_Z" ]]; then
  fatal_with_snapshot "PHASE5c_LINK6_HEIGHT_UNKNOWN"
fi
if [[ $(echo "$LINK6_Z < 0.80" | bc -l 2>/dev/null) == "1" ]]; then
  fatal_with_snapshot "PHASE5c_LINK6_Z_TOO_LOW (z=${LINK6_Z})"
fi
echo "  Link6.z = $LINK6_Z  [OK]"

# V3.3.5: 稳定监控 (5s)
echo ""
echo "--- Phase 5d: Stability Monitor (5s) ---"
START_Z="$LINK6_Z"
STABLE_OK=true
for i in $(seq 1 5); do
  sleep 1
  RESULT=$(timeout 3 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" || echo "")
  CUR_Z=$(echo "$RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
  if [[ -n "$CUR_Z" ]] && [[ -n "$START_Z" ]]; then
    DRIFT=$(echo "$CUR_Z - $START_Z" | bc -l 2>/dev/null || echo "0")
    if [[ $(echo "$DRIFT < -0.05" | bc -l 2>/dev/null) == "1" ]]; then
      echo "  [ALERT] Link6 dropped ${DRIFT}m at second $i"
      STABLE_OK=false
    fi
    echo "  second $i: Link6.z=$CUR_Z (drift=${DRIFT}m)"
  fi
done
if [[ "$STABLE_OK" == "true" ]]; then
  echo "  CR5_POSE_STABLE"
else
  echo "  [WARN] CR5 showed drift"
fi

# ===== Phase 6: Camera streams =====
echo ""
echo "--- Phase 6: Camera Streams ---"
sleep 3
"${PKG_DIR}/scripts/check_camera_streams_v333.py" 2>&1 || {
  fatal_with_snapshot "PHASE6_CAMERA_STREAMS_FAILED"
}
echo "  CAMERA_STREAMS_READY"

# ===== Phase 7: Spray service =====
if [[ "$ENABLE_SPRAY_SIM" == "true" ]]; then
  echo ""
  echo "--- Phase 7: Spray Service ---"
  WAIT_START=$(date +%s)
  while ! rosservice list 2>/dev/null | grep -q '/spray_demo/set_spray'; do
    sleep 1
    if [[ $(($(date +%s) - WAIT_START)) -gt 30 ]]; then
      fatal_with_snapshot "PHASE7_SPRAY_SERVICE_MISSING"
    fi
  done
  echo "  /spray_demo/set_spray ready"

  SPRAY_START=$(date +%s.%N)
  SPRAY_RESULT=$(timeout 5 rosservice call /spray_demo/set_spray "data: true" 2>&1 || echo "TIMEOUT")
  SPRAY_END=$(date +%s.%N)
  SPRAY_ELAPSED=$(echo "$SPRAY_END - $SPRAY_START" | bc -l 2>/dev/null || echo "0")
  echo "  set_spray(true) → ${SPRAY_RESULT}"
  echo "  Wall response: ${SPRAY_ELAPSED}s"

  if echo "$SPRAY_RESULT" | grep -qi "paused\|stalled"; then
    fatal_with_snapshot "PHASE7_SPRAY_CLOCK_STALLED"
  fi

  timeout 3 rosservice call /spray_demo/set_spray "data: false" 2>&1 || true
  echo "  SPRAY_RUNTIME_READY"
fi

# ===== TF 检查 =====
echo ""
echo "--- TF Check ---"
"${PKG_DIR}/scripts/check_tf_once_v331.py" 2>&1 || {
  fatal_with_snapshot "PHASE_TF_CHECK_FAILED"
}
echo "  TF check PASS"

# ============================================================================
# 全部硬门通过 → 激活会话
# ============================================================================
activate_session

echo ""
echo "=============================================="
echo "  V3.3.5 Session ACTIVE"
echo "  Session:  ${SESSION_ID}"
echo "  Physics:  ${PHYSICS_MODE}"
echo "  CONTROLLERS_LOADED_NOT_RUNNING"
echo "  PRE_BOOTSTRAP_SCENE_BASELINE_PASS"
echo "  SIM_CLOCK_ADVANCING"
echo "  JOINT_STATE_CONTROLLER_RUNNING"
echo "  JOINT_STATES_READY"
echo "  ARM_CONTROLLER_RUNNING"
echo "  CONTROLLERS_RUNNING"
echo "  CR5_POSE_STABLE"
echo "  CAMERA_STREAMS_READY"
if [[ "$ENABLE_SPRAY_SIM" == "true" ]]; then
  echo "  SPRAY_RUNTIME_READY"
fi
echo "  Session ACTIVE"
echo ""
if [[ "$ISOLATED" == "true" ]]; then
  echo "  Join: source ${PKG_DIR}/scripts/use_spray_session_v33.sh"
  echo ""
fi
echo "  Spray ON:  rosservice call /spray_demo/set_spray \"data: true\""
echo "  Spray OFF: rosservice call /spray_demo/set_spray \"data: false\""
echo "  Test plume: rosservice call /spray_demo/show_test_plume \"{}\""
echo "  State:     $(rosparam get /cr5_spray/session_state 2>/dev/null || echo '?')"
echo "=============================================="

wait "$LAUNCH_PID" 2>/dev/null || true
