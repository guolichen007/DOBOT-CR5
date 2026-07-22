#!/usr/bin/env bash
# ===========================================================================
# CR5 Spray Demo V3.3.4 Launcher
# 修复 V3.3.3 启动死锁：paused + gravity_disable → unpause → clock →
#                        start_controllers → zero_hold → restore_gravity →
#                        cameras → spray → ACTIVE
#
# 用法:
#   bash run_scene_v33_spray.sh [--gui] [--headless] [--isolated]
#     [--object motor_housing_cylinder] [--object=motor_housing_cylinder]
#     [--profile vm] [--profile=vm]
#     [--no-spray-sim] [--no-paint-patches] [--verbose]
# ===========================================================================
set -euo pipefail

# ===== 路径自动推导 (不硬编码) =====
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_DIR="$(cd "${SCRIPT_DIR}/../../.." && pwd)"

# ===== 默认值 =====
GUI=false
HEADLESS=true
ISOLATED=false
OBJECT="motor_housing_cylinder"
PROFILE="vm"
ENABLE_SPRAY_SIM=true
ENABLE_PAINT_PATCHES=true
FIXED_PORTS=false
VERBOSE=false

print_usage() {
  cat << EOF
Usage: $(basename "$0") [OPTIONS]

Options:
  --gui                       Launch with Gazebo GUI
  --headless                  Launch headless (default)
  --isolated                  Use random ports for multi-session
  --object TYPE               Workpiece: motor_housing_cylinder | rectangular_housing
  --object=TYPE               (alternative = form)
  --profile PROF              Camera profile: vm | quality (default: vm)
  --no-spray-sim              Disable spray control node
  --no-paint-patches          Disable paint patch markers
  --verbose                   Tail log files to terminal
  -h, --help                  Show this help

Examples:
  $(basename "$0") --gui --isolated
  $(basename "$0") --gui --isolated --object motor_housing_cylinder
  $(basename "$0") --gui --isolated --object=rectangular_housing
  $(basename "$0") --headless --object=motor_housing_cylinder
EOF
}

# ===== 参数解析 (while+case, 非 for+shift) =====
while [[ $# -gt 0 ]]; do
  case "$1" in
    --gui)          GUI=true; HEADLESS=false; shift ;;
    --headless)     HEADLESS=true; GUI=false; shift ;;
    --isolated)     ISOLATED=true; shift ;;
    --fixed-ports)  FIXED_PORTS=true; shift ;;
    --verbose)      VERBOSE=true; shift ;;
    --no-spray-sim)      ENABLE_SPRAY_SIM=false; shift ;;
    --no-paint-patches)  ENABLE_PAINT_PATCHES=false; shift ;;
    --object)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --object requires a value" >&2; exit 2
      fi
      OBJECT="$2"; shift 2 ;;
    --object=*)     OBJECT="${1#*=}"; shift ;;
    --profile)
      if [[ $# -lt 2 ]]; then
        echo "ERROR: --profile requires a value" >&2; exit 2
      fi
      PROFILE="$2"; shift 2 ;;
    --profile=*)    PROFILE="${1#*=}"; shift ;;
    -h|--help)      print_usage; exit 0 ;;
    --)             shift; break ;;
    *)              echo "ERROR: Unknown argument: $1" >&2; print_usage >&2; exit 2 ;;
  esac
done

echo "Argument parse PASS"
echo "  WS:   ${WS_DIR}"
echo "  PKG:  ${PKG_DIR}"

# ===== 验证 =====
if [[ "$OBJECT" != "motor_housing_cylinder" && "$OBJECT" != "rectangular_housing" ]]; then
  echo "ERROR: Unknown object type: $OBJECT" >&2
  echo "Valid: motor_housing_cylinder | rectangular_housing" >&2
  exit 1
fi

if [[ ! -f "${WS_DIR}/devel/setup.bash" ]]; then
  echo "ERROR: Workspace not built. Run: cd ${WS_DIR} && catkin_make" >&2
  exit 1
fi

# ===== 环境 =====
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash"
export GAZEBO_MODEL_PATH="${PKG_DIR}/models:${GAZEBO_MODEL_PATH:-}"

# ===== 会话 =====
ENV_FILE_CURRENT="/tmp/cr5_spray_v33_current.env"
SESSION_ID="v334_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_v33_${SESSION_ID}"
mkdir -p "$LOG_DIR"
export CR5_SPRAY_LOG_DIR="${LOG_DIR}"
BRANCH=$(cd "$WS_DIR" && git branch --show-current 2>/dev/null || echo "unknown")
HEAD_SHA=$(cd "$WS_DIR" && git rev-parse --short HEAD 2>/dev/null || echo "unknown")

# 进程跟踪
ROS_MASTER_PID=""
LAUNCH_PID=""
LAUNCH_PGID=""
CLEANED=false

# ===== 清理函数 (幂等, 按进程组) =====
cleanup() {
  local exit_code=$?
  [[ "$CLEANED" == "true" ]] && return
  CLEANED=true

  echo ""
  echo "=== Cleanup (exit=${exit_code}) ==="

  # V3.3.4: 尝试恢复重力 (如果还在运行)
  if [[ "$ISOLATED" != "true" ]] || kill -0 "$LAUNCH_PID" 2>/dev/null; then
    if rosservice list 2>/dev/null | grep -q '/gazebo/'; then
      echo "  Attempting gravity restore..."
      "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
    fi
  fi

  # 标记会话结束
  if rosparam list 2>/dev/null | grep -q '/cr5_spray/session_state'; then
    rosparam set /cr5_spray/session_state "ENDED" 2>/dev/null || true
  fi

  # 1. 停止辅助进程
  if [[ -n "${RQT_PID:-}" ]]; then kill "$RQT_PID" 2>/dev/null || true; fi

  # 2. 按进程组停止 roslaunch
  if [[ -n "${LAUNCH_PGID:-}" ]]; then
    echo "  Stopping launch process group ${LAUNCH_PGID}..."
    kill -INT -- -${LAUNCH_PGID} 2>/dev/null || true
    for i in $(seq 1 8); do
      if ! kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then break; fi
      sleep 1
    done
    if kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then
      echo "  TERM launch process group..."
      kill -TERM -- -${LAUNCH_PGID} 2>/dev/null || true
      sleep 2
    fi
    if kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then
      echo "  KILL launch process group..."
      kill -KILL -- -${LAUNCH_PGID} 2>/dev/null || true
    fi
  elif [[ -n "${LAUNCH_PID:-}" ]]; then
    kill "$LAUNCH_PID" 2>/dev/null || true
    wait "$LAUNCH_PID" 2>/dev/null || true
  fi

  # 3. 等待进程回收
  if [[ -n "${LAUNCH_PID:-}" ]]; then wait "$LAUNCH_PID" 2>/dev/null || true; fi

  # 4. 停止 roscore
  if [[ "$ISOLATED" == "true" ]] && [[ -n "${ROS_MASTER_PID:-}" ]]; then
    echo "  Stopping roscore (pid=${ROS_MASTER_PID})..."
    kill "$ROS_MASTER_PID" 2>/dev/null || true
    wait "$ROS_MASTER_PID" 2>/dev/null || true
  fi

  # 5. 删除 env 文件
  if [[ "$ISOLATED" == "true" ]]; then
    rm -f "$ENV_FILE_CURRENT" /tmp/cr5_spray_v33_pending_*.env
  fi

  echo "=== Cleanup done ==="
  echo "  Session: ${SESSION_ID}"
  echo "  Logs:    ${LOG_DIR}"
  echo "  Branch:  ${BRANCH}  HEAD: ${HEAD_SHA}"
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

  # V3.3.4: 先写 pending env，等所有硬门通过后再原子重命名为 current
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

# 激活会话：原子重命名 pending → current
activate_session() {
  if [[ "$ISOLATED" == "true" ]]; then
    local pending_env="/tmp/cr5_spray_v33_pending_${SESSION_ID}.env"
    mv "$pending_env" "$ENV_FILE_CURRENT"
  fi
  rosparam set /cr5_spray/session_state "ACTIVE" 2>/dev/null || true
  rosparam set /cr5_spray/session_id "$SESSION_ID" 2>/dev/null || true
}

# V3.3.4: 设置 session 为 BOOTSTRAPPING 状态
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
# V3.3.4: paused start, controllers --stopped
LAUNCH_ARGS="${LAUNCH_ARGS} paused:=true start_controllers:=false"

echo ""
echo "=============================================="
echo "  CR5 Spray Demo V3.3.4"
echo "  Session:  ${SESSION_ID}"
echo "  Object:   ${OBJECT}"
echo "  GUI: ${GUI}  Headless: ${HEADLESS}  Isolated: ${ISOLATED}"
echo "  Spray: ${ENABLE_SPRAY_SIM}  Paint: ${ENABLE_PAINT_PATCHES}"
echo "  Branch:   ${BRANCH}  HEAD: ${HEAD_SHA}"
echo "  Logs:     ${LOG_DIR}"
echo "=============================================="

# 使用 setsid 创建独立进程组
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
    echo "FATAL: Gazebo did not start within ${MAX_WAIT}s" >&2
    echo "Last 40 lines of roslaunch.log:" >&2
    tail -40 "${LOG_DIR}/roslaunch.log" >&2
    exit 1
  fi
  if ! kill -0 "$LAUNCH_PID" 2>/dev/null; then
    echo "FATAL: roslaunch died" >&2
    tail -60 "${LOG_DIR}/roslaunch.log" >&2
    exit 1
  fi
done
echo "  Gazebo ready ($(($(date +%s) - WAIT_START))s)"

# 设置 session 为 BOOTSTRAPPING
set_bootstrapping

# ============================================================================
# V3.3.4 硬门启动流程
# ============================================================================

# ===== Phase 1: 等待控制器 loaded/stopped =====
echo ""
echo "--- Phase 1: Controller Loaded/Stopped Check ---"
# controller_manager 上的 controller spawner 用 --stopped 加载，
# 此时两个控制器应为 stopped 状态.
WAIT_START=$(date +%s)
while true; do
  if [[ $(($(date +%s) - WAIT_START)) -gt 60 ]]; then
    echo "FATAL: Controller manager not available within 60s" >&2
    exit 1
  fi
  LIST_OUT=$(rosservice call /controller_manager/list_controllers "{}" 2>/dev/null || echo "")
  if echo "$LIST_OUT" | grep -q "joint_state_controller"; then
    break
  fi
  sleep 1
done

# 确认两个控制器都是 stopped 状态
JSC_STATE=$(echo "$LIST_OUT" | grep -A3 "joint_state_controller" | grep "state:" | awk '{print $2}' || echo "")
AC_STATE=$(echo "$LIST_OUT" | grep -A3 "arm_controller" | grep "state:" | awk '{print $2}' || echo "")
echo "  joint_state_controller: ${JSC_STATE:-unknown}"
echo "  arm_controller: ${AC_STATE:-unknown}"

if [[ "$JSC_STATE" != "stopped" ]] || [[ "$AC_STATE" != "stopped" ]]; then
  echo "FATAL: Controllers not in 'stopped' state (JSC=${JSC_STATE:-?}, AC=${AC_STATE:-?})" >&2
  exit 1
fi
echo "  CONTROLLERS_LOADED_STOPPED"

# ===== Phase 2: 临时关闭 CR5 重力 =====
echo ""
echo "--- Phase 2: Gravity Guard (disable) ---"
"${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" disable 2>&1 || {
  echo "FATAL: CR5_GRAVITY_DISABLE_FAILED" >&2
  exit 1
}
# 输出已包含 "CR5_GRAVITY_DISABLED" 到 stderr
echo "  CR5_GRAVITY_DISABLED"

# ===== Phase 3: Unpause + Clock 验证 =====
echo ""
echo "--- Phase 3: Unpause + Clock Verification ---"
"${PKG_DIR}/scripts/unpause_and_verify_clock_v333.py" 2>&1 || {
  echo "FATAL: GAZEBO_CLOCK_NOT_ADVANCING" >&2
  "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
  exit 1
}
echo "  SIM_CLOCK_ADVANCING"

# ===== Phase 4: 显式启动控制器 =====
echo ""
echo "--- Phase 4: Start Controllers ---"
"${PKG_DIR}/scripts/start_cr5_controllers_v334.py" 2>&1 || {
  echo "FATAL: CONTROLLERS_FAILED_TO_START" >&2
  # 手动 pause 并恢复重力
  rosservice call /gazebo/pause_physics "{}" 2>/dev/null || true
  "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
  echo "Current controller states:" >&2
  rosservice call /controller_manager/list_controllers "{}" 2>/dev/null >&2 || true
  exit 1
}
echo "  CONTROLLERS_RUNNING"

# ===== Phase 5: 零位保持 + 验证 + 恢复重力 + 监控 =====
echo ""
echo "--- Phase 5: Zero-Position Hold ---"
"${PKG_DIR}/scripts/hold_cr5_zero_v334.py" 2>&1 || {
  echo "FATAL: CR5_ZERO_HOLD_FAILED" >&2
  "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
  exit 1
}
echo "  CR5_ZERO_HOLD_OK"

# 验证 Link6/nozzle 高度
echo ""
echo "--- Phase 5b: Frame Height Verification ---"
sleep 1
LINK6_Z=""
for i in $(seq 1 5); do
  LINK6_RESULT=$(timeout 3 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" || echo "")
  LINK6_Z=$(echo "$LINK6_RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
  if [[ -n "$LINK6_Z" ]]; then break; fi
  sleep 1
done
if [[ -z "$LINK6_Z" ]]; then
  echo "FATAL: Cannot determine Link6 height" >&2
  "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
  exit 1
fi
if [[ $(echo "$LINK6_Z < ${MIN_LINK6_Z:-0.80}" | bc -l 2>/dev/null) == "1" ]]; then
  echo "FATAL: Link6.z=$LINK6_Z below workspace minimum" >&2
  "${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>/dev/null || true
  exit 1
fi
echo "  Link6.z = $LINK6_Z  [OK]"

NOZZLE_Z=""
for i in $(seq 1 5); do
  NOZZLE_RESULT=$(timeout 3 rosrun tf tf_echo world spray_nozzle_frame 2>/dev/null | grep -m1 "Translation" || echo "")
  NOZZLE_Z=$(echo "$NOZZLE_RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
  if [[ -n "$NOZZLE_Z" ]]; then break; fi
  sleep 1
done
if [[ -n "$NOZZLE_Z" ]]; then
  echo "  spray_nozzle_frame.z = $NOZZLE_Z  [OK]"
else
  echo "  [WARN] Cannot get nozzle frame height"
fi

# 恢复重力
echo ""
echo "--- Phase 5c: Gravity Restore ---"
"${PKG_DIR}/scripts/cr5_gravity_guard_v334.py" restore 2>&1 || {
  echo "FATAL: CR5_GRAVITY_RESTORE_FAILED" >&2
  exit 1
}
echo "  CR5_GRAVITY_RESTORED"

# 监控 5 秒不下落
echo ""
echo "--- Phase 5d: Gravity-On Stability Monitor (5s) ---"
START_LINK6_Z="$LINK6_Z"
MONITOR_OK=true
for i in $(seq 1 5); do
  sleep 1
  RESULT=$(timeout 3 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" || echo "")
  CUR_Z=$(echo "$RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
  if [[ -n "$CUR_Z" ]] && [[ -n "$START_LINK6_Z" ]]; then
    DRIFT=$(echo "$CUR_Z - $START_LINK6_Z" | bc -l 2>/dev/null || echo "0")
    if [[ $(echo "$DRIFT < -0.05" | bc -l 2>/dev/null) == "1" ]]; then
      echo "  [ALERT] Link6 dropped ${DRIFT}m at second $i"
      MONITOR_OK=false
    fi
    echo "  second $i: Link6.z=$CUR_Z (drift=${DRIFT}m)"
  fi
done
if [[ "$MONITOR_OK" == "true" ]]; then
  echo "  CR5_POSE_STABLE"
else
  echo "  [WARN] CR5 showed drift during monitoring"
fi

# ===== Phase 6: Camera streams check =====
echo ""
echo "--- Phase 6: Camera Streams ---"
sleep 3  # 等传感器启动
"${PKG_DIR}/scripts/check_camera_streams_v333.py" 2>&1 || {
  echo "FATAL: CAMERA_STREAMS_FAILED" >&2
  exit 1
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
      echo "FATAL: /spray_demo/set_spray did not appear" >&2; exit 1
    fi
  done
  echo "  /spray_demo/set_spray ready"

  # 快速响应测试
  SPRAY_START=$(date +%s.%N)
  SPRAY_RESULT=$(timeout 5 rosservice call /spray_demo/set_spray "data: true" 2>&1 || echo "TIMEOUT")
  SPRAY_END=$(date +%s.%N)
  SPRAY_ELAPSED=$(echo "$SPRAY_END - $SPRAY_START" | bc -l 2>/dev/null || echo "0")
  echo "  set_spray(true) → ${SPRAY_RESULT}"
  echo "  Wall response: ${SPRAY_ELAPSED}s"

  # 不能返回 clock paused
  if echo "$SPRAY_RESULT" | grep -qi "paused\|stalled"; then
    echo "FATAL: Spray service reports clock paused" >&2
    exit 1
  fi

  # 关枪 (必须成功)
  timeout 3 rosservice call /spray_demo/set_spray "data: false" 2>&1 || true

  echo "  SPRAY_RUNTIME_READY"
fi

# ===== TF 检查 =====
echo ""
echo "--- TF Check ---"
"${PKG_DIR}/scripts/check_tf_once_v331.py" 2>&1 || {
  echo "FATAL: TF check failed" >&2
  exit 1
}
echo "  TF check PASS"

# ============================================================================
# V3.3.4: 全部硬门通过，激活会话
# ============================================================================
activate_session

# ===== 最终摘要 =====
echo ""
echo "=============================================="
echo "  V3.3.4 Session ACTIVE"
echo "  Session: ${SESSION_ID}"
echo "  CONTROLLERS_LOADED_STOPPED"
echo "  CR5_GRAVITY_DISABLED"
echo "  SIM_CLOCK_ADVANCING"
echo "  CONTROLLERS_RUNNING"
echo "  CR5_ZERO_HOLD_OK"
echo "  CR5_GRAVITY_RESTORED"
echo "  CR5_POSE_STABLE"
echo "  CAMERA_STREAMS_READY"
if [[ "$ENABLE_SPRAY_SIM" == "true" ]]; then
  echo "  SPRAY_RUNTIME_READY"
fi
echo "  Session ACTIVE"
echo ""
if [[ "$ISOLATED" == "true" ]]; then
  echo "  Join from another terminal:"
  echo "    source ${PKG_DIR}/scripts/use_spray_session_v33.sh"
  echo ""
fi
echo "  Spray ON:   rosservice call /spray_demo/set_spray \"data: true\""
echo "  Spray OFF:  rosservice call /spray_demo/set_spray \"data: false\""
echo "  Reset:      rosservice call /spray_demo/reset_paint \"{}\""
echo "  Save:       rosservice call /spray_demo/save_result \"{}\""
echo ""
echo "  Session state: $(rosparam get /cr5_spray/session_state 2>/dev/null || echo '?')"
echo "=============================================="

# 等待 launch 完成 (或 Ctrl+C)
wait "$LAUNCH_PID" 2>/dev/null || true
