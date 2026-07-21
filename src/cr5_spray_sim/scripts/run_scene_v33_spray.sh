#!/usr/bin/env bash
# ===========================================================================
# CR5 Spray Demo V3.3.1 Launcher
# 修复 V3.3 启动器的参数解析、进程泄漏、tf_echo 刷屏问题。
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
ENV_FILE="/tmp/cr5_spray_v33_current.env"
SESSION_ID="v331_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_v33_${SESSION_ID}"
mkdir -p "$LOG_DIR"
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

  # 1. 停止辅助进程
  if [[ -n "${RQT_PID:-}" ]]; then kill "$RQT_PID" 2>/dev/null || true; fi

  # 2. 按进程组停止 roslaunch (先 INT 优雅停止)
  if [[ -n "${LAUNCH_PGID:-}" ]]; then
    echo "  Stopping launch process group ${LAUNCH_PGID}..."
    kill -INT -- -${LAUNCH_PGID} 2>/dev/null || true
    # 等待最多 8 秒
    for i in $(seq 1 8); do
      if ! kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then break; fi
      sleep 1
    done
    # 还没退出则 TERM
    if kill -0 -- -${LAUNCH_PGID} 2>/dev/null; then
      echo "  TERM launch process group..."
      kill -TERM -- -${LAUNCH_PGID} 2>/dev/null || true
      sleep 2
    fi
    # 最后手段 KILL
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

  # 4. 停止 roscore (在 isolated 模式下)
  if [[ "$ISOLATED" == "true" ]] && [[ -n "${ROS_MASTER_PID:-}" ]]; then
    echo "  Stopping roscore (pid=${ROS_MASTER_PID})..."
    kill "$ROS_MASTER_PID" 2>/dev/null || true
    wait "$ROS_MASTER_PID" 2>/dev/null || true
  fi

  # 5. 删除 env
  if [[ "$ISOLATED" == "true" ]]; then
    rm -f "$ENV_FILE"
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

  # 原子创建 env 文件
  local tmp_env="${ENV_FILE}.tmp.$$"
  cat > "$tmp_env" << EOF
export ROS_MASTER_URI=http://localhost:${port}
export GAZEBO_MASTER_URI=http://localhost:${gz_port}
export CR5_SPRAY_SESSION=${SESSION_ID}
export CR5_SPRAY_LOG_DIR=${LOG_DIR}
export CR5_SPRAY_BRANCH=${BRANCH}
export CR5_SPRAY_HEAD=${HEAD_SHA}
EOF
  mv "$tmp_env" "$ENV_FILE"

  echo "ROS master ready (port=${port})"
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
# V3.3.2: paused start, controllers managed by initialize_cr5_pose_v332
LAUNCH_ARGS="${LAUNCH_ARGS} start_paused:=true start_controllers:=false"

echo ""
echo "=============================================="
echo "  CR5 Spray Demo V3.3.2"
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

# ===== V3.3.2: CR5 确定性初始姿态初始化 =====
# 策略: spawn_model -J 设初始关节角 + Gazebo paused 启动 → 无需 controller_manager switch
echo ""
echo "--- CR5 Initial Pose Init ---"

# 1. 等待 controller_manager 就绪
echo "  Waiting for controller_manager..."
WAIT_START=$(date +%s)
while ! rosservice list 2>/dev/null | grep -q '/controller_manager/list_controllers'; do
  sleep 1
  if [[ $(($(date +%s) - WAIT_START)) -gt 30 ]]; then
    echo "FATAL: controller_manager not available" >&2; exit 1
  fi
done

# 2. 双重保险: set_model_configuration (以防 -J 未生效)
echo "  Setting joints to upright_zero..."
rosservice call /gazebo/set_model_configuration \
  "model_name: 'cr5_robot'
urdf_param_name: 'robot_description'
joint_names: ['joint1','joint2','joint3','joint4','joint5','joint6']
joint_positions: [0.0, 0.0, 0.0, 0.0, 0.0, 0.0]" 2>/dev/null
sleep 1

# 3. 等待 joint_states 有数据
echo "  Waiting for joint_states..."
WAIT_START=$(date +%s)
JOINTS_OK=false
while [[ $(($(date +%s) - WAIT_START)) -lt 20 ]]; do
  JS=$(rostopic echo -n1 /joint_states 2>/dev/null)
  if echo "$JS" | grep -q "joint1"; then
    echo "$JS" | grep -E "name:|position:" | head -4
    JOINTS_OK=true; break
  fi
  sleep 1
done
if [[ "$JOINTS_OK" == "false" ]]; then
  echo "FATAL: no joint_states after 20s" >&2; exit 1
fi

# 4. 验证 Link6 高度 (仍在 paused)
sleep 2
echo "  Verifying Link6 height..."
LINK6_RESULT=$(timeout 5 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" || echo "")
echo "  $LINK6_RESULT"
# 提取 z 值
LINK6_Z=$(echo "$LINK6_RESULT" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "")
if [[ -z "$LINK6_Z" ]]; then
  echo "FATAL: Cannot determine Link6 height" >&2; exit 1
fi
if [[ $(echo "$LINK6_Z < 0.30" | bc -l 2>/dev/null) == "1" ]]; then
  echo "FATAL: CR5_ARM_FOLDED_BELOW_WORKSPACE (Link6.z=$LINK6_Z)" >&2; exit 1
fi

# 5. Unpause physics
echo "  Unpausing physics..."
rosservice call /gazebo/unpause_physics 2>/dev/null || true
sleep 3

# 6. 监控 5 秒
echo "  Monitoring stability (5s)..."
for i in $(seq 1 5); do
  sleep 1
  LINK6_Z=$(timeout 3 rosrun tf tf_echo world Link6 2>/dev/null | grep -m1 "Translation" | grep -oP '[-]?\d+\.\d+' | tail -1 || echo "0")
  if [[ $(echo "$LINK6_Z < 0.80" | bc -l 2>/dev/null) == "1" ]]; then
    echo "FATAL: Link6 dropped to z=$LINK6_Z during monitoring" >&2; exit 1
  fi
done
echo "  Link6.z after 5s = $LINK6_Z"

echo "  CR5_INITIAL_POSE_READY"

# ===== V3.3.2: 等待 Spray 服务 =====
if [[ "$ENABLE_SPRAY_SIM" == "true" ]]; then
  echo ""
  echo "--- Waiting for /spray_demo/set_spray ---"
  WAIT_START=$(date +%s)
  while ! rosservice list 2>/dev/null | grep -q '/spray_demo/set_spray'; do
    sleep 1
    if [[ $(($(date +%s) - WAIT_START)) -gt 60 ]]; then
      echo "FATAL: /spray_demo/set_spray did not appear" >&2
      exit 1
    fi
  done
  echo "  /spray_demo/set_spray ready ($(($(date +%s) - WAIT_START))s)"

  # ===== State 验证 =====
  echo ""
  echo "--- Verifying /spray_demo/state (latched) ---"
  STATE_MSG=$(rostopic echo -n1 /spray_demo/state 2>/dev/null || true)
  if [[ -z "$STATE_MSG" ]]; then
    echo "FATAL: /spray_demo/state has NO message" >&2
    exit 1
  fi
  echo "  /spray_demo/state = ${STATE_MSG}"
fi

# ===== TF 一次性检查 (使用 Python, 非 tf_echo) =====
echo ""
echo "--- TF Check ---"
"${PKG_DIR}/scripts/check_tf_once_v331.py" 2>&1 || {
  echo "FATAL: TF check failed" >&2
  exit 1
}
echo "  TF check PASS"

# ===== 模型稳定性检查 =====
echo ""
echo "--- Scene Health Check ---"
sleep 8  # 让物理稳定
"${PKG_DIR}/scripts/check_scene_v332.py" 2>&1 || {
  echo "FATAL: Scene health check failed" >&2
  exit 1
}
echo "  Scene health PASS"

# ===== 最终摘要 =====
echo ""
echo "=============================================="
echo "  V3.3.1 Session ACTIVE"
echo "  Session: ${SESSION_ID}"
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
echo "=============================================="

# 等待 launch 完成 (或 Ctrl+C)
wait "$LAUNCH_PID" 2>/dev/null || true
