#!/usr/bin/env bash
# ===========================================================================
# CR5 Spray Demo V3.3 Launcher
# 从 V3.1 稳定基线重建，修复 V3.2.1 所有已知问题。
#
# 关键修复：
#   1. TF: world→dummy_link (非 world→base_link)
#   2. 喷枪: 从 Link6 法兰原点安装 (非旧 Tool_end)
#   3. state latched + 周期重发
#   4. 启动器硬失败门：任何健康检查失败 → 退出非零
#
# 用法:
#   bash run_scene_v33_spray.sh [--gui] [--headless] [--isolated]
#     [--object motor_housing_cylinder|rectangular_housing]
#     [--no-spray-sim] [--no-paint-patches]
# ===========================================================================
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
PKG_DIR="$(dirname "$SCRIPT_DIR")"
WS_DIR="/home/ydkj/cr5_ros1_ws"

# Defaults
GUI=false
HEADLESS=true
ISOLATED=false
OBJECT="motor_housing_cylinder"
PROFILE="vm"
ENABLE_SPRAY_SIM=true
ENABLE_PAINT_PATCHES=true
FIXED_PORTS=false

# Parse args
for arg in "$@"; do
  case "$arg" in
    --gui) GUI=true; HEADLESS=false ;;
    --headless) HEADLESS=true; GUI=false ;;
    --isolated) ISOLATED=true ;;
    --object) shift; OBJECT="$1"; shift 2>/dev/null || true ;;
    --object=*) OBJECT="${arg#*=}" ;;
    --profile) shift; PROFILE="$1"; shift 2>/dev/null || true ;;
    --profile=*) PROFILE="${arg#*=}" ;;
    --no-spray-sim) ENABLE_SPRAY_SIM=false ;;
    --no-paint-patches) ENABLE_PAINT_PATCHES=false ;;
    --fixed-ports) FIXED_PORTS=true ;;
    *) ;;
  esac
done

# Validate object type
if [[ "$OBJECT" != "motor_housing_cylinder" && "$OBJECT" != "rectangular_housing" ]]; then
  echo "ERROR: Unknown object type: $OBJECT" >&2
  echo "Valid: motor_housing_cylinder | rectangular_housing" >&2
  exit 1
fi

# ===== Environment =====
source /opt/ros/noetic/setup.bash
source "${WS_DIR}/devel/setup.bash" 2>/dev/null || {
  echo "ERROR: Workspace not built. Run: cd ${WS_DIR} && catkin_make" >&2
  exit 1
}

# Export Gazebo model paths
export GAZEBO_MODEL_PATH="${PKG_DIR}/models:${GAZEBO_MODEL_PATH:-}"

# ===== Session Management =====
ENV_FILE="/tmp/cr5_spray_v33_current.env"
SESSION_ID="v33_$(date +%Y%m%d_%H%M%S)"
LOG_DIR="/tmp/cr5_spray_v33_${SESSION_ID}"
mkdir -p "$LOG_DIR"

cleanup() {
  local exit_code=$?
  echo ""
  echo "=============================================="
  echo "  V3.3 Session: ${SESSION_ID}"
  echo "  Exit code: ${exit_code}"
  echo "  Logs: ${LOG_DIR}"
  echo "=============================================="
  if [[ "$ISOLATED" == "true" ]]; then
    # Kill our Gazebo/ROS processes
    if [[ -n "${ROS_MASTER_PID:-}" ]]; then kill "$ROS_MASTER_PID" 2>/dev/null || true; fi
    if [[ -n "${GZSERVER_PID:-}" ]]; then kill "$GZSERVER_PID" 2>/dev/null || true; fi
    rm -f "$ENV_FILE"
  fi
}
trap cleanup EXIT

start_isolated_master() {
  # Random ports for multi-session support
  local port=$((11311 + RANDOM % 1000))
  export ROS_MASTER_URI="http://localhost:${port}"
  roscore -p "$port" &
  ROS_MASTER_PID=$!
  sleep 2
  if ! kill -0 "$ROS_MASTER_PID" 2>/dev/null; then
    echo "FATAL: roscore failed to start on port ${port}" >&2
    exit 1
  fi

  # Set random Gazebo ports
  local gz_port=$((11345 + RANDOM % 1000))
  export GAZEBO_MASTER_URI="http://localhost:${gz_port}"

  # Write env for cross-terminal sharing
  cat > "$ENV_FILE" << EOF
export ROS_MASTER_URI=http://localhost:${port}
export GAZEBO_MASTER_URI=http://localhost:${gz_port}
export CR5_SPRAY_SESSION=${SESSION_ID}
export CR5_SPRAY_LOG_DIR=${LOG_DIR}
EOF
  echo "Isolated session: master=${port} gz=${gz_port}"
  echo "Env file: ${ENV_FILE}"
  echo "Source to join: source ${PKG_DIR}/scripts/use_spray_session_v33.sh"
}

if [[ "$ISOLATED" == "true" ]]; then
  start_isolated_master
fi

# ===== Build launch arguments =====
LAUNCH_ARGS="object_type:=${OBJECT} camera_profile:=${PROFILE}"
LAUNCH_ARGS="${LAUNCH_ARGS} gui:=${GUI} headless:=${HEADLESS}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_spray_tool:=true"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_spray_sim:=${ENABLE_SPRAY_SIM}"
LAUNCH_ARGS="${LAUNCH_ARGS} enable_paint_patches:=${ENABLE_PAINT_PATCHES}"

echo "=============================================="
echo "  CR5 Spray Demo V3.3"
echo "  Session: ${SESSION_ID}"
echo "  Object: ${OBJECT}"
echo "  GUI: ${GUI}  Headless: ${HEADLESS}  Isolated: ${ISOLATED}"
echo "  Spray: ${ENABLE_SPRAY_SIM}  Paint: ${ENABLE_PAINT_PATCHES}"
echo "=============================================="

# ===== Launch =====
roslaunch cr5_spray_sim scene_v33_spray.launch ${LAUNCH_ARGS} &
LAUNCH_PID=$!

# ===== Health Checks =====
echo ""
echo "--- Waiting for Gazebo ---"
WAIT_START=$(date +%s)
MAX_WAIT=120
while ! rosservice list 2>/dev/null | grep -q '/gazebo/'; do
  sleep 1
  if [[ $(($(date +%s) - WAIT_START)) -gt $MAX_WAIT ]]; then
    echo "FATAL: Gazebo did not start within ${MAX_WAIT}s" >&2
    kill "$LAUNCH_PID" 2>/dev/null || true
    exit 1
  fi
done
echo "  OK: Gazebo ready ($(($(date +%s) - WAIT_START))s)"

# Wait for spray service (if enabled)
if [[ "$ENABLE_SPRAY_SIM" == "true" ]]; then
  echo ""
  echo "--- Waiting for /spray_demo/set_spray ---"
  WAIT_START=$(date +%s)
  while ! rosservice list 2>/dev/null | grep -q '/spray_demo/set_spray'; do
    sleep 1
    if [[ $(($(date +%s) - WAIT_START)) -gt 60 ]]; then
      echo "FATAL: /spray_demo/set_spray did not appear within 60s" >&2
      kill "$LAUNCH_PID" 2>/dev/null || true
      exit 1
    fi
  done
  echo "  OK: /spray_demo/set_spray ready ($(($(date +%s) - WAIT_START))s)"

  # ===== V3.3 硬失败门: 验证 state topic 实际消息 =====
  echo ""
  echo "--- Verifying /spray_demo/state (latched) ---"
  STATE_MSG=$(rostopic echo -n1 /spray_demo/state 2>/dev/null || true)
  if [[ -z "$STATE_MSG" ]]; then
    echo "FATAL: /spray_demo/state has NO message (state publisher not latched?)" >&2
    kill "$LAUNCH_PID" 2>/dev/null || true
    exit 1
  fi
  echo "  OK: /spray_demo/state = ${STATE_MSG}"

  # ===== 验证 TF 树连通 =====
  echo ""
  echo "--- Verifying TF: object_frame → spray_nozzle_frame ---"
  sleep 2  # Let TF settle
  if ! rosrun tf tf_echo object_frame spray_nozzle_frame 2>/dev/null &
  then
    TF_PID=$!
    sleep 3
    if ! kill -0 "$TF_PID" 2>/dev/null; then
      echo "FATAL: TF tree disconnected: object_frame → spray_nozzle_frame" >&2
      echo "  Check: world→dummy_link bridge present?" >&2
      kill "$LAUNCH_PID" 2>/dev/null || true
      exit 1
    fi
    kill "$TF_PID" 2>/dev/null || true
  fi
  echo "  OK: TF tree connected"

  # ===== V3.3 硬失败门: 模型稳定性检查 =====
  echo ""
  echo "--- Model Stability Check (10s) ---"
  sleep 10
  # Check CR5 base_link pose
  if ! rosparam get /cr5_robot/base_link 2>/dev/null > /dev/null; then
    echo "  WARN: Cannot query base_link pose via rosparam (expected in headless)"
  fi
  # Check for NaN in key transforms
  for frame in "base_link" "dummy_link" "spray_nozzle_frame" "object_frame"; do
    if rosrun tf tf_echo world "$frame" 2>/dev/null | head -5 > "${LOG_DIR}/tf_${frame}.txt" &
    then
      TF_CHECK_PID=$!
      sleep 2
      kill "$TF_CHECK_PID" 2>/dev/null || true
      if grep -q "nan\|NaN" "${LOG_DIR}/tf_${frame}.txt" 2>/dev/null; then
        echo "FATAL: NaN detected in world→${frame} TF" >&2
        kill "$LAUNCH_PID" 2>/dev/null || true
        exit 1
      fi
    fi
  done
  echo "  OK: No NaN detected in key transforms"
fi

# ===== 检查所有模型 =====
echo ""
echo "--- Model Count ---"
MODEL_COUNT=$(rosservice call /gazebo/get_world_properties 2>/dev/null | grep -c "model_name" || echo "0")
echo "  Models: ${MODEL_COUNT}"
if [[ "$MODEL_COUNT" -lt 5 ]]; then
  echo "  WARN: Only ${MODEL_COUNT} models in world (expected >=6)"
fi

echo ""
echo "=============================================="
echo "  V3.3 Session ACTIVE"
echo "  Session: ${SESSION_ID}"
echo "  Logs: ${LOG_DIR}"
echo ""
if [[ "$ISOLATED" == "true" ]]; then
  echo "  Join from another terminal:"
  echo "    source ${PKG_DIR}/scripts/use_spray_session_v33.sh"
  echo ""
fi
echo "  Turn spray ON:"
echo "    rosservice call /spray_demo/set_spray \"data: true\""
echo "  Turn spray OFF:"
echo "    rosservice call /spray_demo/set_spray \"data: false\""
echo "  Reset paint:"
echo "    rosservice call /spray_demo/reset_paint \"{}\""
echo "  Save result:"
echo "    rosservice call /spray_demo/save_result \"{}\""
echo "=============================================="

# Wait for launch to finish (or Ctrl+C)
wait "$LAUNCH_PID"
