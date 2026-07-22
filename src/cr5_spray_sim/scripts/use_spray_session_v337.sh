#!/usr/bin/env bash
# ===========================================================================
# V4: 接入当前活跃的 CR5 Spray V3.3.7 会话.
#
# 用法: source use_spray_session_v337.sh
# ===========================================================================
ENV_FILE="/tmp/cr5_spray_v337_current.env"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: No active CR5 Spray V3.3.7 session ($ENV_FILE missing)" >&2
  echo "Start first: bash run_scene_v337.sh --gui --object=<type>" >&2
  return 1 2>/dev/null || exit 1
fi

source "$ENV_FILE"

# 验证 workspace
if [[ ! -f "$CR5_SPRAY_WS/devel/setup.bash" ]]; then
  echo "ERROR: Workspace not found: $CR5_SPRAY_WS" >&2
  return 1 2>/dev/null || exit 1
fi
source "$CR5_SPRAY_WS/devel/setup.bash" 2>/dev/null

# 验证 package
if ! rospack find cr5_spray_sim &>/dev/null; then
  echo "ERROR: cr5_spray_sim package not found" >&2
  return 1 2>/dev/null || exit 1
fi

# 验证 master 可达
if ! rostopic list &>/dev/null; then
  echo "ERROR: ROS master at ${ROS_MASTER_URI} is not reachable" >&2
  echo "Session may have ended. Removing stale env file." >&2
  rm -f "$ENV_FILE"
  return 1 2>/dev/null || exit 1
fi

# 检查 session 状态
STATE=$(rosparam get /cr5_spray/session_state 2>/dev/null || echo "UNKNOWN")
if [[ "$STATE" != "ACTIVE" ]]; then
  echo "ERROR: Spray session exists but is not ACTIVE" >&2
  echo "  Current state: ${STATE}" >&2
  if [[ "$STATE" == "ENDED" ]]; then
    echo "  Session has ended. Start a new one." >&2
    rm -f "$ENV_FILE"
  elif [[ "$STATE" == "UNKNOWN" ]]; then
    echo "  Session state unknown." >&2
  fi
  return 1 2>/dev/null || exit 1
fi

echo "Connected to CR5 Spray V3.3.7 session: ${CR5_SPRAY_SESSION_ID:-unknown}"
echo "  Branch:    ${CR5_SPRAY_BRANCH:-unknown}  HEAD: ${CR5_SPRAY_HEAD:-unknown}"
echo "  State:     ${STATE}"
echo "  ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "  Logs:      ${CR5_SPRAY_LOG_DIR:-unknown}"
