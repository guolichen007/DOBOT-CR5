#!/usr/bin/env bash
# ===========================================================================
# V3.3.4: 接入当前活跃的 V3.3 喷涂会话.
#
# V3.3.4 新增: 检查 session_state 必须为 ACTIVE，
# 不允许连接到正在 BOOTSTRAPPING 或已 FAILED 的会话。
#
# 用法: source use_spray_session_v33.sh
# ===========================================================================
ENV_FILE="/tmp/cr5_spray_v33_current.env"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: No active V3.3 spray session ($ENV_FILE missing)" >&2
  echo "Start first: bash run_scene_v33_spray.sh --gui --isolated" >&2
  return 1 2>/dev/null || exit 1
fi

source "$ENV_FILE"
source /opt/ros/noetic/setup.bash
source /home/ydkj/cr5_ros1_ws/devel/setup.bash 2>/dev/null

# 验证 master 是否存活
if ! rostopic list &>/dev/null; then
  echo "ERROR: ROS master at ${ROS_MASTER_URI} is not reachable" >&2
  echo "Session may have ended. Removing stale env file." >&2
  rm -f "$ENV_FILE"
  return 1 2>/dev/null || exit 1
fi

# V3.3.4: 检查 session 状态
STATE=$(rosparam get /cr5_spray/session_state 2>/dev/null || echo "UNKNOWN")
if [[ "$STATE" != "ACTIVE" ]]; then
  echo "ERROR: Spray session exists but is not ACTIVE" >&2
  echo "  Current state: ${STATE}" >&2
  if [[ "$STATE" == "BOOTSTRAPPING" ]]; then
    echo "  Session is still initializing. Wait for 'Session ACTIVE' in launcher." >&2
  elif [[ "$STATE" == "ENDED" ]]; then
    echo "  Session has ended. Start a new one." >&2
    rm -f "$ENV_FILE"
  elif [[ "$STATE" == "UNKNOWN" ]]; then
    echo "  Session state unknown. The launcher may be an older version." >&2
  fi
  return 1 2>/dev/null || exit 1
fi

echo "Connected to V3.3 spray session: ${CR5_SPRAY_SESSION:-unknown}"
echo "  Branch:    ${CR5_SPRAY_BRANCH:-unknown}  HEAD: ${CR5_SPRAY_HEAD:-unknown}"
echo "  State:     ${STATE}"
echo "  ROS_MASTER_URI=${ROS_MASTER_URI}"
echo "  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
echo "  Logs:      ${CR5_SPRAY_LOG_DIR:-unknown}"
