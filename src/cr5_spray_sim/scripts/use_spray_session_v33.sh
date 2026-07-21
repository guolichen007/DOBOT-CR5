#!/usr/bin/env bash
# ===========================================================================
# V3.3: 接入当前活跃的 V3.3 喷涂会话。
# 用法: source use_spray_session_v33.sh
# ===========================================================================
ENV_FILE="/tmp/cr5_spray_v33_current.env"

if [[ ! -f "$ENV_FILE" ]]; then
  echo "ERROR: No active V3.3 spray session found ($ENV_FILE missing)" >&2
  echo "Start a session first: bash run_scene_v33_spray.sh --isolated" >&2
else
  source "$ENV_FILE"
  source /opt/ros/noetic/setup.bash
  source /home/ydkj/cr5_ros1_ws/devel/setup.bash 2>/dev/null
  echo "Connected to V3.3 spray session: ${CR5_SPRAY_SESSION:-unknown}"
  echo "  ROS_MASTER_URI=${ROS_MASTER_URI}"
  echo "  GAZEBO_MASTER_URI=${GAZEBO_MASTER_URI}"
fi
