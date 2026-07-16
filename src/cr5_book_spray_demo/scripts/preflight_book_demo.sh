#!/usr/bin/env bash
set -u

fail=0
ok() { printf '[OK]   %s\n' "$*"; }
bad() { printf '[FAIL] %s\n' "$*"; fail=1; }
warn() { printf '[WARN] %s\n' "$*"; }

need_cmd() {
  command -v "$1" >/dev/null 2>&1 && ok "command $1" || bad "missing command $1"
}

need_topic() {
  rostopic list 2>/dev/null | grep -Fx "$1" >/dev/null && ok "topic $1" || bad "missing topic $1"
}

need_service() {
  rosservice list 2>/dev/null | grep -Fx "$1" >/dev/null && ok "service $1" || bad "missing service $1"
}

need_node() {
  rosnode list 2>/dev/null | grep -Fx "$1" >/dev/null && ok "node $1" || bad "missing node $1"
}

need_cmd rostopic
need_cmd rosservice
need_cmd rosrun

need_topic /camera/color/image_raw
need_topic /camera/aligned_depth_to_color/image_raw
need_topic /camera/color/camera_info
need_topic /joint_states
need_topic /book_demo/estimator/debug_image
need_topic /book_demo/estimator/locked_pose

need_service /book_demo/estimator/lock_target
need_service /book_demo/estimator/clear_target
need_service /book_demo/planner/plan_path
need_service /book_demo/planner/execute_path

need_node /move_group
need_node /book_demo/estimator
need_node /book_demo/planner

# TF 检查：允许 timeout 返回 124，检查输出中是否有有效 TF
TF_OUTPUT_FILE="/tmp/book_tf_check.txt"
TF_EXIT_CODE=0
timeout 4 rosrun tf tf_echo base_link camera_color_optical_frame >"$TF_OUTPUT_FILE" 2>&1 || TF_EXIT_CODE=$?

if [ "$TF_EXIT_CODE" -eq 124 ] || grep -qE "^(Translation|At time)" "$TF_OUTPUT_FILE" 2>/dev/null; then
  ok 'TF base_link -> camera_color_optical_frame'
else
  bad 'TF base_link -> camera_color_optical_frame unavailable'
  sed -n '1,20p' "$TF_OUTPUT_FILE" 2>/dev/null || true
fi

if timeout 4 rostopic echo -n 1 /camera/aligned_depth_to_color/image_raw/header >/tmp/book_depth_header.txt 2>&1; then
  ok 'aligned depth is publishing'
else
  bad 'aligned depth did not publish within timeout'
fi

if rostopic list 2>/dev/null | grep -Fx /dobot_bringup/msg/FeedInfo >/dev/null; then
  ok 'DOBOT FeedInfo topic present'
else
  warn 'DOBOT FeedInfo topic not found; vision-only testing can continue, execution cannot.'
fi

if ((fail)); then
  echo 'PREFLIGHT FAILED. Do not execute robot motion.'
  exit 1
fi

echo 'BOOK DEMO PREFLIGHT PASSED.'
