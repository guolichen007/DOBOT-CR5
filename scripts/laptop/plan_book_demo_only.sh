#!/usr/bin/env bash
set -euo pipefail
WS="${WS:-$HOME/cr5_ros1_ws}"

source /opt/ros/noetic/setup.bash
source "$WS/devel/setup.bash"

echo "PLAN-ONLY MODE: allow_execution=false"

exec roslaunch cr5_book_spray_demo planner_only.launch   allow_execution:=false   path_mode:=single_stroke   orientation_mode:=keep_current   eef_link:=Tool_end
