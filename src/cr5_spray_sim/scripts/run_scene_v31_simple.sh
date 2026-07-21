#!/bin/bash
# CR5 Spray Demo V3.1 Launcher
# 简单吊件 + 两柱门架 + 三相机直接坐标
# Usage: run_scene_v31_simple.sh [--gui] [--headless] [--isolated]
#         [--profile vm|quality] [--object motor_housing_cylinder|rectangular_housing]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../../.." && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

GUI=false
HEADLESS=false
ISOLATED=false
PROFILE="vm"
OBJECT="motor_housing_cylinder"

while [[ $# -gt 0 ]]; do
    case $1 in
        --gui) GUI=true; shift ;;
        --headless) HEADLESS=true; shift ;;
        --isolated) ISOLATED=true; shift ;;
        --profile) PROFILE="$2"; shift 2 ;;
        --object) OBJECT="$2"; shift 2 ;;
        *) shift ;;
    esac
done

if ! $GUI && ! $HEADLESS; then
    HEADLESS=true  # default
fi

if $ISOLATED; then
    ROS_PORT=$(python3 -c "import socket; s=socket.socket(); s.bind(('',0)); print(s.getsockname()[1]); s.close()")
    GZ_PORT=$(python3 -c "import socket; s=socket.socket(); s.bind(('',0)); print(s.getsockname()[1]); s.close()")
    LOG_DIR="/tmp/cr5_v31_simple_${TIMESTAMP}"
    mkdir -p "$LOG_DIR"
    export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
    export GAZEBO_MASTER_URI="http://localhost:${GZ_PORT}"
    export ROS_LOG_DIR="$LOG_DIR"
    echo "=== Isolated Session ==="
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
    echo "ROS_LOG_DIR=$LOG_DIR"
else
    LOG_DIR="/tmp/cr5_v31_simple_${TIMESTAMP}"
    mkdir -p "$LOG_DIR"
fi

echo "Branch: $(git -C "$WORKSPACE" branch --show-current 2>/dev/null || echo unknown)"
echo "HEAD: $(git -C "$WORKSPACE" rev-parse --short HEAD 2>/dev/null || echo unknown)"
echo "Profile: $PROFILE  Object: $OBJECT"
echo "GUI: $GUI  Headless: $HEADLESS"

# Source ROS
source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"

# Preflight: skip in isolated mode (random ports)
if $ISOLATED; then
    echo "=== Preflight: skipped (isolated mode) ==="
else
    echo "=== Preflight ==="
    python3 "$SCRIPT_DIR/scene_v2_preflight.py" || {
        echo "Preflight failed. Aborting."
        exit 1
    }
fi

# Model whitelist check
echo "=== Model Whitelist ==="
echo "Allowed models: ground_plane, cr5_robot, simple_goalpost_frame,"
echo "                simple_hanging_workpiece, pedestal_fl, pedestal_fr,"
echo "                pedestal_rear, cam_front_left, cam_front_right, cam_rear"
echo "Forbidden models: u_cell_frame, rotary_hanger, spray_workpiece,"
echo "                  automotive_fender_panel, bumper_corner_panel"

# Launch args
LAUNCH_ARGS="gui:=${GUI} headless:=${HEADLESS} camera_profile:=${PROFILE} object_type:=${OBJECT}"

echo "=== Launching scene_v31_simple.launch ==="
echo "Log: $LOG_DIR/roslaunch.log"

roslaunch cr5_spray_sim scene_v31_simple.launch $LAUNCH_ARGS > "$LOG_DIR/roslaunch.log" 2>&1 &
LAUNCH_PID=$!
echo "Launch PID=$LAUNCH_PID"
echo "$LAUNCH_PID" > "$LOG_DIR/launch.pid"

# Trap cleanup
cleanup() {
    echo "=== Cleaning up session ==="
    kill $LAUNCH_PID 2>/dev/null || true
    for pid in $(ps -eo pid,ppid,cmd | awk -v ppid=$LAUNCH_PID '$2==ppid {print $1}'); do
        kill $pid 2>/dev/null || true
    done
    sleep 2
    echo "Session cleaned. Logs: $LOG_DIR"
}
trap cleanup EXIT INT TERM

wait $LAUNCH_PID 2>/dev/null || true
