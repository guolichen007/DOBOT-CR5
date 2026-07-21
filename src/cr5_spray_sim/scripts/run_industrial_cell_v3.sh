#!/bin/bash
# CR5 Industrial Spray Cell V3 Launcher
# Usage: run_industrial_cell_v3.sh [--gui] [--headless] [--isolated]
#         [--profile vm|quality] [--object automotive_fender_panel|...]
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "$0")" && pwd)"
WORKSPACE="$(cd "$SCRIPT_DIR/../../.." && pwd)"
TIMESTAMP=$(date +%Y%m%d_%H%M%S)

GUI=false
HEADLESS=false
ISOLATED=false
PROFILE="vm"
OBJECT="automotive_fender_panel"

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
    LOG_DIR="/tmp/cr5_industrial_v3_${TIMESTAMP}"
    mkdir -p "$LOG_DIR"
    export ROS_MASTER_URI="http://localhost:${ROS_PORT}"
    export GAZEBO_MASTER_URI="http://localhost:${GZ_PORT}"
    export ROS_LOG_DIR="$LOG_DIR"
    echo "=== Isolated Session ==="
    echo "ROS_MASTER_URI=$ROS_MASTER_URI"
    echo "GAZEBO_MASTER_URI=$GAZEBO_MASTER_URI"
    echo "ROS_LOG_DIR=$LOG_DIR"
else
    LOG_DIR="/tmp/cr5_industrial_v3_${TIMESTAMP}"
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

# Launch args
LAUNCH_ARGS="gui:=${GUI} headless:=${HEADLESS} camera_profile:=${PROFILE} object_type:=${OBJECT}"

echo "=== Launching scene_v3.launch ==="
echo "Log: $LOG_DIR/roslaunch.log"

roslaunch cr5_spray_sim scene_v3.launch $LAUNCH_ARGS > "$LOG_DIR/roslaunch.log" 2>&1 &
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
