#!/bin/bash
# D455-like RGB-D Simulator Smoke Test Runner
set -e

# Cleanup function
cleanup() {
    echo "=== Cleaning up ==="
    rosnode kill -a 2>/dev/null
    pkill -9 -f "gzserver\|gzclient\|rosmaster" 2>/dev/null || true
    sleep 1
}
cleanup

source /opt/ros/noetic/setup.bash
source /home/ydkj/cr5_ros1_ws/devel/setup.bash

# Critical: set GAZEBO_PLUGIN_PATH so Gazebo can find our plugin
export GAZEBO_PLUGIN_PATH=/home/ydkj/cr5_ros1_ws/devel/lib:/opt/ros/noetic/lib
export GAZEBO_MODEL_PATH=/home/ydkj/cr5_ros1_ws/src/realsense_gazebo_plugin/models:$GAZEBO_MODEL_PATH

echo "GAZEBO_PLUGIN_PATH=$GAZEBO_PLUGIN_PATH"
echo "=== Phase 1: Generate URDF ==="
xacro /home/ydkj/cr5_ros1_ws/src/realsense_gazebo_description/urdf/d455_like_test.urdf.xacro \
  > /tmp/d455_smoke.urdf 2>/dev/null
echo "URDF generated: $(wc -c < /tmp/d455_smoke.urdf) bytes"

echo "=== Phase 2: Start roscore ==="
roscore &
ROSCORE_PID=$!
sleep 3

rosparam set /use_sim_time true
rosparam set /robot_description -t /tmp/d455_smoke.urdf

echo "=== Phase 3: Start Gazebo headless ==="
rosrun gazebo_ros gzserver empty.world __name:=smoke_gzserver &
GZ_PID=$!
echo "gzserver PID=$GZ_PID"
sleep 15

echo "=== Phase 4: Spawn camera model ==="
rosrun gazebo_ros spawn_model -urdf -param robot_description -model smoke_camera -x 0 -y 0 -z 0 2>&1
sleep 10

echo "=== Phase 5: Check topics ==="
rostopic list 2>/dev/null | grep -i "camera\|smoke" || echo "NO CAMERA TOPICS FOUND"

echo "=== Phase 6: Check model ==="
rostopic echo -n 1 /gazebo/model_states 2>/dev/null | python3 -c "
import sys, yaml
data = yaml.safe_load(sys.stdin.read())
if data:
    for i, name in enumerate(data.get('name', [])):
        print(f'  Model: {name}')
" 2>/dev/null || echo "Cannot check models"

echo "=== Phase 7: Run smoke test ==="
if rostopic list 2>/dev/null | grep -q "camera/color/image_raw"; then
    timeout 20 python3 /home/ydkj/cr5_ros1_ws/src/realsense_gazebo_description/scripts/d455_like_smoke_test.py 2>&1
    EXIT=$?
else
    echo "SKIP: No camera topics available"
    EXIT=1
fi

echo "=== Phase 8: Cleanup ==="
kill $GZ_PID 2>/dev/null || true
pkill -9 -f "gzserver" 2>/dev/null || true
kill $ROSCORE_PID 2>/dev/null || true

echo "Smoke test exit code: ${EXIT:-1}"
exit ${EXIT:-1}
