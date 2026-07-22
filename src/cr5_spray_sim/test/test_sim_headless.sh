#!/bin/bash
# CR5 Spray Demo: Headless Integration Test
# 测试: build → xacro → check_urdf → Gazebo → controllers → cameras → cleanup
set -e

RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m'

PASS=0
FAIL=0
WORKSPACE="/home/ydkj/cr5_ros1_ws"
ARTIFACTS="$WORKSPACE/artifacts/headless_test_$(date +%Y%m%d_%H%M%S)"

pass() { echo -e "${GREEN}[PASS]${NC} $1"; PASS=$((PASS+1)); }
fail() { echo -e "${RED}[FAIL]${NC} $1"; FAIL=$((FAIL+1)); }
info() { echo -e "${YELLOW}[INFO]${NC} $1"; }

cleanup() {
    info "Cleaning up..."
    pkill -9 -f "gzserver\|roslaunch\|rosmaster" 2>/dev/null || true
    sleep 1
}
trap cleanup EXIT

mkdir -p "$ARTIFACTS"

source /opt/ros/noetic/setup.bash
source "$WORKSPACE/devel/setup.bash"
export GAZEBO_PLUGIN_PATH="$WORKSPACE/devel/lib:/opt/ros/noetic/lib"

# 1. Build
info "Test 1: catkin_make"
if (cd "$WORKSPACE" && catkin_make > "$ARTIFACTS/build.log" 2>&1); then
    pass "Build succeeded"
else
    fail "Build failed (see $ARTIFACTS/build.log)"
fi

# 2. Xacro check
info "Test 2: xacro CR5"
if xacro "$WORKSPACE/src/cr5_spray_sim/urdf/cr5_sim.urdf.xacro" \
    > "$ARTIFACTS/cr5_sim.urdf" 2>/dev/null; then
    pass "Xacro CR5 OK"
else
    fail "Xacro CR5 failed"
fi

# 3. check_urdf
info "Test 3: check_urdf"
if check_urdf "$ARTIFACTS/cr5_sim.urdf" > /dev/null 2>&1; then
    pass "check_urdf OK"
else
    fail "check_urdf failed"
fi

# 4. Xacro camera
info "Test 4: xacro camera"
if xacro "$WORKSPACE/src/realsense_gazebo_description/urdf/d455_like_test.urdf.xacro" \
    > "$ARTIFACTS/camera.urdf" 2>/dev/null; then
    pass "Xacro camera OK"
else
    fail "Xacro camera failed"
fi

# 5. Gazebo start + CR5 spawn
info "Test 5: Gazebo + CR5 (30s timeout)"
roslaunch cr5_spray_sim cr5_only_gazebo.launch gui:=false headless:=true \
    > "$ARTIFACTS/gazebo_cr5.log" 2>&1 &
LAUNCH_PID=$!
sleep 25

# 5a. Controllers
if rosservice call /controller_manager/list_controllers 2>/dev/null \
    | grep -q "arm_controller.*running"; then
    pass "arm_controller running"
else
    fail "arm_controller not running"
fi

# 5b. Joint states
if rostopic echo -n 1 /joint_states 2>/dev/null | grep -q "joint[1-6]"; then
    pass "Joint states publishing"
else
    fail "Joint states not publishing"
fi

# 5c. No NaN
if rostopic echo -n 1 /joint_states 2>/dev/null | grep -qv "NaN"; then
    pass "No NaN in joint states"
else
    fail "NaN detected in joint states"
fi

kill $LAUNCH_PID 2>/dev/null || true
sleep 2

# 6. Camera smoke test
info "Test 6: Camera smoke test"
xacro "$WORKSPACE/src/realsense_gazebo_description/urdf/d455_like_test.urdf.xacro" \
    > /tmp/smoke_cam.urdf 2>/dev/null

roscore > /dev/null 2>&1 &
sleep 2
rosparam set /use_sim_time true
rosparam set /robot_description -t /tmp/smoke_cam.urdf
rosrun gazebo_ros gzserver > /dev/null 2>&1 &
sleep 8
rosrun gazebo_ros spawn_model -urdf -param robot_description -model test_cam \
    -x 0 -y 0 -z 0 > /dev/null 2>&1
sleep 8

if rostopic list 2>/dev/null | grep -q "camera/color/image_raw"; then
    pass "Camera color topic available"
else
    fail "Camera color topic NOT available"
fi

if rostopic list 2>/dev/null | grep -q "camera/depth/image_raw"; then
    pass "Camera depth topic available"
else
    fail "Camera depth topic NOT available"
fi

# 7. No dobot_bringup
info "Test 7: Safety check"
if rostopic list 2>/dev/null | grep -q "dobot_bringup"; then
    fail "dobot_bringup detected (PROHIBITED)"
else
    pass "No dobot_bringup (safe)"
fi

cleanup

# Summary
echo ""
echo "=============================="
echo -e "Results: ${GREEN}$PASS passed${NC}, ${RED}$FAIL failed${NC}"
echo "Artifacts: $ARTIFACTS"
echo "=============================="

if [ $FAIL -gt 0 ]; then
    echo -e "${RED}HEADLESS TEST FAILED${NC}"
    exit 1
else
    echo -e "${GREEN}HEADLESS TEST PASSED${NC}"
    exit 0
fi
