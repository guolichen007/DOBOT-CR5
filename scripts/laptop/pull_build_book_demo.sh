#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# pull_build_book_demo.sh
# 拉取最新代码并编译 CR5 书本识别 Demo
# ============================================================

BRANCH="${BRANCH:-feature/book-vision-spray-demo-v1}"
SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
DEV_DIR="$(cd "$SCRIPT_DIR/../dev" && pwd)"

# 复用 common.sh
source "$DEV_DIR/common.sh"

STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/book_demo_laptop_build_${STAMP}.log"

# ============================================================
# 1. 检查 Git 工作树
# ============================================================
[ -d "$CR5_WS/.git" ] || { echo "[ERROR] $CR5_WS is not a Git worktree"; exit 1; }
cd "$CR5_WS"

echo "当前 Git 状态："
git remote -v
git status --short --branch

[ -z "$(git status --porcelain)" ] || { echo "[ERROR] Working tree is not clean"; exit 1; }

# ============================================================
# 2. 拉取最新代码
# ============================================================
echo "拉取最新代码..."
git fetch origin --prune
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    git switch "$BRANCH"
else
    git switch --track -c "$BRANCH" "origin/$BRANCH"
fi
git pull --ff-only origin "$BRANCH"

echo "当前版本："
git rev-parse HEAD
git log -1 --oneline --decorate

# ============================================================
# 3. 编译
# ============================================================
echo "开始编译..."
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 2>&1 | tee "$LOG_FILE"

if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    echo "[ERROR] 编译失败，日志: $LOG_FILE"
    exit 1
fi

# ============================================================
# 4. 加载环境并验证
# ============================================================
echo "加载 ROS 环境..."
load_cr5_environment

echo
echo "--- ROS 包验证 ---"
verify_ros_package cr5_book_spray_demo
verify_ros_package realsense2_camera

echo
echo "[INFO] ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
    [ -n "$p" ] && echo "  $p"
done

# ============================================================
# 5. 输出结果
# ============================================================
echo
echo "=========================================="
echo "  编译完成"
echo "=========================================="
echo "Branch: $(git -C "$CR5_WS" branch --show-current)"
echo "SHA: $(git -C "$CR5_WS" rev-parse HEAD)"
echo "CR5 Workspace: $CR5_WS"
echo "RealSense Workspace: $REALSENSE_WS"
echo "Build log: $LOG_FILE"
