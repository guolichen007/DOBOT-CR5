#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# pull_build_book_demo.sh
# 拉取最新代码并编译 CR5 书本识别 Demo
# ============================================================

BRANCH="${BRANCH:-feature/book-vision-spray-demo-v1}"
WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"
LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/book_demo_laptop_build_${STAMP}.log"

fail() { echo "[ERROR] $*" >&2; exit 1; }
info() { echo "[INFO] $*"; }

# ============================================================
# 环境加载函数（使用 --extend 叠加）
# ============================================================
load_ros_environment() {
    # 1. 加载 ROS 基础环境
    if [ ! -f "/opt/ros/noetic/setup.bash" ]; then
        fail "ROS Noetic 未安装"
    fi
    source /opt/ros/noetic/setup.bash

    # 2. 加载 RealSense 工作空间
    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        fail "RealSense 工作空间未编译: $REALSENSE_WS
请运行: bash $WS/scripts/laptop/setup_realsense_ros1.sh"
    fi
    source "$REALSENSE_WS/devel/setup.bash"

    # 3. 加载 CR5 工作空间（使用 --extend 叠加）
    if [ ! -f "$WS/devel/setup.bash" ]; then
        fail "CR5 工作空间未编译: $WS"
    fi
    source "$WS/devel/setup.bash" --extend
}

# ============================================================
# 1. 检查 Git 工作树
# ============================================================
[ -d "$WS/.git" ] || fail "$WS is not a Git worktree"
mkdir -p "$LOG_DIR"
cd "$WS"

info "当前 Git 状态："
git remote -v
git status --short --branch

[ -z "$(git status --porcelain)" ] || fail "Working tree is not clean."

# ============================================================
# 2. 拉取最新代码
# ============================================================
info "拉取最新代码..."
git fetch origin --prune
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
    git switch "$BRANCH"
else
    git switch --track -c "$BRANCH" "origin/$BRANCH"
fi
git pull --ff-only origin "$BRANCH"

info "当前版本："
git rev-parse HEAD
git log -1 --oneline --decorate

# ============================================================
# 3. 编译
# ============================================================
info "开始编译..."
source /opt/ros/noetic/setup.bash
catkin_make -DCMAKE_POLICY_VERSION_MINIMUM=3.5 2>&1 | tee "$LOG_FILE"

if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    fail "编译失败，日志: $LOG_FILE"
fi

# ============================================================
# 4. 加载环境并验证
# ============================================================
info "加载 ROS 环境..."
load_ros_environment

echo ""
echo "--- ROS 包验证 ---"
rospack find cr5_book_spray_demo || fail "cr5_book_spray_demo 未找到"
rospack find realsense2_camera || fail "realsense2_camera 未找到"

echo ""
echo "[INFO] ROS_PACKAGE_PATH:"
echo "$ROS_PACKAGE_PATH" | tr ':' '\n' | while read -r p; do
    [ -n "$p" ] && echo "  $p"
done

# ============================================================
# 5. 输出结果
# ============================================================
echo ""
echo "=========================================="
echo "  编译完成"
echo "=========================================="
echo "Branch: $(git branch --show-current)"
echo "SHA: $(git rev-parse HEAD)"
echo "CR5 Workspace: $WS"
echo "RealSense Workspace: $REALSENSE_WS"
echo "Build log: $LOG_FILE"
