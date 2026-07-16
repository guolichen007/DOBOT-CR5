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
# 环境加载函数
# ============================================================
load_ros_environment() {
    source /opt/ros/noetic/setup.bash

    if [ ! -f "$REALSENSE_WS/devel/setup.bash" ]; then
        echo "[ERROR] RealSense workspace is not built:"
        echo "        $REALSENSE_WS"
        echo
        echo "Run:"
        echo "  bash $WS/scripts/laptop/setup_realsense_ros1.sh"
        exit 1
    fi

    source "$REALSENSE_WS/devel/setup.bash"

    if [ ! -f "$WS/devel/local_setup.bash" ]; then
        echo "[ERROR] CR5 workspace is not built:"
        echo "        $WS"
        exit 1
    fi

    source "$WS/devel/local_setup.bash"
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
load_ros_environment
catkin_make 2>&1 | tee "$LOG_FILE"

if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    fail "编译失败，日志: $LOG_FILE"
fi

source "$WS/devel/local_setup.bash"
rospack find cr5_book_spray_demo

# ============================================================
# 4. 输出结果
# ============================================================
echo ""
echo "=========================================="
echo "  编译完成"
echo "=========================================="
echo "Branch: $(git branch --show-current)"
echo "Commit: $(git rev-parse HEAD)"
echo "Build log: $LOG_FILE"
