#!/usr/bin/env bash
set -euo pipefail
BRANCH="${BRANCH:-feature/book-vision-spray-demo-v1}"
WS="${WS:-$HOME/cr5_ros1_ws}"
LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/book_demo_laptop_build_${STAMP}.log"

fail(){ echo "[ERROR] $*" >&2; exit 1; }

[ -d "$WS/.git" ] || fail "$WS is not a Git worktree"
mkdir -p "$LOG_DIR"
cd "$WS"

git remote -v
git status --short --branch

[ -z "$(git status --porcelain)" ] || fail "Working tree is not clean."

git fetch origin --prune
if git show-ref --verify --quiet "refs/heads/$BRANCH"; then
  git switch "$BRANCH"
else
  git switch --track -c "$BRANCH" "origin/$BRANCH"
fi
git pull --ff-only origin "$BRANCH"

git rev-parse HEAD
git log -1 --oneline --decorate

source /opt/ros/noetic/setup.bash
catkin_make 2>&1 | tee "$LOG_FILE"
source "$WS/devel/setup.bash"
rospack find cr5_book_spray_demo

echo "Branch: $(git branch --show-current)"
echo "Commit: $(git rev-parse HEAD)"
echo "Build log: $LOG_FILE"
