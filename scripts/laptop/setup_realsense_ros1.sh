#!/usr/bin/env bash
set -euo pipefail

# ============================================================
# setup_realsense_ros1.sh
# 创建独立 RealSense ROS1 工作空间（软链接方式）
# ============================================================

WS="${WS:-$HOME/cr5_ros1_ws}"
REALSENSE_WS="${REALSENSE_WS:-$HOME/realsense_ros1_ws}"
LEGACY_REALSENSE_REPO="${LEGACY_REALSENSE_REPO:-$HOME/cr5_ws/src/realsense-ros}"
EXPECTED_HEAD="${EXPECTED_HEAD:-f400d682beee6c216052a419f419e95b797255ad}"
EXPECTED_VERSION="2.3.2"

LOG_DIR="${LOG_DIR:-$HOME/cr5_test_logs}"
STAMP="$(date +%Y%m%d_%H%M%S)"
LOG_FILE="$LOG_DIR/realsense_ros1_build_${STAMP}.log"

fail() { echo "[ERROR] $*" >&2; exit 1; }
warn() { echo "[WARN] $*" >&2; }
info() { echo "[INFO] $*"; }

mkdir -p "$LOG_DIR"

# ============================================================
# 1. 记录系统信息
# ============================================================
echo "=========================================="
echo "  RealSense ROS1 工作空间设置"
echo "=========================================="

echo ""
echo "--- 系统信息 ---"
info "cmake 版本: $(cmake --version | head -1)"
info "Librealsense 版本: $(pkg-config --modversion realsense2 2>/dev/null || echo '未知')"
info "Librealsense 路径: $(pkg-config --variable=prefix realsense2 2>/dev/null || echo '未知')"

# ============================================================
# 2. 检查旧 RealSense 仓库
# ============================================================
echo ""
echo "--- 检查旧 RealSense 仓库 ---"

[ -d "$LEGACY_REALSENSE_REPO/.git" ] || fail "$LEGACY_REALSENSE_REPO is not a Git worktree"

cd "$LEGACY_REALSENSE_REPO"

info "旧 RealSense 仓库状态："
echo "  路径: $LEGACY_REALSENSE_REPO"
echo "  HEAD: $(git rev-parse HEAD)"
echo "  Tag: $(git describe --tags --always --dirty)"
echo "  分支: $(git branch --show-current 2>/dev/null || echo 'detached HEAD')"

# ============================================================
# 3. 检查 HEAD 是否匹配
# ============================================================
ACTUAL_HEAD="$(git rev-parse HEAD)"
if [ "$ACTUAL_HEAD" != "$EXPECTED_HEAD" ]; then
    warn "HEAD 不匹配！"
    echo "  期望: $EXPECTED_HEAD"
    echo "  实际: $ACTUAL_HEAD"
    if [ "${ALLOW_UNEXPECTED_REALSENSE_HEAD:-0}" != "1" ]; then
        fail "HEAD 不匹配，停止。设置 ALLOW_UNEXPECTED_REALSENSE_HEAD=1 可绕过。"
    fi
    warn "ALLOW_UNEXPECTED_REALSENSE_HEAD=1，继续执行..."
else
    info "HEAD 匹配 ✓"
fi

# ============================================================
# 4. 检查包版本
# ============================================================
CAMERA_VER="$(grep '<version>' realsense2_camera/package.xml | sed 's/.*<version>\(.*\)<\/version>.*/\1/')"
DESC_VER="$(grep '<version>' realsense2_description/package.xml | sed 's/.*<version>\(.*\)<\/version>.*/\1/')"

if [ "$CAMERA_VER" != "$EXPECTED_VERSION" ]; then
    fail "realsense2_camera 版本不匹配: 期望 $EXPECTED_VERSION, 实际 $CAMERA_VER"
fi
if [ "$DESC_VER" != "$EXPECTED_VERSION" ]; then
    fail "realsense2_description 版本不匹配: 期望 $EXPECTED_VERSION, 实际 $DESC_VER"
fi
info "包版本: realsense2_camera=$CAMERA_VER, realsense2_description=$DESC_VER ✓"

# ============================================================
# 5. 创建独立工作空间
# ============================================================
info "创建 RealSense 工作空间: $REALSENSE_WS"
mkdir -p "$REALSENSE_WS/src"

# ============================================================
# 6. 创建软链接
# ============================================================
LINK_CAMERA="$REALSENSE_WS/src/realsense2_camera"
LINK_DESC="$REALSENSE_WS/src/realsense2_description"

# 检查 realsense2_camera
if [ -L "$LINK_CAMERA" ]; then
    TARGET="$(readlink -f "$LINK_CAMERA")"
    EXPECTED_TARGET="$(readlink -f "$LEGACY_REALSENSE_REPO/realsense2_camera")"
    if [ "$TARGET" = "$EXPECTED_TARGET" ]; then
        info "realsense2_camera 软链接已存在且正确 ✓"
    else
        fail "realsense2_camera 软链接指向错误位置: $TARGET"
    fi
elif [ -e "$LINK_CAMERA" ]; then
    fail "realsense2_camera 已存在但不是软链接"
else
    ln -s "$LEGACY_REALSENSE_REPO/realsense2_camera" "$LINK_CAMERA"
    info "创建 realsense2_camera 软链接 ✓"
fi

# 检查 realsense2_description
if [ -L "$LINK_DESC" ]; then
    TARGET="$(readlink -f "$LINK_DESC")"
    EXPECTED_TARGET="$(readlink -f "$LEGACY_REALSENSE_REPO/realsense2_description")"
    if [ "$TARGET" = "$EXPECTED_TARGET" ]; then
        info "realsense2_description 软链接已存在且正确 ✓"
    else
        fail "realsense2_description 软链接指向错误位置: $TARGET"
    fi
elif [ -e "$LINK_DESC" ]; then
    fail "realsense2_description 已存在但不是软链接"
else
    ln -s "$LEGACY_REALSENSE_REPO/realsense2_description" "$LINK_DESC"
    info "创建 realsense2_description 软链接 ✓"
fi

# ============================================================
# 7. 检查依赖（跳过 librealsense2）
# ============================================================
echo ""
echo "--- 检查依赖 ---"
source /opt/ros/noetic/setup.bash

if ! rosdep check --from-paths "$REALSENSE_WS/src" --ignore-src --skip-keys=librealsense2 --rosdistro noetic 2>&1; then
    warn "存在缺失依赖"
    echo ""
    echo "请手动执行以下命令安装依赖："
    echo "  rosdep install --from-paths $REALSENSE_WS/src --ignore-src --skip-keys=librealsense2 --rosdistro noetic -y"
    echo ""
    fail "依赖检查未通过，停止编译。"
fi
info "依赖检查通过 ✓"

# ============================================================
# 8. 编译（使用实机验证成功的参数）
# ============================================================
echo ""
echo "--- 编译 ---"
info "开始编译 RealSense 工作空间..."
cd "$REALSENSE_WS"

catkin_make \
    -DCMAKE_BUILD_TYPE=Release \
    -DCMAKE_POLICY_VERSION_MINIMUM=3.5 \
    -Drealsense2_DIR=/usr/local/lib/cmake/realsense2 \
    2>&1 | tee "$LOG_FILE"

if [ "${PIPESTATUS[0]}" -ne 0 ]; then
    fail "编译失败，日志: $LOG_FILE"
fi
info "编译成功 ✓"
info "编译日志: $LOG_FILE"

# ============================================================
# 9. 验证运行时库链接
# ============================================================
echo ""
echo "--- 验证运行时库 ---"
CAMERA_SO="$REALSENSE_WS/devel/lib/librealsense2_camera.so"

if [ -f "$CAMERA_SO" ]; then
    info "检查 $CAMERA_SO 的库依赖..."
    LRS_LINK="$(ldd "$CAMERA_SO" 2>/dev/null | grep librealsense2 || true)"
    echo "  $LRS_LINK"

    if echo "$LRS_LINK" | grep -q "/usr/local/lib/librealsense2.so"; then
        info "Librealsense 链接正确 ✓"
    else
        warn "Librealsense 链接可能不正确"
    fi
else
    warn "$CAMERA_SO 不存在，跳过库验证"
fi

# ============================================================
# 10. 验证包可用
# ============================================================
echo ""
echo "--- 验证 ROS 包 ---"
source "$REALSENSE_WS/devel/setup.bash"

CAMERA_PATH="$(rospack find realsense2_camera 2>/dev/null || true)"
DESC_PATH="$(rospack find realsense2_description 2>/dev/null || true)"

if [ -z "$CAMERA_PATH" ]; then
    fail "rospack find realsense2_camera 失败"
fi
if [ -z "$DESC_PATH" ]; then
    fail "rospack find realsense2_description 失败"
fi

echo "[PASS] realsense2_camera: $CAMERA_PATH"
echo "[PASS] realsense2_description: $DESC_PATH"

# ============================================================
# 11. 输出环境加载命令
# ============================================================
echo ""
echo "=========================================="
echo "  设置完成！"
echo "=========================================="
echo ""
echo "新终端需要执行的环境加载命令（正确叠加方式）："
echo ""
echo "  source /opt/ros/noetic/setup.bash"
echo "  source $REALSENSE_WS/devel/setup.bash"
echo "  source $WS/devel/setup.bash --extend"
echo ""
echo "注意：使用 --extend 叠加，避免覆盖 RealSense overlay。"
echo "不要使用 local_setup.bash（已在实机验证无效）。"
