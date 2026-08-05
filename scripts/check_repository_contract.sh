#!/usr/bin/env bash
# check_repository_contract.sh — 验证仓库结构契约
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

PASS=0
FAIL=0
ERRORS=()

check() {
    local desc="$1"; shift
    if "$@"; then
        echo "  PASS: $desc"
        PASS=$((PASS + 1))
    else
        echo "  FAIL: $desc"
        FAIL=$((FAIL + 1))
        ERRORS+=("$desc")
    fi
}

check_negative() {
    # check_negative desc cmd — PASS if cmd fails (returns non-zero)
    local desc="$1"; shift
    if "$@"; then
        echo "  FAIL: $desc"
        FAIL=$((FAIL + 1))
        ERRORS+=("$desc")
    else
        echo "  PASS: $desc"
        PASS=$((PASS + 1))
    fi
}

echo "========================================="
echo " Repository Contract Check"
echo "========================================="
echo ""

# ── Required directories ──
check "docs/ exists"         test -d docs
check "scripts/ exists"      test -d scripts
check "config/ exists"       test -d config
check "src/ exists"          test -d src

# ── 7 required packages ──
for pkg in cr5_moveit cr5_spray_perception cr5_spray_sim \
           dobot_bringup dobot_description \
           realsense_gazebo_description realsense_gazebo_plugin; do
    check "package src/$pkg" test -d "src/$pkg"
done

# ── Forbidden legacy packages ──
for legacy in a4_spray_demo cr5_spray_planning cr5_book_spray_demo; do
    check "legacy src/$legacy absent" test ! -d "src/$legacy"
done

# ── Build artifacts should be gitignored ──
check "build/ gitignored"    git check-ignore -q build 2>/dev/null || true
check "devel/ gitignored"    git check-ignore -q devel 2>/dev/null || true

# ── No tracked build artifacts ──
check "no tracked *.pyc"     test -z "$(git ls-files '*.pyc' 2>/dev/null)"
check "no tracked __pycache__" test -z "$(git ls-files '*/__pycache__/*' 2>/dev/null)"

# ── Key files ──
check "README.md exists"     test -f README.md
check "docs/README.md exists" test -f docs/README.md
check "calibration_target.yaml" test -f src/cr5_spray_sim/config/calibration/calibration_target.yaml
check "build.sh exists"      test -f scripts/build.sh

# config
check "config/local.example.yaml" test -f config/local.example.yaml

# ── README: no version numbers ──
check_negative "README no V1.0.x version strings" \
    grep -qE 'V[0-9]+\.[0-9]+\.[0-9]+' README.md

# ── README: no hardcoded commit SHAs ──
check_negative "README no hardcoded commit SHA (40 hex)" \
    grep -qP '\b[0-9a-f]{40}\b' README.md

# ── README: mentions three long-lived branches ──
check "README mentions main branch" \
    grep -q 'main' README.md
check "README mentions calibration stable branch" \
    grep -q 'stable/three-camera-calibration-v1' README.md
check "README mentions reconstruction stable branch" \
    grep -q 'stable/visible-surface-reconstruction-v1' README.md

# ── CHANGELOG exists ──
check "CHANGELOG.md exists"  test -f CHANGELOG.md

# ── Quality contract modules exist ──
_RECON_SRC="src/cr5_spray_perception/src/cr5_spray_perception/reconstruction"
check "quality_contract.py exists" test -f "$_RECON_SRC/quality_contract.py"
check "evaluation_runner.py exists" test -f "$_RECON_SRC/evaluation_runner.py"

# ── Production runner in CMake install ──
check "runner in CMakeLists.txt install" \
    grep -q 'run_visible_surface_reconstruction.py' src/cr5_spray_perception/CMakeLists.txt

# ── CI checks ──
_CI=".github/workflows/ci.yml"
check "CI file exists"        test -f "$_CI"
check "CI contains reconstruction-quality-gate" \
    grep -q 'reconstruction-quality-gate' "$_CI"
check "CI uses PYTHONPATH with default" \
    grep -q '${PYTHONPATH:-}' "$_CI" || grep -q 'PYTHONPATH.*:-' "$_CI"
check_negative "CI no || echo after test commands" \
    grep -E 'python3.*unittest.*\|\| echo' "$_CI" || \
    grep -E 'pip.*install.*\|\| echo' "$_CI"
check_negative "CI no || true after test commands" \
    grep -E 'python3.*unittest.*\|\| true' "$_CI"
check_negative "CI no tail hiding unittest output" \
    grep -E 'unittest.*\| *tail' "$_CI"
check "CI uses checkout@v4" \
    grep -q 'actions/checkout@v4' "$_CI"

# ── Integration test actually executes run_evaluator ──
_INTTEST="src/cr5_spray_perception/test/test_reconstruction_runner_integration.py"
check "integration test imports run_evaluator" \
    grep -q 'run_evaluator' "$_INTTEST"
check "integration test uses subprocess" \
    grep -q 'subprocess' "$_INTTEST"

# ── Runner checks ──
_RUNNER="src/cr5_spray_perception/scripts/run_visible_surface_reconstruction.py"
check "runner uses require_open3d (lazy import)" \
    grep -q 'require_open3d' "$_RUNNER"
check_negative "runner no module-level sys.exit(1) on Open3D" \
    grep -qE 'except ImportError:.*sys\.exit' "$_RUNNER"

# ── Production config checks ──
_PROD_CFG="src/cr5_spray_perception/config/reconstruction/visible_surface_production.yaml"
check "production config: allow_oracle=false" \
    grep -q 'allow_oracle.*false' "$_PROD_CFG"
check "production config: allow_runtime_refinement=false" \
    grep -q 'allow_runtime_refinement.*false' "$_PROD_CFG"
check "production config: bottom=UNKNOWN" \
    grep -q 'UNKNOWN' "$_PROD_CFG"

# ── Gitignore: no tracked data files ──
check_negative "no tracked *.ply" \
    test -n "$(git ls-files '*.ply' 2>/dev/null)"
check_negative "no tracked *.pcd" \
    test -n "$(git ls-files '*.pcd' 2>/dev/null)"
check_negative "no tracked *.npy" \
    test -n "$(git ls-files '*.npy' 2>/dev/null)"
check_negative "no tracked *.zip" \
    test -n "$(git ls-files '*.zip' 2>/dev/null)"
check_negative "no tracked config/local.yaml" \
    test -n "$(git ls-files 'config/local.yaml' 2>/dev/null)"
check_negative "no tracked docs/会话上下文.md" \
    test -n "$(git ls-files 'docs/会话上下文.md' 2>/dev/null)"

# ── docs/releases/ exists ──
check "docs/releases/ exists" test -d docs/releases

# ── Permanent naming convention (mainline evergreen) ──
_PERM_CFG="src/cr5_spray_perception/config/reconstruction/visible_surface_production.yaml"
check "permanent production config exists" test -f "$_PERM_CFG"
check_negative "old V1 config deleted" \
    test -f "src/cr5_spray_perception/config/reconstruction/visible_surface_production_v1.yaml"
check_negative "old V2 isolation config deleted" \
    test -f "src/cr5_spray_perception/config/reconstruction/visible_surface_target_isolation_v2.yaml"
check_negative "release doc migrated to validation" \
    test -f "docs/releases/target-isolation-v2.md"
check "validation doc exists" test -f "docs/validation/fixed-camera-reconstruction.md"
check_negative "README no V1 config reference" \
    grep -q 'visible_surface_production_v1' README.md
check "README references permanent config" \
    grep -q 'visible_surface_production.yaml' README.md
check_negative "no target isolation V2 in active code" \
    grep -rq 'target.isolation.V2\|目标隔离.V2\|target_isolation_v2' \
    src/cr5_spray_perception/config/reconstruction/ src/cr5_spray_perception/scripts/ \
    src/cr5_spray_perception/src/ docs/validation/ 2>/dev/null

echo ""
echo "========================================="
echo " Results: $PASS passed, $FAIL failed"
echo "========================================="

if [ "$FAIL" -gt 0 ]; then
    echo ""
    echo "FAILURES:"
    for e in "${ERRORS[@]}"; do
        echo "  - $e"
    done
    exit 1
else
    echo "CONTRACT_PASS"
fi
