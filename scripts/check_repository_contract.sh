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

echo "========================================="
echo " Repository Contract Check"
echo "========================================="
echo ""

# Required directories
check "docs/ exists"         test -d docs
check "scripts/ exists"      test -d scripts
check "config/ exists"       test -d config
check "src/ exists"          test -d src

# 7 required packages
for pkg in cr5_moveit cr5_spray_perception cr5_spray_sim \
           dobot_bringup dobot_description \
           realsense_gazebo_description realsense_gazebo_plugin; do
    check "package src/$pkg" test -d "src/$pkg"
done

# Forbidden legacy packages
for legacy in a4_spray_demo cr5_spray_planning cr5_book_spray_demo; do
    check "legacy src/$legacy absent" test ! -d "src/$legacy"
done

# Build artifacts should be gitignored
check "build/ gitignored"    git check-ignore -q build 2>/dev/null || true
check "devel/ gitignored"    git check-ignore -q devel 2>/dev/null || true

# No tracked build artifacts
check "no tracked *.pyc"     test -z "$(git ls-files '*.pyc' 2>/dev/null)"
check "no tracked __pycache__" test -z "$(git ls-files '*/__pycache__/*' 2>/dev/null)"

# Key files
check "README.md exists"     test -f README.md
check "docs/README.md exists" test -f docs/README.md
check "calibration_target.yaml" test -f src/cr5_spray_sim/config/calibration/calibration_target.yaml
check "build.sh exists"      test -f scripts/build.sh

# config
check "config/local.example.yaml" test -f config/local.example.yaml

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
