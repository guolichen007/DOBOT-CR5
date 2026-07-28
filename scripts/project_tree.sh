#!/usr/bin/env bash
# project_tree.sh — 显示项目源码树 (排除编译产物)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DEPTH=4
while [[ $# -gt 0 ]]; do
    case "$1" in
        --depth) DEPTH="$2"; shift 2 ;;
        *) echo "Usage: $0 [--depth N]"; exit 1 ;;
    esac
done

EXCLUDE=".git|build|devel|install|log|logs|.venv|__pycache__|.pytest_cache|artifacts"

if command -v tree &>/dev/null; then
    tree -a -L "$DEPTH" -I "$EXCLUDE" --dirsfirst
else
    echo "(tree not installed — using find fallback)"
    echo "Install with: sudo apt install tree"
    echo ""
    find . -maxdepth "$DEPTH" -not -path './.git/*' \
        -not -path './build/*' -not -path './devel/*' \
        -not -path './.venv*/*' -not -path './install/*' \
        -not -name '__pycache__' -not -name '.pytest_cache' \
        -not -path './artifacts/*' \
        | sort | head -200
fi
