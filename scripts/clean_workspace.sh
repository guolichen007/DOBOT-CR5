#!/usr/bin/env bash
# clean_workspace.sh — 清理编译产物和缓存 (默认 dry-run)
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

DRY_RUN=true
if [[ "${1:-}" == "--yes" ]]; then
    DRY_RUN=false
fi

TARGETS=(build devel install log logs)
CLEAN_FILES=0

echo "=== CR5 Workspace Cleaner ==="
echo "Mode: $([ "$DRY_RUN" = true ] && echo 'DRY RUN (no deletion)' || echo 'LIVE')"
echo ""

for d in "${TARGETS[@]}"; do
    if [ -d "$d" ]; then
        n=$(find "$d" -type f 2>/dev/null | wc -l)
        s=$(du -sh "$d" 2>/dev/null | cut -f1)
        echo "  $d: $n files, $s"
        CLEAN_FILES=$((CLEAN_FILES + n))
        if [ "$DRY_RUN" = false ]; then
            rm -rf "$d"
        fi
    fi
done

# Python cache
pycache_dirs=$(find . -type d -name '__pycache__' -not -path './.venv*/*' 2>/dev/null | wc -l)
pyc_files=$(find . -type f \( -name '*.pyc' -o -name '*.pyo' \) -not -path './.venv*/*' 2>/dev/null | wc -l)
pytest_dirs=$(find . -type d -name '.pytest_cache' 2>/dev/null | wc -l)
echo "  __pycache__/: $pycache_dirs dirs"
echo "  *.pyc/*.pyo: $pyc_files files"
echo "  .pytest_cache/: $pytest_dirs dirs"

if [ "$DRY_RUN" = false ]; then
    find . -type d -name '__pycache__' -not -path './.venv*/*' -prune -exec rm -rf {} + 2>/dev/null || true
    find . -type f \( -name '*.pyc' -o -name '*.pyo' \) -not -path './.venv*/*' -delete 2>/dev/null || true
    find . -type d -name '.pytest_cache' -prune -exec rm -rf {} + 2>/dev/null || true
fi

echo ""
if [ "$DRY_RUN" = true ]; then
    echo "DRY RUN complete. Use --yes to execute."
else
    echo "Clean complete."
fi
echo ""
echo "Protected (never deleted): src/ docs/ config/ tools/ scripts/ .git/"
