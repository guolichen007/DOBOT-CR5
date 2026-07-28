#!/usr/bin/env bash
# workspace_stats.sh — 工作区统计概览
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
cd "$ROOT"

echo "========================================="
echo " CR5 ROS1 Workspace Stats"
echo "========================================="
echo ""
echo "Git:"
echo "  branch: $(git branch --show-current)"
echo "  HEAD:   $(git rev-parse --short HEAD)"
echo "  tracked: $(git ls-files | wc -l) files"
echo ""

echo "Files:"
printf "  total:      %s\n" "$(find . -type f 2>/dev/null | wc -l)"
printf "  src:        %s\n" "$(find src -type f 2>/dev/null | wc -l)"
printf "  build:      %s\n" "$(find build -type f 2>/dev/null | wc -l)"
printf "  devel:      %s\n" "$(find devel -type f 2>/dev/null | wc -l)"
echo ""

echo "Size:"
for d in build devel .git .venv-spray src docs; do
    [ -e "$d" ] || continue
    printf "  %-20s %s\n" "$d:" "$(du -sh "$d" 2>/dev/null | cut -f1)"
done
echo ""

echo "Packages:"
for d in src/*; do
    [ -d "$d" ] || continue
    tracked=$(git ls-files "$d" 2>/dev/null | wc -l)
    files=$(find "$d" -type f 2>/dev/null | wc -l)
    printf "  %-38s tracked=%-5s files=%-5s\n" "$(basename "$d")" "$tracked" "$files"
done
echo ""

echo "Data:"
echo "  CR5_DATA_ROOT: ${CR5_DATA_ROOT:-$HOME/cr5_data}"
echo "  CR5_VENV_DIR:  ${CR5_VENV_DIR:-$HOME/.venvs/cr5-spray}"
[ -d "${CR5_DATA_ROOT:-$HOME/cr5_data}" ] && echo "  data size: $(du -sh "${CR5_DATA_ROOT:-$HOME/cr5_data}" 2>/dev/null | cut -f1)"
echo ""

untracked=$(git ls-files --others --exclude-standard | wc -l)
echo "Untracked files: $untracked"
if [ "$untracked" -gt 0 ] && [ "$untracked" -lt 20 ]; then
    git ls-files --others --exclude-standard | sed 's/^/  /'
fi
