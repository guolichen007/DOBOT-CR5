#!/bin/bash
# CR5 Spray Demo: Python Environment Setup
# 创建隔离 venv，不影响系统 Python 和 ROS cv_bridge

set -e
WORKSPACE="$(cd "$(dirname "$0")/.." && pwd)"
VENV="$WORKSPACE/.venv-spray"

if [ ! -d "$VENV" ]; then
    echo "Creating venv at $VENV..."
    python3 -m venv "$VENV"
fi

source "$VENV/bin/activate"
echo "Installing dependencies..."
pip install -r "$WORKSPACE/scripts/requirements-spray.txt"

echo ""
echo "Setup complete. Activate with:"
echo "  source $VENV/bin/activate"
echo ""
echo "Verify:"
echo "  python3 -c 'import open3d, numpy, cv2, yaml; print(\"OK\")'"
