#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/config/lab.env"
echo "Checking CR5 controller at ${ROBOT_IP}..."
ping -c 2 -W 2 "$ROBOT_IP"
for port in 29999 30003 30004; do
  nc -zvw2 "$ROBOT_IP" "$port"
done
echo "CR5 network and all V3 ports are reachable."
