#!/usr/bin/env bash
set -euo pipefail
if [[ $# -ne 1 ]]; then
  echo "Usage: sudo $0 <ethernet-interface>"
  echo "Example: sudo $0 enp3s0"
  exit 2
fi
IFACE="$1"
ROOT="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$ROOT/config/lab.env"
CONN="cr5-robot-${IFACE}"
if nmcli -t -f NAME connection show | grep -Fxq "$CONN"; then
  nmcli connection modify "$CONN" \
    ipv4.method manual ipv4.addresses "${ROBOT_PC_IP}/${ROBOT_PREFIX}" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv6.method disabled
else
  nmcli connection add type ethernet ifname "$IFACE" con-name "$CONN" \
    ipv4.method manual ipv4.addresses "${ROBOT_PC_IP}/${ROBOT_PREFIX}" \
    ipv4.gateway "" ipv4.dns "" ipv4.never-default yes ipv6.method disabled
fi
nmcli connection up "$CONN"
echo "Configured $IFACE as ${ROBOT_PC_IP}/${ROBOT_PREFIX} for CR5."
