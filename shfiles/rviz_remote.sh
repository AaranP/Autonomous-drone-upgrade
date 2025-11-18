#!/usr/bin/env bash
set -euo pipefail

usage() {
  echo "Usage: $0 <ROS_MASTER_IP> [LOCAL_IP] [RVIZ_CONFIG]"
  echo "Example: $0 128.189.246.106 206.87.209.35 ~/config/my.rviz"
  exit 1
}

if [ "$#" -lt 1 ]; then
  usage
fi

MASTER_IP="$1"
LOCAL_IP="${2:-}"
RVIZ_CFG="${3:-}"

if [ -z "$LOCAL_IP" ]; then
  # Try to detect the local IP used to reach the master
  if command -v ip >/dev/null 2>&1; then
    LOCAL_IP=$(ip route get "$MASTER_IP" 2>/dev/null | awk '{for(i=1;i<=NF;i++) if($i=="src") {print $(i+1); exit}}') || true
  fi

  if [ -z "$LOCAL_IP" ] && command -v hostname >/dev/null 2>&1; then
    LOCAL_IP=$(hostname -I 2>/dev/null | awk '{print $1}') || true
  fi
fi

if [ -z "$LOCAL_IP" ]; then
  echo "Could not detect local IP. Provide it as the 2nd argument." >&2
  usage
fi

export ROS_MASTER_URI="http://$MASTER_IP:11311"
export ROS_IP="$LOCAL_IP"

echo "Using ROS_MASTER_URI=$ROS_MASTER_URI"
echo "Using ROS_IP=$ROS_IP"

if [ -n "$RVIZ_CFG" ]; then
  exec rviz -d "$RVIZ_CFG"
else
  exec rviz
fi
