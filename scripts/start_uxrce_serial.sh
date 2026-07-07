#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/ttyAMA0}"
BAUD="${2:-921600}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-42}"

exec MicroXRCEAgent serial -D "$DEVICE" -b "$BAUD"
