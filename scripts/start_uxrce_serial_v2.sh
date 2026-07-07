#!/usr/bin/env bash
set -euo pipefail

DEVICE="${1:-/dev/ttyAMA0}"
BAUD="${2:-921600}"
AGENT="/home/john/autonomous_drone_px4_vio/tools/Micro-XRCE-DDS-Agent-v2.4.3/build_static/MicroXRCEAgent"

exec "$AGENT" serial -D "$DEVICE" -b "$BAUD"
