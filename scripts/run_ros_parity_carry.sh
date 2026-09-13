#!/usr/bin/env bash
set -eo pipefail
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source /home/john/rtabmap_latest_frame_overlay/install/local_setup.bash
exec python3 -u /home/john/autonomous_drone_px4_vio/scripts/ros_parity_local_carry.py "$@"
