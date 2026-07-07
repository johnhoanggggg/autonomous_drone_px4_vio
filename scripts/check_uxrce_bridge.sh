#!/usr/bin/env bash
set -eo pipefail

source /opt/ros/jazzy/setup.bash 2>/dev/null || source /opt/ros/humble/setup.bash
source /home/john/ros2_ws/install/setup.bash 2>/dev/null || true
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash 2>/dev/null || true
set -u

echo "ROS_DOMAIN_ID:"
echo "${ROS_DOMAIN_ID:-unset}"

echo
echo "Micro XRCE-DDS Agent executable:"
command -v MicroXRCEAgent || true

echo
echo "Micro XRCE-DDS Agent process:"
pgrep -a MicroXRCEAgent || echo "not running"

echo
echo "Likely Pixhawk serial devices:"
ls -l /dev/serial/by-id /dev/ttyACM* /dev/ttyUSB* 2>/dev/null || echo "no ttyACM/ttyUSB serial devices found"

echo
echo "PX4 ROS 2 topics:"
ros2 topic list | grep -E '^/fmu/' || echo "no /fmu topics discovered"

echo
echo "Visual odometry input endpoint:"
ros2 topic info /fmu/in/vehicle_visual_odometry || true
