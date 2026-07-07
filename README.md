# Autonomous Drone PX4 VIO

ROS 2 Jazzy workspace for sending OAK-D Lite VIO into PX4 visual odometry.

Current default: RTAB-Map VIO from the OAK-D Lite publishes a compatibility pose on `/basalt/pose`; `px4_vio_bridge` converts that pose into PX4 `VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`.

## Main Launch

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_vio_px4.launch.py
```

Despite the legacy filename, `basalt_vio_px4.launch.py` now starts RTAB-Map VIO, not Basalt.

It starts:

- `/usr/local/bin/MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600 -v 4`
- OAK-D Lite RTAB-Map VIO
- `vio_to_px4_odometry`
- PX4 local-position-to-ROS converter
- Foxglove bridge on port `8765`

Performance defaults in the main launch:

- `/rtabmap/image` publishing is disabled.
- `/rtabmap/path` publishes every 10 odometry poses.
- `/rtabmap/path` is capped at 1000 poses.
- Per-frame trajectory file logging is disabled.

Yaw-offset tester:

- Keep PX4 on `EKF2_EV_CTRL=3` while testing yaw alignment.
- Relaunch with `vio_yaw_offset_deg:=90.0` or `vio_yaw_offset_deg:=-90.0`.
- Compare raw `/basalt/pose` with corrected `/vio/yaw_offset/pose` and `/vio/yaw_offset/path` in Foxglove.
- The corrected VIO pose is what the bridge sends toward `/fmu/in/vehicle_visual_odometry`.

Foxglove URL:

```text
ws://<pi-ip>:8765
```

Useful topics:

```text
/rtabmap/path
/rtabmap/odometry
/rtabmap/image
/basalt/pose
/vio/yaw_offset/pose
/vio/yaw_offset/path
/vio/yaw_offset/odometry
/px4/local_position/pose
/px4/local_position/odometry
/px4/local_position/path
/fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_odometry
/fmu/out/vehicle_local_position_v1
```

## RTAB-Map Foxglove Only

Use this when you want to view OAK-D Lite RTAB-Map VIO without PX4/XRCE:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge rtabmap_oak_foxglove.launch.py
```

Observed RTAB-Map VIO rate is about `14-15 Hz` even with camera input requested at `30 fps`.

This Foxglove-only launch keeps `/rtabmap/image` enabled for debugging.

## Basalt Foxglove Only

Basalt is still available for comparison, but it is not the recommended path right now:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_oak_foxglove.launch.py
```

Basalt may be more timing-sensitive on this setup. Previous failures included native DepthAI/Basalt assertions from non-monotonic IMU/frame timestamps.

Basalt VIO combined with RTAB-Map SLAM was also tried and removed as a test path. It briefly advertised SLAM topics, then hit DepthAI/XLink/native shutdown failures. Treat Basalt as a dead end for now unless revisiting DepthAI/Basalt internals.

## Rebuild

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
colcon build --packages-select px4_vio_bridge
```

## Health Checks

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42

ros2 topic hz /basalt/pose
ros2 topic hz /rtabmap/path
ros2 topic echo /px4/local_position/odometry --once
ros2 topic info /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

Healthy PX4 input topic:

```text
/fmu/in/vehicle_visual_odometry
Publisher count: 1
Subscription count: 1
```

If `Subscription count` is `0` after restarting the XRCE agent, reboot/replug the Pixhawk or restart PX4 `uxrce_dds_client` while the agent is already running.

## Safety

Keep props removed. Do not test estimator fusion on a live vehicle until PX4 is receiving visual odometry and estimator status is verified.
