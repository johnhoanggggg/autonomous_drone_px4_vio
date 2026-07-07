# Autonomous Drone PX4 VIO Handoff

Date: 2026-07-03

Latest operational update: 2026-07-06

- The main launch file has been rewired to use OAK-D Lite RTAB-Map VIO instead of Basalt:

```bash
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_vio_px4.launch.py
```

- The filename is legacy. It now starts `rtabmap_vio_ros2.py`, Micro XRCE-DDS Agent, `vio_to_px4_odometry`, and Foxglove.
- RTAB-Map publishes `/basalt/pose` as a compatibility topic for the existing PX4 bridge, so `vio_to_px4_odometry` still subscribes to `/basalt/pose`.
- RTAB-Map Foxglove-only launch was also added:

```bash
ros2 launch px4_vio_bridge rtabmap_oak_foxglove.launch.py
```

- Basalt Foxglove-only launch remains available:

```bash
ros2 launch px4_vio_bridge basalt_oak_foxglove.launch.py
```

## Project

Project folder:

```bash
/home/john/autonomous_drone_px4_vio
```

ROS 2 bridge package:

```bash
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge
```

The bridge subscribes to:

```bash
/basalt/pose
```

and publishes PX4 visual odometry to:

```bash
/fmu/in/vehicle_visual_odometry
```

Current default OAK-D Lite VIO source is RTAB-Map:

```bash
/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/rtabmap/rtabmap_vio_ros2.py
```

It publishes RTAB-Map visualization topics and also publishes `/basalt/pose` for bridge compatibility.

The OAK-D Lite Basalt VIO script also publishes `/basalt/pose` but is no longer the default main launch source:

```bash
/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/basalt/basalt_vio_ros2.py
```

## ROS Environment

This Pi is using ROS 2 Jazzy:

```bash
/opt/ros/jazzy/setup.bash
ROS_DOMAIN_ID=42
```

Existing PX4 messages are installed from:

```bash
/home/john/ros2_ws/install/px4_msgs
```

The bridge package builds successfully:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
colcon build --symlink-install
```

## Hardware State

Hardware:

- Raspberry Pi 5
- OAK-D Lite
- Pixhawk 4 / PX4 FMU v5.x
- Pixhawk connected to Pi via TELEM2 UART and USB

Pi serial devices observed:

```bash
/dev/ttyAMA0
/dev/ttyAMA10
/dev/ttyACM0
```

Pixhawk USB appears as:

```bash
/dev/ttyACM0
/dev/serial/by-id/usb-3D_Robotics_PX4_FMU_v5.x_0-if00
```

TELEM2/uXRCE physical wiring appears to reach Pi UART:

```bash
/dev/ttyAMA0
```

## Micro XRCE-DDS Agent

System-installed agent is v3.0.1:

```bash
/usr/local/bin/MicroXRCEAgent
```

PX4/Jazzy docs expect Micro XRCE-DDS Agent v2.4.3 unless PX4 firmware was built for DDS v3. A project-local v2.4.3 agent was built here:

```bash
/home/john/autonomous_drone_px4_vio/tools/Micro-XRCE-DDS-Agent-v2.4.3/build_static/MicroXRCEAgent
```

Earlier testing focused on the v2.4.3 agent:

```bash
/home/john/autonomous_drone_px4_vio/scripts/start_uxrce_serial_v2.sh
```

That runs:

```bash
MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600
```

Update from 2026-07-03 current session:

- `ModemManager` was stopped/disabled by the user.
- Pixhawk USB MAVLink briefly worked on `/dev/ttyACM0` at 115200 after that.
- The system v3.0.1 agent at `/usr/local/bin/MicroXRCEAgent` successfully established an XRCE session on `/dev/ttyAMA0` and created participant `/px4_micro_xrce_dds`.
- The v3 agent then only showed time-sync/session traffic; PX4 did not send topic/datawriter/datareader creation requests, so ROS still showed no `/fmu/...` topics.
- PX4 NSH over MAVProxy reported:

```text
uxrce_dds_client status
INFO  [uxrce_dds_client] Running, disconnected
```

- The default start script now uses the system v3 agent and defaults `ROS_DOMAIN_ID=42`:

```bash
/home/john/autonomous_drone_px4_vio/scripts/start_uxrce_serial.sh
```

Update after full reboot test:

- The link was established with the system v3.0.1 agent, not the project-local v2.4.3 agent:

```bash
ROS_DOMAIN_ID=42 /usr/local/bin/MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600 -v 4
```

- PX4 initially created only the XRCE participant and reported:

```text
uxrce_dds_client status
Running, disconnected
timesync converged: false
```

- The fix was disabling PX4 uXRCE time sync and sync-convergence gating:

```sh
param set UXRCE_DDS_SYNCC 0
param set UXRCE_DDS_SYNCT 0
param save
uxrce_dds_client stop
uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600
```

- After that, PX4 reported:

```text
uxrce_dds_client status
Running, connected
```

- ROS 2 then discovered the expected `/fmu/...` graph, including:

```text
/fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_odometry
```

- The bridge was launched successfully and now publishes to the PX4 input topic:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge vio_to_px4.launch.py
```

`/fmu/in/vehicle_visual_odometry` showed `Publisher count: 1` and `Subscription count: 1`.

- The OAK-D/Basalt publisher is not currently running because DepthAI reported:

```text
RuntimeError: No available devices
```

Once the OAK-D Lite is visible again, run:

```bash
source /opt/ros/jazzy/setup.bash
export ROS_DOMAIN_ID=42
python3 /home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/basalt/basalt_vio_ros2.py
```

Update after OAK-D reconnect:

- OAK-D Lite enumerated and VIO pose flow was tested.
- `basalt_vio_ros2.py` produced poses briefly, but repeatedly crashed inside DepthAI/Basalt with non-monotonic IMU/frame timestamps:

```text
Assertion (data.t_ns > curr_state.t_ns) failed
Assertion (prev_frame->t_ns < curr_frame->t_ns) failed
```

- Lowering Basalt camera FPS from 60 to 30 did not fix the crash.
- RTAB-Map VIO was stable at about 13 Hz, so `/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/rtabmap/rtabmap_vio_ros2.py` was patched to publish each `PoseStamped` to `/basalt/pose`.
- With RTAB-Map VIO running, the PX4 bridge published real transformed messages to `/fmu/in/vehicle_visual_odometry`.
- Current long-running processes:

```text
MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600 -v 4
vio_to_px4_odometry
rtabmap_vio_ros2.py
```

- Current issue: after restarting the XRCE agent, PX4 did not recreate its DDS subscription. `/basalt/pose` and `/fmu/in/vehicle_visual_odometry` have local publishers, but `/fmu/in/vehicle_visual_odometry` may show `Subscription count: 0` until the Pixhawk uXRCE client reconnects.
- Pixhawk USB MAVLink later became byte-quiet (`/dev/ttyACM0` raw read returned 0 bytes), so the Pi could not command a Pixhawk reboot at that point.
- Next step: physically reboot/replug the Pixhawk while the v3 agent is already running, then check:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
ROS_DOMAIN_ID=42 ros2 topic info /fmu/in/vehicle_visual_odometry
```

Expected healthy state:

```text
Publisher count: 1
Subscription count: 1
```

Update after Basalt retry with camera motion at startup:

- OAK-D Lite was visible to DepthAI.
- Starting Basalt while moving the camera produced stable `/basalt/pose` output at about 30 Hz.
- The system v3.0.1 XRCE agent on `/dev/ttyAMA0` at 921600 established a PX4 session and PX4 created DDS topics/readers/writers.
- The bridge launched successfully and published Basalt poses to PX4:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge vio_to_px4.launch.py
```

- Verified healthy PX4 input topic:

```text
/fmu/in/vehicle_visual_odometry
Type: px4_msgs/msg/VehicleOdometry
Publisher count: 1
Subscription count: 1
```

- Measured rates:

```text
/basalt/pose: about 30 Hz
/fmu/in/vehicle_visual_odometry: about 30 Hz
```

- Current live processes from this retry:

```text
python3 .../basalt/basalt_vio_ros2.py
/usr/local/bin/MicroXRCEAgent serial -D /dev/ttyAMA0 -b 921600 -v 4
ros2 launch px4_vio_bridge vio_to_px4.launch.py
vio_to_px4_odometry
```

Update after 60 FPS Basalt retry:

- `basalt_vio_ros2.py` was updated to accept CLI arguments:

```bash
--fps
--width
--height
```

- New combined launch file:

```bash
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_vio_px4.launch.py
```

- Rebuilt successfully:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
colcon build --symlink-install
```

- Launch command:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge basalt_vio_px4.launch.py basalt_fps:=60
```

- Basalt did not reproduce the previous timestamp assertion during the 60 FPS run.
- Actual measured `/basalt/pose` and `/fmu/in/vehicle_visual_odometry` publish rate was about 40-42 Hz, not a full 60 Hz.
- The bridge reached about 1900 published visual odometry messages before the test was stopped.
- After restarting the XRCE agent, PX4 did not recreate its `/fmu/in/vehicle_visual_odometry` DDS subscriber during this run:

```text
Publisher count: 1
Subscription count: 0
```

This matches the earlier reconnect caveat: if the agent is restarted, physically reboot/replug the Pixhawk or restart `uxrce_dds_client` while the agent is already running.

- The test launch was stopped and no Basalt/XRCE/bridge processes were left running.

## PX4 Parameters Seen

From NSH/QGC MAVLink Console:

```text
UXRCE_DDS_CFG     = 102
SER_TEL2_BAUD     = 921600
UXRCE_DDS_DOM_ID  = 42
UXRCE_DDS_KEY     = 1
UXRCE_DDS_PTCFG   = 0
UXRCE_DDS_SYNCC   = 1
MAV_1_CONFIG      = 0
MAV_2_CONFIG      = 0
MAV_0_CONFIG      = 101
```

PX4 reports:

```text
uxrce_dds_client status
Running, disconnected
Using transport: serial
timesync converged: false
```

Available PX4 `/dev` devices:

```text
/dev/ttyACM0
/dev/ttyS0
/dev/ttyS1
/dev/ttyS2
/dev/ttyS3
/dev/ttyS4
/dev/ttyS5
/dev/ttyS6
```

## Important Findings

1. `/dev/ttyAMA0` on the Pi is the UART that can receive XRCE bytes from PX4.

2. When PX4 starts via the saved `UXRCE_DDS_CFG=102` path, Pi-side `strace` showed XRCE frames arriving and the agent replying.

3. PX4 only sent participant creation XML and time-sync messages:

```xml
<dds><participant><rtps><name>/px4_micro_xrce_dds</name></rtps></participant></dds>
```

It did not send topic/datawriter/datareader creation requests, so ROS never saw `/fmu/...` topics.

4. Manual PX4 starts with these did not reach the Pi:

```sh
uxrce_dds_client start -t serial -d /dev/ttyS1 -b 921600
uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600
```

`/dev/ttyS3` was suggested by PX4 docs for Holybro Pixhawk 6C TELEM2, but this vehicle is Pixhawk 4, so mapping may differ.

5. Pi-side ROS currently shows only:

```bash
/parameter_events
/rosout
```

No `/fmu/...` topics have appeared yet.

## USB MAVLink/NSH Access

Pixhawk USB is now plugged into the Pi and appears as `/dev/ttyACM0`.

MAVProxy and pymavlink were installed into:

```bash
/home/john/autonomous_drone_px4_vio/.venv-mavlink
```

MAVProxy command:

```bash
/home/john/autonomous_drone_px4_vio/.venv-mavlink/bin/mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
```

However, no heartbeat was received in early tests.

`ModemManager` is running and the Pixhawk USB device is marked as a ModemManager candidate:

```text
ID_MM_CANDIDATE=1
```

Likely next step:

```bash
sudo systemctl stop ModemManager
sudo systemctl disable ModemManager
```

Then retry MAVProxy on `/dev/ttyACM0` with common baud rates:

```bash
/home/john/autonomous_drone_px4_vio/.venv-mavlink/bin/mavproxy.py --master=/dev/ttyACM0 --baudrate 115200
/home/john/autonomous_drone_px4_vio/.venv-mavlink/bin/mavproxy.py --master=/dev/ttyACM0 --baudrate 57600
```

## Suggested Next Session Plan

1. Stop/disable ModemManager so Pixhawk USB is not disturbed.

2. Get MAVLink heartbeat from `/dev/ttyACM0` using MAVProxy or pymavlink.

3. Use MAVLink shell/NSH from the Pi if possible.

4. Start the v2.4.3 XRCE agent:

```bash
/home/john/autonomous_drone_px4_vio/scripts/start_uxrce_serial_v2.sh
```

5. On PX4, restart uXRCE using saved params after reboot, not manual `/dev/ttyS*` guesses:

```sh
param set UXRCE_DDS_CFG 102
param set SER_TEL2_BAUD 921600
param set UXRCE_DDS_DOM_ID 42
param save
reboot
```

6. Check:

```sh
uxrce_dds_client status
```

7. On Pi, check:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
ros2 topic list | grep /fmu
```

8. If `/fmu` topics appear, run the bridge:

```bash
ros2 launch px4_vio_bridge vio_to_px4.launch.py
```

9. If PX4 still says `disconnected`, use Pi-side `strace` on the v2.4.3 agent to verify whether bytes are arriving:

```bash
timeout 8 strace -f -e trace=read,write -s 80 \
  /home/john/autonomous_drone_px4_vio/tools/Micro-XRCE-DDS-Agent-v2.4.3/build_static/MicroXRCEAgent \
  serial -D /dev/ttyAMA0 -b 921600
```

## Safety

Keep props removed. Do not test localization/fusion on a live vehicle until `/fmu/in/vehicle_visual_odometry` is visible and PX4 estimator status is verified.

## Latest Update - 2026-07-06

DepthAI was upgraded to test whether newer DepthAI releases improve the Basalt timestamp crash.

Previous version:

```text
depthai 3.5.0
```

Upgrade command used:

```bash
python3 -m pip install --user --break-system-packages -U depthai==3.7.1
```

Verified after upgrade:

```text
depthai 3.7.1
devices 1
DeviceInfo(name=2.1, deviceId=19443010C17FDE5900, X_LINK_UNBOOTED, X_LINK_USB_VSC, X_LINK_MYRIAD_X, X_LINK_SUCCESS)
```

`pip show depthai` now reports:

```text
Name: depthai
Version: 3.7.1
Location: /home/john/.local/lib/python3.12/site-packages
```

No VIO/XRCE/Foxglove stack processes were left running after the upgrade.

Important Basalt crash diagnosis from before the upgrade:

```text
data.t_ns 1969202 curr_state.t_ns 5025091
Assertion (data.t_ns > curr_state.t_ns) failed
basalt/imu/preintegration.h:79
Aborted
```

This is a native DepthAI/Basalt abort caused by non-monotonic IMU timestamps reaching Basalt. It is not caused by Foxglove or PX4. The upgrade may or may not fix it; no release note explicitly confirmed this exact Basalt assertion.

Historical next test from immediately after the DepthAI upgrade:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge basalt_vio_px4.launch.py
```

Note: this command no longer starts Basalt after the later RTAB-Map default change. Use `basalt_oak_foxglove.launch.py` for Basalt-only testing.

For the current RTAB-Map main launch, check:

```bash
ros2 topic info /basalt/pose
ros2 topic hz /basalt/pose
ros2 topic info /fmu/in/vehicle_visual_odometry
ros2 topic echo /fmu/out/vehicle_local_position_v1 --once
```

Expected healthy bridge state:

```text
/basalt/pose: Publisher count 1
/fmu/in/vehicle_visual_odometry: Publisher count 1, Subscription count 1
/fmu/out/vehicle_local_position_v1: Publisher count 1
```

If Basalt still dies, likely fallback is RTAB-Map VIO or adding a launch-level Basalt respawn/watchdog. A possible additional Basalt mitigation is changing:

```python
imu.setMaxBatchReports(10)
```

to:

```python
imu.setMaxBatchReports(1)
```

## Latest Update - 2026-07-06 RTAB-Map Default

The OAK-D Lite RTAB-Map VIO path was tested in Foxglove and looked more stable than Basalt on this setup.

Why RTAB-Map looked better:

- RTAB-Map VIO uses stereo rectification, `StereoDepth`, `FeatureTracker`, IMU, and `RTABMapVIO`.
- Basalt uses the OAK-D `BasaltVIO` node directly and has previously been sensitive to non-monotonic IMU/frame timestamps.
- RTAB-Map output rate is lower, about `14-15 Hz`, which appears more stable on this hardware/software path.

Changed files:

```bash
/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/rtabmap/rtabmap_vio_ros2.py
/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/basalt/basalt_vio_ros2.py
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_vio_px4.launch.py
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/rtabmap_oak_foxglove.launch.py
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_oak_foxglove.launch.py
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_odometry_test_foxglove.launch.py
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/px4_vio_bridge/basalt_odometry_test.py
```

RTAB-Map script updates:

- Added CLI args: `--fps`, `--width`, `--height`.
- Publishes `/rtabmap/path`.
- Publishes `/rtabmap/image`.
- Publishes `/rtabmap/odometry`.
- Publishes `/basalt/pose` as a compatibility topic for the existing PX4 bridge.

Basalt script updates:

- Publishes `/basalt/path` every VIO frame instead of every 10 frames.
- Publishes `/basalt/odometry`.

Main launch update:

```bash
ros2 launch px4_vio_bridge basalt_vio_px4.launch.py
```

Despite the legacy filename, this now starts:

- Micro XRCE-DDS Agent on `/dev/ttyAMA0` at `921600`
- OAK-D Lite RTAB-Map VIO
- `vio_to_px4_odometry`
- `px4_local_position_to_ros`
- Foxglove bridge on port `8765`

Default main launch topics:

```text
/rtabmap/path
/rtabmap/odometry
/rtabmap/image
/basalt/pose
/px4/local_position/pose
/px4/local_position/odometry
/px4/local_position/path
/fmu/in/vehicle_visual_odometry
/fmu/out/vehicle_local_position_v1
/fmu/out/vehicle_odometry
/fmu/out/estimator_status_flags
```

Main launch command:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_vio_px4.launch.py
```

RTAB-Map Foxglove-only launch:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge rtabmap_oak_foxglove.launch.py
```

Basalt Foxglove-only launch:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_oak_foxglove.launch.py
```

Observed RTAB-Map test results:

```text
/rtabmap/path: about 14.6-15 Hz
/basalt/pose: about 14.6-15 Hz
/rtabmap/odometry: live samples received
Foxglove channels: /rtabmap/path, /rtabmap/odometry, /rtabmap/image, /basalt/pose
```

DepthAI printed startup warnings like:

```text
Message has too much metadata ... Maximum is 51200B. Dropping message
```

Pose/path/odometry still published successfully after those warnings.

Rebuild used after launch changes:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
colcon build --packages-select px4_vio_bridge
```

Important caveat:

If the main launch is started while the previous RTAB-Map Foxglove-only launch is still running, the OAK-D Lite may already be in use. Stop the old launch first.

Performance update to the main launch:

- `rtabmap_vio_ros2.py` now accepts:

```bash
--publish-image
--path-publish-stride
--path-size
--trajectory-log
--trajectory-flush-stride
```

- Main PX4 launch defaults:

```text
rtabmap_publish_image:=false
rtabmap_path_publish_stride:=10
rtabmap_path_size:=1000
rtabmap_trajectory_log:=none
rtabmap_trajectory_flush_stride:=30
```

- Main Foxglove whitelist no longer includes `/rtabmap/image`.
- `rtabmap_oak_foxglove.launch.py` still defaults image publishing on for debugging.

PX4 local-position Foxglove converter added:

- New node:

```bash
px4_local_position_to_ros
```

- Converts PX4 NED `/fmu/out/vehicle_local_position_v1` into ROS ENU topics:

```text
/px4/local_position/pose
/px4/local_position/odometry
/px4/local_position/path
```

- The main launch starts this node automatically and whitelists those topics for Foxglove.
- Live test confirmed `/px4/local_position/pose` and `/px4/local_position/odometry` publish standard ROS messages.

Yaw-offset tester restored for alignment work:

- PX4 was set back to `EKF2_EV_CTRL=3` and saved over MAVLink USB.
- `vio_to_px4_odometry.py` now accepts `vio_yaw_offset_deg` again, but it also publishes visible debug outputs:

```text
/vio/yaw_offset/pose
/vio/yaw_offset/odometry
/vio/yaw_offset/path
```

- Main launch whitelists those topics for Foxglove.
- Use these topics to see yaw-offset impact directly, independent of whether EKF2 is fusing yaw.
- Test with:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_vio_px4.launch.py vio_yaw_offset_deg:=90.0
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_vio_px4.launch.py vio_yaw_offset_deg:=-90.0
```

- Keep `EKF2_EV_CTRL=3` until the debug yaw-offset pose agrees with Pixhawk/body axes. Only then consider enabling yaw fusion again.

## Latest Update - 2026-07-06 Basalt SLAM Test Removed

Basalt VIO feeding RTAB-Map SLAM was tested after a Pi reboot using:

```bash
/home/john/oak_d_vins_cpp/depthai-core/examples/python/RVC2/VSLAM/basalt/basalt_vio_rtabmap_slam_ros2.py
```

The test briefly advertised Foxglove topics:

```text
/rtabmap/path
/rtabmap/pose
/rtabmap/odometry
/rtabmap/image
/rtabmap/obstacle_cloud
/rtabmap/ground_cloud
/basalt/pose
```

It showed `/rtabmap/path` around `30 Hz` and produced `/rtabmap/odometry`, but the path is not worth keeping:

- Before reboot, the pipeline hit DepthAI `X_LINK_ERROR` stream failures and closed the device connection.
- After reboot, it ran briefly but aborted messily on shutdown with ROS context/native DepthAI errors.
- Basalt remains too fragile on this setup compared with RTAB-Map VIO.

Cleanup done:

- Deleted experimental launch:

```bash
/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/launch/basalt_rtabmap_slam_foxglove.launch.py
```

- Restored `basalt_vio_rtabmap_slam_ros2.py` to its original pre-test shape.

Current recommendation: keep the main launch on RTAB-Map VIO and do not spend more time on Basalt unless specifically debugging DepthAI/Basalt timestamp and shutdown behavior.

## Latest Update - 2026-07-07 xy_valid Diagnosis

Started from: `/fmu/out/vehicle_local_position_v1` had `xy_valid: false`, blocking position control.

Ran the main launch:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge basalt_vio_px4.launch.py
```

Found two separate problems stacked on top of each other:

1. **uXRCE-DDS client stuck disconnected (recurrence of the earlier documented issue).**
   - The launch file's default `xrce_agent` is the project-local v2.4.3 static build. It attached to `/dev/ttyAMA0` (`lsof` confirmed fd open) but produced zero session/log output even at `-v 4` and stayed idle — it was not actually completing a handshake with PX4.
   - Switched to the system agent with `xrce_agent:=/usr/local/bin/MicroXRCEAgent` (v3.0.1). This one logged real activity but got stuck in a `timeout during time synchronization` / `session setup failed, will retry now` loop. PX4 NSH showed `uxrce_dds_client status` -> `Running, disconnected`, `timesync converged: false`.
   - Checked PX4 params over MAVLink USB NSH (`/dev/ttyACM0`, 115200, via `SERIAL_CONTROL` MAVLink shell + pymavlink): `UXRCE_DDS_SYNCC` and `UXRCE_DDS_SYNCT` were both still `1` (enabled).
   - Fix (same as the earlier documented one, re-applied): 
     ```sh
     param set UXRCE_DDS_SYNCC 0
     param set UXRCE_DDS_SYNCT 0
     param save
     uxrce_dds_client stop
     uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600
     ```
   - After this, PX4 immediately created the full `rt/fmu/out/...` data-writer graph and ROS 2 showed `/fmu/out/vehicle_local_position_v1` and `/fmu/in/vehicle_visual_odometry` both with `Publisher count: 1, Subscription count: 1`. `z_valid` became `true`.
   - Takeaway: prefer the system v3.0.1 agent (`/usr/local/bin/MicroXRCEAgent`) over the project-local v2.4.3 build going forward — the v2.4.3 build did not visibly attempt a handshake in this session. Also: `UXRCE_DDS_SYNCC`/`UXRCE_DDS_SYNCT` can silently revert/persist as enabled across sessions and are worth checking first whenever `uxrce_dds_client status` shows `disconnected` with `timesync converged: false`.

2. **`xy_valid` stayed `false` even after the DDS link was healthy, because the OAK-D Lite VIO camera itself was crash-looping.**
   - `rtabmap_vio_ros2.py` logs showed a repeating ~10s cycle: `Couldn't read data from stream ... (X_LINK_ERROR)` -> `Closed connection` -> `Device ... has crashed` (crash dumps written under `ros2_ws/.cache/depthai/crashdumps/`) -> `Reconnection successful` -> crash again.
   - Confirmed `/basalt/pose` had zero messages (`ros2 topic hz /basalt/pose` timed out with nothing received), so `vio_to_px4_odometry` never actually called `.publish()` — its output topic existed (declared publisher) but carried no real data. With no vision source being fused and (presumably) no GPS on this indoor bench setup, EKF2 has nothing to fuse for horizontal position, hence `xy_valid: false`.
   - Ruled out a USB-speed cause: `lsusb -t` shows the OAK-D Lite enumerating on the Bus 003 xhci-hcd SuperSpeed root hub at `5000M`, not downgraded to USB2 (`480M`), so this is not the classic OAK-D "wrong port" bandwidth problem.
   - `python3 -c "import depthai as dai; dai.Device.getAllAvailableDevices()"` returned 0 devices while `rtabmap_vio_ros2.py` was still running/crash-looping, consistent with the process still holding a (crashing) handle to the device rather than the device being absent from the bus.
   - Not yet resolved. This looks like the same category of OAK-D Lite instability already logged above for Basalt (native DepthAI/XLink crashes), just now also affecting the RTAB-Map path. Likely next steps: physically unplug/replug the OAK-D Lite (check USB cable/power, this Pi 5's USB3 ports can brown out sketchy cables/hubs under load), retry after a clean device state, and if it recurs, try lowering `rtabmap_fps` from the 30 default or capture a DepthAI crash dump for inspection (`ros2_ws/.cache/depthai/crashdumps/*.tar.gz`).

Processes left running at end of this session: `MicroXRCEAgent` (system v3.0.1, `/dev/ttyAMA0`), `rtabmap_vio_ros2.py` (crash-looping), `vio_to_px4_odometry`, `px4_local_position_to_ros`, `foxglove_bridge`. PX4 param changes (`UXRCE_DDS_SYNCC=0`, `UXRCE_DDS_SYNCT=0`) were saved to PX4 flash via `param save`.

## Latest Update - 2026-07-07 xy_valid RESOLVED

`xy_valid` is now **`true`**. Two follow-on problems from the section above were fixed, and then the real root cause was found.

### 1. OAK-D Lite crash-loop fixed by power cycle
The `X_LINK_ERROR` crash-loop cleared after the user physically power-cycled the OAK-D Lite (the whole Pi rebooted). After reboot, `/basalt/pose` (RTAB-Map VIO) was rock-solid: measured 256 samples over 25 s, mean gap 97.8 ms, max 126.6 ms, **zero gaps > 200 ms**. So the VIO source is stable; the earlier crash-loop was a transient device/USB state, not a persistent fault.

### 2. uXRCE-DDS reconnect (same procedure as before)
After reboot the client again came up `Running, disconnected`. Fixed with the documented sequence over NSH (`UXRCE_DDS_SYNCC 0`, `UXRCE_DDS_SYNCT 0` were already saved; just needed `uxrce_dds_client stop` then `start -t serial -d /dev/ttyS2 -b 921600`). Note: `timesync converged: false` is shown **even when the session is fully connected and working** — with `SYNCT=0`, PX4 falls back to stamping inbound messages with local reception time, which is fine. Confirmed that re-enabling `SYNCT=1` makes the timesync handshake time out and the session never connects on this serial link, so `SYNCT=0` must stay.

### 3. ROOT CAUSE of xy_valid=false: corrupted `EKF2_EV_CTRL` param
Even with the DDS link healthy and valid VIO streaming into `/fmu/in/vehicle_visual_odometry` at ~10 Hz, EKF2 computed EV innovations every cycle (`estimator_aid_src_ev_pos` populated, `innovation_rejected: false`, tiny test ratios) but **never started fusion** (`fused: false`, `time_last_fuse: 0`, `cs_ev_pos: false`, event flag `starting_vision_pos_fusion: false`, EKF stuck in `const_pos_mode`).

The cause: **`EKF2_EV_CTRL` was stored as `1077936128`, not `3`.** `1077936128` = `0x40400000` = the IEEE-754 float bit pattern of `3.0`. `EKF2_EV_CTRL` is an INT32 bitmask (bit0=HPOS, bit1=VPOS, bit2=VEL, bit3=YAW); `1077936128 & 0b1111 == 0`, so **no EV control bit was set** and EKF2 did zero EV fusion. This happened because the param was previously "set to 3" over **MAVLink using REAL32 (float) encoding** for an INT32 param, so PX4 stored the float's raw bits as the integer.

Diagnostic tell: on NSH, `param show EKF2_EV_CTRL` printed `1077936128` while every other int param printed clean (`EKF2_HGT_REF: 3`, `EKF2_GPS_CTRL: 0`, `EKF2_MULTI_IMU: 1`). A MAVLink `PARAM_VALUE` dump **hides this** — it reads the int32 back as float and shows `3.0`, which looks correct. Always verify EKF2 int params via the NSH `param show`, not a MAVLink/QGC float readout.

**Fix (set the integer correctly via NSH, not MAVLink float):**
```sh
param set EKF2_EV_CTRL 3
param save
ekf2 stop
ekf2 start
```
Immediately after: `estimator_event_flags` showed `reset_pos_to_vision: true`, `starting_vision_pos_fusion: true`; `ekf2 status` went to `local position: 1`; both `estimator_aid_src_ev_pos` and `estimator_aid_src_ev_hgt` show `fused: true`; and `/fmu/out/vehicle_local_position_v1` now reports `xy_valid: true`, `v_xy_valid: true`, `z_valid: true` (stable across reads, position tracks camera motion).

### Bridge code change
`vio_to_px4_odometry.py`: `angular_velocity` and the first-sample/out-of-range velocity now publish `0.0` instead of `NaN` (finite-sample hygiene for EKF ingestion). Rebuilt with `colcon build --packages-select px4_vio_bridge`. (This was not the root cause but is correct to keep.)

### PX4 param backup
Full backup of all 1127 params saved to `params_backup_20260707_*.params` via new helper `scripts/dump_params.py`. NOTE: that dump reads int params as float over MAVLink, so it does not reveal the `EKF2_EV_CTRL` corruption — it's a restore backup, not a source of truth for int-param values.

### Persistent PX4 params now saved to flash
`UXRCE_DDS_SYNCC=0`, `UXRCE_DDS_SYNCT=0`, `EKF2_EV_CTRL=3` (corrected), `EKF2_HGT_REF=3` (Vision).

### Current running processes (healthy)
`MicroXRCEAgent` (system v3.0.1 on `/dev/ttyAMA0`, launched via `xrce_agent:=/usr/local/bin/MicroXRCEAgent`), `rtabmap_vio_ros2.py` (stable ~10 Hz), `vio_to_px4_odometry` (rebuilt), `px4_local_position_to_ros`, `foxglove_bridge`. PX4 `uxrce_dds_client` `Running, connected`.

## Latest Update - 2026-07-07 EV YAW FUSION ENABLED + VALIDATED

External-vision **yaw** fusion is now enabled and validated on the bench (props off).

### Change
- `EKF2_EV_CTRL`: `3` (HPOS+VPOS) -> **`11`** (HPOS=1 + VPOS=2 + YAW=8). Set as a correct **integer** via NSH `param set EKF2_EV_CTRL 11; param save`, then `ekf2 stop; ekf2 start`. Verified with NSH `param show EKF2_EV_CTRL` -> `: 11` (saved). Do NOT set this over MAVLink/QGC float — see the int-param float-corruption note above; this session set it via NSH so it stored cleanly.

### NSH access finally working from the Pi (important tooling win)
New helper: `scripts/nsh.py` runs PX4 NSH commands over MAVLink `SERIAL_CONTROL` on `/dev/ttyACM0` @115200 (uses `.venv-mavlink`). Usage: `.venv-mavlink/bin/python scripts/nsh.py "param show EKF2_EV_CTRL" "ekf2 status"`. Each arg is one NSH line.
- The prior blocker was the SERIAL_CONTROL flags: must send `RESPOND(2)|EXCLUSIVE(4)|MULTI(16)`, and must **NOT** set `REPLY(1)` (that marks the frame as an autopilot reply and PX4 ignores it as a command).
- pymavlink on this box occasionally throws a `_instances`/`NoneType` parse error on connect — just retry (the helper's callers loop up to 3x).
- Note: `/dev/ttyACM0` is single-reader. Don't run a MAVLink monitor (e.g. yaw watcher) and `nsh.py` at the same time — they conflict on the port.

### Reading int params over MAVLink without NSH (verified this session)
PX4 packs INT32 params into the PARAM_VALUE float field by **bit-reinterpret**. To recover the true int from pymavlink's `param_value` float: `struct.unpack('<i', struct.pack('<f', v))[0]`. Confirmed `EKF2_EV_CTRL`(type 6/INT32) read back as int `3` before the change while raw float showed `4.2e-45` (= bits `0x00000003`). This is the non-NSH way to check int params correctly.

### Validation results (EKF2_EV_CTRL=11)
- `estimator_aid_src_ev_yaw`: `fused: True`, `innovation ~0.01-0.04 rad`, `test_ratio < 0.003`, `innovation_rejected: False`.
- `vehicle_local_position`: `heading` switched from mag (`3.13 rad`) to vision (`~1.54 rad`, = EV observation); `heading_var` dropped `0.00115 -> 0.00058`; **`heading_good_for_control` flipped `False -> True`**. `xy_valid`/`z_valid`/`v_xy_valid` stayed true; `ekf2 status` `local position: 1`, healthy. `heading_reset_counter` bumped 1->3 (expected reset-to-vision on fusion start).
- **Physical rotation test:** rotated the drone in place; PX4 fused heading (watched live via MAVLink `ATTITUDE.yaw`, helper `scratchpad/yaw_watch.py`) tracked smoothly out to ~180deg and back to baseline with correct magnitude and correct direction, no jumps. Sign/frame convention in `vio_to_px4_odometry` (ENU/FLU -> NED/FRD) is therefore correct for yaw.

### Persistent PX4 params now (flash)
`UXRCE_DDS_SYNCC=0`, `UXRCE_DDS_SYNCT=0`, `EKF2_HGT_REF=3` (Vision), `EKF2_EV_CTRL=11` (HPOS+VPOS+YAW), `EKF2_GPS_CTRL=0`.

### Notes / next steps
- DDS came up `Running, connected` on this session's launch without needing the `uxrce_dds_client stop/start` dance (FCU had not rebooted).
- Still bench-only. Before any flight, re-confirm `heading_good_for_control: True` and that EV yaw innovation stays low during sustained motion; watch for VIO yaw drift over longer runs.
