# Autonomous Drone PX4 VIO

ROS 2 Jazzy workspace for sending OAK-D Lite VIO into PX4 visual odometry.

Current default: RTAB-Map VIO/SLAM from the OAK-D Lite publishes its continuous VIO
pose on `/rtabmap/vio_pose`; `px4_vio_bridge` converts that pose into PX4
`VehicleOdometry` on `/fmu/in/vehicle_visual_odometry`.

DepthAI is pinned to 3.5.0. Version 3.7.1 repeatedly crashes this OAK-D Lite's
CAM_B during device-side MIPI startup. Install the validated version with:

```bash
cd /home/john/autonomous_drone_px4_vio
python3 -m pip install --user --break-system-packages \
  -r requirements-depthai.txt
python3 -c "import depthai; print(depthai.__version__)"  # must print 3.5.0
```

## Main Launch

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py
```

It starts:

- OAK-D Lite RTAB-Map VIO
- `vio_to_px4_odometry`
- PX4 local-position-to-ROS converter
- Foxglove bridge on port `8765`

The Micro XRCE-DDS Agent is owned by systemd, independently of the ROS launch. See
"Micro XRCE-DDS Agent ownership" below.

Performance defaults in the main launch:

- `/rtabmap/depth` publishing is disabled.
- The compressed RGB camera feed is enabled and available to Foxglove by default;
  disable it with `slam_publish_image:=false`.
- Point clouds are available to Foxglove when enabled with
  `slam_publish_clouds:=true`.
- RTAB-Map VIO defaults to `slam_num_features:=400`. A target of 1000 produced
  about 59 KB of tracked-feature metadata at 30 Hz, exceeding DepthAI's
  51,200-byte XLink metadata limit; 700 also left VIO stuck at identity in live testing.
- `/rtabmap/path` publishes every 10 odometry poses.
- `/rtabmap/path` is capped at 1000 poses.

EV yaw fusion (enabled 2026-07-07):

- PX4 now fuses external-vision **yaw** as well as position: `EKF2_EV_CTRL=11` (bit0 HPOS + bit1 VPOS + bit3 YAW), `EKF2_HGT_REF=3` (Vision).
- Validated on the bench: `estimator_aid_src_ev_yaw.fused=True` with tiny innovation, `heading_good_for_control` went `False -> True`, and PX4 heading tracked a physical rotation with correct sign/magnitude. Vision is now the only heading aid (mag heading fusion is inhibited while EV yaw is active).
- Set this param as a true **integer** via the NSH shell, never as a MAVLink/QGC float (a float set stores the IEEE-754 bit pattern as the int and silently disables the bits). See "NSH Access" below.

Yaw-offset tester (kept for re-alignment work):

- Relaunch with `vio_yaw_offset_deg:=90.0` or `vio_yaw_offset_deg:=-90.0`.
- Compare raw `/rtabmap/vio_pose` with corrected `/vio/yaw_offset/pose` and `/vio/yaw_offset/path` in Foxglove.
- The corrected VIO pose is what the bridge sends toward `/fmu/in/vehicle_visual_odometry`.

Foxglove URL:

```text
ws://<pi-ip>:8765
```

Useful topics:

```text
/rtabmap/path
/rtabmap/odometry
/rtabmap/vio_pose
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

This Foxglove-only launch keeps the camera feed enabled for debugging.

### RTAB-Map VIO + RTAB-Map SLAM

Use this comparison launch to feed `RTABMapVIO` odometry into `RTABMapSLAM` and publish
the SLAM pose, path, image, and optional obstacle/ground point clouds:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  rtabmap_vio_slam_foxglove.launch.py \
  slam_publish_clouds:=true
```

The raw VIO pose is on `/rtabmap/vio_pose`; SLAM-corrected outputs are on
`/rtabmap/pose`, `/rtabmap/odometry`, and `/rtabmap/path`.

### RTAB-Map SLAM + PX4 bridge

Use the combined Raspberry Pi launch to run RTAB-Map VIO/SLAM, the PX4
visual-odometry bridge, PX4 local-position visualization, and one Foxglove bridge.
The serial XRCE-DDS agent is normally already running as a systemd service:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  rtabmap_slam_px4.launch.py \
  slam_publish_clouds:=true
```

Important pose distinction:

- **PX4 does not receive the loop-corrected SLAM pose.** The PX4 bridge consumes the
  continuous raw VIO pose on `/rtabmap/vio_pose` and publishes it to
  `/fmu/in/vehicle_visual_odometry`. This avoids injecting loop-closure position/yaw
  jumps into EKF2.
- **Foxglove's SLAM visualization is loop-corrected.** `/rtabmap/pose`,
  `/rtabmap/odometry`, and `/rtabmap/path` use RTAB-Map's optimized SLAM frame.
- `/px4/local_position/*` shows PX4/EKF2's estimated vehicle position. It should be
  compared with `/rtabmap/vio_pose`, not expected to follow a later SLAM loop closure.

The `input_pose_topic` launch argument can select `/rtabmap/pose` experimentally, but
feeding a discontinuous loop-corrected pose to PX4 is not the flight default.

### Micro XRCE-DDS Agent ownership

Exactly one process may own `/dev/ttyAMA0`. The normal flight configuration uses
the system v3.0.1 agent as a systemd service; `rtabmap_slam_px4.launch.py` therefore
defaults to `start_xrce_agent:=false`. Starting a second agent from ROS while the
service is active causes serial-port contention and can leave PX4 DDS disconnected.

Install and start the supplied service once:

```bash
cd /home/john/autonomous_drone_px4_vio
sudo install -m 0644 systemd/micro-xrce-agent.service \
  /etc/systemd/system/micro-xrce-agent.service
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
systemctl status micro-xrce-agent.service
```

Inspect its logs with:

```bash
journalctl -u micro-xrce-agent.service -f
```

For a temporary launch-owned-agent fallback, stop the service first, then opt in:

```bash
sudo systemctl stop micro-xrce-agent.service
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  rtabmap_slam_px4.launch.py start_xrce_agent:=true
```

Do not use `start_xrce_agent:=true` while the service is active. The legacy
`basalt_vio_px4.launch.py` still starts an agent unconditionally, so stop the
systemd service before using that launch until it is migrated to the same ownership
model.

### Low-latency camera feed

The camera feed defaults to **JPEG-compressed** `sensor_msgs/CompressedImage` on
`/rtabmap/image/compressed`, published on a **best-effort, keep-last-1** QoS. This is what
keeps Foxglove real-time: a 640x400 mono frame drops from ~256 KB raw to ~15-25 KB
(~15x less over the WebSocket), and stale frames are dropped instead of queued — queueing
is what makes the raw feed's delay grow without bound. Add an **Image** panel in Foxglove
and point it at `/rtabmap/image/compressed`.

Tuning knobs (launch args, apply to both `rtabmap_oak_foxglove.launch.py` and
`basalt_vio_px4.launch.py`):

- `rtabmap_image_format:=jpeg` (default) or `raw` (legacy `/rtabmap/image`, heavy — backs up).
- `rtabmap_image_jpeg_quality:=60` (1-95; lower = smaller/faster).
- `rtabmap_image_publish_stride:=1` (publish every Nth frame, e.g. `2` to halve the rate).

Still laggy? Drop quality (`:=40`), raise the stride (`:=2`), or lower `rtabmap_width`/`rtabmap_height`.
The pose/VIO path is unaffected — the image now uses a non-blocking `tryGet`, so the feed can
never stall the `/basalt/pose` stream that feeds PX4.

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

### Measure EV/VIO delay

With props removed and the main launch running, make several distinct hand-yaw reversals while this
helper records EV yaw and PX4 gyro Z:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
python3 scripts/measure_ev_fusion_delay.py --duration 30
```

Two captures on 2026-07-16 measured 245 ms and 270 ms, giving a practical current estimate of
about **260 ms** (overlapping peak-width 195-320 ms). `EKF2_EV_DELAY` remains `0.0`; the measurement
did not change PX4 parameters. The helper's gyro-referenced result is the relevant one because EKF
IMU propagation can make `vehicle_local_position` lead the delayed vision observation.

### Calibrate the EV sensor position offset

`EKF2_EV_POS_X/Y/Z` are the external-vision sensor position relative to the vehicle/FC
origin in body **FRD** coordinates: positive X forward, Y right, Z down. To estimate the
lever arm from motion rather than a ruler, remove the props, start the main launch, and run:

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
python3 scripts/calibrate_ev_position_offset.py --duration 35 --delay-ms 260
```

Hold the flight controller near one point and repeatedly rock the whole drone through roll,
pitch, and yaw. Use quick, smooth rotations on all three axes and keep translation slow. The
script time-aligns `/fmu/in/vehicle_visual_odometry` with FC attitude from
`/fmu/out/vehicle_odometry`, removes slow hand motion in short windows, and fits the three
offsets. It is read-only: it only prints suggested parameters and NSH commands.

Repeat the capture at least twice and only use values that agree. Reject any run with warnings
about excitation, residual, conditioning, or uncertainty. The existing nominal values in
`mav.parm` are X=0.100 m, Y=0.000 m, Z=0.038 m, which are also a useful physical sanity check.
Set accepted values through NSH and then `param save`; do not set PX4 parameters through this
calibration script. Keep the props removed for the entire procedure.

## NSH Access (PX4 shell from the Pi)

`scripts/nsh.py` runs PX4 NSH commands over MAVLink `SERIAL_CONTROL` on the Pixhawk USB link (`/dev/ttyACM0` @115200), using the `.venv-mavlink` interpreter. Each argument is one NSH command line:

```bash
cd /home/john/autonomous_drone_px4_vio
.venv-mavlink/bin/python scripts/nsh.py "param show EKF2_EV_CTRL" "ekf2 status"
.venv-mavlink/bin/python scripts/nsh.py "param set EKF2_EV_CTRL 11" "param save"
.venv-mavlink/bin/python scripts/nsh.py "uxrce_dds_client status"
```

Notes:

- Use NSH (not MAVLink/QGC) to set/verify integer EKF2 params; MAVLink float encoding corrupts them.
- `/dev/ttyACM0` is single-reader: don't run another MAVLink client at the same time as `nsh.py`.
- pymavlink occasionally throws a `_instances` NoneType error on connect; just retry (callers loop up to 3x).

## Autonomous Hover (Offboard)

Node `offboard_hover` (`px4_vio_bridge`) flies a short position-controlled hover over the existing uXRCE-DDS link: latch current NED x/y/yaw, stream `TrajectorySetpoint` at 50 Hz, request OFFBOARD, arm, climb to `hover_height`, hold `hold_time`, then `NAV_LAND` (auto-disarms on ground detect). Requires healthy EV pos+yaw fusion (`xy_valid`, `z_valid`, `heading_good_for_control` all true).

Safety design:

- `auto_arm` defaults to **false**: the node runs the whole sequence but never sends an arm command, so you can dry-run (props off) and confirm setpoint streaming + the OFFBOARD request without flying.
- Aborts (never arms) if OFFBOARD+ARM are not confirmed via `vehicle_control_mode` within `engage_timeout`.
- `max_flight_time` watchdog forces LAND; lost local position in flight forces LAND; Ctrl-C while armed commands AUTO.LAND (never a mid-air disarm).
- **Tracking-loss landing:** while armed, the node monitors `/rtabmap/vio_pose` and `/rtabmap/vio_feature_count`. A stale VIO pose (default `0.75 s`), stale feature data (`1.0 s`), or fewer than 15 tracked features for `1.0 s` commands AUTO.LAND. The first second after arming is a grace period.
- **Keyboard land:** while the command's terminal has focus, press **L** (no Enter) to request AUTO.LAND.
- **Keyboard kill:** while the command's terminal has focus, press **K** (no Enter). The node immediately sends PX4's forced-disarm command repeatedly for one second. This is a true motor kill, not a landing command; using it airborne will make the vehicle fall.
- Fly only with an RC transmitter bound as manual-override / kill switch (`COM_RC_IN_MODE=0`).

Dry run (props off) — walks `STREAM -> ENGAGE -> CLIMB_HOLD -> LAND -> DONE` without arming:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash && source /home/john/ros2_ws/install/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=false -p hover_height:=0.30 -p hold_time:=10.0
```

Live flight (props on, area clear, RC ready, hand on the kill switch):

```bash
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=true -p hover_height:=0.30 -p hold_time:=10.0
```

At startup, verify the terminal prints `KEYBOARD CONTROLS: press L for AUTO.LAND; press K to
FORCE-DISARM immediately`. If stdin is redirected or the node is started without an interactive
terminal, both keyboard controls are unavailable. They can be disabled independently with
`-p keyboard_land:=false` and `-p keyboard_kill:=false`. The Pi-side controls and automatic
tracking-loss landing rely on the ROS process and TELEM2/DDS link, so they are not replacements for
the independent RC kill switch.

Tracking-loss thresholds are configurable with `vio_pose_timeout`, `vio_feature_timeout`,
`min_vio_features`, `vio_feature_loss_time`, and `tracking_arm_grace`. Disable this monitor only for
diagnosis with `-p tracking_loss_land:=false`.

Pre-flight gate (all must be green, via `scripts/nsh.py`):

- `uxrce_dds_client status` -> `Running, connected`
- `listener vehicle_local_position 1` -> `xy_valid`/`z_valid`/`v_xy_valid` true, `heading_good_for_control` true
- `listener estimator_status_flags 1` -> no `reject_*`, no `fs_*`, `cs_ev_pos`/`cs_ev_yaw` true

Notes:

- `hover_height=0.30` m is very low; height is pure vision (no rangefinder) and ground effect can disturb VIO features near the floor. Confirm Z holds steady in the dry run; consider `0.5-0.6` m if VIO gets jittery low.

## Known Issues & Operational Notes

### TELEM2 / DDS link is marginal (works USB-free, but near the edge)

The PX4 <-> Pi comms path is TELEM2 (`/dev/ttyAMA0`, 921600) carrying uXRCE-DDS; the Pixhawk USB (`/dev/ttyACM0`) is dev/debug only. Observed 2026-07-09:

- With the USB cable **unplugged**, a cold-started uXRCE session sometimes got stuck in a reset loop: the agent re-created its whole datawriter graph repeatedly and no PX4 telemetry reached ROS (`px4_local_position_to_ros` published 0 messages, `vehicle_local_position` probe got 0).
- Plugging the USB cable back in (no software restart) let the session establish and stream; it then kept streaming at a full ~50 Hz **even after the USB was unplugged again**, and it also came up cleanly USB-free across several Pixhawk power-cycles.
- Conclusion: the fragile phase is **session establishment**, not steady state. Once established the link tolerates errors and is robust. This points to a **marginal serial link** (signal-integrity / grounding margin — USB's ground bond helps at the edges, but is not strictly required). Working theory: an unreliable TELEM2 UART cable.

Recommendations before flight:

- Replace the TELEM2 harness with a short shielded/twisted cable (twist TX/RX with GND, solid common ground).
- Consider lowering `SER_TEL2_BAUD` to `460800` (and match the agent `-b`) for timing margin.
- On the real vehicle the Pi and Pixhawk share the battery/BEC ground, so TELEM2 should be stable USB-free — but verify DDS stability on battery power, USB unplugged, before trusting it in the air.
- Note: removing USB also removes the NSH/MAVLink shell (`scripts/nsh.py`), so the documented `uxrce_dds_client stop; start` recovery is unavailable USB-free. USB-free recovery = physical Pixhawk power-cycle.

Health check for a stable session (log-based, no ROS CLI needed):

```bash
# session is stable if create_datawriter stops churning and px4_local_position keeps publishing
grep -c "create_datawriter" <stack-log>          # should stop increasing once established
grep -c "Published PX4 local position" <stack-log> # should keep increasing (each log line = 100 msgs)
```

### VIO "shake to align" at startup is normal VIO initialization

On cold boot the camera (VIO) frame and the FC body frame appear rotated apart, and the drone must be moved ("shaken") before they align. This is expected visual-inertial behavior, not a bug:

- **Roll/pitch** of the VIO frame come from gravity (IMU) and are correct almost immediately.
- **Yaw is arbitrary**: RTAB-Map sets its world-frame yaw origin to wherever the camera pointed at the first keyframe (no north reference).
- VIO also needs **motion (excitation)** to converge (IMU bias, camera-IMU alignment, scale). Until moved, its pose is unreliable.
- Moving the drone converges VIO and produces a valid EV yaw; EKF2 then does `reset_yaw_to_vision` and snaps PX4 heading onto the vision yaw -> aligned.

Two effects to keep separate:

1. **Dynamic init (the motion requirement)** — inherent to VIO; fixed by *procedure*, not a param.
2. **Static camera-mount offset** — a constant rotation if the OAK-D is bolted on rotated vs the FC. This would be a repeatable offset that motion does NOT remove; bake it into the bridge `vio_yaw_offset_deg` (and check pitch/roll if the camera is tilted).

Test to tell them apart: after motion aligns it, is it then correct and stable? Yes -> pure init, mount offset ~0. Repeatable residual offset -> calibrate `vio_yaw_offset_deg`.

Recommended bringup (make init a checklist step, not a live-prop shake):

- Before arming, pick the drone up and give it a smooth few seconds of translation + gentle yaw in each axis, then set it down.
- Boot facing textured, well-lit scene (not a blank wall) so RTAB-Map locks quickly.
- Confirm VIO<->PX4 alignment in Foxglove **before** props spin. Roll/pitch should be right immediately; if they are also wrong and motion doesn't fix them, that's a camera-IMU extrinsic problem in the RTAB-Map config (separate issue).

## Safety

Keep props removed for all bench/estimator work. Do not test estimator fusion on a live vehicle until PX4 is receiving visual odometry and estimator status is verified. For any powered/offboard flight: props on only when spotting, RC transmitter bound as manual override / kill switch, clear area, and only after the offboard dry run (`auto_arm:=false`) has passed and the pre-flight gate above is green.
