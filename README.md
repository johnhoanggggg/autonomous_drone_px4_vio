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

EV yaw fusion (enabled 2026-07-07):

- PX4 now fuses external-vision **yaw** as well as position: `EKF2_EV_CTRL=11` (bit0 HPOS + bit1 VPOS + bit3 YAW), `EKF2_HGT_REF=3` (Vision).
- Validated on the bench: `estimator_aid_src_ev_yaw.fused=True` with tiny innovation, `heading_good_for_control` went `False -> True`, and PX4 heading tracked a physical rotation with correct sign/magnitude. Vision is now the only heading aid (mag heading fusion is inhibited while EV yaw is active).
- Set this param as a true **integer** via the NSH shell, never as a MAVLink/QGC float (a float set stores the IEEE-754 bit pattern as the int and silently disables the bits). See "NSH Access" below.

Yaw-offset tester (kept for re-alignment work):

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
- Aborts (never arms) if OFFBOARD+ARM are not confirmed via `vehicle_status` within `engage_timeout`.
- `max_flight_time` watchdog forces LAND; lost local position in flight forces LAND; Ctrl-C while armed commands AUTO.LAND (never a mid-air disarm).
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
