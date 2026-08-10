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
- RTAB-Map VIO defaults to the live-tested `slam_num_features:=400`; 700 left
  VIO stuck at identity under the same motion, but the cause was not isolated.
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
- `/rtabmap/odom_correction` is DepthAI RTAB-Map's native map-to-odometry
  correction. The observation-only route follower consumes it directly and
  fails closed on stale, non-finite, or oversized corrections and unhealthy raw
  VIO. It is not added to TF or sent to PX4.
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
about **260 ms** (overlapping peak-width 195-320 ms). `EKF2_EV_DELAY` is now saved at `270.0 ms`.
The helper's gyro-referenced result is the relevant one because EKF IMU propagation can make
`vehicle_local_position` lead the delayed vision observation.

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
about excitation, residual, conditioning, or uncertainty. The values verified live on 2026-07-25
are X=+0.100 m, Y=-0.036 m, Z=+0.056 m in body FRD.
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
- **Tracking-loss landing:** while armed, the node monitors the raw VIO pose, feature count, bridge output, VIO/EKF yaw agreement, gyro yaw rate, and horizontal hold error. Defaults trip on fewer than 160 features for 0.25 s, more than 20 degrees yaw disagreement for 0.20 s, yaw rate above 90 deg/s for 0.10 s, or hold error above 0.35 m for 0.25 s. The first second after arming is a grace period.
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

## Battery Indicator in Foxglove

`battery_to_ros` flattens `/fmu/out/battery_status_v1` (a `px4_msgs/BatteryStatus`,
which Foxglove can only render as raw fields, and whose `remaining` is 0..1) into
plain `std_msgs` that Foxglove panels bind to directly. It starts automatically with
`rtabmap_slam_px4.launch.py`; disable with `battery_monitor:=false`.

These companion topics are display/logging telemetry only. They do not inhibit
planner arming or trigger planner HOLD/LAND; PX4 remains the sole authority for
battery arming checks and low-battery failsafe actions.

| Topic | Type | Panel |
|---|---|---|
| `/battery/percent` | `Float32` (0–100) | **Gauge**, min 0 max 100 |
| `/battery/voltage` | `Float32` (V) | Gauge or Plot |
| `/battery/cell_voltage` | `Float32` (V/cell) | Gauge — the honest signal under load |
| `/battery/current` | `Float32` (A) | Plot |
| `/battery/power` | `Float32` (W) | Plot |
| `/battery/level` | `Int32` 0–3 | **Indicator** (0 OK, 1 LOW, 2 CRITICAL, 3 EMPTY) |
| `/battery/status` | `String` | Raw Message — e.g. `OK 100% 12.12V (4.04V/cell) 0.0A 0W` |

Recommended: a **Gauge** panel on `/battery/percent` field `data`, plus an **Indicator**
panel on `/battery/level` field `data` with rules for 0/1/2/3.

`level` is the **worse** of three sources — the percent thresholds, the per-cell
voltage thresholds, and PX4's own `warning` enum — so an optimistic state-of-charge
estimate can never mask a real low-voltage warning. Thresholds are launch arguments:
`battery_warn_percent` (40), `battery_critical_percent` (25), `battery_empty_percent`
(15). Level escalations are also logged to the flight bag via `/rosout`.

Invalid PX4 fields are dropped rather than published: PX4 signals "unknown" with
`voltage_v=0`, `current_a=-1`, `remaining=-1`, and putting those on a gauge would read
as 0 V / −100%.

## Interactive Waypoints from Foxglove

Node `offboard_waypoint` (`px4_vio_bridge`) flies to points you click in the Foxglove
3D panel. It subclasses `OffboardHover`, so engagement, the VIO tracking-loss
watchdogs, K/L keyboard controls and `max_flight_time` all behave identically.

Foxglove has **no** RViz-style `InteractiveMarker` support — there are no draggable
6-DOF handles. The interaction is the 3D panel's **Publish** tool: click a point, it
publishes a `geometry_msgs/PointStamped` on `/waypoint/clicked`.

### One-time Foxglove bridge change

`rtabmap_slam_px4.launch.py` previously ran the bridge with
`capabilities:=[connectionGraph]` and `client_topic_whitelist:=['^$']`, which blocks
all publishing from the browser. It now defaults to:

- `foxglove_capabilities:=[clientPublish,connectionGraph]`
- `foxglove_client_topic_whitelist:=['^/waypoint/clicked(_pose)?$','^/planner/flight/teleop$']`

The narrow client whitelist is deliberate: a browser tab may reach the waypoint
intake topics and the validated flight-control intake, but nothing else. Do
**not** widen it to `['.*']` — that would put `/fmu/in/*` (raw setpoints and arm
commands) within reach of anything that can open the WebSocket.

### Foxglove LAND / emergency KILL controls

The global-planner flight launch subscribes to `/planner/flight/teleop` using
Foxglove's built-in Teleop panel `geometry_msgs/msg/Twist` schema. Configure a
dedicated panel as follows:

- Topic: `/planner/flight/teleop`
- Publish rate: `1 Hz`
- Stop on release: **disabled**
- Center Stop button: field `linear.z`, value `-1` (**controlled AUTO.LAND**)
- Down button: field `linear.z`, value `-2` (**EMERGENCY KILL / motors stop**)
- Up, Left and Right: value `0` (no action)

Rename the panel `Flight Safety — STOP=LAND, DOWN=KILL`. A zero Twist and button
release are deliberately inert. LAND is ignored while disarmed by the existing
flight state machine; browser KILL is also ignored while disarmed. The browser
control supplements rather than replaces the independent RC kill switch.

### Foxglove panel setup

1. 3D panel → set the **display frame** to `world`. The publish tool stamps whatever
   frame the panel is following, and the node rejects anything that is not `world`.
2. Open the panel's **Publish** tool, topic `/waypoint/clicked`, type
   `geometry_msgs/PointStamped`.
3. Add topics `/waypoint/target` and `/waypoint/commanded` to the 3D panel to see the
   accepted goal and the rate-limited point actually being sent to PX4.
4. Add a Raw Message panel on `/waypoint/status` for accepted/rejected counts,
   distance to go, live hold error against its current limit, and the idle countdown.

The planner publishes its effective parameters, including launch overrides, as
latched JSON on `/planner/config`; the route follower does the same on
`/planner/follower/config`. Both topics are included in planner and flight bags,
so values such as `robot_radius`, `safety_margin`, `lookahead`, and
`max_cross_track` can be recovered during analysis even when those nodes started
before the recorder.

To also command heading, publish a `PoseStamped` on `/waypoint/clicked_pose` and launch
with `accept_waypoint_yaw:=true`. Off by default — yawing while translating is the
combination that stresses VIO hardest.

### Running it

Stack first (in one terminal):

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash && source /home/john/ros2_ws/install/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py
```

Props-off DRY RUN (never arms; the whole click → geofence → slew → setpoint path is
exercised and recorded, so you can verify it in Foxglove and in the bag before flying):

```bash
ros2 launch px4_vio_bridge offboard_waypoint.launch.py auto_arm:=false climb_timeout:=5.0
```

LIVE (props on, RC bound as kill, area clear, pre-flight gate green):

```bash
ros2 launch px4_vio_bridge offboard_waypoint.launch.py auto_arm:=true
```

Both record a flight bag to `flight_logs/offboard_waypoint_<UTC>` with the same
`fastwrite` MCAP profile and `.launchinfo` sidecar as the yaw test. `/waypoint/*` is in
the bag, so a session can be replayed click by click.

### What bounds a click

A click is a network message from a browser, so nothing it says is trusted:

| Guard | Default | Behavior |
|---|---|---|
| `waypoint_frame` | `world` | wrong `frame_id` → rejected, vehicle does not move |
| `geofence_radius` | 1.5 m | target clamped into a disc around the **latched takeoff point** |
| click z | — | **ignored**; altitude is always `hover_height` |
| `waypoint_speed` | 0.25 m/s | the setpoint slews; PX4 never sees a position step |
| `idle_timeout` | 20 s | parked at a waypoint with nothing pending → AUTO.LAND |
| `arrival_tol` | 0.12 m | arrival is **latched** once reached, not re-tested each tick |
| `max_flight_time` | 90 s | armed watchdog → LAND (launch default, vs 40 s for the yaw test) |
| `min_vio_features` | 80 | tracking-loss floor — **lower than the 160 used by the hover/yaw tests**, see below |

`min_vio_features` is 80 in this launch rather than 160. Waypoint flight translates and
repoints the camera at whatever the room offers, so it samples worse scenes than a
station-keeping hover does; a 2026-07-27 bench run aborted at 134 features. This buys
tolerance, not tracking quality — if counts are routinely near the floor, fix the scene
(lighting, texture, no blank walls), because low-feature VIO is what drifts. The hover
and yaw tests keep 160.

Clicks are **absolute positions in `world`**, not offsets from the drone. A click
outside the geofence is not ignored — it is pulled onto the fence boundary along its
own bearing, and the log says `(CLAMPED to geofence)`.

Arrival is latched, not re-tested every tick. The 2026-07-27 flight held station
0.135 m from target on average against an `arrival_tol` of 0.12 m, so an
instantaneous test flickered, reset the idle clock every few ticks, and the idle
timeout could never fire. Once the vehicle touches `arrival_tol` it counts as
arrived until the next click, and `/waypoint/status` reports `arrived=True/False`.

### The horizontal-error gate during transit

`OffboardHover` measures hold error from the latched takeoff point. Left alone that
would land a waypoint flight the moment it successfully translated, so `hold_point`
and `horizontal_error_limit` are now overridable properties; `offboard_waypoint`
points them at the commanded position.

PX4 trails a moving position setpoint by roughly `waypoint_speed / MPC_XY_P` — about
0.26 m at the defaults, which sits right on top of the 0.35 m hold gate. So the tight
gate applies only once the commanded point has been stationary for
`transit_settle_time` (1.0 s); during transit the looser `transit_horizontal_error`
(0.60 m) applies. Raising `waypoint_speed` raises that lag proportionally — raise
`transit_horizontal_error` with it, or turn on `velocity_feedforward:=true`, which
publishes the slew velocity in `TrajectorySetpoint.velocity` and cancels most of the
lag. Feedforward is opt-in and has not been flown.

## Square Path (scripted)

Node `offboard_square` flies a closed square: **go straight, turn, go straight, turn,
go straight, turn, go straight, turn**. It subclasses `OffboardWaypoint`, so the
position rate limiter, the `hold_point` re-pointing and the settled/transit gate split
all carry over; only the target source changes — corners are computed up front instead
of arriving as clicks. Foxglove clicks are refused while a square is running, because a
stray one would deform the shape.

```bash
# props-off DRY RUN (never arms; exercises the whole state machine)
ros2 launch px4_vio_bridge offboard_square.launch.py auto_arm:=false climb_timeout:=5.0

# LIVE
ros2 launch px4_vio_bridge offboard_square.launch.py auto_arm:=true
```

Defaults: `side_m:=0.40`, `turn_deg:=90.0` (positive = turn right), `sides:=4`.
Setting `turn_deg:=-90.0` mirrors the square; `sides:=3 turn_deg:=120.0` gives a
triangle. The planned shape is published as a `nav_msgs/Path` on `/square/path` — add
it to the Foxglove 3D panel to see the intended square against the flown one.

### Two things that are different from the other launches

**Yaw rate defaults to 15 deg/s, not 5.** A 90 deg turn at the hold-yaw default of
5 deg/s takes 18 s; four of those plus four legs do not fit in any sane flight time. 15
is well inside `MC_YAWRATE_MAX=60` and the 60 deg/s abort, but it is **3x the fastest
yaw rate flown so far** — the largest previous test was 45 deg at 5 deg/s.

**Timeouts are derived from the geometry, not fixed.** HANDOFF records a near-miss
where a fixed 6 s yaw timeout would have aborted a 45 deg leg that needed 8.4 s. Here
`leg_timeout = side/speed + leg_timeout_margin` and `turn_timeout = turn/yaw_rate +
turn_timeout_margin`. The node logs the worst-case budget at startup and **errors if it
exceeds `max_flight_time`** (launch default 150 s), so the armed watchdog can never
land you mid-square. If a leg or turn times out, raise the *margin*, not the timeout.

### Bounds

Corners are computed from the latched start pose and are **not** chained off the
vehicle's actual position — chaining would fold each leg's tracking error into the next
corner and walk the square across the room. A 0.40 m square started from a corner
reaches 0.566 m from the latch, comfortably inside the 1.5 m geofence; if a requested
square would not fit, the node **refuses and lands rather than clamping**, since
clamping a corner deforms the shape.

Sequence per side: fly the leg → `leg_settle_time` (1.5 s) parked → turn →
`turn_settle_time` (1.0 s). A side counts as complete when the vehicle is inside
`corner_tol` (0.15 m) *and* the setpoint has finished slewing; a turn completes on
`yaw_tolerance_deg` (5.0). Bench-verified timing at the defaults is ~9.8 s per
side, ~42 s for the whole square including the climb.

### Before flying it

This is the most demanding manoeuvre in the project so far — 90 deg turns are double
the largest yaw flown, and it combines yaw with translation, which is what stresses VIO
hardest. Worth doing in this order:

1. Props-off dry run, confirm all four sides and turns in the log.
2. `sides:=1` armed, to fly one leg and one 90 deg turn before committing to four.
3. Full square, on a **full battery** — hover draw is ~262 W and the 2026-07-27
   waypoint flight finished at 11% SoC. Watch `/battery/level` in Foxglove.

## Global A* Planner — EXPERIMENTAL

The global planner monitor continuously replans over RTAB-Map's 2D occupancy
grid and draws the result in Foxglove. Its position-only route follower is
started by the same launch and draws a smoothed 0.60 m lookahead carrot. Both
publish no PX4 topics and cannot move the drone. Full design, topics,
validation limits, and next steps are in
`HANDOFF_GLOBAL_PLANNER.md`.

Simulator, requiring no camera or vehicle:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  global_planner_monitor.launch.py simulate:=true
```

Live observation requires the map output to be explicitly enabled:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py \
  slam_publish_grid:=true slam_grid_3d:=false \
  slam_grid_ray_tracing:=true slam_grid_footprint_radius:=0.40
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge global_planner_monitor.launch.py
```

Click a `world` goal through the existing `/waypoint/clicked` Foxglove Publish
tool. Add `/rtabmap/grid`, `/planner/inflated_map`, `/planner/path`,
`/planner/candidate_path`, and `/planner/markers` to the 3D panel; watch
`/planner/status` and `/planner/planning_ms` separately. For the follower, add
`/planner/follower/markers` and watch `/planner/follower/status` plus the
structured `/planner/follower/valid` boolean. Its
`/planner/follower/displacement` is the smoothed position displacement in the
corrected `world` frame. `/planner/follower/vio_displacement` rotates that
vector back into continuous VIO axes. The separately launched
`offboard_global_planner` adapter can apply the latter relative to PX4's current
local position; it defaults `auto_arm=false` and must pass a real-PX4 props-off
run before first use. Neither displacement topic is a velocity command.

A cross-track violation is latched: the follower freezes its relative carrot
and remains invalid until it receives a newer path and cross-track stays below
`cross_track_resume` (default 0.05 m) continuously for
`cross_track_recovery_time` (default 1.0 s). Samples merely dipping below
`max_cross_track` cannot restart flight or reset the adapter's LAND timer.

Props-off attempt 1 remained disarmed and validated OFFBOARD entry, fixed yaw,
position-only fields, the 0.15 m/s speed cap, geofence and data health. It also
found a target-arrival snap that bypassed the 0.30 m/s^2 limiter near noisy
rebased targets. The snap has been removed and regression-tested. A second
props-off bag with an exact known-free goal and a deliberate planner-loss HOLD
is still required before `auto_arm=true`; see `HANDOFF_GLOBAL_PLANNER.md`.

The click expresses the requested destination and is accepted even if it lies
in unknown space, outside the current map, or on an obstacle. Unknown and
lethal cells remain blocked. For unknown/outside/disconnected requests the
planner reports `EXPLORING` and flies only to the closest reachable known-safe
frontier, updating that endpoint as the map expands. For obstacle clicks it
reports `SAFE_APPROACH` and stops outside the 0.40 m lethal envelope. The
orange `/planner/effective_goal` marker shows where the current route actually
ends; the white marker remains the requested goal. A temporary exploration
frontier is not reported as final arrival, so the PX4 adapter holds and waits
rather than landing there. Live
observation confirmed the decoded grid aligns with the obstacle cloud and the
planner produces a route. A real loop-closure bag captured a 12.1 cm corrected
pose step while the relative position proposal remained within its 0.25 m/s
limit. The correction gate and progress telemetry were subsequently improved,
then live-validated in a second 137 s recording with zero backward cumulative
progress events. A later native-correction bag confirmed zero-frame
raw/corrected synchronization and effectively exact map-to-odom transform
direction after the host pairing fix. An asymmetric-scene Foxglove visual
check remains useful. See `HANDOFF_GLOBAL_PLANNER.md` for the mandatory
props-off gate and first-flight procedure.

## PARKED: Obstacle Avoidance (VFH2D) — EXPERIMENTAL, NEVER ARMED

This implementation is preserved for reference and testing, but it is not the
current path-planning direction and should not be treated as a supported flight
mode. VFH supplies reactive local steering, not a global route; it has weak
dead-end behavior and is sensitive to cloud density, height filtering, and its
short-lived off-camera memory. No normal stack launch starts it. See
`HANDOFF_VFH.md` for the frozen design, failure analysis, parameters, and tests.

A 2D Vector Field Histogram (VFH+) planner, split into three pieces so the
algorithm can be judged long before it is given authority over the vehicle:

| file | what it is |
|---|---|
| `px4_vio_bridge/vfh2d.py` | the algorithm — no ROS, no numpy, no vehicle |
| `px4_vio_bridge/vfh_obstacles.py` | `/rtabmap/obstacle_cloud` → world-frame voxel memory → body-frame `(range, bearing)` samples |
| `vfh_monitor` | runs the planner live and publishes what it decided. **Cannot move the drone** |
| `offboard_vfh` | parked experimental flight node; never armed |
| `scripts/vfh_sim_obstacles.py` | fake cloud + pose, so all of the above runs with no camera and no drone |

**The obstacle cloud is not published by default.** Everything here needs the
stack started with `slam_publish_clouds:=true`, or it sees nothing at all:

```bash
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py slam_publish_clouds:=true
```

### Historical validation sequence (only use if deliberately reviving it)

```bash
# 0. no hardware at all: fake a wall 2.2 m ahead with a 1.4 m gap to the right
python3 scripts/vfh_sim_obstacles.py --wall-distance 2.2 --gap-width 1.4 --gap-offset -0.8
ros2 run px4_vio_bridge vfh_monitor        # expect a positive (right) steer angle

# 1. real camera, real room, nothing armed — carry the drone around by hand
ros2 launch px4_vio_bridge vfh_monitor.launch.py

# 2. props off, whole flight state machine, never arms
ros2 launch px4_vio_bridge offboard_vfh.launch.py auto_arm:=false climb_timeout:=5.0

# The offboard mode was never armed and is now parked. Reassess the planning
# architecture before attempting an armed VFH flight.
```

It also logs a nose-centred ASCII bar once a second, where `#` is blocked, `.` is
empty, `-` is free-with-returns and `^` is the chosen direction:

```text
...........................############....^............................  steer=+35deg nearest=2.20m
```

A goal is optional — publish a `PointStamped` on `/waypoint/clicked` from the
Foxglove 3D panel and it will steer toward it, exactly as the flight node does.

### What VFH publishes to Foxglove

Both nodes publish the **same** telemetry (`vfh_telemetry.py`), so a monitor
session is a genuine rehearsal of the flight display. Everything is drawn in the
ENU `world` frame at the pose the obstacle cloud was measured against, so it
lines up with `/rtabmap/obstacle_cloud` and the SLAM path.

| topic | type | panel |
|---|---|---|
| `/vfh/markers` | `MarkerArray` | **3D** — the whole picture, see below |
| `/vfh/samples` | `PointCloud2` | **3D** — the points that actually reached the histogram |
| `/vfh/status` | `String` | Raw Message — one line, everything |
| `/vfh/blocked` | `Int32` | **Indicator** — 0 clear, 1 no way forward |
| `/vfh/nearest` | `Float32` | **Gauge** — closest return in metres (−1 = nothing in range) |
| `/vfh/nearest_bearing_deg` | `Float32` | Plot — where that closest thing is, relative to the nose |
| `/vfh/heading_deg` | `Float32` | Plot — vehicle heading, PX4 NED (0 = north) |
| `/vfh/direction_deg` | `Float32` | Plot — chosen steer angle, relative to the nose |
| `/vfh/direction_heading_deg` | `Float32` | Plot — the same direction as an absolute NED heading |
| `/vfh/goal_bearing_deg`, `/vfh/goal_distance` | `Float32` | Plot — where the goal is (only while one is set) |
| `/vfh/opening_width_deg` | `Float32` | **Gauge** — how wide the gap it chose actually is |
| `/vfh/blocked_sectors`, `/vfh/obstacle_blocked_sectors`, `/vfh/samples_count` | `Int32` | Plot — non-flyable sectors, physically obstacle-blocked sectors, and how much data fed VFH |
| `/vfh/memory_points` | `Int32` | Plot — number of remembered world-frame obstacle voxels |
| `/vfh/cost` | `Float32` | Plot — cost of the winning candidate |
| `/vfh/histogram`, `/vfh/binary`, `/vfh/obstacle_binary` | `Float32MultiArray` | Plot — density, final non-flyable mask, and obstacle-only mask per sector |
| `/vfh/direction`, `/vfh/goal` | `PoseStamped` | 3D — plain arrow/point if you prefer them separate |

`/vfh/markers` is the one to add first. It contains, in the 3D panel:

- **the histogram fan** — one ray per sector inside `display_fov_deg` (±90 deg
  by default), drawn out to the range that sector measured (or `max_range` if it
  saw nothing). Red means physically blocked after vehicle-radius enlargement;
  green means clear and inside the legal steering FOV; grey means no remembered
  obstacle but outside the legal steering FOV. The `max_steer_deg` wedge shows
  the smaller region in which a heading may actually be generated;
- **the chosen direction** as a thick blue arrow;
- **the rejected candidates** as thin yellow lines — this is what explains a
  surprising choice;
- **the goal** as a white sphere, and a text label repeating steer/nearest.

Plot `/vfh/heading_deg` and `/vfh/direction_heading_deg` on one Plot panel to see
where the vehicle is pointing against where the planner wants to go; the gap
between the two is what `yaw_follows_direction` is closing.

**The Foxglove bridge whitelist had to be widened for any of this to arrive** —
`rtabmap_slam_px4.launch.py` now includes `'^/vfh/.*$'` in
`foxglove_topic_whitelist`. It stays read-only: nothing subscribes to `/vfh/*`,
and the client publish whitelist is untouched.

### How the flight node differs from `offboard_waypoint`

It subclasses it, so the setpoint rate limiter, the `hold_point` re-pointing, the
settled/transit gate split, the VIO watchdogs, K/L and `max_flight_time` are
unchanged. After reaching a verified stable hover, it first holds XY and runs
`0 -> -90 -> +90 -> 0 deg` at 15 deg/s, where 0 is the original heading. This
clears pre-takeoff obstacle memory and repopulates the useful forward hemisphere;
translation stays disabled until it returns within 5 deg of 0 and settles for
1 s. A click during the sweep is retained as the next goal but cannot move the
vehicle early. Set both `startup_sweep_min_deg` and `startup_sweep_max_deg` to
0 to disable this phase.

After that, **a click is a goal, not a setpoint.** The setpoint is a
carrot placed `lookahead` (0.60 m) along the direction VFH picks every
`plan_period` (0.2 s), so the vehicle curves around obstacles rather than driving
through them, and `yaw_follows_direction` turns it to face where it is going.
Obstacle enlargement is evaluated against the finite segment to the goal, not
an infinite ray: a return beyond a short waypoint only blocks headings whose
endpoint would enter `robot_radius + safety_margin` around that return.

| condition | response |
|---|---|
| `nearest < stop_distance` (0.90 m) | freeze the carrot, hold position |
| planner reports blocked | freeze the carrot, hold position |
| blocked for `blocked_timeout` (10 s) | AUTO.LAND |
| obstacle data stale > `obstacle_timeout` (1 s) | freeze the carrot |
| stale > `obstacle_stale_land_time` (2 s) | AUTO.LAND |
| `nearest < abort_distance` (0.50 m) for 0.5 s | AUTO.LAND |

### Three things that decide whether this works

- **The camera sees ~70 deg, and unknown is not free.** `max_steer_deg` (35)
  marks histogram sectors outside that cone non-flyable and bounds every chosen
  direction to the region actually observed. `display_fov_deg` is visualization
  only and defaults to 90 deg, so remembered side obstacles remain visible. A
  consequence worth internalising: the tuned 0.40 m clearance envelope expands
  an obstacle at 1 m by 23.6 deg on each side.
- **A gap must be wider than `2 * (robot_radius + safety_margin)` = 0.8 m.**
  The measured radius is approximately 0.30 m and the tuned margin is 0.10 m.
  Shrink `safety_margin` only if you also believe the position hold.
- **It has bounded world-frame obstacle memory.** The newest point batch in
  each 0.10 m voxel is retained for 30 s, capped at 20,000 points. This prevents
  yawing away from an obstacle and immediately steering back when it leaves the
  camera. Repeated observations replace a voxel batch rather than accumulating
  frames, preserving the tuned current-cloud density; expiry removes moved
  objects and noise. Small SLAM-correction jitter is ignored; an accumulated
  correction of 0.05 m or 2 deg clears memory and holds until a fresh cloud.
  Set `memory_duration:=0` to disable it.
- **The height slab must clear the floor.** `z_below` is measured *from the
  vehicle*, so a value at or above `hover_height` turns every ground return
  inside `max_range` into an obstacle — a broad, symmetric red fan across the
  whole forward arc with genuinely empty space ahead. It is a silent failure:
  the histogram looks entirely plausible. The default is 0.15 m for a 0.30 m
  hover, and `offboard_vfh` clamps it to `hover_height - 0.15` with an error log.

Obstacle geometry is measured against `/rtabmap/pose` (the SLAM pose), not the
raw VIO pose PX4 flies on, because that is the frame the cloud lives in; only the
resulting *relative* bearing crosses over to PX4's heading. That is safe while
the two headings agree, which the inherited 20 deg `max_vio_yaw_error_deg`
watchdog enforces.

Tuning is all launch arguments (`tau_high`/`tau_low`, `min_points`, `smoothing`,
`sectors`, `robot_radius`, `safety_margin`, `max_range`, `memory_duration`,
`memory_voxel_size`, `memory_max_points`, the memory-correction reset gates,
`display_fov_deg`, the `startup_sweep_*` controls, and the three `mu_*` cost
weights).
Tune them against `vfh_monitor` in the actual room before arming —
density thresholds depend on how many points that room's surfaces return.

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
