# Autonomous Drone PX4 VIO — Handoff

Authoritative current-state doc. Full chronological history is in `HANDOFF_ARCHIVE.md`
(kept for forensics; some of it is superseded — trust this file).

Last updated: 2026-07-14.

## What this project is

Raspberry Pi 5 + OAK-D Lite + Pixhawk 4 (PX4 v1.17.0). OAK-D **RTAB-Map VIO/SLAM** →
`/rtabmap/vio_pose` (continuous VIO pose) → `vio_to_px4_odometry` bridge → PX4
`/fmu/in/vehicle_visual_odometry` over uXRCE-DDS on TELEM2. EKF2 fuses external vision
for horizontal position, height, and yaw (no GPS). Goal: indoor autonomous flight.

- Project: `/home/john/autonomous_drone_px4_vio`
- Bridge pkg: `ros2_ws/src/px4_vio_bridge` (ROS 2 Jazzy, `ROS_DOMAIN_ID=42`)
- px4_msgs from `/home/john/ros2_ws/install`
- VIO script: `scripts/rtabmap_vio_ros2.py` (also `oak_d_vins_cpp/.../basalt/` — Basalt is a dead end here, too timing-fragile)

## Current status (2026-07-09)

- **EV position + yaw fusion working and validated.** `xy_valid`, `z_valid`, `v_xy_valid`,
  `heading_good_for_control` all true; EV yaw tracked physical rotation with correct sign/magnitude.
- **Autonomous hover node built (`offboard_hover`); props-off dry run PASSED.** Full sequence
  `latch → STREAM → ENGAGE → CLIMB_HOLD → LAND → DONE` with no arming; PX4 accepts OFFBOARD while
  disarmed. Only the actual arm+climb is unexercised. **Not yet flown with props.**
- **TELEM2/DDS link is marginal** (works USB-free but near the edge — see Known Issues).

## Bring-up

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash && source /home/john/ros2_ws/install/setup.bash && source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py
```
Starts: RTAB-Map VIO/SLAM, `vio_to_px4_odometry`, `px4_local_position_to_ros`, and
Foxglove (:8765). The systemd-owned MicroXRCEAgent runs independently on
`/dev/ttyAMA0` @921600. Foxglove-only variants:
`rtabmap_oak_foxglove.launch.py`, `basalt_oak_foxglove.launch.py`.

Rebuild: `colcon build --packages-select px4_vio_bridge` (note: **ament_cmake** — see Gotchas).

### Micro XRCE-DDS Agent ownership (important)

There must be exactly one owner of `/dev/ttyAMA0`. The normal configuration is the
system v3.0.1 MicroXRCEAgent managed by systemd. Accordingly,
`rtabmap_slam_px4.launch.py` defaults to `start_xrce_agent:=false`; enabling its
launch-owned fallback while the service is active creates serial-port contention
and can prevent the PX4 DDS session from connecting.

The repository service file is `systemd/micro-xrce-agent.service`. Install it once:

```bash
cd /home/john/autonomous_drone_px4_vio
sudo install -m 0644 systemd/micro-xrce-agent.service \
  /etc/systemd/system/micro-xrce-agent.service
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
systemctl status micro-xrce-agent.service
```

Logs: `journalctl -u micro-xrce-agent.service -f`.

Manual fallback only: first run `sudo systemctl stop micro-xrce-agent.service`, then
launch with `start_xrce_agent:=true`. Never use both owners together. The legacy
`basalt_vio_px4.launch.py` still starts an agent unconditionally, so its use also
requires stopping the systemd service until that launch is migrated.

## PX4 params (saved to flash — the ones that matter)

| Param | Value | Why |
|---|---|---|
| `EKF2_EV_CTRL` | `11` | HPOS(1)+VPOS(2)+YAW(8). Must be a true **integer** (set via NSH, never MAVLink float). |
| `EKF2_HGT_REF` | `3` | Height reference = Vision. |
| `EKF2_GPS_CTRL` | `0` | No GPS. |
| `UXRCE_DDS_SYNCC` | `0` | Timesync off — required or the uXRCE session never connects on this serial link. |
| `UXRCE_DDS_SYNCT` | `0` | Same. `timesync converged: false` is normal/OK even when fully connected. |
| `COM_ARM_WO_GPS` | `1` | Allow arming without GPS. |
| `COM_RC_IN_MODE` | `0` | RC transmitter expected (kept — RC is the manual-override/kill switch). |

## NSH access (PX4 shell from the Pi)

`scripts/nsh.py` runs NSH over MAVLink `SERIAL_CONTROL` on USB `/dev/ttyACM0` @115200
(venv `.venv-mavlink`). USB is **dev/debug only**; there is no NSH shell without it.

```bash
.venv-mavlink/bin/python scripts/nsh.py "param show EKF2_EV_CTRL" "ekf2 status"
```

## Autonomous hover (offboard)

```bash
# props-off DRY RUN (never arms — validates plumbing):
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=false -p hover_height:=0.30 -p hold_time:=10.0
# LIVE (props on, RC bound as kill, area clear):
ros2 run px4_vio_bridge offboard_hover --ros-args -p auto_arm:=true  -p hover_height:=0.30 -p hold_time:=10.0
```
Node confirms arm/offboard from `/fmu/out/vehicle_control_mode` (this build does NOT publish
`vehicle_status`). Safety: `auto_arm` default false; aborts if OFFBOARD+ARM not confirmed in time;
`max_flight_time` watchdog → LAND; lost position in flight → LAND; Ctrl-C while armed → AUTO.LAND.
When the node runs in an interactive terminal, pressing **K** (no Enter) sends PX4 forced-disarm
commands for one second: the motors stop immediately, even in the air. This Pi-side switch depends
on the terminal process and TELEM2/DDS link, so it supplements rather than replaces the RC kill switch.

## Pre-flight gate (all green before `auto_arm:=true`)

- uXRCE `Running, connected`; DDS streaming (`px4_local_position` publishing, not stuck re-creating).
- `vehicle_local_position`: `xy_valid`/`z_valid`/`v_xy_valid`/`heading_good_for_control` all true.
- `estimator_status_flags`: no `reject_*`, no `fs_*`; `cs_ev_pos`/`cs_ev_yaw` true.
- Props on & secure, RC on with hand on kill, area clear.
- VIO initialized: move the drone (translation + gentle yaw) before arming so VIO converges (see Gotchas).

## Open risks before live flight

1. **TELEM2 link reliability on battery power, USB-free** — verify DDS is rock-solid on the vehicle's
   own power before trusting it airborne (see Known Issues).
2. **0.30 m is very low** — vision-only height (no rangefinder); ground effect can disturb VIO features.
   Confirm Z holds steady near the floor or raise to 0.5–0.6 m.
3. Live arm+climb never exercised yet.

## Known Issues & Gotchas

- **TELEM2/DDS link is marginal.** USB `/dev/ttyACM0` is dev/debug; flight comms is TELEM2/uXRCE
  `/dev/ttyAMA0` @921600. A cold uXRCE session sometimes gets stuck re-establishing its whole
  datawriter graph (0 telemetry to ROS: `px4_local_position` publishes 0, probes get 0). Observed
  2026-07-09: it established after a USB reconnect and then kept streaming at 50 Hz **even with USB
  back out**, and came up clean USB-free across several Pixhawk power-cycles. Fragile phase is
  **establishment**, not steady state → marginal serial link (likely an unreliable TELEM2 cable;
  grounding margin helps at the edge). Fixes to try before flight: short shielded/twisted cable
  (TX/RX twisted with a solid common GND), `SER_TEL2_BAUD=460800` for margin. **Without USB there is
  no NSH**, so USB-free DDS recovery = physical Pixhawk power-cycle. Health check: `create_datawriter`
  count in the launch log should stop increasing once established; `Published PX4 local position`
  should keep increasing (each log line = 100 msgs).

- **EKF2 int/bitmask params corrupt if set over MAVLink/QGC as float.** PX4 stores the float's raw
  bit pattern as the int (e.g. `EKF2_EV_CTRL` became `1077936128` = float `3.0` bits → no EV bits set →
  no fusion). Set/verify via NSH `param show`/`param set` only. (Also captured in memory
  `px4-int-param-float-corruption`.)

- **uXRCE bring-up:** use the system `/usr/local/bin/MicroXRCEAgent` (v3.0.1), NOT the project-local
  v2.4.3 (never handshakes). Needs `UXRCE_DDS_SYNCC/SYNCT=0`. If `Running, disconnected` after an agent
  restart with USB present: NSH `uxrce_dds_client stop; uxrce_dds_client start -t serial -d /dev/ttyS2 -b 921600`.
  (Also in memory `px4-uxrce-vio-bringup`.)

- **This PX4 build publishes `vehicle_control_mode`, not `vehicle_status`.** Use
  `flag_armed`/`flag_control_offboard_enabled` for arm/offboard state.

- **`px4_vio_bridge` is ament_cmake, not ament_python.** `setup.py` entry_points are vestigial/ignored.
  To add a node executable: (1) create `scripts/<name>` wrapper (`from px4_vio_bridge.<mod> import main`,
  chmod +x), (2) add it to `install(PROGRAMS ...)` in `CMakeLists.txt`, (3) rebuild. Verify with
  `ros2 pkg executables px4_vio_bridge`.

- **VIO "shake to align" at boot is normal.** VIO yaw origin is arbitrary and VIO needs motion to
  converge; EKF2 then does `reset_yaw_to_vision`. Roll/pitch are correct immediately (gravity). Make a
  deliberate pre-arm move-to-initialize a checklist step. A *repeatable residual* offset after
  convergence = static camera mount → bake into bridge `vio_yaw_offset_deg`.

- **DepthAI 3.7.1 is the isolated OAK startup failure.** There are 244 saved OAK
  crash dumps across multiple days; at least 185 contain the same device-side
  `RTEMS_FATAL_SOURCE_INVALID_HEAP_FREE` / MIPI `Invalid config steps` assertion. On 2026-07-13 a
  camera-only test reproduced it with no StereoDepth, VIO, SLAM, ROS, image/depth publication, or
  point clouds. Under 3.7.1, CAM_B failed at both 640x400 and native 640x480, and at 30, 15, and
  10 FPS, without producing a frame; CAM_C worked. The identical CAM_B test and a two-mono-camera
  test both worked immediately under DepthAI 3.5.0. The full RTAB-Map VIO/SLAM executable then ran
  at ~13.4 Hz on `/rtabmap/vio_pose`. Keep `depthai==3.5.0` from `requirements-depthai.txt`; the
  executable fails fast if 3.7.1 is installed. The host-side `X_LINK_ERROR` messages are consequences
  of the device firmware crash, not an XRCE failure or evidence that the OAK hardware needs an RMA.

- **Camera feed in Foxglove: use the compressed topic.** The feed now defaults to JPEG
  `CompressedImage` on `/rtabmap/image/compressed` (best-effort, keep-last-1) — raw `/rtabmap/image`
  (256 KB/frame) backed up over the WebSocket and the delay grew unbounded. In Foxglove point an
  Image panel at `/rtabmap/image/compressed`. Tune via launch args `rtabmap_image_format`
  (`jpeg`/`raw`), `rtabmap_image_jpeg_quality`, `rtabmap_image_publish_stride`. Image publish uses a
  non-blocking `tryGet`, so it can't stall the `/basalt/pose` → PX4 path. Enabled in the Foxglove-only
  launches; opt-in in the main launch (`rtabmap_publish_image:=true`).

- **RTAB-Map VIO feature metadata must fit XLink.** With `slam_num_features:=1000`,
  `FeatureTracker.outputFeatures` produced about 59 KB of metadata at 30 Hz and XLink dropped every
  message because its metadata limit is 51,200 bytes. On 2026-07-14, `700` also left VIO stuck at
  the identity pose while `400` initialized immediately under the same motion. The feature target
  now defaults to `400` on this OAK-D Lite.

- **Keep `RTABMapVIO.transform` single-consumer in the combined SLAM graph.** Fan-out directly to
  both `RTABMapSLAM.odom` and the ROS bridge left VIO publishing identity poses. Publish raw VIO to
  ROS from `RTABMapSLAM.passthroughOdom` instead. With that wiring, the live obstacle map grew from
  3,090 to 3,900 points with nine distinct updates over 20 seconds.

- **ROS 2 CLI is flaky here** (`ros2 topic list/echo` miss BEST_EFFORT topics / hang). Prefer a small
  rclpy probe with matching QoS, or the launch-log grep checks above.

## ROS topics (reference)

`/basalt/pose` (VIO in), `/fmu/in/vehicle_visual_odometry` (bridge out),
`/fmu/out/vehicle_local_position_v1`, `/fmu/out/vehicle_control_mode`,
`/fmu/out/estimator_status_flags`, `/px4/local_position/{pose,odometry,path}`,
`/vio/yaw_offset/{pose,odometry,path}` (yaw-offset tester), `/rtabmap/{path,odometry}`.

## Safety

Props removed for all bench/estimator work. No powered/offboard flight until: pre-flight gate green,
props secure, RC bound as manual override / kill switch, area clear, and the props-off dry run
(`auto_arm:=false`) has passed. Do not fly on the TELEM2 link until it's verified stable on battery power.
