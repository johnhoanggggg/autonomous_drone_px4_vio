# Autonomous Drone PX4 VIO

ROS 2 Jazzy workspace flying a Raspberry Pi 5 + Pixhawk 4 quadrotor on OAK-D Lite
visual odometry, with a 2D global planner over an RTAB-Map occupancy grid.

RTAB-Map VIO/SLAM on the OAK-D Lite publishes its continuous VIO pose on
`/rtabmap/vio_pose`; `px4_vio_bridge` converts that into PX4 `VehicleOdometry` on
`/fmu/in/vehicle_visual_odometry`, and EKF2 fuses it as external vision.

**Nothing here is a supported autonomous product.** Every flight mode is
experimental, every armed flight needs an RC kill switch in someone's hand, and
`auto_arm` defaults to false everywhere so the full state machine can be
rehearsed props-off first.

- Deep design notes: `HANDOFF.md`, `HANDOFF_GLOBAL_PLANNER.md`
- Parked work: `HANDOFF_VFH.md`, `HANDOFF_LOOP_CLOSURE.md`, `HANDOFF_ARCHIVE.md`

## Contents

1. [Install and pinned versions](#install-and-pinned-versions)
2. [Quick start — the three terminals](#quick-start--the-three-terminals)
3. [Flight parameters](#flight-parameters)
4. [Observability](#observability)
5. [Launch reference](#launch-reference)
6. [Health checks, NSH and calibration](#health-checks-nsh-and-calibration)
7. [Measured facts](#measured-facts)
8. [Known issues](#known-issues)
9. [Safety](#safety)

## Install and pinned versions

DepthAI is pinned to **3.5.0**. Version 3.7.1 repeatedly crashes this OAK-D Lite's
CAM_B during device-side MIPI startup.

```bash
cd /home/john/autonomous_drone_px4_vio
python3 -m pip install --user --break-system-packages -r requirements-depthai.txt
python3 -c "import depthai; print(depthai.__version__)"  # must print 3.5.0
```

The process monitor needs `psutil` (`sudo apt install python3-psutil`).

Rebuild after any source change:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
colcon build --packages-select px4_vio_bridge
```

Run the test suite (272 tests, no hardware needed):

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
python3 -m pytest src/px4_vio_bridge/test/ -q
```

### Which node runs where

DepthAI splits work between the OAK-D's VPU and the Pi. This decides what is
worth optimising:

| DepthAI node | runs on | notes |
|---|---|---|
| `Camera`, `IMU`, `StereoDepth`, `FeatureTracker` | **OAK-D VPU** | depth and feature extraction are already offloaded |
| `RTABMapVIO`, `RTABMapSLAM` | **Pi host** | ~215-230% CPU, the dominant cost |

Consequence: `slam_num_features` throttles the *camera*, not the Pi. See
[Measured facts](#measured-facts).

## Quick start — the three terminals

Every terminal needs the same preamble:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
```

**Terminal 1 — SLAM, VIO bridge, PX4 telemetry, Foxglove.** The XRCE agent is
owned by systemd; confirm it before starting.

```bash
systemctl is-active micro-xrce-agent.service

ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py \
  slam_publish_grid:=true \
  slam_grid_3d:=false \
  slam_grid_ray_tracing:=true \
  slam_grid_cell_size:=0.05 \
  slam_grid_footprint_radius:=0.25
```

**Terminal 2 — global planner and route follower.** Publishes no PX4 topics and
cannot move the drone.

```bash
ros2 launch px4_vio_bridge global_planner_monitor.launch.py \
  robot_radius:=0.25 \
  safety_margin:=0.05 \
  inflation_extra:=0.20 \
  max_carrot_speed:=0.20 \
  max_carrot_acceleration:=0.30 \
  max_cross_track:=0.30 \
  cross_track_resume:=0.05 \
  cross_track_recovery_time:=1.0 \
  max_correction_m:=1 \
  max_correction_yaw_deg:=10.0
```

Click a `world` goal with the Foxglove 3D panel's Publish tool on
`/waypoint/clicked`, and confirm `/planner/follower/valid: true` before terminal 3.

**Terminal 3 — the flight adapter.** This one arms.

```bash
ros2 launch px4_vio_bridge offboard_global_planner.launch.py \
  auto_arm:=true \
  hover_height:=0.30 \
  command_speed:=0.20 \
  command_acceleration:=0.30 \
  path_command_projection_tolerance:=0.05 \
  path_corner_tolerance:=0.05 \
  geofence_radius:=3.0 \
  max_flight_time:=60.0 \
  max_correction_m:=1 \
  max_correction_yaw_deg:=10.0
```

Set `auto_arm:=false` for a props-off dry run — the whole state machine runs and
records, and no arm command is ever sent. Terminal 3 records the flight bag and
starts the process monitor; **do not pass `bag_output`**, every launch names its
own bag `<mode>_<UTC>` through `px4_vio_bridge.log_paths`. Set
`PX4_VIO_FLIGHT_LOGS` to record somewhere other than `ros2_ws/flight_logs`.

## Flight parameters

### Altitude: ramp and feedforward

A hard z step gives PX4 nothing but a position error, and the takeoff ramp hands
the position controller a collective about 0.08 below true hover, so the vertical
integrator starts that far negative. Measured 2026-08-28: **20.4 s to climb
0.30 m**, against a commanded 48 mm/s that only delivered 7.8 mm/s.

`OffboardHover.ramp_z` ramps the published altitude and sends the matching
`velocity[2]`. PX4 sums `TrajectorySetpoint.velocity` with the position-P output,
so the feedforward puts the full commanded rate into the velocity loop instead of
letting a shrinking position error decide it.

| parameter | default | meaning |
|---|---|---|
| `climb_rate` | 0.25 m/s | altitude slew rate. **`0` restores the old raw step** |
| `climb_leash` | 0.12 m | how far the ramp may lead the vehicle |
| `climb_release` | 0.05 m | band over which the feedforward tapers out |
| `climb_feedforward` | `true` | publish `velocity[2]`; `false` sends NaN |

Two things that are easy to get wrong and are covered by tests:

- The **leash** exists so a vehicle that cannot follow gets a bounded error rather
  than the same step delivered late.
- The feedforward is released on the **vehicle's** remaining distance, not the
  ramp's. Keying it off the ramp switched it off with 0.09 m still to climb — the
  ramp had arrived, the drone had not — and the rest reverted to the slow crawl
  the ramp exists to avoid.

### Horizontal: velocity and acceleration feedforward

`HorizontalCommandLimiter.velocity` is already the speed- and acceleration-limited
velocity of the point being published, in the same NED frame, and is that point's
own derivative. Publishing it costs nothing to compute and saves PX4 rediscovering
it from position error through `MPC_XY_P`.

| parameter | default | meaning |
|---|---|---|
| `horizontal_feedforward` | `true` | publish `velocity[0..1]`; `false` is the older position-only command |

The acceleration setpoint is the derivative of that velocity, bounded by
`command_acceleration`. Both go NaN outside an advancing route — in a fault hold,
a command hold, or LAND — so a stale command can never keep pushing the vehicle.

### Corner blending

`PathCommandLimiter` stops the command dead at **every** path vertex and waits for
the vehicle to arrive within `path_corner_tolerance`. Measured A* paths carry 4.5-6
vertices over 1.7-2.7 m, so that is a full stop roughly every 50 cm, and it is the
main reason routes run at about half their commanded cruise.

Blending replaces the stop with a junction-deviation speed cap:

```text
v_corner = sqrt(a * d * cos(t/2) / (1 - cos(t/2)))
```

where `t` is the turn away from straight and `d` is `junction_deviation`. The
published command still rides the polyline exactly — only the airframe rounds the
corner — so every existing projection and clearance check applies unchanged.

| parameter | default | meaning |
|---|---|---|
| `corner_blending` | `false` | carry speed through bends instead of stopping |
| `junction_deviation` | 0.05 m | how far the airframe may cut a corner |

At 0.20 m/s cruise and `d=0.05`, the worst observed corner (70 deg) allows
0.26 m/s — above cruise — so no observed corner needs any slowdown and the
stop-and-wait disappears outright. The cap starts to bite if `command_speed` rises
or routes get sharper. `junction_deviation` must stay well inside the follower's
`max_cross_track`, because it is what the vehicle is permitted to cut.

**Off by default**: it removes a stop the route has always had, so give it one
deliberate flight before trusting it.

### Everything else worth knowing

`hover_height` is 0.30 m in the quick start — very low. Height is pure vision with
no rangefinder, and ground effect disturbs VIO features near the floor. Confirm Z
holds steady in a dry run; consider 0.5-0.6 m if VIO gets jittery low.

`yaw_rate_deg` is **20 deg/s** in the global-planner launch (the node default is 5).
Yaw follows the commanded path heading, so the airframe — and the forward-facing
camera the VIO depends on — points along the route. Translation pauses above
`yaw_align_error_deg` (40) and resumes below `yaw_resume_error_deg` (15).

## Observability

### Process load — `/perf/*`

`process_monitor` samples per-process CPU and memory and publishes them on the
same time base as the flight data, so a bag can answer "was the Pi saturated when
that happened?" after the fact. It starts with the flight launch
(`perf_monitor:=true`) and is recorded by `--all-topics`.

| topic | type | meaning |
|---|---|---|
| `/perf/process/<label>/cpu_percent` | `Float32` | summed over matching PIDs |
| `/perf/process/<label>/mem_mb` | `Float32` | RSS |
| `/perf/cpu_percent`, `/perf/mem_percent` | `Float32` | machine-wide |
| `/perf/cpu_temp_c`, `/perf/load1` | `Float32` | thermals and load average |
| `/perf/throttled` | `Int32` | Raspberry Pi throttle word, `-1` if unavailable |
| `/perf/processes` | `String` | JSON with everything, incl. per-core and PIDs |

Labels: `slam`, `xrce_agent`, `vio_bridge`, `planner`, `astar`, `follower`,
`planner_sim`, `bag_record`, `foxglove`, `battery`, `px4_pos`. They are parameters
(`labels` / `patterns`), so the set can be retargeted without editing code.

CPU follows psutil's convention — **100% is one core**, so threaded RTAB-Map
legitimately reads above 100 on this 4-core Pi. `cpu_percent_of_machine` in the
JSON is the normalised companion.

Check `/perf/throttled` first when reading a bag. Non-zero means the board
browned out or overheated and the clock was cut, which makes any timing analysis
meaningless. Bits 0-3 are under-voltage / frequency-capped / throttled / soft
temperature limit *now*; bits 16-19 are the same four latched since boot.

### Configuration snapshots

Nodes publish their effective parameters as latched JSON, so a bag records what
was *configured* and not only what was achieved — even when the node started
before the recorder.

| topic | from |
|---|---|
| `/rtabmap/config` | SLAM: `num_features`, `fps`, resolution, grid cell size, ray tracing |
| `/planner/config` | A* planner: `robot_radius`, `safety_margin`, inflation, timeouts |
| `/planner/follower/config` | route follower: `lookahead`, `max_cross_track`, correction gates |

### Battery in Foxglove

`battery_to_ros` flattens `/fmu/out/battery_status_v1` into plain `std_msgs` that
Foxglove panels bind to directly. It starts with `rtabmap_slam_px4.launch.py`;
disable with `battery_monitor:=false`.

These are display/logging telemetry only. They do not inhibit planner arming or
trigger HOLD/LAND — **PX4 remains the sole authority** for battery arming checks
and low-battery failsafe.

| Topic | Type | Panel |
|---|---|---|
| `/battery/percent` | `Float32` (0-100) | **Gauge**, min 0 max 100 |
| `/battery/voltage` | `Float32` (V) | Gauge or Plot |
| `/battery/cell_voltage` | `Float32` (V/cell) | Gauge — the honest signal under load |
| `/battery/current` | `Float32` (A) | Plot |
| `/battery/power` | `Float32` (W) | Plot |
| `/battery/level` | `Int32` 0-3 | **Indicator** (0 OK, 1 LOW, 2 CRITICAL, 3 EMPTY) |
| `/battery/status` | `String` | Raw Message |

`level` is the **worse** of three sources — percent thresholds, per-cell voltage
thresholds, and PX4's own `warning` enum — so an optimistic state-of-charge
estimate can never mask a real low-voltage warning. Thresholds are launch
arguments: `battery_warn_percent` (40), `battery_critical_percent` (25),
`battery_empty_percent` (15). Escalations are logged to the bag via `/rosout`.

Invalid PX4 fields are dropped rather than published: PX4 signals "unknown" with
`voltage_v=0`, `current_a=-1`, `remaining=-1`, which on a gauge would read as
0 V / -100%.

### Foxglove

```text
ws://<pi-ip>:8765
```

The camera feed defaults to **JPEG-compressed** `sensor_msgs/CompressedImage` on
`/rtabmap/image/compressed`, best-effort keep-last-1. A 640x400 mono frame drops
from ~256 KB raw to ~15-25 KB, and stale frames are dropped rather than queued —
queueing is what makes the raw feed's delay grow without bound. Add an **Image**
panel pointed at `/rtabmap/image/compressed`.

- `rtabmap_image_format:=jpeg` (default) or `raw` (heavy — backs up)
- `rtabmap_image_jpeg_quality:=60` (1-95)
- `rtabmap_image_publish_stride:=1` (publish every Nth frame)

Still laggy? Drop quality to 40, raise stride to 2, or lower
`rtabmap_width`/`rtabmap_height`. The pose path is unaffected — the image uses a
non-blocking `tryGet` and can never stall the pose stream that feeds PX4.

**Client publishing is deliberately narrow.** The bridge runs with
`foxglove_capabilities:=[clientPublish,connectionGraph]` and
`foxglove_client_topic_whitelist:=['^/waypoint/clicked(_pose)?$','^/planner/flight/teleop$']`.
A browser tab may reach the waypoint intake and the validated flight-control
intake, and nothing else. Do **not** widen it to `['.*']` — that would put
`/fmu/in/*` (raw setpoints and arm commands) within reach of anything that can
open the WebSocket.

#### LAND / KILL panel

The flight launch subscribes to `/planner/flight/teleop` using Foxglove's Teleop
panel `Twist` schema:

- Topic `/planner/flight/teleop`, publish rate 1 Hz, **stop on release disabled**
- Center Stop button: `linear.z` = `-1` — **controlled AUTO.LAND**
- Down button: `linear.z` = `-2` — **EMERGENCY KILL / motors stop**
- Up / Left / Right: `0` (no action)

Rename it `Flight Safety — STOP=LAND, DOWN=KILL`. A zero Twist and button release
are deliberately inert, and both are ignored while disarmed. This supplements
rather than replaces the RC kill switch.

#### 3D panel setup

1. Set the **display frame** to `world`. The publish tool stamps whatever frame
   the panel follows, and nodes reject anything that is not `world`.
2. Publish tool: topic `/waypoint/clicked`, type `geometry_msgs/PointStamped`.
3. Add `/rtabmap/grid`, `/planner/inflated_map`, `/planner/path`,
   `/planner/candidate_path`, `/planner/markers`, `/planner/follower/markers`.
4. Raw Message panels on `/planner/status`, `/planner/follower/status`,
   `/planner/flight/status`, and the boolean `/planner/follower/valid`.

### Which pose is which

- **PX4 does not receive the loop-corrected SLAM pose.** The bridge consumes the
  continuous raw VIO pose on `/rtabmap/vio_pose`. This keeps loop-closure jumps
  out of EKF2.
- **Foxglove's SLAM visualization is loop-corrected**: `/rtabmap/pose`,
  `/rtabmap/odometry`, `/rtabmap/path`.
- `/rtabmap/odom_correction` is DepthAI RTAB-Map's native map-to-odometry
  correction. The route follower consumes it directly and fails closed on stale,
  non-finite or oversized corrections. It is not added to TF or sent to PX4.
- `/px4/local_position/*` is EKF2's estimate. Compare it with
  `/rtabmap/vio_pose`, not with a later SLAM loop closure.

`input_pose_topic` can select `/rtabmap/pose` experimentally, but feeding a
discontinuous pose to PX4 is not the flight default.

## Launch reference

### `rtabmap_slam_px4.launch.py` — the stack

Starts OAK-D RTAB-Map VIO/SLAM, `vio_to_px4_odometry`, the PX4
local-position-to-ROS converter, `battery_to_ros`, and one Foxglove bridge on
8765. Defaults: depth publishing off, compressed image on, clouds off, grid off.

| argument | default | notes |
|---|---|---|
| `slam_fps` | 20 | stereo camera, VIO and SLAM processing rate |
| `slam_num_features` | 500 | see [Measured facts](#measured-facts) — **stay at 400-500** |
| `slam_publish_grid` | `false` | must be `true` for the planner to see anything |
| `slam_grid_3d` | `true` | `false` for the 2D planner |
| `slam_grid_cell_size` | 0.10 | quick start uses 0.05 |
| `slam_grid_ray_tracing` | `false` | host-side cost, scales with cell count |
| `slam_publish_clouds` | `false` | needed by VFH and cloud visualisation |
| `slam_publish_image` | `true` | compressed feed for Foxglove |
| `start_xrce_agent` | `false` | systemd owns the agent — see below |
| `battery_monitor` | `true` | |

`/rtabmap/path` publishes every 10 odometry poses and is capped at 1000.

#### Micro XRCE-DDS Agent ownership

Exactly one process may own `/dev/ttyAMA0`. The flight configuration uses the
system v3.0.1 agent as a systemd service, so this launch defaults to
`start_xrce_agent:=false`. Starting a second agent while the service is active
causes serial contention and can leave PX4 DDS disconnected.

```bash
cd /home/john/autonomous_drone_px4_vio
sudo install -m 0644 systemd/micro-xrce-agent.service /etc/systemd/system/
sudo systemctl daemon-reload
sudo systemctl enable --now micro-xrce-agent.service
journalctl -u micro-xrce-agent.service -f
```

For a temporary launch-owned fallback, stop the service **first**, then opt in
with `start_xrce_agent:=true`. The legacy `basalt_vio_px4.launch.py` still starts
an agent unconditionally, so stop the service before using it.

### `offboard_global_planner.launch.py` — global-planner flight

The flight adapter for the A* route. It rebases the follower's continuous-VIO
displacement from PX4's current local position, advances the command along the
accepted map-frame polyline, and validates that exact output against the raw
occupancy grid before publishing. Invalid planner data latches a stationary HOLD;
persistent faults request AUTO.LAND. See [Flight parameters](#flight-parameters)
and `HANDOFF_GLOBAL_PLANNER.md`.

Key launch defaults: `hover_height` 0.40, `command_speed` 0.10,
`geofence_radius` 1.0, `max_flight_time` 45, `yaw_rate_deg` 20,
`min_vio_features` 80, `planner_fault_land_time` 3.0, `goal_hold_time` 3.0.
The quick start overrides several of these.

A cross-track violation is **latched**: the follower freezes its relative carrot
and stays invalid until it receives a newer path *and* cross-track stays below
`cross_track_resume` (0.05 m) continuously for `cross_track_recovery_time` (1.0 s).
Samples merely dipping below `max_cross_track` cannot restart flight or reset the
adapter's LAND timer.

The flight adapter refuses to fly if the follower's advertised speed differs from
its final-command speed.

### `global_planner_monitor.launch.py` — planner, no vehicle authority

Continuously replans over the RTAB-Map 2D grid and draws the result. Publishes no
PX4 topics and cannot move the drone. Simulator needs no camera or vehicle:

```bash
ros2 launch px4_vio_bridge global_planner_monitor.launch.py simulate:=true
```

A click is accepted even if it lies in unknown space, outside the map, or on an
obstacle; unknown and lethal cells stay blocked. For unknown/outside/disconnected
requests the planner reports `EXPLORING` and flies only to the closest reachable
known-safe frontier, updating as the map expands. For obstacle clicks it reports
`SAFE_APPROACH` and stops outside the 0.30 m lethal envelope. The orange
`/planner/effective_goal` marker shows where the route actually ends; white is
the requested goal. A temporary exploration frontier is not reported as arrival,
so the adapter holds rather than landing there.

### `offboard_hover` — the base flight node

Latch NED x/y/yaw, stream `TrajectorySetpoint` at 50 Hz, request OFFBOARD, arm,
climb, hold, then `NAV_LAND`. Every other flight node subclasses it, so the
watchdogs below apply everywhere.

```bash
# props-off dry run: STREAM -> ENGAGE -> CLIMB_HOLD -> LAND -> DONE, never arms
ros2 run px4_vio_bridge offboard_hover --ros-args \
  -p auto_arm:=false -p hover_height:=0.30 -p hold_time:=10.0
```

Safety design:

- `auto_arm` defaults **false**.
- Aborts (never arms) if OFFBOARD+ARM are not confirmed within `engage_timeout`.
- `max_flight_time` watchdog forces LAND; lost local position forces LAND; Ctrl-C
  while armed commands AUTO.LAND, never a mid-air disarm.
- **Tracking-loss landing** while armed: monitors raw VIO pose, feature count,
  bridge output, VIO/EKF yaw agreement, gyro yaw rate, and horizontal hold error.
  Defaults trip below `min_vio_features` for 0.25 s, above 20 deg yaw
  disagreement for 0.20 s, yaw rate above 60 deg/s for 0.10 s, or hold error
  above 0.35 m for 0.25 s. First second after arming is a grace period.
  Also detects a VIO relocalization reset — the pose keeps publishing but snaps
  to bit-exact `(0,0,0)`, which staleness and feature checks miss.
- **Keyboard L** requests AUTO.LAND, **keyboard K** force-disarms immediately.
  K is a true motor kill, not a landing — airborne, the vehicle falls. Both need
  an interactive terminal; disable with `keyboard_land`/`keyboard_kill:=false`.
- Fly only with an RC transmitter bound as manual-override / kill switch
  (`COM_RC_IN_MODE=0`). The Pi-side controls depend on the ROS process and the
  DDS link and are **not** a replacement for it.

Pre-flight gate, all green, via `scripts/nsh.py`:

- `uxrce_dds_client status` -> `Running, connected`
- `listener vehicle_local_position 1` -> `xy_valid`, `z_valid`, `v_xy_valid`,
  `heading_good_for_control` all true
- `listener estimator_status_flags 1` -> no `reject_*`, no `fs_*`,
  `cs_ev_pos` and `cs_ev_yaw` true

### `offboard_waypoint` — Foxglove click-to-fly

Subclasses `OffboardHover`. Foxglove has no RViz-style interactive markers; the
interaction is the 3D panel's **Publish** tool on `/waypoint/clicked`.

```bash
ros2 launch px4_vio_bridge offboard_waypoint.launch.py auto_arm:=false climb_timeout:=5.0
ros2 launch px4_vio_bridge offboard_waypoint.launch.py auto_arm:=true
```

A click is a network message from a browser, so nothing it says is trusted:

| Guard | Default | Behavior |
|---|---|---|
| `waypoint_frame` | `world` | wrong `frame_id` -> rejected, vehicle does not move |
| `geofence_radius` | 1.5 m | clamped into a disc around the **latched takeoff point** |
| click z | — | **ignored**; altitude is always `hover_height` |
| `waypoint_speed` | 0.25 m/s | the setpoint slews; PX4 never sees a position step |
| `idle_timeout` | 20 s | parked with nothing pending -> AUTO.LAND |
| `arrival_tol` | 0.12 m | arrival is **latched** once reached, not re-tested |
| `max_flight_time` | 90 s | armed watchdog -> LAND |
| `min_vio_features` | 80 | lower than the 160 used by hover — see below |

Clicks are **absolute positions in `world`**, not offsets. A click outside the
geofence is pulled onto the boundary along its own bearing, logged `(CLAMPED to
geofence)`.

Arrival is latched because a 2026-07-27 flight held station 0.135 m from target
against a 0.12 m tolerance, so an instantaneous test flickered and the idle
timeout could never fire.

`min_vio_features` is 80 here rather than 160 because waypoint flight repoints the
camera at whatever the room offers and samples worse scenes than a station-keeping
hover. This buys tolerance, not tracking quality — if counts sit near the floor,
fix the scene (lighting, texture, no blank walls).

**The horizontal-error gate during transit.** PX4 trails a moving setpoint by
roughly `waypoint_speed / MPC_XY_P` — about 0.26 m at the defaults, right on top
of the 0.35 m hold gate. So the tight gate applies only once the commanded point
has been stationary for `transit_settle_time` (1.0 s); during transit the looser
`transit_horizontal_error` (0.60 m) applies. Raising `waypoint_speed` raises the
lag proportionally — raise `transit_horizontal_error` with it, or set
`velocity_feedforward:=true`, which cancels most of it.

To command heading too, publish `PoseStamped` on `/waypoint/clicked_pose` with
`accept_waypoint_yaw:=true`. Off by default — yawing while translating stresses
VIO hardest.

### `offboard_square` — scripted closed square

Subclasses `OffboardWaypoint`; only the target source changes, corners are
computed up front. Foxglove clicks are refused while a square runs.

```bash
ros2 launch px4_vio_bridge offboard_square.launch.py auto_arm:=false climb_timeout:=5.0
ros2 launch px4_vio_bridge offboard_square.launch.py auto_arm:=true
```

Defaults `side_m:=0.40`, `turn_deg:=90.0` (positive = right), `sides:=4`.
`sides:=3 turn_deg:=120.0` gives a triangle. The planned shape publishes as
`nav_msgs/Path` on `/square/path`.

- **Yaw rate defaults to 15 deg/s, not 5.** A 90 deg turn at 5 deg/s takes 18 s.
- **Timeouts derive from geometry**: `leg_timeout = side/speed + margin`. HANDOFF
  records a near-miss where a fixed 6 s yaw timeout would have aborted a 45 deg
  leg needing 8.4 s. The node errors at startup if the worst-case budget exceeds
  `max_flight_time`. If a leg times out, raise the *margin*, not the timeout.
- Corners are computed from the latched start pose and **not** chained off actual
  position — chaining folds each leg's error into the next corner and walks the
  square across the room. If a square would not fit the geofence the node
  **refuses and lands rather than clamping**, since clamping deforms the shape.

### PARKED: `offboard_vfh` / VFH2D — never armed

Reactive local steering, preserved for reference. It is **not** the current
planning direction: weak dead-end behaviour, sensitive to cloud density, height
filtering, and its short-lived off-camera memory. No normal launch starts it.
Full design, failure analysis and parameters are in `HANDOFF_VFH.md`.

| file | what it is |
|---|---|
| `px4_vio_bridge/vfh2d.py` | the algorithm — no ROS, no numpy, no vehicle |
| `px4_vio_bridge/vfh_obstacles.py` | cloud -> world voxel memory -> body-frame samples |
| `vfh_monitor` | runs it live and publishes decisions. **Cannot move the drone** |
| `offboard_vfh` | parked flight node; never armed |
| `scripts/vfh_sim_obstacles.py` | fake cloud + pose, no camera or drone needed |

Everything here needs `slam_publish_clouds:=true` or it sees nothing.

Three things that decide whether it works, worth reading before reviving it:

- **The camera sees ~70 deg, and unknown is not free.** `max_steer_deg` (35)
  bounds every chosen direction to the observed cone. The 0.40 m clearance
  envelope expands an obstacle at 1 m by 23.6 deg on each side.
- **A gap must exceed `2 * (robot_radius + safety_margin)` = 0.8 m.**
- **The height slab must clear the floor.** `z_below` is measured *from the
  vehicle*, so a value at or above `hover_height` turns every ground return into
  an obstacle — a broad symmetric red fan with genuinely empty space ahead. It is
  a silent failure: the histogram looks entirely plausible.

### `rtabmap_oak_foxglove.launch.py` / Basalt — visualisation only

`rtabmap_oak_foxglove.launch.py` views VIO without PX4/XRCE.
`rtabmap_vio_slam_foxglove.launch.py` feeds `RTABMapVIO` into `RTABMapSLAM`.

Basalt (`basalt_oak_foxglove.launch.py`) is available for comparison but is **not
recommended** — it is more timing-sensitive here, with native DepthAI/Basalt
assertions from non-monotonic IMU/frame timestamps. Basalt VIO + RTAB-Map SLAM
was tried and removed. Treat Basalt as a dead end unless revisiting DepthAI
internals.
