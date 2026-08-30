# Autonomous Drone PX4 VIO

ROS 2 Jazzy workspace flying a Raspberry Pi 5 + Pixhawk 4 quadrotor on OAK-D Lite
visual odometry, with a flown 2D global planner over an RTAB-Map occupancy grid
and an observation-only 3D OctoMap planner under development.

RTAB-Map VIO/SLAM on the OAK-D Lite publishes its continuous VIO pose on
`/rtabmap/vio_pose`; `px4_vio_bridge` converts that into PX4 `VehicleOdometry` on
`/fmu/in/vehicle_visual_odometry`, and EKF2 fuses it as external vision.

**Nothing here is a supported autonomous product.** Every flight mode is
experimental, every armed flight needs an RC kill switch in someone's hand, and
`auto_arm` defaults to false everywhere so the full state machine can be
rehearsed props-off first.

- Deep design notes: `HANDOFF.md`, `HANDOFF_GLOBAL_PLANNER.md`,
  `HANDOFF_3D_NAVIGATION.md`
- Parked work: `HANDOFF_VFH.md`, `HANDOFF_LOOP_CLOSURE.md`, `HANDOFF_ARCHIVE.md`

## Contents

1. [Install and pinned versions](#install-and-pinned-versions)
2. [Quick start — the three terminals](#quick-start--the-three-terminals)
3. [3D planner monitor](#3d-planner-monitor)
4. [Flight parameters](#flight-parameters)
5. [Observability](#observability)
6. [Launch reference](#launch-reference)
7. [Measured facts](#measured-facts)
8. [Known issues](#known-issues)

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

Run the Python test suite (404 tests, no hardware needed; `colcon test` also
runs the C++ gtests):

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
  slam_grid_cell_size:=0.03 \
  slam_grid_footprint_radius:=0.25
```

**Terminal 2 — global planner and route follower.** Publishes no PX4 topics and
cannot move the drone.

```bash
ros2 launch px4_vio_bridge global_planner_monitor.launch.py \
  cpp_nodes:=true \
  robot_radius:=0.25 \
  safety_margin:=0.00 \
  inflation_extra:=0.25 \
  lookahead:=0.25 \
  lookahead_step:=0.03 \
  max_carrot_speed:=0.20 \
  max_carrot_acceleration:=0.30 \
  max_cross_track:=0.15 \
  cross_track_resume:=0.06 \
  cross_track_recovery_time:=1.0 \
  path_retain_tolerance:=0.12 \
  max_correction_m:=1 \
  max_correction_yaw_deg:=10.0
```

`path_retain_tolerance` **must stay below `max_cross_track`**, or the follower
faults on cross-track before the planner will rebuild the path and the vehicle
stalls in the gap between them with no new route coming. The launch defaults
keep a 0.03 m replan-before-fault band; see [Known issues](#known-issues).

This is the narrow-hallway profile. It deliberately has no extra collision
margin: the planner requires 0.25 m from the vehicle centre to an obstacle, the
physical radius of this airframe. The 0.12 m retained-path band and 0.15 m
cross-track limit are the configuration that completed tunnel flight
`20260829T114405Z`; they leave a 0.03 m replan-before-fault band. This trades
wall-contact margin for route continuity; it is not a collision-free
configuration.

Click a `world` goal with the Foxglove 3D panel's Publish tool on
`/waypoint/clicked`, and confirm `/planner/follower/valid: true` before terminal 3.

**Terminal 3 — the flight adapter.** This one arms.

```bash
ros2 launch px4_vio_bridge offboard_global_planner.launch.py \
  cpp_nodes:=true \
  auto_arm:=true \
  hover_height:=0.30 \
  command_speed:=0.20 \
  command_acceleration:=0.30 \
  path_command_projection_tolerance:=0.05 \
  path_command_suffix_tolerance:=0.03 \
  path_corner_tolerance:=0.05 \
  geofence_radius:=3.0 \
  max_flight_time:=60.0 \
  max_correction_m:=1 \
  max_correction_yaw_deg:=10.0 \
  corner_blending:=true \
  planner_fault_land_time:=6.0
```

`cpp_nodes:=true` selects the C++ adapter as well as the C++ planner and
follower; see [Which flight adapter](#which-flight-adapter). Set
`auto_arm:=false` for a props-off dry run — the whole state machine runs and
records, and no arm command is ever sent. Terminal 3 records the flight bag and
starts the process monitor; **do not pass `bag_output`**, every launch names its
own bag `<mode>_<UTC>` through `px4_vio_bridge.log_paths`. Set
`PX4_VIO_FLIGHT_LOGS` to record somewhere other than `ros2_ws/flight_logs`.

## 3D planner monitor

The 3D implementation is deliberately a separate, observation-only C++ mode.
The launch starts a keyframe-grid-to-OctoMap producer, 26-connected A* planner,
and swept-sphere XYZ follower. The producer consumes `rtabmap_msgs/msg/MapData`
on `/rtabmap/mapData`, rebuilds from the raw ground/obstacle/empty cells at the
latest optimized poses, and publishes a paired `/rtabmap/octomap` plus
`/rtabmap/octomap_metadata` generation. Ground is occupied, ray tracing is
enabled, and a loop correction rebuilds the tree rather than leaving voxels at
old poses.

The planner also consumes corrected `PoseStamped` on `/rtabmap/pose` and a
`PointStamped` goal on `/waypoint/clicked`. It publishes `/planner3d/path`,
candidate paths, structured status, map/path generations, and markers. The
follower pairs the path with that exact map generation, checks both its
lookahead and rate-limited carrot chords, and publishes displacement, velocity,
acceleration, validity, and status under `/planner3d/follower/*`. Neither node
has a PX4 publisher; this mode cannot arm or move the vehicle.

### Live OctoMap from the OAK SLAM stack

Use the dedicated launch in place of `rtabmap_slam_px4.launch.py` to run the
normal continuous-VIO/PX4 stack plus a namespaced ROS RTAB-Map 3D mapper and
OctoMap visualizer:

```bash
ros2 launch px4_vio_bridge rtabmap_slam_px4_3d.launch.py
```

The extra mapper samples the actual rectified OAK image, registered metric
depth and raw RTAB-Map VIO odometry at 3 Hz. It creates keyframe-local ground,
obstacle and observed-empty grids at 1 Hz with `Grid/3D=true`, ground occupied,
and ray tracing enabled. The OctoMap is rebuilt from those cached grids at the
latest optimized poses, so loop corrections move old voxels instead of leaving
duplicates. Its topics are deliberately isolated under `/rtabmap3d/*`; they are
not consumed by the flown 2D planner or sent to PX4.

Connect Foxglove to `ws://<pi-ip>:8765`, add a **3D** panel, select fixed frame
`rtabmap3d_map`, then add `/rtabmap3d/octomap_ground_markers` and
`/rtabmap3d/octomap_obstacle_markers`. Open each topic's settings to override
its color independently. The published defaults are brown ground and red
obstacles. `/rtabmap3d/octomap_markers` remains available as a combined view.
Useful checks are:

```bash
ros2 topic hz /rtabmap3d/mapData
ros2 topic echo /rtabmap3d/octomap_metadata --once
ros2 topic hz /rtabmap3d/octomap_ground_markers
ros2 topic hz /rtabmap3d/octomap_obstacle_markers
```

This launch is a live mapping/visualization gate, not flight authorization. It
runs a second host-side RTAB-Map process and therefore needs CPU measurement on
the Pi before its output is connected to the 3D planner.

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

ros2 launch px4_vio_bridge global_planner_3d_monitor.launch.py \
  voxel_size:=0.05 \
  robot_radius:=0.25 \
  safety_margin:=0.10 \
  max_cross_track:=0.05 \
  planning_radius_xy:=3.0 \
  min_z:=0.20 \
  max_z:=2.00 \
  record_bag:=true
```

Run the deterministic two-generation fixture without a camera using matching
fixture/planner resolution:

```bash
ros2 launch px4_vio_bridge global_planner_3d_monitor.launch.py \
  simulate:=true fixture_resolution:=0.10 voxel_size:=0.10 \
  fixture_loop_correction_after:=3.0 foxglove:=true record_bag:=true
```

That fixture publishes one observed room with a floor and obstacle, then shifts
its optimized keyframe pose by 0.20 m. A successful run reaches `PATH_VALID` and
`FOLLOWING`, keeps the follower at 20 Hz, and exposes no `/fmu/in/*` topic.
Recorded generations can be extracted to the line-oriented JSON accepted by
`planner_3d_replay`; the verifier reruns planning and rejects any accepted path
or follower chord whose generation differs or whose swept sphere intersects
occupied, unknown, or outside-map volume.

Connect Foxglove to `ws://<pi-ip>:8765`, add a **3D** panel, set its display
frame to `world`, and enable `/rtabmap/octomap_markers`. Brown cubes are occupied
ground/floor and red cubes are occupied obstacles. Add `/planner3d/path` and
`/planner3d/markers` to see the accepted route and its clearance envelope. The
raw `octomap_msgs/Octomap` is intentionally not sent over the WebSocket because
Foxglove does not render that schema directly; the marker topic is generated
from the same OctoMap leaves and is visualization-only. `max_marker_voxels`
(default 20000 in this launch) bounds browser bandwidth.

When `rtabmap_slam_px4.launch.py` is already running, leave `foxglove:=false` on
the 3D launch: the existing bridge now exposes the OctoMap markers and all
`/planner3d/*` topics. Enable `foxglove:=true` only for a standalone 3D monitor,
because only one bridge should own port 8765.

`min_z` and `max_z` are hard planning boundaries and must be replaced with
independently measured site values. The launch refuses configurations where
`safety_margin < 0.10` or `safety_margin < max_cross_track`. Goals must use the
`world` frame; unlike the 2D mode, clicked Z is retained.

The installed DepthAI 3.5.0 node still publishes only the height-collapsed
`/rtabmap/grid`; setting its `slam_grid_3d:=true` does **not** make it publish
keyframe grids. `rtabmap_slam_px4_3d.launch.py` closes that interface gap with a
separate ROS RTAB-Map host instead of reconstructing occupancy from point
clouds, which would lose observed free space and its original viewpoint. Live
planner pose/correction integration, CPU acceptance, SITL, props-off,
physical-course measurement, and constrained-flight gates remain. There is
intentionally no `offboard_global_planner_3d.launch.py` yet.

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
| `corner_blending` | `true` | carry speed through bends instead of stopping |
| `junction_deviation` | 0.05 m | how far the airframe may cut a corner |

At 0.20 m/s cruise and `d=0.05`, the worst observed corner (70 deg) allows
0.26 m/s — above cruise — so no observed corner needs any slowdown and the
stop-and-wait disappears outright. The cap starts to bite if `command_speed` rises
or routes get sharper. `junction_deviation` must stay well inside the follower's
`max_cross_track`, because it is what the vehicle is permitted to cut.

Blending is now the launch default after repeated armed flights, including the
completed `20260829T114405Z` tunnel run. Set it false to restore a full stop at
every vertex.

`path_command_suffix_tolerance` defaults to 0.03 m. In that tunnel run, seven
republications of the same retained physical route moved 1.1-2.4 cm in map
coordinates as the SLAM correction filter converged. The former 0.01 m match
treated them as new paths and restarted the command rejoin from zero speed;
0.03 m preserves speed across those coordinate-only updates. Genuinely changed
routes still use the bounded rejoin.

### Which flight adapter

Two implementations of the same flight adapter exist. `cpp_mode` selects which
process `offboard_global_planner.launch.py` starts; both take the identical
parameter set (the launch file builds one dict and hands it to whichever runs).

| `cpp_mode` | executable | node |
|---|---|---|
| `false` | `offboard_global_planner` | legacy Python implementation |
| `true` (default) | `cpp_flight_adapter` | flown C++ implementation |

**`cpp_nodes` is the master switch.** Both launch files accept it under the same
name, and every per-node C++ flag defaults to it, so one argument moves the whole
stack:

```bash
ros2 launch px4_vio_bridge global_planner_monitor.launch.py  cpp_nodes:=true ...
ros2 launch px4_vio_bridge offboard_global_planner.launch.py cpp_nodes:=true ...
```

| flag | defaults to | selects |
|---|---|---|
| `cpp_nodes` | `true` | every node below |
| `cpp_astar` | `cpp_nodes` | `cpp_astar_planner` instead of `global_planner_monitor` |
| `cpp_follower` | `cpp_nodes` | `cpp_route_follower` instead of `route_follower_monitor` |
| `cpp_mode` | `cpp_nodes` | `cpp_flight_adapter` instead of `offboard_global_planner` |

A single node can still be pinned back to Python to bisect a regression:
`cpp_nodes:=true cpp_astar:=false`. Because the two launch files are separate
commands there is no shared process to carry the toggle — spelling the same
argument on both is the whole mechanism.

**The Python planner and follower are legacy.** They are kept for reference and
for the parity tests that still cover the shared surface, but the 2026-08-29
stability work — the monotonic clearance escape, correction-canonical accepted
paths, generation-paired correction holds and goal-mode hysteresis — exists only
in `cpp_astar_planner` and `cpp_route_follower`. Pinning either back to Python
now reverts real fixes, not just an implementation language.

`cpp_shadow:=true` runs a third, **non-commanding** node (`cpp_clearance_shadow`)
alongside the Python adapter. It publishes no `/fmu/in/*` topics — it re-derives
the Python adapter's final map-frame command and independently checks clearance,
reporting on `/planner/flight/cpp_shadow/*`. It exists for parity and CPU
measurement while Python keeps flight authority.

The C++ port's command math is parity-tested against the Python limiters to 1e-9
over randomized replay (`test_planner_flight_parity.py` drives both through the
`planner_flight_replay` binary). Its state machine, watchdogs and PX4 command
sequencing are **not** covered by that test.

The C++ route follower is separately parity-tested against Python to 1e-9 over
randomized routes, replans, clearance refusals, cross-track recovery and arrival
hysteresis (`test_route_follower_parity.py`). Both follower executables publish
the same topics, take the same parameters and share the same singleton lock.

The three executable names deliberately share no substring, because
`process_monitor` matches on command lines — see
[Process load](#process-load--perf).

### Control mode: position, not velocity

`OffboardControlMode` is published in exactly one place
(`offboard_hover.publish_offboard_mode`) and sets `position=true`,
`velocity=false`, `acceleration=false`. Every flight mode inherits it. The
velocity and acceleration fields of `TrajectorySetpoint` are **feedforward terms
summed into the position loop**, not commands — nothing in this repo has ever
flown velocity control.

The practical consequence: a hold is not "stop", it is "return to the latched
point". When the follower blocks, PX4 actively flies the vehicle back to the
position it was blocked at.

### Everything else worth knowing

`rate_hz` is **20 Hz** for the global-planner adapter; every other offboard mode
runs the node default of 50. PX4 only requires the offboard stream to stay above
about 2 Hz, so 20 keeps a wide margin while matching the rates that actually feed
the adapter (follower 10 Hz, VIO 11-15 Hz, planner 2 Hz). Dropping 50 to 20 was
expected to save CPU and did not — see [Measured facts](#measured-facts).

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

Labels: `slam`, `xrce_agent`, `vio_bridge`, `planner`, `planner_cpp`,
`planner_cpp_shadow`, `astar`, `astar_cpp`, `follower`, `follower_cpp`,
`planner_sim`, `bag_record`,
`foxglove`, `battery`, `px4_pos`. They are parameters (`labels` / `patterns`), so
the set can be retargeted without editing code.

Patterns match **command-line substrings**, which has two consequences worth
knowing:

- Node executables are matched by their install path
  (`lib/px4_vio_bridge/offboard_global_planner`), not by bare name. Until
  2026-08-28 the bare name also matched `ros2 launch px4_vio_bridge
  offboard_global_planner.launch.py`, so the launch process's own CPU was added
  to the adapter's row — and in a `cpp_mode` run, where no Python adapter exists
  at all, that row was measuring *only* the launcher. `astar` had the same
  defect. **Bags recorded before 2026-08-28 09:00 carry roughly 1.5-3 percentage
  points of launcher CPU in the `planner` and `astar` rows.**
- No label's pattern may be a substring of another's, or one process fills two
  rows. This is why the adapters are named `offboard_global_planner`,
  `cpp_flight_adapter` and `cpp_clearance_shadow` rather than sharing a prefix.

CPU follows psutil's convention — **100% is one core**, so threaded RTAB-Map
legitimately reads above 100 on this 4-core Pi. `cpu_percent_of_machine` in the
JSON is the normalised companion.

Check `/perf/throttled` first when reading a bag. Non-zero means the board
browned out or overheated and the clock was cut, which makes any timing analysis
meaningless. Bits 0-3 are under-voltage / frequency-capped / throttled / soft
temperature limit *now*; bits 16-19 are the same four latched since boot.

### Seeing a flight: `scripts/render_flight_map.py`

Clearance failures are statements about geometry, and reading them one status
line at a time is how a whole flight goes by before the shape of the problem is
obvious. This renders the occupancy grid with the trajectory on it:

```bash
python3 scripts/render_flight_map.py \
  ros2_ws/flight_logs/offboard_global_<stamp> --clearance 0.25 --events
```

Offline only — it reads an MCAP bag and writes PNGs into `<bag>/render/`. Black
is occupied, dark slate is unknown (which is *blocked*, not free), light grey is
known free, and the peach band is everything within `--clearance` of an
obstacle: the space the vehicle may not normally command into. The track is
coloured by follower state (green `FOLLOWING`, amber `CLEARANCE_ESCAPING`, red
`CLEARANCE_BLOCKED`, magenta cross-track, blue correction settling), the blue
line is the accepted path as it stood at that instant, white is the requested
goal and orange the effective one.

It also prints a fault table with the pose's exact clearance at each event:

```text
   time  source   pose clearance  status
  31.33  adapter         0.243 m  COMMAND_HOLD post-limiter command has insufficient clearance
  31.42  follower        0.227 m  CLEARANCE_ESCAPING start=0.227m end=0.336m required=0.250m
```

Two properties matter for trusting it. Every clearance question is answered
against **the grid current at the time asked** — scoring a mid-flight event
against the final map is a different and more flattering question, worth about
7 cm in the example above. And the printed figures are *exact*: distance from
the pose to the full axis-aligned square of each occupied cell, the same measure
`segment_minimum_clearance()` uses in flight, which is why the tool independently
reproduces the follower's own `start=0.227m`. The shaded band is a distance
transform accurate to about `resolution / supersample` and is there to show
shape, not to be read off.

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

The flown planner consumes only the projected 2D grid. The separate fail-closed
3D architecture in [`HANDOFF_3D_NAVIGATION.md`](HANDOFF_3D_NAVIGATION.md) now has
an observation-only producer/planner/follower/replay implementation, but is not
a flight-ready launch mode.

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

Key launch defaults: `hover_height` 0.30, `command_speed` 0.20,
`geofence_radius` 3.0, `max_flight_time` 60, `yaw_rate_deg` 20,
`min_vio_features` 80, `planner_fault_land_time` 6.0, `goal_hold_time` 3.0,
`rate_hz` 20.0, `cpp_mode` true, `cpp_shadow` false. `auto_arm` remains false and
must always be opted into explicitly.

`cpp_mode:=true` replaces the Python adapter process with the C++ one;
`cpp_shadow:=true` co-runs the non-commanding clearance shadow alongside Python.
See [Which flight adapter](#which-flight-adapter).

A cross-track violation is **latched**: the follower freezes its relative carrot
and stays invalid until it receives a newer path *and* cross-track stays below
`cross_track_resume` (0.06 m) continuously for `cross_track_recovery_time` (1.0 s).
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

`cpp_nodes:=true` selects both `cpp_astar_planner` and `cpp_route_follower`.
Pin either back independently with `cpp_astar:=false` or
`cpp_follower:=false`. Each C++ process takes the identical parameters and the
same role-specific singleton lock as its Python counterpart, so two
implementations can never drive the same topics. Never delete a lock file to
clear a "duplicate" error — kill the holding process instead
(`pkill -f lib/px4_vio_bridge/<executable>`); deleting it lets a second process
take a new inode's lock while the first still holds the old one.

A click is accepted even if it lies in unknown space, outside the map, or on an
obstacle; unknown and lethal cells stay blocked. For unknown/outside/disconnected
requests the planner reports `EXPLORING` and flies only to the closest reachable
known-safe frontier, updating as the map expands. For obstacle clicks it reports
`SAFE_APPROACH` and stops outside the 0.30 m lethal envelope. The orange
`/planner/effective_goal` marker shows where the route actually ends; white is
the requested goal. A temporary exploration frontier is not reported as arrival,
so the adapter holds rather than landing there.

#### Stability parameters (C++ nodes)

The three parameters below exist because flight
`offboard_global_planner_20260829T071555Z` spent 28 of its 57 route seconds
oscillating between `PATH_VALID` and `EXPLORING`, produced 124 plans for 2.51 m
of net progress, and froze three times with the pose inside its own clearance.

| argument | default | effect |
|---|---|---|
| `mode_confirmation_maps` | 2 | distinct **occupancy grids** — never planner ticks — that must agree before a semantic mode change is committed |
| `escape_minimum_improvement` | 0.01 m | clearance a sub-clearance escape chord must gain at its endpoint before it counts as recovery. Set it on the follower only: the adapter reads it from `/planner/follower/config` alongside the clearance it belongs to, so the two cannot disagree |
| `correction_rearm_guard` | 0.20 s | replaces the old blind 8 s correction cooldown; long enough that a settling episode cannot reopen on its own residual |

Three things follow from them.

**A pose already inside the clearance can move again.** `safe_lookahead` used to
test the whole chord *including the pose*, so a pose 0.24 m from an obstacle
failed every candidate by construction. The follower now measures the exact
clearance of the pose and, only when it is already below the envelope, accepts a
chord whose every point is at least as far from the obstacle as the pose already
is and whose endpoint gains at least `escape_minimum_improvement`. Status reads
`CLEARANCE_ESCAPING start=0.238m end=0.252m required=0.250m status=FOLLOWING`,
and `CLEARANCE_BLOCKED reason=POSE_INSIDE_CLEARANCE_NO_ESCAPE` when no such
chord exists. Normal chords still require the full hard clearance, unchanged.

The flight adapter enforces the same rule on the chord it actually puts on the
wire. It has to: its post-limiter gate was a plain `segment_has_clearance`
threshold, so it vetoed every escape the follower proposed and the deadlock
simply moved one layer up, from `POSE_INSIDE_CLEARANCE` to
`COMMAND_HOLD post-limiter command has insufficient clearance` (flight
`20260829T085734Z`, adapter veto at 31.33 s against `CLEARANCE_ESCAPING
start=0.227m`). The adapter validates the acceleration-limited command, so it
applies the non-worsening half of the rule only — demanding a centimetre of gain
from one 20 Hz step would reject every escape again. The endpoint-improvement
half stays with the follower's target selection. When a command is on the wire
under the escape rule the adapter says so: `ROUTE valid ... escaping
clearance=0.227/0.250m`.

**A loop closure is not cross-track.** The accepted route, the commanded
displacement and the command velocity are stored in continuous VIO coordinates
and rendered into whichever map solution the current correction describes, so a
4-6 cm correction moves pose and route together. Re-publishing the same physical
route under a new correction is a coordinate change, not a new path generation,
and does not reset route progress.

**A post-correction episode defers new paths, it does not stop the aircraft.**
The planner publishes `/planner/map_generation` and
`/planner/path_map_generation`; the follower accepts no new path until the
correction has been quiet for `correction_settle_time` *and* a path planned from
a grid received after the last material correction step has arrived.
`/planner/correction_epoch` counts the episodes. A path built from the
pre-correction grid can no longer be installed, and a run of material steps a
second apart can no longer hide inside one cooldown.

What the episode does **not** do any more is stop the follower commanding.
Holding validity low cost up to ~2 s — two thirds of the adapter's 3 s
`planner_fault_land_time` — in flight `20260829T085734Z`, and the wait is
structural: quiet time, plus the next map at ~1 Hz, plus the next plan at 2 Hz.
It was guarding against a frame mismatch that correction-canonical route storage
already removes: the route still being flown is re-expressed into the newest
correction every tick and its command is revalidated against the newest grid
every tick. So the follower keeps flying the route it has and only refuses to
take a new one on trust; the status carries `CORRECTION_SETTLING epoch=...`
while it does.

Mode debounce is deliberately **not** collision debounce: the retained route is
revalidated against every new raw grid and replaced immediately when it is
unsafe, whatever the pending transition says. While a transition is pending,
`/planner/goal_terminal` goes false on the first raw nonterminal result and is
promoted back only on confirmation, so a temporary exploration frontier can
never be reported as reaching the requested goal.

### `offboard_hover` — the base flight node

Latch NED x/y/yaw, stream `TrajectorySetpoint` at `rate_hz` (node default 50 Hz;
the global-planner launch sets 20), request OFFBOARD, arm, climb, hold, then
`NAV_LAND`. Every other flight node subclasses it, so the watchdogs below apply
everywhere. The C++ adapter reimplements this state machine rather than
subclassing it.

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

## Measured facts

Numbers here come from recorded bags, not estimates. Dates are the flights they
came from.

### SLAM dominates, and VIO rate is thread-bound — not core-starved

`RTABMapVIO` + `RTABMapSLAM` on the Pi run **~195-210% of a core** and are the
single largest cost by an order of magnitude. `--num-features` is **not** a CPU
lever — CPU sat at 215-230% across 100, 300 and 500 features — so keep
`slam_num_features` at 400-500 where tracking is healthy.

Per-thread profiling of the SLAM process (2026-08-29; sample
`/proc/<pid>/task/*/stat`, group by `task/*/comm`; 44 threads), flight grid args,
`--fps 20`, **nothing else running on the machine**:

```
RTABMapVIO(4)     101.0%   <- one host core, saturated
RTABMapSLAM(6)    100.5%   <- one host core, saturated
HostNode(9)         5.7%   <- the Python Ros2Node: ALL of the Python in this process
XLink / Sync / Scheduler / dds    ~8%
every bare python3 thread          0.0%
```

Two consequences, both load-bearing:

1. **Freeing cores cannot raise the VIO rate.** Those two threads are each
   single-threaded and already at 100% of a core. On an otherwise idle machine
   VIO still only reached 11.6-13.0 Hz, so `bag_record` was not the cause. The
   measured levers are marginal: `--publish-image false` takes 11.6 -> 12.0 Hz
   (and `HostNode` 7.1% -> 4.4%); adding `--publish-grid false` reaches 12.25 Hz.
   Reclaiming CPU is still worth doing for headroom and jitter — it just is not
   how the rate goes up.
2. **Porting the SLAM host script to C++ would reclaim ~5.7% of a core, not
   ~200%.** `RTABMapVIO`, `RTABMapSLAM`, `StereoDepth` and `FeatureTracker` are
   DepthAI nodes already implemented in C++ inside libdepthai; the Python is only
   the `Ros2Node` marshalling layer. The reference C++ pipeline
   (`depthai-core/examples/cpp/RVC2/VSLAM/rtabmap_vio_slam.cpp`) mirrors this
   script almost exactly and is buildable here — RTABMap 0.22.1, PCL 1.14 and
   OpenCV 4.6 are all installed, so `-DDEPTHAI_RTABMAP_SUPPORT=ON` finds them —
   but the ROS-shipped `libdepthai-core.so` has **zero** RTABMap symbols, so it
   means rebuilding depthai-core from source for a few percent of a core.

### VIO backend: Basalt is faster and cheaper, RTABMapVIO holds position better

`--vio-backend basalt|rtabmap`, identical flight grid args, OAK-D **stationary**
on a desk (2026-08-29). `BasaltVIO` runs on the OAK-D, so it removes the pinned
`RTABMapVIO` host thread entirely:

| | RTABMapVIO | BasaltVIO (stock) |
|---|---|---|
| VIO rate | 11.6-13.0 Hz | **19.4-20.0 Hz** (hits the requested fps) |
| process CPU | 204-212% | **155%** |
| VIO host thread | 101%, pinned | **gone** — ~50% in XLink transport instead |
| `RTABMapSLAM` thread | 100.5% | 100.1% (unchanged) |
| drift, 45 s stationary | **0.45 mm** | 19.29 mm |
| max single-frame step | **1.5 mm** | 15.2 mm |

This is the only change measured so far that moves the VIO rate at all, and it
frees half a core doing it. It is not yet a decision: a 15 mm single-frame jump
fused into EKF2 at 20 Hz is a ~0.3 m/s velocity spike, and the follower already
spends 8.8% of its time in `POSE_INSIDE_CLEARANCE`.

#### Tuning Basalt

The node otherwise runs entirely on `setDefaultVIOConfig()`, whose settings come
from Basalt's EuRoC reference rig — 1 point per 50 px cell (only ~104 points at
640x400) and an industrial-grade IMU. The `--basalt-*` flags expose the knobs;
all default to the node's own defaults, so `--vio-backend basalt` alone is
unchanged. Measured on this airframe, 45 s stationary:

| config | Hz | drift mm | max step mm | rms step mm |
|---|---|---|---|---|
| RTABMapVIO (reference) | 13.0 | **0.45** | **1.53** | 0.639 |
| Basalt, stock defaults | 19.4 | 19.29 | 15.23 | 0.747 |
| + measured IMU noise, `init-bg-weight 1` | 19.4 | 11.30 | 7.82 | 0.527 |
| + `grid-size 30`, 2 pts/cell | 19.4 | 26.30 | 4.34 | 0.420 |
| + `obs-std-dev 0.25`, `image-safe-radius 320` | 18.3 | 11.16 | 2.32 | **0.349** |
| same, but default `init-bg-weight` | 19.4 | 19.28 | 7.55 | 0.618 |

What that says:

- **The IMU model is the biggest single lever.** This OAK-D measures a real
  stationary gyro bias of ~0.0035 rad/s (0.2 deg/s) — enough to integrate to ~9
  degrees of attitude over 45 s. Supplying the measured noise and weakening the
  prior that pins gyro bias at zero (`--basalt-init-bg-weight 1.0`) halves the
  drift; restoring the default prior puts it straight back (last row).
- **Jitter is essentially solved, slow drift is not.** The tuned config brings
  the max single-frame step from 15.2 mm to 2.32 mm and the rms step to 0.349 mm,
  *better* than RTABMapVIO. What remains is slow bias walk: 11 mm over 45 s
  against RTABMapVIO's 0.45 mm.
- **`setAccelNoiseStd`/`setGyroNoiseStd` take VARIANCE**, not standard deviation
  — depthai applies `cwiseSqrt()` to whatever it is given. `setAccelBias` /
  `setGyroBias` are deliberately unused: in depthai 3.5.0 they comma-initialise a
  12-element Eigen vector from 9 values.
- **One setting is unreachable from Python.** `MatchingGuessType`,
  `LinearizationType` and `KeyframeMargCriteria` are unregistered pybind types in
  3.5.0, so `setConfig()` leaves them value-initialised at 0. For two that is the
  intended default; for the third it silently demotes
  `optical_flow_matching_guess_type` from `REPROJ_AVG_DEPTH` to `SAME_PIXEL`.
  Reaching it needs the C++ node — a better argument for the C++ port than CPU.
- **Some combinations abort inside Basalt.** `vio_max_kfs 12` + `vio_max_states 5`
  together with the denser grid tripped an assertion in
  `LandmarkBlockAbsDynamic::allocateLandmark` during marginalization. Change one
  knob at a time.

Best config found so far:

```bash
--vio-backend basalt \
  --basalt-accel-noise-var 1.637e-4 1.559e-4 1.243e-4 \
  --basalt-gyro-noise-var  1.070e-6 1.180e-6 1.172e-6 \
  --basalt-init-bg-weight 1.0 \
  --basalt-grid-size 30 --basalt-points-per-cell 2 \
  --basalt-obs-std-dev 0.25 --basalt-image-safe-radius 320
```

**All of the above is stationary**, which is the worst case for a tightly-coupled
IMU VIO like Basalt and the best case for RTABMapVIO — IMU excitation in flight
makes the biases observable. Do not conclude from this table alone. The next test
is a hand-carried closed loop returning to a marked start, scoring loop-closure
error, before Basalt goes anywhere near a flight. `rtabmap_slam_px4.launch.py`
does not expose `vio_backend`; add the launch argument first.

### The C++ A* planner is worth ~30 points of a core

`cpp_astar_planner` against `global_planner_monitor`, replaying the real
`094653Z` flight grid (269x272 @ 3 cm), 8 runs per stage (2026-08-29):

| stage | Python | C++ | |
|---|---|---|---|
| inflate costmap | 144.2 ms | 14.6 ms | 10x |
| goal-selection BFS | 41.1 ms | 0.25 ms | 163x |
| A* search | 2.4 ms | 0.15 ms | 16x |
| **total per plan** | **187.8 ms** | **15.0 ms** | **13x** |

At `rate_hz:=2.0` that is **38% of a core down to 3.0%**, consistent with the
~30% the `astar` row shows in flight bags.

Worth knowing: **the Python planner exceeds its own planning budget.**
`planning_timeout_ms:=100` covers goal selection plus A* only, where Python
spends 44 ms — but inflation, which the timeout does not cover, is another
144 ms. Each Python plan occupies ~188 ms of a 500 ms tick. C++ is 0.41 ms inside
the budget and 15.0 ms in total.

The two implementations are held together by `test_grid_planner_parity.py`, which
drives both over randomized maps through the `grid_planner_replay` binary and
compares the costmap, display encoding, start recovery, goal selection, A* cells,
expansion count, cost and simplified path exactly. Replaying a real flight bag
through each node end to end gave 51 accepted paths from both, the same status
mix, the same first path length (2.51 m), the same peak expansions (1629) and the
same waypoint counts.

C++ inflation uses a different loop shape from the Python one and is *not* a
transliteration: `grid_planner.py` dilates the whole mask once per kernel offset
because in numpy each pass is a single vectorised max, whereas scalar C++ pays
per cell touched, so it stamps the kernel around each obstacle instead
(`obstacles x offsets` rather than `width x height x offsets`). Flight maps are
~8% occupied, which makes that ~12x cheaper. It picks between the two loops at
runtime, so a map dense enough to invert the tradeoff still takes the mask path.
Both produce identical costmaps, which is what the parity test pins.

### The C++ adapter is worth ~23 points of a core; the rate change was not

Both adapters measured at 20 Hz, launcher CPU excluded (2026-08-28):

| | Python | C++ |
|---|---|---|
| Adapter CPU | **29.2%** of a core | **6.6%** |
| RSS | 85 MB | 39 MB |
| Machine total | 82.1% | 77.3-80.2% |
| SLAM | 201% | 204-209% |

The rate change was separately worthless: the Python adapter cost ~30.5% at 50 Hz
and 29.2% at 20 Hz — about **1.3 points for a 2.5x rate cut**. Its cost is rclpy
fixed overhead in the callbacks, not per-tick work, so essentially the whole
22.6-point saving is the language.

Machine-level gain is much smaller than the adapter delta, because **SLAM expands
into whatever is freed** (201% to 204-209%). Do not expect adapter savings to
show up one-for-one in `/perf/cpu_percent`.

### Bag recording costs ~16% of a core, and it is already C++

`ros2 bag record` is a Python CLI around a C++ recorder: `rosbag2_py.Recorder` is
a pybind11 wrapper over `rosbag2_transport::Recorder`, and the verb only parses
arguments before calling `record()`, which blocks inside C++. Per-message work
happens in C++ subscription callbacks with no Python involved. **Rewriting the
recorder in C++ would gain nothing** — it is not a language cost.

Measured by replaying a real flight bag (`094301Z`, ~50 topics, 611 msg/s) and
profiling the recorder's threads over 30 s (2026-08-29). Use thread sampling, not
`ps -o %cpu`, which reports a lifetime average and overstates a short run:

| recorded set | steady-state CPU |
|---|---|
| everything | **16.5%** of a core |
| minus the 3 ~95 Hz PX4 topics (47% of all messages) | 13.4% (−3.1) |
| minus 16 redundant `/perf/*` scalars | 15.1% (−1.4) |
| a single 12 Hz topic (the floor) | 0.4% |

Both trims together land near 12%, a ~27% reduction — worth doing, not
transformative. Note that dropping 47% of the messages removed only 19% of the
CPU, so cost is split between per-message work and per-subscription overhead
across 77 recorded topics.

What to trim, in order of value per unit of lost capability:

1. **The 16 `/perf/*` scalar topics.** `/perf/processes` already carries
   everything they do — `cpu_percent`, `cpu_temp_c`, `load1`, `mem_percent` and
   a full per-process table — plus `per_core` and thread counts the scalars lack.
   Keep publishing them so the Foxglove plots keep working; just leave them out
   of the recorder's `--topics` list. Zero information lost.
2. **`/fmu/out/vehicle_odometry`** — 15.6% of messages and 10.7% of bytes, and
   its only consumer is `calibrate_ev_position_offset.py`, run deliberately.
3. `/fmu/out/sensor_combined` costs `analyze_flight.py` its gyro-RMS section;
   `analyze_ulog.py` covers it if the ULog is fetched. **Keep**
   `/fmu/out/vehicle_attitude` and `/fmu/out/vehicle_local_position_v1` —
   `analyze_flight.py` reads the *bag*, not the ULog, and they carry the
   roll/pitch/yaw and position analysis.

By bytes rather than messages the picture is different: **`/rtabmap/grid` alone is
53.7% of the payload** (2.76 MB of 5.13 MB) from only 49 messages at 1 Hz. Do not
drop it — it is what makes offline planner replay and `evaluate_planner_bags.py`
possible, and 49 messages cost almost no CPU. For disk instead of CPU, add
`--compression-mode file`: the bag compresses 6.31 MB to 1.41 MB (4.5x) in 0.2 s,
and file mode compresses at close, i.e. after landing rather than in flight.

`--max-cache-size` is already 100 MB — a whole flight fits — and `fastwrite` is
already the lowest-CPU storage preset. Neither is a lever.

### The planner routes well; the vehicle is what goes out of tolerance

Flight 091006Z, 2026-08-28. Sweeping +/-0.60 m laterally at every point of the
published path:

- path clearance **0.36-0.58 m** along its whole length
- the best available lateral alternative was only **0.05-0.15 m** better
- vehicle clearance over the same window fell to **0.22 m**

So the A* route was within 0.15 m of the local optimum and the excursions were
the vehicle's, not the planner's. The map was not moving either: holding one
fixed point and re-evaluating it against every published map over 6 s gives
0.370 m *every time*.

The path's first vertex is anchored at the vehicle, so it inherits whatever
position the vehicle occupies — it cannot start at the corridor centre.

### Corner blending removes a real stop

`PathCommandLimiter` without blending stops at every vertex; measured A* paths
carry 4.5-6 vertices over 1.7-2.7 m, so that is a full stop roughly every 50 cm
and about half the commanded cruise. Measured corners are median 33 deg, p90
44 deg, max 70 deg; at `a=0.30` and `d=0.05` even the 70 deg worst case allows
0.26 m/s against a 0.20 m/s cruise, so no observed corner needs any slowdown.
Flown 2026-08-28 with `corner_blending:=true`.

## Known issues

### `POSE_INSIDE_CLEARANCE` was a deadlock — fixed in the C++ follower

`safe_lookahead` tested `segment_has_clearance(pose -> target)` and **the segment
starts at the pose**. Once the vehicle's own position violated
`required_clearance`, every candidate lookahead failed at its first point —
including the shortest — so the search could not succeed by construction. The
follower then called `hold_command()`, the carrot collapsed onto the vehicle, and
commanded motion went to zero. Because the stack is position-controlled, that was
an active instruction to stay in the offending spot.

Observed 2026-08-28: flight 091228Z spent its final **8 s** frozen 0.27 m from an
obstacle (airframe radius 0.25 m) and never reached its goal; flight 091006Z
chattered BLOCKED/FOLLOWING three times in 5 s while sitting on the threshold.
Both escaped only via position-hold error — the vehicle sagging ~0.12 m off its
own latched setpoint — i.e. by luck of which way the airframe drifts.

`cpp_route_follower` now measures the exact continuous clearance of the pose
(`point_clearance`) instead of only threshold-testing it. Above the envelope
nothing changed: a normal chord must still clear `robot_radius +
safety_margin` in full. Below it, and only there, the follower searches the same
lookahead candidates for an *escape*: every point of the chord must be at least
as far from an occupied cell as the pose already is, and the endpoint must gain
at least `escape_minimum_improvement` (0.01 m). The acceleration-limited carrot
is checked with the same predicate, not just the desired lookahead, and entering
the escape drops any stale displacement that still pointed at the obstacle.
Recovery is therefore monotonic and deterministic rather than a matter of which
way the airframe happens to drift. Lateral motion at constant clearance is
rejected, so a route that merely follows the same unsafe contour cannot pass as
recovery; when no chord qualifies the follower publishes
`CLEARANCE_BLOCKED reason=POSE_INSIDE_CLEARANCE_NO_ESCAPE`, `valid=false`, and
the adapter holds and lands as before.

The Python `route_follower_monitor` is legacy and still has the original
deadlock; run the follower with `cpp_follower:=true` (implied by
`cpp_nodes:=true`, which is what is flown).

### Clearance budget: `safety_margin` must cover `max_cross_track`

The planner guarantees every point of the path clears
`lethal_radius = robot_radius + safety_margin`. The follower separately permits
the vehicle to sit `max_cross_track` off that path. Nothing links the two, so the
vehicle's worst-case clearance is:

```text
lethal_radius - max_cross_track
```

At `robot_radius 0.25`, `safety_margin 0.05`, `max_cross_track 0.30` that is
**-0.02 m** — the airframe is permitted 0.27 m inside an obstacle. The invariant
that makes contact impossible is:

```text
safety_margin >= max_cross_track
```

which is not met by any configuration flown so far. Raising `safety_margin` costs
corridor width (`2 * lethal_radius`), so this is a deliberate trade, not a bug to
silently fix: at `safety_margin 0.0` the block threshold becomes the airframe
radius itself and the minimum plannable gap is 0.50 m; at `safety_margin 0.20`
contact becomes impossible but the minimum gap is 0.90 m.

### `path_retain_tolerance` inverts the replan/fault ordering

`should_replace_path` rebuilds the accepted path only when cross-track exceeds
`path_retain_tolerance`, while the follower faults at `max_cross_track`. If the
retain tolerance is not lower, the vehicle can stall in the band between them:
faulted, but with no new path coming. The operational launch defaults are 0.12 m
and 0.15 m respectively.

In four flights on 2026-08-28, cross-track never once exceeded 0.35 m — so a
cross-track excursion has never actually triggered a replan on this vehicle.

### A killed recorder leaves an unreadable bag

If the flight launch is stopped before `ros2 bag record` finalizes, `metadata.yaml`
is written 0 bytes and the bag will not open, though the `.mcap` itself is intact.
Recover with:

```bash
ros2 bag reindex flight_logs/<bag_dir> -s mcap
```
