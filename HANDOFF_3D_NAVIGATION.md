# 3D Occupancy Navigation Mode — Handoff

## Current flight continuation: unknown-allowed adapter

[The flight adapter handoff](HANDOFF_3D_FLIGHT_ADAPTER.md) describes the dedicated
PX4 consumer of the unknown-allowed reference, its launch and controls, command
checks, CPU recording and offline audit. This supersedes historical statements
that the 3D route has no flight consumer. The monitor remains observation-only;
flight requires the separate `offboard_planner_3d_unknown.launch.py` launch.

## Current behavior: level-camera exploration route

See [the level-camera planner update](PLANNER3D_LEVEL_CAMERA_20260915.md).
The sphere stays at 25 cm plus 10 cm margin. Up/down scan instructions are
withdrawn. The monitor now proposes a penalized-unknown global route and
keeps a separate verified local path for the follower. The update describes
the new topics, map-sampling optimization, recovery from stable SLAM frame
corrections, and current verification (752 tests, zero failures, three skips).

## Latest test: 06:34:47 — start connection and preparation cache

Read [the latest bag report](PLANNER3D_LATEST_BAG_20260915.md). The user confirmed
a 25 cm **radius**; required clearance remains 35 cm with margin. Fixed false
recovery for an actually clear off-center body and cached preparation within
each map generation. At that revision: 745 tests passing, three skips; replay
audit has zero errors. The recording still supports only short partial paths,
not the requested 2 m route. The report separates that coverage limitation
from the software fixes; its test procedure is superseded by the update above.

## Latest follow-up: visible blockers and current-height goals

Read [the guided scan handoff](PLANNER3D_SCAN_HANDOFF_20260915.md). A calibrated
optimistic-visibility test confirms that the existing yaw bag cannot connect
its 50 cm start clearing on the native grid, even if every recorded camera
view is granted free space. New forward-clearance markers identify the unknown
cells to observe. `goal_height_mode:=current` optionally latches a clicked
waypoint at fresh body height; default `clicked` still preserves XYZ. The
historical scan procedure is superseded by the level-camera update above.

## Latest comparison: voxel-perception-tuning and the 05:39 yaw test

Read [the branch comparison](VOXEL_BRANCH_COMPARISON_20260915.md) before another
carry test. The old documented setup used a 1 m unknown-only clear assertion,
decimation 2, range minimum 0.80 m, opt-in frustum fill, and a fixed planning
height. Those tuning commits were absent from the ROS migration. Controlled
reconstruction of the new bag still yields only 10–13 cm paths with the
requested 50 cm clearing; a diagnostic 1 m assertion plus dense measured rays
reaches about 1.9 m, still a partial route. The old cache bound and missing
ROS depth-tuning controls have been restored. Existing defaults, the 50 cm
limit, and current-generation clearance checks are retained. See the report
for measured results, limitations and current verification.

## Latest fix: stationary waypoint rejection, September 15 at 05:15

The latest stationary bag was blocked by **min_z=-0.20**: body Z was 0.056 m,
but the 0.35 m envelope requires a center at least 0.15 m above that configured
plane. The start's recorded OctoMap sphere itself was clear. Use the artificial
test band **min_z:=-0.8 max_z:=1.6** shown below; -0.2 repeats this failure.
See [the diagnosis and verification](WAYPOINT_START_DIAGNOSIS_20260915.md).

The test sphere now re-anchors at the current valid body pose on each fresh
waypoint click, then remains fixed until the next click. This supersedes the
startup-only anchoring described in earlier evidence below. Metadata includes
`anchor_sequence`, `anchor_reason`, and `goal_stamp_ns`; old clearing is not
accumulated in the sensor map.

Planner status now names `START_ENVELOPE_OUTSIDE_GEOFENCE` explicitly and records
the configured bounds and required center heights. Atomic paths carry
`goal_exact` / `goal_terminal`; reaching a partial endpoint is now
`EXPLORATION_ENDPOINT_REACHED` or `SAFE_APPROACH_REACHED`, with
`goal_reached=false`. Only the actual requested waypoint can report
`GOAL_REACHED`. Existing clearance and map-generation checks remain enforced.

The corrected test bounds unblock this bag's start, but its distant clicked
goals still overlap occupied/unknown space. Its resulting exploration paths
are only about 8 cm long. A 50 cm artificial clearing cannot establish the
rest of the room's clearance. Final checks: 739 tests, zero failures, three
intentional skips. Read the latest diagnosis before the historical carries.

## September 15 follow-up: test start volume, performance and CPU recording

Implemented and verified offline. See the [measured performance and test report](PLANNER3D_START_PERF_20260915.md).
Late wider-room map processing fell from **344 ms to 85 ms median**, with all
40 binary maps unchanged. Final package checks: **739 tests, zero failures,
three intentional skips**. A controlled stationary-start test audited 70 paths
and 422 follower chords without violations; it explicitly uses artificial
clearance and does not establish physical flight clearance. Replaying the
original carry motion with the final code still produces no route: the body
has already moved about 12 cm from the fixed anchor before the first map is
usable. Keep still until a nearby initial route is accepted on the next test.

The user requested an artificial 0.50 m free sphere at the initial drone body
position for observation-only planning tests.
Use `test_start_clear_radius:=0.5` with `start_backend:=true` in
`ros_planner_3d_monitor.launch.py`. The normal default remains zero.

The producer latches the first valid raw body pose, including the camera lever
arm, and applies each map's own correction to that anchor. The sphere does not
follow subsequent drone motion. Only native voxel boxes wholly within its
radius are forced free, including occupied cells. This changes a publication
copy; sensor evidence and OctoMap checkpoints are not carved. Each map records
`test_start_clear` with the raw anchor, corrected center, radius and artificial
status. `/planner3d/test_start_clear` displays the sphere. The route auditor
reports `uses_artificial_start_clear` and does not claim physical clearance
validation for such a map. Existing altitude/geofence bounds still apply.

The camera's valid depth rays already provide observed free-space evidence;
ray endpoints provide occupied evidence. Missing depth and space behind a
surface must stay unknown. No blanket camera-frustum clearing is being added.

CPU recording is enabled by default via `process_monitor:=true`. The native
profile covers camera/odometry and mapper containers, OctoMap, pose adapter,
3D planner/follower, recorder and Foxglove. Bags include `/perf/processes`
(per-core CPU, process-group CPU with PIDs, RSS, threads, system metrics), scalar CPU and
memory topics, temperature, load and throttle state. Process 100% means one
core; machine-wide CPU is normalized to 100%. The old wider-room bag contained
no `/perf` messages, so no historical CPU-utilization claim can be made for it.

`scripts/replay_ros_planner_3d.py --mapdata` replays saved MapData/raw odometry
without opening a camera; `--disable-checkpoints` selects the rebuild baseline,
`--test-start-clear-radius 0.5` enables the override, and `--recorded-goals`
reuses the source clicks. Omit the last flag and use `--goal-offset 0.05` for a
small diagnostic goal near the initial pose. It records the new CPU and map
stage timing metadata as well as map/path/pose evidence. `--hold-start-pose`
is a separately labelled synthetic startup diagnostic that holds the first
recorded raw body location with fresh timestamps; it does not replay the
recorded carry motion.

### Start the artificial-clearance test

With other camera-owning launches stopped, use:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
  start_backend:=true foxglove:=true record_bag:=true \
  test_start_clear_radius:=0.5 min_z:=-0.8 max_z:=1.6
```

The altitude band above is an **artificial observation-test band around the
initial frame**, not a measured floor/ceiling for a drone set on the ground.
It lets the start sphere fit in the planner bounds. Keep the drone unarmed,
with props removed. The older physical-clearance carry procedure below still
requires measured floor and ceiling planes. Removing the test override alone
does not turn this example altitude band into a physical clearance claim.

Keep the drone still at startup until the map and valid body pose appear.
Show `/planner3d/test_start_clear` in Foxglove along with the existing map and
path layers. Start with a goal about 5 cm from the initial body position at the
same height; then extend into observed free space. A 50 cm cleared sphere gives
a 35 cm drone envelope only about 15 cm of center travel, reduced further by
voxel discretization. It cannot by itself make a distant goal reachable, and
setting down away from the original anchor can still produce `START_BLOCKED`.

## Latest evidence — September 15: three live carries assessed

All three observation-only carries recorded poses/maps but accepted no routes.
See [the carry and timing assessment](CARRY_3D_ASSESSMENT_20260915.md).
The larger-space run rejected all 13 starts: the user had set the drone down
before selecting the goal, and the 0.35 m envelope overlapped occupied voxels
about 0.19 m away and unknown voxels about 0.09 m away. Earlier in the carry,
unknown patches still prevented a fully clear body envelope at all 77 sampled
poses. `POSE_VALID` does not imply a collision-free start.

Measured median raw pose age was 141 ms; median OctoMap age was 393 ms, with a
1.58 s maximum. Replacing the latest graph node while stationary repeatedly
triggers the adapter's node-removal rebuild; MapData-to-OctoMap arrival delta
rose from median 57 ms in generations 1–28 to 388 ms in generations 30–40.
A separate TF/input synchronization spike occurred near the goal click.
No active-run stale-map/pose planner rejection occurred in that third run.
The report preserves plots, exact-map probes, timing tables and remaining work.
The offline implementation checks below still stand; live navigation acceptance
has not passed, and the historical next-carry instructions need the above context.

## Resume here — September 15: ROS planner integration ready for a live carry

**Implemented and checked offline:** ROS-native odometry -> generation-paired
body pose -> OctoMap 3D A* -> observation-only XYZ follower. There is still no
3D PX4 flight adapter. The original 2D DepthAI mode remains separate and uses
`slam_grid_3d:=false`.

### What changed

- `cpp_ros_planner_pose` consumes `/rtabmap/vio_odometry` (`rtabmap_odom` / `oak`)
  and `/rtabmap/octomap_metadata`. It rejects wrong frames, nonfinite/non-unit
  poses, tracking-loss covariance, stale (>0.5 s), future and non-increasing
  sensor timestamps. It keeps raw camera odometry unchanged, and applies the
  existing FRD camera offset `(0.100, -0.036, 0.056)` m only to planner body poses.
- OctoMap metadata now carries `map_from_odom` as `[x,y,z,qx,qy,qz,qw]`, copied
  from the exact RTAB-Map `MapData.graph.map_to_odom` used for that map update.
  Missing/invalid map corrections do not silently become identity.
- `/planner3d/pose_snapshot` atomically carries the sensor stamp, map generation
  and map stamp, raw/corrected body poses, correction and offset. Planner and
  follower require it to match their accepted map. Public `/rtabmap/vio_pose`
  remains camera-origin in `rtabmap_odom`; `/rtabmap/pose` is camera-origin in
  `world`. The ROS navigation launch consumes the body snapshot, not these
  camera visualization poses. PX4 conversion continues to read raw Odometry.
- `/planner3d/path_snapshot` carries path coordinates, stamp, map generation,
  map stamp and clearance radius together. The follower consumes this atomic
  message. `/planner3d/path` and generation topics remain visualization and
  compatibility telemetry. Empty, mismatched, wrong-frame and future routes clear the active follower;
  identical path heartbeats preserve its carrot and velocity. Stale checks use
  sensor time as well as arrival time. Delayed older paths cannot refresh the
  accepted path lifetime. The follower now declares `path_snapshot_topic`; the
  old unused `path_topic` parameter was removed. No pose callback repeatedly resets the
  follower's command point. Watchdog rejection resets the internal carrot and
  feedforward so recovery starts from the current pose at the acceleration limit.
- `ros_planner_3d_monitor.launch.py` attaches to an existing ROS mapper by
  default; `start_backend:=true` starts the pinned ROS perception profile with
  no PX4 parent launch. Its monitor disables the second OctoMap producer.
  Simulation clock, recording and raw-frame settings propagate to consumers.
- The ROS wrapper now preserves an explicitly selected `ROS_DOMAIN_ID` across
  the Pi environment scripts. Unspecified domains still default to 42.
- `scripts/audit_planner_3d_bag.py` uses `octomap_path_audit` to recheck recorded
  accepted paths and both follower chords against their exact binary OctoMaps.
  Unknown and occupied space both reject a swept sphere. No accepted paths or
  no valid follower chords produces an incomplete/failed audit, never a pass.
  Duplicate map/metadata timestamps are rejected as ambiguous. The auditor
  caches 16 generations; records delayed beyond that window fail rather than
  being checked against a different map. The producer rejects non-increasing
  source map stamps. Timestamp zero is invalid, including at simulated startup.

### Fresh verification and evidence

Package build succeeded. All **37 CTest targets** passed; `colcon test-result`
reports **718 tests, zero errors/failures, three skips**. The skips are the
existing C++ rolling-join versus Python stop-on-replan flight parity cases,
not 3D integration tests.

The real ROS synthetic test injects native Odometry, keyframe grids and a
0.20 m graph correction, then checks XYZ goals, body offsets, stale/future/
wrong-frame/nonfinite/lost odometry, invalid altitude goals, stale maps and
out-of-order path generations. Faults withdraw validity and recover on good
inputs. It also proves that occupied, unknown and wrong-generation audit
fixtures reject. The preserved run audited **17 paths, 202 follower chords,
287 segments across 16 maps, zero violations**:

- `/home/john/planner3d_integration_20260915/test_ros_octomap_navigation_fa0/report.json`
- Same directory: `bag/`, `audit.json`, per-process logs.
- Build/test logs are preserved under `/home/john/planner3d_integration_20260915/`
  as `build.log`, `package_test.log`, `package_test_result.log` and `legacy_launch.log`.

One read-only worker audited the contracts and reviewed the changes. The lead
implemented and integrated the fixes and ran all build, graph and replay checks.
The original synthetic monitor launch also reaches `FOLLOWING` after the changes,
with its raw fixture frame matching the follower's declared frame.

Replayed only RGB-D, CameraInfo and static TF from
`/home/john/ros_live_delay_20260913T121300Z/bag` through the full pinned ROS
estimator + mapper + OctoMap + planner/follower on isolated domain 178.
No camera opened and no `/fmu/in/*` topic existed in that graph. Output:
`/home/john/planner3d_saved_replay_20260915/` (`bag/`, `report.json`,
`samples.jsonl`, `start_probe.json` and process logs).

That replay produced **680 valid body snapshots and 37 map updates**, but no
accepted route: `START_BLOCKED`. The binary-map probe confirms the airframe
center itself is **unknown**, with only two source keyframes (372 ground,
973 obstacle, 21,234 empty cells). This largely stationary recording does not
observe the surrounding airframe volume. Keep unknown blocked; the next test
requires moving into volume previously scanned by the camera. The initial
replay observer used incompatible reliable QoS for raw Odometry, so its raw
Odometry counter is absent; the adapter and recorder consumed it, and the
reusable replay script now requests best-effort QoS.
The reusable script was then run successfully at
`/home/john/planner3d_saved_replay_final_20260915/`: **684 raw Odometry messages,
677 valid body snapshots, 37 maps**, `pose_interface_valid=true`, and again
`START_BLOCKED` with no accepted route. Both replay graphs shut down cleanly.
A separate duplicate-stamp bag control correctly fails the audit with both
map and metadata duplicate errors; see
`/home/john/planner3d_integration_20260915/duplicate_stamp_audit.json`.

### Next action requiring the user: props-off carry

1. Stop any existing camera-owning stack. Keep the vehicle unarmed with props
   removed. Start level and measure the camera's initial height above the floor
   and the floor-to-ceiling height. In the initial `world` frame, floor Z is
   minus the camera height and ceiling Z is ceiling height minus camera height.
   `min_z` / `max_z` are physical boundary planes; the follower additionally
   keeps its 0.35 m sphere inside them. Replace defaults with those measurements.
2. Use the ordinary workspace sourcing from README, then start this single
   observation-only graph (example values below assume a camera initially
   0.80 m above a floor with a 2.40 m ceiling; they are not site measurements):

   ```bash
   ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
     start_backend:=true foxglove:=true record_bag:=true \
     min_z:=-0.80 max_z:=1.60
   ```

   No Micro XRCE agent is needed for this standalone perception/planner test.
   For an already-running `backend:=ros` graph, omit `start_backend:=true` and
   keep `foxglove:=false` so only one bridge owns port 8765. Do not attach this
   launch to the DepthAI-only 2D mapper; it needs ROS MapData-derived metadata.
3. Foxglove frame `world`: show `/rtabmap/octomap_ground_markers`,
   `/rtabmap/octomap_obstacle_markers`, `/planner3d/path`, `/planner3d/markers`
   and `/planner3d/follower/carrot`. Watch `/planner3d/status`,
   `/planner3d/pose_status` and `/planner3d/follower/status`.
4. Observe the corridor, floor, overhead and side clearance, then carry into
   that previously observed volume. A stationary startup may correctly remain
   `START_BLOCKED`. Publish an XYZ `PointStamped` on `/waypoint/clicked` in
   `world` to a nearby point inside scanned space, initially at the current
   body height. Then test a modest height change in already observed space.
   Carry slowly along the displayed route; this monitor cannot move the drone.
   Record a small return loop to exercise a real graph correction if possible.
5. Stop with Ctrl-C and let recording finalize. From the repository root:

   ```bash
   python3 scripts/audit_planner_3d_bag.py /path/to/recorded/bag \
     --out /path/to/new-audit.json
   ```

   Required evidence: correct physical map orientation/scale and floor,
   known-free body envelope, accepted XYZ paths, zero recorded clearance or
   generation violations, and measured timing under motion. A null route is
   not a flight success. Map/pose rejection reason and the bag guide the next fix.

### Reproduce offline

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1
colcon test --packages-select px4_vio_bridge
colcon test-result --verbose
cd ..
python3 scripts/replay_ros_planner_3d.py /path/to/source/bag --out /path/to/new-output
```

The replay helper disables camera startup and plays an explicit sensor-topic
allowlist; default replay domain is 178, and domain 42 is rejected. It uses
simulation time and records new OctoMaps/snapshots. It deliberately does not
replay old VIO outputs or PX4 topics.

Remaining: live carry/physical occupancy validation, moving-scene CPU and timing,
full 3D flight-adapter design/implementation, SITL fault gates, props-off PX4
integration and constrained flight. Observation-only validity is not permission
to command PX4. The historical design below remains the architecture reference.

## Original design (implementation status superseded above)

## Decision summary

Build the 3D mode as a parallel C++ pipeline, not as a flag inside the flown 2D
nodes:

```text
RTAB-Map local 3D occupied + free cells, with optimized poses
                         |
                         v
              loop-corrected OctoMap
                         |
                         v
       inflated 3D A* -> accepted 3D path
                         |
                         v
  swept-sphere follower -> 3D position carrot + velocity/acceleration feedforward
                         |
                         v
       existing PX4 offboard safety state machine
```

The first implementation should be observation-only. It must use a spherical
vehicle envelope, treat unknown voxels as blocked, retain the continuous raw-VIO
feed into PX4, and preserve every HOLD/LAND watchdog already used by the 2D
stack.

## Why `slam_grid_3d:=true` is not enough

The installed DepthAI 3.5.0 `RTABMapSLAM` node internally stores 3D ground,
obstacle and empty cells when `Grid/3D=true`, but its `occupancyGridMap` output is
`dai::MapData`: a two-dimensional image plus `minX` and `minY`. Its
`publishGridMap()` calls `rtabmap::OccupancyGrid::getMap()`, converts that result
to a display image, and loses voxel height before the Python bridge sees it.

The node can also publish global obstacle and ground point clouds. Those are not
an occupancy grid: absence of a point does not prove free space, and ray tracing
from the current camera pose to an old globally assembled point can incorrectly
clear a real obstacle. A flight planner must receive observed-free cells and the
sensor viewpoint associated with each observation.

The recommended perception implementation is therefore a C++ RTAB-Map host that
uses each keyframe's `gridGroundCellsRaw()`, `gridObstacleCellsRaw()`,
`gridEmptyCellsRaw()`, `gridCellSize()` and `gridViewPoint()`. Assemble them with
the latest optimized graph poses into an OctoMap and publish
`octomap_msgs/msg/Octomap`. This machine already has OctoMap 1.9.7,
`liboctomap-dev`, and ROS Jazzy `octomap_msgs` installed.

Keep these mapping rules explicit:

- `Grid/3D=true` and ray tracing enabled.
- Ground is occupied for a flying vehicle. Do not discard the floor merely
  because RTAB-Map labels it ground; use `Grid/GroundIsObstacle=true` or insert
  ground cells as occupied in the OctoMap layer.
- Unknown voxels are blocked. Initial 3D mode does not explore through unknown
  volume.
- Apply optimized map poses to the voxel map, but never send loop-corrected pose
  to PX4 EKF2.
- Publish map stamp, resolution, map-frame bounds, update generation and source
  pose generation. Reject stale or internally inconsistent maps.
- Use a bounded rolling planning volume even if the stored OctoMap is global.
  At 0.03 m, a dense 6 x 6 x 2 m volume contains about 2.7 million voxels;
  start at 0.05 m and crop around the vehicle and requested goal.

## Planner

Create a ROS-free `VoxelGrid`/OctoMap adapter and a C++ planner node. The search
is the direct 3D counterpart of the current A*:

- 26-connected A* with a Euclidean heuristic.
- Forbid diagonal corner cutting: every face/edge voxel crossed by a move must
  also be free.
- Inflate occupied voxels by the vehicle envelope before search. Start with a
  sphere because its clearance proof is independent of attitude; an ellipsoid
  is an optimization for later.
- Use `robot_radius + safety_margin` as the lethal radius and a second graded
  inflation band as cost. Require `safety_margin >= max_cross_track`; the 2D
  flight setting `0.05 < 0.20` does not meet that invariant.
- Bound the search by a takeoff-centered XY geofence plus explicit floor and
  ceiling limits. The goal's Z coordinate is meaningful in 3D mode.
- Keep unknown, outside-map, floor, and ceiling voxels non-traversable.
- Recover a start only inside a small verified-free radius. Never teleport the
  start through an occupied shell.
- Simplify only when the complete 3D segment has continuous swept-sphere
  clearance. Checking voxel centers alone is insufficient.
- Retain the current requested/effective-goal distinction. A blocked requested
  goal may produce a terminal safe approach point; an unknown goal must not be
  reported as reached.

Suggested initial limits:

| parameter | initial value | reason |
|---|---:|---|
| `voxel_size` | 0.05 m | manageable C++ search and map memory |
| `robot_radius` | 0.25 m | current measured cage envelope |
| `safety_margin` | 0.10 m minimum | mapping and tracking allowance |
| `max_cross_track` | 0.05 m | must not exceed the margin |
| `planning_radius_xy` | 3.0 m | matches the current flight geofence |
| `min_z` / `max_z` | site measured | floor/ceiling are hard obstacles |
| `planning_rate_hz` | 2.0 Hz | same starting cadence as 2D A* |

Do not copy the current 2D simplifier unchanged. Flight
`offboard_global_20260829T055027Z` showed why: its simplified 2D paths reached
0.285 m continuous clearance against a requested 0.300 m, while a
continuous-clearance simplifier held at least 0.304 m for only about 0.03 m more
mean path length.

## Follower and PX4 command

The follower becomes a 3D polyline follower:

1. Project the corrected SLAM pose onto the accepted XYZ polyline.
2. Select a lookahead point by arc length.
3. Validate the pose-to-lookahead chord with a swept sphere in the same voxel
   generation used to produce the path.
4. Rate-limit the XYZ carrot with separate horizontal/vertical speed and
   acceleration limits.
5. Transform its map-frame displacement back through the map-to-continuous-VIO
   correction, then rebase it at PX4 local position exactly as the 2D adapter
   does.
6. Publish a 3D position setpoint with matching velocity and acceleration
   feedforward. Position remains the active PX4 control level.

The carrot remains useful in 3D even if a future controller uses velocity-only
PX4 input: it supplies path progress, lookahead, corner behavior, braking
distance and a finite segment on which clearance can be proven. A velocity-only
controller would still need equivalent geometry plus explicit cross-track and
stopping feedback.

Use the full finite map-to-VIO rotation for XYZ displacement. Reject excessive
translation, yaw, roll or pitch corrections; do not silently turn an optimizer
tilt jump into a vertical command. A persistent correction still causes HOLD,
requires a fresh voxel generation and path, then slews out of HOLD.

Before 3D work, fix the known pose-inside-clearance deadlock shared by the 2D
concept: zero motion cannot escape a pose already inside the new inflated map.
Recovery must be a separately bounded state that accepts only commands whose
entire swept segment is known and whose clearance increases monotonically. If no
such escape exists, HOLD and LAND. Never weaken the ordinary route clearance
test globally.

## ROS interface and mode separation

Use separate names so 2D and 3D implementations cannot accidentally share
authority:

| topic | type | role |
|---|---|---|
| `/rtabmap/octomap` | `octomap_msgs/msg/Octomap` | loop-corrected occupied/free voxels |
| `/planner3d/path` | `nav_msgs/msg/Path` | accepted XYZ route |
| `/planner3d/candidate_path` | `nav_msgs/msg/Path` | latest candidate |
| `/planner3d/status` | `std_msgs/msg/String` | planning state and metrics |
| `/planner3d/follower/displacement` | `geometry_msgs/msg/Vector3Stamped` | continuous-VIO-rebased command input |
| `/planner3d/follower/valid` | `std_msgs/msg/Bool` | structured fail-closed gate |
| `/planner3d/markers` | `visualization_msgs/msg/MarkerArray` | voxels, clearance and path debugging |

Add dedicated launches such as `global_planner_3d_monitor.launch.py` and
`offboard_global_planner_3d.launch.py`. Do not overload `cpp_nodes` or silently
change `/planner/*` semantics. The flight launch must refuse to start if a 2D
adapter or another 3D authority owns the singleton lock.

The Foxglove goal remains `geometry_msgs/msg/PointStamped`, but the 3D launch
must reject goals whose frame is not `world` and must preserve rather than zero
the clicked Z value.

## Failure behavior

The 3D mode fails closed on any of the following:

- stale voxel map, corrected pose, raw VIO, correction or path;
- unknown voxel on the commanded swept volume;
- changed map generation between planning and command validation;
- path start, cross-track or vertical-track violation;
- excessive map correction or PX4 estimator reset;
- command outside XY geofence, altitude band, speed or acceleration limits;
- no monotonically safer recovery from an already-invalid pose.

An invalid follower immediately latches the current PX4 XYZ position, sends no
velocity/acceleration feedforward, and starts the existing bounded LAND timer.
Offboard loss, battery and operator LAND/KILL behavior remain owned by the
existing flight state machine and PX4.

The initial mode is static-world navigation. A loop-corrected global map is not
a dynamic-obstacle sensor. People, moving doors and other transient objects need
a fresh local depth/voxel veto with expiry before this mode can leave a
controlled empty test area. The forward OAK-D also leaves rear, lateral,
overhead and close-range blind regions; yaw should follow travel direction and
the planner may use only volume that was actually observed free.

## Implementation sequence and acceptance gates

1. **Perception contract:** extend the C++ RTAB-Map host to publish deterministic
   occupied/free voxel fixtures. Verify transformed keyframes across synthetic
   loop corrections and confirm floor/ceiling classification.
2. **ROS-free geometry:** implement voxel indexing, inflation, 26-neighbor
   validity, swept-sphere clearance, A*, simplification and 3D polyline math.
   Test randomized maps and adversarial diagonal gaps.
3. **Observation-only nodes:** publish 3D paths and follower markers, with no
   `/fmu/in/*` publishers. Hand-carry the camera through a measured obstacle
   course and compare the map against physical distances.
4. **Replay:** record OctoMap generations in bags. Every accepted path and every
   follower chord must replay against its original voxel generation with no
   clearance violation.
5. **SITL:** inject stale maps, loop corrections, estimator resets, blocked
   starts, ceiling/floor goals and process crashes. Each must HOLD/LAND exactly
   once without retaining motion feedforward.
6. **Props-off hardware:** run the complete state machine with `auto_arm=false`.
   Confirm setpoint frames, XYZ signs, limits, singleton ownership and bag
   finalization.
7. **Constrained flight:** first route at no more than 0.10 m/s in a netted,
   measured volume, with RC kill ready. Begin with a level route, then a single
   small altitude change. Do not test a narrow aperture first.

Minimum evidence before arming:

- zero unknown or occupied voxels in every commanded swept sphere;
- minimum replayed clearance at or above the configured lethal radius;
- `safety_margin >= max_cross_track` in the latched configuration;
- bounded map, planning and follower CPU with no missed 20 Hz setpoint stream;
- no map-generation race under repeated loop corrections;
- successful HOLD and AUTO.LAND for every injected fault;
- independent physical measurement of airframe radius, floor and ceiling.

## Proposed source layout

```text
include/px4_vio_bridge/voxel_grid.hpp
include/px4_vio_bridge/grid_planner_3d.hpp
include/px4_vio_bridge/path_geometry_3d.hpp
include/px4_vio_bridge/route_follower_3d.hpp
src/rtabmap_octomap_node.cpp
src/grid_planner_3d.cpp
src/global_planner_3d_node.cpp
src/route_follower_3d.cpp
src/route_follower_3d_node.cpp
src/offboard_global_planner_3d_node.cpp
launch/global_planner_3d_monitor.launch.py
launch/offboard_global_planner_3d.launch.py
test/test_*_3d.cpp
```

Keep 3D classes separate until replay and flight evidence proves their behavior.
Shared utilities may be extracted afterward; premature templating would make the
flown 2D implementation harder to audit.
