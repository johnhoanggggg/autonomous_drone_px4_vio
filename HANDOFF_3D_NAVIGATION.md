# 3D Occupancy Navigation Mode — Design Handoff

Status: design only. Nothing in this document is currently authorized to command
PX4. The existing planner remains a 2D planner and must continue to run with
`slam_grid_3d:=false`.

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
