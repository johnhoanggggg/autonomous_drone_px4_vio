# Global A* Planner — Experimental Handoff

Observation-only global planning work. Nothing here commands PX4 and nothing
has been armed. Last updated: 2026-08-04.

## Status

A continuously replanning, cost-aware 2D A* monitor and observation-only
position route follower are built, simulator verified, and live-observation
verified on the OAK-D, including a real loop-correction run. They consume a
loop-corrected RTAB-Map occupancy grid, `/rtabmap/pose`, and the existing
Foxglove `/waypoint/clicked` goal. The planner publishes the raw latest
candidate, a stable accepted path, an inflated costmap, markers, status, and
planning metrics. The follower publishes a smoothed lookahead position and
relative displacement, never a PX4 setpoint. Unknown cells are non-traversable.

The DepthAI bridge now converts `RTABMapSLAM.occupancyGridMap` (`dai.MapData`:
an `ImgFrame` plus `minX`/`minY`) to `/rtabmap/grid` as
`nav_msgs/OccupancyGrid` when `slam_publish_grid:=true`. DepthAI sends a
vertically flipped display image rather than raw occupancy values; the bridge
inverts its documented-in-source grayscale (`0` occupied, `89` unknown, `178`
free, `200` footprint-free, with probability shades) and flips the rows back.

Live conversion checks passed. With ray tracing and footprint clearing enabled,
the grid published at about 1 Hz, the current pose occupied a free cell, and all
96 points in a simultaneous obstacle cloud landed on occupied grid cells. The
vertically mirrored alternative aligned only 47.9%, confirming the row flip is
being undone in the correct direction. Unexpected grayscale values are rejected
rather than guessed. The loop-correction/replanning bag test passed; a final
operator visual check of asymmetric obstacle sides in Foxglove remains useful.

## Start here next session

The observation-only planner/follower milestone is complete. Do not connect
`/planner/path` or the absolute `/planner/follower/carrot` directly to PX4.
The next implementation milestone is a separately reviewed, position-only
flight adapter for a controlled static-room trial. It should:

1. Consume `/planner/follower/displacement` and the follower status, plus the
   current PX4 local position and the existing estimator/VIO/battery health
   topics.
2. Convert the relative displacement from ENU to NED and add it to PX4's
   current local position. Never treat an absolute corrected-SLAM coordinate as
   a PX4 local coordinate.
3. Reuse the `offboard_waypoint` safety/state-machine patterns: `auto_arm=false`
   by default, offboard/arm confirmation, geofence and maximum-flight-time
   limits, RC kill/manual override, battery gates, and stale estimator/VIO
   handling.
4. Latch and publish a current-PX4-position HOLD target whenever the follower
   is not `FOLLOWING`/`GOAL_REACHED`; land after a bounded persistent planning
   fault while armed. A missing/blocked route must never leave the last moving
   target active.
5. First run entirely props-off with arming disabled. Before any live trial,
   verify TELEM2/DDS on battery power and address the recorded hard/bouncing
   landing. A first route should be only 0.5–1.0 m at about 0.5–0.6 m altitude,
   in an empty mapped room, on a full battery, with RC kill ready.

The fresh local obstacle-cloud layer remains explicitly deferred for the
operator's static environments. That limits any future trial to a controlled
scene with no people, moving doors, or movable obstacles. Add the local layer
before any dynamic-environment use.

## Files

| file | role |
|---|---|
| `px4_vio_bridge/rtabmap_grid.py` | strict DepthAI map-image to ROS occupancy-grid conversion |
| `px4_vio_bridge/grid_planner.py` | ROS-free grid geometry, inflation, A*, collision checks and simplification |
| `px4_vio_bridge/global_planner_monitor.py` | observation-only live monitor |
| `px4_vio_bridge/global_planner_sim.py` | fake room whose lower passage opens and closes |
| `px4_vio_bridge/path_follower.py` | ROS-free path projection, progress and position-carrot smoothing |
| `px4_vio_bridge/route_follower_monitor.py` | observation-only follower ROS node and telemetry |
| `launch/global_planner_monitor.launch.py` | monitor, optional simulator and optional bag |
| `test/test_grid_planner.py` | search, unknown, dead-end, clearance and simplification tests |
| `test/test_rtabmap_grid.py` | map encoding and ROS geometry tests |
| `test/test_path_follower.py` | route, replan and synthetic loop-closure stability tests |

`basalt_rtabmap_slam_ros2`, both main RTAB-Map launch files, `CMakeLists.txt`,
and the combined Foxglove whitelist were extended for the new topics.

## Algorithm and defaults

- 8-connected A* with an octile heuristic.
- Diagonals cannot cut between occupied corners.
- Unknown and lethal cells are forbidden.
- Occupied threshold: 65/100.
- Lethal radius: `robot_radius + safety_margin = 0.30 + 0.10 = 0.40 m`.
- Additional graded inflation: 0.20 m, used as search cost rather than a hard block.
- Cost-aware route search prefers clearance over grazing inflated obstacles.
- 100 ms planning deadline and 2 Hz observation loop.
- Result is simplified only where the same inflated map proves direct line of sight.
- A candidate path is published every replan. The accepted path switches when
  invalid, the start cell moves, the goal changes, or the candidate is at least
  10% cheaper.

There is deliberately no partial-path success and no goal snapping. Invalid,
outside-map, occupied and unknown goals are reported instead of silently changed.

## Simulator

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  global_planner_monitor.launch.py simulate:=true
```

Publish a world-frame goal near `(3, 0)` using Foxglove, or:

```bash
ROS_DOMAIN_ID=42 ros2 topic pub --once /waypoint/clicked \
  geometry_msgs/msg/PointStamped \
  "{header: {frame_id: world}, point: {x: 3.0, y: 0.0, z: 0.0}}"
```

Verified 2026-08-04 on the Pi: lower gap open produced a 6.00 m candidate,
140 expanded cells, 2.0 ms. Closing that gap produced a 7.46 m alternative,
1125 expanded cells, 15.6 ms.

## Live observation procedure

Start the stack with the grid enabled. `slam_grid_3d:=false` is explicit because
the consumer is a 2D planner. Ray tracing is required to represent observed
free space; footprint clearing makes the vehicle's current cell traversable.
All remain opt-in and do not change the normal flight-stack defaults.

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py \
  slam_publish_grid:=true slam_grid_3d:=false \
  slam_grid_ray_tracing:=true slam_grid_footprint_radius:=0.40

ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge global_planner_monitor.launch.py
```

Foxglove topics:

| topic | meaning |
|---|---|
| `/rtabmap/grid` | raw loop-corrected occupancy map from DepthAI RTAB-Map |
| `/planner/inflated_map` | clearance-aware costmap |
| `/planner/path` | accepted blue/display path |
| `/planner/candidate_path` | latest replan, whether or not accepted |
| `/planner/markers` | start, goal and status label |
| `/planner/status` | `WAITING_*`, `PATH_VALID`, `NO_KNOWN_PATH`, stale/error state |
| `/planner/planning_ms` | search time |
| `/planner/path_length` | accepted route length |
| `/planner/expanded_cells` | A* work per plan |
| `/planner/follower/lookahead` | raw interpolated 0.60 m lookahead position |
| `/planner/follower/carrot` | speed/acceleration-limited proposed position |
| `/planner/follower/displacement` | smoothed carrot minus current SLAM pose |
| `/planner/follower/markers` | Foxglove carrot spheres and displacement line |
| `/planner/follower/status` | following, arrival, stale, correction wait, or path fault |
| `/planner/follower/progress` | cumulative progress for the current requested goal |
| `/planner/follower/path_progress` | along-track coordinate on the current A* generation |
| `/planner/follower/{remaining,cross_track,path_generation}` | remaining follower metrics |

First live acceptance checks:

1. Overlay `/rtabmap/grid`, `/rtabmap/obstacle_cloud`, `/rtabmap/pose`, and
   `/planner/inflated_map` in Foxglove as a visual confirmation of the measured alignment.
2. Put a known obstacle on each side of the drone and verify the corresponding
   map side without relying on a symmetric scene.
3. Confirm unexplored space is `-1` and the planner refuses it.
4. Repeat an A/B VIO-rate measurement while moving; the stationary live run with
   grid and clouds enabled measured about 14.3 Hz, but is not a flight-load benchmark.
5. Exercise a loop closure and confirm old cells, current pose, and path remain aligned.
6. Replay or record a moving blockage and confirm path invalidation/replanning.

## Boundary before flight work

This remains a global route monitor, not collision avoidance. It has no fresh
local cloud layer, emergency stop, PX4 setpoint generator, or stale-data
landing behavior. The route follower below is telemetry only. Do not connect
`/planner/path` or `/planner/follower/*` to `offboard_waypoint`. A separately
reviewed flight adapter is required for the controlled static-room milestone;
the deferred short-lived local obstacle/collision layer is required before
dynamic-environment use.

### TODO — fresh local obstacle layer (deferred for static environments)

The operator currently intends to fly only in static environments, so a
high-rate local layer from the latest `/rtabmap/obstacle_cloud` is useful but
not the immediate priority. The global occupancy grid remains the planning
source. Before operating around people, moving objects, or doors that can move,
add a short-lived local collision layer that:

- checks the upcoming path corridor against the newest obstacle cloud;
- uses the same 0.40 m hard clearance envelope;
- invalidates the route immediately when that corridor becomes occupied;
- treats stale cloud data as unsafe; and
- expires temporary obstacles instead of permanently writing them into the
  global map.

This TODO is deferred, not cancelled. The global grid's approximately 1 Hz
update is not an adequate final safety authority in a dynamic environment.

### Completed milestone — observation-only route follower

`route_follower_monitor` consumes `/planner/path` and publishes no PX4
commands. Its output is a proposed lookahead/carrot and follower state for
Foxglove and bag analysis. It:

1. Projects the current `/rtabmap/pose` onto the accepted path and maintains
   monotonic progress so pose noise cannot send it backward along the route.
2. Selects a point a configurable distance ahead along the path, interpolating
   across segments rather than jumping between A* vertices.
3. Limits carrot velocity/acceleration and suppresses identical replans so
   every new 1 Hz path does not cause a setpoint discontinuity.
4. Re-anchors cleanly when the goal changes, the current path becomes invalid,
   or A* replaces it with a materially different route.
5. Publishes the proposed carrot, along-track progress, cross-track error,
   remaining distance, route generation, and a clear follower status.
6. Holds the carrot when the path/pose is stale, the vehicle is too far from
   the path, or there is no complete route.

The follower stores and limits the command as a displacement relative to the
current corrected SLAM pose. A translational loop closure therefore moves the
absolute visualization carrot with the map while leaving the relative command
continuous. A yaw correction rotates toward the new route under the same
limits. A later flight-capable adapter must apply `/planner/follower/displacement`
from PX4's current local position after ENU-to-NED conversion. It must never
send `/planner/follower/carrot` as an absolute PX4 coordinate.

Defaults are a 0.60 m lookahead, 0.25 m/s maximum carrot motion, 0.50 m/s²
maximum carrot acceleration, 0.60 m maximum cross-track error, and 0.12 m
arrival tolerance. Correction targets pass through a 0.35 s low-pass filter.
An accumulated change over 0.05 m or 1.5 degrees starts one coalesced correction
episode; the follower waits for 0.40 s of correction quiet plus a path newer
than the last material movement. An 8 s cooldown prevents one graph
optimization from repeatedly starving the follower.

### Completed validation milestone — real loop closure and replanning stability

Run the global planner and observation-only follower while deliberately
creating a loop closure. Record the grid, SLAM pose, correction target, accepted
and candidate paths, and proposed carrot. Confirm that:

- map, pose, goal, and path remain in one corrected `world` frame;
- no transient path is published using a new pose with an old map, or vice versa;
- the accepted route changes only when actually invalid or materially better;
- follower progress does not jump backward;
- the relative displacement remains continuous; the absolute world carrot is
  expected to follow a translational map correction so it stays map-aligned;
  and
- planning time and VIO rate remain acceptable during graph optimization.

Nothing in this test should subscribe to `/fmu/in/*`.

Synthetic translation/yaw correction, detour-replan, monotonic-progress and
slew-limit tests pass. Simulator integration produced `FOLLOWING`, a valid
world carrot and relative displacement, and no `/fmu/in/trajectory_setpoint`
topic.

The 2026-08-04 bag `global_planner_loop_closure_2` captured a real correction:
the corrected pose stepped 12.1 cm / 2.85 degrees near 71.7 s and settled at a
15.8 cm / -2.9 degree map-to-VIO transform. The relative position proposal
remained limited to 0.257 m/s (timer jitter around the 0.25 m/s setting), A*
took 0.62 ms median / 3.28 ms maximum, and VIO remained about 13.7 Hz.

That bag exposed two defects in the first follower version. Sample-by-sample
correction gating left it waiting for 27.5 s after a route existed, and the
published progress coordinate reset on each replan. The revised detector
coalesces the same recording into four correction episodes, still detects the
real closure at 71.9 s, and waits only 1.64 s total. Replaying all 303 usable
pose/path samples through the revised cumulative-progress logic produced zero
backward events. `/planner/follower/path_progress` retains the deliberately
re-anchored per-generation coordinate. A new goal explicitly resets cumulative
progress and clears the old route while waiting for its replacement.

The follow-up 137.2 s bag `global_planner_loop_closure_3` validated the fixes
live. It contained 156 valid accepted paths and 856 follower proposals.
Cumulative progress reached 3.82 m with zero backward events; the separate
per-generation coordinate reset as designed. Correction waiting fell from
27.5 s in the first implementation to 5.5 s in this longer, noisier recording.
The largest relative-displacement update was 2.59 cm, consistent with the
0.25 m/s limit at 10 Hz. A* took 0.87 ms median, 2.46 ms p95, and 3.93 ms max;
VIO was 12.1 Hz with no gaps. Path starts were 4.3 cm median / 14 cm maximum
from the nearest corrected pose.

The safety refusals in `_3` were correct, not follower regressions. At 97–99 s
the current pose was 0.345 m from an occupied cell, inside the 0.40 m lethal
envelope, so the planner reported `START_BLOCKED`. From 116 s onward an
occupied cell was 0.323 m from the goal, so it reported `GOAL_BLOCKED`,
published an empty path, and the follower moved to `WAITING_FOR_PATH`.

Record it by running the normal SLAM stack and:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  global_planner_monitor.launch.py record_bag:=true \
  bag_output:=flight_logs/global_planner_loop_closure_N
```

The launch records the grid, corrected and raw poses, correction target/applied
correction, waypoint, accepted/candidate routes, planning metrics, and every
follower output. Set a goal that remains in mapped free space, walk/fly a loop
that causes a genuine closure, continue for roughly 10 seconds after the
closure, then stop the launch cleanly so the MCAP index is finalized.
Use a new `bag_output` directory for every run. The launch now shuts itself down
if the recorder exits unexpectedly (including an existing-output-directory
error), instead of letting an unrecorded planner run continue unnoticed.

## Live observations (2026-08-04)

- Without ray tracing, the map was valid but sparse: only 79 known-free cells
  and the current pose cell was unknown. The monitor safely reported
  `START_BLOCKED`.
- `Grid/RayTracing=true` plus `GridGlobal/FootprintRadius=0.40` produced 527
  known-free cells in the same stationary scene and made the start traversable.
- A live goal in the connected known-free component produced a 3.42 m path,
  184 expanded cells, in 2.5 ms.
- With clouds enabled, the sampled grid contained 529 free, 74 occupied and
  3317 unknown cells. All 96 obstacle-cloud points mapped to occupied cells.
- Grid rate was about 1 Hz. Stationary VIO measured about 14.3 Hz with grid and
  clouds enabled. Moving loop-correction validation measured 12.1 Hz with no
  gaps. Nothing was armed.

## Verification and commit boundary

- `colcon build --packages-select px4_vio_bridge --symlink-install`: passed.
- Full package test result: 186 tests, 0 errors, 0 failures, 0 skipped.
- Simulator integration: planner and follower reached `FOLLOWING`; no
  `/fmu/in/trajectory_setpoint` publisher existed.
- Existing bag-output directories now stop the whole monitor launch instead of
  allowing an apparently successful but unrecorded run to continue.
- Runtime bags under project-root `flight_logs/` are intentionally ignored and
  are not part of the source commit.

Suggested commit message:

```text
Add observation-only global planner and position route follower
```
