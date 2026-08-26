# Global A* Planner — Experimental Handoff

Global planning and the position-only PX4 adapter have flown. The 2026-08-26
bags exposed a final-command chord/bias that is now fixed in code. Last updated:
2026-08-26.

## Status

A continuously replanning, cost-aware 2D A* monitor and observation-only
position route follower are built, simulator verified, and live-observation
verified on the OAK-D, including a real loop-correction run. They consume a
loop-corrected RTAB-Map occupancy grid, `/rtabmap/pose`, and the existing
Foxglove `/waypoint/clicked` goal. Clicks may be in known free space, unknown
space, outside the current map, or on an obstacle. The planner publishes the raw latest
candidate, a stable accepted path, an inflated costmap, markers, status, and
planning metrics. The follower publishes a smoothed lookahead position and
relative displacement. Unknown cells remain non-traversable: the planner heads
to the closest reachable known-safe frontier and advances the endpoint as the
map expands instead of treating unknown as free.

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

The position-only `offboard_global_planner` adapter is implemented. Real-PX4
props-off attempt 1 (`flight_logs/offboard_global_props_off_1`, 2026-08-06)
correctly remained disarmed and entered OFFBOARD/ROUTE, but failed the final
acceleration-continuity gate. The defect is fixed and regression-tested; one
more props-off recording is mandatory before arming. Never connect
`/planner/path` or the absolute
`/planner/follower/carrot` directly to PX4. Bag `_4` validated the
native `/rtabmap/odom_correction` direction and exposed/fixed a one-raw-frame
host pairing error. Bag `_5` confirmed the synchronization and native source.
The redundant host correction relay is removed. The follower now consumes
native correction directly and `/planner/follower/valid` fails closed on raw
VIO, correction, pose, path, correction-settle, path-start, and cross-track
faults. Bag `_4` proved this is necessary because corrected pose can remain
finite after raw VIO enters its reset sentinel. The completed adapter:

1. Consume the continuous-VIO-frame displacement and structured follower
   validity, plus the current PX4 local position and the existing
   estimator/VIO/battery health topics. Do not make flight logic parse the
   human-readable follower status string.
2. Undo correction yaw before ENU-to-NED conversion. If
   `T_map<-vio = (R(yaw_correction), translation)`, then for the map-frame
   relative carrot `d_map`, calculate `d_vio = R(-yaw_correction) * d_map`,
   followed by `d_ned = (d_vio.y, d_vio.x, 0)`. Add `d_ned` to PX4's current
   local position. Correction translation cancels in the relative vector;
   correction yaw does not. Never treat an absolute corrected-SLAM coordinate
   as a PX4 local coordinate.
3. Reuse the `offboard_waypoint` safety/state-machine patterns: `auto_arm=false`
   by default, offboard/arm confirmation, geofence and maximum-flight-time
   limits, RC kill/manual override, battery gates, and stale estimator/VIO
   handling.
4. Latch and publish a current-PX4-position HOLD target whenever the follower
   is not `FOLLOWING`/`GOAL_REACHED`; land after a bounded persistent planning
   fault while armed. A missing/blocked route must never leave the last moving
   target active.
5. Apply a second speed/acceleration limiter to the final PX4 NED position
   setpoint. This guarantees a continuous command when leaving HOLD even if the
   map-frame follower or correction transform changed while paused.
6. Must first run entirely props-off with arming disabled. Before any live trial,
   verify TELEM2/DDS on battery power and address the recorded hard/bouncing
   landing. A first route should be only 0.5–1.0 m at about 0.5–0.6 m altitude,
   in an empty mapped room, on a full battery, with RC kill ready.

The fresh local obstacle-cloud layer remains explicitly deferred for the
operator's static environments. That limits any future trial to a controlled
scene with no people, moving doors, or movable obstacles. Add the local layer
before any dynamic-environment use.

### Loop-correction behavior of the flight adapter

PX4 must continue receiving raw, continuous VIO through
`/fmu/in/vehicle_visual_odometry`; neither `/rtabmap/pose` nor a map correction
is injected into EKF2. During a persistent loop correction:

1. The follower correction gate makes structured validity false while RTAB-Map
   and A* settle.
2. The flight adapter immediately latches PX4's current NED position and
   streams that position-only HOLD setpoint. Horizontal velocity and
   acceleration fields remain unset/NaN; altitude stays independently latched
   and the yaw target freezes where its slew had reached, so a fault never
   turns the airframe.
3. Once the correction is quiet and a fresh complete path exists, the follower
   supplies a displacement transformed back into continuous VIO axes with the
   inverse correction yaw.
4. The adapter rebases that displacement from PX4's then-current local
   position and slews its position setpoint out of HOLD. No absolute map jump
   reaches PX4.

Detect PX4 local-position reset counters as a separate event: relatch HOLD at
the new PX4 coordinates and require fresh healthy follower data before
resuming. For the first trial, a correction larger than the measured room
behavior (provisionally 0.25 m or 5 degrees), or a correction/planner hold that
outlives a bounded timeout, should remain HOLD and then request LAND rather
than trying to fly through it.

## Files

| file | role |
|---|---|
| `px4_vio_bridge/rtabmap_grid.py` | strict DepthAI map-image to ROS occupancy-grid conversion |
| `px4_vio_bridge/grid_planner.py` | ROS-free grid geometry, inflation, A*, collision checks and simplification |
| `px4_vio_bridge/global_planner_monitor.py` | observation-only live monitor |
| `px4_vio_bridge/global_planner_sim.py` | fake room whose lower passage opens and closes |
| `px4_vio_bridge/path_follower.py` | ROS-free path projection, progress and position-carrot smoothing |
| `px4_vio_bridge/route_follower_monitor.py` | observation-only follower ROS node and telemetry |
| `px4_vio_bridge/planner_flight.py` | ROS-free final NED position limiter and frame conversion |
| `px4_vio_bridge/offboard_global_planner.py` | fail-closed position-only PX4 adapter |
| `launch/global_planner_monitor.launch.py` | monitor, optional simulator and optional bag |
| `launch/offboard_global_planner.launch.py` | adapter and crash-resistant flight recorder |
| `test/test_grid_planner.py` | search, unknown, dead-end, clearance and simplification tests |
| `test/test_rtabmap_grid.py` | map encoding and ROS geometry tests |
| `test/test_path_follower.py` | route, replan and synthetic loop-closure stability tests |
| `test/test_planner_flight.py` | flight gates, frame conversion, geofence and final limiter tests |

`basalt_rtabmap_slam_ros2`, both main RTAB-Map launch files, `CMakeLists.txt`,
and the combined Foxglove whitelist were extended for the new topics.

## Algorithm and defaults

- 8-connected A* with an octile heuristic.
- Diagonals cannot cut between occupied corners.
- Unknown and lethal cells are forbidden.
- Occupied threshold: 65/100.
- Lethal radius: `robot_radius + safety_margin = 0.25 + 0.05 = 0.30 m`.
- The 0.30 m value is the centre-to-obstacle envelope, not 0.30 m of empty
  padding: it represents the measured 0.25 m caged-airframe radius plus 0.05 m
  clearance. Re-measure before reducing either value.
- Additional graded inflation: 0.20 m, used as search cost rather than a hard block.
- Cost-aware route search prefers clearance over grazing inflated obstacles.
- 100 ms planning deadline and 2 Hz observation loop.
- Result is simplified only where the same inflated map proves direct line of sight.
- A requested goal is intent, not permission to enter unsafe space. The selected
  endpoint is the safe cell in the start-connected component closest to the click.
- A known-safe reachable click is `PATH_VALID` and exact. An unknown, outside-map,
  or currently disconnected click is `EXPLORING`; the route ends at the current
  reachable frontier and is recomputed as mapping expands. An obstacle or point
  inside lethal inflation is `SAFE_APPROACH`; the route ends outside the 0.40 m
  hard clearance envelope.
- A candidate path is published every replan. The accepted path switches when
  invalid, the start cell moves, the requested/effective goal changes, or the
  candidate is at least 10% cheaper.
- Reaching a temporary `EXPLORING` frontier does not publish requested-goal
  completion and therefore does not trigger the flight adapter's arrival landing.
  It holds there until newly mapped free space advances the route. Reaching an
  exact goal or terminal `SAFE_APPROACH` endpoint does complete the request.

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
  slam_grid_ray_tracing:=true slam_grid_footprint_radius:=0.25

ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge global_planner_monitor.launch.py
```

Foxglove topics:

| topic | meaning |
|---|---|
| `/rtabmap/grid` | raw loop-corrected occupancy map from DepthAI RTAB-Map |
| `/planner/inflated_map` | clearance-aware costmap |
| `/planner/path` | accepted blue/display path |
| `/planner/candidate_path` | latest replan, whether or not accepted |
| `/planner/markers` | start, requested goal, effective safe endpoint, and status label |
| `/planner/effective_goal` | current reachable safe endpoint selected for the request |
| `/planner/goal_exact` | whether the effective endpoint is the clicked map cell |
| `/planner/goal_terminal` | whether reaching the effective endpoint completes the request |
| `/planner/status` | `WAITING_*`, `PATH_VALID`, `EXPLORING`, `SAFE_APPROACH`, stale/error state |
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
3. Confirm unexplored space is `-1`; click into it and verify an `EXPLORING`
   path stops in known free space, then advances as mapping reveals safe cells.
4. Click an obstacle and verify `SAFE_APPROACH`, `goal_terminal=true`, and an
   effective endpoint outside the lethal envelope. No path cell may be unknown
   or lethal.
5. Repeat an A/B VIO-rate measurement while moving; the stationary live run with
   grid and clouds enabled measured about 14.3 Hz, but is not a flight-load benchmark.
6. Exercise a loop closure and confirm old cells, current pose, and path remain aligned.
7. Replay or record a moving blockage and confirm path invalidation/replanning.

## Flight boundary and mandatory props-off run

This remains global planning, not dynamic collision avoidance. The
`offboard_global_planner` adapter now supplies the PX4 position setpoint,
stale-data HOLD/LAND behavior, strict 0.25 m / 5 degree correction gate,
one-metre takeoff-centred geofence, PX4-reset rebasing,
and a path-constrained 0.10 m/s / 0.30 m/s² launch-default limiter. It does not add a
fresh local obstacle layer, so any eventual flight is limited to a controlled
static room.

The final limiter consumes `/planner/path` directly. Normal commands use scalar
arc-length progress, stop at a polyline vertex, and wait there until the vehicle
is within `path_corner_tolerance=0.05 m` before changing direction. This keeps
the controller from seeing a diagonal chord while the aircraft still trails
the bend. A replacement route may rejoin through
at most the configured 0.05 m band; a larger discontinuity is rejected. After
that limiting step, the adapter re-runs continuous raw-map clearance from the
current map pose to the exact final command. A failed check latches the existing
stationary HOLD and LAND timer before the point can be published. The adapter
also consumes the follower's latched configuration and refuses flight when its
maximum carrot speed differs from the final 0.10 m/s speed.

Battery authority is intentionally not duplicated in the adapter: the ROS
`/battery/*` topics are telemetry only, while PX4 owns battery arming checks and
low-battery failsafe actions.

### Yaw follows the path heading

The adapter no longer holds the latched takeoff yaw through ROUTE. It takes the
NED bearing of the same commanded displacement it flies (`atan2(east, north)`)
and commands that heading, so the airframe and the forward-facing OAK-D point
along the route instead of crabbing sideways down it.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `yaw_follows_heading` | `true` | `false` restores the fixed takeoff yaw |
| `yaw_rate_deg` | `20.0` (launch) | slew of the published yaw setpoint |
| `yaw_track_min_displacement` | `0.15` m | shorter carrots are noise; hold the last heading |
| `yaw_track_deadband_deg` | `8.0` | re-latch the target only outside this band |
| `yaw_align_error_deg` | `40.0` | above this error, stop translating and turn first |
| `yaw_resume_error_deg` | `15.0` | resume translating below this error (hysteresis) |

Properties that must hold in a bag:

- yaw only ever moves at `yaw_rate_deg` (the base class `ramp_yaw` slew); the
  target changes in steps but the published setpoint never does;
- the measured yaw rate stays far below the node's own `max_yaw_rate_deg` = 60
  LAND watchdog, and VIO-vs-EKF yaw disagreement stays under
  `max_vio_yaw_error_deg` = 20 through each turn;
- while turning onto a leg the command decelerates through the same limiter and
  the setpoint stops moving horizontally, so `/planner/flight/status` reads
  `YAW_ALIGN`; and
- a planner fault, a HOLD, or a PX4 heading reset stops the turn: the target
  freezes at the current slew position (or is dropped entirely on a reset
  counter change) rather than continuing to rotate.

Turning is the weak point for VIO: rotation moves the whole image and can drop
the tracked feature count. `yaw_rate_deg` is the knob to lower if a props-off
bag shows features dipping toward `min_vio_features` during turns.

Before fitting props, run the real stack and planner, click a short route, and
confirm `/planner/follower/valid` is true. Then run:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  offboard_global_planner.launch.py auto_arm:=false
```

Inspect the resulting `offboard_global_*.mcap` before any armed attempt. It
must contain position-only trajectory setpoints, no arm command, continuous
bounded NED setpoint motion, healthy VIO/estimator inputs, and correct
HOLD behavior if follower validity is deliberately removed. Only after that
gate passes should the same launch be considered with `auto_arm:=true`.

For the post-2026-08-26 regression flight, additionally require:

1. `/planner/flight/status` reports `command_speed<=0.10m/s` and normally
   `path_offset=0.000m` (up to 0.05 m only during a bounded rejoin).
2. The final `/fmu/in/trajectory_setpoint` projected into the map stays on that
   route band at every bend; there must be no old free-space chord.
3. Stop the planner while moving and confirm immediate stationary HOLD, then
   restore it and confirm the command restarts from actual pose without a jump.

### Props-off attempt 1 and required repeat

`flight_logs/offboard_global_props_off_1` is a complete 85.15 s / 66,565-message
real-PX4 recording. It passed these gates:

- all 168 control-mode samples were disarmed;
- the only vehicle command was accepted `DO_SET_MODE` (176), with no arm
  command (400);
- OFFBOARD enabled and the position-only stream ran at 49.8 Hz; after engage,
  maximum setpoint/heartbeat gaps were 40.5/35.4 ms;
- all 4,052 trajectory messages had finite position and NaN velocity and
  acceleration fields; yaw was fixed at the latched 89.89 degrees;
- maximum setpoint speed was 0.150 m/s and maximum takeoff-relative command
  distance was 0.491 m inside the 1.0 m geofence;
- PX4 local position was always valid with unchanged reset counters; VIO
  features were median 352/minimum 334 with no reset sentinel; and
- A* took 4.38 ms median / 9.77 ms p95 / 11.62 ms maximum.

It did not pass the acceleration gate. The final limiter snapped to a nearby
target and zeroed its stored velocity. Because the target is continuously
rebased from slightly noisy PX4 position, this made the published position
derivative reverse discontinuously: 244/4050 nominal intervals exceeded
0.303 m/s^2, 41 exceeded 1.0 m/s^2, and the maximum was 5.02 m/s^2. Speed itself
never exceeded 0.15 m/s. `HorizontalCommandLimiter` no longer snaps or zeros
velocity at a nearby target; it permits a tiny overshoot and reverses through
the same acceleration bound. The bag-specific noisy/reversing-target regression
passes, as does the no-snap settling regression and all 12 package test targets
(185 current pytest cases).

Attempt 1 used an `EXPLORING` frontier (`goal_exact=false`, 0.45 m route,
requested point still 1.19 m away), and the planner remained healthy throughout
ROUTE. It therefore did not test either an exact known-free request or the
moving-route-to-HOLD transition. Next session, keep the props removed and use:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  offboard_global_planner.launch.py auto_arm:=false \
  bag_output:=/home/john/autonomous_drone_px4_vio/flight_logs/offboard_global_props_off_2
```

For attempt 2:

1. Use a short known-free click and confirm `PATH_VALID`,
   `/planner/goal_exact=true`, and `/planner/follower/valid=true`.
2. Let `ROUTE` run for at least 10 seconds.
3. Stop `global_planner_monitor.launch.py` for at least four seconds while the
   adapter remains running. Confirm `/planner/flight/status` changes to `HOLD`
   promptly and no moving route target remains active.
4. Stop the adapter cleanly and inspect the finalized bag. Require no arm
   command, 0.15 m/s speed, 0.30 m/s^2 nominal-route acceleration, position-only
   fields, yaw obeying the tracking rules below (bounded slew, no step, and no
   turning while HOLD is latched), geofence compliance, a fresh healthy data
   stream, and the
   deliberate fault HOLD. The emergency current-position relatch itself is an
   intentional discontinuity and should be evaluated separately from nominal
   route limiting.
5. Do not set `auto_arm=true` until attempt 2 passes. Use a full battery for the
   eventual armed trial; attempt 1 was only approximately 55--59 percent.

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
   the path, or there is no complete route. A cross-track violation latches:
   recovery requires a newer path plus cross-track below the lower resume
   threshold continuously for the configured recovery interval.

The follower stores and limits the command as a displacement relative to the
current corrected SLAM pose. A translational loop closure therefore moves the
absolute visualization carrot with the map while leaving the relative command
continuous. A yaw correction rotates toward the new route under the same
limits. A later flight-capable adapter must apply `/planner/follower/displacement`
from PX4's current local position after ENU-to-NED conversion. It must never
send `/planner/follower/carrot` as an absolute PX4 coordinate.

Defaults are a 0.60 m lookahead, 0.10 m/s maximum carrot motion, 0.30 m/s²
maximum carrot acceleration, 0.60 m maximum cross-track error, and 0.12 m
arrival tolerance. The cross-track latch resumes below 0.05 m only after 1.0 s
of continuously healthy input; a new requested goal explicitly clears the old
latch. Correction targets pass through a 0.35 s low-pass filter.
An accumulated change over 0.05 m or 1.5 degrees starts one coalesced correction
episode; the follower waits for 0.40 s of correction quiet plus a path newer
than the last material movement. An 8 s cooldown prevents one graph
optimization from repeatedly starving the follower.

The standard `rtabmap_ros` loop-closure info topic is not present here because
this stack runs DepthAI's on-device `dai.node.RTABMapSLAM`. DepthAI 3.5 does,
however, expose `odomCorrection`, which its source defines as RTAB-Map's
`stats.mapCorrection()` map-to-odom transform. The host wrapper now publishes
that output as PoseStamped on `/rtabmap/odom_correction`; it deliberately is
not inserted into TF. The actual loop-closure ID is only logged internally by
the DepthAI implementation and is not an output. Bag `_4` verified
`C_native * raw_pose -> corrected_pose`. The redundant host correction relay
has now been removed. The follower consumes native correction directly and
keeps the 0.5 m / 15 degree rejection plus magnitude/quiet/fresh-path gating.
It also gates on reset-sentinel/invalid/stale raw VIO and stale correction.

### Completed validation milestone — real loop closure and replanning stability

Run the global planner and observation-only follower while deliberately
creating a loop closure. Record the grid, SLAM pose, native correction, accepted
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

All three existing loop-closure bags predate `/rtabmap/odom_correction`, so
they cannot directly validate the native output. Offline reconstruction of
`corrected_pose * inverse(raw_pose)` in bags `_2` and `_3` agrees with the
deadbanded `/vio/map_correction_target` convention: translation disagreement
was 0.91/0.97 cm p95 and yaw disagreement 0.193/0.199 degrees p95. The original
unnumbered bag does not pass this comparison and remains unsuitable because of
its earlier pairing/reset/recording problems. `scripts/analyze_map_correction.py`
now reports native-vs-target comparison when the native topic exists and the
legacy reconstruction result otherwise.

Bag `global_planner_loop_closure_4` is the first native-correction recording.
During healthy VIO, applying the native map-to-odom correction to the properly
matched raw pose reproduced the corrected pose with 1.08 cm XY p95 and 0.040
degree yaw p95 error, confirming transform direction. The best match was the
*next* raw ROS message (`+1` frame, 71.8 ms median), exposing the host bridge's
non-blocking read of the previous passthrough pose. DepthAI emits corrected
transform, correction, then raw passthrough for one callback; the bridge now
blocks for the latter two, keeping one callback together.

The old reconstructed `/vio/map_correction_target` disagreed with native by
9.78 cm / 2.42 degrees p95 during healthy motion because of that frame skew.
Native showed the real main optimization at 60.06 s: an 11.30 cm / 1.04 degree
step, settling around 12.21 cm / 1.35 degrees by 67.03 s. At 83.28 s raw VIO
entered its reset sentinel and never recovered for the remaining 46.03 s.
DepthAI later produced a bogus 1.47 m / 51.32 degree correction; the existing
VIO-reset freeze and 0.5 m / 15 degree gates correctly define this as a fault.
The observation planner nevertheless resumed on a finite corrected pose, so
raw-VIO validity therefore became an explicit structured follower gate before
PX4 is connected. The analyzer now separates healthy/reset intervals and
validates the native transform relation instead of treating the old target as
ground truth.

Bag `global_planner_loop_closure_5` confirmed the post-fix pipeline over 136.5
s. Native correction, raw VIO, corrected pose, and feature count all ran at
13.2 Hz. The correct raw-frame offset is now `0` with a 0.0 ms median header
offset; the full native transform reproduced corrected pose with effectively
zero p95 XY/yaw error (0.23 cm / 0.079 degree maxima around the transient).
The now-retired `/vio/map_correction_target` relay matched native exactly for at
least 95% of valid samples; isolated maximum differences occurred only at
correction-update ordering instants. One reset-sentinel sample at 46.24 s recovered on the next sample
with only 1.5 cm / 2.86 degrees across the gap, so later corrections are not
reset artifacts. VIO features were 287 median / 31 minimum, with 97 samples
below 160.

This longer 25.26 m carried loop produced genuine corrections up to 39.96 cm
and 11.15 degrees, with a 32.21 cm single optimization step at 69.04 s. Those
remain inside the follower's 0.5 m / 15 degree acceptance gate but
exceed the provisional first-flight 0.25 m / 5 degree HOLD-then-LAND gate; do
not silently raise the flight gate. While a route existed, four coalesced
correction waits totaled 2.62 s. Follower displacement updates stayed at 2.70
cm maximum (0.265 m/s implied), cumulative progress had zero backward events,
and A* was 1.84 ms median / 10.36 ms p95 / 13.07 ms maximum.

Bag `global_planner_loop_closure_6` is the first recording after removing the
correction relay. Its topic set is native-only and includes structured follower
validity. Over 100.4 s, raw VIO/corrected pose/native correction ran at 13.7 Hz;
the native full transform matched exactly at healthy paired samples. The main
correction reached 15.51 cm / 3.37 degrees. Both one-sample VIO reset sentinels
produced `INVALID_VIO`, proving the new structured gate works. The follower
made zero backward progress updates and its displacement changed by at most
2.66 cm (0.2505 m/s p95 implied speed). A* took 2.27 ms median / 6.50 ms p95 /
18.52 ms maximum. At 93.97 s the updated map made the clicked goal fall inside
the lethal envelope, so `GOAL_BLOCKED` and continued invalid follower output
were the correct result. Sixteen malformed non-unit native quaternions occurred
only during DepthAI initialization; source, follower, and analyzer rejection is
now explicit.

Record it by running the normal SLAM stack and:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  global_planner_monitor.launch.py record_bag:=true \
  bag_output:=flight_logs/global_planner_loop_closure_N
```

The launch records the grid, corrected and raw poses, VIO feature count,
native map-to-odom correction, structured follower validity, waypoint,
accepted/candidate routes, planning metrics, and every follower output. Set a
goal that remains in mapped free space, walk/fly a loop that causes a genuine
closure, continue for roughly 10 seconds after the closure, then stop the
launch cleanly so the MCAP index is finalized.
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
- Full package test result after the props-off limiter fix: all 12 active test
  targets passed (185 current pytest cases).
- Simulator integration: planner and follower reached `FOLLOWING`; no
  `/fmu/in/trajectory_setpoint` publisher existed.
- Existing bag-output directories now stop the whole monitor launch instead of
  allowing an apparently successful but unrecorded run to continue.
- Runtime bags under project-root `flight_logs/` are intentionally ignored and
  are not part of the source commit.

Suggested commit message:

```text
Add global planner flight adapter and safe frontier goals
```
