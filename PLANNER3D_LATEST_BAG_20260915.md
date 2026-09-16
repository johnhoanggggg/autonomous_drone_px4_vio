# Latest waypoint test: 06:34:47 UTC, September 15

## Outcome

Inspected `ros2_ws/flight_logs/global_planner_3d_monitor_20260915T063447Z`
and replayed its actual MapData, odometry and waypoint offline in domain 192.
Fixed a false start rejection and repeated map preparation. **The recorded
map still does not support a complete route to the requested waypoint.**
No flight/hardware graph was started and no PX4 inputs were used.

The user clarified **25 cm radius**, not diameter. The configured robot radius
remains 0.25 m, safety margin 0.10 m, and required spherical clearance 0.35 m.
The artificial clear sphere remains fixed at 0.50 m and reanchors on a fresh
click. No additional unknown space was granted to the runtime planner.

## What the recording shows

- 42.4 seconds, 31 maps and one click approximately 2.04 m from the body.
- `goal_height_mode=current` worked: clicked Z=0 became body Z=0.0749 m.
  That height is latched at the click; lifting afterward does not raise the goal.
- Initial paths were 6.8–7.0 cm long. At generation 22, the forward clearance
  probe touched 103 unknown and 17 occupied native voxels.
- The requested endpoint itself is also unsuitable: its center voxel is free,
  but the 35 cm envelope touches 873 unknown and 83 occupied voxels. The
  nearest occupied box is only 5.4 cm away. A free center point is not enough
  for the drone to fit at that requested height.
- After the click, body height rose from about 0.075 m to 0.463 m. The fixed
  clear sphere stayed at the clicked position. During the lift, the actual
  body envelope intersects unobserved space; those `START_BLOCKED` results
  are real, not timing failures.
- On return, the actual 35 cm body sphere was entirely clear (generations
  28–30, nearest unknown box approximately 0.37–0.38 m away), but its containing
  grid-cell center was not traversable. This caused false `RECOVERY_REQUIRED`.
- Recorded yaw spans -59.7° to +41.6°; pitch spans -2.4° to +13.8°.
  At generation 22 an optimistic diagnostic granting all 22 available camera
  frustums free space, even occupied cells and space behind surfaces, grows
  the reachable native-center component from only 20 to 22 cells. It remains
  disconnected. This is an upper-bound diagnostic, never published occupancy;
  increasing ray density or blindly filling those views cannot bridge it.
  Repeating the optimistic check at generation 30, including its additional
  raised-camera views and starting at the clear-sphere anchor, still finds
  only 15 reachable native centers (maximum X=0.025 m).

## Changes

`PreparedPlanner3D` owns an immutable raw grid, configuration and inflated
grid. The ROS planner replaces it on every accepted map generation; pose and
goal replans reuse it. It cannot retain inflation from a different raw map.

An actually clear off-center start can now connect to a nearby traversable
cell within `start_recovery_radius`, provided the **whole connecting swept
sphere** is clear. An actually blocked body still fails closed. A clear body
without a valid local connection reports `NO_SAFE_START_CONNECTION`.

`/planner3d/status.last_plan_timing` records map generation, cache hit,
preparation, planning and probe milliseconds, including failed plans. This
topic is already recorded in the bag. `planning_ms` remains the historical
search-only field; use the new timing fields to assess preparation overhead.

## Timing and validation

Live CPU after the first map: median 51.3% of the four-core machine, maximum
66.4%; no throttle flags. After the click, median machine load was 58.65%
and the planner used 29.9% of one core. The camera process used about 130%
of one core. Native map processing was median 45.0 ms, maximum 198.9 ms;
map age on arrival median 323.5 ms, maximum 1101.3 ms; map interval median
1.04 s, maximum 1.85 s. These are live bag measurements.

In the fixed replay, preparation on a new generation took median 133.7 ms.
Cache hits skipped that work. This is measured preparation work avoided;
it is not a claim that live total CPU has already been retested.

The replay resumes planning at generations 28–31, producing approximately
11.5–12 cm partial paths, and records 45 `FOLLOWING` samples. It still never
reaches the actual requested goal. Startup/drain and scheduling differences
mean raw status counts between live and replay are not one-to-one comparisons.

Fresh checks:

```bash
colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1
./build/px4_vio_bridge/test_grid_planner_3d_cpp
colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

Build passed; 21 focused core tests passed; full result **745 tests, zero
errors/failures, three intentional skips**. Added regressions cover clear
off-center starts, truly blocked starts, prepared-map ownership, and live
ROS cache invalidation on a new map generation.

Native replay audit: **31 maps, 13 paths, 162 follower chords, 175 segments,
zero errors**. Physical clearance remains unverified because of the explicit
artificial start clearing.

Artifacts: `/home/john/planner3d_waypoint_20260915T063447Z/`, including
`metrics.json`, `start_probe.txt`, `coverage_g22.png`, and
`replay_fixed/{report.json,audit.json,timing_summary.json,bag}`.

## Superseded test procedure

The previous up/down camera scan instruction is withdrawn. The user requires
normal level-camera operation and the existing spherical robot model.
See [the level-camera planner update](PLANNER3D_LEVEL_CAMERA_20260915.md) for
the replacement route-planning behavior and current verification.
