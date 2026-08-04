# VFH2D Obstacle Avoidance — PARKED Experimental Handoff

Side-work doc, same role as `HANDOFF_LOOP_CLOSURE.md`: everything about the VFH
obstacle-avoidance work lives here, and `HANDOFF.md` keeps only the summary. The
flight record in `HANDOFF.md` is unaffected by any of this — nothing here has
ever been armed. This code is retained as a tested prototype, not as the current
path-planning direction.

Last updated: 2026-08-04.

## Status in one paragraph

A 2D Vector Field Histogram (VFH+) planner, **built, bench/live-monitor verified,
never armed, and parked on 2026-08-04**. It is split into an algorithm with no
ROS in it, a sensor adapter, an observation-only node, and an offboard flight
node. VFH was useful for learning from the obstacle cloud, but it is not being
used as the vehicle's path planner: it is a reactive local steering method with
no global route, weak dead-end behavior, strong dependence on cloud density and
height filtering, and only a short-lived approximation of off-camera space.
The implementation and tests are preserved so useful geometry, telemetry, and
failure analysis are not lost. **Do not treat `offboard_vfh` as a current flight
mode.** No normal stack launch starts either VFH node; they run only when their
dedicated launch files are invoked explicitly. Final verification at park time:
`colcon build` succeeded and `colcon test-result --verbose` reported **153 tests,
0 errors, 0 failures, 0 skipped**; 79 tests cover the VFH modules directly.

**Nothing here works without `slam_publish_clouds:=true`.** The obstacle cloud
is off by default in `rtabmap_slam_px4.launch.py`; without it the nodes see no
data at all, and the flight node's response to no data is to hold and then land.

## What was added

| file | what it is |
|---|---|
| `px4_vio_bridge/vfh2d.py` | the algorithm. No ROS, no numpy, no vehicle — pure function of `(range, bearing)` samples |
| `px4_vio_bridge/vfh_obstacles.py` | `/rtabmap/obstacle_cloud` + `/rtabmap/pose` → voxel-memory → body-frame samples |
| `px4_vio_bridge/vfh_telemetry.py` | everything Foxglove draws; shared by both nodes |
| `px4_vio_bridge/vfh_monitor.py` | runs the planner live. **Publishes nothing to PX4** |
| `px4_vio_bridge/offboard_vfh.py` | experimental flight node; subclasses `OffboardWaypoint` |
| `launch/vfh_monitor.launch.py` | monitor + optional bag |
| `launch/offboard_vfh.launch.py` | experimental flight + bag |
| `scripts/vfh_sim_obstacles.py` | fake cloud + pose: runs without camera, vehicle, or PX4 |
| `test/test_vfh2d.py` | algorithm and cloud→samples geometry tests |
| `test/test_offboard_vfh.py` | carrot placement, holds, watchdogs, goal intake, and startup-sweep tests |
| `test/test_vfh_telemetry.py` | frame-conversion and marker-geometry tests |

Modified: `CMakeLists.txt` (two executables, three test suites),
`package.xml` (`visualization_msgs`), `rtabmap_slam_px4.launch.py` (Foxglove
whitelist), `README.md` and `HANDOFF.md` (operating notes).

## The design decision that matters

**The click is a goal, not a setpoint.** `offboard_waypoint` sends the clicked
point to PX4 through a rate limiter; `offboard_vfh` keeps the click as `self.goal`
and publishes a **carrot** `lookahead` (0.60 m) along whatever direction VFH picks
every `plan_period` (0.2 s). The vehicle therefore curves around obstacles rather
than driving at the goal. Everything else — the setpoint rate limiter, the
`hold_point` re-pointing, the settled/transit gate split, the VIO tracking-loss
watchdogs, K/L, `max_flight_time` — is inherited unchanged.

Before the carrot is allowed to move, `offboard_vfh` enters `VFH_SWEEP` after a
verified stable hover. It clears obstacle memory, holds XY, and commands
`0 -> -90 -> +90 -> 0 deg` at 15 deg/s, where 0 is the original heading. This
fills the world-frame voxel memory across the useful forward hemisphere without
pointing the camera backward. It returns within 5 deg of 0 and settles for 1 s
before entering `VFH`. Stale obstacle input pauses the sweep and the existing
2 s stale watchdog lands; failure to settle within 5 s also lands. Goals clicked
during the sweep are queued but cannot produce translation. Set both sweep
bounds to 0 only when deliberately disabling the scan.

The carrot is computed from the **commanded** point, never the measured position.
Feeding tracking error back into a rate-limited setpoint is how it starts chasing
its own noise.

## Three properties of this vehicle that govern the whole thing

**1. The camera sees ~70 deg, and unknown is not free.** `max_steer_deg` (35)
bounds every chosen direction to the region actually observed. The consequence
is not obvious and is worth internalising: with
`robot_radius + safety_margin = 0.4 m`, a point obstacle subtends 35 deg at
0.70 m and 23.6 deg at 1 m. Widening `max_steer` past the real FOV would be
lying to the planner.

**2. A gap must be wider than `2 * (robot_radius + safety_margin)` = 0.8 m.**
The measured vehicle radius is approximately 0.30 m and the tuned safety
margin is 0.10 m.

**3. Reactive, with bounded map-frame memory.** The DepthAI obstacle-cloud
output is per-update, so VFH retains the latest point batch in each 0.10 m
world voxel for 30 s.
Yawing an obstacle out of view no longer makes it disappear and pull the drone
back toward it. Memory is capped at 20,000 points and expires so moved objects
and stereo noise are not permanent. Correction jitter is ignored; an accumulated
SLAM map correction of 0.05 m or 2 deg clears memory and holds until a fresh
cloud arrives. It still cannot reverse out of a dead end.

## Defects found and fixed by bench work

The first three would have flown into something; the fourth prevented flight.

1. **Flat smoothing erased solid obstacles.** A moving average over 3 sectors
   divides an isolated blob by the whole window, so 36 stereo returns off a wall
   edge at 2.5 m came out under threshold and read as *free*. Now the triangular
   VFH+ kernel (1,2,1), so a sector keeps half its own weight. Regression test
   `test_an_isolated_narrow_obstacle_is_not_smoothed_away`.
2. **Rounding enlargement up to whole sectors closed flyable gaps.** A 1.4 m gap
   at 2.2 m needs 12.5 deg of clearance and has 17.6, but inflating each edge to
   the next whole sector took it away. Neighbours are now blocked by testing
   their centre against the enlarged cone.
3. **`z_below` 0.35 m made the floor an obstacle at a 0.30 m hover.** The height
   slab is measured *from the vehicle*, so 0.35 m below a 0.30 m hover reaches
   under the floor and every ground return inside `max_range` arrived as an
   obstacle — a broad, roughly symmetric red fan across the whole forward arc
   with genuinely empty space in front of the drone, nearest return ~1.1–1.2 m
   where the floor enters the camera's downward view. **This is a silent failure:
   the histogram looks entirely plausible while being wrong.** Default is now
   0.15 m in both nodes and both launch files, and `offboard_vfh.checked_z_below()`
   clamps it to `hover_height - 0.15` with an error log. RTAB-Map is supposed to
   segment the floor into `/rtabmap/ground_cloud`; near-field ground leaks into
   the obstacle cloud routinely, so this filter is what has to catch it.
4. **Infinite-ray enlargement blocked a safe short waypoint.** Live on
   2026-08-03, the goal was 0.88 m ahead while real cabinet returns began at
   1.37 m and +23 deg. Classic VFH treated every candidate as an infinite ray
   and reported 72/72 blocked. Enlargement now uses finite line-segment geometry
   when a goal distance is supplied: obstacles beyond the endpoint block only
   headings whose endpoint enters the vehicle+safety radius. The same live
   cloud then selected the goal at +1.5 deg with 61/72 sectors blocked. Candidate
   selection also no longer applies the wide-valley border margin to a target
   sector already made safe by obstacle enlargement.

Found by inspection while writing, not by test, and worth remembering: the ENU
and NED relative-bearing forms do **not** mirror each other (ENU angles increase
counter-clockwise, NED headings clockwise). `relative_bearing_enu` subtracts,
`relative_bearing_ned` does not. Mixing them puts every obstacle on the wrong
side of the vehicle. Both are tested.

## How the algorithm is parameterised

The pipeline, in order, with defaults:

1. **Sampling** — `min_range` 0.25, `max_range` 2.0, height slab `z_below` 0.15 /
   `z_above` 0.60, `max_samples` 1200 (uniform stride; `/vfh/nearest` is always
   computed from the undecimated set so the emergency-stop distance is exact).
2. **World memory** — merge the newest cloud into 0.10 m voxels for
   `memory_duration` 30 s, capped by `memory_max_points` 20000. Re-observing a
   voxel replaces its complete point batch instead of accumulating frames, so
   current-cloud density is preserved. Set `memory_duration:=0` to recover
   current-frame-only behaviour. `memory_correction_topic` defaults to
   `/vio/map_correction_target`; reset gates are 0.05 m and 2 deg.
3. **Density** — `density += 1 - range/max_range` per point. `max_range` is
   therefore both the cutoff *and* the weight scale: raising it makes far points
   count and every point count less.
4. **Noise gate** — `min_points` 4. Fewer returns than that and the sector is
   free regardless of how close they claim to be.
5. **Smoothing** — triangular (1,2,1) over `smoothing` 3 sectors.
6. **Threshold with hysteresis** — `tau_high` 6.0 blocks, `tau_low` 3.0 releases.
   These values were tuned against the live cloud in the actual room.
   **This is the one default that cannot be derived; it is scene-dependent.**
7. **Enlargement** — along the path, use
   `asin((robot_radius + safety_margin) / range)`, saturating to 90 deg inside
   0.4 m: 23.6 deg at 1 m and 11.5 deg at 2 m. Beyond a finite goal,
   use endpoint-circle intersection; returns farther than
   `goal_distance + enlargement_radius` cannot block that goal.
8. **Camera-FOV mask** — sectors centred outside `max_steer_deg` are marked
   non-flyable before openings are found. Unknown space cannot form a valley.
   The pre-mask obstacle histogram is retained separately for telemetry, so
   widening the Foxglove display does not falsely paint blind space red.
9. **Candidates** — openings wider than `wide_valley_deg` 40 offer both borders
   (pulled in 20 deg) plus the goal direction if it fits; narrower openings offer
   only their centre. Openings and output headings are clipped to the exact
   camera FOV; an off-camera goal can command the FOV edge but never beyond it.
10. **Cost** — `mu_target` 5.0 · angle-to-goal + `mu_heading` 2.0 · angle-off-nose
   + `mu_previous` 2.0 · angle-from-last-choice, minimised. `mu_target` must stay
   dominant or it never commits; `mu_previous` is what stops it weaving when a
   wall is symmetric ahead.

**Tuning order:** `tau_high` first, from the live histogram in the real room;
then `safety_margin`, which sets how early it starts dodging; `max_steer` only
ever downward from a measured FOV.

**Wart:** the range band is `min_range`/`max_range` on `vfh_monitor` but
`vfh_min_range`/`vfh_max_range` on `offboard_vfh` — the plain names would collide
with inherited waypoint parameters. Unify if it causes a mistake.

## Safety layers in the flight node

On top of everything inherited from `OffboardHover`/`OffboardWaypoint`:

| condition | response |
|---|---|
| `nearest < stop_distance` (0.90 m) | freeze the carrot, hold position |
| planner reports blocked | freeze the carrot, hold position |
| blocked for `blocked_timeout` (10 s) | AUTO.LAND |
| obstacle data stale > `obstacle_timeout` (1 s) | freeze the carrot |
| stale > `obstacle_stale_land_time` (2 s) | AUTO.LAND |
| `nearest < abort_distance` (0.50 m) for `abort_time` (0.5 s) | AUTO.LAND |

With **no goal set** it hovers and does not land on the idle timeout (the idle
clock only runs after arriving at a goal), so it holds until `max_flight_time`
(90 s) forces AUTO.LAND. This remains relevant only if the work is revived.

Obstacle geometry is measured against `/rtabmap/pose` — the **SLAM** pose, the
frame the cloud lives in — not the raw VIO pose PX4 flies on, so a loop closure
cannot shift obstacles relative to the vehicle. Only the resulting *relative*
bearing crosses to PX4's heading, which is safe while the two headings agree;
the inherited 20 deg `max_vio_yaw_error_deg` watchdog enforces that by landing.

## Foxglove

`vfh_telemetry.py` is shared, so a monitor session is a genuine rehearsal of the
flight display. Everything is drawn in ENU `world` at the pose the cloud was
measured against, so it overlays `/rtabmap/obstacle_cloud` directly.

`/vfh/markers` is the one to add first — histogram fan (one ray per sector
inside `display_fov_deg`, ±90 deg by default, drawn to the range that sector
measured). Red is physically obstacle-blocked, green is clear inside the legal
`max_steer_deg` cone, and grey is clear/unknown but not steerable. The chosen
direction as a blue arrow, rejected candidates in yellow, the goal as a white
sphere, a text label, the `max_steer` wedge, and range rings at `stop_distance`
(amber), `abort_distance` (red) and `max_range` (dim). `/vfh/samples` is the
point cloud that actually reached the histogram, including remembered points.
`/vfh/memory_points` shows the current voxel-map size. The rest are one-number topics
for Gauge/Indicator/Plot panels; full table in README. Headings are PX4 NED.

**Reading the fan** — this caused real confusion once already:

- **short red** = measured returns, dense enough to cross `tau_high`;
- **full-length red** = *nothing measured there*, blocked only by a neighbour's
  clearance envelope. This is the vehicle's width, not an object, and a wide red
  skirt either side of a real obstacle is correct;
- **short green** = returns present but under `tau_high` or `min_points`. Many of
  these where a wall is means `tau_high` is too high;
- **full-length green** = empty and flyable.
- **grey** = no remembered obstacle in that sector, but the sector is outside
  `max_steer_deg` and cannot be selected.

`/vfh/binary` remains the planner's final non-flyable mask; sectors outside the
camera FOV are 1 there. `/vfh/obstacle_binary` is the pre-FOV physical obstacle
mask used to colour the wider fan.

**`rtabmap_slam_px4.launch.py` changed:** `'^/vfh/.*$'` added to
`foxglove_topic_whitelist`. Without it none of this reaches the browser — the
whitelist is an allow-list and `/vfh/*` was simply absent. It stays read-only:
nothing subscribes to `/vfh/*`, and `foxglove_client_topic_whitelist` is
untouched.

## Bench verification (2026-07-28)

`scripts/vfh_sim_obstacles.py` fakes `/rtabmap/obstacle_cloud` and
`/rtabmap/pose` — no camera, no vehicle, no PX4 — with scenes `wall`, `pillar`,
`corridor`, `box` and an `--approach` option that walks the pose forward so the
stop/abort thresholds can be watched firing.

| scene | result |
|---|---|
| wall 2.2 m, 1.4 m gap 0.8 m right | `steer=+35deg`, 12/72 sectors blocked |
| walls on three sides | `steer=BLOCKED`, 58/72 blocked, nearest 1.20 m at −90 deg |
| gap narrowed to 0.9 m | refused under the historical 0.50 m envelope used for this bench run |

All six marker namespaces confirmed publishing live (`vfh_histogram`,
`vfh_candidates`, `vfh_direction`, `vfh_label`, `vfh_fov`, `vfh_rings`), and the
scalar topics read back sane (`heading_deg` 90, `direction_deg` 35,
`direction_heading_deg` 125, `nearest` 2.20, `samples_count` 240). The recorded
`opening_width_deg` 300 and `blocked_sectors` 12 predate the 2026-08-03 FOV
mask; the same display now caps opening width at 70 deg and counts blind sectors
as blocked.

## If this work is revived

Do not resume at an armed VFH flight. First decide whether the real planner
should consume a persistent 2D occupancy grid or a 3D voxel map, then use a
route-search layer plus a local collision checker/controller. The existing VFH
code is best treated as a reference implementation for obstacle inflation,
camera-FOV constraints, world-frame memory experiments, and Foxglove telemetry.

1. **Tune `tau_high`/`tau_low` in the real room** with `vfh_monitor` against a
   known wall. Nothing else should happen first.
2. **Verify the approximately 0.30 m `robot_radius` measurement** prop-tip to prop-tip.
3. **Confirm the floor is out of the slab** — with `z_below` 0.15 the red fan
   should track real obstacles only. If a broad symmetric fan persists with open
   space ahead, check `/rtabmap/obstacle_cloud` in the 3D panel for points lying
   on the ground plane, and compare `/vfh/heading_deg` against PX4's heading in
   `/px4/local_position/odometry` — if those disagree by more than a few degrees
   the histogram is rotated and that is a different bug.
4. **Props-off dry run** (`auto_arm:=false climb_timeout:=5.0`), reading the bag
   afterwards for carrot placement and holds.

There is deliberately no armed procedure in this parked handoff. A future flight
requires a fresh architecture/safety review and a new test plan.

Unresolved and deliberately not built: a scripted goal source (the
`offboard_square` equivalent — corners computed up front rather than clicked)
and a proper map/search planner. A startup `0 -> -90 -> +90 -> 0 deg` scan is
built and tested, but scanning does not solve VFH's lack of route planning or
explicit free-space representation.
