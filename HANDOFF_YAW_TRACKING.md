# Heading versus travel investigation — 2026-09-06

Branch: `enclosed-spaces-tuning`. Investigation used existing MCAP files.
The follow-up heading change is now implemented in the C++ flight adapter and
built locally. No PX4 parameters were changed and no flight was started.

## Findings

The previous analysis incorrectly subtracted ENU map yaw from NED heading and
reported a roughly -145 degree frame offset. Convert continuous body VIO yaw
using `yaw_ned = wrap(pi/2 - yaw_enu)` instead. A map pose additionally contains
SLAM correction rotation, so it is not interchangeable with the VIO pose.

Corrected comparisons, using moving route samples above 0.10 m/s, latest past
messages no older than 0.30 s, and ROUTE/YAW_ALIGN/CORNER_HOLD status:

| Aug 31 flight (UTC) | Samples | Body VIO vs PX4 mean | Heading vs transmitted yaw, median absolute | Heading vs course, p90 absolute | Transmitted yaw vs velocity feedforward, p90 absolute |
|---|---:|---:|---:|---:|---:|
| 08:25:07 | 216 | -0.1° | 1.6° | 34.5° | 31.3° |
| 06:42:36 | 884 | -1.3° | 4.5° | 77.5° | 64.2° |
| 06:48:28 | 553 | +0.2° | 2.0° | 84.2° | 55.8° |

The yaw/velocity comparison uses fields from the **same TrajectorySetpoint**.
The vehicle heading tracks transmitted yaw closely, while some transmitted
velocity vectors disagree substantially with that yaw. A clear example at
roughly 28 s in 06:48:28 has heading 86°, transmitted yaw 88°, velocity
feedforward -19°, and measured course -20° (rounded receipt-time sample).

There is no evidence here for a large fixed yaw calibration offset. The code
previously contained separate direction choices: in
`src/offboard_global_planner_node.cpp`, `update_route_command()` first calls
`update_yaw_target()` with the follower's VIO displacement. Only afterwards it
installs/retains the route, advances the path command limiter, converts the
final position and velocity, and adopts those for transmission. Yaw deadband,
minimum follower displacement and yaw slew further separate the commands.
Path replacements, joins and retained paths can therefore command translation
in a direction different from the direction used for yaw alignment.

## What these logs cannot establish

- Position feedback, velocity/acceleration feedforward, and vehicle momentum
  jointly determine course. Matching yaw to velocity feedforward alone is not
  a complete control solution.
- There is no measured camera-to-wall viewing angle. Feature counts were
  actually higher on average in the >=30° crab group in all three flights;
  these counts do not prove that the heading mismatch caused tracking loss.
- Comparisons use receipt times and include sensor/transport latency.
- All recorded `/fmu/out/vehicle_status_v1` messages failed to decode with the
  installed message definitions (41/114/76 messages respectively). The analysis
  reports this explicitly and uses planner status for phase filtering; it
  cannot independently verify arming/offboard state from those messages.
- One body VIO pose in 08:25:07 also failed to decode and is excluded.
- 08:25:39 has no moving route samples. 08:26:53 has a zero-byte MCAP and no
  recoverable samples. No bag was reindexed or modified.

## Implemented flight-code follow-up

Use the accepted, constrained command direction to choose yaw before releasing
translation, with preview/snapshot semantics so deciding alignment does not
advance the real limiter. Account for stopped corners, entry/join motion,
rejected replacements and braking. Deriving yaw only from the current output
velocity would deadlock when stopped for yaw alignment, and blindly following
instantaneous measured velocity would turn the camera into drift/corrections.
`PathCommandLimiter::preview_direction()` now performs that preview on a copy
with shared immutable path geometry. The adapter removes map correction yaw,
converts to NED, then applies its existing yaw deadband and alignment hysteresis.
When there is no advancing velocity, a sufficiently distant standing position
target supplies the heading; otherwise it retains the last yaw. Actual braking
still uses the original `advance=false` behavior. Rejected commands restore yaw
target/alignment state as well as the limiter snapshot. First route entry can
install a path while alignment is latched, avoiding a stopped-entry deadlock.

Seven C++ regressions cover stopped preview, full join-state preservation,
rejected replacements, braking during alignment, incoming/outgoing corner
headings, no-motion/invalid previews, and correction-to-NED rotation. Existing
limiter parity remains unchanged. Speed limits, yaw thresholds, corner blending,
clearance and geofence margins have not been retuned. Python remains legacy.
Final local verification: package build succeeded; `colcon test-result` reports
566 tests, zero errors/failures and three skips. The five standalone heading
analysis regressions also pass. `git diff --check` is clean.

Geometry-only replay using three existing flights checked 1,011 non-mutating
previews and 913 advancing directions, with maximum unit-vector difference
3.2e-16 between preview and the subsequent advancing command. No replay
operations were rejected. **This is not full ROS-node or closed-loop replay:**
recorded poses/holds remain fixed, clearance is stubbed true, and current launch
limiter settings are assumed because these old bags lack flight config. It
does not validate actual course, the ROS safety-gate wiring, or flight safety.

Next gate: props-off `auto_arm:=false` using the C++ adapter. Check first entry,
an L-turn, a replacement during yaw hold, and a blocked route. Only then perform
a controlled low-speed flight with an accessible RC kill. A fixed yaw offset
is not supported by the recorded evidence. The installed executable takes
effect on the next launch with `cpp_mode:=true`; verify
`/planner/flight/config` contains
`"yaw_direction_source":"constrained_command_preview"`.

## Reproduce

```bash
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
python3 scripts/analyze_yaw_tracking.py \
  ros2_ws/flight_logs/offboard_global_20260831T082507Z \
  ros2_ws/flight_logs/offboard_global_20260831T064236Z \
  ros2_ws/flight_logs/offboard_global_20260831T064828Z \
  --out /tmp/yaw_tracking_20260906.json
python3 -m pytest -q scripts/test_analyze_yaw_tracking.py
python3 scripts/replay_command_heading.py \
  ros2_ws/flight_logs/offboard_global_20260831T082507Z \
  ros2_ws/flight_logs/offboard_global_20260831T064236Z \
  ros2_ws/flight_logs/offboard_global_20260831T064828Z
```

The JSON contains per-phase statistics and one-second timeline samples. Five
regressions cover ENU/NED cardinal directions, stale/future messages, wrapped
angles, invalid quaternions, and an aligned body pose with divergent yaw and
velocity commands. All pass.

## Flight results, 2026-09-06 — this change is a regression, do not re-fly it

Two armed flights ran with this heading behaviour
(`offboard_global_20260906T113122Z`, `...T113239Z`). Both
`/planner/flight/config` messages confirm `implementation=cpp` and
`yaw_direction_source=constrained_command_preview`, so the change was live.
Full analysis in `FLIGHT_DIAGNOSIS_20260906.md`.

- 11:31:22 flew 1.08 m of net VIO displacement with good heading tracking
  (heading vs course median 3.35°, p90 8.61°) and then auto-landed on the
  feature-count watchdog (`below 150 for 0.25 s, latest=146`) — a separate
  vision-quality issue, not a heading failure.
- 11:32:39 spent 39.5 s of a 42.5 s route in YAW_ALIGN (12 entries, ~93%) and
  netted 0.295 m. VIO held 12.9 Hz with a 0.135 s worst gap throughout, so
  tracking loss does not explain the stall.

Root cause is in this change. `preview_direction()` normalises the *next*
advancing velocity even when the real limiter has braked to rest for yaw
alignment. From rest that velocity is acceleration-limited, so its along-path
term is `max_acceleration * dt` (0.015 m/s) while `update_join()`'s lateral
offset-closing term is up to `sqrt(2*a*offset)` (0.134 m/s at 3 cm). The
preview then reports the offset-closing direction as the travel heading.
Replay with the flown config: NED 85.8° while moving, NED 6.4° at rest on the
same path with a 3 cm offset; 4.3° at 10 cm. `join_tolerance_` is 3 mm, so any
path replacement beyond 3 mm opens a join and is exposed, and the planner
replaces the path about once a second. Alignment therefore changes the very
direction it is aligning to. The adapter also installs a pending path
immediately after `yaw_holding_` clears, which can replace the just-aligned
join direction on the next tick.

What the earlier validation missed: the geometry-only replay checked preview
purity and one-tick agreement between preview and the subsequent advancing
command. Both hold here — the preview is faithful. It never exercised braking,
alignment release and path replacement across multiple ticks, which is where
the reference rotates.

Corrected next steps, superseding "Next gate" above:

1. Do not repeat an armed flight with this heading selection unchanged.
2. Derive the heading from accepted-path geometry (tangent at the projected or
   lookahead point) whenever the command point is at or near rest, and keep the
   join's lateral term out of the heading decision entirely.
3. Coordinate pending-path acceptance with alignment release so a newly
   installed path cannot invalidate the alignment just completed.
4. Add a multi-tick regression: brake to rest, replace the path with a
   several-centimetre offset, release alignment, assert the heading target
   stays on the route direction. Then re-run the props-off `auto_arm:=false`
   gate before any armed flight.

Latching the last yaw alone is not a fix: it would preserve a sideways heading
if one is latched, and it must not be allowed to ignore an unsafe path.

## Fix implemented, 2026-09-06 — heading comes from route geometry

`PathCommandLimiter::preview_direction()` no longer normalises the previewed
tick's command velocity. It previews the tick exactly as before (same copy, same
purity, same corner and join state), then reports the **forward path tangent at
the previewed progress** via the new private `advance_tangent()`. A forward
difference, not the central one `path_tangent()` uses, so at a vertex it is the
outgoing leg rather than the bisector. The tangent does not depend on the
current speed, so braking, joining and releasing all report the same heading,
and `update_join()`'s lateral term never reaches the nose.

The position-target fallback is kept, but now applies only when the previewed
tick commands no along-path motion at all (`speed_` zero and progress
unchanged): at the goal, or held off a corner the vehicle has not reached. That
preserves "retain the last yaw when nothing is being commanded".

Same deterministic experiment as the diagnosis, now at four offsets:

| Lateral offset at replacement | Preview, moving | Preview, at rest |
|---|---:|---:|
| 0.02 m | NED 90.0° | NED 90.0° |
| 0.03 m | NED 90.0° | NED 90.0° |
| 0.10 m | NED 90.0° | NED 90.0° |
| 0.25 m | NED 90.0° | NED 90.0° |

Previously 85.8° moving and 6.4° at rest for the 3 cm case. Note the moving
number changed too: yaw no longer crabs by the lateral correction even while the
join is being flown.

Coverage added to `test/test_planner_flight_cpp.cpp`:

- `PreviewFollowsTheRouteNotTheJoinAndLeavesAllJoinStateIntact` — replaces the
  old assertion that preview equals the next advancing join velocity, which is
  the assertion that encoded the regression. It now asserts the route direction
  and that the tick it ignores really does carry the lateral term.
- `HeadingSurvivesBrakingThenAReplacementDuringAlignment` — the flown failure:
  run up to speed, brake to a standstill under alignment, replace the path
  0.02/0.03/0.10 m sideways, preview at rest. Offsets must exceed
  `suffix_tolerance` or the replacement is renumbered geometry and no join opens.
- `RepeatedReplacementsWhileHeldCannotWalkTheHeading` — 20 replacements at
  alternating offsets across 100 held ticks; commanded heading must stay on the
  route to 1e-9 rad.

`scripts/replay_command_heading.py` checked the old invariant, so it was
adjusted rather than left to enforce the bug: preview must still equal the next
advancing velocity exactly while the command point is on the path, joins are
reported instead as `joining_direction_degrees`, and a chord that straddles a
direction change (blended corner, or the first tick on a replacement leg) is
accepted only when the velocity lies in the angular wedge between the previous
and current preview. `planner_flight_replay` now dumps `joining` for it; the
Python parity test reads named fields only and is unaffected.

Geometry replay over five flights (the three 20260831 bags plus both 20260906):
1,974 non-mutating previews, 637 on-path directions agreeing to 1.9e-12, 4
corner/replacement chords, and 437 joining ticks whose command velocity sits up
to **85.7°** away from the heading now commanded — the regression, measured on
recorded geometry, and the margin the fix removes.

Verification: `colcon test` reports 594 tests, 0 errors, 0 failures, 3 skips
(previously 566/0/0/3). `scripts/test_analyze_yaw_tracking.py` 5 passed.
`git diff --check` clean. No PX4 parameters or launch defaults changed.

### Deliberately not changed: pending-path installation

The diagnosis also proposed gating pending-path acceptance on alignment release.
That gate was not added. With the heading taken from geometry, a replacement
installed the tick after `yaw_holding_` clears now yields the new path's tangent,
which for a small lateral shift is within a degree of the heading just aligned
to — the churn source is gone. A large tangent change now means the route
genuinely turned, and pausing to face it is the intended behaviour. Adding a
gate on top would only delay legitimate replans and starve the follower.

### Still required before flying this

Nothing here is closed-loop. Run the props-off `auto_arm:=false` gate with
`cpp_mode:=true` and confirm `/planner/flight/config` still reports
`"yaw_direction_source":"constrained_command_preview"`, then walk the four cases
from the original gate — first entry, an L-turn, a replacement during a yaw
hold, and a blocked route — watching `/planner/flight/status` for repeated
YAW_ALIGN entries on a route that is not turning. Only then a low-speed flight
with an accessible RC kill. The 11:31:22 feature-count landing is untouched by
this change and is still open.
