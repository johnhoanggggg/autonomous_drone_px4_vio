# Diagnosis: two flights with constrained-command heading, 2026-09-06

Read-only investigation of existing MCAPs; no flight code, configuration, or
PX4 parameters changed. No live commands published.

Both `/planner/flight/config` messages confirm `implementation=cpp` and
`yaw_direction_source=constrained_command_preview`. Both used corner blending,
0.20 m/s command speed, 0.30 m/s² acceleration, yaw pause/resume 40°/15°,
horizontal feedforward, and no replanning during yaw alignment.

## Flight 11:31:22 UTC

Bag: `/home/john/autonomous_drone_px4_vio/ros2_ws/flight_logs/offboard_global_20260906T113122Z`

- Route started at bag +6.19 s; automatic landing triggered at +11.89 s.
- Over that 5.70 s window, continuous body VIO shows 1.078 m net horizontal
  displacement and follower remaining distance fell 2.876 -> 1.906 m.
- Command speed median 0.20 m/s; no yaw-align episodes recorded.
- For 211 moving samples above 0.10 m/s, heading/course absolute error median
  3.35°, p90 8.61°; heading/wire-yaw median 1.36°. Wire yaw versus nonzero
  velocity feedforward median 0.28°, p90 15.73°.
- Explicit landing reason: `RTAB-Map tracking features stayed below 150 for
  0.25s (latest=146) -> AUTO.LAND`. This was the feature-count watchdog, not
  evidence of a preceding prolonged pose-output loss.
- During route: body VIO 13.30 Hz, maximum receipt gap 0.103 s; PX4 external
  vision input 13.35 Hz, maximum gap 0.112 s.
- Follower cross-track fault begins +12.21 s, reset sentinel +13.51 s, and
  keyboard forced disarm +14.09 s: all after the landing trigger. These later
  symptoms cannot explain the original decision to land.
- Reported battery voltage falls to 10.82 V near the trigger, then 10.12 V at
  +12.7 s during descent. Worth investigating separately; the logged trigger
  was not battery voltage. No proof of a brownout in this recording.

## Flight 11:32:39 UTC

Bag: `/home/john/autonomous_drone_px4_vio/ros2_ws/flight_logs/offboard_global_20260906T113239Z`

- Route starts +9.54 s; keyboard forced disarm +51.99 s.
- 12 YAW_ALIGN entries: 39.54 s of 42.45 s (~93%) spent aligning; about 2.86 s
  in ROUTE status. Median command speed zero; p90 0.015 m/s.
- Net horizontal body-VIO displacement 0.295 m; follower remaining distance
  2.589 -> 2.288 m. Its cumulative progress 0.792 m should not be confused with
  useful net progress, given oscillations/replans.
- At +10.14 s yaw target changes to 73° while actual heading is ~125°.
  While stopped in a join, target continues to 9°; path offset is only ~0.026 m.
- At +15.54 s alignment finally releases at target 9°; +15.59 s target becomes
  92° and alignment immediately restarts. Join telemetry simultaneously changes
  true -> false. Similar cycles repeat at +22.09/+22.14 and +33.09/+33.14 s.
- The first route leg in this interval is nearly east in map coordinates,
  approximately NED 92°, not the NED 9° sideways join correction.
- Body VIO continues at 12.94 Hz, max route-window gap 0.135 s; external vision
  input 12.93 Hz, max gap 0.134 s. Follower remains FOLLOWING throughout route.
  VIO reset sentinel appears only after kill. Startup stale inputs and reset
  counters preceded route entry and do not explain the repeated alignment.
- Moving heading/course median error 74.75°, but 291/323 samples occur during
  YAW_ALIGN, so this primarily measures drift/position correction while turning,
  not successful route travel. Actual heading tracks transmitted yaw with
  median error 7.79°: PX4 is following the changing turn commands.

## Root cause and independent reproduction

The new heading choice is unstable in the existing rolling path-join behavior.
`PathCommandLimiter::update_join()` combines an along-path component dependent
on current speed with a lateral component dependent on path offset. The new
`preview_direction()` normalizes the next advancing velocity, including when
the actual limiter is stopped for yaw alignment. Braking removes the forward
component, so a few-centimetre lateral correction becomes the main heading.
Alignment then changes the very direction it is trying to align with.

A deterministic experiment using the built C++ replay harness and one straight
eastbound path with a 3 cm offset reproduced this independently of ROS, VIO or
map updates. Preview heading at 0.20 m/s: NED 85.80°. After braking to rest on
the same path, same lateral offset: NED 6.38°. Almost 80° of reference rotation
without any actual route turn.

The adapter also permits pending path installation immediately after
`yaw_holding_` clears (`update_route_command`). Installing that path can replace
the just-aligned join direction, causing another pause on the following tick.
Logs and joining telemetry support this mechanism; exact internal accepted-path
snapshots were not recorded, so every individual installation cannot be proven
from the bag alone.

This is a regression introduced by the new heading selection. The previous
tests verified preview purity and one-tick velocity agreement, not stability
across braking, alignment, release and pending-path installation. That coverage
was insufficient for this failure mode.

## Other checks and next change to design

- Mean continuous-body-VIO/NED heading offsets remain small (-0.15°, +1.25°),
  not a large fixed frame calibration error. Map correction yaw stays within
  about ±0.36° during both route windows.
- CPU medians ~74%, peaks 83.5/84.7%; sampled Pi throttling flags are all zero.
  CPU headroom is limited, but these logs do not implicate throttling or a
  sustained VIO dropout as the second flight's stall cause.
- Installed VehicleStatus definitions cannot decode 35/115 recorded messages.
  Route/landing windows use adapter logs/status instead of independently
  decoded vehicle arming state. Angle comparisons use causal receipt-time
  pairing with maximum age 0.30 s, not synchronized sensor timestamps.

Do not repeat an armed flight with this heading behavior unchanged. The next
fix should define a stable, clearance-valid trajectory heading across stopping
and joining, prevent tiny lateral corrections dominating yaw at rest, and
coordinate pending path acceptance with alignment release. Latching alone is
not enough if it preserves a bad sideways heading or ignores an unsafe path.
Add a full multi-tick alignment/replan regression and props-off integration
checks; do not mask it by weakening tracking, clearance or yaw safety gates.

The first flight's feature-count landing is a separate issue: inspect vision
quality before deciding whether its threshold is appropriate. No threshold
change is supported merely by this short successful tracking window.

## Follow-up: how far the at-rest preview swings, and its trigger threshold

Re-ran the replay experiment across offsets, same limiter config as flown
(0.20 m/s, 0.30 m/s², corner blending on, dt 0.05 s), one straight eastbound
path, previewing while moving and again after braking to rest:

| Lateral offset at replacement | Preview heading, moving | Preview heading, at rest |
|---|---:|---:|
| 0.01 m | NED 90.0° | NED 90.0° |
| 0.03 m | NED 85.8° | NED 6.4° |
| 0.10 m | NED 85.9° | NED 4.3° |

The swing is not confined to small offsets: it grows with them. From rest the
first tick is acceleration-limited, so the preview velocity is the *change*
vector, whose along-path term is only `max_acceleration * dt` (0.015 m/s) while
the lateral term is `min(max_speed, offset/dt, sqrt(2*a*offset))` (0.134 m/s at
3 cm). The one-tick velocity direction is therefore the offset-closing
direction, not the route direction.

The 0.01 m row does not swing only because the replacement never opened a join:
`PathCommandLimiter::join_tolerance_` is 3 mm (`command_limiter.hpp:153`), so
anything beyond 3 mm of cross-track enters `update_join()` and is exposed. With
the planner replacing the accepted path roughly once a second, nearly every
replacement while stopped for alignment re-points the heading sideways, which
is the self-sustaining loop seen in the 11:32:39 flight: align -> stop ->
replacement opens a join -> at-rest preview reads ~85° off route -> exceeds the
40° pause threshold -> align again.

Implication for the fix: the heading reference must not be a one-tick velocity
whenever the limiter is at or near rest. The stable quantity available at rest
is path geometry -- the tangent at the projected/lookahead point of the
*accepted* path -- with the join's lateral term excluded from the heading
decision. Any redesign needs a multi-tick regression that brakes, replaces the
path with a several-centimetre offset, releases alignment and asserts the
heading target stays on the route direction.
