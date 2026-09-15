# Follow-up after the 12:56 and 12:57 tunnel flights

## What the new flights establish

Both flights ran the rebuilt strict C++ adapter. Consecutive active command
velocity steps were at most 15 mm/s, versus 67.1 mm/s in the earlier 11:37
flight. This confirms the intended command improvement at the configured
20 Hz rate. Receipt-time acceleration estimates retain transport jitter.

The user observed excessive proximity to the right tunnel wall. The accepted
route through the affected section was already nearly straight and pointed
back toward the corridor. In the second flight the body crossed 4 cm toward
one wall at bag-local t=23.253, entered degraded tracking at 23.699, crossed
10 cm at 24.033, entered clearance escape at 24.056, and encountered a strict
command rejection at 24.647. Thus the later rejection does not explain the
initial lateral excursion. The maximum measured wallward cross-track was
13.3 cm; occupancy-derived body clearance reached 15.7 cm while the nearest
accepted-route point had 28.9 cm. These are map estimates, not measured rotor
clearances. The sign is +map-x here, not a claim about body-frame right.

Onboard logs pair exactly with the bags. Added EV diagnostics recorded
successfully: no rejected position/yaw fusion updates or position resets in
the analyzed flight windows. This does not rule out smooth VIO bias or drift.
See the [flight analysis](reports/new_flights_20260913/ANALYSIS.md) and
[onboard comparison](reports/new_onboard_20260913/README.md).

## Change made

The strict limiter had an avoidable rejection when the follower's requested
point fell behind an already-moving command. Ordinary tracking set its scalar
speed to zero, but the exact position secant could not stop the carried velocity
within one acceleration-limited tick. A related case rejected a requested
forward step below the physically reachable braking interval. These failure
mechanisms are reproduced in focused tests; the recordings do not contain
complete internal limiter snapshots to attribute every rejected tick exactly.

The limiter now chooses the nearest reachable forward braking step along the
accepted straight segment. A soft follower target can be passed by the distance
needed to brake; sharp bends and path endpoints remain hard geometric bounds.
Actual speed follows the selected secant. Collinear waypoint subdivisions do
not create artificial braking boundaries. Rejoins honor requested progress
and check the transition back to ordinary tracking, so retaining the join
cannot turn a retreated target into indefinite cruise.

The adapter still validates the resulting chord and geofence, and rolls back
rejected commands. The full vector acceleration and speed bounds stay in place.
Flight configuration now records
`path_target_retreat=brake_on_validated_segment`.

This change addresses interrupted recovery, not the initial physical tracking
error. It does not demonstrate a contact-free tunnel traversal. Route costs,
wall margins, flight speeds, yaw settings and PX4 gains were not changed.

## Saved flight renders

- [12:56:31](ros2_ws/flight_logs/offboard_global_20260913T125631Z/render/index.html)
- [12:57:30](ros2_ws/flight_logs/offboard_global_20260913T125730Z/render/index.html)

The second MCAP has a truncated trailing record and empty metadata. Valid
messages were recovered by direct sequential reading; the original recording
was preserved. Both folders also contain their matched ULogs, response plots,
and exact pairing results.

## Validation

Focused regressions exercise a target inside the braking distance, a target
retreating behind the command, stopping during a rejoin, crossing collinear
waypoints while braking, exact secants, vector bounds and finite stopping
distance. Existing corner, replan and varying-timestep tests remain relevant.

Root package build succeeded; `colcon test-result` reports **704 tests, zero
errors/failures and three existing legacy rolling-join parity skips**. The C++
limiter target passed all 58 cases. `git diff --check` passed and the installed
C++ adapter resolves to the freshly rebuilt binary. Commands, with documented
ROS/DepthAI underlays sourced, from `ros2_ws`:

```bash
colcon build --packages-select px4_vio_bridge
colcon test --packages-select px4_vio_bridge --event-handlers console_direct+
colcon test-result --test-result-base build/px4_vio_bridge --verbose
```

Transcripts: `reports/new_onboard_20260913/build.log` and `tests.log`. The build
retains the existing overlay-path warning and GCC ARM ABI notes. No hardware
flight or closed-loop simulation was performed. Two workers
handled independent bag analysis/rendering and the limiter fix; root reviewed
the causal ordering, integrated and extended the fix, downloaded/analyzed the
onboard logs, and ran package verification.

Restart the flight launch after this build to load it. The same three-terminal
commands select the C++ adapter by default; no flight was launched by this work.
