# Planner/Follower Stability Changes — Implementation Handoff

Status: implementation requested, not started. This document is the complete
handoff for a fresh Claude/Codex session to implement three related fixes in the
flown 2D global-planner stack:

1. deterministic escape from `POSE_INSIDE_CLEARANCE`;
2. correction-aware accepted-path handling;
3. debounced `PATH_VALID` / `EXPLORING` mode transitions.

The repository was clean before this handoff was added. Current branch is
`main`; starting commit is `bd7eb5f` (`3d navigation md`). Do not stage or
commit unrelated generated files.

## Paste this into the new session

> Read `README.md` and `HANDOFF_PLANNER_STABILITY.md` completely. Implement all
> three changes in the C++ and Python 2D planner/follower paths, preserving
> parity and fail-closed behavior. Add ROS-free unit/parity regressions, build
> the package, and run the full package test suite. Do not arm or launch PX4.
> Use the recorded 2026-08-29 bags only for offline validation. Do not weaken
> current hard clearance except for the explicitly bounded monotonic escape
> rule described in the handoff.

## Repository and environment

- Repository: `/home/john/autonomous_drone_px4_vio`
- ROS workspace: `/home/john/autonomous_drone_px4_vio/ros2_ws`
- Package: `ros2_ws/src/px4_vio_bridge`
- ROS 2: Jazzy
- Overlay containing `px4_msgs`: `/home/john/ros2_ws/install`
- Normal domain: `ROS_DOMAIN_ID=42`
- Flown implementation: `cpp_nodes:=true` and `cpp_mode:=true`

Build/test environment:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash

colcon build --packages-select px4_vio_bridge --symlink-install
colcon test --packages-select px4_vio_bridge --event-handlers console_direct+
colcon test-result --verbose
```

Use `apply_patch` for edits. Preserve user changes if the worktree is no longer
clean when the new session begins.

## Why these changes are required

The latest relevant bags are:

| bag | result |
|---|---|
| `ros2_ws/flight_logs/offboard_global_20260829T071239Z` | reached the requested goal and landed normally |
| `ros2_ws/flight_logs/offboard_global_20260829T071457Z` | landed after a persistent cross-track fault |
| `ros2_ws/flight_logs/offboard_global_20260829T071555Z` | timed out with 1.81 m of route remaining |

Effective runtime configuration in all three bags was correct:

- hard clearance: `robot_radius + safety_margin = 0.25 + 0.00 = 0.25 m`;
- graded inflation: another `inflation_extra=0.25 m`, outer radius `0.50 m`;
- `lookahead=0.25 m`, `lookahead_step=0.03 m`;
- `max_cross_track=0.10 m`, `cross_track_resume=0.03 m`;
- `path_retain_tolerance=0.04 m`;
- correction gate `1.0 m / 10 deg`.

The final bag entered ROUTE at 9.0 s and hit the 60-second armed watchdog at
66.5 s. It moved along 7.18 m of corrected-map trajectory for only 2.51 m net
progress. The planner alternated between a complete route and exploration:

```text
 3.10  PATH_VALID
13.10  EXPLORING
14.10  PATH_VALID
22.10  EXPLORING
40.11  PATH_VALID
46.10  EXPLORING
55.10  PATH_VALID
```

It spent about 28 of 57 route seconds in `EXPLORING`. It also produced 124
plans and the follower reached generation 59. Sampled adapter command speed
averaged only 0.092 m/s, with one third of samples at or below 0.05 m/s.

Three `POSE_INSIDE_CLEARANCE` episodes occurred around 13, 14 and 31 seconds.
The third coincided with path replacement/projection holds and a loop/backtrack.
The vehicle resumed only after map/pose motion changed the geometry; that is not
a deterministic recovery.

RTAB-Map emitted repeated 4.3-6.0 cm correction steps from roughly 49-64 s.
Those are the same size as `path_retain_tolerance=0.04 m` and the adapter's
`path_command_projection_tolerance=0.05 m`. The absolute correction remained
safe (about 0.26-0.33 m, below the flown 1.0 m gate), but the frame changes were
large enough to churn accepted paths and commands.

This was not CPU starvation: trajectory setpoints were 20.02 Hz with a 67 ms
maximum gap, planning was normally 1-4 ms, there was no throttling, and the one
VIO reset sentinel occurred after LAND had already started.

Offline replay of the same maps found the requested goal connected for only 29
seconds and disconnected for 28 seconds at 0.25 m hard clearance. Reducing the
hard radius is deliberately out of scope here. The software must behave
predictably at the configured physical envelope.

## Safety invariants

These override convenience and mode stability:

1. Unknown and outside-map space remain blocked.
2. A normal command chord must retain the full configured hard clearance.
3. The only permitted sub-clearance motion is an escape beginning from a pose
   that is already below hard clearance. Every point of that command chord must
   be no closer to an occupied cell than the starting pose, and the endpoint
   must make material progress toward safety.
4. No hysteresis may retain a route that the newest map proves unsafe.
5. A pending mode transition must never report temporary exploration-frontier
   arrival as requested-goal completion.
6. Raw continuous VIO continues to feed PX4. Map corrections must never be
   injected into PX4 EKF2.
7. Missing, stale or internally inconsistent map/correction/path data remains a
   HOLD followed by bounded LAND behavior in the flight adapter.

## Change 1: deterministic monotonic clearance escape

### Current defect

`route_follower_node.cpp::safe_lookahead()` calls
`segment_has_clearance(pose, target, required_clearance)`. The segment includes
the pose. If the pose is already 0.24 m from an obstacle and required clearance
is 0.25 m, every candidate fails at its first point. The node calls
`hold_command()`, actively commanding the aircraft to remain in the offending
location. The same defect exists in Python
`route_follower_monitor.py::safe_lookahead()`.

The known-issues explanation is in `README.md`, under
`POSE_INSIDE_CLEARANCE is a deadlock, not a safety stop`.

### Required geometry API

Extend the ROS-free clearance layer in both languages. Suggested APIs:

```cpp
std::optional<double> point_clearance(
  const GridMap &, const Point2 &, int occupied_threshold = 65);

std::optional<double> segment_minimum_clearance(
  const GridMap &, const Point2 & start, const Point2 & end,
  int occupied_threshold = 65);
```

Python should expose equivalent functions in `grid_planner.py`.

Return no value for invalid input or if the point/segment enters unknown or
outside-map space. Distance is exact to occupied cells treated as full
axis-aligned cell boxes, matching `segment_has_clearance`; do not use obstacle
cell centres. `segment_has_clearance()` should delegate to the new primitive so
there is only one continuous-clearance implementation.

The map may be large, so preserve the current bounded-cell search for segment
queries. A point query may scan an expanding bounded window until the closest
possible unseen cell cannot improve the result, or use a cached exact distance
field keyed by map generation. Do not silently approximate below the required
clearance threshold.

### Escape-selection rule

Normal safe-lookahead behavior is unchanged when:

```text
start_clearance >= required_clearance
```

When the current pose is known free but below required clearance, search path
lookahead candidates for an escape chord. A candidate is permitted only if:

```text
segment is entirely known and in-bounds
segment_minimum_clearance >= start_clearance - numerical_tolerance
endpoint_clearance >= min(required_clearance,
                          start_clearance + escape_minimum_improvement)
```

Use a small explicit parameter for material improvement, suggested default
`escape_minimum_improvement=0.01 m`. The numerical tolerance should only cover
floating-point noise (order `1e-6 m`), not a grid cell. Prefer the farthest
candidate up to normal lookahead that satisfies the complete chord rule. If no
candidate improves clearance, remain invalid and HOLD/LAND normally.

The acceleration-limited intermediate carrot must be checked with the same
escape predicate. It is insufficient to validate only the desired lookahead.
Clear stale relative displacement/velocity when entering escape so a previous
command cannot point toward the obstacle.

Publish a distinct status such as:

```text
CLEARANCE_ESCAPING start=0.238m end=0.252m required=0.250m
```

and publish `valid=true` only while the actual limited carrot passes the escape
validator. Once the pose reaches full clearance, automatically return to normal
`FOLLOWING`. If no escape exists, use an explicit failure such as
`POSE_INSIDE_CLEARANCE_NO_ESCAPE`, `valid=false`.

Do not treat lateral/equal-clearance movement as sufficient indefinitely. The
endpoint improvement requirement prevents a path that merely follows the same
unsafe contour from being accepted as recovery.

### Tests for change 1

Add matching Python and C++ tests covering:

- exact point and segment clearance to full occupied cell boxes;
- unknown/outside segment returns no clearance / is rejected;
- pose at 0.24 m with 0.25 m required, path directly away: escape accepted;
- the entire away chord never falls below the 0.24 m starting clearance;
- path toward the obstacle: rejected;
- path that first gets closer and later gets safer: rejected;
- path parallel with no endpoint improvement: rejected;
- acceleration-limited intermediate carrots also obey the rule;
- normal full-clearance behavior is byte/field compatible with current output;
- escape exits cleanly into normal `FOLLOWING` without resetting cumulative
  requested-goal progress.

Primary files:

- `include/px4_vio_bridge/grid_clearance.hpp`
- `src/grid_clearance.cpp`
- `src/route_follower_node.cpp`
- `px4_vio_bridge/grid_planner.py`
- `px4_vio_bridge/route_follower_monitor.py`
- `test/test_grid_clearance_cpp.cpp`
- `test/test_grid_planner.py`
- `test/test_route_follower_cpp.cpp`
- `test/test_path_follower.py`

Extract the escape decision into ROS-free code rather than testing private ROS
node methods indirectly.

## Change 2: correction-aware accepted-path handling

### Current defect

Planner paths and follower state are stored as map-frame coordinates. When the
native transform `C = T_map<-vio` changes, the current corrected pose moves into
the new map solution but the accepted path is still numerically expressed in
the previous solution until a new planner path is accepted. Four-to-six
centimetre corrections therefore look like cross-track/path-replacement errors.

The follower's `CorrectionReplanGate` filters/coalesces an event and waits for a
fresh path, but it does not rebase the accepted route. Its default eight-second
cooldown can also ignore later material corrections; the final bag contained
multiple material steps about one second apart.

### Required transform math

For a correction represented by translation `t` and yaw `y`:

```text
p_map = R(y) * p_vio + t
```

Re-express a point from an old correction in a new correction with:

```text
p_new = R(y_new) * R(-y_old) * (p_old - t_old) + t_new
```

Equivalently:

```text
p_new = R(y_new - y_old) * (p_old - t_old) + t_new
```

Relative displacement and velocity rotate by `y_new - y_old`; translation
cancels. Put these helpers in ROS-free `path_geometry` and add exact tests for
translation-only, yaw-only, combined, wraparound and inverse round trips.

### Preferred representation

Store the accepted path canonically in continuous VIO coordinates and render it
into the current map solution as needed:

1. Snapshot the correction associated with every accepted planner candidate.
2. Convert accepted map points through `C^-1` into VIO coordinates.
3. For retention, validation and publication, transform the canonical points
   through the correction paired with the current occupancy grid.
4. Validate the transformed remaining route against the newest raw grid before
   retaining it.
5. A pure coordinate re-expression must not count as a new semantic path
   generation and must not reset cumulative route progress.

This avoids accumulating floating-point error from repeatedly applying deltas.
A minimal delta-transform implementation is acceptable only if it proves the
same invariants and has round-trip/drift tests.

The follower should likewise avoid interpreting a correction-only coordinate
change as cross-track motion. Two reasonable designs are:

- operate its polyline/progress state canonically in VIO coordinates, mapping
  pose/carrot segments into the current map only for occupancy clearance; or
- atomically transform the stored polyline, `path_start`, commanded displacement
  and command velocity on every accepted correction delta without incrementing
  route generation.

The first design is cleaner because `/planner/follower/vio_displacement` is the
quantity consumed by the C++ flight adapter.

### Pair map, correction and path generations

Do not clear the correction wait merely because any `/planner/path` message
arrived after the correction callback. It may have been produced from the old
grid. Add explicit generation/epoch association or an equally strong receipt
ordering invariant:

- increment a correction epoch for every material correction episode;
- require at least one map received after the episode's last material change;
- require a path planned from that map generation;
- only then release `WAITING_FOR_POST_CORRECTION_PATH`.

Preferred telemetry is latched scalar topics such as
`/planner/map_generation`, `/planner/path_map_generation` and
`/planner/correction_epoch`, plus corresponding config/status fields. If a
single custom metadata message is introduced, keep bag/replay tooling able to
decode it. A path header timestamp alone is not enough unless the implementation
proves which correction/grid snapshot produced it.

Replace the eight-second blind cooldown with episode settling:

1. filter jitter as today;
2. when a material threshold is crossed, start/update one pending episode;
3. reset the quiet timer on every further material change;
4. transform the retained path as the correction changes;
5. after quiet time and one new-map/new-path pair, set the new baseline and
   re-arm immediately or after a short sub-second guard.

Do not allow a one-second sequence of alternating 5 cm changes to pass through
unobserved merely because the first event entered cooldown.

### Planner integration details

The C++ planner currently decides replacement in
`global_planner_node.cpp` around `accepted_points_`,
`accepted_effective_goal_`, `path_projection()` and `should_replace_path()`.
The Python counterpart is the analogous block in
`global_planner_monitor.py`.

Before projection/replacement:

- re-express accepted points in the correction paired with the current grid;
- trim behind the current corrected pose only after re-expression;
- validate every retained segment against the current raw grid and hard
  clearance;
- compare candidate improvement and off-path distance in one consistent frame;
- transform or rederive the accepted effective goal; do not compare stale cell
  indices across changed grid origins/geometries;
- if the route is unsafe, replace/clear immediately regardless of hysteresis.

The planner needs a native correction subscription and correction validity/
freshness checks consistent with the follower. Do not invent a second transform
direction; reuse the tested helpers.

### Tests for change 2

- old path and pose receive the same 5 cm correction: cross-track stays near
  zero and generation/progress do not reset;
- yaw correction rotates path, pose, displacement and velocity consistently;
- a correction-only update does not trigger `path_retain_tolerance`;
- a real physical deviation after correction still triggers replacement or
  cross-track recovery;
- retained transformed path that collides in the new grid is rejected
  immediately;
- a stale path produced before the post-correction map cannot release the gate;
- several material corrections one second apart remain one settling episode or
  create correctly ordered new episodes; none is hidden by an 8 s cooldown;
- zero-mean sub-threshold jitter still does not rebuild or hold the route;
- C++/Python replay outputs remain equivalent.

Primary files:

- `include/px4_vio_bridge/path_geometry.hpp`
- `src/path_geometry.cpp`
- `include/px4_vio_bridge/route_follower.hpp`
- `src/route_follower.cpp`
- `src/global_planner_node.cpp`
- `src/route_follower_node.cpp`
- `px4_vio_bridge/path_follower.py`
- `px4_vio_bridge/global_planner_monitor.py`
- `px4_vio_bridge/route_follower_monitor.py`
- `src/grid_planner_replay.cpp`
- `src/route_follower_replay.cpp`
- `test/test_planner_flight_cpp.cpp`
- `test/test_route_follower_cpp.cpp`
- `test/test_path_follower.py`
- `test/test_grid_planner_parity.py`
- `test/test_route_follower_parity.py`

## Change 3: mode-transition hysteresis

### Current defect

Every plan immediately derives mode from one grid:

```text
exact                 -> PATH_VALID
not exact, terminal   -> SAFE_APPROACH
not exact, nonterminal-> EXPLORING
```

`effective_goal_changed` then forces an accepted-path replacement. A one-frame
connectivity change can therefore replace the endpoint, path, follower heading
and goal-completion semantics. The final bag oscillated between complete and
exploration routes four times.

### ROS-free state machine

Create a small, independently tested `GoalModeHysteresis` shared concept in C++
and Python. Suggested enum:

```text
PATH_VALID
SAFE_APPROACH
EXPLORING
```

Suggested parameter:

```text
mode_confirmation_maps = 2
```

Use distinct occupancy-grid generations, not 2 Hz planner ticks, for the
confirmation count. With a roughly 1 Hz map and the current 3 s planner-fault
LAND timer, default 2 is safer than waiting for three full additional maps.

Behavior:

1. A new requested goal has no old semantic commitment; commit the first valid
   result immediately.
2. If raw mode equals stable mode, clear any pending transition.
3. If raw mode differs, require `mode_confirmation_maps` consecutive distinct
   map generations with the same raw mode before committing it.
4. A contradictory raw sample resets/replaces the pending candidate.
5. Expose pending state in status, e.g.
   `MODE_PENDING PATH_VALID->EXPLORING 1/2`.

### Safety behavior during a pending transition

Mode debounce is not collision debounce.

- Revalidate the currently accepted remaining route against every new map.
- If it remains continuously safe, retain it and its effective endpoint while
  the semantic transition is pending.
- If it is unsafe, replace it immediately with the newest safe candidate or
  publish an invalid/HOLD result. Never fly the old path for another map merely
  to satisfy hysteresis.
- On any raw nonterminal result, publish conservative
  `goal_terminal=false` immediately, even before mode commitment. Promotion to
  terminal completion may wait for confirmation. This prevents an exploration
  frontier from being reported as the requested goal.
- `goal_exact` and status should distinguish raw/pending/committed state. Do not
  label an exploration endpoint `PATH_VALID` just because the old mode is still
  committed.

One practical output model is:

- `/planner/status`: stable mode plus pending suffix;
- `/planner/goal_exact`: committed exact state;
- `/planner/goal_terminal`: conservative completion permission (immediate false,
  confirmed true);
- `/planner/effective_goal`: endpoint of the route actually published;
- `/planner/candidate_path`: newest raw candidate as today;
- `/planner/path`: retained safe path or immediate safe replacement.

Within a stable `EXPLORING` episode, continue allowing a genuinely advancing
frontier. Do not add a general rule that freezes the endpoint behind the
aircraft. If adding same-mode endpoint hysteresis, require proof that it cannot
prevent forward exploration and keep it separately tested.

Add the new parameter to:

- `launch/global_planner_monitor.launch.py`;
- C++ and Python parameter declarations;
- `/planner/config` JSON;
- README parameter documentation.

### Tests for change 3

- first result commits immediately;
- one `PATH_VALID -> EXPLORING -> PATH_VALID` map spike produces no committed
  mode switch or safe-path replacement;
- two consecutive exploration maps commit `EXPLORING`;
- pending counter advances only on distinct map generations;
- unsafe old path is discarded immediately on the first map despite pending
  mode;
- raw nonterminal state immediately prevents requested-goal completion;
- terminal promotion requires confirmation;
- a new user goal resets stable/pending state;
- `SAFE_APPROACH` transitions are covered, not just exact/exploring;
- C++ and Python planners make identical decisions for the same sequence.

Primary files:

- a new ROS-free header/source, or `grid_planner.hpp/.cpp`, for the C++ tracker;
- `src/global_planner_node.cpp`;
- `px4_vio_bridge/global_planner_monitor.py`;
- `launch/global_planner_monitor.launch.py`;
- `test/test_grid_planner_cpp.cpp`;
- `test/test_grid_planner.py`;
- `test/test_grid_planner_parity.py`;
- `README.md`.

## Recommended implementation order

1. Add exact clearance-distance primitives and their tests.
2. Add the ROS-free escape selector/validator and parity tests.
3. Integrate escape behavior into both follower nodes.
4. Add correction transform helpers and tests.
5. Make planner accepted paths correction-canonical and validate transformed
   paths against each current grid.
6. Make follower route/progress correction-aware.
7. Replace cooldown-only correction gating with map/path generation pairing.
8. Add the ROS-free mode hysteresis state machine.
9. Integrate conservative completion semantics and safe-path retention.
10. Extend config telemetry, launch arguments, README and bag recorder topics if
    new generation topics were added.
11. Run formatting, build, all package tests and offline bag replay.

Keep commits/change groups reviewable; do not combine these fixes with the
separate 3D-navigation design or velocity-only PX4 control.

## Existing tests and replay tools

Relevant tests already present:

- `test/test_grid_clearance_cpp.cpp`
- `test/test_grid_planner_cpp.cpp`
- `test/test_route_follower_cpp.cpp`
- `test/test_planner_flight_cpp.cpp`
- `test/test_grid_planner.py`
- `test/test_path_follower.py`
- `test/test_grid_planner_parity.py`
- `test/test_route_follower_parity.py`
- `test/test_planner_flight_parity.py`

Useful scripts:

- `scripts/analyze_flight.py`
- `scripts/analyze_map_correction.py`
- `scripts/evaluate_planner_bags.py`

Example offline evaluator call:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash

python3 ../scripts/evaluate_planner_bags.py \
  --cost-band-clearance 0.25 \
  --continuous-clearance 0.25 \
  flight_logs/offboard_global_20260829T071555Z
```

Before modification, that bag reported:

```text
plans=124
recorded lookahead chords=661
recorded lookahead minimum clearance=0.251 m
adaptive reductions=0
```

The accepted normal chords were safe. The regression target is reduced
coordinate/mode churn and deterministic recovery, not permission to cut normal
corners.

## Completion criteria

Code is ready for review only when all of these are true:

- full package build succeeds;
- `colcon test-result --verbose` has no failures;
- Python/C++ parity tests cover all three new state machines;
- no normal chord below configured hard clearance is accepted;
- an already-subclearance pose can command only strictly improving,
  non-worsening escape motion;
- correction-only coordinate changes do not create cross-track, generation or
  progress jumps;
- a post-correction hold cannot be released by a pre-correction-map path;
- no material correction is hidden by the old eight-second cooldown;
- a one-map connectivity flip does not commit a mode switch when the retained
  path remains safe;
- an unsafe retained path is rejected immediately regardless of debounce;
- temporary exploration arrival cannot produce requested-goal completion;
- `/planner/config` and follower/planner status expose the new effective
  parameters and pending/escape states;
- README's `POSE_INSIDE_CLEARANCE` known issue is updated from “not implemented”
  to the actual implemented behavior.

Do not perform an armed test as part of this task. The next operational sequence
after code review is simulation/replay, then a props-off `auto_arm:=false` bag,
then a short controlled flight on a fresh battery with RC kill ready.

## Pitfalls to avoid

- Do not fix the deadlock by globally lowering `robot_radius` or
  `required_clearance`.
- Do not omit the starting point from the clearance test; explicitly enforce
  the non-worsening escape invariant instead.
- Do not accept a target that is safer only at its endpoint but passes closer to
  the obstacle in between.
- Do not transform only path vertices while leaving follower displacement,
  velocity, `path_start` or effective-goal state in the old frame.
- Do not increment semantic path generation for a pure coordinate re-expression.
- Do not compare effective-goal cell indices across grids with different origins
  or dimensions.
- Do not use mode hysteresis to delay collision invalidation.
- Do not leave `goal_terminal=true` during a pending transition toward
  `EXPLORING`.
- Do not let planner ticks count as independent map confirmations.
- Do not remove jitter filtering merely to eliminate the correction cooldown.
- Do not change the PX4 controller or implement velocity-only flight in this
  patch set.

