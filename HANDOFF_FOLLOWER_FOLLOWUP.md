# Planner/Follower Stability — Follow-up Handoff

Status: the three changes from `HANDOFF_PLANNER_STABILITY.md` are implemented in
C++, built, unit-tested and flown. Four bugs found in flight since then are also
fixed and flown. **The last flight recorded no faults.** This document is the
state of play for a fresh session.

Python (`global_planner_monitor.py`, `route_follower_monitor.py`,
`path_follower.py`, `grid_planner.py`) is **legacy** by explicit instruction:
implement in C++ only, keep the existing parity tests green by giving new C++
parameters backward-compatible defaults, and do not port features back.

## Environment

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
export ROS_DOMAIN_ID=42

colcon build --packages-select px4_vio_bridge --symlink-install
colcon test --packages-select px4_vio_bridge
colcon test-result --verbose      # baseline: 516 tests, 0 failures
```

Flown configuration is `cpp_nodes:=true` (planner + follower) and
`cpp_mode:=true` (flight adapter). `install/` entries are symlinks into
`build/` and the launch files are symlinks into `src/`, so a rebuild is
immediately live — no reinstall step.

## What is implemented and flight-verified

| change | where | flight evidence |
|---|---|---|
| Exact clearance primitives (`point_clearance`, `segment_minimum_clearance`); `segment_has_clearance` delegates | `grid_clearance.{hpp,cpp}` | offline replay reproduces the follower's own `start=0.227m` to the mm |
| Monotonic clearance escape in the follower | `route_follower.{hpp,cpp}`, `route_follower_node.cpp` | `CLEARANCE_ESCAPING start=0.239m end=0.429m required=0.250m`, 14% of the last route window, no deadlock |
| Same escape rule on the adapter's post-limiter gate | `offboard_global_planner_node.cpp` | `ROUTE valid ... escaping clearance=0.246/0.250m` — was `COMMAND_HOLD` before |
| Correction-canonical routes (VIO storage, re-expressed per tick) | `route_follower.cpp`, `global_planner_node.cpp`, `path_geometry.cpp` | cross-track stays <0.08 m through 3 correction epochs |
| Correction episodes gate path *acceptance*, not commanding | `route_follower_node.cpp` | `CORRECTION_SETTLING epoch=N ...` with `valid` staying true |
| Map/path generation pairing | `/planner/map_generation`, `/planner/path_map_generation`, `/planner/correction_epoch` | a pre-correction-map path cannot release the hold |
| Goal-mode hysteresis (`mode_confirmation_maps`, default 2) | `grid_planner.{hpp,cpp}`, `global_planner_node.cpp` | **not yet exercised in flight** — see open items |

### The four in-flight bugs already fixed

Each was found in a bag, has a regression test, and each test was verified to
fail against the old code:

1. **Escape latch chatter** (`20260829T084036Z`). A pose sitting on the
   clearance threshold re-entered the escape every tick, and the unconditional
   edge-triggered wipe destroyed the accumulating command ~10x/second —
   `progress=0.00m` for six seconds, then a cross-track landing. Fixed by
   making the wipe conditional on the stored command actually failing the
   escape predicate. Test `ClearanceEscape.ThresholdChatterDoesNotStarveTheCommand`.
2. **Adapter vetoed every escape** (`20260829T085734Z`). The post-limiter gate
   was a plain `segment_has_clearance`, so the deadlock reappeared as
   `COMMAND_HOLD ... insufficient clearance`. Fixed by `command_chord_permitted`
   using `ChordRole::IntermediateCarrot`.
3. **Startup deadlock** (`20260829T102157Z`, `102221Z`, two more). A correction
   episode opening before the first path was installed latched permanently:
   `on_path` deferred the first path, and `tick()` returned at
   `WAITING_FOR_PATH` before ever calling `waiting()` — the only thing that
   settles an episode. Fixed by `defer_path_for_correction(pending, have_route)`
   and by advancing the episode ahead of every persistent early return. Tests
   `TheFirstPathIsNeverDeferred`, `SettlesOnReceiptsAndTimeWithoutAnyRoute`.
4. **Filter convergence read as correction motion** (`20260829T103521Z`). The
   raw correction is a staircase — 5 loop closures in 26 s — but the gate
   compared the *lagging filtered* value against a reference taken from that
   same lagging value, turning one 333 mm step into ~7 phantom material events
   over 1.5 s. Episodes ran 2.4 s instead of 0.4 s, the route aged, and
   cross-track faulted. Fixed by asking the **raw** correction whether it is
   still moving, and by snapping the filter to the settled raw value on release
   (otherwise the same lag opened a phantom second episode). Test
   `FilterConvergenceIsNotAMaterialChange` measures settle latency after a
   single 333 mm step at 13 Hz; it fails at 1.15 s against the old code.

## Last flight: `offboard_global_20260829T105429Z`

Flown with `robot_radius:=0.25 safety_margin:=0.00 lookahead:=0.25
max_carrot_speed:=0.20 max_cross_track:=0.10 path_retain_tolerance:=0.04
command_speed:=0.20`, adapter `cpp_mode:=true`.

No faults of any kind. 89.8% of the route window in `FOLLOWING`; zero
cross-track events, zero `COMMAND_HOLD`, zero planner faults, no LAND. The bag
ends at 25.6 s because the recorder was stopped, not because anything failed.

What it is not is fast:

```
closed on goal        2.59 m -> 1.35 m in 19.7 s
ground path travelled 2.75 m for 1.40 m net displacement
straightness          0.51        (1.0 = perfectly direct)
adapter command_speed median 0.11 m/s, 25% of samples below 0.05 (cap 0.20)
follower generations  1 -> 19 in 19.7 s = 0.91/s
planner off_path      median 0.030 m, p90 0.080 m
CORRECTION_SETTLING   2.0 s of 19.7 s in 2 episodes (~1 s each)
CLEARANCE_ESCAPING    2.8 s of 19.7 s (14%)
```

## Open items, most valuable first

### 1. `path_retain_tolerance` sits below the vehicle's own tracking error

**This is tuning, not a defect, and it is the dominant remaining cost.**
`path_retain_tolerance:=0.04` against a measured `off_path` of median 0.030 m
and p90 0.080 m means **33% of planner ticks rebuild the route**. Each rebuild
re-anchors `path_progress` and re-aims the yaw target, which pauses translation
while the vehicle pivots and drifts — which raises the tracking error again.

The window is genuinely narrow, and widening it is a real trade, not a free fix:

* `path_retain_tolerance` must stay below `max_cross_track` (README known issue)
  or the vehicle stalls faulted with no new path coming;
* `max_cross_track` is currently 0.10, so the usable band is 0.08–0.10;
* raising `max_cross_track` eats clearance budget: with `safety_margin:=0.00`,
  worst-case clearance is `0.25 - max_cross_track`.

Do not silently change either. Measure `off_path` from the next few bags, then
put the numbers in front of the operator with the clearance consequence stated.

### 2. Goal-mode hysteresis has no flight evidence

`MODE_PENDING` has appeared in **0** planner ticks across every flight so far.
Every observed mode change committed immediately because the retained route was
genuinely unsafe or off-corridor — which correctly bypasses the debounce. The
spike-suppression path is therefore unit-test-only. Do not assume it works in
flight; look for `MODE_PENDING` in the next bags that contain a connectivity
flip, and if it never appears, question whether `transition_hold` is reachable
with these parameters.

### 3. Correction-episode latency is structural

Episodes now settle in ~0.4 s of quiet, but total ~1 s because the release also
needs the first map after the material change (~1 Hz) and a path planned from it
(planner 2 Hz). About 0.5 s per episode is spent past the quiet time purely on
that pairing. There is no shortcut that keeps the invariant; it is bounded by
the map and planner rates. Only revisit if it starts costing flights.

### 4. Untested wiring

`command_chord_permitted` in the adapter, and the path-deferral branch in
`on_path`, live in ROS node bodies with no automated coverage. The pure
functions underneath them are well covered. If either is touched, re-verify by
replaying a bag rather than trusting the suite.

### 5. Recurring 0-byte `metadata.yaml`

Every bag stopped with Ctrl-C lands with a 0-byte `metadata.yaml`. Recover with
`ros2 bag reindex <bag> -s mcap` before any analysis. Known issue, documented.

## Analysis tools

```bash
# Map + trajectory + fault table. Clearances are measured against the grid
# current at the time asked, and are exact (box distance), which is why the
# tool reproduces the follower's own numbers.
python3 scripts/render_flight_map.py <bag> --clearance 0.25 --events

# Offline planner replay; confirms normal chords were not weakened.
python3 scripts/evaluate_planner_bags.py \
  --cost-band-clearance 0.25 --continuous-clearance 0.25 <bag>
```

When diagnosing a stall, the two fields that have identified every bug so far
are `progress=` in `/planner/follower/status` (does it climb?) and the adapter's
reason string in `/planner/flight/status`.

## Pitfalls this work has already hit

* Do not compare a **filtered** signal against a reference taken from that same
  filtered signal — the filter's own convergence reads as real motion.
* Do not gate a time-driven state machine behind an unrelated early return; it
  starves the call that advances it.
* Do not edge-trigger a destructive action on a threshold the signal sits on.
* Clearance rules live in **two** places: the follower and the adapter's
  post-limiter gate. Changing one alone does nothing in flight.
* Measure a mid-flight clearance against the grid current *then*, not the final
  grid — worth about 7 cm on the one event that mattered.
* Do not perform an armed test to validate a change. Props-off `auto_arm:=false`
  first, then a short flight with RC kill ready.
