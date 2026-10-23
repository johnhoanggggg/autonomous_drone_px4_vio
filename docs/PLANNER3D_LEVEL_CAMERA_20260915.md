# Level-camera 3D route planning — September 15

## User constraints and behavior

Use a sphere: **0.25 m robot radius + 0.10 m margin = 0.35 m clearance**.
Keep the existing **0.50 m test start sphere**. The user explicitly rejected
up/down camera scans and changing the drone to a different collision shape.
Those scan instructions are withdrawn.

The previous planner tried to find the complete route using only already
verified volume. A level forward camera leaves a near-body blind region, so
that search stopped inside the startup clearing. The replacement separates
a proposed global route from the currently verified local path:

| Output | Meaning |
|---|---|
| `/planner3d/reference_path` | Proposed route through an incomplete map; occupied cells still block it |
| `/planner3d/candidate_path` | Same proposed route, for existing candidate-path displays |
| `/planner3d/reference_snapshot` | Atomic proposal, map/goal generations, verification lengths; `executable=false` |
| `/planner3d/path` and `/planner3d/path_snapshot` | A continuously verified prefix or local frontier route, with correct partial-goal flags |
| `/planner3d/markers` | Amber proposed route and label, existing verified path and blocker markers |

The native OctoMap is unchanged. Unknown cells are allowed **only in the
separate reference search grid**, with a cost penalty on centers whose
envelopes are not verified. Detected occupied cells and the geofence remain
blocked. Weighted A* searches that grid. When the requested goal envelope is
blocked, the reference search first tries the nearest traversable approach
point instead of spending the entire budget flood-filling toward it.

Every segment of the proposed route is then checked against the original
map. The longest clear prefix is found using the same exact swept-sphere
geometry as the follower. Checking stops at the first unknown/occupied
intersection; a later clear section cannot bridge an unverified gap. If that
prefix is very short, the strict planner can retain a verified local detour
that makes better progress toward the goal. This prevents an optimistic
proposal from suppressing useful local exploration. Status records both
`verified_prefix_m` and `verified_local_path_m`, with `local_route_mode`. Partial
prefix endpoints cannot report that the actual waypoint was reached.

**A proposed route is not a verified flight corridor.** The follower explicitly
rejects `executable=false`, including a proposal accidentally remapped onto
the accepted-path topic. This remains an observation-only stack without a
new PX4 connection. In the test configuration, even verified local paths depend
on the user-requested artificial start clearing.

## Follower recovery after map correction

The wider-room recording also exposed an independent follower freeze. Its
stable map-to-odometry correction had **6.15 degrees of pitch**, above the old
5-degree absolute correction limit. The follower rejected every subsequent
path, even though fresh map, body pose and path agreed on that correction.

Atomic snapshot mode now requires the correction to match the map metadata,
as well as matching map generations and timestamps. Each new map clears the
old follower state; a fresh matching path restarts its motion ramp. A stable
coordinate-frame offset no longer causes a permanent `CORRECTION_LIMIT`.
Independent legacy topics retain their absolute correction limits.

Horizontal and vertical speed/acceleration bounds are conservatively reduced
in a tilted map frame so inverse-rotated vectors stay within their configured
continuous-odometry limits. The spherical collision model is unchanged.
Regression tests cover recovery at 6.15 degrees, mismatched-correction rejection,
quaternion sign equivalence, and rotated motion limits.

## Short-path braking

Recording the follower vectors exposed a second issue in the latest bag:
clamping its command point at a short endpoint created **0.712 m/s² vertical
acceleration**, exceeding the configured 0.20 limit. Acceleration limiting ran
before that position clamp, so the clamp undid the bound.

The follower now reserves this tick's movement plus braking distance before
selecting velocity. It integrates that bounded velocity without snapping the
command point to the endpoint. Independent X/Y budgets each use the horizontal
limit divided by sqrt(2), so their combined speed and acceleration remain
bounded while retaining a braking budget on each axis. This is conservative:
straight motion along one map axis can use at most 71% of the horizontal limit.
The sphere and clearance checks are unchanged. A regression follows a short
XYZ endpoint in both directions with varying timer intervals and checks actual
velocity differences, acceleration bounds, convergence and no overshoot.

## Performance

Prepared raw/inflated grids remain cached within each map generation.
The new native-grid sampler expands the OctoMap's leaf volumes directly into
the requested crop, including pruned coarse leaves and negative coordinates.
The planner and follower use it instead of performing a separate tree lookup
for every native voxel. Bounding-box iteration avoids traversing unrelated
map branches. Native resolution and key alignment are required.

The independent bag auditor still samples the tree with individual point
queries. It checks accepted paths/chords against occupied **and unknown**
space, and checks explicitly non-executable proposals against occupied space.
Its report counts unverified proposal segments separately.

`last_plan_timing` on `/planner3d/status` records preparation and route work.
CPU metrics and the new reference topics are included in recordings. The
replay recorder also captures follower velocity, acceleration and validity.
The original wider-room `03:53:26` recording has no CPU topics, so its live
CPU use cannot be recovered. The latest original `06:34:47` recording does:
after mapping starts, machine CPU is median 51.3%, maximum 66.4%, with no
throttling flags. Camera use is about 130% of one core; after the click,
planner use is median 29.9% of one core. These are historical live measurements,
not CPU results for this patch.

## Running

Both 3D monitor launch files enable `exploration_route:=true` by default.
`exploration_route:=false` restores the strict known-volume-only planner.
The replay script defaults to the new behavior; `--strict-route` selects the
old behavior for comparisons. Goal height remains an independent choice.

Restart the monitor from a fresh terminal with:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
  start_backend:=true foxglove:=true record_bag:=true \
  test_start_clear_radius:=0.5 min_z:=-0.8 max_z:=1.6 \
  goal_height_mode:=current exploration_route:=true
```

Use a level camera and ordinary waypoint input. Enable `/planner3d/markers`
or `/planner3d/reference_path` to see the proposed route. `current` still
latches goal height at the click; changing body height afterward does not
move that goal. The status distinguishes the proposed distance from the
verified prefix, including `REFERENCE_ONLY` when no prefix is currently clear.

This is a large change spanning route selection, follower input validation,
map sampling and bag auditing. It was handled locally so those contracts
could be checked together. No hardware graph, camera or PX4 commands were
started during development.

## Fresh verification and remaining limit

Commands, run with the existing ROS/workspace environment:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1
./build/px4_vio_bridge/test_route_follower_3d_cpp
colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+
colcon test-result --verbose
```

Replay command template from the repository root (each output must be new):

```bash
python3 scripts/replay_ros_planner_3d.py "$source_bag" --out "$replay_output" \
  --domain 192 --mapdata --recorded-goals --test-start-clear-radius .5 \
  --min-z=-.8 --max-z=1.6 --goal-height-mode current --max-marker-voxels 5000
python3 scripts/audit_planner_3d_bag.py "$replay_output/bag" --out "$replay_output/audit.json"
```


- Package build passed. Focused planner tests: 24 passed; OctoMap tests: 15
  passed. Follower tests: 7 passed. `colcon test --packages-select px4_vio_bridge --executor sequential
  --event-handlers console_direct+` and `colcon test-result --verbose` report
  **752 tests, zero errors/failures, three intentional skips**.
- New regressions cover the unknown/occupied distinction, verified-prefix
  truncation, blocked-goal projection, rejection of non-executable follower
  inputs, fully verified reference routes, topic recording, and native crop
  equivalence including coarse pruned leaves and negative keys.
- A separate sampler comparison checked **31 recorded maps × 702,768 cells =
  21,785,808 voxel states**, all identical to individual native tree queries.
  Median sampler time was **36.13 ms → 2.73 ms**, approximately 13× faster.
  These are sequential offline timings, not a new live CPU measurement.
Final replays with correction recovery and endpoint braking both installed:

| Source recording | Proposed length | Maximum verified local path | Maps / proposals / paths / follower chords | `FOLLOWING` samples |
|---|---:|---:|---|---:|
| Latest `06:34:47` | 3.064–3.274 m | 0.128 m | 31 / 25 / 14 / 126 | 38 |
| Wider room `03:53:26` | 1.384 m | 0.368 m | 40 / 14 / 14 / 144 | 72 |

Both independent clearance audits pass with zero errors. Neither replay has
`CORRECTION_LIMIT`. They contain 109 and 28 unverified proposal segments,
respectively; those segments remain non-executable.

Recorded continuous-frame vector maxima also pass:

| Recording | Horizontal / vertical speed (m/s) | Horizontal / vertical acceleration (m/s²) |
|---|---|---|
| Latest | 0.0982 / 0.0500 | 0.3000 / 0.2000 |
| Wider room | 0.0810 / 0.0418 | 0.2568 / 0.1742 |
| Configured bounds | 0.1000 / 0.0500 | 0.3000 / 0.2000 |

These measure published vectors during offline replay, not actual aircraft
motion. Invalid/stale inputs still withdraw the output immediately.

The earlier `05:39:21` replay checks route updates across **all three clicks**:
32 maps, 60 proposals, 18 accepted paths, 206 follower chords, zero clearance
errors; proposal lengths 1.17–2.35 m. That run predates the follower correction
and braking fixes; it is retained as route-selection evidence.

Before the follower-only fixes, latest replay preparation took median
**63.7 ms per new map**, versus approximately 134 ms before direct leaf
sampling. Proposal/local work took median 132.8 ms on a new map (including
preparation of the reference grid), or 54.7 ms with prepared grids cached.
This improves the first reference implementation's approximately 233 ms of
proposal work. The separate native sampler benchmark above isolates the
measured optimization; these are not new live CPU measurements.

**None of these recordings verifies the complete proposed corridor.** The follower
still uses the short verified local path and holds when clearance is absent.
A 0.35 m envelope has at most 0.15 m of center movement inside a 0.50 m
artificial sphere before additional observed free space is necessary; voxel
boundaries reduce that slightly. This explains the newest bag's 0.128 m limit.
No full reference route was marked verified and the actual waypoint was not
reported reached. The configured artificial start clearing also means these
audits do not establish physical flight clearance. The change fixes useful
route generation through incomplete mapping; complete autonomous traversal
of the unobserved corridor remains unverified.

Artifacts: `/home/john/planner3d_level_camera_20260915/`:
`verification_summary.json`, `grid_benchmark{.cpp,.jsonl,_summary.json}`,
`route_comparison.png`, and final `replay_latest_final/` and `replay_wider_final/`
with samples, recorded bags, reports and independent audits.
`verification/` contains build/test/replay logs. `summarize_correction.py`
checks the recorded vector bounds. Other replay directories are intermediate
evidence, including `replay_latest_corrected/`, whose recorded vectors exposed
the endpoint acceleration bug before it was fixed.
