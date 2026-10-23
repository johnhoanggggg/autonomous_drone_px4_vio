# 3D test-start clearing and performance — September 15, 2026

Later waypoint tests exposed a configured-boundary issue and startup-only
anchoring limitations. The [new waypoint diagnosis](WAYPOINT_START_DIAGNOSIS_20260915.md)
supersedes the anchoring and goal-completion behavior described here; the
performance measurements below remain evidence for their recorded revision.

## Implemented

- Optional `test_start_clear_radius:=0.5` creates an artificial free sphere at
  the first valid **body** pose. The producer transforms that fixed raw anchor
  with each map's own correction. It changes a publication copy and records
  the override in map metadata and `/planner3d/test_start_clear` markers.
  Default is disabled; radius above 0.5 m is rejected. Altitude bounds remain
  enforced. The audit explicitly separates artificial map clearance from
  physical clearance.
- Swept-sphere collision checks now compute segment-to-voxel-box distance,
  replacing an overly conservative expanded-box approximation. A separable
  Euclidean distance transform supplies conservative inflation distances.
  Unknown and occupied voxels remain blocked. Tests cover the actual 5 cm
  voxel resolution, a 35 cm envelope inside the 50 cm sphere, tangencies,
  diagonal/interior contacts, and a brute-force distance oracle.
- The OctoMap assembler keeps an exact stable-prefix checkpoint. Removing or
  replacing the newest graph node restores that prefix and inserts current
  evidence. Changed stable poses, removed stable nodes and older-node retrieval
  fall back to rebuilding. A rebuild also seeds a corrected prefix checkpoint,
  including after previously tolerated small pose changes.
- Tree snapshots allocate typed RTAB-Map nodes recursively; the inherited
  upstream copy constructor slices children and cannot safely preserve custom
  node state. Snapshots preserve topology, log odds, colors, occupancy types,
  observation references, model thresholds and bounds. Unit comparisons use
  full leaf state, not just binary occupied/free classifications.
- New bags record machine/per-core and process-group CPU, PIDs, memory, threads,
  temperature, load and throttling. `process_monitor:=true` is the default.
  The profile covers both composable containers, OctoMap, pose adapter,
  planner/follower, recorder and Foxglove.

Valid depth rays already carve measured free space up to an occupied endpoint.
Missing depth, occlusion and space beyond a surface remain unknown; the whole
camera frustum is not cleared. See the [OctoMap ray insertion API](https://octomap.github.io/octomap/doc/classoctomap_1_1OccupancyOcTreeBase.html).

## Wider-room A/B replay

Source: `ros2_ws/flight_logs/global_planner_3d_monitor_20260915T035326Z`.
Sequential 1x MapData/raw-odometry replays on isolated ROS domain 192, with
recorded goals and 5,000 map marker voxels. The baseline disables checkpoints;
`optimized_v2` enables the final implementation. No camera or PX4 input
publishers ran. Both use the same planner/telemetry code.

| Metric | Rebuild baseline | Optimized |
|---|---:|---:|
| Late map processing median, generations 30–40 | 343.9 ms | 85.1 ms |
| Late map processing p95 / max | 404.7 / 455.8 ms | 236.5 / 380.8 ms |
| Late assembly median | 302.6 ms | 57.9 ms |
| Late MapData-to-OctoMap arrival delta median | 347.1 ms | 86.3 ms |
| Late OctoMap process CPU median | 35.1% | 10.5% |
| Full cache assemblies, whole run | 21 | 6 |
| Checkpoint restores, whole run | 0 | 15 |
| Whole-run map processing median | 67.9 ms | 79.4 ms |
| Whole-run map processing p95 | 352.8 ms | 299.1 ms |

The late bottleneck improves by about **75% / 4x**. Capturing checkpoints adds
work during ordinary incremental mapping, hence the higher whole-run median.
The first required late rebuild remains expensive; this does not eliminate
all latency spikes. All **40 binary OctoMaps are byte-identical** across the
baseline and optimized runs, with identical stamps, bounds, source counts and
map corrections. The larger recorded goal remains blocked without an override.

CPU percentages for a process use **100% = one full core**. The CPU numbers
above are new replay measurements, **not historical live CPU usage**. The
original wider-room bag contains no `/perf/*` topics. Camera/mapper/Foxglove
process counts are zero in these MapData-only replays, as expected; active
navigation and recorder groups each have one process. Throttle word was zero.
Whole-machine median CPU was 17.6% versus 20.1%, and p95 was 39.4% versus 29.5%;
these include other host processes and are not a controlled full-stack CPU test.
The previously observed raw-odometry age and separate TF/input synchronization
spike still need live timing evidence after this change.

Artifacts: `/home/john/planner3d_start_perf_20260915/` contains baseline,
initial optimization diagnostic, final `optimized_v2`, and test-clear replay
bags, logs, samples, reports and `comparison.json`. `compare.py` computes the
same-source-stamp binary hashes, stage percentiles, arrival deltas and CPU
summaries. The initial diagnostic is retained because it exposed the missing
checkpoint-seeding case; it is not the final performance result.

## Test-clear route audit and final verification

`clear_start_stationary` reuses all 40 recorded MapData updates and holds the
first raw pose with fresh timestamps (`--hold-start-pose`). This is a controlled
startup diagnostic, **not a reconstruction of the actual carry trajectory**.
It produced 70 accepted path snapshots and 422 follower chord checks (577 total
segments); the exact-map audit passed with zero errors and
`physical_clearance_valid=false`. Later `CORRECTION_LIMIT` / goal-envelope
rejections remain enforced while the synthetic body stays fixed and the map
correction changes. The test establishes startup planning, not tracking a route.

All 40 maps retain one raw anchor and correctly transform its center as the map
correction changes. The recorded bag contains 40 test-sphere markers and 69
messages on each of the 22 CPU/health topics. Late map processing with the
publication copy and clearing enabled was 103.6 ms median; the copy/clear stage
accounted for 19.1 ms median.

A separate real ROS integration regression verifies a map stamp slightly older
than the first valid raw pose is accepted, and that subsequent body motion
cannot move the raw anchor while a map correction moves its world center.
The original strict stamp ordering unnecessarily discarded the first map.
In the source bag that map stamp precedes the first raw pose by 33 ms and arrives
about 1.07 s after that pose's sensor time; the recorded drone has already moved
about 12 cm by the first available planner body snapshot.

Final build succeeded. All **38 CTest targets / 739 tests** passed, with zero
errors/failures and three intentional parity skips. Verification commands:

```bash
# Repository root: final producer build, using the existing workspace outputs.
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source ros2_ws/install/setup.bash
colcon build --base-paths ros2_ws/src --build-base ros2_ws/build \
  --install-base ros2_ws/install --packages-select px4_vio_bridge \
  --executor sequential --parallel-workers 1
cd ros2_ws
colcon test --packages-select px4_vio_bridge --executor sequential \
  --event-handlers console_direct+
colcon test-result --verbose
cd ..
python3 -m py_compile scripts/replay_ros_planner_3d.py scripts/audit_planner_3d_bag.py
git diff --check
```

Replay commands, from the repository root after sourcing:

```bash
python3 scripts/replay_ros_planner_3d.py \
  ros2_ws/flight_logs/global_planner_3d_monitor_20260915T035326Z \
  --out /path/to/new-baseline --domain 192 --mapdata \
  --disable-checkpoints --recorded-goals
# Repeat with another new output and omit --disable-checkpoints for comparison.
python3 scripts/replay_ros_planner_3d.py \
  ros2_ws/flight_logs/global_planner_3d_monitor_20260915T035326Z \
  --out /path/to/new-stationary-check --domain 192 --mapdata \
  --test-start-clear-radius 0.5 --goal-offset 0.05 --hold-start-pose
python3 scripts/audit_planner_3d_bag.py /path/to/new-stationary-check/bag \
  --out /path/to/new-stationary-check/audit.json
```

## Original carry motion with the final code

`clear_start_motion_final` replays the original 628 raw odometry messages,
all 40 MapData messages, the 50 cm override, and a diagnostic 5 cm goal. The
producer now retains the first map, but this recording still produces no
accepted paths or follower chords: 12 `RECOVERY_REQUIRED` and 67
`START_BLOCKED` planner states. Its audit is incomplete/false with no clearance
violations reported; absence of routes is not a pass.

The body has moved about 12 cm from the first raw anchor before the first map
can be used, and moves farther away during the carry. A 35 cm envelope inside
a 50 cm sphere has at most 15 cm of center travel before voxel discretization
and conservative inflation. The override deliberately remains fixed at the
initial point. The stationary-start diagnostic above confirms that startup
planning works when the body remains there; it does not convert this old carry
into a successful route test. The next live test must hold still until the
initial map/pose is available and a nearby route is accepted, then move into
observed free space. No hardware intervention was performed during this work.

## Run the next observation test

See the command and test-boundary explanation at the top of
[HANDOFF_3D_NAVIGATION.md](HANDOFF_3D_NAVIGATION.md). The 50 cm sphere gives a
35 cm envelope at most roughly 15 cm of center motion, less with voxel
quantization. Start with a nearby goal before moving away from the initial
anchor, then extend into observed free space. It cannot clear an entire route
or validate a physical floor boundary for a drone set on the ground.

Three focused workers handled CPU wiring, sphere geometry and the initial
checkpoint implementation; the lead reviewed/integrated those changes, fixed
copying and checkpoint seeding, and ran package checks and sequential replays.
