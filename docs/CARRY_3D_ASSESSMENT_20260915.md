# September 15 — three 3D carries: clearance rejection and map lag

## Follow-up implementation

The source-bag findings below remain historical measurements. The map rebuild
bottleneck now has a measured offline fix, CPU recording is enabled, and an
optional 50 cm artificial start sphere has been implemented. See the
[follow-up performance and test report](PLANNER3D_START_PERF_20260915.md) and
[current launch instructions](HANDOFF_3D_NAVIGATION.md).

## Findings

The larger-space recording started correctly, tracked motion, built 40 OctoMaps,
and rejected all 13 planning attempts with `START_BLOCKED`. No route or follower
chord was accepted. Lag is measurable, but it did not cause the route rejection.

The user confirmed the drone was set down before clicking the goal. At that
time its body pose was approximately `(0.20, -0.16, 0.025)` m in `world`.
The nearest occupied voxel box was about **0.194 m** away and the nearest unknown
voxel box about **0.089 m** away. Both intersect the required **0.35 m** sphere.
The body-center voxel itself was free. All 13 sampled rejected starts had both
occupied and unknown voxels intersecting the sphere. The clicked goal was
`(1.4917, -0.4502, 0.0)`; a Foxglove ground-plane click does not select a carrying
height automatically.

The set-down is not the whole issue. A sweep through the earlier carry at roughly
0.5-second intervals checked 77 body poses against their matching map generations.
None had a fully observed, unobstructed 0.35 m sphere. At the best sampled position,
occupied space was 0.446 m away, but unknown space was only 0.191 m away, with
35 unknown voxel boxes intersecting the sphere. This proves a mapped clearance
failure, not that all of those unknown cells contain physical obstacles.
Camera coverage, depth holes, map reconstruction and actual surrounding geometry
must be distinguished before changing map generation. Reducing clearance or
marking unknown cells free would not establish flight safety.

## Timing in the larger-space bag

Age means bag receive time minus sensor/header timestamp on this Pi. It includes
processing and delivery; it is not a direct measurement of CPU time, network
delay to the browser, or browser rendering.

| Signal | Rate | Median age | 95th percentile age | Maximum age |
|---|---:|---:|---:|---:|
| Raw odometry | 15.05 Hz | 140.9 ms | 186.7 ms | 273.8 ms |
| Body pose snapshot | 15.12 Hz | 142.1 ms | 187.2 ms | 274.2 ms |
| RTAB-Map MapData | 0.97 Hz | 295.1 ms | 625.2 ms | 1169.2 ms |
| OctoMap | 0.96 Hz | 393.5 ms | 1043.7 ms | 1579.1 ms |

Matched raw-odometry-to-body-snapshot arrival delta was median **1.16 ms**, p95
**3.41 ms**. The pose adapter is not a material latency source here.
Planner status arrived at **1.99 Hz** and follower status at **19.97 Hz**.
Their maximum gaps were **724 ms** and **280 ms**, respectively. These are
publication gaps, not instrumented planning execution durations. The follower
published invalid/zero output because it had no route.

After the first valid body snapshot, planner states through the last snapshot
were 67 `WAITING_FOR_GOAL`, one `POSE_MAP_MISMATCH`, and 13 `START_BLOCKED`.
All 44 planner `MAP_STALE` statuses occurred during startup. There were no
active-run `MAP_STALE` or `POSE_STALE` planner rejections in this third recording.
Startup took about 22 seconds to produce the first map/body snapshot; startup
warnings must not be counted as tracking loss during the carry.

### Two sources of map delay

1. **Repeated OctoMap rebuilds after stopping.** Recorded MapData graphs retain
   nodes `[1,2,8,...,28]`, then successively replace the latest node:
   node 29 disappears when 30 arrives, 30 disappears when 31 arrives, and so on.
   The map correction is unchanged over this interval. The adapter deliberately
   resets the OctoMap when an assembled node disappears, so removed observations
   cannot remain in the map. This is the removal check in
   `RtabmapOctomapAssembler::update()` in `src/rtabmap_octomap.cpp`.
   Matched MapData-to-OctoMap arrival delta is median **57 ms** for generations
   1–28, then **388 ms** for generations 30–40 (range approximately 374–438 ms).
   Generation 29, before the replacement cycle begins, still takes only 75 ms.
   The recorded graph transitions and code identify repeated reconstruction as
   a concrete performance issue. The arrival delta also contains scheduling,
   conversion, serialization and marker work; it is not pure rebuild CPU time.
2. **A mapper input/TF synchronization spike near the click.** RTAB-Map cycles
   33 and 34 are about two seconds apart. A missing odometry-TF warning occurs
   between them. Cycle 34 reports only 113.5 ms of RTAB-Map compute but 1169 ms
   total delay. This supports an input/TF waiting component; the logs do not
   expose enough internal timestamps to assign the remaining delay exactly.
   The additional OctoMap work then raises the published map age to 1579 ms.

The odometry log has no explicit tracking-loss events. Excluding the final
shutdown sample, logged processing time is median 65.3 ms, p95 83.0 ms; reported
delay is median 141.3 ms, p95 186.9 ms. Odometry got somewhat slower after the goal:
median recorded age 170 ms versus 152 ms in the five seconds immediately before
the click. This correlation does not isolate planner CPU cost.

Recorded combined OctoMap markers average about 0.46 Mbit/s over the full bag;
the separate ground/obstacle topics duplicate that geometry if both combined
and separate topics are subscribed. This recording does not measure client
network queues or render time, so it cannot rule out additional Foxglove lag.

## Comparison and acceptance result

| Bag start (UTC) | Maps | Valid body snapshots | Rejected planning attempts | Start-center states at those attempts |
|---|---:|---:|---:|---|
| 02:49:16 | 92 | 1408 | 33 | 28 free, 5 unknown |
| 02:52:15 | 54 | 797 | 56 | 56 unknown |
| 03:53:26 | 40 | 617 | 13 | 13 free |

Every tested rejected start in all three bags intersected unknown space within
the 0.35 m sphere. The second attempt also had real mid-run pose/map stalls;
the third did not. A larger room improved observed space but did not satisfy
the entire body-clearance requirement.

The existing route auditor read all 40 maps from the third bag and found no
accepted paths or chords. It returns **`valid: false`**, with no recorded
clearance violations, because there was nothing accepted to validate. This is
not a passing navigation test.

## Evidence and reproduction

Original bags are under `ros2_ws/flight_logs/`, with names
`global_planner_3d_monitor_20260915T024916Z`, `...T025215Z`, and `...T035326Z`.
The third bag spans 62.79 seconds, including 40.74 seconds between first and
last valid body snapshots. The recorder reports flushing its cache and stopping;
the outer launch reports recorder exit code 2 during Ctrl-C. Sequential reading
and metadata are available, but do not infer a clean exit code from readable data.
The MCAP reader warns that no receive-time index is present.

Analysis artifacts are preserved in
`/home/john/planner3d_carry_analysis_20260915/`:

- `carry_timeline.png`, `clearance_slices.png`: inspected timing and geometry plots.
- Per-bag `timings.json`, `poses.json`, `events.json`, `start_probes.json`,
  `trajectory_probes.json`, and `slices.json`.
- Third bag directory: `audit.json` from the production offline path auditor.
- `timings.py`, `probe.cpp`, `probe_bags.py`, `plot.py`: analysis sources.
  The probe deserializes the original binary OctoMap at native resolution and
  computes Euclidean distance from the body point to each voxel box. Counts
  refer to voxel boxes intersecting a sphere, not voxel centers. Its local
  search extent is 0.65 m per axis; a 999 distance denotes no cell of that state
  found in that crop, not measured clearance to infinity. Pose/map pairing uses
  generation; rejected-start probes use the latest previously recorded body
  pose of that generation, since the old planner status omits its actual pose.
  Thus these are close recorded reconstructions, not direct internal planner
  callback timestamps. The 9 cm and 19 cm blockers are well inside the envelope.
- `plot_venv/` isolates plotting dependencies from the ROS environment.

From the repository root, source Jazzy and `ros2_ws/install/setup.bash`, then:

```bash
python3 /home/john/planner3d_carry_analysis_20260915/timings.py
python3 /home/john/planner3d_carry_analysis_20260915/probe_bags.py
python3 scripts/audit_planner_3d_bag.py \
  ros2_ws/flight_logs/global_planner_3d_monitor_20260915T035326Z \
  --out /tmp/planner3d-carry-audit-new.json
```

The audit output must be a new path; it intentionally exits nonzero here.
One read-only worker independently checked odometry and mapper logs. The lead
read all three bags, checked exact map geometry and matching timestamps, ran
the route auditor, and inspected both plots. No live ROS graph, camera or PX4
command was started. Runtime code and clearance settings were not changed for
this assessment; no package rebuild or runtime regression suite was needed.

## Engineering priorities before another acceptance attempt

Expose the actual start clearance, unknown blockers, current body height and
goal height in the monitor; `POSE_VALID` currently certifies only the pose/map
interface, not a clear starting volume. Investigate the repeated latest-node
replacement/rebuild path using this saved MapData, retaining correct removal
of old observations. Separate mapper synchronization delay from reconstruction
time with stage timestamps. A future carry must select the goal while held in
previously observed clear space and use an explicit Z. The unknown patches
remain a real map-coverage acceptance failure even after accounting for setting
the drone down. These bags do not qualify the 3D planner for flight.
