# ROS migration versus voxel-perception-tuning — September 15

## Finding

The ROS migration did **not** include the working 3D tuning branch. The local
`voxel-perception-tuning` tip is `5c2dbbb`, including `ac6d227` (real corridor),
`65184d9` (bounded observation cache), and `c415dea` (carving and continuity).
Those commits are absent from `ros-rtabmap-migration`. This is separate from
whether the flown 2D planner includes main's changes.

The latest yaw test is `global_planner_3d_monitor_20260915T053921Z`. Its failure
is not explained by a slow A* search or a failure to record yaw. The drone
rotated through about **76 degrees**, but the full 35 cm clearance envelope
could reach only 8–13 connected native grid centers. Accepted paths were
6.5–13 cm long. Exact sphere-to-voxel checks, with even more permissive neighbor
connectivity than the planner, found the same connected components at four
sampled generations. Switching inflation algorithms cannot bridge this gap.

The earlier explanation that the user simply needed more coverage was
incomplete: the previous working setup also asserted a **1 m clear start
region**, used much denser depth rays and optionally assumed missing depth was
free. A 50 cm sphere is not equivalent to 50 cm of travel for a 35 cm-radius
vehicle. The whole vehicle envelope must remain inside known/assumed free
space, and the 5 cm voxel boundaries further reduce the usable movement.

## Branch comparison

These are the old branch's **documented launch command settings**, not all its
factory defaults. Source: that branch's README and HANDOFF_3D_NAVIGATION.

| Behavior | Documented voxel branch setup | Current yaw test |
|---|---|---|
| Local-grid depth decimation | 2 | 8 |
| Minimum retained depth | 0.80 m | 0.30 m |
| Dense frustum fill | Explicitly enabled | Absent; measured depth rays only |
| Clear start region | 1.0 m; unknown only, anchored at startup | 0.5 m test override; includes occupied, re-anchors on clicks |
| Planning height | Fixed startup-pose Z + 0.30 m; clicked Z ignored | Actual body pose, clicked Z preserved; clicks were Z=0 |
| Historical route free-space reuse | Enabled within tracking limits | Current map generation only |
| Clearance | Conservative Chebyshev/cube approximation | Euclidean inflation and exact swept sphere |
| Pose reference | Camera-origin corrected pose | Atomic corrected body pose with camera lever arm |
| Unposed local-grid cache | Pruned to graph | Cleanup was missing; restored in this change |
| Map assembly | Full rebuild, with reuse on some unchanged graphs | Incremental updates and exact stable-prefix checkpoints |

The old frustum code was not simply normal depth ray tracing. Its angular
buffer starts at infinite range where there is no retained return, so such
cells can become free out to the configured 3 m. It preserves occupied cells
and stops behind represented returns. Its own documentation describes the
missing-return assumption explicitly. It cannot establish measured clearance
around surfaces that produce no valid depth. The comparison runs it offline;
it has not been enabled in the current launch.

Historical route reuse cannot solve the first-route failure: it requires an
already accepted corridor. Its old grid mutation also differs from the current
same-generation map audit contract, so it has not been transplanted blindly.

## Controlled test on the new bag

The bag contains compressed RGB/depth and calibration inside MapData. Extracted
all 32 MapData messages and rebuilt generation 26 (11 contributing views,
after the third click) using the old branch's actual OctoMap assembler and
frustum-fill code. Also regenerated local grids from the recorded images using
RTAB-Map LocalGridMaker with the old tuning options.

Held the current body start, geometry, full 3D planner, 35 cm radius, and map
bounds constant. The goal XY is the actual third click; the table uses its Z
at current body height (0.334 m) to separate coverage from the Z=0 click issue.
The 1 m cases change only the local diagnostic grid: they assert unknown as
free while preserving occupied cells. They are **not live configuration changes
or evidence of physical clearance**. The .5 m cases reproduce the current
occupied-and-unknown test override. Both are centered at the current third-click
body anchor for this controlled comparison, rather than reproducing the old
startup-camera anchor or old planar follower.

| Perception variant | Path with 0.5 m clearing | Path with 1 m assertion |
|---|---:|---:|
| Recorded depth grids | 0.097 m | 0.998 m |
| Recorded grids + old frustum fill | 0.097 m | 1.642 m |
| Decimation 8, range minimum 0.80 m | 0.097 m | 0.998 m |
| Decimation 4, range minimum 0.80 m | 0.097 m | 1.485 m |
| Decimation 2, range minimum 0.80 m | 0.127 m | 1.932 m |
| Decimation 2, range minimum 0.80 m + old fill | 0.127 m | 1.932 m |

**All paths above remain partial; none reaches the actual clicked waypoint.**
With the real clicked Z=0, the dense-decimation-2/1 m case produces 1.832 m
instead. The 0.5 m cases remain short with either height.

Regenerating the original decimation-8/range-0.30 grids reproduced the recorded
grid-cell count (166,010 free input cells across the 11 views) and the same
planning results as loading those recorded grids. This checks the local-grid
regeneration setup. These counts sum input observations, not unique global
free voxels. Decimation 4 produced 350,777 and decimation 2 produced 554,115.

This isolates the largest difference: the old 1 m assertion bridged the
near-camera coverage gap. Denser observed rays then helped beyond it. Raising
minimum depth alone did not cure this recording, despite being important in
the old branch's historical hardware tests. Adding frustum fill at the current
50 cm limit did not cure it either.

## Performance evidence and changes

Latest **live bag**, after the first map:

- System CPU median 56.45%, maximum 65.2% across four cores; no throttle flags.
- OctoMap process median 5.8% of one core.
- OctoMap processing median 48.2 ms, maximum 211.1 ms.
- Map arrival age median 317 ms, maximum 1,007 ms; median map interval 1.04 s.
- No planner MAP_STALE event after the first map. Startup accounts for the
  recorded MAP_STALE messages. Accepted-search timing was 4.3–13.2 ms; this is
  search timing, not the entire crop/inflation/node callback.

In the isolated reconstruction, generating all 11 local grids took 294 ms at
step 8/range .30, 713 ms at step 4/range .80, and 1,732 ms at step 2/range .80.
Old full-map assembly took 79–249 ms without fill, versus 259–394 ms with it.
These single offline samples establish a cost tradeoff, not live CPU savings
or the runtime cost of the incremental producer. A denser default is not free.

Restored the old cache-cleanup behavior in the producer and also bounded the
new incremental assembler's separate cache. Both retain only grids in the
current optimized graph, after processing its transition. Retrieved nodes
can repopulate their grids; checkpoints retain their own tree evidence.
Metadata now records `observation_cache_nodes` and `assembler_cache_nodes`,
so future bags can demonstrate that the caches track active graph size.

Restored configurable depth decimation and noise/cluster filters through the
ROS launch chain, with explicit forwarding of range and mapping rate. Existing
defaults and the requested 50 cm test limit remain unchanged. For a measured
ray-only tuning experiment, these arguments now reach the actual mapper:

```text
range_min:=0.80 grid_depth_decimation:=4
```

They improve the available tuning controls; they do **not** alone bridge the
start gap. The alternate 0.80 m filter rejects nearer measurements, so it is a
comparison setting rather than an automatic general replacement.

## Verification and artifacts

Fresh verification:

- `colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1`: passed.
- Focused launch construction/forwarding checks: 11 passed. OctoMap core checks: 14 passed.
- `colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+`
  and `colcon test-result --verbose`: **740 tests, zero errors/failures, three intentional skips**.
- Compiled the pre-change assembler from `f1f564b` and the fixed assembler; fed
  all 32 recorded MapData updates through each. **Every serialized binary map
  is identical.** Both caches finish at 15 active grids instead of 32 retained
  historical grids; during the stationary segment they retain 2 instead of 8.
  Checkpoint counts also match (16 restores, 2 full builds).
- The actual live recording audits 32 maps, 17 paths and 180 follower chords
  with **zero errors**. Its artificial clearing means physical clearance is
  explicitly not validated.
- `git diff --check`: passed.

No live camera,
PX4 connection, arming or flight commands were run. No workers were used:
architecture, branch comparison and the tightly coupled map-cache change were
handled locally.

Artifacts: `/home/john/planner3d_waypoint_20260915T053921Z/`.
`branch_comparison/results_g26.json` contains all 40 controlled planning cases;
`compare.cpp`, `old_octomap.cpp`, the exported old header, `build.py`, and the
32 `.cdr` inputs make the experiment reviewable. `comparison.png` plots the
main cases. `clearance_probes.json` and `exact_components.json` document the
original start-pocket geometry; `audit.json` audits the actual live recording.

The next physical intervention should target the near-camera **volume**,
including above/beside the drone and the start-to-corridor connection. Yaw
alone did not establish that volume in this bag. Restoring the old 1 m
assumption would require actually verifying that larger region is clear;
it has not been silently substituted for the user's requested 50 cm radius.
