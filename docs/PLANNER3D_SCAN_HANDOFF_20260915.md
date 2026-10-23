# 3D planner: guided coverage test and goal height

## Current behavior: level-camera exploration route

See [the level-camera planner update](PLANNER3D_LEVEL_CAMERA_20260915.md).
The sphere stays at 25 cm plus 10 cm margin. Up/down scan instructions are
withdrawn. The monitor now proposes a penalized-unknown global route and
keeps a separate verified local path for the follower. The update describes
the new topics, map-sampling optimization and current verification.

## Newer test and fixes

The [06:34:47 bag report](PLANNER3D_LATEST_BAG_20260915.md) supersedes the
verification and diagnosis below. The confirmed drone radius is 25 cm; the
35 cm clearance envelope remains. The planner now handles a clear off-center
body and reuses preparation within a map generation. Follow that report's
next-intervention notes with the launch instructions below.

## What the additional offline work established

For the 05:39 yaw bag, more aggressive processing cannot fill the missing
near-camera coverage with measured evidence. At map generation 26, projected
all 26 recorded camera views through their recorded calibration/extrinsics.
Granted free space to every native voxel that could intersect any view—even
ignoring occupied returns, occlusion and maximum range. Preserved any existing
free space outside those views and the requested 50 cm test clearing.

Even this deliberately optimistic, nonphysical map has only **13 connected
5 cm grid centers** that fit the 35 cm envelope. The real map has 11. Exact
sphere/voxel tests and permissive 26-neighbor connectivity were used for this
upper-bound diagnostic. The component spans X=-0.125..0.025 m, compared with
the body anchor X=-0.0565 m. This cannot form a route to the clicked X=2.015 m.

The blocking area includes space above and beside the nearby camera view.
For example, the entire native voxel centered at (0.225, 0.275, 0.575) lies
outside every recorded view. The center of the forward blocker at
(0.375, 0.125, 0.475) was never in an image; only an edge could touch one view.
The rotations of the active graph poses match the corresponding recorded
camera poses to quaternion numerical precision; their translations match.

This is stronger evidence than simply noticing unknown cells. It explains why
decimation changes and the old frustum filler did not bridge the 50 cm start
region. It applies to this recording and the current native grid; it is not a
claim that another physical scan cannot work.

Artifacts:
`/home/john/planner3d_waypoint_20260915T053921Z/camera_visibility.json`,
`camera_view_planes.json`, `visibility_upper_bound.json`, and their scripts.
The optimistic map is diagnostic only and is never published to the planner.

## Implemented changes

### Visible clearance probe

On partial or failed routes, `/planner3d/markers` now includes:

- **Purple cubes:** actual unknown native voxels intersecting the probe sphere.
- **Orange sphere:** the required drone envelope at a point 25 cm toward the
  resolved waypoint (or at the goal if closer).
- A label giving unknown and occupied counts. The existing red obstacle
  markers continue to show occupied observations.

The probe is explicitly labelled **not a route**. It does not clear cells,
change accepted paths, or grant motion. Probe markers expire after two seconds
without refresh. Counts include all intersecting voxel boxes; displayed purple
cubes respect `max_marker_voxels`.

Planner status records `forward_probe` (center, radius, step, crop-boundary
flag and blocker counts), `goal_remaining_m`, `goal_progress_m`, and
`short_partial_path`. A 10 cm fragment can now be distinguished from reaching
a distant requested goal without estimating distance from the display.

### Path shape at actual endpoints

The completed path is now simplified after attaching the actual body position
and exact waypoint. This removes redundant voxel-center kinks when a direct
swept-sphere chord is clear. One- and two-point simplification inputs also
undergo clearance checks; they no longer bypass validation. This improves the
shape of partial paths without changing the reachable goal selection.

### Explicit current-height clicks

`goal_height_mode:=current` resolves a click's XY at the current valid body Z.
That Z is latched at the click and remains fixed if the drone subsequently
moves vertically. The route search remains fully 3D. It is not the old branch's
projection of the start onto a hypothetical fixed-height flight plane.

The existing default is `goal_height_mode:=clicked`: XYZ is preserved exactly.
Status records `clicked_goal`, resolved `requested_goal`, and the selected
mode. Current-height mode rejects a click with `GOAL_HEIGHT_POSE_UNAVAILABLE`
when there is no fresh pose paired with the current map. It clears the old goal
rather than guessing a height or retaining an unrelated route.

The offline replay tool supports `--goal-height-mode` and
`--max-marker-voxels` and records these selections in its report.

## Next carry test

A new physical view is now the necessary input. Use the observation-only
monitor; keep the 50 cm clearing and the previously tested altitude band:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source install/setup.bash
export ROS_DOMAIN_ID=42
ros2 launch px4_vio_bridge ros_planner_3d_monitor.launch.py \
  start_backend:=true foxglove:=true record_bag:=true \
  test_start_clear_radius:=0.5 min_z:=-0.8 max_z:=1.6 \
  goal_height_mode:=current
```

The up/down scan procedure previously listed here is withdrawn. Use a level
camera. See [the level-camera planner update](PLANNER3D_LEVEL_CAMERA_20260915.md):
the amber reference route proposes where to go through incomplete mapping;
the existing path and follower topics contain only the verified prefix.


## Verification

Fresh checks on the final code:

- `colcon build --packages-select px4_vio_bridge --executor sequential --parallel-workers 1`: passed.
- Focused grid geometry tests: 19 passed; ROS integration checks cover default
  clicked height, current-height latching across vertical movement, stale-pose
  rejection, and the existing navigation faults.
- `colcon test --packages-select px4_vio_bridge --executor sequential --event-handlers console_direct+`
  and `colcon test-result --verbose`: **743 tests, zero errors/failures, three intentional skips**.
- Final recorded-goal replay in isolated domain 192, with current-height input,
  50 cm test clearing, actual recorded motion and visible probe markers:
  **32 maps, 15 paths, 188 follower chords, zero audit errors**. Artificial
  clearance remains explicitly excluded from physical-clearance validation.
- All 15 published nonempty paths in that replay are now straight two-point
  segments. They remain partial. The final accepted segment is 4.36 cm long,
  makes 4.13 cm of progress toward the goal, and ends 2.052 m from it. Its
  forward probe contains 126 unknown cells and zero occupied cells.
- Checked all 54 final probe marker arrays against their diagnostic counts;
  all 16,724 displayed unknown points intersect their displayed spheres.
  Frames, sphere diameter and two-second marker lifetimes also pass.
- `git diff --check` and Python compilation checks: passed.

Final evidence lives in
`/home/john/planner3d_waypoint_20260915T053921Z/replay_scan_final/`, including
`audit.json`, `verification.json`, `marker_validation.json`, the bag and logs.
An earlier replay checked the new height/diagnostic features before the final
path-shape change; the `replay_scan_final` results supersede it.
No live hardware graph or PX4 publisher was started during this work.
