# ROS-native RTAB-Map Migration — Results Handoff

Status: investigation and bench work ongoing; **nothing here has flown**. The
hybrid described below is committed and is the recommended next thing to take to
props-off. The full ROS-native stack is committed but is **not** flight-eligible
because the original RGB-camera profile fails to initialise about two runs in
three. The new global-shutter mono profile has now passed the instrumented
startup gate 10/10, but it has not passed the moving-trajectory gate.

Branch: `ros-rtabmap-migration`. Nothing on this branch publishes to `/fmu/in/*`
except the pre-existing `vio_to_px4_odometry`, whose input is unchanged.

## Terminology correction that matters

`RTABMapVIO`, `RTABMapSLAM` and `BasaltVIO` are **`ThreadedHostNode`** — they run
on the Pi CPU, not on the OAK. Only `StereoDepth` and `FeatureTracker` are
`DeviceNode` and execute on the RVC2. Verified from the class hierarchy:

```text
RTABMapVIO      ThreadedHostNode  -> Pi CPU
RTABMapSLAM     ThreadedHostNode  -> Pi CPU
BasaltVIO       ThreadedHostNode  -> Pi CPU
FeatureTracker  DeviceNode        -> RVC2 VPU
StereoDepth     DeviceNode        -> RVC2 VPU
```

Earlier notes in this repo, including parts of the README and several commit
messages, call these "on-device". That is wrong and it misled the CPU analysis
for a while. Each host node gets one dedicated thread and is single-threaded and
compute-bound, which is exactly why `RTABMapVIO` + `RTABMapSLAM` together cost
~200%: two threads, each pinning a core.

## The three architectures

```text
A. As flown today
   VPU: StereoDepth + FeatureTracker
   Pi : RTABMapVIO ---> RTABMapSLAM ---> grid/clouds
                  \--> /rtabmap/vio_pose -> PX4
   plus a second ROS rtabmap_slam in the 3D launch = TWO mappers

B. Hybrid  (committed, recommended)
   VPU: StereoDepth + FeatureTracker
   Pi : RTABMapVIO --> /rtabmap/vio_pose -> PX4
                   \-> RGB-D @3 Hz -> ROS rtabmap_slam -> OctoMap
   RTABMapSLAM deleted. ONE mapper.

C. Full ROS-native  (committed, NOT flight-eligible)
   VPU: StereoDepth (ROS extracts its own visual features on the Pi)
   Pi : depthai_ros_driver_v3 -> rtabmap_odom -> rtabmap_slam -> OctoMap
   No DepthAI host VIO at all. Nothing feeds PX4.
```

## What DepthAI's RTABMapVIO actually does

This is verified from the depthai-core 3.5 source, not inferred from the node
name. `RTABMapVIO.cpp` constructs the regular RTAB-Map odometry core with
`rtabmap::Odometry::create()` and no parameter overrides, so its default is
frame-to-map (`Odom/Strategy=0`). At each image timestamp it supplies:

- the rectified global-shutter CAM_B mono frame;
- CAM_B-aligned metric stereo depth;
- Harris points tracked by the RVC2 `FeatureTracker`; and
- one linearly interpolated accelerometer and gyroscope sample, transformed
  with the calibrated IMU-to-camera extrinsic.

It does not run a separate inertial state estimator, integrate every 200 Hz
sample into position, or estimate velocity and IMU biases. In particular it is
not equivalent to Basalt, OpenVINS or VINS-Fusion. The word VIO here means
RTAB-Map frame-to-map RGB-D odometry supplied with IMU measurements. The
separate `RTABMapSLAM` host node consumes that raw odometry and adds the pose
graph, loop closures and map; SLAM never produces the raw VIO pose.

The full ROS graph is therefore structurally equivalent:

| layer | DepthAI graph | ROS-native graph |
| --- | --- | --- |
| depth | RVC2 `StereoDepth` | RVC2 `StereoDepth` through driver v3 |
| visual input | RVC2 tracked CAM_B Harris points | host descriptors on selected image |
| odometry | host `RTABMapVIO` / F2M | `rtabmap_odom::RGBDOdometry` / F2M |
| raw pose | `/rtabmap/vio_pose` | `/rtabmap/vio_odometry` |
| mapping | host `RTABMapSLAM` | ROS `rtabmap_slam` |
| 3D occupancy | no OctoMap API | `mapData` -> `rtabmap_octomap_node` |

The remaining input mismatch was that the ROS launch used rolling-shutter RGB
while DepthAI RTABMapVIO uses global-shutter CAM_B. The new
`config/oak_v3_rtabmap_mono.yaml` exposes `/oak/left/image_rect` and aligns
depth to CAM_B. `ros_rtabmap_oak_v3_mono.launch.py` selects those inputs and
uses the more accurate 1000-feature / 3000-feature-map F2M settings. An
instrumented ten-run sweep of that exact graph passed 10/10. Each attempt
required live mono, live depth, at least 1% nonzero depth, live IMU, five
non-lost odometry samples and at least 20 median inliers. All ten crossed the
gate in 8.24-8.61 s, with 72-99 median inliers, 43.5-51.0% sampled nonzero
depth, and no launch errors. Startup is now a passed gate for this bench scene;
require a carried loop with no resets before connecting it to PX4. The JSON is
`/tmp/rtabmap_mono_tuned_startup_10.json`; reproduce it with
`scripts/ros_rtabmap_startup_sweep.py`.

For the carried ROS-native run, the raw estimator is `nav_msgs/Odometry`, not
the hybrid's `PoseStamped`. The loop harness now accepts either message type:

```bash
python3 scripts/loop_closure_test.py --label ros-native-mono --seconds 180 \
  --raw-odom-topic /rtabmap/vio_odometry \
  --info-topic /rtabmap/info \
  --out /tmp/loop_ros_native_mono.json
```

Two 30 s stationary controls compared F2M sizes on that mono input. The tuned
profile improved RMS by 30%, increased mean inliers by 65%, and cost about 13
CPU percentage points while holding essentially the same rate:

| mono F2M settings | rate | RMS | net | path | max step | mean/min inliers | container CPU |
| --- | --- | --- | --- | --- | --- | --- | --- |
| 600 / 2000 | 10.0 Hz | 1.04 mm | 1.02 mm | 0.35 m | 4.10 mm | 197 / 169 | 105.4% |
| **1000 / 3000** | **9.5 Hz** | **0.73 mm** | **0.40 mm** | **0.24 m** | **3.49 mm** | **326 / 288** | 118.1% |

Both had zero lost samples and zero reset-sized jumps. The tuned profile's
start/end five-second means differed by 0.1 mm. This is close to the 0.52 mm
same-scene DepthAI reference, but moving parity is still open. Reports:
`/tmp/loop_ros_native_mono_stationary_clean.json` and
`/tmp/loop_ros_native_mono_1000_3000_stationary.json`.

## Measured CPU

Per-process, sampled from `/proc` deltas over 20 s.

| process | A (two mappers) | B (hybrid) |
| --- | --- | --- |
| DepthAI process | 208.5% | 118.9% |
| ROS rtabmap_slam | 12.3% | 12.2% |
| OctoMap converter | 2.2% | 2.3% |
| PX4 odom bridge | 10.1% | 17.1% |
| **total** | **233.1%** | **150.6%** |

C measures ~104% total but see the blocker below.

Note the ROS mapper costs the same 12% in both columns: it was already running.
The hybrid removes duplicated work, it does not relocate cost. The ROS mapper is
cheap partly because it ingests RGB-D at ~2.7 Hz with `Rtabmap/DetectionRate` at
1 Hz, whereas `RTABMapSLAM` ingested every frame at ~20 Hz on the host. Both run
detection at 1 Hz; an earlier claim that the DepthAI node ran keyframes at 20 Hz
was wrong.

## Measured stationary drift

45 s windows, camera untouched, same scene and metric per group. `net` is
start-to-end, `rms` is spread about the mean, `path` is accumulated motion, and
`max step` is the largest consecutive-sample jump. Per-sample *speed* is not a
sound metric on these topics: they deliver in bursts, so timestamps can nearly
coincide and inflate any computed velocity.

| estimator | rate | net | rms | path | max step |
| --- | --- | --- | --- | --- | --- |
| DepthAI RTABMapVIO (flown) | 18.7 Hz | 0.31 mm | 0.58 mm | 0.28 m | 2.7 mm |
| RTABMapVIO + ROS SLAM (hybrid) | 25.3 Hz | 0.69 mm | 0.63 mm | 0.40 m | 3.0 mm |
| DepthAI BasaltVIO | 39.5 Hz | 18.29 mm | 4.69 mm | 0.15 m | 2.5 mm |
| ROS rtabmap_odom, v2 driver | 9.7 Hz | 120.66 mm | 86.15 mm | 27.25 m | 318.7 mm |

Basalt and RTABMapVIO fail differently: Basalt is the smoothest by accumulated
path but walks away 18 mm, while RTABMapVIO is frame-to-map, stays anchored and
does not accumulate. That is why the flight chain hardcodes
`--vio-backend rtabmap`.

The hybrid costs nothing in accuracy (0.63 mm against 0.58 mm) because PX4 is fed
from the raw VIO either way and the SLAM correction is identity while stationary.

## The v2 driver IMU defect

`depthai_ros_driver` 2.12.2 delivers `/oak/imu/data` with stamps 0.4-0.5 s behind
the images (IMU age 546.6 ms mean, 442-889 ms spread; image age 189.4 ms).
RTAB-Map requires an IMU sample newer than each image, so `rtabmap_odom` logs
`The previous image is dropped!` and discards nearly every frame. **This, not
compute, is what capped the ROS-native benchmark at 5.9 Hz.**

Ruled out by measurement, none of which changed the offset:

- `imu.i_batch_report_threshold: 1`, `i_max_batch_reports: 10`
- `imu.i_max_q_size: 1`
- IMU at 100 Hz instead of 200 Hz
- `i_update_ros_base_time_on_ros_msg: true`
- `i_get_base_device_timestamp: true` (merely exposes the raw device clock)
- `i_synced: true` + `i_enable_sync: true` — **makes it worse**: the IMU stops
  publishing entirely (0 messages) and odometry never initialises, the same
  failure as `i_pipeline_type: depth` on v2

It is a major-version split: the ROS driver is depthai-core v2 while the flying
stack uses depthai-core/python 3.5.0, where the same BMI270 feeds host VIO with
no such lag. **Any ROS-side inertial estimator on v2 inherits this**, OpenVINS
included, because they all consume the same topic. Note also that this RTAB-Map
build has no tightly coupled backend compiled in — the library reports
`RTAB-Map is not built with OpenVINS/MSCKF_VIO/VINS/OKVIS/FOVIS support`, leaving
only strategy 0 (frame-to-map) and 1 (frame-to-frame).

## The v3 driver fixes the lag

`depthai_ros_driver_v3` (3.2.1, depthai-core v3):

| | v2 | v3 |
| --- | --- | --- |
| IMU stamp age | 546.6 ms | 9.7 ms |
| IMU minus image stamp | -397 ms | +110 ms (ahead, which is what RTAB-Map needs) |
| IMU-caused frame drops | 901 | 0 |

Three things are load-bearing, each found the hard way:

1. **Bandwidth limits are mandatory.** Without explicit fps/resolution the driver
   runs full resolution at 30 fps, the OAK-D-Lite's USB link starves the *image*
   streams while the IMU keeps flowing, and the log fills with
   `Communication exception - Couldn't read data from stream` (69 in one run).
   With the limits in `config/oak_v3_rtabmap.yaml` that count is 0.
2. **Odometry and mapper must be composable nodes in the driver's own
   `/oak_container`.** As separate processes the images cross a process boundary
   twice over best-effort DDS and odometry runs at 2.95 Hz with 42.2 ms of
   compute and no core saturated; composed, the same configuration reaches
   17.44 Hz at 29.4 ms.
3. **RTAB-Map core parameters are ROS strings.** `-p Vis/CorType:=1` aborts the
   node with `InvalidParameterTypeException`; they must be quoted.

Accuracy, swept against a fresh reference measured in the same position:

| config | rate | rms | net | path | max step | CPU |
| --- | --- | --- | --- | --- | --- | --- |
| DepthAI RTABMapVIO (reference) | 17.2 Hz | 0.52 mm | 0.58 mm | 0.208 m | 1.86 mm | - |
| flow, BA off | 17.0 Hz | 3.34 mm | 6.33 mm | 2.853 m | 13.08 mm | 96.0% |
| descriptor, BA off | 14.3 Hz | 3.79 mm | 7.42 mm | 2.696 m | 13.83 mm | 99.0% |
| descriptor + BA, 600/2000 | 17.0 Hz | 0.47 mm | 0.51 mm | 0.389 m | 1.52 mm | 100.4% |
| descriptor + BA, 1000/3000 | 10.5 Hz | 0.44 mm | 0.46 mm | 0.227 m | 1.52 mm | 92.9% |

`OdomF2M/BundleAdjustment` is the whole story. Descriptor matching on its own is
no better than optical flow; BA is a 7x improvement. BA cannot run with
optical-flow correspondences, which is the only reason `Vis/CorType` moves to 0.

## Historical RGB blocker; mono startup gate now passes

Three consecutive bare launches, same scene, no changes between them:

| attempt | tracking | inliers | matches | time/frame |
| --- | --- | --- | --- | --- |
| 1 | lost 100% | 0 | 13 | 28.1 ms |
| 2 | ok | 205 | 346 | 56.9 ms |
| 3 | lost 100% | 0 | 9 | 27.8 ms |

About one in three initialises. Roughly 500 features are detected either way but
matching collapses to single digits and inliers to zero. The failing runs are
*faster*, which is the signature of odometry receiving no usable depth rather
than of a tuning problem. Two of the three failures logged no device error at
all, so the occasional `Device ... has crashed` (2 of 9 v3 logs; 0 of 5 on the
DepthAI-python path) explains only part of it.

The new startup harness confirmed `/oak/stereo/image_raw` on every mono attempt,
sampled whether it contained nonzero depth, and gated on non-lost odometry. The
mono profile passed 10/10; the RGB numbers below remain the reason RGB is not a
safe default.

An earlier reading that "explicit arguments work and defaults do not" was wrong;
it was a coin flip over a small sample.

## What is in the repo

Commits on `ros-rtabmap-migration`, oldest first:

- `f3ddf86` original ROS-native benchmark (pre-existing)
- `5af8acb` tuned rtabmap_odom, isolated the v2 IMU lag
- `48b2bb6` `--enable-slam false`, the hybrid
- `1d2c1e0` corrected the CPU saving with a per-process breakdown
- `1649d55` parameter and output-topic parity for the ROS mapper
- `87236fe` `cpp_slam_pose_relay`, rebuilds `/rtabmap/pose`
- `5d1bc92` quantified drift across all estimators
- `38a5631` ROS-native stack on the v3 driver
- `46538c5` bundle adjustment closes the accuracy gap

New code: `src/slam_pose_relay.cpp` + `_node.cpp`,
`include/px4_vio_bridge/slam_pose_relay.hpp`, `test/test_slam_pose_relay.cpp`
(8 gtest cases). New launches: `ros_rtabmap_oak_v3.launch.py` and
`ros_rtabmap_oak_v3_mono.launch.py`. New configs:
`oak_ros_rtabmap_20hz.yaml`, `oak_v3_rtabmap.yaml`. 577 tests pass.

`depthai_ros_driver_v3` is **not installed**. It is unpacked at
`/home/john/depthai_v3_overlay` and used via three environment variables (see the
`ros_rtabmap_oak_v3.launch.py` docstring). apt has it as
`ros-jazzy-depthai-ros-driver-v3`, `_v3`-suffixed so it coexists with v2, but
installing also upgrades systemd/udev. Reverting is `rm -rf` the overlay.

## First loop-closure result, 2026-08-30

One walk, both mappers on the same feed, ~7.6 m carried loop.

| | nodes | GlobalClosure |
| --- | --- | --- |
| DepthAI RTABMapSLAM | 35 | **5** |
| ROS rtabmap_slam | 18 | **0** |

The ROS mapper was not blind. It detected at least seven closures and rejected
every one, with reasons in its log:

```text
Rejected loop closure 10 -> 36: Not enough inliers after bundle adjustment 19/20 (matches=21)
Loop closure 49->2 rejected!  graph error ratio 7.30 (abs error 35.6 deg)
Loop closure 50->2 rejected!  graph error ratio 6.58 (abs error 21.7 deg)
   51, 52, 53, 54 likewise, 15-35 deg   [RGBD/OptimizeMaxError = 3.0]
```

One failed by a single inlier against `Vis/MinInliers=20`. The other six were
geometrically inconsistent by 15-35 degrees.

**The cause is VIO tracking loss, not the mapper.** The harness recorded a
**3343 mm single-sample jump** at t=12.4 s and a **620.6 mm return-to-origin
error over a 7.56 m walk**, and `vio_to_px4_odometry` rejected 400+ poses as
`RTABMap reset sentinel`. The VIO reset repeatedly while being carried; the log
shows tracked features dipping to 93/500. Each reset teleports the odom frame,
which is exactly what fails a 3-sigma geometric consistency check, so the
rejections were correct behaviour.

Two consequences, in priority order:

1. **620 mm of return error is disqualifying by itself** - 8% of path, against
   0.1-0.2 mm stationary. This is the drift PX4 would see, and no amount of
   loop closure removes it, because PX4 reads the raw pose.
2. **A 3.3 m position step is a flight-safety problem.** If the VIO resets like
   this in the air, EKF2 receives that step directly.

So the blocker is upstream of the whole ROS-vs-DepthAI question: **the VIO does
not survive hand-carried motion**. Investigate that before tuning any mapper.
Candidates are motion blur at 20 fps, occlusion, and the low feature counts;
`--num-features`, exposure and carrying more slowly are the first things to try.
Do not raise `RGBD/OptimizeMaxError` or lower `Vis/MinInliers` to force closures
through until the trajectory is trustworthy - that would accept geometrically
wrong closures.

Secondary: the ROS mapper built half the nodes (18 against 35) because it gets
RGB-D at 3 Hz. Raising `rgbd_publish_hz` gives it more constraints, at CPU cost.

The harness now reports `vio_resets_and_jumps` directly, so a repeat run makes
this visible without reading launch logs.

## First tuned mono motion result, 2026-08-31

The first mono carry was stopped after 60 s rather than the requested 180 s,
but it exposed a useful distinction. It recorded 446 odometry-status samples at
7.4 Hz, 12 lost updates (2.7%) in three episodes lasting roughly 0.2-1.1 s,
and ten combined global-closure-or-proximity events. The old harness did not
retain which event type fired, so they cannot honestly all be called global
closures.

The reported six 0.4-1.18 m "resets" were three pairs: a jump to identity on
loss and a jump back to the continuing pose on recovery. They came from
`publish_null_when_lost=true`, not an odometry-frame reset. The core default
`Odom/ResetCountdown=0` disables automatic resets, the launch log contains no
automatic-reset message, and each identity sample has a same-stamp
`OdomInfo.lost=true`. Consequently the old jump, path and RMS metrics are
contaminated. The 1.243 m hold error is also not interpretable because the test
was stopped after 60 s, leaving no controlled final 20 s stationary hold. None
of those values should be used as a drift measurement.

The launch now sets `publish_null_when_lost=false` and explicitly pins
`Odom/ResetCountdown=0`. A loss therefore pauses valid pose output while
`OdomInfo` and diagnostics continue to report it. The harness also filters
same-stamp null samples, reports their count, and separates global closures from
proximity detections. This prevents a future PX4 adapter from mistaking an
invalid identity for a pose, but it does **not** make the motion result pass:
an output pause approaching 1.1 s is still disqualifying. Repeat the full
180-second loop with the corrected launch/harness, then address any remaining
loss episodes.

## Open gates before flight

1. **Mono VIO stability under motion.** The first 60 s carry lost tracking for
   2.7% of status updates in three episodes. Its large apparent jumps were null
   identity messages rather than true resets and are now suppressed, but a
   roughly 1.1 s pose-output pause is still not flight-safe. Repeat for the full
   180 s, then eliminate the dropouts.
2. Re-run the loop-closure test once the VIO holds tracking, and confirm the
   ROS mapper accepts closures when the trajectory is consistent. The
   corrected-pose path and `cpp_slam_pose_relay`'s non-identity branch remain
   unexercised outside unit tests.
3. Moving-scene CPU. Every CPU number here is stationary.
4. Map quality against the DepthAI map on a real trajectory.
5. `Grid/3D` differs by design: the DepthAI grid was 2D at 0.03 m for the
   planner, the ROS one is 3D at 0.05 m projected down. Verified to publish, not
   verified equivalent for the planner.
6. Planner and follower have never run against the hybrid.
7. Props-off, then flight.

## Hand-carried loop-closure test

Purpose: fire loop closure at least once and record what happens to the pose PX4
would consume, on **both** stacks so they can be compared. No props.

Both mappers detect loop closures — `RTABMapSLAM` is RTAB-Map, that is its job.
The difference is only in what is observable: the ROS mapper publishes
`rtabmap_msgs/Info`, while the DepthAI node exposes `transform`,
`odomCorrection`, grid and clouds but **no statistics output port**. So the
closure count is compared through the RTAB-Map **database** instead, which both
can write and which `rtabmap-info` reads:

```text
Links:
  Neighbor:          N
  GlobalClosure:     N   <- loop closures
  LocalSpaceClosure: N   <- proximity detections
```

`--slam-database-path` was added to `basalt_rtabmap_slam_ros2` for this; without
it the DepthAI mapper keeps the map in memory and writes nothing. The ROS mapper
already writes `/tmp/rtabmap3d_live.db`.

**Preferred: one walk, both mappers.** With `slam_enable_slam:=true` the 3D
launch runs the DepthAI RTABMapSLAM *and* the ROS rtabmap_slam on the same
camera feed, so both close the same physical loop on identical data. That is a
far better controlled comparison than walking twice, and it removes the
trajectory and lighting differences between runs. It costs ~233% CPU, which is
irrelevant for a hand-carried bench test.

Both terminals need the workspace sourced and `export ROS_DOMAIN_ID=42`, and
the harness path is relative to the repo root, so `cd
/home/john/autonomous_drone_px4_vio` first.

Terminal 1 (launch, leave running; give it ~60 s to come up):

```bash
rm -f /tmp/depthai_loop.db
ros2 launch px4_vio_bridge rtabmap_slam_px4_3d.launch.py foxglove:=false \
  slam_enable_slam:=true slam_database_path:=/tmp/depthai_loop.db
```

Terminal 2 (harness, at the same time — source the workspace and
`export ROS_DOMAIN_ID=42` here too):

```bash
python3 scripts/loop_closure_test.py --label both --seconds 180 \
  --out /tmp/loop_both.json
```

Walk the loop while the harness records. Then stop the launch with **Ctrl-C**,
not `kill -9`, and wait for it to exit so the database is written:

```bash
rtabmap-info /tmp/depthai_loop.db      | grep -E "Neighbor:|Closure:"   # DepthAI
rtabmap-info /tmp/rtabmap3d_live.db    | grep -E "Neighbor:|Closure:"   # ROS
```

In this configuration `/rtabmap/pose` and `/rtabmap/odom_correction` come from
the DepthAI mapper (the relay does not start when `slam_enable_slam` is true),
and `/rtabmap3d/info` carries the ROS mapper's closure events. So the harness
reports the DepthAI corrected pose, and the two databases give both closure
counts.

**Alternative: the same physical loop twice, once per stack.** Use this if you
want each stack measured in its real configuration, in particular the hybrid
with its relay-produced corrected pose. The two stacks cannot run at the same
time — both need the OAK, and the second will fail with
`X_LINK_DEVICE_ALREADY_IN_USE`. Allow ~30 s between them for the camera to
release and re-enumerate on USB.

```bash
# 1. original full DepthAI pipeline - its own RTABMapSLAM closes the loop
rm -f /tmp/depthai_loop.db
ros2 launch px4_vio_bridge rtabmap_slam_px4.launch.py foxglove:=false \
  slam_database_path:=/tmp/depthai_loop.db
python3 scripts/loop_closure_test.py --label depthai --seconds 180 \
  --out /tmp/loop_depthai.json
# stop with SIGINT, not SIGKILL, or the database is not written
rtabmap-info /tmp/depthai_loop.db | grep -E "Neighbor:|Closure:"

# 2. hybrid - the ROS rtabmap_slam closes the loop
ros2 launch px4_vio_bridge rtabmap_slam_px4_3d.launch.py foxglove:=false
python3 scripts/loop_closure_test.py --label hybrid --seconds 180 \
  --out /tmp/loop_hybrid.json
rtabmap-info /tmp/rtabmap3d_live.db | grep -E "Neighbor:|Closure:"
```

Compare across the two: closure counts from the databases, `correction_updates`
and `ever_non_identity` from the harness (these work on both stacks), the
`corrected_pose_jumps` each produced, and the `return_to_origin_error_mm` on the
raw pose, which is the drift PX4 would actually see and which closure cannot
remove.

Stationary control values, so a null result is recognisable: return-to-origin
0.1-0.2 mm, 0 closures, identity correction, largest corrected-pose jump under
about 3 mm, and `rtabmap-info` reporting 0 nodes and 0 links because RTAB-Map
adds no graph node while the camera does not move.

Procedure:

1. Place the drone at a marked start point with a textured view. Mark the
   footprint so it can be returned to the same spot within a centimetre or two.
2. Start the harness. Let it record ~20 s stationary to establish a baseline.
3. Lift and carry the drone slowly around a closed loop of a few metres, keeping
   the camera pointed at textured scenery, and return it to the marked spot in
   the original orientation.
4. Hold still for ~20 s at the end.
5. Stop the harness and read the report.

What it reports and what to look for:

- **return-to-origin error** on the raw VIO pose. This is the drift PX4 would
  see; loop closure cannot fix it because PX4 reads the raw pose by design.
- **loop closures detected** and the magnitude of the map-to-odom correction.
  Zero closures means the test did not exercise anything; repeat with a more
  textured route.
- **corrected-pose jump size** at each closure. This is the reason PX4 is fed the
  raw pose; the number quantifies how bad a jump would have been.
- **lost fraction and inlier counts while moving**, which the stationary tests
  cannot show.
- **CPU under motion** for each process.
