# ROS-native RTAB-Map Migration — Results Handoff

Status: ROS/DepthAI parity is **not established**. The OAK-tracked ROS path
now has a corrected body-to-camera transform, a 1 ms feature/image sync gate,
and a repeatable bag workflow. The September 7 carried bag fails tracking
continuity (39.8% lost updates, 12.61 s longest output gap). ROS accepted ten
global closures, but those do not repair the raw-odometry outages. The recording
has no stationary ending hold, so it is a motion stress test, not a valid
return-to-origin test. The ROS-native launch remains disconnected from PX4.

Branch: `ros-rtabmap-migration`. Nothing on this branch publishes to `/fmu/in/*`
except the pre-existing `vio_to_px4_odometry`, whose input is unchanged.

## Continuation, 2026-09-07: camera frame fix and repeatable bags

The tracked adapter had a concrete input-frame defect:
`cameraModelFromROS(*camera_info)` defaults to an **identity** local transform,
while `OdometryROS` transforms the IMU into `frame_id=oak`. Thus the visual
camera model used optical axes as body axes and omitted the camera lever arm.
Standard ROS RGBDOdometry explicitly looks up body-to-camera TF. The tracked
adapter now does the same, rejects a tuple if its transform is unavailable,
and uses OdometryROS's data callback group and pause state. This fixes the
published body-frame geometry and the camera lever arm. The earlier explanation
that it was already mixing active visual and inertial constraints was too
strong: the raw IMU stream supplies no changing orientation and F2M ignores it
(see the IMU correction below).

A stationary raw-input and original-output bag survived the Pi reboot:
`/home/john/ros_parity_stationary_before_20260907`. It contains 33.4 seconds,
1002 rectified images, 1002 feature messages, 1003 depth images, 6750 IMU
messages, static calibration TF and 935 original odometry/status samples.
The 30-second live window before the fix measured 28.0 Hz, 2.25 mm RMS,
2.76 mm net displacement, 7.26 mm maximum step and zero lost updates. These
are today's scene, not the scene used for the older submillimetre references.
The temporary live report was lost on reboot; the original messages remain
in the bag. `input_audit.json` in that directory records sensor stamp checks.

The new `scripts/ros_rtabmap_bag.py` records raw sensor inputs plus baseline
outputs and validates that every required input is present. Playback uses an
explicit sensor/static-TF allowlist, excluding old odometry, dynamic `/tf`,
mapper outputs and diagnostics so the fresh estimator cannot consume or be
confused with the old result. The launch accepts `start_camera:=false` and
`use_sim_time:=true`, creating the component container without a camera driver.
The recording QoS explicitly retains latched `/tf_static`.

Use the environment block in README (including the compatible v3 messages and
source-built feature driver). From the repository root:

```bash
# Terminal 1: live source, estimator and mapper
ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py foxglove:=false

# Terminal 2: record a fresh bag; existing directories are never overwritten
python3 scripts/ros_rtabmap_bag.py record /home/john/ros_parity_carry_01 --seconds 90
# Wait for sensor subscriptions, hold still 10 s, carry a slow closed loop,
# return to the marked position/orientation, and hold still through the end.
python3 scripts/ros_rtabmap_bag.py inspect /home/john/ros_parity_carry_01

# Stop the live launch with Ctrl-C before replay.
# Terminal 1: estimator only, using the bag's calibration and simulated time
ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py \
  start_camera:=false use_sim_time:=true foxglove:=false

# Terminal 2: sensor inputs only; restart the estimator for each new replay
python3 scripts/ros_rtabmap_bag.py play /home/john/ros_parity_carry_01
```

A slower replay (`--rate 0.5`) can separate compute overruns from tracking
failure, but its wall-clock output rate is not a live-performance benchmark.
The current loop harness uses wall time, so use 1x playback for its existing
metrics. Record a fresh output bag or subscribe to the new odometry/status
while replaying for comparison. Input presence alone does not prove a lossless
capture. The new `depthai_rtabmap_bag.py` can also feed its visual inputs into
the SDK host node (see controlled replay below). Moving robustness and DepthAI
motion/map parity remain open, and the ROS launch still has no PX4 output.

## September 7 motion bag and follow-up results

The original carried capture is `/home/john/ros_parity_carry_20260907_01`
(about 89 seconds, all six required sensor/calibration topics). The original
live report is `/home/john/ros_parity_carry_20260907_01_report.json`.
`validated_report.json` inside the bag recomputes the result with the corrected
hold checks and suppresses return error: the ending position span is **2.497 m**.
The original 1.900 m reported return error must not be used as a drift metric.
Saved `frame_*.png` images show continued camera movement and a different view
at the end. Raw images/tracks continue around 30 Hz during the odometry losses;
`sensor_audit.json` inside the bag contains the
per-second image, depth, feature and tracking measurements.

The live capture reported 447 lost updates out of 1123 (39.8%), four loss
episodes, a 12.61 s longest output gap, and three valid-pose steps above 250 mm
(two immediately after long gaps). ROS mapper Info reported **10 global
closures and 2 proximity detections**. Container plus OctoMap CPU averaged
133.3% over the recording; the recorder's CPU is excluded. The live graph was
stopped after recording. Camera/odometry parameter snapshots timed out after
this carry, but successful snapshots from the **same unchanged launch** are
in `/home/john/ros_parity_stationary_fixed_20260907`; the carry manifest
explicitly records those timeouts rather than claiming success.

Same-bag experiments, each with a fresh ROS estimator at 0.5x playback:

| settings | status samples | lost fraction | longest output gap (wall time) |
| --- | ---: | ---: | ---: |
| original tracked settings | 1490 | 31.8% | 19.64 s |
| `odom_bundle_min_motion:=1.0` | 1517 | 45.6% | 29.52 s |
| orientation aiding, separate filter process | 1440 | 28.5% | 19.52 s |
| orientation aiding, composed filter | 1456 | 41.6% | 26.04 s |

Reports are `replay_half_report.json`, `replay_half_minmotion1_report.json`, and
`replay_half_orientation_report.json`, plus
`replay_half_orientation_composed_report.json` inside the carried bag. Different message
arrival/drop patterns mean these are diagnostic runs, not deterministic
frame-for-frame estimator comparisons. Slower playback still loses tracking,
but it also still drops input frames, so it does not rule out compute/delivery
pressure as the cause. The minimum-motion setting
made this recording worse and remains **0.0 by default**. Orientation aiding
remains opt-in (`filter_imu:=true`) and is not a motion-gate pass. The final
launch composes Madgwick into `/oak_container` to avoid the extra process hop.
The composed experiment's full output bag is
`/home/john/ros_parity_carry_20260907_orientation_composed_outputs`; its
`report.json` uses sensor time and includes slightly more ending data:
1468 status samples, 42.0% lost and a 13.01 s longest output gap. Neither filter
arrangement passes the motion gate.
`use_imu:=false` now actually disconnects the odometry IMU subscription instead
of only disabling the wait-for-IMU flag.

The tracked sensor config now pins stereo confidence **55** (DepthAI 3.5's
`StereoDepthConfig` default) and `ACCELEROMETER_RAW`/`GYROSCOPE_RAW`, matching
the reference requests. The carried bag predates these settings and cannot
evaluate their effect. A later 30-second stationary check of the new settings
measured 7.6 Hz and 0.76 mm RMS, with zero lost updates; its report is
`/home/john/ros_parity_inputmatched_stationary_20260907.json`. This was a later
scene/session and is not a controlled accuracy improvement over the morning
reference. The driver also logs a distorted `inputAlignTo` warning; depth
alignment remains worth checking against the native graph before claiming
pixel-for-pixel input parity.

### IMU correction: raw samples are not orientation aiding

The live OAK message has orientation `(0,0,0,1)` with zero covariance and no
changing orientation estimate. The installed RTAB-Map 0.22.1 F2M base caches
IMU only if the orientation quaternion's vector part is nonzero; otherwise it
ignores the IMU. It does not integrate raw gyro/accelerometer inputs into a
motion estimate. The DepthAI 3.5 port pins an earlier RTAB-Map commit with the
same behavior, and its RTABMapVIO adapter only supplies raw gyro/acceleration.
Thus **providing IMU samples and waiting for their timestamps did not establish
inertial fusion** on either F2M path. The earlier v3 timing measurements remain
valid; the claim that they proved fusion does not.

Sources: [installed-core version's IMU handling](https://github.com/introlab/rtabmap/blob/0.22.1/corelib/src/Odometry.cpp),
[DepthAI 3.5 RTAB-Map dependency](https://github.com/luxonis/depthai-core/blob/v3.5.0/cmake/ports/rtabmap/portfile.cmake),
[DepthAI 3.5 adapter](https://github.com/luxonis/depthai-core/blob/v3.5.0/src/rtabmap/RTABMapVIO.cpp).

Optional Madgwick filtering supplies the orientation that F2M can actually use.
It has no magnetometer and publishes no TF. This is orientation aiding, not a
tightly coupled inertial estimator, and it is a deliberate experiment beyond
the native adapter's raw-input behavior.

### Bag reporting and acceptance

`ros_rtabmap_bag.py report BAG --out REPORT.json` computes metrics directly from
recorded outputs and bag receive timestamps. It does not need a running graph.
It now reads the epoch from `metadata.yaml`: Jazzy's Python metadata exposes
ROS Time objects, not Python datetimes. A regression test writes a real small
MCAP bag and checks the report.

Both the live harness and bag report reject an ending pose older than 0.5 s and
hold windows with a position span above 0.05 m (`--max-hold-span` can adjust the
threshold). This rejects continued carrying or unstable estimation rather than
reporting either as measured closed-loop drift. It cannot independently prove
the camera was physically returned to its mark or orientation.

New captures snapshot effective parameters **before** recording starts; wait
for the recorder's subscription messages before moving. Existing bag paths
are never overwritten. For replay output capture, use `ros2 bag record
--use-sim-time` so the saved output uses sensor time even at 0.5x playback.
The original live output remains in the input bag and is excluded from playback.

### Controlled visual replay through both estimators

`scripts/depthai_rtabmap_bag.py` feeds saved image/depth/tracked-feature tuples
into the actual DepthAI 3.5 `RTABMapVIO` host node. It requires the original OAK
for calibration and the SDK's internal Sync node; it creates no live cameras,
mapper or ROS publishers. It checks the bag's focal lengths/principal point
against the connected OAK before processing. `device_calibration.json` records
the calibration used. The SDK's default body convention is retained, so direct
coordinate subtraction against ROS poses is not a trajectory-alignment method.

`scripts/ros_rtabmap_replay_frames.py` supplies the same tuples to a fresh ROS
tracked estimator and waits for each matching OdomInfo result before sending
another. Both scripts use the same bounded 1 ms matcher, canonicalize tuple
stamps to the image stamp, skip the SDK's initialization-only first tuple, and
save original image stamps/sequence numbers in `poses.jsonl`. Unmatched inputs
are counted. A September 8 forced blank-image/depth test found that the SDK
emits **zero position and quaternion `(0.5,0.5,0.5,0.5)`** on failure with its
default local transform. This quaternion is unit length, so the original
norm-only classifier was wrong. The replay now conservatively rejects that
exact sentinel as well as nonfinite/non-unit output. The SDK exposes no lost
bit; an actual pose exactly equal to the sentinel is indistinguishable.

These are **offline visual diagnostics**, not real-time or full-stack parity
measurements. Raw IMU is deliberately omitted because both F2M cores ignore it
without orientation. Per-frame elapsed time includes conversion/transport and
waiting, including a USB round trip for the SDK. Native core parameters remain
at defaults unless `--params` supplies an explicit JSON string map. In
particular, native defaults are `Vis/MaxFeatures=1000` and `OdomF2M/MaxSize=2000`,
larger than the current ROS tracked profile's 600/1000 tuning.

```bash
# Source the environment from README first. Run the two estimators separately.
PYTHONPATH="$PYTHONPATH:/usr/lib/python3/dist-packages" \
  /home/john/depthai_env/bin/python scripts/depthai_rtabmap_bag.py \
  /home/john/ros_parity_carry_20260907_01 --out /home/john/new_native_comparison

ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py \
  start_camera:=false use_sim_time:=true use_imu:=false \
  odometry_rate_hz:=0.0 sensor_qos:=1 vis_max_features:=1000 odom_f2m_max_size:=2000 foxglove:=false
# In another terminal with the same sourced environment:
python3 scripts/ros_rtabmap_replay_frames.py \
  /home/john/ros_parity_carry_20260907_01 --out /home/john/new_ros_comparison
```

The native full replay is `/home/john/ros_parity_carry_20260907_native_serial`.
Use **`validated_report.json` and `validated_poses.jsonl`** there: the original
files incorrectly count two failure sentinels as valid. Corrected results:
2593 outputs, 2 lost (0.077%), longest valid-output gap 0.100 s in sensor time,
maximum step between valid poses 0.132 m. Failure times are 40.128 and 76.454 s.
The forced-failure evidence is `/home/john/ros_parity_native_null_probe_20260908`.
This is not a live-rate benchmark or a return-to-origin accuracy measurement.

The initial ROS frame-at-a-time replay stalled: image/tracks/calibration arrived,
but a depth message did not. The tracked adapter hard-coded best-effort input
QoS despite the parent node's `qos` parameter. It now honors that setting, and
the controlled replay requires `sensor_qos:=1` for reliable delivery of large
depth messages. Live defaults remain best effort (`sensor_qos:=2`).

On September 8 the package build passed and the full suite passed **597 tests,
0 errors/failures/skips**, including both reliable and best-effort component
inputs, frame correspondence, depth byte order/stride, and SDK failure-sentinel
classification. The ROS frame replay also accounts for an initialization
detail: with `publish_null_when_lost=false`, OdometryROS acknowledges its first
frame through OdomInfo but withholds the pose until a velocity estimate exists.
The harness records that state without inventing a pose or timing out.

### September 8 controlled replay findings

The first complete ROS frame-at-a-time replay is
`/home/john/ros_parity_carry_20260908_ros_serial_complete`. Its 2593 submitted
image sequence numbers and nanosecond stamps **exactly match** the native
replay. It lost 478 updates (18.43%), with a 12.10 s longest valid-pose gap in
sensor time. Its launch log and effective parameters are saved there. A prior
partial run followed a different loss pattern; do not claim bitwise
determinism merely because the input frames match.

`/home/john/ros_parity_optimizer_audit_20260908/native_defaults_probe.cpp`
loads the installed SDK's exported RTAB-Map parameter getter without starting
a device. The 127 dumped defaults are in `native_runtime_defaults.tsv`. Among
the parameters also exposed by the ROS odometry node, only two defaults differ
after explicitly matching the feature/map limits: native `Optimizer/Strategy=1`
and `Optimizer/Epsilon=0.0`, versus ROS 2 and 0.00001. This confirms the installed
SDK's feature settings directly rather than inferring them from build files.

The next replay disabled the ROS mapper and loaded those optimizer defaults
through `odom_config_path` (a standard RTAB-Map INI). It is saved at
`/home/john/ros_parity_carry_20260908_ros_native_optimizer`: 346/2593 lost
(13.34%), longest gap 7.03 s. These two changes do not establish causation or
parity, and neither changed the live defaults. `start_mapper:=false` disables
both the mapper component and OctoMap for isolated odometry experiments.

The tracked adapter now also clears the stereo projection translation in
CameraInfo when constructing its left RGB-D model. The driver left P[3] is
-34.3176, while DepthAI's `model.left()` has zero projection translation. The
focal lengths, principal point and calibrated local transform are preserved.
This matches the SDK's RGB-D camera construction and is covered by both QoS
variants of the component test. It is not yet a motion-parity result.

For a diagnostic using DepthAI's internal reference frame, launch with
`base_frame_id:=native_sdk_internal start_mapper:=false` and pass
`--native-body-frame` to `ros_rtabmap_replay_frames.py`. The harness adds that
virtual frame as the parent of the left optical frame, replacing the physical
rig's static TF tree for this diagnostic only. This reproduces the SDK 3.5
camera model's fixed rotation and zero lever arm without conflicting with
OdometryROS's dynamic odom-to-base TF. The output is
explicitly labeled with this virtual frame; it must not be interpreted as
the physical `oak` body pose. The SDK additionally conjugates its output pose,
so direct coordinate subtraction is still not a comparison method.

The projection-normalized replay is
`/home/john/ros_parity_carry_20260908_ros_rgbd_projection`: 342/2593 lost
(13.19%), longest gap 6.965 s. The SDK internal-frame replay is
`/home/john/ros_parity_carry_20260908_ros_native_frame_root`: 343/2593 lost
(13.23%), longest gap 6.999 s. Both completed the same frame sequence. Thus
these geometry adjustments do not explain the native/ROS tracking gap.
Earlier `ros_native_frame`/`ros_native_frame_paced` directories are incomplete
probes with a conflicting TF parent; do not use them as estimator benchmarks.

The binary parameter probe also calls `Parameters::getVersion()` and confirms
**RTAB-Map 0.21.12** inside the installed DepthAI 3.5 wheel. The source port's
package version label is not the runtime version. ROS is 0.22.1. An isolated
source comparison is being prepared in `/home/john/rtabmap_parity_0_21_12`:

- Core source commit: `08f031e11c45589fc2b68440383a3e40982dc06f` (the SDK port pin).
- ROS wrappers: `5723507617c4bcd16b35da391d506da2661c3784`
  (`0.21.10-jazzy`, the latest available 0.21 Jazzy wrapper tag).
- Separate `build`, `install`, and `ros_ws`; nothing replaces system packages.
- Core build uses system OpenCV 4.6, g2o enabled and GTSAM disabled. This isolates
  core-version behavior from the installed ROS core, but does **not** reproduce
  every dependency/build choice in the SDK wheel.
- Configuration/build logs and source archives are retained in that directory.

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
not equivalent to Basalt, OpenVINS or VINS-Fusion. The node is named VIO and supplies IMU measurements, but this F2M core ignores
raw measurements without orientation; see the correction above. The
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

## OAK-tracked ROS odometry path, 2026-08-31

Both host-detected mono variants then failed a 60-second moving screen: the
600/2000 descriptor profile stopped producing valid poses for the final 32.85
seconds, and the optical-flow/no-BA profile lost for 31.05 seconds before
recovering with a 516 mm discontinuity. This motivated a closer parity path.

`ros_rtabmap_oak_v3_tracked.launch.py` runs `StereoDepth` and `FeatureTracker`
on the OAK, but no DepthAI VIO or SLAM. The new
`px4_vio_bridge::OakTrackedOdometry` component derives from ROS RTAB-Map's
`OdometryROS`, so ROS still owns IMU interpolation, TF, reset behavior and the
standard odometry/info outputs. It synchronizes rectified CAM_B, CAM_B-aligned
depth, camera calibration and the RVC2 Harris/KLT output, constructs
`rtabmap::SensorData`, calls `setFeatures()` with the device points, and then
uses the ordinary RTAB-Map F2M core. ROS `rtabmap_slam` and the OctoMap adapter
are unchanged.

Do not replace that direct `SensorData` handoff with a populated
`rtabmap_msgs/RGBDImage`: Jazzy's `rgbdImageFromROS()` ignores `key_points`.
The discarded-keypoint prototype showed 239 OAK points but 564 host-detected
features. In the corrected live graph, asynchronous samples showed 235 OAK
points and 216 odometry input features. The adapter must mirror depthai-core by
passing locations only; putting OAK feature IDs in `cv::KeyPoint::class_id`
made the local-map processing substantially slower. Duplicate synchronized
image stamps are filtered before `OdometryROS` and the standard
`/diagnostics` input/output status is enabled.

With that correction, a 2000-point local map retained about 125-135 stationary
inliers but aged to 5-6 Hz and ~141% total CPU. The selected 1000-point default
retained about 110-117 inliers; at 70 seconds it measured 8.0-8.2 Hz and ~125%
total CPU. It stayed tracked through a 108-second stationary soak, though frame
processing still grew to 110-120 ms near the end. This proves the data path,
not moving robustness or full DepthAI performance parity.

Because the unpacked binary `depthai_ros_msgs_v3` was built against a newer
Fast-CDR, source `/home/john/depthai_v3_compat_ws/install/setup.bash` after the
v3 overlay and before the project workspace. Never overlay the newer Fast-CDR
itself; doing so made the system Fast DDS stack crash. The next required carry
is:

```bash
ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py

python3 scripts/loop_closure_test.py \
  --label ros-oak-tracked --seconds 60 --hold-seconds 10 \
  --raw-odom-topic /rtabmap/vio_odometry --info-topic /rtabmap/info \
  --out /tmp/loop_ros_oak_tracked.json
```

## Open gates before flight

1. **OAK-tracked ROS VIO stability under motion.** Both host-detected mono
   profiles fail for roughly half of a 60 s carry. The direct OAK-feature path
   also fails the September 7 motion bag (39.8% lost updates live). Resolve
   the recorded-motion failures and establish live continuity before extending
   a new marked closed-loop capture to 180 s.
2. Re-run the loop-closure test once the VIO holds tracking, and confirm the
   ROS mapper accepts closures when the trajectory is consistent. The
   corrected-pose path and `cpp_slam_pose_relay`'s non-identity branch remain
   unexercised outside unit tests.
3. Moving-scene CPU. The September 7 carried capture measured 133.3% average
   for container plus OctoMap, excluding recorder; compare against native on
   equivalent workloads once tracking is continuous.
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
