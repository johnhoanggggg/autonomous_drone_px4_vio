# ROS-native RTAB-Map Migration — Results Handoff

## Latest integration update — September 13: selectable ROS-to-PX4 mode

Foxglove bandwidth follow-up: live compressed image traffic measured 24.424
Mbit/s at 29.13 Hz; combined OctoMap markers 0.513 Mbit/s and TF 0.028 Mbit/s.
The client TCP connection delivered roughly 9–12 Mbit/s and had 0.5–0.7 MB
queued, supporting a visualization/network bottleneck. The archived diagnostic
bag is `/home/john/ros_live_delay_20260913T121300Z/bag`; disk/RAM SHA-256 hashes
matched before temporary RAM copies were removed. Raw image age was median
76.4 ms, ROS odometry 173.3 ms, PX4 input 179.0 ms. No client render latency
was measured.

Implemented a ROS/Foxglove-only compressed image relay, default 5 FPS via
`foxglove_image_rate_hz`, on `/foxglove/camera/image/compressed`. It uses depth-1
best-effort QoS, monotonic callback gating, no backlog/re-encoding, and publishes
only with subscribers. Raw OAK images (including full-rate compressed) are now
excluded from the default bridge whitelist. VIO camera rate and DepthAI flight
behavior are unchanged. Users must select the new image topic after restarting;
custom whitelists can bypass the restriction. Implementation did not restart
the live stack; actual end-to-end latency improvement remains to be measured.

Follow-up runtime fix: the first user-started combined launch crashed the
camera/odometry container with an undefined Fast-CDR symbol in the unpacked
`depthai_ros_msgs_v3` library. Re-sourcing the base environment in an already
sourced shell prepended the incompatible overlay, while setup-script path
deduplication did not move the compatible rebuild back ahead of it. The wrapper
now explicitly prioritizes the compatible message and patched driver prefixes
after all setup scripts. It also loads the Fast DDS C++ message type support
with immediate symbol resolution before starting the graph, failing early on
ABI errors. `ros_rtabmap_backend --check-environment` performs that check without
opening the camera. Regression tests cover once- and twice-sourced user shells.
The user stopped the failed stack; no hardware restart was performed for this
fix. Earlier build/launch-construction success did not test lazy runtime symbol
loading and must not be treated as proof of a successful live launch.

This section supersedes historical statements below that no ROS path can feed
PX4. The user explicitly requested ROS visual odometry forwarding toward flight
testing. `rtabmap_slam_px4.launch.py backend:=ros foxglove:=true` now selects the
tested host-feature/latest-frame/cached-CameraInfo/PnP-3 profile and forwards raw
`/rtabmap/vio_odometry` via the existing bridge. Default `backend:=depthai` is
unchanged, and the existing 3D wrapper is pinned to DepthAI.

The ROS bridge accepts only `rtabmap_odom` / `oak` odometry, retains sensor sample
time for PX4 and velocity differentiation, rejects invalid/stale (>500 ms),
future and non-increasing samples, and never uses corrected map poses. ROS mode
loads the Pi-specific base environment and latest-frame overlay through an
installed wrapper; a missing patched odometry overlay fails closed.
Foxglove exposes the full binary OctoMap plus bounded ground/obstacle markers
(default 5000 voxels with Foxglove enabled, zero otherwise), fixed frame `world`.

Historical CPU is ~137–157% of one-core convention across different earlier
runs, including failed motion, not a fresh benchmark: camera/odom ~114–131%,
mapper ~20–23%, OctoMap ~2–4%. The successful 10:27 RAM carry did not measure CPU.
The new combined ROS/PX4/Foxglove mode still needs props-off reception, frame,
timestamp/latency, tracking-loss and visualization checks before flight. No
hardware graph was started and no PX4 messages were sent during implementation.

Verification: `colcon build --packages-select px4_vio_bridge --executor sequential
--parallel-workers 1` succeeded. `colcon test --packages-select px4_vio_bridge`
passed all 32 CTest targets. The focused bridge/backend-launch pytest suite
passed 15 tests, including synthetic raw input to PX4 payload conversion with
no ROS graph. Installed parent and wrapper `--show-args` checks succeeded from
the user's ordinary sourcing environment. `git diff --check` passed. A read-only
CPU/Foxglove worker audited historical artifacts; implementation and final
verification stayed with the lead. Launch condition deprecation and existing
overlay/CMake warnings remain non-fatal.

Historical acceptance discussion and results follow; they are not a current
flight-readiness declaration.

Status: **Neither ROS visual candidate is ready for PX4.** The OAK-keypoint ROS
path fails the September 12 acceptance2 carry, while the host-feature ROS path
passes that saved bag twice at deterministic 10 Hz with mapping and native
OctoMap. However, a fresh live host-feature carry on September 12 failed after
an abrupt visual transition, and an exact same-bag DepthAI
FeatureTracker -> RTABMapVIO -> RTABMapSLAM control failed at the identical
sensor time. The new bag therefore establishes a shared visual-observability
limit rather than ROS-only regression. The 10 Hz host graph is still the best
ROS efficiency candidate. A September 13 experimental latest-pending host
candidate now passes acceptance2 twice at 16.0–16.6 Hz and runs stationary
at 18.15 Hz, but the fresh September 13 30 Hz carry also failed tracking
and ended with a 61.79 s stale pose. Throughput gains did not establish
motion robustness.
Both SDK and ROS subsequently pass every frame of the new carry serially;
replaying the live-selected subset reproduces all tracking statuses exactly.
PnP-3 with normal keyframe thresholds passes the September 13 saved carry
twice at real-time input rate (~15.7–16.3 Hz), but its fresh live motion retry
loses tracking for 20.35 s and exposes a 21.06 s pose gap. Neither candidate
is qualified for flight. No migration path is connected to PX4.

**September 8 dense-cohort finding:** host GFTT/ORB detection now gives both the SDK
and the standard ROS RGBDOdometry node **1 lost frame out of 2593**, at the same
recorded timestamp, with a 0.100 s longest valid-pose gap. Use the new
`ros_rtabmap_oak_v3_native_features.launch.py` profile. This is an offline
continuity result; full live and map parity remain open. The native replay's supplied feature input
has not been validated as consumed. A forced out-of-image feature probe still
produced ten valid poses, and `passthroughFeatures` timed out while image/pose
outputs arrived. Thus the strong native results below must **not** be described
as proven same-feature estimator comparisons. Explicit host detection now
provides the reproducible comparison; optional supplied-feature mode fails
closed unless passthrough confirms consumption.

Branch: `ros-rtabmap-migration`. Nothing on this branch publishes to `/fmu/in/*`
except the pre-existing `vio_to_px4_odometry`, whose input is unchanged.

Session end state: all ROS/DepthAI test graphs and recorders are stopped. The
working tree is intentionally dirty with the migration implementation, tests,
reports and handoff updates; do not discard or reset it. Start a new shell with:

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source /home/john/autonomous_drone_px4_vio/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=42
```

The latest implementation validation before this handoff update was
31/31 CTest targets / 627 tests passing; `git diff --check` is
clean. The broad repository-root `pytest` invocation is not canonical because
generated install trees create duplicate-module collection/import mismatches.

## Resume here — September 13 processing-cost continuation

The Pi restarted before the last September 12 combination finished. All test
graphs were stopped on resumption; the current boot began September 13 at
01:18:18 UTC. Preserve the unfinished
`ros_parity_acceptance2_host_cost_projection_decimate2_15_qos2_fixed_20260912`
directory, now explicitly marked `incomplete.json`. Its log ends mid-run and
contains no completed report. The completed fresh repeat is reported below under September 13
paths. No estimator defaults have been changed.

The corrected-pacing best-effort September 12 controls that did complete are:

| Host variant, 15 Hz input | statuses | lost | status Hz | max pose gap | max pose step |
| --- | ---: | ---: | ---: | ---: | ---: |
| baseline | 791 | 0 | 9.12 | 0.333 s | 185 mm |
| `Optimizer/Iterations=10` | 833 | 75 | 9.60 | 6.798 s | 233 mm |
| iterations 10 plus reversed correspondence direction | 1037 | 45 | 11.96 | 6.398 s | 360 mm |

Reject both reduced-iteration variants. They retain BA but do not preserve
continuity. The unfinished half-resolution/reversed-direction combination had
22–26 ms updates in its final logged section but only 54–96 inliers; this is
partial profiling evidence, not an acceptance result. The new audit directory
is `/home/john/ros_parity_optimizer_audit_20260913`. IMU remains off, reset
countdown zero, and no PX4-facing graph is launched.

The fresh half-resolution/reversed-direction full-stack repeat **fails**:
983 statuses, 283 lost, 699 valid poses, a 418.7 mm maximum adjacent-valid
step and a **33.458 s stale final pose**. The maximum gap *between valid poses*
is only 0.467 s because the terminal outage has no later recovery pose; it must
always be read alongside final pose age. Median update time is 40.8 ms, but
median inliers are 113 and the local-map keyframe count reaches 344. Reject
this combination; the early low processing times did not predict robustness.
Result:
`/home/john/ros_parity_acceptance2_host_cost_projection_decimate2_15_qos2_20260913`.

The package rebuild and all **31 CTest targets / 627 tests** pass with zero
errors, failures or skipped tests; compilation and `git diff --check` also
pass. Build/test logs are `package_{build,test,test_result}.log` in the new
audit directory. The latest completed output/timing summary is
`host_cost_summary.json`, regenerated by `summarize_host_cost.py`.

The full-resolution matching sweep also fails to establish a candidate:

| Variant, 15 Hz input / QoS 2 | statuses | lost | max pose gap | max pose step |
| --- | ---: | ---: | ---: | ---: |
| reversed search, 20-pixel window | 977 | 25 | 3.799 s | 278 mm |
| global brute-force matching, window 0 / NN type 3 | 1115 | 84 | 6.398 s | 157 mm |
| reversed search, normal 40-pixel window repeat | 939 | 48 | 6.731 s | 99 mm |

All kept 1000 features, 2000 map points, normal 20 optimizer iterations, IMU
off and reset countdown 0. The 40-pixel repeat does not reproduce its earlier
zero-loss reliable-QoS result, so reversed correspondence direction is not a
validated continuity fix. Logs/commands are `host_cost_*` and runner
`run_host_matching_sweep.py` in the new audit directory. CPU was observed at
2.4 GHz with `throttled=0x0`; reduced frequency was not the active limit.

Frame-level inspection shows the fast global matcher loses at sequence 1408
(47.0625 s) with 14 inliers, then cannot reattach until about 53.3 s. The
zero-loss baseline omitted sequence 1408 under load. However, the earlier
zero-loss reversed-search run did process 1408 successfully with 22 inliers,
so dropping that one frame is not the sole explanation: local-map history and
registration matter as well. The narrow-window variant crosses this turn but
fails later at 77.99–81.25 s.

### Exact stride-2 control establishes another shared visual limit

`run_host_serial_stride2.py` completed all **1299** even selected frames through
unchanged host odometry, one frame in flight, no mapper and reliable QoS.
It lost **95/1299 (7.313%)**, published 1202 poses, recovered by the end and
had a 6.465 s longest gap between published valid poses. This is an offline
continuity diagnostic, not an FPS benchmark. Result:
`/home/john/ros_parity_acceptance2_host_cost_baseline_serial_stride2_20260913`.

An exact comparison against the retained DepthAI **device-regenerated feature**
stride-2 replay confirms **1299 common sequences, matching nanosecond stamps,
the same 95 lost frames and zero tracking-status disagreements**. Evidence:
`serial_stride2_native_comparison.json` in the September 13 audit directory.
ROS uses host GFTT/ORB and the SDK uses freshly generated device features, so
this is not a supplied-keypoint identity claim or trajectory-error measurement.
ROS's published-pose gap is one input interval longer than native's because
its first recovery status has no velocity yet and suppresses the pose.

**Do not equate a 15 Hz pose-rate target with feeding the estimator a fixed
15 Hz visual subset.** This exact even subset fails in both stacks even with
lossless serial delivery. Some zero-loss real-time runs omit difficult frames
under load; they do not prove every frame of this subset is trackable. More
parameter sweeps on the same subset are not a sufficient path to parity.

`run_host_full30.py` then supplied the complete 30 Hz sequence at real time,
leaving frame selection to stock host odometry with full mapping/native OctoMap,
QoS 2, IMU off and reset countdown 0:

| Full 30 Hz input | statuses | lost | status Hz | max pose gap | max step |
| --- | ---: | ---: | ---: | ---: | ---: |
| unchanged host | 887 | 47 | 10.22 | 6.665 s | 138 mm |
| reversed correspondence direction | 1027 | 0 | 11.84 | 0.267 s | 197 mm |

Results use
`ros_parity_acceptance2_host_cost_{baseline,projection}_full30_qos2_20260913`.
The passing reversed-search result is one replay, not a robust repeat gate.

A subsequent live stationary reversed-search/30 Hz source check produced
621 poses at **13.799 Hz**, 620 lite statuses with zero loss, 0.488 mm RMS,
0.285 mm net displacement and 119.2 ms median pose age. Largest receive gap
was 0.412 s and maximum pose age 0.444 s. CPU: camera/odometry 114.05%, mapper
20.23%, OctoMap 2.61% (136.90% total), no throttling. Median core update was
59.5 ms (p95 68.9 ms), BA 16.4 ms, and inliers 339. No keyframes were added in
the timing window. The original runner exited before sampling, leaving its
launch alive; the sample was collected directly from the retained process IDs
and the graph was then stopped cleanly. The CLI parameter dump/get returned
`Node not found` despite live topic discovery, so the empty effective YAML is
not parameter-validation evidence. Launch commands and live sampling artifacts
are `host_live_projection40_30_*` in the new audit directory. Do not call this
a matched-scene accuracy comparison with yesterday's live baseline.

`run_host_pending_full30.py` uses only the existing isolated
`/home/john/rtabmap_latest_frame_overlay/install/local_setup.bash` overlay,
with full-resolution reversed search and full mapper/OctoMap stack. Runtime
parameter snapshots bypass the ROS CLI daemon. The latest-pending policy
completed 1235 statuses (14.23 Hz), 55 lost, 1178 poses, a 6.665 s gap and
87.6 mm maximum step. Median input-to-status latency was 96.7 ms. It is faster
but fails continuity; do not promote it.

The oldest-pending policy then passed once: **1250 statuses, zero lost, 1249
poses**, a 0.267 s maximum pose gap, 184 mm maximum step and 33 ms final pose
age. Median input-to-status latency was 124.1 ms, maximum 430 ms. It produced
82 mapper/OctoMap updates, 14 global closures and 17 proximity detections;
OctoMap used 49 source nodes with 12,962 ground, 27,459 obstacle and 849,495
empty cells. This remains one passing replay on the isolated dependency
overlay, not a default promotion. Output:
`ros_parity_acceptance2_host_cost_projection_oldest_full30_qos2_20260913`.
The repeat runner `run_host_oldest_repeat.py` **failed**: 1231 statuses,
56 lost, 1173 poses, a 6.565 s gap and 124 mm maximum step. Median latency
remained 124.7 ms. Thus oldest-pending is not a robust continuity candidate,
and its prepared live stationary runner `run_live_projection40_oldest.py`
was not executed. No handoff policy is promoted.

Follow-up runner `run_host_pending_registration.py` retains latest-pending,
normal BA and 20 minimum inliers. It separately tests `Vis/PnPReprojError=3`
and modest keyframe thresholds (ratio 0.15 / absolute 100). Both first passes completed without loss:

| Latest-pending, reversed search, full 30 Hz input | statuses | lost | status Hz | max pose gap | median latency | max step |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| PnP reprojection tolerance 3 pixels | 1346 | 0 | 15.51 | 0.267 s | 93.2 ms | 205 mm |
| keyframe ratio 0.15 / absolute 100, original PnP 2 pixels | 1439 | 0 | 16.58 | 0.267 s | 88.5 ms | 187 mm |

Both have zero final sensor-time pose age. The PnP variant produced 83
mapper/OctoMap updates and 17 global closures; the keyframe variant produced
84 updates and 15 global closures. These are the first passes in this round
to meet both the 15 Hz throughput and zero-loss replay targets. They still
skip input frames under load, and this bag has no valid return hold for an
accuracy measurement. Repeat runners `run_host_pnp3_repeat.py` and
`run_host_kf015_repeat.py` check reproducibility before live validation. These
are diagnostic parameters on the isolated overlay. No installed
package/default is replaced or promoted.

Both repeats passed continuity. PnP-3: 1316 statuses / zero lost, 1315
poses, 15.17 status Hz, 0.267 s maximum gap, 196 mm maximum step,
93.3 ms median latency and 33 ms final pose age. Keyframe-0.15/100:
1389 statuses / zero lost, 1388 poses, approximately 16.0 status Hz,
0.233 s maximum gap, 246 mm maximum step and 87.7 ms median latency.
Reports append `_repeat_20260913` to the respective first-pass directory
names (replacing `_20260913`). Effective YAML confirms PnP=2 for the
keyframe candidate, reversed projection matching and latest-pending enabled.
The faster keyframe candidate completed its live stationary measurement
with full mapping; this does not change the default profile.

Live sample (45.002 s): 817 poses / **18.155 Hz**, 820 lite statuses / zero
lost, 0.707 mm position RMS and 0.779 mm net displacement. Median pose age
130.1 ms, maximum 324.1 ms, longest receive gap 109.1 ms. Mapper and OctoMap
each emitted 43 updates (~0.956 Hz). CPU: camera/odometry 131.16%, mapper
23.15%, OctoMap 2.85%, **157.17% total**; 100% means one core. Temperature
56.2 C and `throttled=0x0` at start/end. The independent timing window has
799 statuses / zero lost, median update 53.5 ms (p95 58.9 ms), BA 11.6 ms,
inliers 244 and local map 1492 points; no keyframes were added. This scene
and map differ from earlier live samples, so do not attribute all rate gains
to the keyframe setting or call the RMS a matched-scene accuracy comparison.
Effective parameter snapshot succeeded. Artifacts: audit directory
`host_live_projection_kf015_latest30_{commands.json,effective.yaml,cpu.json,timing.json,launch.log}`.

The separate 10 Hz live retry bag (which previously failed in both SDK and
ROS) completed before the user's next Pi restart. `run_host_kf015_failure_bag.py`
produced 833 statuses, 315 lost, 517 poses and **35.499 s final pose age**;
maximum adjacent-valid gap 0.300 s and maximum step 184 mm. Thus the new
keyframe candidate does not solve this terminal failure and loses its final
track about 0.4 s earlier than the unchanged serial control. Output:
`/home/john/ros_parity_acceptance2_host_cost_projection_latest_kf015_host_retry10_qos2_20260913`.
It cannot validate the benefit of 30 Hz capture because those intermediate
images were never recorded. No test graphs survived the restart. The PnP-3
alternative then completed this failure bag: 799 statuses / 33 lost, 764
poses, **5.299 s maximum gap**, 690 mm maximum step and zero final pose age.
It recovers, unlike the keyframe variant, but does not pass continuity or
establish recovery accuracy. Output:
`ros_parity_acceptance2_host_cost_projection_latest_pnp3_host_retry10_qos2_20260913`.

The user is available for a fresh carry, so the planned third acceptance2
replay is deferred in favor of actual 30 Hz live motion validation. The
keyframe candidate is recording a 110 s carry with 5 s discovery, 20 s
stationary pre-roll, 65 s motion and 20 s stationary post-roll. Bag:
`/home/john/ros_parity_live_motion_host_kf015_latest30_20260913`.
Capture completed successfully: recorder exit 0, all required input topics
present, 109.146 s duration, 3270 left images, 3269 depth images, 3267
camera infos and 3270 tracked-feature messages. Commands, effective
parameters, CPU and recorder logs use the `host_live_projection_kf015_carry30_`
prefix in the audit directory.

**Fresh carry fails.** Report has 1358 statuses / 547 lost (40.3%), 808
poses, one ~60.27 s loss episode starting at bag-relative 48.6 s and
**61.79 s final valid-pose age**. Maximum step before terminal failure is
75 mm; internal valid-pose gaps reach 1.21 s. No ending-hold poses and no
closures: return-to-origin error is unavailable. CPU sample 110 s:
camera/odometry 127.46%, mapper 16.03%, OctoMap 1.77%, total 145.26%,
56.2 C and no throttling. This includes the long loss interval and must not
be used as a healthy motion efficiency score.

The exact new bag completed unchanged DepthAI 3.5 device
FeatureTracker -> RTABMapVIO -> RTABMapSLAM, all frames, serial, IMU off:
`/home/john/depthai_host_kf015_carry30_device_full_stack_20260913`.
**3266 outputs / zero invalid**, 3266 SLAM poses, 450 grid outputs,
66.66 ms maximum sensor-time gap, zero final pose age and 136 mm maximum
adjacent raw step (185 mm corrected). Input audit: 3267 matched tuples,
8 unmatched messages, maximum tuple skew 963 ns. Native passes the live
failure point and retains tracking to the end; unlike the September 12
10 Hz carry, this is not a demonstrated shared terminal visual failure.
Serial delivery and different feature/estimator configuration remain
confounders; do not attribute the difference solely to ROS transport.

Native first/last 10 s hold spans are 13.6 mm / **253.8 mm**. The ending
pose is not stable enough for the 50 mm hold gate; the 644.5 mm displacement
between hold means is **not a valid return-to-origin accuracy result**.
Evidence: `host_live_projection_kf015_carry30_native_holds.json` in audit.
The matching ROS keyframe settings also **pass every frame serially**:
3266 statuses, zero lost, 3265 poses, 66.66 ms maximum sensor-time gap and
zero final pose age. Stock odometry (pending buffer irrelevant with only
one frame in flight), mapping/OctoMap off, IMU off, reset 0. Runner:
`run_host_kf015_carry30_serial.py`; output:
`ros_parity_acceptance2_host_cost_kf015_carry30_serial_20260913`.
This points toward live frame delivery/selection or initialization as the
remaining issue; it is not a live throughput or whole-stack equivalence test.

`extract_carry30_live_selection.py` matches the recorded live OdomInfo stamps
to exact input sequences and writes a replay selection/audit. `run_host_kf015_carry30_live_selection.py` then completed only those
frames serially with the same keyframe settings: **1355 statuses / 546
lost**, 808 poses, 1.199 s internal gap and **61.622 s final sensor-time
pose age**. Exact sequence/stamp join to live status gives **zero tracking
status disagreements across all 1355 common frames**, including all 546
losses. Three live statuses are unmatched to available recorded tuples
(two before the replay sequence begins, one during the terminal outage).
Live odometry initialized before capture whereas replay starts a fresh map;
despite that difference the selected sequence reproduces every status.

Evidence: `host_live_projection_kf015_carry30_selection{,_audit,_comparison}`
JSON/JSONL in audit; replay output:
`ros_parity_acceptance2_host_cost_kf015_carry30_live_selection_serial_20260913`.
This establishes a reproducible frame-selection failure without live
scheduling or mapping load. Full 30 Hz serial passes in both SDK and ROS;
15–18 Hz average output is insufficient if the selected subsequence breaks
tracking. `run_host_pnp3_carry30.py` then passes full 30 Hz new-bag real-time
input: **1774 statuses / zero lost, 1773 poses, 16.289 Hz**, 0.200 s
maximum gap, 149 mm maximum step, 89.3 ms median latency, zero final pose
age, 13 global closures and 104 OctoMap updates. Settings: reversed matching,
latest-pending, PnP=3, normal keyframe thresholds, full mapper/OctoMap,
IMU off and reset 0. This changes both PnP and keyframe thresholds relative
to the failed carry; do not attribute the pass to PnP alone. Output:
`ros_parity_acceptance2_host_cost_projection_latest_pnp3_carry30_qos2_20260913`.
`run_host_pnp3_carry30_repeat.py` also passes: 1714 statuses / zero lost,
1713 poses, 15.733 Hz, 0.200 s maximum gap, 125 mm maximum
step, 92.2 ms median latency and zero final pose age. It produced 10 global
closures and 103 OctoMap updates. Exact fields remain in
`host_cost_summary.json`; full report appends `_repeat_20260913` to the
first-pass output name (replacing `_20260913`). This variant now has two
zero-loss full-stack real-time passes on each of acceptance2 and the new
30 Hz carry. It remains an experimental replay candidate, not a successful
live carry or default promotion. It still failed continuity on the separate
September 12 10 Hz retry recording.

The first live PnP-3 carry was interrupted by a Pi restart shortly after the
motion cue. Preserve
`/home/john/ros_parity_live_motion_host_pnp3_latest30_20260913`; it has a
277 MiB partial MCAP, no rosbag metadata, and an `incomplete.json` marker. It
must be excluded from acceptance results.

The clean retry **fails the live motion gate**. Bag:
`/home/john/ros_parity_live_motion_host_pnp3_latest30_retry_20260913`.
It is a complete 109.05 s capture with all required inputs, 1399 recorded
statuses, 165 lost updates (11.8%), 1227 recorded poses, and two loss episodes.
The dominant episode begins at bag-relative 73.6 s, just before the return
warning, and lasts **20.35 s** into the final hold. The raw pose stream pauses
for **21.06 s**. It recovers by the end and has zero final pose age, but the
ending 10 s hold spans 1.818 m, so return-to-origin accuracy is invalid. The
maximum adjacent raw pose step is 220 mm. The mapper reports three global
closures and one proximity detection after recovery; these do not repair the
PX4-facing raw-pose outage.

The 110 s CPU sample measured camera/odometry 130.57%, mapper 21.99% and
OctoMap 3.71%, **156.28% total** (100% is one core), 56.75--58.4 C and no
throttling. Median pose receive age was 145.1 ms. Effective parameters confirm
PnP=3, reversed projection matching, normal keyframe thresholds 0.3/150,
latest-pending, IMU off and reset countdown 0. Artifacts use the
`host_live_projection_pnp3_carry30_retry_` prefix in the September 13 audit
directory. The audit runner is `run_live_projection_pnp3_carry_retry.py`.

Exact serial controls locate the failure in frame selection. PnP-3 processes
all 3251 matched frames with only **1 lost update** near 100 s, 3247 published
poses, a 0.100 s maximum sensor-time gap and zero final pose age. In contrast,
serially replaying the 1398 exact frames chosen live reproduces all **165 lost
updates**, the 21.13 s gap and final recovery. The exact sequence/stamp join
has zero tracking-status disagreements across all 1398 common inputs. Thus
live scheduling or mapper load is not required after the bad subsequence has
been selected, while the full visual sequence crosses the difficult section.
Evidence: `host_live_projection_pnp3_carry30_retry_selection{,_audit,_comparison}`
and runners `run_host_pnp3_live_retry30_{serial,selection}.py` in the September
13 audit directory. Outputs begin
`ros_parity_acceptance2_host_cost_pnp3_live_retry30_`.

The first failed selected frame is sequence 2208 (73.589 s sensor-relative),
after sequence 2189 (72.956 s): a **633 ms jump skipping 18 available frames**.
The controlled replay restores only sequences 2190--2207 to the otherwise
unchanged live selection. It completes **1416 statuses, zero lost, 1415 poses**
and zero final pose age. Thus restoring this short segment eliminates all 165
losses in this replay. Other gaps retained from the live selection still reach
1.500 s, so this is a causal diagnostic, not a live continuity pass. Runner:
`run_pnp3_retry_bridge.py`; output:
`/home/john/ros_parity_acceptance2_host_cost_pnp3_live_retry30_bridge18_serial_20260913`.

The recorder received all 18 intermediate images/depth pairs; image ages in
the inspected window peak at 104 ms and input stamps remain spaced by 33 ms.
This excludes a camera-wide outage but does not prove delivery to the odometry
subscriber. The live log's last successful core update takes 66 ms, followed
by roughly 559 ms before the next core update starts. Instrument synchronized
callback arrival and worker timing to distinguish subscriber/synchronizer gaps
from worker stalls. Evidence: `host_live_projection_pnp3_carry30_retry_gap_delivery.json`,
generated by `audit_pnp3_retry_gap.py`.

Capture caveat: the assistant received the return/hold cue together with the
recorder-stop output and did not relay that cue on time. Do not describe the
ending hold as correctly cued or infer physical instability from its span alone.
The independently recorded tracking outage remains valid evidence.

The isolated overlay now includes opt-in timing instrumentation via
`RTABMAP_TIMING_AUDIT=1`: synchronized callback entry/exit, enqueue, worker
dequeue/exit and core begin/end. Source additions are `TimingAudit.h` plus
hooks in `OdometryROS.cpp` and `nodelets/rgbd_odometry.cpp` under
`/home/john/rtabmap_latest_frame_overlay/src/rtabmap_ros/rtabmap_odom`.
The overlay package rebuild passes; default instrumentation is disabled.
No project launch/default parameters changed.

`run_pnp3_retry_timing.py` completes the new retry bag at real-time input with
full mapping: 1683 statuses, one lost, 1680 poses, a 0.367 s maximum
sensor-time pose gap, 137 mm maximum step and 94.6 ms median output latency.
Trace has 2556 synchronized callbacks and 1683 core calls. Callback-to-enqueue
time is 0.713 ms median / 5.662 ms maximum; enqueue-to-dequeue 18.1 / 75.0 ms;
dequeue-to-core-start at most 0.264 ms; core processing 60.3 / 165.4 ms; and
core-end-to-worker-exit 1.31 / 5.20 ms. Largest synchronized sensor-stamp gap
is 167 ms. The original 633 ms selected-frame jump and roughly 559 ms interval
between the old log line and the next core call are **not reproduced**.
The old log interval is not proven idle time because it excludes some worker
work. Trace logging can perturb scheduling, and paced tuple replay does not
preserve original per-topic arrival timing. Do not claim the live cause fixed.
Results: `ros_parity_acceptance2_host_cost_pnp3_live_retry30_timing_20260913`;
audit: `host_cost_pnp3_live_retry30_timing_{launch.log,summary.json}` and
`summarize_pnp3_timing.py`. The camera was not used in this replay.

Recorded-arrival replay now completes with raw subscriber tracing. The isolated
RGBD component adds opt-in `raw_image`, `raw_depth`, and `raw_info` hooks to its
existing subscriptions before synchronization. The overlay rebuild passes.
`run_pnp3_arrival_timing.py` uses `ros2 bag play` at 1x recorded receive timing,
the input-only allowlist, full mapper/OctoMap and a fresh output recorder.
It produces 1386 statuses, 22 lost, 1360 poses, a 2.71 s output gap, 327 mm
maximum adjacent step and zero final pose age. The original 21 s outage is not
reproduced. The output bag is
`/home/john/ros_parity_pnp3_live_retry30_arrival_timing_20260913`.

Trace counts: 3234 raw images, 3229 depth, 3253 CameraInfo, 3214 synchronized
tuples and 1386 core calls. Largest raw/synchronized sensor-stamp gaps are
about 67 ms, but selected worker stamps jump by up to 367 ms. Calibration
arrival explains the largest **replay** bursts: matched CameraInfo arrives
300--318 ms after image/depth, then synchronization releases the tuple within
tens of microseconds. Across 3214 matched tuples, post-last-input sync wait
peaks at only 2.11 ms. Latest-pending can overwrite the intermediate tuples
as delayed calibration releases a burst. Worker handoff wait peaks at 95.6 ms,
core duration 191.5 ms, post-core work 5.16 ms and between-worker-call wait
260.5 ms. This identifies a concrete replay delay mechanism, not proof of the
original live 633 ms jump's complete cause. Rosbag playback/DDS and tracing
can alter arrival timing; do not claim identical live execution.
Artifacts: `pnp3_live_retry30_arrival_timing_{commands.json,effective.yaml,report.json,summary.json,bursts.json}`
and logs in the September 13 audit directory. Analysis scripts are
`summarize_pnp3_timing.py` and `analyze_arrival_bursts.py`.

### Calibration arrival control passes twice

Semantic inspection confirms all 3253 CameraInfo messages have identical frame
ID, dimensions, distortion model, D/K/R/P, binning and ROI. Compare canonical
field JSON excluding only `header.stamp`; do not use reserialized CDR bytes as
a semantic identity test. The earlier byte-hash guard failure was intermittent
and its exact byte-level cause was not reproduced in the lead's sample.
Canonical SHA-256 is `2a145ad25a5fe6fd8214da3d2eb279652858210805bdebfea0322d2251228c55`.

`retime_retry_calibration.py` creates
`/home/john/ros_parity_pnp3_retry_calibration_retimed_20260913` with only
CameraInfo receive timing changed to its corresponding image receive time.
There are 3252 exact stamp matches; one unmatched image and one unmatched
CameraInfo are preserved unchanged. All serialized message payloads and all
non-CameraInfo receive timestamps are retained. The lead independently checked
both bags with `verify_retimed_inputs.py`: identical counts/payloads/timestamps
under this rule, zero missing or extra entries. Original calibration delivery
lags images by more than 200 ms on 187 messages (maximum 317 ms).

`run_pnp3_calibration_control.py` runs the unchanged full-stack recorded-timing
replay twice with that derived input bag. Effective odometry YAML is identical
between the earlier baseline and both controls:

| Recorded timing | statuses | lost | poses | pose Hz | largest selected stamp gap |
| --- | ---: | ---: | ---: | ---: | ---: |
| Original CameraInfo delivery | 1386 | 22 | 1360 | 12.6 | 367 ms |
| CameraInfo delivered with image | 1773 | 0 | 1772 | 16.4 | 167 ms |
| Same control repeat | 1740 | 0 | 1739 | 16.1 | 167 ms |

The baseline valid-pose outage is 2.71 s; neither control has any outage above
the report's 0.5 s threshold. Maximum adjacent raw steps are 126 and 118 mm,
and final pose age is zero in both. Maximum raw tuple-arrival spread falls from
318 ms to 112/111 ms. This supports calibration arrival delay as a cause of
degradation in recorded-timing replay. It does not establish a live fix, explain
all of the original live 21 s outage, or provide valid return-hold accuracy.
The driver and estimator defaults are unchanged. All replay graphs are stopped.

Evidence in the September 13 audit directory:
`pnp3_retry_{calibration_validation,retimed_integrity}.json` and
`pnp3_live_retry30_calibration_{retimed,retimed_repeat}_{report,summary,bursts}.json`,
plus commands/effective YAML/logs. Python compile checks, independent bag integrity
verification, parameter equality checks and `git diff --check` pass. The earlier
implementation test suite was not rerun for these diagnostic-script changes.
One Luna worker independently audited semantic calibration; the lead reviewed
the control, verified the derived bag and ran/reviewed both replays.

### Opt-in immutable calibration cache tested on the original bag

Implemented and rebuilt **only the isolated `rtabmap_latest_frame_overlay`**.
`rtabmap_odom` now declares `cache_camera_info`, default false. Existing launch
files can opt in with `RTABMAP_CACHE_CAMERA_INFO=1`; explicit parameter overrides
win. No project launch/default, stock `/opt/ros/jazzy` dependency or PX4 graph
was changed. Raw image/depth use a two-input synchronizer with the existing
approximate/exact settings and interval; CameraInfo is observed independently.

`ImmutableCameraInfo.h` validates finite calibration, positive focal lengths,
dimensions and frame ID, and compares every semantic field except stamp. The
first valid calibration is immutable. Changed/invalid calibration, image/depth
frame or dimension mismatch, and non-increasing accepted image/depth times
latch input closed until restart. Frames older than the first calibration are
rejected. Cached copies are restamped without mutating source messages.
Odometry reset retains cache identity, faults, timestamp floor and bounded pair
queues; it does not reconstruct a synchronizer from the reset service thread.

This is explicitly a **fixed-calibration contract**, not a general dynamic
calibration solution or safety interlock. There is no expiry after CameraInfo
disconnects, a delayed change cannot be detected retroactively, and an already
queued/in-flight odometry update may finish after a fault. Keep it off PX4.
Implementation/contract/tests are in `rtabmap_odom/{include/rtabmap_odom/ImmutableCameraInfo.h,
src/nodelets/rgbd_odometry.cpp,test/}` in the overlay source tree. CMake and
package.xml register the new gtest dependency/target. Existing timing and
latest-pending modifications are preserved.

`run_pnp3_cached_calibration.py` replays the **untouched original retry bag** at
its original receive timing twice, with full mapping, no camera and no PX4.
Both effective parameter dumps equal the old original-timing baseline except
for the new cache flag:

| Original-bag path | statuses | lost | poses | pose Hz | largest selected stamp gap |
| --- | ---: | ---: | ---: | ---: | ---: |
| Earlier triple-sync baseline | 1386 | 22 | 1360 | 12.6 | 367 ms |
| Immutable cache | 1706 | 0 | 1705 | 15.7 | 200 ms |
| Immutable cache repeat | 1713 | 1 | 1710 | 15.8 | 167 ms |

Neither cache run has an output gap above 0.5 s; maximum adjacent raw steps are
104/113 mm and final pose age is zero. Both have zero cache rejections. In
993/982 exact-stamp joins, synchronized pairs are accepted **before** their
matching CameraInfo arrives, by up to 313.5/314.3 ms. Maximum synchronization
wait after the image/depth pair arrives is 3.01/3.09 ms. This directly verifies
that calibration delivery no longer gates this replay input path.

The repeat still **fails the zero-dropout moving gate**: one 0.13 s loss at
bag-relative 100.1 s. Its core stamp is `1789284275230769296` (8 inliers), the
**same frame** as the all-frame serial control's sole loss, sequence 2997,
sensor-relative 99.951762 s (10 inliers). This remaining issue is not unique to
the cache. Neither replay qualifies live motion or return-to-origin accuracy;
the original hold-cue caveat still applies. The original live 21 s outage has
not been reproduced in full or proven fixed live.

Lead verification: final `colcon build --packages-select rtabmap_odom --executor
sequential --parallel-workers 1 --allow-overriding rtabmap_odom --cmake-args
-DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON` passes (existing PCL/overlay
warnings). `colcon test --packages-select rtabmap_odom` passes 1 CTest target
containing 7 gtests; `colcon test-result --test-result-base build/rtabmap_odom
--verbose` reports zero failures. Input-free standalone-node smoke checks in
ROS domain 142 verify default cache=false, enabled exact-sync startup, and
explicit false overriding environment=1. Both replay runs, parameter comparison,
Python compile checks and both worktrees' `git diff --check` pass. The broader
project suite was not rerun; its prior 31 CTest targets / 627 tests are historical.
One Luna low-effort worker performed read-only review; lead implemented,
resolved reset/diagnostic concerns, built, ran and analyzed the checks.

Artifacts in the September 13 audit directory:
`pnp3_live_retry30_{cache,cache_repeat}_{commands.json,effective.yaml,report.json,summary.json}`,
logs, `pnp3_cached_calibration_comparison.json`, and
`analyze_cached_calibration.py`. Output bags are
`/home/john/ros_parity_pnp3_live_retry30_{cache,cache_repeat}_20260913`.
All task replay and smoke-test processes are stopped.

### Remaining dropout localized to one dark RGB/depth input

Lead inspected the original images/depth around sequence 2997, not just odometry
logs. Its mean grayscale intensity collapses from **85.30 to 14.17**, then
recovers to **73.57** on the next frame. Nonzero finite depth coverage falls
from **32.80% to 5.91%**, then recovers to **27.39%**. Image stamps remain
33.328 ms apart. The original image visibly becomes dark/noisy for that frame;
this is not solely an odometry transport problem or evidence to lower MinInliers.
Simple image-quality feature counts are diagnostic OpenCV GFTT, not the
estimator's exact feature set. Laplacian variance is elevated by noise and must
not be interpreted as evidence of good image quality.

`audit_dropout2997.py` uses the canonical tuple matcher and explicitly excludes
the initialization tuple to preserve serial sequence numbering. Every inspected
stamp is asserted equal to the original serial pose row. Results and a contact
sheet: September 13 audit `dropout2997/{inputs.json,neighbors.jpg}`.
`scan_carry_brightness.py` separately scans all **3253 images / 3254 depths**.
Sequence 2997 has the lowest intensity relative to its two neighbors, **0.178**;
it is the only ratio below 0.4. Five other transient dips have ratios roughly
0.409--0.479. These are descriptive observations, not proposed runtime gates.
Full scan artifacts: `dropout2997/whole_bag_{quality,summary}.json`.

Two fresh-map stock-RTABMap serial controls use sequences 2700--3050, the original
bag and unchanged PnP-3 settings:

| Window input | statuses | lost | poses | largest valid sensor-time gap |
| --- | ---: | ---: | ---: | ---: |
| All 351 frames | 351 | 1 | 348 | 100 ms |
| Omit only sequence 2997 | 350 | 0 | 349 | 67 ms |

The window's sole loss is exactly stamp `1789284275230769296`, now with zero
inliers. Thus the dark frame can fail with a fresh recent map, without the
earlier long trajectory. Both finish with zero final pose age. Lead verified
effective parameter equality between both windows and the original all-frame
serial run, and exact selection equality after removing only sequence 2997.
The omission is a diagnostic counterfactual, **not a stream acceptance pass or
implemented input filter**. Do not hide bad frames or relax acceptance gates.
Core `log_info` was requested via service, but no detailed RegistrationVis
rejection message appeared; do not claim a captured PnP rejection trace.

Runners: `run_dropout2997_{window,skip}.py`; selections:
`dropout2997_{window,skip}_selection.jsonl`; launch/replay logs and effective YAML:
`host_cost_pnp3_dropout2997_{window,skip}_*` in the audit directory. Output
directories: `/home/john/ros_parity_acceptance2_host_cost_pnp3_dropout2997_{window,skip}_20260913`.
Each contains complete report.json and poses.jsonl. Original bags are unchanged.

One Luna low-effort worker reviewed logs and local camera-driver controls; the
lead verified relevant source/config, inspected the images, ran both controls
and compared parameters/selections. Nonzero `quality` does not imply tracking
success: the earlier serial frame had quality=10 with lost=true and MinInliers=20.
Local sensor_param_handler.cpp defaults `r_set_man_exposure=false`; declared
manual exposure/ISO values are not evidence of actual per-frame auto exposure.
The selected camera profile does not explicitly override exposure. The recorded
sensor_msgs/Image fields do not contain exposure/ISO telemetry, and no such
per-frame telemetry topic was recorded. **Auto exposure, lighting flicker,
occlusion or a device/driver event remain hypotheses, not established causes.**

Python compilation, exact parameter/selection comparisons and `git diff --check`
pass. Image analysis initially hit user-site NumPy 2 versus system OpenCV ABI
incompatibility; rerunning with `PYTHONNOUSERSITE=1` succeeds without changing
installed packages. Use that environment for both image-analysis scripts.
No runtime code, launch defaults, camera settings or estimator thresholds were
changed in this investigation. No package rebuild was needed for audit scripts.
All task replay processes are stopped; the camera and PX4 were not used.

### Full DepthAI stack also fails exactly frame 2997

At the user's request, replayed this exact original retry bag through DepthAI
3.5's device Harris/KLT FeatureTracker -> RTABMapVIO -> RTABMapSLAM, all frames,
serial, IMU off, default native odometry parameters. This uses the OAK device
for calibration and feature processing, but creates no live Camera node or
ROS/PX4 publishers. Runner: September 13 audit
`run_depthai_pnp3_retry_full_stack.py`; command/log artifacts:
`depthai_pnp3_retry30_full_stack_commands.json` and
`depthai_pnp3_retry30_full_stack.log`.

Complete output: `/home/john/depthai_pnp3_retry30_device_full_stack_20260913`.
**3250 VIO outputs, exactly one invalid, 3250 SLAM outputs, 433 grid outputs**.
The only invalid native output is sequence **2997**, stamp
`1789284275230769296`: zero translation and quaternion (0.5,0.5,0.5,0.5), the
previously validated SDK failure sentinel. SDK exposes no explicit lost bit;
the replay validity rule excludes this sentinel. It recovers at sequence 2998.
Maximum valid raw-pose sensor-time gap is **66.66 ms**, final valid pose age
zero, largest adjacent valid raw step **97 mm** (corrected SLAM **104 mm**).
Invalid sentinel poses are excluded from motion-step metrics.

`compare_depthai_retry2997.py` verifies all **3250 sequence/stamp pairs** exactly
match the ROS all-frame serial control. Both have the same sole failed frame,
2997; `depthai_pnp3_retry30_comparison.json` contains the joined neighborhood.
Thus this remaining isolated dark-frame dropout is shared by both visual
stacks, not unique to the ROS migration. This does **not** show the native live
camera pipeline would generate the same dark frame, reproduce the original
21 s live ROS outage, or establish real-time/flight parity. These are serial
saved-input controls with different native/ROS feature and estimator settings.

Lead ran and verified this comparison without workers. Replay completed/closed
cleanly; Python compile checks, exact stamp join and `git diff --check` pass.
Only audit scripts/artifacts and this handoff changed; no production defaults,
camera controls or estimator thresholds changed. OAK replay is stopped.

### User parity criterion and incomplete cache-enabled live attempt

The user explicitly accepts the isolated dark-frame failure shared with native
DepthAI and wants migration parity now. **Do not pursue exposure/ISO diagnostics
or impose a zero-loss criterion stricter than the native same-input baseline.**
This does not waive ROS-specific multi-second output stalls or authorize flight.

User approved a fresh cache-enabled carry. Runner:
`/home/john/ros_parity_optimizer_audit_20260913/run_live_pnp3_cache_carry.py`.
It reuses the prior 110 s capture, verifies the isolated overlay and effective
`cache_camera_info=true`, enables `RTABMAP_TIMING_AUDIT=1`, uses latest-pending,
30 Hz input, PnP-3, IMU off/reset countdown 0, full mapper/OctoMap and no PX4 node.
Free disk was 3.6 GiB before capture. The graph warmed up and recording started.

**Capture is incomplete and excluded from acceptance**:
`/home/john/ros_parity_live_motion_host_pnp3_cache30_20260913` contains a 1.8 GiB
MCAP, metadata.yaml and incomplete.json. Recorder logs report 9756 cache-buffer
message losses (including 801 images, 801 depth, 804 calibration, 176 statuses,
175 odometry and 5410 IMU messages). Shutdown exceeded the recorder's 20 s wait
and subsequent 5 s termination wait; its wrapper exited 1. Camera/mapper
processes also required launch shutdown escalation. All task processes are now
stopped. No data was deleted; free storage afterwards is about 1.8 GiB.

The assistant relayed MOVE NOW, but received RETURN SOON and RETURN/HOLD together
with the recorder failure after capture ended and did not relay them in time.
The assistant then told the user to stop/set down safely. Do not describe this
as a correctly cued carry or use return-hold accuracy. Recorded raw pose span
was only millimeters (net 1.37 mm, RMS 0.64 mm); actual user motion is unconfirmed.

Partial bag report records 847 statuses, zero lost, 844 poses and a 17.5 s
recorded output gap; severe recorder loss means that gap is not the exact live
gap. Independent CPU/topic sampler received 1048 statuses (zero lost) and 1046
poses, with **6.235 s maximum receive gap**. Timing trace independently shows a
6.232 s selected sensor-stamp gap. Crucially, core processing peaks at 186 ms,
while **core-end to worker-exit reaches 6.150 s**. There are zero cache rejections
or observed calibration faults. This points to post-core output/logging work,
not a demonstrated tracking failure or delayed-calibration gate; exact blocking
operation remains unproven. Do not claim the cache establishes live parity.
No thermal throttling was reported. Kernel log has normal OAK boot/disconnect
around graph startup/shutdown, no inspected mid-run disconnect.

Artifacts use `host_live_projection_pnp3_cache_carry30_` in the September 13
audit directory: commands, effective YAML, launch/record logs, CPU JSON,
`partial_report.json` and `summary.json`. The latter is generated by existing
`summarize_pnp3_timing.py`; the partial report by `scripts/ros_rtabmap_bag.py report`.
No production defaults or estimator thresholds changed.

User requested a command to operate the test directly. Added
`scripts/run_ros_parity_carry.sh` and `scripts/ros_parity_local_carry.py`.
Run `bash /home/john/autonomous_drone_px4_vio/scripts/run_ros_parity_carry.sh`
in an interactive Pi terminal. This sources the isolated overlay, enables cache,
disables high-volume timing traces, warms up, verifies parameters, waits for
Enter and prints local one-second countdowns: 5 s recorder discovery, 20 s
still, 35 s motion, 15 s return, 20 s still. New timestamped output directories
preserve logs, parameters, cue timestamps, report and incomplete marker on failure.
It launches the same camera/VIO/SLAM/OctoMap graph but records **only odometry,
lite status, map info and static TF**, to reduce disk load. No raw image/depth
bag: this is a lightweight live-output diagnostic, not same-input replay parity.
It requires free disk headroom and does not delete anything. Recorder flush is
allowed up to 90 s before escalation. Syntax/compile/help checks pass; this new
runner has **not been live-tested** and is not proof the stall is fixed.

### Clean RAM-recorded live carry and native same-bag comparison

The user completed the stationary RAM-recording check and then the motion run
locally. Both archives verified and temporary RAM copies were removed. The
stationary archive is `/home/john/ros_parity_local_carry_20260913T102037Z/capture`:
2819 images/depths each, 1358 statuses/poses, zero lost, 14.5 Hz and no output
gaps above 0.5 s. It is stationary evidence only, not motion/loop-closure accuracy.

The clean carry is `/home/john/ros_parity_local_carry_20260913T102714Z/capture`.
No recorder cache drops were reported. It has 2791 images/depths each, 2786
CameraInfo and 2782 feature messages, plus IMU/static TF and compact outputs.
ROS live: **1366 statuses/poses, zero lost, 14.7 Hz**, maximum measured receive
gap **155.229 ms**, maximum sensor-stamp gap **133.311 ms**, final pose age 0.03 s.
Both hold windows are stable; report raw return offset is **280 mm**. There
are 3 global closures and 3 proximity detections, but raw offset is not repaired
by map corrections. Cache/latest-pending/PnP-3/reset0 verified in effective YAML;
timing tracing off, recording in RAM, full live camera/VIO/mapper/OctoMap graph.

At user request, lead replayed this same bag through unchanged DepthAI 3.5
device Harris/KLT FeatureTracker -> RTABMapVIO -> RTABMapSLAM, every matched
tuple, one in flight, no IMU. Output:
`/home/john/depthai_local_carry102714_device_full_stack_20260913`.
**2775 outputs, zero invalid**, 2775 SLAM outputs and 384 grid outputs, maximum
valid sensor-time gap 66.656 ms and zero final pose age. The native input matcher
has 2776 tuples and 46 unmatched messages, maximum tuple skew 314 ns. No live
camera or ROS/PX4 publishers were created for replay. The OAK replay is stopped.

Lead compared identical first/last 10-second **sensor-time** hold windows:

| Metric | ROS live latest-pending | Native serial full stack |
| --- | ---: | ---: |
| Failed status / invalid pose | 0 | 0 |
| Raw start/end hold displacement | 280.03 mm | 293.61 mm |
| Start/end hold span | 3.41 / 2.07 mm | 6.80 / 3.80 mm |
| Maximum adjacent valid raw step | 154.45 mm | 71.55 mm |

**For this carry, tracking continuity and raw return displacement are comparable
to native** under the user's parity criterion. The roughly 28 cm ROS offset is
not a demonstrated migration-specific regression: native has a similar offset.
These are measured pose differences, not ground-truth drift; physical placement
error and SDK camera/body-frame conventions remain caveats. Different sample
density explains why adjacent steps are not directly matched increments.
There are 1359 exact common processed stamps; 7 ROS stamps are absent from the
native matched-input replay. Native serial execution does not establish equal
real-time CPU use, frame selection, mapping accuracy, broader reliability or
flight qualification. Do not claim universal parity from one carry.

Artifacts in the September 13 audit directory:
`run_depthai_local_carry102714.py`, `depthai_local_carry102714_full_stack{_commands.json,.log}`,
`compare_local_carry102714.py`, `local_carry102714_native_comparison.json`.
Native report and poses.jsonl are in its output directory. Archived ROS report
still names its original `/dev/shm/.../bag` provenance; use the persistent bag
path above, not that deleted RAM path. Lead ran the replay, comparison, Python
compile checks and diff checks; no worker or production setting change needed.

Next: retain this reproducible candidate and result as the first clean
cache-enabled live continuity/native-return comparison. Any request to promote
defaults or connect PX4 requires separate authority; native parity remains the
objective, not removal of the shared single-frame dark-input failure.

### Earlier recorder failures and RAM staging implementation (historical)

Earlier full-input locally cued attempt:
`/home/john/ros_parity_local_carry_20260913T101237Z` stopped cleanly but recorder
reported **8476 dropped messages**, including 768 left images and 766 depths.
Raw bag (~1.5 GiB) and logs are preserved, incomplete/acceptance-failed. There
was 8 GiB free afterward: this was not disk exhaustion. Partial data shows
15/900 lost statuses, recovery and zero-ending-age approximately (0.02 s), but
gaps are confounded by missing recordings. SD write throughput/backpressure is
a hypothesis, not proven by a dedicated storage benchmark.

User approved RAM staging preparation. `run_ros_parity_carry.sh --record-inputs
--ram --stationary` now runs a stationary recorder validation with the same
full live graph and local countdown. `--ram` stages logs/bag/results in an owned
`/dev/shm/ros_parity_carry_*` temporary directory; requires capture headroom in
tmpfs and available memory plus a 1 GiB reserve, and monitors remaining RAM.
After graph shutdown, copies to the persistent result directory's `capture/`,
verifies SHA-256 of every file, fsyncs files, then removes only the verified RAM
copy. Existing archives are never overwritten. Copy failure preserves source
RAM/partial disk data with a do-not-reboot warning. Even failed captures are
archived with incomplete markers. Wait for `ARCHIVE VERIFIED` before closing
the terminal/rebooting. RAM data alone is volatile. Omit `--stationary` only
for a subsequent carry after the recording path is validated.
Six hardware-free command/archive tests pass (including corruption and existing
destination guards), wrapper help and diff checks pass. No live RAM recording
was run by the assistant; throughput and end-to-end hardware remain unvalidated.

Subsequent locally operated outputs-only capture
`/home/john/ros_parity_local_carry_20260913T100037Z` completed cleanly, but tracking
failed about 10 s after MOVE NOW: 557/1047 statuses lost, 487 poses, 60.12 s
final pose age and no recovery. Cache/latest-pending were enabled; no cache
fault was found. This is not the accepted shared single-frame dropout. There
are no raw inputs for a same-bag native comparison.

User approved preparing the next capture with raw inputs. The same local command
now accepts `--record-inputs`: records left image, depth, CameraInfo, features,
IMU and static TF plus the prior compact outputs, with existing input QoS
overrides. It requires 3 GiB free on the output filesystem, stops below 512 MiB,
and verifies input presence after recording. Timing tracing stays disabled;
normal graph settings/countdown are unchanged. Default remains outputs-only.
Two hardware-free recorder-command tests pass, plus wrapper help and diff checks.
No new camera run was started by the assistant. Command:
`bash /home/john/autonomous_drone_px4_vio/scripts/run_ros_parity_carry.sh --record-inputs`.

### Approved storage cleanup

On September 13 the user approved permanent deletion of five September 7--9
motion bags. Removed **only their raw MCAP files** from these directories:
`ros_parity_carry_20260907_01`, `ros_parity_live_motion_10hz_20260909`,
`ros_parity_live_motion_cued_10hz_20260909`,
`ros_parity_live_motion_oak20_min10_cued_20260909`, and
`ros_parity_live_motion_oak15_min10_cued2_20260909` under `/home/john`.
Reports, metadata, parameter/calibration snapshots and extracted images remain;
each directory has `RAW_DATA_DELETED.txt` and is no longer a replayable bag.
Free disk increased from 1.8 to 9.4 GiB. September 12--13 bags are untouched.
Read-only large-file scan also found the derived retimed bag (2.4 GiB),
`ros_parity_optimizer_audit_20260908/direct_core/frames.bin` (1.9 GiB),
Trash (1.4 GiB), user cache (562 MiB) and cached VSIX files (385 MiB).
These additional files were **not deleted**; no further cleanup was authorized.

Historical next step (superseded by the clean RAM-recorded comparison above):
diagnose post-core stalls/recorder backpressure before repeating a full-input capture. Resolve
storage headroom with user approval (do not delete preserved bags). Future
motion phases must not depend on timely chat delivery: agree a user-visible
local timer or user-acknowledged phase protocol before capture. Confirm whether
the rig actually moved during this failed attempt. Native parity, not eliminating
the shared dark-frame dropout, remains the objective; no PX4 connection.

Reproduce the experimental live PnP-3 graph (no recorder, no PX4):

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
source /home/john/rtabmap_latest_frame_overlay/install/local_setup.bash
# Must resolve to the isolated overlay, not /opt/ros/jazzy:
ros2 pkg prefix rtabmap_odom
ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_native_features.launch.py \
  foxglove:=false use_imu:=false odometry_rate_hz:=0.0 \
  odom_reset_countdown:=0 keep_latest_frame:=true \
  odom_key_frame_thr:=0.3 odom_vis_keyframe_thr:=150 \
  odom_config_path:=/home/john/ros_parity_optimizer_audit_20260913/host_projection_pnp3.ini \
  camera_config:=/home/john/autonomous_drone_px4_vio/ros2_ws/src/px4_vio_bridge/config/oak_v3_rtabmap_tracked.yaml
```


## September 12 processing-cost continuation

The next session followed the host-feature keyframe/update-cost recommendation.
Full-stack acceptance2 replays used explicit 1000-feature/2000-point-map host
odometry, BA enabled, IMU off, reset countdown 0, separate mapper/native OctoMap,
and deterministic stride-2 input at approximately 15 Hz. They used stock
`/opt/ros/jazzy` odometry. No defaults have been promoted.

| Host variant | statuses | lost | status Hz | max pose gap | median core update |
| --- | ---: | ---: | ---: | ---: | ---: |
| unchanged baseline at 15 Hz input | 836 | 27 | 9.64 | 4.400 s | 61.3 ms |
| `Odom/VisKeyFrameThr=50` | 814 | 47 | 9.38 | 6.731 s | 68.2 ms |
| additionally `Odom/KeyFrameThr=0.1` | 1021 | 48 | 11.77 | 6.665 s | 52.7 ms |

The combined thresholds reduced keyframe addition from 48.0% to 8.3% of
processed frames but worsened continuity. Neither threshold variant passes.
`OdomF2M/MaxNewFeatures=100` timed out in replay transport after 150 seconds;
its output directory contains `incomplete.json`, and it is excluded from the
comparison. All variants retain effective parameter dumps and commands in
`/home/john/ros_parity_optimizer_audit_20260908/host_bounded_*`. Completed
output directories begin `/home/john/ros_parity_acceptance2_host_bounded_`.
The consolidated timing report is `host_bounded_summary_sep12.json`.

`scripts/ros_rtabmap_replay_realtime.py` now saves per-status core update time,
local BA time/constraint count, local-map/keyframe counts and keyframe-added
status from the existing lite OdomInfo subscription. This adds no new topic
subscription or estimator behavior.

A healthy stationary live 30 Hz source/uncapped-host-odometry control used the
unchanged feature/map budgets and produced 450/450 valid updates at 9.993 Hz,
0.889 mm position RMS, 0.669 mm net displacement and 140.2 ms median pose age.
Camera+odometry used 108.55% CPU, mapper 18.65%, OctoMap 1.91% (129.11% total,
100%=one core), with no throttling. Core update time was 81.5 ms median and
86.5 ms p95; BA was 18.9 ms median, with a 2000-point map and 313 median
inliers. **No odometry keyframes were added in this 45-second window**, so
ongoing local-map insertion does not explain its stationary cost. Artifacts
are `host_live_baseline30_profile_{cpu,timing,commands,effective,launch}.*` in
the optimizer audit directory. The later log-debug service call did not emit
internal core debug traces in this component build; do not claim it profiled
individual feature/matching stages.

The second full-stack 15 Hz-input sweep produced:

| Host variant | statuses | lost | status Hz | max pose gap | max pose step |
| --- | ---: | ---: | ---: | ---: | ---: |
| `g2o/Solver=0` (CSparse available in ROS) | 769 | 0 | 8.87 | 0.267 s | 178 mm |
| `Vis/CorGuessMatchToProjection=true` | 968 | 0 | 11.17 | 0.267 s | 236 mm |
| `Odom/ImageDecimation=2` | 853 | 50 | 9.83 | 6.665 s | 171 mm |

The solver and correspondence variants each preserve continuity once but do
not meet the rate target. Half resolution alone drops median inliers to 88
and does not pass. Commands, effective parameters and logs use `host_cost_*`
in the optimizer audit directory; outputs begin
`/home/john/ros_parity_acceptance2_host_cost_`. Current results are consolidated
in `host_cost_summary_sep12.json`. No variant is promoted.

An isolated direct-core diagnostic in the audit directory's `host_cost_core/`
uses installed RTAB-Map 0.22.1 and the earlier September 7 exported frames.
Its 200-frame host-feature stationary prefix has zero losses, a median 55.0 ms
core update, 37.3 ms registration and 16.3 ms BA, with an 882-point local map.
Console debug output on the last five frames shows roughly 10–13 ms keypoint
extraction, 5 ms descriptors, 9 ms spatial radius search and 6 ms descriptor
copy/matching. This is stage-cost evidence on that prefix, not a new
acceptance2 result or live CPU benchmark. Source/config/build/logs are retained;
no installed library was changed.

### Replay deadline bug found and fixed

A reliable-QoS `Optimizer/Iterations=10` attempt also timed out after 150 s,
with the estimator idle and source clock stopped. A subsequent best-effort
baseline stalled too. Its watchdog repeatedly located the replay thread in
`rclpy`'s `wait_set.wait()`, called by the frame-deadline wait at the former
line 190. The loop read `time.monotonic()` once for the condition and again for
the timeout. If the frame deadline passed between reads, it supplied a negative
ROS timeout, which means **wait forever**. This is a concrete replay-tool race;
changing QoS did not fix it. It can explain interrupted replay transport and
must not be attributed to a particular estimator setting. It does not explain
live-camera tracking loss, and completed historical results remain their
recorded evidence.

`spin_until_deadline()` now samples remaining time once per iteration and
never passes a negative timeout to ROS. A deterministic regression test crosses
the deadline between successive clock samples; another verifies bounded
callback servicing and late-frame behavior. The focused replay/bag tests pass
20/20. The new test is registered with CTest. Incomplete runs in this
continuation retain `incomplete.json` and must remain excluded from comparisons.

The next sweep uses the live graph's best-effort sensor QoS (2), a fresh baseline
and unique `_qos2_fixed_20260912` result paths. It compares fewer optimizer
iterations and combinations with correspondence direction/image decimation.
Candidate acceptance still requires complete real-time continuity and a healthy
live rate; none above reaches the 15 Hz target. IMU remains off and reset
countdown remains zero.

## Resume here — September 12 live host-feature retry

The Pi was accidentally powered off during the first requested host-feature
carry. Preserve `/home/john/ros_parity_live_motion_host_candidate_20260912` as
an **incomplete** bag. After reboot, the graph and recorder were restarted from
scratch and the completed retry is:

`/home/john/ros_parity_live_motion_host_candidate_retry_20260912`

The retry used `ros_rtabmap_oak_v3_native_features.launch.py` with its exact
10 Hz/1000-feature/2000-point-map defaults, no IMU and no PX4 node. The bag has
884 matched RGB/depth/OAK-feature tuples over 88.58 s, 655 odometry statuses,
178 lost updates in three episodes, and only 474 valid raw poses. The terminal
loss starts at 53.4 s and lasts 34.76 s; no valid pose is produced during the
ending hold, so the final pose is stale by 35.09 s. The largest raw-pose step
is 318.79 mm after a 0.80 s gap. There are no closures because odometry fails
before completing the route. Canonical report:
`/home/john/ros_parity_optimizer_audit_20260908/live_host_candidate_retry_report_sep12.json`.

The simultaneous 60 s moving CPU sample measured camera+host-odometry 72.2%,
mapper 5.8% and native OctoMap 0.4%, **78.3% total**, with no throttling. That
low number is not a success: valid pose output averaged only 0.98 Hz during the
sample because tracking was lost for most of it. Do not use it as the healthy
ROS-versus-DepthAI efficiency comparison. Artifact:
`/home/john/ros_parity_optimizer_audit_20260908/live_host_candidate_retry_cpu_sep12.json`.

### Same-bag controls locate the failure

A literal DepthAI 3.5 full-stack replay used a fresh on-device 400-point
Harris/KLT FeatureTracker linked directly to `RTABMapVIO`, followed by
`RTABMapSLAM`, at stride 1. It completed 883 VIO and 883 SLAM outputs, but
**352 VIO outputs were invalid (39.9%) and its final valid-pose age was the
same 35.0993 s**. Maximum adjacent valid raw and SLAM steps were both 112 mm.
Thus the full DepthAI stack does not do well on this particular carry either.
Result:
`/home/john/depthai_live_motion_host_candidate_retry_device_features_full_stack_20260912`.

Standard ROS host odometry in one-frame-in-flight serial replay processed every
tuple but likewise returned 351/883 lost and the identical 35.0993 s stale
ending. Result:
`/home/john/ros_parity_live_motion_host_candidate_retry_serial_host_20260912`.
This rules out live ROS frame dropping as the cause of the terminal failure,
although dropping did amplify earlier short gaps.

Offline image/depth/OAK-track/IMU inspection identifies the trigger. At 53.50 s
the median surviving-track displacement was **106 px per 100 ms** (158 px 90th
percentile), only **21 feature IDs** survived from the previous frame, whole
image valid depth was 21.0%, and only **17.3% of tracked features had valid
depth**. Peak gyro magnitude around this section was 1.73 rad/s. Earlier the
route also crossed severe exposure changes: one frame was 75% near-black and
another 49% near-white. Both estimators lose at this same abrupt transition and
cannot reattach to their F2M local maps when `Odom/ResetCountdown=0` preserves
the flight-critical odometry frame.

Increasing `Vis/CorGuessWinSize` from 40 to 160 did not help. A real-time host
replay still ended stale by exactly 35.0993 s, with 177/705 lost statuses and
178 of 883 selected frames dropped before status. Result:
`/home/john/ros_parity_live_motion_host_candidate_retry_replay_window160_20260912`.
Do not spend another carry on a wider correspondence window.

### Rate/CPU probes and next experiment

Changing the live source to 30 Hz, requesting 20 Hz odometry and reducing host
features from 1000 to 600 did **not** raise pose rate. The estimator still
published 9.987 Hz because 69-77 ms updates caused the single odometry worker
to select roughly every third camera frame. It cost camera+odometry 101.8%,
mapper 19.4%, and OctoMap 2.4% (**123.6% total**) while stationary, with
0.77 mm pose RMS and no throttling. Artifact:
`/home/john/ros_parity_optimizer_audit_20260908/live_host_600_requested20_stationary_cpu_sep12.json`.
Making the camera/odometry component container multithreaded did not change the
approximately 10 Hz output; RTAB-Map's odometry worker is the serialization
point. A brief 400-feature probe became slower as the local map matured
(roughly 110-125 ms/update, about 7-8 Hz, 116-142 inliers), so do not assume
lower `Vis/MaxFeatures` improves throughput.

**Recommended next work:** do not immediately request another carry. First
profile why BA/F2M update time grows when the map matures and test a bounded
configuration on the retained *successful* acceptance2 bag and stationary live
graph. The useful target is at least 15 Hz sustained, zero losses on
acceptance2, and healthy stationary inlier margin. Candidate levers not yet
isolated for the host path are BA cost/update frequency (`Odom/KeyFrameThr`,
`Odom/VisKeyFrameThr`, `OdomF2M/MaxNewFeatures`) rather than another feature
count or map-size sweep. Only after one reaches the rate target should a fresh
carry be run, with slower turns than the failed retry and deliberate avoidance
of near-black/near-white transitions. Preserve `Odom/ResetCountdown=0`; an
automatic reset may recover visually but creates an origin discontinuity that
cannot be sent directly to PX4.

## September 12 fresh live carry gate

The requested fresh moving acceptance was run with the tracked wrapper's exact
promoted defaults: `use_imu=false`, `separate_mapper_container=true`,
`odometry_rate_hz=0`, `oak_input_stride=2`, `Vis/MinInliers=10`,
`Odom/ResetCountdown=1`, `OdomF2M/MaxSize=1000`, BA enabled, mapper at 1 Hz and
native OctoMap with marker traversal disabled. Runtime parameter queries
confirmed those values. The live graph contained only the OAK driver, tracked
ROS odometry, ROS RTAB-Map, native OctoMap, state publisher and containers; no
PX4-facing node was present.

Two new raw bags are retained. The first is
`/home/john/ros_parity_live_motion_oak_promoted_acceptance_20260912`; it has
1085 poses, 1/1085 lost status (0.1%), three output gaps up to 1.445 s, a 330 mm
maximum step, and 21 global plus 2 proximity closures. The repeat is
`/home/john/ros_parity_live_motion_oak_promoted_acceptance2_20260912`; it has
1035 poses at 12.0 Hz, 6/1047 lost statuses (0.6%), four output gaps up to
0.970 s, a 271 mm maximum step, and 15 global plus 1 proximity closure. Its
mapper published 81 updates and OctoMap 82. Final OctoMap generation 120 used
48 source nodes and contained 12,684 ground, 27,628 obstacle and 850,637 empty
cells. The corresponding adjacent `_report.json` files are the canonical
reports.

Both ending-hold comparisons are invalid because Codex's tool/UI polling
batched the return/hold cue until late; the recorder itself already prints each
cue with `flush=True`. This prevents a return-to-origin conclusion, but does
not affect the independently timestamped moving continuity failures. Do not
describe either bag as a correctly cued closed-loop acceptance.

Timestamp correlation rules out the OAK input path as the cause. During the
first run's 1.445 s odometry gap, 43 left images, 44 depth images and 43 OAK
feature messages arrived; the feature-count range was 130-255. All seven gaps
across the two live runs retained approximately 30 Hz image/depth delivery and
substantial feature counts. Failed F2M registrations took roughly 0.15-0.18 s;
the longer pose pauses combine registration failure with subsequent selected
tuples omitted while the worker recovers.

### OAK FeatureTracker consumption is now proven for the custom ROS path

This statement applies to `px4_vio_bridge::OakTrackedOdometry`, not to the
native DepthAI `RTABMapVIO` supplied-feature replay whose consumption remains
unproven. The September 12 carry bag contains 2604
`depthai_ros_msgs_v3/msg/TrackedFeatures` messages from
`/oak/left_rect_feature_tracker/tracked_features`. The tracked launch loaded
`liboak_tracked_rgbd_component.so` and instantiated
`px4_vio_bridge::OakTrackedOdometry`, not standard `RGBDOdometry`. Its
four-input synchronizer requires that feature message, and the callback copies
each `features[].position` into `cv::KeyPoint` and calls
`SensorData::setFeatures()` before `processData()`.

A fresh perturbation control establishes actual use rather than mere topic
gating. Serial replay of the same first 50 stride-2 image/depth tuples with the
original OAK locations produced **50/50 OdomInfo acknowledgements, 49 poses and
0 losses**; its first tracked frames had 295-312 supplied OAK points and
139-164 odometry inliers. Holding images, depth, calibration and feature count
constant while collapsing every supplied point to the image center produced
only the initialization acknowledgement and no pose; the second update stayed
inside F2M for more than five seconds until controlled shutdown. Results are
`/home/john/ros_parity_oak_feature_proof_original_20260912` and
`/home/john/ros_parity_oak_feature_proof_center_20260912`; matching logs are in
`/home/john/ros_parity_optimizer_audit_20260908`. The replay-only diagnostic
switch is `--oak-feature-control`; its default is `original`.

### Serial versus real-time isolation and motion-prediction probes

One-frame-in-flight replay of the second carry is not yet a completed full-bag
result. With exact defaults, every selected frame was valid through output 704,
then the update at approximately 47.1 sensor seconds remained in F2M for over
60 seconds. The immediately preceding update had only 13 inliers. This is a
deterministic pathological optimizer/registration transition in the selected
visual sequence, so dropped or irregular real-time delivery is not the only
cause of this carry's gaps. The replay now accepts `--ack-timeout` (unchanged
default 10 seconds) so such stalls are reported explicitly instead of being
misclassified.

Turning `Odom/GuessMotion` off made the serial failure much earlier (428 valid
outputs, about 28.6 sensor seconds, then a greater-than-15-second update).
Doubling `Vis/CorGuessWinSize` from 40 to 80 did not move the principal failure:
703 outputs were acknowledged before the same greater-than-15-second stall.
Reject both variants. The base/tracked launches expose `Odom/GuessMotion`,
`Odom/GuessSmoothingDelay`, and `Vis/CorGuessWinSize` behind their unchanged
native defaults (`true`, `0.0`, and `40`). They also now expose the next bounded
map-update probes, `Odom/KeyFrameThr`, `Odom/VisKeyFrameThr` and
`OdomF2M/MaxNewFeatures`, behind unchanged defaults (`0.3`, `150`, and `0`).

### Exact-bag DepthAI device-feature result

The full September 12 acceptance2 visual sequence has now been replayed through
DepthAI 3.5 with a new OAK `FeatureTracker` configured like the live graph (400
Harris/KLT targets, motion estimator enabled, minimum spacing 49, one SHAVE and
two memory slices) and linked **directly on-device** to `RTABMapVIO`. This avoids
the unresolved host-injected `RTABMapVIO.features` passthrough entirely. It is
strong evidence for actual device-feature use: images enter
`FeatureTracker.inputImage`, `passthroughInputImage` enters `RTABMapVIO.rect`,
and `outputFeatures` enters `RTABMapVIO.features` with `setUseFeatures(true)`.

The stride-1, one-frame-in-flight run completed all 2599 evaluated frames with
**1 invalid pose (0.038%)**, a 0.09998 s longest valid-pose gap, zero final pose
age and a 0.188 m maximum adjacent-valid-pose step. The sole failure sentinel
was sequence 1408 at 47.062530537 sensor seconds—the same low-quality boundary
where serial ROS subsequently enters its unbounded update—but DepthAI recovered
on sequence 1409. Median host-observed processing was 0.133 s (95th percentile
0.149 s, maximum 0.252 s). Those times include offline host conversion, USB
input and USB output; they are not live-camera FPS or CPU measurements. The
result is `/home/john/depthai_full_visual_acceptance2_device_features_20260912`.

A literal full-stack replay then inserted DepthAI `RTABMapSLAM` downstream of
the same device-feature `RTABMapVIO`, enabled the flight stack's 2D occupancy
grid parameters and saved its database. It also completed **2599 VIO outputs
with the same single invalid frame**, 2599 corrected SLAM poses and 359 grid
outputs. Maximum adjacent-valid step was 0.155 m for raw VIO and 0.204 m for
the loop-corrected SLAM pose. The 118 MB RTAB-Map 0.21.12 database contains 360
nodes, 152 optimized poses, **40 global closures and 48 local-space closures**.
This directly answers the user's exact-bag question: the full DepthAI VIO+SLAM
graph does well at stride 1. The result is
`/home/john/depthai_full_stack_acceptance2_device_features_20260912`; the
`rtabmap-info --stats` output is retained in the optimizer-audit directory.
It remains an offline continuity/map test, not live CPU/FPS evidence.

The earlier odometry-only stride-1 replay answers raw continuity; the separate
full-stack replay above additionally exercises mapping. A stride-2 device-feature run
was then run for exact schedule parity with the current ROS candidate. It did
**not** preserve the stride-1 result: 95/1299 outputs were invalid (7.31%), its
longest valid-pose gap was 6.398 s, and it recovered by the end. Thus DepthAI
handles this bag well only when it sees every frame; it does not make the fixed
stride-2 visual sequence intrinsically robust.

The critical scheduling detail is concrete. At stride 1, DepthAI rejects
sequence 1408 and recovers on 1409. Stride 2 processes the even sequence 1408
but omits recovery frame 1409. Flipping serial ROS to the odd stride phase did
not solve the problem: it reached 707 valid acknowledgements and then entered
another greater-than-15-second update. Serial ROS stride 1 also stayed valid
through sequence 1412 but stalled on the next update. Dense imagery delays the
ROS pathology but does not eliminate it, so both schedule and estimator/update
behavior matter.

Matching DepthAI's `Vis/MinInliers=20` alone moved the ROS stall to sequence
1408; pairing it with native `Odom/ResetCountdown=0` made processing bounded,
but all 23 frames from sequence 1408 through the end of the 1430-frame prefix
were lost (0.767 s stale ending). Adding the native 2000-point map was much
worse: 495/1450 lost and 16.46 s stale at the prefix end. Keep the current
600/1000 limits; do not promote these native-default combinations. Results use
descriptive `ros_parity_live_motion_oak_acceptance2_serial_*_20260912` paths.

Standard ROS RGBDOdometry with host GFTT/ORB features cleanly crossed the same
boundary in a 1450-frame serial prefix: **1 lost status, 1447 poses, 0.09998 s
maximum valid-pose gap and zero stale ending**. The first deterministic
real-time full-stack replay then passed the complete bag at every third frame
(9.986 Hz input): **828 statuses, zero lost, 827 poses, 0.29995 s maximum gap,
zero stale ending and 0.185 m maximum pose step**. RTAB-Map and native OctoMap
each produced 80 updates; mapping accepted 15 global closures and 18 proximity
detections. Final OctoMap generation 80 used 53 source nodes and contained
13,747 ground, 30,036 obstacle and 929,499 empty cells with non-degenerate 3D
bounds. Median input-to-status latency was 82.9 ms, maximum 311 ms.

The passing output is
`/home/john/ros_parity_live_motion_acceptance2_ros_host_stride3_full_realtime_retry_20260912`.
This is replay continuity/map evidence, not a correctly cued live return test.
A repeat also passed with **823 statuses, zero lost, 822 poses, 0.29995 s
maximum gap, zero stale ending and a 0.176 m maximum step**. It produced 79 map
and OctoMap updates with 13 global plus 12 proximity detections. During 50
seconds of that moving replay the isolated processes used 73.1% odometry,
22.3% mapper and 19.0% OctoMap CPU: **114.4% total**, with no thermal
throttling. The replay omits the live OAK ROS driver, so this number must not be
presented as an apples-to-apples live comparison with the 187.5% DepthAI
VIO+SLAM process. The repeat is the adjacent `_repeat_20260912` path; CPU is
`/home/john/ros_parity_optimizer_audit_20260908/acceptance2_host_stride3_full_realtime_repeat_cpu_sep12.json`.

These repeats isolate the current OAK-keypoint instability: identical bag
imagery/depth can cross the hard turn in standard ROS host-feature odometry,
while the custom supplied-keypoint path enters pathological F2M updates. The
fresh live host-feature carry requested here was subsequently completed and
failed on a different, much larger visual transition; see **Resume here** at
the top. That newer result supersedes this paragraph's former pending gate.

Controlled full-stack replays of the second bag preserve all source evidence:

| Variant | poses / rate | lost status | pose gaps | max gap | max step | closures (global + proximity) |
| --- | ---: | ---: | ---: | ---: | ---: | ---: |
| shipped stride 2, BA-10 (live) | 1035 / 12.0 Hz | 6/1047 (0.6%) | 4 | 0.97 s | 271 mm | 15 + 1 |
| stride 3 | 637 / 7.4 Hz | 6/650 (0.9%) | 5 | 0.97 s | 325 mm | 16 + 1 |
| F2M map 800 | 825 / 9.5 Hz | 6/838 (0.7%) | 2 | 0.70 s | 1003 mm | 15 + 0 |
| BA maximum 5 frames | 867 / 10.0 Hz | 4/876 (0.5%) | 3 | 0.84 s | 190 mm | 18 + 1 |
| BA maximum 3 frames | 872 / 10.1 Hz | 4/881 (0.5%) | 2 | 0.87 s | 235 mm | 18 + 2 |
| Madgwick orientation, BA-10 | 662 / 7.7 Hz | 11/684 (1.6%) | 6 | 0.68 s | 207 mm | 15 + 1 |
| F2F (`Odom/Strategy=1`) | 914 / 10.6 Hz | 3/921 (0.3%) | 3 | 0.71 s | 300 mm | 14 + 2 |
| F2M reset countdown 3 | 827 / 9.6 Hz | 10/842 (1.2%) | 3 | 1.07 s | 213 mm | 17 + 3 |

None passes. Do not promote stride 3, F2M 800, the smaller BA windows or
Madgwick, F2F or reset countdown 3. The F2M-map result has metre-scale jumps;
BA-5 is the best workload probe but still fails continuity; Madgwick reduces
the largest step while making throughput and loss worse. F2F still has two
large steps, and delaying reset makes the loss episodes longer. Replay bags and reports use descriptive
`ros_parity_live_motion_oak_promoted_acceptance2_*_20260912` names. Logs are in
`/home/john/ros_parity_optimizer_audit_20260908` with matching `sep12` names.

The base launch now exposes `odom_bundle_adjustment_max_frames` with RTAB-Map's
unchanged default 10. The tracked wrapper forwards it and also forwards
`filter_imu`. It also exposes `odom_strategy` with the unchanged F2M default 0.
The filter default remains false, so shipped behavior is unchanged.
The exact promoted defaults remain unmodified pending a passing result.

Post-change validation on September 12 passes: launch Python compilation,
resolved launch arguments, focused tracked/bag tests (10/10), all 30 CTest
targets, and **624 tests with 0 errors, 0 failures and 0 skipped**. `git
diff --check` is clean. All live and replay graphs were stopped; no PX4 node was
started at any point in this gate.

## Resume here — September 9, 2026

The user accepts either ROS host features or the DepthAI feature tracker and
asked Codex to keep iterating while updating this handoff periodically. CPU
measurement is complete; **full live/map parity is not**. The current
investigation has isolated the main remaining problem to which frames survive
real-time delivery/processing. It is not an intrinsic ROS estimator failure:
stock ROS processed the SDK's exact 1x-selected frame sequence with zero loss.

Confirmed results:

- Dense offline cohort: SDK and ROS each lose 1/2593 evaluated frames at the
  same timestamp. These historical runs use the old 250 ms replay buffer.
- Sparse sequence from the real-time ROS VIO run: SDK and ROS lose **the exact
  same 191/703 evaluated frames**, with zero tracking-status disagreements.
  The SDK uses the first of 704 selected frames only for initialization.
- Real-time ROS default: full stack loses 42.1%; VIO only loses 27.1%; both
  have about 21-second valid-pose gaps. Disabling bundle adjustment,
  reducing feature/map limits, and a Python latest-frame relay did not pass.
  Defaults were not changed.
- Native SDK 1x host-fed replay: **1367 outputs, zero lost**, longest sensor-time
  pose gap **0.2333 s**, final valid pose age **0 s**. It accepted 2654 submitted
  RGB-D tuples and omitted 1287 under load. Producer lateness was 0.104 ms
  median / 7.31 ms maximum, but input-to-output latency was **0.7447 s median**.
  This host-fed diagnostic includes extra USB transfer/buffering, so do not
  treat it as native live-camera latency or a live CPU benchmark.
- Exact SDK-schedule replay through unpatched stock ROS: **1367/1367 valid**,
  1366 published poses, 0.2333 s longest sensor-time pose gap and zero ending
  age. This uses normal physical `oak` geometry and the standard host-feature
  RGBDOdometry node. It proves the SDK's selected sequence is sufficient for
  ROS continuity; it does not provide a deployable way to choose those frames.
- Three isolated C++ handoff policies improved throughput but failed continuity:
  latest-one-pending processed 937 updates (35.3% lost), oldest-one-pending 954
  (27.3% lost), and a depth-10 ordered FIFO 1071 (41.0% lost, never recovered
  after 41.8 sensor seconds). All remain disabled diagnostic code outside the
  project and must not be promoted. The FIFO's 0.924 s median / 1.461 s maximum
  recorded delay is also flight-inappropriate.
- A deterministic pre-synchronization publisher established the usable cadence
  boundary on this Pi and motion bag. Fixed 15 Hz produced 877/1327 statuses,
  6 lost, and a 1.400 s valid-pose gap. An alternating 2/3-frame 12 Hz cadence
  produced 734/1061 statuses, 145 lost, and a 20.729 s gap; its 66 ms source
  intervals are still too short. Fixed every-third-frame 10 Hz produced
  **883/884 statuses, zero lost, 882 poses, and a 0.200 s maximum pose gap**.
  Median input-to-status latency was 77.0 ms. A serial fixed-stride control was
  also effectively lossless (1/1327 lost), confirming that load/cadence rather
  than the selected visual content determines the real-time outcome.
- Full mapping at 10 Hz initially broke the otherwise stable VIO. Shared
  single-threaded composition lost 158/606 statuses; shared multithreaded lost
  20/754; separate VIO/mapper processes lost 89/728. Disabling the mapper's
  full OdomInfo subscription did not help (144/657 lost). Mapper-only in a
  separate process, with the custom OctoMap consumer disabled, was clean:
  **864/884 statuses, zero lost**, while RTAB-Map published at 0.916 Hz.
- The custom OctoMap node was rebuilding and traversing the complete tree at
  each map update. Retaining RTAB-Map's local-grid cache/tree and using its
  incremental update path reduced one test from 12.2% to 7.8% loss but did not
  remove a 12.3 s outage. Disabling its visualization marker traversal while
  retaining binary `octomap_msgs/Octomap` publication then passed the full
  motion replay: **842/884 statuses, zero lost, 841 poses, 0.300 s maximum
  pose gap, zero ending age**, with RTAB-Map and OctoMap metadata both at
  0.899 Hz. Median input-to-status latency was 80.0 ms. The efficient v3 default
  is now `max_marker_voxels:=0`; markers remain an explicit visualization opt-in.
  A repeat run also passed (830/884 statuses, zero lost, 0.300 s maximum gap),
  and captured generation 64 with 45 keyframes, 15,004 ground cells, 42,471
  obstacle cells, 1,025,944 observed-free cells, and non-degenerate 3D bounds.
- A lit stationary live-camera run of the same candidate produced **589/589
  valid updates at 9.82 Hz**, 0.592 mm pose RMS, 0.618 mm net displacement,
  140 ms median capture-to-pose age, and a 0.195 s maximum receive gap. Mapper
  and OctoMap metadata each published at 0.917 Hz; generation 67 contained two
  stationary keyframes with nonzero ground/obstacle/free cells. Total measured
  compute was 114.4% of one core (VIO 102.2%, mapper 11.4%, OctoMap 0.8%), with
  no thermal throttling. The immediately preceding dark-scene run was 600/600
  initialization losses; a saved raw frame confirmed the view was nearly black,
  so that run is scheduling-only evidence and is excluded from robustness.
- A correctly pre-rolled 10 Hz host-feature carry is retained at
  `/home/john/ros_parity_live_motion_cued_10hz_20260909`. It tracked the first
  45.2 s of the bag, then lost for the final 53.7 s (345/706 lost). Serially
  processing every recorded tuple fails at the same 45.19 s transition and
  never recovers (534/985 lost), so this failure is visual motion/viewpoint,
  not live mapper or recorder load. Frames show a fast turn under/around a desk
  as inliers fall 55 -> 25 -> 22 -> 0. Lowering odometry `Vis/MinInliers` from
  20 to 10 postpones failure and recovers, but still has 19.6 s and 7.1 s loss
  episodes plus a 477 mm post-gap step; it is not a viable host-feature fix.
- The user explicitly accepts OAK FeatureTracker in the ROS graph. A live
  30 Hz OAK-tracker / nominal-20 Hz ROS F2M experiment with `Vis/MinInliers=10`
  reduced loss to 125/1603 (7.8%) and let ROS mapping accept 25 global closures
  plus 7 proximity detections. Its ending hold is invalid because the return
  cue reached the user late; all output after 90.6 s was lost. The bag is
  `/home/john/ros_parity_live_motion_oak20_min10_cued_20260909`.
- Deterministic every-second-frame input from that 30 Hz bag is the strongest
  moving candidate so far. At **14.99 Hz**, VIO-only and full mapper/native-
  OctoMap replays have **zero lost statuses through sensor time 90.35 s**, the
  complete actual route; their only losses are the same invalid final segment.
  Full-stack output is 1298/1459 statuses, 95 losses confined to 90.35-97.35 s,
  a 0.400 s maximum valid-pose gap before that ending failure, and 58.3 ms
  median status latency. It produced 85 mapper/OctoMap updates, 36 global
  closures, 37 proximity detections, and a 55-node native OctoMap with 2,668
  ground, 32,026 obstacle and 273,990 observed-free cells. Evidence:
  `/home/john/ros_parity_live_motion_oak15_min10_{realtime,full_realtime}_20260909`.
  This motivated a fresh live fixed-15 Hz carry with an early return warning;
  it did not by itself establish a valid ending hold or flight eligibility.
- The second, correctly cued live OAK carry at nominal 15 Hz is
  `/home/john/ros_parity_live_motion_oak15_min10_cued2_20260909`. Its ending
  hold is valid, but RTAB-Map's `max_update_rate` selected frames irregularly:
  447/861 statuses were lost and the primary outage lasted 31.93 s. It did
  recover for an 8.45 s ending hold with 35.9 mm span, 66.5 mm apparent return
  error and a current final pose. This is a failed continuity run, not a flight
  candidate.
- `OakTrackedOdometry` now has an `input_stride` parameter that discards
  synchronized tuples before image copies and `processData()`. The base launch
  exposes it as `oak_input_stride` (base default 1). The experimental tracked
  wrapper now selects every second 30 Hz tuple, disables the second irregular
  `max_update_rate` gate, uses `Vis/MinInliers=10`, and sets
  `Odom/ResetCountdown=1`. It also defaults raw-IMU input off and runs the
  mapper in its own process, matching the validated replay graph. These
  defaults are confined to that explicitly experimental wrapper.
- On the correctly cued bag, deterministic external 15 Hz delivery plus
  immediate reset produced **993 statuses, 4 lost (0.40%), 985 poses, a 0.467 s
  maximum valid-output gap, a current final pose, and a 113 mm maximum pose
  step**. The full mapper produced 73 updates with 5 global and 5 proximity
  closures. Evidence:
  `/home/john/ros_parity_live_motion_oak_external_stride2_reset1_qos2_20260909`.
- The deployable in-component selector then passed the same full-stack replay:
  **944 statuses, 3 lost (0.32%), 938 poses, a 0.600 s maximum valid-output gap,
  69 mm maximum pose step, and 104 mm net start/end displacement**. The final
  pose was 33 ms old. RTAB-Map/native OctoMap produced 73 updates with 1 global
  and 2 proximity detections; generation 73 used 33 source nodes and contained
  767 ground, 15,894 obstacle and 126,851 observed-free cells with non-degenerate
  3D bounds. Evidence:
  `/home/john/ros_parity_live_motion_oak_internal_stride2_reset1_full_qos2_20260909`.
  This is the strongest full ROS moving result, but still needs a live repeat
  before any flight-facing integration.
- A subsequent 45 s live-camera stationary smoke test used the promoted wrapper
  defaults without parameter overrides. It produced **345/345 valid status
  outputs at 7.67 Hz**, 0.439 mm pose RMS and 0.828 mm net displacement. Mapper
  and native OctoMap each produced 43 updates at 0.955 Hz. CPU was 86.0% for
  the combined OAK driver/VIO process, 15.6% mapper and 0.7% OctoMap (102.3%
  total); the Pi ended at 53.45 C with `throttled=0x0`. OAK features arrived at
  28.22 Hz, confirming that the lower pose rate is aged F2M throughput rather
  than camera starvation. Evidence:
  `/home/john/ros_parity_live_stationary_oak_tracked_candidate_20260909.json`.
  This validates live startup/stationary stability, not moving robustness.
- A 600-point F2M-map efficiency probe on the hard carry improved output from
  944 to 988 statuses, reduced post-selector worker omissions from 222 to 178,
  and cut median latency from 59.4 to 52.6 ms. It still lost only 4 outputs, but
  maximum pose step worsened from 69 to 156 mm and start/end displacement from
  104 to 281 mm. Do **not** promote it; the tracked default remains 1000 points.
  Evidence: `/home/john/ros_parity_live_motion_oak_internal_stride2_reset1_f2m600_qos2_20260909`.
  An 800-point attempt stalled its separate-process replay transport after 1121
  published raw tuples and is correctly marked `complete: false`; it is not a
  comparison result. Preserve
  `/home/john/ros_parity_live_motion_oak_internal_stride2_reset1_f2m800_qos2_20260909`
  only as incomplete diagnostic evidence.
- Stationary CPU (100% = one core): ROS VIO 105.90%, 9.98 Hz; native VIO
  110.71%, 12.51 Hz; ROS full including OctoMap 111.90%, 9.10 Hz; native full
  215.73%, 12.31 Hz. Thus ROS full used 48.1% less CPU with 26.1% fewer poses/s.
  Detailed methodology, exclusions and process/thread breakdowns are below.

### Workspace and safe resumption

Repository: `/home/john/autonomous_drone_px4_vio`, branch
`ros-rtabmap-migration`, last observed HEAD `df89f45` (`a`). No commits or staging
were performed in this continuation. Preserve the user's untracked `reports/`
and all existing bag/audit directories. The user has committed changes during
this task; inspect `git status` before editing. Do not overwrite existing result
paths: all replay runners deliberately reject existing output directories.

Source the persistent environment, which survives reboot:

```bash
cd /home/john/autonomous_drone_px4_vio
source /home/john/rtabmap_parity_0_21_12/base_ros_env.bash
# This selects the installed ROS core, v3 driver/message overlays and project.
# ROS_DOMAIN_ID must be 42 for all diagnostics.
```

Use `/home/john/depthai_env/bin/python` for SDK diagnostics, appending
`/usr/lib/python3/dist-packages` to `PYTHONPATH` for rosbag/yaml imports.
Use `PYTHONNOUSERSITE=1 python3` for apt OpenCV/numpy diagnostics. Do not source
the incomplete old-core ROS overlay or restart the abandoned OpenCV/core
builds: installed ROS 0.22.1 already matches tracking status on common frames.
All diagnostic launches omit PX4 output. Do not launch the existing stock PX4
bridge as part of these checks. The original raw bag remains available for
software experiments, but it lacks a valid ending stationary hold and cannot
establish return-to-origin accuracy. The next acceptance test is a fresh live
carry using the newly promoted tracked-wrapper defaults exactly as shipped.

### Current files and verification

Uncommitted work includes the host-feature launch/INI, explicit SDK feature
source and passthrough guard, host-mode ROS frame replay, the CPU sampler,
README/handoff updates, and two fixes made September 9:

1. Shared offline tuple matching now retains one second of delivery reordering
   while preserving the strict 1 ms inter-sensor stamp gate; EOF flushes complete
   tuples behind incomplete ones. Recorded CameraInfo can arrive more than
   300 ms after its image. The old 250 ms bound discarded valid recorded frames.
   Both replay manifests label the new buffer duration. Historical cohort
   counts remain historical; do not silently relabel them as the new cohort.
2. Bag parameter snapshots include `/rtabmap/rgbd_odometry`, in addition to the
   tracked adapter, so recordings capture the host-feature estimator settings.

Full package tests before the scheduling continuation: **617 tests, 0 errors,
0 failures, 0 skipped**. The newest component/bag/replay focus is **20 passed**;
the post-component-change package run passes **594 tests, 0 errors, 0 failures, 0
skipped** (30/30 CTest targets). Python compile, launch default resolution and
`git diff --check` pass.
Logs for the earlier full run are `package_tests_sep9.log` and
`package_test_result_sep9.log` in the optimizer audit directory.

New project changes let bag reports consume either full or lightweight
`OdomInfo`, remap the lite publisher consistently to
`/rtabmap/vio_odom_info_lite`, and let serial ROS replay accept an exact
sequence/stamp JSONL selection. Lightweight status is now the capture default;
full OdomInfo recording is an explicit diagnostic opt-in. Experimental launch
arguments `keep_latest_frame`, `keep_next_frame`
and `processing_queue_size` default off and are recognized only when the
isolated patched odometry overlay is sourced; do not enable them on the normal
stack.

The September 9 continuation also adds a deterministic real-time tuple replay,
10 Hz OAK camera configuration, separate mapper-container controls, an
independent `start_octomap` switch, and incremental native-OctoMap assembly.
The incremental assembler keeps cached 3D local grids, inserts new keyframes
without reconstructing old ones, and still rebuilds when optimized poses move
beyond `octomap_pose_update_tolerance_m` (default 0.01 m), when their rotation
changes beyond `octomap_pose_update_tolerance_rad` (default 0.01 rad), or when
nodes leave the optimized graph. This explicitly covers pure rotational loop
corrections that RTAB-Map's translation-only update threshold does not detect.
Its focused C++ suite passes all 9 tests. `ros_parity_cpu.py` now
captures header-age latency and the final JSON payload of String metadata
topics. The complete package suite now passes **623 tests, 0 errors, 0 failures,
0 skipped**. The experimental native-feature launch now defaults to the tested
10 Hz camera profile, a separate mapper process, and marker traversal off.
The replay tool now supports matching reliable or best-effort sensor QoS,
stops calibration scanning once both required OAK transform groups are found,
marks interrupted input as incomplete, and reports maximum pose step and net
displacement.

### September 9 scheduling audit continuation

Profiling the original 1x VIO-only replay found 704 updates over 88.2 wall
seconds. Median core/update time was 72.0 ms (82.7 ms mean); lost updates cost
109.8 ms median versus 63.4 ms for valid updates. The median interval between
outputs was 101.5 ms. Recording no OdomInfo at all produced 701 updates with a
70.0 ms median update, ruling out full diagnostic serialization as the main
bottleneck. That incomplete no-status bag is
`ros_parity_carry_20260909_host_vio_lite_1x_outputs`; its report has no status
topic and must not be used as a continuity benchmark.

An isolated source overlay at `/home/john/rtabmap_latest_frame_overlay` checks
out upstream `rtabmap_ros` tag `0.22.1-jazzy` and overrides only
`rtabmap_odom`. System packages are untouched. It adds three opt-in diagnostic
handoffs after RGB-D synchronization: latest depth one, oldest depth one, and a
bounded oldest-preserving FIFO. They reject incompatible combinations and
disable themselves with `wait_imu_to_init`. Build by sourcing `/opt/ros/jazzy`
and running the retained colcon command; source the overlay *after* the normal
base environment only for these diagnostics. Exact runners and logs are
`run_host_vio_cpp_{latest,next,fifo10}_replay_sep9.py` and matching log names in
the optimizer audit directory.

The latest-one policy raised throughput from 704 to 937 updates but lost
331/937 (35.3%) with a 20.21 s loss episode. The oldest-one policy processed
954 and lost 260 (27.3%), also with a 20.16 s episode. The depth-10 FIFO matched
the SDK's pre-failure mean selected-frame step (1.94 source frames), but lost
439/1071 (41.0%) and never recovered after 41.8 sensor seconds. Its recorded
status receive delay was 0.924 s median, 1.268 s p90 and 1.461 s maximum. This
closely reproduces the SDK host-fed latency while disproving queue depth alone
as the continuity explanation.

The decisive control is
`/home/john/ros_parity_carry_20260909_ros_native_paced_selection`. The new
`--selection-jsonl` mode serially submitted the exact 1367 sequence/stamp rows
from the successful native paced replay to unpatched stock ROS 0.22.1. ROS
reported **zero lost**, 1366 poses, a 0.2333 s longest sensor-time pose gap and
zero final age. The selection uses host features, physical `oak` body geometry
and unchanged CameraInfo projection. Runner/logs are
`run_ros_native_paced_selection_sep9.py` and
`ros_native_paced_selection_sep9_{launch,replay}.log`. This establishes that
the exact scheduling selection, not an SDK-only estimator capability, explains
the successful native-paced result.

Next: run a slow live carried loop in the lit, textured scene with a deliberate
stationary ending hold. Capture raw inputs, lite loss status, poses, RTAB-Map
Info, correction, and OctoMap metadata. Gate on zero material tracking-loss
episodes, bounded pose age/jumps, growing 3D occupancy, loop correction, and
return-to-origin error. Do not connect any result to PX4 during diagnostics.

### Next experiment and evidence locations

The latest experiment is a **Python diagnostic relay outside the repository**:
`/home/john/ros_parity_optimizer_audit_20260908/latest_rgbd_relay_probe.py`.
It caches the latest complete image/depth/CameraInfo tuple, forwards a single
frame to standard ROS RGBDOdometry, then waits for matching OdomInfo before
forwarding again. It uses a 10 ms guard because OdometryROS publishes OdomInfo
before releasing its processing mutex, a 500 ms feedback timeout, and counts
received/published/overwritten/late tuples, acknowledgments and timeouts.
This is a scheduling hypothesis, not a production change. Source inspection
shows OdometryROS drops arriving frames when its worker owns `dataMutex_`
(`processData()` uses `lockTry()`), while SDK's synchronized input retains one
pending frame. The relay aims to reduce idle time between processed frames.

Runner: `run_host_vio_relay_replay_sep9.py` in the same audit directory;
output: `/home/john/ros_parity_carry_20260909_host_vio_relay_1x_outputs`.
It uses a mapper-disabled standard ROS launch with host profile settings,
remapping sensor inputs to `/paced/image`, `/paced/depth`, `/paced/camera_info`.
The relay itself consumes the original `/oak/*` inputs. Logs are
`host_vio_relay_1x_sep9_{launch,relay,play,record,run}.log`.

Completed relay result: **722 status updates, 212 lost (29.4%), 507 valid
poses, a 20.79 s longest pose gap, and a 3.20 s stale ending pose**. The relay
received 2655 complete tuples, published 724, acknowledged 722, overwrote 1931
pending tuples, rejected zero late tuples and reported **2 feedback timeouts**.
It did not improve continuity over the ordinary VIO-only run (27.1% lost,
20.80 s gap). Do not promote this Python relay or assume a C++ translation
would fix the outage. These counters and the complete bag/report are retained.
All diagnostic launches, relay and native replay processes were stopped at
handoff (checked September 9, 04:03:46 UTC).

Next session should profile why ROS processes about 700 frames while the SDK
paced diagnostic processes 1367 from this recording. Compare core processing
time, frame-selection timestamps, delivery latency and callback/executor idle
time before implementing another scheduling change. Inspect the relay's two
feedback timeouts and whether Python overhead or the 10 ms guard cancels any
benefit. The simple relay hypothesis failed this test; estimator agreement on
identical input does not by itself establish the root cause of the scheduling
gap. The native paced diagnostic's 0.745 s latency must also be understood
before using its higher output count as a desired live scheduling policy.

A change to the installed RTAB-Map dependency has not been made or validated.
The ROS launch currently uses a single-threaded component container;
mapper/executor contention is another possible follow-up, though VIO-only
already fails. If a candidate eventually improves continuity, test it with
mapping enabled and measure CPU and latency before changing defaults. Full
map/trajectory agreement and a fresh properly held carried loop remain open.

Key durable evidence:

- Original motion input: `/home/john/ros_parity_carry_20260907_01`.
- CPU reports, exact commands, runner and consolidated JSON/Markdown:
  `/home/john/ros_parity_cpu_20260908/comparison_summary.{json,md}`.
- Current audit scripts/logs and exact sparse comparison:
  `/home/john/ros_parity_optimizer_audit_20260908/ros_native_exact_subset_comparison.json`.
- Completed native sparse replay:
  `/home/john/ros_parity_carry_20260909_native_ros_vio_rgbd_buffered_subset/report.json`.
- Completed native paced replay:
  `/home/john/ros_parity_carry_20260909_native_paced_rgbd/report.json`, produced by
  `native_paced_rgbd_replay_sep9.py` in the audit directory.
- September 9 ROS reports: directories beginning
  `/home/john/ros_parity_carry_20260909_host_`; the result table below identifies
  each configuration. Runners save exact launch arguments and fresh output bags.
- September 8 interrupted full replay MCAP is preserved in
  `/home/john/ros_parity_carry_20260908_host_full_1x_outputs`; it has no complete
  report and is not a valid completed benchmark. Earlier native sparse attempts
  marked `complete=false` are also excluded, not deleted.

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
# Obey the printed phases: remain still through discovery and the 10 s pre-roll,
# move only after MOVE NOW, and restore the marked pose at RETURN/HOLD NOW.
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

New captures snapshot effective parameters **before** recording starts and use
timed discovery, stationary pre-roll, motion, and stationary post-roll phases.
Their durations are retained in `parity_capture.json`. Move only after the
explicit `MOVE NOW` cue. The default capture records the built-in lite status
instead of duplicating the full feature/local-map arrays; use
`--record-full-odom-info` only for diagnostics that need those arrays. Existing
bag paths are never overwritten. For replay output capture, use `ros2 bag record
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

### September 8: core version ruled out as the main cause

The native repeat `/home/john/ros_parity_carry_20260908_native_repeat` completed
all 2593 frames with **1 lost update (0.039%)**, longest valid gap 0.100 s.
Its current classifier is already correct; use `report.json` and `poses.jsonl`.
The earlier native run lost two frames, so the exact failure count varies,
but both runs preserve continuity far better than the ROS builds.

The isolated 0.21.12 core compiled and installed successfully. Its wrapper
build was interrupted by the later Pi reboot during `rtabmap_sync`; messages
and conversions finished. The source/env/build scripts remain under
`/home/john/rtabmap_parity_0_21_12`. Do not treat that partial wrapper overlay
as a completed ROS installation.

A direct C++ core harness now lives in
`/home/john/ros_parity_optimizer_audit_20260908/direct_core`. Its
`export_frames.py` exports the exact matched bag tuples to `frames.bin`;
`main.cpp` calls `Odometry::process()` directly without ROS, DDS, the SDK, a
device, or a mapper. Both builds use native camera geometry, zero projection
translation, sequence IDs, microsecond relative timestamps, and optimizer
strategy 1/epsilon 0. Output is conjugated into the SDK convention. Each run
processed all 2593 tuples, and both use system OpenCV 4.6:

| direct core | lost frames | lost fraction | longest valid gap (sensor time) |
| --- | ---: | ---: | ---: |
| 0.21.12 (DepthAI's exact RTAB-Map source pin) | 417 | 16.08% | 9.965 s |
| 0.22.1 (installed ROS core) | 418 | 16.12% | 9.998 s |

The `poses_021_relative_seq` and `poses_022_relative_seq` JSONL/report files
and build/run logs retain the evidence. This reproduces severe tracking loss
without the ROS wrapper and shows that the core version alone does not close
the gap. It does not rule out additional ROS delivery/performance issues in
live use.

The SDK binary's embedded OpenCV build information confirms **OpenCV 4.11.0**,
versus system **4.6.0**. Both full build descriptions are retained as
`native_opencv_build.txt` and `ros_opencv_build.txt` in the optimizer audit
directory. An isolated OpenCV 4.11 comparison is being prepared under
`/home/john/rtabmap_parity_opencv411`; system packages and the active live
stack are unchanged. g2o/compiler/build differences remain other candidates;
the OpenCV difference is not yet a demonstrated cause.

Additional dependency checks:

- `direct_core/descriptor_probe.py` computes ORB descriptors from every saved
  image and OAK location with the audited ORB defaults. System OpenCV 4.6 and
  a separately installed OpenCV 4.11.0.86 Python wheel produce identical
  retained keypoints and descriptor bytes for all 2593 frames. Results are in
  `descriptors_comparison.json`; this compares those builds, not the SDK's
  hidden OpenCV symbols directly.
- The SDK's exported `OptimizerG2O::isCSparseAvailable()` and
  `isCholmodAvailable()` both return false (`native_g2o_solvers.json`). Its
  g2o port disables CSparse. RTAB-Map consequently falls back from nominal
  `g2o/Solver=0` to **PCG (1)**. The ROS g2o build has CSparse and actually uses
  it at setting 0. The earlier parameter-default audit missed this effective
  solver difference. `native_effective_optimizer.ini` explicitly selects PCG
  with strategy 1 and epsilon 0; diagnostic replays are testing it.
- The OpenCV source comparison is configured without Carotene, matching
  both recorded build descriptions' `Custom HAL: NO`. `build.sh` under
  `/home/john/rtabmap_parity_opencv411` resumes it after a reboot. A second
  RTAB-Map build in that directory uses the 4.11 build-tree headers/libraries;
  it may need resuming after OpenCV completes if linking reaches a dependency
  that has not yet been built. All these installations are isolated.

Completed optimizer diagnostics retain the tracking gap: direct 0.22.1 with
explicit PCG lost 349/2593 frames (13.46%, longest gap 6.999 s); direct 0.21.12
with both bundle-adjustment stages disabled lost 330 (12.73%, gap 6.965 s).
The SDK with both stages disabled lost only one frame; its run is
`/home/john/ros_parity_carry_20260908_native_no_ba`. However, the feature-input
finding above now takes precedence over attributing this gap to dependencies.

Feature-routing probes are preserved under the optimizer audit directory:
`native_invalid_features_probe.py` added a feature passthrough queue and timed
out with zero recorded outputs; `native_invalid_features_without_passthrough.py`
changed every point to (-1000,-1000) from frame 5 onward and still received
ten valid poses. Their result directories are
`/home/john/ros_parity_native_invalid_features_20260908` and
`/home/john/ros_parity_native_invalid_features_no_passthrough_20260908`.
The attempted `odom.inputs["features"]` route is not exposed by the Python
RTABMapVIO binding and failed before replay; do not count it as a data test.

### Host-feature continuity result and current launch

The user accepts ROS using the native feature behavior. A direct run of the
installed 0.22.1 core with **no supplied keypoints** lost only 1/2593 frames
(`direct_core/poses_022_detect.jsonl`), versus 418 with supplied OAK points.
The large tracking gap therefore does not require upgrading OpenCV or
RTAB-Map. The isolated dependency builds were stopped and are retained only
for investigation; no system library was replaced.

Full explicit host-feature runs:

- SDK: `/home/john/ros_parity_carry_20260908_native_explicit_host`.
- ROS RGBDOdometry: `/home/john/ros_parity_carry_20260908_ros_host_features`.
- Both: 2593 processed frames, **1 lost (0.039%)**, at sequence 2240 / 76.454 s;
  longest valid-pose gap 0.100 s. ROS publishes 2590 poses because the first
  success and first recovery success have no velocity yet.
- ROS comparison used virtual SDK body frame, zero projection translation,
  native feature/map limits 1000/2000, explicit optimizer strategy 1/epsilon 0
  and PCG solver 1. Native uses default optimizer settings with its PCG fallback.
- Known-frame rotation only (no fitted rotation/translation/scale) gives
  **0.145 m RMS / 0.253 m maximum / 0.161 m final position difference** on
  2590 common poses. This is estimator agreement, not ground-truth accuracy.
  `host_feature_trajectory_comparison.json` retains these metrics.

`ros_rtabmap_oak_v3_native_features.launch.py` uses standard RGBDOdometry on
left mono/depth with host GFTT/ORB, 1000/2000 feature/map limits and
`config/rtabmap_native_features.ini` for effective optimizer defaults. It
retains OAK tracks in the camera profile for reproducible input bags, but the
estimator does not subscribe to them. Default raw-IMU aiding is off because
F2M ignores it. The live frame remains the calibrated physical `oak` frame;
no PX4 publisher is launched. A normal-body/unchanged-projection controlled
replay `/home/john/ros_parity_carry_20260908_ros_host_body` completed all 2593
frames with the same single lost frame and 0.100 s longest valid-pose gap.
Thus this continuity result also holds with the profile's physical body frame
and the driver's unchanged projection, without replay-only geometry changes.

The native replay now defaults to explicit `--feature-source host`, creates
no feature input queue, and records that choice. Its optional `oak` mode
requires matching feature passthrough sequence and coordinates before any
frame is accepted. The tested SDK route times out, so it cannot silently
pass a same-feature comparison again. The ROS replay accepts
`--feature-source host`; it preserves normal CameraInfo unless
`--normalize-projection` is explicitly requested. Both still select the
same tuples from the bag, including the tracks-presence gate, for direct
comparison of this recorded dataset.

Package build/full tests passed **615 tests, 0 errors/failures/skips** after
the new profile and feature-passthrough guard. The subsequent camera-config
launch override build also passed. Logs are in the optimizer audit directory
(`native_features_package_*`, `native_features_test_result.log`).

The user additionally requested live CPU comparison. The sequential runner
`/home/john/ros_parity_cpu_20260908/run_comparison.py` measures ROS VIO only,
ROS VIO+SLAM/OctoMap, DepthAI VIO only, and DepthAI VIO+SLAM, using 640x400
at 20 FPS, 35 s startup/warmup and 60 s measurement. Source the persistent
`/home/john/rtabmap_parity_0_21_12/base_ros_env.bash` first. It runs only
diagnostic domain42 launches/scripts with no PX4 bridge, preserves logs,
and skips existing reports rather than overwriting them. New reusable
`scripts/ros_parity_cpu.py` captures process/thread CPU, RSS, actual topic
rates, losses, pose stability, temperature and throttle status. **100% CPU
means one core**, not the whole four-core Pi.

Completed CPU comparison (September 8; reports, commands and logs in
`/home/john/ros_parity_cpu_20260908`, consolidated in `comparison_summary.json`):

| Stack | Total CPU (100% = one core) | Actual pose rate | Stationary RMS |
| --- | ---: | ---: | ---: |
| ROS VIO (`ros_vio_repeat`) | 105.90% | 9.98 Hz | 0.777 mm |
| DepthAI VIO (`depthai_vio`) | 110.71% | 12.51 Hz | 0.665 mm |
| ROS VIO+SLAM+OctoMap (`ros_full_repeat`) | 111.90% | 9.10 Hz | 0.714 mm |
| DepthAI VIO+SLAM (`depthai_full`) | 215.73% | 12.31 Hz | 0.626 mm |

ROS full uses **48.1% less total CPU**, while delivering **26.1% fewer poses/s**.
This does not establish performance parity. ROS VIO and full runs reported
zero losses in 598 and 547 status messages respectively. Native full reported
zero verified failure sentinels; native VIO-only predates sentinel counting,
so its loss count remains unknown. All four selected windows reported
`throttled=0x0` at both ends (temperature endpoints 50.7–57.85 C).

ROS full breaks down into 109.80% for the camera/VIO/SLAM component container
and 2.10% for OctoMap; mapper Info arrives at 0.917 Hz. Native full threads use
99.48% for RTABMapVIO and 98.57% for RTABMapSLAM, with remaining overhead
included in the total; the grid arrives at 1.000 Hz. The saved SDK SLAM source
busy-waits between its 1 Hz processing intervals, so thread CPU is not a
measurement of time spent doing useful map computation.

`ros_vio.json` is excluded because the user confirmed moving/covering the
camera (394/441 lost updates). `ros_full.json` has no valid tracking and is
also excluded. Both original diagnostic runs are preserved; the repeats above
are the healthy stationary windows, not averages including tracking failure.

These are sequential stationary measurements of the existing native script
and the new ROS profile, with visualization and PX4 bridge omitted. Sampler
and launchers are excluded from process totals. ROS full additionally runs
OctoMap. Native full publishes a 2D grid generated from 3D grids; mapper feature
limits are 500 native versus 400 ROS. This is a practical stack comparison
with actual output rates, not an equal-work microbenchmark or proof of moving
accuracy. The low jitter figures are stationary pose spread, not ground-truth
error. The September 8 full-stack 1x motion replay was interrupted by reboot;
its partial MCAP is preserved.

### September 9: real-time replay still fails, and delayed calibration audit

Fresh runs of the same raw carry at 1x, each with a restarted estimator:

| ROS host-feature configuration | Status samples | Lost updates | Longest valid-pose gap | Closures |
| --- | ---: | ---: | ---: | ---: |
| Full default | 592 | 249 (42.1%) | 20.88 s | 0 |
| Mapper disabled | 704 | 191 (27.1%) | 20.80 s | 0 |
| Mapper disabled, Python latest-frame relay | 722 | 212 (29.4%) | 20.79 s | 0 |
| Full, local F2M bundle adjustment off | 744 | 198 (26.6%) | 20.97 s | 9 |
| Full, F2M and registration bundle adjustment off | 746 | 197 (26.4%) | 20.86 s | 9 |
| Full, feature/map budget 500/1000 | 871 | 371 (42.6%) | 20.41 s | 0 |

Reports are in `/home/john/ros_parity_carry_20260909_host_*_1x_outputs/report.json`;
exact commands and launch/play/record logs are retained in
`/home/john/ros_parity_optimizer_audit_20260908/run_host*_sep9.py` and matching
logs. No setting above passes motion continuity; accepted map closures do not
repair the raw-odometry gaps. Disabling mapping reduces losses but does not
remove the roughly 20-second outage. Neither disabling bundle adjustment nor halving the feature/map budget passes;
the defaults are unchanged.
The source bag still has no valid ending hold, so no return-to-origin accuracy
is claimed.

An exact-frame SDK follow-up exposed a replay-tool limitation: the 250 ms
buffer expired some tuples before their recorded CameraInfo arrived. All 35
frames missing from a three-topic audit actually had image/depth/calibration
in the bag; example calibration delivery was 332–370 ms after sensor time.
This is delayed delivery, not missing recorded images or timestamp rounding.
The shared offline matcher now retains one second of history while keeping
the **1 ms inter-sensor sync gate** unchanged, and flushes complete tuples
behind an incomplete tuple at EOF. Two regression cases cover these defects;
all **13 focused bag/replay tests passed**. Both replay manifests record the
new buffer duration. The earlier 2593-frame continuity result is still a
comparison of that explicitly recorded cohort, not a claim about every raw
frame under the new buffer policy.

The corrected three-topic audit finds all **704/704** timestamps accepted by
the ROS VIO-only real-time run. The SDK uses the first of those frames only
for camera initialization, then evaluates the remaining **703**. Both
estimators lose **exactly the same 191 frames**, with **zero disagreements**
in per-frame tracking status. This establishes tracking-status parity on the
sparse input sequence as well as on the earlier dense cohort. It does not
establish trajectory identity, native real-time scheduling or map parity.
Native's longest internal valid-pose gap is 20.596 s, with no valid pose for
the last 3.366 s of sensor time.

The complete SDK run is
`/home/john/ros_parity_carry_20260909_native_ros_vio_rgbd_buffered_subset`;
`ros_native_exact_subset_comparison.json` in the optimizer audit directory
contains the explicit comparison and initialization-frame exclusion. Earlier
subset attempts are preserved with `complete=false` and must not be used as
exact-cohort parity results. The bag recorder's parameter snapshot now also
includes `/rtabmap/rgbd_odometry` for the host-feature profile. Full package
verification after the matcher fix passed **617 tests, 0 errors/failures/skips**
(`package_tests_sep9.log`, `package_test_result_sep9.log`).

A completed 1x SDK host-fed RGB-D replay checks native frame selection under load.
It uses the SDK's own single-frame nonblocking synchronized input, sends the
original recorded RGB-D sequence at sensor-time pace, and records passthrough
frame IDs, valid/lost output and delivery latency. This adds host-to-device
transfer absent from the native live camera graph, so it is a diagnostic
rather than a live-camera CPU benchmark. Its source and log are
`native_paced_rgbd_replay_sep9.py` and `native_paced_rgbd_sep9.log` in the audit
directory. The result is
`/home/john/ros_parity_carry_20260909_native_paced_rgbd/report.json`: 2654
submitted frames, 1367 outputs, zero invalid/failure poses, a 0.2333 s longest
sensor-time output gap and zero ending pose age. Input-to-output latency is
0.7447 s median; producer lateness is only 0.104 ms median / 7.31 ms maximum.
Thus the native path maintains tracking at real-time delivery in this
host-fed setup, while extra input transfer/buffering prevents claiming native
live-camera latency parity from this test.

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
itself; doing so made the system Fast DDS stack crash. The following is a
historical OAK-tracked carry command, **not the current next action**; the
September 12 results at the top supersede it:

```bash
ros2 launch px4_vio_bridge ros_rtabmap_oak_v3_tracked.launch.py

python3 scripts/loop_closure_test.py \
  --label ros-oak-tracked --seconds 60 --hold-seconds 10 \
  --raw-odom-topic /rtabmap/vio_odometry --info-topic /rtabmap/info \
  --out /tmp/loop_ros_oak_tracked.json
```

## Open gates before flight

1. **Sustained visual-odometry continuity under flight-like angular motion.**
   The direct OAK-feature ROS path has 0.97-1.45 s gaps on the September 12
   acceptance2 carry. The 10 Hz host-feature path passes that retained bag but
   the fresh retry and the exact same-bag DepthAI full stack both lose for the
   final 35.1 s after a 106-158 px/frame transition with poor feature depth.
   Profile and reduce host BA/F2M workload enough to test at least 15 Hz before
   requesting another carry. Do not mask this gate with automatic odom resets.
2. Re-run the loop-closure test once the VIO holds tracking, and confirm the
   ROS mapper accepts closures when the trajectory is consistent. The
   corrected-pose path and `cpp_slam_pose_relay`'s non-identity branch remain
   unexercised outside unit tests.
3. **Healthy moving-scene CPU comparison.** The passing 10 Hz saved-bag ROS
   replay costs 114.4% but excludes the camera driver. The failed fresh live
   run measured only 78.3% because valid output collapsed to about 1 Hz. The
   600-feature/requested-20 Hz stationary graph costs 123.6% yet still emits
   only 9.99 Hz. Compare with the measured 187.5% DepthAI VIO+SLAM process only
   after ROS remains tracked on an equivalent live moving route.
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
