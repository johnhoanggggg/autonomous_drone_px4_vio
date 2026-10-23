# Native Loop-Closure Correction Handoff

Updated 2026-08-04. The experimental host-side `map_correction` relay is
retired. Its reconstructed target, applied slew, preview, residual topics, live
probe, executable, and dedicated tests were removed after DepthAI's native
correction was exposed and validated. Historical versions remain recoverable
from git history.

## Current architecture

DepthAI 3.5's on-device `dai.node.RTABMapSLAM` publishes three synchronized
outputs from each callback:

- `/rtabmap/vio_pose`: continuous raw odometry-frame VIO; this is the only pose
  fed to PX4/EKF2.
- `/rtabmap/pose`: loop-corrected world pose for mapping and global planning.
- `/rtabmap/odom_correction`: native full-SE(3) map-to-odometry correction,
  published as `PoseStamped` so it is not inserted into TF.

The host uses blocking reads for corrected pose, correction, and raw
passthrough, eliminating the prior one-frame pairing skew. Native correction is
also used directly when transforming timestamped raw poses for person
projection; it is no longer reconstructed from separately delivered poses.

The observation-only route follower subscribes to native correction directly.
It:

- rejects non-finite corrections and absolute corrections above 0.5 m or 15
  degrees;
- rejects reset-sentinel, invalid, missing, or stale raw VIO;
- rejects missing or stale correction;
- coalesces correction motion with its existing low-pass, material-change,
  quiet-time, fresh-path, and cooldown gate; and
- publishes the aggregate result on `/planner/follower/valid` and the reason on
  `/planner/follower/status`.

The route follower is still observation only. No correction or planner output
is sent to PX4. A flight adapter should use stricter provisional correction
limits of 0.25 m / 5 degrees and HOLD-then-LAND if they are exceeded.

Only correction yaw is required by the 2D route follower. Native roll/pitch is
retained in the recorded source message but intentionally does not tilt the
horizontal route or change the altitude latch.

## Evidence

Bag `global_planner_loop_closure_4` established the native transform direction:
`C_native * raw_pose -> corrected_pose`. It also exposed the old one-frame host
pairing skew. Raw VIO then failed permanently at 83.28 s and DepthAI emitted a
bogus 1.47 m / 51.32 degree correction, proving raw-VIO and correction-health
gates are both necessary.

Bag `global_planner_loop_closure_5` validated the synchronized implementation:

- native correction, raw VIO, corrected pose, and feature count ran at 13.2 Hz;
- best raw-frame offset was zero with 0.0 ms median header offset;
- native full-transform error had effectively zero p95 XY/yaw error, with 0.23
  cm / 0.079 degree maxima;
- one reset-sentinel sample recovered continuously on the next sample;
- genuine correction reached 39.96 cm / 11.15 degrees, including a 32.21 cm
  optimization step;
- four follower correction waits totalled 2.62 s, with zero backward cumulative
  progress; and
- A* took 1.84 ms median, 10.36 ms p95, and 13.07 ms maximum.

Bag `global_planner_loop_closure_6` validated the trimmed native-only runtime.
It contains `/rtabmap/odom_correction` and `/planner/follower/valid` with none
of the retired `/vio/map_correction*` topics. Raw VIO, corrected pose, and
native correction ran at 13.7 Hz with zero-frame/0.0 ms pairing and exact
healthy-sample transform agreement. The largest correction was 15.51 cm / 3.37
degrees, including a 10.73 cm / 3.74 degree step. Two isolated reset-sentinel
VIO samples caused immediate `INVALID_VIO` follower output and recovered on
valid input. A later correction correctly held the follower; the planner then
reported `GOAL_BLOCKED`, so it did not resume without a valid route. Progress
reached 3.86 m with zero backward events. Bag `_6` also exposed 16 non-unit
native correction quaternions during DepthAI initialization at t=1.95–3.05 s;
the bridge and follower now reject them and the analyzer excludes them from
transform validation.

## Recording and analysis

Run the normal SLAM stack, then:

```bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge \
  global_planner_monitor.launch.py record_bag:=true \
  bag_output:=flight_logs/global_planner_loop_closure_N
```

New bags record only the native correction and follower validity, not the
retired relay topics. Analyze either new native-only bags or old relay-era bags
with:

```bash
python3 scripts/analyze_map_correction.py flight_logs/<bag>
```

The analyzer uses `/rtabmap/odom_correction` when present and falls back to
`/vio/map_correction_target` for historical bags. When both are present it also
reports their agreement. Use a new bag output directory each run and stop the
launch cleanly so the MCAP index is finalized.
