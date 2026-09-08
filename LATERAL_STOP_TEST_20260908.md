# Fixed-heading lateral braking test

Current repeat: [ROLL_RATE_TUNING_20260908.md](ROLL_RATE_TUNING_20260908.md)
documents the saved roll-rate P trial (0.150 -> 0.165) and rollback. The profile
below is unchanged; the original baseline flew with P=0.150.

Prepared for the next open-area flight after runs 2/3. The aircraft has not been
armed or flown by this preparation. Keep the usual SLAM/PX4 bridge stack running;
run only this flight adapter for the experiment. The test has no obstacle avoidance.
Use a charged pack and clear space at least 1 m to each side of the start.

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py auto_arm:=true
```

The launch records automatically. `L` requests landing; `K` force-disarms.
Without `auto_arm:=true`, the node does not arm (the inherited dry-run engagement
still requests Offboard mode, so a live dry run is not a read-only check).

## Sequence

1. Climb to 0.30 m while holding initial NED position and heading.
2. Hold for at least 3 s and verify settled motion before starting.
3. Move 0.35 m **right relative to initial heading**, brake, settle.
4. Move back to the original position, brake, settle.
5. Move 0.35 m left of the original position, brake, settle.
6. Return to the original position, brake, settle, request AUTO.LAND.

Two stops in each direction. All endpoints use the initial pose; errors do not
accumulate into successive endpoints. Yaw stays at its initially latched value.
The profile and feedforward remain in the PX4 local frame without map correction
or replanning.

Each default leg lasts 2.797 s, reaches 0.20 m/s and has a 0.30 m/s² peak
acceleration/deceleration. Cosine velocity ramps make position, velocity and
acceleration continuous; braking begins 1.750 s into each leg and lasts 1.047 s.
The scalar profile is rotated onto the fixed right axis in NED. Both velocity
and acceleration feedforward are its analytic derivatives. Holds publish zero
horizontal velocity and acceleration feedforward.

Minimum scripted duration after entering the climb/hold phase is about 26.2 s;
allow roughly 30–40 s including climb and settling. Global armed watchdog: 50 s.

## Gates and aborts

- Each hold lasts at least 3 s, including at least 1 continuous second with
  horizontal position error <=0.10 m, measured horizontal speed <=0.06 m/s,
  altitude error <=0.07 m and heading error <=5 degrees.
- An unsettled hold does not advance to another leg. Initial climb/hold timeout
  is 15 s; subsequent settle timeout is 10 s. A timeout requests landing.
- The measured-position geofence is 1.0 m from the initial position. Local
  position must remain fresh within 0.25 s; a control callback gap >0.15 s aborts.
- Inherited VIO, yaw-rate, horizontal tracking-error and armed-time watchdogs
  remain active. Default feature threshold 160 and tracking-error threshold
  0.30 m. A fresh pose is not itself proof of accurate localization.

## Recording and analysis

ROS bag prefix: `offboard_lateral_stop_<UTC timestamp>` under the configured
flight-log directory. The existing `fastwrite` recorder/lifecycle pattern is
used, with a launch marker and automatic bag finalization when the node exits.
`/lateral_stop/debug` records phase (`ACCEL`, `CRUISE`, `BRAKE`, `HOLD`), leg,
signed displacement/velocity/acceleration reference, actual lateral motion,
initial pose and complete profile configuration on every setpoint tick.

Match the bag to the SD ULog by PX4 boot timestamps. Evaluate each stop using
the internally requested acceleration and attitude/rates, not just the scheduled
brake marker. Measure peak velocity, reversal/response delay, displacement after
braking begins, endpoint overshoot and the subsequent settling motion. Record
loaded voltage per leg. No stopping model is assumed from the earlier impacts.

PX4 was confirmed disarmed. Added `rate_ctrl_status 5`,
`control_allocator_status 5`, `battery_status 100`, `actuator_outputs 5` and
`esc_status 100` to `/fs/microsd/etc/logging/logger_topics.txt` (intervals in ms).
Readback verified all five lines; restarting `logger start -t` reported 26 custom
topics loaded and `Not logging` while disarmed. ESC/RPM data will only appear if
the hardware publishes it. The previous file is backed up on SD as
`/fs/microsd/etc/logging/pre_lateral.txt`; the pre-change readback is saved locally
as `reports/logger_topics_before_lateral_20260908.txt`.
No PX4 control gains or motion-limit parameters were changed for this test.

## Software validation

Seventeen new offline tests cover analytic derivative consistency, short-leg
limits, boundary continuity, relative-right geometry, complete four-leg sequence,
fixed heading, hold duration, dwell reset, unsettled timeout, stale position,
geofence, invalid/nonfinite velocity and callback-gap landing. Existing hover
tests also pass. Aggregate stored package results: **620 tests, 0 errors,
0 failures, 3 skipped**.

Built/installed; launch argument discovery and actual node construction without
spinning pass. No setpoints, Offboard requests or arming commands were sent by
those checks. Logs: `/tmp/lateral_stop_build.log`, `/tmp/lateral_stop_tests.log`.
