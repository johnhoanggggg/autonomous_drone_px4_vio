# VIO timing correction trial — 13 September 2026

> **Trial flown:** [12:11 braking results](../ros2_ws/flight_logs/offboard_lateral_stop_20260913T121110Z/ANALYSIS.md). Three legs; third-hold settling timeout. Less peak overspeed, mixed initial braking and persistent 16.3 cm hold swing. 225 ms remains saved; no tolerance/gain changes. The preparation below is historical.

**Saved `EKF2_EV_DELAY: 270 → 225 ms` over NSH while disarmed.** The
readback confirms 225.0000 with the saved marker. The vehicle remained
disarmed. Roll P stays 0.150; all PID gains and trajectory settings are
unchanged. `EKF2_DELAY_MAX` remains 270 ms so its buffer still covers the
measurement delay. No reboot, flight, motor test or ROS publisher was started.

This is an evidence-backed **calibration trial**, not proof the braking/hold
oscillation is fixed. Power-cycle PX4 before the validation flight. Source
timestamp handling is unchanged; this value applies to the native VIO path
flown in these logs, not automatically to the experimental ROS VIO backend.

## Why this is the first change

The September 8 stationary holds oscillate despite a constant target and zero
horizontal feedforward. Planner smoothing cannot explain or cure all of that
motion. The earlier roll-P increase also failed to resolve it. The user reports
no intervening prop, motor, ESC or camera-mount changes.

The old 270 ms setting came from July hand-yaw measurements. Re-estimating
timing against the **onboard gyro**, using all four recent flight ULogs, gives:

| ULog | Best delay | Correlation | First / second half | Peak-minus-0.005 range |
|---|---:|---:|---:|---:|
| Sept 8 baseline `20_22_29` | 225 ms | 0.9904 | 225 / 225 ms | 200–255 ms |
| Sept 8 roll-P trial `20_51_29` | 225 ms | 0.9902 | 220 / 225 ms | 195–255 ms |
| Sept 13 flight 1 `03_04_20` | 205 ms | 0.9943 | 210 / 200 ms | 175–240 ms |
| Sept 13 flight 2 `03_06_45` | 230 ms | 0.9898 | 225 / 230 ms | 195–265 ms |

225 ms is a representative setting within all four sensitivity ranges. The
first tunnel flight differs by 20 ms, so this does not establish a perfectly
constant delay. Comparing 150, 250 and 350 ms yaw increments changes the
individual estimates by at most 5 ms. The displayed ranges are correlation
peak widths, **not confidence intervals**. July's evidence is not invalidated;
pipeline/load/configuration changes over time can change this measurement.

`vehicle_visual_odometry.timestamp_sample` is effectively the arrival stamp
in these logs (median receipt-minus-sample -1 microsecond). Its orientation
therefore represents motion roughly 225 ms older than the supplied stamp.
270 ms assigns these measurements about 45 ms too far into the past relative
to the baseline/trial estimate. This can affect EKF correction timing; these
logs cannot establish how much of the lateral oscillation it causes.

The method follows PX4's recommended comparison of external-vision rotation
with IMU rotation. It uses gyro-derived Euler yaw rate, accounting for
roll/pitch coupling, and integrates equal-duration intervals. It does not
correlate VIO position with EKF position, which shares the same vision input
and propagates forward using the IMU. Reset intervals and recorded gaps are
excluded; weak excitation and nonmonotonic/nonfinite inputs are rejected.
Yaw timing is a pipeline diagnostic, not independent position ground truth,
a full EKF replay, or a certified translational latency bound.

Reference: [PX4 VIO delay tuning](https://docs.px4.io/v1.16/en/computer_vision/visual_inertial_odometry#tuning-ekf2-ev-delay).

## Validation flight

After power-cycling PX4, use the same native SLAM/VIO bridge stack and the
same open-area lateral test. Use a charged pack, at least 1 m of clear space
on either side, and keep the RC kill control ready. This test has no obstacle
avoidance; it is not a tunnel flight. Do not change other gains or the profile
for this comparison.

With the normal native stack running, in a separate terminal:

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source /home/john/ros2_ws/install/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py auto_arm:=true
```

The command arms and runs the four-leg test. Let the launch/recorder finish
before removing companion power. Download and pair its onboard ULog, verify
it actually recorded EV delay 225, and compare all four stops and subsequent
holds with the Sept 8 baseline, using the new flight's phase timestamps.

Evaluate peak velocity, initial endpoint overshoot, first-reversal travel,
subsequent hold excursion, settling time and yaw/rate tracking. The existing
settle gate requires <=10 cm XY error and <=0.06 m/s speed (plus altitude/yaw
gates) for one continuous second. The trial needs consistent improvement
across stops/holds, not just one better leg, before retaining it as a braking
improvement. It does not establish sufficient clearance for tunnel flight.
If excursions remain unchanged or worsen, roll back and continue the inner
response/propulsion and estimator investigation rather than stacking gains.

Rollback, while disarmed, then power-cycle before comparison:

```bash
.venv-mavlink/bin/python3 scripts/nsh.py 'param set EKF2_EV_DELAY 270' 'param save' 'param show EKF2_EV_DELAY'
```

The rollback command above runs from the repository root.

## Reproduction and checks

```bash
python3 -m pytest scripts/test_analyze_ev_delay.py -q
.venv-mavlink/bin/python3 scripts/analyze_ev_delay.py \
  ros2_ws/flight_logs/px4_20_22_29.ulg \
  ros2_ws/flight_logs/px4_20_51_29.ulg \
  ros2_ws/flight_logs/offboard_global_20260913T113753Z/px4_03_04_20.ulg \
  ros2_ws/flight_logs/offboard_global_20260913T114019Z/px4_03_06_45.ulg \
  --out reports/ev_delay_20260913 --plot
```

Five tests pass: known 75/225/350 ms delays at irregular VIO intervals,
reset/gap rejection, and rejection of stationary/duplicate-time input.
The MAVLink venv lacks pytest; tests ran successfully with system Python.
All four ULogs analyzed successfully. No runtime source changed, so no ROS
build or flight-control tests were required for this calibration/tool change.

Artifacts: [timing plot](../reports/ev_delay_20260913.png),
[analysis JSON](../reports/ev_delay_20260913.json),
[before snapshot](../reports/ev_delay_before_20260913.txt),
[guarded write and readback](../reports/ev_delay_trial_20260913.txt).

Independent read-only audits cover native VIO timestamps and the separate
trajectory-vector discontinuity. The latter remains a software limitation;
no trajectory change was combined with this calibration trial.
