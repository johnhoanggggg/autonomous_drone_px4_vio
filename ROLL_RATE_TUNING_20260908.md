# Roll-rate P trial, 2026-09-08

**Trial complete; rolled back to P=0.150.** See
[ROLL_RATE_TRIAL_RESULTS_20260908.md](ROLL_RATE_TRIAL_RESULTS_20260908.md).
The original preparation and experimental rationale follow as history.

Applied over USB after the user requested tuning. PX4 was confirmed disarmed
immediately before the write and remained disarmed afterward. The only parameter
written was **MC_ROLLRATE_P: 0.150 -> 0.165 (+10%)**. Readback showed `0.1650`
with the `+` saved marker: this persists across reboot. No flight was started.

Raw before/after evidence, including MC, MPC and external-vision parameter
snapshots: [roll_rate_p_trial_20260908T045445Z.txt](reports/roll_rate_p_trial_20260908T045445Z.txt).
Roll I=0.10, D=0.003, K=1 and attitude P=4 remain at baseline. Pitch, yaw,
position/velocity gains, estimator timing and limits were not adjusted.

## Hypothesis and limits

The baseline lateral test shows delayed corrective roll and imperfect roll-rate
tracking while commanding gentle lateral stops. Increasing rate P modestly tests
whether stronger immediate rate-error correction improves this response.
The increment is a conservative experimental choice, not a gain calculated from
an identified airframe model. PX4 describes P as improving response/tracking,
with excessive P producing high-frequency oscillations in its
[rate tuning guide](https://docs.px4.io/v1.17/en/config_mc/pid_tuning_guide_multicopter#proportional-gain-p-k).

This closed-loop position test is a repeatable screening experiment, not isolated
rate-loop identification. Growing yaw compensation, propulsion effects and
estimator delay remain unresolved. The historical July motor-direction defect
was corrected then; it does not establish the cause of today's yaw correction.
Autotune has not been started or enabled by this change.

## Next flight

Use a freshly charged pack and the same open area, starting heading, airframe and
0.30 m flight height as the baseline. Keep at least 1 m clear to either side;
this test has no obstacle avoidance. Run the usual SLAM/PX4 bridge stack and only
one flight adapter. Disconnect USB before flight.

```bash
cd /home/john/autonomous_drone_px4_vio/ros2_ws
source /opt/ros/jazzy/setup.bash
source install/setup.bash
ROS_DOMAIN_ID=42 ros2 launch px4_vio_bridge offboard_lateral_stop.launch.py auto_arm:=true
```

This command automatically arms and flies the existing right/return/left/return
sequence, recording a bag. All motion settings and watchdogs match the baseline.
Press `L` to land if new twitching, buzzing or worsening oscillation appears;
`K` force-disarms and is the emergency option. Reconnect USB afterward to pair
the ROS recording with its onboard ULog.

## Evaluation before another gain change

Baseline: ROS `offboard_lateral_stop_20260908T043031Z`, SD `px4_20_22_29.ulg`.
See [baseline results](LATERAL_STOP_RESULTS_20260908.md).

- Confirm the new ULog records P=0.165 and check relevant parameter differences.
- Align each flight by its actual leg/hold events. The baseline onboard plotting
  script contains fixed time windows; those must not be reused blindly.
- Compare roll-rate and roll-angle tracking, including oscillation and output
  activity, in corresponding maneuver phases. Lower position error alone does
  not establish a better rate tune if the rate demands or voltage differ.
- Compare all four peak speeds, first-stop overshoots, subsequent hold excursions
  and settling durations. Baseline leg 3 peaks at 0.316 m/s and overshoots 12.2 cm;
  hold 2 spans 25.8 cm and lasts 9.85 s. Inspect all legs, not only those maxima.
- Check yaw error/integral, motor split, allocator status, loaded voltage and VIO
  health. A different pack/load or yaw disturbance can confound a single repeat.

Retain provisionally only if roll tracking improves without new oscillation or
worse stopping/holding. Roll back if it worsens; an inconclusive result calls for
an unchanged repeat or return to baseline, not an automatic second increase.
Tunnel flight remains unvalidated by this parameter write.

## Rollback

With USB connected and the aircraft disarmed, confirm status first:

```bash
cd /home/john/autonomous_drone_px4_vio
.venv-mavlink/bin/python3 scripts/nsh.py 'commander status'
```

Only after the output explicitly says `Disarmed`, restore and verify:

```bash
.venv-mavlink/bin/python3 scripts/nsh.py 'param set MC_ROLLRATE_P 0.150' 'param show MC_ROLLRATE_P'
```

Expected readback: `MC_ROLLRATE_P ... : 0.1500`. This restores the one changed
gain; the raw snapshot also preserves the surrounding configuration for review.
