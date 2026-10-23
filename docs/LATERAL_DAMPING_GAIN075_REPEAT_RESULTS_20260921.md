# Gain 0.75 repeat — September 21, 10:12

## Result

The repeat is a **clean 4/4 completion** with adapter damping enabled at gain
0.75, correction cap 0.10 m/s² and filter time constant 0.10 s. It used the
restored PX4 baseline: `MPC_XY_P=0.95`, horizontal velocity P/I/D
1.8/0.4/0.2 and `MC_ROLLRATE_P=0.150`. No parameters changed in flight.

This is the best gain-0.75 run so far. All four holds passed in
3.04 / 3.05 / 4.35 / 4.63 s. Directional endpoint overshoot was
0.0 / 2.4 / 2.8 / 4.7 cm; the largest whole-hold lateral swing was 10.0 cm
on the last return. The first hold's first-three-second lateral velocity RMS
was 0.027 m/s.

The ROS bag stopped before its closing messages, but this does not make the
outcome ambiguous. Its matched PX4 ULog records 1.109 s of uninterrupted joint
position/speed/yaw/altitude gate satisfaction immediately before the first
external NAV_LAND command at 36.851 s. This independently satisfies the
adapter's one-second dwell rule, well before the ten-second settling timeout.
PX4 accepted the command and subsequently disarmed normally.

| Flight | Damping | Holds passed | Worst endpoint overshoot | Worst hold swing | First-hold velocity RMS | First-hold loaded voltage |
| --- | --- | --- | --- | --- | --- | --- |
| 08:27:29 | ON, 0.75 | 4/4 | 10.4 cm | 18.8 cm | 0.023 m/s | 11.16 V |
| 08:28:34 | OFF | 0/4 | 17.4 cm | 25.3 cm | 0.036 m/s | 10.92 V |
| 08:29:06 | ON, 0.75 | 3/4 | 15.9 cm | 25.3 cm | 0.088 m/s | 10.77 V |
| **10:12:33** | **ON, 0.75** | **4/4** | **4.7 cm** | **10.0 cm** | **0.027 m/s** | **11.21 V** |

The first three flights are not a perfectly controlled comparison. In
particular, the failed enabled run had the lowest loaded voltage. The new run
does establish a second 4/4 gain-0.75 completion, under a battery condition
comparable to the first successful run. Together with the same-batch OFF
failure, the evidence now supports a guarded adapter trial; it does not prove
that damping alone explains every difference.

## Remaining limitations

The last hold acquired the required dwell, but it was not completely quiet.
Horizontal speed exceeded the 0.06 m/s gate for about 32% of the full hold,
its first-three-second lateral speed RMS was 0.057 m/s, and yaw temporarily
reached 6.5 degrees. It then recovered and all gates remained true for the
final 1.109 s. The last hold's first-three-second yaw-rate tracking-error RMS
was 12.0 deg/s. Therefore the result does not justify raising gain or
correction cap.

The applied correction peaked at 0.0836 m/s² and never exceeded
0.095 m/s² during a hold. Maximum measured-velocity age was 34.5 ms, with no
stale-velocity or callback-gap events. All estimator reset counters remained
constant. The only ULog dropout during lateral testing was 15 ms in the final
hold, too short to explain the result. No sampled motor command exceeded 0.95;
the largest first-three-second hold peak was 0.657.

## Adapter decision

Yes: gain 0.75 is now supported for an **opt-in, default-OFF port** to the
global/tunnel adapter, followed by a controlled tunnel trial at the proven
0.20 m/s route speed. Do not enable it globally by default yet.

The port should preserve these safety boundaries:

- Add `0.75 * (reference_velocity - measured_velocity)` only while the route
  adapter is in a valid normal `Route` command and is braking or holding a
  validated goal; retain the 0.10 m/s² correction cap, 0.10 s filter and the
  existing 0.30 m/s² total acceleration cap.
- Require fresh, finite PX4 horizontal velocity. Clear the filter on stale or
  invalid velocity, callback gaps, local-frame resets and state exits.
- Do not add damping in climb, LAND, planner-fault hold, command hold, escape,
  or the base class's NaN/pure-position safety modes. Existing clearance,
  geofence, path-validation, yaw and landing logic remain authoritative.
- Expose recorded configuration and correction diagnostics so the first tunnel
  flight can prove what was applied. Keep a launch switch for immediate A/B
  rollback.

The integration point is the global adapter's setpoint feedforward path around
`setpoint_acceleration_xy()`, with measured velocity captured in
`on_local_position()`. It should be implemented and bench-tested before any
armed tunnel run; this analysis did not change the tunnel runtime.

Implementation follow-up: the guarded default-OFF port is now built and tested.
See [the global damping implementation note](GLOBAL_DAMPING_OPT_IN_20260921.md).

## Evidence and artifacts

The ROS recording is
`ros2_ws/flight_logs/offboard_lateral_stop_20260921T101233Z`. Its empty metadata
and truncated MCAP were handled using the matched onboard file
`reports/damping075_repeat_20260921/00_49_15.ulg` (4,866,197 bytes). Pairing has
1,433 exact PX4 local-position timestamp matches and zero x-position
difference; it does not rely on host/SD filenames.

- [Complete sequence plot](../reports/damping075_repeat_20260921/flight.png)
- [Equal-duration hold plot](../reports/damping075_repeat_20260921/holds_first3.png)
- [Full synchronized metrics](../reports/damping075_repeat_20260921/analysis.json)
- [Reproducible analysis](../reports/damping075_repeat_20260921/analyze.py)

The analysis asserts exact log pairing, fixed hold references, zero hold
reference velocity, bounded acceleration, stable estimator reset counters and
no onboard parameter changes. Both figures were inspected. No runtime code,
PX4 parameter or live vehicle state was changed during analysis.
