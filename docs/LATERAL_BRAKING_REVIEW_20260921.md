# Lateral stop review — braking and renewed motion during hold

**Implementation follow-up:** the user subsequently approved the experiment.
It is now implemented, built and tested in the lateral-stop adapter, disabled
by default. See [trial instructions and results](LATERAL_DAMPING_TRIAL_20260921.md).
The analysis and original recommendation below remain the supporting evidence.

2026-09-21, branch `tune/migration-tunnel-smoothing`, baseline `893cbea`.
User request: analyse the lateral-stop test, isolate the error, and suggest an
adapter change that addresses overshoot. **No runtime code or vehicle parameters
were changed by this review.** The proven 0.20 m/s cruise settings remain in place.

## Conclusion

The latest two lateral-stop tests isolate the problem from the tunnel planner:
overshoot and repeated motion occur with fixed heading, no replanning/map
rebasing, constant hold position and zero hold velocity/acceleration feedforward.
The aircraft can brake: several legs nearly stop or reverse by the end of the
scheduled ramp. The larger errors include motion that starts again during hold.

The evidence supports a combined control-response/settling problem, not a
single identified bad gain or a constant braking delay. A small adapter damping
experiment is reasonable. Keep the path controller and speed, and add bounded
feedback from measured velocity during braking and hold, first in the existing
open-area lateral-stop adapter. Do not interpret this recommendation as a flown
fix or as proof that the underlying attitude/estimation issue is resolved.

## Inputs and checks

- `offboard_lateral_stop_20260917T080456Z`, onboard `px4_23_10_12.ulg`.
- `offboard_lateral_stop_20260917T080556Z`, onboard `px4_23_11_12.ulg`.
- These are the latest lateral-stop recordings in this workspace, not the
  September 21 tunnel flight. Both used 0.35 m legs, 0.20 m/s reference speed,
  0.30 m/s² peak cosine-ramp acceleration and 3 s minimum holds.
- Verified hashes and sizes of both MCAPs, metadata and ULogs against the
  existing extraction manifest. Reused the decoded ROS messages and recomputed
  motion/control measurements directly from the ULogs.
- Matched 1928 and 2503 exact PX4 local-position timestamps respectively, with
  zero x-position discrepancy. PX4-clock-to-bag offsets are
  -187.4407930725 s and -247.5152174710 s. Internal control/response comparisons
  use the onboard clock, rather than pairing different clocks by wall time.
- Excluded LAND explicitly: debug retains the last leg/phase labels during
  landing, which must not be mistaken for stationary hold.
- Asserted each hold's onboard XY position target was constant and onboard
  XY velocity/acceleration feedforward was zero, away from phase boundaries.
- The second bag has empty metadata but its MCAP and matching onboard log
  contain the flight. Existing extraction handled that incomplete finalization.

## Braking versus settling

Speeds below are signed along each leg: a negative stop speed means reversal.
Hold swing is peak-to-peak lateral displacement, not endpoint overshoot.

| Flight / leg | Speed at brake → reference stop → one second later (m/s) | Largest endpoint overshoot | Hold swing |
| --- | --- | --- | --- |
| 08:04:56 / right | 0.160 → 0.047 → 0.030 | 0.0 cm | 4.5 cm |
| 08:04:56 / return left | 0.149 → 0.024 → 0.024 | 7.3 cm | 5.3 cm |
| 08:04:56 / left endpoint | 0.092 → 0.002 → 0.119 | 13.3 cm | 22.5 cm |
| 08:05:56 / right | 0.219 → -0.025 → -0.005 | 0.0 cm | 9.8 cm |
| 08:05:56 / return left | 0.092 → 0.018 → 0.024 | 6.2 cm | 9.8 cm |
| 08:05:56 / left endpoint | 0.212 → -0.055 → -0.002 | 12.2 cm | 11.0 cm |
| 08:05:56 / final return | 0.174 → 0.069 → 0.027 | 10.0 cm | 18.7 cm |

The first flight times out settling after leg 3; the second executes all four
legs but times out during the final hold. Both problematic final holds last
about ten seconds. Merely starting the same reference brake earlier would not
directly address the renewed motion while a stationary target is already held.
Measured average braking deceleration varies about 0.074–0.254 m/s² across the
legs; these tests do not identify one constant latency or maximum brake force.
Motion is estimator-derived, not independent physical ground truth.

## Where the control chain disagrees

For the second flight at 44.2–44.7 s, during the final stationary hold, the
fresh calculation gives these means in the initial-heading right axis:

- Estimated velocity: **+0.077 m/s**; PX4 internal velocity demand:
  **-0.078 m/s**.
- Velocity-loop proportional contribution: about **-0.278 m/s²**.
- Inferred residual compensation: about **+0.286 to +0.289 m/s²** across three
  finite-difference derivative windows. This is an estimate, not a directly
  logged velocity-integrator state. It largely cancels the immediate P term.
- Total internal acceleration demand: **+0.012 m/s²**.
- Tilt-equivalent acceleration: **+0.243 m/s²**, under a near-hover thrust
  approximation. This is not measured net translational acceleration.
- Roll-rate demand: **-5.85 deg/s**; measured roll rate: **-1.49 deg/s**.
  Roll-angle error RMS is 1.54 degrees in that half-second window.

There are two effects to address: the outer loop's transient demand, including
persistent compensation, and the aircraft's lag in reaching the requested tilt.
Do not label the compensation as erroneous windup or remove it: it may counter
a real bias/disturbance. Likewise, this disagreement does not identify a motor
fault, nor prove that increasing roll P is the correct fix.

In the first flight at 31.2–31.7 s, the return velocity demand is already
+0.046 m/s while estimated velocity remains -0.079 m/s. Acceleration demand
is +0.450 m/s² versus +0.390 m/s² tilt-equivalent; roll-rate demand is
+1.26 deg/s while measured roll rate is -0.90 deg/s. A stronger reference
braking number alone therefore cannot explain all of the observed response.

Recorded parameters agree between the tests: `EKF2_EV_DELAY=225`,
`MPC_XY_P=0.95`, velocity P/I/D = 1.8/0.4/0.2, roll/pitch rate P/I/D =
0.15/0.10/0.003. No parameter changes appear in either ULog. Recorded board
offsets (-2.9380°, +1.2593°) agree with the subsequent live readback.
The live -1.1°/-0.3° attitude reading alone cannot identify a calibration error
without knowing that the airframe was physically level.

## Suggested adapter experiment

Add a small **velocity-error damping term to acceleration feedforward**:

```text
correction = limit_vector(k * (reference_velocity - measured_velocity), 0.10)
acceleration_to_publish = limit_vector(reference_acceleration + correction, 0.30)
```

Suggested initial trial: `k = 0.5 s⁻¹`, correction cap 0.10 m/s², with about
0.10 s smoothing of the correction, including its transition back to zero.
These are proposed trial values, not fitted/validated gains.

During hold, reference velocity is zero, so this term opposes drift. During
braking, it reacts to velocity tracking error even after the reference ramp
has ended. At 0.10 m/s residual drift it adds roughly 0.05 m/s² opposing
acceleration. Existing PX4 feedback remains active: this is explicitly extra
velocity damping, not a replacement for it or an independent physical brake.
It adds feedback bandwidth, so the known response delay still matters.

Scope the first implementation to `offboard_lateral_stop.py` and its tests:

1. Default disabled. Enable explicitly for the same lateral-stop profile;
   retain 0.20 m/s speed, 0.30 m/s² total acceleration-feedforward cap, positions,
   heading, dwell gates and endpoint sequence.
2. Apply during BRAKE and LATERAL_HOLD. Preserve launch/climb and landing/abort
   behavior. Smooth activation/deactivation; reset the correction at flight end
   and disable it on stale, invalid or nonfinite velocity.
3. Use actual PX4 NED velocity and the NED reference vector, with a vector-norm
   cap rather than independent per-axis clipping. Log raw/filtered correction,
   measured velocity, final acceleration and enable reason.
4. Test zero-gain baseline equivalence; correction sign in both directions;
   vector caps; reference-frame rotation; stale/invalid inputs; smooth phase
   transitions; unchanged position/velocity targets; and landing exclusion.
5. Compare repeat flights primarily on endpoint overshoot, hold swing and
   settling time, with no worsening of attitude/rate oscillation. Only then
   consider the same small helper for the migration C++ adapter.

In the global adapter, the relevant hook is `setpoint_acceleration_xy()` in
`src/offboard_global_planner_node.cpp`. It currently differentiates reference
velocity and does not use measured velocity. Keep all path acceptance, clearance,
yaw, finite position targets and recovery behavior. Its existing NaN/native
position-hold modes require explicit treatment before porting; do not casually
turn feedforward back on in fault holds or LAND.

An alternative later experiment is measured-speed-aware early reference braking,
but it should not be bundled with damping. The fixed-hold evidence makes
damping the more directly targeted first adapter experiment. Neither proposal
justifies resetting PX4 integrators, guessing attitude gains, shifting the
position target backwards, or weakening obstacle clearance.

## Offline candidate check and limits

Ran the candidate on recorded inputs only. During the difficult holds, the
95th-percentile correction magnitude is about 0.059–0.072 m/s². At the second
flight's 44.2–44.7 s drift, the proposed extra lateral acceleration averages
-0.037 m/s². Assertions verified the 0.10 correction and 0.30 total vector caps.
The position and velocity reference traces are unchanged by construction.

This is a check of proposed **commands**, not a closed-loop simulation or proof
of reduced stopping distance. The actual measured trajectory remains the
recorded baseline; do not claim a predicted improvement from the plot.

- [Motion/control and candidate-command plot](../reports/lateral_braking_20260921/braking_and_damping.png)
- [Fresh numerical results](../reports/lateral_braking_20260921/summary.json)
- [Offline analysis script](../reports/lateral_braking_20260921/analyze.py)

Reproduce from the repository root:

```bash
.venv-mavlink/bin/python reports/lateral_braking_20260921/analyze.py
git diff --check
```

Analysis completed successfully, the figure was inspected, input identity and
onboard hold-command assertions passed. Reports are git-ignored local artifacts;
this note preserves the findings. No package rebuild or flight-control tests
were needed because runtime code is unchanged. No live hardware was accessed
for this review. Work was performed by the lead agent only.
