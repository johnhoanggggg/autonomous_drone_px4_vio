# Gain 0.75 lateral-stop results — September 21, 08:27–08:29

Settings confirm **0.75 ON / OFF / 0.75 ON**, correction cap 0.10 m/s²,
smoothing 0.10 s, speed 0.20 m/s, reference acceleration cap 0.30 m/s²,
distance 0.35 m and unchanged settling gates. OFF disables extra damping;
the original reference braking still operates. Its recorded gain 0.5 is inactive.

## Finding

The first enabled run passed all four movement/settling stages. The disabled
run failed at the first hold. The third enabled run traversed all four legs but
landed after approximately ten seconds at the final hold, consistent with a
settling timeout. Its missing ROS tail was recovered from the matching ULog.

This is better sequence progress than the earlier 0.5 trial, but **not a
repeatable fix for overshoot**. The third enabled run is visibly oscillatory and
has worse first-stop motion than OFF over equal-duration windows.

| Flight | Damping | Holds passed | First hold | First 3 s lateral velocity RMS | First 3 s lateral swing | Largest endpoint overshoot |
| --- | --- | --- | --- | --- | --- | --- |
| 08:27:29 | ON, 0.75 | 4/4 | 3.05 s | 0.023 m/s | 3.8 cm | 10.4 cm, leg 4 |
| 08:28:34 | OFF | 0/4 | 10.06 s, timeout | 0.036 m/s | 4.6 cm | 17.4 cm, leg 1 |
| 08:29:06 | ON, 0.75 | 3/4 inferred from continued movement | 4.50 s | 0.088 m/s | 12.9 cm | 15.9 cm, leg 3 |

Overshoot is the excursion beyond the endpoint in that leg's travel direction,
from reference braking until hold end, not total hold swing or radial error.
Flights reached different legs; the final column is an outcome summary,
not an equal-condition comparison.

## Motion and remaining problem

- Flight 1 holds lasted 3.05 / 3.25 / 5.61 / 4.16 s. Last two overshoots
  were 9.6 / 10.4 cm. Its final hold still swung 18.8 cm and reached 12.5 cm
  forward error. Peak final-leg lateral speed was 0.324 m/s despite the
  0.20 m/s reference cap: PX4 position feedback adds to the velocity reference.
- Flight 2 initially stopped within about 2 mm beyond the endpoint, then
  moved away again during a fixed-target hold. Full hold swing was 25.3 cm
  and eventual overshoot 17.4 cm. Speed failed the gate about 51% of the
  hold and radial position 46%. This is not just braking too late.
- Flight 3 holds lasted approximately 4.50 / 7.85 / 5.6 / 10 s. Leg 2/3
  overshoots were 15.2 / 15.9 cm. Final hold swing was 16.1 cm; speed failed
  about 55%, position 11%, yaw 8%. Longest reconstructed eligible joint
  dwell was about 0.70 s, below the required 1 s. Final directional
  overshoot was only 4.6 cm, illustrating why that metric alone is insufficient.

Keep the settling gates. Dwell reconstruction has sampling uncertainty and
does not reproduce the adapter's exact sampled decisions.

## Braking demand versus achieved response

Matched onboard-clock samples, projected onto the fixed lateral axis
(positive right). Acceleration demand is PX4's internal output, not just
adapter feedforward.

| Flight / bag time | Estimated velocity | PX4 velocity demand | PX4 acceleration demand | Tilt-equivalent acceleration |
| --- | --- | --- | --- | --- |
| OFF, 18.0 s | +0.077 m/s | −0.011 m/s | −0.194 m/s² | +0.044 m/s² |
| ON flight 3, 18.0 s | +0.158 m/s | +0.030 m/s | −0.390 m/s² | +0.039 m/s² |
| ON flight 3, 18.5 s | +0.100 m/s | −0.027 m/s | −0.437 m/s² | −0.323 m/s² |
| ON flight 3, 50.0 s | −0.064 m/s | +0.052 m/s | +0.216 m/s² | −0.010 m/s² |

PX4 already asks for braking while achieved tilt can lag or point the other
way, including with damping ON. Tilt-equivalent acceleration assumes near-hover
thrust; it is not measured net acceleration and does not isolate a motor, PID,
trim or estimator-delay cause.

First-three-second roll-rate tracking-error RMS was 2.94 / 3.42 / 4.86 deg/s;
pitch was 2.87 / 3.93 / 3.71 deg/s. Flight 3's first hold reached 9.1° yaw error
and 17.16 deg/s yaw-rate tracking-error RMS. This batch does not show that
increased damping consistently reduces attitude oscillation. No motor command
exceeded 0.95 in the sampled first-three-second hold windows (largest peak
about 0.703); that does not rule out a response fault.

## Application and comparison limitations

Enabled ROS active samples show no stale-velocity or callback-reset damping
events. Maximum velocity receipt ages were 30 / 31 / 33 ms. Onboard hold
acceleration is nonzero for ON and zero for OFF. Both ON runs approach the
0.10 correction cap. Flight 3 spent about 12% of its first hold and 11% of
its last hold above 0.095 m/s² correction. This does not justify increasing
the cap while achieved response lags. After flight 3's ROS truncation,
application evidence comes from onboard trajectory setpoints.

First-hold loaded voltages were **11.16 / 10.92 / 10.77 V**, currents
35.3 / 35.8 / 36.8 A. Headings were 90.1 / 87.5 / 88.3 degrees.
Starting positions and battery conditions differ. Voltage is a confound,
not proof of the deterioration's cause. Relevant recorded PX4 position/rate
gains, board offsets and EV delay match; there were no parameter changes
or estimator reset-counter changes within these onboard recordings.

Read-only USB downloads from SD directory /fs/microsd/log/2026-09-18/:

| ROS flight | ULog | Bytes verified against SD | Exact local-position timestamp matches |
| --- | --- | --- | --- |
| 082729 | 23_06_28.ulg | 5,351,975 | 1923 |
| 082834 | 23_07_33.ulg | 3,165,340 | 1120 |
| 082906 | 23_08_05.ulg | 7,236,033 | 1446 |

All matches have zero x-position discrepancy. SD and host clocks differ;
pairing did not rely on names.

Flight 3's metadata is empty and MCAP ends mid-record near 33.5 s, during
leg 3. Onboard holds begin near 34.857 s (left) and 43.354 s (return).
Landing is requested near 53.353 s, about 49 s after arming, before the
50 s armed watchdog. The ROS timeout message is missing: timeout is inferred
from the ten-second hold and unsettled motion. Recovered boundaries have
roughly 0.1 s uncertainty. Metrics end at the final stationary reference
sample before landing, giving 9.89 s of the last hold. ROS boundaries have
approximately one control tick of alignment uncertainty.

The OFF ULog has a 131 ms dropout during the first hold. Continuous ROS local
position cross-checks first-three-second RMS at 0.035 versus onboard 0.036 m/s;
maximum ROS gap is 33 ms. Other nonzero onboard dropouts are before lateral
motion, except flight 3's 9 ms dropout in its second hold.

## Recommendation

Keep 0.75 as a **test setting** with the existing cap. Do not increase gain/cap
or port to the proven tunnel adapter based on this batch. Repeat ON/OFF with
similarly charged packs and the same setup, alternating order. Require
repeated clean later stops, not just a completed sequence. The demand/response
mismatch deserves the next focused investigation; stronger reference braking
alone is not supported as the solution.

Compared with the earlier 0.5 batch (neither enabled run completed), 0.75 gave
one complete run and one final-hold failure, but first-stop speed/swing did not
improve consistently. Do not claim a monotonic gain benefit.
No runtime code, launch defaults, PX4 parameters or tunnel settings changed.
Default damping remains disabled; default gain remains 0.5.

## Artifacts and verification

- [Full flights](reports/lateral_damping_0827_20260921/all_flights.png)
- [Equal-duration first stops](reports/lateral_damping_0827_20260921/first_stop_comparison.png)
- [Controller response](reports/lateral_damping_0827_20260921/control_response.png)
- [Metrics](reports/lateral_damping_0827_20260921/comparison.json)
- [Alignment](reports/lateral_damping_0827_20260921/alignment.json)

Reproduce using .venv-mavlink/bin/python on
reports/lateral_damping_0827_20260921/align.py and then analyze.py;
run git diff --check.

Assertions passed for file sizes, exact pairing, ON/OFF settings, valid XY
estimates, fixed hold targets, zero hold reference velocities, bounded
hold acceleration, unchanged relevant parameters and stable reset counters.
Figures were inspected. Runtime tests were not rerun for this analysis/docs-only
session. Work was lead-only; no flight commands or parameter writes were sent.

