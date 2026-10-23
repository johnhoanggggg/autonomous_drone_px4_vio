# Roll-rate P lateral-stop results — September 21, 08:54 / 09:14

## Finding

The two flights confirm **MC_ROLLRATE_P 0.165 then 0.150**, with adapter
`brake_damping=false` in both. The higher gain did not improve the end-to-end
stop. It arrested the initial motion more strongly in parts of the sequence,
but renewed motion during the fixed-target holds was much worse. Both flights
passed the first two holds and timed out on the third; neither reached leg 4.

Keep **0.150**. It is already restored on the disarmed FC. Do not increase the
roll-rate P gain based on this pair.

| Flight | Roll-rate P | Holds passed | First hold duration | First-stop overshoot | Eventual first-leg overshoot | First 3 s lateral velocity RMS | First 3 s hold swing |
| --- | ---: | ---: | ---: | ---: | ---: | ---: | ---: |
| 08:54:06 | 0.165 | 2/3, leg 3 timeout | 6.50 s | 0.0 cm | 13.8 cm | 0.078 m/s | 13.1 cm |
| 09:14:48 | 0.150 | 2/3, leg 3 timeout | 3.05 s | 3.0 cm | 3.0 cm | 0.026 m/s | 6.4 cm |

First-stop overshoot ends at the first velocity reversal. Eventual overshoot
includes the subsequent fixed-target hold. The distinction matters here:
0.165 first stopped about 5 cm short of the endpoint, then accelerated through
it and reached 13.8 cm beyond it. This is not a clean braking improvement.

## Corresponding legs

| Leg | Metric | P=0.165 | P=0.150 |
| ---: | --- | ---: | ---: |
| 1 | Hold duration | 6.50 s | 3.05 s |
| 1 | Eventual directional overshoot | 13.8 cm | 3.0 cm |
| 1 | Full hold swing | 22.9 cm | 6.4 cm |
| 1 | Speed-gate failure fraction | 47% | 0% |
| 2 | Hold duration | 6.34 s | 3.95 s |
| 2 | Eventual directional overshoot | 10.2 cm | 3.2 cm |
| 2 | Full hold swing | 13.8 cm | 15.6 cm |
| 3 | Hold duration/outcome | 10.07 s, timeout | 10.01 s, timeout |
| 3 | Eventual directional overshoot | 9.8 cm | 8.5 cm |
| 3 | Full hold swing | 16.8 cm | 14.2 cm |

At the reference stop, P=0.165 had already reversed lateral velocity on legs
1 and 2 (−0.026 and −0.038 m/s in the leg direction), while P=0.150 was near
zero on leg 1 and still moving at +0.086 m/s on leg 2. Initial conditions and
speeds differed, so this is not isolated system identification. More
importantly, the higher gain did not hold the stopped position afterward.

The third holds show mixed secondary effects. P=0.165 had less forward-axis
error (13.3 versus 18.3 cm) and a lower speed-gate failure fraction (61% versus
76%), but worse yaw error (8.3 versus 4.8 degrees), slightly greater lateral
overshoot, and the same timeout outcome.

## Rate tracking and actuation

Equal-duration onboard windows do not show a roll-loop improvement:

| Window | Metric | P=0.165 | P=0.150 |
| --- | --- | ---: | ---: |
| All three braking ramps | Roll-rate error RMS | 3.67 deg/s | 3.39 deg/s |
| All three braking ramps | Roll-angle error RMS | 0.86 deg | 0.70 deg |
| First 3 s of all holds | Roll-rate error RMS | 3.44 deg/s | 3.37 deg/s |
| First 3 s of all holds | Roll-angle error RMS | 0.91 deg | 0.73 deg |
| First hold only | Roll-rate error RMS | 3.63 deg/s | 2.63 deg/s |
| All three braking ramps | Yaw-rate error RMS | 13.10 deg/s | 7.01 deg/s |
| All three braking ramps | Peak motor command | 0.712 | 0.623 |

No sampled motor command exceeded 0.95. Exact-sample rate-controller
reconstruction in the first hold shows the expected larger roll P contribution
(RMS 0.0104 versus 0.0070) and larger roll torque (RMS 0.0248 versus 0.0182),
but not lower tracking error. This lateral test is not an isolated rate-loop
identification flight, so the yaw and torque differences are run-level
observations, not proof that the 10% P increase directly caused each one.

## Comparability and log integrity

- Onboard snapshots record roll-rate P/I/D/K as 0.165/0.100/0.003/1.0 and
  0.150/0.100/0.003/1.0. Aside from ordinary flight counters, the only initial
  parameter difference is `MC_ROLLRATE_P`. Neither flight changed parameters.
- Adapter settings match: damping OFF, 0.20 m/s speed, 0.30 m/s² reference
  acceleration, 35 cm endpoints, and unchanged settle gates.
- First-hold loaded voltage/current are close: 11.19 V / 35.50 A versus
  11.24 V / 35.07 A. Origins and headings are also closely matched.
- Estimator reset counters are constant within both flights. P=0.165 has one
  isolated visual feature-count sample of 58 during its first hold; its active
  median is 383 and there is no coincident estimator reset. P=0.150's active
  minimum/median are 343/413.
- The only ULog dropouts occur before lateral motion: 34 ms for P=0.165 and
  204 ms for P=0.150.
- The P=0.150 MCAP is truncated at 32.22 s with empty metadata. Its matched
  ULog continues through the third-hold timeout, landing at 36.25 s, and
  disarm at 38.11 s. Recorder truncation is not a flight-controller failure.

Read-only onboard downloads and exact timestamp pairing:

| ROS flight | ULog | Bytes | Exact local-position matches | X difference |
| --- | --- | ---: | ---: | ---: |
| 08:54:06 | 23_32_28.ulg | 5,861,396 | 2,111 | 0 m |
| 09:14:48 | 23_52_43.ulg | 4,758,798 | 1,363 | 0 m |

## Recommendation

Retain 0.150 and do not escalate roll-rate P. The trial suggests that the key
failure is renewed fixed-target hold motion, not simply late roll response.
If another flight is needed before changing a different control parameter,
repeat the 0.150 baseline with the same setup to establish repeatability.

The live readback after these flights was **Disarmed**, navigation mode Hold,
no failsafe, and `MC_ROLLRATE_P=0.1500` (saved). No parameter was written during
this analysis.

## Artifacts and reproduction

- [Whole flights](../reports/roll_rate_0854_0914_20260921/all_flights.png)
- [Equal-duration first holds](../reports/roll_rate_0854_0914_20260921/first_hold_comparison.png)
- [Metrics](../reports/roll_rate_0854_0914_20260921/comparison.json)
- [Timestamp pairing](../reports/roll_rate_0854_0914_20260921/alignment.json)
- [Analysis script](../reports/roll_rate_0854_0914_20260921/analyze.py)

Reproduce with `.venv-mavlink/bin/python3
reports/roll_rate_0854_0914_20260921/analyze.py`. File sizes, exact pairing,
gain snapshots, unchanged in-flight parameters, matching adapter settings,
stable reset counters, final-hold recovery, and the expected parameter-diff set
are asserted. Figures were inspected. No runtime code changed and runtime tests
were not rerun. Analysis was lead-only.
