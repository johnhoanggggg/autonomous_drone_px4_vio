# MPC_XY_P 0.75 lateral-stop result — September 21, 09:40

## Finding

The flight confirms **MPC_XY_P=0.75**, `MC_ROLLRATE_P=0.150`, and adapter
damping OFF. Compare it with the immediately preceding 09:14 baseline at
MPC_XY_P=0.95. The lower position gain substantially reduced PX4's early hold
velocity demand and early lateral motion, but it did **not** solve the sequence:
both flights passed two holds and timed out on leg 3.

Do not retain 0.75. Restore **MPC_XY_P=0.95**. The 0.75 FC value was still
saved at the post-flight readback; no parameter was written during analysis.

| Metric | Baseline 0.95 | Trial 0.75 |
| --- | ---: | ---: |
| Holds passed | 2/3, leg 3 timeout | 2/3, leg 3 timeout |
| Leg 1 hold duration | 3.05 s | 5.60 s |
| Leg 2 hold duration | 3.95 s | 4.05 s |
| Leg 3 hold duration | 10.01 s | 10.02 s |
| Leg 3 full lateral swing | 14.2 cm | **27.1 cm** |
| Leg 3 eventual overshoot | 8.5 cm | 9.9 cm |

## Intended effect was present

The parameter change clearly reached the controller. First-three-second hold
metrics show smaller internal velocity corrections and smaller early lateral
motion at every stop:

| Leg | Metric | P=0.95 | P=0.75 |
| ---: | --- | ---: | ---: |
| 1 | Internal lateral velocity-demand RMS | 0.020 m/s | 0.010 m/s |
| 1 | Lateral velocity RMS | 0.026 m/s | 0.023 m/s |
| 1 | Lateral swing | 6.4 cm | 3.2 cm |
| 2 | Internal lateral velocity-demand RMS | 0.058 m/s | 0.019 m/s |
| 2 | Lateral velocity RMS | 0.050 m/s | 0.034 m/s |
| 2 | Lateral swing | 14.4 cm | 4.7 cm |
| 3 | Internal lateral velocity-demand RMS | 0.060 m/s | 0.028 m/s |
| 3 | Lateral velocity RMS | 0.052 m/s | 0.033 m/s |
| 3 | Lateral swing | 12.4 cm | 4.9 cm |

The leg-2 whole-hold lateral swing improved from 15.6 to 5.6 cm and eventual
directional overshoot from 3.2 to 0.6 cm. The lower position gain therefore did
make PX4 less aggressive around the target; this was not a null experiment.

## Why it is not a fix

The calmer early response did not persist. During the third fixed-target hold,
the 0.75 flight later traversed 27.1 cm laterally, versus 14.2 cm at 0.95, and
failed the same ten-second settling timeout. It initially reversed before the
endpoint, then renewed motion and eventually reached 9.9 cm beyond it.

The first hold also took 2.55 s longer. Its lateral response was calmer, but
forward-axis error reached 10.0 cm and intermittently failed the radial-position
gate. Because `MPC_XY_P` applies to both horizontal axes, lowering it also makes
recovery of uncommanded forward displacement slower.

Equal first-three-second hold windows show similar roll tracking but worse yaw:

| Metric across all three early holds | P=0.95 | P=0.75 |
| --- | ---: | ---: |
| Roll-rate error RMS | 3.37 deg/s | 3.34 deg/s |
| Pitch-rate error RMS | 2.83 deg/s | 2.70 deg/s |
| Yaw-rate error RMS | 6.32 deg/s | **10.84 deg/s** |
| Peak motor command | 0.648 | 0.666 |

No sampled motor command exceeded 0.95. The lower position gain reduced early
position correction as predicted, but it did not address the later coupled
lateral/yaw/forward motion.

## Comparability and recovery

- Loaded first-hold voltage is identical to displayed precision: 11.236 V in
  both flights. Current is 35.07 A versus 34.57 A.
- Start headings are 89.91 and 90.08 degrees; origins differ by only a few mm.
- Active visual feature-count medians are 413 and 409. Estimator reset counters
  remain constant within both flights.
- Relevant position, velocity, rate, board-offset and EV-delay parameters match.
  Aside from flight counters, initial parameter differences are `MPC_XY_P` and
  accelerometer offsets automatically committed after the preceding flight.
- ULog dropouts are before lateral motion: 204 ms baseline and 106 ms trial.
- The trial MCAP is truncated at 35.63 s. Its ULog continues through the leg-3
  timeout, landing at 37.82 s and external disarm at 40.22 s. Truncation is a
  recorder issue, not a controller failure.
- Exact vehicle-local-position timestamp pairing gives 1,363 baseline and
  1,472 trial matches with zero x-position discrepancy.

## Recommendation

Restore **MPC_XY_P=0.95**. Do not combine 0.75 with damping or change another
parameter before rollback. The trial shows that reducing position correction
can clean up the first few seconds but is not sufficient for the late renewed
motion. A future experiment should target that coupled delayed response without
accepting a larger final-hold swing.

## Artifacts and verification

- [Whole flights](../reports/mpc_xy_p075_20260921/all_flights.png)
- [Equal-duration first holds](../reports/mpc_xy_p075_20260921/first_hold_comparison.png)
- [Metrics](../reports/mpc_xy_p075_20260921/comparison.json)
- [Timestamp pairing](../reports/mpc_xy_p075_20260921/alignment.json)
- [Analysis](../reports/mpc_xy_p075_20260921/analyze.py)

Reproduce with `.venv-mavlink/bin/python3
reports/mpc_xy_p075_20260921/analyze.py`. Assertions cover file sizes, exact
timestamp pairing, recorded gain values, unchanged in-flight parameters,
matching adapter settings, stable estimator reset counters, recovered final
hold duration and the expected initial-parameter differences. Figures were
inspected. No runtime code changed and runtime tests were not rerun. Analysis
was lead-only.
