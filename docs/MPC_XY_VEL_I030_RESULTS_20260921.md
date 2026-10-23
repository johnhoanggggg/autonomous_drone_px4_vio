# MPC_XY_VEL_I_ACC 0.30 result — September 21, 09:55

## Finding

The flight confirms **MPC_XY_VEL_I_ACC=0.30**, `MPC_XY_P=0.95`,
`MC_ROLLRATE_P=0.150`, and adapter damping OFF. Compare it with the 09:14
baseline at velocity I=0.40.

The lower-I trial progressed farther: it passed the first three holds and
reached the fourth, while the baseline timed out on hold 3. However, this was
not a clean or complete improvement. The final hold swung **29.2 cm** and timed
out, corresponding-hold yaw tracking was worse, and the inferred velocity-loop
residual was not reduced.

Restore **MPC_XY_VEL_I_ACC=0.40**. Do not reduce it further based on this run.
The 0.30 value was still saved at the post-flight readback; no parameter was
written during analysis.

| Metric | Baseline I=0.40 | Trial I=0.30 |
| --- | ---: | ---: |
| Holds passed | 2/3, hold 3 timeout | 3/4, hold 4 timeout |
| Hold 1 duration | 3.05 s | 4.30 s |
| Hold 2 duration | 3.95 s | 4.85 s |
| Hold 3 duration | 10.01 s, failed | 3.41 s, passed |
| Final attempted hold swing | 14.2 cm, hold 3 | **29.2 cm**, hold 4 |
| Final attempted hold overshoot | 8.5 cm | 10.1 cm |

## Corresponding first three holds

| Leg | Metric | I=0.40 | I=0.30 |
| ---: | --- | ---: | ---: |
| 1 | Eventual overshoot | 3.0 cm | 7.4 cm |
| 1 | Full hold swing | 6.4 cm | 5.8 cm |
| 1 | First-3-s lateral velocity RMS | 0.026 m/s | 0.020 m/s |
| 2 | Eventual overshoot | 3.2 cm | 0.7 cm |
| 2 | Full hold swing | 15.6 cm | 8.7 cm |
| 2 | First-3-s lateral velocity RMS | 0.050 m/s | 0.034 m/s |
| 3 | Eventual overshoot | 8.5 cm | 7.0 cm |
| 3 | Full hold swing | 14.2 cm | 12.1 cm |
| 3 | First-3-s lateral velocity RMS | 0.052 m/s | 0.064 m/s |

The trial's third hold passed because it found approximately 1.05 s of joint
eligible dwell after the minimum hold period, despite speed exceeding the gate
about 40% of that hold and yaw exceeding its gate about 27%. Its final hold
never found more than about 0.64 s and failed after ten seconds; speed failed
about 67% and radial position about 36% of that hold.

## Control response

The known P, D and trajectory-acceleration contributions were removed from the
onboard acceleration output to estimate the remaining velocity-loop
compensation. This residual is mostly the integrator during stationary holds,
but it is not a directly logged integrator state.

Reducing the configured I coefficient did not consistently reduce this residual:

| First 3 s | Inferred residual RMS, I=0.40 | Inferred residual RMS, I=0.30 |
| --- | ---: | ---: |
| Hold 1 | 0.127 m/s² | 0.172 m/s² |
| Hold 2 | 0.213 m/s² | 0.241 m/s² |
| Hold 3 | 0.176 m/s² | 0.213 m/s² |

Different velocity-error histories can accumulate different integrator states;
the parameter coefficient alone does not determine the instantaneous output.
This pair therefore does not demonstrate that the prior late swing was caused
by an excessively large velocity-I coefficient.

Equal first-three-second windows across the corresponding first three holds:

| Metric | I=0.40 | I=0.30 |
| --- | ---: | ---: |
| Roll-rate error RMS | 3.37 deg/s | 3.23 deg/s |
| Pitch-rate error RMS | 2.83 deg/s | 2.53 deg/s |
| Yaw-rate error RMS | 6.32 deg/s | **9.05 deg/s** |
| Peak motor command | 0.648 | 0.666 |

No sampled motor command exceeded 0.95.

## Comparability and integrity

- First-hold loaded voltage/current are closely matched: 11.236 V / 35.07 A
  versus 11.229 V / 35.17 A. Start headings differ by 0.16 degrees and origins
  by about 1 mm.
- Active visual feature-count median is lower in the trial (326 versus 413),
  with minima 223 versus 343. There are no estimator reset-counter changes;
  VIO scene quality remains a comparison limitation.
- Relevant position, velocity, rate, board-offset and EV-delay parameters match.
  Aside from flight counters, initial differences are `MPC_XY_VEL_I_ACC` and
  accelerometer offsets automatically committed after the baseline flight.
- ULog dropouts are before lateral motion: 204 ms baseline and 150 ms trial.
- The trial ROS recording is complete. LAND is explicitly excluded because its
  debug fields retain leg 4. Onboard events confirm landing at 47.29 s after the
  ten-second final-hold timeout and external disarm at 49.83 s.
- Exact local-position timestamp pairing gives 1,363 baseline and 2,206 trial
  matches with zero x-position discrepancy.

## Recommendation

Restore **MPC_XY_VEL_I_ACC=0.40**. The additional sequence progress is worth
noting, but a gain that ends in a 29.2 cm hold swing, worse yaw tracking and the
same overall timeout outcome should not be retained from one run. Do not reduce
I further or combine 0.30 with damping.

## Artifacts and verification

- [Whole flights](../reports/mpc_xy_vel_i030_20260921/all_flights.png)
- [Equal-duration first holds](../reports/mpc_xy_vel_i030_20260921/first_hold_comparison.png)
- [Metrics](../reports/mpc_xy_vel_i030_20260921/comparison.json)
- [Timestamp pairing](../reports/mpc_xy_vel_i030_20260921/alignment.json)
- [Analysis](../reports/mpc_xy_vel_i030_20260921/analyze.py)

Reproduce with `.venv-mavlink/bin/python3
reports/mpc_xy_vel_i030_20260921/analyze.py`. Assertions cover ULog sizes,
exact pairing, recorded gains, unchanged in-flight parameters, adapter settings,
stable estimator counters, timeout boundaries and expected initial-parameter
differences. Figures were inspected. No runtime code changed and runtime tests
were not rerun. Analysis was lead-only.
