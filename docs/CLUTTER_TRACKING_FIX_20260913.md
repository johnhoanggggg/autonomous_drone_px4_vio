# Clutter tracking changes — 2026-09-13

## Outcome and limits

The C++ global-flight adapter now enables a strict command limiter: full XY
velocity changes are acceleration bounded, including replacement-path joins.
Unrounded polyline corners decelerate to a stop. The command point stays on
the accepted route or its bounded rejoin, and the adapter still checks the
resulting swept chord, clearance and geofence before accepting it.

This repairs a command-generation defect; it does **not** establish that the
aircraft can yet hold the clearance needed for tight tunnels. The independent
lateral-stop test still has a 16.3 cm stationary-target swing and weak physical
braking. No flight or motor test was started, no tolerances were widened, and
no further control gains were changed. EV delay remains the saved 225 ms trial.

## Evidence

- The September 13 tunnel command velocity changed by up to approximately
  1.31 m/s² with a configured scalar acceleration limit of 0.30 m/s². Limiting
  speed magnitude or clipping acceleration feedforward did not bound turns.
- The latest braking test has a constant third-hold position target and zero
  XY feedforward. In-flight reset counters are constant; small map corrections
  do not enter this fixed-local-frame test. Discrete loop-closure jumps do not
  explain this oscillation. Continuous estimation error is not ruled out.
- During bag time 27.975–28.678 s, demanded lateral acceleration averages
  approximately 0.302 m/s²; actual tilt and rotated-IMU acceleration correspond
  to approximately 0.053 and 0.063 m/s². Weak motion response is observed in
  physical attitude/IMU signals as well as estimated velocity.
- Rate-controller I opposes P at several reversals. This is an observation,
  not proof that I is faulty: integral torque may compensate disturbances or
  trim. Do not reset it on reversals based only on sign. The exact installed
  firmware does not consume the logged `vehicle_rates_setpoint.reset_integral`
  field in multicopter rate control; its false value is not a missing-reset
  diagnosis. [Firmware source](https://raw.githubusercontent.com/PX4/PX4-Autopilot/d6f12ad1c4f70ad3230afd7d86e971421e02fef4/src/modules/mc_rate_control/MulticopterRateControl.cpp).

See [latest braking results](../ros2_ws/flight_logs/offboard_lateral_stop_20260913T121110Z/ANALYSIS.md),
[control-response plot](../ros2_ws/flight_logs/offboard_lateral_stop_20260913T121110Z/render/control_response/chain.png),
and [detailed response investigation](../reports/braking_response_20260913/ANALYSIS.md).
The two tunnel render folders remain inside their respective flight folders.

## Implementation

`PathCommandLimiter` has an optional strict mode; the C++ global adapter
enables it and records that setting. Existing legacy/Python parity behavior is
retained when strict mode is absent. Exact velocity secants and vector bounds
are checked at joins and corners. Tiny replacement offsets no longer snap
the point onto the new path. Infeasible updates are rejected and the adapter
restores the preceding validated state. The replay harness supports the same
strict transaction and speed caps.

Strict bounds apply in the route's map frame, not to actual aircraft motion.
Map-to-PX4 rebasing can independently change position; accepted map corrections
and emergency hold behavior are not a proof of globally smooth dynamics.
Exact sharp corners now stop even when the historical `corner_blending`
parameter is true. Continuous-speed turns need explicitly clearance-checked
rounded geometry; diagonal shortcuts are not introduced here.

## Onboard diagnostics

With PX4 confirmed disarmed and the logger inactive, preserved the old topic
file as `/fs/microsd/etc/logging/pre_clutter_20260913.txt`, then added instance 0:

```text
estimator_aid_src_ev_pos 20 0
estimator_aid_src_ev_yaw 20 0
estimator_status_flags 100 0
estimator_states 100 0
```

Restarted the logger and verified 30 configured topics, 254 subscriptions and
no subscription-capacity warnings. Explicit instance selection fixed an
initial attempt that exceeded the 255-subscription limit. The current log
configuration has one EKF IMU instance. Verification transcript:
`reports/clutter_logging_restart_fixed_20260913.txt`.

## Validation

Focused tests cover sharp/shallow corner departure and endpoint settling,
20–80 ms timestep variation, tiny-offset replans, changed-tangent joins,
join completion and pauses. The package built successfully. Full `colcon test` reported 701 tests, zero
errors/failures and three existing legacy rolling-join parity skips. After the
final join-speed guard, the package was rebuilt and the C++ limiter and Python
parity targets passed again. `git diff --check` passed. These are software
checks, not a closed-loop flight simulation.

Commands from `ros2_ws`, with the documented ROS/DepthAI underlays sourced:

```bash
colcon build --packages-select px4_vio_bridge
colcon test --packages-select px4_vio_bridge --event-handlers console_direct+
colcon test --packages-select px4_vio_bridge --ctest-args -R 'test_planner_flight_cpp|test_planner_flight_parity'
colcon test-result --test-result-base build/px4_vio_bridge --verbose
```

Build output contains the existing overlay-path warning and GCC ARM ABI notes.
Test/build transcripts are copied under `reports/clutter_validation_20260913/`.

Two workers independently audited the motion-response chain and implemented
the limiter core; root reviewed the firmware interpretation, integrated the
adapter/replay changes and owns package verification.

The next physical investigation remains a controlled open-area response
check, with the added estimator diagnostics. Motor-start/direction testing
requires the operator's confirmation that all four props are removed, per
[the existing procedure](YAW_MOTOR_DIAGNOSIS_20260908.md). That confirmation
has not arrived; no motor-test command has been sent.
