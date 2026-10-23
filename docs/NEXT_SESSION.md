# Current tunnel work — 2026-09-21

**Newest global damping flights:** read
[the 11:16–11:21 comparison](../reports/global_damping_flights_20260921/REVIEW.md)
first. Of four recordings, two were real flights with damping gain 0.75. The
11:17 flight reached the goal, held it continuously for 3.00 s and initiated
normal goal landing; 11:21 did not reach the goal and landed on the VIO reset
sentinel after repeated follower/clearance faults. Against the 07:26 damping-OFF
baseline, route position-target p95 error improved from 0.193 m to 0.171/0.162
m, but peak measured speed did not improve (0.295 versus 0.303/0.325 m/s) and
cross-track/fault behavior was not reliably better. Routes/maps differ, so keep
damping opt-in/default-OFF. The next causal test is a same-route, similar-battery
OFF/ON pair, not a gain or cap increase. Renders, matched ULog evidence and
metrics are in `reports/global_damping_flights_20260921/`.

**Highlighted trajectory jaggedness:** the sharp magenta/orange knots are in
the commanded PX4-local position as well as the flown track. RTAB-Map added
discrete display shifts of up to 0.094/0.126 m, but a fixed-correction raw-VIO
comparison proves the 11:21 knot is real recovery motion. Both flights had
roughly 2.1--2.6 s correction-settling follower dropouts; the current 1.5 s
`follower_latch_grace` expires mid-episode, clears the route limiter and enters
HOLD/clearance escape. The next one-variable flight should keep every other
setting unchanged and launch with `follower_latch_grace:=3.0`. This remains an
opt-in trial; do not change the 1.5 s default yet. See the jagged-region section
and plots in the comparison report above.

**Newest damping repeat:** read
[the gain-0.75 repeat result](LATERAL_DAMPING_GAIN075_REPEAT_RESULTS_20260921.md)
first. The 10:12 flight used the restored PX4 baseline and damping gain 0.75;
it passed all four holds in 3.04 / 3.05 / 4.35 / 4.63 s with worst directional
overshoot 4.7 cm. The matched ULog contains 1.109 s of uninterrupted joint-gate
dwell immediately before NAV_LAND, proving the truncated ROS tail is a normal
completion. This is the second 4/4 gain-0.75 run. The opt-in, default-OFF
global/tunnel adapter port is now implemented and tested; read
[the implementation note](GLOBAL_DAMPING_OPT_IN_20260921.md). Keep gain 0.75,
correction cap 0.10 m/s² and filter 0.10 s; do not enable it by default. The
next step is the documented `auto_arm:=false brake_damping:=true` props-off
check with the full planner/VIO graph, then review its recorded config/debug
before an armed tunnel trial.

**Newest experiment:** read
[the MPC_XY_VEL_I_ACC 0.30 result](MPC_XY_VEL_I030_RESULTS_20260921.md) first.
With position P restored to 0.95, roll-rate P 0.150 and damping OFF, the 09:55
flight passed three holds and reached the final return, versus two holds for the
I=0.40 baseline. It still timed out: the final hold swung 29.2 cm, yaw tracking
was worse, and inferred velocity compensation did not decrease. Restore
MPC_XY_VEL_I_ACC=0.40 and do not reduce it further. Post-flight live state was
Disarmed with 0.30 still saved. No parameter was written during analysis.

**Previous position-gain experiment:** read
[the MPC_XY_P 0.75 result](MPC_XY_P075_RESULTS_20260921.md) first. The 09:40
flight used roll-rate P 0.150 and damping OFF. Lowering horizontal position P
from 0.95 to 0.75 reduced early internal velocity demand and early hold motion,
but both settings passed only two holds and timed out on leg 3. The 0.75 trial's
late third-hold swing was worse (27.1 versus 14.2 cm) and yaw-rate tracking was
worse. Restore MPC_XY_P=0.95; post-flight live state was Disarmed with 0.75
still saved. No parameter was written during analysis.

**Previous roll-rate experiment:** read
[the roll-rate P results](ROLL_RATE_TRIAL_RESULTS_20260921.md) first, then the
[Sol braking/PID handoff](HANDOFF_SOL_BRAKING_PID_20260921.md). The user flew
adapter damping OFF with `MC_ROLLRATE_P` 0.165 and then 0.150. Both passed two
holds and timed out on the third. The higher gain arrested some initial motion
more strongly but produced much worse renewed first-hold motion and did not
improve roll-rate tracking. Keep 0.150; do not escalate this gain. Post-flight
live readback confirmed Disarmed, no failsafe, and saved value 0.1500.

Read [HANDOFF_SOL_MIGRATION_TUNNEL.md](HANDOFF_SOL_MIGRATION_TUNNEL.md) for
the broader migration/tunnel context after the new braking/PID handoff.
Also read [the lateral-stop follow-up](LATERAL_BRAKING_REVIEW_20260921.md) for
the diagnosis. The user approved implementation: read
[the implemented damping trial](LATERAL_DAMPING_TRIAL_20260921.md) for the next
comparison flight. It is built/tested, disabled by default, and limited to the
lateral-stop adapter. It has now flown ON/OFF/ON; read
[the comparison](LATERAL_DAMPING_RESULTS_20260921.md) first. Both enabled runs
settled better at the first stop, but later holds still failed. A gain 0.75 trial
with the same 0.10 cap was subsequently flown: read the newer
[0.75 results](LATERAL_DAMPING_GAIN075_RESULTS_20260921.md) first.
At 08:27/08:28/08:29 the settings were 0.75 ON / OFF / 0.75 ON. First run
passed all four holds; OFF failed the first; third reached the final hold but
landed after about ten seconds unsettled (tail recovered from ULog).
Third run was much wobblier, so the repeat with comparable batteries was needed
before any gain increase or tunnel port. At that point, defaults and the tunnel
adapter were unchanged; the newer decision at the top supersedes this history.

Working branch: `tune/migration-tunnel-smoothing`, based directly on the proven
`ros-rtabmap-migration` commit `893cbea`. The latest successful flight is
`offboard_global_20260921T072654Z`. Global-adapter settings remain at the successful
baseline, including 0.20 m/s cruise. The proposed 0.18 m/s trial was withdrawn
after the user clarified that the objective is correcting overshoot itself.

The abandoned experimental work is saved at `fc99cc4` on
`fix/velocity-aware-stop-recovery`. Do not use that branch's recovery/controller
design as the starting point. Preserve the migration controller and make one
small, measured change at a time.
